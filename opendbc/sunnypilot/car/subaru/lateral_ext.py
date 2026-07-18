"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Subaru LKAS_ANGLE lateral extension: driver-override hysteresis, MADS-only
guards, engage/disengage shaping, and a jerk-limited motion planner that
bounds both angle rate and angle acceleration on the commanded output.
"""

import math
import numpy as np

from opendbc.car.vehicle_model import VehicleModel

DRIVER_OVERRIDE_TORQUE = 120
DRIVER_OVERRIDE_TORQUE_RELEASE = 100     # must clear resting-hand torque so light grip doesn't block resume
WHEEL_SETTLED_RATE = 25.                 # deg/s; torque dips mid-maneuver, wheel motion doesn't
RESUME_MAX_TARGET_ERR = 20.              # deg; don't take over while plan and hand-held angle disagree
SUSPEND_HOLD_FRAMES = 25                 # ~0.5 s
MADS_ONLY_MAX_STEER_ANGLE = 120          # deg

PRE_ENGAGE_CLEAN_FRAMES = 5              # ~100 ms
DISENGAGE_TAPER_FRAMES = 8               # ~160 ms; keeps LKAS_Request from edge-falling
CAMERA_SETTLE_FRAMES = 50                # ~1 s; raising LKAS_Request inside a camera LKAS-state transition hard-faults the EPS

# short dash lead before LKAS_Request rises; engage is latched so the request always follows and the dash is never stranded active
ENGAGE_DASH_LEAD_FRAMES = 8

# Roll compensation in actuators.steeringAngleDeg diverges as v -> 0 and cranks the wheel at stops on
# crowned roads; fade to a roll-free target rebuilt from actuators.curvature when approaching a stop.
ROLL_COMP_FADE_BP = [2.0, 8.0]           # m/s
ROLL_COMP_FADE_V  = [0.0, 1.0]

# Noise filter on the planner target; heavy at creep to kill jitter, lighter through 8-29 mph to cut LPF lag and damp the weave the highway-learned delay under-compensates
PLANNER_ANGLE_LP_ALPHA_BP = [0., 2.5, 3.5, 9., 18., 30.]    # m/s
PLANNER_ANGLE_LP_ALPHA_V  = [0.07, 0.11, 0.24, 0.28, 0.33, 0.30]


class AnglePlanner:
  """Jerk-limited motion planner for the LKAS_ANGLE command: bounds rate and
  acceleration so corrections build and release smoothly instead of stepping."""

  # Asymmetric like ANGLE_RATE_LIMIT_UP/DOWN. Below 4.5 m/s authority is deliberately reduced —
  # the model target flails at creep speed and full envelope rates saw the wheel at parking pace.
  MAX_RATE_BP     = [0., 0.9, 2.2, 3.1, 4.5, 15., 35.]              # m/s
  MAX_RATE_UP_V   = [0.35, 0.35, 0.45, 0.55, 0.72, 0.54, 0.18]      # deg/frame
  MAX_RATE_DOWN_V = [0.45, 0.45, 0.60, 0.75, 1.05, 0.80, 0.22]      # deg/frame

  # Tuned so reaching peak rate from rest takes ~0.25-0.30 s at every speed; gentler below 7 mph.
  MAX_ACCEL_BP = [0., 3.1, 5., 15., 35.]             # m/s
  MAX_ACCEL_V  = [0.025, 0.032, 0.035, 0.030, 0.012] # deg/frame^2

  # Scale accel up with error so big maneuvers (lane changes, recovery) don't feel sluggish.
  ERR_SCALE_BP = [1.5, 15.0]                         # deg wheel
  ERR_SCALE_V  = [1.0, 3.0]

  # Scale peak rate up with error too, so sharp turns slew faster; small/noisy errors keep the gentle base rate.
  RATE_ERR_SCALE_BP = [3.0, 15.0]                    # deg wheel
  RATE_ERR_SCALE_V  = [1.0, 2.0]

  # Safety angle-rate ceiling (mirrors CarControllerParams ANGLE_RATE_LIMIT) so the boost can't exceed the outer clip / panda.
  SAFETY_RATE_BP   = [0., 1.5, 5., 15., 35.]         # m/s
  SAFETY_RATE_UP   = [1.2, 1.0, 0.72, 0.54, 0.18]    # deg/frame
  SAFETY_RATE_DOWN = [1.7, 1.5, 1.05, 0.80, 0.22]    # deg/frame

  def __init__(self):
    self.pos = 0.0
    self.vel = 0.0

  def reset(self, angle: float) -> None:
    self.pos = float(angle)
    self.vel = 0.0

  def update(self, target: float, v_ego: float) -> float:
    err = float(target) - self.pos

    # moving away from center uses UP limits, unwinding toward center uses the looser DOWN limits
    winding_up = self.pos * np.sign(err) >= 0.
    rate_v = self.MAX_RATE_UP_V if winding_up else self.MAX_RATE_DOWN_V
    base_max_rate  = float(np.interp(v_ego, self.MAX_RATE_BP,  rate_v))
    rate_boost     = float(np.interp(abs(err), self.RATE_ERR_SCALE_BP, self.RATE_ERR_SCALE_V))
    safety_rate_v  = self.SAFETY_RATE_UP if winding_up else self.SAFETY_RATE_DOWN
    max_rate       = min(base_max_rate * rate_boost, float(np.interp(v_ego, self.SAFETY_RATE_BP, safety_rate_v)))
    base_max_accel = float(np.interp(v_ego, self.MAX_ACCEL_BP, self.MAX_ACCEL_V))
    max_accel = base_max_accel * float(np.interp(abs(err), self.ERR_SCALE_BP, self.ERR_SCALE_V))

    # v^2 = 2 a d  ->  brake distance to reach 0 from |vel| at max_accel
    brake_dist = (self.vel * self.vel) / (2.0 * max_accel) if max_accel > 0.0 else 0.0

    if abs(err) > brake_dist:
      desired_vel = np.sign(err) * max_rate
    else:
      desired_vel = np.sign(err) * np.sqrt(max(2.0 * max_accel * abs(err), 0.0))

    new_vel = float(np.clip(desired_vel, self.vel - max_accel, self.vel + max_accel))
    new_vel = float(np.clip(new_vel, -max_rate, max_rate))

    self.pos += new_vel
    self.vel = new_vel
    return self.pos


class LkasAngleStateMachine:
  def __init__(self, CP):
    self.VM = VehicleModel(CP)
    self.suspended = False
    self.below_release_count = 0
    self.pre_engage_clean_frames = 0
    self.disengage_taper_remaining = 0
    self.active_last = False
    self.dash_active = False
    self.dash_active_frames = 0
    self.dash_lead_frames = 0
    self.engaged = False
    self.enabled_last = False
    self.planner_angle_filt = 0.0
    self.last_lkas_button = 0
    self.lkas_button_settled = CAMERA_SETTLE_FRAMES
    self.planner = AnglePlanner()

  def _target_angle(self, CC, CS) -> float:
    """actuators.steeringAngleDeg with roll compensation faded out approaching a stop."""
    w = float(np.interp(CS.out.vEgoRaw, ROLL_COMP_FADE_BP, ROLL_COMP_FADE_V))
    if w >= 1.0:
      return CC.actuators.steeringAngleDeg
    angle_no_roll = math.degrees(self.VM.get_steer_from_curvature(-CC.actuators.curvature, CS.out.vEgoRaw, 0.0))
    return w * CC.actuators.steeringAngleDeg + (1.0 - w) * angle_no_roll

  def update(self, CC, CS):
    """Returns (commanded_angle, active) — feed to apply_std_steer_angle_limits."""
    torque = abs(CS.out.steeringTorque)
    extreme_angle = abs(CS.out.steeringAngleDeg) > MADS_ONLY_MAX_STEER_ANGLE
    extreme_angle_mads_only = extreme_angle and not CC.enabled
    target_angle = self._target_angle(CC, CS)

    # handoff is clear only when torque, wheel motion, and plan-vs-hand disagreement are all low
    handoff_clear = (torque < DRIVER_OVERRIDE_TORQUE_RELEASE
                     and abs(CS.out.steeringRateDeg) < WHEEL_SETTLED_RATE
                     and abs(target_angle - CS.out.steeringAngleDeg) < RESUME_MAX_TARGET_ERR
                     and not extreme_angle_mads_only)

    # camera-settle gate: block fresh engagement until the camera's LKAS state has been stable ~1 s
    lkas_button = int(getattr(CS, 'lkas_button', 0))
    if lkas_button != self.last_lkas_button:
      self.lkas_button_settled = 0
      self.last_lkas_button = lkas_button
    else:
      self.lkas_button_settled = min(self.lkas_button_settled + 1, CAMERA_SETTLE_FRAMES)

    # pre-engage clean-frame gate
    if handoff_clear:
      self.pre_engage_clean_frames = min(self.pre_engage_clean_frames + 1, PRE_ENGAGE_CLEAN_FRAMES)
    else:
      self.pre_engage_clean_frames = 0
    pre_engage_ok = (self.pre_engage_clean_frames >= PRE_ENGAGE_CLEAN_FRAMES
                     and self.lkas_button_settled >= CAMERA_SETTLE_FRAMES)

    # ACC dropping (e.g. brake) once suspended LKAS, but MADS lateral is independent of ACC: only
    # suspend when lateral is actually ending, so LKAS stays engaged through a brake while MADS holds it.
    if self.enabled_last and not CC.enabled and not CC.latActive:
      self.suspended = True
      self.below_release_count = 0
    self.enabled_last = CC.enabled

    # suspend hysteresis on driver override / extreme angle
    if self.suspended:
      if handoff_clear:
        self.below_release_count += 1
        if self.below_release_count >= SUSPEND_HOLD_FRAMES:
          self.suspended = False
          self.below_release_count = 0
      else:
        self.below_release_count = 0
    else:
      if torque > DRIVER_OVERRIDE_TORQUE or extreme_angle_mads_only:
        self.suspended = True
        self.below_release_count = 0

    # latch the engage once the gates pass so gate flicker (LKAS-button camera-state transitions) can't drop it mid-lead and strand an active dash with no LKAS_Request -> EPS LKAS fault
    raw_want = CC.latActive and not self.suspended
    if raw_want and (self.active_last or pre_engage_ok):
      self.engaged = True
    if not raw_want:
      self.engaged = False
    want_active = self.engaged

    if want_active and not self.active_last:
      self.planner_angle_filt = CS.out.steeringAngleDeg
      self.planner.reset(CS.out.steeringAngleDeg)

    # Taper holds LKAS_Request high briefly on clean disengage so the EyeSight watchdog doesn't
    # see a request edge; bypassed when suspended so command-vs-measured frames can't get dropped.
    if want_active:
      self.disengage_taper_remaining = DISENGAGE_TAPER_FRAMES
    elif self.disengage_taper_remaining > 0:
      self.disengage_taper_remaining -= 1

    # dash advertises intent (ES_LKAS_State); request is held back a lead so the dash reaches the EPS first.
    dash_active = want_active or (self.disengage_taper_remaining > 0 and not self.suspended)

    if dash_active:
      self.dash_active_frames = min(self.dash_active_frames + 1, ENGAGE_DASH_LEAD_FRAMES)
    else:
      self.dash_active_frames = 0

    request_active = dash_active and (self.active_last or self.dash_active_frames >= ENGAGE_DASH_LEAD_FRAMES)

    # safety net: EPS throws a LKAS fault if ES_LKAS_State advertises active without LKAS_Request, so never let the dash lead longer than the bounded engage lead
    self.dash_lead_frames = self.dash_lead_frames + 1 if (dash_active and not request_active) else 0
    if self.dash_lead_frames > ENGAGE_DASH_LEAD_FRAMES + 2:
      dash_active = False
      self.engaged = False

    if request_active:
      # Stage 1: LPF on the planner target (noise reject).
      alpha = np.interp(CS.out.vEgoRaw, PLANNER_ANGLE_LP_ALPHA_BP, PLANNER_ANGLE_LP_ALPHA_V)
      self.planner_angle_filt = alpha * target_angle + (1.0 - alpha) * self.planner_angle_filt

      # During taper, chase the live EPS angle for a smooth merge into the inactive path.
      target = self.planner_angle_filt if want_active else CS.out.steeringAngleDeg

      # Stage 2: jerk-limited trajectory (accel bound also shapes engage pull-in).
      out_angle = self.planner.update(target, CS.out.vEgoRaw)
    else:
      # inactive or holding for the lead: pin to measured so LKAS_Request rises from zero error, not a step
      self.planner_angle_filt = CS.out.steeringAngleDeg
      self.planner.reset(CS.out.steeringAngleDeg)
      out_angle = CS.out.steeringAngleDeg

    self.dash_active = dash_active
    self.active_last = request_active
    return out_angle, request_active
