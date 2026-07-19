import numpy as np
try:
  from openpilot.common.params import Params
except ImportError:  # standalone opendbc (e.g. safety tests) — openpilot only exists in the full tree
  Params = None
from opendbc.can import CANPacker
from opendbc.car import Bus, make_tester_present_msg, structs
from opendbc.car.carlog import carlog
from opendbc.car.lateral import apply_center_deadzone, apply_driver_steer_torque_limits, apply_steer_angle_limits_vm, common_fault_avoidance
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.values import DBC, GLOBAL_ES_ADDR, CanBus, CarControllerParams, SubaruFlags
from opendbc.car.vehicle_model import VehicleModel

from opendbc.sunnypilot.car.subaru.stop_and_go import SnGCarController

# FIXME: These limits aren't exact. The real limit is more than likely over a larger time period and
# involves the total steering angle change rather than rate, but these limits work well for now
MAX_STEER_RATE = 25  # deg/s
MAX_STEER_RATE_FRAMES = 7  # tx control frames needed before torque can be cut
# The Subaru angle EPS hard-faults if the first LKAS command is issued while the steering
# wheel is rotating (independent of speed, accel, and wheel position). The torque path
# already guards against this via common_fault_avoidance; the angle path had no equivalent.
ANGLE_ENGAGE_MAX_STEER_RATE = 25.0  # deg/s — same EPS hardware as torque-path MAX_STEER_RATE
ANGLE_ENGAGE_RATE_SETTLE_FRAMES = 30  # 0.3 s at 100 Hz — wheel must be settled before engaging
LOW_SPEED_ANGLE_HOLD_SPEED = 2.24  # m/s (5 mph) — below this, slew-limit the commanded angle
LOW_SPEED_MIN_ANGLE_DELTA = 0.3    # deg/cmd step near standstill (~15 deg/s at 50 Hz), gentlest where EPS is most fault-prone
LOW_SPEED_MAX_ANGLE_DELTA = 3.0    # deg/cmd step approaching the threshold (~150 deg/s at 50 Hz)
MADS_ONLY_MIN_SPEED = 0.44704  # m/s (1 mph)
MADS_ONLY_MAX_STEER_ANGLE = 120.0  # deg
ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES = 10  # steering command frames (~200 ms with STEER_STEP=2)
ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES = 36  # validated default reclaim ramp (steering command frames, ~720 ms with STEER_STEP=2)
ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_MIN = 0
ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_MAX = 6
ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT = 4
ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS = [1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 3.5]
ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MIN = 1
ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MAX = 3
ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_DEFAULT = 2
ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS = [4, 8, 12]  # additional steering command frames (~80/160/240 ms)
ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_RATE_THRESHOLDS = [3.0, 2.0, 1.0]  # deg/s
ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_ANGLE_DELTA = 1.0  # deg
# Soft-capture engage blending (SubiPilot 1.1 staging experiment)
# Maps UI level 0 (off) / 1-5 to (ramp_frames, alpha_start) pairs.
# Level 5 is the most damped (longest ramp, gentlest start).
SOFT_CAPTURE_LEVEL_PARAMS = [
  # (ramp_frames, alpha_start)
  (0, 1.0),    # 0 - disabled (instant snap, stock behavior)
  (15, 0.25),  # 1 - light
  (22, 0.15),  # 2 - mild
  (30, 0.08),  # 3 - medium
  (40, 0.05),  # 4 - strong
  (50, 0.02),  # 5 - max
]


def get_safety_CP():
  # Use the Ascent for lateral limiting to match safety (most restrictive slip factor)
  from opendbc.car.subaru.interface import CarInterface
  return CarInterface.get_non_essential_params("SUBARU_ASCENT")


class CarController(CarControllerBase, SnGCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    SnGCarController.__init__(self, CP, CP_SP)
    self.apply_torque_last = 0
    self.apply_angle_last = 0

    self.cruise_button_prev = 0
    self.steer_rate_counter = 0
    self._debug_state = {}

    self.p = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])
    self.params = Params()
    self.mc_subaru_manual_yield_resume_softness_enabled = False
    self.mc_subaru_manual_yield_resume_softness = ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT
    self.mc_subaru_manual_yield_release_guard_enabled = False
    self.mc_subaru_manual_yield_release_guard_level = ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_DEFAULT
    self.mc_subaru_soft_capture_enabled = False
    self.mc_subaru_soft_capture_level = 3
    self.angle_driver_override_hold_frames = 0
    self.angle_driver_override_ramp_frames = 0
    self.angle_driver_override_ramp_total_frames = ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES
    self.angle_driver_override_ramp_start_angle = 0.0
    self.angle_driver_override_ramp_softness_exponent = ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS[ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT]
    self.angle_driver_override_release_guard_pending = False
    self.angle_driver_override_release_guard_confirm_frames = 0
    self.angle_driver_override_release_guard_required_frames = 0
    self.angle_driver_override_release_guard_reference_angle = 0.0
    self.angle_driver_override_release_guard_rate_threshold = 0.0
    self.lat_active_prev = False
    self.soft_capture_frame = -(SOFT_CAPTURE_LEVEL_PARAMS[-1][0] + 1)
    self.lkas_request_last = False
    self.last_high_steer_rate_frame = -ANGLE_ENGAGE_RATE_SETTLE_FRAMES

    if CP.flags & SubaruFlags.LKAS_ANGLE:
      self.VM = VehicleModel(get_safety_CP())

    self._update_params()

  def _log_transition(self, key, value, message):
    if self._debug_state.get(key) != value:
      carlog.info(f"subaru[{self.CP.carFingerprint}] {message}")
      self._debug_state[key] = value

  def _get_int_param(self, key: str, default: int = 0) -> int:
    value = self.params.get(key, return_default=True)
    try:
      return int(value)
    except (TypeError, ValueError):
      return default

  def _get_bool_param(self, key: str, default: bool = False) -> bool:
    value = self.params.get(key, return_default=True)
    if value is None:
      return default
    if isinstance(value, bool):
      return value
    if isinstance(value, bytes):
      return value not in (b"", b"0")
    if isinstance(value, str):
      return value not in ("", "0", "false", "False")
    return bool(value)

  @staticmethod
  def _get_resume_softness_exponent(softness_setting: int) -> float:
    return ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS[int(np.clip(
      softness_setting,
      ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_MIN,
      ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_MAX,
    ))]

  @staticmethod
  def _get_release_guard_confirm_frames(level: int) -> int:
    idx = int(np.clip(level, ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MIN, ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MAX)) - 1
    return ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[idx]

  @staticmethod
  def _get_release_guard_rate_threshold(level: int) -> float:
    idx = int(np.clip(level, ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MIN, ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MAX)) - 1
    return ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_RATE_THRESHOLDS[idx]

  def _manual_yield_handoff_enabled(self) -> bool:
    return self.mc_subaru_manual_yield_resume_softness_enabled or self.mc_subaru_manual_yield_release_guard_enabled

  def _get_soft_capture_level(self) -> int:
    if not self.mc_subaru_soft_capture_enabled:
      return 0

    return int(np.clip(
      self.mc_subaru_soft_capture_level,
      1,
      len(SOFT_CAPTURE_LEVEL_PARAMS) - 1,
    ))

  def _get_soft_capture_angle(self, model_target: float, wheel_angle: float) -> float:
    level = self._get_soft_capture_level()
    if level == 0:
      return model_target

    ramp_frames, alpha_start = SOFT_CAPTURE_LEVEL_PARAMS[level]
    frames_since_engage = max(0, self.frame - self.soft_capture_frame)

    if frames_since_engage >= ramp_frames:
      return model_target

    t = frames_since_engage / ramp_frames
    alpha = float(np.interp(t, [0.0, 1.0], [alpha_start, 1.0]))
    return wheel_angle + alpha * (model_target - wheel_angle)

  def _update_params(self):
    self.mc_subaru_manual_yield_resume_softness_enabled = self._get_bool_param("MCSubaruManualYieldResumeSoftnessEnabled")
    manual_yield_resume_softness = int(np.clip(
      self._get_int_param("MCSubaruManualYieldResumeSoftness", ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT),
      ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_MIN,
      ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_MAX,
    ))
    self.mc_subaru_manual_yield_resume_softness = manual_yield_resume_softness if self.mc_subaru_manual_yield_resume_softness_enabled \
      else ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT
    self.mc_subaru_manual_yield_release_guard_enabled = self._get_bool_param("MCSubaruManualYieldReleaseGuardEnabled")
    self.mc_subaru_manual_yield_release_guard_level = int(np.clip(
      self._get_int_param("MCSubaruManualYieldReleaseGuardLevel", ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_DEFAULT),
      ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MIN,
      ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MAX,
    ))
    self.mc_subaru_soft_capture_enabled = self._get_bool_param("MCSubaruSoftCaptureEnabled")
    self.mc_subaru_soft_capture_level = int(np.clip(
      self._get_int_param("MCSubaruSoftCaptureLevel", 3),
      1,
      len(SOFT_CAPTURE_LEVEL_PARAMS) - 1,
    ))

  def _reset_angle_driver_override_ramp(self):
    self.angle_driver_override_ramp_frames = 0
    self.angle_driver_override_ramp_total_frames = ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES
    self.angle_driver_override_ramp_start_angle = 0.0
    self.angle_driver_override_ramp_softness_exponent = ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS[ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT]

  def _reset_angle_driver_override_release_guard(self):
    self.angle_driver_override_release_guard_pending = False
    self.angle_driver_override_release_guard_confirm_frames = 0
    self.angle_driver_override_release_guard_required_frames = 0
    self.angle_driver_override_release_guard_reference_angle = 0.0
    self.angle_driver_override_release_guard_rate_threshold = 0.0

  def _reset_angle_driver_override_state(self):
    self.angle_driver_override_hold_frames = 0
    self._reset_angle_driver_override_release_guard()
    self._reset_angle_driver_override_ramp()

  def _start_angle_driver_override_ramp(self, measured_angle: float):
    self.angle_driver_override_ramp_frames = ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES
    self.angle_driver_override_ramp_total_frames = ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES
    self.angle_driver_override_ramp_start_angle = measured_angle
    self.angle_driver_override_ramp_softness_exponent = self._get_resume_softness_exponent(
      self.mc_subaru_manual_yield_resume_softness
    )

  def _start_angle_driver_override_release_guard(self, measured_angle: float):
    self.angle_driver_override_release_guard_pending = True
    self.angle_driver_override_release_guard_confirm_frames = 0
    self.angle_driver_override_release_guard_required_frames = self._get_release_guard_confirm_frames(
      self.mc_subaru_manual_yield_release_guard_level
    )
    self.angle_driver_override_release_guard_reference_angle = measured_angle
    self.angle_driver_override_release_guard_rate_threshold = self._get_release_guard_rate_threshold(
      self.mc_subaru_manual_yield_release_guard_level
    )

  def _update_angle_driver_override_release_guard(self, measured_angle: float, steering_rate: float) -> bool:
    if not self.angle_driver_override_release_guard_pending:
      return False

    within_angle_window = abs(measured_angle - self.angle_driver_override_release_guard_reference_angle) <= \
      ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_ANGLE_DELTA
    within_rate_window = abs(steering_rate) <= self.angle_driver_override_release_guard_rate_threshold

    if within_angle_window and within_rate_window:
      self.angle_driver_override_release_guard_confirm_frames += 1
    else:
      self.angle_driver_override_release_guard_confirm_frames = 0
      self.angle_driver_override_release_guard_reference_angle = measured_angle

    if self.angle_driver_override_release_guard_confirm_frames >= self.angle_driver_override_release_guard_required_frames:
      self._reset_angle_driver_override_release_guard()
      return True

    return False

  def _update_angle_driver_override_state(self, steering_pressed: bool, lkas_allowed: bool,
                                          measured_angle: float, steering_rate: float) -> tuple[bool, bool]:
    if not lkas_allowed or not self._manual_yield_handoff_enabled():
      self._reset_angle_driver_override_state()
      return False, False

    if steering_pressed:
      self.angle_driver_override_hold_frames = ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES
      self._reset_angle_driver_override_release_guard()
      self._reset_angle_driver_override_ramp()
      return True, False

    if self.angle_driver_override_hold_frames > 0:
      self.angle_driver_override_hold_frames -= 1
      if self.angle_driver_override_hold_frames == 0:
        if self.mc_subaru_manual_yield_release_guard_enabled:
          self._start_angle_driver_override_release_guard(measured_angle)
          return True, False
        if self.mc_subaru_manual_yield_resume_softness_enabled:
          return True, True
        return False, False
      return True, False

    if self.angle_driver_override_release_guard_pending:
      if self._update_angle_driver_override_release_guard(measured_angle, steering_rate):
        if self.mc_subaru_manual_yield_resume_softness_enabled:
          return True, True
        return False, False
      return True, False

    return False, False

  def _apply_angle_driver_override_ramp(self, live_steer_target: float) -> tuple[float, bool]:
    if self.angle_driver_override_ramp_frames <= 0:
      return live_steer_target, False

    progress = (self.angle_driver_override_ramp_total_frames - self.angle_driver_override_ramp_frames + 1) / \
      self.angle_driver_override_ramp_total_frames
    eased_progress = progress ** self.angle_driver_override_ramp_softness_exponent
    ramped_target = self.angle_driver_override_ramp_start_angle + eased_progress * (
      live_steer_target - self.angle_driver_override_ramp_start_angle
    )

    self.angle_driver_override_ramp_frames -= 1
    if self.angle_driver_override_ramp_frames <= 0:
      self._reset_angle_driver_override_ramp()

    return ramped_target, True

  def _get_angle_lkas_target(self, raw_target: float) -> float:
    # Keep the angle target stock/raw; tuning experiments live behind explicit toggles.
    return raw_target

  def handle_angle_lateral(self, CC, CS):
    lat_active_rising = CC.latActive and not self.lat_active_prev
    if lat_active_rising:
      # Re-anchor the first active angle command to the live wheel angle so panda
      # safety and the controller start from the same measured reference.
      self.apply_angle_last = CS.out.steeringAngleDeg

    # Angle-LKAS can hard fault during very low-speed MADS lateral-only maneuvers.
    # Keep MADS behavior above 1 mph, but block sharp parking-lot style steering in lateral-only mode.
    mads_only = CC.latActive and not CC.enabled
    mads_only_ok = CS.out.vEgoRaw > MADS_ONLY_MIN_SPEED and abs(CS.out.steeringAngleDeg) < MADS_ONLY_MAX_STEER_ANGLE
    lkas_allowed = CC.latActive and (CC.enabled or not mads_only or mads_only_ok) and \
      CS.out.gearShifter == structs.CarState.GearShifter.drive and not CS.out.standstill
    angle_driver_override, ramp_will_start = self._update_angle_driver_override_state(
      CS.out.steeringPressed,
      lkas_allowed,
      CS.out.steeringAngleDeg,
      CS.out.steeringRateDeg,
    )
    lkas_request = lkas_allowed and not angle_driver_override

    # Engagement steering-rate guard: the Subaru angle EPS hard-faults if the first LKAS
    # command is issued while the wheel is rotating (per multiple field reports, independent
    # of speed/accel/position). Only gate the inactive->active transition — once engaged,
    # openpilot moves the wheel itself so a nonzero rate is expected and must not disengage.
    if abs(CS.out.steeringRateDeg) > ANGLE_ENGAGE_MAX_STEER_RATE:
      self.last_high_steer_rate_frame = self.frame
    engage_rate_settled = True
    if not self.lkas_request_last:
      engage_rate_settled = (self.frame - self.last_high_steer_rate_frame) >= ANGLE_ENGAGE_RATE_SETTLE_FRAMES
      lkas_request = lkas_request and engage_rate_settled
    self.lkas_request_last = lkas_request

    inhibit_reason = "none"
    if not CC.latActive:
      inhibit_reason = "lat_inactive"
    elif angle_driver_override:
      inhibit_reason = "manual_override"
    elif CS.out.gearShifter != structs.CarState.GearShifter.drive:
      inhibit_reason = "gear_not_drive"
    elif CS.out.standstill:
      inhibit_reason = "standstill"
    elif mads_only and not mads_only_ok:
      inhibit_reason = "mads_below_min_speed" if CS.out.vEgoRaw <= MADS_ONLY_MIN_SPEED else "mads_angle_limit"
    elif not engage_rate_settled:
      inhibit_reason = "engage_rate_unsettled"

    self._log_transition("angle_lkas_inhibit", inhibit_reason, f"angle LKAS inhibit={inhibit_reason}")
    self._log_transition(
      "angle_driver_override_hold",
      self.angle_driver_override_hold_frames > 0,
      (
        f"angle driver override hold active={self.angle_driver_override_hold_frames > 0} "
        + f"frames={self.angle_driver_override_hold_frames} steeringPressed={CS.out.steeringPressed}"
      ),
    )
    self._log_transition(
      "angle_driver_override_release_guard",
      self.angle_driver_override_release_guard_pending,
      (
        f"angle driver override release guard active={self.angle_driver_override_release_guard_pending} "
        + f"frames={self.angle_driver_override_release_guard_confirm_frames}/"
        + f"{self.angle_driver_override_release_guard_required_frames} "
        + f"referenceAngle={self.angle_driver_override_release_guard_reference_angle:.2f} "
        + f"rateThreshold={self.angle_driver_override_release_guard_rate_threshold:.2f}"
      ),
    )

    steer_target = self._get_angle_lkas_target(CC.actuators.steeringAngleDeg)

    if ramp_will_start:
      self._start_angle_driver_override_ramp(CS.out.steeringAngleDeg)

    if lkas_request:
      steer_target, manual_override_ramp_active = self._apply_angle_driver_override_ramp(steer_target)
    else:
      manual_override_ramp_active = False

    handoff_active = angle_driver_override or ramp_will_start or manual_override_ramp_active
    self._log_transition(
      "angle_lkas_request",
      lkas_request,
      (
        f"angle LKAS request={lkas_request} inhibit={inhibit_reason} target={steer_target:.2f} "
        + f"lastApplied={self.apply_angle_last:.2f} measuredAngle={CS.out.steeringAngleDeg:.2f} "
        + f"measuredRate={CS.out.steeringRateDeg:.2f} handoffActive={handoff_active} "
        + f"rampActive={manual_override_ramp_active} latActive={CC.latActive} enabled={CC.enabled}"
      ),
    )

    self._log_transition(
      "angle_driver_override_ramp",
      manual_override_ramp_active,
      (
        f"angle driver override ramp active={manual_override_ramp_active} "
        + f"framesRemaining={self.angle_driver_override_ramp_frames} totalFrames={self.angle_driver_override_ramp_total_frames} "
        + f"softnessExponent={self.angle_driver_override_ramp_softness_exponent:.2f} "
        + f"start={self.angle_driver_override_ramp_start_angle:.2f} "
        + f"steerTarget={steer_target:.2f}"
      ),
    )

    if lat_active_rising and lkas_request:
      self.soft_capture_frame = self.frame
    self.lat_active_prev = CC.latActive

    soft_capture_blending = False
    if lkas_request:
      captured_target = self._get_soft_capture_angle(steer_target, CS.out.steeringAngleDeg)
      soft_capture_blending = captured_target != steer_target
      steer_target = captured_target

    # Heavy steering oscillation at low speeds — apply speed-dependent deadzone.
    # Skip it while an override-resume ramp or soft capture is actively blending: those
    # produce small transitional increments by design, which the deadzone would quantize.
    handoff_blending = manual_override_ramp_active or soft_capture_blending
    if lkas_request and not handoff_blending and CS.out.vEgoRaw < 10.0:
      deadzone = np.interp(CS.out.vEgoRaw, [2., 10.0], [6.0, 3.0])
      steer_target = self.apply_angle_last + apply_center_deadzone(steer_target - self.apply_angle_last, deadzone)

    # Below ~5 mph, the lateral planner can produce oscillating angle commands while the EPS is
    # already heavily loaded, which has caused permanent EPS faults. Rather than freezing the
    # wheel outright (dead steering plus a snap when crossing the threshold on a MADS resume),
    # track the target under a speed-scaled slew limit: gentlest near standstill where the EPS
    # is most fault-prone, ramping up toward the threshold so the handoff to the normal limiter
    # is seamless. Fast oscillation that faults the EPS is bounded out, but LKAS keeps following
    # the path through stop-and-go / low-speed resume instead of going dead.
    if lkas_request and CS.out.vEgoRaw < LOW_SPEED_ANGLE_HOLD_SPEED:
      low_speed_delta = float(np.interp(CS.out.vEgoRaw, [0.0, LOW_SPEED_ANGLE_HOLD_SPEED],
                                        [LOW_SPEED_MIN_ANGLE_DELTA, LOW_SPEED_MAX_ANGLE_DELTA]))
      steer_target = float(np.clip(steer_target, self.apply_angle_last - low_speed_delta,
                                   self.apply_angle_last + low_speed_delta))

    apply_steer = apply_steer_angle_limits_vm(
      steer_target,
      self.apply_angle_last,
      CS.out.vEgoRaw,
      CS.out.steeringAngleDeg,
      lkas_request,
      self.p,
      self.VM,
    )

    if not lkas_request:
      apply_steer = CS.out.steeringAngleDeg

    self.apply_angle_last = apply_steer
    return subarucan.create_steering_control_angle(self.packer, apply_steer, lkas_request)

  def handle_torque_lateral(self, CC, CS):
    apply_torque = int(round(CC.actuators.torque * self.p.STEER_MAX))

    new_torque = int(round(apply_torque))
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.p)

    if not CC.latActive:
      apply_torque = 0

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      msg = subarucan.create_preglobal_steering_control(self.packer, self.frame // self.p.STEER_STEP, apply_torque, CC.latActive)
    else:
      apply_steer_req = CC.latActive

      if self.CP.flags & SubaruFlags.STEER_RATE_LIMITED:
        # Steering rate fault prevention
        self.steer_rate_counter, apply_steer_req = common_fault_avoidance(
          abs(CS.out.steeringRateDeg) > MAX_STEER_RATE,
          apply_steer_req,
          self.steer_rate_counter,
          MAX_STEER_RATE_FRAMES,
        )

      msg = subarucan.create_steering_control(self.packer, apply_torque, apply_steer_req)

    self.apply_torque_last = apply_torque
    return msg

  def update(self, CC, CC_SP, CS, now_nanos):
    if self.frame % 100 == 0:
      self._update_params()

    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    # *** steering ***
    if (self.frame % self.p.STEER_STEP) == 0:
      if self.CP.flags & SubaruFlags.LKAS_ANGLE:
        can_sends.append(self.handle_angle_lateral(CC, CS))
      else:
        can_sends.append(self.handle_torque_lateral(CC, CS))

    # *** longitudinal ***

    if CC.longActive:
      apply_throttle = int(round(np.interp(actuators.accel, CarControllerParams.THROTTLE_LOOKUP_BP, CarControllerParams.THROTTLE_LOOKUP_V)))
      apply_rpm = int(round(np.interp(actuators.accel, CarControllerParams.RPM_LOOKUP_BP, CarControllerParams.RPM_LOOKUP_V)))
      apply_brake = int(round(np.interp(actuators.accel, CarControllerParams.BRAKE_LOOKUP_BP, CarControllerParams.BRAKE_LOOKUP_V)))

      # limit min and max values
      cruise_throttle = np.clip(apply_throttle, CarControllerParams.THROTTLE_MIN, CarControllerParams.THROTTLE_MAX)
      cruise_rpm = np.clip(apply_rpm, CarControllerParams.RPM_MIN, CarControllerParams.RPM_MAX)
      cruise_brake = np.clip(apply_brake, CarControllerParams.BRAKE_MIN, CarControllerParams.BRAKE_MAX)
    else:
      cruise_throttle = CarControllerParams.THROTTLE_INACTIVE
      cruise_rpm = CarControllerParams.RPM_MIN
      cruise_brake = CarControllerParams.BRAKE_MIN

    # *** alerts and pcm cancel ***
    if self.CP.flags & SubaruFlags.PREGLOBAL:
      if self.frame % 5 == 0:
        # 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
        # disengage ACC when OP is disengaged
        if pcm_cancel_cmd:
          cruise_button = 1
        # turn main on if off and past start-up state
        elif not CS.out.cruiseState.available and CS.ready:
          cruise_button = 1
        else:
          cruise_button = CS.cruise_button

        # unstick previous mocked button press
        if cruise_button == 1 and self.cruise_button_prev == 1:
          cruise_button = 0
        self.cruise_button_prev = cruise_button

        can_sends.append(subarucan.create_preglobal_es_distance(self.packer, cruise_button, CS.es_distance_msg))

    else:
      if self.frame % 10 == 0:
        can_sends.append(subarucan.create_es_dashstatus(self.packer, self.frame // 10, CS.es_dashstatus_msg, CC.enabled,
                                                        self.CP.openpilotLongitudinalControl, CC.longActive, hud_control.leadVisible))

        can_sends.append(subarucan.create_es_lkas_state(self.packer, self.frame // 10, CS.es_lkas_state_msg, CC.latActive, hud_control.visualAlert,
                                                        hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                        hud_control.leftLaneDepart, hud_control.rightLaneDepart))

        if self.CP.flags & SubaruFlags.SEND_INFOTAINMENT:
          can_sends.append(subarucan.create_es_infotainment(self.packer, self.frame // 10, CS.es_infotainment_msg, hud_control.visualAlert))

      if self.CP.openpilotLongitudinalControl:
        if self.frame % 5 == 0:
          can_sends.append(subarucan.create_es_status(self.packer, self.frame // 5, CS.es_status_msg,
                                                      self.CP.openpilotLongitudinalControl, CC.longActive, cruise_rpm))

          can_sends.append(subarucan.create_es_brake(self.packer, self.frame // 5, CS.es_brake_msg,
                                                     self.CP.openpilotLongitudinalControl, CC.longActive, cruise_brake))

          can_sends.append(subarucan.create_es_distance(self.packer, self.frame // 5, CS.es_distance_msg, 0, pcm_cancel_cmd,
                                                        self.CP.openpilotLongitudinalControl, cruise_brake > 0, cruise_throttle))
      else:
        if pcm_cancel_cmd:
          if not (self.CP.flags & SubaruFlags.HYBRID):
            bus = CanBus.alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else CanBus.main
            can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg["COUNTER"] + 1, CS.es_distance_msg, bus, pcm_cancel_cmd))

      if self.CP.flags & SubaruFlags.DISABLE_EYESIGHT:
        # Tester present (keeps eyesight disabled)
        if self.frame % 100 == 0:
          can_sends.append(make_tester_present_msg(GLOBAL_ES_ADDR, CanBus.camera, suppress_response=True))

        # Create all of the other eyesight messages to keep the rest of the car happy when eyesight is disabled
        if self.frame % 5 == 0:
          can_sends.append(subarucan.create_es_highbeamassist(self.packer))

        if self.frame % 10 == 0:
          can_sends.append(subarucan.create_es_static_1(self.packer))

        if self.frame % 2 == 0:
          can_sends.append(subarucan.create_es_static_2(self.packer))

    can_sends.extend(SnGCarController.create_stop_and_go(self, self.packer, CC, CS, self.frame))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.torque = self.apply_torque_last / self.p.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
