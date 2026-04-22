import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, make_tester_present_msg, structs
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
MADS_ONLY_MIN_SPEED = 2.24  # m/s (5 mph)
MADS_ONLY_MAX_STEER_ANGLE = 120.0  # deg
MADS_MANUAL_OVERRIDE_RELEASE_FRAMES = 30  # 0.3 s at 100 Hz
LOW_SPEED_ANGLE_HOLD_SPEED = 2.24  # m/s (5 mph) — below this, freeze the commanded angle
LOW_SPEED_HIGH_ANGLE_GUARD_MAX_SPEED = 2.7  # m/s (6 mph)
LOW_SPEED_HIGH_ANGLE_GUARD_MAX_STEER_ANGLE = 135.0  # deg
POST_NON_DRIVE_COOLDOWN_MAX_SPEED = 4.4704  # m/s (10 mph)
POST_NON_DRIVE_COOLDOWN_FRAMES = 150  # 1.5 s at 100 Hz


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
    self.last_non_drive_frame = -POST_NON_DRIVE_COOLDOWN_FRAMES
    self.last_mads_manual_override_frame = -MADS_MANUAL_OVERRIDE_RELEASE_FRAMES

    self.p = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])

    if CP.flags & SubaruFlags.LKAS_ANGLE:
      self.VM = VehicleModel(get_safety_CP())

  def handle_angle_lateral(self, CC, CS):
    # Angle-LKAS can hard fault during low-speed MADS lateral-only maneuvers.
    # Keep MADS behavior above 5 mph, but block sharp parking-lot style steering in lateral-only mode.
    in_drive = CS.out.gearShifter == structs.CarState.GearShifter.drive
    if not in_drive:
      self.last_non_drive_frame = self.frame

    mads_only = CC.latActive and not CC.enabled
    if mads_only and CS.out.steeringPressed:
      self.last_mads_manual_override_frame = self.frame

    mads_manual_override = mads_only and (
      CS.out.steeringPressed or
      (self.frame - self.last_mads_manual_override_frame) < MADS_MANUAL_OVERRIDE_RELEASE_FRAMES
    )
    mads_only_ok = CS.out.vEgoRaw > MADS_ONLY_MIN_SPEED and abs(CS.out.steeringAngleDeg) < MADS_ONLY_MAX_STEER_ANGLE
    low_speed_high_angle_guard = CS.out.vEgoRaw < LOW_SPEED_HIGH_ANGLE_GUARD_MAX_SPEED and \
      abs(CS.out.steeringAngleDeg) > LOW_SPEED_HIGH_ANGLE_GUARD_MAX_STEER_ANGLE
    post_non_drive_cooldown_guard = CS.out.vEgoRaw < POST_NON_DRIVE_COOLDOWN_MAX_SPEED and \
      (self.frame - self.last_non_drive_frame) < POST_NON_DRIVE_COOLDOWN_FRAMES
    lkas_request = CC.latActive and (CC.enabled or not mads_only or mads_only_ok) and in_drive and \
      not CS.out.standstill and not low_speed_high_angle_guard and not post_non_drive_cooldown_guard and \
      not mads_manual_override

    apply_angle = CC.actuators.steeringAngleDeg
    # Heavy steering oscillation at low speeds — apply speed-dependent deadzone
    if lkas_request and CS.out.vEgoRaw < 10.0:
      deadzone = np.interp(CS.out.vEgoRaw, [2., 10.0], [6.0, 3.0])
      apply_angle = self.apply_angle_last + apply_center_deadzone(apply_angle - self.apply_angle_last, deadzone)

    # Below ~5 mph, the lateral planner can produce oscillating angle commands while the EPS is
    # already heavily loaded, which has caused permanent EPS faults. Freeze the commanded angle
    # below LOW_SPEED_ANGLE_HOLD_SPEED so LKAS keeps whatever wheel position it had — the driver
    # retains manual steering authority, and LKAS resumes tracking once speed rises. This keeps
    # LKAS engaged through stop-lights / stop-and-go without reacting to planner noise.
    if lkas_request and CS.out.vEgoRaw < LOW_SPEED_ANGLE_HOLD_SPEED:
      apply_angle = self.apply_angle_last

    self.apply_angle_last = apply_steer_angle_limits_vm(apply_angle, self.apply_angle_last, CS.out.vEgoRaw,
                                                        CS.out.steeringAngleDeg, lkas_request, CarControllerParams, self.VM)

    return subarucan.create_steering_control_angle(self.packer, self.apply_angle_last, lkas_request)

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

        can_sends.append(subarucan.create_es_lkas_state(self.packer, self.frame // 10, CS.es_lkas_state_msg, CC.enabled, hud_control.visualAlert,
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
