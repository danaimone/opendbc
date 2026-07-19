import inspect
import unittest
from types import SimpleNamespace

from openpilot.common.params import Params
from opendbc.car import structs
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.carcontroller import (
  ANGLE_ENGAGE_MAX_STEER_RATE,
  ANGLE_ENGAGE_RATE_SETTLE_FRAMES,
  LOW_SPEED_ANGLE_HOLD_SPEED,
  LOW_SPEED_MIN_ANGLE_DELTA,
  LOW_SPEED_MAX_ANGLE_DELTA,
  ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES,
  ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT,
  ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES,
  ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS,
  ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS,
  ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_RATE_THRESHOLDS,
  CarController,
  MADS_ONLY_MIN_SPEED,
  SOFT_CAPTURE_LEVEL_PARAMS,
)
from opendbc.car.subaru.carstate import (
  CarState,
  MANUAL_YIELD_TORQUE_THRESHOLD_DEFAULT,
  MANUAL_YIELD_TORQUE_THRESHOLD_MAX,
  MANUAL_YIELD_TORQUE_THRESHOLD_MIN,
)
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR
from opendbc.car.tests.routes import routes


class TestSubaruCarController(unittest.TestCase):
  PARAM_KEYS = (
    "MCSubaruManualYieldResumeSoftnessEnabled",
    "MCSubaruManualYieldResumeSoftness",
    "MCSubaruManualYieldReleaseGuardEnabled",
    "MCSubaruManualYieldReleaseGuardLevel",
    "MCSubaruManualYieldTorqueThresholdEnabled",
    "MCSubaruManualYieldTorqueThreshold",
    "MCSubaruSoftCaptureEnabled",
    "MCSubaruSoftCaptureLevel",
  )

  def setUp(self):
    self.params = Params()
    for key in self.PARAM_KEYS:
      self.params.remove(key)

  def tearDown(self):
    for key in self.PARAM_KEYS:
      self.params.remove(key)

  @staticmethod
  def _build_cs(v_ego_raw, steering_angle_deg, steering_pressed=False, standstill=False, steering_rate_deg=0.0):
    return SimpleNamespace(out=SimpleNamespace(
      vEgoRaw=v_ego_raw,
      steeringAngleDeg=steering_angle_deg,
      steeringRateDeg=steering_rate_deg,
      gearShifter=structs.CarState.GearShifter.drive,
      standstill=standstill,
      steeringPressed=steering_pressed,
    ))

  @staticmethod
  def _build_cc(lat_active, enabled, steering_angle_deg):
    return SimpleNamespace(
      latActive=lat_active,
      enabled=enabled,
      actuators=SimpleNamespace(steeringAngleDeg=steering_angle_deg),
    )

  def _build_controller(self, *, soft_capture_enabled=False, soft_capture_level=3,
                        resume_softness_enabled=False, resume_softness_setting=None,
                        release_guard_enabled=False, release_guard_level=2):
    self.params.put_bool("MCSubaruSoftCaptureEnabled", soft_capture_enabled)
    self.params.put("MCSubaruSoftCaptureLevel", str(soft_capture_level))
    self.params.put_bool("MCSubaruManualYieldResumeSoftnessEnabled", resume_softness_enabled)
    self.params.put_bool("MCSubaruManualYieldReleaseGuardEnabled", release_guard_enabled)
    self.params.put("MCSubaruManualYieldReleaseGuardLevel", str(release_guard_level))
    if resume_softness_setting is not None:
      self.params.put("MCSubaruManualYieldResumeSoftness", str(resume_softness_setting))
    CP = CarInterface.get_non_essential_params(CAR.SUBARU_OUTBACK_2023)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.SUBARU_OUTBACK_2023)
    return CarController({}, CP, CP_SP)

  def _build_carstate(self, *, torque_threshold_enabled=False, torque_threshold=80):
    self.params.put_bool("MCSubaruManualYieldTorqueThresholdEnabled", torque_threshold_enabled)
    self.params.put("MCSubaruManualYieldTorqueThreshold", str(torque_threshold))
    CP = CarInterface.get_non_essential_params(CAR.SUBARU_OUTBACK_2023)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.SUBARU_OUTBACK_2023)
    return CarState(CP, CP_SP)

  @staticmethod
  def _set_resume_profile(controller, softness_setting=4):
    controller.mc_subaru_manual_yield_resume_softness = softness_setting

  @staticmethod
  def _build_release_guard_cs(v_ego_raw, steering_angle_deg=10.0, steering_rate_deg=0.0, steering_pressed=False):
    return TestSubaruCarController._build_cs(
      v_ego_raw,
      steering_angle_deg,
      steering_pressed=steering_pressed,
      steering_rate_deg=steering_rate_deg,
    )

  def _prime_angle_driver_override_ramp(self, controller, cc, v_ego_raw=8.0, measured_angle=10.0,
                                        softness_setting=4, use_current_profile=False):
    if not use_current_profile:
      controller.mc_subaru_manual_yield_resume_softness_enabled = True
      self._set_resume_profile(controller, softness_setting)
    controller.apply_angle_last = measured_angle

    controller.handle_angle_lateral(cc, self._build_cs(v_ego_raw, measured_angle, steering_pressed=True))
    released_cs = self._build_cs(v_ego_raw, measured_angle, steering_pressed=False)
    for _ in range(ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES):
      controller.handle_angle_lateral(cc, released_cs)

    expected_softness_setting = controller.mc_subaru_manual_yield_resume_softness
    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertEqual(controller.angle_driver_override_ramp_total_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertAlmostEqual(controller.angle_driver_override_ramp_start_angle, measured_angle)
    self.assertAlmostEqual(controller.angle_driver_override_ramp_softness_exponent, ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS[expected_softness_setting])
    return released_cs

  def _prime_angle_driver_override_release_guard(self, controller, cc, *, v_ego_raw=8.0, measured_angle=10.0,
                                                 steering_rate_deg=0.0, softness_setting=4,
                                                 use_current_profile=False):
    if not use_current_profile:
      controller.mc_subaru_manual_yield_resume_softness_enabled = True
      self._set_resume_profile(controller, softness_setting)
    controller.apply_angle_last = measured_angle

    controller.handle_angle_lateral(cc, self._build_cs(v_ego_raw, measured_angle, steering_pressed=True))
    released_cs = self._build_release_guard_cs(v_ego_raw, measured_angle, steering_rate_deg=steering_rate_deg)
    for _ in range(ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES):
      controller.handle_angle_lateral(cc, released_cs)

    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertTrue(controller.angle_driver_override_release_guard_pending)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)
    return released_cs

  def test_angle_driver_override_is_not_controller_inhibited_when_tuning_is_off_in_mads_only(self):
    controller = self._build_controller()
    cs = self._build_cs(9.5, 20.56, steering_pressed=True)
    # target far enough from measured to clear the low-speed anti-oscillation deadzone
    cc = self._build_cc(True, False, 26.0)

    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertNotAlmostEqual(controller.apply_angle_last, cs.out.steeringAngleDeg)
    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_is_not_controller_inhibited_when_tuning_is_off_in_full_engaged(self):
    controller = self._build_controller()
    cs = self._build_cs(9.5, 20.56, steering_pressed=True)
    # target far enough from measured to clear the low-speed anti-oscillation deadzone
    cc = self._build_cc(True, True, 26.0)

    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertNotAlmostEqual(controller.apply_angle_last, cs.out.steeringAngleDeg)
    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_hold_does_not_persist_when_tuning_is_off_in_mads_only(self):
    controller = self._build_controller()
    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True)
    cc = self._build_cc(True, False, 14.0)

    controller.apply_angle_last = cs_pressed.out.steeringAngleDeg
    controller.handle_angle_lateral(cc, cs_pressed)

    cs_released = self._build_cs(8.0, 10.0, steering_pressed=False)
    msg = controller.handle_angle_lateral(cc, cs_released)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs_released.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)
    self.assertGreater(controller.apply_angle_last, cs_released.out.steeringAngleDeg)

  def test_angle_driver_override_hold_does_not_persist_when_tuning_is_off_in_full_engaged(self):
    controller = self._build_controller()
    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True)
    cc = self._build_cc(True, True, 14.0)

    controller.apply_angle_last = cs_pressed.out.steeringAngleDeg
    controller.handle_angle_lateral(cc, cs_pressed)

    cs_released = self._build_cs(8.0, 10.0, steering_pressed=False)
    msg = controller.handle_angle_lateral(cc, cs_released)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs_released.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)
    self.assertGreater(controller.apply_angle_last, cs_released.out.steeringAngleDeg)

  def test_angle_driver_override_default_resume_profile_is_jacob_parity_off(self):
    controller = self._build_controller()
    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True)
    cc = self._build_cc(True, True, 14.0)

    controller.apply_angle_last = cs_pressed.out.steeringAngleDeg
    controller.handle_angle_lateral(cc, cs_pressed)

    self.assertFalse(controller._manual_yield_handoff_enabled())
    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_resume_softness_profiles_map_to_expected_exponents(self):
    expected_exponents = {
      0: 1.0,
      1: 1.25,
      2: 1.5,
      3: 2.0,
      4: 2.5,
      5: 3.0,
      6: 3.5,
    }

    for softness_setting, expected_exponent in expected_exponents.items():
      controller = self._build_controller()
      cc = self._build_cc(True, True, 14.0)

      self._prime_angle_driver_override_ramp(controller, cc, softness_setting=softness_setting)

      self.assertAlmostEqual(controller.angle_driver_override_ramp_softness_exponent, expected_exponent)

  def test_angle_driver_override_resume_softness_toggle_off_disables_reclaim_ramp(self):
    controller = self._build_controller(
      resume_softness_enabled=False,
      resume_softness_setting=6,
    )
    cc = self._build_cc(True, True, 14.0)
    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True)
    cs_released = self._build_cs(8.0, 10.0, steering_pressed=False)

    self.assertEqual(controller.mc_subaru_manual_yield_resume_softness, 4)

    controller.apply_angle_last = cs_pressed.out.steeringAngleDeg
    controller.handle_angle_lateral(cc, cs_pressed)
    controller.handle_angle_lateral(cc, cs_released)

    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_resume_softness_reenable_restores_saved_custom_exponent(self):
    disabled = self._build_controller(
      resume_softness_enabled=False,
      resume_softness_setting=6,
    )
    reenabled = self._build_controller(
      resume_softness_enabled=True,
      resume_softness_setting=6,
    )

    self.assertEqual(disabled.mc_subaru_manual_yield_resume_softness, ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT)
    self.assertEqual(reenabled.mc_subaru_manual_yield_resume_softness, 6)

  def test_angle_driver_override_release_guard_off_preserves_current_reclaim_timing(self):
    controller = self._build_controller(resume_softness_enabled=True, release_guard_enabled=False, release_guard_level=3)
    cc = self._build_cc(True, True, 14.0)

    self._prime_angle_driver_override_ramp(controller, cc)

    self.assertFalse(controller.angle_driver_override_release_guard_pending)
    self.assertEqual(controller.angle_driver_override_ramp_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)

  def test_angle_driver_override_release_guard_blocks_immediate_reclaim_after_hold_expiry(self):
    controller = self._build_controller(release_guard_enabled=True, release_guard_level=2)
    cc = self._build_cc(True, True, 14.0)
    released_cs = self._prime_angle_driver_override_release_guard(controller, cc)

    required_frames = ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[1]
    self.assertEqual(controller.angle_driver_override_release_guard_required_frames, required_frames)
    self.assertEqual(controller.angle_driver_override_release_guard_rate_threshold, ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_RATE_THRESHOLDS[1])

    for expected_frames in range(1, required_frames):
      controller.handle_angle_lateral(cc, released_cs)
      self.assertTrue(controller.angle_driver_override_release_guard_pending)
      self.assertEqual(controller.angle_driver_override_release_guard_confirm_frames, expected_frames)
      self.assertEqual(controller.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_release_guard_starts_ramp_after_quiet_confirmation(self):
    controller = self._build_controller(release_guard_enabled=True, release_guard_level=2)
    cc = self._build_cc(True, True, 14.0)
    released_cs = self._prime_angle_driver_override_release_guard(controller, cc)

    for _ in range(ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[1]):
      controller.handle_angle_lateral(cc, released_cs)

    self.assertFalse(controller.angle_driver_override_release_guard_pending)
    self.assertEqual(controller.angle_driver_override_ramp_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertEqual(controller.angle_driver_override_ramp_total_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertAlmostEqual(controller.angle_driver_override_ramp_start_angle, released_cs.out.steeringAngleDeg)

  def test_angle_driver_override_release_guard_without_resume_softness_does_not_start_ramp(self):
    controller = self._build_controller(release_guard_enabled=True, resume_softness_enabled=False, release_guard_level=2)
    cc = self._build_cc(True, True, 14.0)
    released_cs = self._prime_angle_driver_override_release_guard(controller, cc, use_current_profile=True)

    for _ in range(ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[1]):
      controller.handle_angle_lateral(cc, released_cs)

    self.assertFalse(controller.angle_driver_override_release_guard_pending)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_release_guard_cancels_when_driver_input_returns(self):
    controller = self._build_controller(release_guard_enabled=True, release_guard_level=2)
    cc = self._build_cc(True, True, 14.0)
    released_cs = self._prime_angle_driver_override_release_guard(controller, cc)

    controller.handle_angle_lateral(cc, released_cs)
    self.assertTrue(controller.angle_driver_override_release_guard_pending)

    pressed_cs = self._build_release_guard_cs(8.0, steering_angle_deg=10.0, steering_rate_deg=2.5, steering_pressed=True)
    msg = controller.handle_angle_lateral(cc, pressed_cs)
    expected = subarucan.create_steering_control_angle(controller.packer, pressed_cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertFalse(controller.angle_driver_override_release_guard_pending)
    self.assertEqual(controller.angle_driver_override_hold_frames, ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_release_guard_levels_change_confirmation_strictness(self):
    light = self._build_controller(release_guard_enabled=True, release_guard_level=1)
    strong = self._build_controller(release_guard_enabled=True, release_guard_level=3)
    cc = self._build_cc(True, True, 14.0)
    light_released_cs = self._prime_angle_driver_override_release_guard(light, cc, steering_rate_deg=2.5)
    strong_released_cs = self._prime_angle_driver_override_release_guard(strong, cc, steering_rate_deg=2.5)

    for _ in range(ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[0]):
      light.handle_angle_lateral(cc, light_released_cs)
      strong.handle_angle_lateral(cc, strong_released_cs)

    self.assertFalse(light.angle_driver_override_release_guard_pending)
    self.assertEqual(light.angle_driver_override_ramp_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertTrue(strong.angle_driver_override_release_guard_pending)
    self.assertEqual(strong.angle_driver_override_release_guard_confirm_frames, 0)
    self.assertEqual(strong.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_release_guard_reenable_restores_saved_strength(self):
    disabled = self._build_controller(release_guard_enabled=False, release_guard_level=3)
    enabled = self._build_controller(release_guard_enabled=True, release_guard_level=3)
    cc = self._build_cc(True, True, 14.0)

    self.assertFalse(disabled.mc_subaru_manual_yield_release_guard_enabled)
    self.assertEqual(disabled.mc_subaru_manual_yield_release_guard_level, 3)

    released_cs = self._prime_angle_driver_override_release_guard(enabled, cc)

    self.assertEqual(
      enabled.angle_driver_override_release_guard_required_frames,
      ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[2],
    )
    self.assertEqual(
      enabled.angle_driver_override_release_guard_rate_threshold,
      ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_RATE_THRESHOLDS[2],
    )
    self.assertTrue(enabled.angle_driver_override_release_guard_pending)
    self.assertAlmostEqual(released_cs.out.steeringAngleDeg, 10.0)

  def test_angle_driver_override_release_guard_preserves_fixed_resume_timing_and_custom_softness_after_confirmation(self):
    controller = self._build_controller(
      release_guard_enabled=True,
      release_guard_level=2,
      resume_softness_enabled=True,
      resume_softness_setting=6,
    )
    cc = self._build_cc(True, True, 14.0)
    released_cs = self._prime_angle_driver_override_release_guard(controller, cc, use_current_profile=True)

    for _ in range(ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[1]):
      controller.handle_angle_lateral(cc, released_cs)

    self.assertEqual(controller.angle_driver_override_ramp_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertEqual(controller.angle_driver_override_ramp_total_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertAlmostEqual(controller.angle_driver_override_ramp_softness_exponent, ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS[6])

  def test_angle_driver_override_ramp_progresses_monotonically_toward_live_target_in_mads_only(self):
    controller = self._build_controller()
    cc = self._build_cc(True, False, 14.0)
    cs_released = self._prime_angle_driver_override_ramp(controller, cc)

    ramped_angles = []
    for _ in range(6):
      controller.handle_angle_lateral(cc, cs_released)
      ramped_angles.append(controller.apply_angle_last)

    self.assertTrue(all(left <= right for left, right in zip(ramped_angles, ramped_angles[1:], strict=True)))
    self.assertGreater(ramped_angles[-1], cs_released.out.steeringAngleDeg)
    self.assertLessEqual(ramped_angles[-1], cc.actuators.steeringAngleDeg)

  def test_angle_driver_override_ramp_uses_live_target_in_full_engaged(self):
    controller = self._build_controller()
    cc_release = self._build_cc(True, True, 14.0)
    cs_released = self._prime_angle_driver_override_ramp(controller, cc_release)
    cc_changed = self._build_cc(True, True, 18.0)

    ramped_angles = []
    for _ in range(10):
      controller.handle_angle_lateral(cc_changed, cs_released)
      ramped_angles.append(controller.apply_angle_last)

    self.assertTrue(all(left <= right for left, right in zip(ramped_angles, ramped_angles[1:], strict=True)))
    self.assertGreater(ramped_angles[-1], 14.0)
    self.assertLessEqual(ramped_angles[-1], cc_changed.actuators.steeringAngleDeg)

  def test_angle_driver_override_softer_profiles_reduce_the_initial_reclaim_delta(self):
    cc = self._build_cc(True, True, 14.0)

    standard_controller = self._build_controller()
    extra_soft_controller = self._build_controller()
    max_soft_controller = self._build_controller()

    standard_released_cs = self._prime_angle_driver_override_ramp(standard_controller, cc, softness_setting=0)
    extra_soft_released_cs = self._prime_angle_driver_override_ramp(extra_soft_controller, cc, softness_setting=4)
    max_soft_released_cs = self._prime_angle_driver_override_ramp(max_soft_controller, cc, softness_setting=6)

    standard_controller.handle_angle_lateral(cc, standard_released_cs)
    extra_soft_controller.handle_angle_lateral(cc, extra_soft_released_cs)
    max_soft_controller.handle_angle_lateral(cc, max_soft_released_cs)

    standard_delta = standard_controller.apply_angle_last - standard_released_cs.out.steeringAngleDeg
    extra_soft_delta = extra_soft_controller.apply_angle_last - extra_soft_released_cs.out.steeringAngleDeg
    max_soft_delta = max_soft_controller.apply_angle_last - max_soft_released_cs.out.steeringAngleDeg

    self.assertGreater(standard_delta, 0.0)
    self.assertGreater(extra_soft_delta, 0.0)
    self.assertGreater(max_soft_delta, 0.0)
    self.assertLess(extra_soft_delta, standard_delta)
    self.assertLess(max_soft_delta, extra_soft_delta)

  def test_angle_driver_override_ramp_cancels_when_driver_input_returns(self):
    controller = self._build_controller()
    cc = self._build_cc(True, True, 14.0)
    cs_released = self._prime_angle_driver_override_ramp(controller, cc)

    controller.handle_angle_lateral(cc, cs_released)
    self.assertLess(controller.angle_driver_override_ramp_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)

    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True, steering_rate_deg=2.0)
    msg = controller.handle_angle_lateral(cc, cs_pressed)
    expected = subarucan.create_steering_control_angle(controller.packer, cs_pressed.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertEqual(controller.angle_driver_override_hold_frames, ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)
    self.assertAlmostEqual(controller.apply_angle_last, cs_pressed.out.steeringAngleDeg)

  def test_soft_capture_disabled_is_a_no_op(self):
    controller = self._build_controller(soft_capture_enabled=False, soft_capture_level=5)
    controller.soft_capture_frame = controller.frame

    self.assertEqual(controller._get_soft_capture_level(), 0)
    self.assertAlmostEqual(controller._get_soft_capture_angle(18.0, 10.0), 18.0)

  def test_soft_capture_reenable_restores_saved_level(self):
    disabled = self._build_controller(soft_capture_enabled=False, soft_capture_level=5)
    reenabled = self._build_controller(soft_capture_enabled=True, soft_capture_level=5)

    self.assertEqual(disabled._get_soft_capture_level(), 0)
    self.assertEqual(reenabled._get_soft_capture_level(), 5)

  def test_soft_capture_engage_edge_starts_ramp_and_reduces_first_reclaim_step(self):
    baseline = self._build_controller(soft_capture_enabled=False)
    softened = self._build_controller(soft_capture_enabled=True, soft_capture_level=3)
    cc = self._build_cc(True, True, 14.0)
    cs = self._build_cs(8.0, 10.0)

    baseline.apply_angle_last = cs.out.steeringAngleDeg
    softened.apply_angle_last = cs.out.steeringAngleDeg

    baseline.handle_angle_lateral(cc, cs)
    softened.handle_angle_lateral(cc, cs)

    self.assertEqual(softened.soft_capture_frame, 0)
    self.assertTrue(softened.lat_active_prev)
    self.assertLess(softened.apply_angle_last, baseline.apply_angle_last)

  def test_fresh_angle_engage_reanchors_safety_reference_to_measured_steering(self):
    controller = self._build_controller(soft_capture_enabled=False)
    cc = self._build_cc(True, True, 20.0)
    cs = self._build_cs(8.0, 20.0)
    controller.apply_angle_last = -30.0

    controller.handle_angle_lateral(cc, cs)

    self.assertAlmostEqual(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_soft_capture_higher_levels_reduce_the_initial_blend_delta(self):
    light = self._build_controller(soft_capture_enabled=True, soft_capture_level=1)
    medium = self._build_controller(soft_capture_enabled=True, soft_capture_level=3)
    maximum = self._build_controller(soft_capture_enabled=True, soft_capture_level=5)

    for controller in (light, medium, maximum):
      controller.soft_capture_frame = 0
      controller.frame = 0

    model_target = 20.0
    wheel_angle = 10.0
    light_delta = light._get_soft_capture_angle(model_target, wheel_angle) - wheel_angle
    medium_delta = medium._get_soft_capture_angle(model_target, wheel_angle) - wheel_angle
    max_delta = maximum._get_soft_capture_angle(model_target, wheel_angle) - wheel_angle

    self.assertGreater(light_delta, medium_delta)
    self.assertGreater(medium_delta, max_delta)

  def test_soft_capture_ramp_completes_and_returns_full_model_control(self):
    controller = self._build_controller(soft_capture_enabled=True, soft_capture_level=3)
    ramp_frames, _ = SOFT_CAPTURE_LEVEL_PARAMS[3]
    controller.soft_capture_frame = 0
    controller.frame = ramp_frames

    self.assertAlmostEqual(controller._get_soft_capture_angle(18.0, 10.0), 18.0)

  def test_soft_capture_does_not_stack_on_manual_override_reclaim(self):
    baseline = self._build_controller(soft_capture_enabled=False)
    softened = self._build_controller(soft_capture_enabled=True, soft_capture_level=5)
    cc = self._build_cc(True, True, 14.0)

    baseline_released_cs = self._prime_angle_driver_override_ramp(baseline, cc)
    softened_released_cs = self._prime_angle_driver_override_ramp(softened, cc)

    self.assertEqual(softened.soft_capture_frame, -(SOFT_CAPTURE_LEVEL_PARAMS[-1][0] + 1))

    baseline.handle_angle_lateral(cc, baseline_released_cs)
    softened.handle_angle_lateral(cc, softened_released_cs)

    self.assertAlmostEqual(softened.apply_angle_last, baseline.apply_angle_last)
    self.assertEqual(softened.soft_capture_frame, -(SOFT_CAPTURE_LEVEL_PARAMS[-1][0] + 1))

  def test_manual_yield_torque_threshold_only_changes_when_enabled(self):
    disabled = self._build_carstate(torque_threshold_enabled=False, torque_threshold=MANUAL_YIELD_TORQUE_THRESHOLD_MAX)
    clamped = self._build_carstate(torque_threshold_enabled=True, torque_threshold=10)
    floor = self._build_carstate(torque_threshold_enabled=True, torque_threshold=MANUAL_YIELD_TORQUE_THRESHOLD_MIN)
    high = self._build_carstate(torque_threshold_enabled=True, torque_threshold=MANUAL_YIELD_TORQUE_THRESHOLD_MAX)
    ceiling = self._build_carstate(torque_threshold_enabled=True, torque_threshold=MANUAL_YIELD_TORQUE_THRESHOLD_MAX + 50)

    self.assertEqual(disabled._get_active_manual_yield_torque_threshold(), MANUAL_YIELD_TORQUE_THRESHOLD_DEFAULT)
    self.assertEqual(clamped._get_active_manual_yield_torque_threshold(), MANUAL_YIELD_TORQUE_THRESHOLD_MIN)
    self.assertEqual(floor._get_active_manual_yield_torque_threshold(), MANUAL_YIELD_TORQUE_THRESHOLD_MIN)
    self.assertEqual(high._get_active_manual_yield_torque_threshold(), MANUAL_YIELD_TORQUE_THRESHOLD_MAX)
    self.assertEqual(ceiling._get_active_manual_yield_torque_threshold(), MANUAL_YIELD_TORQUE_THRESHOLD_MAX)

  def test_manual_yield_torque_threshold_update_uses_direct_jacob_threshold_without_hysteresis(self):
    update_source = inspect.getsource(CarState.update)

    self.assertIn("ret.steeringPressed = abs(ret.steeringTorque) > steer_threshold", update_source)
    self.assertNotIn("update_steering_pressed", update_source)

  def test_lkas_hud_state_uses_lateral_active_not_full_openpilot_enabled(self):
    update_source = inspect.getsource(CarController.update)

    self.assertIn("create_es_lkas_state(self.packer, self.frame // 10, CS.es_lkas_state_msg, CC.latActive", update_source)
    self.assertNotIn("create_es_lkas_state(self.packer, self.frame // 10, CS.es_lkas_state_msg, CC.enabled", update_source)

  def test_mads_only_below_one_mph_still_inhibits_angle_lkas(self):
    controller = self._build_controller()
    cs = self._build_cs(0.22352, 10.0)
    cc = self._build_cc(True, False, 14.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    expected = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertAlmostEqual(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_mads_only_just_above_one_mph_allows_angle_lkas(self):
    controller = self._build_controller()
    cs = self._build_cs(MADS_ONLY_MIN_SPEED + 0.01, 10.0)
    # target far enough from measured to clear the low-speed anti-oscillation deadzone
    cc = self._build_cc(True, False, 20.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertGreater(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_mads_only_standstill_still_inhibits_above_one_mph(self):
    controller = self._build_controller()
    cs = self._build_cs(MADS_ONLY_MIN_SPEED + 0.5, 10.0, standstill=True)
    cc = self._build_cc(True, False, 14.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    expected = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertAlmostEqual(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_mads_only_angle_limit_still_inhibits_above_one_mph(self):
    controller = self._build_controller()
    cs = self._build_cs(MADS_ONLY_MIN_SPEED + 0.5, 120.0)
    cc = self._build_cc(True, False, 124.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    expected = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertAlmostEqual(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_full_engaged_lateral_ignores_mads_only_low_speed_floor(self):
    controller = self._build_controller()
    cs = self._build_cs(0.22352, 10.0)
    # target far enough from measured to clear the low-speed anti-oscillation deadzone
    cc = self._build_cc(True, True, 20.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertGreater(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_engagement_blocked_while_wheel_rotating(self):
    # The angle EPS hard-faults if the first LKAS command arrives while the wheel is moving
    controller = self._build_controller()
    cs = self._build_cs(9.5, 0.0, steering_rate_deg=ANGLE_ENGAGE_MAX_STEER_RATE * 4)
    cc = self._build_cc(True, True, 10.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    controller.handle_angle_lateral(cc, cs)

    self.assertFalse(controller.lkas_request_last)
    self.assertAlmostEqual(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_engagement_allowed_after_rate_settles(self):
    controller = self._build_controller()
    cc = self._build_cc(True, True, 10.0)
    controller.apply_angle_last = 0.0

    # wheel spinning at frame 0 — engagement must be blocked
    spinning = self._build_cs(9.5, 0.0, steering_rate_deg=ANGLE_ENGAGE_MAX_STEER_RATE * 4)
    controller.handle_angle_lateral(cc, spinning)
    self.assertFalse(controller.lkas_request_last)

    # settled for the full window — engagement proceeds
    settled = self._build_cs(9.5, 0.0, steering_rate_deg=0.0)
    controller.frame = ANGLE_ENGAGE_RATE_SETTLE_FRAMES
    controller.handle_angle_lateral(cc, settled)

    self.assertTrue(controller.lkas_request_last)
    self.assertGreater(controller.apply_angle_last, settled.out.steeringAngleDeg)

  def test_engaged_lkas_not_dropped_by_high_steer_rate(self):
    # once engaged, openpilot moves the wheel itself — a high rate must not disengage
    controller = self._build_controller()
    cc = self._build_cc(True, True, 10.0)
    controller.apply_angle_last = 0.0

    settled = self._build_cs(9.5, 0.0, steering_rate_deg=0.0)
    controller.handle_angle_lateral(cc, settled)
    self.assertTrue(controller.lkas_request_last)

    fast = self._build_cs(9.5, 1.0, steering_rate_deg=ANGLE_ENGAGE_MAX_STEER_RATE * 4)
    controller.frame += 1
    controller.handle_angle_lateral(cc, fast)
    self.assertTrue(controller.lkas_request_last)

  def test_low_speed_slew_limits_angle_step(self):
    # below ~5 mph the commanded angle must creep toward the target, never jump
    controller = self._build_controller()
    v_ego = 1.0
    cs = self._build_cs(v_ego, 0.0)
    cc = self._build_cc(True, True, 20.0)
    controller.apply_angle_last = 0.0

    controller.handle_angle_lateral(cc, cs)

    expected_delta = LOW_SPEED_MIN_ANGLE_DELTA + \
      (v_ego / LOW_SPEED_ANGLE_HOLD_SPEED) * (LOW_SPEED_MAX_ANGLE_DELTA - LOW_SPEED_MIN_ANGLE_DELTA)
    self.assertGreater(controller.apply_angle_last, 0.0)
    self.assertAlmostEqual(controller.apply_angle_last, expected_delta, places=5)

  def test_retired_low_speed_tuning_stack_keeps_raw_angle_target(self):
    controller = self._build_controller()

    target = controller._get_angle_lkas_target(1.2)

    self.assertAlmostEqual(target, 1.2)

  def test_outback_2023_angle_steering_route_still_present(self):
    route = next(route for route in routes if route.platform == CAR.SUBARU_OUTBACK_2023)
    self.assertEqual(route.platform, CAR.SUBARU_OUTBACK_2023)

  def test_crosstrek_2025_fw_versions_still_present(self):
    self.assertIn(CAR.SUBARU_CROSSTREK_2025, FW_VERSIONS)


if __name__ == "__main__":
  unittest.main()
