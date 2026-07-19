import math
import unittest
from types import SimpleNamespace

from opendbc.car.subaru.carcontroller import CarController, MAX_LATERAL_ACCEL
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR


def _build_cs(v_ego_raw, steering_angle_deg):
  return SimpleNamespace(out=SimpleNamespace(
    vEgoRaw=v_ego_raw,
    steeringAngleDeg=steering_angle_deg,
    steeringRateDeg=0.0,
    steeringTorque=0.0,
    standstill=False,
    steeringPressed=False,
  ))


def _build_cc(target_angle_deg):
  return SimpleNamespace(
    latActive=True,
    enabled=True,
    actuators=SimpleNamespace(steeringAngleDeg=target_angle_deg, curvature=0.0),
  )


class TestAngleLateralAccelClamp(unittest.TestCase):
  def _build_controller(self):
    CP = CarInterface.get_non_essential_params(CAR.SUBARU_OUTBACK_2023)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.SUBARU_OUTBACK_2023)
    return CarController({}, CP, CP_SP)

  def _engage(self, controller, cc, cs):
    # drive the state machine through pre-engage clean frames and the dash lead
    for _ in range(20):
      controller.handle_angle_lateral(cc, cs)
    self.assertTrue(controller.angle_sm.active_last)

  def _max_angle(self, controller, v_ego):
    return math.degrees(controller.angle_sm.VM.get_steer_from_curvature(MAX_LATERAL_ACCEL / (v_ego ** 2), v_ego, 0.0))

  def test_clamp_binds_at_highway_speed(self):
    controller = self._build_controller()
    v_ego = 30.0
    cs = _build_cs(v_ego, 0.0)
    # engage with a target that agrees with the wheel (pre-engage gate), then demand a large angle
    self._engage(controller, _build_cc(0.0), cs)
    cc = _build_cc(100.0)

    # force the pipeline to a large commanded angle, beyond the accel bound at this speed
    controller.apply_angle_last = 100.0
    controller.angle_sm.planner.pos = 100.0
    controller.angle_sm.planner_angle_filt = 100.0
    controller.handle_angle_lateral(cc, cs)

    max_angle = self._max_angle(controller, v_ego)
    self.assertLess(max_angle, 40.0)
    self.assertLessEqual(controller.apply_angle_last, max_angle + 1e-9)

  def test_clamp_does_not_bind_at_parking_speed(self):
    controller = self._build_controller()
    v_ego = 2.0
    cs = _build_cs(v_ego, 0.0)
    cc = _build_cc(30.0)
    self._engage(controller, cc, cs)

    before = controller.apply_angle_last
    controller.handle_angle_lateral(cc, cs)
    after = controller.apply_angle_last

    # accel bound at parking speed is far beyond full lock, so the command keeps
    # slewing toward the target unimpeded by the clamp
    self.assertGreater(self._max_angle(controller, v_ego), 720.0)
    self.assertGreaterEqual(after, before)


if __name__ == "__main__":
  unittest.main()
