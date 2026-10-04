import unittest
from types import SimpleNamespace

from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR, CarControllerParams
from opendbc.sunnypilot.car.subaru.lateral_ext import LkasAngleStateMachine, LANE_CHANGE_NUDGE_MAX_TORQUE


def _cs(v_ego, torque=0.0, angle=0.0, left_bsm=False, right_bsm=False):
  return SimpleNamespace(out=SimpleNamespace(vEgoRaw=v_ego, steeringAngleDeg=angle, steeringRateDeg=0.0, steeringTorque=torque,
                                             leftBlindspot=left_bsm, rightBlindspot=right_bsm))


def _cc(target=0.0, left=False, right=False):
  return SimpleNamespace(enabled=True, latActive=True, leftBlinker=left, rightBlinker=right,
                         actuators=SimpleNamespace(steeringAngleDeg=target, curvature=0.0))


class TestLaneChangeNudge(unittest.TestCase):
  def _engaged_sm(self, v_ego=25.0):
    CP = CarInterface.get_non_essential_params(CAR.SUBARU_OUTBACK_2023)
    sm = LkasAngleStateMachine(CP, CarControllerParams(CP).ANGLE_LIMITS)
    for _ in range(20):
      _, active = sm.update(_cc(), _cs(v_ego))
    self.assertTrue(active)
    return sm

  def _stays_active(self, cc, torque, frames=50, **bsm):
    sm = self._engaged_sm()
    return all(sm.update(cc, _cs(25.0, torque, **bsm))[1] for _ in range(frames))

  def test_nudge_toward_lane_change_keeps_steering(self):
    # steeringTorque > 0 is a push to the left
    self.assertTrue(self._stays_active(_cc(left=True), 200.))
    self.assertTrue(self._stays_active(_cc(right=True), -200.))

  def test_push_against_lane_change_overrides(self):
    self.assertFalse(self._stays_active(_cc(left=True), -200.))
    self.assertFalse(self._stays_active(_cc(right=True), 200.))

  def test_hard_push_toward_lane_change_overrides(self):
    self.assertFalse(self._stays_active(_cc(left=True), LANE_CHANGE_NUDGE_MAX_TORQUE + 1.))

  def test_push_toward_occupied_blind_spot_overrides(self):
    self.assertFalse(self._stays_active(_cc(left=True), 200., left_bsm=True))
    self.assertFalse(self._stays_active(_cc(right=True), -200., right_bsm=True))
    # a car in the other lane doesn't matter
    self.assertTrue(self._stays_active(_cc(left=True), 200., right_bsm=True))

  def test_override_unchanged_without_lane_change(self):
    self.assertFalse(self._stays_active(_cc(), 200.))
    self.assertFalse(self._stays_active(_cc(), -200.))

  def _resumes(self, cc, torque):
    sm = self._engaged_sm()
    sm.update(_cc(), _cs(25.0, 300.))  # override outside a lane change
    self.assertFalse(sm.update(_cc(), _cs(25.0, 300.))[1])
    return any(sm.update(cc, _cs(25.0, torque))[1] for _ in range(60))

  def test_resume_still_needs_light_hands(self):
    self.assertFalse(self._resumes(_cc(left=True), 150.))
    self.assertFalse(self._resumes(_cc(), 150.))
    self.assertTrue(self._resumes(_cc(left=True), 50.))

  def _angle_after_step(self, v_ego, lane_change, frames=15):
    sm = self._engaged_sm(v_ego)
    for _ in range(frames):
      angle, _ = sm.update(_cc(10., left=lane_change), _cs(v_ego))
    return angle

  def test_planner_builds_faster_only_during_lane_change(self):
    self.assertGreater(self._angle_after_step(25., True), self._angle_after_step(25., False) + 0.1)

  def test_planner_unchanged_below_lane_change_speeds(self):
    self.assertAlmostEqual(self._angle_after_step(8., True), self._angle_after_step(8., False))


if __name__ == "__main__":
  unittest.main()
