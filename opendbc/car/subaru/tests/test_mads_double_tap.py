import unittest
from types import SimpleNamespace

from opendbc.car import Bus, structs
from opendbc.car.subaru.values import SubaruFlags
from opendbc.sunnypilot.car.subaru.mads import MadsCarState, LKAS_DOUBLE_TAP_WINDOW_FRAMES


class TestMadsDoubleTap(unittest.TestCase):
  def setUp(self):
    self.mads = MadsCarState(SimpleNamespace(flags=SubaruFlags.LKAS_ANGLE), None)
    self.dash = 0

  def _frames(self, n, dash=None):
    """Run n 100 Hz carstate frames with the stock LKAS dash state held at `dash`; return the lkas button events."""
    if dash is not None:
      self.dash = dash
    cp_cam = SimpleNamespace(vl={"ES_LKAS_State": {"LKAS_Dash_State": self.dash}})
    events = []
    for _ in range(n):
      ret = structs.CarState()
      self.mads.update_mads(ret, {Bus.cam: cp_cam})
      events += ret.buttonEvents
    return events

  def _tap(self):
    # each press flips the stock LKA, seen as an off <-> active dash state crossing
    return self._frames(20, 0 if self.dash else 2)

  def test_single_tap_is_ignored(self):
    self._frames(50)
    self.assertEqual(self._tap(), [])
    self.assertEqual(self._frames(LKAS_DOUBLE_TAP_WINDOW_FRAMES + 50), [])

  def test_double_tap_toggles_once(self):
    self._frames(50)
    events = self._tap() + self._frames(30) + self._tap()
    self.assertEqual(len(events), 1)
    self.assertEqual(events[0].type, structs.CarState.ButtonEvent.Type.lkas)

  def test_taps_outside_window_are_ignored(self):
    self._frames(50)
    events = self._tap() + self._frames(LKAS_DOUBLE_TAP_WINDOW_FRAMES) + self._tap()
    self.assertEqual(events, [])

  def test_triple_tap_toggles_once(self):
    # the first two taps pair up; the third starts a new, unmatched pair
    self._frames(50)
    events = self._tap() + self._frames(20) + self._tap() + self._frames(20) + self._tap() + self._frames(200)
    self.assertEqual(len(events), 1)

  def test_two_double_taps_toggle_twice(self):
    self._frames(50)
    events = self._tap() + self._frames(20) + self._tap() + self._frames(200) + self._tap() + self._frames(20) + self._tap()
    self.assertEqual(len(events), 2)

  def test_ready_active_flicker_is_not_a_tap(self):
    # Ready (1) <-> Active (2) stays inside the active range: not a button press
    self._frames(50, 1)
    events = self._frames(10, 2) + self._frames(10, 1) + self._frames(10, 2)
    self.assertEqual(events, [])


if __name__ == "__main__":
  unittest.main()
