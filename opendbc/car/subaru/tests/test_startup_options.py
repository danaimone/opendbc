import pytest

from opendbc.car import structs
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR, SubaruFlags
from opendbc.sunnypilot.car.interfaces import _initialize_subaru_startup_preferences
from opendbc.sunnypilot.car.subaru.startup_preferences import (
  AVH_REQUEST, BUS, GEAR, REQUIRED, STOP_REQUEST, STOP_STATUS, THROTTLE,
  StartupPreferences, checksum, startup_preferences_supported,
)
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP, SubaruSafetyFlagsSP

AVH_OPTION = "SubaruEnableAVHAtStartup"
STOP_OPTION = "SubaruDisableStartStopAtStartup"
AUTHORIZATION = "SubaruStartupCycleAuthorized"


@pytest.fixture
def supported_params():
  """Simulate the separately reviewed angle port; never change upstream defaults."""
  cp = CarInterface.get_non_essential_params(CAR.SUBARU_OUTBACK_2023)
  assert cp.dashcamOnly
  cp.dashcamOnly = False
  cp.passive = False
  return cp, structs.CarParamsSP()


def test_upstream_outback_remains_inactive():
  cp = CarInterface.get_non_essential_params(CAR.SUBARU_OUTBACK_2023)
  sp = structs.CarParamsSP()
  assert cp.dashcamOnly
  assert not startup_preferences_supported(cp)
  _initialize_subaru_startup_preferences(cp, sp, {AVH_OPTION: "1", STOP_OPTION: "1", AUTHORIZATION: True})
  assert sp.flags == 0
  assert sp.safetyParam == 0
  assert cp.dashcamOnly


@pytest.mark.parametrize("avh,stop", [(False, False), (True, False), (False, True), (True, True)])
def test_independent_options_preserve_existing_configuration(supported_params, avh, stop):
  cp, sp = supported_params
  sp.flags = SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE
  sp.safetyParam = SubaruSafetyFlagsSP.STOP_AND_GO
  original_cp = cp.to_dict()
  options = {AVH_OPTION: str(int(avh)), STOP_OPTION: str(int(stop)), AUTHORIZATION: True}
  _initialize_subaru_startup_preferences(cp, sp, options)
  expected_flags = SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE
  expected_safety = SubaruSafetyFlagsSP.STOP_AND_GO
  if avh:
    expected_flags |= SubaruFlagsSP.ENABLE_AVH_AT_STARTUP
    expected_safety |= SubaruSafetyFlagsSP.ENABLE_AVH_AT_STARTUP
  if stop:
    expected_flags |= SubaruFlagsSP.DISABLE_START_STOP_AT_STARTUP
    expected_safety |= SubaruSafetyFlagsSP.DISABLE_START_STOP_AT_STARTUP
  assert sp.flags == expected_flags
  assert sp.safetyParam == expected_safety
  assert cp.to_dict() == original_cp
  assert options[AUTHORIZATION] is True  # Caller owns the consumed ignition token.


@pytest.mark.parametrize("options", [{}, {AUTHORIZATION: True}, {AVH_OPTION: "1", STOP_OPTION: "1"}])
def test_defaults_and_missing_permission_are_disabled(supported_params, options):
  cp, sp = supported_params
  _initialize_subaru_startup_preferences(cp, sp, options)
  assert sp.flags == sp.safetyParam == 0


@pytest.mark.parametrize("permission", [False, None, "", "0", "1", "false", "true", 0, 1, [], {}])
def test_malformed_ephemeral_permission_cannot_authorize(supported_params, permission):
  cp, sp = supported_params
  _initialize_subaru_startup_preferences(cp, sp, {AVH_OPTION: "1", STOP_OPTION: "1", AUTHORIZATION: permission})
  assert sp.flags == sp.safetyParam == 0


@pytest.mark.parametrize("value", [None, "", "garbage", "2", "-1", [], {}])
@pytest.mark.parametrize("option", [AVH_OPTION, STOP_OPTION])
def test_invalid_option_fails_closed(supported_params, option, value):
  cp, sp = supported_params
  _initialize_subaru_startup_preferences(cp, sp, {option: value, AUTHORIZATION: True})
  assert sp.flags == sp.safetyParam == 0


@pytest.mark.parametrize("reason", ["passive", "dashcam", "long", "unsupported", "no_gen2", "no_angle", "other_brand",
                                   "no_panda", "multiple_pandas", "no_output", "other_safety"])
def test_unsupported_configurations_do_not_receive_permissions(supported_params, reason):
  cp, sp = supported_params
  if reason == "passive":
    cp.passive = True
  elif reason == "dashcam":
    cp.dashcamOnly = True
  elif reason == "long":
    cp.openpilotLongitudinalControl = True
  elif reason == "unsupported":
    cp.carFingerprint = "SUBARU_FORESTER"
  elif reason == "no_gen2":
    cp.flags &= ~int(SubaruFlags.GLOBAL_GEN2)
  elif reason == "no_angle":
    cp.flags &= ~int(SubaruFlags.LKAS_ANGLE)
  elif reason == "other_brand":
    cp.brand = "toyota"
  elif reason in ("no_panda", "multiple_pandas"):
    cp.safetyConfigs = [] if reason == "no_panda" else [cp.safetyConfigs[0].to_dict()] * 2
  else:
    cp.safetyConfigs[0].safetyModel = "noOutput" if reason == "no_output" else "toyota"
  assert not startup_preferences_supported(cp)
  _initialize_subaru_startup_preferences(cp, sp, {AVH_OPTION: "1", STOP_OPTION: "1", AUTHORIZATION: True})
  assert sp.flags == sp.safetyParam == 0


@pytest.mark.parametrize("kwargs,expected", [
  ({}, []),
  ({"enable_avh": False, "disable_start_stop": False}, []),
  ({"enable_avh": True}, [AVH_REQUEST]),
  ({"disable_start_stop": True}, [STOP_REQUEST]),
  ({"enable_avh": True, "disable_start_stop": True}, [AVH_REQUEST, STOP_REQUEST]),
])
def test_policy_sends_only_requested_settings(kwargs, expected):
  policy = StartupPreferences(**kwargs)
  policy.set_ignition(False, 0)
  policy.set_ignition(True, 0.1)
  frames = {address: bytearray(8) for address in REQUIRED}
  frames[GEAR][3] = 4
  frames[THROTTLE][2:4] = (800).to_bytes(2, "little")
  frames[STOP_STATUS][2] = 8
  sent = []
  for tick in range(1, 251):
    now = tick / 10
    for address, data in frames.items():
      data[1] = (data[1] + 1) & 15
      data[0] = checksum(address, data)
      policy.observe(address, bytes(data), BUS, now)
    # No AVH TX receipt: exactly one first-frame proposal per requested setting.
    sent.extend(proposal.address for proposal in policy.update(now))
  assert sent == expected
