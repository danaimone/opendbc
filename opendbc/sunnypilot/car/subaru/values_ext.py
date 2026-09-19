"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class SubaruSafetyFlagsSP:
  STOP_AND_GO = 1
  ENABLE_AVH_AT_STARTUP = 2
  DISABLE_START_STOP_AT_STARTUP = 4


class SubaruFlagsSP(IntFlag):
  STOP_AND_GO = 1
  STOP_AND_GO_MANUAL_PARKING_BRAKE = 2
  ENABLE_AVH_AT_STARTUP = 4
  DISABLE_START_STOP_AT_STARTUP = 8
