import unittest
from itertools import product

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR


class TestCrosstrekIdentification(unittest.TestCase):
  # Firmware responses from a 2026 gas Crosstrek, independent of the database.
  def captured_firmware(self):
    return [structs.CarParams.CarFw.new_message(
      ecu=ecu, address=address, fwVersion=bytes.fromhex(version), brand='subaru', logging=logging,
    ) for ecu, address, version, logging in [
      ('fwdCamera', 0x787, '2021080049132108013d', False),
      ('fwdCamera', 0x787, '20020e', True),
      ('abs', 0x7b0, 'a220261700', False),
      ('engine', 0x7a2, '0522623007', False),
      ('transmission', 0x7a3, '4212356350', False),
    ]]

  def test_captured_firmware_matches_exactly(self):
    self.assertEqual(match_fw_to_car(self.captured_firmware(), '0' * 17, log=False),
                     (True, {CAR.SUBARU_CROSSTREK_2026}))

  def test_jacob_startup_capture_matches_exactly(self):
    fw = self.captured_firmware()[:-1]  # Transmission absent from this capture.
    fw[0].fwVersion = bytes.fromhex('20210800490000000000')
    self.assertEqual(match_fw_to_car(fw, '0' * 17, log=False),
                     (True, {CAR.SUBARU_CROSSTREK_2026}))

  def test_changed_firmware_rejects_exact_match(self):
    for index in (0, 2, 3, 4):
      with self.subTest(index=index):
        fw = self.captured_firmware()
        fw[index].fwVersion = b'unknown'
        self.assertEqual(match_fw_to_car(fw, '0' * 17, allow_fuzzy=False, log=False)[1], set())

  def test_missing_essential_firmware_rejects_exact_match(self):
    for index in (0, 2, 3):
      with self.subTest(index=index):
        fw = self.captured_firmware()
        del fw[index]
        self.assertEqual(match_fw_to_car(fw, '0' * 17, allow_fuzzy=False, log=False)[1], set())

  def test_identification_remains_non_actuating(self):
    for alpha_long, release, docs in product((False, True), repeat=3):
      with self.subTest(alpha_long=alpha_long, release=release, docs=docs):
        fp = gen_empty_fingerprint()
        cp = CarInterface.get_params(CAR.SUBARU_CROSSTREK_2026, fp, self.captured_firmware(), alpha_long, release, docs)
        for with_sp in (False, True):
          if with_sp:
            CarInterface.get_params_sp(cp, CAR.SUBARU_CROSSTREK_2026, fp, self.captured_firmware(), alpha_long, release, docs)
          self.assertTrue(cp.dashcamOnly)
          self.assertFalse(cp.openpilotLongitudinalControl)
          self.assertEqual(len(cp.safetyConfigs), 1)
          self.assertEqual(cp.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)
          self.assertEqual(cp.safetyConfigs[0].safetyParam, 0)

  def test_outback_configuration_unchanged(self):
    cp = CarInterface.get_non_essential_params(CAR.SUBARU_OUTBACK_2023)
    CarInterface.get_non_essential_params_sp(cp, CAR.SUBARU_OUTBACK_2023)
    self.assertFalse(cp.dashcamOnly)
    self.assertEqual(cp.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.subaru)
    self.assertEqual(cp.safetyConfigs[0].safetyParam, 9)
