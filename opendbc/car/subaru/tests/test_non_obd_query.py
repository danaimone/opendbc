import unittest
from copy import deepcopy
from unittest.mock import Mock, patch

from opendbc.car.car_helpers import fingerprint
from opendbc.car.fw_versions import get_fw_versions
from opendbc.car.structs import CarParams
from opendbc.car.subaru.values import CAR


class TestNonObdQuery(unittest.TestCase):
  # Responses recorded on the Outback: camera on bus 0, powertrain on bus 1
  # with OBD multiplexing disabled. No vehicle identity override is used.
  OUTBACK = {
    0: {(0x787, None): bytes.fromhex('1a210800430000000000')},
    1: {(0x7a3, None): bytes.fromhex('a917772172'),
        (0x7b0, None): bytes.fromhex('a120241700'),
        (0x7a2, None): bytes.fromhex('fb2ca27107')},
  }
  CROSSTREK = {
    0: {(0x787, None): bytes.fromhex('2021080049132108013d')},
    1: {(0x7a3, None): bytes.fromhex('4212356350'),
        (0x7b0, None): bytes.fromhex('a220261700'),
        (0x7a2, None): bytes.fromhex('0522623007')},
  }

  def run_scan(self, responses, enabled=True, cached=None):
    mux = Mock()
    queried_buses = []

    def make_query(send, recv, bus, addrs, *args):
      # Verify the actual query execution, not just the filtered configuration.
      if enabled:
        self.assertTrue(all(call.args == (False,) for call in mux.call_args_list))
      queried_buses.append(bus)
      return Mock(get_data=Mock(return_value={a: v for a, v in responses.get(bus, {}).items() if a in addrs}))

    with patch('opendbc.car.fw_versions.IsoTpParallelQuery', side_effect=make_query), \
         patch('opendbc.car.car_helpers.can_fingerprint', return_value=(None, {})), \
         patch('opendbc.car.car_helpers.get_vin', return_value=(-1, -1, '0' * 17)) as vin, \
         patch('opendbc.car.car_helpers.get_present_ecus', return_value=set()) as present, \
         patch('opendbc.car.car_helpers.get_fw_versions_ordered', return_value=[]) as ordered, \
         patch.dict('os.environ', {}, clear=True):
      result = fingerprint(Mock(), Mock(), mux, cached, None, subaru_non_obd=enabled)
    return result, mux, queried_buses, vin, present, ordered

  def test_captured_cars_match_without_obd(self):
    for responses, car in ((self.OUTBACK, CAR.SUBARU_OUTBACK_2023), (self.CROSSTREK, CAR.SUBARU_CROSSTREK_2026)):
      with self.subTest(car=car):
        result, mux, buses, vin, present, ordered = self.run_scan(responses)
        self.assertEqual(result[0], car)
        self.assertEqual(result[4], CarParams.FingerprintSource.fw)
        self.assertTrue(result[5])
        self.assertEqual(set(buses), {0, 1})
        self.assertTrue(all(c.args == (False,) for c in mux.call_args_list))
        for query in (vin, present, ordered):
          query.assert_not_called()

  def test_missing_or_unknown_ecu_does_not_fall_back_to_obd(self):
    for responses in ({}, {0: self.OUTBACK[0]}, {0: self.OUTBACK[0], 1: {(0x7a2, None): b'unknown'}}):
      with self.subTest(responses=responses):
        result, mux, _, vin, present, ordered = self.run_scan(responses)
        self.assertIsNone(result[0])
        self.assertTrue(all(c.args == (False,) for c in mux.call_args_list))
        for query in (vin, present, ordered):
          query.assert_not_called()

  def test_unknown_essential_firmware_rejects_fuzzy_match(self):
    for bus, addr in ((0, 0x787), (1, 0x7a2), (1, 0x7b0)):
      responses = deepcopy(self.OUTBACK)
      responses[bus][addr, None] = b'unknown'
      result, *_ = self.run_scan(responses)
      self.assertIsNone(result[0])

  def test_other_brand_cache_is_not_reused(self):
    result, *_ = self.run_scan(self.OUTBACK)
    cached = CarParams(brand='toyota', carFw=result[3], carVin=result[2])
    result, _, buses, *_ = self.run_scan({}, cached=cached)
    self.assertTrue(buses)
    self.assertIsNone(result[0])

  def test_default_query_path_unchanged(self):
    _, mux, buses, vin, present, ordered = self.run_scan({}, enabled=False)
    self.assertIn(((True,), {}), mux.call_args_list)
    self.assertEqual(buses, [])
    for query in (vin, present, ordered):
      query.assert_called_once()

  def test_subaru_cache_avoids_query(self):
    result, *_ = self.run_scan(self.OUTBACK)
    cached = CarParams(brand='subaru', carFw=result[3], carVin=result[2])
    cached_result, mux, buses, *_ = self.run_scan({}, cached=cached)
    self.assertEqual(cached_result[0], CAR.SUBARU_OUTBACK_2023)
    self.assertEqual(buses, [])
    mux.assert_called_once_with(False)

  def test_unfiltered_subaru_query_still_uses_obd(self):
    mux = Mock()
    with patch('opendbc.car.fw_versions.IsoTpParallelQuery') as query:
      query.return_value.get_data.return_value = {}
      get_fw_versions(Mock(), Mock(), mux, query_brand='subaru')
    self.assertIn(((True,), {}), mux.call_args_list)
