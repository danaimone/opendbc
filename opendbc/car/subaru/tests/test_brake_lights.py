import unittest
from opendbc.can import CANPacker, CANParser
from opendbc.car.subaru.brake_lights import brake_light_status


class TestBrakeLights(unittest.TestCase):
  def test_real_can_decode_driver_cruise_and_staleness(self):
    packer = CANPacker('subaru_global_2017_generated')
    camera = CANParser('subaru_global_2017_generated', [('ES_DashStatus', 10)], 2)
    brakes = CANParser('subaru_global_2017_generated', [('ES_Brake', 50)], 1)
    self.assertEqual(brake_light_status(camera, brakes, 1), (False, False))
    for n, (driver, cruise) in enumerate(((0, 0), (1, 0), (0, 1), (1, 1), (0, 0))):
      now = (n + 1) * 100_000_000
      frames = [packer.make_can_msg('ES_DashStatus', 2, {'Brake_Lights': driver, 'COUNTER': n}),
                packer.make_can_msg('ES_Brake', 1, {'Cruise_Brake_Lights': cruise, 'Cruise_Brake_Active': 1, 'COUNTER': n})]
      camera.update([(now, frames)])
      brakes.update([(now, frames)])
      self.assertEqual(brake_light_status(camera, brakes, now), (True, bool(driver or cruise)))
    self.assertEqual(brake_light_status(camera, brakes, now + 500_000_001), (False, False))
    self.assertEqual(brake_light_status(camera, brakes, now - 1), (False, False))

  def test_outback_carstate_routes_the_actual_buses(self):
    from opendbc.car import Bus
    from opendbc.car.subaru.carstate import CarState
    from opendbc.car.subaru.interface import CarInterface
    from opendbc.car.subaru.values import CAR
    cp = CarInterface.get_non_essential_params(CAR.SUBARU_OUTBACK_2023)
    cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.SUBARU_OUTBACK_2023)
    state = CarState(cp, cp_sp)
    parsers = state.get_can_parsers(cp, cp_sp)
    state.update(parsers)  # Subscribe to the messages used by CarState.
    packer = CANPacker('subaru_global_2017_generated')
    for n, (driver, cruise) in enumerate(((0, 0), (1, 0), (0, 1), (0, 0))):
      frames = [packer.make_can_msg('ES_DashStatus', 2, {'Brake_Lights': driver, 'COUNTER': n}),
                packer.make_can_msg('ES_Brake', 1, {'Cruise_Brake_Lights': cruise, 'COUNTER': n})]
      for parser in parsers.values():
        parser.update([((n + 1) * 100_000_000, frames)])
      _, result = state.update(parsers)
      self.assertTrue(result.brakeLightsAvailable)
      self.assertEqual(result.brakeLightsOn, bool(driver or cruise))
    parsers[Bus.pt].update([(2_000_000_000, [packer.make_can_msg('Throttle', 0, {})])])
    _, result = state.update(parsers)
    self.assertFalse(result.brakeLightsAvailable)
    self.assertFalse(result.brakeLightsOn)
