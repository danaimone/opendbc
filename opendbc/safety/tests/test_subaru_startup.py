import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


class TestSubaruStartupSafety(unittest.TestCase):
  TX_MSGS = [[0x6BB, 1], [0x390, 1]]
  ADDRESSES = (0x6BB, 0x390, 0x32B, 0x174, 0x40, 0x48, 0x13A)

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_timer(0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, 1 | 8 | 16)
    self.safety.init_tests()
    self.data = {address: bytearray(8) for address in self.ADDRESSES}
    self.data[0x40][2:4] = (800).to_bytes(2, 'little')
    self.data[0x48][3] = 4
    self.data[0x174][2] = 8

  @staticmethod
  def checksum(address, data):
    data[0] = ((address & 255) + (address >> 8) + sum(data[1:])) & 255
    return data

  def receive(self, now=10_000_000, omit=()):
    self.safety.set_timer(now)
    for address, data in self.data.items():
      if address not in omit:
        data[1] = (data[1] + 1) & 15
        self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, 1, self.checksum(address, data)))

  def ready(self):
    self.receive(9_900_000)
    self.receive(10_000_000)

  def packet(self, address, bus=1, counter_step=1):
    data = bytearray(self.data[address])
    data[1] = (data[1] & 0xF0) | ((data[1] + counter_step) & 15)
    if address == 0x6BB:
      data[2] |= 2
    else:
      data[6] |= 0x40
    return libsafety_py.make_CANPacket(address, bus, self.checksum(address, data))

  def test_one_request_each_and_no_retry(self):
    self.ready()
    for address in (0x6BB, 0x390):
      self.assertTrue(self.safety.safety_tx_hook(self.packet(address)))
      self.assertFalse(self.safety.safety_tx_hook(self.packet(address)))

  def receive_base_checks(self):
    for address, bus in ((0x40, 0), (0x119, 0), (0x11A, 0), (0x13C, 1), (0x220, 1), (0x321, 2), (0x322, 2)):
      data = bytearray(8)
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, self.checksum(address, data)))

  def test_requests_after_periodic_safety_tick(self):
    self.ready()
    self.receive_base_checks()
    self.safety.safety_tick_current_safety_config()
    for address in (0x6BB, 0x390):
      self.assertTrue(self.safety.safety_tx_hook(self.packet(address)))

  def test_slow_avh_still_requires_fresh_status(self):
    self.ready()
    self.receive(11_500_001, omit=(0x6BB,))
    self.receive_base_checks()
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.safety_tx_hook(self.packet(0x390)))

  def test_avh_two_frame_press_has_fixed_counter_spacing_and_limit(self):
    self.ready()
    self.assertTrue(self.safety.safety_tx_hook(self.packet(0x6BB)))
    self.safety.set_timer(10_050_000)
    self.assertFalse(self.safety.safety_tx_hook(self.packet(0x6BB)))
    self.assertTrue(self.safety.safety_tx_hook(self.packet(0x6BB, counter_step=2)))
    self.safety.set_timer(10_100_000)
    self.assertFalse(self.safety.safety_tx_hook(self.packet(0x6BB, counter_step=3)))

  def test_avh_followup_too_early_or_late_is_blocked(self):
    for spacing in (0, 44_999, 80_001, 1_000_000):
      self.setUp()
      self.ready()
      self.assertTrue(self.safety.safety_tx_hook(self.packet(0x6BB)))
      self.safety.set_timer(10_000_000 + spacing)
      self.assertFalse(self.safety.safety_tx_hook(self.packet(0x6BB, counter_step=2)))

  def test_avh_followup_cannot_change_unrelated_payload_bits(self):
    for byte in range(8):
      for bit in range(8):
        self.setUp()
        self.ready()
        self.assertTrue(self.safety.safety_tx_hook(self.packet(0x6BB)))
        self.safety.set_timer(10_050_000)
        msg = self.packet(0x6BB, counter_step=2)
        msg.data[byte] ^= 1 << bit
        if byte != 0:
          msg.data[0] = ((0x6BB & 255) + (0x6BB >> 8) + sum(msg.data[i] for i in range(1, 8))) & 255
        self.assertFalse(self.safety.safety_tx_hook(msg))

  def test_avh_followup_after_new_factory_template_is_blocked(self):
    self.ready()
    self.assertTrue(self.safety.safety_tx_hook(self.packet(0x6BB)))
    self.receive(10_050_000)
    self.assertFalse(self.safety.safety_tx_hook(self.packet(0x6BB, counter_step=2)))

  def test_avh_followup_after_ack_or_manual_override_is_blocked(self):
    for address, byte, value in ((0x32B, 5, 32), (0x6BB, 2, 1)):
      self.setUp()
      self.ready()
      self.assertTrue(self.safety.safety_tx_hook(self.packet(0x6BB)))
      self.data[address][byte] = value
      self.data[address][1] = (self.data[address][1] + 1) & 15
      self.safety.set_timer(10_050_000)
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, 1, self.checksum(address, self.data[address])))
      self.assertFalse(self.safety.safety_tx_hook(self.packet(0x6BB, counter_step=2)))

  def test_default_and_other_platforms_block(self):
    for flags in (0, 1, 8, 9, 16, 17, 24, 27):
      self.setUp()
      self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, flags)
      self.ready()
      for address in (0x6BB, 0x390):
        self.assertFalse(self.safety.safety_tx_hook(self.packet(address)))

  def test_all_payload_bit_changes_rejected(self):
    for address in (0x6BB, 0x390):
      for byte in range(8):
        for bit in range(8):
          self.setUp()
          self.ready()
          msg = self.packet(address)
          msg.data[byte] ^= 1 << bit
          # Recalculate a valid checksum: unrelated changes still must fail.
          if byte != 0:
            msg.data[0] = ((address & 255) + (address >> 8) + sum(msg.data[i] for i in range(1, 8))) & 255
          self.assertFalse(self.safety.safety_tx_hook(msg), (hex(address), byte, bit))

  def test_wrong_bus_and_length(self):
    self.ready()
    for address in (0x6BB, 0x390):
      for bus in (0, 2, 3):
        self.assertFalse(self.safety.safety_tx_hook(self.packet(address, bus)))
      msg = self.packet(address)
      msg.data_len_code = 7
      self.assertFalse(self.safety.safety_tx_hook(msg))

  def test_time_window(self):
    for time in (9_999_999, 30_000_001):
      self.setUp()
      self.receive(time - 100_000)
      self.receive(time)
      for address in (0x6BB, 0x390):
        self.assertFalse(self.safety.safety_tx_hook(self.packet(address)))

  def test_missing_inputs(self):
    for missing in self.ADDRESSES:
      self.setUp()
      self.receive(9_900_000, (missing,))
      self.receive(10_000_000, (missing,))
      for address in (0x6BB, 0x390):
        self.assertFalse(self.safety.safety_tx_hook(self.packet(address)), hex(missing))

  def test_stale_template(self):
    self.ready()
    self.safety.set_timer(10_030_001)
    for address in (0x6BB, 0x390):
      self.assertFalse(self.safety.safety_tx_hook(self.packet(address)))

  def test_bad_rx_checksum_blocks_cached_template(self):
    for bad_address in self.ADDRESSES:
      self.setUp()
      self.ready()
      bad = bytearray(self.data[bad_address])
      bad[0] ^= 1
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(bad_address, 1, bad))
      for request in (0x6BB, 0x390):
        self.assertFalse(self.safety.safety_tx_hook(self.packet(request)))

  def test_counter_wrap_and_duplicate_staleness(self):
    self.data[0x6BB][1] = 13
    self.data[0x390][1] = 13
    self.ready()  # last received counter 15; request wraps to zero
    for request in (0x6BB, 0x390):
      self.assertTrue(self.safety.safety_tx_hook(self.packet(request)))
    self.setUp()
    self.ready()
    self.safety.set_timer(10_100_000)
    for address, data in self.data.items():
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, 1, data))
    for request in (0x6BB, 0x390):
      self.assertFalse(self.safety.safety_tx_hook(self.packet(request)))

  def test_timeout_remains_latched_after_timer_wrap(self):
    self.ready()
    self.receive(31_000_000)
    self.receive(10_000_000)
    for request in (0x6BB, 0x390):
      self.assertFalse(self.safety.safety_tx_hook(self.packet(request)))

  def test_motion_or_driver_input_aborts_cycle(self):
    for address, byte, value in ((0x48, 3, 121), (0x40, 4, 1), (0x13A, 2, 1)):
      self.setUp()
      self.ready()
      old = self.data[address][byte]
      self.data[address][byte] = value
      self.receive(10_100_000)
      self.data[address][byte] = old
      self.receive(10_200_000)
      for request in (0x6BB, 0x390):
        self.assertFalse(self.safety.safety_tx_hook(self.packet(request)))

  def test_manual_request_retires_setting(self):
    for address, byte, value in ((0x6BB, 2, 1), (0x390, 6, 0x40)):
      self.setUp()
      self.ready()
      self.data[address][byte] = value
      self.receive(10_100_000)
      self.data[address][byte] = 0
      self.receive(10_200_000)
      self.assertFalse(self.safety.safety_tx_hook(self.packet(address)))

  def test_already_correct_or_unknown_feedback(self):
    for status, byte, value, request in ((0x32B, 5, 0x20, 0x6BB), (0x174, 4, 0xC0, 0x390), (0x174, 4, 0x80, 0x390)):
      self.setUp()
      self.data[status][byte] = value
      self.ready()
      self.assertFalse(self.safety.safety_tx_hook(self.packet(request)))

  def test_engine_not_running_or_not_ready(self):
    for address, byte in ((0x40, 2), (0x174, 2)):
      self.setUp()
      self.data[address][byte] = 0
      if address == 0x40:
        self.data[address][3] = 0
      self.ready()
      for request in (0x6BB, 0x390):
        self.assertFalse(self.safety.safety_tx_hook(self.packet(request)))


if __name__ == '__main__':
  unittest.main()
