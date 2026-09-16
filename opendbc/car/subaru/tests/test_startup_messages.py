import random
import unittest

from opendbc.can import CANPacker, CANParser


class TestSubaruStartupMessages(unittest.TestCase):
  """Packet definitions only: no controller or safety TX permission is enabled."""

  DBC = "subaru_global_2017_generated"
  MESSAGES = {0x174: "Engine_Stop_Start", 0x32B: "AVH_Status", 0x390: "Dashlights", 0x6BB: "AVH_Request"}

  @staticmethod
  def with_checksum(address, payload):
    data = bytearray(payload)
    data[0] = ((address & 0xff) + (address >> 8) + sum(data[1:])) & 0xff
    return bytes(data)

  def decode(self, address, data):
    name = self.MESSAGES[address]
    parser = CANParser(self.DBC, [(name, 0)], 1)
    parser.update([0, [(address, data, 1)]])
    return dict(parser.vl[name])

  def test_recorded_button_requests(self):
    # Successful Outback capture (SUBARU_OUTBACK_2023 platform). Counters/checksums are examples, not
    # reusable payloads; each request is derived from its factory template.
    cases = (
      (0x390, "ea0414c76dfa1100", "STOP_START", 1, ("2b0514c76dfa5100",)),
      (0x6BB, "d40c000100000600", "AVH_Request", 1, ("d70d020100000600", "d80e020100000600")),
    )
    for address, factory, signal, request, expected_frames in cases:
      values = self.decode(address, bytes.fromhex(factory))
      self.assertEqual(values[signal], 0)
      counter = values["COUNTER"]
      for step, expected in enumerate(expected_frames, start=1):
        with self.subTest(address=address, step=step):
          requested = dict(values, **{signal: request, "COUNTER": (counter + step) % 16})
          msg = CANPacker(self.DBC).make_can_msg(self.MESSAGES[address], 1, requested)
          self.assertEqual(msg, (address, bytes.fromhex(expected), 1))
          self.assertEqual(self.decode(address, msg[1])[signal], request)

  def test_recorded_status_messages(self):
    for address, payload, signal, expected in (
      (0x32B, "bb071cc228800000", "AVH_Enabled", 0),
      (0x32B, "35011c4208a00000", "AVH_Enabled", 1),
      (0x174, "840f000000000000", "STOP_START_STATE", 0),
      (0x174, "4a0d0800c0000000", "STOP_START_STATE", 3),
    ):
      with self.subTest(address=address, payload=payload):
        self.assertEqual(self.decode(address, bytes.fromhex(payload))[signal], expected)

  def test_avh_request_preserves_adjacent_unknown_bit(self):
    for bit16 in (0, 1):
      factory = self.with_checksum(0x6BB, bytes([0, 15, bit16, 1, 0, 0, 6, 0]))
      values = self.decode(0x6BB, factory)
      values.update(AVH_Request=1, COUNTER=0)
      _, payload, _ = CANPacker(self.DBC).make_can_msg("AVH_Request", 1, values)
      self.assertEqual(payload[2], bit16 | 2)
      self.assertEqual(payload[1] & 15, 0)

  def test_status_bit_positions(self):
    # Synthetic frames exercise the observed status positions; these are not
    # additional vehicle captures or evidence of other status encodings.
    for enabled in (0, 1):
      data = bytearray(8)
      data[5] = enabled << 5
      values = self.decode(0x32B, self.with_checksum(0x32B, data))
      self.assertEqual(values["AVH_Enabled"], enabled)
    for status, expected in ((0x00, 0), (0xC0, 3), (0xC1, 3)):
      data = bytearray(8)
      data[4] = status
      values = self.decode(0x174, self.with_checksum(0x174, data))
      self.assertEqual(values["STOP_START_STATE"], expected)
      # Preserve the lower six bits: state 3 does not establish byte4 == C0.
      self.assertEqual(values["Signal2"], status & 0x3f)

  def test_all_payload_bits_round_trip(self):
    rng = random.Random(0)
    for address, name in self.MESSAGES.items():
      for _ in range(100):
        data = self.with_checksum(address, rng.randbytes(8))
        with self.subTest(address=address, data=data.hex()):
          values = self.decode(address, data)
          self.assertEqual(CANPacker(self.DBC).make_can_msg(name, 1, values), (address, data, 1))


if __name__ == "__main__":
  unittest.main()
