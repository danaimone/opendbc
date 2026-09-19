import random
import pytest
from opendbc.sunnypilot.car.subaru.startup_preferences import (
  AVH_REQUEST,
  AVH_STATUS,
  BUS,
  GEAR,
  REQUIRED,
  STOP_REQUEST,
  STOP_STATUS,
  THROTTLE,
  WHEELS,
  StartupPreferences,
  checksum,
  request_packet,
)


class TestStartupPreferences:
  def setup_method(self):
    self.policy = StartupPreferences(enable_avh=True, disable_start_stop=True)
    self.policy.set_ignition(False, 0)
    self.policy.set_ignition(True, 0.1)
    self.frames = {a: bytearray(8) for a in REQUIRED}
    self.frames[GEAR][3] = 4
    self.frames[THROTTLE][2:4] = (800).to_bytes(2, 'little')
    self.frames[STOP_STATUS][2] = 8
    self.now = 0.1
    self.proposals = []

  def step(self, seconds=0.1, omit=()):
    self.now = round(self.now + seconds, 6)
    for address, data in self.frames.items():
      if address in omit:
        continue
      data[1] = (data[1] & 0xf0) | ((data[1] + 1) & 15)
      data[0] = checksum(address, data)
      self.policy.observe(address, bytes(data), BUS, self.now)
    out = self.policy.update(self.now)
    self.proposals.extend(out)
    return out

  def advance(self, until, **kwargs):
    while self.now < until:
      self.step(**kwargs)

  def test_startup_delay_then_avh_then_stop(self):
    self.advance(10)
    assert self.proposals == []
    self.step()
    assert [p.address for p in self.proposals] == [AVH_REQUEST]
    self.frames[AVH_STATUS][5] = 32
    self.step()
    assert [p.address for p in self.proposals] == [AVH_REQUEST, STOP_REQUEST]
    self.frames[STOP_STATUS][4] = 192
    self.step()
    assert set(self.policy.settled.values()) == {'acknowledged'}

  def test_old_host_template_waits_without_consuming_attempt(self):
    self.frames[AVH_STATUS][5] = 32
    self.advance(10)
    self.now = 10.1
    for address, data in self.frames.items():
      data[1] = (data[1] + 1) & 15
      data[0] = checksum(address, data)
      # Recorded failed request used a 22.94 ms old host template. It must
      # wait for another factory frame, leaving panda's 30 ms gate unchanged.
      stamp = self.now - .02294 if address == STOP_REQUEST else self.now
      self.policy.observe(address, bytes(data), BUS, stamp)
    assert self.policy.update(self.now) == []
    assert STOP_REQUEST not in self.policy.pending
    assert STOP_REQUEST not in self.policy.settled
    assert [p.address for p in self.step(.08)] == [STOP_REQUEST]

  def test_already_correct_and_later_manual_changes(self):
    self.frames[AVH_STATUS][5] = 32
    self.frames[STOP_STATUS][4] = 192
    self.advance(11)
    self.frames[AVH_STATUS][5] = 0
    self.frames[STOP_STATUS][4] = 0
    self.advance(25)
    assert self.proposals == []

  def test_no_retry_without_acknowledgement(self):
    self.advance(25)
    assert [p.address for p in self.proposals] == [AVH_REQUEST, STOP_REQUEST]
    assert set(self.policy.settled.values()) == {'unacknowledged; no retry'}

  def test_avh_press_second_frame_and_no_third(self):
    self.advance(10.1)
    original = self.policy.frames[AVH_REQUEST][1]
    self.policy.observe(AVH_REQUEST, self.proposals[-1].data, BUS + 128, self.now + .005)
    result = self.step(.06, omit=(AVH_REQUEST,))
    assert len(result) == 1
    assert result[0].data == request_packet(AVH_REQUEST, original, counter_step=2)
    self.step(.06, omit=(AVH_REQUEST,))
    assert [p.address for p in self.proposals] == [AVH_REQUEST, AVH_REQUEST]

  @pytest.mark.parametrize('reason', ['late', 'template', 'manual', 'ack'])
  def test_avh_followup_cancelled(self, reason):
    self.advance(10.1)
    self.policy.observe(AVH_REQUEST, self.proposals[-1].data, BUS + 128, self.now + .005)
    if reason == 'manual':
      self.frames[AVH_REQUEST][2] = 1
    if reason == 'ack':
      self.frames[AVH_STATUS][5] = 32
    self.step(.08 if reason == 'late' else .06, omit=(AVH_REQUEST,) if reason in ('late', 'ack') else ())
    assert len([p for p in self.proposals if p.address == AVH_REQUEST]) == 1

  @pytest.mark.parametrize('receipt_delay', [.018841, .019])
  def test_avh_delayed_first_transmission_does_not_compress_second(self, receipt_delay):
    self.advance(10.1)
    first = self.proposals[-1]
    self.policy.observe(AVH_REQUEST, first.data, BUS + 128, self.now + receipt_delay)
    # Previously this produced the second frame, which was rejected on two drives.
    assert self.step(.052, omit=(AVH_REQUEST,)) == []
    second = self.step(.018, omit=(AVH_REQUEST,))
    assert len(second) == 1
    assert second[0].time - (first.time + receipt_delay) >= .05
    assert second[0].time - first.time <= .075
    assert AVH_REQUEST not in self.policy.settled  # receipt is not a manual override

  @pytest.mark.parametrize('kind', ['missing', 'rejected', 'wrong_payload', 'wrong_bus', 'late', 'old'])
  def test_avh_requires_timely_matching_accepted_receipt(self, kind):
    self.advance(10.1)
    first = self.proposals[-1]
    if kind != 'missing':
      bus = BUS + (192 if kind == 'rejected' else 128)
      if kind == 'wrong_bus':
        bus += 1
      data = first.data if kind != 'wrong_payload' else request_packet(AVH_REQUEST, self.policy.frames[AVH_REQUEST][1], 2)
      delay = .026 if kind == 'late' else (-.01 if kind == 'old' else .005)
      self.policy.observe(AVH_REQUEST, data, bus, self.now + delay)
    self.step(.06, omit=(AVH_REQUEST,))
    self.step(.02, omit=(AVH_REQUEST,))
    assert [p.address for p in self.proposals] == [AVH_REQUEST]
    self.advance(15)
    assert len([p for p in self.proposals if p.address == AVH_REQUEST]) == 1

  def test_duplicate_receipt_cannot_move_followup_clock(self):
    self.advance(10.1)
    first = self.proposals[-1]
    self.policy.observe(AVH_REQUEST, first.data, BUS + 128, self.now + .005)
    self.policy.observe(AVH_REQUEST, first.data, BUS + 128, self.now + .02)
    assert len(self.step(.06, omit=(AVH_REQUEST,))) == 1

  def test_manual_override_before_startup(self):
    self.frames[AVH_REQUEST][2] = 1
    self.frames[STOP_REQUEST][6] = 64
    self.step()
    self.frames[AVH_REQUEST][2] = 0
    self.frames[STOP_REQUEST][6] = 0
    self.advance(25)
    assert self.proposals == []
    assert set(self.policy.settled.values()) == {'manual override'}

  def test_attach_mid_ignition_does_not_arm(self):
    self.policy = StartupPreferences(enable_avh=True, disable_start_stop=True)
    self.policy.set_ignition(True, 0.1)
    self.advance(25)
    assert self.proposals == []

  def test_missing_or_stale_input_blocks_requests(self):
    for missing in REQUIRED:
      self.setup_method()
      self.advance(8)
      self.advance(25, omit=(missing,))
      assert self.proposals == []

  def test_reverse_waits_then_drive_can_request(self):
    self.frames[GEAR][3] = 3
    self.advance(20)
    assert not self.proposals and not self.policy.aborted
    self.frames[GEAR][3] = 121
    self.frames[THROTTLE][4] = 20
    self.advance(24)
    assert self.proposals and not self.policy.aborted

  @pytest.mark.parametrize('gear', [0, 2, 3, 137, 145, 153, 161, 169, 177])
  def test_other_gears_wait_without_request(self, gear):
    self.frames[GEAR][3] = gear
    self.advance(40)
    assert not self.proposals and not self.policy.aborted

  @pytest.mark.parametrize('bit', [12, 25, 38, 51])
  @pytest.mark.parametrize('speed', [1, 350, 351, 1000, 8191])
  def test_drive_motion_does_not_cancel_startup(self, bit, speed):
    self.frames[GEAR][3] = 121
    self.frames[THROTTLE][4] = 20
    self.frames[WHEELS] = bytearray((speed << bit).to_bytes(8, 'little'))
    self.advance(15)
    assert self.proposals and not self.policy.aborted

  def test_engine_not_ready(self):
    self.frames[THROTTLE][2:4] = (0).to_bytes(2, 'little')
    self.advance(25)
    assert self.proposals == []

  def test_unknown_stop_state_does_not_toggle(self):
    self.frames[AVH_STATUS][5] = 32
    self.frames[STOP_STATUS][4] = 128
    self.advance(25)
    assert self.proposals == []

  def test_timeout_never_rearms_at_later_stop(self):
    self.frames[STOP_STATUS][2] = 0
    self.advance(121)
    self.frames[STOP_STATUS][2] = 8
    self.advance(130)
    assert self.proposals == []
    assert self.policy.aborted

  def test_bad_checksum_invalidates_cached_state(self):
    self.advance(9.9)
    self.policy.observe(GEAR, bytes(8), BUS, self.now)
    assert GEAR not in self.policy.frames
    self.advance(12, omit=(GEAR,))
    assert self.proposals == []

  def test_wrong_bus_and_duplicate_counter_do_not_refresh(self):
    self.advance(9)
    record = self.policy.frames[GEAR]
    self.policy.observe(GEAR, record[1], 0, self.now + 1)
    self.policy.observe(GEAR, record[1], BUS, self.now + 1)
    assert self.policy.frames[GEAR] == record
    self.advance(25, omit=(GEAR,))
    assert self.proposals == []

  def test_backwards_time_fails_closed(self):
    self.advance(9)
    self.policy.update(8)
    self.advance(25)
    assert self.proposals == []

  def test_new_ignition_cycle_clears_completed_state(self):
    self.advance(25)
    self.policy.set_ignition(False, self.now)
    self.policy.set_ignition(True, self.now + 0.1)
    self.advance(50)
    assert [p.address for p in self.proposals] == [AVH_REQUEST, STOP_REQUEST] * 2


class TestRequestPacket:
  def test_known_capture_vectors(self):
    for address, original, expected in ((AVH_REQUEST, 'd40c000100000600', 'd70d020100000600'), (STOP_REQUEST, 'fc0714c77cfa1100', '3d0814c77cfa5100')):
      assert request_packet(address, bytes.fromhex(original)).hex() == expected

  def test_unrelated_bits_preserved_and_counter_wraps(self):
    rng = random.Random(2024)
    for address, index, mask, value in ((AVH_REQUEST, 2, 3, 2), (STOP_REQUEST, 6, 64, 64)):
      for counter in range(16):
        for _ in range(100):
          original = bytearray(rng.randbytes(8))
          original[1] = original[1] & 240 | counter
          original[index] &= ~mask
          original[0] = checksum(address, original)
          result = request_packet(address, bytes(original))
          assert result[0] == checksum(address, result)
          assert result[1] == original[1] & 240 | counter + 1 & 15
          assert result[index] == original[index] | value
          for byte in set(range(8)) - {0, 1, index}:
            assert result[byte] == original[byte]

  def test_refuse_unknown_corrupt_or_active_request(self):
    with pytest.raises(ValueError):
      request_packet(291, bytes(8))
    for address, index, mask in ((AVH_REQUEST, 2, 3), (STOP_REQUEST, 6, 64)):
      for length in (0, 7, 9):
        with pytest.raises(ValueError):
          request_packet(address, bytes(length))
      with pytest.raises(ValueError):
        request_packet(address, bytes(8))
      data = bytearray(8)
      data[index] = mask
      data[0] = checksum(address, data)
      with pytest.raises(ValueError):
        request_packet(address, bytes(data))


@pytest.mark.parametrize('address,frame,index,mask', [
  (0x390, '90061187845a8100', 6, 0x40),
  (0x6BB, '6d09000100802200', 2, 0x02),
])
def test_crosstrek_capture_templates_preserve_unrelated_bits(address, frame, index, mask):
  original = bytes.fromhex(frame)
  packet = request_packet(address, original)
  assert packet[0] == checksum(address, packet)
  assert packet[1] == (original[1] & 0xf0) | ((original[1] + 1) & 0xf)
  assert packet[index] == original[index] | mask
  for i in set(range(8)) - {0, 1, index}:
    assert packet[i] == original[i]
