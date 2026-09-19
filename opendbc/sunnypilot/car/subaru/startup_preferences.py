"""Factory AVH and start-stop requests; no filesystem or application dependencies."""

from dataclasses import dataclass

BUS = 1
AVH_REQUEST = 0x6BB
STOP_REQUEST = 0x390
AVH_STATUS = 0x32B
STOP_STATUS = 0x174
THROTTLE = 0x40
GEAR = 0x48
WHEELS = 0x13A
REQUIRED = (AVH_REQUEST, STOP_REQUEST, AVH_STATUS, STOP_STATUS, THROTTLE, GEAR, WHEELS)
STARTUP_WINDOW = 120
# Panda allows 30 ms from physical RX to TX. Host timestamps arrive later;
# reserve 20 ms for receive buffering, scheduling and transport back to panda.
MAX_HOST_TEMPLATE_AGE = 0.010


def startup_preferences_supported(CP) -> bool:
  from opendbc.car.subaru.values import SubaruFlags

  # Crosstrek support remains dependent on its separately reviewed vehicle port.
  return (CP.brand == "subaru" and str(CP.carFingerprint) in ("SUBARU_OUTBACK_2023", "SUBARU_CROSSTREK_2026")
          and bool(CP.flags & SubaruFlags.GLOBAL_GEN2) and bool(CP.flags & SubaruFlags.LKAS_ANGLE)
          and not CP.dashcamOnly and not CP.passive and not CP.openpilotLongitudinalControl
          and len(CP.safetyConfigs) == 1 and str(CP.safetyConfigs[0].safetyModel) == "subaru")


def checksum(address: int, data: bytes) -> int:
  return ((address & 0xFF) + (address >> 8) + sum(data[1:])) & 0xFF


def request_packet(address: int, original: bytes, counter_step: int = 1) -> bytes:
  """Construct one candidate pulse, preserving every unrelated payload bit."""
  if address not in (AVH_REQUEST, STOP_REQUEST) or len(original) != 8 or checksum(address, original) != original[0]:
    raise ValueError("Invalid request template")
  if counter_step != 1 and not (address == AVH_REQUEST and counter_step == 2):
    raise ValueError("Invalid request counter step")
  index, mask = (2, 0x03) if address == AVH_REQUEST else (6, 0x40)
  if original[index] & mask:
    raise ValueError("Manual request already active")
  data = bytearray(original)
  data[1] = (data[1] & 0xF0) | ((data[1] + counter_step) & 0x0F)
  data[index] |= 0x02 if address == AVH_REQUEST else 0x40
  data[0] = checksum(address, data)
  return bytes(data)


@dataclass(frozen=True)
class Proposal:
  time: float
  address: int
  data: bytes
  bus: int = BUS


class StartupPreferences:
  """One-shot factory-setting requests for an authorized Subaru startup.

  Inputs and deadlines use one monotonic clock, in seconds. Unknown/corrupt
  state never means off. A missing acknowledgement never triggers a retry of
  the start-stop toggle. A manual request retires only its respective setting.
  """

  def __init__(self, enable_avh: bool = False, disable_start_stop: bool = False):
    self.requested = {a for a, enabled in ((AVH_REQUEST, enable_avh), (STOP_REQUEST, disable_start_stop)) if enabled}
    self.ignition = None
    self.started = None
    self.stable_since = None
    self.frames = {}
    self.settled = {}
    self.pending = {}
    self.aborted = False
    self.last_time = None
    self.avh_followup = None
    self.avh_tx_confirmed_at = None

  def set_ignition(self, on: bool, now: float):
    if not on:
      self.started = None
      self.frames.clear()
      self.stable_since = None
      self.settled.clear()
      self.pending.clear()
      self.aborted = False
      self.avh_followup = None
      self.avh_tx_confirmed_at = None
    elif self.ignition is False:
      self.started = now
      self.frames.clear()
    self.ignition = on

  def observe(self, address: int, data: bytes, bus: int, now: float):
    if address == AVH_REQUEST and bus in (BUS + 128, BUS + 192):
      # Returned/rejected packets are panda metadata, not factory button input.
      # Anchor the follow-up to the first accepted transmission, since unequal
      # host-to-panda delays can compress two publication times below 45 ms.
      if self.avh_followup is not None and self.ignition is True:
        sent, _, template = self.avh_followup
        if data == request_packet(AVH_REQUEST, template) and now >= sent:
          if bus == BUS + 192 or now - sent > 0.025:
            self.avh_followup = None
          elif self.avh_tx_confirmed_at is None:
            self.avh_tx_confirmed_at = now
      return
    if bus != BUS or address not in REQUIRED or self.ignition is not True:
      return
    if len(data) != 8 or checksum(address, data) != data[0]:
      self.frames.pop(address, None)
      self.stable_since = None
      return
    previous = self.frames.get(address)
    if previous and now < previous[0]:
      self.aborted = True
      return
    # Replayed counters cannot keep state fresh. Any discontinuity requires a
    # new consecutive sample before using this message again.
    sequential = bool(previous and (data[1] & 15) == ((previous[1][1] + 1) & 15))
    if previous and (data[1] & 15) == (previous[1][1] & 15):
      return
    self.frames[address] = (now, data, sequential)
    if address == AVH_REQUEST and data[2] & 3:
      self.settled[address] = "manual override"
      self.pending.pop(address, None)
      self.avh_followup = None
    if address == STOP_REQUEST and data[6] & 0x40 and address not in self.pending:
      self.settled[address] = "manual override"

  def update(self, now: float, *, requests_allowed: bool = True) -> list[Proposal]:
    if self.last_time is not None and now < self.last_time:
      self.aborted = True
    self.last_time = now
    if self.ignition is not True or self.started is None or self.aborted:
      return []
    if now - self.started > STARTUP_WINDOW:
      self.aborted = True
      return []
    # Requests run at roughly 1 Hz / 10 Hz; all other inputs must be recent.
    for address in REQUIRED:
      record = self.frames.get(address)
      max_age = 1.5 if address == AVH_REQUEST else 0.3
      if record is None or not record[2] or not 0 <= now - record[0] <= max_age:
        self.stable_since = None
        return []
    data = {a: self.frames[a][1] for a in REQUIRED}
    wheel_bits = int.from_bytes(data[WHEELS], 'little')
    wheel_speeds = [(wheel_bits >> bit) & 0x1FFF for bit in (12, 25, 38, 51)]
    parked = data[GEAR][3] == 4 and not any(wheel_speeds) and data[THROTTLE][4] == 0
    driving_away = data[GEAR][3] == 121
    if not (parked or driving_away):
      self.stable_since = None
      self.avh_followup = None
      return []
    rpm = int.from_bytes(data[THROTTLE][2:4], 'little') & 0x1FFF
    if rpm < 400 or not (data[STOP_STATUS][2] & 0x08):
      self.stable_since = None
      return []
    if self.stable_since is None:
      self.stable_since = now
    if not requests_allowed or now - self.started < 10 or now - self.stable_since < 3:
      return []

    desired = {AVH_REQUEST: bool(data[AVH_STATUS][5] & 0x20), STOP_REQUEST: data[STOP_STATUS][4] == 0xC0}
    for address in tuple(self.pending):
      if desired[address]:
        self.settled[address] = "acknowledged"
        del self.pending[address]
      elif now - self.pending[address] >= 2:
        self.settled[address] = "unacknowledged; no retry"
        del self.pending[address]
    # One AVH button press consists of at most two frames, 50 ms apart. A new
    # factory template, manual action, acknowledgement, or late scheduler drops
    # the second frame; it is never retried later in the ignition cycle. Wait
    # at least 50 ms after the accepted TX receipt, while retaining the original
    # 75 ms publication deadline to bound late delivery and template age.
    if self.avh_followup is not None:
      sent, template_time, template = self.avh_followup
      age = now - sent
      if desired[AVH_REQUEST] or AVH_REQUEST not in self.pending or self.frames[AVH_REQUEST][0] != template_time or age > 0.075:
        self.avh_followup = None
      elif self.avh_tx_confirmed_at is not None and now - self.avh_tx_confirmed_at >= 0.05:
        self.avh_followup = None
        return [Proposal(now, AVH_REQUEST, request_packet(AVH_REQUEST, template, counter_step=2))]
    for address in (AVH_REQUEST, STOP_REQUEST):
      if address not in self.requested:
        continue
      if address in self.settled or address in self.pending:
        continue
      if desired[address]:
        self.settled[address] = "already correct"
        continue
      # Only the demonstrated start-stop enabled value may trigger a toggle.
      if address == STOP_REQUEST and data[STOP_STATUS][4] != 0:
        continue
      if self.pending or now - self.frames[address][0] > MAX_HOST_TEMPLATE_AGE:
        continue
      if (data[AVH_REQUEST][2] & 3) or (data[STOP_REQUEST][6] & 0x40):
        continue
      self.pending[address] = now
      if address == AVH_REQUEST:
        self.avh_followup = (now, self.frames[address][0], data[address])
        self.avh_tx_confirmed_at = None
      return [Proposal(now, address, request_packet(address, data[address]))]
    return []
