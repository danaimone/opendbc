#pragma once

// Experimental, opt-in parked startup preferences. Does not replace factory
// traffic or grant general access to either multi-purpose settings message.
// Requests remain blocked in production builds without ALLOW_DEBUG.
static const unsigned int SUBARU_STARTUP_ADDRS[] = {0x6BBU, 0x390U, 0x32BU, 0x174U, 0x40U, 0x48U, 0x13AU};
static uint8_t subaru_startup_data[7][8];
static uint32_t subaru_startup_ts[7];
static bool subaru_startup_seen[7];
static bool subaru_startup_sequential[7];
static bool subaru_startup_done[2];
static bool subaru_startup_enabled = false;
static bool subaru_startup_aborted = false;
static uint32_t subaru_startup_start = 0U;
static unsigned int subaru_startup_avh_count = 0U;
static uint32_t subaru_startup_avh_sent = 0U;
static uint32_t subaru_startup_avh_template_ts = 0U;

static void subaru_startup_init(bool enabled) {
  subaru_startup_enabled = enabled;
  subaru_startup_aborted = false;
  subaru_startup_start = microsecond_timer_get();
  subaru_startup_avh_count = 0U;
  subaru_startup_avh_sent = 0U;
  subaru_startup_avh_template_ts = 0U;
  for (int i = 0; i < 7; i++) {
    subaru_startup_seen[i] = false;
    subaru_startup_sequential[i] = false;
    subaru_startup_ts[i] = 0U;
    for (int j = 0; j < 8; j++) {
      subaru_startup_data[i][j] = 0U;
    }
  }
  subaru_startup_done[0] = false;
  subaru_startup_done[1] = false;
}

static void subaru_startup_rx(const CANPacket_t *msg) {
  if (subaru_startup_enabled && (msg->bus == 1U) && (GET_LEN(msg) == 8U)) {
    if (safety_get_ts_elapsed(microsecond_timer_get(), subaru_startup_start) > 30000000U) {
      subaru_startup_aborted = true;
    }
    for (int i = 0; i < 7; i++) {
      if (msg->addr == SUBARU_STARTUP_ADDRS[i]) {
        const uint8_t old_counter = subaru_startup_data[i][1] & 0xFU;
        const uint8_t new_counter = msg->data[1] & 0xFU;
        // Duplicates never refresh liveness, even before generic counter fault.
        if (!subaru_startup_seen[i] || (new_counter != old_counter)) {
          subaru_startup_sequential[i] = subaru_startup_seen[i] && (new_counter == ((old_counter + 1U) & 0xFU));
          subaru_startup_seen[i] = true;
          subaru_startup_ts[i] = microsecond_timer_get();
          for (int j = 0; j < 8; j++) {
            subaru_startup_data[i][j] = msg->data[j];
          }
        }
      }
    }
    if ((msg->addr == 0x6BBU) && ((msg->data[2] & 3U) != 0U)) {
      subaru_startup_done[0] = true;
    }
    if ((msg->addr == 0x390U) && ((msg->data[6] & 0x40U) != 0U)) {
      subaru_startup_done[1] = true;
    }
    if (((msg->addr == 0x48U) && (msg->data[3] != 4U)) ||
        ((msg->addr == 0x40U) && (msg->data[4] != 0U)) ||
        ((msg->addr == 0x13AU) && vehicle_moving)) {
      subaru_startup_aborted = true;
    }
  }
}

static bool subaru_startup_tx(const CANPacket_t *msg) {
  const int index = (msg->addr == 0x6BBU) ? 0 : 1;
  const uint32_t now = microsecond_timer_get();
  const uint32_t elapsed = safety_get_ts_elapsed(now, subaru_startup_start);
  const bool avh_second = (index == 0) && (subaru_startup_avh_count == 1U);
  bool allowed = subaru_startup_enabled && !subaru_startup_aborted && !subaru_startup_done[index] &&
                 (msg->bus == 1U) && (GET_LEN(msg) == 8U) && (elapsed >= 10000000U) && (elapsed <= 30000000U);
  allowed &= !safety_rx_checks_invalid;
  // A rejected RX frame does not invoke our RX hook. Consult the generic RX
  // status too, so a subsequent corrupt frame cannot leave a usable template.
  for (int i = 0; i < current_safety_config.rx_checks_len; i++) {
    const RxCheck *check = &current_safety_config.rx_checks[i];
    for (int j = 0; j < 7; j++) {
      if (((unsigned int)check->msg[check->status.index].addr == SUBARU_STARTUP_ADDRS[j]) && (check->msg[check->status.index].bus == 1U)) {
        allowed &= check->status.valid_checksum && (check->status.wrong_counters < MAX_WRONG_COUNTERS);
      }
    }
  }
  for (int i = 0; i < 7; i++) {
    const uint32_t max_age = (i == 0) ? 1500000U : 300000U;
    allowed &= subaru_startup_seen[i] && subaru_startup_sequential[i] && (safety_get_ts_elapsed(now, subaru_startup_ts[i]) <= max_age);
  }
  // Require a just-received factory template to bound counter races.
  if (avh_second) {
    const uint32_t spacing = safety_get_ts_elapsed(now, subaru_startup_avh_sent);
    allowed &= (spacing >= 45000U) && (spacing <= 80000U);
    allowed &= subaru_startup_ts[0] == subaru_startup_avh_template_ts;
    allowed &= safety_get_ts_elapsed(now, subaru_startup_ts[0]) <= 110000U;
  } else {
    allowed &= safety_get_ts_elapsed(now, subaru_startup_ts[index]) <= 30000U;
  }
  allowed &= !vehicle_moving && (subaru_startup_data[5][3] == 4U) && (subaru_startup_data[4][4] == 0U);
  const unsigned int rpm = ((unsigned int)subaru_startup_data[4][2] | ((unsigned int)subaru_startup_data[4][3] << 8U)) & 0x1FFFU;
  allowed &= (rpm >= 400U) && ((subaru_startup_data[3][2] & 8U) != 0U);
  allowed &= ((subaru_startup_data[0][2] & 3U) == 0U) && ((subaru_startup_data[1][6] & 0x40U) == 0U);
  allowed &= (index == 0) ? ((subaru_startup_data[2][5] & 0x20U) == 0U) : (subaru_startup_data[3][4] == 0U);

  const int request_byte = (index == 0) ? 2 : 6;
  const uint8_t request_bit = (index == 0) ? 2U : 0x40U;
  uint8_t expected_checksum = (uint8_t)((msg->addr & 0xFFU) + (msg->addr >> 8U));
  for (int i = 1; i < 8; i++) {
    uint8_t expected = subaru_startup_data[index][i];
    if (i == 1) {
      expected = (expected & 0xF0U) | ((expected + (avh_second ? 2U : 1U)) & 0xFU);
    } else if (i == request_byte) {
      expected |= request_bit;
    } else {
      // All unrelated bits must exactly match the most recent factory frame.
    }
    allowed &= msg->data[i] == expected;
    expected_checksum += expected;
  }
  allowed &= msg->data[0] == expected_checksum;
  if (allowed) {
    if ((index == 0) && !avh_second) {
      subaru_startup_avh_count = 1U;
      subaru_startup_avh_sent = now;
      subaru_startup_avh_template_ts = subaru_startup_ts[0];
    } else {
      subaru_startup_done[index] = true;  // at most one press; never retry a toggle
    }
  }
  return allowed;
}
