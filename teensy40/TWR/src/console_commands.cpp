#include "console_commands.h"

#include <ctype.h>
#include <string.h>

#include "main.h"
#include "test_can_transmit_mode.h"

namespace {

static void trim_ascii(char* s) {
  if (s == nullptr) return;
  size_t len = strlen(s);
  size_t start = 0;
  while (start < len && isspace((unsigned char)s[start])) start++;
  size_t end = len;
  while (end > start && isspace((unsigned char)s[end - 1])) end--;
  if (start > 0 && end > start) {
    memmove(s, s + start, end - start);
  }
  s[end - start] = '\0';
}

static const char* mode_to_str(SupervisorMode mode) {
  switch (mode) {
    case SUP_MODE_IDLE: return "SUP_MODE_IDLE";
    case SUP_MODE_CALIBRATE: return "SUP_MODE_CALIBRATE";
    case SUP_VERIFY_ANGLE: return "SUP_VERIFY_ANGLE";
    case SUP_MODE_BALANCE_HOLD: return "SUP_MODE_BALANCE_HOLD";
    case SUP_MODE_BALANCE_TWR: return "SUP_MODE_BALANCE_TWR";
    case SUP_MODE_BALANCE_DEBUG: return "SUP_MODE_BALANCE_DEBUG";
    case SUP_MODE_TEST_CAN: return "SUP_MODE_TEST_CAN";
    default: return "SUP_MODE_UNKNOWN";
  }
}

static void print_rc_channels_snapshot(const Supervisor_typedef& sup) {
  Serial.printf(
      "{\"cmd\":\"RC_CH\",\"count\":%u,\"throttle_ch\":%u,\"steer_ch\":%u,\"throttle_invert\":%d,\"steer_invert\":%d,\"deadband\":%.3f,\"max_torque_nm\":%.3f",
      (unsigned int)sup.rc_count,
      (unsigned int)(sup.user_rc_throttle_ch + 1u),
      (unsigned int)(sup.user_rc_steer_ch + 1u),
      sup.user_rc_throttle_invert ? 1 : 0,
      sup.user_rc_steer_invert ? 1 : 0,
      sup.user_rc_deadband,
      sup.user_rc_max_torque_nm);
  for (uint8_t i = 0u; i < sup.rc_count; ++i) {
    Serial.printf(",\"ch%u_valid\":%d,\"ch%u_raw_us\":%u,\"ch%u_norm\":%.3f",
                  (unsigned int)(i + 1u), sup.rc[i].valid ? 1 : 0,
                  (unsigned int)(i + 1u), (unsigned int)sup.rc[i].raw_us,
                  (unsigned int)(i + 1u), sup.rc[i].norm);
  }
  Serial.printf("}\r\n");
}

static void process_serial_line(ConsoleCommandContext& ctx, const char* line) {
  uint32_t run_s = 0u;
  Supervisor_typedef& sup = *ctx.sup;

  if (line == nullptr) return;

  if (strcmp(line, "run") == 0 || sscanf(line, "run %lu", &run_s) == 1) {
    const SupervisorMode prev_mode = sup.mode;
    const bool restart = (prev_mode == SUP_MODE_TEST_CAN);
    uint32_t run_us = ctx.can_test_run_default_us;
    if (run_s > 0u) {
      const uint64_t requested_us = (uint64_t)run_s * 1000000ull;
      run_us = (requested_us > UINT32_MAX) ? UINT32_MAX : (uint32_t)requested_us;
    }
    tone_start(ctx.tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
    canResetPosvelStats();
    canResetRuntimeStats();
    test_can_transmit_mode_request_restart();
    ctx.can_rx_buf1->overflow_count = 0;
    ctx.can_rx_buf2->overflow_count = 0;
    for (uint16_t i = 0; i < sup.esc_count; ++i) {
      sup.esc_alive_false_count[i] = 0u;
    }
    // Preserve current TX enable state so "tx off" is honored across runs.
    sup.user_total_us = run_us;
    sup.user_rc_drive_enable = false;
    sup.mode = SUP_MODE_TEST_CAN;
    *ctx.balance_mode_enter_us = 0u;
    Serial.printf(
        "{\"cmd\":\"USER_RUN_REQUEST\",\"ok\":1,\"mode_from\":\"%s\",\"mode_to\":\"%s\",\"restart\":%d,\"run_us\":%lu,\"run_s\":%lu,\"tx_enable\":%d,\"tx_period_us\":%lu}\r\n",
        mode_to_str(prev_mode),
        mode_to_str(sup.mode),
        restart ? 1 : 0,
        (unsigned long)sup.user_total_us,
        (unsigned long)(sup.user_total_us / 1000000u),
        sup.user_tx_enable ? 1 : 0,
        (unsigned long)sup.user_tx_period_us);
    return;
  }

  if (strcmp(line, "balance run") == 0) {
    tone_start(ctx.tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
    canResetPosvelStats();
    canResetRuntimeStats();
    ctx.can_rx_buf1->overflow_count = 0;
    ctx.can_rx_buf2->overflow_count = 0;
    for (uint16_t i = 0; i < sup.esc_count; ++i) {
      sup.esc_alive_false_count[i] = 0u;
    }
    sup.user_tx_enable = true;
    sup.user_total_us = ctx.balance_button_run_us;
    sup.mode = ctx.balance_run_mode;
    *ctx.balance_mode_enter_us = micros();
    Serial.printf("{\"cmd\":\"MODE\",\"source\":\"serial\",\"mode\":\"%s\",\"run_us\":%lu}\r\n",
                  mode_to_str(sup.mode),
                  (unsigned long)sup.user_total_us);
    return;
  }

  if (strcmp(line, "rc show") == 0) {
    print_rc_channels_snapshot(sup);
    return;
  }

  if (strcmp(line, "rc run") == 0) {
    tone_start(ctx.tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
    ctx.can_rx_buf1->overflow_count = 0;
    ctx.can_rx_buf2->overflow_count = 0;
    for (uint16_t i = 0; i < sup.esc_count; ++i) {
      sup.esc_alive_false_count[i] = 0u;
    }
    sup.user_tx_enable = true;
    sup.user_total_us = 0u;
    sup.user_rc_drive_enable = true;
    sup.mode = SUP_MODE_TEST_CAN;
    Serial.printf(
        "{\"cmd\":\"MODE\",\"source\":\"serial\",\"mode\":\"SUP_MODE_TEST_CAN\",\"rc_drive\":1,"
        "\"throttle_ch\":%u,\"steer_ch\":%u,\"throttle_invert\":%d,\"steer_invert\":%d,\"deadband\":%.3f,\"max_torque_nm\":%.3f,\"tx_period_us\":%lu,\"tx_hz\":%.2f}\r\n",
        (unsigned int)(sup.user_rc_throttle_ch + 1u),
        (unsigned int)(sup.user_rc_steer_ch + 1u),
        sup.user_rc_throttle_invert ? 1 : 0,
        sup.user_rc_steer_invert ? 1 : 0,
        sup.user_rc_deadband,
        sup.user_rc_max_torque_nm,
        (unsigned long)sup.user_tx_period_us,
        (sup.user_tx_period_us > 0u)
            ? (1000000.0f / (float)sup.user_tx_period_us)
            : 0.0f);
    return;
  }

  if (strcmp(line, "rc stop") == 0) {
    sup.user_rc_drive_enable = false;
    sup.mode = SUP_MODE_IDLE;
    *ctx.balance_mode_enter_us = 0u;
    sup.user_total_us = 0u;
    Serial.printf("{\"cmd\":\"MODE\",\"source\":\"serial\",\"mode\":\"SUP_MODE_IDLE\",\"reason\":\"rc_stop\"}\r\n");
    return;
  }

  float rc_max_nm = 0.0f;
  if (sscanf(line, "rc max %f", &rc_max_nm) == 1) {
    if (!(rc_max_nm >= 0.0f && rc_max_nm <= 2.0f)) {
      Serial.printf("{\"cmd\":\"RC_CFG_ERR\",\"reason\":\"max_torque_out_of_range\",\"value\":%.3f,\"min\":0.0,\"max\":2.0}\r\n",
                    rc_max_nm);
    } else {
      sup.user_rc_max_torque_nm = rc_max_nm;
      Serial.printf("{\"cmd\":\"RC_CFG\",\"max_torque_nm\":%.3f}\r\n", sup.user_rc_max_torque_nm);
    }
    return;
  }

  uint32_t rc_ch_t = 0u;
  uint32_t rc_ch_s = 0u;
  if (sscanf(line, "rc ch %lu %lu", &rc_ch_t, &rc_ch_s) == 2) {
    if (rc_ch_t < 1u || rc_ch_s < 1u ||
        rc_ch_t > sup.rc_count || rc_ch_s > sup.rc_count) {
      Serial.printf("{\"cmd\":\"RC_CFG_ERR\",\"reason\":\"channel_out_of_range\",\"count\":%u}\r\n",
                    (unsigned int)sup.rc_count);
    } else {
      sup.user_rc_throttle_ch = (uint8_t)(rc_ch_t - 1u);
      sup.user_rc_steer_ch = (uint8_t)(rc_ch_s - 1u);
      Serial.printf("{\"cmd\":\"RC_CFG\",\"throttle_ch\":%u,\"steer_ch\":%u}\r\n",
                    (unsigned int)rc_ch_t, (unsigned int)rc_ch_s);
    }
    return;
  }

  uint32_t inv_t = 0u;
  uint32_t inv_s = 0u;
  if (sscanf(line, "rc invert %lu %lu", &inv_t, &inv_s) == 2) {
    if ((inv_t > 1u) || (inv_s > 1u)) {
      Serial.printf("{\"cmd\":\"RC_CFG_ERR\",\"reason\":\"invert_out_of_range\",\"expected\":\"0_or_1\"}\r\n");
    } else {
      sup.user_rc_throttle_invert = (inv_t != 0u);
      sup.user_rc_steer_invert = (inv_s != 0u);
      Serial.printf("{\"cmd\":\"RC_CFG\",\"throttle_invert\":%d,\"steer_invert\":%d}\r\n",
                    sup.user_rc_throttle_invert ? 1 : 0,
                    sup.user_rc_steer_invert ? 1 : 0);
    }
    return;
  }

  float rc_deadband = 0.0f;
  if (sscanf(line, "rc deadband %f", &rc_deadband) == 1) {
    if (!(rc_deadband >= 0.0f && rc_deadband < 0.5f)) {
      Serial.printf("{\"cmd\":\"RC_CFG_ERR\",\"reason\":\"deadband_out_of_range\",\"value\":%.3f,\"min\":0.0,\"max\":0.49}\r\n",
                    rc_deadband);
    } else {
      sup.user_rc_deadband = rc_deadband;
      Serial.printf("{\"cmd\":\"RC_CFG\",\"deadband\":%.3f}\r\n", sup.user_rc_deadband);
    }
    return;
  }

  if (strcmp(line, "verify_angle") == 0) {
    tone_start(ctx.tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
    sup.user_verify_motor_enable = false;
    sup.mode = SUP_VERIFY_ANGLE;
    Serial.printf(
      "{\"cmd\":\"MODE\",\"mode\":\"SUP_VERIFY_ANGLE\",\"motor\":0}\r\n");
    return;
  }

  if (strcmp(line, "motor") == 0) {
    tone_start(ctx.tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
    ctx.can_rx_buf1->overflow_count = 0;
    ctx.can_rx_buf2->overflow_count = 0;
    for (uint16_t i = 0; i < sup.esc_count; ++i) {
      sup.esc_alive_false_count[i] = 0u;
    }
    sup.mode = SUP_VERIFY_ANGLE;
    if (sup.user_verify_motor_enable) {
      sup.user_verify_motor_enable = false;
      Serial.printf(
        "{\"cmd\":\"MODE\",\"mode\":\"SUP_VERIFY_ANGLE\",\"motor\":0,\"tau_left_nm\":%.3f,\"tau_right_nm\":%.3f}\r\n",
        sup.user_verify_tau_left,
        sup.user_verify_tau_right);
    } else {
      sup.user_verify_motor_enable = true;
      sup.user_verify_tau_left = 2.0f;
      sup.user_verify_tau_right = -2.0f;
      Serial.printf(
        "{\"cmd\":\"MODE\",\"mode\":\"SUP_VERIFY_ANGLE\",\"motor\":1,\"tau_left_nm\":%.3f,\"tau_right_nm\":%.3f}\r\n",
        sup.user_verify_tau_left,
        sup.user_verify_tau_right);
    }
    return;
  }

  if (strcmp(line, "tx off") == 0) {
    sup.user_tx_enable = false;
    Serial.printf("{\"cmd\":\"CAN_TX_CFG\",\"tx_enable\":0,\"tx_period_us\":%lu,\"tx_hz\":%.2f}\r\n",
                  (unsigned long)sup.user_tx_period_us,
                  (sup.user_tx_period_us > 0u)
                    ? (1000000.0f / (float)sup.user_tx_period_us)
                    : 0.0f);
    return;
  }

  if (strcmp(line, "tx on") == 0) {
    sup.user_tx_enable = true;
    Serial.printf("{\"cmd\":\"CAN_TX_CFG\",\"tx_enable\":1,\"tx_period_us\":%lu,\"tx_hz\":%.2f}\r\n",
                  (unsigned long)sup.user_tx_period_us,
                  (sup.user_tx_period_us > 0u)
                    ? (1000000.0f / (float)sup.user_tx_period_us)
                    : 0.0f);
    return;
  }

  if (strcmp(line, "stats reset") == 0) {
    canResetPosvelStats();
    canResetRuntimeStats();
    ctx.can_rx_buf1->overflow_count = 0;
    ctx.can_rx_buf2->overflow_count = 0;
    for (uint16_t i = 0; i < sup.esc_count; ++i) {
      sup.esc_alive_false_count[i] = 0u;
    }
    Serial.printf("{\"cmd\":\"CAN_STATS_RESET\",\"ok\":1}\r\n");
    return;
  }

  uint32_t hz = 0;
  if (sscanf(line, "tx hz %lu", &hz) == 1) {
    if (hz == 0u || hz > 2000u) {
      Serial.printf("{\"cmd\":\"CAN_TX_CFG_ERR\",\"reason\":\"hz_out_of_range\",\"hz\":%lu,\"min\":1,\"max\":2000}\r\n",
                    (unsigned long)hz);
    } else {
      sup.user_tx_period_us = 1000000u / hz;
      if (sup.user_tx_period_us == 0u) sup.user_tx_period_us = 1u;
      Serial.printf("{\"cmd\":\"CAN_TX_CFG\",\"tx_enable\":%d,\"tx_period_us\":%lu,\"tx_hz\":%.2f}\r\n",
                    sup.user_tx_enable ? 1 : 0,
                    (unsigned long)sup.user_tx_period_us,
                    (sup.user_tx_period_us > 0u)
                      ? (1000000.0f / (float)sup.user_tx_period_us)
                      : 0.0f);
    }
    return;
  }

  Serial.printf("{\"cmd\":\"CAN_CMD_ERR\",\"line\":\"%s\"}\r\n", line);
}

} // namespace

void console_process_serial(ConsoleCommandContext& ctx) {
  static char input_buf[96] = {0};
  static size_t input_len = 0;
  uint32_t chars_processed = 0u;

  while (Serial.available()) {
#if (SERIAL_CONSOLE_CHAR_BUDGET_PER_CALL > 0u)
    if (chars_processed >= SERIAL_CONSOLE_CHAR_BUDGET_PER_CALL) break;
#endif
    char c = Serial.read();
    chars_processed++;
#if SERIAL_CONSOLE_ECHO
    Serial.write((uint8_t)c);  // Echo input characters back to terminal.
#endif
    if (c == '\r' || c == '\n') {
      if (input_len > 0u) {
        input_buf[input_len] = '\0';
        trim_ascii(input_buf);
        if (input_buf[0] != '\0') {
          process_serial_line(ctx, input_buf);
        }
        input_len = 0u;
        input_buf[0] = '\0';
      }
    } else if (c == '\b' || c == 127) {
      if (input_len > 0u) {
        input_len--;
        input_buf[input_len] = '\0';
      }
    } else if (isprint((unsigned char)c)) {
      if (input_len < (sizeof(input_buf) - 1u)) {
        input_buf[input_len++] = c;
        input_buf[input_len] = '\0';
      }
    }
  }
}
