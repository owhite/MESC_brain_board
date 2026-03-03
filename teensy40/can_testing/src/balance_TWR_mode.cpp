#include "supervisor.h"
#include "balance_TWR_mode.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>

#define SEND_TORQUE
#define SEND_TELEMETRY
#define CAN_TX_PROOF_SUMMARY 1

static constexpr float TORQUE_START_LEFT_NM = 2.0f;
static constexpr float TORQUE_START_RIGHT_NM = -2.0f;

static int report_counter = 0;
static bool first_entry = true;
static uint32_t start_time_us = 0;
static uint32_t last_posvel_rx_used_L_us = 0;
static uint32_t last_posvel_rx_used_R_us = 0;

struct CanTxProofStats {
  uint32_t tx_attempts = 0;
  uint32_t tx_ok = 0;
  uint32_t tx_fail = 0;
  uint32_t last_report_us = 0;
};
static CanTxProofStats g_can_tx_proof;

void balance_TWR_mode(Supervisor_typedef *sup,
                      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {
  if (!sup) return;

  if (!sup->esc[0].state.alive) {
    CAN_message_t msgL, msgR;
    msgL.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);
    msgR.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[1].config.node_id);
    msgL.len = msgR.len = 8;
    msgL.flags.extended = msgR.flags.extended = 1;
    canPackFloat(0.0f, msgL.buf);
    canPackFloat(0.0f, msgL.buf + 4);
    canPackFloat(0.0f, msgR.buf);
    canPackFloat(0.0f, msgR.buf + 4);
#ifdef SEND_TORQUE
    can.write(msgL);
    can.write(msgR);
#endif

    Serial.println("balance exit: left ESC not alive -> idle");
    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    last_posvel_rx_used_L_us = 0;
    last_posvel_rx_used_R_us = 0;
    return;
  }

  if (first_entry) {
    first_entry = false;
    start_time_us = micros();
    last_posvel_rx_used_L_us = 0;
    last_posvel_rx_used_R_us = 0;
    g_can_tx_proof = CanTxProofStats{};
    g_can_tx_proof.last_report_us = start_time_us;
    Serial.println("{\"cmd\":\"PRINT\",\"note\":\"Balance mode started\"}");
  }

  const uint32_t elapsed_us = micros() - start_time_us;

  const float pos_L = sup->esc[0].state.pos_rad;
  const float pos_R = sup->esc[1].state.pos_rad;
  const float vel_L = sup->esc[0].state.vel_rad_s;
  const float vel_R = sup->esc[1].state.vel_rad_s;

  const float dt_loop_s = CONTROL_PERIOD_US * 1e-6f;
  const uint32_t pos_L_us = sup->esc[0].status.last_update_us;
  const uint32_t pos_R_us = sup->esc[1].status.last_update_us;
  const bool new_pos_L = (pos_L_us != 0u) && (pos_L_us != last_posvel_rx_used_L_us);
  const bool new_pos_R = (pos_R_us != 0u) && (pos_R_us != last_posvel_rx_used_R_us);
  const bool new_pos = new_pos_L || new_pos_R;

  float dt_pos_L_s = dt_loop_s;
  float dt_pos_R_s = dt_loop_s;
  if (new_pos_L) {
    dt_pos_L_s = (last_posvel_rx_used_L_us != 0u)
                   ? ((float)((uint32_t)(pos_L_us - last_posvel_rx_used_L_us)) * 1e-6f)
                   : 0.002f;
    last_posvel_rx_used_L_us = pos_L_us;
  }
  if (new_pos_R) {
    dt_pos_R_s = (last_posvel_rx_used_R_us != 0u)
                   ? ((float)((uint32_t)(pos_R_us - last_posvel_rx_used_R_us)) * 1e-6f)
                   : 0.002f;
    last_posvel_rx_used_R_us = pos_R_us;
  }

  CAN_message_t msgL, msgR;
  msgL.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);
  msgR.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[1].config.node_id);
  msgL.len = 8;
  msgR.len = 8;
  msgL.flags.extended = 1;
  msgR.flags.extended = 1;
  canPackFloat(TORQUE_START_LEFT_NM, msgL.buf);
  canPackFloat(0.0f, msgL.buf + 4);
  canPackFloat(TORQUE_START_RIGHT_NM, msgR.buf);
  canPackFloat(0.0f, msgR.buf + 4);

#ifdef SEND_TORQUE
  const bool okL = can.write(msgL);
  const bool okR = can.write(msgR);
  g_can_tx_proof.tx_attempts += 2u;
  g_can_tx_proof.tx_ok += (okL ? 1u : 0u) + (okR ? 1u : 0u);
  g_can_tx_proof.tx_fail += (!okL ? 1u : 0u) + (!okR ? 1u : 0u);
#endif

#if CAN_TX_PROOF_SUMMARY
  const uint32_t now_us = micros();
  if ((uint32_t)(now_us - g_can_tx_proof.last_report_us) >= 1000000u) {
    const uint32_t attempts = g_can_tx_proof.tx_attempts;
    const float fail_pct = (attempts > 0u)
                             ? (100.0f * (float)g_can_tx_proof.tx_fail / (float)attempts)
                             : 0.0f;
    const uint32_t last_posvel_rx_us = canGetLastPosVelRxUs();
    const uint32_t posvel_age_us =
        (last_posvel_rx_us > 0u) ? (uint32_t)(now_us - last_posvel_rx_us) : UINT32_MAX;
    Serial.printf(
        "{\"cmd\":\"CAN_TXQ_SUM\",\"attempts\":%lu,\"ok\":%lu,\"fail\":%lu,\"fail_pct\":%.3f,\"mode\":%d,\"posvel_age_us\":%lu}\r\n",
        (unsigned long)g_can_tx_proof.tx_attempts,
        (unsigned long)g_can_tx_proof.tx_ok,
        (unsigned long)g_can_tx_proof.tx_fail,
        fail_pct,
        (int)sup->mode,
        (unsigned long)posvel_age_us);
    g_can_tx_proof.last_report_us += 1000000u;
  }
#endif

  if (++report_counter >= TELEMETRY_DECIMATE) {
    report_counter = 0;
#ifdef SEND_TELEMETRY
    const uint32_t loop_dt_us = sup->timing.dt_us;
    const float loop_hz = (loop_dt_us > 0u) ? (1000000.0f / (float)loop_dt_us) : 0.0f;
    const int32_t dt_err_from_500hz_us = (int32_t)loop_dt_us - 2000;
    const int dt_ok_500hz = (abs(dt_err_from_500hz_us) <= 200) ? 1 : 0;

    Serial.printf(
        "{\"t\":%lu,\"pos_L_raw\":%.6f,\"pos_R_raw\":%.6f,\"vel_L_raw\":%.6f,\"vel_R_raw\":%.6f,"
        "\"new_pos\":%d,\"new_pos_L\":%d,\"new_pos_R\":%d,"
        "\"dt_pos_us\":%lu,\"dt_pos_L_us\":%lu,\"dt_pos_R_us\":%lu,"
        "\"loop_dt_us\":%lu,\"loop_hz\":%.2f,\"dt_err_500hz_us\":%ld,\"dt_ok_500hz\":%d,"
        "\"exec_us\":%lu,\"ovr\":%lu}\r\n",
        micros(),
        pos_L,
        pos_R,
        vel_L,
        vel_R,
        new_pos ? 1 : 0,
        new_pos_L ? 1 : 0,
        new_pos_R ? 1 : 0,
        (unsigned long)lroundf(((new_pos_L || new_pos_R) ? (0.5f * (dt_pos_L_s + dt_pos_R_s)) : dt_loop_s) * 1e6f),
        (unsigned long)lroundf(dt_pos_L_s * 1e6f),
        (unsigned long)lroundf(dt_pos_R_s * 1e6f),
        (unsigned long)loop_dt_us,
        loop_hz,
        (long)dt_err_from_500hz_us,
        dt_ok_500hz,
        sup->timing.exec_time_us,
        sup->timing.overruns);
#endif
  }

  if (sup->user_total_us > 0 && elapsed_us > sup->user_total_us) {
    CAN_message_t stopL, stopR;
    stopL.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);
    stopR.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[1].config.node_id);
    stopL.len = stopR.len = 8;
    stopL.flags.extended = stopR.flags.extended = 1;
    canPackFloat(0.0f, stopL.buf);
    canPackFloat(0.0f, stopL.buf + 4);
    canPackFloat(0.0f, stopR.buf);
    canPackFloat(0.0f, stopR.buf + 4);
#ifdef SEND_TORQUE
    const bool okStopL = can.write(stopL);
    const bool okStopR = can.write(stopR);
    g_can_tx_proof.tx_attempts += 2u;
    g_can_tx_proof.tx_ok += (okStopL ? 1u : 0u) + (okStopR ? 1u : 0u);
    g_can_tx_proof.tx_fail += (!okStopL ? 1u : 0u) + (!okStopR ? 1u : 0u);
#endif

    Serial.printf("balance exit: timed stop -> idle (elapsed=%lu us, limit=%lu us)\r\n",
                  (unsigned long)elapsed_us,
                  (unsigned long)sup->user_total_us);
    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    last_posvel_rx_used_L_us = 0;
    last_posvel_rx_used_R_us = 0;
  }
}
