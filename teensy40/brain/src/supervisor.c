#include "supervisor.h"
#include "esc.h"
#include <string.h>

void init_supervisor(Supervisor_typedef *sup,
                     uint16_t esc_count,
                     const char *esc_names[], const uint16_t node_ids[]) {
    if (!sup) return;
    memset(sup, 0, sizeof(*sup));

    if (esc_count > SUPERVISOR_MAX_ESCS) esc_count = SUPERVISOR_MAX_ESCS;
    sup->esc_count = esc_count;

    for (uint16_t i = 0; i < sup->esc_count; ++i) {
        const char *nm = (esc_names && esc_names[i]) ? esc_names[i] : "";
        uint16_t nid   = node_ids ? node_ids[i] : 0;
        init_ESC(&sup->esc[i], nm, nid);
        sup->last_esc_heartbeat_us[i] = 0;
    }

    sup->imu.valid = false;
    sup->last_imu_update_us = 0;

    sup->mode      = SUP_MODE_IDLE;
    sup->gait_mode = GAIT_IDLE;
    sup->sp.v_forward_mps  = 0.0f;
    sup->sp.yaw_rate_rps   = 0.0f;
    sup->sp.body_pitch_rad = 0.0f;
    sup->xhat.v_forward_mps  = 0.0f;
    sup->xhat.yaw_rate_rps   = 0.0f;
    sup->xhat.body_pitch_rad = 0.0f;

    sup->t_now_us  = 0;
    sup->t_prev_us = 0;
    sup->dt_s      = 0.0f;

    sup->error_flags = SUP_ERR_OK;

    sup->control_cycle_count = 0;
    sup->can_rx_count = 0;
    sup->can_tx_count = 0;
}
