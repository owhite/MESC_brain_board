// esc.c
#include "esc.h"
#include <string.h>   // for memset, strncpy

void init_ESC(ESC_motor_typedef *esc, const char *name, uint16_t node_id) {
    if (esc == NULL) return;

    // Reset everything to 0
    memset(esc, 0, sizeof(ESC_motor_typedef));

    // Initialize Config
    strncpy(esc->Config.name, name, sizeof(esc->Config.name) - 1);
    esc->Config.node_id = node_id;
    esc->Config.pole_pairs = 7;       // default example
    esc->Config.encoder_offset = 0;
    esc->Config.max_amps = 50;        // default max current
    esc->Config.max_volts = 480;      // default max voltage
    esc->Config.min_volts = 0;        // default min voltage
    esc->Config.direction = 1;        // forward by default

    // Initialize runtime state
    esc->error = ESC_ERR_OK;
    esc->adc1_req = 0;
    esc->direction = 1;

    // Raw data defaults
    esc->Raw.adc1 = 0;
    esc->Raw.adc2 = 0;
    esc->Raw.bus_volt = 0;
    esc->Raw.bus_current = 0;
    esc->Raw.FOC_angle = 0;
    esc->Raw.TMOS = 25; // assume room temp
    esc->Raw.TMOT = 25;
}

