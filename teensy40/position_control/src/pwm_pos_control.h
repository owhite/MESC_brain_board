#pragma once
#include "supervisor.h"
#include <FlexCAN_T4.h>
#include <Arduino.h>

void pwm_pos_control_mode(Supervisor_typedef *sup,
                          FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can);