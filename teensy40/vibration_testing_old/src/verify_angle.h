#pragma once
#include "supervisor.h"
#include <FlexCAN_T4.h>

void verify_angle(Supervisor_typedef *sup, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can);

