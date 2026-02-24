#ifndef TEST_CAN_TRANSMIT_MODE_H
#define TEST_CAN_TRANSMIT_MODE_H

#include "supervisor.h"
#include <FlexCAN_T4.h>

void test_can_transmit_mode(
    Supervisor_typedef *sup,
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can);

#endif
