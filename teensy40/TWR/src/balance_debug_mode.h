#pragma once

#include "supervisor.h"
#include <FlexCAN_T4.h>

void balance_debug_mode(Supervisor_typedef *sup,
                      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                      FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2);

// Called by supervisor when leaving balance mode through external mode change
// (for example pushbutton/CLI), so in-RAM diagnostics are dumped once.
void balance_debug_dump_on_mode_exit(const char *reason);
