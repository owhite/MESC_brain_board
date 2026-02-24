#pragma once
#include "supervisor.h"
#include <FlexCAN_T4.h>
#include <Arduino.h>

void balance_TWR_mode(Supervisor_typedef *sup,
			      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can);

bool telemetry_start_unwrap_dump();
bool telemetry_service_unwrap_dump(Stream &out, uint16_t max_bytes_per_call);
bool telemetry_unwrap_dump_active();
void telemetry_dump_unwrap(Stream &out);
