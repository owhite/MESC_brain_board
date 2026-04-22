#ifndef CONSOLE_COMMANDS_H
#define CONSOLE_COMMANDS_H

#include <Arduino.h>
#include "supervisor.h"
#include "tone_player.h"
#include "CAN_helper.h"

struct ConsoleCommandContext {
  Supervisor_typedef* sup;
  TonePlayer* tone;
  CANBuffer* can_rx_buf1;
  CANBuffer* can_rx_buf2;
  uint32_t* balance_mode_enter_us;
  SupervisorMode balance_run_mode;
  uint32_t can_test_run_default_us;
  uint32_t balance_button_run_us;
};

void console_process_serial(ConsoleCommandContext& ctx);

#endif // CONSOLE_COMMANDS_H
