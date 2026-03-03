#include "test_can_transmit_mode.h"

static bool test_can_first_entry = true;
static uint32_t test_can_start_us = 0;

void test_can_transmit_mode(
    Supervisor_typedef *sup,
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {
  if (!sup) return;

  (void)can;

  if (test_can_first_entry) {
    test_can_first_entry = false;
    test_can_start_us = micros();
    Serial.println("test_can: entered");
  }

  const uint32_t elapsed_us = micros() - test_can_start_us;
  if (sup->user_total_us > 0 && elapsed_us > sup->user_total_us) {
    Serial.printf("test_can exit: timed stop -> idle (elapsed=%lu us, limit=%lu us)\r\n",
                  (unsigned long)elapsed_us,
                  (unsigned long)sup->user_total_us);
    sup->mode = SUP_MODE_IDLE;
    test_can_first_entry = true;
    test_can_start_us = 0;
  }
}
