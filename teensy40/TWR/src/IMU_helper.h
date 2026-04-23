#pragma once

#include <Arduino.h>

#include "ICM42688.h"
#include "supervisor.h"

struct IMUHelperInitResult {
  bool ok;
  const char *step;
  int status;
};

IMUHelperInitResult imu_helper_init(ICM42688 &imu_dev, uint8_t cs_pin, uint8_t int_pin);
void imu_helper_poll(ICM42688 &imu_dev, Supervisor_typedef &sup, uint32_t now_us);
void update_supervisor_imu(ICM42688 &imu_dev, Supervisor_typedef &sup, uint32_t now_us);
float imu_helper_pitch_accel_from_last_sample();
void imu_helper_get_last_accel_g(float *ax_g, float *ay_g, float *az_g);
