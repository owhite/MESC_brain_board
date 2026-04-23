#include "IMU_helper.h"

#include <math.h>

#include <SPI.h>

namespace {

static volatile bool g_imu_data_ready = false;

static float g_last_accel_x_g = 0.0f;
static float g_last_accel_y_g = 0.0f;
static float g_last_accel_z_g = 0.0f;

// ---- Mahony 6DOF state (ported from TWR_test) ----
static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;
static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;
static constexpr float TWO_KP = 2.0f * 0.5f;  // Kp = 0.5

// Keep Mahony integral enabled to reject slow gyro bias drift.
static constexpr bool IMU_MAHONY_INTEGRAL_ENABLE = true;
static constexpr float IMU_MAHONY_KI = 0.08f;
static constexpr float TWO_KI = IMU_MAHONY_INTEGRAL_ENABLE ? (2.0f * IMU_MAHONY_KI) : 0.0f;
static constexpr float IMU_MAHONY_INT_CLAMP_RAD_S = 0.35f;
static constexpr float IMU_MAHONY_INT_DECAY_WHEN_UNGATED = 0.9995f;

// Accelerometer trust window for Mahony correction.
static constexpr float ACCEL_CORR_FULL_ERR_G = 0.02f;
static constexpr float ACCEL_CORR_MAX_ERR_G = 0.06f;

// During balance mode, suppress accel correction briefly after high |a|-1 events.
static constexpr bool IMU_ACCEL_BUMP_HOLDOFF_ENABLE = true;
static constexpr float IMU_ACCEL_BUMP_TRIGGER_ERR_G = 0.05f;
static constexpr uint32_t IMU_ACCEL_BUMP_HOLDOFF_US = 120000u;

// In active balance, only trust accelerometer corrections when platform is quiet.
static constexpr bool IMU_BALANCE_ACCEL_ONLY_WHEN_QUIET = true;
static constexpr float IMU_BALANCE_QUIET_PITCH_RATE_MAX_RAD_S = 0.35f;
static constexpr float IMU_BALANCE_QUIET_WHEEL_VEL_MAX_RAD_S = 4.0f;
static uint32_t g_imu_accel_holdoff_until_us = 0u;

static inline bool is_balance_mode(SupervisorMode mode) {
  return (mode == SUP_MODE_BALANCE_HOLD) ||
         (mode == SUP_MODE_BALANCE_TWR) ||
         (mode == SUP_MODE_BALANCE_DEBUG);
}

static void imu_data_ready_isr() {
  g_imu_data_ready = true;
}

static void mahony_update_imu(float gx,
                              float gy,
                              float gz,
                              float ax,
                              float ay,
                              float az,
                              float dt_s,
                              bool allow_accel_correction) {
  float acc_mag = sqrtf(ax * ax + ay * ay + az * az);
  float accel_corr_gain = 0.0f;

  if (acc_mag > 1e-6f) {
    float recip = 1.0f / acc_mag;
    ax *= recip;
    ay *= recip;
    az *= recip;

    const float acc_err_g = fabsf(acc_mag - 1.0f);
    if (acc_err_g <= ACCEL_CORR_FULL_ERR_G) {
      accel_corr_gain = 1.0f;
    } else if (acc_err_g < ACCEL_CORR_MAX_ERR_G) {
      accel_corr_gain = (ACCEL_CORR_MAX_ERR_G - acc_err_g) /
                        (ACCEL_CORR_MAX_ERR_G - ACCEL_CORR_FULL_ERR_G);
    }
  }

  float ex = 0.0f;
  float ey = 0.0f;
  float ez = 0.0f;

  if (accel_corr_gain > 0.0f && allow_accel_correction) {
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
    ex *= accel_corr_gain;
    ey *= accel_corr_gain;
    ez *= accel_corr_gain;

    if (TWO_KI > 0.0f) {
      integralFBx += TWO_KI * ex * dt_s;
      integralFBy += TWO_KI * ey * dt_s;
      integralFBz += TWO_KI * ez * dt_s;
      integralFBx = constrain(integralFBx, -IMU_MAHONY_INT_CLAMP_RAD_S, IMU_MAHONY_INT_CLAMP_RAD_S);
      integralFBy = constrain(integralFBy, -IMU_MAHONY_INT_CLAMP_RAD_S, IMU_MAHONY_INT_CLAMP_RAD_S);
      integralFBz = constrain(integralFBz, -IMU_MAHONY_INT_CLAMP_RAD_S, IMU_MAHONY_INT_CLAMP_RAD_S);
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = integralFBy = integralFBz = 0.0f;
    }

    gx += TWO_KP * ex;
    gy += TWO_KP * ey;
    gz += TWO_KP * ez;
  } else {
    // During holdoff/out-of-gate windows, keep learned bias estimate with slow decay.
    if (TWO_KI > 0.0f) {
      integralFBx *= IMU_MAHONY_INT_DECAY_WHEN_UNGATED;
      integralFBy *= IMU_MAHONY_INT_DECAY_WHEN_UNGATED;
      integralFBz *= IMU_MAHONY_INT_DECAY_WHEN_UNGATED;
    } else {
      integralFBx = integralFBy = integralFBz = 0.0f;
    }
  }

  gx *= 0.5f * dt_s;
  gy *= 0.5f * dt_s;
  gz *= 0.5f * dt_s;

  float qa = q0;
  float qb = q1;
  float qc = q2;
  float qd = q3;

  q0 += (-qb * gx - qc * gy - qd * gz);
  q1 += (qa * gx + qc * gz - qd * gy);
  q2 += (qa * gy - qb * gz + qd * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  float norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= norm;
  q1 *= norm;
  q2 *= norm;
  q3 *= norm;
}

}  // namespace

IMUHelperInitResult imu_helper_init(ICM42688 &imu_dev, uint8_t cs_pin, uint8_t int_pin) {
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);
  pinMode(int_pin, INPUT);
  SPI.begin();

  int imu_status = imu_dev.begin();
  if (imu_status < 0) return {false, "begin", imu_status};

  imu_status = imu_dev.setAccelODR(ICM42688::odr2k);
  if (imu_status < 0) return {false, "setAccelODR", imu_status};

  imu_status = imu_dev.setGyroODR(ICM42688::odr2k);
  if (imu_status < 0) return {false, "setGyroODR", imu_status};

  imu_status = imu_dev.enableDataReadyInterrupt();
  if (imu_status < 0) return {false, "enableDataReadyInterrupt", imu_status};

  // Clear any pending DRDY latch before attaching ISR.
  (void)imu_dev.getRawAGT();
  g_imu_data_ready = false;
  attachInterrupt(digitalPinToInterrupt(int_pin), imu_data_ready_isr, RISING);
  return {true, "ok", 0};
}

void imu_helper_poll(ICM42688 &imu_dev, Supervisor_typedef &sup, uint32_t now_us) {
  if (!g_imu_data_ready) return;

  noInterrupts();
  g_imu_data_ready = false;
  interrupts();

  if (imu_dev.getAGT() > 0) {
    update_supervisor_imu(imu_dev, sup, now_us);
  } else {
    sup.imu.valid = false;
    sup.imu.last_update_us = now_us;
  }
}

void update_supervisor_imu(ICM42688 &imu_dev, Supervisor_typedef &sup, uint32_t now_us) {
  float dt_s = (now_us - sup.imu.last_update_us) * 1e-6f;
  if (dt_s < 0.0005f || dt_s > 0.005f) {
    dt_s = CONTROL_PERIOD_US * 1e-6f;
  }

  float ax = imu_dev.accX();
  float ay = imu_dev.accY();
  float az = imu_dev.accZ();
  g_last_accel_x_g = ax;
  g_last_accel_y_g = ay;
  g_last_accel_z_g = az;
  float acc_mag = sqrtf(ax * ax + ay * ay + az * az);
  const float acc_err_g = fabsf(acc_mag - 1.0f);
  uint8_t accel_valid = (acc_err_g <= ACCEL_CORR_MAX_ERR_G) ? 1u : 0u;
  const bool in_balance_mode = is_balance_mode(sup.mode);
  if (IMU_ACCEL_BUMP_HOLDOFF_ENABLE && in_balance_mode && acc_err_g > IMU_ACCEL_BUMP_TRIGGER_ERR_G) {
    // Refresh holdoff on every detected bump-like sample.
    g_imu_accel_holdoff_until_us = now_us + IMU_ACCEL_BUMP_HOLDOFF_US;
  }
  const bool allow_accel_correction =
      !(IMU_ACCEL_BUMP_HOLDOFF_ENABLE && in_balance_mode && now_us < g_imu_accel_holdoff_until_us);
  const bool quiet_for_balance_accel =
      (fabsf(imu_dev.gyrY() * DEG_TO_RAD) <= IMU_BALANCE_QUIET_PITCH_RATE_MAX_RAD_S) &&
      (fabsf(sup.esc[0].state.vel_rad_s) <= IMU_BALANCE_QUIET_WHEEL_VEL_MAX_RAD_S) &&
      (fabsf(sup.esc[1].state.vel_rad_s) <= IMU_BALANCE_QUIET_WHEEL_VEL_MAX_RAD_S);
  const bool allow_accel_correction_final =
      allow_accel_correction && (!in_balance_mode || !IMU_BALANCE_ACCEL_ONLY_WHEN_QUIET || quiet_for_balance_accel);

  float gx = imu_dev.gyrX() * DEG_TO_RAD;
  float gy = imu_dev.gyrY() * DEG_TO_RAD;
  float gz = imu_dev.gyrZ() * DEG_TO_RAD;

  mahony_update_imu(gx, gy, gz, ax, ay, az, dt_s, allow_accel_correction_final);

  float pitch_rad = asinf(2.0f * (q0 * q2 - q3 * q1));
  float pitch_rate_raw = gy;

  const float rate_alpha = 0.15f;
  float pitch_rate = rate_alpha * pitch_rate_raw + (1.0f - rate_alpha) * sup.imu.pitch_rate;

  sup.imu.pitch_rad = pitch_rad;
  sup.imu.pitch_rate_raw = pitch_rate_raw;
  sup.imu.pitch_rate = pitch_rate;
  sup.imu.accel_mag_g = acc_mag;
  sup.imu.accel_valid = accel_valid;
  sup.imu.mahony_int_fb_y = integralFBy;
  sup.imu.valid = true;
  sup.imu.last_update_us = now_us;
}

float imu_helper_pitch_accel_from_last_sample() {
  return atan2f(g_last_accel_x_g, g_last_accel_z_g);
}

void imu_helper_get_last_accel_g(float *ax_g, float *ay_g, float *az_g) {
  if (ax_g != nullptr) *ax_g = g_last_accel_x_g;
  if (ay_g != nullptr) *ay_g = g_last_accel_y_g;
  if (az_g != nullptr) *az_g = g_last_accel_z_g;
}
