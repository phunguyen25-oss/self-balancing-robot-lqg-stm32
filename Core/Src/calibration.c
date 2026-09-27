/**
 * @file calibration.c
 * @see  calibration.h
 */
#include "calibration.h"
#include "config.h"
#include <math.h>
#include <stddef.h>

static CalibrationResult_t fail(const char *reason)
{
    CalibrationResult_t r;
    r.ok = false;
    r.fail_reason = reason;
    r.gyro_bias_radps[0] = r.gyro_bias_radps[1] = r.gyro_bias_radps[2] = 0.0f;
    r.accel_theta_offset_rad = 0.0f;
    return r;
}

CalibrationResult_t Calibration_Run(I2C_HandleTypeDef *hi2c_imu,
                                     I2C_HandleTypeDef *hi2c_enc_l,
                                     I2C_HandleTypeDef *hi2c_enc_r,
                                     MPU6050_t *imu_state,
                                     AS5600_t *enc_l, AS5600_t *enc_r)
{
    (void)hi2c_enc_l;
    (void)hi2c_enc_r;

    /* ---- 1) self-test everything before touching any calibration math ---- */
    if (MPU6050_SelfTest(hi2c_imu) != HAL_OK) {
        return fail("MPU6050 WHO_AM_I self-test failed (check wiring/address)");
    }
    if (AS5600_SelfTest(enc_l) != HAL_OK) {
        return fail("left AS5600 magnet not detected (check magnet alignment/gap)");
    }
    if (AS5600_SelfTest(enc_r) != HAL_OK) {
        return fail("right AS5600 magnet not detected (check magnet alignment/gap)");
    }

    /* ---- 2) gyro bias, robot must be held still ---- */
    float gyro_bias[3];
    float gyro_std_dps[3];
    if (MPU6050_CalibrateGyroBias(hi2c_imu, gyro_bias, gyro_std_dps) != HAL_OK) {
        return fail("gyro bias calibration: too many I2C read failures");
    }
    for (int a = 0; a < 3; a++) {
        if (gyro_std_dps[a] > CALIB_GYRO_BIAS_MAX_DPS) {
            return fail("gyro bias calibration: robot was moved/disturbed while calibrating "
                         "(std-dev exceeded CALIB_GYRO_BIAS_MAX_DPS) -- hold still and retry");
        }
    }

    /* ---- 3) accel-based mounting-tilt offset, robot must be ~upright ---- */
    double theta_acc_sum = 0.0;
    uint32_t n_ok = 0;
    for (uint32_t i = 0; i < CALIB_ACCEL_SAMPLES; i++) {
        MPU6050_Raw_t raw;
        if (MPU6050_ReadRaw(hi2c_imu, &raw) == HAL_OK) {
            float theta_acc = MPU6050_ComputeAccelTiltRad(raw.accel_g[0], raw.accel_g[2]);
            theta_acc_sum += (double)theta_acc;
            n_ok++;
        }
        HAL_Delay(CONTROL_PERIOD_MS);
    }
    if (n_ok < (CALIB_ACCEL_SAMPLES / 2U)) {
        return fail("accel tilt-offset calibration: too many I2C read failures");
    }
    float theta_offset = (float)(theta_acc_sum / (double)n_ok);
    if (fabsf(theta_offset) > CALIB_MAX_INITIAL_TILT_RAD) {
        return fail("accel tilt-offset calibration: robot was not held upright enough "
                     "(exceeds CALIB_MAX_INITIAL_TILT_RAD) -- re-seat it and retry");
    }

    /* ---- 4) zero both encoders at the current physical position ---- */
    if (AS5600_Zero(enc_l) != HAL_OK) {
        return fail("left encoder zero failed (I2C read error)");
    }
    if (AS5600_Zero(enc_r) != HAL_OK) {
        return fail("right encoder zero failed (I2C read error)");
    }

    /* ---- commit results ---- */
    CalibrationResult_t result;
    result.ok = true;
    result.fail_reason = NULL;
    for (int a = 0; a < 3; a++) result.gyro_bias_radps[a] = gyro_bias[a];
    result.accel_theta_offset_rad = theta_offset;

    imu_state->gyro_bias_radps[0] = gyro_bias[0];
    imu_state->gyro_bias_radps[1] = gyro_bias[1];
    imu_state->gyro_bias_radps[2] = gyro_bias[2];
    imu_state->theta = 0.0f; /* complementary filter zeroed relative to this pose */
    imu_state->calibrated = true;

    return result;
}
