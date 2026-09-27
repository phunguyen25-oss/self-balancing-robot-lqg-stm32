/**
 * @file    calibration.h
 * @brief   Startup calibration sequence: I2C self-test, gyro bias, IMU
 *          mounting-offset (accel zero-tilt reference), encoder zero.
 *
 * Runs once at boot, BLOCKING (the control ISR is not yet enabled at this
 * point -- see main.c boot flow). The robot must be held/rested still and
 * approximately upright while this runs.
 */
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "mpu6050.h"
#include "as5600.h"

typedef struct {
    bool ok;
    const char *fail_reason;      /* static string literal, set only if !ok */
    float gyro_bias_radps[3];
    float accel_theta_offset_rad; /* IMU mounting-tilt correction: subtract
                                      this from every future accel-derived
                                      theta reading */
} CalibrationResult_t;

/**
 * Runs the full sequence:
 *   1. Self-test MPU6050 (WHO_AM_I) + both AS5600 (magnet detected)
 *   2. Gyro bias calibration (robot must be still) + variance sanity check
 *   3. Accel-based mounting-tilt offset (robot must be roughly upright) +
 *      sanity check against CALIB_MAX_INITIAL_TILT_RAD
 *   4. Zero both encoders at their current physical position
 *
 * Any failed sub-step aborts immediately with ok=false and fail_reason set
 * -- caller (main.c) must go to ROBOT_STATE_FAULT, not attempt to balance
 * on a partial/unvalidated calibration.
 */
CalibrationResult_t Calibration_Run(I2C_HandleTypeDef *hi2c_imu,
                                     I2C_HandleTypeDef *hi2c_enc_l,
                                     I2C_HandleTypeDef *hi2c_enc_r,
                                     MPU6050_t *imu_state,
                                     AS5600_t *enc_l, AS5600_t *enc_r);

#endif /* CALIBRATION_H */
