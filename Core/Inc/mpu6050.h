/**
 * @file    mpu6050.h
 * @brief   MPU6050 I2C driver + gyro/accel calibration + complementary
 *          filter for tilt angle (theta).
 *
 * WHY A COMPLEMENTARY FILTER AND NOT AN EKF HERE:
 *   This is a single-axis (pitch) tilt estimate from a 2-axis subset of
 *   accel + 1-axis gyro. A complementary filter is the standard, cheap,
 *   well-proven tool for exactly this job (it's a fixed-structure,
 *   frequency-domain blend: trust the gyro short-term, trust accel
 *   long-term to correct gyro drift). An EKF would add real cost
 *   (Jacobians, a 2x2+ covariance propagated every cycle) for no benefit
 *   at this scope -- there's no multi-sensor nonlinear fusion problem to
 *   solve (no magnetometer, no 3D attitude, no varying model). Reserve
 *   EKF/AHRS (Madgwick/Mahony) for full 3D orientation if this robot ever
 *   grows a magnetometer or needs yaw-from-IMU; not needed here since yaw
 *   hold in this build uses encoder differential instead.
 *
 *   This filter's output (theta_imu) feeds the SEPARATE, fixed-gain
 *   discrete Kalman/Luenberger observer in estimator.c as one of its two
 *   measurements -- two different filters, two different jobs, do not
 *   conflate them (see README "EKF vs KF observer vs complementary filter").
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"  /* provides I2C_HandleTypeDef, HAL_StatusTypeDef */

typedef struct {
    float accel_g[3];   /* raw accel, g,     [x,y,z], NOT bias corrected  */
    float gyro_radps[3];/* raw gyro,  rad/s, [x,y,z], NOT bias corrected  */
} MPU6050_Raw_t;

typedef struct {
    float theta;         /* fused tilt estimate, rad                      */
    float gyro_bias_radps[3];
    bool  calibrated;
} MPU6050_t;

/** Init peripheral registers (wake from sleep, set ranges). Blocking. */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

/** WHO_AM_I register check. Returns HAL_OK only if the expected ID reads back. */
HAL_StatusTypeDef MPU6050_SelfTest(I2C_HandleTypeDef *hi2c);

/** Single burst read of accel+gyro, converted to physical units, NOT bias-corrected. */
HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_Raw_t *out);

/**
 * Blocking gyro bias calibration: averages CALIB_GYRO_SAMPLES readings.
 * Robot MUST be held still and level during this call.
 * @param std_dev_out  [out] per-axis std-dev (dps) of the samples used --
 *                     caller should reject calibration if this exceeds
 *                     CALIB_GYRO_BIAS_MAX_DPS (robot wasn't actually still).
 * @return HAL_OK if I2C reads all succeeded, HAL_ERROR otherwise (bias not updated).
 */
HAL_StatusTypeDef MPU6050_CalibrateGyroBias(I2C_HandleTypeDef *hi2c,
                                             float bias_radps_out[3],
                                             float std_dev_dps_out[3]);

/**
 * theta_acc = atan2(accel_forward, accel_up), the accelerometer-only tilt
 * estimate (noisy, but drift-free). Axis convention: robot mounted with
 * IMU X = forward, Z = up (see README wiring section) -- VERIFY THE SIGN
 * on your actual hardware (tip the robot forward by hand, confirm this
 * returns a positive angle) before trusting the balance direction.
 */
float MPU6050_ComputeAccelTiltRad(float accel_forward_g, float accel_up_g);

/** Complementary filter state -- separate from MPU6050_t so it can be unit
 *  tested without I2C (pure function, no HAL dependency). */
typedef struct {
    float theta;
} ComplementaryFilter_t;

void ComplementaryFilter_Init(ComplementaryFilter_t *cf, float theta0);

/**
 * @param gyro_rate_radps  bias-corrected pitch rate (rad/s), sign convention
 *                          must match theta_acc's (positive = tipping the
 *                          same direction as positive theta_acc)
 * @param theta_acc_rad    from MPU6050_ComputeAccelTiltRad()
 * @param dt                control period, s
 * @return fused theta, rad
 */
float ComplementaryFilter_Update(ComplementaryFilter_t *cf, float gyro_rate_radps,
                                  float theta_acc_rad, float dt);

#endif /* MPU6050_H */
