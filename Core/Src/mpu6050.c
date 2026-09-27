/**
 * @file mpu6050.c
 * @see  mpu6050.h
 */
#include "mpu6050.h"
#include "config.h"
#include <math.h>

/* ---- MPU6050 register map (RM-MPU-6000A-00, relevant subset) ---- */
#define REG_GYRO_CONFIG    0x1BU
#define REG_ACCEL_CONFIG   0x1CU
#define REG_ACCEL_XOUT_H   0x3BU  /* first of 14 contiguous bytes: accel(6) temp(2) gyro(6) */
#define REG_PWR_MGMT_1     0x6BU
#define REG_WHO_AM_I       0x75U

#define I2C_TIMEOUT_MS     50U

static HAL_StatusTypeDef write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                              &val, 1, I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef st;

    /* wake from sleep, use internal 8MHz oscillator (default clksel=0 is fine
     * here; a PLL-locked gyro clock source is nicer but not required) */
    st = write_reg(hi2c, REG_PWR_MGMT_1, 0x00U);
    if (st != HAL_OK) return st;

    HAL_Delay(10); /* datasheet: allow time after waking from sleep */

    /* gyro full scale +-500 dps (FS_SEL=1 -> bits [4:3]=01) */
    st = write_reg(hi2c, REG_GYRO_CONFIG, (0x01U << 3));
    if (st != HAL_OK) return st;

    /* accel full scale +-2g (AFS_SEL=0 -> bits [4:3]=00) */
    st = write_reg(hi2c, REG_ACCEL_CONFIG, 0x00U);
    if (st != HAL_OK) return st;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_SelfTest(I2C_HandleTypeDef *hi2c)
{
    uint8_t whoami = 0;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, REG_WHO_AM_I,
                                             I2C_MEMADD_SIZE_8BIT, &whoami, 1, I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;
    return (whoami == MPU6050_WHOAMI_EXPECTED) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_Raw_t *out)
{
    uint8_t buf[14];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, REG_ACCEL_XOUT_H,
                                             I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf),
                                             I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    /* buf[6..7] = temperature, unused */
    int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    out->accel_g[0] = (float)ax / MPU6050_ACCEL_LSB_PER_G;
    out->accel_g[1] = (float)ay / MPU6050_ACCEL_LSB_PER_G;
    out->accel_g[2] = (float)az / MPU6050_ACCEL_LSB_PER_G;

    const float deg2rad = 3.14159265f / 180.0f;
    out->gyro_radps[0] = ((float)gx / MPU6050_GYRO_LSB_PER_DPS) * deg2rad;
    out->gyro_radps[1] = ((float)gy / MPU6050_GYRO_LSB_PER_DPS) * deg2rad;
    out->gyro_radps[2] = ((float)gz / MPU6050_GYRO_LSB_PER_DPS) * deg2rad;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_CalibrateGyroBias(I2C_HandleTypeDef *hi2c,
                                             float bias_radps_out[3],
                                             float std_dev_dps_out[3])
{
    double sum[3] = {0, 0, 0};
    double sumsq[3] = {0, 0, 0};
    const float rad2deg = 180.0f / 3.14159265f;
    uint32_t n_ok = 0;

    for (uint32_t i = 0; i < CALIB_GYRO_SAMPLES; i++) {
        MPU6050_Raw_t raw;
        if (MPU6050_ReadRaw(hi2c, &raw) != HAL_OK) {
            continue; /* skip a bad sample rather than aborting the whole calibration */
        }
        for (int a = 0; a < 3; a++) {
            double dps = (double)(raw.gyro_radps[a] * rad2deg);
            sum[a] += dps;
            sumsq[a] += dps * dps;
        }
        n_ok++;
        HAL_Delay(CONTROL_PERIOD_MS);
    }

    if (n_ok < (CALIB_GYRO_SAMPLES / 2U)) {
        return HAL_ERROR; /* too many I2C failures -- don't trust this calibration */
    }

    for (int a = 0; a < 3; a++) {
        double mean_dps = sum[a] / (double)n_ok;
        double var_dps = (sumsq[a] / (double)n_ok) - (mean_dps * mean_dps);
        if (var_dps < 0.0) var_dps = 0.0; /* guard tiny negative from float rounding */
        std_dev_dps_out[a] = (float)sqrt(var_dps);
        bias_radps_out[a] = (float)(mean_dps * (3.14159265 / 180.0));
    }

    return HAL_OK;
}

float MPU6050_ComputeAccelTiltRad(float accel_forward_g, float accel_up_g)
{
    /* atan2 stays well-behaved near accel_up_g ~ 0 (robot near +-90deg),
     * unlike a plain asin/division formula -- keep atan2 even though this
     * robot should never legitimately operate anywhere near there (see
     * THETA_FAULT_RAD, which cuts the motors long before this range). */
    return atan2f(accel_forward_g, accel_up_g);
}

void ComplementaryFilter_Init(ComplementaryFilter_t *cf, float theta0)
{
    cf->theta = theta0;
}

float ComplementaryFilter_Update(ComplementaryFilter_t *cf, float gyro_rate_radps,
                                  float theta_acc_rad, float dt)
{
    float theta_gyro = cf->theta + gyro_rate_radps * dt;
    cf->theta = COMP_FILTER_ALPHA * theta_gyro + (1.0f - COMP_FILTER_ALPHA) * theta_acc_rad;
    return cf->theta;
}
