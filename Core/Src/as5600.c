/**
 * @file as5600.c
 * @see  as5600.h
 */
#include "as5600.h"
#include "config.h"
#include <math.h>

#define REG_STATUS       0x0BU
#define REG_RAW_ANGLE_H  0x0CU  /* 2 bytes, top 4 bits of MSB are don't-care */
#define STATUS_MD_BIT    (1U << 5) /* magnet detected */

#define I2C_TIMEOUT_MS   50U

HAL_StatusTypeDef AS5600_Init(AS5600_t *enc, I2C_HandleTypeDef *hi2c)
{
    enc->hi2c = hi2c;
    enc->raw_prev = 0;
    enc->angle_rad_unwrapped = 0.0f;
    enc->initialized = false;

    uint16_t raw;
    HAL_StatusTypeDef st = AS5600_ReadRawAngle(enc, &raw);
    if (st != HAL_OK) return st;

    enc->raw_prev = raw;
    enc->initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_SelfTest(AS5600_t *enc)
{
    uint8_t status = 0;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(enc->hi2c, AS5600_I2C_ADDR, REG_STATUS,
                                             I2C_MEMADD_SIZE_8BIT, &status, 1, I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;
    return (status & STATUS_MD_BIT) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef AS5600_ReadRawAngle(AS5600_t *enc, uint16_t *raw_out)
{
    uint8_t buf[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(enc->hi2c, AS5600_I2C_ADDR, REG_RAW_ANGLE_H,
                                             I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    *raw_out = (uint16_t)(((buf[0] << 8) | buf[1]) & 0x0FFFU); /* 12-bit */
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_Update(AS5600_t *enc)
{
    uint16_t raw;
    HAL_StatusTypeDef st = AS5600_ReadRawAngle(enc, &raw);
    if (st != HAL_OK) return st; /* leave accumulator untouched on failure */

    int32_t delta = (int32_t)raw - (int32_t)enc->raw_prev;
    if (delta > (int32_t)(AS5600_RAW_ANGLE_MAX / 2)) {
        delta -= (int32_t)AS5600_RAW_ANGLE_MAX; /* wrapped backward (4095->0 crossing) */
    } else if (delta < -(int32_t)(AS5600_RAW_ANGLE_MAX / 2)) {
        delta += (int32_t)AS5600_RAW_ANGLE_MAX; /* wrapped forward (0->4095 crossing) */
    }

    float delta_rad = ((float)delta / (float)AS5600_RAW_ANGLE_MAX) * (2.0f * 3.14159265f);
    enc->angle_rad_unwrapped += delta_rad;
    enc->raw_prev = raw;

    return HAL_OK;
}

HAL_StatusTypeDef AS5600_Zero(AS5600_t *enc)
{
    uint16_t raw;
    HAL_StatusTypeDef st = AS5600_ReadRawAngle(enc, &raw);
    if (st != HAL_OK) return st;

    enc->raw_prev = raw;
    enc->angle_rad_unwrapped = 0.0f;
    return HAL_OK;
}

float AS5600_GetWheelPositionM(const AS5600_t *enc)
{
    return enc->angle_rad_unwrapped * WHEEL_RADIUS_M;
}
