/**
 * @file    as5600.h
 * @brief   AS5600 magnetic angle encoder driver -- wraparound-safe angle
 *          unwrapping, zero calibration, wheel linear position.
 *
 * Each wheel gets its own AS5600_t instance bound to its own I2C bus
 * (I2C1 shared with the MPU6050, I2C2 alone) -- see config.h pin map for
 * why: the AS5600's I2C address is fixed in silicon, so two of them
 * cannot coexist on the same bus.
 */
#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t raw_prev;          /* last raw 12-bit reading, for unwrap    */
    float angle_rad_unwrapped;  /* accumulated wheel angle since last zero,
                                    rad -- NOTE: float accumulation is fine
                                    for normal test/demo run lengths; for
                                    very long endurance runs consider
                                    periodically re-zeroing or switching
                                    this accumulator to double            */
    bool initialized;
} AS5600_t;

HAL_StatusTypeDef AS5600_Init(AS5600_t *enc, I2C_HandleTypeDef *hi2c);

/** Checks the MD (magnet detected) status bit. HAL_ERROR if magnet missing/misaligned. */
HAL_StatusTypeDef AS5600_SelfTest(AS5600_t *enc);

/** Low-level: read the raw 12-bit angle register (0..4095), no unwrap. */
HAL_StatusTypeDef AS5600_ReadRawAngle(AS5600_t *enc, uint16_t *raw_out);

/**
 * Read + unwrap + accumulate. Call once per control cycle.
 * @return HAL_OK on success; on I2C failure, angle_rad_unwrapped is left
 *         unchanged (caller's fault counter should track repeated failures).
 */
HAL_StatusTypeDef AS5600_Update(AS5600_t *enc);

/** Reset the accumulated angle to 0 at the encoder's CURRENT physical
 *  position (calibration step -- call once at boot after the robot is
 *  settled in its calibration pose). */
HAL_StatusTypeDef AS5600_Zero(AS5600_t *enc);

/** angle_rad_unwrapped * WHEEL_RADIUS_M */
float AS5600_GetWheelPositionM(const AS5600_t *enc);

#endif /* AS5600_H */
