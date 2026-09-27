/**
 * @file    stepper_drv8825.h
 * @brief   DRV8825 STEP/DIR driver: velocity command -> timer PWM step
 *          frequency, direction GPIO, enable/disable, and the LAST-STAGE
 *          hard actuator-limit clamp (independent of control.c's earlier
 *          velocity saturation -- defense in depth: if a future change to
 *          control.c ever produces an out-of-range command, this layer
 *          still can't exceed what the motor/driver can physically do).
 *
 * DRV8825 has no current/torque feedback in this BOM (open-loop STEP/DIR),
 * so this driver is a pure velocity source. Torque comes from correctly
 * setting the DRV8825's current-limit trimpot (Vref) for your motor --
 * that's a hardware calibration step, see README, not something firmware
 * can do.
 */
#ifndef STEPPER_DRV8825_H
#define STEPPER_DRV8825_H

#include <stdbool.h>
#include "stm32f1xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;         /* e.g. TIM_CHANNEL_1 */
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef *en_port;
    uint16_t en_pin;
    bool dir_invert;
    bool enabled;
} Stepper_t;

void Stepper_Init(Stepper_t *s, TIM_HandleTypeDef *htim, uint32_t channel,
                   GPIO_TypeDef *dir_port, uint16_t dir_pin,
                   GPIO_TypeDef *en_port, uint16_t en_pin, bool dir_invert);

/** Drives EN low (DRV8825 ENABLE is active-low) and starts the PWM channel. */
void Stepper_Enable(Stepper_t *s);

/** Immediate hard stop: PWM channel stopped AND EN driven high. Use for
 *  faults. For a graceful stop, ramp the commanded velocity to 0 first
 *  (in control.c / calibration.c) and only call Disable() once it's near 0. */
void Stepper_Disable(Stepper_t *s);

/**
 * Convert a signed velocity command (m/s) to step frequency + direction
 * and apply it to the timer. No-op (steps held at 0) if the stepper is
 * currently disabled.
 * @param velocity_mps  already saturated/ramped by control.c; this
 *                        function applies its OWN independent clamp to
 *                        [MIN_STEP_FREQ_HZ, MAX_STEP_FREQ_HZ] as a final
 *                        actuator-capability safety net.
 */
void Stepper_SetVelocity(Stepper_t *s, float velocity_mps);

#endif /* STEPPER_DRV8825_H */
