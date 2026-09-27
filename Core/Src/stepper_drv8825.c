/**
 * @file stepper_drv8825.c
 * @see  stepper_drv8825.h
 */
#include "stepper_drv8825.h"
#include "config.h"
#include <math.h>

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void Stepper_Init(Stepper_t *s, TIM_HandleTypeDef *htim, uint32_t channel,
                   GPIO_TypeDef *dir_port, uint16_t dir_pin,
                   GPIO_TypeDef *en_port, uint16_t en_pin, bool dir_invert)
{
    s->htim = htim;
    s->channel = channel;
    s->dir_port = dir_port;
    s->dir_pin = dir_pin;
    s->en_port = en_port;
    s->en_pin = en_pin;
    s->dir_invert = dir_invert;
    s->enabled = false;

    /* Fix the prescaler once; only ARR/CCR change per control cycle. */
    __HAL_TIM_SET_PRESCALER(s->htim, STEP_TIMER_PSC);

    /* Start disabled and silent -- CALLER must explicitly Enable() after
     * calibration completes, never on power-up by default (safety). */
    HAL_GPIO_WritePin(s->en_port, s->en_pin, GPIO_PIN_SET); /* active-low: SET = disabled */
}

void Stepper_Enable(Stepper_t *s)
{
    HAL_GPIO_WritePin(s->en_port, s->en_pin, GPIO_PIN_RESET); /* active-low: RESET = enabled */
    HAL_TIM_PWM_Start(s->htim, s->channel);
    s->enabled = true;
}

void Stepper_Disable(Stepper_t *s)
{
    HAL_TIM_PWM_Stop(s->htim, s->channel);
    HAL_GPIO_WritePin(s->en_port, s->en_pin, GPIO_PIN_SET); /* active-low: SET = disabled,
                                                                also lets the motor free-spin
                                                                (no holding torque) */
    s->enabled = false;
}

void Stepper_SetVelocity(Stepper_t *s, float velocity_mps)
{
    if (!s->enabled) {
        return; /* driver is off -- nothing to update */
    }

    bool forward = (velocity_mps >= 0.0f);
    bool dir_level_high = s->dir_invert ? !forward : forward;
    HAL_GPIO_WritePin(s->dir_port, s->dir_pin, dir_level_high ? GPIO_PIN_SET : GPIO_PIN_RESET);

    float speed_mps = fabsf(velocity_mps);
    float freq_hz = speed_mps * STEPS_PER_METER;

    /* final, independent actuator-capability clamp (defense in depth vs.
     * control.c's own WHEEL_SPEED_MAX_MPS saturation) */
    if (freq_hz < MIN_STEP_FREQ_HZ) {
        /* below reliable minimum -- just stop pulsing rather than risk an
         * unreliable/noisy near-DC PWM period */
        __HAL_TIM_SET_COMPARE(s->htim, s->channel, 0);
        return;
    }
    freq_hz = clampf(freq_hz, MIN_STEP_FREQ_HZ, MAX_STEP_FREQ_HZ);

    uint32_t arr = (uint32_t)((float)STEP_TIMER_TICK_HZ / freq_hz) - 1U;
    if (arr < 1U) arr = 1U;      /* guard against div-by-huge-freq underflow */
    if (arr > 0xFFFFU) arr = 0xFFFFU; /* 16-bit timer ceiling */

    __HAL_TIM_SET_AUTORELOAD(s->htim, arr);
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, arr / 2U); /* ~50% duty */
}
