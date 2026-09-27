/**
 * @file main.c
 * @brief Boot flow + 200Hz control ISR wiring for the 2-wheel LQG
 *        self-balancing robot (STM32F103C8T6 "Blue Pill").
 *
 * This file assumes a CubeMX project generated for STM32F103C8T6 with the
 * peripherals listed in config.h section 14 (I2C1, I2C2, TIM2, TIM3, TIM4,
 * IWDG, the GPIO pins in the pin-macro block). The MX_*_Init() functions
 * below are written at "this is exactly what those peripherals need"
 * level of detail so you can diff them against whatever CubeMX generates
 * for your exact project settings -- if CubeMX's clock config differs
 * from the 72MHz assumption baked into STEP_TIMER_PSC (config.h section
 * 14), update that constant to match.
 *
 * Boot flow: HAL/clock/peripheral init -> sensor init -> BLOCKING
 * calibration (robot must be held still+upright) -> enable steppers with
 * soft-start -> start the 5ms TIM2 interrupt, which drives everything
 * from then on via HAL_TIM_PeriodElapsedCallback() below. main()'s while(1)
 * only does non-time-critical housekeeping: watchdog kick, fault LED.
 */
#include "stm32f1xx_hal.h"
#include "config.h"
#include "matrix4.h"
#include "mpu6050.h"
#include "as5600.h"
#include "stepper_drv8825.h"
#include "estimator.h"
#include "control.h"
#include "safety.h"
#include "calibration.h"
#include <stdbool.h>
#include <stddef.h>

/* ============================ peripheral handles ======================= */
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
IWDG_HandleTypeDef hiwdg;

/* ============================ application state ========================= */
static MPU6050_t g_imu;
static ComplementaryFilter_t g_comp_filter;
static float g_accel_theta_offset = 0.0f;
static AS5600_t g_enc_l, g_enc_r;
static Stepper_t g_stepper_l, g_stepper_r;
static Estimator_t g_est;
static Control_t g_ctrl;
static Safety_t g_safety;

static volatile uint8_t g_isr_busy_flag = 0;
static volatile bool g_isr_overrun_flag = false;
static float g_soft_start_elapsed_s = 0.0f;

/* ============================ prototypes ================================ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_IWDG_Init(void);
static void ControlLoop_Tick(void);
static void EnterFault(const char *reason);

/* ============================ main ======================================= */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_IWDG_Init();

    Safety_Init(&g_safety);

    Stepper_Init(&g_stepper_l, &STEP_L_TIM, STEP_L_CHANNEL,
                 DIR_L_GPIO_Port, DIR_L_Pin, EN_L_GPIO_Port, EN_L_Pin, STEP_L_DIR_INVERT);
    Stepper_Init(&g_stepper_r, &STEP_R_TIM, STEP_R_CHANNEL,
                 DIR_R_GPIO_Port, DIR_R_Pin, EN_R_GPIO_Port, EN_R_Pin, STEP_R_DIR_INVERT);

    if (MPU6050_Init(&hi2c1) != HAL_OK) {
        EnterFault("MPU6050 init failed (I2C1)");
    }
    if (!Safety_IsFaulted(&g_safety) && AS5600_Init(&g_enc_l, &hi2c1) != HAL_OK) {
        EnterFault("left encoder init failed (I2C1)");
    }
    if (!Safety_IsFaulted(&g_safety) && AS5600_Init(&g_enc_r, &hi2c2) != HAL_OK) {
        EnterFault("right encoder init failed (I2C2)");
    }

    if (!Safety_IsFaulted(&g_safety)) {
        g_safety.state = ROBOT_STATE_CALIBRATING;

        CalibrationResult_t cal = Calibration_Run(&hi2c1, &hi2c1, &hi2c2, &g_imu, &g_enc_l, &g_enc_r);
        if (!cal.ok) {
            EnterFault(cal.fail_reason);
        } else {
            g_accel_theta_offset = cal.accel_theta_offset_rad;
            ComplementaryFilter_Init(&g_comp_filter, 0.0f);

            float xhat0[N4] = {0.0f, 0.0f, 0.0f, 0.0f};
            Estimator_Init(&g_est, xhat0);
            Control_Init(&g_ctrl);

            Stepper_Enable(&g_stepper_l);
            Stepper_Enable(&g_stepper_r);

            g_soft_start_elapsed_s = 0.0f;
            g_safety.state = ROBOT_STATE_SOFT_START;

            HAL_TIM_Base_Start_IT(&htim2); /* control ISR now fires every CONTROL_PERIOD_MS */
        }
    }

    while (1) {
        HAL_IWDG_Refresh(&hiwdg); /* kicked only in this non-critical loop -- a genuine
                                      lockup elsewhere stops refreshes and the hardware
                                      watchdog resets the MCU as a last-resort safety net */

        if (Safety_IsFaulted(&g_safety)) {
            /* Belt-and-suspenders: the ISR already zeroes control output
             * and this loop repeatedly forces a hard actuator disable
             * regardless of which code path triggered the fault. */
            Stepper_Disable(&g_stepper_l);
            Stepper_Disable(&g_stepper_r);
            HAL_GPIO_TogglePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin);
            HAL_Delay(100);
        } else {
            HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
        }

        if (g_isr_overrun_flag) {
            /* diagnostic hook -- wire to a log/telemetry channel if you add one */
            g_isr_overrun_flag = false;
        }
    }
}

static void EnterFault(const char *reason)
{
    Stepper_Disable(&g_stepper_l);
    Stepper_Disable(&g_stepper_r);
    Safety_TriggerFault(&g_safety, reason);
}

/* ============================ 200Hz control ISR ========================== */

/**
 * HAL weak-function override -- do NOT hand-edit stm32f1xx_it.c's
 * TIM2_IRQHandler; HAL_TIM_IRQHandler() (called from there, generated by
 * CubeMX) invokes this callback for you once TIM2's update interrupt and
 * NVIC line are enabled.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    if (g_isr_busy_flag) {
        /* Previous cycle's ControlLoop_Tick() had not returned when this
         * one fired -- the ISR body took longer than CONTROL_PERIOD_MS.
         * Skip this tick rather than re-entering/stacking control cycles. */
        Safety_ReportLoopTiming(&g_safety, true);
        g_isr_overrun_flag = true;
        return;
    }

    g_isr_busy_flag = 1;
    ControlLoop_Tick();
    g_isr_busy_flag = 0;
    Safety_ReportLoopTiming(&g_safety, false);
}

static void ControlLoop_Tick(void)
{
    if (Safety_IsFaulted(&g_safety)) {
        Control_Zero(&g_ctrl);
        return;
    }

    /* ---- 1) sensors ---- */
    MPU6050_Raw_t raw;
    HAL_StatusTypeDef st_imu = MPU6050_ReadRaw(&hi2c1, &raw);
    Safety_ReportI2CResult(&g_safety, SENSOR_IMU, st_imu == HAL_OK);

    HAL_StatusTypeDef st_l = AS5600_Update(&g_enc_l);
    Safety_ReportI2CResult(&g_safety, SENSOR_ENCODER_L, st_l == HAL_OK);

    HAL_StatusTypeDef st_r = AS5600_Update(&g_enc_r);
    Safety_ReportI2CResult(&g_safety, SENSOR_ENCODER_R, st_r == HAL_OK);

    if (Safety_IsFaulted(&g_safety)) { /* one of the I2C fault counters just tripped */
        Control_Zero(&g_ctrl);
        return;
    }

    /* attitude fusion (separate from the LQG state observer -- see
     * mpu6050.h "why complementary filter, not EKF") */
    float theta_imu = g_comp_filter.theta; /* hold last value if this cycle's read failed */
    if (st_imu == HAL_OK) {
        float theta_dot_raw = raw.gyro_radps[1] - g_imu.gyro_bias_radps[1]; /* Y axis = pitch,
                                                                                verify on hw */
        float theta_acc = MPU6050_ComputeAccelTiltRad(raw.accel_g[0], raw.accel_g[2])
                           - g_accel_theta_offset;
        theta_imu = ComplementaryFilter_Update(&g_comp_filter, theta_dot_raw, theta_acc,
                                                CONTROL_PERIOD_S);
    }

    float x_l = AS5600_GetWheelPositionM(&g_enc_l);
    float x_r = AS5600_GetWheelPositionM(&g_enc_r);
    float y[N2] = { 0.5f * (x_l + x_r), theta_imu };

    /* ---- 2) estimate (fixed-gain discrete observer, doc Buoc 10) ---- */
    if (Estimator_Step(&g_est, y, g_ctrl.u_cb_realized) != 0) {
        EnterFault("estimator NaN/blow-up guard tripped");
        Control_Zero(&g_ctrl);
        return;
    }

    /* ---- 3) fall/fault check on the estimate, before acting on it ---- */
    if (!Safety_CheckTilt(&g_safety, g_est.xhat[2], g_est.xhat[3])) {
        Control_Zero(&g_ctrl);
        return;
    }

    /* ---- 4) soft-start authority ramp ---- */
    float soft_start_gain = 1.0f;
    if (g_safety.state == ROBOT_STATE_SOFT_START) {
        g_soft_start_elapsed_s += CONTROL_PERIOD_S;
        soft_start_gain = g_soft_start_elapsed_s / SOFT_START_RAMP_TIME_S;
        if (soft_start_gain >= 1.0f) {
            soft_start_gain = 1.0f;
            g_safety.state = ROBOT_STATE_BALANCING;
        }
    }

    /* ---- 5) heading hold: differential-drive approx from wheel encoders,
     *     target = drive straight (0). Fully separate from the LQG loop,
     *     which only ever sees the AVERAGED x_enc above (doc "Control
     *     allocation" note). Sign/scale must be validated on real hw:
     *     nose-turn the robot by hand, confirm this drives v_L/v_R apart
     *     in the direction that opposes the turn. ---- */
    float yaw_error = (x_l - x_r) / TRACK_WIDTH_M;

    /* ---- 6) LQR + control allocation + saturation/anti-windup ---- */
    float v_L, v_R;
    Control_Step(&g_ctrl, g_est.xhat, yaw_error, soft_start_gain, CONTROL_PERIOD_S, &v_L, &v_R);

    /* ---- 7) actuate ---- */
    Stepper_SetVelocity(&g_stepper_l, v_L);
    Stepper_SetVelocity(&g_stepper_r, v_R);
}

/* ============================ peripheral init ============================ */
/* Reference-level init matching config.h's pin/peripheral map. Compare
 * against your own CubeMX-generated code for your exact project -- if you
 * changed the clock tree, update STEP_TIMER_PSC in config.h to match. */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9; /* 8MHz HSE * 9 = 72MHz */
    HAL_RCC_OscConfig(&osc);

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1
                    | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;   /* HCLK = 72MHz  */
    clk.APB1CLKDivider = RCC_HCLK_DIV2;    /* PCLK1 = 36MHz -> TIM2-7 clock = 72MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV1;    /* PCLK2 = 72MHz */
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* DIR / EN outputs, push-pull */
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    gi.Pin = DIR_L_Pin;      HAL_GPIO_Init(DIR_L_GPIO_Port, &gi);
    gi.Pin = DIR_R_Pin;      HAL_GPIO_Init(DIR_R_GPIO_Port, &gi);
    gi.Pin = EN_L_Pin;       HAL_GPIO_Init(EN_L_GPIO_Port, &gi);
    gi.Pin = EN_R_Pin;       HAL_GPIO_Init(EN_R_GPIO_Port, &gi);
    gi.Pin = STATUS_LED_Pin; HAL_GPIO_Init(STATUS_LED_GPIO_Port, &gi);
    gi.Pin = FAULT_LED_Pin;  HAL_GPIO_Init(FAULT_LED_GPIO_Port, &gi);

    /* start both drivers disabled (active-low EN, so idle HIGH) */
    HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_SET);

    /* mode button, input pull-up (reserved, not read by this core-only build) */
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLUP;
    gi.Pin = MODE_BTN_Pin;
    HAL_GPIO_Init(MODE_BTN_GPIO_Port, &gi);
}

static void MX_I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gi = {0};
    gi.Mode = GPIO_MODE_AF_OD; /* I2C pins: alternate function open-drain */
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    gi.Pin = GPIO_PIN_6 | GPIO_PIN_7; /* PB6=SCL, PB7=SDA */
    HAL_GPIO_Init(GPIOB, &gi);

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000; /* 400kHz fast mode -- MPU6050 + AS5600 both support it */
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static void MX_I2C2_Init(void)
{
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gi = {0};
    gi.Mode = GPIO_MODE_AF_OD;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    gi.Pin = GPIO_PIN_10 | GPIO_PIN_11; /* PB10=SCL, PB11=SDA */
    HAL_GPIO_Init(GPIOB, &gi);

    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 400000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c2);
}

static void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 input clock = 72MHz (see SystemClock_Config: APB1=36MHz, and
     * APB1 prescaler != 1 means TIMxCLK = 2*APB1). PSC=71 -> 1MHz tick,
     * ARR=(1MHz*CONTROL_PERIOD_S)-1 -> update interrupt every 5ms. */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = (uint32_t)(STEP_TIMER_TICK_HZ * CONTROL_PERIOD_S) - 1U; /* 4999 */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0); /* high priority: this IS the control loop */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

static void MX_TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gi = {0};
    gi.Mode = GPIO_MODE_AF_PP;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    gi.Pin = GPIO_PIN_6; /* TIM3_CH1 = PA6 = STEP_L */
    HAL_GPIO_Init(GPIOA, &gi);

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = STEP_TIMER_PSC;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999; /* placeholder ARR -- overwritten every cycle by
                                 Stepper_SetVelocity(); Stepper_Enable() starts the
                                 channel, this value is never actually output as-is */
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1);
}

static void MX_TIM4_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gi = {0};
    gi.Mode = GPIO_MODE_AF_PP;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    gi.Pin = GPIO_PIN_8; /* TIM4_CH3 = PB8 = STEP_R (CH3, not CH1, to avoid I2C1 on PB6) */
    HAL_GPIO_Init(GPIOB, &gi);

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = STEP_TIMER_PSC;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 999; /* placeholder, see MX_TIM3_Init note */
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim4);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim4, &oc, TIM_CHANNEL_3);
}

static void MX_IWDG_Init(void)
{
    /* LSI ~40kHz, prescaler 64 -> 625Hz, reload 625 -> ~1s timeout.
     * Comfortably longer than CONTROL_PERIOD_MS so a single missed
     * refresh from a slow main-loop iteration never false-trips it, but
     * short enough that a true lockup recovers quickly. */
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 625;
    HAL_IWDG_Init(&hiwdg);
}

void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* HAL init itself failed (before Safety_t even exists) -- fall
         * back to bare register toggling so there is still SOME visible
         * fault indication rather than a silent hang. */
    }
}
