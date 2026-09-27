/**
 * @file control.c
 * @see  control.h
 */
#include "control.h"
#include "config.h"

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void Control_Init(Control_t *ctrl)
{
    ctrl->v_cb = 0.0f;
    ctrl->yaw_integral = 0.0f;
    ctrl->u_cb_realized = 0.0f;
}

void Control_Zero(Control_t *ctrl)
{
    Control_Init(ctrl);
}

void Control_Step(Control_t *ctrl, const float xhat[N4], float yaw_error_rad,
                   float soft_start_gain, float dt,
                   float *v_L_out, float *v_R_out)
{
    soft_start_gain = clampf(soft_start_gain, 0.0f, 1.0f);

    /* ---- 1-2) LQR balance law, soft-started, saturated ----
     * u_cb is the value that MUST be fed back to Estimator_Step() as
     * u_prev_realized -- Bd was derived against this force input directly
     * (see doc Buoc 2, u enters the x-double-dot equation as a force). */
    float u_cb_raw = -mat4_dot_K(KD, xhat) * soft_start_gain;
    float u_cb = clampf(u_cb_raw, -U_MAX_N, U_MAX_N);
    ctrl->u_cb_realized = u_cb;

    /* ---- 3) force -> wheel acceleration ----
     * DRV8825 in open-loop STEP/DIR is a velocity actuator, not a torque
     * actuator, so u [N] cannot be sent to the driver directly. Convert
     * to an acceleration command first (see config.h U_TO_ACCEL_GAIN). */
    float a_cb = clampf(u_cb * U_TO_ACCEL_GAIN, -WHEEL_ACCEL_MAX_MPS2, WHEEL_ACCEL_MAX_MPS2);

    /* ---- 4) integrate to velocity, with slew limit + anti-windup ----
     * Integrating acceleration is itself a natural ramp; the slew-rate
     * clamp below is an independent, second layer on top of it (defends
     * against an oversized dt, e.g. after a delayed ISR, still producing
     * too large a single-step jump). */
    float dv = a_cb * dt;
    float dv_limit = VELOCITY_SLEW_MAX_MPS_PER_S * dt;
    dv = clampf(dv, -dv_limit, dv_limit);

    /* Anti-windup note: unlike the yaw PI term below, v_cb has no hidden
     * accumulator separate from its own (clamped) value -- what's stored
     * back into ctrl->v_cb every cycle IS the saturated command. So simply
     * clamping the state itself, every cycle, is already complete windup
     * protection here: next cycle always resumes from the rail, never
     * from an unbounded value the actuator could never have reached. */
    ctrl->v_cb = clampf(ctrl->v_cb + dv, -WHEEL_SPEED_MAX_MPS, WHEEL_SPEED_MAX_MPS);

    /* ---- 5) heading-hold PI loop, fully separate from balance ----
     * Lives entirely in the velocity domain: its output is a wheel-speed
     * differential, not a force, since the actuator interface below it
     * is velocity-based anyway (avoids splitting U_TO_ACCEL_GAIN's
     * implicit "effective mass" across two independently-saturating
     * wheels, which would be physically ambiguous). */
    float yaw_p = YAW_KP * yaw_error_rad;
    float yaw_i_unclamped = ctrl->yaw_integral + YAW_KI * yaw_error_rad * dt;
    float yaw_i_sat = clampf(yaw_i_unclamped, -YAW_INTEGRAL_MAX, YAW_INTEGRAL_MAX);
    /* clamped-integrator anti-windup on the yaw PI term itself */
    ctrl->yaw_integral = yaw_i_sat;

    float v_yaw_raw = (yaw_p + ctrl->yaw_integral) * soft_start_gain;
    float v_yaw = clampf(v_yaw_raw, -YAW_SPEED_MAX_MPS, YAW_SPEED_MAX_MPS);

    /* ---- 6-7) control allocation + final per-wheel saturation ----
     * (doc "Control allocation (thieu)": tau_L = u_cb/2 - u_yaw/2,
     * tau_R = u_cb/2 + u_yaw/2 -- re-derived here in velocity domain). */
    float v_L = clampf(ctrl->v_cb - 0.5f * v_yaw, -WHEEL_SPEED_MAX_MPS, WHEEL_SPEED_MAX_MPS);
    float v_R = clampf(ctrl->v_cb + 0.5f * v_yaw, -WHEEL_SPEED_MAX_MPS, WHEEL_SPEED_MAX_MPS);

    *v_L_out = v_L;
    *v_R_out = v_R;
}
