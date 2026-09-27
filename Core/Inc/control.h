/**
 * @file    control.h
 * @brief   LQR balance law + heading-hold loop + control allocation +
 *          saturation + anti-windup + ramp limiting.
 *
 * Pipeline each 5 ms (see control.c for the full derivation of each step):
 *
 *   1. u_cb_raw   = -Kd . xhat                          (doc Buoc 10, u = -Kd*xhat)
 *   2. u_cb       = clamp(soft_start * u_cb_raw, +-U_MAX_N)
 *                     <-- this exact value is what must be fed back into
 *                         Estimator_Step() as u_prev_realized (see estimator.h)
 *   3. a_cb       = clamp(u_cb * U_TO_ACCEL_GAIN, +-WHEEL_ACCEL_MAX_MPS2)
 *   4. v_cb      += clamp(a_cb*dt, +-VELOCITY_SLEW_MAX_MPS_PER_S*dt)   [with
 *                     conditional-integration anti-windup at WHEEL_SPEED_MAX_MPS]
 *   5. v_yaw      = PI(yaw_error) with clamped-integrator anti-windup,
 *                     saturated to +-YAW_SPEED_MAX_MPS, slew-limited
 *   6. v_L = v_cb - v_yaw/2 ; v_R = v_cb + v_yaw/2   (doc "Control allocation",
 *                     re-derived in velocity domain -- see control.c comment)
 *   7. v_L, v_R individually re-clamped to +-WHEEL_SPEED_MAX_MPS
 *
 * This is NOT in the reference doc (which explicitly flags control
 * allocation as missing, and stops at "u -> clamp -> steps" without
 * defining the force->velocity actuator interface). Both additions are
 * implemented here, clearly separated from the LQG core in estimator.c.
 *
 * Pure functions, no HAL dependency -- safe to unit-test on a PC.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include "matrix4.h"

typedef struct {
    float v_cb;          /* combined (straight-line) wheel velocity command, m/s */
    float yaw_integral;  /* heading-loop PI integral term                        */
    float u_cb_realized; /* last cycle's saturated balance force, N -- feed this
                             into Estimator_Step() as u_prev_realized            */
} Control_t;

void Control_Init(Control_t *ctrl);

/**
 * @param ctrl            controller state (in/out)
 * @param xhat            state estimate [x, xdot, theta, thetadot]
 * @param yaw_error_rad   desired_heading - actual_heading (0 if just holding
 *                          straight); caller derives this from encoder
 *                          differential or gyro-z, entirely outside the LQG loop
 * @param soft_start_gain  0..1, ramps control authority in after calibration
 * @param dt              control period, seconds (== CONTROL_PERIOD_S)
 * @param v_L_out         [out] left wheel velocity command, m/s, signed
 * @param v_R_out         [out] right wheel velocity command, m/s, signed
 */
void Control_Step(Control_t *ctrl, const float xhat[N4], float yaw_error_rad,
                   float soft_start_gain, float dt,
                   float *v_L_out, float *v_R_out);

/** Force all integrators to zero and hold outputs at zero (fault/idle entry). */
void Control_Zero(Control_t *ctrl);

#endif /* CONTROL_H */
