/**
 * @file    estimator.h
 * @brief   Fixed-gain discrete-time observer (steady-state Kalman gain,
 *          precomputed offline -- see doc "Buoc 7-9" and tools/lqg_design.py).
 *
 * This is NOT an EKF. The physical model was already linearized about
 * theta=0 (doc "Buoc 1"), so a linear time-invariant observer with a
 * CONSTANT gain Ld (solved once, offline, from the algebraic Riccati
 * equation) is the correct and sufficient tool here -- there is no
 * on-line Jacobian, no re-linearization, no covariance propagation at
 * run time. The STM32 only ever does: predict (Ad,Bd) + correct (Ld).
 * See README.md "EKF vs KF observer vs complementary filter" for why an
 * EKF would be the wrong tool to reach for on this particular loop.
 *
 * Pure functions, no HAL dependency -- safe to unit-test on a PC.
 */
#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "matrix4.h"

typedef struct {
    float xhat[N4];        /* [x, xdot, theta, thetadot] estimate      */
    uint32_t invalid_count; /* consecutive NaN/blow-up guard trips      */
} Estimator_t;

/** Reset the estimator state (call at the end of calibration). */
void Estimator_Init(Estimator_t *est, const float xhat0[N4]);

/**
 * One discrete step of predict+correct:
 *   xhat = Ad*xhat + Bd*u_prev_realized + Ld*(y - Cd*xhat)
 *
 * IMPORTANT (anti-windup for the observer): pass the ACTUAL realized
 * control effort (after all saturation in control.c), not the raw
 * unsaturated LQR output. If the predictor is told "we applied 5N" while
 * the actuator only ever delivered 3N (clamped), the estimate silently
 * diverges from reality every single cycle the actuator saturates. This
 * is the "consistent control feedback" pattern -- the estimator-side
 * counterpart to classic actuator anti-windup.
 *
 * @param est            estimator state (in/out)
 * @param y               latest measurement [x_enc_m, theta_imu_rad]
 * @param u_prev_realized  N applied last cycle, AFTER saturation/allocation
 *                          feedback (see control.c U_ForEstimator())
 * @return 0 on success, -1 if the resulting state failed the NaN/blow-up
 *         guard (est->xhat is left at its last good value in that case).
 */
int Estimator_Step(Estimator_t *est, const float y[N2], float u_prev_realized);

#endif /* ESTIMATOR_H */
