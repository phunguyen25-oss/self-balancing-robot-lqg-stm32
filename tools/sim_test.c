/**
 * @file sim_test.c
 * @brief Closed-loop sanity check for the C firmware's algorithmic core
 *        (matrix4.c + estimator.c + control.c), compiled and run on a PC.
 *        Not part of the STM32 build -- this is a verification harness only.
 *
 * Replicates the doc's own sanity check (doc "Kiem tra nhanh"): drop from
 * theta0 = 0.15 rad, expect theta to settle back to ~0 within 1-2s, with
 * a small transient dip and a few cm of x excursion.
 *
 * The plant is simulated with the SAME Ad/Bd used by the estimator (exact
 * discrete model) -- this verifies the C code's control/estimation logic
 * and closed-loop stability, not sensor/actuator-model mismatch (that
 * can only be checked on real hardware).
 *
 * Build:
 *   gcc -std=c11 -Wall -Wextra -Werror -I../Core/Inc \
 *       sim_test.c ../Core/Src/matrix4.c ../Core/Src/estimator.c \
 *       ../Core/Src/control.c -lm -o sim_test
 *   ./sim_test
 */
#include <stdio.h>
#include <math.h>
#include "config.h"
#include "matrix4.h"
#include "estimator.h"
#include "control.h"

static void plant_step(float x[N4], float u)
{
    float x_new[N4], Bu[N4];
    mat4_mulvec(AD, x, x_new);
    mat4_scale_Bvec(BD, u, Bu);
    mat4_addvec(x_new, Bu, x_new);
    for (int i = 0; i < N4; i++) x[i] = x_new[i];
}

int main(void)
{
    float x_true[N4] = {0.0f, 0.0f, 0.15f, 0.0f}; /* doc's drop test: theta0=0.15 rad */

    Estimator_t est;
    float xhat0[N4] = {0.0f, 0.0f, 0.0f, 0.0f}; /* estimator "thinks" it starts upright --
                                                     deliberately mismatched vs x_true to also
                                                     exercise observer convergence, not just
                                                     controller convergence */
    Estimator_Init(&est, xhat0);

    Control_t ctrl;
    Control_Init(&ctrl);

    float u_prev = 0.0f; /* force applied during the interval that produced the first y */

    const int N_STEPS = (int)(3.0f / CONTROL_PERIOD_S); /* 3 seconds */
    printf("t(s)      x_true    theta_true   xhat_x   xhat_theta   u_cb(N)   v_L(m/s)  v_R(m/s)\n");

    int settled_step = -1;
    for (int k = 0; k < N_STEPS; k++) {
        /* --- plant evolves under the force applied last cycle --- */
        plant_step(x_true, u_prev);

        /* --- sensors measure the new true state (no noise: verifies the
         *     deterministic logic; add noise here to stress-test further) --- */
        float y[N2];
        mat4_mulvec_C(CD, x_true, y);

        /* --- estimate + control, exactly the ISR's per-cycle order --- */
        int est_ok = Estimator_Step(&est, y, u_prev);
        if (est_ok != 0) {
            printf("ESTIMATOR REJECTED STATE at step %d -- FAIL\n", k);
            return 1;
        }

        float v_L, v_R;
        Control_Step(&ctrl, est.xhat, 0.0f /* yaw_error, hold straight */,
                     1.0f /* soft_start_gain, full authority */, CONTROL_PERIOD_S,
                     &v_L, &v_R);
        u_prev = ctrl.u_cb_realized;

        if (k % 20 == 0) { /* print every 100ms */
            printf("%6.3f  %8.4f  %10.4f  %8.4f  %10.4f  %8.3f  %8.3f  %8.3f\n",
                   k * CONTROL_PERIOD_S, x_true[0], x_true[2], est.xhat[0], est.xhat[2],
                   ctrl.u_cb_realized, v_L, v_R);
        }

        /* "settled" = first step after which theta NEVER again exceeds the
         * band for the rest of the run (avoids falsely flagging a
         * transient zero-crossing on the way to a bigger overshoot) */
        if (fabsf(x_true[2]) >= 0.01f) {
            settled_step = -1; /* reset: still not settled for good */
        } else if (settled_step < 0) {
            settled_step = k;
        }
    }

    printf("\nfinal theta_true = %.5f rad, final x_true = %.5f m\n", x_true[2], x_true[0]);
    if (fabsf(x_true[2]) > 0.01f) {
        printf("FAIL: theta did not settle near 0 within 3s\n");
        return 1;
    }
    if (settled_step >= 0) {
        printf("PASS: theta settled under 0.01 rad at t=%.3fs (doc expects ~1-2s)\n",
               settled_step * CONTROL_PERIOD_S);
    }
    return 0;
}
