/**
 * @file estimator.c
 * @see  estimator.h
 */
#include "estimator.h"
#include "config.h"
#include <string.h>

void Estimator_Init(Estimator_t *est, const float xhat0[N4])
{
    memcpy(est->xhat, xhat0, sizeof(est->xhat));
    est->invalid_count = 0;
}

int Estimator_Step(Estimator_t *est, const float y[N2], float u_prev_realized)
{
    float x_pred[N4];
    float Bu[N4];
    float y_pred[N2];
    float innov[N2];
    float correction[N4];
    float x_new[N4];

    /* --- predict: x_pred = Ad*xhat + Bd*u_prev_realized --- */
    mat4_mulvec(AD, est->xhat, x_pred);
    mat4_scale_Bvec(BD, u_prev_realized, Bu);
    mat4_addvec(x_pred, Bu, x_pred);

    /* --- correct: x_new = x_pred + Ld*(y - Cd*x_pred) --- */
    mat4_mulvec_C(CD, x_pred, y_pred);
    mat2_subvec(y, y_pred, innov);
    mat4_mulvec_L(LD, innov, correction);
    mat4_addvec(x_pred, correction, x_new);

    /* --- guard: reject a corrupted step rather than propagate NaN/Inf
     *     or an unphysical blow-up into the control law --- */
    if (mat4_state_is_invalid(x_new, ESTIMATOR_MAX_ABS_STATE)) {
        est->invalid_count++;
        return -1; /* est->xhat left unchanged */
    }

    est->invalid_count = 0;
    memcpy(est->xhat, x_new, sizeof(est->xhat));
    return 0;
}
