/**
 * @file    matrix4.h
 * @brief   Minimal fixed-size float matrix/vector helpers for the 4-state,
 *          2-output, 1-input LQG loop. Deliberately NOT a general matrix
 *          library (no CMSIS-DSP dependency, no dynamic sizing) -- on an
 *          STM32F103 (Cortex-M3, no hardware FPU) every operation here is
 *          software-emulated float, so keeping it small and loop-unrolled
 *          keeps worst-case ISR time predictable and tiny relative to the
 *          5 ms control period.
 *
 * Pure functions, no HAL dependency -- safe to unit-test on a PC (see
 * tools/test_matrix4.c).
 */
#ifndef MATRIX4_H
#define MATRIX4_H

#include <stdint.h>

#define N4 4  /* number of states  */
#define N2 2  /* number of outputs */

/** y[N4] = A[N4][N4] * x[N4] */
void mat4_mulvec(const float A[N4][N4], const float x[N4], float y[N4]);

/** y[N4] = x[N4] + b[N4] (elementwise) */
void mat4_addvec(const float x[N4], const float b[N4], float y[N4]);

/** y[N4] = a[N4] - b[N4] (elementwise) */
void mat4_subvec(const float a[N4], const float b[N4], float y[N4]);

/** y[N2] = a[N2] - b[N2] (elementwise) -- for the innovation y - Cd*xhat */
void mat2_subvec(const float a[N2], const float b[N2], float y[N2]);

/** y[N4] = B[N4] * u (scalar input, single control channel) */
void mat4_scale_Bvec(const float B[N4], float u, float y[N4]);

/** y[N2] = C[N2][N4] * x[N4] */
void mat4_mulvec_C(const float C[N2][N4], const float x[N4], float y[N2]);

/** y[N4] = L[N4][N2] * innov[N2] */
void mat4_mulvec_L(const float L[N4][N2], const float innov[N2], float y[N4]);

/** scalar = K[N4] (dot) x[N4] */
float mat4_dot_K(const float K[N4], const float x[N4]);

/** returns 1 if any element is NaN/Inf or |element| > limit, else 0 */
int mat4_state_is_invalid(const float x[N4], float abs_limit);

#endif /* MATRIX4_H */
