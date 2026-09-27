/**
 * @file matrix4.c
 * @see  matrix4.h
 */
#include "matrix4.h"
#include <math.h>

void mat4_mulvec(const float A[N4][N4], const float x[N4], float y[N4])
{
    for (int i = 0; i < N4; i++) {
        float acc = 0.0f;
        for (int j = 0; j < N4; j++) {
            acc += A[i][j] * x[j];
        }
        y[i] = acc;
    }
}

void mat4_addvec(const float x[N4], const float b[N4], float y[N4])
{
    for (int i = 0; i < N4; i++) {
        y[i] = x[i] + b[i];
    }
}

void mat4_subvec(const float a[N4], const float b[N4], float y[N4])
{
    for (int i = 0; i < N4; i++) {
        y[i] = a[i] - b[i];
    }
}

void mat2_subvec(const float a[N2], const float b[N2], float y[N2])
{
    for (int i = 0; i < N2; i++) {
        y[i] = a[i] - b[i];
    }
}

void mat4_scale_Bvec(const float B[N4], float u, float y[N4])
{
    for (int i = 0; i < N4; i++) {
        y[i] = B[i] * u;
    }
}

void mat4_mulvec_C(const float C[N2][N4], const float x[N4], float y[N2])
{
    for (int i = 0; i < N2; i++) {
        float acc = 0.0f;
        for (int j = 0; j < N4; j++) {
            acc += C[i][j] * x[j];
        }
        y[i] = acc;
    }
}

void mat4_mulvec_L(const float L[N4][N2], const float innov[N2], float y[N4])
{
    for (int i = 0; i < N4; i++) {
        float acc = 0.0f;
        for (int j = 0; j < N2; j++) {
            acc += L[i][j] * innov[j];
        }
        y[i] = acc;
    }
}

float mat4_dot_K(const float K[N4], const float x[N4])
{
    float acc = 0.0f;
    for (int i = 0; i < N4; i++) {
        acc += K[i] * x[i];
    }
    return acc;
}

int mat4_state_is_invalid(const float x[N4], float abs_limit)
{
    for (int i = 0; i < N4; i++) {
        if (isnan(x[i]) || isinf(x[i])) {
            return 1;
        }
        if (fabsf(x[i]) > abs_limit) {
            return 1;
        }
    }
    return 0;
}
