/**
 * @file safety.c
 * @see  safety.h
 */
#include "safety.h"
#include "config.h"
#include <math.h>
#include <stddef.h>

void Safety_Init(Safety_t *sf)
{
    sf->state = ROBOT_STATE_BOOT;
    sf->fault_reason = NULL;
    for (int i = 0; i < SENSOR_COUNT; i++) sf->i2c_fail_count[i] = 0;
    sf->loop_overrun_count = 0;
}

void Safety_TriggerFault(Safety_t *sf, const char *reason)
{
    sf->state = ROBOT_STATE_FAULT;
    sf->fault_reason = reason;
}

void Safety_Reset(Safety_t *sf)
{
    Safety_Init(sf);
}

bool Safety_CheckTilt(Safety_t *sf, float theta_rad, float theta_dot_rad_s)
{
    if (isnan(theta_rad) || isinf(theta_rad) || isnan(theta_dot_rad_s) || isinf(theta_dot_rad_s)) {
        Safety_TriggerFault(sf, "tilt estimate NaN/Inf");
        return false;
    }
    if (fabsf(theta_rad) > THETA_FAULT_RAD) {
        Safety_TriggerFault(sf, "tilt angle exceeded THETA_FAULT_RAD -- robot has fallen");
        return false;
    }
    if (fabsf(theta_dot_rad_s) > THETA_DOT_FAULT_RADPS) {
        Safety_TriggerFault(sf, "tilt rate exceeded THETA_DOT_FAULT_RADPS -- sensor glitch or impact");
        return false;
    }
    return true;
}

void Safety_ReportI2CResult(Safety_t *sf, SensorId_t sensor, int hal_status_ok)
{
    if (sensor >= SENSOR_COUNT) return; /* SensorId_t is unsigned-backed here, no need for <0 */

    if (hal_status_ok) {
        sf->i2c_fail_count[sensor] = 0;
        return;
    }

    sf->i2c_fail_count[sensor]++;
    if (sf->i2c_fail_count[sensor] >= I2C_FAULT_LIMIT) {
        /* Losing EITHER sensor is safety-critical here, not just degraded
         * accuracy: the doc's own observability analysis (Buoc 4) shows
         * theta-only is NOT observable (rank 3/4) -- lose the encoder and
         * the estimator is running open-loop on x/xdot with no way to
         * correct drift; lose the IMU and there is no tilt measurement
         * at all. Either one is a hard stop, not a "run in degraded mode". */
        static const char *reasons[SENSOR_COUNT] = {
            "IMU I2C failures exceeded I2C_FAULT_LIMIT",
            "left encoder I2C failures exceeded I2C_FAULT_LIMIT",
            "right encoder I2C failures exceeded I2C_FAULT_LIMIT",
        };
        Safety_TriggerFault(sf, reasons[sensor]);
    }
}

void Safety_ReportLoopTiming(Safety_t *sf, bool overran_this_cycle)
{
    if (overran_this_cycle) {
        sf->loop_overrun_count++;
        if (sf->loop_overrun_count >= LOOP_OVERRUN_LIMIT) {
            Safety_TriggerFault(sf, "control ISR missed its deadline LOOP_OVERRUN_LIMIT times in a row");
        }
    } else {
        sf->loop_overrun_count = 0;
    }
}
