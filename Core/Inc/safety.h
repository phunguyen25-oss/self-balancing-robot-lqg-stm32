/**
 * @file    safety.h
 * @brief   Fault detection and top-level robot state: fall-angle cutoff,
 *          implausible-tilt-rate check, per-sensor I2C fault counters,
 *          control-loop overrun detection, watchdog wrapper.
 *
 * Design principle: ANY of these tripping is a one-way door into
 * ROBOT_STATE_FAULT (drivers disabled) until an explicit Safety_Reset()
 * from a fresh recalibration -- no automatic silent recovery mid-fault,
 * since "it fixed itself" is exactly the kind of thing you don't want to
 * trust on a robot about to fall over.
 */
#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ROBOT_STATE_BOOT = 0,
    ROBOT_STATE_CALIBRATING,
    ROBOT_STATE_SOFT_START,
    ROBOT_STATE_BALANCING,
    ROBOT_STATE_FAULT
} RobotState_t;

typedef enum {
    SENSOR_IMU = 0,
    SENSOR_ENCODER_L,
    SENSOR_ENCODER_R,
    SENSOR_COUNT
} SensorId_t;

typedef struct {
    RobotState_t state;
    const char *fault_reason;      /* static string literal, for a debugger/log, not owned */
    uint32_t i2c_fail_count[SENSOR_COUNT];
    uint32_t loop_overrun_count;
} Safety_t;

void Safety_Init(Safety_t *sf);

/** Call once per control cycle with the current estimate. Transitions to
 *  ROBOT_STATE_FAULT (and returns false) if outside safe bounds. */
bool Safety_CheckTilt(Safety_t *sf, float theta_rad, float theta_dot_rad_s);

/** Call after every I2C sensor transaction with its HAL result (HAL_OK==0).
 *  Resets that sensor's counter on success, increments on failure, and
 *  trips FAULT once I2C_FAULT_LIMIT consecutive failures are reached. */
void Safety_ReportI2CResult(Safety_t *sf, SensorId_t sensor, int hal_status_ok);

/** Call with the measured duration (in control-cycle "ticks over budget",
 *  e.g. 1 if this cycle ran long, 0 if it finished on time). Trips FAULT
 *  after LOOP_OVERRUN_LIMIT consecutive overruns. */
void Safety_ReportLoopTiming(Safety_t *sf, bool overran_this_cycle);

/** Explicitly force a fault (e.g. calibration failed a sanity check). */
void Safety_TriggerFault(Safety_t *sf, const char *reason);

/** Clear all fault counters and return to ROBOT_STATE_BOOT -- only call
 *  this at the start of a fresh, explicit recalibration attempt. */
void Safety_Reset(Safety_t *sf);

static inline bool Safety_IsFaulted(const Safety_t *sf) { return sf->state == ROBOT_STATE_FAULT; }

#endif /* SAFETY_H */
