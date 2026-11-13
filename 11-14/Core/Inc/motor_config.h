#ifndef BARAM_MOTOR_CONFIG_H
#define BARAM_MOTOR_CONFIG_H

/* ==================== motor_slot_targets.h ==================== */
#ifndef MOTOR_SLOT_TARGETS_H
#define MOTOR_SLOT_TARGETS_H

#include <stdint.h>

/* 2: each B1 press starts ONE PAIR (P2 then P4), then waits.
 * 1: each B1 press starts ONE move (P2, P4, P2, P4, ...).
 * 0: automatically repeat the same targets after the arrival hold.
 * The first B1 after reset always requests nearest-home alignment.
 */
#ifndef MOTOR_SLOT_STEP_BY_BUTTON
#define MOTOR_SLOT_STEP_BY_BUTTON 0U
#endif
#if MOTOR_SLOT_STEP_BY_BUTTON != 0U && MOTOR_SLOT_STEP_BY_BUTTON != 1U && MOTOR_SLOT_STEP_BY_BUTTON != 2U
#error MOTOR_SLOT_STEP_BY_BUTTON must be 0, 1 or 2.
#endif

/* Automatic task speed rises after each completed individual motor move.
 * P2 starts at24deg/s, P4 follows at36deg/s, then P2 at48deg/s, capped below.
 * Initial alignment has a separate speed and retains its 15deg/s^2 ramp.
 */
#define MOTOR_SLOT_ALIGNMENT_SPEED_DPS 6.0f
#define MOTOR_SLOT_START_SPEED_DPS 24.0f
#define MOTOR_SLOT_MAX_SPEED_DPS 180.0f
#define MOTOR_SLOT_SPEED_INCREMENT_DPS 12.0f
#define MOTOR_SLOT_SPEEDUP_EVERY_MOVES 1U
#define MOTOR_SLOT_TASK_ACCEL_DPS2 300.0f
/* Task-only measured handoff; initial alignment keeps its 200ms confirmation.
 * Observe both encoders before handing over, with no extra timed pause. */
#define MOTOR_SLOT_TASK_HANDOFF_WINDOW_MS 10U
#define MOTOR_SLOT_TASK_HANDOFF_SPEED_DPS 5.0f
#define MOTOR_SLOT_TASK_HANDOFF_SPAN_DEG 0.5f
#if MOTOR_SLOT_TASK_HANDOFF_WINDOW_MS < 10U || MOTOR_SLOT_TASK_HANDOFF_WINDOW_MS > 1000U
#error Task handoff observation must be between 10 and 1000 milliseconds.
#endif
#if MOTOR_SLOT_SPEEDUP_EVERY_MOVES < 1U || MOTOR_SLOT_SPEEDUP_EVERY_MOVES > 2147483647U
#error MOTOR_SLOT_SPEEDUP_EVERY_MOVES must be a positive move count.
#endif

/* EDIT THE RAW TARGETS BELOW (0..16383).
 * Columns are visited left to right, wrapping from home 4 to home 1.
 * P2: larger raw = CCW. P4: smaller raw = CW (both wrap at 16384).
 * These same targets serve initial alignment and task moves.
 */
#ifdef __cplusplus
static constexpr
#else
static const
#endif
uint32_t MOTOR_SLOT_TARGET_RAW[2][4] = {
    /* Home 1  Home 2  Home 3  Home 4 */
    {14785U,  2373U,  6684U, 10400U }, /* P2 / encoder 1 / CCW */
    {14595U, 10605U, 6389U,  2482U }  /* P4 / encoder 2 / CW  */
};

#endif


/* ==================== motor_geometry.h ==================== */
#ifndef MOTOR_GEOMETRY_H
#define MOTOR_GEOMETRY_H
/* Step3 measured references, shaft end looking toward motor.
   User confirmed each listed pose is BOTH task start and clearance groove.
   Keep home and groove references independent of task travel settings.
   P2: pre-trial raw11245; P4: observed aligned raw8379.
   Visual calibration only; clearance width and repeatability still pending.
   Consumed by the isolated return trial AND the (currently inactive) integrated planner. */
#define MOTOR_GEOM_P2_HOME_RAW 11245U
#define MOTOR_GEOM_P2_SLOT_RAW 11245U
#define MOTOR_GEOM_P4_HOME_RAW 8379U
#define MOTOR_GEOM_P4_SLOT_RAW 8379U
/* Alternate paired start observed clear at rest. Use supervised operation. */
#define MOTOR_GEOM_ALT_P2_START_RAW 15381U
#define MOTOR_GEOM_ALT_P4_START_RAW 9898U
#define MOTOR_GEOM_ALT_P2_RETURN_RAW 2638U /* P2 slot confirmed after the extra clockwise move. */
/* Start windows include the measured resting pair and the recovery setpoints. */
#define MOTOR_GEOM_ALT_TASK_P2_START_RAW 2628U
#define MOTOR_GEOM_ALT_TASK_P4_START_RAW 9886U
#define MOTOR_GEOM_SLOT_PITCH_COUNTS 4096U
#endif


/* ==================== motor_tuning.h ==================== */
#ifndef MOTOR_TUNING_H
#define MOTOR_TUNING_H
#include <stdint.h>

/* View from shaft end toward motor. Measured references: motor_geometry.h.
   Home offsets move task origins; slot offsets only calibrate clearance.
   Task travel never changes the registered clearance groove references. */
#define MOTOR_TUNE_P2_HOME_BASE_RAW MOTOR_GEOM_P2_HOME_RAW
#define MOTOR_TUNE_P4_HOME_BASE_RAW MOTOR_GEOM_P4_HOME_RAW
/* P2 positive = CCW; P4 positive = CW. */
#define MOTOR_TUNE_P2_HOME_OFFSET_DEG 0.0f
#define MOTOR_TUNE_P4_HOME_OFFSET_DEG 0.0f
#define MOTOR_TUNE_P2_SLOT_OFFSET_DEG 0.0f
#define MOTOR_TUNE_P4_SLOT_OFFSET_DEG 0.0f
/* P2 travel below is historical; P4 value is now the active calibration trial.
   P2 preparation remains fixed at verified 90deg; integrated task is inactive. */
#define MOTOR_TUNE_P2_STEP_DEG 85.5f
#define MOTOR_TUNE_P4_STEP_DEG 85.0f /* Provisional: user requested ~5deg less than 90. */
/* Commissioning limits for forward task moves. Actual PWM remains feedback-controlled. */
#define MOTOR_TUNE_TASK_P2_DUTY_PCT 40U
#define MOTOR_TUNE_TASK_P4_DUTY_PCT 40U
/* Initial P4 trial: 0.15 duty percentage points per deg/s, i.e. 18% at120deg/s.
 * Added inside the PI saturation/anti-windup calculation; total cap still40%.
 * Applies only to large forward measured-task P4 moves, not alignment/holding. */
#define MOTOR_TUNE_TASK_P4_SPEED_FF_DUTY_PER_DPS 0.0015f
/* Large forward task motion only; alignment and fine holding keep0.0015. */
#define MOTOR_TUNE_TASK_SPEED_KP 0.0018f
#define MOTOR_TUNE_TASK_P2_FIRST_SPEED_DPS 6.0f
#define MOTOR_TUNE_TASK_HOME_HOLD_MS 1000U
#define MOTOR_TUNE_TASK_ARRIVAL_HOLD_MS 2000U
/* Enable six automatic rounds only after the supervised route is verified. */
#ifndef MOTOR_TUNE_ROUNDTRIP_AUTOMATIC
#define MOTOR_TUNE_ROUNDTRIP_AUTOMATIC 0U
#endif
#define MOTOR_TUNE_ROUNDTRIP_COUNT 6U
/* 0 legacy, 1 backs out, 2 survey, 3 measured task, 4 P2 jog, 5 P2 slots, 6 P4 slots. */
#define MOTOR_SLOT_CALIBRATION_SURVEY 2U
#define MOTOR_SLOT_CALIBRATION_MEASURED_TASK 3U
#define MOTOR_SLOT_CALIBRATION_P2_RECALIBRATION 4U
#define MOTOR_SLOT_CALIBRATION_P2_SLOT_TEST 5U
#define MOTOR_SLOT_CALIBRATION_P4_RECALIBRATION 6U
#define MOTOR_SLOT_CALIBRATION_P2_REPEAT30 7U
#define MOTOR_SLOT_CALIBRATION_SLOT_BANKS 8U
#define MOTOR_SLOT_CALIBRATION_P4_REPEAT30 9U
#define MOTOR_TUNE_SURVEY_P2_STEP_DEG (-10.0f) /* Negative is clockwise. */
#ifndef MOTOR_TUNE_SLOT_CALIBRATION
#define MOTOR_TUNE_SLOT_CALIBRATION MOTOR_SLOT_CALIBRATION_MEASURED_TASK
#endif
#define MOTOR_TUNE_ROUNDTRIP_FAST_SCALE 2.0f
#if MOTOR_TUNE_TASK_P2_DUTY_PCT < 10U || MOTOR_TUNE_TASK_P2_DUTY_PCT > 40U || \
    MOTOR_TUNE_TASK_P4_DUTY_PCT < 10U || MOTOR_TUNE_TASK_P4_DUTY_PCT > 40U
#error Task duty limits must be between 10 and 40 percent.
#endif
#define MOTOR_TUNE_CONFIRM_P4_START 1U
#define MOTOR_TUNE_CONFIRM_RECOVERY_HANDOFF 1U
#define MOTOR_TUNE_DEG_COUNTS(deg) \
    ((int32_t)((deg) * (16384.0f / 360.0f) + ((deg) >= 0.0f ? 0.5f : -0.5f)))
#define MOTOR_TUNE_HOME_RAW(base, deg, sign) \
    ((uint32_t)((int32_t)(base) + MOTOR_TUNE_DEG_COUNTS((deg) * (float)(sign))) & 0x3FFFU)
#endif


#endif /* BARAM_MOTOR_CONFIG_H */
