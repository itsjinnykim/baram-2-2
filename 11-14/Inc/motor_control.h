#ifndef BARAM_MOTOR_CONTROL_H
#define BARAM_MOTOR_CONTROL_H
#include "motor_config.h"

/* ==================== pid.h ==================== */
#ifndef PID_H
#define PID_H

/* Hardware-independent control arithmetic. The constexpr path keeps the
 * existing compile-time controller regressions usable; C firmware calls pid.c.
 * A zero feedforward gain retains the legacy speed PI arithmetic. */
typedef struct {
    float position_kp,position_kd,speed_kp,speed_ki;
    float speed_limit,accel_limit,max_duty;
    float braking_accel,braking_margin_deg;
    float speed_ff_duty_per_dps; /* Duty fraction per deg/s of ramped reference. */
} PIDSpeedConfig;
typedef struct {float speed_ref,integral,duty;} PIDSpeedResult;
typedef struct {float integral,duty;} PIDHoldResult;

#ifdef __cplusplus
#define PID_MATH_FN static constexpr
#else
#define PID_MATH_FN static inline
#endif
PID_MATH_FN float PID_ClampImpl(float value,float low,float high)
{return value<low?low:(value>high?high:value);}

PID_MATH_FN float PID_StoppingSpeedImpl(float distance,float decel)
{
    if (distance <= 0.0f || decel <= 0.0f) return 0.0f;
    const float squared = 2.0f * decel * distance;
    float root = squared > 1.0f ? squared : 1.0f;
    for (unsigned i = 0U; i < 12U; ++i) root = 0.5f * (root + squared / root);
    return root;
}

PID_MATH_FN PIDSpeedResult PID_SpeedControlImpl(float position_error,float speed,
    float speed_ref,float integral,float dt,const PIDSpeedConfig *c)
{
    float wanted=PID_ClampImpl(c->position_kp*position_error-c->position_kd*speed,0.0f,c->speed_limit);
    if (c->braking_accel > 0.0f)
    {
        const float predicted_travel=MOTOR_FAST_TRIAL&&c->speed_ff_duty_per_dps>0.0f&&speed>0.0f?
            speed*MOTOR_TUNE_TASK_BRAKE_LATENCY_S:0.0f;
        const float stopping_speed = PID_StoppingSpeedImpl(
            position_error-c->braking_margin_deg-predicted_travel,c->braking_accel);
        if (wanted > stopping_speed) wanted = stopping_speed;
    }
    const float decel=MOTOR_FAST_TRIAL&&c->braking_accel>0.0f?c->braking_accel:c->accel_limit;
    speed_ref+=PID_ClampImpl(wanted-speed_ref,-decel*dt,c->accel_limit*dt);
    float feedforward=PID_ClampImpl(c->speed_ff_duty_per_dps*speed_ref,0.0f,c->max_duty);
    float error=speed_ref-speed;
    float candidate=PID_ClampImpl(integral+c->speed_ki*error*dt,0.0f,c->max_duty);
    float proposed=feedforward+c->speed_kp*error+candidate;
    if(!((proposed>c->max_duty&&error>0.0f)||(proposed<0.0f&&error<0.0f)))integral=candidate;
    float duty=PID_ClampImpl(feedforward+c->speed_kp*error+integral,0.0f,c->max_duty);
    PIDSpeedResult result={speed_ref,integral,duty};return result;
}
PID_MATH_FN PIDHoldResult PID_HoldControlImpl(float error,float speed,float dt,float previous_integral)
{
    float integral=PID_ClampImpl(previous_integral+0.05f*error*dt,-0.1f,0.1f);
    float proposed=0.15f*error-0.0025f*speed+integral;
    if(!((proposed>0.1f&&error>0.0f)||(proposed< -0.1f&&error<0.0f)))previous_integral=integral;
    float duty=PID_ClampImpl(0.15f*error-0.0025f*speed+previous_integral,-0.1f,0.1f);
    PIDHoldResult result={previous_integral,duty};return result;
}
#undef PID_MATH_FN

#ifdef __cplusplus
static constexpr PIDSpeedResult PID_SpeedControl(float position_error,float speed,
    float speed_ref,float integral,float dt,const PIDSpeedConfig *c)
{return PID_SpeedControlImpl(position_error,speed,speed_ref,integral,dt,c);}
static constexpr PIDHoldResult PID_HoldControl(float error,float speed,float dt,float integral)
{return PID_HoldControlImpl(error,speed,dt,integral);}
#else
PIDSpeedResult PID_SpeedControl(float position_error,float speed,float speed_ref,
    float integral,float dt,const PIDSpeedConfig *config);
PIDHoldResult PID_HoldControl(float error,float speed,float dt,float integral);
#endif
#endif


/* ==================== motor_sequence.h ==================== */
#ifndef MOTOR_SEQUENCE_H
#define MOTOR_SEQUENCE_H

#include <stdint.h>



#if MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_P4_REPEAT30
#define MOTORSEQ_BUILD_ID 2026111233U
#elif MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_SLOT_BANKS
#define MOTORSEQ_BUILD_ID 2026111218U
#elif MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_P2_REPEAT30
#define MOTORSEQ_BUILD_ID 2026111234U
#elif MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_P4_RECALIBRATION
#define MOTORSEQ_BUILD_ID 2026111232U
#elif MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_P2_SLOT_TEST
#define MOTORSEQ_BUILD_ID 2026111220U
#elif MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_P2_RECALIBRATION
#define MOTORSEQ_BUILD_ID 2026111208U
#elif MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_MEASURED_TASK
#if MOTOR_FAST_TRIAL
#define MOTORSEQ_BUILD_ID 2026111250U
#else
#define MOTORSEQ_BUILD_ID 2026111245U
#endif
#elif MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_SURVEY
#define MOTORSEQ_BUILD_ID 2026111187U
#elif MOTOR_TUNE_SLOT_CALIBRATION
#define MOTORSEQ_BUILD_ID 2026111185U
#elif MOTOR_TUNE_ROUNDTRIP_AUTOMATIC
#define MOTORSEQ_BUILD_ID 2026111172U
#else
#define MOTORSEQ_BUILD_ID 2026111180U
#endif
/* Feedback orientation and bridge polarity are INDEPENDENT. Latest view
   reference: looking from the OUTPUT SHAFT END toward the motor, P2/TIM1
   CH2 was reported clockwise with fault4. Use CH1 for P2 CCW; keep J1 sign.
   The requested P4 task direction is now CW: reverse its PWM and feedback
   coordinates relative to the previous CCW setup. Keep the leads unchanged. */
#define MOTORSEQ_AXIS1_FEEDBACK_SIGN (1)
#define MOTORSEQ_AXIS2_FEEDBACK_SIGN (-1)
#define MOTORSEQ_AXIS1_PWM_SIGN (1)
#define MOTORSEQ_AXIS2_PWM_SIGN (-1)
#define MOTORSEQ_BUTTON_DEBOUNCE_MS 20U
#define MOTORSEQ_SPEED_FILTER_TAU_MS 20U
#define MOTORSEQ_FILTER_DECAY_MS (5U * MOTORSEQ_SPEED_FILTER_TAU_MS)
#define MOTORSEQ_SETTLE_WINDOW_MS 50U

enum { MOTORSEQ_WAIT = 0, MOTORSEQ_MOVE = 1, MOTORSEQ_PAUSE = 2,
       MOTORSEQ_DONE = 3, MOTORSEQ_FAULT = 4, MOTORSEQ_STOPPED = 5 };
/* Faults: 1=feedback/gap, 2=step timeout, 3=overshoot,
   4=wrong direction, 5=braked axis drift, 6=B1 stop. */
typedef struct
{
    float step_deg, tolerance_deg, hold_tolerance_deg, speed_tolerance_dps;
    float speed_limit_dps, accel_limit_dps2, position_kp, position_kd;
    float speed_kp, speed_ki;
    uint32_t period, max_compare, timeout_ms, settle_ms, pause_ms, rounds;
    uint32_t max_steps; /* 0=all rounds; 1=only P2/J1 calibration, then stop. */
    uint32_t use_fixed_origin;
    float fixed_origin_deg[2]; /* Calibrated task coordinates, not B1 samples. */
    uint32_t continuous, ramp_steps;
    float max_speed_dps, speed_increment_dps;
    uint32_t min_pause_ms, pause_decrement_ms;
    uint32_t hold_enabled;
    float axis2_step_deg; /* 0 inherits step_deg; task-only P4 calibration. */
    uint32_t confirm_axis2_start; /* Calibration: fresh B1 required before P4. */
    float parked_tolerance_deg; /* Zero uses the arrival tolerance for parked-axis guarding. */
    float arrival_tolerance_deg; /* Zero uses tolerance_deg for moving-axis arrival. */
    float braking_accel_dps2; /* Zero keeps the legacy approach; otherwise limit speed by stopping distance. */
    uint32_t arrival_hold_ms; /* Reference-plan arrival confirmation; zero keeps the legacy 200ms. */
    uint32_t arrival_window_ms; /* Zero keeps timed arrival; otherwise observe physical motion. */
    float arrival_speed_dps, arrival_span_deg; /* Limits on both filtered/net speed and window excursion. */
    float speed_ff_duty_per_dps; /* Zero disables moving-axis speed feedforward. */
} MotorSequenceConfig;

typedef struct
{
    uint32_t good[2];
    float position[2], speed[2], dt_s[2];
} MotorSequenceFeedback;

typedef struct
{
    uint32_t compare, max_compare;
    float origin_deg, target_deg, measured_deg, delta_deg, error_deg;
    float step_start_deg, speed_dps, speed_ref_dps, integral;
    float peak_speed_dps;
} MotorSequenceAxis;

typedef struct
{
    uint32_t axis, elapsed_ms;
    float final_error_deg, peak_speed_dps;
} MotorSequenceStepResult;

typedef struct
{
    uint32_t ready, done, fault, fault_axis;
    uint32_t phase, active_axis; /* 1=P2/J1, 2=P4/J2; 0=not started. */
    uint32_t steps_done, total_steps;
    uint32_t step_started_ms, phase_started_ms, settling, settle_started_ms;
    uint32_t elapsed_ms, settle_ms;
    MotorSequenceAxis axis[2]; /* Cumulative from fixed origin, or B1 in legacy modes. */
    uint32_t arrival_latched;
    uint32_t build_id, brake_entries, settling_resets, first_brake_elapsed_ms;
    uint32_t settle_anchor_ms, settle_rejects;
    float settle_anchor_deg, settle_min_deg, settle_max_deg;
    float settle_speed_dps, settle_span_deg;
    MotorSequenceStepResult step_log[8];
    uint32_t stop_requested, ramp_origin_steps, pause_after_ms;
    float current_speed_limit_dps;
    int32_t hold_command[2], hold_polarity[2];
    float hold_integral[2];
    uint32_t hold_blank[2];
    uint32_t axis2_confirmed;
} MotorSequenceState;

/* First stop command only: later coasting, power-switch changes, or DONE
   monitoring must not replace the evidence of the original stop. RAM only. */
typedef struct
{
    uint32_t captured, fault, axis, steps_done, target_raw, raw_at_stop;
    float error_deg, delta_deg;
} MotorSequenceStop;

typedef struct
{
    uint32_t raw_pressed, changed_ms, armed;
} MotorSequenceButton;

#ifdef __cplusplus
#define MOTORSEQ_FN static constexpr
#else
#define MOTORSEQ_FN static inline
#endif

/* Position-only stops retain dynamic braking; feedback/electrical faults stay OFF. */
MOTORSEQ_FN uint32_t MotorSequence_FaultUsesBrake(uint32_t fault)
{ return fault == 3U || fault == 5U; }

MOTORSEQ_FN float MotorSequence_Abs(float value)
{
    return value < 0.0f ? -value : value;
}

MOTORSEQ_FN uint32_t MotorSequence_Repeating(const MotorSequenceConfig *c)
{
    return c->continuous && !c->max_steps;
}

MOTORSEQ_FN float MotorSequence_Clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

MOTORSEQ_FN MotorSequenceStop MotorSequence_CaptureStop(MotorSequenceStop result,
    const MotorSequenceState *state, uint32_t raw, int32_t feedback_sign)
{
    if (result.captured || !state->done) { return result; }
    result.fault = state->fault;
    result.axis = state->active_axis;
    result.steps_done = state->steps_done;
    result.raw_at_stop = raw;
    result.error_deg = state->axis[0].error_deg;
    result.delta_deg = state->axis[0].delta_deg;
    float counts = state->axis[0].error_deg * (16384.0f / 360.0f) * (float)feedback_sign;
    int32_t offset = (int32_t)(counts + (counts < 0.0f ? -0.5f : 0.5f));
    result.target_raw = state->active_axis ?
        (uint32_t)((int32_t)raw + offset) & 0x3FFFU : 0U;
    result.captured = 1U;
    return result;
}

/* Express both feedbacks in the chosen task directions without changing
   raw encoder diagnostics. A +90deg controller target can drive either leg. */
MOTORSEQ_FN MotorSequenceFeedback MotorSequence_OrientFeedback(
    const MotorSequenceFeedback *physical, int32_t direction1, int32_t direction2)
{
    MotorSequenceFeedback feedback = *physical;
    feedback.position[0] *= (float)direction1;
    feedback.speed[0] *= (float)direction1;
    feedback.position[1] *= (float)direction2;
    feedback.speed[1] *= (float)direction2;
    return feedback;
}

MOTORSEQ_FN uint32_t MotorSequence_ChannelCompare(const MotorSequenceState *state,
    uint32_t axis, int32_t direction, uint32_t negative_channel)
{
    if (axis >= 2U || (direction != 1 && direction != -1) || state->fault ||
        state->phase == MOTORSEQ_WAIT || state->phase == MOTORSEQ_FAULT ||
        state->phase == MOTORSEQ_DONE) { return 0U; }
    if (state->phase == MOTORSEQ_MOVE && state->active_axis == axis + 1U)
    { return ((direction < 0) == (negative_channel != 0U)) ? state->axis[axis].compare : 0U; }
    int32_t command = state->hold_command[axis];
    if (!command) { return 0U; }
    int32_t sign = direction * (command < 0 ? -1 : 1);
    uint32_t compare = (uint32_t)(command < 0 ? -command : command);
    return ((sign < 0) == (negative_channel != 0U)) ? compare : 0U;
}

/* A button held across reset never starts; require a stable release first. */
MOTORSEQ_FN uint32_t MotorSequence_ButtonPress(MotorSequenceButton *button,
                                              uint32_t pressed, uint32_t now)
{
    if (pressed != button->raw_pressed)
    {
        button->raw_pressed = pressed;
        button->changed_ms = now;
    }
    if ((uint32_t)(now - button->changed_ms) < MOTORSEQ_BUTTON_DEBOUNCE_MS) { return 0U; }
    if (!pressed) { button->armed = 1U; return 0U; }
    if (!button->armed) { return 0U; }
    button->armed = 0U;
    return 1U;
}

MOTORSEQ_FN void MotorSequence_Brake(MotorSequenceState *state)
{
    for (uint32_t i = 0U; i < 2U; ++i)
    {
        state->axis[i].compare = 0U;
        state->axis[i].speed_ref_dps = 0.0f;
        state->axis[i].integral = 0.0f;
    }
}

MOTORSEQ_FN void MotorSequence_Trip(MotorSequenceState *state,
                                   uint32_t fault, uint32_t axis)
{
    if (state->phase == MOTORSEQ_FAULT) { return; }
    MotorSequence_Brake(state);
    for (uint32_t i = 0U; i < 2U; ++i)
    { state->hold_command[i] = 0; state->hold_integral[i] = 0.0f; }
    state->fault = fault;
    state->fault_axis = axis;
    state->ready = 0U;
    state->done = 1U;
    state->phase = MOTORSEQ_FAULT;
}

MOTORSEQ_FN void MotorSequence_BeginStep(MotorSequenceState *state,
                                        const MotorSequenceConfig *config,
                                        uint32_t now)
{
    uint32_t index = state->steps_done % 2U;
    uint32_t level = MotorSequence_Repeating(config) && config->ramp_steps ?
        (state->steps_done - state->ramp_origin_steps) / config->ramp_steps : 0U;
    float ceiling = config->max_speed_dps > config->speed_limit_dps ?
        config->max_speed_dps : config->speed_limit_dps;
    state->current_speed_limit_dps = MotorSequence_Clamp(config->speed_limit_dps +
        (float)level * config->speed_increment_dps, config->speed_limit_dps, ceiling);
    uint32_t floor_pause = config->min_pause_ms < config->pause_ms ? config->min_pause_ms : config->pause_ms;
    float reduction = MotorSequence_Clamp((float)level * (float)config->pause_decrement_ms,
        0.0f, (float)(config->pause_ms - floor_pause));
    state->pause_after_ms = config->pause_ms - (uint32_t)reduction;
    state->active_axis = index + 1U;
    if (!index) state->axis2_confirmed = 0U;
    /* Fixed origins prevent the previous step's ~2deg residual accumulating. */
    float step = index == 1U && config->axis2_step_deg > 0.0f ?
                 config->axis2_step_deg : config->step_deg;
    state->axis[index].target_deg = state->axis[index].origin_deg +
        (float)(state->steps_done / 2U + 1U) * step;
    state->axis[index].step_start_deg = state->axis[index].measured_deg;
    state->axis[index].peak_speed_dps = 0.0f;
    state->axis[index].error_deg = state->axis[index].target_deg -
                                  state->axis[index].measured_deg;
    state->step_started_ms = now;
    state->elapsed_ms = 0U;
    state->settling = 0U;
    state->settle_ms = 0U;
    state->arrival_latched = 0U;
    state->brake_entries = 0U;
    state->settling_resets = 0U;
    state->first_brake_elapsed_ms = 0U;
    state->settle_rejects = 0U;
    state->settle_speed_dps = 0.0f;
    state->settle_span_deg = 0.0f;
    state->phase = MOTORSEQ_MOVE;
    MotorSequence_Brake(state);
}

/* Low-duty position holding, separate from the moving-axis speed PI.
   Output is bounded at 10% compare; this is not current control. Reversing
   the holding direction requires two zero-output polls before the new leg. */
MOTORSEQ_FN void MotorSequence_Hold(MotorSequenceState *s, const MotorSequenceConfig *c,
                                   const MotorSequenceFeedback *f)
{
    for (uint32_t i = 0U; i < 2U; ++i)
    {
        if (!c->hold_enabled || s->phase == MOTORSEQ_DONE ||
            (s->phase == MOTORSEQ_MOVE && s->active_axis == i + 1U))
        {
            s->hold_command[i] = 0; s->hold_integral[i] = 0.0f;
            s->hold_polarity[i] = 0; s->hold_blank[i] = 0U;
            continue;
        }
        float dt = f->dt_s[i];
        if (!(dt > 0.0f && dt <= 0.005f))
        { MotorSequence_Trip(s, 1U, i + 1U); return; }
        float error = s->axis[i].error_deg;
        if (MotorSequence_Abs(error) <= 0.15f)
        { s->hold_command[i] = 0; s->hold_integral[i] = 0.0f; continue; }
        PIDHoldResult control = PID_HoldControl(error,f->speed[i],dt,s->hold_integral[i]);
        s->hold_integral[i] = control.integral;
        float duty = control.duty;
        int32_t sign = duty < 0.0f ? -1 : 1;
        if (sign != s->hold_polarity[i])
        { s->hold_polarity[i] = sign; s->hold_blank[i] = 2U; }
        if (s->hold_blank[i])
        { s->hold_blank[i]--; s->hold_command[i] = 0; continue; }
        uint32_t compare = (uint32_t)(MotorSequence_Abs(duty) * (float)c->period + 0.5f);
        uint32_t cap = c->period / 10U;
        if (cap > c->max_compare) { cap = c->max_compare; }
        if (compare > cap) { compare = cap; }
        s->hold_command[i] = sign * (int32_t)compare;
    }
}

/* The active axis follows the speed PI; inactive axes may use bounded holding. */
MOTORSEQ_FN void MotorSequence_Update(MotorSequenceState *state,
    const MotorSequenceConfig *config, const MotorSequenceFeedback *feedback,
    uint32_t now, uint32_t preflight_ready, uint32_t button_press)
{
    state->build_id = MOTORSEQ_BUILD_ID;
    if (state->phase == MOTORSEQ_FAULT) { return; }
    for (uint32_t i = 0U; i < 2U; ++i)
    {
        state->axis[i].measured_deg = feedback->position[i];
        state->axis[i].speed_dps = feedback->speed[i];
    }
    if (state->phase == MOTORSEQ_WAIT)
    {
        state->ready = preflight_ready && feedback->good[0] && feedback->good[1];
        if (config->use_fixed_origin)
        {
            for (uint32_t i = 0U; i < 2U; ++i)
                if (MotorSequence_Abs(config->fixed_origin_deg[i] - feedback->position[i]) >
                    config->tolerance_deg) { state->ready = 0U; }
        }
        state->total_steps = MotorSequence_Repeating(config) ? 0U : 2U * config->rounds;
        if (config->max_steps && config->max_steps < state->total_steps)
        { state->total_steps = config->max_steps; }
        if (!state->ready || !button_press) { return; }
        for (uint32_t i = 0U; i < 2U; ++i)
        {
            state->axis[i].origin_deg = config->use_fixed_origin ?
                config->fixed_origin_deg[i] : feedback->position[i];
            state->axis[i].target_deg = state->axis[i].origin_deg;
        }
        state->ready = 0U;
        MotorSequence_BeginStep(state, config, now);
        return;
    }
    for (uint32_t i = 0U; i < 2U; ++i)
    {
        if (!feedback->good[i]) { MotorSequence_Trip(state, 1U, i + 1U); return; }
        state->axis[i].delta_deg = feedback->position[i] - state->axis[i].origin_deg;
        state->axis[i].error_deg = state->axis[i].target_deg - feedback->position[i];
    }
    if (button_press && state->phase == MOTORSEQ_PAUSE && config->confirm_axis2_start &&
        (state->steps_done % 2U) == 1U)
    { state->axis2_confirmed = 1U; button_press = 0U; }
    if (button_press && state->phase != MOTORSEQ_DONE)
    {
        if (MotorSequence_Repeating(config))
        {
            state->stop_requested = 1U;
            if (state->phase == MOTORSEQ_PAUSE)
            {
                MotorSequence_Brake(state);
                state->done = 1U; state->phase = MOTORSEQ_DONE; return;
            }
        }
        else { MotorSequence_Trip(state, 6U, state->active_axis); return; }
    }
    for (uint32_t i = 0U; i < 2U; ++i)
    {
        if ((state->phase == MOTORSEQ_MOVE) && (state->active_axis == i + 1U))
        { continue; }
        if (MotorSequence_Abs(state->axis[i].error_deg) > config->hold_tolerance_deg)
        { MotorSequence_Trip(state, 5U, i + 1U); return; }
    }
    MotorSequence_Hold(state, config, feedback);
    if (state->phase == MOTORSEQ_FAULT) { return; }
    if (state->phase == MOTORSEQ_DONE)
    {
        if (button_press && MotorSequence_Repeating(config))
        {
            state->done = 0U; state->stop_requested = 0U;
            state->ramp_origin_steps = state->steps_done;
            MotorSequence_BeginStep(state, config, now);
        }
        return;
    }
    if (state->phase == MOTORSEQ_PAUSE)
    {
        if ((uint32_t)(now - state->phase_started_ms) >= state->pause_after_ms &&
            (!config->confirm_axis2_start || (state->steps_done % 2U) == 0U || state->axis2_confirmed))
        { MotorSequence_BeginStep(state, config, now); }
        return;
    }
    uint32_t index = state->active_axis - 1U;
    MotorSequenceAxis *axis = &state->axis[index];
    if (MotorSequence_Abs(axis->speed_dps) > axis->peak_speed_dps)
    { axis->peak_speed_dps = MotorSequence_Abs(axis->speed_dps); }
    /* Approach the INNER half of the final position tolerance before braking.
       Once braking, retain it throughout the original full tolerance. This
       prevents an outer-edge sample from clearing the PI too early, followed
       by recoil/chatter and repeated low-speed restarts near the deadline.
       Default: enter at <=1deg error, settle within the original +/-2deg. */
    float arrival_tolerance = config->tolerance_deg *
                              (state->arrival_latched ? 1.0f : 0.5f);
    state->elapsed_ms = (uint32_t)(now - state->step_started_ms);
    if (state->elapsed_ms >= config->timeout_ms)
    {
        /* The 4s drive deadline stays absolute. A move which entered the
           arrival band BEFORE that deadline may finish its 200ms observation
           with BOTH compares already zero. Never resume PWM in this grace.
           Allow five speed-filter time constants separately from the full
           low-speed confirmation window. The absolute bound is therefore
           timeout_ms + FILTER_DECAY_MS + settle_ms (default 4300ms). */
        uint32_t brake_only = state->arrival_latched &&
            axis->error_deg >= -config->tolerance_deg &&
            axis->error_deg <= arrival_tolerance &&
            !state->axis[0].compare && !state->axis[1].compare;
        if (!brake_only ||
            state->elapsed_ms - config->timeout_ms >=
                MOTORSEQ_FILTER_DECAY_MS + config->settle_ms)
        { MotorSequence_Trip(state, 2U, state->active_axis); return; }
    }
    if (axis->measured_deg - axis->step_start_deg < -3.0f)
    { MotorSequence_Trip(state, 4U, state->active_axis); return; }
    if (axis->error_deg < -config->tolerance_deg)
    { MotorSequence_Trip(state, 3U, state->active_axis); return; }
    if (axis->error_deg <= arrival_tolerance)
    {
        if (!state->arrival_latched)
        {
            state->brake_entries++;
            if (state->brake_entries == 1U)
            { state->first_brake_elapsed_ms = state->elapsed_ms; }
            state->settle_anchor_ms = now;
            state->settle_anchor_deg = axis->measured_deg;
            state->settle_min_deg = axis->measured_deg;
            state->settle_max_deg = axis->measured_deg;
        }
        state->arrival_latched = 1U;
        MotorSequence_Brake(state);
        /* The control-loop velocity remains unchanged. Motion completion uses
           measured position over 50ms windows: average speed <=5deg/s AND
           full position span <=0.25deg per 50ms at the default settings.
           Four consecutive valid windows prove 200ms of low measured motion;
           net-zero oscillation cannot pass the span check. This avoids
           resetting arrival on single-sample differentiated encoder noise. */
        if (axis->measured_deg < state->settle_min_deg)
        { state->settle_min_deg = axis->measured_deg; }
        if (axis->measured_deg > state->settle_max_deg)
        { state->settle_max_deg = axis->measured_deg; }
        uint32_t window_ms = (uint32_t)(now - state->settle_anchor_ms);
        if (window_ms >= MOTORSEQ_SETTLE_WINDOW_MS)
        {
            float window_s = (float)window_ms * 0.001f;
            state->settle_speed_dps = (axis->measured_deg - state->settle_anchor_deg) / window_s;
            state->settle_span_deg = state->settle_max_deg - state->settle_min_deg;
            if (MotorSequence_Abs(state->settle_speed_dps) <= config->speed_tolerance_dps &&
                state->settle_span_deg <= config->speed_tolerance_dps * window_s)
            {
                if (!state->settling)
                { state->settling = 1U; state->settle_started_ms = state->settle_anchor_ms; }
                state->settle_ms = (uint32_t)(now - state->settle_started_ms);
                if (state->settle_ms >= config->settle_ms)
                {
                    if (state->steps_done < 8U || MotorSequence_Repeating(config))
                    {
                        MotorSequenceStepResult *result = &state->step_log[state->steps_done % 8U];
                        result->axis = state->active_axis;
                        result->elapsed_ms = state->elapsed_ms;
                        result->final_error_deg = axis->error_deg;
                        result->peak_speed_dps = axis->peak_speed_dps;
                    }
                    state->steps_done++;
                    state->phase_started_ms = now;
                    if (state->stop_requested ||
                        (!MotorSequence_Repeating(config) && state->steps_done == state->total_steps))
                    { state->done = 1U; state->phase = MOTORSEQ_DONE; }
                    else { state->phase = MOTORSEQ_PAUSE; }
                }
            }
            else
            {
                state->settle_rejects++;
                if (state->settling) { state->settling_resets++; }
                state->settling = 0U;
                state->settle_ms = 0U;
            }
            state->settle_anchor_ms = now;
            state->settle_anchor_deg = axis->measured_deg;
            state->settle_min_deg = axis->measured_deg;
            state->settle_max_deg = axis->measured_deg;
        }
        return;
    }
    if (state->settling) { state->settling_resets++; }
    state->settling = 0U;
    state->settle_ms = 0U;
    state->arrival_latched = 0U;
    float dt = feedback->dt_s[index];
    if (!(dt > 0.0f && dt <= 0.005f))
    { MotorSequence_Trip(state, 1U, state->active_axis); return; }
    float max_duty = (float)config->max_compare / (float)config->period;
    float brake_margin=config->tolerance_deg*0.5f;
#if MOTOR_FAST_TRIAL
    if(config->braking_accel_dps2>0.0f&&config->speed_ff_duty_per_dps>0.0f)
        brake_margin=MOTOR_TUNE_TASK_BRAKE_MARGIN_DEG;
#endif
    PIDSpeedConfig pid_config = {config->position_kp,config->position_kd,config->speed_kp,
        config->speed_ki,state->current_speed_limit_dps,config->accel_limit_dps2,max_duty,
        config->braking_accel_dps2,brake_margin,config->speed_ff_duty_per_dps};
    PIDSpeedResult control = PID_SpeedControl(axis->error_deg,axis->speed_dps,
        axis->speed_ref_dps,axis->integral,dt,&pid_config);
    axis->speed_ref_dps = control.speed_ref;
    axis->integral = control.integral;
    float duty = control.duty;
    axis->compare = (uint32_t)(duty * (float)config->period + 0.5f);
    if (axis->compare > config->max_compare) { axis->compare = config->max_compare; }
    if (axis->compare > axis->max_compare) { axis->max_compare = axis->compare; }
}

#undef MOTORSEQ_FN
#endif


/* ==================== motor_reference.h ==================== */
#ifndef MOTOR_REFERENCE_H
#define MOTOR_REFERENCE_H



#define MOTORREF_RAW1 MOTOR_TUNE_HOME_RAW(MOTOR_TUNE_P2_HOME_BASE_RAW, MOTOR_TUNE_P2_HOME_OFFSET_DEG, MOTORSEQ_AXIS1_FEEDBACK_SIGN)
#define MOTORREF_RAW2 MOTOR_TUNE_HOME_RAW(MOTOR_TUNE_P4_HOME_BASE_RAW, MOTOR_TUNE_P4_HOME_OFFSET_DEG, MOTORSEQ_AXIS2_FEEDBACK_SIGN)
#define MOTORREF_SLOT_PITCH_COUNTS MOTOR_GEOM_SLOT_PITCH_COUNTS
#define MOTORREF_SLOT_ENTRY_DEG 3.0f
#define MOTORREF_PLAN_CAPACITY 4U

enum { MOTORREF_BOOT = 0, MOTORREF_ALIGN = 1, MOTORREF_READY = 2, MOTORREF_FAULT = 3,
       MOTORREF_PARTIAL_DONE = 4 };
enum { MOTORREF_PLAN_OK = 0, MOTORREF_NO_CLEARANCE = 1, MOTORREF_BAD_GOAL = 2,
       MOTORREF_BAD_CONFIG = 3, MOTORREF_NO_LINK_TO_HOME = 4 };
enum { MOTORREF_ENTRY = 1, MOTORREF_WAYPOINT = 2, MOTORREF_HOME = 3 };

typedef struct
{
    uint32_t valid, blocked_reason, count, initial_parked_axis;
    uint32_t partial;
    uint32_t axis[MOTORREF_PLAN_CAPACITY], raw[MOTORREF_PLAN_CAPACITY];
    uint32_t kind[MOTORREF_PLAN_CAPACITY];
    uint32_t park_raw[MOTORREF_PLAN_CAPACITY];
    float cost_deg;
} MotorReferencePlan;

typedef struct
{
    uint32_t phase, ready, fault, fault_axis, fault_site;
    uint32_t fine_retry_count, fine_settling, fine_settle_ms;
    uint32_t recovery_waiting, recovery_requested;
    uint32_t recovery_release_seen, recovery_release_started_ms;
    uint32_t clearance_waiting, clearance_requested, clearance_granted_step;
    uint32_t clearance_started_ms, clearance_release_seen, clearance_release_started_ms;
    uint32_t return_stage, return_parked_axis, moving_axis;
    uint32_t raw[2], slot_raw[2], slot_index[2];
    float target_deg[2], error_deg[2], slot_error_deg[2];
    float hold_target_deg[2], waypoint_target_deg, waypoint_error_deg;
    int32_t direction[2];
    MotorReferencePlan plan;
    uint32_t plan_step, recovery_started_ms, recovery_deadline_ms, recovery_elapsed_ms;
    uint32_t leg_started_ms, leg_timeout_ms, leg_elapsed_ms, leg_settling;
    uint32_t leg_settle_started_ms, leg_settle_ms, leg_stable;
    uint32_t handoff_window_active, handoff_window_started_ms, handoff_last_sample_ms, handoff_samples;
    float handoff_anchor_deg[2], handoff_min_deg[2], handoff_max_deg[2];
    float handoff_speed_dps[2], handoff_span_deg[2];
    uint32_t ready_outside_active, ready_outside_started_ms, ready_outside_ms;
    uint32_t park_outside_active[2], park_outside_started_ms[2], park_outside_ms[2];
    float park_error_deg[2], park_peak_error_deg[2];
    MotorSequenceState motion[2], ready_hold;
    uint32_t startup_plan;
} MotorReferenceState;

#ifdef __cplusplus
#define MOTORREF_FN static constexpr
#else
#define MOTORREF_FN static inline
#endif

MOTORREF_FN uint32_t MotorReference_HomeRaw(uint32_t axis)
{ return axis ? MOTORREF_RAW2 : MOTORREF_RAW1; }

MOTORREF_FN int32_t MotorReference_FeedbackSign(uint32_t axis)
{ return axis ? MOTORSEQ_AXIS2_FEEDBACK_SIGN : MOTORSEQ_AXIS1_FEEDBACK_SIGN; }

MOTORREF_FN int32_t MotorReference_PwmSign(uint32_t axis)
{ return axis ? MOTORSEQ_AXIS2_PWM_SIGN : MOTORSEQ_AXIS1_PWM_SIGN; }

MOTORREF_FN float MotorReference_Offset(uint32_t raw, uint32_t target)
{
    int32_t counts = (int32_t)(target & 0x3FFFU) - (int32_t)(raw & 0x3FFFU);
    if (counts > 8192) counts -= 16384;
    if (counts < -8192) counts += 16384;
    return (float)counts * (360.0f / 16384.0f);
}

/* Clearance groove calibration is independent of home offsets and task travel. */
MOTORREF_FN uint32_t MotorReference_SlotZeroRaw(const MotorSequenceConfig *task, uint32_t axis)
{
    (void)task;
    return axis ? MOTOR_TUNE_HOME_RAW(MOTOR_GEOM_P4_SLOT_RAW, MOTOR_TUNE_P4_SLOT_OFFSET_DEG, MOTORSEQ_AXIS2_FEEDBACK_SIGN) :
        MOTOR_TUNE_HOME_RAW(MOTOR_GEOM_P2_SLOT_RAW, MOTOR_TUNE_P2_SLOT_OFFSET_DEG, MOTORSEQ_AXIS1_FEEDBACK_SIGN);
}

MOTORREF_FN uint32_t MotorReference_NearestSlotRaw(uint32_t raw, uint32_t zero)
{
    uint32_t phase = (raw - zero) & (MOTORREF_SLOT_PITCH_COUNTS - 1U);
    int32_t delta = phase > 2048U ? (int32_t)phase - 4096 : (int32_t)phase;
    return (uint32_t)((int32_t)(raw & 0x3FFFU) - delta) & 0x3FFFU;
}

MOTORREF_FN uint32_t MotorReference_IsSlot(uint32_t raw, uint32_t zero, float tolerance)
{
    return MotorSequence_Abs(MotorReference_Offset(raw, MotorReference_NearestSlotRaw(raw, zero))) <= tolerance;
}

MOTORREF_FN void MotorReference_Append(MotorReferencePlan *p, uint32_t predicted[2],
    uint32_t axis, uint32_t raw, uint32_t kind)
{
    raw &= 0x3FFFU;
    if (predicted[axis] == raw) return;
    if (p->count >= MOTORREF_PLAN_CAPACITY) { p->valid = 0U; return; }
    p->axis[p->count] = axis + 1U;
    p->raw[p->count] = raw;
    p->kind[p->count] = kind;
    p->cost_deg += MotorSequence_Abs(MotorReference_Offset(predicted[axis], raw));
    p->count++;
    predicted[axis] = raw;
}

MOTORREF_FN MotorReferencePlan MotorReference_Candidate(const MotorSequenceConfig *task,
    uint32_t raw0, uint32_t raw1, uint32_t park)
{
    MotorReferencePlan p = {0}; p.valid = 1U; p.initial_parked_axis = park + 1U;
    uint32_t current[2] = {raw0 & 0x3FFFU, raw1 & 0x3FFFU};
    uint32_t predicted[2] = {current[0], current[1]};
    uint32_t home[2] = {MOTORREF_RAW1, MOTORREF_RAW2};
    uint32_t zero[2] = {MotorReference_SlotZeroRaw(task, 0U), MotorReference_SlotZeroRaw(task, 1U)};
    uint32_t other = 1U - park;
    uint32_t home_slot[2] = {
        MotorReference_IsSlot(home[0], zero[0], task->tolerance_deg),
        MotorReference_IsSlot(home[1], zero[1], task->tolerance_deg)};
    uint32_t anchor = MotorReference_NearestSlotRaw(current[park], zero[park]);
    if (MotorSequence_Abs(MotorReference_Offset(current[park], anchor)) > MOTORREF_SLOT_ENTRY_DEG)
    { p.valid = 0U; p.blocked_reason = MOTORREF_NO_CLEARANCE; return p; }
    if (home_slot[park] && MotorSequence_Abs(MotorReference_Offset(anchor, home[park])) <= task->tolerance_deg &&
        MotorSequence_Abs(MotorReference_Offset(current[park], home[park])) <= MOTORREF_SLOT_ENTRY_DEG)
        anchor = home[park];
    MotorReference_Append(&p, predicted, park, anchor, MOTORREF_ENTRY);
    if (predicted[park] == home[park] && home_slot[park])
    { MotorReference_Append(&p, predicted, other, home[other], MOTORREF_HOME); }
    else if (home_slot[other])
    {
        MotorReference_Append(&p, predicted, other, home[other], MOTORREF_HOME);
        MotorReference_Append(&p, predicted, park, home[park], MOTORREF_HOME);
    }
    else if (home_slot[park])
    {
        /* The other HOME is not itself a groove: use a groove near that home
           to release the parked disk, return it, then finish the other home. */
        MotorReference_Append(&p, predicted, other,
            MotorReference_NearestSlotRaw(home[other], zero[other]), MOTORREF_WAYPOINT);
        MotorReference_Append(&p, predicted, park, home[park], MOTORREF_HOME);
        MotorReference_Append(&p, predicted, other, home[other], MOTORREF_HOME);
    }
    else { p.valid = 0U; p.blocked_reason = MOTORREF_BAD_GOAL; return p; }

    /* Validate every path handoff independently of the chosen ordering.
       Only the first <=3deg slot alignment may move without the other at a
       groove. All subsequent sweeps require the other disk at a groove. */
    predicted[0] = current[0]; predicted[1] = current[1];
    for (uint32_t j = 0U; j < p.count; ++j)
    {
        uint32_t i = p.axis[j] - 1U;
        uint32_t entry = j == 0U && p.kind[j] == MOTORREF_ENTRY &&
            MotorSequence_Abs(MotorReference_Offset(predicted[i], p.raw[j])) <= MOTORREF_SLOT_ENTRY_DEG;
        if (!entry && !MotorReference_IsSlot(predicted[1U-i], zero[1U-i], task->tolerance_deg))
        { p.valid = 0U; p.blocked_reason = MOTORREF_NO_CLEARANCE; return p; }
        predicted[i] = p.raw[j];
    }
    if (predicted[0] != home[0] || predicted[1] != home[1])
    { p.valid = 0U; p.blocked_reason = MOTORREF_BAD_GOAL; }
    return p;
}

MOTORREF_FN MotorReferencePlan MotorReference_PlanReturn(const MotorSequenceConfig *task,
    uint32_t raw0, uint32_t raw1)
{
    MotorReferencePlan empty = {0}; empty.blocked_reason = MOTORREF_NO_CLEARANCE;
    if (!(task->step_deg > 0.0f && task->step_deg <= 360.0f))
    { empty.blocked_reason = MOTORREF_BAD_CONFIG; return empty; }
    MotorReferencePlan a = MotorReference_Candidate(task, raw0, raw1, 0U);
    MotorReferencePlan b = MotorReference_Candidate(task, raw0, raw1, 1U);
    if (a.valid && b.valid) return a.cost_deg < b.cost_deg ? a : b;
    if (a.valid) return a;
    if (b.valid) return b;
    return empty;
}

MOTORREF_FN void MotorReference_Fail(MotorReferenceState *h, uint32_t fault, uint32_t axis, uint32_t site)
{
    if (h->phase == MOTORREF_FAULT) return;
    h->fault = fault; h->fault_axis = axis; h->fault_site = site;
    h->ready = 0U; h->phase = MOTORREF_FAULT;
    MotorSequence_Trip(&h->motion[0], fault, axis);
    MotorSequence_Trip(&h->motion[1], fault, axis);
    MotorSequence_Trip(&h->ready_hold, fault, axis);
}

MOTORREF_FN void MotorReference_Trip(MotorReferenceState *h, uint32_t fault, uint32_t axis)
{ MotorReference_Fail(h, fault, axis, 0U); }

MOTORREF_FN void MotorReference_ReadPose(MotorReferenceState *h, const MotorSequenceConfig *task,
    const MotorSequenceFeedback *f, uint32_t raw0, uint32_t raw1)
{
    h->raw[0] = raw0 & 0x3FFFU; h->raw[1] = raw1 & 0x3FFFU;
    for (uint32_t i = 0U; i < 2U; ++i)
    {
        uint32_t zero = MotorReference_SlotZeroRaw(task, i);
        h->error_deg[i] = MotorReference_Offset(h->raw[i], MotorReference_HomeRaw(i)) * (float)MotorReference_FeedbackSign(i);
        h->target_deg[i] = f->position[i] + h->error_deg[i];
        h->slot_raw[i] = MotorReference_NearestSlotRaw(h->raw[i], zero);
        h->slot_error_deg[i] = MotorReference_Offset(h->raw[i], h->slot_raw[i]) * (float)MotorReference_FeedbackSign(i);
        h->slot_index[i] = ((h->slot_raw[i] - zero) & 0x3FFFU) / 4096U;
    }
}

MOTORREF_FN MotorSequenceConfig MotorReference_Config(const MotorSequenceConfig *task, float distance)
{
    MotorSequenceConfig c = *task;
    c.step_deg = MotorSequence_Abs(distance); c.axis2_step_deg = 0.0f;
    c.use_fixed_origin = 0U; c.continuous = 0U; c.rounds = 1U; c.max_steps = 1U;
    c.hold_enabled = 0U;
    c.confirm_axis2_start = 0U;
    c.speed_limit_dps = MotorSequence_Abs(distance) <= MOTORREF_SLOT_ENTRY_DEG ? 5.0f :
        (task->speed_limit_dps < 15.0f ? task->speed_limit_dps : 15.0f);
    c.position_kp = MotorSequence_Abs(distance) <= MOTORREF_SLOT_ENTRY_DEG ? 20.0f : task->position_kp;
    c.accel_limit_dps2 = task->accel_limit_dps2 < 60.0f ? task->accel_limit_dps2 : 60.0f;
    c.timeout_ms = MotorSequence_Abs(distance) <= MOTORREF_SLOT_ENTRY_DEG ? 10000U : 20000U;
    return c;
}

MOTORREF_FN MotorSequenceConfig MotorReference_LegConfig(const MotorReferenceState *h,
    const MotorSequenceConfig *task, float distance)
{
    MotorSequenceConfig c = MotorReference_Config(task, distance);
    c.speed_ff_duty_per_dps = 0.0f;
    if (task->arrival_tolerance_deg > 0.0f) c.tolerance_deg = task->arrival_tolerance_deg;
    if (h->startup_plan)
    {
#if MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_MEASURED_TASK || \
    MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_P2_RECALIBRATION || \
    MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_P2_SLOT_TEST || \
    MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_P4_RECALIBRATION || \
    MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_P2_REPEAT30 || \
    MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_P4_REPEAT30 || \
    MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_SLOT_BANKS
        /* Measured-slot and recalibration legs use their configured limits. */
        c.speed_limit_dps = task->speed_limit_dps;
        c.accel_limit_dps2 = task->accel_limit_dps2;
#else
        c.speed_limit_dps = 3.0f;
#endif
        c.position_kp = task->position_kp; c.timeout_ms = 45000U;
    }
    else if (h->plan.kind[h->plan_step] == MOTORREF_ENTRY)
    { c.speed_limit_dps = 5.0f; c.position_kp = 20.0f; c.timeout_ms = 10000U; }
#if MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_MEASURED_TASK
    if (h->startup_plan && h->moving_axis >= 1U && h->moving_axis <= 2U &&
        h->direction[h->moving_axis-1U] == 1 && distance > MOTORREF_SLOT_ENTRY_DEG &&
        task->braking_accel_dps2 > 0.0f && task->arrival_window_ms)
    {
        c.speed_kp = MOTOR_TUNE_TASK_SPEED_KP;
#if MOTOR_FAST_TRIAL
        c.speed_ki = MOTOR_TUNE_TASK_SPEED_KI;
        c.position_kp = MOTOR_TUNE_TASK_POSITION_KP;
        c.position_kd = MOTOR_TUNE_TASK_POSITION_KD;
#endif
        c.speed_ff_duty_per_dps = h->moving_axis==1U?
            MOTOR_TUNE_TASK_P2_SPEED_FF_DUTY_PER_DPS:MOTOR_TUNE_TASK_P4_SPEED_FF_DUTY_PER_DPS;
    }
#endif
    return c;
}

MOTORREF_FN uint32_t MotorReference_Guard(MotorReferenceState *h, const MotorSequenceConfig *task,
    uint32_t i, float error, uint32_t now, uint32_t grace)
{
    float e = MotorSequence_Abs(error); h->park_error_deg[i] = error;
    if (e > h->park_peak_error_deg[i]) h->park_peak_error_deg[i] = e;
    const float tolerance = task->parked_tolerance_deg > 0.0f ?
        task->parked_tolerance_deg : task->tolerance_deg;
    if (e <= tolerance)
    { h->park_outside_active[i] = 0U; h->park_outside_ms[i] = 0U; return 1U; }
    if (e > task->hold_tolerance_deg)
    { MotorReference_Fail(h, 5U, i+1U, 4U); return 0U; }
    if (!h->park_outside_active[i])
    { h->park_outside_active[i] = 1U; h->park_outside_started_ms[i] = now; }
    h->park_outside_ms[i] = (uint32_t)(now - h->park_outside_started_ms[i]);
    if (h->park_outside_ms[i] >= grace)
    { MotorReference_Fail(h, 5U, i+1U, 5U); return 0U; }
    return 1U;
}

#if MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_MEASURED_TASK
#define MOTORREF_FINE_SPEED_DPS 8.0f
#define MOTORREF_FINE_ACCEL_DPS2 MOTOR_TUNE_TASK_FINE_ACCEL_DPS2
#define MOTORREF_FINE_DUTY_PCT 10U
#define MOTORREF_FINE_QUIET_DEG 0.25f
/* One-time moving-axis PI preload, not a fixed PWM kick or duty floor. */
#define MOTORREF_P4_START_DUTY_PCT 3U
/* Signed low-speed control for task holding. Error/guard/stability remain in
 * physical degrees; only the actuator control changes from direct position PI.
 * The existing two zero-output samples protect every polarity change. */
MOTORREF_FN void MotorReference_TaskHold(MotorReferenceState *h,const MotorSequenceConfig *c,
    const MotorSequenceFeedback *f)
{
    MotorSequenceState *s=&h->ready_hold;
    for(uint32_t i=0U;i<2U;++i)
    {
        MotorSequenceAxis *a=&s->axis[i];
        const float dt=f->dt_s[i],error=a->error_deg;
        if(!(dt>0.0f&&dt<=0.005f))
        {MotorReference_Fail(h,1U,i+1U,1U);return;}
        s->hold_integral[i]=0.0f;
        /* Keep correcting through the one-degree completion boundary. Reset
         * only near the target center, so one encoder count of noise cannot
         * repeatedly erase the breakaway effort. Stability still uses 1deg. */
        const float quiet=c->arrival_tolerance_deg<MOTORREF_FINE_QUIET_DEG?
            c->arrival_tolerance_deg:MOTORREF_FINE_QUIET_DEG;
        if(MotorSequence_Abs(error)<=quiet)
        {
            s->hold_command[i]=0;s->hold_polarity[i]=0;s->hold_blank[i]=0U;
            a->speed_ref_dps=0.0f;a->integral=0.0f;a->compare=0U;continue;
        }
        const int32_t direction=error<0.0f?-1:1;
        if(direction!=s->hold_polarity[i])
        {
            s->hold_polarity[i]=direction;s->hold_blank[i]=2U;
            a->speed_ref_dps=0.0f;a->integral=0.0f;
        }
        if(s->hold_blank[i])
        {--s->hold_blank[i];s->hold_command[i]=0;a->compare=0U;continue;}
        uint32_t cap=c->period*MOTORREF_FINE_DUTY_PCT/100U;
        if(cap>c->max_compare)cap=c->max_compare;
        PIDSpeedConfig fine={0};
        fine.position_kp=MOTOR_TUNE_TASK_FINE_POSITION_KP;fine.speed_kp=c->speed_kp;fine.speed_ki=c->speed_ki;
        fine.speed_limit=MOTORREF_FINE_SPEED_DPS;fine.accel_limit=MOTORREF_FINE_ACCEL_DPS2;
        fine.max_duty=c->period?(float)cap/(float)c->period:0.0f;
        const float control_error=MotorSequence_Abs(error)-quiet;
        PIDSpeedResult result=PID_SpeedControl(control_error,f->speed[i]*(float)direction,
            a->speed_ref_dps,a->integral,dt,&fine);
        a->speed_ref_dps=result.speed_ref;a->integral=result.integral;
        a->compare=(uint32_t)(result.duty*(float)c->period+0.5f);
        if(a->compare>cap)a->compare=cap;
        s->hold_command[i]=direction*(int32_t)a->compare;
    }
}
#endif

MOTORREF_FN void MotorReference_Hold(MotorReferenceState *h, const MotorSequenceConfig *task,
    const MotorSequenceFeedback *f, uint32_t now, uint32_t check_guard)
{
    for (uint32_t i = 0U; i < 2U; ++i)
    {
        uint32_t moving = h->phase == MOTORREF_ALIGN && h->moving_axis == i+1U && !h->leg_settling;
        float e = moving ? 0.0f : h->hold_target_deg[i] - f->position[i];
        if (!moving && check_guard && !MotorReference_Guard(h, task, i, e, now,
            h->clearance_waiting || (h->leg_settling && h->moving_axis == i+1U) ? 100U : 20U)) return;
        h->ready_hold.axis[i].error_deg = e;
    }
    MotorSequenceConfig c = *task; c.hold_enabled = 1U;
    h->ready_hold.phase = MOTORSEQ_PAUSE;
#if MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_MEASURED_TASK
    if(task->braking_accel_dps2>0.0f&&task->arrival_tolerance_deg>0.0f)
    {MotorReference_TaskHold(h,&c,f);return;}
#endif
    MotorSequence_Hold(&h->ready_hold, &c, f);
    if (h->ready_hold.fault) MotorReference_Fail(h, 1U, 0U, 1U);
}

MOTORREF_FN void MotorReference_SetReady(MotorReferenceState *h,
    const MotorSequenceConfig *task, const MotorSequenceFeedback *f, uint32_t now)
{
    h->phase = MOTORREF_READY; h->return_stage = 4U; h->moving_axis = 0U;
    h->return_parked_axis = 0U; h->recovery_waiting = 0U;
    h->leg_settling = 0U; h->fine_settling = 0U;
    h->ready_outside_active = 0U; h->ready_outside_ms = 0U;
    h->ready = 1U;
    for (uint32_t i = 0U; i < 2U; ++i)
    { h->hold_target_deg[i] = h->target_deg[i]; h->park_outside_active[i] = 0U; }
    MotorReference_Hold(h, task, f, now, 0U);
}

MOTORREF_FN void MotorReference_StartLeg(MotorReferenceState *h, const MotorSequenceConfig *task,
    const MotorSequenceFeedback *f, uint32_t now)
{
    uint32_t next_axis = h->plan.axis[h->plan_step];
    float next_distance = MotorSequence_Abs(MotorReference_Offset(h->raw[next_axis-1U], h->plan.raw[h->plan_step]));
    uint32_t check_clearance = !h->startup_plan && ((task->confirm_axis2_start && next_axis == 2U) ||
        (MOTOR_TUNE_CONFIRM_RECOVERY_HANDOFF && next_distance > MOTORREF_SLOT_ENTRY_DEG));
    if (check_clearance &&
        h->clearance_granted_step != h->plan_step + 1U)
    {
        h->clearance_waiting = 1U; h->clearance_requested = 0U;
        h->clearance_release_seen = 0U; h->clearance_started_ms = now;
        h->moving_axis = 0U; h->return_parked_axis = 3U-next_axis;
        h->leg_settling = 1U; h->return_stage = 2U;
        return;
    }
    uint32_t i = h->plan.axis[h->plan_step] - 1U;
    if (h->startup_plan)
    {
        const uint32_t parked = 1U - i;
        const float offset = MotorReference_Offset(h->raw[parked], h->plan.park_raw[h->plan_step]);
        if (MotorSequence_Abs(offset) > task->tolerance_deg)
        { MotorReference_Fail(h, 5U, parked+1U, 4U); return; }
        h->hold_target_deg[parked] = f->position[parked] + offset * (float)MotorReference_FeedbackSign(parked);
    }
    float distance = MotorReference_Offset(h->raw[i], h->plan.raw[h->plan_step]) * (float)MotorReference_FeedbackSign(i);
    h->moving_axis = i+1U; h->return_parked_axis = 2U-i;
    h->direction[i] = distance < 0.0f ? -1 : 1;
    h->waypoint_target_deg = f->position[i] + distance;
    h->waypoint_error_deg = distance;
    h->leg_started_ms = now; h->leg_elapsed_ms = 0U;
    h->leg_settling = 0U; h->leg_settle_ms = 0U; h->leg_stable = 0U; h->fine_settling = 0U;
    h->handoff_window_active = 0U; h->handoff_samples = 0U;
    for (uint32_t j = 0U; j < 2U; ++j)
    { h->handoff_speed_dps[j] = 0.0f; h->handoff_span_deg[j] = 0.0f; }
    h->return_stage = h->plan.kind[h->plan_step] == MOTORREF_ENTRY ? 1U :
        (h->plan_step+1U == h->plan.count ? 3U : 2U);
    MotorSequenceConfig c = MotorReference_LegConfig(h, task, distance);
    h->leg_timeout_ms = c.timeout_ms;
    MotorSequenceState fresh = {0}; h->motion[i] = fresh;
    MotorSequenceFeedback local = {{1U, 1U}, {f->position[i] * (float)h->direction[i], 0.0f},
        {f->speed[i] * (float)h->direction[i], 0.0f}, {f->dt_s[i], 0.001f}};
    MotorSequence_Update(&h->motion[i], &c, &local, now, 1U, 1U);
#if MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_MEASURED_TASK
    /* Only a forward task leg starting at low residual speed. Alignment
     * has no braking/arrival window; fine/reverse/recovery legs are excluded.
     * BeginStep clears the PI, so seed AFTER that initialization. Subsequent
     * polls use the ordinary speed PI and its existing anti-windup/caps. */
    if ((i == 1U || MOTOR_TUNE_TASK_P2_START_DUTY_PCT) &&
        h->startup_plan && distance > MOTORREF_SLOT_ENTRY_DEG &&
        task->braking_accel_dps2 > 0.0f && task->arrival_window_ms &&
        task->arrival_speed_dps > 0.0f && f->good[0] && f->good[1] &&
        MotorSequence_Abs(f->speed[i]) <= task->arrival_speed_dps &&
        !h->fault && h->motion[i].phase == MOTORSEQ_MOVE && c.period)
    {
        const float cap = (float)c.max_compare / (float)c.period;
        h->motion[i].axis[0].integral = MotorSequence_Clamp(
            (float)(i==1U?MOTORREF_P4_START_DUTY_PCT:MOTOR_TUNE_TASK_P2_START_DUTY_PCT) * 0.01f, 0.0f, cap);
    }
#endif
}

MOTORREF_FN void MotorReference_BeginPlan(MotorReferenceState *h, const MotorSequenceConfig *task,
    const MotorSequenceFeedback *f, const MotorReferencePlan *plan, uint32_t now, uint32_t small)
{
    h->plan = *plan; h->plan_step = 0U; h->ready = 0U; h->recovery_waiting = 0U;
    h->clearance_waiting = 0U; h->clearance_granted_step = 0U;
    h->recovery_started_ms = now; h->recovery_elapsed_ms = 0U;
    h->recovery_deadline_ms = h->startup_plan ? 45000U * plan->count : (small ? 15000U : 60000U);
    h->phase = MOTORREF_ALIGN;
    for (uint32_t i = 0U; i < 2U; ++i)
    { h->hold_target_deg[i] = f->position[i]; h->park_outside_active[i] = 0U; h->park_outside_ms[i] = 0U; }
    if (!h->plan.count) { MotorReference_SetReady(h, task, f, now); return; }
    MotorReference_StartLeg(h, task, f, now);
}

MOTORREF_FN void MotorReference_RequestRecovery(MotorReferenceState *h, uint32_t press)
{
    if (h->phase == MOTORREF_BOOT && h->recovery_waiting && press) h->recovery_requested = 1U;
    else if (h->phase == MOTORREF_ALIGN && h->clearance_waiting && press) h->clearance_requested = 1U;
}

/* Use unwrapped physical encoder positions over a short observation window.
 * Net speed alone can cancel during oscillation: also bound the full excursion
 * and the existing filtered encoder speed. A missed polling interval invalidates
 * the window; two stale endpoint samples must never prove that a disk stopped. */
MOTORREF_FN uint32_t MotorReference_HandoffReady(MotorReferenceState *h,
    const MotorSequenceConfig *task, const MotorSequenceFeedback *f, uint32_t now)
{
    if (!(task->arrival_speed_dps > 0.0f && task->arrival_span_deg > 0.0f)) return 0U;
    if (!h->handoff_window_active || (uint32_t)(now-h->handoff_last_sample_ms) > 5U)
    {
        h->handoff_window_active = 1U; h->handoff_window_started_ms = now;
        h->handoff_last_sample_ms = now; h->handoff_samples = 1U;
        for (uint32_t j = 0U; j < 2U; ++j)
        {
            h->handoff_anchor_deg[j] = f->position[j];
            h->handoff_min_deg[j] = f->position[j]; h->handoff_max_deg[j] = f->position[j];
            h->handoff_speed_dps[j] = 0.0f; h->handoff_span_deg[j] = 0.0f;
        }
        return 0U;
    }
    if (now != h->handoff_last_sample_ms) ++h->handoff_samples;
    h->handoff_last_sample_ms = now;
    for (uint32_t j = 0U; j < 2U; ++j)
    {
        if (f->position[j] < h->handoff_min_deg[j]) h->handoff_min_deg[j] = f->position[j];
        if (f->position[j] > h->handoff_max_deg[j]) h->handoff_max_deg[j] = f->position[j];
    }
    const uint32_t elapsed = (uint32_t)(now-h->handoff_window_started_ms);
    if (elapsed < task->arrival_window_ms || h->handoff_samples < 3U) return 0U;
    uint32_t ready = 1U;
    for (uint32_t j = 0U; j < 2U; ++j)
    {
        h->handoff_speed_dps[j] = (f->position[j]-h->handoff_anchor_deg[j]) * 1000.0f / (float)elapsed;
        h->handoff_span_deg[j] = h->handoff_max_deg[j]-h->handoff_min_deg[j];
        if (!(MotorSequence_Abs(h->handoff_speed_dps[j]) <= task->arrival_speed_dps &&
              MotorSequence_Abs(f->speed[j]) <= task->arrival_speed_dps &&
              h->handoff_span_deg[j] <= task->arrival_span_deg)) ready = 0U;
    }
    if (!ready) h->handoff_window_active = 0U;
    return ready;
}

MOTORREF_FN void MotorReference_Update(MotorReferenceState *h, const MotorSequenceConfig *task,
    const MotorSequenceFeedback *f, uint32_t raw0, uint32_t raw1, uint32_t now,
    uint32_t preflight, uint32_t raw_button_pressed)
{
    if (h->phase == MOTORREF_FAULT || h->phase == MOTORREF_PARTIAL_DONE) return;
    if (h->phase == MOTORREF_BOOT && (!preflight || !f->good[0] || !f->good[1] || raw_button_pressed))
    { h->recovery_release_seen = 0U; return; }
    for (uint32_t i = 0U; i < 2U; ++i)
        if (!f->good[i]) { MotorReference_Fail(h, 1U, i+1U, 1U); return; }
    MotorReference_ReadPose(h, task, f, raw0, raw1);

    if (h->phase == MOTORREF_BOOT)
    {
        if (h->recovery_requested)
        {
            if (!h->recovery_release_seen)
            { h->recovery_release_seen = 1U; h->recovery_release_started_ms = now; return; }
            if ((uint32_t)(now-h->recovery_release_started_ms) < 20U) return;
        }
        uint32_t at_home = MotorSequence_Abs(h->error_deg[0]) <= task->tolerance_deg &&
                           MotorSequence_Abs(h->error_deg[1]) <= task->tolerance_deg;
        if (at_home) { MotorReference_SetReady(h, task, f, now); return; }
        MotorReferencePlan plan = MotorReference_PlanReturn(task, raw0, raw1);
        h->plan = plan;
        if (!plan.valid) { MotorReference_Fail(h, 7U, 0U, 6U); return; }
        uint32_t small = MotorSequence_Abs(h->error_deg[0]) <= 2.0f && MotorSequence_Abs(h->error_deg[1]) <= 2.0f;
        if (!small && !h->recovery_requested) { h->recovery_waiting = 1U; return; }
        MotorReference_BeginPlan(h, task, f, &plan, now, small);
        return;
    }

    if (h->phase == MOTORREF_READY)
    {
        h->ready = 1U;
        uint32_t outside_axis = 0U;
        for (uint32_t i = 0U; i < 2U; ++i)
        {
            h->hold_target_deg[i] = h->target_deg[i];
            float e = MotorSequence_Abs(h->error_deg[i]);
            if (e > task->hold_tolerance_deg) { MotorReference_Fail(h, 5U, i+1U, 4U); return; }
            if (e > task->tolerance_deg) { h->ready = 0U; outside_axis = i+1U; }
        }
        MotorReference_Hold(h, task, f, now, 0U);
        if (h->phase == MOTORREF_FAULT) return;
        if (h->ready) { h->ready_outside_active = 0U; h->ready_outside_ms = 0U; return; }
        if (!h->ready_outside_active)
        { h->ready_outside_active = 1U; h->ready_outside_started_ms = now; }
        h->ready_outside_ms = (uint32_t)(now-h->ready_outside_started_ms);
        if (h->ready_outside_ms < 100U) return;
        if (h->fine_retry_count) { MotorReference_Fail(h, 5U, outside_axis, 5U); return; }
        if (!preflight || raw_button_pressed) return;
        MotorReferencePlan plan = MotorReference_PlanReturn(task, raw0, raw1);
        if (!plan.valid) { h->plan = plan; MotorReference_Fail(h, 7U, 0U, 6U); return; }
        h->fine_retry_count = 1U;
        MotorReference_BeginPlan(h, task, f, &plan, now, 1U);
        return;
    }

    if (h->clearance_waiting)
    {
        MotorReference_Hold(h, task, f, now, 1U);
        if (h->phase == MOTORREF_FAULT) return;
        if ((uint32_t)(now-h->clearance_started_ms) >= 120000U)
        { MotorReference_Fail(h, 2U, h->plan.axis[h->plan_step], 8U); return; }
        if (raw_button_pressed) { h->clearance_release_seen = 0U; return; }
        if (!h->clearance_requested) return;
        if (!h->clearance_release_seen)
        { h->clearance_release_seen = 1U; h->clearance_release_started_ms = now; return; }
        if ((uint32_t)(now-h->clearance_release_started_ms) < 20U) return;
        for (uint32_t j=0U; j<2U; ++j)
            if (MotorSequence_Abs(h->hold_target_deg[j]-f->position[j]) > task->tolerance_deg) return;
        h->recovery_started_ms += (uint32_t)(now-h->clearance_started_ms);
        h->clearance_granted_step = h->plan_step+1U; h->clearance_waiting = 0U;
        MotorReference_StartLeg(h, task, f, now);
        return;
    }
    if (raw_button_pressed) { MotorReference_Fail(h, 6U, h->moving_axis, 7U); return; }
    h->recovery_elapsed_ms = (uint32_t)(now-h->recovery_started_ms);
    h->leg_elapsed_ms = (uint32_t)(now-h->leg_started_ms);
    if (h->recovery_elapsed_ms >= h->recovery_deadline_ms || h->leg_elapsed_ms >= h->leg_timeout_ms)
    { MotorReference_Fail(h, 2U, h->moving_axis, 2U); return; }
    uint32_t i = h->moving_axis-1U;
    h->waypoint_error_deg = h->waypoint_target_deg-f->position[i];
    const float arrival_tolerance = task->arrival_tolerance_deg > 0.0f ?
        task->arrival_tolerance_deg : task->tolerance_deg;
    if (!h->leg_settling)
    {
        float remaining = h->waypoint_error_deg*(float)h->direction[i];
        if (remaining <= arrival_tolerance*0.5f)
        {
            if (remaining < -task->hold_tolerance_deg) { MotorReference_Fail(h, 3U, i+1U, 3U); return; }
            MotorSequence_Brake(&h->motion[i]); h->motion[i].phase = MOTORSEQ_DONE;
            h->hold_target_deg[i] = h->waypoint_target_deg;
            h->leg_settling = 1U; h->leg_settle_started_ms = now;
        }
        else
        {
            MotorSequenceConfig c = MotorReference_LegConfig(h, task,
                h->motion[i].axis[0].target_deg-h->motion[i].axis[0].origin_deg);
            MotorSequenceFeedback local = {{1U, 1U}, {f->position[i]*(float)h->direction[i], 0.0f},
                {f->speed[i]*(float)h->direction[i], 0.0f}, {f->dt_s[i], 0.001f}};
            MotorSequence_Update(&h->motion[i], &c, &local, now, 1U, 0U);
            if (h->motion[i].fault) { MotorReference_Fail(h, h->motion[i].fault, i+1U, 3U); return; }
        }
    }
    h->fine_settling = h->leg_settling;
    MotorReference_Hold(h, task, f, now, 1U);
    if (h->phase == MOTORREF_FAULT) return;
    uint32_t handoff_ready = 0U;
    if (task->arrival_window_ms)
    {
        /* Observe the final low-speed approach before entering arrival hold.
         * With current limits this begins at 0.75deg, but the existing 0.5deg
         * arrival entry and both-axis motion checks still decide completion.
         * A completed observation can therefore hand off on the arrival tick
         * instead of starting another 20ms window after reaching the target. */
        float approach_band = arrival_tolerance*0.5f + task->arrival_span_deg*0.5f;
        if (approach_band > arrival_tolerance) approach_band = arrival_tolerance;
        if (h->leg_settling || MotorSequence_Abs(h->waypoint_error_deg) <= approach_band)
            handoff_ready = MotorReference_HandoffReady(h, task, f, now);
        else
        { h->handoff_window_active = 0U; h->handoff_samples = 0U; }
    }
    if (!h->leg_settling) return;
    uint32_t stable = 1U;
    for (uint32_t j = 0U; j < 2U; ++j)
    {
        const float tolerance = j == i ? arrival_tolerance : task->tolerance_deg;
        if (MotorSequence_Abs(h->hold_target_deg[j]-f->position[j]) > tolerance) stable = 0U;
    }
    if (!stable)
    {
        h->leg_stable = 0U; h->leg_settle_ms = 0U;
        h->handoff_window_active = 0U; h->handoff_samples = 0U; return;
    }
    if (!h->leg_stable) { h->leg_stable = 1U; h->leg_settle_started_ms = now; }
    h->leg_settle_ms = (uint32_t)(now-h->leg_settle_started_ms);
    h->fine_settle_ms = h->leg_settle_ms;
    if (task->arrival_window_ms)
    {
        if (!handoff_ready) return;
    }
    else
    {
        const uint32_t arrival_hold_ms = task->arrival_hold_ms ? task->arrival_hold_ms : 200U;
        if (h->leg_settle_ms < arrival_hold_ms) return;
    }
    h->plan_step++;
    if (h->plan_step >= h->plan.count)
    {
        if (h->plan.partial)
        {
            h->phase = MOTORREF_PARTIAL_DONE;
            h->moving_axis = 0U;
            h->return_parked_axis = 0U;
            h->ready = 0U;
        }
        else MotorReference_SetReady(h, task, f, now);
    }
    else MotorReference_StartLeg(h, task, f, now);
}

MOTORREF_FN uint32_t MotorReference_ChannelCompare(const MotorReferenceState *h,
    uint32_t axis, uint32_t negative_channel)
{
    if (axis >= 2U || h->fault || h->phase == MOTORREF_BOOT || h->phase == MOTORREF_FAULT ||
        h->phase == MOTORREF_PARTIAL_DONE) return 0U;
    if (h->phase == MOTORREF_ALIGN && h->moving_axis == axis+1U && !h->leg_settling)
        return MotorSequence_ChannelCompare(&h->motion[axis], 0U,
            MotorReference_PwmSign(axis)*h->direction[axis], negative_channel);
    return MotorSequence_ChannelCompare(&h->ready_hold, axis, MotorReference_PwmSign(axis), negative_channel);
}

#undef MOTORREF_FN
#endif


/* ==================== motor_task_config.h ==================== */
#ifndef MOTOR_TASK_CONFIG_H
#define MOTOR_TASK_CONFIG_H


/* Verified base controller settings shared by initial alignment and the
 * measured-slot task. Task-specific limits are applied by motor_measured_slots.h. */
#ifdef __cplusplus
#define TASK_CONFIG_FN static constexpr
#else
#define TASK_CONFIG_FN static inline
#endif
TASK_CONFIG_FN MotorSequenceConfig MotorTask_BaseConfig(void)
{
    MotorSequenceConfig c={0};
    c.step_deg=90.0f;c.period=2100U;c.max_compare=840U;c.hold_enabled=1U;
    c.tolerance_deg=0.5f;c.hold_tolerance_deg=0.75f;c.speed_tolerance_dps=5.0f;
    c.speed_limit_dps=5.0f;c.accel_limit_dps2=15.0f;
    c.position_kp=4.0f;c.position_kd=0.05f;c.speed_kp=0.0015f;c.speed_ki=0.015f;
    c.timeout_ms=35000U;c.settle_ms=300U;c.rounds=1U;c.max_steps=1U;
    c.use_fixed_origin=1U;
    return c;
}
#undef TASK_CONFIG_FN
#endif


/* ==================== motor_measured_slots.h ==================== */
#ifndef MOTOR_MEASURED_SLOTS_H
#define MOTOR_MEASURED_SLOTS_H




typedef struct {
    uint32_t phase, slot_index[2], next_axis, steps_done, pause_started_ms, aligned;
    float current_speed_limit_dps;
    /* Normal B1 stop keeps the unfinished goal and never clears a real fault. */
    uint32_t resume_phase, resume_axis, resume_step, resume_requested, ramp_origin_steps;
    float resume_start_deg, resume_target_deg;
} MotorMeasuredSlotsState;

#ifdef __cplusplus
#define MEASURED_SLOTS_FN static constexpr
#else
#define MEASURED_SLOTS_FN static inline
#endif

MEASURED_SLOTS_FN uint32_t MotorMeasuredSlots_Raw(uint32_t axis, uint32_t slot)
{
    return MOTOR_SLOT_TARGET_RAW[axis][slot];
}

MEASURED_SLOTS_FN MotorSequenceConfig MotorMeasuredSlots_Config(void)
{
    MotorSequenceConfig config = MotorTask_BaseConfig();
    config.tolerance_deg = 7.0f;
    config.hold_tolerance_deg = 7.0f;
    config.parked_tolerance_deg = 7.0f;
    config.arrival_tolerance_deg = 1.0f;
    config.speed_limit_dps = MOTOR_SLOT_START_SPEED_DPS;
    config.max_speed_dps = MOTOR_SLOT_MAX_SPEED_DPS;
    config.speed_increment_dps = MOTOR_SLOT_SPEED_INCREMENT_DPS;
    config.ramp_steps = MOTOR_SLOT_SPEEDUP_EVERY_MOVES;
    config.pause_ms = 0U; /* No extra delay after arrival confirmation. */
    config.timeout_ms = 45000U;
    return config;
}

MEASURED_SLOTS_FN uint32_t MotorMeasuredSlots_Find(uint32_t axis, uint32_t raw, float tolerance)
{
    for (uint32_t slot = 0U; slot < 4U; ++slot)
        if (MotorSequence_Abs(MotorReference_Offset(raw, MotorMeasuredSlots_Raw(axis, slot))) <= tolerance)
            return slot;
    return 4U;
}

MEASURED_SLOTS_FN void MotorMeasuredSlots_ReadPose(const MotorMeasuredSlotsState *task,
    MotorReferenceState *pose, const MotorSequenceFeedback *feedback, uint32_t raw1, uint32_t raw2)
{
    pose->raw[0] = raw1 & 0x3FFFU;
    pose->raw[1] = raw2 & 0x3FFFU;
    for (uint32_t axis = 0U; axis < 2U; ++axis)
    {
        pose->error_deg[axis] = MotorReference_Offset(pose->raw[axis],
            MotorMeasuredSlots_Raw(axis, task->slot_index[axis])) * (float)MotorReference_FeedbackSign(axis);
        pose->target_deg[axis] = feedback->position[axis] + pose->error_deg[axis];
    }
}

MEASURED_SLOTS_FN uint32_t MotorMeasuredSlots_Nearest(uint32_t axis, uint32_t raw)
{
    uint32_t nearest = 0U;
    float smallest_error = 181.0f;
    for (uint32_t slot = 0U; slot < 4U; ++slot)
    {
        const float error = MotorSequence_Abs(MotorReference_Offset(raw, MotorMeasuredSlots_Raw(axis, slot)));
        if (error < smallest_error) { smallest_error = error; nearest = slot; }
    }
    return nearest;
}

MEASURED_SLOTS_FN MotorReferencePlan MotorMeasuredSlots_AlignmentPlan(const MotorMeasuredSlotsState *task,
    uint32_t raw1, uint32_t raw2, float tolerance)
{
    MotorReferencePlan plan = {0};
    uint32_t current[2] = {raw1 & 0x3FFFU, raw2 & 0x3FFFU};
    const uint32_t targets[2] = {MotorMeasuredSlots_Raw(0U, task->slot_index[0]),
        MotorMeasuredSlots_Raw(1U, task->slot_index[1])};
    const float error1 = MotorSequence_Abs(MotorReference_Offset(raw1, targets[0]));
    const float error2 = MotorSequence_Abs(MotorReference_Offset(raw2, targets[1]));
    const uint32_t first_axis = error1 <= error2 ? 0U : 1U;
    plan.valid = 1U;
    plan.partial = 1U;
    for (uint32_t leg = 0U; leg < 2U; ++leg)
    {
        const uint32_t axis = leg == 0U ? first_axis : 1U - first_axis;
        if (MotorSequence_Abs(MotorReference_Offset(current[axis], targets[axis])) <= tolerance) continue;
        plan.park_raw[plan.count] = current[1U - axis];
        MotorReference_Append(&plan, current, axis, targets[axis], MOTORREF_HOME);
    }
    plan.initial_parked_axis = plan.count ? 3U - plan.axis[0] : 0U;
    return plan;
}

MEASURED_SLOTS_FN MotorSequenceConfig MotorMeasuredSlots_AlignmentConfig(const MotorSequenceConfig *base)
{
    MotorSequenceConfig c=*base;
    c.speed_limit_dps=MOTOR_SLOT_ALIGNMENT_SPEED_DPS;
    return c;
}

MEASURED_SLOTS_FN void MotorMeasuredSlots_Align(MotorMeasuredSlotsState *task, MotorReferenceState *pose,
    const MotorSequenceConfig *base_config, const MotorSequenceFeedback *feedback,
    uint32_t raw1, uint32_t raw2, uint32_t now, uint32_t preflight, uint32_t press)
{
    MotorSequenceConfig config = MotorMeasuredSlots_AlignmentConfig(base_config);

    pose->ready = 0U;
    if (pose->phase == MOTORREF_ALIGN)
    {
        MotorReference_Update(pose, &config, feedback, raw1, raw2, now, preflight, press);
        MotorMeasuredSlots_ReadPose(task, pose, feedback, raw1, raw2);
        if (pose->fault) return;
    }
    else
    {
        if (!preflight || !feedback->good[0] || !feedback->good[1]) return;
        task->slot_index[0] = MotorMeasuredSlots_Nearest(0U, raw1);
        task->slot_index[1] = MotorMeasuredSlots_Nearest(1U, raw2);
        MotorMeasuredSlots_ReadPose(task, pose, feedback, raw1, raw2);
        pose->plan = MotorMeasuredSlots_AlignmentPlan(task, raw1, raw2, config.arrival_tolerance_deg);
        pose->ready = 1U;
        pose->recovery_waiting = 1U;
        if (!press) return;
        pose->startup_plan = 1U;
        MotorReference_BeginPlan(pose, &config, feedback, &pose->plan, now, 0U);
        MotorMeasuredSlots_ReadPose(task, pose, feedback, raw1, raw2);
        if (pose->fault) return;
    }
    if (pose->phase == MOTORREF_PARTIAL_DONE || pose->phase == MOTORREF_READY)
    {
        task->aligned = 1U;
        pose->phase = MOTORREF_READY;
        pose->ready = 0U;
        for (uint32_t axis = 0U; axis < 2U; ++axis)
            pose->hold_target_deg[axis] = pose->target_deg[axis];
        MotorReference_Hold(pose, base_config, feedback, now, 1U);
    }
}

MEASURED_SLOTS_FN MotorSequenceConfig MotorMeasuredSlots_TaskConfig(const MotorMeasuredSlotsState *task,
    const MotorSequenceConfig *base_config)
{
    MotorSequenceConfig config = *base_config;
    const uint32_t level = MOTOR_SLOT_STEP_BY_BUTTON == 0U && config.ramp_steps ?
        (task->steps_done-task->ramp_origin_steps) / config.ramp_steps : 0U;
    const float ceiling = config.max_speed_dps > config.speed_limit_dps ?
        config.max_speed_dps : config.speed_limit_dps;
    config.speed_limit_dps = MotorSequence_Clamp(config.speed_limit_dps +
        (float)level * config.speed_increment_dps, config.speed_limit_dps, ceiling);
    config.accel_limit_dps2 = MOTOR_SLOT_TASK_ACCEL_DPS2;
    config.braking_accel_dps2 = MOTOR_SLOT_TASK_BRAKE_DPS2;
    config.arrival_window_ms = MOTOR_SLOT_TASK_HANDOFF_WINDOW_MS;
    config.arrival_speed_dps = MOTOR_SLOT_TASK_HANDOFF_SPEED_DPS;
    config.arrival_span_deg = MOTOR_SLOT_TASK_HANDOFF_SPAN_DEG;
    config.max_compare = config.period * (task->next_axis == 0U ?
        MOTOR_TUNE_TASK_P2_DUTY_PCT : MOTOR_TUNE_TASK_P4_DUTY_PCT) / 100U;
    return config;
}

MEASURED_SLOTS_FN void MotorMeasuredSlots_Begin(MotorMeasuredSlotsState *task, MotorReferenceState *pose,
    const MotorSequenceConfig *base_config, const MotorSequenceFeedback *feedback, uint32_t now)
{
    const MotorSequenceConfig config = MotorMeasuredSlots_TaskConfig(task, base_config);
    const uint32_t axis = task->next_axis;
    const uint32_t next_slot = (task->slot_index[axis] + 1U) % 4U;
    MotorReferencePlan plan = {0};
    uint32_t current[2] = {pose->raw[0], pose->raw[1]};
    plan.valid = 1U;
    plan.partial = 1U;
    plan.initial_parked_axis = 2U - axis;
    plan.park_raw[0] = MotorMeasuredSlots_Raw(1U - axis, task->slot_index[1U - axis]);
    MotorReference_Append(&plan, current, axis, MotorMeasuredSlots_Raw(axis, next_slot), MOTORREF_WAYPOINT);
    pose->startup_plan = 1U;
    MotorReference_BeginPlan(pose, &config, feedback, &plan, now, 0U);
    task->slot_index[axis] = next_slot;
    task->phase = MOTORSEQ_MOVE;
    task->current_speed_limit_dps = config.speed_limit_dps;
    pose->ready = 0U;
}

MEASURED_SLOTS_FN uint32_t MotorMeasuredSlots_WaitForButton(const MotorMeasuredSlotsState *task)
{
    return MOTOR_SLOT_STEP_BY_BUTTON == 1U ||
        (MOTOR_SLOT_STEP_BY_BUTTON == 2U && task->steps_done % 2U == 0U);
}

#if MOTOR_FAST_TRIAL
MEASURED_SLOTS_FN void MotorMeasuredSlots_OutputsZero(MotorReferenceState *pose)
{
    for(uint32_t j=0U;j<3U;++j)
    {
        MotorSequenceState *s=j<2U?&pose->motion[j]:&pose->ready_hold;
        MotorSequence_Brake(s);s->phase=MOTORSEQ_DONE;
        for(uint32_t a=0U;a<2U;++a)
        {
            s->hold_command[a]=0;s->hold_integral[a]=0.0f;
            s->hold_polarity[a]=0;s->hold_blank[a]=0U;
        }
    }
}

MEASURED_SLOTS_FN void MotorMeasuredSlots_Stop(MotorMeasuredSlotsState *task,
    MotorReferenceState *pose)
{
    task->resume_phase=task->phase;task->resume_axis=pose->moving_axis;
    task->resume_step=pose->plan_step;task->resume_requested=0U;
    if(task->resume_axis>=1U&&task->resume_axis<=2U)
    {
        const uint32_t a=task->resume_axis-1U;
        task->resume_start_deg=pose->motion[a].axis[0].origin_deg*(float)pose->direction[a];
        task->resume_target_deg=pose->waypoint_target_deg;
    }
    MotorMeasuredSlots_OutputsZero(pose);
    task->phase=MOTORSEQ_STOPPED;pose->phase=MOTORREF_READY;
    pose->moving_axis=0U;pose->return_parked_axis=0U;pose->ready=0U;
    pose->handoff_window_active=0U;pose->handoff_samples=0U;
    pose->leg_settling=0U;pose->fine_settling=0U;
    for(uint32_t a=0U;a<2U;++a)pose->park_outside_active[a]=0U;
}

/* Called only while normally stopped. A resume press may wait for measured
 * standstill, but a blocked preflight press is never queued. No PWM holding
 * runs during the stop or this pending window. */
MEASURED_SLOTS_FN void MotorMeasuredSlots_Stopped(MotorMeasuredSlotsState *task,
    MotorReferenceState *pose,const MotorSequenceConfig *base,const MotorSequenceFeedback *f,
    uint32_t raw1,uint32_t raw2,uint32_t now,uint32_t preflight,uint32_t press)
{
    MotorMeasuredSlots_OutputsZero(pose);
    MotorMeasuredSlots_ReadPose(task,pose,f,raw1,raw2);
    pose->ready=preflight;
    for(uint32_t a=0U;a<2U;++a)
        if(!f->good[a]||!(f->dt_s[a]>0.0f&&f->dt_s[a]<=0.005f))
        {MotorReference_Fail(pose,1U,a+1U,1U);task->phase=MOTORSEQ_FAULT;return;}
    if(press)
    {
        task->resume_requested=preflight&&!task->resume_requested;
        pose->handoff_window_active=0U;pose->handoff_samples=0U;
    }
    if(!preflight)
    {task->resume_requested=0U;pose->handoff_window_active=0U;return;}
    if(!task->resume_requested)return;
    pose->ready=0U;
    const MotorSequenceConfig motion=MotorMeasuredSlots_TaskConfig(task,base);
    if(!MotorReference_HandoffReady(pose,&motion,f,now))return;
    if(task->resume_axis)
    {
        const uint32_t a=task->resume_axis-1U;
        if(a>=2U||task->resume_step>=pose->plan.count)
        {MotorReference_Fail(pose,7U,0U,6U);task->phase=MOTORSEQ_FAULT;return;}
        const float low=task->resume_start_deg<task->resume_target_deg?task->resume_start_deg:task->resume_target_deg;
        const float high=task->resume_start_deg>task->resume_target_deg?task->resume_start_deg:task->resume_target_deg;
        const float parked=MotorSequence_Abs(MotorReference_Offset(pose->raw[1U-a],pose->plan.park_raw[task->resume_step]));
        if(parked>base->tolerance_deg||f->position[a]<low-base->hold_tolerance_deg||f->position[a]>high+base->hold_tolerance_deg)
        {MotorReference_Fail(pose,5U,parked>base->tolerance_deg?2U-a:a+1U,9U);task->phase=MOTORSEQ_FAULT;return;}
        MotorReferencePlan rest=pose->plan;
        rest.count-=task->resume_step;
        for(uint32_t j=0U;j<rest.count;++j)
        {
            rest.axis[j]=pose->plan.axis[j+task->resume_step];rest.raw[j]=pose->plan.raw[j+task->resume_step];
            rest.kind[j]=pose->plan.kind[j+task->resume_step];rest.park_raw[j]=pose->plan.park_raw[j+task->resume_step];
        }
        /* Keep a zero-distance current goal too: it still needs arrival
         * confirmation and must count exactly once, never skip a slot. */
        task->ramp_origin_steps=task->steps_done;
        const MotorSequenceConfig c=task->resume_phase==MOTORSEQ_WAIT?
            MotorMeasuredSlots_AlignmentConfig(base):MotorMeasuredSlots_TaskConfig(task,base);
        pose->startup_plan=1U;
        MotorReference_BeginPlan(pose,&c,f,&rest,now,0U);
        if(pose->fault){task->phase=MOTORSEQ_FAULT;return;}
        task->phase=task->resume_phase;
        if(task->phase==MOTORSEQ_MOVE)task->current_speed_limit_dps=c.speed_limit_dps;
    }
    else
    {
        /* Stop between legs: retain next_axis/slot indexes; the normal
         * departure path rechecks both target positions before starting. */
        for(uint32_t a=0U;a<2U;++a)
            if(MotorSequence_Abs(pose->error_deg[a])>base->tolerance_deg)
            {MotorReference_Fail(pose,5U,a+1U,9U);task->phase=MOTORSEQ_FAULT;return;}
        task->ramp_origin_steps=task->steps_done;
        task->phase=MOTORSEQ_PAUSE;task->pause_started_ms=now;
        task->current_speed_limit_dps=base->speed_limit_dps;
        pose->phase=MOTORREF_READY;
        for(uint32_t a=0U;a<2U;++a)pose->hold_target_deg[a]=pose->target_deg[a];
    }
    task->resume_requested=0U;
}
#endif

MEASURED_SLOTS_FN void MotorMeasuredSlots_Update(MotorMeasuredSlotsState *task, MotorReferenceState *pose,
    const MotorSequenceConfig *base_config, const MotorSequenceFeedback *feedback,
    uint32_t raw1, uint32_t raw2, uint32_t now, uint32_t preflight, uint32_t press)
{
    if (pose->fault) { task->phase = MOTORSEQ_FAULT; return; }
#if MOTOR_FAST_TRIAL
    if(task->phase==MOTORSEQ_STOPPED)
    {
        MotorMeasuredSlots_Stopped(task,pose,base_config,feedback,raw1,raw2,now,preflight,press);
        if(task->phase==MOTORSEQ_STOPPED||task->phase==MOTORSEQ_FAULT||task->phase==MOTORSEQ_MOVE||task->phase==MOTORSEQ_WAIT)return;
        press=0U;
    }
    if(press&&((task->phase==MOTORSEQ_WAIT&&!task->aligned&&pose->phase==MOTORREF_ALIGN)||
        ((task->phase==MOTORSEQ_MOVE||task->phase==MOTORSEQ_PAUSE)&&
         !(MotorMeasuredSlots_WaitForButton(task)&&task->phase==MOTORSEQ_PAUSE))))
    {
        MotorMeasuredSlots_Stop(task,pose);
        MotorMeasuredSlots_ReadPose(task,pose,feedback,raw1,raw2);
        for(uint32_t a=0U;a<2U;++a)
            if(!feedback->good[a]){MotorReference_Fail(pose,1U,a+1U,1U);task->phase=MOTORSEQ_FAULT;return;}
        return;
    }
#endif
    if (task->phase == MOTORSEQ_WAIT)
    {
        if (!task->aligned)
        {
            MotorMeasuredSlots_Align(task, pose, base_config, feedback, raw1, raw2, now, preflight, press);
            return;
        }
        pose->ready = 0U;
        for (uint32_t axis = 0U; axis < 2U; ++axis)
            if (!feedback->good[axis]) { MotorReference_Fail(pose, 1U, axis + 1U, 1U); return; }
        MotorMeasuredSlots_ReadPose(task, pose, feedback, raw1, raw2);
        MotorReference_Hold(pose, base_config, feedback, now, 1U);
        if (pose->fault || !preflight) return;
        for (uint32_t axis = 0U; axis < 2U; ++axis)
            if (MotorSequence_Abs(pose->error_deg[axis]) > base_config->tolerance_deg) return;
        pose->ready = 1U;
        if (!press) return;
    }
    else
    {
        /* At a manual group boundary, a fresh press may start the next group
         * once settled. During movement or between P2 and P4, B1 is a stop.
         * Early presses are discarded, never queued for a later move.
         */
        if (press && !(MotorMeasuredSlots_WaitForButton(task) && task->phase == MOTORSEQ_PAUSE))
        { MotorReference_Fail(pose, 6U, pose->moving_axis, 7U); return; }
        for (uint32_t axis = 0U; axis < 2U; ++axis)
            if (!feedback->good[axis]) { MotorReference_Fail(pose, 1U, axis + 1U, 1U); return; }
    }

    MotorSequenceConfig config = MotorMeasuredSlots_TaskConfig(task, base_config);
    if (task->phase == MOTORSEQ_WAIT)
    {
        MotorMeasuredSlots_Begin(task, pose, base_config, feedback, now);
        MotorMeasuredSlots_ReadPose(task, pose, feedback, raw1, raw2);
        return;
    }

    if (task->phase == MOTORSEQ_MOVE)
    {
        MotorReference_Update(pose, &config, feedback, raw1, raw2, now, preflight, 0U);
        MotorMeasuredSlots_ReadPose(task, pose, feedback, raw1, raw2);
        if (pose->fault) return;
        if (pose->phase != MOTORREF_PARTIAL_DONE) return;
        ++task->steps_done;
        task->next_axis = 1U - task->next_axis;
        task->phase = MOTORSEQ_PAUSE;
        task->pause_started_ms = now;
        pose->phase = MOTORREF_READY;
        pose->ready = 0U;
    }
    else MotorMeasuredSlots_ReadPose(task, pose, feedback, raw1, raw2);

    pose->ready = 0U;
    MotorReference_Hold(pose, &config, feedback, now, 1U);
    if (pose->fault || now - task->pause_started_ms < config.pause_ms) return;
    for (uint32_t axis = 0U; axis < 2U; ++axis)
    {
        if (MotorSequence_Abs(pose->error_deg[axis]) > config.tolerance_deg)
        { MotorReference_Fail(pose, 5U, axis + 1U, 4U); return; }
    }
    const uint32_t arrived_axis = 1U - task->next_axis;
    if (MotorSequence_Abs(pose->error_deg[arrived_axis]) > config.arrival_tolerance_deg)
    {
        pose->handoff_window_active = 0U; pose->handoff_samples = 0U;
        if (now - task->pause_started_ms >= config.timeout_ms)
            MotorReference_Fail(pose, 2U, arrived_axis + 1U, 2U);
        return;
    }
    if (config.arrival_window_ms && !MotorReference_HandoffReady(pose, &config, feedback, now))
    {
        if (now - task->pause_started_ms >= config.timeout_ms)
            MotorReference_Fail(pose, 2U, arrived_axis + 1U, 2U);
        return;
    }
    if (!preflight) return;
    if (MotorMeasuredSlots_WaitForButton(task))
    {
        pose->ready = 1U;
        if (!press) return;
    }
    MotorMeasuredSlots_Begin(task, pose, base_config, feedback, now);
    MotorMeasuredSlots_ReadPose(task, pose, feedback, raw1, raw2);
}

#undef MEASURED_SLOTS_FN
#endif


#endif /* BARAM_MOTOR_CONTROL_H */
