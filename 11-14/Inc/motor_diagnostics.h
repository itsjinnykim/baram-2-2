#ifndef BARAM_MOTOR_DIAGNOSTICS_H
#define BARAM_MOTOR_DIAGNOSTICS_H
#include "motor_control.h"

/* ==================== motor_debug_compat.h ==================== */
#ifndef MOTOR_DEBUG_COMPAT_H
#define MOTOR_DEBUG_COMPAT_H


/* Keep the established Live Expressions types and member layouts. These are
 * status containers, not executable calibration/jog/test modes. The small
 * fault helpers below preserve the existing g_return_repeat fault snapshot. */

/* Status layout from p2_registration.h. */
enum { P2REG_WAIT = 0, P2REG_MOVE = 1, P2REG_DONE = 2, P2REG_FAULT = 3 };
typedef struct
{
    uint32_t ready, done, fault, phase, target_raw, start_raw;
    float remaining_deg, command_deg, delta_deg;
    MotorSequenceState motion;
} P2RegistrationState;

/* Status layout from p4_registration.h. */
enum { P4REG_WAIT = 0, P4REG_MOVE = 1, P4REG_DONE = 2, P4REG_FAULT = 3 };
typedef struct
{
    uint32_t ready, done, fault, phase, target_raw, start_raw;
    float remaining_deg, command_deg, delta_deg;
    MotorSequenceState motion;
} P4RegistrationState;

/* Status layout from p2_step2.h. */
enum { P2TRIAL_WAIT = 0, P2TRIAL_MOVE = 1, P2TRIAL_DONE = 2, P2TRIAL_FAULT = 3 };
typedef struct
{
    uint32_t ready, done, fault, phase, target_raw, start_raw;
    float remaining_deg, command_deg, delta_deg;
    MotorSequenceState motion;
} P2TrialState;

/* Status layout from motor_hold_trial.h. */
enum { HOLDTRIAL_WAIT=0,HOLDTRIAL_HOLD=1,HOLDTRIAL_DONE=2,HOLDTRIAL_FAULT=3,HOLDTRIAL_MOVE=4 };
typedef struct {
    uint32_t phase,ready,done,fault,fault_axis,started_ms,elapsed_ms;
    float error_deg[2],peak_error_deg[2]; /* peak only while that axis is holding */
    MotorSequenceState hold;
    uint32_t hold_started_ms,hold_elapsed_ms;
    float delta_deg,final_move_error_deg,command_deg;
} MotorHoldTrialState;

/* Status layout from motor_return_out.h. */
enum { RETURNOUT_WAIT=0,RETURNOUT_HOLD=1,RETURNOUT_DONE=2,RETURNOUT_FAULT=3,RETURNOUT_MOVE=4 };
typedef struct {
    uint32_t phase,ready,done,fault,fault_axis,started_ms,elapsed_ms;
    float error_deg[2],peak_error_deg[2]; /* peak only while that axis is holding */
    MotorSequenceState hold;
    uint32_t hold_started_ms,hold_elapsed_ms;
    float delta_deg,final_move_error_deg;
    uint32_t task_pass; /* 0: baseline; 1 and 2: forward task legs. */
    float speed_scale;
} MotorReturnOutState;

/* Status layout from motor_return_back.h. */
enum { RETURNBACK_WAIT=0,RETURNBACK_HOLD=1,RETURNBACK_DONE=2,RETURNBACK_FAULT=3,RETURNBACK_MOVE=4 };
typedef struct {
    uint32_t phase,ready,done,fault,fault_axis,started_ms,elapsed_ms;
    float error_deg[2],peak_error_deg[2]; /* peak only while that axis is holding */
    MotorSequenceState hold;
    uint32_t hold_started_ms,hold_elapsed_ms;
    float delta_deg,final_move_error_deg,command_deg;
    uint32_t from_second;
    float speed_scale;
} MotorReturnBackState;

/* Status layout from motor_p2_prepare.h. */
enum { P2PREP_WAIT=0,P2PREP_HOLD=1,P2PREP_DONE=2,P2PREP_FAULT=3,P2PREP_MOVE=4 };
enum { P2_RETURN_HOME=1, P2_RETURN_FROM_SECOND=2 };
typedef struct {
    uint32_t phase,ready,done,fault,fault_axis,started_ms,elapsed_ms;
    float error_deg[2],peak_error_deg[2]; /* peak only while that axis is holding */
    MotorSequenceState hold;
    uint32_t hold_started_ms,hold_elapsed_ms;
    float delta_deg,final_move_error_deg;
    uint32_t returning; /* Zero for a forward leg, otherwise a P2_RETURN_* path. */
    uint32_t task_pass; /* 0: baseline; 1 and 2: forward task legs. */
    float speed_scale;
} MotorP2PrepareState;

/* Status layout from motor_home_align.h. */
enum { HOMEALIGN_WAIT=0,HOMEALIGN_HOLD=1,HOMEALIGN_DONE=2,HOMEALIGN_FAULT=3,HOMEALIGN_MOVE=4 };
typedef struct {
    uint32_t phase,ready,done,fault,fault_axis,started_ms,elapsed_ms;
    float error_deg[2],peak_error_deg[2]; /* peak only while that axis is holding */
    MotorSequenceState hold;
    uint32_t hold_started_ms,hold_elapsed_ms;
    float delta_deg,final_move_error_deg,command_deg;
    int32_t direction;
    uint32_t minimum_hold_ms; /* Zero selects the baseline 10-second observation. */
} MotorHomeAlignState;

/* Status layout from motor_return_repeat.h. */
enum { REPEAT_WAIT_OUT=0,REPEAT_OUT=1,REPEAT_WAIT_BACK=2,REPEAT_BACK=3,
       REPEAT_ROUND_DONE=4,REPEAT_DONE=5,REPEAT_FAULT=6,REPEAT_PREP_WAIT=7,REPEAT_PREP=8,
       REPEAT_HOME_WAIT=9,REPEAT_HOME_ALIGN=10,REPEAT_START=11,
       REPEAT_P2_BACK_WAIT=12,REPEAT_P2_BACK=13,REPEAT_ALTERNATING_MOVE=14,REPEAT_ALTERNATING_PAUSE=15 };
enum { ROUNDTRIP_SUPERVISED=1, ROUNDTRIP_AUTOMATIC=2 };
typedef struct {
    uint32_t phase,ready,fault,fault_axis,completed_returns;
    float delta_deg,error_deg[2],peak_error_deg[2];
    MotorReturnOutState outward;MotorReturnBackState backward;
    MotorP2PrepareState preparation;MotorHomeAlignState initial_align;
    uint32_t waiting_hold_enabled,waiting_fault_latched,waiting_started_ms,waiting_elapsed_ms;
    uint32_t waiting_target_raw[2];
    float waiting_error_deg[2];
    MotorSequenceState waiting_hold;
    uint32_t integrated;
    uint32_t forward_task,task_steps_done;
    uint32_t task_move_ms[4],task_peak_compare[4];
    uint32_t recovery_only;
    uint32_t roundtrip_mode;
    uint32_t indexed_start, indexed_start_raw[2];
    uint32_t fixed_home_start;
} MotorReturnRepeatState;

/* Status layout from motor_p2_recalibration.h. */
enum { P2CAL_WAIT_P4=0U,P2CAL_RETURN_P4=1U,P2CAL_READY_P2=2U,P2CAL_MOVE_P2=3U,P2CAL_FAULT=4U };
typedef struct {
    uint32_t phase,p4_restored,jog_steps,p2_target_raw,p4_target_raw;
} MotorP2CalibrationState;

/* Status layout from motor_p2_slot_test.h. */
enum { P2TEST_WAIT_PRESS=0U,P2TEST_READY=2U,P2TEST_MOVE=3U,P2TEST_FAULT=4U };
typedef struct {
    uint32_t phase,has_target,moves_done,slot_index,p2_target_raw,p4_target_raw;
} MotorP2SlotTestState;

/* Status layout from motor_p4_recalibration.h. */
enum { P4CAL_WAIT_PRESS=0U,P4CAL_READY=2U,P4CAL_MOVE=3U,P4CAL_FAULT=4U };
typedef struct {
    uint32_t phase,jog_steps,p4_target_raw,slot_index;
} MotorP4CalibrationState;

/* Status layout from motor_p2_repeat30.h. */
enum { P2R30_WAIT=0U,P2R30_STOPPED=2U,P2R30_MOVE=3U,P2R30_FAULT=4U };
typedef struct {
    uint32_t phase,running,moves_done,stop_count,target_raw;
} MotorP2Repeat30State;

/* Status layout from motor_p4_repeat30.h. */
enum { P4R30_WAIT=0U,P4R30_STOPPED=2U,P4R30_MOVE=3U,P4R30_FAULT=4U };
typedef struct {
    uint32_t phase,running,moves_done,stop_count,target_raw;
} MotorP4Repeat30State;

/* Status layout from motor_slot_banks.h. */
enum { BANK_WAIT=0U,BANK_READY=2U,BANK_MOVE=3U,BANK_FAULT=4U,BANK_STOPPED=5U };
typedef struct {
    uint32_t phase,next_axis,next_slot,steps_done,cycles_done,stop_count;
    uint32_t target_raw[2];
} MotorSlotBanksState;

#ifdef __cplusplus
#define DEBUG_COMPAT_FN static constexpr
#else
#define DEBUG_COMPAT_FN static inline
#endif
DEBUG_COMPAT_FN void MotorHomeAlign_Trip(MotorHomeAlignState *s,uint32_t fault,uint32_t axis)
{
    if(s->phase==HOMEALIGN_FAULT)return;
    s->fault=fault;s->fault_axis=axis;s->ready=0U;s->done=1U;s->phase=HOMEALIGN_FAULT;
    MotorSequence_Trip(&s->hold,fault,axis);
}
DEBUG_COMPAT_FN void MotorP2Prepare_Trip(MotorP2PrepareState *s,uint32_t fault,uint32_t axis)
{
    if(s->phase==P2PREP_FAULT)return;
    s->fault=fault;s->fault_axis=axis;s->ready=0U;s->done=1U;s->phase=P2PREP_FAULT;
    MotorSequence_Trip(&s->hold,fault,axis);
}
DEBUG_COMPAT_FN void MotorReturnOut_Trip(MotorReturnOutState *s,uint32_t fault,uint32_t axis)
{
    if(s->phase==RETURNOUT_FAULT)return;
    s->fault=fault;s->fault_axis=axis;s->ready=0U;s->done=1U;s->phase=RETURNOUT_FAULT;
    MotorSequence_Trip(&s->hold,fault,axis);
}
DEBUG_COMPAT_FN void MotorReturnBack_Trip(MotorReturnBackState *s,uint32_t fault,uint32_t axis)
{
    if(s->phase==RETURNBACK_FAULT)return;
    s->fault=fault;s->fault_axis=axis;s->ready=0U;s->done=1U;s->phase=RETURNBACK_FAULT;
    MotorSequence_Trip(&s->hold,fault,axis);
}
DEBUG_COMPAT_FN void MotorReturnRepeat_Trip(MotorReturnRepeatState *s,uint32_t fault,uint32_t axis)
{
    if(s->phase==REPEAT_FAULT)return;
    s->phase=REPEAT_FAULT;s->ready=0U;s->fault=fault;s->fault_axis=axis;
    MotorSequence_Trip(&s->waiting_hold,fault,axis);
    MotorHomeAlign_Trip(&s->initial_align,fault,axis);MotorP2Prepare_Trip(&s->preparation,fault,axis);
    MotorReturnOut_Trip(&s->outward,fault,axis);MotorReturnBack_Trip(&s->backward,fault,axis);
}
DEBUG_COMPAT_FN uint32_t MotorReturnRepeat_PreparationPhase(uint32_t phase)
{return phase==REPEAT_PREP_WAIT||phase==REPEAT_PREP||phase==REPEAT_P2_BACK_WAIT||phase==REPEAT_P2_BACK;}
#undef DEBUG_COMPAT_FN
#endif


/* ==================== motor_fault_record.h ==================== */
#ifndef MOTOR_FAULT_RECORD_H
#define MOTOR_FAULT_RECORD_H

#define MOTOR_FAULT_RECORD_MAGIC 0x4D465231U
#define MOTOR_FAULT_HISTORY_COUNT 32U
typedef struct {uint32_t ms,raw_pair,p2_pwm_pair,p4_pwm_pair;} MotorFaultSample;
typedef struct {uint32_t head,count;MotorFaultSample sample[MOTOR_FAULT_HISTORY_COUNT];} MotorFaultHistory;
typedef struct {
    uint32_t phase,previous_raw[2],pwm[4];
    int32_t previous_error_mdeg[2];
} MotorFaultBefore;
typedef struct {
    uint32_t magic,version,build_id,fault,fault_axis,phase_before,completed_returns,capture_ms;
    uint32_t steps_done,elapsed_ms,hold_elapsed_ms,raw[2],previous_raw[2],pwm_before[4];
    int32_t error_mdeg[2],previous_error_mdeg[2],raw_step_mdeg[2],speed_mdps[2];
    uint32_t sample_head,sample_count;
    MotorFaultSample sample[MOTOR_FAULT_HISTORY_COUNT];
    uint32_t hash;
} MotorFaultRecord;
#ifdef __cplusplus
#define FAULTREC_FN static constexpr
#else
#define FAULTREC_FN static inline
#endif
FAULTREC_FN int32_t MotorFaultRecord_Milli(float x)
{return (int32_t)(x*1000.0f+(x<0.0f?-0.5f:0.5f));}
FAULTREC_FN uint32_t MotorFaultRecord_Mix(uint32_t h,uint32_t x)
{return (h^x)*16777619U;}
FAULTREC_FN uint32_t MotorFaultRecord_Hash(const MotorFaultRecord *r)
{
    uint32_t h=2166136261U;
    h=MotorFaultRecord_Mix(h,r->version);h=MotorFaultRecord_Mix(h,r->build_id);
    h=MotorFaultRecord_Mix(h,r->fault);h=MotorFaultRecord_Mix(h,r->fault_axis);
    h=MotorFaultRecord_Mix(h,r->phase_before);h=MotorFaultRecord_Mix(h,r->completed_returns);
    h=MotorFaultRecord_Mix(h,r->capture_ms);h=MotorFaultRecord_Mix(h,r->steps_done);
    h=MotorFaultRecord_Mix(h,r->elapsed_ms);h=MotorFaultRecord_Mix(h,r->hold_elapsed_ms);
    for(uint32_t i=0U;i<2U;++i)
    {
        h=MotorFaultRecord_Mix(h,r->raw[i]);h=MotorFaultRecord_Mix(h,r->previous_raw[i]);
        h=MotorFaultRecord_Mix(h,(uint32_t)r->error_mdeg[i]);
        h=MotorFaultRecord_Mix(h,(uint32_t)r->previous_error_mdeg[i]);
        h=MotorFaultRecord_Mix(h,(uint32_t)r->raw_step_mdeg[i]);
        h=MotorFaultRecord_Mix(h,(uint32_t)r->speed_mdps[i]);
    }
    for(uint32_t i=0U;i<4U;++i)h=MotorFaultRecord_Mix(h,r->pwm_before[i]);
    h=MotorFaultRecord_Mix(h,r->sample_head);h=MotorFaultRecord_Mix(h,r->sample_count);
    for(uint32_t i=0U;i<MOTOR_FAULT_HISTORY_COUNT;++i)
    {
        h=MotorFaultRecord_Mix(h,r->sample[i].ms);h=MotorFaultRecord_Mix(h,r->sample[i].raw_pair);
        h=MotorFaultRecord_Mix(h,r->sample[i].p2_pwm_pair);h=MotorFaultRecord_Mix(h,r->sample[i].p4_pwm_pair);
    }
    return h;
}
FAULTREC_FN uint32_t MotorFaultRecord_Valid(const MotorFaultRecord *r)
{return r->magic==MOTOR_FAULT_RECORD_MAGIC&&r->version==1U&&r->hash==MotorFaultRecord_Hash(r);}
FAULTREC_FN void MotorFaultHistory_Push(MotorFaultHistory *h,uint32_t now,uint32_t raw1,uint32_t raw2,const uint32_t pwm[4])
{
    MotorFaultSample s={now,(raw1&0x3FFFU)|((raw2&0x3FFFU)<<16U),
        (pwm[0]&0xFFFU)|((pwm[1]&0xFFFU)<<12U),(pwm[2]&0xFFFU)|((pwm[3]&0xFFFU)<<12U)};
    h->sample[h->head]=s;h->head=(h->head+1U)%MOTOR_FAULT_HISTORY_COUNT;
    if(h->count<MOTOR_FAULT_HISTORY_COUNT)++h->count;
}
FAULTREC_FN MotorFaultRecord MotorFaultRecord_Make(const MotorReturnRepeatState *s,const MotorFaultBefore *b,
    const MotorFaultHistory *h,uint32_t raw1,uint32_t raw2,const MotorSequenceFeedback *f,uint32_t now)
{
    MotorFaultRecord r={0};r.version=1U;r.build_id=MOTORSEQ_BUILD_ID;
    r.fault=s->fault;r.fault_axis=s->fault_axis;r.phase_before=b->phase;
    r.completed_returns=s->completed_returns;r.capture_ms=now;r.raw[0]=raw1;r.raw[1]=raw2;
    uint32_t waiting=s->waiting_fault_latched;
    uint32_t aligning=b->phase==REPEAT_HOME_WAIT||b->phase==REPEAT_HOME_ALIGN;
    uint32_t preparing=MotorReturnRepeat_PreparationPhase(b->phase);
    uint32_t outward=b->phase==REPEAT_OUT||b->phase==REPEAT_WAIT_OUT;
    r.steps_done=waiting?0U:aligning?s->initial_align.hold.steps_done:preparing?s->preparation.hold.steps_done:outward?s->outward.hold.steps_done:s->backward.hold.steps_done;
    r.elapsed_ms=waiting?s->waiting_elapsed_ms:aligning?s->initial_align.elapsed_ms:preparing?s->preparation.elapsed_ms:outward?s->outward.elapsed_ms:s->backward.elapsed_ms;
    r.hold_elapsed_ms=waiting?s->waiting_elapsed_ms:aligning?s->initial_align.hold_elapsed_ms:preparing?s->preparation.hold_elapsed_ms:outward?s->outward.hold_elapsed_ms:s->backward.hold_elapsed_ms;
    for(uint32_t i=0U;i<2U;++i)
    {
        r.previous_raw[i]=b->previous_raw[i];r.previous_error_mdeg[i]=b->previous_error_mdeg[i];
        r.error_mdeg[i]=MotorFaultRecord_Milli(waiting?s->waiting_error_deg[i]:aligning?s->initial_align.error_deg[i]:preparing?s->preparation.error_deg[i]:outward?s->outward.error_deg[i]:s->backward.error_deg[i]);
        r.raw_step_mdeg[i]=MotorFaultRecord_Milli(MotorReference_Offset(b->previous_raw[i],r.raw[i]));
        r.speed_mdps[i]=MotorFaultRecord_Milli(f->speed[i]);
    }
    for(uint32_t i=0U;i<4U;++i)r.pwm_before[i]=b->pwm[i];
    r.sample_head=h->head;r.sample_count=h->count;
    for(uint32_t i=0U;i<MOTOR_FAULT_HISTORY_COUNT;++i)r.sample[i]=h->sample[i];
    r.hash=MotorFaultRecord_Hash(&r);r.magic=MOTOR_FAULT_RECORD_MAGIC;
    return r;
}
#undef FAULTREC_FN
#endif


/* ==================== motor_motion_diag.h ==================== */
#ifndef MOTOR_MOTION_DIAG_H
#define MOTOR_MOTION_DIAG_H


#define MOTOR_MOTION_TRACE_COUNT 256U
#define MOTOR_MOTION_LEG_COUNT 32U
#define MOTOR_MOTION_SAMPLE_MS 20U
enum { MOTION_END_COMPLETE=1U, MOTION_END_FAULT=2U, MOTION_END_B1=3U };

/* PWM values are applied timer commands, not measured current or torque. */
typedef struct {
    uint32_t ms,steps_done,active_axis,phase,fault,raw[2],target_raw;
    float speed_limit_dps,speed_ref_dps,speed_dps[2],duty_pct[2],error_deg;
} MotorMotionSample;

typedef struct {
    uint32_t axis,step,start_ms,end_ms,duration_ms,drive_ms,settle_ms;
    uint32_t start_raw,target_raw,end_raw,end_reason,fault;
    int32_t direction;
    float speed_limit_dps,peak_ref_dps,peak_speed_dps,peak_duty_pct,duty_limit_pct;
    float saturation_pct,mean_abs_speed_error_dps,final_error_deg;
    float peak_settle_duty_pct,settle_saturation_pct,peak_settle_speed_dps;
} MotorMotionLeg;

typedef struct {
    uint32_t generation,trace_head,trace_count,leg_head,leg_count,frozen;
    MotorMotionSample live;
    MotorMotionLeg last[2]; /* Last finished/interrupted P2 and P4 task legs. */
} MotorMotionDiagnostic;

typedef struct {
    uint32_t active,started,frozen,sample_started,last_sample_ms,start_steps;
    uint32_t drive_end_seen,drive_end_ms,drive_samples,saturated_samples;
    uint32_t settle_samples,settle_saturated_samples;
    float speed_error_sum;
    MotorMotionLeg leg;
} MotorMotionObserver;

typedef struct {
    uint32_t sample_ready,trace_ready,leg_ready,freeze;
    MotorMotionSample sample;
    MotorMotionLeg leg;
} MotorMotionReport;

#ifdef __cplusplus
#define MOTION_DIAG_FN static constexpr
extern "C" {
#else
#define MOTION_DIAG_FN static inline
#endif
extern volatile MotorMotionDiagnostic g_motion_diag;
extern volatile MotorMotionSample g_motion_trace[MOTOR_MOTION_TRACE_COUNT];
extern volatile MotorMotionLeg g_motion_legs[MOTOR_MOTION_LEG_COUNT];
#ifdef __cplusplus
}
#endif

MOTION_DIAG_FN uint32_t MotorMotionDiag_Compare(const uint32_t pwm[4],uint32_t axis)
{
    return pwm[2U*axis]>pwm[2U*axis+1U]?pwm[2U*axis]:pwm[2U*axis+1U];
}

MOTION_DIAG_FN MotorMotionReport MotorMotionDiag_Update(MotorMotionObserver *s,
    const MotorMeasuredSlotsState *task,const MotorReferenceState *pose,
    const MotorSequenceConfig *config,const MotorSequenceFeedback *feedback,
    uint32_t raw1,uint32_t raw2,const uint32_t pwm[4],uint32_t now)
{
    MotorMotionReport report={0};
    if(s->frozen)return report;
    const uint32_t raw[2]={raw1&0x3FFFU,raw2&0x3FFFU};
    const uint32_t axis=task->aligned&&task->phase==MOTORSEQ_MOVE&&
        pose->phase==MOTORREF_ALIGN&&!pose->fault?pose->moving_axis:0U;

    /* A completion may start the other motor in the same controller tick.
     * Close the previous leg against its stored target before opening it.
     */
    if(s->active&&(pose->fault||axis!=s->leg.axis||task->steps_done!=s->start_steps))
    {
        const uint32_t i=s->leg.axis-1U;
        s->leg.end_ms=now;s->leg.duration_ms=now-s->leg.start_ms;
        s->leg.drive_ms=s->drive_end_seen?s->drive_end_ms-s->leg.start_ms:s->leg.duration_ms;
        s->leg.settle_ms=s->leg.duration_ms-s->leg.drive_ms;
        s->leg.end_raw=raw[i];s->leg.fault=pose->fault;
        s->leg.end_reason=task->phase==MOTORSEQ_STOPPED||pose->fault==6U?MOTION_END_B1:
            pose->fault?MOTION_END_FAULT:MOTION_END_COMPLETE;
        s->leg.final_error_deg=MotorReference_Offset(raw[i],s->leg.target_raw)*(float)MotorReference_FeedbackSign(i);
        if(s->drive_samples)
        {
            s->leg.saturation_pct=100.0f*(float)s->saturated_samples/(float)s->drive_samples;
            s->leg.mean_abs_speed_error_dps=s->speed_error_sum/(float)s->drive_samples;
        }
        if(s->settle_samples)
            s->leg.settle_saturation_pct=100.0f*(float)s->settle_saturated_samples/(float)s->settle_samples;
        report.leg=s->leg;report.leg_ready=1U;s->active=0U;
    }
    if(!s->active&&axis)
    {
        MotorMotionLeg fresh={0};s->leg=fresh;
        s->leg.axis=axis;s->leg.step=task->steps_done+1U;s->start_steps=task->steps_done;
        s->leg.start_ms=now;s->leg.start_raw=raw[axis-1U];
        s->leg.target_raw=MotorMeasuredSlots_Raw(axis-1U,task->slot_index[axis-1U]);
        s->leg.direction=pose->direction[axis-1U];s->leg.speed_limit_dps=task->current_speed_limit_dps;
        s->leg.duty_limit_pct=(float)(axis==1U?MOTOR_TUNE_TASK_P2_DUTY_PCT:MOTOR_TUNE_TASK_P4_DUTY_PCT);
        s->drive_end_seen=0U;s->drive_end_ms=0U;s->drive_samples=0U;s->saturated_samples=0U;
        s->settle_samples=0U;s->settle_saturated_samples=0U;
        s->speed_error_sum=0.0f;s->active=1U;s->started=1U;
    }
    if(s->active&&axis)
    {
        const uint32_t i=axis-1U;
        if(pose->leg_settling)
        {
            if(!s->drive_end_seen){s->drive_end_ms=now;s->drive_end_seen=1U;}
            const uint32_t compare=MotorMotionDiag_Compare(pwm,i);
#if MOTOR_TUNE_SLOT_CALIBRATION == MOTOR_SLOT_CALIBRATION_MEASURED_TASK
            const uint32_t cap=config->period*MOTORREF_FINE_DUTY_PCT/100U;
#else
            const uint32_t cap=config->period/10U;
#endif
            const float duty=config->period?100.0f*(float)compare/(float)config->period:0.0f;
            const float speed=MotorSequence_Abs(feedback->speed[i]);
            if(duty>s->leg.peak_settle_duty_pct)s->leg.peak_settle_duty_pct=duty;
            if(speed>s->leg.peak_settle_speed_dps)s->leg.peak_settle_speed_dps=speed;
            ++s->settle_samples;
            if(cap&&compare*100U>=cap*98U)++s->settle_saturated_samples;
        }
        else
        {
            const float reference=pose->motion[i].axis[0].speed_ref_dps*(float)pose->direction[i];
            const float measured=MotorSequence_Abs(feedback->speed[i]);
            const uint32_t compare=MotorMotionDiag_Compare(pwm,i);
            const uint32_t cap=config->period*(axis==1U?MOTOR_TUNE_TASK_P2_DUTY_PCT:MOTOR_TUNE_TASK_P4_DUTY_PCT)/100U;
            const float duty=config->period?100.0f*(float)compare/(float)config->period:0.0f;
            if(MotorSequence_Abs(reference)>s->leg.peak_ref_dps)s->leg.peak_ref_dps=MotorSequence_Abs(reference);
            if(measured>s->leg.peak_speed_dps)s->leg.peak_speed_dps=measured;
            if(duty>s->leg.peak_duty_pct)s->leg.peak_duty_pct=duty;
            ++s->drive_samples;
            if(cap&&compare*100U>=cap*98U)++s->saturated_samples; // At least 98% of the duty cap.
            s->speed_error_sum+=MotorSequence_Abs(reference-feedback->speed[i]);
        }
    }
    if(!s->sample_started||now-s->last_sample_ms>=MOTOR_MOTION_SAMPLE_MS||report.leg_ready||pose->fault)
    {
        MotorMotionSample sample={0};
        sample.ms=now;sample.steps_done=task->steps_done;sample.active_axis=axis;
        sample.phase=task->phase;sample.fault=pose->fault;
        sample.raw[0]=raw[0];sample.raw[1]=raw[1];sample.speed_limit_dps=task->current_speed_limit_dps;
        for(uint32_t i=0U;i<2U;++i)
        {
            sample.speed_dps[i]=feedback->speed[i];
            sample.duty_pct[i]=config->period?100.0f*(float)MotorMotionDiag_Compare(pwm,i)/(float)config->period:0.0f;
        }
        if(axis)
        {
            sample.target_raw=s->leg.target_raw;
            sample.speed_ref_dps=pose->motion[axis-1U].axis[0].speed_ref_dps*(float)pose->direction[axis-1U];
            sample.error_deg=MotorReference_Offset(raw[axis-1U],sample.target_raw)*(float)MotorReference_FeedbackSign(axis-1U);
        }
        report.sample=sample;report.sample_ready=1U;
        /* Keep live raw values updating while stopped, but preserve the last
         * motion trace for inspection. Resume appends again without reset. */
        report.trace_ready=pose->fault||(s->started&&
            (task->phase!=MOTORSEQ_STOPPED||report.leg_ready));
        s->last_sample_ms=now;s->sample_started=1U;
    }
    if(pose->fault){s->frozen=1U;report.freeze=1U;}
    return report;
}
#undef MOTION_DIAG_FN
#endif


#endif /* BARAM_MOTOR_DIAGNOSTICS_H */
