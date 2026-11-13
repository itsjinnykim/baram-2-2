/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sequence.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "sequence.h"
#include "motor.h"
#include "encoder.h"
#include "gpio.h"

volatile uint32_t g_feedback_ready = 0U;
volatile uint32_t g_feedback_period_us = 0U;
volatile uint32_t g_feedback_max_period_us = 0U;
volatile uint32_t g_feedback_overruns = 0U;
volatile uint32_t g_feedback_fault_gap_us = 0U;
volatile MotorSequenceState g_sequence = {0};
volatile MotorSequenceStop g_p2_stop = {0};
volatile uint32_t g_p2_start_raw = 0U; /* Encoder sample at the accepted B1 command. */
volatile MotorReferenceState g_home = {0};
volatile MotorP2CalibrationState g_p2_cal = {0};
volatile MotorP2SlotTestState g_p2_test = {0};
volatile MotorP4CalibrationState g_p4_cal = {0};
volatile MotorP2Repeat30State g_p2_repeat30 = {0};
volatile MotorP4Repeat30State g_p4_repeat30 = {0};
volatile MotorSlotBanksState g_slot_banks = {0};
volatile P2RegistrationState g_p2_jog = {0};
volatile P2TrialState g_p2_trial = {0};
volatile MotorHoldTrialState g_hold_trial = {0};
volatile MotorReturnRepeatState g_return_repeat = {0};
volatile uint32_t g_exhibition_completed_cycles = 0U;
volatile MotorFaultRecord g_fault_record __attribute__((section(".fault_record"),used,aligned(4)));
volatile uint32_t g_fault_record_valid = 0U;
volatile P4RegistrationState g_p4_jog = {0}; /* motion slot0 = physical P4 */
volatile MotorMotionDiagnostic g_motion_diag = {0};
volatile MotorMotionSample g_motion_trace[MOTOR_MOTION_TRACE_COUNT] = {0};
volatile MotorMotionLeg g_motion_legs[MOTOR_MOTION_LEG_COUNT] = {0};

static void MotionDiag_Publish(const MotorMotionReport *report)
{
    if(!report->sample_ready&&!report->leg_ready&&!report->freeze)return;
    ++g_motion_diag.generation;__DMB();
    if(report->leg_ready)
    {
        const uint32_t head=g_motion_diag.leg_head;
        g_motion_legs[head]=report->leg;
        g_motion_diag.leg_head=(head+1U)%MOTOR_MOTION_LEG_COUNT;
        if(g_motion_diag.leg_count<MOTOR_MOTION_LEG_COUNT)++g_motion_diag.leg_count;
        if(report->leg.axis>=1U&&report->leg.axis<=2U)
            g_motion_diag.last[report->leg.axis-1U]=report->leg;
    }
    if(report->sample_ready)
    {
        g_motion_diag.live=report->sample;
        if(report->trace_ready)
        {
            const uint32_t head=g_motion_diag.trace_head;
            g_motion_trace[head]=report->sample;
            g_motion_diag.trace_head=(head+1U)%MOTOR_MOTION_TRACE_COUNT;
            if(g_motion_diag.trace_count<MOTOR_MOTION_TRACE_COUNT)++g_motion_diag.trace_count;
        }
    }
    if(report->freeze)g_motion_diag.frozen=1U;
    __DMB();++g_motion_diag.generation;
}

/* Active application: nearest-slot alignment, then alternating P2/P4 task.
 * Old calibration runners are archived outside the project. Debug status
 * containers remain declared above so existing Live Expressions still work. */
#if MOTOR_TUNE_SLOT_CALIBRATION != MOTOR_SLOT_CALIBRATION_MEASURED_TASK
#error This cleaned application supports the measured-slot task only.
#endif

void Sequence_Run(void)
{
    EncoderContext encoder1={.cs_pin=GPIO_PIN_8,.result=&g_encoder1};
    EncoderContext encoder2={.cs_pin=GPIO_PIN_9,.result=&g_encoder2};
    MotorReturnRepeatState state={.phase=REPEAT_HOME_WAIT,.integrated=1U,.forward_task=1U,
        .roundtrip_mode=MOTOR_TUNE_ROUNDTRIP_AUTOMATIC?ROUNDTRIP_AUTOMATIC:ROUNDTRIP_SUPERVISED};
    state.fixed_home_start=1U;
    MotorReferenceState home={0};
    MotorMeasuredSlotsState measured_slots={0};
    MotorMotionObserver motion_observer={0};
    const MotorSequenceConfig home_config=MotorMeasuredSlots_Config();
    MotorFaultHistory history={0};
    uint32_t previous_raw[2]={0U,0U};
    uint32_t record_saved=0U;
    MotorFaultRecord saved=g_fault_record;
    g_fault_record_valid=MotorFaultRecord_Valid(&saved);
    MotorSequenceButton button={0};
    Motor_InputsOff();
    Motor_StartBrakeOutputs();
    Encoder_TimebaseInit();
    Encoder_DeassertAll();
    HAL_Delay(20U);
    button.raw_pressed=GPIO_ButtonPressed();
    button.changed_ms=HAL_GetTick();
    uint32_t last_poll=DWT->CYCCNT;
    const uint32_t poll_cycles=ENCODER_POLL_US*Encoder_CyclesPerUs();
    for(;;)
    {
        uint32_t cycles=DWT->CYCCNT;
        uint32_t elapsed=cycles-last_poll;
        if(elapsed<poll_cycles)continue;
        last_poll=cycles;
        MotorFaultBefore before={0};before.phase=state.phase;
        before.previous_raw[0]=previous_raw[0];before.previous_raw[1]=previous_raw[1];
        for(uint32_t i=0U;i<2U;++i)
            before.previous_error_mdeg[i]=MotorFaultRecord_Milli(home.error_deg[i]);
        Motor_ReadCompare(before.pwm);
        g_feedback_period_us=elapsed/Encoder_CyclesPerUs();
        if(g_feedback_period_us>g_feedback_max_period_us)g_feedback_max_period_us=g_feedback_period_us;
        uint32_t checking_gap=home.phase==MOTORREF_ALIGN||home.phase==MOTORREF_READY;
        if(checking_gap&&g_feedback_period_us>ENCODER_MAX_GAP_US)
        {
            g_feedback_fault_gap_us=g_feedback_period_us;
            MotorReference_Fail(&home,1U,0U,1U);
        }
        Encoder_Poll(&encoder1);Encoder_Poll(&encoder2);
        uint32_t startup_waiting=home.phase==MOTORREF_BOOT||home.phase==MOTORREF_PARTIAL_DONE;
        Encoder_RecoverBeforeStart(&encoder1,startup_waiting);
        Encoder_RecoverBeforeStart(&encoder2,startup_waiting);
        uint32_t good1=g_encoder1.valid&&g_encoder1.tracking_valid&&g_encoder1.sample_interval_us&&g_encoder1.sample_interval_us<=ENCODER_MAX_GAP_US;
        uint32_t good2=g_encoder2.valid&&g_encoder2.tracking_valid&&g_encoder2.sample_interval_us&&g_encoder2.sample_interval_us<=ENCODER_MAX_GAP_US;
        g_feedback_ready=good1&&g_encoder1.consecutive_good>=100U&&good2&&g_encoder2.consecutive_good>=100U;
        uint32_t now=HAL_GetTick();
        uint32_t press=GPIO_ButtonPress(&button,now);
        MotorSequenceFeedback feedback={
            {good1,good2},{g_encoder1.position_deg,g_encoder2.position_deg},
            {g_encoder1.speed_dps,g_encoder2.speed_dps},
            {(float)g_encoder1.sample_interval_us*0.000001f,(float)g_encoder2.sample_interval_us*0.000001f}
        };
        feedback=MotorSequence_OrientFeedback(&feedback,MOTORSEQ_AXIS1_FEEDBACK_SIGN,MOTORSEQ_AXIS2_FEEDBACK_SIGN);
        if(!record_saved)MotorFaultHistory_Push(&history,now,g_encoder1.raw,g_encoder2.raw,before.pwm);
        MotorMeasuredSlots_Update(&measured_slots,&home,&home_config,&feedback,
            g_encoder1.raw,g_encoder2.raw,now,g_feedback_ready,press);
        uint32_t completed_cycles=measured_slots.steps_done/2U;
        g_home=home;
        if(home.fault&&!state.fault)
        {
            state.waiting_fault_latched=1U;
            state.waiting_elapsed_ms=home.recovery_elapsed_ms;
            for(uint32_t i=0U;i<2U;++i)state.waiting_error_deg[i]=home.error_deg[i];
            MotorReturnRepeat_Trip(&state,home.fault,home.fault_axis);
        }
        else if(!state.fault)
        {
            state.phase=home.phase==MOTORREF_ALIGN?REPEAT_HOME_ALIGN:
                home.phase==MOTORREF_PARTIAL_DONE?REPEAT_DONE:REPEAT_HOME_WAIT;
            if(measured_slots.phase!=MOTORSEQ_WAIT)
                state.phase=measured_slots.phase==MOTORSEQ_MOVE?REPEAT_ALTERNATING_MOVE:REPEAT_ALTERNATING_PAUSE;
            state.ready=home.ready;
        }
        if(state.fault)
        {
            if(!MotorSequence_FaultUsesBrake(state.fault))Motor_InputsOff();
            g_motor_fault=0xD400U|(state.fault_axis<<8U)|state.fault;
        }
        Motor_ApplyCompare(MotorReference_ChannelCompare(&home,0U,0U),
            MotorReference_ChannelCompare(&home,0U,1U),
            MotorReference_ChannelCompare(&home,1U,0U),
            MotorReference_ChannelCompare(&home,1U,1U));
        /* Stop outputs precede diagnostic hashing/saving, as before cleanup. */
        uint32_t applied_pwm[4];Motor_ReadCompare(applied_pwm);
        const MotorMotionReport report=MotorMotionDiag_Update(&motion_observer,
            &measured_slots,&home,&home_config,&feedback,
            g_encoder1.raw,g_encoder2.raw,applied_pwm,now);
        MotionDiag_Publish(&report);
        if(state.fault&&!record_saved)
        {
            MotorFaultRecord next=MotorFaultRecord_Make(&state,&before,&history,
                g_encoder1.raw,g_encoder2.raw,&feedback,now);
            next.steps_done=measured_slots.steps_done;
            next.hash=MotorFaultRecord_Hash(&next);
            g_fault_record_valid=0U;g_fault_record.magic=0U;
            next.magic=0U;g_fault_record=next;__DMB();
            g_fault_record.magic=MOTOR_FAULT_RECORD_MAGIC;__DMB();
            g_fault_record_valid=1U;record_saved=1U;
        }
        previous_raw[0]=g_encoder1.raw;previous_raw[1]=g_encoder2.raw;
        g_return_repeat=state;
        g_sequence.ready=home.ready;
        g_sequence.phase=home.fault?MOTORSEQ_FAULT:measured_slots.phase;
        g_sequence.fault=home.fault;
        g_sequence.fault_axis=home.fault_axis;
        g_sequence.active_axis=home.moving_axis;
        g_sequence.steps_done=measured_slots.steps_done;
        g_sequence.current_speed_limit_dps=measured_slots.current_speed_limit_dps;
        for(uint32_t i=0U;i<2U;++i)
        {
            g_sequence.axis[i].target_deg=home.target_deg[i];
            g_sequence.axis[i].error_deg=home.error_deg[i];
        }
        g_sequence.build_id=MOTORSEQ_BUILD_ID;
        g_exhibition_completed_cycles=completed_cycles;
        g_motor_stage=80U+state.phase;
        uint32_t led_on=home.recovery_waiting?(now/250U)&1U:
            home.phase==MOTORREF_ALIGN||(home.ready&&((now/500U)&1U));
        GPIO_SetLed(led_on&&!state.fault);
    }
}
