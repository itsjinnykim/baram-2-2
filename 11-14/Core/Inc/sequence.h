#ifndef SEQUENCE_H
#define SEQUENCE_H
#include "def.h"
#include "motor_control.h"

#include "motor_diagnostics.h"


#ifdef __cplusplus
extern "C" {
#endif
extern volatile uint32_t g_feedback_ready;
extern volatile uint32_t g_feedback_period_us;
extern volatile uint32_t g_feedback_max_period_us;
extern volatile uint32_t g_feedback_overruns;
extern volatile uint32_t g_feedback_fault_gap_us;
extern volatile MotorSequenceState g_sequence;
extern volatile MotorSequenceStop g_p2_stop;
extern volatile uint32_t g_p2_start_raw;
extern volatile MotorReferenceState g_home;
extern volatile MotorP2CalibrationState g_p2_cal;
extern volatile MotorP2SlotTestState g_p2_test;
extern volatile MotorP4CalibrationState g_p4_cal;
extern volatile MotorP2Repeat30State g_p2_repeat30;
extern volatile MotorP4Repeat30State g_p4_repeat30;
extern volatile MotorSlotBanksState g_slot_banks;
extern volatile P2RegistrationState g_p2_jog;
extern volatile P2TrialState g_p2_trial;
extern volatile MotorHoldTrialState g_hold_trial;
extern volatile MotorReturnRepeatState g_return_repeat;
extern volatile uint32_t g_exhibition_completed_cycles;
extern volatile MotorFaultRecord g_fault_record;
extern volatile uint32_t g_fault_record_valid;
extern volatile P4RegistrationState g_p4_jog;
void Sequence_Run(void);
#ifdef __cplusplus
}
#endif
#endif
