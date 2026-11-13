#ifndef MOTOR_H
#define MOTOR_H
#include "tim.h"
#ifdef __cplusplus
extern "C" {
#endif
extern volatile uint32_t g_motor_stage;
extern volatile uint32_t g_motor_fault;
void Motor_InputsOff(void);
void Motor_InitBrake(TIM_HandleTypeDef *htim);
void Motor_StartBrakeOutputs(void);
void Motor_ReadCompare(uint32_t compare[4]);
void Motor_ApplyCompare(uint32_t p2_positive,uint32_t p2_negative,uint32_t p4_positive,uint32_t p4_negative);
#ifdef __cplusplus
}
#endif
#endif
