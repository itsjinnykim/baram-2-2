#ifndef TIM_H
#define TIM_H
#include "def.h"
#ifdef __cplusplus
extern "C" {
#endif
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
void MX_TIM1_Init(void);
void MX_TIM8_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
#ifdef __cplusplus
}
#endif
#endif
