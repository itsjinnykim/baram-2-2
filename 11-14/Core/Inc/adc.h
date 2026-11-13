#ifndef ADC_H
#define ADC_H
#include "def.h"
#ifdef __cplusplus
extern "C" {
#endif
extern ADC_HandleTypeDef hadc1;
extern volatile CurrentDiagnostic g_current1;
extern volatile CurrentDiagnostic g_current2;
extern volatile uint32_t g_current_polls;
extern volatile uint32_t g_adc_vdda_mv;
extern volatile MotorCurrentDiagnostic g_current_run;
void MX_ADC1_Init(void);
void ADC_ReadChannel(uint32_t channel,volatile CurrentDiagnostic *result);
void ADC_ReadCurrents(void);
#ifdef __cplusplus
}
#endif
#endif
