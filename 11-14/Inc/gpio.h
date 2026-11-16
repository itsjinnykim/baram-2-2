#ifndef GPIO_H
#define GPIO_H
#include "def.h"
#include "motor_control.h"
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#ifdef __cplusplus
extern "C" {
#endif
void MX_GPIO_Init(void);
void GPIO_InitStatusLed(void);
void GPIO_InitUserButton(void);
void GPIO_BootBlink(void);
uint32_t GPIO_ButtonPressed(void);
uint32_t GPIO_ButtonPress(MotorSequenceButton *button,uint32_t now);
void GPIO_SetLed(uint32_t on);
#ifdef __cplusplus
}
#endif
#endif
