#ifndef UART_H
#define UART_H
#include "def.h"
#ifdef __cplusplus
extern "C" {
#endif
extern UART_HandleTypeDef huart2;
void MX_USART2_UART_Init(void);
HAL_StatusTypeDef UART_Write(const uint8_t *data,uint16_t length,uint32_t timeout_ms);
#ifdef __cplusplus
}
#endif
#endif
