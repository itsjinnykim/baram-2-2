/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor.c
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
#include "motor.h"
#include "main.h"

volatile uint32_t g_motor_stage = 0U;
volatile uint32_t g_motor_fault = 0U;

void Motor_InputsOff(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* Encoder test/error path: disconnect timers and drive every HI/LI low. */
    if (htim1.Instance != NULL) { __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1); }
    if (htim8.Instance != NULL) { __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8); }
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_13 | GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &gpio);
}

static void Motor_Require(HAL_StatusTypeDef status, uint32_t code)
{
    if (status != HAL_OK)
    {
        g_motor_fault = code;
        Error_Handler();
    }
}

static void Motor_CheckOutputs(TIM_HandleTypeDef *htim)
{
    const uint32_t enabled = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                             TIM_CCER_CC2E | TIM_CCER_CC2NE;
    const uint32_t polarity = TIM_CCER_CC1P | TIM_CCER_CC1NP |
                              TIM_CCER_CC2P | TIM_CCER_CC2NP;
    const uint32_t preload = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    const uint32_t id = (htim->Instance == TIM1) ? 0x100U : 0x800U;

    if (((htim->Instance->CR1 & (TIM_CR1_CEN | TIM_CR1_UDIS)) != TIM_CR1_CEN) ||
        ((htim->Instance->BDTR & TIM_BDTR_MOE) == 0U) ||
        ((htim->Instance->CCER & (enabled | polarity)) != enabled) ||
        ((htim->Instance->CCMR1 & preload) != preload))
    {
        g_motor_fault = id | 6U;
        Error_Handler();
    }
}

void Motor_InitBrake(TIM_HandleTypeDef *htim)
{
    const uint32_t id = (htim->Instance == TIM1) ? 0x100U : 0x800U;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COUNTER(htim, 0U);
    Motor_Require(HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE), id | 1U);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);

    Motor_Require(HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1), id | 2U);
    Motor_Require(HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1), id | 3U);
    Motor_Require(HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2), id | 4U);
    Motor_Require(HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2), id | 5U);
    Motor_CheckOutputs(htim);

    /* A natural update proves the counter/update path runs after the initial UG. */
    HAL_Delay(1U);
    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) == RESET)
    {
        g_motor_fault = id | 7U;
        Error_Handler();
    }
}
void Motor_StartBrakeOutputs(void)
{
    HAL_TIM_MspPostInit(&htim1);Motor_InitBrake(&htim1);
    HAL_TIM_MspPostInit(&htim8);Motor_InitBrake(&htim8);
}
void Motor_ReadCompare(uint32_t compare[4])
{
    compare[0]=__HAL_TIM_GET_COMPARE(&htim1,TIM_CHANNEL_1);
    compare[1]=__HAL_TIM_GET_COMPARE(&htim1,TIM_CHANNEL_2);
    compare[2]=__HAL_TIM_GET_COMPARE(&htim8,TIM_CHANNEL_1);
    compare[3]=__HAL_TIM_GET_COMPARE(&htim8,TIM_CHANNEL_2);
}
void Motor_ApplyCompare(uint32_t p2_positive,uint32_t p2_negative,uint32_t p4_positive,uint32_t p4_negative)
{
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,p2_positive);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,p2_negative);
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,p4_positive);
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,p4_negative);
}
