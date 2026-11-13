/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : adc.c
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
#include "adc.h"
#include "main.h"

ADC_HandleTypeDef hadc1;
volatile CurrentDiagnostic g_current1 = {0};
volatile CurrentDiagnostic g_current2 = {0};
volatile uint32_t g_current_polls = 0U;
volatile uint32_t g_adc_vdda_mv = 3300U;
volatile MotorCurrentDiagnostic g_current_run = {0};

void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */
  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1U;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1U;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* PC3/IN13 is selected at runtime by Current_ReadChannel(). */
  /* USER CODE END ADC1_Init 2 */
}

static void Current_RecordError(volatile CurrentDiagnostic *result, uint32_t step)
{
    result->valid = 0U;
    result->errors++;
    result->fault_step = step;
    result->hal_error = HAL_ADC_GetError(&hadc1);
}

static uint32_t Current_Convert(uint32_t *raw, volatile CurrentDiagnostic *result)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        Current_RecordError(result, 2U);
        (void)HAL_ADC_Stop(&hadc1);
        return 0U;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
    {
        Current_RecordError(result, 3U);
        (void)HAL_ADC_Stop(&hadc1);
        return 0U;
    }
    *raw = HAL_ADC_GetValue(&hadc1);
    if (HAL_ADC_Stop(&hadc1) != HAL_OK)
    {
        Current_RecordError(result, 4U);
        return 0U;
    }
    return 1U;
}

void ADC_ReadChannel(uint32_t channel, volatile CurrentDiagnostic *result)
{
    ADC_ChannelConfTypeDef config = {0};
    uint32_t raw = 0U, sum = 0U, minimum = 4095U, maximum = 0U;
    config.Channel = channel;
    config.Rank = 1U;
    config.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK)
    {
        Current_RecordError(result, 1U);
        return;
    }
    /* Discard the first conversion after changing the input multiplexer. */
    if (Current_Convert(&raw, result) == 0U) { return; }
    for (uint32_t i = 0U; i < 16U; i++)
    {
        if (Current_Convert(&raw, result) == 0U) { return; }
        sum += raw;
        if (raw < minimum) { minimum = raw; }
        if (raw > maximum) { maximum = raw; }
    }
    result->raw = (sum + 8U) / 16U;
    result->raw_min = minimum;
    result->raw_max = maximum;
    result->voltage_mv = ((float)sum / 16.0f) * (float)g_adc_vdda_mv / 4095.0f;
    result->samples += 16U;
    result->fault_step = 0U;
    result->hal_error = HAL_ADC_GetError(&hadc1);
    result->valid = 1U;
}
void ADC_ReadCurrents(void)
{
    ADC_ReadChannel(ADC_CHANNEL_12,&g_current1);
    ADC_ReadChannel(ADC_CHANNEL_13,&g_current2);
    g_current_polls++;
}
