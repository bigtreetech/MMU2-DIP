#include <Arduino.h>
#include "adc.h"

__IO uint16_t adc_val;

void adc_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  DMA_HandleTypeDef hdma_adc = {0}; 
  ADC_HandleTypeDef hadc = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_ADC1_CLK_ENABLE();                       

  GPIO_InitStruct.Pin  = GPIO_PIN_2;        
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // DMA Init
  hdma_adc.Instance           = DMA1_Channel1;
  hdma_adc.Init.Direction       = DMA_PERIPH_TO_MEMORY;
  hdma_adc.Init.PeriphInc       = DMA_PINC_DISABLE;
  hdma_adc.Init.MemInc         = DMA_MINC_ENABLE;
  hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc.Init.MemDataAlignment     = DMA_MDATAALIGN_HALFWORD;
  hdma_adc.Init.Mode           = DMA_CIRCULAR;
  hdma_adc.Init.Priority         = DMA_PRIORITY_MEDIUM;
  HAL_DMA_Init(&hdma_adc);

  // ADC init
  hadc.Instance           = ADC1;
  hadc.Init.ClockPrescaler     = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution       = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign       = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode       = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection       = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait     = DISABLE;
  hadc.Init.LowPowerAutoPowerOff   = DISABLE;
  hadc.Init.ContinuousConvMode   = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv     = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge   = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun         = ADC_OVR_DATA_PRESERVED;
  hadc.Init.SamplingTimeCommon    = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADC_Init(&hadc);
  ///  Start calibration
  HAL_ADCEx_Calibration_Start(&hadc);

  //Configure for the selected ADC regular channel to be converted. 
  sConfig.Rank    = ADC_RANK_CHANNEL_NUMBER;
  sConfig.Channel = ADC_CHANNEL_2;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
  __HAL_LINKDMA(&hadc,DMA_Handle,hdma_adc);

  HAL_ADC_Start_DMA(&hadc, (uint32_t *)&adc_val, 1);
}

