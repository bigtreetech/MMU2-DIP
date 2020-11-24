//shr16.c - 16bit shift register (2x74595)
#include <Arduino.h>
#include "shr16.h"

#include "config.h"

uint16_t shr16_v;

static void shr16_write(uint16_t v);

SPI_HandleTypeDef hspi2;   // 74hc595 Drive
void shr16_init(void)
{
  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /**SPI2 GPIO Configuration    
  PB13     ------> SPI2_SCK  595SHCP
  PB15     ------> SPI2_MOSI 595DS
  */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);   //PB12=0
  /* SPI Init */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  HAL_SPI_Init(&hspi2);

  shr16_v = 0;
  shr16_write(shr16_v);
}

void shr16_write(uint16_t v)
{
  asm("nop");
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&v, 1, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  asm("nop");
  asm("nop");
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  shr16_v = v;
}

void shr16_set_led(uint16_t led)
{
  led = ((led & 0x00ff) << 8) | ((led & 0x0300) >> 2);
  shr16_write((shr16_v & ~SHR16_LED_MSK) | led);
}

void shr16_set_ena(uint8_t ena)
{
  ena = ((ena & 1) << 1) | ((ena & 2) << 2) | ((ena & 4) << 3); // 0. << 1 == 1., 1. << 2 == 3., 2. << 3 == 5. EN
  shr16_write((shr16_v & ~SHR16_ENA_MSK) | ena);
}

void shr16_set_dir(uint8_t dir)
{
  dir = (dir & 1) | ((dir & 2) << 1) | ((dir & 4) << 2); // 0., 1. << 1 == 2., 2. << 2 == 4. DIR
  shr16_write((shr16_v & ~SHR16_DIR_MSK) | dir);
}

uint8_t shr16_get_ena(void)
{
  return ((shr16_v & 2) >> 1) | ((shr16_v & 8) >> 2) | ((shr16_v & 0x20) >> 3);
}

uint8_t shr16_get_dir(void)
{
  return (shr16_v & 1) | ((shr16_v & 4) >> 1) | ((shr16_v & 0x10) >> 2);
}