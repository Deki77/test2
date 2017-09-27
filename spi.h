#ifndef _SPI_HEADER_
#define _SPI_HEADER_

#include "stm32l0xx_hal.h"
#define jon 1
#define SPI_MISO GPIO_PIN_14 //GPIOB14
#define SPI_MOSI GPIO_PIN_15 //GPIOB15
#define SPI_CLK GPIO_PIN_13 //GPIOB13
#define SPI_CS GPIO_PIN_12 //GPIOB12

HAL_StatusTypeDef status = HAL_OK;

void initSPIPins(GPIO_InitTypeDef* GPIO_InitStruct);
HAL_StatusTypeDef initSPIPeriph(SPI_HandleTypeDef* SPI_InitStruct);

void initSPIint();

HAL_StatusTypeDef initSPI(GPIO_InitTypeDef* GPIO_InitStruct, SPI_InitTypeDef* SPI_InitStruct);


#endif
