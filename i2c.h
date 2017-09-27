#ifndef _IIC_HEADER_
#define _IIC_HEADER_

#include "stm32l0xx_hal.h"

#define I2C_TIMECONFIG 0x0000020B //0x00310309 this ones from the reference manual for 8MHz clock
#define I2C1_SDA_PIN GPIO_PIN_9;
#define I2C1_SCLK_PIN GPIO_PIN_8;


void initI2cPins(GPIO_InitTypeDef* GPIO_InitStruct);
void initI2cPeriph(I2C_HandleTypeDef* I2cHandle);
void initI2cInt(void);

void I2cINIT(GPIO_InitTypeDef* GPIO_InitStruct_t, I2C_HandleTypeDef* I2cHandle_t);

//void I2cWrite();

HAL_StatusTypeDef I2cReadReg(I2C_HandleTypeDef *I2cStruct, uint16_t SlaveDevAddr, uint8_t DevReg, uint8_t* readBfr, int numData);


#endif
