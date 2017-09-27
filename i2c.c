#include "i2c.h"

void initI2cPins(GPIO_InitTypeDef* GPIO_InitStruct){
	__GPIOB_CLK_ENABLE();
	GPIO_InitStruct->Pin       = I2C1_SCLK_PIN;
  GPIO_InitStruct->Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct->Pull      = GPIO_PULLUP;
  GPIO_InitStruct->Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct->Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, GPIO_InitStruct);
		
	GPIO_InitStruct->Pin       = I2C1_SDA_PIN;
	GPIO_InitStruct->Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct->Pull      = GPIO_PULLUP;
  GPIO_InitStruct->Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct->Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, GPIO_InitStruct);
}

void initI2cPeriph(I2C_HandleTypeDef* I2cHandle){
	__I2C1_CLK_ENABLE();
	
	I2cHandle->Instance 	= I2C1;
  I2cHandle->Init.Timing	= I2C_TIMECONFIG;
	I2cHandle->Init.OwnAddress1     = 0;
  I2cHandle->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle->Init.OwnAddress2     = 0;
  I2cHandle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
	I2cHandle->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	
	HAL_I2C_Init(I2cHandle);
}

void initI2cInt(void){
	HAL_NVIC_EnableIRQ(I2C1_IRQn);
	HAL_NVIC_SetPriority(I2C1_IRQn, 1, 0);
}

void I2cINIT(GPIO_InitTypeDef* GPIO_InitStruct_t, I2C_HandleTypeDef* I2cHandle_t){
	initI2cPins(GPIO_InitStruct_t);
	initI2cPeriph(I2cHandle_t);
	initI2cInt();
}

HAL_StatusTypeDef I2cReadReg(I2C_HandleTypeDef *I2cStruct, uint16_t SlaveDevAddr, uint8_t DevReg, uint8_t* readBfr, int numData){
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(I2cStruct, SlaveDevAddr, &DevReg, 1, 400);
	if(rc == HAL_OK)
		rc = HAL_I2C_Master_Receive(I2cStruct, SlaveDevAddr, readBfr, numData, 400);
	
	return rc;
	
}
