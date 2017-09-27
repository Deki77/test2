#include "spi.h"

void initSPIPins(GPIO_InitTypeDef* GPIO_InitStruct){
	//Paramters for MISO, MOSI, CLK
	GPIO_InitStruct->Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct->Pull = GPIO_NOPULL;
	GPIO_InitStruct->Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct->Alternate = GPIO_AF0_SPI2;

	GPIO_InitStruct->Pin = SPI_MOSI;
	HAL_GPIO_Init(GPIOB, GPIO_InitStruct);
	
	GPIO_InitStruct->Pin = SPI_MISO;
	HAL_GPIO_Init(GPIOB, GPIO_InitStruct);

	GPIO_InitStruct->Pin = SPI_CLK;
	HAL_GPIO_Init(GPIOB, GPIO_InitStruct);
	

	//Parameters for CS pin
	GPIO_InitStruct->Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct->Pull = GPIO_PULLUP;
	GPIO_InitStruct->Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct->Alternate = GPIO_AF0_SPI2;
	
	GPIO_InitStruct->Pin = SPI_CS;
	HAL_GPIO_Init(GPIOB, GPIO_InitStruct);
}

HAL_StatusTypeDef initSPIPeriph(SPI_HandleTypeDef* SPI_HandleStruct){
	
	SPI_HandleStruct->Instance = SPI2;
	
	SPI_HandleStruct->Init.Mode = SPI_MODE_MASTER;
	SPI_HandleStruct->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; //Giving clock of 250kHz
	SPI_HandleStruct->Init.Direction = SPI_DIRECTION_2LINES; //Using two line communication
	SPI_HandleStruct->Init.DataSize = SPI_DATASIZE_8BIT;
	SPI_HandleStruct->Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI_HandleStruct->Init.CLKPhase = SPI_PHASE_1EDGE; //First clock transition is first data capture
	SPI_HandleStruct->Init.CLKPolarity = SPI_POLARITY_LOW;
	SPI_HandleStruct->Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
	SPI_HandleStruct->Init.CRCPolynomial = 7;
	
	SPI_HandleStruct->Init.NSS = SPI_NSS_SOFT;
	SPI_HandleStruct->Init.TIMode = SPI_TIMODE_DISABLE;
	
	status = HAL_SPI_Init(SPI_HandleStruct);
	
	return status;
}
