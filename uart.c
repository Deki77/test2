#include "uart.h"


void initUARTPins(GPIO_InitTypeDef* GPIO_InitStruct){
	
	__GPIOA_CLK_ENABLE();
	
	GPIO_InitStruct->Pin       = GPIO_PIN_9;
  GPIO_InitStruct->Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct->Pull      = GPIO_NOPULL;
  GPIO_InitStruct->Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct->Alternate = USART_TX_AF;
  HAL_GPIO_Init(GPIOA, GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct->Pin = GPIO_PIN_10;
	GPIO_InitStruct->Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct->Pull      = GPIO_NOPULL;
  GPIO_InitStruct->Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct->Alternate = USART_RX_AF;
  HAL_GPIO_Init(GPIOA, GPIO_InitStruct);
}

void initUART(UART_HandleTypeDef* UartHandle){
	
	__USART1_CLK_ENABLE();
	
	UartHandle->Instance = USART1;
	
	UartHandle->Init.BaudRate = 9600;
	UartHandle->Init.Mode = UART_MODE_TX_RX;
	UartHandle->Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle->Init.Parity = UART_PARITY_NONE;
	UartHandle->Init.StopBits = UART_STOPBITS_1;
	UartHandle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	
	if(HAL_UART_Init(UartHandle) != HAL_OK)
		Error_Handler();
}

void initUARTInt(void){
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
}

void UARTINIT(GPIO_InitTypeDef* GPIOstruct, UART_HandleTypeDef* UARTstruct){
	initUARTPins(GPIOstruct);
	initUART(UARTstruct);
	initUARTInt();
}

