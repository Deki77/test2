#ifndef _UART_HEADER_
#define _UART_HEADER_

#include "stm32l0xx_hal.h"

#define USART_TX_AF GPIO_AF4_USART1
#define USART_RX_AF GPIO_AF4_USART1


void initUARTPins(GPIO_InitTypeDef* GPIO_InitStruct);
void initUART(UART_HandleTypeDef* UartHandle);
void initUARTInt(void);
void UARTINIT(GPIO_InitTypeDef* GPIOstruct, UART_HandleTypeDef* UARTstruct);
#endif
