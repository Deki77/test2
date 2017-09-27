/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "i2c.h"
#include "uart.h"
#define LED_PIN GPIO_PIN_7;


//TEMP SENSOR
#define PCT2075_ADDRESS 0x92

#define PCT2075_TEMPREG 0x00 //R
#define PCT2075_CONFREG 0x01 //RW
#define PCT2075_THYSTREG 0x02 //RW
#define PCT2075_TOSREG 0x03 //RW
#define PCT2075_TIDLEREG 0x04 //RW

#define PCT2075_SHTDWNCONFIG 0x1

I2C_HandleTypeDef I2cHandle;
GPIO_InitTypeDef GPIO_InitStruct;
TIM_HandleTypeDef s_TimerInstance;
SPI_HandleTypeDef spiHandle;
UART_HandleTypeDef uartHandle;

uint8_t dataBuffer1[8];
uint8_t receiveBuffer[8];
uint8_t tofreceiveBuffer[8];
uint8_t uartReceiveBfr[8];
int type = 0;


volatile uint32_t count2;
volatile uint32_t count;
volatile int count3;
uint32_t NEWCOUNT = 0;
int tempThresh = 0;

HAL_StatusTypeDef rc = HAL_OK;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);


uint8_t STATE_FLAG = 0;

int exitStat = 0;
static void state_idle(void);
static void state_checkTOF(void);
static void state_checkTEMP(void);
static void state_DONE(void);
static void(*statePtr)(void) = state_idle;




void initTimer(){
	__TIM2_CLK_ENABLE();
	s_TimerInstance.Instance = TIM2;
	s_TimerInstance.Init.Prescaler = 8000;
	s_TimerInstance.Init.CounterMode = TIM_COUNTERMODE_UP;
	s_TimerInstance.Init.Period = 5000;
	s_TimerInstance.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&s_TimerInstance);
	HAL_TIM_Base_Start_IT(&s_TimerInstance);
	
	HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void initPins(){
	__GPIOB_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = LED_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
 __GPIOA_CLK_ENABLE();
  //Configure GPIO pin : PA5 
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//__GPIOA_CLK_ENABLE();
	//set configuration for SYSCFG_EXTICR2 to set EXTI5 for PA5
	//EXTI->IMR |= 0x0020;
	//EXTI->RTSR |= 0x0020;
}

void initEXTIIRQ(){
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}
float formatTemp(uint16_t value){
	float retVal = value * 0.125;
	return retVal;
}

float readTemp(){
	rc = I2cReadReg(&I2cHandle, PCT2075_ADDRESS, PCT2075_TEMPREG, receiveBuffer, 2);
	if(rc != HAL_OK)
		return ERROR;
	uint16_t tempVal = (receiveBuffer[1] | (receiveBuffer[0] << 8)) >> 5;
	
	float valRet = formatTemp(tempVal);
	return valRet;
}

//Peripheral proximity sensor interrupt
void EXTI4_15_IRQHandler(void){
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_5);
	count3++;
	count2++;
}

//timer interrupt
void TIM2_IRQHandler(void){
	TIM2->SR = ~TIM_SR_UIF;
	//s_TimerInstance.Instance->SR = ~TIM_SR_UIF;
	__HAL_TIM_CLEAR_FLAG(&s_TimerInstance, TIM_SR_UIF);
	count++;
	count2++;
		//CLEAR_BIT(PWR->CR, (PWR_CR_PDDS | PWR_CR_LPSDSR));
}

int calcTempThresh(){
	int valRet = 10*(uartReceiveBfr[0] & 0xF) + (uartReceiveBfr[1] &0xF);
	return valRet;
}

void USART1_IRQHandler(void){
		HAL_UART_IRQHandler(&uartHandle);
		tempThresh = calcTempThresh();	
}


int main(void)
{
	type = 2;
	int DUTY_CYCLE = 1;
	int PERIOD = 3*DUTY_CYCLE;

	dataBuffer1[0] = PCT2075_THYSTREG;
	dataBuffer1[1] = 0x09;
	dataBuffer1[2] = 0x88;

  HAL_Init();
  SystemClock_Config();

	initPins();
	initTimer();
	I2cINIT(&GPIO_InitStruct, &I2cHandle);
	UARTINIT(&GPIO_InitStruct, &uartHandle);
	if(type == 2)
		initEXTIIRQ();
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	
	rc = HAL_I2C_Master_Transmit(&I2cHandle, PCT2075_ADDRESS, dataBuffer1, 3, 400);
	if(rc != HAL_OK)
		return ERROR;
	
	//rc = I2cRead(&I2cHandle, PTC2075_ADDRESS, PTC2075_THYSTREG, recieveBuffer1, 2);
	//rc = I2cReadReg(&I2cHandle, 0x52, 0xC0, receiveBuffer, 3);
	if(rc != HAL_OK)
		return ERROR;
	//Turn off 

	SysTick->CTRL = 0;	
	//HAL_UART_Transmit(&uartHandle, uartReceiveBfr, 1, 400);
	//HAL_UART_Receive(&uartHandle, uartReceiveBfr, 2, 400);
	
	HAL_UART_Receive_IT(&uartHandle, uartReceiveBfr, 2);
	
	for(;;){
		(*statePtr)();
		if(exitStat == 1)
			break;
	}
	//int tempThresh = calcTempThresh();
	//******************************************************************************************************************************//
	  //Enable the UART Error Interrupt: (Frame error, noise error, overrun error) 
    //SET_BIT(uartHandle.Instance->CR3, USART_CR3_EIE);
     //Enable the UART Parity Error and Data Register not empty Interrupts 
    //SET_BIT(uartHandle.Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
   while (1)
  {
		if(type == 1){
			if(count == DUTY_CYCLE)				
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
			else if(count == 2*DUTY_CYCLE)
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
			else if(count == PERIOD)
				count = 0;
			
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			NEWCOUNT++;
		}

		if(type == 2){
			if(count3 == 1){
					SysTick->CTRL = 1;
					float temperature = readTemp();
					SysTick->CTRL = 0;
					if(temperature > tempThresh)
						HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
						
					count3 = 0;
			}
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			
		}
	NEWCOUNT++;
	}	
}

static void state_idle(void){
	if(STATE_FLAG != 0x3)
		statePtr = state_checkTOF;
	else
		statePtr = state_DONE;
}

static void state_checkTOF(void){
	rc = I2cReadReg(&I2cHandle, 0x52, 0xC0, tofreceiveBuffer, 3);
	if(rc != HAL_OK){
		STATE_FLAG &= 0x2;
	}
	else{
		uint32_t checkVal = (tofreceiveBuffer[2] << 16 | tofreceiveBuffer[1] << 8 | tofreceiveBuffer[0]);
		if(checkVal == 0x10AAEE)
			STATE_FLAG |= 0x1;
		else
			STATE_FLAG &= 0x2;
	}
	statePtr = state_checkTEMP;
}

static void state_checkTEMP(void){
	SysTick->CTRL = 1;
	float temperature = readTemp();
	SysTick->CTRL = 0;
	if((20 < temperature) && (temperature<30))
		STATE_FLAG |= 0x2;
	else
		STATE_FLAG &= 0x1;
	
	statePtr = state_DONE;
}

static void state_DONE(void){
	if(STATE_FLAG == 0x3)
		exitStat = 1;
	else
		statePtr = state_idle;
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
