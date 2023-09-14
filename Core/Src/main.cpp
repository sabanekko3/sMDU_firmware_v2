/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../user_lib/motor.hpp"
#include "../user_lib/motor_measure.hpp"
#include "../user_lib/can.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern "C" {
	int _write(int file, char *ptr, int len) {
		HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len,100);
		return len;
	}
}

//setup led
PWM R(&htim2, TIM_CHANNEL_1, 1000,0,1,false);
PWM G(&htim2, TIM_CHANNEL_2, 1000,0,1,false);
PWM B(&htim2, TIM_CHANNEL_3, 1000,0,1,false);

//setup pwm
PWM U(&htim1, TIM_CHANNEL_3, 980,-1,1,true);
PWM V(&htim1, TIM_CHANNEL_2, 980,-1,1,true);
PWM W(&htim1, TIM_CHANNEL_1, 980,-1,1,true);

DRIVER driver(U,V,W,MD_EN_GPIO_Port,MD_EN_Pin);

ADC_DMA adc1_dma(&hadc1,(int)ADC_data::adc1_n);
ADC_DMA adc2_dma(&hadc2,(int)ADC_data::adc2_n);
ANALOG_SENS analog(adc1_dma,adc2_dma,3.3/(0.05*4096.0),11.0/4096.0*3.3);

AS5600 as5600_enc(&hi2c1);
AB_LINER ab_liner_enc(analog);

std::array<ENCODER*,(int)ENC_type::n> enc_array = {
		&as5600_enc,
		&ab_liner_enc,
};

MOTOR_measure measure(&htim17,driver,analog);
MOTOR motor(driver,analog,enc_array);

CAN_COM can(&hcan,CAN_FILTER_FIFO0);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim7){
		R.out(0.5);
		motor.control();
	}else if(htim == &htim6){

		enc_array[(int)ENC_type::AB_LINER]->read_start();
	}
}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
	enc_array[(int)motor.get_enc_type()]->read_completion_task();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    can.rx_interrupt_task();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t uart_rx_buff[16] = {0};
int rx_ID = 0;
int rx_reg = 0;
float rx_val = 0;
bool uart_rx_flag = false;

float byte_to_float(uint8_t *buff){
	return *(float*)(void*)buff;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	B.out(0.5);
	rx_ID = uart_rx_buff[0];
	rx_reg = uart_rx_buff[1];
	rx_val = byte_to_float(&uart_rx_buff[2]);
	uart_rx_flag = true;

    HAL_UART_Receive_DMA(&huart1, uart_rx_buff, 7);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  R.start();
  G.start();
  B.start();
  G.out(0.5);
  printf("LED OK\r\n");

  //motor peripherals start
  driver.pwms_start();
  analog.init();
  analog.dma_start();

  while(analog.get_power_v() < 12.0){
	  G.out(0);
	  R.out(0.5);
	  HAL_Delay(250);
	  G.out(0.5);
	  R.out(0);
	  HAL_Delay(250);
  }
  G.out(0.5);

  motor.set_enc_type(ENC_type::AB_LINER);
  motor.enc_calibration(0.1,7);
  float motor_R = measure.measure_R(0.5);
  float motor_L = measure.measure_L(motor_R,0.5);
  printf("R:%f,L:%f\r\n",motor_R,motor_L);
  motor.init(7,motor_R,motor_L,1000);
  printf("motor init\r\n");

  //enc timer
   if(HAL_TIM_Base_Start_IT(&htim6) == HAL_OK) printf("enc timer start\r\n");
   else printf("enc timer error\r\n");

   //motor timer
   if(HAL_TIM_Base_Start_IT(&htim7) == HAL_OK) printf("motor timer start\r\n");
   else printf("motor timer error\r\n");

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  can.set_filter_free();
  can_frame_t data;
  dq_t dq_target = {0,0};

  HAL_UART_Receive_DMA(&huart1, uart_rx_buff, 7);
  printf("start\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(can.rx_available()){
		  can.rx(data);
		  if(data.id == 0b1){
			  dq_target.q = *reinterpret_cast<float*>(data.data);
		  }
	  }
	  motor.set_dq_current(dq_target);
	  if(uart_rx_flag){
		  printf("ID:%d,reg:%d,get:%4.3f\r\n",rx_ID,rx_reg,rx_val);
		  dq_target.q = rx_val;
		  uart_rx_flag = false;
	  }

	  //motor.print_debug();

	  HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_2, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(72000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM1_CLKSOURCE_PCLK2);
  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSRC_PLL_DIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
