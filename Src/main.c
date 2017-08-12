/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

#include <math.h>
#include <string.h>

#include "SimCom.h"
#include "ServiceLayer.h"

#include "flash.h"
#include "motion.h"
#include "mpu6050.h"
#include "pwm.h"
#include "pid.h"
#include "sr04.h"
#include "time.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 256 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId sendTaskHandle;
uint32_t sendTaskBuffer[ 256 ];
osStaticThreadDef_t sendTaskControlBlock;
osThreadId receiveTaskHandle;
uint32_t receiveTaskBuffer[ 512 ];
osStaticThreadDef_t receiveTaskControlBlock;
osThreadId controlTaskHandle;
uint32_t controlTaskBuffer[ 512 ];
osStaticThreadDef_t controlTaskControlBlock;
osThreadId tasksTaskHandle;
uint32_t tasksTaskBuffer[ 256 ];
osStaticThreadDef_t tasksTaskControlBlock;
osMutexId sl_send_lockHandle;
osStaticMutexDef_t sl_send_lockControlBlock;
osMutexId dl_send_lockHandle;
osStaticMutexDef_t dl_send_lockControlBlock;
osMutexId ph_send_lockHandle;
osStaticMutexDef_t ph_send_lockControlBlock;
osMutexId ks_lockHandle;
osStaticMutexDef_t ks_lockControlBlock;
osMutexId task_lockHandle;
osStaticMutexDef_t task_lockControlBlock;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

struct kine_state ks = {0};
char currentTask = '0';

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM13_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
extern void StartSendTask(void const * argument);
extern void StartReceiveTask(void const * argument);
void StartControlTask(void const * argument);
void StartTasksTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM13_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  simcom_init(&huart1);

  time_init(&htim13);

  flash_init();

  resistor_init(&hadc1);
  resistor_start_dma();

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of sl_send_lock */
  osMutexStaticDef(sl_send_lock, &sl_send_lockControlBlock);
  sl_send_lockHandle = osMutexCreate(osMutex(sl_send_lock));

  /* definition and creation of dl_send_lock */
  osMutexStaticDef(dl_send_lock, &dl_send_lockControlBlock);
  dl_send_lockHandle = osMutexCreate(osMutex(dl_send_lock));

  /* definition and creation of ph_send_lock */
  osMutexStaticDef(ph_send_lock, &ph_send_lockControlBlock);
  ph_send_lockHandle = osMutexCreate(osMutex(ph_send_lock));

  /* definition and creation of ks_lock */
  osMutexStaticDef(ks_lock, &ks_lockControlBlock);
  ks_lockHandle = osMutexCreate(osMutex(ks_lock));

  /* definition and creation of task_lock */
  osMutexStaticDef(task_lock, &task_lockControlBlock);
  task_lockHandle = osMutexCreate(osMutex(task_lock));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of sendTask */
  osThreadStaticDef(sendTask, StartSendTask, osPriorityNormal, 0, 256, sendTaskBuffer, &sendTaskControlBlock);
  sendTaskHandle = osThreadCreate(osThread(sendTask), NULL);

  /* definition and creation of receiveTask */
  osThreadStaticDef(receiveTask, StartReceiveTask, osPriorityNormal, 0, 512, receiveTaskBuffer, &receiveTaskControlBlock);
  receiveTaskHandle = osThreadCreate(osThread(receiveTask), NULL);

  /* definition and creation of controlTask */
  osThreadStaticDef(controlTask, StartControlTask, osPriorityRealtime, 0, 512, controlTaskBuffer, &controlTaskControlBlock);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* definition and creation of tasksTask */
  osThreadStaticDef(tasksTask, StartTasksTask, osPriorityHigh, 0, 256, tasksTaskBuffer, &tasksTaskControlBlock);
  tasksTaskHandle = osThreadCreate(osThread(tasksTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/8000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 40;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM13 init function */
static void MX_TIM13_Init(void)
{

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 40;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 100;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart1, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE9 PE10 PE11 
                           PE12 PE13 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

#ifdef MPU6050_USE_MAG

void akm8963_calib()
{
	extern bool akm8963_calib_flag;
	akm8963_calib_flag = true;

	extern float mxd, myd, mzd, mxs, mys, mzs;

	sl_send(8, 1, "Calibrating", 12);
	osDelay(10);

	const float scale = 60.0f;

	float mmax[3] = {-MAG_RANGE, -MAG_RANGE, -MAG_RANGE};
	float mmin[3] = {MAG_RANGE, MAG_RANGE, MAG_RANGE};
	float *ms = &(ks.mx);
	float lms[3];

	for(int i = 0; i < 300; i++) {
		for(int j = 0; j < 3; j++) {
			lms[j] = 0;
		}

		for(int k = 0; k < 5; k++) {
			osMutexWait(ks_lockHandle, osWaitForever);

			for(int j = 0; j < 3; j++) {
				lms[j] += ms[j];
			}

			osMutexRelease(ks_lockHandle);

			osDelay(10);
		}


		for(int j = 0; j < 3; j++) {

			lms[j] /= 5;

			if(lms[j] > mmax[j]) {
				mmax[j] = lms[j];
			}
			if(lms[j] < mmin[j]) {
				mmin[j] = lms[j];
			}
		}
	}

	mxd = (mmax[0] + mmin[0]) / 2;
	mxs = scale / ((mmax[0] - mmin[0]) / 2);

	myd = (mmax[1] + mmin[1]) / 2;
	mys = scale / ((mmax[1] - mmin[1]) / 2);

	mzd = (mmax[2] + mmin[2]) / 2;
	mzs = scale / ((mmax[2] - mmin[2]) / 2);

	akm8963_calib_flag = false;

	flash_write(MAG_CALIB_FLASH_ADDR, (uint8_t*)(&mxd), 4);
	flash_write(MAG_CALIB_FLASH_ADDR + 4, (uint8_t*)(&myd), 4);
	flash_write(MAG_CALIB_FLASH_ADDR + 8, (uint8_t*)(&mzd), 4);
	flash_write(MAG_CALIB_FLASH_ADDR + 12, (uint8_t*)(&mxs), 4);
	flash_write(MAG_CALIB_FLASH_ADDR + 16, (uint8_t*)(&mys), 4);
	flash_write(MAG_CALIB_FLASH_ADDR + 20, (uint8_t*)(&mzs), 4);

	osDelay(10);
	sl_send(8, 1, "Calibrated", 11);
	sl_send(8, 0, &mxd, 4);
	sl_send(8, 0, &myd, 4);
	sl_send(8, 0, &mzd, 4);
	sl_send(8, 0, &mxs, 4);
	sl_send(8, 0, &mys, 4);
	sl_send(8, 0, &mzs, 4);
	osDelay(10);
}

#endif

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */

  char *msg;

  /* Infinite loop */
  for(int i = 0;; i++)
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
	  osDelay(100);

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
	  osDelay(100);
  }
  /* USER CODE END 5 */ 
}

/* StartControlTask function */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */

  osDelay(200);

  motion_init(&htim2);

  uint32_t counter = 0;

  /* Infinite loop */
  for(;;)
  {
	counter++;

	if(counter % 50 == 0) {
		motion_control();
	}

	if(counter % 5 == 0) {
		motor_control();
	}

	osDelay(1);
  }

  /* USER CODE END StartControlTask */
}

/* StartTasksTask function */
void StartTasksTask(void const * argument)
{
  /* USER CODE BEGIN StartTasksTask */

  char task;

  extern uint16_t holes[9][2];

  /* Infinite loop */
  for(;;)
  {
	  osMutexWait(task_lockHandle, osWaitForever);

	  task = currentTask;

	  osMutexRelease(task_lockHandle);

	  extern mypid_t px, py;

	  switch(task) {
	  case '1':
		  sl_send(2, 2, "Starting task 1", 16);
		  move_to_pos(get_pos_x(), get_pos_y());
		  motor_start();
		  osDelay(6000);
		  motion_reset();
		  move_to_pos(320, 240);
		  sl_send(2, 2, "Stopping task 1", 16);
		  break;
	  case '2':
		  sl_send(2, 2, "Starting task 2", 16);

		  move_to_pos(holes[4][0], holes[4][1]);
		  motion_init_pid();
		  px.kp = 4.5;
		  px.kd = 5.0;

		  py.kp = 4.5;
		  py.kd = 5.0;

		  motor_start();

		  osDelay(15000);

		  motion_reset();
		  move_to_pos(320, 240);
		  sl_send(2, 2, "Stopping task 2", 16);
		  break;
	  case '3':
		  sl_send(2, 2, "Starting task 3", 16);

		  move_to_pos(holes[3][0] - 20, holes[3][1]);
		  motion_init_pid();
		  px.kp = 4.5;
		  px.kd = 0.5;

		  py.kp = 4.5;
		  py.kd = 0.5;

		  motor_start();

		  osDelay(11000);

		  move_to_pos(holes[4][0] + 10, holes[4][1]);

		  osDelay(8000);

		  motion_reset();
		  move_to_pos(320, 240);

		  sl_send(2, 2, "Stopping task 3", 16);
		  break;
	  case '4':
		  sl_send(2, 2, "Starting task 4", 16);

		  move_to_pos(holes[8][0], holes[8][1]);
		  motion_init_pid();
		  px.kp = 1.05;
		  px.kd = 1.5;

		  py.kp = 0.95;
		  py.kd = 1.5;

		  motor_start();


		  osDelay(10000);

		  motion_reset();
		  move_to_pos(320, 240);


		  sl_send(2, 2, "Stopping task 4", 16);
		  break;
	  case '5':
		  sl_send(2, 2, "Starting task 5", 16);

		  move_to_pos(holes[0][0], holes[0][1]);
		  motion_init_pid();
		  px.kp = 3.5;
		  px.kd = 1.5;

		  py.kp = 3.5;
		  py.kd = 1.5;

		  motor_start();

		  move_to_pos(holes[1][0], holes[1][1] - 40);

		  osDelay(6000);

		  px.kp = 3.5;
		  px.kd = 2.5;

		  py.kp = 3.5;
		  py.kd = 2.5;

		  move_to_pos(holes[5][0] + 10, holes[5][1] - 10);

		  osDelay(6000);

		  px.kp = 1.5;
		  px.kd = 1.5;

		  py.kp = 4.0;
		  py.kd = 3.5;

		  move_to_pos(holes[8][0], holes[8][1]);

		  osDelay(6000);

		  motion_reset();
		  move_to_pos(320, 240);


		  sl_send(2, 2, "Stopping task 5", 16);
		  break;
	  case '6':

		  sl_send(2, 2, "Starting task 6", 16);

		  move_to_pos(holes[3][0], holes[3][1]);
		  motion_init_pid();
		  px.kp = 4.0;
		  px.kd = 1.5;

		  py.kp = 4.0;
		  py.kd = 1.5;

		  motor_start();

		  for(int i = 0; i < 3; i++) {
			  move_to_pos(holes[7][0], holes[7][1] - 20);
			  osDelay(2000);

			  move_to_pos(holes[5][0] - 20, holes[5][1]);
			  osDelay(2000);

			  move_to_pos(holes[1][0], holes[1][1] + 20);
			  osDelay(2000);

			  move_to_pos(holes[3][0] + 20, holes[3][1]);
			  osDelay(2000);
		  }

		  move_to_pos(holes[7][0], holes[7][1]);

		  osDelay(2500);

		  move_to_pos(holes[8][0], holes[8][1]);

		  osDelay(10000);

		  motion_reset();
		  move_to_pos(320, 240);


		  sl_send(2, 2, "Stopping task 6", 16);

		  break;
	  default:
		  break;
	  }

	  osMutexWait(task_lockHandle, osWaitForever);
	  currentTask = '0';
	  osMutexRelease(task_lockHandle);


//    while(move_to_hole(0) != true) {
//    	osDelay(1);
//    }
//    osDelay(2000);
//
//    move_to_hole(3);
//
//    osDelay(5000);
//
//    move_to_hole(4);
//
	  osDelay(1);
  }
  /* USER CODE END StartTasksTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

  time_callback(htim);

/* USER CODE END Callback 1 */
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
