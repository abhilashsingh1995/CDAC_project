/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// GPIOs
#define USS_PORT           GPIOE
#define SB_USS_TRIG        0
#define SB_USS_ECHO        1
#define OBJ_USS_TRIG       2
#define OBJ_USS_ECHO       3
// BMP180 Commands
#define BMP180_DEV_ADDR    0x77
#define BMP180_CHIPID      0xd0
#define BMP180_CTRL_MEAS   0xf4
#define BMP180_OUT_MSB     0xf6
#define BMP180_OUT_LSB     0xf7
#define BMP180_OUT_XLSB    0xf8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
uint16_t sb_uss_distance = 0;
uint16_t obj_uss_distance = 0;
int64_t air_pressure = 0;
uint8_t drowsiness = 0;

osThreadId sb_uss_task_handle;
osThreadId obj_uss_task_handle;
osThreadId air_pressure_task_handle;
osThreadId drowsiness_task_handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
uint32_t get_millis(TIM_HandleTypeDef*);
uint16_t get_distance_cm(TIM_HandleTypeDef*, GPIO_TypeDef*, uint8_t, uint8_t);
int64_t bmp180_uncompensated_pressure(I2C_HandleTypeDef*, uint8_t);

// Tasks
void sb_uss_task(void const*);
void obj_uss_task(void const*);
void air_pressure_task(void const*);
void drowsiness_task(void const*);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  htim2.Instance->CNT = 0;
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(SBUSS, sb_uss_task, osPriorityAboveNormal, 0, 128);
  sb_uss_task_handle = osThreadCreate(osThread(SBUSS), NULL);

  osThreadDef(OBJUSS, obj_uss_task, osPriorityAboveNormal, 0, 128);
  obj_uss_task_handle = osThreadCreate(osThread(OBJUSS), NULL);

  osThreadDef(PSENS, air_pressure_task, osPriorityHigh, 0, 128);
  air_pressure_task_handle = osThreadCreate(osThread(PSENS), NULL);

  osThreadDef(DDECT, drowsiness_task, osPriorityAboveNormal, 0, 128);
  drowsiness_task_handle = osThreadCreate(osThread(DDECT), NULL);
  /* USER CODE END RTOS_THREADS */

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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE3 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint16_t get_distance_cm(TIM_HandleTypeDef *htim_ptr, GPIO_TypeDef *port, uint8_t trig, uint8_t echo)
{
	uint16_t dist = 0;
	uint32_t t1, t2;
	uint32_t timeout_millis;

	HAL_GPIO_WritePin(port, (1<<trig), 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(port, (1<<trig), 0);

	timeout_millis = get_millis(&htim2);
	while(HAL_GPIO_ReadPin(port, (1<<echo)) == 1 && (get_millis(&htim2)-timeout_millis) < 14);
	if((get_millis(&htim2)-timeout_millis) >= 14)
		return(0);
	timeout_millis = get_millis(&htim2);
	while(HAL_GPIO_ReadPin(port, (1<<echo)) == 0 && (get_millis(&htim2)-timeout_millis) < 14);
	if((get_millis(&htim2)-timeout_millis) >= 14)
		return(0);
	t1 = htim_ptr->Instance->CNT;
	timeout_millis = get_millis(&htim2);
	while(HAL_GPIO_ReadPin(port, (1<<echo)) == 1 && (get_millis(&htim2)-timeout_millis) < 14);
	if((get_millis(&htim2)-timeout_millis) >= 14)
		return(0);
	t2 = htim_ptr->Instance->CNT;

	dist = (uint32_t)((t2-t1)/58);
	return(dist);
}

uint32_t get_millis(TIM_HandleTypeDef *htim_ptr)
{
	uint32_t millis = 0;
	millis = (uint32_t)((htim_ptr->Instance->CNT)/1000);
	return(millis);
}

int64_t bmp180_uncompensated_pressure(I2C_HandleTypeDef *i2cptr, uint8_t oss)
{
	int64_t uc_pressure = 0;
	uint8_t buff[4] = {0, 0, 0, 0};

	buff[0] = 0x34;
	buff[0] |= (oss << 6);
	HAL_I2C_Mem_Write(i2cptr, (BMP180_DEV_ADDR << 1), BMP180_CTRL_MEAS, 1, buff, 1, HAL_MAX_DELAY);
	HAL_Delay(10);
	HAL_I2C_Mem_Read(i2cptr, (BMP180_DEV_ADDR << 1) | 1, BMP180_OUT_MSB, 1, buff, 3, HAL_MAX_DELAY);
	uc_pressure = (int64_t)(((buff[0]<<16)+(buff[1]<<8)+buff[2]) >> (8-oss));
	return(uc_pressure);
}

// Tasks
void sb_uss_task(void const *arg)
{
	while(1)
	{
		sb_uss_distance = get_distance_cm(&htim2, USS_PORT, SB_USS_TRIG, SB_USS_ECHO);
		osDelay(200);
	}
}

void obj_uss_task(void const *arg)
{
	while(1)
	{
		obj_uss_distance = get_distance_cm(&htim2, USS_PORT, OBJ_USS_TRIG, OBJ_USS_ECHO);
		osDelay(200);
	}
}

void air_pressure_task(void const *arg)
{
	while(1)
	{
		air_pressure = bmp180_uncompensated_pressure(&hi2c1, 2);
		osDelay(10);
	}
}

void drowsiness_task(void const *arg)
{
	char buff[6];
	while(1)
	{
		bzero(buff, 6);
		HAL_UART_Receive(&huart4, (uint8_t*)buff, 3, HAL_MAX_DELAY);
		if(strncmp(buff, "CCC", 3) == 0)
			drowsiness = 1;
		osDelay(2);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
