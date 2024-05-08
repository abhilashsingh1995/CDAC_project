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

// LCD Commands
#define LCD_CMD_INIT1      0x22
#define LCD_CMD_INIT2      0x28
#define LCD_CMD_CLR        0x01
#define LCD_CMD_CRHOME     0x02
#define LCD_CMD_DONCROFF   0x0c
#define LCD_CMD_EMINC      0x06
#define LCD_CMD_CRLINE1    0x80
#define LCD_CMD_CRLINE2    0xc0

#define LCD_PORT GPIOE
#define LCD_RS   0
#define LCD_EN   1
#define LCD_D4   2
#define LCD_D5   3
#define LCD_D6   4
#define LCD_D7   5

#define LED_PORT GPIOC
#define LED_SIG  6
#define LED_ALT  7

#define BUZZ_PORT GPIOC
#define BUZZ      9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId sled_task_handle;
osThreadId aled_task_handle;
osThreadId buzz_task_handle;
osThreadId lcd_task_handle;

uint8_t aled_flag = 0;
uint8_t buzz_flag = 0;
char lcd_line1_buff[17];
char lcd_line2_buff[17];
const uint8_t lcd_pins[6] = {LCD_RS, LCD_EN, LCD_D4,
							 LCD_D5, LCD_D6, LCD_D7};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void sled_task(void const*);
void aled_task(void const*);
void buzz_task(void const*);
void lcd_task(void const*);

void lcd_init(GPIO_TypeDef*, const uint8_t*);
void lcd_cmd(GPIO_TypeDef*, const uint8_t*, uint8_t);
void lcd_data(GPIO_TypeDef*, const uint8_t*, uint8_t);
void lcd_write(GPIO_TypeDef*, const uint8_t*, uint8_t);
void lcd_string(GPIO_TypeDef*, const uint8_t*, char*);
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
  /* USER CODE BEGIN 2 */
  lcd_init(LCD_PORT, lcd_pins);
  bzero(lcd_line1_buff, 0);
  bzero(lcd_line2_buff, 0);
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

  osThreadDef(SLEDTASK, sled_task, osPriorityAboveNormal, 0, 128);
  sled_task_handle = osThreadCreate(osThread(SLEDTASK), NULL);

  osThreadDef(ALEDTASK, aled_task, osPriorityAboveNormal, 0, 128);
  aled_task_handle = osThreadCreate(osThread(ALEDTASK), NULL);

  osThreadDef(BUZZTASK, buzz_task, osPriorityAboveNormal, 0, 128);
  buzz_task_handle = osThreadCreate(osThread(BUZZTASK), NULL);

  osThreadDef(LCDTASK, lcd_task, osPriorityAboveNormal, 0, 128);
  lcd_task_handle = osThreadCreate(osThread(LCDTASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void sled_task(void const *arg)
{
	while(1)
	{
		HAL_GPIO_WritePin(LED_PORT, (1 << LED_SIG), 1);
		osDelay(100);
		HAL_GPIO_WritePin(LED_PORT, (1 << LED_SIG), 0);
		osDelay(2000);
	}
}

void aled_task(void const *arg)
{
	while(1)
	{
		if(aled_flag == 1)
			HAL_GPIO_WritePin(LED_PORT, (1 << LED_ALT), 1);
		else
			HAL_GPIO_WritePin(LED_PORT, (1 << LED_ALT), 0);
		osDelay(10);
	}
}

void buzz_task(void const *arg)
{
	while(1)
	{
		if(buzz_flag == 1)
		{
			HAL_GPIO_WritePin(BUZZ_PORT, (1 << BUZZ), 1);
			osDelay(100);
		}
		else
		{
			HAL_GPIO_WritePin(BUZZ_PORT, (1 << BUZZ), 0);
		}
		osDelay(10);
	}
}

void lcd_task(void const *arg)
{
	while(1)
	{
		lcd_cmd(LCD_PORT, lcd_pins, LCD_CMD_CLR);
		lcd_cmd(LCD_PORT, lcd_pins, LCD_CMD_CRLINE1);
		lcd_string(LCD_PORT, lcd_pins, lcd_line1_buff);
		lcd_cmd(LCD_PORT, lcd_pins, LCD_CMD_CRLINE2);
		lcd_string(LCD_PORT, lcd_pins, lcd_line2_buff);
		osDelay(250);
	}
}

void lcd_init(GPIO_TypeDef *lcdport, const uint8_t *lcdpins)
{
	HAL_GPIO_WritePin(lcdport, (1 << *(lcdpins+0)), 0);
	HAL_GPIO_WritePin(lcdport, (1 << *(lcdpins+1)), 0);
	HAL_Delay(100);
	lcd_cmd(lcdport, lcdpins, 0x02);HAL_Delay(3);
	lcd_cmd(lcdport, lcdpins, LCD_CMD_INIT1);HAL_Delay(3);
	lcd_cmd(lcdport, lcdpins, LCD_CMD_INIT2);HAL_Delay(3);
	lcd_cmd(lcdport, lcdpins, LCD_CMD_CRHOME);
	lcd_cmd(lcdport, lcdpins, LCD_CMD_EMINC);
	lcd_cmd(lcdport, lcdpins, LCD_CMD_DONCROFF);
	lcd_cmd(lcdport, lcdpins, LCD_CMD_CLR);
}

void lcd_cmd(GPIO_TypeDef *lcdport, const uint8_t *lcdpins, uint8_t cmd)
{
	HAL_GPIO_WritePin(lcdport, (1 << *(lcdpins+0)), 0);
	lcd_write(lcdport, lcdpins, cmd);
	if(cmd == LCD_CMD_CLR)
		HAL_Delay(5);
}

void lcd_data(GPIO_TypeDef *lcdport, const uint8_t *lcdpins, uint8_t data)
{
	HAL_GPIO_WritePin(lcdport, (1 << *(lcdpins+0)), 1);
	lcd_write(lcdport, lcdpins, data);
}

void lcd_write(GPIO_TypeDef *lcdport, const uint8_t *lcdpins, uint8_t byte)
{
	uint8_t i;
	for(i = 0 ; i < 4 ; i++)
	{
		if(((byte >> 4) & (1 << i)))
			HAL_GPIO_WritePin(lcdport, (1 << *(lcdpins+i+2)), 1);
		else
			HAL_GPIO_WritePin(lcdport, (1 << *(lcdpins+i+2)), 0);
	}
	HAL_GPIO_WritePin(lcdport, (1 << *(lcdpins+1)), 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(lcdport, (1 << *(lcdpins+1)), 0);
	HAL_Delay(1);

	for(i = 0 ; i < 4 ; i++)
	{
		if((byte & (1 << i)))
			HAL_GPIO_WritePin(lcdport, (1 << *(lcdpins+i+2)), 1);
		else
			HAL_GPIO_WritePin(lcdport, (1 << *(lcdpins+i+2)), 0);
	}
	HAL_GPIO_WritePin(lcdport, (1 << *(lcdpins+1)), 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(lcdport, (1 << *(lcdpins+1)), 0);
	HAL_Delay(1);
}

void lcd_string(GPIO_TypeDef *lcdport, const uint8_t *lcdpins, char *strptr)
{
	while(*strptr != '\0')
	{
		lcd_data(lcdport, lcdpins, (uint8_t)*strptr);
		strptr++;
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
