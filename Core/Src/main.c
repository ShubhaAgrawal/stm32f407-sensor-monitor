/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "cJSON.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint32_t timestamp;
	float temperature;
	float accel_x;
	float accel_y;
	float accel_z;
	uint8_t status;
} sensor_reading_t;

#define BUFFER_SIZE 10

typedef struct{
	sensor_reading_t readings[BUFFER_SIZE];
	uint8_t head;
	uint8_t tail;
	uint8_t count;
} circular_buffer_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

osThreadId DisplayTaskHandle;
osThreadId SensorTaskHandle;
/* USER CODE BEGIN PV */
volatile uint32_t last_button_press = 0;
volatile uint32_t seconds_elapsed = 0;
circular_buffer_t sensor_buffer;
volatile uint8_t system_status;
osMessageQId sensorQueueHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
void StartDisplayTask(void const * argument);
void StartTSensorTask(void const * argument);

/* USER CODE BEGIN PFP */
void led_green_on(void);
void led_green_off(void);
void led_green_toggle(void);
void led_orange_on(void);
void led_orange_off(void);
void led_orange_toggle(void);
void led_red_on(void);
void led_red_off(void);
void led_red_toggle(void);
void led_blue_on(void);
void led_blue_off(void);
void led_blue_toggle(void);
void led_all_off(void);
void uart_send_string(char*str);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
uint8_t spi_read_register(uint8_t reg_addr);
void spi_write_register(uint8_t reg_addr, uint8_t value);
void read_accelerometer(float *ax, float *ay, float *az);
float read_temperature(void);
void buffer_init(circular_buffer_t *buf);
bool buffer_is_full(circular_buffer_t *buf);
bool buffer_is_empty(circular_buffer_t *buf);
bool buffer_push(circular_buffer_t *buf, sensor_reading_t *reading);
bool buffer_pop(circular_buffer_t *buf, sensor_reading_t *reading);
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
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  buffer_init(&sensor_buffer);

  uint8_t who_am_i = spi_read_register(0x0F);
  if(who_am_i == 0x3F)
  {
	  //led_green_on();
  }
  else
  {
	  //led_red_on();
  }
  spi_write_register(0x20, 0x67);

  float temp = read_temperature();
  if (temp > 5.0f && temp < 50.0f) {
      led_blue_on();
  } else {
      led_red_on();
  }
  HAL_Delay(1000);

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
  osMessageQDef(sensorQueue, 10, sesnor_reading_t);
  sensorQueueHandle = osMessageCreate(osMessageQ(sensorQueue), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of DisplayTask */
  osThreadDef(DisplayTask, StartDisplayTask, osPriorityNormal, 0, 512);
  DisplayTaskHandle = osThreadCreate(osThread(DisplayTask), NULL);

  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, StartTSensorTask, osPriorityAboveNormal, 0, 256);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

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
	  float ax, ay, az;
	  read_accelerometer(&ax, &ay, &az);
	  if (az > 0.8f) {
	      //led_blue_on();   // board is flat — Z near 1g
	  } else {
	      //led_blue_off();  // board is tilted
	  }
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void led_green_on(void)    {HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);}
void led_green_off(void)   { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); }
void led_green_toggle(void) {HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);}
void led_orange_on(void)    { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); }
void led_orange_off(void)   { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); }
void led_orange_toggle(void){ HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); }

void led_red_on(void)    { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); }
void led_red_off(void)   { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); }
void led_red_toggle(void){ HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); }

void led_blue_on(void)    { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); }
void led_blue_off(void)   { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); }
void led_blue_toggle(void){ HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15); }

void led_all_off(void)
{
	led_green_off();
	led_orange_off();
	led_red_off();
	led_blue_off();
}

void uart_send_string(char*str)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
	return len;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		uint32_t now = HAL_GetTick();
		if(now-last_button_press > 200)
		{
			last_button_press = now;
			led_green_toggle();
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		seconds_elapsed++;
		led_orange_toggle();

        sensor_reading_t reading;
        reading.timestamp = HAL_GetTick();
        reading.temperature = read_temperature();
        read_accelerometer(&reading.accel_x, &reading.accel_y, &reading.accel_z);
        reading.status = 0;

        buffer_push(&sensor_buffer, &reading);
		}
}

uint8_t spi_read_register(uint8_t reg_addr)
{
	uint8_t tx_byte = reg_addr | 0x80;
	uint8_t rx_byte = 0;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &tx_byte, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &rx_byte, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	return rx_byte;
}

void spi_write_register(uint8_t reg_addr, uint8_t value)
{
	uint8_t tx_data[2];
	tx_data[0] = reg_addr;
	tx_data[1] = value;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, tx_data, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

void read_accelerometer(float *ax, float *ay, float *az)
{
	int16_t raw_x, raw_y, raw_z;

	uint8_t xl = spi_read_register(0x28);
	uint8_t xh = spi_read_register(0x29);
	uint8_t yl = spi_read_register(0x2A);
	uint8_t yh = spi_read_register(0x2B);
	uint8_t zl = spi_read_register(0x2C);
	uint8_t zh = spi_read_register(0x2D);

	raw_x = (int16_t)((xh<<8) | xl);
	raw_y = (int16_t)((yh<<8) | yl);
	raw_z = (int16_t)((zh<<8) | zl);

    *ax = (float)raw_x * 0.061f / 1000.0f;
    *ay = (float)raw_y * 0.061f / 1000.0f;
    *az = (float)raw_z * 0.061f / 1000.0f;

}

float read_temperature(void)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	float voltage = (float)raw_adc * 3.3f/4096.0f;
	float temperature = ((voltage-0.76f)/0.0025f) + 25.0f;
    // Calibration offset — internal sensor reads ~35°C higher than actual
    temperature -= 35.0f;
	return temperature;
}

void buffer_init(circular_buffer_t *buf)
{
	buf->head = 0;
	buf->tail = 0;
	buf->count = 0;
}

bool buffer_is_full(circular_buffer_t *buf)
{
	return (buf->count == BUFFER_SIZE);
}

bool buffer_is_empty(circular_buffer_t *buf)
{
	return(buf->count == 0);
}

bool buffer_push(circular_buffer_t *buf, sensor_reading_t *reading)
{
	buf->readings[buf->head] = *reading;
	buf->head = (buf->head+1) % BUFFER_SIZE;
	if(buf->count == BUFFER_SIZE)
	{
		buf->tail = (buf->tail+1) % BUFFER_SIZE;
	}
	else
	{
		buf->count++;
	}
	return true;
}

bool buffer_pop(circular_buffer_t *buf, sensor_reading_t *reading)
{
	if (buffer_is_empty(buf)){return false;}

	*reading = buf->readings[buf->tail];
	buf->tail = (buf->tail + 1) % BUFFER_SIZE;
	buf->count--;
	return true;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDisplayTask */
/**
  * @brief  Function implementing the DisplayTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  osDelay(200);
  /* Infinite loop */
  for(;;)
  {
	  osEvent event = osMessageGet(sensorQueueHandle, osWaitForever);

	  if(event.status == osEventMessage)
	  {
		  sensor_reading_t *reading = (sensor_reading_t *)event.value.p;
          char msg[150];
          sprintf(msg, "T:%lu  X:%.2f  Y:%.2f  Z:%.2f  Temp:%.1f  Status:0x%02X\r\n",
                  reading->timestamp,
                  reading->accel_x, reading->accel_y, reading->accel_z,
                  reading->temperature,
                  reading->status);
          HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	  }

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTSensorTask */
void StartTSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartTSensorTask */

	osDelay(100);
	  uint8_t who_am_i = spi_read_register(0x0F);
	  if (who_am_i == 0x3F) {
	      status_set(STATUS_ACCEL_OK);
	      spi_write_register(0x20, 0x67);
	  } else {
	      status_set(STATUS_SPI_ERROR);
	  }

	  float temp = read_temperature();
	  if (temp > 5.0f && temp < 50.0f) {
	      status_set(STATUS_TEMP_OK);
	  }

	  update_status_leds();

  /* Infinite loop */
  for(;;)
  {
      sensor_reading_t reading;
      reading.timestamp = HAL_GetTick();
      reading.temperature = read_temperature();
      read_accelerometer(&reading.accel_x, &reading.accel_y, &reading.accel_z);
      reading.status = system_status;

      osMessagePut(sensorQueueHandle, (uint8_t)&reading, 100);

      led_blue_on();
      osDelay(50);
      led_blue_off();

    osDelay(950);
  }
  /* USER CODE END StartTSensorTask */
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
