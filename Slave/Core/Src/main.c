/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
// === Піни ===
#define TRIG1_PORT GPIOA
#define TRIG1_PIN GPIO_PIN_3
#define LED1_G_PORT GPIOB
#define LED1_G_PIN GPIO_PIN_0
#define LED1_R_PORT GPIOB
#define LED1_R_PIN GPIO_PIN_12


#define TRIG2_PORT GPIOB
#define TRIG2_PIN GPIO_PIN_1
#define LED2_G_PORT GPIOB
#define LED2_G_PIN GPIO_PIN_4
#define LED2_R_PORT GPIOB
#define LED2_R_PIN GPIO_PIN_5


#define TRIG3_PORT GPIOC
#define TRIG3_PIN GPIO_PIN_8
#define LED3_G_PORT GPIOB
#define LED3_G_PIN GPIO_PIN_13
#define LED3_R_PORT GPIOB
#define LED3_R_PIN GPIO_PIN_14


volatile uint32_t IC_First[3] = {0};
volatile uint32_t IC_Second[3] = {0};
volatile uint32_t Pulse_Width[3] = {0};
volatile uint8_t Captured[3] = {0};

// Нова змінна для підтвердження, що вимірювання завершено
volatile uint8_t Measurement_Done[3] = {0};

uint8_t free_spots = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C3_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
float Measure_Distance(uint8_t sensor);
void delay_us(uint32_t us);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

    uint8_t ch = 0;
    uint32_t channel_constant = 0; // <--- ЦЯ ЗМІННА ОБОВ'ЯЗКОВА

    // Визначаємо індекс масиву ТА константу каналу
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        ch = 0;
        channel_constant = TIM_CHANNEL_1; // <--- ВАЖЛИВО
    }
    else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        ch = 1;
        channel_constant = TIM_CHANNEL_2; // <--- ВАЖЛИВО
    }
    else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        ch = 2;
        channel_constant = TIM_CHANNEL_3; // <--- ВАЖЛИВО
    }
    else return;

    if(Captured[ch] == 0) // (1) Початок імпульсу
    {
        // Використовуємо channel_constant!
        IC_First[ch] = HAL_TIM_ReadCapturedValue(htim, channel_constant);
        Captured[ch] = 1;
        // Використовуємо channel_constant!
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel_constant, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else // (2) Кінець імпульсу
    {
        // Використовуємо channel_constant!
        IC_Second[ch] = HAL_TIM_ReadCapturedValue(htim, channel_constant);

        if(IC_Second[ch] >= IC_First[ch])
            Pulse_Width[ch] = IC_Second[ch] - IC_First[ch];
        else
            Pulse_Width[ch] = (0xFFFF - IC_First[ch] + IC_Second[ch]);

        Captured[ch] = 0;
        Measurement_Done[ch] = 1;

        // Використовуємо channel_constant!
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel_constant, TIM_INPUTCHANNELPOLARITY_RISING);
    }
}

float Measure_Distance(uint8_t sensor)
{
    GPIO_TypeDef* port;
    uint16_t pin;

    // Оголошення змінних (ЗБЕРІГАЄМО)
    uint32_t start_time = HAL_GetTick(); // Для таймауту в мс
    uint32_t timeout_ms = 40;            // Максимальний час очікування

    TIM_HandleTypeDef* htim = &htim1; // Посилання на ваш таймер
    uint32_t channel;

    // ... (Визначення port, pin, channel)
    if(sensor == 0)      { port = TRIG1_PORT; pin = TRIG1_PIN; channel = TIM_CHANNEL_1; }
    else if(sensor == 1) { port = TRIG2_PORT; pin = TRIG2_PIN; channel = TIM_CHANNEL_2; }
    else                 { port = TRIG3_PORT; pin = TRIG3_PIN; channel = TIM_CHANNEL_3; }


    // 0. ЗУПИНЯЄМО ТА СКИДАЄМО ПРАПОРИ ПЕРЕД ПОЧАТКОМ
    HAL_TIM_IC_Stop_IT(htim, channel); // Зупиняємо переривання на цьому каналі

    // 1. Скидаємо прапори перед ініціацією
    Measurement_Done[sensor] = 0;
    Captured[sensor] = 0;
    Pulse_Width[sensor] = 0;

    // Скидаємо полярність на RISING
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_RISING);

    // ОНОВЛЮЄМО ЧАС ПОЧАТКУ (ВИДАЛИТИ `uint32_t`)
    start_time = HAL_GetTick();

    // 2. Ініціація імпульсу TRIG
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

    // 3. ЗАПУСКАЄМО ПЕРЕРИВАННЯ, ЩОБ ВОНО СЛУХАЛО
    HAL_TIM_IC_Start_IT(htim, channel);

    // 4. Очікування завершення вимірювання ECHO в перериванні
    // ВИКОРИСТОВУЄМО ІСНУЮЧУ timeout_ms (ВИДАЛИТИ `uint32_t`)
    while (Measurement_Done[sensor] == 0 && (HAL_GetTick() - start_time) < timeout_ms)
    {
        // Чекаємо
    }

    // 5. Обов'язково зупиняємо переривання після завершення (або таймауту)
    HAL_TIM_IC_Stop_IT(htim, channel);

    // 6. Перевірка результату та таймауту
    if (Measurement_Done[sensor] == 0 || Pulse_Width[sensor] == 0)
    {
        return 400.0f;
    }

    // 7. Розрахунок відстані
    return (float)Pulse_Width[sensor] * 0.017f;
}

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
  // Включаємо DWT (для точної microsecond delay)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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
  MX_USB_HOST_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

/*  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3); */

  HAL_I2C_Slave_Transmit_IT(&hi2c3, &free_spots, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  float d1 = Measure_Distance(0);
	  float d2 = Measure_Distance(1);
	  float d3 = Measure_Distance(2);


	  if(d1 < 20) {
	  HAL_GPIO_WritePin(LED1_G_PORT, LED1_G_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED1_R_PORT, LED1_R_PIN, GPIO_PIN_SET);
	  } else {
	  HAL_GPIO_WritePin(LED1_G_PORT, LED1_G_PIN, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LED1_R_PORT, LED1_R_PIN, GPIO_PIN_RESET);
	  }


	  if(d2 < 20) {
	  HAL_GPIO_WritePin(LED2_G_PORT, LED2_G_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED2_R_PORT, LED2_R_PIN, GPIO_PIN_SET);
	  } else {
	  HAL_GPIO_WritePin(LED2_G_PORT, LED2_G_PIN, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LED2_R_PORT, LED2_R_PIN, GPIO_PIN_RESET);
	  }


	  if(d3 < 20) {
	  HAL_GPIO_WritePin(LED3_G_PORT, LED3_G_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED3_R_PORT, LED3_R_PIN, GPIO_PIN_SET);
	  } else {
	  HAL_GPIO_WritePin(LED3_G_PORT, LED3_G_PIN, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LED3_R_PORT, LED3_R_PIN, GPIO_PIN_RESET);
	  }

	  uint8_t count = 0;

	        // Якщо відстань >= 20, значить місце вільне (горить зелений)
	  if (d1 >= 20) count++;
	  if (d2 >= 20) count++;
	  if (d3 >= 20) count++;

	        // Оновлюємо глобальну змінну для I2C
	  free_spots = count;

	  HAL_Delay(100);

 /* while (1)
     {

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // PC0
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); // PC1
	  HAL_Delay(1000);
         // 1. Перемикаємо всі світлодіоди (LED)
         // PB0, PB12 (Датчик 1) | PB4, PB5 (Датчик 2) | PB13, PB14 (Датчик 3)
         HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_13|GPIO_PIN_14);
         HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2|GPIO_PIN_1);

         // 2. Перемикаємо всі TRIG (Запуск)
         HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3); // TRIG 1
         HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1); // TRIG 2
         HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // TRIG 3

         // 3. Перемикаємо всі ECHO (Тільки для тесту! Ми зробили їх виходами)
         HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13);

         // Затримка 1 секунда, щоб було видно оком
         HAL_Delay(1000);*/

    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 10000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 96;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 0;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  // Вимикаємо всі LED спочатку
  HAL_GPIO_WritePin(LED1_R_PORT, LED1_R_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED1_G_PORT, LED1_G_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_R_PORT, LED2_R_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_G_PORT, LED2_G_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_R_PORT, LED3_R_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_G_PORT, LED3_G_PIN, GPIO_PIN_RESET);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin PC1 PC2 PC4
                           PC8 */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB12 PB13
                           PB14 PB4 PB5 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void delay_us(uint32_t us)
{
    // Отримуємо тактову частоту ядра (на STM32F407 це 168 МГц)
    const uint32_t num_cycles = SystemCoreClock / 1000000;

    // Розраховуємо кінцеве значення лічильника
    uint32_t start_cycles = DWT->CYCCNT;
    uint32_t target_cycles = start_cycles + (us * num_cycles);

    // Чекаємо, враховуючи можливе переповнення 32-бітного лічильника
    if (target_cycles > start_cycles) {
        while (DWT->CYCCNT < target_cycles);
    } else {
        // Обробка переповнення (якщо воно відбулося)
        while (DWT->CYCCNT > start_cycles);
        while (DWT->CYCCNT < target_cycles);
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Перевіряємо, чи подія прийшла від I2C3
    if (hi2c->Instance == I2C3)
    {
        // Перезапускаємо передачу для наступного запиту
        HAL_I2C_Slave_Transmit_IT(&hi2c3, &free_spots, 1);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    // Якщо сталася помилка (шум, розрив зв'язку), ця функція спрацює.
    if (hi2c->Instance == I2C3)
    {
        // Ми просто скидаємо помилки і знову запускаємо прослуховування!
        // Це "воскрешає" Слейва після збою.
        HAL_I2C_DeInit(&hi2c3);
        HAL_I2C_Init(&hi2c3);
        HAL_I2C_Slave_Transmit_IT(&hi2c3, &free_spots, 1);
    }
}
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
#ifdef USE_FULL_ASSERT
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
