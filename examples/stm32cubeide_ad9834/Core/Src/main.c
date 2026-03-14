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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AD983x.h"
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

static AD983x_Handle_t ad9834_dds = {
    .gpioPort                     = GPIOA,                     /* GPIO port used for DDS control pins              */
    .serialDataGpioPin            = GPIO_PIN_7,                /* SDATA: serial data line to the DDS               */
    .serialClockGpioPin           = GPIO_PIN_5,                /* SCLK: serial clock for shifting data             */
    .frameSyncGpioPin             = GPIO_PIN_0,                /* FSYNC: frame sync / chip select for DDS          */
    .spiHandle                    = &hspi1,                    /* Hardware SPI handle (NULL for software SPI)      */
    .masterClockFrequencyHz       = 75000000u,                 /* 75 MHz MCLK input provided to AD9834             */
    .currentWaveformType          = AD983X_WAVE_SINE,          /* Default waveform output (sine wave)              */
    .activeFrequencyRegister      = AD983X_REG_0,              /* Currently selected frequency register (FREQ0)    */
    .activePhaseRegister          = AD983X_REG_0,              /* Currently selected phase register (PHASE0)       */
    .sleep1BitState               = AD983X_SLEEP_DISABLED,     /* DAC sleep control bit state                      */
    .sleep12BitState              = AD983X_SLEEP_DISABLED,     /* Master clock / internal sleep control            */
    .deviceType                   = AD983X_DEVICE_AD9834,      /* Specifies the DDS device type                    */
    .ad9834RegSelectMode          = AD9834_REG_SELECT_SW,      /* Register selection controlled by software        */
    .ad9834SquareMode             = AD9834_SQUARE_DISABLED,    /* Comparator square wave output disabled           */
    .frequencyHz                  = { 0u, 0u },                /* Stored frequencies for FREQ0 and FREQ1 registers */
    .phaseDeg                     = { 0u, 0u },                /* Stored phases for PHASE0 and PHASE1 registers    */
    .ad9834ComparatorVinAvailable = 0u                         /* Comparator VIN pin availability flag             */
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize: sine 1 kHz — scope should show 1 kHz sine */
  AD983x_Init(&ad9834_dds, AD983X_WAVE_SINE, 1000u, 0u);
  HAL_Delay(3000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		/* --- Frequency tests (sine waveform) --- */

		/* 10 kHz sine — scope: 10 kHz sine */
		AD983x_SetFrequencyHz(&ad9834_dds, AD983X_REG_0, 10000u);
		HAL_Delay(3000);

		/* 100 kHz sine — scope: 100 kHz sine */
		AD983x_SetFrequencyHz(&ad9834_dds, AD983X_REG_0, 100000u);
		HAL_Delay(3000);

		/* 1 MHz sine — scope: 1 MHz sine */
		AD983x_SetFrequencyHz(&ad9834_dds, AD983X_REG_0, 1000000u);
		HAL_Delay(3000);

		/* Back to 1 kHz for register switching tests */
		AD983x_SetFrequencyHz(&ad9834_dds, AD983X_REG_0, 1000u);
		HAL_Delay(3000);

		/* --- FREQ register switching tests --- */

		/* Load 5 kHz into FREQ1 — scope: still 1 kHz (FREQ0 active) */
		AD983x_SetFrequencyHz(&ad9834_dds, AD983X_REG_1, 5000u);
		HAL_Delay(3000);

		/* Switch to FREQ1 — scope: jumps to 5 kHz */
		AD983x_SelectFreqReg(&ad9834_dds, AD983X_REG_1);
		HAL_Delay(3000);

		/* Switch back to FREQ0 — scope: returns to 1 kHz */
		AD983x_SelectFreqReg(&ad9834_dds, AD983X_REG_0);
		HAL_Delay(3000);

		/* --- Waveform tests (1 kHz) --- */

		/* Triangle — scope: 1 kHz triangle */
		AD983x_SetWaveform(&ad9834_dds, AD983X_WAVE_TRIANGLE);
		HAL_Delay(3000);

		/* Sine — scope: 1 kHz sine */
		AD983x_SetWaveform(&ad9834_dds, AD983X_WAVE_SINE);
		HAL_Delay(3000);

		/* MSB square on SIGN BIT OUT — scope: IOUT still sine, SIGN BIT OUT shows 1 kHz square */
		AD9834_SetSquareWaveMode(&ad9834_dds, AD9834_SQUARE_MSB);
		HAL_Delay(3000);

		/* Disable square — scope: SIGN BIT OUT goes inactive */
		AD9834_SetSquareWaveMode(&ad9834_dds, AD9834_SQUARE_DISABLED);
		HAL_Delay(3000);

		/* --- Power tests --- */

		/* Sleep — scope: output freezes at DC (MCLK stopped, DAC holds) */
		AD983x_Sleep(&ad9834_dds);
		HAL_Delay(3000);

		/* Wake — scope: 1 kHz sine resumes */
		AD983x_Wake(&ad9834_dds);
		HAL_Delay(3000);

		/* Deep sleep — scope: output drops to 0 V (MCLK + DAC off) */
		AD983x_DeepSleep(&ad9834_dds);
		HAL_Delay(3000);

		/* Wake — scope: 1 kHz sine resumes */
		AD983x_Wake(&ad9834_dds);
		HAL_Delay(3000);

		/* DAC off — scope: output goes to 0 V (DAC off, MCLK still runs) */
		AD983x_DacOff(&ad9834_dds);
		HAL_Delay(3000);

		/* Wake — scope: 1 kHz sine resumes */
		AD983x_Wake(&ad9834_dds);
		HAL_Delay(3000);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
	while (1) {
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
