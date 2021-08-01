/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
//float universal_lookup[] = { 3, 6, 9, 12, 15, 18, 21, 25, 28, 31, 34, 37, 40, 43, 47, 50, 53, 56, 59, 62, 65, 68, 72, 75, 78, 81, 84, 87, 90, 93, 96, 100, 103, 106, 109, 112, 115, 118, 121, 124, 127, 130, 133, 137, 140, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173, 176, 179, 182, 185, 188, 191, 194, 197, 200, 203, 206, 209, 212, 215, 217, 220, 223, 226, 229, 232, 235, 238, 241, 244, 246, 249, 252, 255, 258, 261, 263, 266, 269, 272, 275, 277, 280, 283, 286, 289, 291, 294, 297, 300, 302, 305, 308, 310, 313, 316, 318, 321, 324, 326, 329, 332, 334, 337, 339, 342, 345, 347, 350, 352, 355, 357, 360, 362, 365, 367, 370, 372, 375, 377, 380, 382, 384, 387, 389, 392, 394, 396, 399, 401, 403, 406, 408, 410, 413, 415, 417, 419, 422, 424, 426, 428, 430, 433, 435, 437, 439, 441, 443, 445, 447, 450, 452, 454, 456, 458, 460, 462, 464, 466, 468, 470, 472, 474, 476, 477, 479, 481, 483, 485, 487, 489, 490, 492, 494, 496, 498, 499, 501, 503, 504, 506, 508, 509, 511, 513, 514, 516, 518, 519, 521, 522, 524, 525, 527, 528, 530, 531, 533, 534, 536, 537, 538, 540, 541, 542, 544, 545, 546, 548, 549, 550, 551, 553, 554, 555, 556, 557, 559, 560, 561, 562, 563, 564, 565, 566, 567, 568, 569, 570, 571, 572, 573, 574, 575, 576, 577, 577, 578, 579, 580, 581, 581, 582, 583, 584, 584, 585, 586, 586, 587, 588, 588, 589, 589, 590, 591, 591, 592, 592, 593, 593, 594, 594, 594, 595, 595, 596, 596, 596, 597, 597, 597, 597, 598, 598, 598, 598, 599, 599, 599, 599, 599, 599, 599, 599, 599, 599, 599, 1200, 599, 599, 599, 599, 599, 599, 599, 599, 599, 599, 599, 598, 598, 598, 598, 597, 597, 597, 597, 596, 596, 596, 595, 595, 594, 594, 594, 593, 593, 592, 592, 591, 591, 590, 589, 589, 588, 588, 587, 586, 586, 585, 584, 584, 583, 582, 581, 581, 580, 579, 578, 577, 577, 576, 575, 574, 573, 572, 571, 570, 569, 568, 567, 566, 565, 564, 563, 562, 561, 560, 559, 557, 556, 555, 554, 553, 551, 550, 549, 548, 546, 545, 544, 542, 541, 540, 538, 537, 536, 534, 533, 531, 530, 528, 527, 525, 524, 522, 521, 519, 518, 516, 514, 513, 511, 509, 508, 506, 504, 503, 501, 499, 498, 496, 494, 492, 490, 489, 487, 485, 483, 481, 479, 477, 476, 474, 472, 470, 468, 466, 464, 462, 460, 458, 456, 454, 452, 450, 447, 445, 443, 441, 439, 437, 435, 433, 430, 428, 426, 424, 422, 419, 417, 415, 413, 410, 408, 406, 403, 401, 399, 396, 394, 392, 389, 387, 384, 382, 380, 377, 375, 372, 370, 367, 365, 362, 360, 357, 355, 352, 350, 347, 345, 342, 339, 337, 334, 332, 329, 326, 324, 321, 318, 316, 313, 310, 308, 305, 302, 300, 297, 294, 291, 289, 286, 283, 280, 277, 275, 272, 269, 266, 263, 261, 258, 255, 252, 249, 246, 244, 241, 238, 235, 232, 229, 226, 223, 220, 217, 215, 212, 209, 206, 203, 200, 197, 194, 191, 188, 185, 182, 179, 176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 143, 140, 137, 133, 130, 127, 124, 121, 118, 115, 112, 109, 106, 103, 100, 96, 93, 90, 87, 84, 81, 78, 75, 72, 68, 65, 62, 59, 56, 53, 50, 47, 43, 40, 37, 34, 31, 28, 25, 21, 18, 15, 12, 9, 6, 3 };
uint16_t lookUp1[] = {50 ,100 ,151 ,201 ,250 ,300 ,349 ,398 ,446 ,494 ,542 ,589 ,635 ,681 ,726 ,771 ,814 ,857 ,899 ,940 ,981 ,1020 ,1058 ,1095 ,1131 ,1166 ,1200 ,1233 ,1264 ,1294 ,1323 ,1351 ,1377 ,1402 ,1426 ,1448 ,1468 ,1488 ,1505 ,1522 ,1536 ,1550 ,1561 ,1572 ,1580 ,1587 ,1593 ,1597 ,1599 ,1600 ,1599 ,1597 ,1593 ,1587 ,1580 ,1572 ,1561 ,1550 ,1536 ,1522 ,1505 ,1488 ,1468 ,1448 ,1426 ,1402 ,1377 ,1351 ,1323 ,1294 ,1264 ,1233 ,1200 ,1166 ,1131 ,1095 ,1058 ,1020 ,981 ,940 ,899 ,857 ,814 ,771 ,726 ,681 ,635 ,589 ,542 ,494 ,446 ,398 ,349 ,300 ,250 ,201 ,151 ,100 ,50 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0};
uint16_t lookUp2[] = {0  ,0   ,0   ,0 	 ,0 	,0 		,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,50 ,100 ,151 ,201 ,250 ,300 ,349 ,398 ,446 ,494 ,542 ,589 ,635 ,681 ,726 ,771 ,814 ,857 ,899 ,940 ,981 ,1020 ,1058 ,1095 ,1131 ,1166 ,1200 ,1233 ,1264 ,1294 ,1323 ,1351 ,1377 ,1402 ,1426 ,1448 ,1468 ,1488 ,1505 ,1522 ,1536 ,1550 ,1561 ,1572 ,1580 ,1587 ,1593 ,1597 ,1599 ,1600 ,1599 ,1597 ,1593 ,1587 ,1580 ,1572 ,1561 ,1550 ,1536 ,1522 ,1505 ,1488 ,1468 ,1448 ,1426 ,1402 ,1377 ,1351 ,1323 ,1294 ,1264 ,1233 ,1200 ,1166 ,1131 ,1095 ,1058 ,1020 ,981 ,940 ,899 ,857 ,814 ,771 ,726 ,681 ,635 ,589 ,542 ,494 ,446 ,398 ,349 ,300 ,250 ,201 ,151 ,100 ,50 ,0};


//const uint16_t TIMER_PERIOD = 3600;
float EE;
float freq;
float amplitude=0;
double uu;
	
uint16_t SINE_counter_A=0;
uint16_t SINE_counter_B=200*1/3;
uint16_t SINE_counter_C=200*1/3;

_Bool 	phase_A_toggle=0;
_Bool 	phase_B_toggle=0;			//as it has 60
_Bool 	phase_C_toggle=1;
	
uint16_t ADC_VALs[4];
float result;
float NO_of_s = 0;
	
uint16_t period = 0;
uint16_t PWM_freq = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	// Analog DMA settings
	HAL_ADC_Start_DMA(&hadc1,( uint32_t *)ADC_VALs,4);


	// PWM settings
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint16_t)0);	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint16_t)0);	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint16_t)0);	



/*		
	// init 
	freq =  map(ADC_VALs[0],1,4048,1,100);			
	NO_of_s = 1e3/freq;
	SINE_counter_A = 0;
	SINE_counter_B = NO_of_s*2/3;
	SINE_counter_C = NO_of_s*2/3;
*/
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;	
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_2);
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_3);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

	if (htim->Instance == TIM1) {
			

		//algorithms 
			freq =  map(ADC_VALs[0],1,4048,1,80);			
			amplitude = map(freq,1,50,.05,1);
			if (amplitude >= .99) amplitude=.99;
			period = (uint16_t)(36e4/freq);
			TIM1->ARR = period - 1;
			TIM2->ARR = period - 1;
			PWM_freq = (72e6)/(period);
		
			SINE_counter_A++;
			SINE_counter_B++;
			SINE_counter_C++;
		//end of algorithms 
		
		
		// selection of channel for phase A
			if ( phase_A_toggle == 0 ) {	
				result = (lookUp1[SINE_counter_A]*period);
				result = result * .000625;
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint16_t)result);	
				phase_A_toggle = 1;
			}else{
				result = (lookUp2[SINE_counter_A]*period);
				result = result * .000625;
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(uint16_t)result);	
				phase_A_toggle = 0;				
			}
			if (SINE_counter_A >= 200) { SINE_counter_A=0; phase_A_toggle = !phase_A_toggle; }
			// end of selections
			
			
		// selection of channel for phase B
			if ( phase_B_toggle == 0 ) {	
				result = (lookUp1[SINE_counter_B]*period);
				result = result * .000625;
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint16_t)result);	
				phase_B_toggle = 1;
			}else{
				result = (lookUp2[SINE_counter_B]*period);
				result = result * .000625;
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,(uint16_t)result);	
				phase_B_toggle = 0;				
			}
			if (SINE_counter_B >= 200) { SINE_counter_B=0; phase_B_toggle = !phase_B_toggle; }
			// end of selections
				
			
			
		// selection of channel for phase C
			if ( phase_C_toggle == 0 ) {	
				result = (lookUp1[SINE_counter_C]*period);
				result = result * .000625;
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint16_t)result);	
				phase_C_toggle = 1;
			}else{
				result = (lookUp2[SINE_counter_C]*period);
				result = result * .000625;
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,(uint16_t)result);	
				phase_C_toggle = 0;				
			}
			if (SINE_counter_C >= 200) { SINE_counter_C=0; phase_C_toggle = !phase_C_toggle; }
			// end of selections
							
	} 
}


/*


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

	if (htim->Instance == TIM1) {
		SINE_counter_A++;
//		SINE_counter_B = SINE_counter_A + (uint16_t).6667*NO_of_s ;
//		SINE_counter_C = SINE_counter_A + (uint16_t).6667*NO_of_s;	
		
		result = PI*SINE_counter_A/NO_of_s;
		
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint16_t)3600*sin(result));	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint16_t)3600*(sin(result - (2/3)*NO_of_s )));	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint16_t)3600*(sin(result - (2/3)*NO_of_s )));	

		if (SINE_counter_A >= NO_of_s && k==0) { SINE_counter_A=0;  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); k=1;}
		else if ( SINE_counter_A >= NO_of_s && k==1  ){SINE_counter_A=0;  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); k=0;}
		
		if (SINE_counter_B >= NO_of_s && l==0) { 			  SINE_counter_B=0;  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);		 HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); l=1;}
		else if ( SINE_counter_B >= NO_of_s && l==1  ){ SINE_counter_B=0;  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); 		l=0;}
		if (SINE_counter_C >= NO_of_s && m==1) { SINE_counter_C=0;  			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3); 		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); 	m=0;}
		else if ( SINE_counter_C >= NO_of_s && m==0  ){SINE_counter_C=0;  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); 		m=1;}
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);
		
		freq =  map(ADC_VALs[0],1,4048,1,100);			
		NO_of_s = 1e3/freq;
		amplitude = map(freq,1,50,.05,1);
		if (amplitude >= .99) amplitude=.99;
		
	} 
}

*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
