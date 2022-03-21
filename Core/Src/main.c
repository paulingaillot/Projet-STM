/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define RCCAHB 0x4002381C //Offset Correct, utilisé
#define RCCAPB2 0x40023820 //Offset Correct, utilisé
#define GPIOA_MODER 0x40020000 //Offset Correct, utilisé
#define GPIOA_ODR 0x40020014 //Offset Correct, utilisé
#define GPIOC_MODER 0x40020800 //Offset Correct, utilisé
#define GPIOC_IDR 0x40020810 //Offset Correct, utilisé

#define EXTI_IMR 0x40010400 //Offset Correct, utilisé
#define EXTI_FTSR 0x4001040C //Offset Correct, utilisé
#define EXTI_PR 0x40010414 //Offset Correct
#define SYSCFG_EXTICR4 0x40010014 //Offset Correct, utilisé
#define NVIC_ISER1 0xE000E104 //Offset Correct, utilisé

#define TIM2_PSC ((uint32_t)0x40000028)
#define TIM2_ARR ((uint32_t)0x4000002C)
#define TIM2_CNT ((uint32_t)0x40000024)

#define AD7991_ADDRESS 0x50

uint32_t led_allume=0;
uint32_t debut = 0;
uint32_t fin = 0;
uint32_t compteur_echantillon = 0;
uint32_t compteur_echantillon2 = 0;
uint32_t BPM = 30;
uint32_t etat =1;
uint16_t ac_cap =0;
uint8_t ac_cap_simu[2];
uint32_t k = 1;

uint32_t mbpm[8] = {0,0,0,0,0,0,0,0};
uint32_t nextv = 0;
uint32_t testt = 0;

const int32_t tab_ech[1000] = {
		0,-1,-1,-1,-2,-2,-2,-2,-3,-3,-3,-3,-4,-4,-4,-4,-5,-5,-5,-5,-6,-6,-6,-6,-7,-7,-7,-7,-7,-8,-8,-8,-8,-9,-9,-9,-9,-9,-10,-10,-10,-10,-10,-10,-11,-11,-11,-11,-11,-11,-12,-12,-12,-12,-12,-12,-12,-12,-12,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-13,-12,-12,-12,-12,-12,-12,-12,-12,-11,-11,-11,-11,-11,-10,-10,-10,-10,-10,-9,-9,-9,-9,-8,-8,-8,-7,-7,-7,-6,-6,-6,-6,-5,-5,-4,-4,-4,-3,-3,-3,-2,-2,-1,-1,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,8,8,9,9,10,10,11,12,12,13,13,14,15,15,16,17,17,18,18,19,20,20,21,22,23,23,24,25,25,26,27,28,28,29,30,31,31,32,33,34,34,35,36,37,37,38,39,40,41,41,42,43,44,45,45,46,47,48,49,49,50,51,52,53,54,54,55,56,57,58,59,59,60,61,62,63,64,64,65,66,67,68,69,69,70,71,72,73,73,74,75,76,77,78,78,79,80,81,82,82,83,84,85,85,86,87,88,89,89,90,91,92,92,93,94,94,95,96,97,97,98,99,99,100,101,101,102,103,103,104,105,105,106,107,107,108,108,109,110,110,111,111,112,112,113,113,114,114,115,115,116,116,117,117,118,118,119,119,119,120,120,121,121,121,122,122,122,123,123,123,123,124,124,124,124,125,125,125,125,126,126,126,126,126,126,126,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,126,126,126,126,126,126,126,125,125,125,125,124,124,124,124,123,123,123,122,122,122,121,121,121,120,120,119,119,118,118,117,117,116,116,115,115,114,114,113,113,112,111,111,110,110,109,108,108,107,106,106,105,104,103,103,102,101,100,99,99,98,97,96,95,94,94,93,92,91,90,89,88,87,86,85,84,83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,66,64,63,62,61,60,59,58,56,55,54,53,52,50,49,48,47,46,44,43,42,41,39,38,37,36,34,33,32,31,29,28,27,26,24,23,22,20,19,18,17,15,14,13,11,10,9,7,6,5,3,2,1,0,-2,-3,-4,-6,-7,-8,-10,-11,-12,-14,-15,-16,-18,-19,-20,-21,-23,-24,-25,-27,-28,-29,-30,-32,-33,-34,-35,-37,-38,-39,-40,-42,-43,-44,-45,-47,-48,-49,-50,-51,-53,-54,-55,-56,-57,-59,-60,-61,-62,-63,-64,-65,-67,-68,-69,-70,-71,-72,-73,-74,-75,-76,-77,-78,-79,-80,-81,-82,-83,-84,-85,-86,-87,-88,-89,-90,-91,-92,-93,-94,-95,-95,-96,-97,-98,-99,-100,-100,-101,-102,-103,-104,-104,-105,-106,-107,-107,-108,-109,-109,-110,-111,-111,-112,-112,-113,-114,-114,-115,-115,-116,-116,-117,-117,-118,-118,-119,-119,-120,-120,-121,-121,-122,-122,-122,-123,-123,-123,-124,-124,-124,-125,-125,-125,-125,-126,-126,-126,-126,-127,-127,-127,-127,-127,-127,-127,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-127,-127,-127,-127,-127,-127,-127,-126,-126,-126,-126,-125,-125,-125,-125,-124,-124,-124,-124,-123,-123,-123,-122,-122,-122,-121,-121,-120,-120,-120,-119,-119,-118,-118,-117,-117,-116,-116,-115,-115,-114,-114,-113,-113,-112,-112,-111,-111,-110,-109,-109,-108,-108,-107,-106,-106,-105,-104,-104,-103,-102,-102,-101,-100,-100,-99,-98,-98,-97,-96,-95,-95,-94,-93,-93,-92,-91,-90,-90,-89,-88,-87,-86,-86,-85,-84,-83,-83,-82,-81,-80,-79,-79,-78,-77,-76,-75,-74,-74,-73,-72,-71,-70,-70,-69,-68,-67,-66,-65,-65,-64,-63,-62,-61,-60,-60,-59,-58,-57,-56,-55,-55,-54,-53,-52,-51,-50,-50,-49,-48,-47,-46,-46,-45,-44,-43,-42,-42,-41,-40,-39,-38,-38,-37,-36,-35,-35,-34,-33,-32,-32,-31,-30,-29,-29,-28,-27,-26,-26,-25,-24,-24,-23,-22,-21,-21,-20,-19,-19,-18,-18,-17,-16,-16,-15,-14,-14,-13,-13,-12,-11,-11,-10,-10,-9,-9,-8,-7,-7,-6,-6,-5,-5,-4,-4,-3,-3,-2,-2,-1,-1,-1,0,0,1,1,2,2,2,3,3,3,4,4,5,5,5,5,6,6,6,7,7,7,8,8,8,8,9,9,9,9,9,10,10,10,10,10,11,11,11,11,11,11,11,11,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,11,11,11,10,10,10,10,10,10,9,9,9,9,9,9,8,8,8,8,8,7,7,7,7,6,6,6,6,6,5,5,5,5,4,4,4,4,3,3,3,3,2,2,2,2,1,1,1,1,0,0,0
};

#define SysTick_VAL 0xE000E018
#define SysTick_CTRLL 0xE000E010

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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t alarm_high = 130;
uint32_t alarm_low = 30;
uint8_t receiveBuffer = 0;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
		  *((uint32_t*)RCCAHB) |= (1<<0); //On passe à 1 le bit 0 du RCC_AHBENR pour activer l'horloge du GPIOA
	      //On met en General purpose output mode la pin 5 du GPIOA
	      *((uint32_t*)GPIOA_MODER) |= (1<<10); //On passe à 1 le bit 10 du MODER5 du GPIOA
	      *((uint32_t*)GPIOA_MODER) &= ~(1<<1); //On passe à 0 le bit 11 du MODER5 du GPIOA
	      //End of LED Initialization

	      //Button Initialization
	      *((uint32_t*)RCCAHB) |= (1<<2); //On passe à 1 le bit 2 du RCC_AHBENR pour activer l'horloge du GPIOC
	      *((uint32_t*)GPIOC_MODER) &= ~(1<<26); //On passe à 0 le bit 26  du MODER13 du GPIOC
	      *((uint32_t*)GPIOC_MODER) &= ~(1<<27); //On passe à 0 le bit 27 du MODER13 du GPIOC




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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);


  HAL_TIM_Base_Start_IT(&htim6);

  //HAL_I2C_Init(&hi2c1);

  *((uint32_t*)RCCAPB2) |= (1<<0); //On passe à 1 le bit 0 du RCC_APB2ENR pour activer le bus APB2 pour le périphérique SYSCFG
  	  		              *((uint32_t*)SYSCFG_EXTICR4) &=0xFFFFFF2F; //On configure le EXTI13 sur le PC13 avec 0010 (2) en laissant les autres valeurs à ce quelles sont
  	  		              *((uint32_t*)EXTI_IMR) |= (1<<13); //On met le maskage de l'interruption à 1 pour l'activer sur la pin13
  	  		              *((uint32_t*)EXTI_FTSR) |= (1<<13); //On active la détection de l'interruption pour la pin13



  	  		              *((uint32_t*)NVIC_ISER1) |= (1<<(40 & 0x1F));
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 650;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
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
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_I2C_SLaveTxCpltCallback(I2C_HandleTypeDef * hi2c) {

}

uint32_t var = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {


	if(htim->Instance == TIM2) {
		*((uint32_t*)GPIOA_ODR) ^= (1<<5);
	}
	if(htim->Instance == TIM6) {

		uint8_t address[1] = {0x18};
		HAL_I2C_Master_Transmit(&hi2c1, 0x50, address, 1 ,50); //On envoie la configuration de conversion de CH0 à l'esclave.

		HAL_I2C_Master_Receive(&hi2c1, 0x50, ac_cap_simu, 2 ,50);

		uint8_t adc_MSB = ac_cap_simu[0];
		uint8_t adc_LSB = ac_cap_simu[1];
		ac_cap = (((uint16_t)(adc_MSB&0x0F))<<8) + (uint16_t)(adc_LSB); //Conversion sur 16 bits

		if(etat == 1) {
			  		if(ac_cap>=2685) {
			  			etat=2;
			  		}
			  	}else if(etat == 2) {
			  		if(ac_cap<=1590) {
			  			etat=3;
			  		}
			  	}else if(etat == 3) {
			  		compteur_echantillon2++;
			  		if(ac_cap>=2685) {
			  			etat=4;
			  		}
			  	}else if(etat == 4) {
					compteur_echantillon2++;
			  		if(ac_cap<=1590) {
			  			etat=3;

			  			uint32_t BPMa = (30000)/compteur_echantillon2;

			  			mbpm[nextv] = BPMa;
			  			nextv++;
			  			if(nextv == 8) nextv = 0;

			  			uint32_t tot = 0;
			  			for(uint32_t i=0; i<8; i++) {
			  				tot += mbpm[i];
			  			}
			  			BPM = tot / 8;


						if(BPM < alarm_high) {
							  				if(BPM > 60) {
							  					*((uint32_t*)TIM2_ARR) = 1000*(60.0/BPM);
							  					if(*((uint32_t*)TIM2_CNT) >= *((uint32_t*)TIM2_ARR)) *((uint32_t*)TIM2_CNT) = 1;
							  				}
							  				else  *((uint32_t*)TIM2_ARR) = 1999;
							  			 }else {
							  				__HAL_TIM_SET_AUTORELOAD(&htim2, 10);
							  				*((uint32_t*)GPIOA_ODR) |= (1<<5);
							  			 }

			  			compteur_echantillon2 = 0;
			  		}
			  	}

		//if(compteur_echantillon > 999) compteur_echantillon =0;

	  	/*if(etat == 1) {
	  		compteur_echantillon++;
	  		ac_cap = 2048+tab_ech[compteur_echantillon];
	  		if(ac_cap>=2148) {
	  			etat=2;
	  		}
	  	}else if(etat == 2) {
			compteur_echantillon++;
			ac_cap = 2048+tab_ech[compteur_echantillon];
	  		if(ac_cap<=1948) {
	  			etat=3;
	  			//cpt_ech=0;
	  		}
	  	}else if(etat == 3) {
			for(uint32_t j=0; j<=k; j++) {
			ac_cap = 2048+tab_ech[(compteur_echantillon+j)%999];
	  		if(ac_cap>=2148) {
	  			etat=4;
	  		}
			}
			compteur_echantillon += k;
			compteur_echantillon2 ++;
			ac_cap = 2048+tab_ech[compteur_echantillon%999];
	  	}else if(etat == 4) {

	  		uint32_t test = 0;
	  		for(uint32_t j=0; j<=k; j++) {
	  			ac_cap = 2048+tab_ech[(compteur_echantillon+j)%999];
	  			if(ac_cap<=1948) {
	  				test= 1;
	  			}
	  		}

			compteur_echantillon += k;
			compteur_echantillon2 ++;
			ac_cap = 2048+tab_ech[compteur_echantillon%999];
	  		if(test==1) {
	  			etat=3;
	  			//cpt_ech=0;

	  			BPM = (30000)/compteur_echantillon2;


	  			if(1000*(60.0/BPM) != *((uint32_t*)TIM2_ARR) ) {
				if(BPM < alarm_high) {
					  				if(BPM > 60) {
					  					*((uint32_t*)TIM2_ARR) = 1000*(60.0/BPM);
					  					if(*((uint32_t*)TIM2_CNT) >= *((uint32_t*)TIM2_ARR)) *((uint32_t*)TIM2_CNT) = 1;
					  				}
					  				else  *((uint32_t*)TIM2_ARR) = 1999;
					  			 }else {
					  				__HAL_TIM_SET_AUTORELOAD(&htim2, 10);
					  				*((uint32_t*)GPIOA_ODR) |= (1<<5);
					  			 }

	  		}
	  			compteur_echantillon2 = 0;
	  		}
	  	}*/



	}

}

void EXTI15_10_IRQHandler()
{

    int stateB = *((uint32_t*)GPIOC_IDR) & 1<<13;

    if (stateB != 1<<13) {
    	 fin = uwTick;
    } else {
    	 debut = uwTick;

    	 uint32_t resultat = (debut - fin);
    	 fin = debut;

    	     	 	if(resultat < 1000 || resultat >= 4000) {
    	     	 		k++;
						if(k>8) k=1;
    	     	 	}else if(resultat >= 1000 && resultat < 2000) {
    	     	 		alarm_high = 130;

    	     	 	}else if(resultat >=2000 && resultat <3000) {
    	     	 		alarm_high = 160;
    	     	 	}else if(resultat >= 3000 && resultat < 4000){
    	     	 		alarm_high = 190;
    	     	}

    }

    *((uint32_t*)EXTI_PR) |= (1<<13);
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

