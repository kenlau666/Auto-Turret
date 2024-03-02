/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "lcd.h"
#include "bsp_ov7725.h"
#include "bsp_sccb.h"
#include "math.h"
#include "stdio.h"

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

TIM_HandleTypeDef htim3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
volatile uint8_t Ov7725_vsync ;
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
	uint8_t HMC5883L_Addr = 0x1E; uint8_t CRA = 0x70; uint8_t CRB = 0xA0;
	  uint8_t first = 0;
	  uint8_t initial_sx = 120;
	  uint8_t initial_lx = 140;
	  uint8_t initial_sy = 120;
	  uint8_t initial_ly = 140;
	  uint8_t offset = 0;
	  uint8_t capture = 0;
	  int16_t x, y, angle, degree1 = 1000, degree2 = 1000;
	  uint8_t autoMode = 0, c1, c2 = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_INIT();
	HAL_I2C_Mem_Write(&hi2c1,HMC5883L_Addr<<1,0x00,1,&CRA,1,100);
	  HAL_I2C_Mem_Write(&hi2c1,HMC5883L_Addr<<1,0x01,1,&CRB,1,100);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(Ov7725_Init() != SUCCESS);
	Ov7725_vsync = 0;
	int a[2] = {0,0};
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, degree1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, degree2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 400);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 10000);
	HAL_Delay(50);
	//uint16_t px = 0;
	//uint16_t py = 0;
	//char xv[5];
	//char yv[5];
	//LCD_DrawString(0,0,"Press the keys to chose the mode:");
	//LCD_DrawString(0,40,"K1 -> Auto mode");
	//LCD_DrawString(0,70,"K2 -> Manual mode");
	//HAL_Delay(2000);
	//bool state = true;
	//while (state)
	//{
	//}

  while (1)
  {
	  	    LCD_Cam_Gram();
	  	    if (first == 0)
	  	    {
	  	    	LCD_Clear(0,0,200,150,0xFFFF);
			    LCD_DrawString(0,0,"Manual Mode:");
			    LCD_DrawString(0,30,"Press the Switches");
			    first = first +1;
			    HAL_Delay(500);
	  	    }
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1)
			  {
				  autoMode = (autoMode+1)%2;
				  if (autoMode == 0)
				  {
					  LCD_Clear(0,0,200,150,0xFFFF);
					  LCD_DrawString(0,0,"Manual Mode:");
					  LCD_DrawString(0,30,"Press the Switches");
					  HAL_Delay(500);
				  }
				  else
				  {
					  LCD_Clear(0,0,200,150,0xFFFF);
					  LCD_DrawString(0,0,"Auto Mode:");
					  LCD_DrawString(0,30,"Use the Laser");
					  HAL_Delay(500);
				  }
			  }
			if(!autoMode)
			  {
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 1){ //up
					 degree1+=50;
					HAL_Delay(150);
				}
			    else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1){ //down
					 degree1-=50;
					HAL_Delay(150);
				}
			    else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1){ //left
					degree2-=50;
					HAL_Delay(150);
				}
			    else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 1){ //right
					degree2+=50;
					HAL_Delay(100);
				}

				//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
				  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
				  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
				  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, degree1);
				  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, degree2);
//				  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 20000);
				  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1)
				  {
					  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
					HAL_Delay(1000);
					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 200);
					HAL_Delay(100);
					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 900);
					HAL_Delay(1000);
					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 400);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
				  }
//				  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
//					HAL_Delay(1000);
//					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 600);
			  }
			else
			  {
				//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
				if (Ov7725_vsync == 2)
				{
				  FIFO_PREPARE;
				  ImagDisp(a,capture);
				  capture = (capture+1)%2;
				  Ov7725_vsync = 0;
			    }
				if (capture)
				{
					if ((a[0] && a[1]) &&((a[0] >= initial_lx + offset) || (a[0] <= initial_sx - offset) || (a[1] >= initial_ly + offset) || (a[1] <= initial_sy - offset)))
					{
						if (a[0] >= initial_lx + offset)
							degree1 -= 20;
						else if (a[0] <= initial_sx - offset)
							degree1 += 20;
						if (a[1] >= initial_ly + offset)
							degree2 -= 20;
						else if (a[1] <= initial_sy - offset)
							degree2 += 20;
						HAL_Delay(100);
						HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
						HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	//				    if (degree1 < 2000)
	//				    	degree1 += 100;
		//			    if (degree1 == 1000 || degree1 == 2000)
		//			    {
		//				  c1 = (c1+1)%2;
		//			    }
		//			    if (degree2 == 1000 || degree2 == 2000)
		//			    {
		//				    c2 = (c2+1)%2;
		//			    }
		//			    if (c1) degree1 += 100;
		//			    else degree1 -= 100;
		//			    if (c2) degree2 += 100;
		//			    else degree2 -= 100;
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, degree1);
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, degree2);
						HAL_Delay(60);
					}
//					if(1)
					if ((a[0] < initial_lx + offset) && (a[0] > initial_sx - offset) && (a[1] < initial_ly + offset) && (a[1] > initial_sy - offset))
					{
						LCD_Clear(0,0,160,120,0xFFFF);
						LCD_DrawString(0,40,"Target Found");
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
						HAL_Delay(1000);
						HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 200);
						HAL_Delay(100);
						HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 900);
						HAL_Delay(1000);
						HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 400);
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
					}
				}
			  }
			//uint16_t i, j;
			//uint16_t Camera_Data;
			//uint16_t x = 0;

			//LCD_Cam_Gram();

			//for(i = 0; i < 240; i++)
			//{
				//for(j = 0; j < 320; j++)
				//{
					//READ_FIFO_PIXEL(Camera_Data);
					//if (LCD_GetPointPixel(j,i)== 0xF800)
					//{
						//px = i;
						//py = j;
						//break;
					//}
				//}
				//if (px && py)
					//break;

			//}
			//if (px && py)
			//{
				//LCD_Clear(0,0,320,240,0xFFFF);
				//LCD_DrawString(0,40,"x_value : ");
			    //LCD_DrawString(0,70,"y_value : ");
			    //sprintf(xv,"%d",py);
			    //LCD_DrawString(80,40,xv);
			    //sprintf(yv,"%d",px);
			    //LCD_DrawString(80,70,yv);
			    //LCD_DrawChar(py,px,'D');
			    //HAL_Delay(3000);
//			    px = 0;
//			    py = 0;

			//}
		//}
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 39;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pins : PE6 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
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
