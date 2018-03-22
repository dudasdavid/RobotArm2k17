
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "../BSP/stm32f429i_discovery.h"
#include "../BSP/stm32f429i_discovery_lcd.h"
#include "../BSP/stm32f429i_discovery_ts.h"
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;

SDRAM_HandleTypeDef hsdram1;

osThreadId defaultTaskHandle;
osThreadId commTaskHandle;
osThreadId sensorTaskHandle;
osThreadId controlTaskHandle;
osThreadId servoRampTaskHandle;
osThreadId tsTaskHandle;
osThreadId oldGraphicsTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define LCD_FRAME_BUFFER_LAYER0                  (LCD_FRAME_BUFFER+0x130000)
#define LCD_FRAME_BUFFER_LAYER1                  LCD_FRAME_BUFFER
#define F32_PI 3.1415927

typedef struct 
{
  float X;
  float Y;
} XYCoordinates;

typedef struct 
{
  float joint1Angle;
  float joint2angle;
} JointAngles;

typedef struct 
{
  float jointXAngle;
  float jointYAngle;
  float jointZAngle;
} JointAngles3Axis;


static volatile float horLinkLength = 0.15;
static volatile float verLinkLength = 0.135;

static volatile float horLinkAngle = 45;
static volatile float verLinkAngle = 45;
static volatile float turnAngle = 0;

static volatile float dutyCycle2DegFactor = 17.0;

#define servo1calib 0.7
#define servo4calib -0.85

static volatile float servo1NewPosRef = 7.5 + servo1calib; // forgato 8.2 a valodi kozep
static volatile float servo2NewPosRef = 8.8;  // vizszintes
static volatile float servo3NewPosRef = 3.9;   // fuggoleges
static volatile float servo4NewPosRef = 7.5 + servo4calib; // megfogo 11.25
static volatile float servo5NewPosRef = 7.5;

static volatile float servo1NewPos = 7.5;
static volatile float servo2NewPos = 10; 
static volatile float servo3NewPos = 3.7;
static volatile float servo4NewPos = 7.5;
static volatile float servo5NewPos = 7.5;

static volatile float servo1CurrPos = 7.5;
static volatile float servo2CurrPos = 8.8; 
static volatile float servo3CurrPos = 3.9;
static volatile float servo4CurrPos = 7.5; //11.25
static volatile float servo5CurrPos = 7.5;

static volatile float servo1Min = 2.5;
static volatile float servo2Min = 6.5;
static volatile float servo3Min = 3.7;
static volatile float servo4Min = 2.5;
static volatile float servo5Min = 5;

static volatile float servo1Max = 12.5;
static volatile float servo2Max = 11.5;
static volatile float servo3Max = 8.3;
static volatile float servo4Max = 12.5;//11.3
static volatile float servo5Max = 10;



static volatile float servo1StepSize = 0.014;
static volatile float servo2StepSize = 0.024;
static volatile float servo3StepSize = 0.024;
static volatile float servo4StepSize = 0.024;
static volatile float servo5StepSize = 0.024;

static volatile int mode = 0;
static volatile int cycleRepeat = 0;
static volatile int graphicsEna =1;

char txBuf[30];
char rxBuf[20];
uint8_t receiveState = 0;

Point verLinkPoints[4];
Point horLinkPoints[4];

XYCoordinates coordinates;
JointAngles calculatedAngles;
JointAngles3Axis calculatedAngles3Axis;

static volatile uint32_t ADC_Buf[3];
static volatile uint32_t val[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART5_Init(void);
static void MX_SPI5_Init(void);
static void MX_I2C3_Init(void);
static void MX_RNG_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
void StartDefaultTask(void const * argument);
void StartCommTask(void const * argument);
void StartSensorTask(void const * argument);
void StartControlTask(void const * argument);
void StartServoRampTask(void const * argument);
void StartTsTask(void const * argument);
void StartOldGraphicsTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  
  
  if(hadc->Instance==ADC1){
    val[0] = ADC_Buf[0];
    val[1] = ADC_Buf[1];
    val[2] = ADC_Buf[2];
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_SPI5_Init();
  MX_I2C3_Init();
  MX_RNG_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);    //servoPWM1
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);    //servoPWM2
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);    //servoPWM3
  HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);    //servoPWM4
  HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);    //servoPWM5
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_2);
  
  //TIM2->CCR1 = (int)(7.5*45000/100);//7.5 = 1.5ms
  //TIM2->CCR2 = (int)(7.5*45000/100);//10 = 2ms
  //TIM3->CCR1 = (int)(6.3*45000/100);//5 = 1ms
  //TIM3->CCR3 = (int)(7.5*45000/100);
  //TIM4->CCR2 = (int)(7.5*45000/100);

  //min 2.5 max 12.5                   -->      
  //2: 2.12 a max 1.2ms a min         -->      10.5...6.5
  //3: 1.65 ms max, 1.3ms a min  --> %-ban 8...6.3
  
  
  
  BSP_LED_On(LED3);
  BSP_LED_On(LED4);
  
  
  /* Configure USER Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  
  /*##-1- LCD Initialization #################################################*/ 
  /* Initialize the LCD */
  BSP_LCD_Init();
  
  
  BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER_LAYER0);
  BSP_LCD_SelectLayer(0);
  
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DrawRect(0,0,239,319);
  BSP_LCD_DisplayStringAt(5,8,"RobotArm Control 2.0",LEFT_MODE);
  BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
  //BSP_LCD_DisplayStringAt(5,30,"RobotArm Control 2.0",LEFT_MODE);
  //BSP_LCD_FillCircle(120, 160, 80);

  
  //BSP_LCD_SetFont(&Font24);
  //Touchscreen_Calibration();
  BSP_TS_Init(240, 320);
  
  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buf,3);
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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of commTask */
  osThreadDef(commTask, StartCommTask, osPriorityHigh, 0, 128);
  commTaskHandle = osThreadCreate(osThread(commTask), NULL);

  /* definition and creation of sensorTask */
  osThreadDef(sensorTask, StartSensorTask, osPriorityBelowNormal, 0, 128);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  /* definition and creation of controlTask */
  osThreadDef(controlTask, StartControlTask, osPriorityAboveNormal, 0, 128);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* definition and creation of servoRampTask */
  osThreadDef(servoRampTask, StartServoRampTask, osPriorityNormal, 0, 128);
  servoRampTaskHandle = osThreadCreate(osThread(servoRampTask), NULL);

  /* definition and creation of tsTask */
  osThreadDef(tsTask, StartTsTask, osPriorityNormal, 0, 128);
  tsTaskHandle = osThreadCreate(osThread(tsTask), NULL);

  /* definition and creation of oldGraphicsTask */
  osThreadDef(oldGraphicsTask, StartOldGraphicsTask, osPriorityLow, 0, 256);
  oldGraphicsTaskHandle = osThreadCreate(osThread(oldGraphicsTask), NULL);

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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 84;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 7;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DMA2D init function */
static void MX_DMA2D_Init(void)
{

  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0xFF;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* LTDC init function */
static void MX_LTDC_Init(void)
{

  LTDC_LayerCfgTypeDef pLayerCfg;

  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 9;
  hltdc.Init.VerticalSync = 1;
  hltdc.Init.AccumulatedHBP = 29;
  hltdc.Init.AccumulatedVBP = 3;
  hltdc.Init.AccumulatedActiveW = 269;
  hltdc.Init.AccumulatedActiveH = 323;
  hltdc.Init.TotalWidth = 279;
  hltdc.Init.TotalHeigh = 327;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 240;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 320;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xD0200000;
  pLayerCfg.ImageWidth = 240;
  pLayerCfg.ImageHeight = 320;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RNG init function */
static void MX_RNG_Init(void)
{

  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
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
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000;
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

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}
/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_SDRAM_TimingTypeDef SdramTiming;

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = B1_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void saturateFloat(volatile float* i, float min, float max) {
  float val = *i;
  if (val < min) val = min;
  else if (val > max) val = max;
  *i = val;
}

float absfloat(float num){
  if (num < 0) return (num*-1);
  else return num;
}

float deg2rad(float deg){
  float rad = 0;
  rad = (deg * F32_PI) / 180.0;
  return rad;
}

float rad2deg(float rad){
  float deg = 0;
  deg = (rad * 180) / F32_PI;
  return deg;
}

float deg2dc(float angleDeg) {
  float dc = 0;
  dc = (180-angleDeg)/dutyCycle2DegFactor;
  return dc;
}

float dc2deg(float dc) {
  float angleDeg = 0;
  angleDeg = 180 - (dc * dutyCycle2DegFactor);
  return angleDeg;
}

float receivedTurn2dc(uint16_t turnNumber){
  return (turnNumber/20.0 + 2.5);
}

float xRotate2dc(float angleDeg) {
    return (angleDeg/8.0 + 7.5 + servo1calib);
}

float gripRotate2dc(float angleDeg) {
    return (-angleDeg/14.0 + 7.5 + servo4calib);
}

float dc2turnDeg(float dc){
  return ((dc - 7.5)*9);
}

float receivedGrip2dc(uint16_t gripValue){
  return (gripValue/16.0 + 5.0);
}

XYCoordinates calculateForwardKinematics(float joint1Angle, float joint2Angle){
  
  coordinates.X = verLinkLength * cosf(deg2rad(joint1Angle)) + horLinkLength * cosf(deg2rad(joint2Angle));
  coordinates.Y = verLinkLength * sinf(deg2rad(joint1Angle)) - horLinkLength * sinf(deg2rad(joint2Angle));
  
  return coordinates;
}

JointAngles calculateInverseKinematics(float xPos, float yPos){
  
  static volatile float x_temp;
  static volatile float y_temp;
  static float c;
  static float gamma;
  static float alpha;
  static float beta;
  static float epsilon;
  static float delta;
  static float omega;
  
  x_temp = xPos;
  y_temp = yPos;
  
  c = sqrtf(xPos*xPos + yPos*yPos); //c^2=a^2+b^2
  gamma = asinf(yPos/c);
  alpha = acosf((verLinkLength*verLinkLength + c*c - horLinkLength*horLinkLength)/(2*verLinkLength*c));
  
  calculatedAngles.joint1Angle = rad2deg(gamma + alpha);
  
  beta = acosf((verLinkLength*verLinkLength + horLinkLength*horLinkLength - c*c)/(2*verLinkLength*horLinkLength));
  epsilon = 90 - rad2deg(gamma + alpha);
  delta = 90 - epsilon;
  omega = 180 - delta - rad2deg(beta);
  
  calculatedAngles.joint2angle = omega;
  
  return calculatedAngles;
}

JointAngles3Axis calculateInverseKinematics3Axis(float xPos, float yPos, float zPos){
  
  static volatile float x_temp;
  static volatile float y_temp;
  static float yPosRecalculated;
  static float c;
  static float gamma;
  static float alpha;
  static float beta;
  static float epsilon;
  static float delta;
  static float omega;
  
  x_temp = xPos;
  y_temp = yPos;
  
  calculatedAngles3Axis.jointXAngle = rad2deg(atanf(xPos/yPos));
  
  yPosRecalculated = sqrtf(yPos*yPos + xPos*xPos);
  
  c = sqrtf(yPosRecalculated*yPosRecalculated + zPos*zPos); //c^2=a^2+b^2
  gamma = asinf(zPos/c);
  alpha = acosf((verLinkLength*verLinkLength + c*c - horLinkLength*horLinkLength)/(2*verLinkLength*c));
  
  calculatedAngles3Axis.jointZAngle = rad2deg(gamma + alpha);
  
  beta = acosf((verLinkLength*verLinkLength + horLinkLength*horLinkLength - c*c)/(2*verLinkLength*horLinkLength));
  epsilon = 90 - rad2deg(gamma + alpha);
  delta = 90 - epsilon;
  omega = 180 - delta - rad2deg(beta);
  
  calculatedAngles3Axis.jointYAngle = omega;
  
  return calculatedAngles3Axis;
}

/**
   * @brief Count characters in char array
	 * @param ptr: pointer to char array
   * @retval Number of characters in array
   */
uint16_t SizeofCharArray(char *ptr)
{
  /* Local variables */
  uint16_t len = 0;
  
  /* Search until end char */
  while (ptr[len] != '\0') {   
    len++;
  }	
  return len;
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
  
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
  }
  /* USER CODE END 5 */ 
}

/* StartCommTask function */
void StartCommTask(void const * argument)
{
  /* USER CODE BEGIN StartCommTask */
  uint16_t Len = 0;
  //strcpy(txBuf, "echo: ");
  
  static int16_t turnValue = 100;
  static int16_t xPosValue = 100;
  static int16_t yPosValue = 100;
  static int16_t gripperValue = 100;
  
  static int16_t xAxisValue = 0;
  static int16_t yAxisValue = 100;
  static int16_t zAxisValue = 100;
  static int16_t aAxisValue = 100;
  
  static float xPoscm = 0;
  static float yPoscm = 0;
  
  static float xAxisCm = 0;
  static float yAxisCm = 0;
  static float zAxisCm = 0;
  
  static volatile float targetStepSizeVar = 0.024; // 1 deg/s = 0.00588
  static float numberOfSteps = 0;

  static JointAngles finalJointAngles;
  static JointAngles3Axis finalJointAngles3Axis;
  
  //uint16_t servo1ComPosRef = 75;
  /* Infinite loop */
  for(;;)
  {

    if (receiveState == 1){

      if ( (rxBuf[0] == 'S') && (rxBuf[4] == ';') && (rxBuf[8] == ';') && (rxBuf[12] == ';') && (rxBuf[16] == '\r')) { // check the frame
      
        turnValue = (int16_t)((rxBuf[1]  - '0')*100 + (rxBuf[2]  - '0')*10 + (rxBuf[3]  - '0')*1);
        xPosValue = (int16_t)((rxBuf[5]  - '0')*100 + (rxBuf[6]  - '0')*10 + (rxBuf[7]  - '0')*1);
        yPosValue = (int16_t)((rxBuf[9]  - '0')*100 + (rxBuf[10]  - '0')*10 + (rxBuf[11]  - '0')*1);
        gripperValue = (int16_t)((rxBuf[13]  - '0')*100 + (rxBuf[14]  - '0')*10 + (rxBuf[15]  - '0')*1);
          
        xPoscm = ((float)xPosValue)/10.0;
        yPoscm = (((float)yPosValue)-50)/10.0;
        
        if (cycleRepeat == 0){
        
          finalJointAngles = calculateInverseKinematics(xPoscm/100.0,yPoscm/100.0);
          servo3NewPosRef = deg2dc(finalJointAngles.joint1Angle);
          servo2NewPosRef = deg2dc(finalJointAngles.joint2angle);
          servo1NewPosRef = receivedTurn2dc(turnValue);
          servo4NewPosRef = receivedGrip2dc(gripperValue);
          
          if ((absfloat(servo3NewPosRef-servo3CurrPos) > absfloat(servo2NewPosRef-servo2CurrPos)) && (absfloat(servo3NewPosRef-servo3CurrPos) > absfloat(servo1NewPosRef-servo1CurrPos))){
            servo3StepSize = targetStepSizeVar;
            numberOfSteps = absfloat(servo3NewPosRef-servo3CurrPos)/targetStepSizeVar;
            servo2StepSize = absfloat(servo2NewPosRef-servo2CurrPos)/numberOfSteps;
            servo1StepSize = absfloat(servo1NewPosRef-servo1CurrPos)/numberOfSteps;
          }
          else if ((absfloat(servo2NewPosRef-servo2CurrPos) > absfloat(servo3NewPosRef-servo3CurrPos)) && (absfloat(servo2NewPosRef-servo2CurrPos) > absfloat(servo1NewPosRef-servo1CurrPos))){
            servo2StepSize = targetStepSizeVar;
            numberOfSteps = absfloat(servo2NewPosRef-servo2CurrPos)/targetStepSizeVar;
            servo3StepSize = absfloat(servo3NewPosRef-servo3CurrPos)/numberOfSteps;
            servo1StepSize = absfloat(servo1NewPosRef-servo1CurrPos)/numberOfSteps;
          }
          else{
            servo1StepSize = targetStepSizeVar;
            numberOfSteps = absfloat(servo1NewPosRef-servo1CurrPos)/targetStepSizeVar;
            servo3StepSize = absfloat(servo3NewPosRef-servo3CurrPos)/numberOfSteps;
            servo2StepSize = absfloat(servo2NewPosRef-servo2CurrPos)/numberOfSteps;
          }
        }
        
        sprintf(txBuf, "OK: %s\r", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_HS((uint8_t*)txBuf, Len);
      }
      else if ((rxBuf[0] == 'P') && (rxBuf[4] == ';') && (rxBuf[8] == ';') && (rxBuf[12] == '\r')){//&& (rxBuf[12] == ';') && (rxBuf[16] == '\r')){
        xAxisValue = (int16_t)((rxBuf[1]  - '0')*100 + (rxBuf[2]  - '0')*10 + (rxBuf[3]  - '0')*1);
        yAxisValue = (int16_t)((rxBuf[5]  - '0')*100 + (rxBuf[6]  - '0')*10 + (rxBuf[7]  - '0')*1);
        zAxisValue = (int16_t)((rxBuf[9]  - '0')*100 + (rxBuf[10]  - '0')*10 + (rxBuf[11]  - '0')*1);
        //aAxisValue = (int16_t)((rxBuf[13]  - '0')*100 + (rxBuf[14]  - '0')*10 + (rxBuf[15]  - '0')*1);
          
        xAxisCm = ((float)xAxisValue-100)/10.0;
        yAxisCm = ((float)yAxisValue)/10.0;
        zAxisCm = ((float)zAxisValue-50)/10.0;
        
        if (cycleRepeat == 0){
          finalJointAngles3Axis = calculateInverseKinematics3Axis(xAxisCm/100.0,yAxisCm/100.0,zAxisCm/100.0);
          servo3NewPosRef = deg2dc(finalJointAngles3Axis.jointZAngle);
          servo2NewPosRef = deg2dc(finalJointAngles3Axis.jointYAngle);
          servo1NewPosRef = xRotate2dc(finalJointAngles3Axis.jointXAngle);
          servo4NewPosRef = gripRotate2dc(finalJointAngles3Axis.jointXAngle);
          targetStepSizeVar = 0.024;
          
          if ((absfloat(servo3NewPosRef-servo3CurrPos) > absfloat(servo2NewPosRef-servo2CurrPos)) && (absfloat(servo3NewPosRef-servo3CurrPos) > absfloat(servo1NewPosRef-servo1CurrPos))){
            servo3StepSize = targetStepSizeVar;
            numberOfSteps = absfloat(servo3NewPosRef-servo3CurrPos)/targetStepSizeVar;
            servo2StepSize = absfloat(servo2NewPosRef-servo2CurrPos)/numberOfSteps;
            servo1StepSize = absfloat(servo1NewPosRef-servo1CurrPos)/numberOfSteps;
            servo4StepSize = absfloat(servo4NewPosRef-servo4CurrPos)/numberOfSteps;
          }
          else if ((absfloat(servo2NewPosRef-servo2CurrPos) > absfloat(servo3NewPosRef-servo3CurrPos)) && (absfloat(servo2NewPosRef-servo2CurrPos) > absfloat(servo1NewPosRef-servo1CurrPos))){
            servo2StepSize = targetStepSizeVar;
            numberOfSteps = absfloat(servo2NewPosRef-servo2CurrPos)/targetStepSizeVar;
            servo3StepSize = absfloat(servo3NewPosRef-servo3CurrPos)/numberOfSteps;
            servo1StepSize = absfloat(servo1NewPosRef-servo1CurrPos)/numberOfSteps;
            servo4StepSize = absfloat(servo4NewPosRef-servo4CurrPos)/numberOfSteps;
          }
          else{
            servo1StepSize = targetStepSizeVar;
            numberOfSteps = absfloat(servo1NewPosRef-servo1CurrPos)/targetStepSizeVar;
            servo3StepSize = absfloat(servo3NewPosRef-servo3CurrPos)/numberOfSteps;
            servo2StepSize = absfloat(servo2NewPosRef-servo2CurrPos)/numberOfSteps;
            servo4StepSize = absfloat(servo4NewPosRef-servo4CurrPos)/numberOfSteps;
          }
        }
        sprintf(txBuf, "OK: %s\r", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_HS((uint8_t*)txBuf, Len);
      }
      else{
        sprintf(txBuf, "ERR: %s\r", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_HS((uint8_t*)txBuf, Len);
        
      }
      
      receiveState = 0;

    }
    osDelay(1000);
  }
  /* USER CODE END StartCommTask */
}

/* StartSensorTask function */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END StartSensorTask */
}

/* StartControlTask function */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  for(;;)
  {
    
    saturateFloat(&servo1NewPosRef, servo1Min, servo1Max);
    saturateFloat(&servo2NewPosRef, servo2Min, servo2Max);
    saturateFloat(&servo3NewPosRef, servo3Min, servo3Max);
    saturateFloat(&servo4NewPosRef, servo4Min, servo4Max);
    saturateFloat(&servo5NewPosRef, servo5Min, servo5Max);
    
    if (servo1NewPosRef > servo1CurrPos){
      if (servo1NewPosRef > servo1CurrPos + servo1StepSize){
        servo1NewPos = servo1CurrPos + servo1StepSize;
      }
      else{
        servo1NewPos = servo1NewPosRef;
      }
    }
    else if (servo1NewPosRef < servo1CurrPos){
      if (servo1NewPosRef < servo1CurrPos - servo1StepSize){
        servo1NewPos = servo1CurrPos - servo1StepSize;
      }
      else{
        servo1NewPos = servo1NewPosRef;
      }
    }
    
    if (servo2NewPosRef > servo2CurrPos){
      if (servo2NewPosRef > servo2CurrPos + servo2StepSize){
        servo2NewPos = servo2CurrPos + servo2StepSize;
      }
      else{
        servo2NewPos = servo2NewPosRef;
      }
    }
    else if (servo2NewPosRef < servo2CurrPos){
      if (servo2NewPosRef < servo2CurrPos - servo2StepSize){
        servo2NewPos = servo2CurrPos - servo2StepSize;
      }
      else{
        servo2NewPos = servo2NewPosRef;
      }
    }
    
    if (servo3NewPosRef > servo3CurrPos){
      if (servo3NewPosRef > servo3CurrPos + servo3StepSize){
        servo3NewPos = servo3CurrPos + servo3StepSize;
      }
      else{
        servo3NewPos = servo3NewPosRef;
      }
    }
    else if (servo3NewPosRef < servo3CurrPos){
      if (servo3NewPosRef < servo3CurrPos - servo3StepSize){
        servo3NewPos = servo3CurrPos - servo3StepSize;
      }
      else{
        servo3NewPos = servo3NewPosRef;
      }
    }
    
    if (servo4NewPosRef > servo4CurrPos){
      if (servo4NewPosRef > servo4CurrPos + servo4StepSize){
        servo4NewPos = servo4CurrPos + servo4StepSize;
      }
      else{
        servo4NewPos = servo4NewPosRef;
      }
    }
    else if (servo4NewPosRef < servo4CurrPos){
      if (servo4NewPosRef < servo4CurrPos - servo4StepSize){
        servo4NewPos = servo4CurrPos - servo4StepSize;
      }
      else{
        servo4NewPos = servo4NewPosRef;
      }
    }
    
    if (servo5NewPosRef > servo5CurrPos){
      if (servo5NewPosRef > servo5CurrPos + servo5StepSize){
        servo5NewPos = servo5CurrPos + servo5StepSize;
      }
      else{
        servo5NewPos = servo5NewPosRef;
      }
    }
    else if (servo5NewPosRef < servo5CurrPos){
      if (servo5NewPosRef < servo5CurrPos - servo5StepSize){
        servo5NewPos = servo5CurrPos - servo5StepSize;
      }
      else{
        servo5NewPos = servo5NewPosRef;
      }
    }
    
    TIM2->CCR1 = (int)(servo1NewPos*120000/100);//7.5 = 1.5ms
    TIM2->CCR2 = (int)(servo2NewPos*120000/100);//10 = 2ms
    TIM3->CCR1 = (int)(servo3NewPos*120000/100);//5 = 1ms
    TIM3->CCR3 = (int)(servo4NewPos*120000/100);
    TIM4->CCR2 = (int)(servo5NewPos*120000/100);
    
    servo1CurrPos = servo1NewPos;
    servo2CurrPos = servo2NewPos;
    servo3CurrPos = servo3NewPos;
    servo4CurrPos = servo4NewPos;
    servo5CurrPos = servo5NewPos;
    
    osDelay(10);
  }
  /* USER CODE END StartControlTask */
}

/* StartServoRampTask function */
void StartServoRampTask(void const * argument)
{
  /* USER CODE BEGIN StartServoRampTask */
  int taskCounter = 0;
  int taskDelay = 1000;
  /* Infinite loop */
  for(;;)
  {
    if (mode % 2 == 1) {
      if (taskCounter % 2 == 0){
        servo2NewPosRef = 8;
        servo3NewPosRef = 6.8;
      }
      else{
        servo2NewPosRef = 10;
        servo3NewPosRef = 6.8;
      }
      taskCounter++;
      osDelay(500);
    }
    else if (cycleRepeat == 1){
      if (taskCounter % 12 == 0){
        servo3NewPosRef = 6.8;
        servo4NewPosRef = 6;
        servo1StepSize = 0.03;
        taskDelay = 1000;
      }
      else if (taskCounter % 12 == 1){
        servo2NewPosRef = 9;
        taskDelay = 1000;
      }
      else if (taskCounter % 12 == 2){
        servo1NewPosRef = 7.5;
        taskDelay = 2000;
      }
      else if (taskCounter % 12 == 3){
        servo1NewPosRef = 3;
        servo2NewPosRef = 7.5;
        taskDelay = 2000;
      }
      else if (taskCounter % 12 == 4){
        
        servo3NewPosRef = 9.25;
        taskDelay = 1500;
      }
      else if (taskCounter % 12 == 5){
        servo4NewPosRef = 9.2;
        taskDelay = 1500;
      }
      else if (taskCounter % 12 == 6){
        servo1NewPosRef = 7.5;
        servo2NewPosRef = 10.7;
        servo3NewPosRef = 6.8;
        servo2StepSize = 0.03;
        servo3StepSize = 0.03;
        taskDelay = 2500;
      }
      else if (taskCounter % 12 == 7){
        servo2NewPosRef = 10.7;
        servo3NewPosRef = 8;
        servo2StepSize = 0.03;
        taskDelay = 500;
      }
      else if (taskCounter % 12 == 8){
        servo2NewPosRef = 10.7;
        servo3NewPosRef = 6.8;
        taskDelay = 1000;
      }
      else if (taskCounter % 12 == 9){
        servo1NewPosRef = 3;
        servo2NewPosRef = 7.5;
        servo2StepSize = 0.03;
        taskDelay = 2000;
      }
      else if (taskCounter % 12 == 10){
        servo3NewPosRef = 9.25;
        taskDelay = 1500;
      }
      else if (taskCounter % 12 == 11){
        servo2StepSize = 0.03;
        servo4NewPosRef = 6;
        taskDelay = 1500;
        taskDelay = 30000;
        //cycleRepeat = 0;
      }
      
      taskCounter++;
      osDelay(taskDelay);
    }
    else{
      osDelay(500);
    }
  }
  /* USER CODE END StartServoRampTask */
}

/* StartTsTask function */
void StartTsTask(void const * argument)
{
  /* USER CODE BEGIN StartTsTask */
  TS_StateTypeDef  State;
  static uint32_t x = 0, y = 0;
  /* Infinite loop */
  for(;;)
  {
    BSP_TS_GetState(&State);

    x = State.X;
    y = State.Y; 
    /*
    if (State.TouchDetected){
      BSP_LCD_DrawCircle(x,y,3);
      BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
      BSP_LCD_FillCircle(120, 160, 80);
    }
    else{
      BSP_LCD_SetTextColor(LCD_COLOR_RED);
      BSP_LCD_FillCircle(120, 160, 80);
    }
    */
    osDelay(10);
  }
  /* USER CODE END StartTsTask */
}

/* StartOldGraphicsTask function */
void StartOldGraphicsTask(void const * argument)
{
  /* USER CODE BEGIN StartOldGraphicsTask */
  int elapsedTime = 0;
  char textBuf[30];
  float verLinkAngleAfterRefresh = 0;
  float horLinkAngleAfterRefresh = 0;
  float turnAngleAfterRefresh = 0;
  
  XYCoordinates endPoint;
  //JointAngles finalJointAngles;
  
  
  /* Infinite loop */
  for(;;)
  {
    BSP_LED_Toggle(LED4);
    if ((BSP_PB_GetState(BUTTON_KEY) == SET) && (elapsedTime > 10)){
      //mode++;
      //cycleRepeat = 1;
      cycleRepeat ^= 1;
      BSP_LCD_DisplayChar(5,25,(char)(cycleRepeat+48));
      elapsedTime = 0;
    }
    
    if (elapsedTime < 100){
      elapsedTime+=1;
    }
    
    verLinkAngle = dc2deg(servo3CurrPos);
    horLinkAngle = dc2deg(servo2CurrPos);
    turnAngle    = dc2turnDeg(servo1CurrPos - servo1calib);
    
    sprintf(textBuf, "V joint: %.2f deg",  verLinkAngle);
    BSP_LCD_DisplayStringAt(5, 45, (unsigned char*) textBuf, LEFT_MODE);
    sprintf(textBuf, "H joint: %.2f deg",  horLinkAngle);
    BSP_LCD_DisplayStringAt(5, 60, (unsigned char*) textBuf, LEFT_MODE);
    
    
    endPoint = calculateForwardKinematics(verLinkAngle,horLinkAngle);
    
    sprintf(textBuf, "X: %.1f cm", (endPoint.X*100));
    BSP_LCD_DisplayStringAt(5, 75, (unsigned char*) textBuf, LEFT_MODE);
    sprintf(textBuf, "Y: %.1f cm", (endPoint.Y*100));
    BSP_LCD_DisplayStringAt(5, 90, (unsigned char*) textBuf, LEFT_MODE);
    
    if (((verLinkAngleAfterRefresh != verLinkAngle) || (horLinkAngleAfterRefresh != horLinkAngle) || (turnAngleAfterRefresh != turnAngle)) && (graphicsEna)) {
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(5,135,230,180);
      BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
      
      verLinkAngleAfterRefresh = verLinkAngle;
      horLinkAngleAfterRefresh = horLinkAngle;
      turnAngleAfterRefresh    = turnAngle;
      
      verLinkPoints[0].X = 45;
      verLinkPoints[0].Y = 250;
      verLinkPoints[1].X = verLinkPoints[0].X + 10;
      verLinkPoints[1].Y = verLinkPoints[0].Y;
      
      verLinkPoints[3].X = (int16_t)(verLinkPoints[0].X + (74.25*cosf(deg2rad(verLinkAngle))));
      verLinkPoints[3].Y = (int16_t)(verLinkPoints[0].Y - (74.25*sinf(deg2rad(verLinkAngle))));
      verLinkPoints[2].X = verLinkPoints[3].X + 10;
      verLinkPoints[2].Y = verLinkPoints[3].Y;
      
      horLinkPoints[0].X = verLinkPoints[2].X;
      horLinkPoints[0].Y = verLinkPoints[2].Y;
      horLinkPoints[1].X = verLinkPoints[2].X;
      horLinkPoints[1].Y = verLinkPoints[2].Y + 10;
      
      horLinkPoints[3].X = (int16_t)(horLinkPoints[0].X + (82.5*cosf(deg2rad(horLinkAngle))));
      horLinkPoints[3].Y = (int16_t)(horLinkPoints[0].Y + (82.5*sinf(deg2rad(horLinkAngle))));
      horLinkPoints[2].X = horLinkPoints[3].X;
      horLinkPoints[2].Y = horLinkPoints[3].Y + 10; 
        
      BSP_LCD_FillRect(30, 250, 40, 40);
      BSP_LCD_FillPolygon(verLinkPoints, 4);
      BSP_LCD_FillPolygon(horLinkPoints, 4);
      BSP_LCD_FillRect(horLinkPoints[3].X, horLinkPoints[3].Y, 30, 10);
      
      BSP_LCD_FillCircle(50,250,10);
      BSP_LCD_FillCircle(verLinkPoints[3].X+5,verLinkPoints[3].Y+5,10);
      BSP_LCD_FillCircle(horLinkPoints[3].X,horLinkPoints[3].Y+5,10);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillCircle(50,250,5);
      BSP_LCD_FillCircle(verLinkPoints[3].X+5,verLinkPoints[3].Y+5,5);
      BSP_LCD_FillCircle(horLinkPoints[3].X,horLinkPoints[3].Y+5,5);
      BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
      
      if (turnAngle < -0.1){
        sprintf(textBuf, "   %.1f ->",  (turnAngle*-1));
      }
      else if (turnAngle > 0.1){
        sprintf(textBuf, "<- %.1f",  turnAngle);
      }
      else {
        sprintf(textBuf, "   %.1f",  turnAngle);
      }
      BSP_LCD_SetFont(&Font12);
      BSP_LCD_DisplayStringAt(15, 300, (unsigned char*) textBuf, LEFT_MODE);
      BSP_LCD_SetFont(&Font16);
    }
    
    
    
    osDelay(500);
  }
  /* USER CODE END StartOldGraphicsTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
