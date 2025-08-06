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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "gps.h"
#include "LiquidCrystal_I2C.h"
#include "DFPLAYER.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t engine_status;
    uint8_t light_status;
    uint8_t tire_pressure;
    uint8_t door_status;
    uint8_t seat_belt_status;
    uint8_t battery_level;
    uint8_t speed;
    uint8_t arrived_distance;
    uint8_t total_distance;
    uint8_t arrived_time;
    uint8_t transmission_gear;
    uint8_t reserved1;
    uint8_t reserved2;
    float gps_lat;
    float gps_lon;
} __attribute__((packed)) VehicleStatus;

typedef enum {
    exceed_speed,
    low_tire_pressure,
    door_open,
    seat_belt_open,
    air_condition_increase,
    air_condition_decrease,
    speed_limit_increase,
    speed_limit_decrease,
} WarningCode_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VEHICLE_STATUS_CAN_ID 0x023
#define PACKET_HEADER_ID 0x024
#define PACKET_DATA_ID 0x025
#define M_PI 3.14159265358979323846

#define EXCEED_SPEED_SOUND                2
#define LOW_TIRE_PRESSURE_SOUND           5
#define DOOR_OPEN_SOUND                   1
#define SEAT_BELT_OPEN_SOUND              6
#define AIR_CONDITION_INCREASE_SOUND      7
#define AIR_CONDITION_DECREASE_SOUND      8
#define SPEED_LIMIT_INCREASE_SOUND        4
#define SPEED_LIMIT_DECREASE_SOUND        3

#define P_Gear 0
#define R_Gear 1
#define N_Gear 2
#define D_Gear 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId CAN_HandleHandle;
osThreadId Input_HandleHandle;
osThreadId Compute_HandleHandle;
osThreadId Control_HandleHandle;
osThreadId Notify_HandleHandle;
/* USER CODE BEGIN PV */
VehicleStatus vehicle_data={0};
uint8_t air_condition_temp = 0;
uint8_t speed_limit = 0;
//--------- CAN message structures------------
CAN_TxHeaderTypeDef TxMsg;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint32_t pu32Mailbox;

uint8_t *data_ptr;
uint32_t data_size;
uint8_t frame_count;

volatile uint16_t adc_buffer[4]= {0, 0, 0, 0}; // ADC buffer for joystick and battery

LiquidCrystal_I2C hlcd;
DFPLAYER_Name mp3;

uint8_t can_send_flag = 0;

GPS_t GPS;
int joystick_x = 0;
int joystick_y = 0;

uint32_t distance_metter = 0;
uint32_t arrived_time_s = 0;

WarningCode_t warning_code;
uint8_t warning_notify_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);
void CAN_Handler(void const * argument);
void Input_Handler(void const * argument);
void Compute_Handler(void const * argument);
void Control_Handler(void const * argument);
void Notify_Handler(void const * argument);

/* USER CODE BEGIN PFP */
void InitVehicleData(void) {
    vehicle_data.engine_status = 0;       
    vehicle_data.light_status = 0;         
    vehicle_data.tire_pressure = 0;      
    vehicle_data.door_status = 0;         
    vehicle_data.seat_belt_status = 0;     
    vehicle_data.battery_level = 0;      
    vehicle_data.speed = 0;               
    vehicle_data.arrived_distance = 0;    
    vehicle_data.total_distance = 0;    
    vehicle_data.arrived_time = 0;          
    vehicle_data.transmission_gear = 0;   
    vehicle_data.gps_lat = 0.0f;       
    vehicle_data.gps_lon = 0.0f;     
}

void SendVehicleStatus(void) {
    data_ptr = (uint8_t*)&vehicle_data;
    data_size = sizeof(VehicleStatus);
    frame_count = (data_size + 6) / 7;  // Changed from (data_size + 7) / 8
    
    // Send header with frame information
    TxMsg.StdId = PACKET_HEADER_ID;
    TxMsg.DLC = 8;
    uint8_t header_data[8] = {0};
    header_data[0] = frame_count;
    header_data[1] = data_size & 0xFF;
    header_data[2] = (data_size >> 8) & 0xFF;
    header_data[3] = 0x01; // Message type: VehicleStatus
    
    HAL_CAN_AddTxMessage(&hcan, &TxMsg, header_data, &pu32Mailbox);
    HAL_Delay(10);
    
    // Send data frame by frame (7 bytes each)
    TxMsg.StdId = PACKET_DATA_ID;
    for (uint8_t i = 0; i < frame_count; i++) {
        uint8_t can_data[8] = {0};
        uint8_t bytes_to_send = (i == frame_count - 1) ? 
            (data_size - i * 7) : 7;  // Always 7 bytes except last frame
        
        can_data[0] = i; // Frame number
        memcpy(&can_data[1], data_ptr + i * 7, bytes_to_send);
        
        TxMsg.DLC = bytes_to_send + 1;  // +1 for frame number
        HAL_CAN_AddTxMessage(&hcan, &TxMsg, can_data, &pu32Mailbox);
        HAL_Delay(10);
    }
}


void Servo360_Control(uint8_t speed, char direction) {
    uint16_t pulse;
    if (speed == 0) {
        pulse = 1500;
    } else if (direction == 'D') {
        pulse = 1500 + (500 * speed) / 250;
    } else { // 'R'
        pulse = 1500 - (500 * speed) / 250;
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
  {
    GPS_UART_CallBack(&huart1,&GPS);
  } 
}

#if (configSUPPORT_STATIC_ALLOCATION == 1)

#define TIMER_TASK_STACK_DEPTH  (128 )

static StaticTask_t xTimerTaskTCB;
static StackType_t xTimerStack[TIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = xTimerStack;
    *pulTimerTaskStackSize = TIMER_TASK_STACK_DEPTH;
}

#endif


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
  MX_CAN_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

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

  /* definition and creation of CAN_Handle */
  osThreadDef(CAN_Handle, CAN_Handler, osPriorityHigh, 0, 128);
  CAN_HandleHandle = osThreadCreate(osThread(CAN_Handle), NULL);

  /* definition and creation of Input_Handle */
  osThreadDef(Input_Handle, Input_Handler, osPriorityAboveNormal, 0, 128);
  Input_HandleHandle = osThreadCreate(osThread(Input_Handle), NULL);

  /* definition and creation of Compute_Handle */
  osThreadDef(Compute_Handle, Compute_Handler, osPriorityNormal, 0, 128);
  Compute_HandleHandle = osThreadCreate(osThread(Compute_Handle), NULL);

  /* definition and creation of Control_Handle */
  osThreadDef(Control_Handle, Control_Handler, osPriorityNormal, 0, 128);
  Control_HandleHandle = osThreadCreate(osThread(Control_Handle), NULL);

  /* definition and creation of Notify_Handle */
  osThreadDef(Notify_Handle, Notify_Handler, osPriorityBelowNormal, 0, 128);
  Notify_HandleHandle = osThreadCreate(osThread(Notify_Handle), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  InitVehicleData();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  sConfigOC.Pulse = 0;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9990;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_CAN_Handler */
/**
* @brief Function implementing the CAN_Handle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Handler */
void CAN_Handler(void const * argument)
{
  /* USER CODE BEGIN CAN_Handler */
  TxMsg.DLC = 8;
  TxMsg.IDE = CAN_ID_STD;
  TxMsg.RTR = CAN_RTR_DATA;
  TxMsg.TransmitGlobalTime = DISABLE;
  
  // Cấu hình CAN filter để nhận message air condition
  CAN_FilterTypeDef canfilterconfig;

  // Gộp filter cho các ID 0x024, 0x025, 0x030 bằng cách dùng ID mask
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x000 << 5;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x000 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  HAL_CAN_Start(&hcan); // start CAN
  /* Infinite loop */
  for(;;)
  {
    // Send vehicle status periodically
    if(can_send_flag) {
        can_send_flag = 0; // Reset flag
        SendVehicleStatus();
    }
    // Wait for CAN messages
    if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            if (RxHeader.StdId == 0x030 && RxHeader.DLC >= 1) {
                vehicle_data.engine_status = RxData[2]; // Update engine status
                vehicle_data.light_status = RxData[3]; // Update engine status
                vehicle_data.door_status = RxData[4]; // Update tire pressure status
                if(RxData[0] != air_condition_temp) {
                    warning_notify_flag = 1; // Set warning flag for air condition change
                    if(RxData[0] > air_condition_temp) {
                        warning_code = air_condition_increase; // Set warning code
                    } else {
                        warning_code = air_condition_decrease; // Set warning code
                    }
                    air_condition_temp = RxData[0];
                }
                if(RxData[1] != speed_limit) {
                    warning_notify_flag = 1; // Set warning flag for speed limit change
                    if(RxData[1] > speed_limit) {
                        warning_code = speed_limit_increase; // Set warning code
                    } else {
                        warning_code = speed_limit_decrease; // Set warning code
                    }
                    speed_limit = RxData[1];
                }
            }
        }
    }
    osDelay(50); // Delay to avoid busy loop
  }
  /* USER CODE END CAN_Handler */
}

/* USER CODE BEGIN Header_Input_Handler */
/**
* @brief Function implementing the Input_Handle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Input_Handler */
void Input_Handler(void const * argument)
{
  /* USER CODE BEGIN Input_Handler */
  InitVehicleData();
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 4);
  HAL_TIM_Base_Start_IT(&htim3);
  //UART2 receive callback
  GPS_Init(&huart1);
  /* Infinite loop */
  for(;;)
  {
    // Update GPS data
    if(GPS.dec_latitude != 0.0f && GPS.dec_longitude != 0.0f) {
      vehicle_data.gps_lat = GPS.dec_latitude;
      vehicle_data.gps_lon = GPS.dec_longitude;
    }
    // Read GPIO inputs
    if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15))
    {
      can_send_flag=1;
      vehicle_data.engine_status = (vehicle_data.engine_status + 1) % 2; // Toggle engine status
    }
    
    if(vehicle_data.engine_status == 1) {
      if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14))
      {
        can_send_flag=1;
        vehicle_data.light_status = (vehicle_data.light_status + 1) % 4; // Cycle through light status
      }

      if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
      {
        can_send_flag=1;
        vehicle_data.tire_pressure = (vehicle_data.tire_pressure + 1) % 2; // Toggle tire pressure status
      }

      if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
      {
        can_send_flag=1;
        vehicle_data.door_status = (vehicle_data.door_status + 1) % 2; // Toggle door status
      }
      
      if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
      {
        can_send_flag=1;
        vehicle_data.seat_belt_status = (vehicle_data.seat_belt_status + 1) % 2; // Toggle seat belt status
      }
    
      // Read ADC values 
      if(vehicle_data.transmission_gear == D_Gear || vehicle_data.transmission_gear == R_Gear) {
          if(abs(adc_buffer[0] * 250 / 4095 - vehicle_data.speed) > 2) {
              can_send_flag = 1; // Set flag to send vehicle status
              vehicle_data.speed = adc_buffer[0] * 250 / 4095; // Assuming 0-4095 ADC range
          }
      } else {
          vehicle_data.speed = 0; // Reset speed if not in D or R gear
      }


      if(abs(vehicle_data.battery_level  - adc_buffer[1] * 100 / 4095) > 2) {
          can_send_flag = 1; // Set flag to send vehicle status
          vehicle_data.battery_level = adc_buffer[1] * 100 / 4095; // Assuming 0-4095 ADC range
      }
      
      if(abs(joystick_x - adc_buffer[2])>200 || abs(joystick_y - adc_buffer[3])>200) {
          can_send_flag = 1; // Set flag to send vehicle status

          joystick_x = adc_buffer[2]; // Joystick X-axis
          joystick_y = adc_buffer[3]; // Joystick Y-axis

          float dx = (float)joystick_x - 3150;
          float dy = (float)joystick_y - 3150;

          if( !(dx < 100 && dx > -100 && dy <100 && dy > -100))
          {
              // Tính góc (angle) theo đơn vị độ
            float angle = atan2f(dy, dx) * 180.0f / M_PI;

            // Chuẩn hóa v�? [0, 360) nếu cần
            if (angle < 0)
                angle += 360.0f;

            if (angle >= 45 && angle < 135) {
                vehicle_data.transmission_gear = D_Gear;
            } else if (angle >= 135 && angle < 225) {
                vehicle_data.transmission_gear = P_Gear;
            } else if (angle >= 225 && angle < 315) {
                vehicle_data.transmission_gear = R_Gear;
            } else { // từ 315–360 hoặc 0–45
                vehicle_data.transmission_gear = N_Gear;
            }
          }
          
      }
      
    }
    else
    {      
      vehicle_data.light_status = 0;                     
      vehicle_data.speed = 0;                          
      vehicle_data.transmission_gear = P_Gear;   
    }
    osDelay(20);
  }
  /* USER CODE END Input_Handler */
}

/* USER CODE BEGIN Header_Compute_Handler */
/**
* @brief Function implementing the Compute_Handle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Compute_Handler */
void Compute_Handler(void const * argument)
{
  /* USER CODE BEGIN Compute_Handler */
  /* Infinite loop */
  for(;;)
  {
    vehicle_data.arrived_time = arrived_time_s/ 60; // Convert seconds to minutes
    vehicle_data.arrived_distance = distance_metter/1000; // Convert meters to kilometers
    vehicle_data.total_distance = vehicle_data.battery_level *2;
    osDelay(100);
  }
  /* USER CODE END Compute_Handler */
}

/* USER CODE BEGIN Header_Control_Handler */
/**
* @brief Function implementing the Control_Handle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Handler */
void Control_Handler(void const * argument)
{
  /* USER CODE BEGIN Control_Handler */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  lcd_init(&hlcd, &hi2c1, LCD_ADDR_DEFAULT);// Khởi tạo LCD với địa chỉ mặc định
  DFPLAYER_Init(&mp3, &huart2);                       // Khởi tạo module MP3 DFPlayer
  DFPLAYER_SetVolume(&mp3, 100);
  /* Infinite loop */
  for(;;)
  {
    if(vehicle_data.engine_status==1)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // Turn on engine
      // Display light status on LEDs
      switch (vehicle_data.light_status)
      {
        case 0:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); 
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); 
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); 
          break;
        case 1:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
          break;
        case 2:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
          break;  
        case 3:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); 
          break;
        default:
          break;
      }
      //Display tire pressure on LEDs
      if(vehicle_data.tire_pressure == 1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // Tire pressure enough
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Tire pressure low
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
      }
      // Display door status on LEDs
      if(vehicle_data.door_status == 1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // All doors closed
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);    
      } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // At least one door open
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
      }
      // Display seat belt status on LEDs
      if(vehicle_data.seat_belt_status == 1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Seat belt fastened
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Seat belt unfastened
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      }

      //Display speed 
      if(vehicle_data.transmission_gear == D_Gear) {
        Servo360_Control(vehicle_data.speed, 'D'); // Forward
      }
      else if(vehicle_data.transmission_gear == R_Gear)
      {
        Servo360_Control(vehicle_data.speed, 'R');
      }

      //Display lCD
      lcd_backlight_on(&hlcd);
      lcd_set_cursor(&hlcd, 0,0);
      lcd_printf(&hlcd, "SpeedLimit: %d", speed_limit);
      lcd_set_cursor(&hlcd, 1,0);
      lcd_printf(&hlcd, "AirTemp: %d", air_condition_temp);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        vehicle_data.speed = 0; // Reset speed when engine is off
        Servo360_Control(0, 'R');
        lcd_backlight_off(&hlcd);
    }
    osDelay(20);
  }
  /* USER CODE END Control_Handler */
}

/* USER CODE BEGIN Header_Notify_Handler */
/**
* @brief Function implementing the Notify_Handle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Notify_Handler */
void Notify_Handler(void const * argument)
{
  /* USER CODE BEGIN Notify_Handler */
  /* Infinite loop */
  for(;;)
  {
    if(vehicle_data.transmission_gear ==R_Gear || vehicle_data.transmission_gear == D_Gear)
    {
      if(vehicle_data.speed > speed_limit)
      {
        warning_notify_flag = 1; // Set warning flag for speed limit exceeded
        warning_code = exceed_speed; // Set warning code
      }
      if(vehicle_data.tire_pressure == 0)
      {
        warning_notify_flag = 1; // Set warning flag for low tire pressure
        warning_code = low_tire_pressure; // Set warning code
      }
      if(vehicle_data.door_status == 0)
      {
        warning_notify_flag = 1; // Set warning flag for door open
        warning_code = door_open; // Set warning code
      }
      if(vehicle_data.seat_belt_status == 0)
      {
        warning_notify_flag = 1; // Set warning flag for seat belt unfastened
        warning_code = seat_belt_open; // Set warning code
      }
    }
    if(warning_notify_flag)
    {
      warning_notify_flag=0;
      switch (warning_code)
      {
        case exceed_speed:
          DFPLAYER_PlayTrack(&mp3,EXCEED_SPEED_SOUND); // Play sound for speed limit exceeded
          osDelay(6000);
          DFPLAYER_Stop(&mp3);
          break;
        case low_tire_pressure:
          DFPLAYER_PlayTrack(&mp3,LOW_TIRE_PRESSURE_SOUND); // Play sound for low tire pressure
          osDelay(6000);
          DFPLAYER_Stop(&mp3);
          break;
        case door_open:
          DFPLAYER_PlayTrack(&mp3,DOOR_OPEN_SOUND); // Play sound for door open
          osDelay(6000);
          DFPLAYER_Stop(&mp3);
          break;
        case seat_belt_open:
          DFPLAYER_PlayTrack(&mp3,SEAT_BELT_OPEN_SOUND); // Play sound for seat belt unfastened
          osDelay(6000);
          DFPLAYER_Stop(&mp3);
          break;
        case air_condition_increase:
          DFPLAYER_PlayTrack(&mp3,AIR_CONDITION_INCREASE_SOUND); // Play sound for air condition change
          osDelay(6000);
          DFPLAYER_Stop(&mp3);
          break;
        case air_condition_decrease:
          DFPLAYER_PlayTrack(&mp3,AIR_CONDITION_DECREASE_SOUND); // Play sound for speed limit change
          osDelay(6000);
          DFPLAYER_Stop(&mp3);
        case speed_limit_increase:
          DFPLAYER_PlayTrack(&mp3,SPEED_LIMIT_INCREASE_SOUND); // Play sound for speed limit change
          osDelay(6000);
          DFPLAYER_Stop(&mp3);
          break;
        case speed_limit_decrease:
          DFPLAYER_PlayTrack(&mp3,SPEED_LIMIT_DECREASE_SOUND); // Play sound for
          osDelay(6000);
          DFPLAYER_Stop(&mp3);
        default:
          break;
      }
    }
    osDelay(50);
  }
  /* USER CODE END Notify_Handler */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM3) {
    arrived_time_s +=1;
    distance_metter += round( ((float)vehicle_data.speed / 3.6f) ); // Convert speed from km/h to m/s); 
    if(arrived_time_s % 2 ==0)
    {
      can_send_flag = 1; // Set flag to send data
    }
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
