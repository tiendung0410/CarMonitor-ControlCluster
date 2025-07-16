/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <time.h>
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Ð?nh nghia struct VehicleStatus
typedef struct {
    uint8_t engine_status;
    uint8_t light_status;
    uint8_t tire_pressure;
    uint8_t door_status;
    uint8_t seat_belt_status;
    uint8_t battery_level;
    uint8_t speed;
    uint8_t arrived_distance;
    uint8_t remain_distance;
    uint8_t avg_speed;
    uint8_t engine_temperature;
    uint8_t transmission_gear;
    uint8_t speed_limit;
    float gps_lat;
    float gps_lon;
} __attribute__((packed)) VehicleStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VEHICLE_STATUS_CAN_ID 0x023
#define PACKET_HEADER_ID 0x024
#define PACKET_DATA_ID 0x025
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
VehicleStatus vehicle_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void SendVehicleStatus(void);
void InitVehicleData(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxMsg;
uint32_t pu32Mailbox;

volatile uint32_t can_status = 0;

// Kh?i t?o d? li?u m?u
void InitVehicleData(void) {
    vehicle_data.engine_status = 1;        // Engine ON
    vehicle_data.light_status = 0;         // Light OFF
    vehicle_data.tire_pressure = 1;       // Tire Pressure Enough
    vehicle_data.door_status = 0;          // All doors closed
    vehicle_data.seat_belt_status = 1;     // Seat belt fastened
    vehicle_data.battery_level = 85;       // 85%
    vehicle_data.speed = 60;               // 60 km/h
    vehicle_data.arrived_distance = 25;    // 25 km
    vehicle_data.remain_distance = 150;    // 150 km
    vehicle_data.avg_speed = 55;           // 55 km/h
    vehicle_data.engine_temperature = 90;  // 90°C
    vehicle_data.transmission_gear = 4;    // Gear 4
    vehicle_data.speed_limit = 80;         // 80 km/h
    vehicle_data.gps_lat = 21.0285f;       // Hanoi latitude
    vehicle_data.gps_lon = 105.8542f;      // Hanoi longitude
}

void RandomVehicleData(void) {
    // Seed random number generator (ch? c?n g?i 1 l?n trong main)
    // srand(time(NULL));
    
    // Engine status: 0 = OFF, 1 = ON
    vehicle_data.engine_status = rand() % 2;
    
    // Light status: 0 = OFF, 1 = ON
    vehicle_data.light_status = rand() % 2;
    
    // Tire pressure: 0 = Low, 1 = Normal, 2 = High
    vehicle_data.tire_pressure = rand() % 3;
    
    // Door status: 0 = All closed, 1 = Some open
    vehicle_data.door_status = rand() % 2;
    
    // Seat belt: 0 = Not fastened, 1 = Fastened
    vehicle_data.seat_belt_status = rand() % 2;
    
    // Battery level: 0-100%
    vehicle_data.battery_level = rand() % 101;
    
    // Speed: 0-120 km/h
    vehicle_data.speed = rand() % 121;
    
    // Arrived distance: 0-500 km
    vehicle_data.arrived_distance = rand() % 501;
    
    // Remain distance: 0-1000 km
    vehicle_data.remain_distance = rand() % 1001;
    
    // Average speed: 20-100 km/h
    vehicle_data.avg_speed = 20 + (rand() % 81);
    
    // Engine temperature: 60-120°C
    vehicle_data.engine_temperature = 60 + (rand() % 61);
    
    // Transmission gear: 1-6
    vehicle_data.transmission_gear = 1 + (rand() % 6);
    
    // Speed limit: 30, 50, 60, 80, 100 km/h
    int speed_limits[] = {30, 50, 60, 80, 100};
    vehicle_data.speed_limit = speed_limits[rand() % 5];
    
    // GPS coordinates around Hanoi area
    // Latitude: 20.9 - 21.1
    vehicle_data.gps_lat = 20.9f + ((float)(rand() % 201) / 1000.0f);
    
    // Longitude: 105.7 - 105.9
    vehicle_data.gps_lon = 105.7f + ((float)(rand() % 201) / 1000.0f);
}

uint8_t *data_ptr;
uint32_t data_size;
uint8_t frame_count;
// Hàm g?i struct qua CAN (chia nh? thành nhi?u frame)
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  TxMsg.DLC = 8;
  TxMsg.IDE = CAN_ID_STD;
  TxMsg.RTR = CAN_RTR_DATA;
  TxMsg.TransmitGlobalTime = DISABLE;
  
  HAL_CAN_Start(&hcan); // start CAN
  InitVehicleData(); // Kh?i t?o d? li?u m?u
  /* USER CODE END 2 */
	srand(HAL_GetTick());
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    
    // G?i struct qua CAN
		RandomVehicleData();
    SendVehicleStatus();
    
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
    HAL_Delay(1000);
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
  * in the RCC_OscInitStruct structure.
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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
