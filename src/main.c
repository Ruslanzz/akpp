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
#include <stdbool.h>

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
#define BASE_RELAY        0x00
#define BASE_ADC1         0x10
#define BASE_ADC2         0x20
#define BASE_CONTROL      0x30
#define BASE_COMP         0x40
#define BASE_RELAY_OUT    
#define BASE_SELECTOR     0x50

#define RELAY_COUNT       5
#define ADC1_COUNT        2
#define ADC2_COUNT        2
#define CONTROL_COUNT     1
#define COMP_COUNT        1
#define RELAY_OUT_COUNT   1
#define SELECTOR_COUNT    1

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint32_t ADS_RES_BUFFER[8];
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint32_t TxMailbox = 0;

uint16_t l_pwm_value = 100;
uint16_t r_pwm_value = 100;
int8_t step = 0;
unsigned char Selector = 'N';
unsigned char Position;
unsigned int selector_int;
unsigned int position_int;
int period = 200;
int mperiod = 150;

int selector_r;
int selector_d;
int temp_sens_1;
int temp_sens_2;

GPIO_PinState position_p;
GPIO_PinState position_r;
GPIO_PinState position_n;
GPIO_PinState position_d;
GPIO_PinState position_2;
GPIO_PinState position_1;
GPIO_PinState position_pn;

uint16_t gpio_pin;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
typedef struct {
  GPIO_TypeDef* port;  // Указатель на порт GPIO
  uint16_t pin;        // Номер пина
} GPIO_Config;

GPIO_Config comp[] = {
{ GPIOB, COMP_ADC_1_Pin },
{ GPIOB, COMP_ADC_2_Pin },
{ GPIOB, COMP_ADC_3_Pin },
{ GPIOB, COMP_ADC_4_Pin },
{ GPIOC, COMP_ADC_5_Pin },
{ GPIOC, COMP_ADC_6_Pin },
{ GPIOC, COMP_ADC_7_Pin },
{ GPIOA, COMP_ADC_8_Pin }
};

GPIO_Config relay[] = {
{ GPIOB, EN_RELAY_1_Pin },
{ GPIOB, EN_RELAY_2_Pin },
{ GPIOB, EN_RELAY_3_Pin },
{ GPIOB, EN_RELAY_4_Pin },
{ GPIOB, EN_RELAY_5_Pin }
};

bool master = false;
uint8_t device_id = 0x03;

/* USER CODE END PFP */
// Функция для чтения состояния пина с использованием структуры
int Read_GPIO_Pin(GPIO_Config config) {
  return (HAL_GPIO_ReadPin(config.port, config.pin) == GPIO_PIN_SET) ? 0 : 1;
}


// Функция для включения реле по индексу
void turn_on_relay(uint8_t relay_index) {
  if (relay_index < RELAY_COUNT) {
      HAL_GPIO_WritePin(relay[relay_index].port, relay[relay_index].pin, GPIO_PIN_SET);
}
}

// Функция для выключения реле по индексу
void turn_off_relay(uint8_t relay_index) {
  if (relay_index < RELAY_COUNT) {
      HAL_GPIO_WritePin(relay[relay_index].port, relay[relay_index].pin, GPIO_PIN_RESET);
  }
}

// Генерация stdid
uint32_t generate_stdid(uint8_t device_id, uint8_t base_index, uint8_t parameter_index) {
  return ((uint32_t)device_id << 8) | (uint32_t)(base_index + parameter_index);
}

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
    uint32_t stdid = RxHeader.StdId;
    // Извлекаем device_id и parameter_index из stdid
    uint8_t std_device_id = (stdid >> 8) & 0xFF; // Старшие 8 бит
    uint8_t parameter_index = stdid & 0xFF;   // Младшие 8 бит
    
    if (std_device_id == device_id) {
      if (parameter_index >= BASE_RELAY && parameter_index < BASE_RELAY + RELAY_COUNT) {
        // Обработка реле
        uint8_t relay_index = parameter_index - BASE_RELAY;
        if (RxData[0] == 1) {          
          turn_on_relay(relay_index);
        }
        else {
          turn_off_relay(relay_index);  
        } 
      }        
    }
    if (std_device_id == 1 ) { 
      if (parameter_index == BASE_COMP + 1) {
        if (RxData[2] == 1) {
          Selector = 'D';          
        }         
        if (RxData[3] == 1) {
          Selector = 'R';          
        }
        if (RxData[2] == 0 && RxData[3] == 0){
          Selector = 'N';
        } 
        if (RxData[4] == 1){
          HAL_GPIO_WritePin(GPIOB, EN_RELAY_2_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, EN_RELAY_4_Pin, GPIO_PIN_RESET);
        }
        if (RxData[4] == 0){
          HAL_GPIO_WritePin(GPIOB, EN_RELAY_2_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, EN_RELAY_4_Pin, GPIO_PIN_SET);
        }      
        if (RxData[5] == 1){
          HAL_GPIO_WritePin(GPIOB, EN_RELAY_3_Pin, GPIO_PIN_SET);
        }
        if (RxData[5] == 0){
          HAL_GPIO_WritePin(GPIOB, EN_RELAY_3_Pin, GPIO_PIN_RESET);
        }  
      }     
    }
  }
}

void CAN_SendMessage(uint32_t StdId, uint8_t* data, uint8_t dataLength) {
  uint32_t TxMailbox = 0;
  // Заголовок CAN-сообщения
  TxHeader.StdId = StdId;       // Идентификатор сообщения (передаётся как параметр)
  TxHeader.ExtId = 0x00;        // Расширенный идентификатор (не используется)
  TxHeader.IDE = CAN_ID_STD;    // Стандартный идентификатор
  TxHeader.RTR = CAN_RTR_DATA;  // Тип сообщения (данные)
  TxHeader.DLC = dataLength;    // Длина данных (передаётся как параметр)

  // Копирование данных в TxData
  for (uint8_t i = 0; i < dataLength; i++) {
      TxData[i] = data[i];  // Копируем данные из переданного массива
  }

  // Очистка оставшихся байтов (если dataLength < 8)
    if (dataLength < 8){
  for (uint8_t i = dataLength; i < 8; i++) {
      TxData[i] = 0x00;
      }
  }

  // Отправка сообщения
  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
      // Обработка ошибки отправки
      //Error_Handler();
  }
}


struct pwm
{
  int rpwm;
  int lpwm;
  int r_en;
  int l_en;
};

struct retrac
{
  int enable;
  int left;
  int right;
};

struct retrac retrac_status;


struct pwm position_case(char Position, int selector_int) {
    struct pwm ret;
    switch (Position)
    {   case 'P':
           position_int =  1;
           if (position_int > selector_int) {               
               ret.rpwm = 0;
               ret.lpwm = mperiod;
               ret.l_en = period;
               ret.r_en = period;           
               return ret;
           }
           if (position_int < selector_int) {
               struct pwm ret;
               ret.rpwm = mperiod;
               ret.lpwm = 0;
               ret.l_en = period;
               ret.r_en = period;
               return ret;
           } 
          break;
        case 'R':
           position_int =  2;
           if (position_int > selector_int) {               
               ret.rpwm = 0;
               ret.lpwm = mperiod;
               ret.l_en = period;
               ret.r_en = period;
               return ret;
           }
           if (position_int < selector_int) {
               struct pwm ret;
               ret.rpwm = mperiod;
               ret.lpwm = 0;
               ret.l_en = period;
               ret.r_en = period;
               return ret;
           } 
          break;        
        case 'N':
           position_int =  3;
           if (position_int > selector_int) {               
               ret.rpwm = 0;
               ret.lpwm = mperiod;
               ret.l_en = period;
               ret.r_en = period;
               return ret;
           }
           if (position_int < selector_int) {
               struct pwm ret;
               ret.rpwm = mperiod;
               ret.lpwm = 0;
               ret.l_en = period;
               ret.r_en = period;
               return ret;
           }
          break;                              
        case 'D':
           position_int =  4;
           if (position_int > selector_int) {               
               ret.rpwm = 0;
               ret.lpwm = mperiod;
               ret.l_en = period;
               ret.r_en = period;
               return ret;
           }
           if (position_int < selector_int) {
               struct pwm ret;
               ret.rpwm = mperiod;
               ret.lpwm = 0;
               ret.l_en = period;
               ret.r_en = period;
               return ret;
           }
          break;  
        case '2':
           position_int =  5;
           if (position_int > selector_int) {               
               ret.rpwm = 0;
               ret.lpwm = mperiod;
               ret.l_en = period;
               ret.r_en = period;
               return ret;
           }
           if (position_int < selector_int) {
               struct pwm ret;
               ret.rpwm = mperiod;
               ret.lpwm = 0;
               ret.l_en = period;
               ret.r_en = period;
               return ret;
           } 
          break;
        case '1':
           position_int =  6;          
           if (position_int > selector_int) {               
               ret.rpwm = 0;
               ret.lpwm = mperiod;
               ret.l_en = period;
               ret.r_en = period;
               return ret;
           }
           if (position_int < selector_int) {
               struct pwm ret;
               ret.rpwm = mperiod;
               ret.lpwm = 0;
               ret.l_en = period;
               ret.r_en = period;
               return ret;
           } 
          break; 
        default:
          ret.lpwm = 0;
          ret.rpwm = 0;
          ret.l_en = 0;
          ret.r_en = 0;
          return ret;

               
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_CAN_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);

  HAL_GPIO_WritePin(GPIOA, CAN_STB_Pin, GPIO_PIN_RESET);

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);  

  while (1)
  {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADS_RES_BUFFER, 8);   

    if (TIM_CHANNEL_STATE_GET(&htim1, TIM_CHANNEL_3) == HAL_TIM_CHANNEL_STATE_READY) {
            uint8_t data_comp[8];            
            uint32_t stdid;            
            // for (uint8_t i = 0; i < 8; i++) {
            //     data_comp[i] = Read_GPIO_Pin(comp[i]);
            // }            
            // stdid = generate_stdid(device_id, BASE_COMP, COMP_COUNT);      
            // CAN_SendMessage(stdid, data_comp, 8);


            // uint8_t data_adc1[8];
            // uint8_t j = 0;
            // for (uint8_t i = 0; i < 4; i++) {                
            //     data_adc1[j] = (uint8_t)(ADS_RES_BUFFER[i] & 0xFF);       // Младший байт
            //     j += 1;
            //     data_adc1[j] = (uint8_t)((ADS_RES_BUFFER[i] >> 8) & 0xFF); // Старший байт
            //     j += 1;
            // }
            // stdid = generate_stdid(device_id, BASE_ADC1, ADC1_COUNT);
            // CAN_SendMessage(stdid, data_adc1, 8);

            // uint8_t data_adc2[8];
            // j = 0;
            // for (uint8_t i = 0; i < 4; i++) {                
            //     data_adc2[j] = (uint8_t)(ADS_RES_BUFFER[i] & 0xFF);       // Младший байт
            //     j += 1;
            //     data_adc2[j] = (uint8_t)((ADS_RES_BUFFER[i] >> 8) & 0xFF); // Старший байт 
            //     j += 1;
            // }
            // stdid = generate_stdid(device_id, BASE_ADC2, ADC2_COUNT);
            // CAN_SendMessage(stdid, data_adc2, 8);

            // uint8_t data_relay[4]; 
            // for (uint8_t i = 0; i < 5; i++) {
            //     data_relay[i] = Read_GPIO_Pin(relay[i]);
            // }
            // stdid = generate_stdid(device_id, BASE_RELAY_OUT, RELAY_OUT_COUNT);
            // CAN_SendMessage(stdid, data_relay, 4);
      __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE); 
      HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
    }  
   
    position_p = Read_GPIO_Pin(comp[0]);
    position_r = Read_GPIO_Pin(comp[1]);
    position_n = Read_GPIO_Pin(comp[2]);
    position_d = Read_GPIO_Pin(comp[3]);
    position_2 = Read_GPIO_Pin(comp[4]);
    position_1 = Read_GPIO_Pin(comp[5]);
    position_pn = Read_GPIO_Pin(comp[6]);
    
    /* USER CODE END WHILE */

    //if ((position_p == GPIO_PIN_RESET) && (position_pn == GPIO_PIN_RESET))
    if (position_p == GPIO_PIN_RESET)
    {
        Position = 'P';
    }
    if (position_r == GPIO_PIN_RESET)
    {
        Position = 'R';
    }
   // if ((position_n == GPIO_PIN_RESET) && (position_pn == GPIO_PIN_RESET))
    if (position_n == GPIO_PIN_RESET)
    {
        Position = 'N';
    }
    if (position_d == GPIO_PIN_RESET)
    {
        Position = 'D';
    }
    if (position_2 == GPIO_PIN_RESET)
    {
        Position = '2';
    }
    if (position_1 == GPIO_PIN_RESET)
    {
        Position = '1';
    }  
   

    switch ( Selector ) {
        case 'R':
            selector_int = 2;
            gpio_pin = 2;
            if (Position != 'R') {                           
                struct pwm pwm_result = position_case(Position, selector_int);                
                setPWM(pwm_result.lpwm,pwm_result.rpwm,pwm_result.r_en,pwm_result.l_en);                
            } 
            else {
              setPWM(0,0,0,0);
            }
            break;          
        case 'N':
            selector_int = 3;
            gpio_pin = 3;
            if (Position != 'N') {                 
                struct pwm pwm_result = position_case(Position, selector_int);                
                setPWM(pwm_result.lpwm,pwm_result.rpwm,pwm_result.r_en,pwm_result.l_en);                 
            } 
            else {
              setPWM(0,0,0,0);
            }
            break;          
        case 'D':
            selector_int = 4;
            gpio_pin = 4;
            if (Position != 'D') {                
                struct pwm pwm_result = position_case(Position, selector_int);                
                setPWM(pwm_result.lpwm,pwm_result.rpwm,pwm_result.r_en,pwm_result.l_en);                  
            } 
            else {
              setPWM(0,0,0,0);
            }
            break; 
          }
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_8;
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
  CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; 
  sFilterConfig.FilterIdHigh = 0x000;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
  Error_Handler();
  }
  /* USER CODE END CAN_Init 2 */

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
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 10;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = period-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DRV2_EN_B_Pin|EN_RELAY_1_Pin|EN_RELAY_2_Pin|EN_RELAY_3_Pin
                          |EN_RELAY_4_Pin|EN_RELAY_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_B_Pin|CAN_STB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COMP_ADC_5_Pin COMP_ADC_6_Pin COMP_ADC_7_Pin */
  GPIO_InitStruct.Pin = COMP_ADC_5_Pin|COMP_ADC_6_Pin|COMP_ADC_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : COMP_ADC_8_Pin */
  GPIO_InitStruct.Pin = COMP_ADC_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(COMP_ADC_8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COMP_ADC_1_Pin COMP_ADC_2_Pin COMP_ADC_3_Pin COMP_ADC_4_Pin */
  GPIO_InitStruct.Pin = COMP_ADC_1_Pin|COMP_ADC_2_Pin|COMP_ADC_3_Pin|COMP_ADC_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV2_EN_B_Pin EN_RELAY_1_Pin EN_RELAY_2_Pin EN_RELAY_3_Pin
                           EN_RELAY_4_Pin EN_RELAY_5_Pin */
  GPIO_InitStruct.Pin = DRV2_EN_B_Pin|EN_RELAY_1_Pin|EN_RELAY_2_Pin|EN_RELAY_3_Pin
                          |EN_RELAY_4_Pin|EN_RELAY_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV1_EN_B_Pin */
  GPIO_InitStruct.Pin = DRV1_EN_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRV1_EN_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV1_EN_A_Pin */
  GPIO_InitStruct.Pin = DRV1_EN_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRV1_EN_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_STB_Pin */
  GPIO_InitStruct.Pin = CAN_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_STB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV2_EN_A_Pin */
  GPIO_InitStruct.Pin = DRV2_EN_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRV2_EN_A_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void setPWM(int lpwm, int rpwm, int r_en, int l_en)
{
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, lpwm);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, rpwm);

   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, l_en);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, r_en);  
}

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//   if(GPIO_Pin == gpio_pin) {    
//    setPWM(0,0,0,0);                               
//   }
// }
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
