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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LEFT_IN1   GPIOA, GPIO_PIN_0
#define LEFT_IN2   GPIOA, GPIO_PIN_1
#define RIGHT_IN1  GPIOA, GPIO_PIN_2
#define RIGHT_IN2  GPIOA, GPIO_PIN_3

#define PWM_MAX  999

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t t_led = 0;
uint32_t t_uart = 0;

uint32_t LED_PERIOD  = 500;   // ms
uint32_t UART_PERIOD = 1000;  // ms

uint8_t i2c_rx_buf[32];
uint8_t i2c_tx_buf[32];

volatile int16_t encoderLeftPulses  = 0;
volatile int16_t encoderRightPulses = 0;
volatile int16_t leftSpeedPidSetPoint = 0;
volatile int16_t rightSpeedPidSetPoint = 0;
volatile int16_t leftSpeedPidInputLast = 0;
volatile int16_t rightSpeedPidInputLast = 0;
volatile int16_t steeringPidInputLast = 0;

int16_t leftMotorCmd = 0;
int16_t rightMotorCmd = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  HAL_I2C_EnableListen_IT(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  uint32_t now = HAL_GetTick();

	  /* LED blink */
	  if ((now - t_led) >= LED_PERIOD)
	  {
	    t_led = now;
	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  }

	  /* Leitura dos encoders */
	  encoderLeftPulses = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
	  encoderRightPulses = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

	  apply_motors(leftSpeedPidSetPoint, rightSpeedPidSetPoint);

	  /* UART debug */
	  if ((now - t_uart) >= UART_PERIOD)
	  {
	    t_uart = now;
	    char buf[64];
	    int n = sprintf(buf, "E1=%ld E2=%ld s1=%ld s2=%ld\r\n", encoderLeftPulses, encoderRightPulses, leftSpeedPidSetPoint, rightSpeedPidSetPoint);
	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 50);
	  }

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

/* USER CODE BEGIN 4 */
static inline void left_forward(void){
  HAL_GPIO_WritePin(LEFT_IN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LEFT_IN2, GPIO_PIN_RESET);
}
static inline void left_backward(void){
  HAL_GPIO_WritePin(LEFT_IN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEFT_IN2, GPIO_PIN_SET);
}
static inline void left_stop(void){
  HAL_GPIO_WritePin(LEFT_IN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEFT_IN2, GPIO_PIN_RESET);
}

static inline void right_forward(void){
  HAL_GPIO_WritePin(RIGHT_IN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RIGHT_IN2, GPIO_PIN_RESET);
}
static inline void right_backward(void){
  HAL_GPIO_WritePin(RIGHT_IN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RIGHT_IN2, GPIO_PIN_SET);
}
static inline void right_stop(void){
  HAL_GPIO_WritePin(RIGHT_IN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RIGHT_IN2, GPIO_PIN_RESET);
}

void apply_motors(int16_t left, int16_t right)
{
  // LEFT
  if (left == 0) {
    left_stop();
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  } else if (left > 0) {
    left_forward();
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (left > PWM_MAX ? PWM_MAX : left));
  } else {
    left_backward();
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ((-left) > PWM_MAX ? PWM_MAX : -left));
  }

  // RIGHT
  if (right == 0) {
    right_stop();
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  } else if (right > 0) {
    right_forward();
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (right > PWM_MAX ? PWM_MAX : right));
  } else {
    right_backward();
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, ((-right) > PWM_MAX ? PWM_MAX : -right));
  }
}






void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c,
                          uint8_t TransferDirection,
                          uint16_t AddrMatchCode)
{
  if (hi2c->Instance == I2C1)
  {
    if (TransferDirection == I2C_DIRECTION_TRANSMIT)
    {
      // Master vai escrever → nós vamos receber
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, i2c_rx_buf, 32, I2C_FIRST_AND_LAST_FRAME);
    }
    else
    {
      prepare_i2c_tx();
      // Master vai ler → nós vamos enviar
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2c_tx_buf, 32, I2C_FIRST_FRAME);
    }
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1) return;

  uint8_t cmd = i2c_rx_buf[0];

  //int16_t *data = (int16_t*)&i2c_rx_buf[0];

  switch (cmd)
  {
    case 1: // CMD_RESET_ENCODERS
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      encoderLeftPulses = 0;
      encoderRightPulses = 0;
      break;

    case 2: // CMD_SET_PWM
      //leftSpeedPidSetPoint  = data[1];
      //rightSpeedPidSetPoint = data[2];
      // com inversão de bytes no pi
      leftSpeedPidSetPoint  = ((int16_t*)i2c_rx_buf)[1];
      rightSpeedPidSetPoint = ((int16_t*)i2c_rx_buf)[2];
      // sem inversão dos bytesno pi //
      // buf[1] | (buf[2]<<8)
      //leftSpeedPidSetPoint  = (int16_t)(i2c_rx_buf[1] | (i2c_rx_buf[2] << 8));
      //rightSpeedPidSetPoint = (int16_t)(i2c_rx_buf[3] | (i2c_rx_buf[4] << 8));
      break;

    case 3: // CMD_LIDAR_MOTOR_ON
      // placeholder
      break;

    case 4: // CMD_LIDAR_MOTOR_OFF
      // placeholder
      break;

    case 5: // CMD_SET_MOTION
    case 6: // CMD_SET_VEL
      //leftSpeedPidSetPoint  = data[1];
      //rightSpeedPidSetPoint = data[2];
      leftSpeedPidSetPoint  = ((int16_t*)i2c_rx_buf)[1];
      rightSpeedPidSetPoint = ((int16_t*)i2c_rx_buf)[2];
      break;

    case 81:
    case 82:
    case 83:
      // PID tunings → ignorado nesta fase
      break;
  }
}

void prepare_i2c_tx(void)
{
  int16_t *tx = (int16_t*)i2c_tx_buf;

  tx[0] = 0;
  tx[1] = encoderLeftPulses;
  tx[2] = encoderRightPulses;
  tx[3] = leftSpeedPidSetPoint - leftSpeedPidInputLast;
  tx[4] = rightSpeedPidSetPoint - rightSpeedPidInputLast;
  tx[5] = steeringPidInputLast;
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  prepare_i2c_tx();
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  HAL_I2C_EnableListen_IT(hi2c);
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
