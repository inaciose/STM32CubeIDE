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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// micro-ros includes
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

//#include <rcutils/logging.h>
//#include <rcl/logging.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

#include <std_msgs/msg/string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define WHEEL_RADIUS   0.05f   // m
#define WHEEL_BASE     0.30f   // m
#define MAX_WHEEL_V    1.0f    // m/s

//#define PWM_MAX        1000
//#define PWM_MIN 	   300
#define PWM_MAX   (__HAL_TIM_GET_AUTORELOAD(&htim5))
#define PWM_MIN   (PWM_MAX * 3 / 10)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rosTask */
osThreadId_t rosTaskHandle;
const osThreadAttr_t rosTask_attributes = {
  .name = "rosTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

// micro-ros node
rcl_node_t node;

// micro-ros publisher
rcl_publisher_t publisher1;
rcl_publisher_t publisher2;
rcl_publisher_t publisher3;

// micro-ros subscriber
rcl_subscription_t cmd_vel_sub;
rclc_executor_t executor;
geometry_msgs__msg__Twist cmd_vel_msg;

static int32_t leticks = 0;
static int32_t reticks = 0;

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FWD,
    MOTOR_REV
} motor_dir_t;

typedef struct {
    motor_dir_t dir;
    uint16_t pwm;
} motor_cmd_t;

typedef struct {
    float v;   // m/s
    float w;   // rad/s
} cmd_vel_t;

//volatile float v_linear = 0.0f;
//volatile float v_angular = 0.0f;
volatile cmd_vel_t last_cmd_vel = {0};

motor_cmd_t cmdA, cmdB;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void *argument);
void StartRosTask(void *argument);
void StartMotorTask(void *argument);

/* USER CODE BEGIN PFP */

// micro-ros
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

// micro-ros subscriber
void cmd_vel_callback(const void * msgin);

// bridge H
void motor_set_A(motor_dir_t dir, uint16_t pwm);
void motor_set_B(motor_dir_t dir, uint16_t pwm);


static inline void cmd_vel_to_wheels(const cmd_vel_t *cv, float *vL, float *vR);
static inline motor_cmd_t wheel_to_motor(float v, int inverted);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

__attribute__((noinline)) void cmd_vel_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  //cb_count++;

  /*
  v_linear  = msg->linear.x;
  v_angular = msg->angular.z;
  */

  last_cmd_vel.v = msg->linear.x;
  last_cmd_vel.w = msg->angular.z;
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  // encoders
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // PWM motores
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of rosTask */
  rosTaskHandle = osThreadNew(StartRosTask, NULL, &rosTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 89;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void motor_set_A(motor_dir_t dir, uint16_t pwm)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);

/*
    RCUTILS_LOG_INFO_NAMED(
      "motor",
      "%ld, %ld",
	  dir,
	  pwm
    );
*/

    if (dir == MOTOR_FWD) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    } else if (dir == MOTOR_REV) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm);
}

void motor_set_B(motor_dir_t dir, uint16_t pwm)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

    if (dir == MOTOR_FWD) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    } else if (dir == MOTOR_REV) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm);
}

static inline void cmd_vel_to_wheels(const cmd_vel_t *cv, float *vL, float *vR)
{
    *vL = cv->v - 0.5f * WHEEL_BASE * cv->w;
    *vR = cv->v + 0.5f * WHEEL_BASE * cv->w;
}

static inline motor_cmd_t wheel_to_motor(float v, int inverted)
{
    motor_cmd_t cmd;

    // handle mirror motors
    if (inverted) {
        v = -v;
    }

    if (v > 0.0f) {
        cmd.dir = MOTOR_FWD;
    } else if (v < 0.0f) {
        cmd.dir = MOTOR_REV;
        v = -v;
    } else {
        cmd.dir = MOTOR_STOP;
        cmd.pwm = 0;
        return cmd;
    }

    if (v > MAX_WHEEL_V)
        v = MAX_WHEEL_V;

    //cmd.pwm = (uint16_t)((v / MAX_WHEEL_V) * PWM_MAX);
    cmd.pwm = PWM_MIN + (uint16_t)((v / MAX_WHEEL_V) * (PWM_MAX - PWM_MIN));

    return cmd;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartRosTask */
/**
* @brief Function implementing the rosTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRosTask */
void StartRosTask(void *argument)
{
  /* USER CODE BEGIN StartRosTask */
  int32_t msg_counter = 0;

  //std_msgs__msg__Int32 msg;

  std_msgs__msg__String msg;
  std_msgs__msg__String__init(&msg);

  std_msgs__msg__Int32 letickmsg;
  std_msgs__msg__Int32 retickmsg;

  rclc_support_t support;
  rcl_allocator_t allocator;

  /*
  static bool initialized = false;
  if (initialized) vTaskDelete(NULL);
  initialized = true;
  */
  osDelay(1000);

  rmw_uros_set_custom_transport(
	true,
	(void *) &huart1,
	cubemx_transport_open,
	cubemx_transport_close,
	cubemx_transport_write,
	cubemx_transport_read);

  /*
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK)
  {
	osDelay(1000);
  }
  */

  //rmw_uros_sync_session(1000);

  /*
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	  printf("Error on default allocators (line %d)\n", __LINE__);
  }
  */

  // micro-ROS app

  //rcutils_logging_initialize();
  //rcutils_logging_set_logger_level("motor", RCUTILS_LOG_SEVERITY_INFO);


  allocator = rcl_get_default_allocator();

  // create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  rclc_node_init_default(&node, "stm32f4_node2", "", &support);


  //rclc_logging_rosout_init(&node);
/*
  rclc_logging_rosout_init_with_allocator(
      &node,
      rcl_get_default_allocator()
  );
*/

  // create publishers
  rclc_publisher_init_default(
	&publisher1,
	&node,
	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	"leticks");

  rclc_publisher_init_default(
	&publisher2,
	&node,
	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	"reticks");

  rclc_publisher_init_default(
	&publisher3,
	&node,
	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
	"log");


  msg_counter =0;

  // create subscriber
  geometry_msgs__msg__Twist__init(&cmd_vel_msg);

  rclc_subscription_init_default(
	&cmd_vel_sub,
	&node,
	ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	"/cmd_vel"
  );

  // micro-ros subscriber executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  rclc_executor_add_subscription(
	&executor,
	&cmd_vel_sub,
	&cmd_vel_msg,
	&cmd_vel_callback,
	ON_NEW_DATA
  );


  rcl_ret_t ret;
  // infinite loop
  for(;;)
  {
	// publisher
	//msg.data=leticks;
	letickmsg.data=leticks;
	retickmsg.data=reticks;
	if ((msg_counter % 100) == 0)
	{
		ret = rcl_publish(&publisher1, &letickmsg, NULL);
		if (ret != RCL_RET_OK)
		{
			printf("Error publishing1 (line %d)\n", __LINE__);
		}


		ret = rcl_publish(&publisher2, &retickmsg, NULL);
		if (ret != RCL_RET_OK)
		{
			printf("Error publishing2 (line %d)\n", __LINE__);
		}

		static char buf[64];
		snprintf(buf, sizeof(buf), "%u %u", cmdA.pwm, cmdB.pwm);

		msg.data.data = buf;
		msg.data.size = strlen(buf);
		msg.data.capacity = sizeof(buf);

		rcl_publish(&publisher3, &msg, NULL);

	}
	msg_counter++;

	// spin (subscribers run on spin?)
	rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
	osDelay(1);

  }
  /* USER CODE END StartRosTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */

  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz

  static int16_t last_l = 0, last_r = 0;

  // motors
  cmd_vel_t cv;
  float vL, vR;
  //motor_cmd_t cmdA, cmdB;


  /* Infinite loop */
  for(;;)
  {
	int16_t now_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
	int16_t now_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

	int16_t dl = now_l - last_l;
	int16_t dr = now_r - last_r;

    last_l = now_l;
    last_r = now_r;

	leticks += (int32_t)dl;
	reticks += (int32_t)dr;

	// cálculo de velocidade / PID / publicação


    //  CMD_VEL (MINIMAL)
    cv.v = last_cmd_vel.v;
    cv.w = last_cmd_vel.w;

    // cmd_vel > velocidades das rodas
    cmd_vel_to_wheels(&cv, &vL, &vR);

    // velocidades > comandos de motor
    cmdA = wheel_to_motor(vL, 1);
    cmdB = wheel_to_motor(vR, 0);

    // APLICAR MOTORES
    motor_set_A(cmdA.dir, cmdA.pwm);
    motor_set_B(cmdB.dir, cmdB.pwm);



    //osDelay(10);
	vTaskDelayUntil(&last_wake, period);
  }
  /* USER CODE END StartMotorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
