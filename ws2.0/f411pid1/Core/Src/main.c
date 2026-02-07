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

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

#include "pid_v1.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// i2c commands protocol
#define CMD_RESET_ENCODERS 1
#define CMD_SET_PWM 2           // params: pwmLeft, pwmRight
#define CMD_LIDAR_MOTOR_ON 3
#define CMD_LIDAR_MOTOR_OFF 4
#define CMD_SET_MOTION 5        // params: leftSetPoint, rightSetpoint, distance
#define CMD_SET_VEL 6
#define CMD_SET_PID_KL  81
#define CMD_SET_PID_KR  82
#define CMD_SET_PID_KS  83

// motors pins
//#define LEFT_IN1   GPIOA, GPIO_PIN_4
//#define LEFT_IN2   GPIOB, GPIO_PIN_10
#define DAIN1  GPIOB, GPIO_PIN_10
#define DAIN2  GPIOA, GPIO_PIN_4
#define DAIN3  GPIOB, GPIO_PIN_12
#define DAIN4  GPIOB, GPIO_PIN_13

// LeftMotor
//#define leftMotorEn ENA
#define leftMotorEnENxTIM 5
#define leftMotorEnENxCH 1
#define leftMotorForward DAIN1
#define leftMotorBackward DAIN2

// RightMotor
//#define rightMotorEn ENB
#define rightMotorEnENxTIM 5
#define rightMotorEnENxCH 2
#define rightMotorForward DAIN3
#define rightMotorBackward DAIN4

#define PWM_MAX  9999

// pid.h (defines no antigo ficheiro pid.h e usadas no pid.ino e no diffbase.ino)

#define SPEED_PID_SAMPLE_TIME 25.0

// left speed PID control
#define LEFT_SPEED_PID_MIN_OUTPUT 5630
#define LEFT_SPEED_PID_MAX_OUTPUT 9999

// right speed PID control
#define RIGHT_SPEED_PID_MIN_OUTPUT 5630
#define RIGHT_SPEED_PID_MAX_OUTPUT 9999

// steering PID
#define STEERING_PID_SAMPLE_TIME 50 // 5
#define STEERING_PID_TARGET 0
#define STEERING_PID_MIN_OUTPUT -100
#define STEERING_PID_MAX_OUTPUT 100

// adaptative pid
//#define STEERING_PID_ADAPTATIVE_LIMIT 20
//#define STEERING_PID_ADAPTATIVE_DELAY 100
//#define SPEED_PID_ADAPTATIVE_LIMIT1 15
//#define SPEED_PID_ADAPTATIVE_LIMIT2 30
// speed change process
//#define SPEED_PID_CHANGE_DELAY 300   // time to wait before start change speed (SPEED_PID_SAMPLE_TIME * 12)
//unsigned long speedPidChangeTimer;
//int speedPidStepInterval = 250; // time between speed change steps
//byte speedPidChangeStatus = 0;


//#define MOTOR_TIME 200
//#define LIDAR_MOTOR_PIN PB12

// eof pid.h

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

// left encoder
volatile int16_t encoderLeftPulsesRead  = 0;
int32_t encoderLeftPulses = 0;
int16_t encoderLeftLastRead = 0;
int16_t encoderLeftSpeedZeroRef = 0;
int16_t encoderLeftSteeringZeroRef = 0;
int16_t encoderLeftPulsesSpeedPID; 		// the number of pulses for PID
int16_t encoderLeftPulsesSteeringPID; 	// the number of pulses for PID, must be reset at same time
int8_t  encoderLeftSign = +1;   		// usar -1 se motor estiver invertido


// right encoder
volatile int16_t encoderRightPulsesRead = 0;
int32_t encoderRightPulses = 0;
int16_t encoderRightLastRead = 0;
int16_t encoderRightSpeedZeroRef = 0;
int16_t encoderRightSteeringZeroRef = 0;
int16_t encoderRightPulsesSpeedPID; // the number of pulses PID
int16_t encoderRightPulsesSteeringPID; // the number of pulses for PID, must be reset at same time
int8_t  encoderRightSign = +1;  // usar -1 se motor estiver invertido


// pid.h (variaveis declaradas no antigo ficheiro pid.h e usadas no pid.ino e no diffbase.ino)

// steering PID control
bool useSteeringPid = false;

double steeringPidKp0 = 1.0;
double steeringPidKi0 = 0.01;
double steeringPidKd0 = 0.0;

double steeringPidInput = 0;
int16_t steeringPidInputLast = 0;
double steeringPidOutput; // Power supplied to the motor PWM value.
double steeringPidSetPoint;
bool steeringdPidResult = false;

//PID steeringPid(&steeringPidInput, &steeringPidOutput, &steeringPidSetPoint, steeringPidKp0, steeringPidKi0, steeringPidKd0, REVERSE);
PID_t steeringPid;


// left speed PID control
double leftSpeedPidKp0 = 100;
double leftSpeedPidKi0 = 0;
double leftSpeedPidKd0 = 0;

double leftSpeedPidInput = 0;
int16_t leftSpeedPidInputLast = 0; // debug
double leftSpeedPidOutput; // Power supplied to the motor PWM value.
double leftSpeedPidSetPoint;
bool leftSpeedPidResult = false;

double leftSpeedPidSetPointTmp;
int16_t leftSpeedPidSetPointDirection;

//PID leftSpeedPid(&leftSpeedPidInput, &leftSpeedPidOutput, &leftSpeedPidSetPoint, leftSpeedPidKp0, leftSpeedPidKi0, leftSpeedPidKd0, DIRECT);
PID_t leftSpeedPid;


// right speed PID control
double rightSpeedPidKp0 = 100;
double rightSpeedPidKi0 = 0;
double rightSpeedPidKd0 = 0;

double rightSpeedPidInput = 0;
int16_t rightSpeedPidInputLast = 0; // debug
double rightSpeedPidOutput; // Power supplied to the motor PWM value.
double rightSpeedPidSetPoint;
bool rightSpeedPidResult = false;

double rightSpeedPidSetPointTmp;
int16_t rightSpeedPidSetPointDirection;

//PID rightSpeedPid(&rightSpeedPidInput, &rightSpeedPidOutput, &rightSpeedPidSetPoint, rightSpeedPidKp0, rightSpeedPidKi0, rightSpeedPidKd0, DIRECT);
PID_t rightSpeedPid;


// adaptative steering pid
//bool steeringPidAdaptativeStart = false;
//unsigned long steeringPidAdaptativeTimer;
//double steeringPidKp1 = 100.0;
//double steeringPidKi1 = 10.0;
//double steeringPidKd1 = 0;

// adaptative left pid
//double leftSpeedPidKp1 = 6000;
//double leftSpeedPidKi1 = 500;
//double leftSpeedPidKd1 = 0;
//double leftSpeedPidKp2 = 6000;
//double leftSpeedPidKi2 = 500;
//double leftSpeedPidKd2 = 0;

// adaptative right pid
//double rightSpeedPidKp1 = 6000;
//double rightSpeedPidKi1 = 500;
//double rightSpeedPidKd1 = 0;
//double rightSpeedPidKp2 = 6000;
//double rightSpeedPidKi2 = 500;
//double rightSpeedPidKd2 = 0;

// other
double minPidVel = 0.40;
double maxPidVel = 1.40;

// eof pid.h

/*
long encoderLeftPulsesTarget = 0;
long encoderRightPulsesTarget = 0;
bool encoderLeftPulsesOnTarget = false;
bool encoderRightPulsesOnTarget = false;
bool encoderPulsesTargetEnabled = false;
int encoderLeftPulsesTargetStopOffset = 0;
int encoderRightPulsesTargetStopOffset = 0;

long encoderLeftPulsesTargetStart = 0;
long encoderRightPulsesTargetStart = 0;
*/

int16_t leftMotorPwmOut = 0;
int16_t rightMotorPwmOut = 0;
int16_t leftMotorPwmOutCmd = 0;
int16_t rightMotorPwmOutCmd = 0;
bool PWMdirect = false;

//unsigned long motorTimer = 0;
//int motorPwm = 0;
//bool motorPwmUp = true;
//int lidarMotorPower = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//void apply_motors(int16_t left, int16_t right);
void prepare_i2c_tx(void);

void setup_PID(void);
void update_PID(void);
bool leftSpeedPidCompute(void);
bool rightSpeedPidCompute(void);
bool steeringPidCompute(void);

void pwmWrite(uint8_t tim, uint8_t ch, int16_t pwm);
void bodyMotorsControl(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
*/

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

  HAL_I2C_EnableListen_IT(&hi2c1);

  setup_PID();

  /*
  // left pid control start
  PID_Init(&leftSpeedPid, &leftSpeedPidInput, &leftSpeedPidOutput, &leftSpeedPidSetPoint,
             2.0, 5.0, 1.0, P_ON_E, DIRECT);

  PID_SetMode(&leftSpeedPid, AUTOMATIC);

  PID_Init(&rightSpeedPid, &rightSpeedPidInput, &rightSpeedPidOutput, &rightSpeedPidSetPoint,
             2.0, 5.0, 1.0, P_ON_E, DIRECT);

  PID_SetMode(&rightSpeedPid, AUTOMATIC);
  */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  static double encoderLeftPulsesLast = 999;
	  static double encoderRightPulsesLast = 999;

	  uint32_t now = HAL_GetTick();

	  // LED blink
	  if ((now - t_led) >= LED_PERIOD)
	  {
	    t_led = now;
	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  }

	  // read encoders
	  encoderLeftPulsesRead = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
	  encoderRightPulsesRead = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

	  // -------- LEFT --------
	  int16_t leftDelta = encoderLeftPulsesRead - encoderLeftLastRead;
	  // overflow tratado automaticamente por aritmética int16_t
	  encoderLeftLastRead = encoderLeftPulsesRead;
	  encoderLeftPulses += (int32_t)(encoderLeftSign * leftDelta);
	  encoderLeftPulsesSpeedPID = (int16_t)(encoderLeftSign * (encoderLeftPulsesRead - encoderLeftSpeedZeroRef));
	  encoderLeftPulsesSteeringPID = (int16_t)(encoderLeftSign * (encoderLeftPulsesRead - encoderLeftSteeringZeroRef));

	  // -------- RIGHT --------
	  int16_t rightDelta = encoderRightPulsesRead - encoderRightLastRead;
	  encoderRightLastRead = encoderRightPulsesRead;
	  encoderRightPulses += (int32_t)(encoderRightSign * rightDelta);
	  encoderRightPulsesSpeedPID = (int16_t)(encoderRightSign * (encoderRightPulsesRead - encoderRightSpeedZeroRef));
	  encoderRightPulsesSteeringPID = (int16_t)(encoderRightSign * (encoderRightPulsesRead - encoderRightSteeringZeroRef));


	  if(encoderLeftPulses != encoderLeftPulsesLast || encoderRightPulses != encoderRightPulsesLast) {
	    if(encoderLeftPulses != encoderLeftPulsesLast) encoderLeftPulsesLast = encoderLeftPulses;
	    if(encoderRightPulses != encoderRightPulsesLast) encoderRightPulsesLast = encoderRightPulses;
	    //Serial.print(encoderLeftPulses); Serial.print("\t"); Serial.print(encoderRightPulses); Serial.println("\t");
	    //Serial.print(leftMotorPwmOut); Serial.print("\t"); Serial.print(rightMotorPwmOut); Serial.print("\t"); Serial.print(encoderLeftPulses); Serial.print("\t"); Serial.print(encoderRightPulses); Serial.println("\t");
	  }


	  // check encoder targets
	  // TODO

	  //PID_Compute(&pidfl);
	  //PID_Compute(&pidfr);
	  update_PID();

	  // set speed
	  if(PWMdirect) {
	    leftMotorPwmOut = leftMotorPwmOutCmd;
	    rightMotorPwmOut = rightMotorPwmOutCmd;
	    //PWMdirect = false;
	  } else {
	    leftMotorPwmOut = 0;
	    rightMotorPwmOut = 0;
	    if(leftSpeedPidSetPoint != 0) {
	      leftMotorPwmOut = leftSpeedPidOutput * leftSpeedPidSetPointDirection; // - steeringPidOutput;
	    }
	    if(rightSpeedPidSetPoint != 0) {
	      rightMotorPwmOut = rightSpeedPidOutput * rightSpeedPidSetPointDirection; // + steeringPidOutput;
	    }
	  }


	  // UART debug
	  if ((now - t_uart) >= UART_PERIOD)
	  {
	    t_uart = now;
	    char buf[64];
	    int n = sprintf(buf, "El=%ld Er=%ld spl=%d spr=%d pwl=%d pwr=%d pws=%d\r\n",
	    		encoderLeftPulses,
				encoderRightPulses,
				(int)leftSpeedPidSetPoint,
				(int)rightSpeedPidSetPoint,
				leftMotorPwmOut,
				rightMotorPwmOut,
				(int)PWMdirect
				);
	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 50);
	  }

	  bodyMotorsControl();

	  //apply_motors(leftSpeedPidSetPoint, rightSpeedPidSetPoint);

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

/* USER CODE BEGIN 4 */

void setup_PID() {

  // steering PID
  steeringPidSetPoint = STEERING_PID_TARGET;
  PID_Init(&steeringPid, &steeringPidInput, &steeringPidOutput, &steeringPidSetPoint,
		  steeringPidKp0, steeringPidKi0, steeringPidKd0, P_ON_E, REVERSE);

  //PID_SetMode(&steeringPid, AUTOMATIC);
  PID_SetMode(&steeringPid, MANUAL);
  PID_SetSampleTime(&steeringPid, STEERING_PID_SAMPLE_TIME);
  PID_SetOutputLimits(&steeringPid, STEERING_PID_MIN_OUTPUT, STEERING_PID_MAX_OUTPUT);


  // left speed PID
  leftSpeedPidSetPoint = 0;

  PID_Init(&leftSpeedPid, &leftSpeedPidInput, &leftSpeedPidOutput, &leftSpeedPidSetPoint,
		  leftSpeedPidKp0, leftSpeedPidKi0, leftSpeedPidKd0, P_ON_E, DIRECT);

  //PID_SetMode(&leftSpeedPid, AUTOMATIC);
  PID_SetMode(&leftSpeedPid, MANUAL);
  PID_SetSampleTime(&leftSpeedPid, SPEED_PID_SAMPLE_TIME);
  PID_SetOutputLimits(&leftSpeedPid, LEFT_SPEED_PID_MIN_OUTPUT, LEFT_SPEED_PID_MAX_OUTPUT);

  // right speed PID
  rightSpeedPidSetPoint = 0;

  PID_Init(&rightSpeedPid, &rightSpeedPidInput, &rightSpeedPidOutput, &rightSpeedPidSetPoint,
		  rightSpeedPidKp0, rightSpeedPidKi0, rightSpeedPidKd0, P_ON_E, DIRECT);

  //PID_SetMode(&rightSpeedPid, AUTOMATIC);
  PID_SetMode(&rightSpeedPid, MANUAL);
  PID_SetSampleTime(&rightSpeedPid, SPEED_PID_SAMPLE_TIME);
  PID_SetOutputLimits(&rightSpeedPid, RIGHT_SPEED_PID_MIN_OUTPUT, RIGHT_SPEED_PID_MAX_OUTPUT);

}


void update_PID() {
  // calculate steeringPid
  if(useSteeringPid) {
    steeringdPidResult = steeringPidCompute();
  } else steeringdPidResult = false;

  // calculate speedPid
  leftSpeedPidResult = leftSpeedPidCompute();
  rightSpeedPidResult = rightSpeedPidCompute();

  if(leftSpeedPidSetPoint && (leftSpeedPidResult || rightSpeedPidResult)) {
    ////Serial.print(useSteeringPid);
    ////Serial.print("\t"); Serial.print(steeringPidInputLast); Serial.print("\t");
    //Serial.print(encoderLeftPulsesTarget); Serial.print("\t"); Serial.print(encoderRightPulsesTarget); Serial.print("\t");
    //Serial.print(encoderLeftPulses - encoderLeftPulsesTargetStart); Serial.print("\t"); Serial.print(encoderRightPulses - encoderRightPulsesTargetStart); Serial.print("\t");
//    Serial.print(leftSpeedPidSetPoint); Serial.print("\t"); //Serial.print(rightSpeedPidSetPoint); Serial.print("\t");
//    Serial.print(leftSpeedPidInputLast - leftSpeedPidSetPoint); Serial.print("\t");
//    Serial.print(rightSpeedPidInputLast - rightSpeedPidSetPoint); Serial.print("\t");
//    Serial.print(leftSpeedPidOutput); Serial.print("\t"); Serial.print(rightSpeedPidOutput); Serial.print("\t");
    //Serial.println("\t");

	    char buf[128];
	    int n = sprintf(buf, "usp= %d \t %d psp= %d %d %d %d po=%d %d %d %d pws=%d\r\n",
	    		(int)useSteeringPid,
				steeringPidInputLast,
				(int)leftSpeedPidSetPoint,
				(int)rightSpeedPidSetPoint,
				leftSpeedPidInputLast - (int)leftSpeedPidSetPoint,
				rightSpeedPidInputLast - (int)rightSpeedPidSetPoint,
				(int)leftSpeedPidOutput,
				(int)rightSpeedPidOutput,
				leftMotorPwmOut,
				rightMotorPwmOut,
				(int)PWMdirect
				);
	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 50);
  }
}

bool leftSpeedPidCompute() {
  bool pidResult;

  if(useSteeringPid) {
    leftSpeedPidInput = abs(encoderLeftPulsesSpeedPID) + (abs(encoderLeftPulsesSpeedPID) * steeringPidOutput / 100);
  } else {
    leftSpeedPidInput = abs(encoderLeftPulsesSpeedPID);
  }

  // adaptative code here. see old file

  pidResult = PID_Compute(&leftSpeedPid);
  if(pidResult) {
	encoderLeftSpeedZeroRef  = encoderLeftPulsesRead;
    encoderLeftPulsesSpeedPID = 0;
    leftSpeedPidInputLast = leftSpeedPidInput;
  }
  return pidResult;
}

bool rightSpeedPidCompute() {
  bool pidResult;

  if(useSteeringPid) {
    rightSpeedPidInput = abs(encoderRightPulsesSpeedPID) - (abs(encoderRightPulsesSpeedPID) * steeringPidOutput / 100);
  } else {
    rightSpeedPidInput = abs(encoderRightPulsesSpeedPID);
  }

  // adaptative code here. see old file

  pidResult = PID_Compute(&rightSpeedPid);
  if(pidResult) {
	encoderRightSpeedZeroRef = encoderRightPulsesRead;
    encoderRightPulsesSpeedPID = 0;
    rightSpeedPidInputLast = rightSpeedPidInput;
  }
  return pidResult;
}

bool steeringPidCompute() {
  bool pidResult;

  // calculate pwm wheel diference correction using pid
  steeringPidInput = abs(encoderLeftPulsesSteeringPID) - abs(encoderRightPulsesSteeringPID);

  // adaptative code here. see old file

  pidResult = PID_Compute(&steeringPid);
  if(pidResult) {
    steeringPidInputLast = steeringPidInput;
  }
  return pidResult;
}


static inline void setBodyMotorLeftForward(void){
  HAL_GPIO_WritePin(leftMotorForward, GPIO_PIN_SET);
  HAL_GPIO_WritePin(leftMotorBackward, GPIO_PIN_RESET);
}

static inline void setBodyMotorLeftBackward(void){
  HAL_GPIO_WritePin(leftMotorForward, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(leftMotorBackward, GPIO_PIN_SET);
}

static inline void setBodyMotorLeftStop(void){
  HAL_GPIO_WritePin(leftMotorForward, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(leftMotorBackward, GPIO_PIN_RESET);
}

static inline void setBodyMotorLeftBrake() {
  HAL_GPIO_WritePin(leftMotorForward, GPIO_PIN_SET);
  HAL_GPIO_WritePin(leftMotorBackward, GPIO_PIN_SET);
}

static inline void setBodyMotorRightForward(void){
  HAL_GPIO_WritePin(rightMotorForward, GPIO_PIN_SET);
  HAL_GPIO_WritePin(rightMotorBackward, GPIO_PIN_RESET);
}

static inline void setBodyMotorRightBackward(void){
  HAL_GPIO_WritePin(rightMotorForward, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(rightMotorBackward, GPIO_PIN_SET);
}

static inline void setBodyMotorRightStop(void){
  HAL_GPIO_WritePin(rightMotorForward, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(rightMotorBackward, GPIO_PIN_RESET);
}

static inline void setBodyMotorRightBrake() {
  HAL_GPIO_WritePin(rightMotorForward, GPIO_PIN_SET);
  HAL_GPIO_WritePin(rightMotorBackward, GPIO_PIN_SET);
}

/*
void apply_motors(int16_t left, int16_t right)
{
  // LEFT
  if (left == 0) {
    left_stop();
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
  } else if (left > 0) {
    left_forward();
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (left > PWM_MAX ? PWM_MAX : left));
  } else {
    left_backward();
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, ((-left) > PWM_MAX ? PWM_MAX : -left));
  }

  // RIGHT
  if (right == 0) {
    right_stop();
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
  } else if (right > 0) {
    right_forward();
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (right > PWM_MAX ? PWM_MAX : right));
  } else {
    right_backward();
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, ((-right) > PWM_MAX ? PWM_MAX : -right));
  }
}
*/

void pwmWrite(uint8_t tim, uint8_t ch, int16_t pwm) {

	switch (tim) {
	case 5:
		switch (ch) {
		case 1:
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm);
			break;
		case 2:
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm);
			break;
		}
		break;
	// add other tim's cases
	}

}

void bodyMotorsControl(void) {
  // brake control vars
  uint8_t leftBrake = false;
  uint8_t rightBrake = false;

  // lógica de controlo do brake
  // TODO

  // set motor left direction & speed
  if(leftMotorPwmOut == 0) {

    if(leftBrake) {
      setBodyMotorLeftBrake();
      //pwmWrite(leftMotorEn,64000);
      //__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, PWM_MAX);
      pwmWrite(leftMotorEnENxTIM, leftMotorEnENxCH, PWM_MAX);
    } else {
      setBodyMotorLeftStop();
      //pwmWrite(leftMotorEn, 0);
      //__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
      pwmWrite(leftMotorEnENxTIM, leftMotorEnENxCH, 0);
    }

  } else if(leftMotorPwmOut > 0) {
    setBodyMotorLeftForward();
    //pwmWrite(leftMotorEn, leftMotorPwmOut);
    //__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (leftMotorPwmOut > PWM_MAX ? PWM_MAX : leftMotorPwmOut));
    pwmWrite(leftMotorEnENxTIM, leftMotorEnENxCH, (leftMotorPwmOut > PWM_MAX ? PWM_MAX : leftMotorPwmOut));

  } else {
    setBodyMotorLeftBackward();
    //pwmWrite(leftMotorEn, abs(leftMotorPwmOut));
    //__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (abs(leftMotorPwmOut) > PWM_MAX ? PWM_MAX : abs(leftMotorPwmOut)));
    pwmWrite(leftMotorEnENxTIM, leftMotorEnENxCH, (abs(leftMotorPwmOut) > PWM_MAX ? PWM_MAX : abs(leftMotorPwmOut)));
  }

  // set motor right direction & speed
  if(rightMotorPwmOut == 0) {

    if(rightBrake) {
      setBodyMotorRightBrake();
      //pwmWrite(rightMotorEn, 64000);
      //__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, PWM_MAX);
      pwmWrite(rightMotorEnENxTIM, rightMotorEnENxCH, PWM_MAX);
    } else {
      setBodyMotorRightStop();
      //pwmWrite(rightMotorEn, 0);
      //__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
      pwmWrite(rightMotorEnENxTIM, rightMotorEnENxCH, 0);
    }

  } else if(rightMotorPwmOut > 0) {
    setBodyMotorRightForward();
    //pwmWrite(rightMotorEn, rightMotorPwmOut);
    //__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (rightMotorPwmOut > PWM_MAX ? PWM_MAX : rightMotorPwmOut));
    pwmWrite(rightMotorEnENxTIM, rightMotorEnENxCH, (rightMotorPwmOut > PWM_MAX ? PWM_MAX : rightMotorPwmOut));
  } else {
    setBodyMotorRightBackward();
    //pwmWrite(rightMotorEn, abs(rightMotorPwmOut));
    //__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (abs(rightMotorPwmOut) > PWM_MAX ? PWM_MAX : abs(rightMotorPwmOut)));
    pwmWrite(rightMotorEnENxTIM, rightMotorEnENxCH, (abs(rightMotorPwmOut) > PWM_MAX ? PWM_MAX : abs(rightMotorPwmOut)));
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

  int16_t encoderPulsesTarget = 0;

  uint8_t cmd = i2c_rx_buf[0];

  //int16_t *data = (int16_t*)&i2c_rx_buf[0];

  switch (cmd)
  {
    case 1: // CMD_RESET_ENCODERS
	  HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
	  HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);

      __HAL_TIM_SET_COUNTER(&htim2, 0);
      __HAL_TIM_SET_COUNTER(&htim3, 0);

      HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
      HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

      encoderLeftPulses = 0;
      encoderRightPulses = 0;
      break;

    case 2: // CMD_SET_PWM
      // com inversão de bytes no pi
      leftMotorPwmOutCmd  = ((int16_t*)i2c_rx_buf)[1];
      rightMotorPwmOutCmd = ((int16_t*)i2c_rx_buf)[2];
      // sem inversão dos bytesno pi //
      // buf[1] | (buf[2]<<8)
      //leftSpeedPidSetPoint  = (int16_t)(i2c_rx_buf[1] | (i2c_rx_buf[2] << 8));
      //rightSpeedPidSetPoint = (int16_t)(i2c_rx_buf[3] | (i2c_rx_buf[4] << 8));
      PWMdirect = true;
      break;

    case 3: // CMD_LIDAR_MOTOR_ON
        //Serial.println("set lidar on");
        // lidar motor on
        //lidarMotorPower = HIGH;
        //digitalWrite(LIDAR_MOTOR_PIN, lidarMotorPower);

    	//printf("XXXXXXXXXX: %d\r\n", encoderLeftPulses);
      break;

    case 4: // CMD_LIDAR_MOTOR_OFF
        // lidar motor off
        //Serial.println("set lidar off");
        //lidarMotorPower = LOW;
        //digitalWrite(LIDAR_MOTOR_PIN, lidarMotorPower);
      break;

    case 5: // CMD_SET_MOTION
    case 6: // CMD_SET_VEL

    	PWMdirect = false;

      //leftSpeedPidSetPoint  = ((int16_t*)i2c_rx_buf)[1];
      //rightSpeedPidSetPoint = ((int16_t*)i2c_rx_buf)[2];

      leftSpeedPidSetPointTmp  = ((int16_t*)i2c_rx_buf)[1];
      rightSpeedPidSetPointTmp = ((int16_t*)i2c_rx_buf)[2];

      if(cmd == CMD_SET_MOTION) {
        encoderPulsesTarget = ((int16_t*)i2c_rx_buf)[3];
      } else {
        encoderPulsesTarget = 0;
      }

      //Serial.print(leftSpeedPidSetPointTmp); Serial.print("\t"); Serial.print(rightSpeedPidSetPointTmp); Serial.print("\t"); Serial.print(encoderPulsesTarget); Serial.println();

      // set left motors direction and speed
      if(leftSpeedPidSetPointTmp > 0) {
        leftSpeedPidSetPointDirection = 1;
        leftSpeedPidSetPoint = round(leftSpeedPidSetPointTmp);
      } else if(leftSpeedPidSetPointTmp < 0) {
        leftSpeedPidSetPointDirection = -1;
        leftSpeedPidSetPoint = round(abs(leftSpeedPidSetPointTmp));
      } else {
        leftSpeedPidSetPointDirection = 0;
        leftSpeedPidSetPoint = round(leftSpeedPidSetPointTmp);
      }

      // set right motors direction and speed
      if(rightSpeedPidSetPointTmp > 0) {
        rightSpeedPidSetPointDirection = 1;
        rightSpeedPidSetPoint = round(rightSpeedPidSetPointTmp);
      } else if(rightSpeedPidSetPointTmp < 0) {
        rightSpeedPidSetPointDirection = -1;
        rightSpeedPidSetPoint = round(abs(rightSpeedPidSetPointTmp));
      } else {
        rightSpeedPidSetPointDirection = 0;
        rightSpeedPidSetPoint = 0;
      }

      //Serial.print(leftSpeedPidSetPointDirection); Serial.print("\t"); Serial.print(rightSpeedPidSetPointDirection); Serial.print("\t"); Serial.print(encoderPulsesTarget); Serial.println();

      PID_SetMode(&leftSpeedPid, AUTOMATIC);
      PID_SetMode(&rightSpeedPid, AUTOMATIC);

      // switch on/off the steering pid
      if(leftSpeedPidSetPoint == rightSpeedPidSetPoint && leftSpeedPidSetPoint != 0) {
        // rotate or linear translation
    	//PID_SetMode(&steeringPid, AUTOMATIC);
        useSteeringPid = true;
        encoderLeftSteeringZeroRef  = encoderLeftPulsesRead;
        encoderLeftPulsesSteeringPID = 0;
        encoderRightSteeringZeroRef = encoderRightPulsesRead;
        encoderRightPulsesSteeringPID = 0;
      } else {
        // curve
    	//PID_SetMode(&steeringPid, MANUAL);
        useSteeringPid = false;
        encoderLeftSteeringZeroRef  = encoderLeftPulsesRead;
        encoderLeftPulsesSteeringPID = 0;
        encoderRightSteeringZeroRef = encoderRightPulsesRead;
        encoderRightPulsesSteeringPID = 0;
      }

      // set the target for encoders if any
      if(encoderPulsesTarget) {
    	char str[64];
    	const char *buf =  "TODO: set the target for encoders if any ";
		int n = sprintf(str, "%s\r\n",buf);
		HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 50);
    	// TODO : check old file
      }

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
