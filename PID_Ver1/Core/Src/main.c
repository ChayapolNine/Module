/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "math.h"
#include "ModBusRTU.h"
#include "arm_math.h"
#include "main.h"
#include "Qubic.h"
#include "Qubic_emxAPI.h"
#include "Qubic_terminate.h"
#include "Qubic_types.h"
#include "rt_nonfinite.h"

//Chayapol 6411
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint64_t micros();
void flowmodbus();
void velocity();
static double argInit_real_T(void);

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void) {
	return 0.0;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
//I2C
int LOL;
int laser_flag,laser_on_off,laser_power;
int Em_state;
int score;
int homeflag = 0;
float posy_point;
uint64_t timestampI2Cdone;
int set_home = 0;
int I2Cdone;
int done;
int CaseTray = 0;
uint64_t abc = 0;
int plustray = -1;
int gentest = 0;
int choice;
uint8_t data_read;
uint8_t starttray;
GPIO_PinState lastButtonState;
GPIO_PinState buttonState;
float start_p, stop_p, start_v, stop_v, timecycle;
int posx;
int orenationtray = 0;
float point_x[18] = {100,-100,100,-100,100,-100,100,-100,100,-100,100,-100,100,-100,100,-100,100,-100,};
float point_y[18] = { 100, -110, 190, -100, 210, -80, 170, -120, 180, -110, 200,
		-90, 160, -130, 170, -120, 190, -100};
float rectangle[5][2] = { { 0, 0 }, { 60, 0 }, { 60, 50 }, { 0, 50 }, { 0, 0 } };

// Define the rotation matrix
float T_rotation[2][2];

// Declare and initialize the translation array
float translation[2];

float T[3][3];
float points[9][2] = { { 10, 40 }, { 30, 40 }, { 50, 40 }, { 10, 25 },
		{ 30, 25 }, { 50, 25 }, { 10, 10 }, { 30, 10 }, { 50, 10 } };

float homogeneousRectangle[5][3];
float transformedRectangle[5][3];
// Transform the points
float homogeneousPoints[9][3];
float transformedPoints[9][3];

//find theta
float dot_product;
float vectorsize = 3600.0;
float theta;
float in_theta;

float bottom_left_jog[2];
float bottom_right_jog[2];

float rectangle2[5][2] =
		{ { 0, 0 }, { 60, 0 }, { 60, 50 }, { 0, 50 }, { 0, 0 } };

// Define the rotation matrix
float T_rotation2[2][2];

double q_k1_tmp;
// Declare and initialize the translation array
float translation2[2];

float T2[3][3];
float points2[9][2] = { { 10, 40 }, { 30, 40 }, { 50, 40 }, { 10, 25 },
		{ 30, 25 }, { 50, 25 }, { 10, 10 }, { 30, 10 }, { 50, 10 } };

float homogeneousRectangle2[5][3];
float transformedRectangle2[5][3];
// Transform the points
float homogeneousPoints2[9][3];
float transformedPoints2[9][3];

//find theta
float dot_product2;
float vectorsize2 = 3600.0;
float theta2;
float in_theta2;

float bottom_left_jog2[2];
float bottom_right_jog2[2];
//Modbus Protocol
typedef enum {
	Initial, Jogging_Place, Jogging_Pick, Home, Run_PointMode, Run_TrayMode ,EM_Run_PointMode,EM_Run_TrayMode
} state;
state Mobus = 0;
ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[70];
int x_done, y_done;

int SoftReset = 0;
float speed;
float acceleration;
uint32_T path = 0;
uint32_t QEIReadRaw;
uint32_t Button1 = 0;
float positionTraject;
float velocityTraject;

//Jogging
uint32_t Joystick_AND_Sensor[4];
uint32_t Joystick_Control = 1;
uint8_t CheckTray;
int JoystickSpeed = 16383;

typedef struct _QEIStructure {
	uint32_t data[2]; //Position data container
	uint32_t timestamp[2];
	float QEIPosition; //step
	float QEIVelocity;
//step/sec
} QEIStructureTypedef;
QEIStructureTypedef QEIData = { 0 };
QEIStructureTypedef QEIAcc = { 0 };

uint64_t _micros = 0;

emxArray_real_T *q_velocityN;
emxArray_real_T *q_positionN;
emxArray_real_T *q_accN;
int indexposition = 0;
float timestep = 0;

// PID variable

float pu1 = 0;
float pe1 = 0;
float pe2 = 0;
float u;
float p = 0;
float s = 0;
float u2;
float p2 = 0;
float s2 = 0;
float error2 = 0;
float delta_u;
float K_P = 0; //1
float K_I = 0.0500000007; // 0.1
float K_D = 0; //0
float K_Pvelo = 250; // 200
float K_Ivelo = 0.100000001; // 0.5
float K_Dvelo = 0; // 0

float SetVelocity = 500;
float ReadDegree; // Encoder value
float SetDegree; // Set point
float DegreeFeedback; // Feedback position
float error; // error

int testgo = 0;
uint64_t timeI2C = 0;
uint64_t timestamp = 0;
uint64_t timemodbus = 0;
uint64_t timestampTrajact = 0;
uint64_t timestamptray = 2000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void I2C_all();
void I2C_read_status();
void transformRectangleAndPointsPlace();
void transformRectangleAndPointsPick();
float control_interrupt();
float control_velocity();
void accelerate();
void main_Qubic();
void home_yaxis();
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM11_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	start_p = 0;
	stop_p = 15;
	start_v = 0;
	stop_v = 0;
	timecycle = 2;
	transformRectangleAndPointsPlace();
	HAL_ADC_Start_DMA(&hadc1, Joystick_AND_Sensor, 4);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim5);
	hmodbus.huart = &huart2;
	hmodbus.htim = &htim11;
	hmodbus.slaveAddress = 0x15;
	hmodbus.RegisterSize = 70;
	Modbus_init(&hmodbus, registerFrame);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (starttray == 1) {
			starttray = 0;
			transformRectangleAndPointsPlace();
		}
		Modbus_Protocal_Worker();
		flowmodbus();
		limitsensor();

		if (SoftReset == 1) {
			NVIC_SystemReset();
			SoftReset = 0;
		}
//		static uint32_t timestamp_I2C = 0;
//		      if(HAL_GetTick() >= timestamp_I2C){
//		          I2C_all(&data_read);
//		          timestamp_I2C = HAL_GetTick()+ 20;
//		      }

		if (HAL_GetTick() >= timemodbus) { // heartbeat
			timemodbus = HAL_GetTick() + 0.5;
			registerFrame[0].U16 = 22881;
			registerFrame[17].U16 = (int) (ReadDegree - 350) * 10;
			registerFrame[18].U16 = abs(speed);
			registerFrame[19].U16 = acceleration;
		}
		if (HAL_GetTick() >= timestampTrajact) {
			timestampTrajact = HAL_GetTick() + 10;
			if (indexposition < (timecycle * 100) && path == 1) {
				positionTraject = q_positionN->data[indexposition];
				velocityTraject = q_velocityN->data[indexposition];
				SetDegree = positionTraject;
				SetVelocity = velocityTraject;
				indexposition += 1;
			}
		}
		if(testgo == 1)
		{
			Mobus = Run_TrayMode;
			testgo = 0;
		}
		if (HAL_GetTick() >= timestamp) {
			timestamp = HAL_GetTick() + 5;
			//abc++;
			QEIReadRaw = __HAL_TIM_GET_COUNTER(&htim2); // Read QEI
			ReadDegree = (QEIReadRaw / 8192.0 * 360.0) * 160.0 / 360.0; // pulse to degree
			error = SetDegree - ReadDegree;
			velocity();
			accelerate();
			speed = ((QEIData.QEIVelocity / 8192.0) * 360.0) * 160 / 360;
			acceleration = QEIAcc.QEIVelocity;
			DegreeFeedback = control_interrupt(); // PID function

			if (velocityTraject == 0) {
				s2 = 0;
			}
			if (Joystick_Control == 1) {
				DegreeFeedback = 0;
				s = 0;
				s2 = 0;
				error = 0;
				error2 = 0;
				if (Joystick_AND_Sensor[0] >= 3150) {
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 16383); // 20 %
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
				} else if (Joystick_AND_Sensor[0] <= 100) {
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 16383); // 20 %
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
				} else {
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
				}
			} else if (Joystick_Control == 0) {
				if (SetDegree < 0) {
					SetDegree = 0; // minimum value
				}
				if (SetDegree > 700) {
					SetDegree = 700; // maximum value
				}

				if (error > 0) { // setpoint > read_encoder

					if (error < 0.2) {
						DegreeFeedback = 0; // Limit Position
						s = 0;
					}
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, DegreeFeedback);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
				}
				if (error < 0) { // setpoint < read_encoder
					if (error * -1 < 0.2) {
						DegreeFeedback = 0; // Limit Position
						s = 0;
					}
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, DegreeFeedback * -1);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
				}
			}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
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
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = QEI_PERIOD-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 2005;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim11, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 1433;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, DIR_Pin|Em_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Sensor_Home_Pin Sensor_1_Pin Set_Tray_Pin Clear_Tray_Pin */
  GPIO_InitStruct.Pin = Sensor_Home_Pin|Sensor_1_Pin|Set_Tray_Pin|Clear_Tray_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin Em_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|Em_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Sensor_2_Pin */
  GPIO_InitStruct.Pin = Sensor_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Sensor_2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void runQubicMultipleTimes(int numIterations) {
	double q_k1 = 0.0;
	double q_k2 = 1.0;
	double qdot_k1 = 0.0;
	double qdot_k2 = 0.0;
	double tf = 1.0;

	emxArray_real_T *q_position;
	emxArray_real_T *q_velocity;
	emxArray_real_T *q_acc;

	// Initialize emxArray_real_T instances
	q_position = emxCreate_real_T(1, 1);
	q_velocity = emxCreate_real_T(1, 1);
	q_acc = emxCreate_real_T(1, 1);

	// Run the Qubic function multiple times
	for (int i = 0; i < numIterations; i++) {
		// Call the Qubic function
		Qubic(q_k1, q_k2, qdot_k1, qdot_k2, tf, q_position, q_velocity, q_acc);

		// Access the calculated values
		double *q_position_data = q_position->data;
		double *q_velocity_data = q_velocity->data;
		double *q_acc_data = q_acc->data;

		// Update the input values for the next iteration
		q_k1 = *q_position_data;
		qdot_k1 = *q_velocity_data;

	}

	// Free the memory
	emxDestroyArray_real_T(q_position);
	emxDestroyArray_real_T(q_velocity);
	emxDestroyArray_real_T(q_acc);
}
void I2C_all(uint8_t * Rdata) {
	uint8_t data_1[1];
	uint8_t data_2[2];
	uint8_t data_4[4];

    switch (choice) {
        // I2C_testmode_on
        case 1:
            data_2[0] = 0x01;
            data_2[1] = 0x01;
            HAL_I2C_Master_Transmit(&hi2c1, 0x15 << 1, data_2, 2, 50);
            break;
        // I2C_testmode_off
        case 2:
            data_2[0] = 0x01;
            data_2[1] = 0x00;
            HAL_I2C_Master_Transmit(&hi2c1, 0x15 << 1, data_2, 2, 50);
            break;
        // I2C_soft_reset
        case 3:
            data_4[0] = 0x00;
            data_4[1] = 0xFF;
            data_4[2] = 0x55;
            data_4[3] = 0xAA;
            HAL_I2C_Master_Transmit(&hi2c1, 0x15 << 1, data_4, 4, 50);
            break;
        // I2C_open_emergency
        case 4:
            data_1[0] = 0xF0;
            HAL_I2C_Master_Transmit(&hi2c1, 0x15 << 1, data_1, 1, 50);
            break;
        // I2C_close_emergency
        case 5:
            data_4[0] = 0xE5;
            data_4[1] = 0x7A;
            data_4[2] = 0xFF;
            data_4[3] = 0x81;
            HAL_I2C_Master_Transmit(&hi2c1, 0x15 << 1, data_4, 4, 50);
            break;
        // I2C_gripper_runmode_on
        case 6:
            data_2[0] = 0x10;
            data_2[1] = 0x13;
            HAL_I2C_Master_Transmit(&hi2c1, 0x15 << 1, data_2, 2, 50);
            break;
        // I2C_gripper_runmode_off
        case 7:
            data_2[0] = 0x10;
            data_2[1] = 0x8C;
            HAL_I2C_Master_Transmit(&hi2c1, 0x15 << 1, data_2, 2, 50);
            break;
        // I2C_gripper_pick
        case 8:
            data_2[0] = 0x10;
            data_2[1] = 0x5A;
            HAL_I2C_Master_Transmit(&hi2c1, 0x15 << 1, data_2, 2, 50);
            break;
        // I2C_gripper_place
        case 9:
            data_2[0] = 0x10;
            data_2[1] = 0x69;
            HAL_I2C_Master_Transmit(&hi2c1, 0x15 << 1, data_2, 2, 50);
            break;
        // I2C_read_status
        case 10:
        	HAL_I2C_Master_Receive(&hi2c1, 0x15 << 1, Rdata, 1, 50);
        	break;
    }
}
void limitsensor() {
	if (Joystick_AND_Sensor[2] > 3000)
	{
		LOL++;
		s=0;
		s2=0;
		TIM2->CNT = 0;
	}
	else if (Joystick_AND_Sensor[3] > 3000)
		{
		s=0;
		s2=0;
	}

}
void transformRectangleAndPointsPick() {

	translation[0] = bottom_left_jog[0];
	translation[1] = bottom_left_jog[1];

	bottom_right_jog[0] = bottom_right_jog[0] - translation[0];
	bottom_right_jog[1] = bottom_right_jog[1] - translation[1];

	dot_product = 60 * bottom_right_jog[0] + bottom_right_jog[1] * 0;
	in_theta = dot_product / vectorsize;
	theta = -acos(in_theta);

	T_rotation[0][0] = cos(theta);
	T_rotation[0][1] = -sin(theta);
	T_rotation[1][0] = sin(theta);
	T_rotation[1][1] = cos(theta);

	T[0][0] = T_rotation[0][0];
	T[0][1] = T_rotation[0][1];
	T[0][2] = translation[0];
	T[1][0] = T_rotation[1][0];
	T[1][1] = T_rotation[1][1];
	T[1][2] = translation[1];
	T[2][0] = 0;
	T[2][1] = 0;
	T[2][2] = 1;

	// Transform the rectangle
	for (int i = 0; i < 5; i++) {
		homogeneousRectangle[i][0] = rectangle[i][0];
		homogeneousRectangle[i][1] = rectangle[i][1];
		homogeneousRectangle[i][2] = 1;
	}

	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 3; j++) {
			transformedRectangle[i][j] = 0;
			for (int k = 0; k < 3; k++) {
				transformedRectangle[i][j] += homogeneousRectangle[i][k]
						* T[k][j];
			}
		}
	}

	// Transform the points
	for (int i = 0; i < 9; i++) {
		homogeneousPoints[i][0] = points[i][0];
		homogeneousPoints[i][1] = points[i][1];
		homogeneousPoints[i][2] = 1;
	}

	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 3; j++) {
			transformedPoints[i][j] = 0;
			for (int k = 0; k < 3; k++) {

				transformedPoints[i][j] += homogeneousPoints[i][k] * T[k][j];
			}
		}
	}

	// Translation points
	for (int i = 0; i < 9; i++) {
		transformedPoints[i][0] = transformedPoints[i][0] + translation[0];
		transformedPoints[i][1] = transformedPoints[i][1] + translation[1];
		//transformedPoints[i][0] = transformedPoints[i][0] *(-1);

	}

	// Translation rectangle
	for (int i = 0; i < 5; i++) {
		transformedRectangle[i][0] = transformedRectangle[i][0]
				+ +translation[0];
		transformedRectangle[i][1] = transformedRectangle[i][1]
				+ translation[1];
	}

}
void transformRectangleAndPointsPlace() {

	translation2[0] = bottom_left_jog2[0];
	translation2[1] = bottom_left_jog2[1];

	bottom_right_jog2[0] = bottom_right_jog2[0] - translation2[0];
	bottom_right_jog2[1] = bottom_right_jog2[1] - translation2[1];

	dot_product2 = 60 * bottom_right_jog2[0] + bottom_right_jog2[1] * 0;
	in_theta2 = dot_product2 / vectorsize2;
	theta2 = -acos(in_theta2);

	T_rotation2[0][0] = cos(theta2);
	T_rotation2[0][1] = -sin(theta2);
	T_rotation2[1][0] = sin(theta2);
	T_rotation2[1][1] = cos(theta2);

	T2[0][0] = T_rotation2[0][0];
	T2[0][1] = T_rotation2[0][1];
	T2[0][2] = translation2[0];
	T2[1][0] = T_rotation2[1][0];
	T2[1][1] = T_rotation2[1][1];
	T2[1][2] = translation2[1];
	T2[2][0] = 0;
	T2[2][1] = 0;
	T2[2][2] = 1;

	// Transform the rectangle
	for (int i = 0; i < 5; i++) {
		homogeneousRectangle2[i][0] = rectangle2[i][0];
		homogeneousRectangle2[i][1] = rectangle2[i][1];
		homogeneousRectangle2[i][2] = 1;
	}

	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 3; j++) {
			transformedRectangle2[i][j] = 0;
			for (int k = 0; k < 3; k++) {
				transformedRectangle2[i][j] += homogeneousRectangle2[i][k]
						* T2[k][j];
			}
		}
	}

	// Transform the points
	for (int i = 0; i < 9; i++) {
		homogeneousPoints2[i][0] = points2[i][0];
		homogeneousPoints2[i][1] = points2[i][1];
		homogeneousPoints2[i][2] = 1;
	}

	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 3; j++) {
			transformedPoints2[i][j] = 0;
			for (int k = 0; k < 3; k++) {

				transformedPoints2[i][j] += homogeneousPoints2[i][k] * T2[k][j];
			}
		}
	}

	// Translation points
	for (int i = 0; i < 9; i++) {
		transformedPoints2[i][0] = transformedPoints2[i][0] + translation2[0];
		transformedPoints2[i][1] = transformedPoints2[i][1] + translation2[1];
		//transformedPoints2[i][0] = transformedPoints2[i][0] *(-1);

	}

	// Translation rectangle
	for (int i = 0; i < 5; i++) {
		transformedRectangle2[i][0] = transformedRectangle2[i][0]
				+ +translation2[0];
		transformedRectangle2[i][1] = transformedRectangle2[i][1]
				+ translation2[1];
	}

}
void home_yaxis(){
	if(homeflag == 0){
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 16383); // 20 %
		registerFrame[16].U16 = 4;
		path = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	}
	if (Joystick_AND_Sensor[2] > 3000)
	{
		registerFrame[16].U16 = 0;
		Joystick_Control = 0;
		homeflag = 1;
		s=0;
		s2=0;
		LOL++;
		TIM2->CNT = 0;
		start_p = 0;
		stop_p = 350;
		start_v = 0; // qk
		stop_v = 0; // q_dotk+1
		timecycle = 1.5;
		main_Qubic();
		path = 1;
		Mobus = Initial;
	}
}
void flowmodbus() {
	switch (Mobus) {
	case Initial:
		if(registerFrame[2].U16 == 0b0001 && laser_on_off == 0){ // Laser On
			laser_flag = 1;
			if(laser_flag == 1){
			laser_on_off = 1;
			choice = 1;
			I2C_all(&data_read);
			laser_flag = 0;
			registerFrame[2].U16 = 0;
			}
		}
		if(registerFrame[2].U16 == 0b0001 && laser_on_off == 1){ // Laser Off
			laser_flag = 1;
			if(laser_flag == 1){
			laser_on_off = 0;
			choice = 2;
			I2C_all(&data_read);
			laser_flag = 0;
			registerFrame[2].U16 = 0;
			}
		}
		if(registerFrame[2].U16 == 0b0010 && laser_power == 0){ // Laser Power
			laser_flag = 1;
			if(laser_flag == 1){
			laser_power = 1;
			choice = 6;
			I2C_all(&data_read);
			laser_flag = 0;
			registerFrame[2].U16 = 0;
			}
		}
		if(registerFrame[2].U16 == 0b0010 && laser_power == 1){ // Laser Power off
			laser_flag = 1;
			if(laser_flag == 1){
			laser_power = 0;
			choice = 6;
			I2C_all(&data_read);
			laser_flag = 0;
			registerFrame[2].U16 = 0;
			}
		}
		if(registerFrame[2].U16 == 0b0100){ // Laser Power off
			laser_flag = 1;
			if(laser_flag == 1){
			choice = 8;
			I2C_all(&data_read);
			laser_flag = 0;
			registerFrame[2].U16 = 0;
			}
		}
		if(registerFrame[2].U16 == 0b1000){ // Laser Power off
			laser_flag = 1;
			if(laser_flag == 1){
			choice = 9;
			I2C_all(&data_read);
			laser_flag = 0;
			registerFrame[2].U16 = 0;
			}
		}
		if (registerFrame[1].U16 == 0b00010) { // Set Place
			registerFrame[1].U16 = 0; // 0x01 base system reset place tray
			registerFrame[16].U16 = 2; // 0x10 y-axis Set Place
			Joystick_Control = 1;
			choice = 1;
			I2C_all(&data_read);
			Mobus = Jogging_Place;
		} else if (registerFrame[1].U16 == 0b00001) { //Set Pick
			registerFrame[1].U16 = 0; // 0x01 base system reset place tray
			registerFrame[16].U16 = 1; // 0x10 y-axis Set Pick
			Joystick_Control = 1;
			choice = 1;
			I2C_all(&data_read);
			//I2C_all();
			Mobus = Jogging_Pick;
		} else if (registerFrame[1].U16 == 0b10000) { // Run point Mode
			registerFrame[1].U16 = 0; // base system run point mode reset
			registerFrame[16].U16 = 32; // y-axis moving status go point x
			Joystick_Control = 0;
			indexposition = 0;
			if (registerFrame[49].U16 > 60000)
				posy_point = -((UINT16_MAX - registerFrame[49].U16) / 10.0);
			else if (registerFrame[49].U16 <= 3500) {
				posy_point = (registerFrame[49].U16 / 10.0);
			}
			start_p = ReadDegree;
			stop_p = posy_point+350.0;
			start_v = 0; // qk
			stop_v = 0; // q_dotk+1
			timecycle = 1.5;
			main_Qubic();
			path = 1;
			Mobus = Run_PointMode;
		} else if (registerFrame[1].U16 == 0b00100) { // Set Home
			registerFrame[1].U16 = 0;
			homeflag = 0;
			path = 0;
			indexposition = 0;
			Mobus = Home;
		} else if (registerFrame[1].U16 == 0b01000) {
			Joystick_Control = 0;
			plustray = -1;
			CaseTray = 0;
			registerFrame[1].U16 = 0;
			int i, j, k, b;
			k = 0;
			b = 0;
			for (i = 0; i < 9; i++) {
				point_y[k++] = transformedPoints[i][1];
				point_y[k++] = transformedPoints2[i][1];
			}
			for (j = 0; j < 9; j++) {
				point_x[b++] = transformedPoints[j][0];
				point_x[b++] = transformedPoints2[j][0];
			}
			Mobus = Run_TrayMode;
		}
		break;
	case Jogging_Place:
		//y-axis jogging
		if (Joystick_AND_Sensor[1] >= 3150) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, JoystickSpeed);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		} else if (Joystick_AND_Sensor[1] <= 100) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, JoystickSpeed);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		} else {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		//x-axis jogging
		if (Joystick_AND_Sensor[0] >= 3150) {
			registerFrame[64].U16 = 8;
		} else if (Joystick_AND_Sensor[0] <= 100) {
			registerFrame[64].U16 = 4;
		} else {
			registerFrame[64].U16 = 0;
		}

		// Set position
		buttonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
		if (buttonState != lastButtonState) {
			// Button press is valid, perform desired action
			if (CheckTray == 0) {
				if (registerFrame[68].U16 > 60000) {
					posx = registerFrame[68].U16 - UINT16_MAX - 1;
					bottom_left_jog2[0] = (float) (posx) / 10;
				} else {
					posx = registerFrame[68].U16;
					bottom_left_jog2[0] = (float) (posx) / 10;
				} // Calculate Point x-axis
				bottom_left_jog2[1] = (float) (ReadDegree - 350); // Calulate Point y-axis
				registerFrame[35].U16 = (int) posx; // Place Tray Origin x
				registerFrame[36].U16 = (int) (ReadDegree - 350) * 10; // Place Tray Origin y
				CheckTray++;
			} else if (CheckTray == 1) {
				if (registerFrame[68].U16 > 60000) {
					posx = registerFrame[68].U16 - UINT16_MAX - 1;
					bottom_right_jog2[0] = (float) (posx) / 10;
				} else {
					posx = registerFrame[68].U16;
					bottom_right_jog2[0] = (float) (posx) / 10;
				}
				posx = registerFrame[68].U16;
				bottom_right_jog2[0] = (float) (posx) / 10;
				bottom_right_jog2[1] = (float) (ReadDegree - 350); // Calculate Point y-axis
				CheckTray++;
			} else if (CheckTray == 2) {
				transformRectangleAndPointsPlace();
				orenationtray = abs((theta2) * 57.2958 * 100);
				registerFrame[37].U16 = orenationtray;
				registerFrame[16].U16 = 0; //0x10 y-status jogging fisnish reset to 0
				CheckTray = 0;
				choice = 2;
				I2C_all(&data_read);
				Mobus = Initial;
			}
		}

		// Update the previous state
		lastButtonState = buttonState;
		break;
	case Jogging_Pick:
		//y-axis jogging
		if (Joystick_AND_Sensor[1] >= 3150) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, JoystickSpeed);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		} else if (Joystick_AND_Sensor[1] <= 100) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, JoystickSpeed);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		} else {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		//x-axis jogging
		if (Joystick_AND_Sensor[0] >= 3150) {
			registerFrame[64].U16 = 8;
		} else if (Joystick_AND_Sensor[0] <= 100) {
			registerFrame[64].U16 = 4;
		} else {
			registerFrame[64].U16 = 0;
		}

		// Set position
		buttonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
		if (buttonState != lastButtonState) {
			// Button press is valid, perform desired action
			if (CheckTray == 0) {
				if (registerFrame[68].U16 > 60000) {
					posx = registerFrame[68].U16 - UINT16_MAX - 1;
					bottom_left_jog[0] = (float) (posx) / 10;
				} else {
					posx = registerFrame[68].U16;
					bottom_left_jog[0] = (float) (posx) / 10;
				}
				bottom_left_jog[1] = ((float) ReadDegree - 350); // Calulate Point y-axis
				registerFrame[32].U16 = (int) posx; // Place Tray Origin x
				registerFrame[33].U16 = (int) (ReadDegree - 350) * 10; // Place Tray Origin y
				CheckTray++;
			} else if (CheckTray == 1) {
				if (registerFrame[68].U16 > 60000) {
					posx = registerFrame[68].U16 - UINT16_MAX - 1;
					bottom_right_jog[0] = (float) (posx) / 10;
				} else {
					posx = registerFrame[68].U16;
					bottom_right_jog[0] = (float) (posx) / 10;
				}
				bottom_right_jog[1] = (float) (ReadDegree - (float) 350); // Calculate Point y-axis
				CheckTray++;
			} else if (CheckTray == 2) {
				transformRectangleAndPointsPick();
				orenationtray = abs((theta) * 57.2958 * 100);
				registerFrame[34].U16 = orenationtray;
				registerFrame[16].U16 = 0; // 0x10 y-status jogging finish reset to 0
				CheckTray = 0;
				choice = 2;
				I2C_all(&data_read);
				Mobus = Initial;
			}
		}
		// Update the previous state
		lastButtonState = buttonState;
		break;
	case Home:
		// x axis
		registerFrame[64].U16 = 1; // 0x40 Moving Status x-axis - Home
		home_yaxis();
		// y axis
		break;
	case Run_PointMode:
		// x axis
		registerFrame[65].U16 = registerFrame[48].U16; // position Tray pick/place
		registerFrame[66].U16 = 3000; // speed x-axis 300mm
		registerFrame[67].U16 = 1; // Acc time 1mms
		registerFrame[64].U16 = 2; //0x40 Moving Status x-axis - run mode
		// y axis
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 1){ // if Emergency Point mode on
			choice = 4;
			I2C_all(&data_read);
			path = 0;
			s = 0;
			s2 = 0;
			Mobus = EM_Run_PointMode;
		}
		if(abs(error) < 0.2 && indexposition == (timecycle*100)){
				Mobus = Initial;
				registerFrame[16].U16 = 0;
		}
		break;
	case EM_Run_PointMode:

		switch (Em_state) {
			case 0:
				path = 0;
				s = 0;
				s2 = 0;
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0){ // if Emergency Tray mode On
					path = 1;
					Em_state = 1;
					I2Cdone = 0;
				}
				break;
			case 1:
				if(I2Cdone == 0){
					timestampI2Cdone = HAL_GetTick() + 500;
					I2Cdone = 1;
				}
				if(HAL_GetTick() >= timestampI2Cdone){
					choice = 5;
					I2C_all(&data_read);
					if(I2Cdone == 1){
						I2Cdone = 0;
						Em_state = 0;
						Mobus = Run_PointMode;
					}
				}
				break;
		}
		break;
	case EM_Run_TrayMode:
		switch (Em_state) {
			case 0:
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0){ // if Emergency Tray mode On
					CaseTray = 0;
					plustray = -1;
					path = 0;
					data_read = 0;
					done = 0;
					I2Cdone = 0;
					Em_state = 1;
				}
				break;
			case 1:
				if(I2Cdone == 0){
					timestampI2Cdone = HAL_GetTick() + 500;
					I2Cdone = 1;
				}
				if(HAL_GetTick() >= timestampI2Cdone){
					choice = 5;
					I2C_all(&data_read);
					if(I2Cdone == 1){
						registerFrame[16].U16 = 0;
						I2Cdone = 0;
						Em_state = 0;
						Mobus = Initial;;
					}
				}
				break;
		}
		break;
	case Run_TrayMode:
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 1){ // if Emergency Tray mode On
			CaseTray = 0;
			plustray = -1;
			path = 0;
			data_read = 0;
			done = 0;
			I2Cdone = 0;
			choice = 4;
			I2C_all(&data_read);
			Mobus = EM_Run_TrayMode;
		}
		switch (CaseTray) {
			case 0:
				path = 1;
				indexposition = 0;
				choice = 6;
				I2C_all(&data_read);
				// y axis
				if(plustray == -1){ // home
					// y-axis
					start_p = 0+350.0;
					stop_p = point_y[plustray+1]+350.0;
					start_v = 0; // qk
					stop_v = 0; // q_dotk+1
					timecycle = 1.5;
					main_Qubic();
					// x axis
					if(registerFrame[64].U16 == 0){
					 registerFrame[65].U16 = (int16_t)(point_x[plustray+1]*10); // position Tray pick/place
					 registerFrame[66].U16 = 3000; // speed x-axis 300mm
					 registerFrame[67].U16 = 1; // Acc time 1mms
					 registerFrame[64].U16 = 2; //0x40 Moving Status x-axis - run mode
					}
					 CaseTray = 1;
				}
				else{
				start_p = point_y[plustray]+350 ;
				stop_p = point_y[plustray + 1]+350 ;
				start_v = 0; // qk
				stop_v = 0; // q_dotk+1
				timecycle = 1.5;
				main_Qubic();
				//Qubic(start_p, stop_p, start_v, stop_v, timecycle, 0, 0, 0);

				 // x axis
				 registerFrame[65].U16 = (int16_t)point_x[plustray+1]*10; // position Tray pick/place
				 registerFrame[66].U16 = 3000; // speed x-axis 300mm
				 registerFrame[67].U16 = 1; // Acc time 1mms
				 registerFrame[64].U16 = 2; //0x40 Moving Status x-axis - run mode
				 CaseTray = 1;
				}
				break;
			case 1:
				if(done == 0){
				timestamptray = HAL_GetTick() + 2000;
				done = 1;
				}
				if(HAL_GetTick() >= timestamptray){
					CaseTray = 2;
					done = 0;
				}
			break;
			case 2:
				 if(plustray == 17){
					Mobus = Initial;
					plustray = -1;
					CaseTray = 0;
					data_read = 0;
					choice = 7;
					I2C_all(&data_read);
					}
				 else{
				if(indexposition >= (timecycle*100)){
						s2 = 0;
						indexposition = 0;
						path = 0;
					}
				 if(abs(error) < 0.2){
					 if((plustray%2) != 0 || plustray == -1){ // Pick Case
						registerFrame[16].U16 = 8; //Status Picking
						s2 = 0;
						s = 0;
						 choice = 8;
						 I2C_all(&data_read);
						 CaseTray = 3;
					 }
					 if(plustray%2 == 0){ // Place Case
						registerFrame[16].U16 = 16; //Status Placing
						s2 = 0;
						s = 0;
						 choice = 9;
						 I2C_all(&data_read);
						 CaseTray = 3;
					 	 }
				 	 }
				 }
				break;
			case 3:
				if(I2Cdone == 0){
					timestampI2Cdone = HAL_GetTick() + 2500;
				I2Cdone = 1;
				}
				if(HAL_GetTick() >= timestampI2Cdone){
					choice = 10;
					I2C_all(&data_read);
					CaseTray = 4;
					I2Cdone = 0;
				}
				break;
			case 4: // Pick Case
				if(data_read == 7 || data_read == 4){
					if(plustray < 17 || plustray == -1){
					choice = 0;
					plustray++;
					CaseTray = 0;
					done = 0;
					}
				}
				break;
		}
		break;
	}
}
float control_interrupt() {
//loop 1
	error = SetDegree - ReadDegree;
	s = s + error;
	u = K_P * error + K_I * s + K_D * (error - p);
	p = error;
// loop 2
	error2 = (u + SetVelocity) - speed;
	s2 = s2 + error2;
	u2 = K_Pvelo * error2 + K_Ivelo * s2 + K_Dvelo * (error2 - p2);
	if (u2 > 65534)
		u2 = 65535;
	if (u2 < -65534)
		u2 = -65535;
	p2 = error2;
	return u2;
}
float control_velocity() {
	error2 = (control_interrupt() + SetVelocity) - speed;
	if (abs(error2) <= 0.5)
		s2 = 0;
	s2 = s2 + error2;
	u2 = K_P * error2 + K_I * s2 + K_D * (error2 - p2);
	if (u2 > 65535)
		u2 = 65535;
	if (u2 < -65535)
		u2 = -65535;
	p2 = error2;
	return u2;
}
void velocity() {
	QEIData.data[0] = __HAL_TIM_GET_COUNTER(&htim2);
	QEIData.timestamp[0] = micros();

	int32_t diffposition = QEIData.data[0] - QEIData.data[1];
	float difftime = QEIData.timestamp[0] - QEIData.timestamp[1];

	if (ReadDegree > 750) {
		SetDegree = 0;
	}
	if (diffposition < -(QEI_PERIOD >> 1)) {
		diffposition += QEI_PERIOD;
	}

	QEIData.QEIPosition = __HAL_TIM_GET_COUNTER(&htim2) % 8192;
	QEIData.QEIVelocity = (diffposition * 1000000) / difftime;

	QEIData.data[1] = QEIData.data[0];
	QEIData.timestamp[1] = QEIData.timestamp[0];

}
void accelerate() {
	QEIAcc.data[0] = speed;
	QEIAcc.timestamp[0] = micros();

	int32_t diffposition = QEIAcc.data[0] - QEIAcc.data[1];
	float difftime = QEIAcc.timestamp[0] - QEIAcc.timestamp[1];
	difftime = difftime / 1000000;

	QEIAcc.QEIVelocity = (diffposition / difftime);

	QEIAcc.data[1] = QEIAcc.data[0];
	QEIAcc.timestamp[1] = QEIAcc.timestamp[0];
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim5) {
		_micros += UINT32_MAX;
	}
}
uint64_t micros() {
	return __HAL_TIM_GET_COUNTER(&htim5) + _micros;
}
void main_Qubic() {
	emxArray_real_T *q_acc;
	emxArray_real_T *q_position;
	emxArray_real_T *q_velocity;
	/* Initialize function 'Qubic' input arguments. */
	q_k1_tmp = argInit_real_T();
	/* Call the entry-point 'Qubic'. */
	emxInitArray_real_T(&q_position, 2);
	emxInitArray_real_T(&q_velocity, 2);
	emxInitArray_real_T(&q_acc, 2);
	Qubic(start_p, stop_p, start_v, stop_v, timecycle, q_position, q_velocity,
			q_acc);
	q_positionN = q_position;
	q_velocityN = q_velocity;
	q_accN = q_acc;
	emxDestroyArray_real_T(q_position);
	emxDestroyArray_real_T(q_velocity);
	emxDestroyArray_real_T(q_acc);
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
	while (1) {
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
