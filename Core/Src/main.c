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
#include <stdlib.h>

//*************************** DEBUG MODE **********************************//
//#define TEST_HUSKY
//#define TEST_DYNA
//#define TEST_PING
//#define TEST_COMMUNICATION

//*************************** FUZZY CONST **********************************//
#define INPUT_BATAS_BAWAH 	3
#define INPUT_BATAS_TENGAH	6
#define INPUT_BATAS_ATAS		9

#define OUTPUT_BATAS_BAWAH 	-10
#define OUTPUT_BATAS_TENGAH	0
#define OUTPUT_BATAS_ATAS		10
//*************************** MODE STABLISASI ********************************//
//#define USE_STABILIZE_FUZZY
//#define USE_STABILIZE_PID
//*************************** MODE KALKULASI ********************************//
#define USE_FUZZY 				
#define USE_PID						
//#define USE_PID_FUZZY		

//*************************** ON/OFF MODUL ********************************//
#define HUSKY_ON 					
#define DYNA_ON 					
#define PING_ON						
#define COMMUNICATION_ON	

//*************************** FILE EKSTERNAL ******************************//
#ifdef PING_ON
#include "Ping_driver.h"
#include "DWT_Delay.h"
#endif

#ifdef COMMUNICATION_ON
#include "Komunikasi.h"
#endif

#ifdef HUSKY_ON
#include "Huskylens_driver.h"
#endif

#ifdef DYNA_ON
#include "Dynamixel.h"
#endif

#ifdef USE_FUZZY
#include "Fuzzy.h"
#endif

#ifdef USE_PID
#include "pid.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Filter untuk sensor PING
//#define FILTER_KAKI
#define FILTER_AVG

// State untuk deteksi Wall
#define STATE_KANAN 						0x01
#define STATE_KIRI 							0x02
#define STATE_DOUBLE						0x03
#define STATE_DEPAN 						0x04
#define STATE_BELAKANG 					0x05

// State untuk belok
#define BELOK_KANAN 						0x01
#define BELOK_KIRI 							0x02

// State untuk mode jalan
#define MODE_MENDAKI 						0x01
#define MODE_MELEWATI_KELERENG 	0x02
#define MODE_MENURUN 						0x03
#define MODE_MENCARI_KORBAN 		0x04

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Luas pembacaan diperoleh dari (luas arena - luas robot bagian luar) = 45 - 33 = 12
#define LEBAR_PEMBACAAN 				12.00

// Panjang ping kaki diperoleh dari pengukuran jarak dari ping ke kaki bagian dalam
#define PANJANG_PING_KAKI 			9.00

// Mengetahui Jarak terhadap tembok yang ada di depan (perlu diperhitungkan terkait dengan daerah minimal yang diperlukan untuk melakukan rotasi
#define BATAS_DEPAN 						9.00

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//****************************** CONFIG  HUSKYLENS *******************************//
#ifdef HUSKY_ON
huskylens_info_t huskAll;
huskylens_status_t status;
huskylens_arrow_t arrows;
huskylens_block_t blocks;
huskylens_all_byid_t id;
double husky_distance = 0;
#endif

//****************************** CONFIG  PING *******************************//
#ifdef PING_ON
Ping_t FR;
Ping_t BR;
Ping_t FL;
Ping_t BL;
Ping_t FF;
Ping_t BB;

double kanan = 0,kiri = 0;
double depan = 0, belakang = 0;

double FRV,BRV,FLV,BLV,FFV,BBV;

uint8_t error_kanan = 0;
uint8_t error_kiri = 0;
uint8_t error_depan = 0;
uint8_t error_belakang = 0;
#endif

//****************************** CONFIG  DYNAMIXEL *******************************//
#ifdef DYNA_ON
dynamixel_t ax;
uint16_t dyna_sudut = 0;
#endif

//***************************** CONFIG FUZZY**********************************//
#ifdef USE_FUZZY
Fuzzy_input_t input_fuzzy, rotasi_input_fuzzy, husky_input_fuzzy;
Fuzzy_output_t output_fuzzy, rotasi_output_fuzzy, husky_output_fuzzy;
Fuzzy_fuzzyfication_t fuz_fic_input, rotasi_fuz_fic_input, husky_fuz_fic_input;
Fuzzy_defuz_t defuz, rotasi_defuz, husky_defuz;
double res, rotasi_res, husky_res;	
#endif

//***************************** CONFIG PID **********************************//
#ifdef USE_PID
PID_TypeDef hPID;
double distance, PID_output, distance_setpoint;
double rot, PID_rot_output, rot_setpoint;
#endif

//**************************** CONFIG COMMUNICATION ********************************//
#ifdef COMMUNICATION_ON
feedback_t feeding;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// reset
	feeding.jalan = 0x00;
	feeding.ping = 0x00;
	feeding.req = 0x00;
	feeding.rotasi = 0x00;
	feeding.standby = 0x00;
	feeding.statis = 0x00;
	feeding.translasi = 0x00;
	rx_feedback(&feeding);
}
#endif
//****************************** PROTOTYPE CALCULATION  *******************************//
void fuzzy_run(uint8_t state_jalan, double input);
void pid_run(uint8_t state_jalan, double input);

//****************************** PROTOTYPE ALGORITMA JALAN *******************************//
void scp_wall_follower(uint8_t state);
void scp_belok(uint8_t direction);
void scp_wall_stabilizer(uint8_t state);
void scp_deteksi_korban(uint8_t id);
void scp_deteksi_arena(uint8_t id);
#ifdef COMMUNICATION_ON
void scp_mode_jalan(mode_jalan_t mode);
#endif
void scp_pengangkatan_korban(void);
void scp_penurunan_korban(void);

//****************************** PROTOTYPE ALGORITMA RUANGAN *******************************//
void kri_home(void);
void kri_ruangan_1(void);
void kri_ruangan_2(void);
void kri_ruangan_3(void);
void kri_ruangan_4(void);
void kri_ruangan_5(void);
void kri_ruangan_6(void);
void kri_ruangan_7(void);
void kri_ruangan_8(void);
void kri_ruangan_9(void);
void kri_ruangan_10(void);
void kri_ruangan_11(void);
void kri_finish(void);
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
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
	//*********************** COMMUNICATION CONFIG **************************//
	#ifdef COMMUNICATION_ON
	komunikasi_init(&huart2);
	rx_start();
	tx_move_steady();
	for(int i=0; i<5; i++) {
			tx_statis(90,0,-90);
			HAL_Delay(1000);
	}
	#endif
	
	//********************** Config For Huskylens *************************/
	#ifdef HUSKY_ON
	if(husky_setup(&hi2c3) == HUSKY_OK){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	}
	
	huskAll = husky_getAllArrowBlock();
	#endif
	
	//********************** Config For PING *************************/
	#ifdef PING_ON
	DWT_Delay_Init();
	
	FL.PING_PORT = GPIOE;
	FL.PING_PIN = GPIO_PIN_5;
	BL.PING_PORT = GPIOA;
	BL.PING_PIN = GPIO_PIN_1;
	FR.PING_PORT =GPIOB;
	FR.PING_PIN = GPIO_PIN_4;
	BR.PING_PORT = GPIOA;
	BR.PING_PIN = GPIO_PIN_15;
	FF.PING_PORT = GPIOB;
	FF.PING_PIN = GPIO_PIN_9;
	BB.PING_PORT = GPIOA;
	BB.PING_PIN = GPIO_PIN_11;
	#endif
	
	//*********************** DYNAMIXEL CONFIG **************************//
	#ifdef DYNA_ON
	dyna_init(&huart4, &ax, 0x11);
	dyna_calibrate(&ax);
	dyna_scan(&ax, 0, 100,MOVING_CW);
	#endif
	
	//*********************** FUZZY CONFIG **************************//
	#ifdef USE_FUZZY
	// PING SENSOR
	fuzzy_set_membership_input(&input_fuzzy, INPUT_BATAS_BAWAH, INPUT_BATAS_TENGAH, INPUT_BATAS_ATAS);
	fuzzy_set_membership_output(&output_fuzzy, OUTPUT_BATAS_BAWAH, OUTPUT_BATAS_TENGAH, OUTPUT_BATAS_ATAS);
	
	// PING STABILIZER
	fuzzy_set_membership_input(&rotasi_input_fuzzy, -10, 0, 10);
	fuzzy_set_membership_output(&rotasi_output_fuzzy, -20, 0, 20);
	
	// HUSKY
	fuzzy_set_membership_input(&husky_input_fuzzy, 0, 160, 320);
	fuzzy_set_membership_output(&husky_output_fuzzy, -23, 0, 23);
	#endif
	
	//*********************** PID CONFIG **************************//
	#ifdef USE_PID
	distance_setpoint = 9;
	PID(&hPID, &distance, &PID_output, &distance_setpoint, 5, 1, 0, PID_PROPOTIONAL_ERROR, PID_CONTROL_DIRECTION_FORWARD);
	PID_SetMode(&hPID, PID_AUTOMATIC_MODE);
  PID_SetSampleTime(&hPID, 10);
  PID_SetOutputLimits(&hPID, -10, 10);
	rot_setpoint = 0;
	PID(&hPID, &rot, &PID_rot_output, &rot_setpoint, 1, 1, 0, PID_PROPOTIONAL_ERROR, PID_CONTROL_DIRECTION_FORWARD);
	PID_SetMode(&hPID, PID_AUTOMATIC_MODE);
	PID_SetSampleTime(&hPID, 10);
	PID_SetOutputLimits(&hPID, -20, 20);
	#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		blocks = husky_getBlocks();
		husky_distance = husky_distance_prediction();
//		scp_deteksi_korban(1);
		
		//************************* HUSKY TEST ***********************//
		#ifdef TEST_HUSKY
		#ifdef HUSKY_ON
		blocks = husky_getBlocks();
		#endif
		#endif
		
		//************************* DYNAMIXEL TEST ***********************//
		#ifdef TEST_DYNA
		#ifdef DYNA_ON
		dyna_set_moving_speed(&ax, 0x0300, MOVING_CCW);
		dyna_set_goal_position(&ax, 0x0000);
		HAL_Delay(1000);
		dyna_set_goal_position(&ax, 0x0267);
		dyna_read_posisition(&ax);
		HAL_Delay(1000);
		dyna_set_goal_position(&ax, 0x0000);
		#endif
		#endif
			
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 1000000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//************************* FUZZY CALCULATION *******************//
#ifdef USE_FUZZY
void fuzzy_run(uint8_t state_jalan, double input){
		
		if((state_jalan == STATE_KANAN)&&(input > 0.0)){
			fuzzy_fuzfication_input(&input_fuzzy, &fuz_fic_input, input);
			fuzzy_logic_rule(&output_fuzzy, &fuz_fic_input, &defuz, FUZZY_MIN_TO_MAX);
			res = fuzzy_defuz(&defuz,&fuz_fic_input);
			tx_move_jalan(res, 15, 50, 10, JALAN_NORMAL);
		}
		else if((state_jalan == STATE_KIRI)&&(input > 0.0)){
			fuzzy_fuzfication_input(&input_fuzzy, &fuz_fic_input, input);
			fuzzy_logic_rule(&output_fuzzy, &fuz_fic_input, &defuz, FUZZY_MIN_TO_MAX);
			res = fuzzy_defuz(&defuz,&fuz_fic_input);
			tx_move_jalan(res*(-1), 15, 50, 10, JALAN_NORMAL);
		}
		else if ((state_jalan == STATE_DEPAN)&&(input > 0.0)){
			fuzzy_fuzfication_input(&input_fuzzy, &fuz_fic_input, input);
			fuzzy_logic_rule(&output_fuzzy, &fuz_fic_input, &defuz, FUZZY_MIN_TO_MAX);
			res = fuzzy_defuz(&defuz,&fuz_fic_input);
			tx_move_jalan(15, res, 50, 10, JALAN_NORMAL);
		}
		else if((state_jalan == STATE_BELAKANG)&&(input > 0.0)){
			fuzzy_fuzfication_input(&input_fuzzy, &fuz_fic_input, input);
			fuzzy_logic_rule(&output_fuzzy, &fuz_fic_input, &defuz, FUZZY_MIN_TO_MAX);
			res = fuzzy_defuz(&defuz,&fuz_fic_input);
			tx_move_jalan(15, res*(-1), 50, 10, JALAN_NORMAL);
		}
}
#endif

//************************* PID CALCULATION *******************//
#ifdef USE_PID
void pid_run(uint8_t state_jalan, double input){
		
		if((state_jalan == STATE_KANAN)&&(input > 0.0)){
			distance = input;
			PID_Compute(&hPID);
			tx_move_jalan(PID_output*(-1), 15, 30, 15, JALAN_NORMAL);
		}
		else if((state_jalan == STATE_KIRI)&&(input > 0.0)){
			distance = input;
			PID_Compute(&hPID);
			tx_move_jalan((PID_output)*(-1), 15, 30, 15, JALAN_NORMAL);
		}
}
#endif

//***************************************************************************************************/
//************************************ IMPLEMENTASI ALGORITMA ***************************************/

void scp_wall_follower(uint8_t state){
	
	#ifdef PING_ON
	// READ VALUE PING 
		BBV = ping_read(BB);
		FRV = ping_read(FR);
		BRV = ping_read(BR);
		BLV = ping_read(BL);
		FLV = ping_read(FL);
		FFV = ping_read(FF);
	
	//************************* Filter for PING **********************//

		#ifdef FILTER_AVG
		// Belok
		if((FFV <= 18) && !(state == STATE_DEPAN)){
			
			// Belok Kanan
			if((FRV >= LEBAR_PEMBACAAN) && (BRV >= LEBAR_PEMBACAAN) && (FFV <= 18) && (state == STATE_KANAN)){
				scp_belok(BELOK_KANAN);
			}
			
			// Belok Kiri
			else if((FLV >= LEBAR_PEMBACAAN) && (BLV >= LEBAR_PEMBACAAN) && (FFV <= 18) && (state == STATE_KIRI)){
				scp_belok(BELOK_KIRI);
			}
			
			tx_move_steady();
		}
		
		// Kiri
		else if((FLV > 0) && (BLV > 0) && (state == STATE_KIRI)){
			
			// Filter Tembok Rata
			if((FLV <= LEBAR_PEMBACAAN) && (BLV <= LEBAR_PEMBACAAN)){
				kiri = (FLV + BLV)/2;
				#ifdef USE_PID
				pid_run(STATE_KIRI, kiri);
				#endif
				#ifdef USE_FUZZY
				fuzzy_run(STATE_KIRI, kiri);
				#endif
			}
			
			// Fiter indikasi Loss
			else if((FLV >= LEBAR_PEMBACAAN) || (BLV >= LEBAR_PEMBACAAN)){
				kiri = INPUT_BATAS_TENGAH;
				#ifdef USE_PID
				pid_run(STATE_KIRI, kiri);
				#endif
				#ifdef USE_FUZZY
				fuzzy_run(STATE_KIRI, kiri);
				#endif
			}		
		#endif
		
		}
		
		// Kanan
		else if((FRV > 0) && (BRV > 0) && (state == STATE_KANAN)){
			
			// Filter Tembok Rata
			if((FRV <= LEBAR_PEMBACAAN) && (BRV <= LEBAR_PEMBACAAN)){
				kanan = (FRV + BRV)/2;
				#ifdef USE_FUZZY
				fuzzy_run(STATE_KANAN, kanan);
				#endif
				#ifdef USE_PID
				pid_run(STATE_KANAN, kanan);
				#endif
			}
			
			// Fiter indikasi Loss
			else if((FRV >= LEBAR_PEMBACAAN) || (BRV >= LEBAR_PEMBACAAN)){
				kanan = INPUT_BATAS_TENGAH;
				#ifdef USE_FUZZY
				fuzzy_run(STATE_KANAN, kanan);
				#endif
				#ifdef USE_PID
				pid_run(STATE_KANAN, kanan);
				#endif
			}
			
		}
		
		// Depan
		else if((state == STATE_DEPAN) && (FFV >= 0)){
			depan = (FFV);
			#ifdef USE_FUZZY
			fuzzy_run(STATE_DEPAN, depan);
			#endif
			#ifdef USE_PID
			pid_run(STATE_DEPAN, depan);
			#endif
		}
		
		// Belakang
		else if((state == STATE_BELAKANG) && (BBV >= 0)){
			belakang = (BBV);
			#ifdef USE_FUZZY
			fuzzy_run(STATE_BELAKANG, belakang);
			#endif
			#ifdef USE_PID
			pid_run(STATE_BELAKANG, belakang);
			#endif
		}
		#endif
		
}

void scp_belok(uint8_t direction){
		#ifdef COMMUNICATION_ON
		// Rotasi Clock Wise
		if(direction == BELOK_KANAN) tx_move_rotasi(0, 0, -30, 30, 1, 10);
	
		// Rotasi Counter Clock Wise
		if(direction == BELOK_KIRI)  tx_move_rotasi(0, 0, 30, 30, 1, 10);
				
		// Delay selama rotasi -> Menyesuaikan parameter rotasi
		HAL_Delay(1000);
	
		// Meluruskan PING berdasarkan State -> Fuzzy
	
		// Looping untuk membaca Ping bagian depan dan belakang
//		while(true){
//			BBV = ping_read(BB);
//			FFV = ping_read(FF);
//			
//			if( (BBV <= 18)){
//				tx_move_jalan(0, 15, 0, 5, JALAN_NORMAL);
//				break;
//			}
//		}
		#endif
}

void scp_wall_stabilizer(uint8_t state){
	#ifdef PING_ON
		double front_v = 0, back_v = 0;
	
		if(state == STATE_KANAN){
			front_v = ping_read(FR);
			back_v = ping_read(BR);
		}
		else if(state == STATE_KIRI){
			front_v = ping_read(FL);
			back_v = ping_read(BL);
		}
		
		if(((FRV - back_v) < 1.0) && ((FRV - back_v) > (-1.0)) ){
			tx_move_steady();
		}
		else{
			#ifdef USE_STABILIZE_FUZZY
			fuzzy_fuzfication_input(&rotasi_input_fuzzy, &rotasi_fuz_fic_input, (front_v-back_v));
			fuzzy_logic_rule(&rotasi_output_fuzzy, &rotasi_fuz_fic_input, &rotasi_defuz, FUZZY_MIN_TO_MAX);
			rotasi_res = fuzzy_defuz(&rotasi_defuz,&rotasi_fuz_fic_input);
			tx_move_rotasi(0, 0, rotasi_res*(-1), 30, 1, 10);
			#endif
			
			#ifdef USE_STABILIZE_PID
			rot = (front_v-back_v);
			PID_Compute(&hPID);
			tx_move_rotasi(0, 0, PID_rot_output*(-1), 30, 1, 10);
			#endif
		}
	#endif
}
#ifdef HUSKY_ON
#ifdef DYNA_ON
void scp_deteksi_korban(uint8_t id){

	#ifdef HUSKY_ON
		blocks = husky_getBlocks();
		dyna_sudut = dyna_read_posisition(&ax);
		if(blocks.id == id){
			
			#ifdef DYNA_ON
			
			fuzzy_fuzfication_input(&husky_input_fuzzy, &husky_fuz_fic_input, blocks.X_center);
			fuzzy_logic_rule(&husky_output_fuzzy, &husky_fuz_fic_input, &husky_defuz, FUZZY_MIN_TO_MAX);
			husky_res = fuzzy_defuz(&husky_defuz,&husky_fuz_fic_input);
			
			if(husky_res >= 0){
				dyna_set_moving_speed(&ax, 50, MOVING_CW);
				dyna_set_goal_position(&ax, dyna_sudut-husky_res);
			}
			else if(husky_res < 0){
				dyna_set_moving_speed(&ax, 50, MOVING_CCW);
				dyna_set_goal_position(&ax, dyna_sudut-husky_res);
			}
			#endif
			
			// Send Message
			if(dyna_sudut <= 1023){
					tx_move_jalan(((511.5-dyna_sudut)/(1023/40))*(-1), -15, 30, 15, JALAN_NORMAL);
			}
		}
		if(dyna_sudut >= 1023){
			dyna_scan(&ax, 0, 100,MOVING_CW);
		}
		#endif
	}
#endif
#endif

#ifdef HUSKY_ON
#ifdef DYNA_ON
void scp_deteksi_arena(uint8_t id){
	// Set ke Algoritma color detection
	status = husky_setAlgorithm(ALGORITHM_OBJECT_CLASSIFICATION);
	
	// Set Kecepatan Putaran Dynamixel
	dyna_set_moving_speed(&ax,0x0200,MOVING_CW);

//	// Memutar sepanjang 300 derajat -> 5 derajat setiap instruksi
//	int i = 0;
//	for(; i < 300; i+=5){

//		if(blocks.id == id) break;

//		// Memutar setiap 5 derajat
//		dyna_set_goal_position(&ax, i);

//		// Mulai pembacaan warna
//		blocks = husky_getBlocks();
//	
//	}
//	
//	// Instruksi rotasi sejauh sudut i
//	tx_move_rotasi(i, 0, 0, 30, 100, 20, 1);
}
#endif
#endif

#ifdef COMMUNICATION_ON
void scp_mode_jalan(mode_jalan_t mode){
	tx_move_jalan(0, 15, 0, 5, mode);
}
#endif

//***************************************************************************************************/
//***************************************************************************************************/

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
