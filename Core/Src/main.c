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
//*************************** MODE KALKULASI ********************************//
//#define USE_FUZZY 				
#define USE_PID						
//#define USE_PID_FUZZY		

//*************************** ON/OFF MODUL ********************************//
//#define HUSKY_ON 					
//#define DYNA_ON 					
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

// State untuk deteksi Wall
#define STATE_KANAN 						0x01
#define STATE_KIRI 							0x02

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

// Luas pembacaan diperoleh dari (luas arena - luas robot bagian luar) = 45 - 27.5 = 18
#define LEBAR_PEMBACAAN 				18.00

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
#endif

//***************************** CONFIG FUZZY**********************************//
#ifdef USE_FUZZY
Fuzzy_input_t input_fuzzy;
Fuzzy_output_t output_fuzzy;
Fuzzy_fuzzyfication_t fuz_fic_input;
Fuzzy_defuz_t defuz;
double res;	
#endif

//***************************** CONFIG PID **********************************//
#ifdef USE_PID
PID_TypeDef hPID;
double distance, PID_output, distance_setpoint;
#endif

//**************************** CONFIG COMMUNICATION ********************************//
#ifdef COMMUNICATION_ON
feedback_t feeding;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	rx_feedback(&feeding);
}
#endif
//****************************** PROTOTYPE ALGORITMA JALAN *******************************//
void scp_wall_follower(uint8_t state);
void scp_belok(uint8_t direction);
void scp_deteksi_korban(uint8_t id);
void scp_deteksi_arena(uint8_t id);
void scp_mode_jalan(mode_jalan_t mode);
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
  /* USER CODE BEGIN 2 */
	
	//********************** Config For Huskylens *************************/
	#ifdef HUSKY_ON
	if(husky_setup(&hi2c2) == HUSKY_OK){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	}
	
	huskAll = husky_getAllArrowBlock();
	#endif
	
	//********************** Config For PING *************************/
	#ifdef PING_ON
	DWT_Delay_Init();
	
	FR.PING_PORT = GPIOA;
	FR.PING_PIN = GPIO_PIN_0;
	BR.PING_PORT = GPIOA;
	BR.PING_PIN = GPIO_PIN_1;
	FL.PING_PORT =GPIOB;
	FL.PING_PIN = GPIO_PIN_4;
	BL.PING_PORT = GPIOA;
	BL.PING_PIN = GPIO_PIN_15;
	FF.PING_PORT = GPIOB;
	FF.PING_PIN = GPIO_PIN_9;
	BB.PING_PORT = GPIOA;
	BB.PING_PIN = GPIO_PIN_11;
	#endif
	
	//*********************** DYNAMIXEL CONFIG **************************//
	#ifdef DYNA_ON
	dyna_init(&huart2, &ax, 0x11);
	dyna_set_limit_CW(&ax, 0x0000);
	dyna_set_limit_CCW(&ax, 0x03FF);
	dyna_set_torque_enabler(&ax, TORQUE_ON);
	#endif
	
	//*********************** FUZZY CONFIG **************************//
	#ifdef USE_FUZZY
	fuzzy_set_membership_input(&input_fuzzy, 15, 20, 25);
	fuzzy_set_membership_output(&output_fuzzy, 0, 15, 30);
	#endif
	
	//*********************** PID CONFIG **************************//
	#ifdef USE_PID
	distance_setpoint = 10;
	PID(&hPID, &distance, &PID_output, &distance_setpoint, 2, 1, 1, PID_PROPOTIONAL_ERROR, PID_CONTROL_DIRECTION_FORWARD);
	PID_SetMode(&hPID, PID_AUTOMATIC_MODE);
  PID_SetSampleTime(&hPID, 500);
  PID_SetOutputLimits(&hPID, 0, 30);
	#endif
	
	//*********************** COMMUNICATION CONFIG **************************//
	#ifdef COMMUNICATION_ON
	komunikasi_init(&huart2);
	rx_start();
	#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		scp_wall_follower(STATE_KANAN);
		
		//************************* HUSKY DETECTION ***********************//
		#ifdef HUSKY_ON
		blocks = husky_getBlocks();
		#endif
		
		//************************* DYNAMIXEL ROTATION ***********************//
		#ifdef DYNA_ON
		dyna_set_moving_speed(&ax, 0x0300, MOVING_CCW);
		dyna_set_goal_position(&ax, 0x0000);
		HAL_Delay(1000);
		dyna_set_goal_position(&ax, 0x0267);
		dyna_read_posisition(&ax);
		HAL_Delay(1000);
		dyna_set_goal_position(&ax, 0x0000);
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

//***************************************************************************************************/
//************************************ IMPLEMENTASI ALGORITMA ***************************************/

void scp_wall_follower(uint8_t state){
	
	// READ VALUE PING 
		BBV = ping_read(BB);
		FRV = ping_read(FR);
		BRV = ping_read(BR);
		BLV = ping_read(BL);
		FLV = ping_read(FL);
		FFV = ping_read(FF);
	
	//************************* Filter for PING **********************//
		/*
		* Filter dimulai dari pemilihan pembacaan sensor dengan nilai tertinggi (dengan syarat dibawah nilai luas pembacaan).
		* Setelah diperoleh maka nilai dimasukkan ke dalam variabel kiri / kanan
		* Jika terdapat error pembacaan / tidak memenuhi syarat, maka akan disimpan ke dalam variabel error
		* Jika Error pembacaan lebih dari 1 kali -> indikasi adanya belokan
		* Jika ada indikasi error pada kanan / kiri dan nilai pembacaan di depan pendek ( < 17.5) maka akan mengindikasi untuk belok kanan/kiri
		*/
		
		// Mencari Jarak terhadap Wall Kanan / Kiri
		if((FRV >= BRV) && (FRV > PANJANG_PING_KAKI) && (FRV < LEBAR_PEMBACAAN)){
			kanan = FRV;
			error_kanan = 0;
		}
		
		else if((FRV <= BRV) && (BRV > PANJANG_PING_KAKI) && (BRV < LEBAR_PEMBACAAN)){
			kanan = BRV;
			error_kanan = 0;
		}
		else error_kanan += 1;
		
		if((FLV >= BLV) && (FLV > PANJANG_PING_KAKI) && (FLV < LEBAR_PEMBACAAN)){
			kiri = FLV;
			error_kiri = 0;
		}
		else if((FLV <= BLV) && (BLV > PANJANG_PING_KAKI) && (BLV < LEBAR_PEMBACAAN)){
			kiri = BLV;
			error_kiri = 0;
		}
		else error_kiri += 1;
		
		
		// Mengetahui Jarak Depan terhadap tembok untuk menentukan arah belok
		if(depan <= BATAS_DEPAN){
			
			// Rotasi "Clock Wise" sebesar 90deg Jika Jarak di depan kurang dari batas dan jarak di kanan melebihi range batas
			if(error_kanan >= 1){
				scp_belok(BELOK_KANAN);
				
			}
			// Rotasi "Counter Clock Wise" sebesar 90deg Jika Jarak di depan kurang dari batas dan jarak di kanan melebihi range batas
			else if(error_kanan >= 1){
				scp_belok(BELOK_KIRI);
			}
		}
		#ifdef USE_FUZZY
		//************************* FUZZY CALCULATION *******************//
		if((state == STATE_KANAN)&&(kanan > 0.0)){
			fuzzy_fuzfication_input(&input_fuzzy, &fuz_fic_input, kanan);
			fuzzy_logic_rule(&output_fuzzy, &fuz_fic_input, &defuz, FUZZY_MIN_TO_MAX);
			res = fuzzy_defuz(&defuz,&fuz_fic_input);
			tx_move_jalan(res, 15, 0, 5, JALAN_NORMAL);
		}
		else if((state == STATE_KIRI)&&(kiri > 0)){
			fuzzy_fuzfication_input(&input_fuzzy, &fuz_fic_input, kiri);
			fuzzy_logic_rule(&output_fuzzy, &fuz_fic_input, &defuz, FUZZY_MIN_TO_MAX);
			res = fuzzy_defuz(&defuz,&fuz_fic_input);
			tx_move_jalan(res, 15, 0, 5, JALAN_NORMAL);
		}
		#endif
		
		#ifdef USE_PID
		//************************* PID CALCULATION *******************//
		if((STATE_KANAN == 1)&&(kanan > 0.0)){
			distance = kanan;
			PID_Compute(&hPID);
		}
		else if((STATE_KIRI == 1)&&(kiri > 0)){
			distance = kiri;
			PID_Compute(&hPID);
		}
		
		//************************* SEND MESSAGE ***********************//
		// Wall follower Kanan -> Nilai > 30
		if((STATE_KANAN == 1) && PID_output > 0.0) tx_move_jalan(PID_output, 15, 0, 5, JALAN_NORMAL);
		
		// Wall follower Kiri -> Nilai < 30
		else if((STATE_KIRI == 1) && PID_output > 0.0) tx_move_jalan(PID_output, 15, 0, 5, JALAN_NORMAL);
		
		else tx_move_jalan(PID_output, 0, 0, 5, JALAN_NORMAL);
		#endif
		
}

void scp_belok(uint8_t direction){
		
		// Rotasi Clock Wise
		if(direction == 0x01) tx_move_rotasi(100, 100, 100, 10, 100, 30, 1);
	
		// Rotasi Counter Clock Wise
		if(direction == 0x02) tx_move_rotasi(100, 100, 100, 10, 100, 30, 1);
				
		// Delay selama rotasi -> Menyesuaikan parameter rotasi
		HAL_Delay(1000);
	
		// Looping untuk membaca Ping bagian depan dan belakang
		while(true){
			BBV = ping_read(BB);
			FFV = ping_read(FF);
			
			if( (BBV <= 18)){
				tx_move_jalan(0, 15, 0, 5, JALAN_NORMAL);
				break;
			}
		}
}
#ifdef HUSKY_ON
#ifdef DYNA_ON
void scp_deteksi_korban(uint8_t id){

	// Set ke Algoritma color detection
	status = husky_setAlgorithm(ALGORITHM_COLOR_RECOGNITION);
	
	// Set Kecepatan Putaran Dynamixel
	dyna_set_moving_speed(&ax,0x0200,MOVING_CW);

	// Memutar sepanjang 300 derajat -> 5 derajat setiap instruksi
	int i = 0;
	for(; i < 300; i+=5){

		if(blocks.id == id) break;

		// Memutar setiap 5 derajat
		dyna_set_goal_position(&ax, i);

		// Mulai pembacaan warna
		blocks = husky_getBlocks();
	
	}
	
	// Instruksi rotasi sejauh sudut i
	tx_move_rotasi(i, 0, 0, 30, 100, 20, 1);
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

	// Memutar sepanjang 300 derajat -> 5 derajat setiap instruksi
	int i = 0;
	for(; i < 300; i+=5){

		if(blocks.id == id) break;

		// Memutar setiap 5 derajat
		dyna_set_goal_position(&ax, i);

		// Mulai pembacaan warna
		blocks = husky_getBlocks();
	
	}
	
	// Instruksi rotasi sejauh sudut i
	tx_move_rotasi(i, 0, 0, 30, 100, 20, 1);
}
#endif
#endif


void scp_mode_jalan(mode_jalan_t mode){
	tx_move_jalan(0, 15, 0, 5, mode);
}

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
