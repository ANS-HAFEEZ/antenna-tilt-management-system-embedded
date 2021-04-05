/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "filtering.h"
#include "lis3dh.h"
#include "gyroscope.h"
#include <math.h> 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define roundz(x,d) ((floor(((x)*pow(10,d))+.5))/pow(10,d))
#define AXES 3
#define RAD2PI (180/3.141592)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
PUTCHAR_PROTOTYPE{HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF); return ch;}
uint32_t micros(void) 
{
  uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
  register uint32_t ms, cycle_cnt;
  do{
		ms = HAL_GetTick();
    cycle_cnt = SysTick->VAL;
  }while (ms != HAL_GetTick());
  return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//#define Calib 1
IMUS Acc;
IMUS Gyro;

const int SegLen = 8;
static int CalibCount = 5000;

double GScale = 0.4;
double AScale = 1.0;
double roll=0;
double pitch=0;
double yaw=0;

double dt=0.0;
kalmanS RollKF;
kalmanS PitchKF;
kalmanS YawKF;

int16_t RawAcc[3]   = {0};
int16_t RawGyro[3]  = {0};

static uint8_t whoamI;

static void Acc_write(uint8_t reg, uint8_t bufp){
	reg |= 0x80;
  HAL_I2C_Mem_Write(&hi2c1, LIS3DH_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, &bufp,1, 1000);
}

static void Acc_read(uint8_t reg, uint8_t *bufp){
	reg |= 0x80;
  HAL_I2C_Mem_Read(&hi2c1, LIS3DH_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp,1, 1000);
}

void AccGetPos(int16_t *raw){
	static uint8_t buff[6]= {0};
	HAL_I2C_Mem_Read(&hi2c1, LIS3DH_I2C_ADD_H, LIS3DH_OUT_X_L | 0x80, I2C_MEMADD_SIZE_8BIT, buff, 6, 1000);
	raw[0] = (int16_t) (buff[0]| (((uint16_t)buff[1]) << 8));
  raw[1] = (int16_t) (buff[2]| (((uint16_t)buff[3]) << 8));
	raw[2] = (int16_t) (buff[4]| (((uint16_t)buff[5]) << 8));
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\nProram Staring.........\n");
	while(whoamI != LIS3DH_ID){ 	Acc_read(LIS3DH_WHO_AM_I, &whoamI);	HAL_Delay(10);	}
    printf("\r\nLIS3DH Activated \n");

    //static uint8_t val = 0x70| 0x04| 0x02| 0x01;
		static uint8_t val = 0x97;
    printf("\r\nLIS3DH Writting Setting  %02X\n",val);
		Acc_write(LIS3DH_CTRL_REG1,val);
    printf("\r\nLIS3DH Reading Setting  %02X\n",val);
		HAL_Delay(10);
    Acc_write(LIS3DH_CTRL_REG2,0);
		HAL_Delay(10);
//    AccSetGet(LIS3DH_CTRL_REG3,0);
//		HAL_Delay(200);
//    AccSetGet(LIS3DH_CTRL_REG4,0);
//		HAL_Delay(200);
//    AccSetGet(LIS3DH_CTRL_REG5,0);
//		HAL_Delay(200);
//    AccSetGet(LIS3DH_CTRL_REG6,0);
//		HAL_Delay(200);
    
    Acc_read(LIS3DH_STATUS_REG,&val);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	static uint8_t Acci[6];

	BSP_GYRO_Init();
	uint8_t pTX[15] = "I am STM32F030\n";

	printf("Starting Calibration...\n");
	
	
	for(int i=0;i<200;i++){ AccGetPos(RawAcc);	GyroGetRate(RawGyro); }
	HAL_Delay(5000);
#ifdef Calib	
	for(int i=0;i<CalibCount;i++){
		AccGetPos(RawAcc);	GyroGetRate(RawGyro);
		for(int j=0;j<AXES;j++){	Acc.Bais[j] 	+=  RawAcc[j];	Gyro.Bais[j]	+=	RawGyro[j];	}
	}
	
	for(int j=0;j<AXES;j++){
		Gyro.Bais[j]= Gyro.Bais[j]/(CalibCount);	Acc.Bais[j] = Acc.Bais[j]/(CalibCount);
		if(j>1){	Acc.Bais[j] -= 16383;	}
	}
	printf("\r\nA : %d\t%d\t%d\r\nG: %d\t%d\t%d\t\n",Acc.Bais[0],Acc.Bais[1],Acc.Bais[2],Gyro.Bais[0],Gyro.Bais[1],Gyro.Bais[2]);
#else	

	Acc.Bais[0]= 239;
	Acc.Bais[1]= -461;
	Acc.Bais[2]= -386;
	
	Gyro.Bais[0]= -10;
	Gyro.Bais[1]= 2;
	Gyro.Bais[2]= 0;
#endif	
	static char AntO[40]= {0};
	InitKalman(&RollKF,0.001,1.0017);
	InitKalman(&PitchKF,0.001,1.0017);
	InitKalman(&YawKF,0.0001,0.0017);
	
	uint32_t StartTime=micros();
	
	char pRX[30]={0};
	static uint16_t DLen=0;
	
	float KalROLL  = 0.0;
	float KalPITCH = 0.0;
	float KalYAW 	 = 0.0;

	int reSPI=0;
	while (true){
		StartTime = micros();
		for(int i=0;i<SegLen;i++){			
			AccGetPos(RawAcc);	GyroGetRate(RawGyro);	
			for(int j=0;j<AXES;j++){Acc.In[j][i]=RawAcc[j]-Acc.Bais[j];	Gyro.In[j][i]=RawGyro[j]-Gyro.Bais[j];}
		}
		for(int j=0;j<AXES;j++){ GFilter(Gyro.In[j],Gyro.Out[j]); AFilter(Acc.In[j],Acc.Out[j]);	}
		for(int i=0;i<SegLen;i++){
			for(int j=0;j<AXES;j++){ Gyro.mean[j]+=Gyro.Out[j][i];	Acc.mean[j] +=Acc.Out[j][i];	}
		}
		for(int j=0;j<AXES;j++){ Gyro.mean[j] = Gyro.mean[j]/(float)SegLen;	Acc.mean[j]  = Acc.mean[j]/(float)SegLen;	}
		dt = (micros()-StartTime)/(1000000.0f - 0.0f);
		Gyro.roll 	= (Gyro.mean[X] * dt * GScale) + roundz(Gyro.roll,1);
		Gyro.pitch	= (Gyro.mean[Y] * dt * GScale) + roundz(Gyro.pitch,1);
		Gyro.yaw		= (Gyro.mean[Z] * dt * GScale) + roundz(Gyro.yaw,1);
		
		Acc.roll 	  = RAD2PI * atan(Acc.mean[Y] / sqrt(pow(Acc.mean[X],2) + pow(Acc.mean[Z],2))) *AScale;
		Acc.pitch 	= RAD2PI * atan(Acc.mean[X] / sqrt(pow(Acc.mean[Y],2) + pow(Acc.mean[Z],2))) *AScale; 
		Acc.yaw 		= RAD2PI * atan(sqrt(pow(Acc.mean[Y],2) + pow(Acc.mean[X],2)) / Acc.mean[Z]);
		
		roll 	= 0.9 * (roll +  (Gyro.mean[X]/(float)SegLen)* dt) + 0.1*Acc.roll;
		pitch = 0.9 * (pitch + (Gyro.mean[Y]/(float)SegLen)* dt) + 0.1*Acc.pitch;

		KalROLL 	= ProcessKalman(&RollKF,roll);
		KalPITCH 	= ProcessKalman(&PitchKF,pitch);
		KalYAW 		= ProcessKalman(&YawKF,Gyro.yaw);

#ifdef Calib			
		sprintf(AntO,"\n%d,%d,%d,%d,%d,%d,\n",Acc.Bais[0],Acc.Bais[1],Acc.Bais[2],Gyro.Bais[0],Gyro.Bais[1],Gyro.Bais[2]);
#else		
		sprintf(AntO,"ANT:R%.02lf,P%.02lf,Y%.02lf,%c",KalPITCH,KalROLL,KalYAW*5,0x0A);
#endif
		memset(pRX,0,sizeof(pRX));
		HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)AntO,(uint8_t *)pRX,40,100);
		//printf("\r\nSPI Rec Length %d \r\n",strlen(pRX));		
		if(strlen(pRX)>0){
			if(strstr(pRX,"OK")){
				reSPI=0;
				printf("\r\nMatched \r\n");		
			}
			else{
				HAL_Delay(1);
				HAL_SPIEx_FlushRxFifo(&hspi1);
				HAL_SPI_DeInit(&hspi1);
				HAL_Delay(1);
				HAL_SPI_Init(&hspi1);
			}
		}
		else{
			if(reSPI++>20){
				printf("\r\nReinit SPI\r\n");		
				reSPI=0;
				HAL_Delay(1);
				HAL_SPIEx_FlushRxFifo(&hspi1);
				HAL_SPI_DeInit(&hspi1);
				HAL_Delay(1);
				HAL_SPI_Init(&hspi1);
			}
		}
		printf("\r%s",AntO);		
  }  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  //hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if (GPIO_Pin == GPIO_PIN_0 || GPIO_Pin == GPIO_PIN_1 || GPIO_Pin == GPIO_PIN_2 || GPIO_Pin == GPIO_PIN_3)
//    {
//        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
//        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
//    }
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
