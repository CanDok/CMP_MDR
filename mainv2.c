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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t tx_buff[]={'H','e','l','l','o',13,10};
uint8_t SB[] = {13,10};
uint8_t S1[] = {"CCW SET!"};
uint8_t S2[] = {"CW SET!"};
uint8_t S3[] = {"ARMED!"};
uint8_t S4[] = {"DISARMED!"};
uint8_t S5[] = {"FS SET!"};
uint8_t S6[] = {"HS SET!"};
uint8_t S7[] = {"QS SET!"};
uint8_t S8[] = {"ES SET!"};
uint8_t S9[] = {"SS SET!"};
uint8_t S10[] = {"Delay SET!"};
uint8_t S11[] = {"Step SET!"};
uint8_t S12[] = {"LIMIT1"};
uint8_t S14[] = {"LIMIT2"};
uint8_t S15[] = {"HOME SET!"};
uint8_t S16[] = {"LIMIT2"};
uint8_t S17[] = {"CMP_DRV_V01"};
uint8_t S18[] = {"SESAME Compact Motor Controller"};
uint8_t S19[] = {"SN:7052188"};
uint8_t S20[] = {"By:cdokuyucu"};

int stp =0;
int pos=0;
int delay=1;
int parity=0;
uint8_t rx_buff[11];
uint8_t st[10];
uint8_t sst[1];
int state=0;
int lim1=0;
int lim2=0;
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
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, rx_buff, 11);
  HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  if((rx_buff[0]=='C')&(rx_buff[1]=='T')&(rx_buff[2]=='R')&(rx_buff[3]=='I')&(rx_buff[4]=='D')&(rx_buff[5]=='N')&(rx_buff[6]=='/')&(rx_buff[7]=='*')&(rx_buff[8]=='?')){
		  HAL_UART_Transmit(&huart1, S17, 11, 1000);
		  HAL_UART_Transmit(&huart1, SB, 2, 1000);
		  HAL_UART_Transmit(&huart1, S18, 31, 1000);
		  HAL_UART_Transmit(&huart1, SB, 2, 1000);
		  HAL_UART_Transmit(&huart1, S19, 10, 1000);
		  HAL_UART_Transmit(&huart1, SB, 2, 1000);
		  HAL_UART_Transmit(&huart1, S20, 12, 1000);
	 	 HAL_UART_Transmit(&huart1, SB, 2, 1000);
	 	 	 	  	       rx_buff[0]='x';
	 	 	 	  	 	   rx_buff[1]='x';
	 	 	 	  	 	   rx_buff[2]='x';
	 	 	 	  	 	   rx_buff[3]='x';
	 	 	 	  	 	   rx_buff[4]='x';
	 	 	 	  	 	   rx_buff[5]='x';
	 	 	 	  	 	   rx_buff[6]='x';
	 	 	 	  	 	   rx_buff[7]='x';
	 	 	 	  	 	   rx_buff[8]='x';
	 	 	 	  	 }

	  if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='S')&(rx_buff[4]=='E')&(rx_buff[5]=='T')&(rx_buff[6]=='H')&(rx_buff[7]=='O')&(rx_buff[8]=='M')){
	 		pos=0;
	 		itoa (pos,st,10);
	 HAL_UART_Transmit(&huart1, S15, 9, 1000);
	 HAL_UART_Transmit(&huart1, SB, 2, 1000);
	 	 	  	       rx_buff[0]='x';
	 	 	  	 	   rx_buff[1]='x';
	 	 	  	 	   rx_buff[2]='x';
	 	 	  	 	   rx_buff[3]='x';
	 	 	  	 	   rx_buff[4]='x';
	 	 	  	 	   rx_buff[5]='x';
	 	 	  	 	   rx_buff[6]='x';
	 	 	  	 	   rx_buff[7]='x';
	 	 	  	 	   rx_buff[8]='x';
	 	 	  	 }



	  if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='C')&(rx_buff[4]=='H')&(rx_buff[5]=='K')&(rx_buff[6]=='L')&(rx_buff[7]=='S')&(rx_buff[8]=='1')){
		  itoa (lim1,sst,10);
HAL_UART_Transmit(&huart1, sst, 1, 1000);
HAL_UART_Transmit(&huart1, SB, 2, 1000);
	 	  	       rx_buff[0]='x';
	 	  	 	   rx_buff[1]='x';
	 	  	 	   rx_buff[2]='x';
	 	  	 	   rx_buff[3]='x';
	 	  	 	   rx_buff[4]='x';
	 	  	 	   rx_buff[5]='x';
	 	  	 	   rx_buff[6]='x';
	 	  	 	   rx_buff[7]='x';
	 	  	 	   rx_buff[8]='x';
	 	  	 }



	  if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='C')&(rx_buff[4]=='H')&(rx_buff[5]=='K')&(rx_buff[6]=='L')&(rx_buff[7]=='S')&(rx_buff[8]=='2')){
		  itoa (lim2,sst,10);
		  HAL_UART_Transmit(&huart1, sst, 1, 1000);
		  HAL_UART_Transmit(&huart1, SB, 2, 1000);
	 	  	       rx_buff[0]='x';
	 	  	 	   rx_buff[1]='x';
	 	  	 	   rx_buff[2]='x';
	 	  	 	   rx_buff[3]='x';
	 	  	 	   rx_buff[4]='x';
	 	  	 	   rx_buff[5]='x';
	 	  	 	   rx_buff[6]='x';
	 	  	 	   rx_buff[7]='x';
	 	  	 	   rx_buff[8]='x';
	 	  	 }





if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_10)){
	 HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_SET);
	 lim1=1;
	 }else{lim1=0;}

if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_11)){
	 HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_RESET);
	 lim2=1;
     }else{lim2=0;}




	  if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='C')&(rx_buff[4]=='S')&(rx_buff[5]=='T')&(rx_buff[6]=='A')&(rx_buff[7]=='R')&(rx_buff[8]=='T')){


	  	 		 for( uint32_t i=0;i<stp;i++){
	  	 		HAL_GPIO_WritePin(STP_GPIO_Port,STP_Pin, GPIO_PIN_RESET);
	  	 		HAL_Delay(delay);
	  	 		HAL_GPIO_WritePin(STP_GPIO_Port,STP_Pin, GPIO_PIN_SET);
	  	 		 HAL_Delay(delay);
	  	 		 if(parity==1){
	  	 		 pos++;}
	  	 		 else
	  	 		 {pos--;}

	  	 		 itoa (pos,st,10);
	  	 	 		  		  }

	  	 		       rx_buff[0]='x';
	  	 			   rx_buff[1]='x';
	  	 			   rx_buff[2]='x';
	  	 			   rx_buff[3]='x';
	  	 			   rx_buff[4]='x';
	  	 			   rx_buff[5]='x';
	  	 			   rx_buff[6]='x';
	  	 			   rx_buff[7]='x';
	  	 			   rx_buff[8]='x';
	  	 	 }



	  	 if((rx_buff[0]=='S')&(rx_buff[1]=='E')&(rx_buff[2]=='T')&(rx_buff[3]=='S')&(rx_buff[4]=='P')){
	  		HAL_UART_Transmit(&huart1, S11, 9, 1000);
	  		HAL_UART_Transmit(&huart1, SB, 2, 1000);
	  	 	char sayi= rx_buff[5];
	  	 	int ss1 = sayi-'0';
	  	 	ss1=ss1*1000;
	  	 	 sayi= rx_buff[6];
	  	 	int	ss2 = sayi-'0';
	  	 	ss2=ss2*100;
	  	 		 sayi= rx_buff[7];
	  	 		int	ss3 = sayi-'0';
	  	 		ss3=ss3*10;
	  	 			 sayi= rx_buff[8];
	  	 			int	ss4 = sayi-'0';
	  	 			stp =ss1+ss2+ss3+ss4;

	  	 		       rx_buff[0]='x';
	  	 			   rx_buff[1]='x';
	  	 			   rx_buff[2]='x';
	  	 			   rx_buff[3]='x';
	  	 			   rx_buff[4]='x';
	  	 			   rx_buff[5]='x';
	  	 			   rx_buff[6]='x';
	  	 			   rx_buff[7]='x';
	  	 			   rx_buff[8]='x';
	  	 }




	  	 if((rx_buff[0]=='S')&(rx_buff[1]=='E')&(rx_buff[2]=='T')&(rx_buff[3]=='D')&(rx_buff[4]=='L')){
	  	 	HAL_UART_Transmit(&huart1, S10, 10, 1000);
	  	 	HAL_UART_Transmit(&huart1, SB, 2, 1000);

	  	 	char sayi= rx_buff[5];
	  	 	int sd1 = sayi-'0';
	  	 	sd1=sd1*1000;
	  	 	 sayi= rx_buff[6];
	  	 	int	sd2 = sayi-'0';
	  	 	sd2=sd2*100;
	  	 		 sayi= rx_buff[7];
	  	 		int	sd3 = sayi-'0';
	  	 		sd3=sd3*10;
	  	 			 sayi= rx_buff[8];
	  	 			int	sd4 = sayi-'0';
	  	 			delay =sd1+sd2+sd3+sd4;

	  	 		 rx_buff[0]='x';
	  	 			   rx_buff[1]='x';
	  	 			   rx_buff[2]='x';
	  	 			   rx_buff[3]='x';
	  	 			   rx_buff[4]='x';
	  	 			   rx_buff[5]='x';
	  	 			   rx_buff[6]='x';
	  	 			   rx_buff[7]='x';
	  	 			   rx_buff[8]='x';
	  	 }




	  	 if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='S')&(rx_buff[4]=='E')&(rx_buff[5]=='T')&(rx_buff[6]=='D')&(rx_buff[7]=='R')&(rx_buff[8]=='-')){
	  	 	HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_SET);
	  	 	HAL_UART_Transmit(&huart1, S2, 7, 1000);
	  	 	HAL_UART_Transmit(&huart1, SB, 2, 1000);
	  		parity=0;
	  	       rx_buff[0]='x';
	  	 	   rx_buff[1]='x';
	  	 	   rx_buff[2]='x';
	  	 	   rx_buff[3]='x';
	  	 	   rx_buff[4]='x';
	  	 	   rx_buff[5]='x';
	  	 	   rx_buff[6]='x';
	  	 	   rx_buff[7]='x';
	  	 	   rx_buff[8]='x';
	  	 }

	  	 if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='S')&(rx_buff[4]=='E')&(rx_buff[5]=='T')&(rx_buff[6]=='D')&(rx_buff[7]=='R')&(rx_buff[8]=='+')){
	  		 HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_RESET);
	  	 	HAL_UART_Transmit(&huart1, S1, 8, 1000);
	  	 	HAL_UART_Transmit(&huart1, SB, 2, 1000);
	  	 	parity=1;
	  	  rx_buff[0]='x';
	  	 	   rx_buff[1]='x';
	  	 	   rx_buff[2]='x';
	  	 	   rx_buff[3]='x';
	  	 	   rx_buff[4]='x';
	  	 	   rx_buff[5]='x';
	  	 	   rx_buff[6]='x';
	  	 	   rx_buff[7]='x';
	  	 	   rx_buff[8]='x';
	  	 }






	  	 if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='S')&(rx_buff[4]=='E')&(rx_buff[5]=='T')&(rx_buff[6]=='A')&(rx_buff[7]=='R')&(rx_buff[8]=='M')){
	  			 HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin, GPIO_PIN_RESET);
	  		 	HAL_UART_Transmit(&huart1, S3, 6, 1000);
	  		 	HAL_UART_Transmit(&huart1, SB, 2, 1000);
	  		  rx_buff[0]='x';
	  		 	   rx_buff[1]='x';
	  		 	   rx_buff[2]='x';
	  		 	   rx_buff[3]='x';
	  		 	   rx_buff[4]='x';
	  		 	   rx_buff[5]='x';
	  		 	   rx_buff[6]='x';
	  		 	   rx_buff[7]='x';
	  		 	   rx_buff[8]='x';
	  		 	  		  }


	  	 if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='S')&(rx_buff[4]=='E')&(rx_buff[5]=='T')&(rx_buff[6]=='D')&(rx_buff[7]=='I')&(rx_buff[8]=='S')){
	  			 HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin, GPIO_PIN_SET);
	  		 	HAL_UART_Transmit(&huart1, S4, 9, 1000);
	  		 	HAL_UART_Transmit(&huart1, SB, 2, 1000);
	  		  rx_buff[0]='x';
	  		 	   rx_buff[1]='x';
	  		 	   rx_buff[2]='x';
	  		 	   rx_buff[3]='x';
	  		 	   rx_buff[4]='x';
	  		 	   rx_buff[5]='x';
	  		 	   rx_buff[6]='x';
	  		 	   rx_buff[7]='x';
	  		 	   rx_buff[8]='x';
	  		 	  		  }




	  	 if((rx_buff[0]=='S')&(rx_buff[1]=='E')&(rx_buff[2]=='T')&(rx_buff[3]=='M')&(rx_buff[4]=='S')&(rx_buff[5]=='T')&(rx_buff[6]=='P')){
	  		 if((rx_buff[7]=='F')&(rx_buff[8]=='S')){

	  	 	HAL_GPIO_WritePin(MS1_GPIO_Port,MS1_Pin, GPIO_PIN_RESET);
	  	 	HAL_GPIO_WritePin(MS2_GPIO_Port,MS2_Pin, GPIO_PIN_RESET);
	  	 	HAL_GPIO_WritePin(MS3_GPIO_Port,MS3_Pin, GPIO_PIN_RESET);
	  	 	HAL_UART_Transmit(&huart1, S5, 10, 1000);
	  	 	HAL_UART_Transmit(&huart1, SB, 2, 1000);
	  		 }

	  		 if((rx_buff[7]=='H')&(rx_buff[8]=='S')){

	  			 HAL_GPIO_WritePin(MS1_GPIO_Port,MS1_Pin, GPIO_PIN_SET);
	  			 HAL_GPIO_WritePin(MS2_GPIO_Port,MS2_Pin, GPIO_PIN_RESET);
	  			 HAL_GPIO_WritePin(MS3_GPIO_Port,MS3_Pin, GPIO_PIN_RESET);
	  			 HAL_UART_Transmit(&huart1, S6, 10, 1000);
	  			HAL_UART_Transmit(&huart1, SB, 2, 1000);

	  		 		 }
	  		 if((rx_buff[7]=='Q')&(rx_buff[8]=='S')){

	  			 HAL_GPIO_WritePin(MS1_GPIO_Port,MS1_Pin, GPIO_PIN_RESET);
	  			 HAL_GPIO_WritePin(MS2_GPIO_Port,MS2_Pin, GPIO_PIN_SET);
	  			 HAL_GPIO_WritePin(MS3_GPIO_Port,MS3_Pin, GPIO_PIN_RESET);
	  			HAL_UART_Transmit(&huart1, S7, 10, 1000);
	  			HAL_UART_Transmit(&huart1, SB, 2, 1000);
	  		 		 }
	  		 if((rx_buff[7]=='E')&(rx_buff[8]=='S')){
	  			 HAL_GPIO_WritePin(MS1_GPIO_Port,MS1_Pin, GPIO_PIN_SET);
	  			 HAL_GPIO_WritePin(MS2_GPIO_Port,MS2_Pin, GPIO_PIN_SET);
	  			 HAL_GPIO_WritePin(MS3_GPIO_Port,MS3_Pin, GPIO_PIN_RESET);
	  			HAL_UART_Transmit(&huart1, S8, 10, 1000);
	  			HAL_UART_Transmit(&huart1, SB, 2, 1000);
	  		 		 }
	  		 if((rx_buff[7]=='S')&(rx_buff[8]=='S')){
	  			 HAL_GPIO_WritePin(MS1_GPIO_Port,MS1_Pin, GPIO_PIN_SET);
	  			 HAL_GPIO_WritePin(MS2_GPIO_Port,MS2_Pin, GPIO_PIN_SET);
	  			 HAL_GPIO_WritePin(MS3_GPIO_Port,MS3_Pin, GPIO_PIN_SET);
	  			 HAL_UART_Transmit(&huart1, S9, 10, 1000);
	  			HAL_UART_Transmit(&huart1, SB, 2, 1000);
	  		 		 }

	  		 rx_buff[0]='x';
	  		 rx_buff[1]='x';
	  		 rx_buff[2]='x';
	  		 rx_buff[3]='x';
	  	     rx_buff[4]='x';
	  		 rx_buff[5]='x';
	  		 rx_buff[6]='x';
	  		 rx_buff[7]='x';
	  		 rx_buff[8]='x';
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STP_Pin|MS1_Pin|MS2_Pin|MS3_Pin
                          |DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STP_Pin MS1_Pin MS2_Pin MS3_Pin
                           DIR_Pin */
  GPIO_InitStruct.Pin = STP_Pin|MS1_Pin|MS2_Pin|MS3_Pin
                          |DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_Pin */
  GPIO_InitStruct.Pin = EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIM1_Pin LIM2_Pin */
  GPIO_InitStruct.Pin = LIM1_Pin|LIM2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1, rx_buff, 11);
	if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='R')&(rx_buff[4]=='D')&(rx_buff[5]=='S')&(rx_buff[6]=='T')&(rx_buff[7]=='P')&(rx_buff[8]=='?')){
		  	 	HAL_UART_Transmit(&huart1, st, 5, 1);
		  	 	HAL_UART_Transmit(&huart1, SB, 2, 1000);
		  	       rx_buff[0]='x';
		  	 	   rx_buff[1]='x';
		  	 	   rx_buff[2]='x';
		  	 	   rx_buff[3]='x';
		  	 	   rx_buff[4]='x';
		  	 	   rx_buff[5]='x';
		  	 	   rx_buff[6]='x';
		  	 	   rx_buff[7]='x';
		  	 	   rx_buff[8]='x';
		  	 }
	if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='S')&(rx_buff[4]=='T')&(rx_buff[5]=='O')&(rx_buff[6]=='P')&(rx_buff[7]=='M')&(rx_buff[8]=='T')){
		stp=0;
			  	       rx_buff[0]='x';
			  	 	   rx_buff[1]='x';
			  	 	   rx_buff[2]='x';
			  	 	   rx_buff[3]='x';
			  	 	   rx_buff[4]='x';
			  	 	   rx_buff[5]='x';
			  	 	   rx_buff[6]='x';
			  	 	   rx_buff[7]='x';
			  	 	   rx_buff[8]='x';
			  	 }
 //You need to toggle a breakpoint on this line!
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_6)
	    {
		stp=0;
		 HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_RESET);
	    }
	if(GPIO_Pin == GPIO_PIN_7)
		    {
			stp=0;
			 HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_SET);
		    }
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
