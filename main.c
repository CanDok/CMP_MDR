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
uint8_t S1[] = {"CCW SET\n\r"};
uint8_t S2[] = {"CW SET\n\r"};
uint8_t S3[] = {"OK\n\r"};
uint8_t S4[] = {"SET!\n\r"};
uint8_t S5[] = {"test1234\n\r"};
uint8_t S6[] = {"test1234\n\r"};
int stp =9;
int delay=100;
uint8_t rx_buff[6];
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(HAL_UART_Receive(&huart1, rx_buff, 9, 1000)==HAL_OK) //if transfer is successful
	    {



	 if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='C')&(rx_buff[4]=='S')&(rx_buff[5]=='T')&(rx_buff[6]=='A')&(rx_buff[7]=='R')&(rx_buff[8]=='T')){


		 for( uint32_t i=0;i<stp;i++){
		 //HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_SET);
		 HAL_Delay(delay);
	 		  		  }
	 }




if((rx_buff[0]=='S')&(rx_buff[1]=='E')&(rx_buff[2]=='T')&(rx_buff[3]=='S')&(rx_buff[4]=='P')){
	HAL_UART_Transmit(&huart1, rx_buff, 10, 1000);

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


}
if((rx_buff[0]=='S')&(rx_buff[1]=='E')&(rx_buff[2]=='T')&(rx_buff[3]=='D')&(rx_buff[4]=='L')){
	HAL_UART_Transmit(&huart1, rx_buff, 10, 1000);

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


}




if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='S')&(rx_buff[4]=='E')&(rx_buff[5]=='T')&(rx_buff[6]=='D')&(rx_buff[7]=='R')&(rx_buff[8]=='-')){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, S2, 10, 1000);
}




if((rx_buff[0]=='S')&(rx_buff[1]=='S')&(rx_buff[2]=='M')&(rx_buff[3]=='S')&(rx_buff[4]=='E')&(rx_buff[5]=='T')&(rx_buff[6]=='D')&(rx_buff[7]=='R')&(rx_buff[8]=='+')){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_RESET);
	HAL_UART_Transmit(&huart1, S1, 10, 1000);
	  		  }

if((rx_buff[0]=='S')&(rx_buff[1]=='E')&(rx_buff[2]=='T')&(rx_buff[3]=='M')&(rx_buff[4]=='S')&(rx_buff[4]=='T')){
	HAL_UART_Transmit(&huart1, rx_buff, 10, 1000);

	//char b1= rx_buff[5];
	//char b2= rx_buff[6];
	//char b3= rx_buff[7];



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
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_RESET);
 HAL_Delay(1000);
 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
 /* if(GPIO_Pin == GPIO_PIN_6) {
 HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_SET);
 HAL_Delay(1000);
 HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_RESET);
 HAL_Delay(1000);
 HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);

  } else {
      __NOP();
  }*/
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
