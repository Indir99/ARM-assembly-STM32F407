/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t size =12;
//Duzina poruke je 12B, prvi bajt je primalac, drugi bajt je posiljalac, a ostalih 10 bajta je poruka
// TX bufferi, nizovi koji imaju 12 elemenata
uint8_t huart1_tx_buffer[12];
uint8_t huart2_tx_buffer[12];
uint8_t huart3_tx_buffer[12];

// RX bufferi, nizovi koji imaju 12 elemenata

uint8_t huart1_rx_buffer[12];
uint8_t huart2_rx_buffer[12];
uint8_t huart3_rx_buffer[12];


// RX bufferi, nizovi koji imaju 12 elemenata


// Pomocni flagovi koji ce nam omoguciti pravilnu kontrolu toka programa
uint8_t tx1_flag = 0;
uint8_t tx2_flag = 0;
uint8_t tx3_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
		{
		// Ukoliko USART1 salje poruku, prvo provjeravamo kome salje
		// tako sto mu provjeravamo prvi element niza
		// ako je prvi element 2 salje se poruka na usart 2
		// ako je prvi element 3 salje se poruka na usart 6
			if(huart1_rx_buffer[0]=='2')
				{
		// Petlja prolazi kroz rx_buffer usarta 1 i smijesta ga u usart 2
				for(int i = 0; i < size; i++)
					huart2_tx_buffer[i] = huart1_rx_buffer[i];
		// Setujemo flag na 1 da bismo znali da je poruka primljena
				tx2_flag = 1;
				}

				if(huart1_rx_buffer[0]=='3')
				{
		// Petlja prolazi kroz rx_buffer usarta 1 i smijesta ga u usart 6
				for(int i = 0; i < size; i++)
					huart3_tx_buffer[i] = huart1_rx_buffer[i];
		// Setujemo flag na 1 da bismo znali da je poruka primljena
				tx3_flag = 1;
				}
		}

		if (huart->Instance == USART2)
		{
		// Isti postupak kao u prethodnom slucaju, samo je ovaj put USART2
		// onaj koji salje poruku, a USART 1 i USART 6 primaju poruku

			if(huart2_rx_buffer[0]=='1')
				{
				for(int i = 0; i < size; i++)
					huart1_tx_buffer[i] = huart2_rx_buffer[i];
				tx1_flag = 1;
				}

			if(huart2_rx_buffer[0]=='3')
			{
			for(int i = 0; i < size; i++)
				huart3_tx_buffer[i] = huart2_rx_buffer[i];
			tx3_flag = 1;
			}
		}

		if (huart->Instance == USART6)
		{

		// Isti postupak kao u prethodnom slucaju, samo je ovaj put USART6
		// onaj koji salje poruku, a USART 1 i USART 2 primaju poruku

			if(huart3_rx_buffer[0]=='1')
				{
				for(int i = 0; i < size; i++)
					huart1_tx_buffer[i] = huart3_rx_buffer[i];
				tx1_flag = 1;
				}

			if(huart3_rx_buffer[0]=='2')
				{
				for(int i = 0; i < size; i++)
					huart2_tx_buffer[i] = huart3_rx_buffer[i];
				tx2_flag = 1;
				}
		}
};

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
		{
			// transmit complete callback za UART 1
			// Prebacivanje podataka iz tx_buffera u rx_buffer i
			// nakon toga cistimo tx_buffer te ga oslobadjamo za novu poruku
			for(int i = 0; i < size; i++)
			{
				huart1_rx_buffer[i] = huart1_tx_buffer[i];
				huart1_tx_buffer[i] = 0;
			}
			tx1_flag = 0;
		}

		if (huart->Instance == USART2)
		{
			// transmit complete callback za UART 2
			for(int i = 0; i < size; i++)
			{
				huart2_rx_buffer[i] = huart2_tx_buffer[i];
				huart2_tx_buffer[i] = 0;
			}
			tx2_flag = 0;
		}

		if (huart->Instance == USART6)
		{
		// transmit complete callback za UART 6

			for(int i = 0; i < size; i++)
			{
				huart3_rx_buffer[i] = huart3_tx_buffer[i];
				huart3_tx_buffer[i] = 0;
			}
			tx3_flag = 0;
		}
};

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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  // Potrebno je da omogucimo interrupt za svaki UART
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

// NAPOMENA: BaudRate je 115200 za svaki usart
  while (1)
  {
    /* USER CODE END WHILE */

	// Provjeravamo da li usart zauzet
	  if(huart1.RxState != HAL_UART_STATE_BUSY_RX)
	  			HAL_UART_Receive_IT(&huart1,huart1_rx_buffer,sizeof(huart1_rx_buffer));
	  	if(huart2.RxState != HAL_UART_STATE_BUSY_RX)
	  			HAL_UART_Receive_IT(&huart2,huart2_rx_buffer,sizeof(huart2_rx_buffer));
	  	if(huart6.RxState != HAL_UART_STATE_BUSY_RX)
	  			HAL_UART_Receive_IT(&huart6,huart3_rx_buffer,sizeof(huart3_rx_buffer));


	  // Provjeravamo da li usart zauzet i da li imamo spremnu poruku
	  	if(huart1.gState != HAL_UART_STATE_BUSY_TX && tx1_flag)
	  			HAL_UART_Transmit_IT(&huart1,huart1_tx_buffer,sizeof(huart1_tx_buffer));
	  	if(huart2.gState != HAL_UART_STATE_BUSY_TX && tx2_flag)
	  			HAL_UART_Transmit_IT(&huart2,huart2_tx_buffer,sizeof(huart2_tx_buffer));
	  	if(huart6.gState != HAL_UART_STATE_BUSY_TX && tx3_flag)
	  			HAL_UART_Transmit_IT(&huart6,huart3_tx_buffer,sizeof(huart3_tx_buffer));

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
