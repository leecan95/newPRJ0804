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
#include "string.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define BUFSIZE 10

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

/* USER CODE BEGIN PV */
uint8_t rx_buff[BUFSIZE] = {0,};
uint16_t rx_buff_len;
uint8_t test_buff[20] = {0,};
char  Rx_data[2],Rx_data2[2], Rx_Buffer[40], Transfer_cplt, Rx_Buffer2[40], Transfer_cplt2;
int Rx_indx = 0, Rx_indx2 = 0;
uint8_t rxBuffer[4], rxBuffer2[4];
uint8_t Header[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
 /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
 HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);


 return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		rx_buff_len = BUFSIZE - huart->RxXferCount;
		uint8_t res = HAL_UART_Transmit_IT(&huart1, (uint8_t*)rx_buff, rx_buff_len);
		if(res == HAL_ERROR) HAL_UART_Transmit(&huart2, (uint8_t*)"HAL_ERROR - rx_buff == NULL or rx_buff_len == 0\n", 48, 1000);
		else if(res == HAL_BUSY) HAL_UART_Transmit(&huart2, (uint8_t*)"HAL_BUSY\n", 9, 1000);
		HAL_UART_AbortReceive(&huart1);
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buff, BUFSIZE);
	}
}

///////////////////////////////////  ///////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	  if(huart == &huart1)
	  {
		  __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		  HAL_UART_Transmit_IT(&huart1, (uint8_t*)"Full buffer\n", 12);
		  HAL_UART_Transmit_IT(&huart2, (uint8_t*)"Full buffer\n", 12);
		  HAL_UART_AbortReceive(&huart1);
		  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
		  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		  HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buff, BUFSIZE);
	  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		//HAL_GPIO_TogglePin(pc13_GPIO_Port, pc13_Pin);
		uint32_t er = HAL_UART_GetError(&huart1);
		HAL_UART_Abort_IT(&huart1);

		switch(er)
		{
			case HAL_UART_ERROR_PE:
				HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - Parity error\n", 27, 1000);
				HAL_UART_Transmit(&huart2, (uint8_t*)"ERR_Callbck - Parity error\n", 27, 1000);
				__HAL_UART_CLEAR_PEFLAG(&huart1);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			case HAL_UART_ERROR_NE:
				HAL_UART_Transmit(&huart2, (uint8_t*)"ERR_Callbck - Noise error\n", 26, 1000);

				__HAL_UART_CLEAR_NEFLAG(&huart1);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			case HAL_UART_ERROR_FE:
				HAL_UART_Transmit(&huart2, (uint8_t*)"ERR_Callbck - Frame error\n", 26, 1000);
				__HAL_UART_CLEAR_FEFLAG(&huart1);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			case HAL_UART_ERROR_ORE:
				HAL_UART_Transmit(&huart2, (uint8_t*)"ERR_Callbck - Overrun error\n", 28, 1000);
				__HAL_UART_CLEAR_OREFLAG(huart);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			case HAL_UART_ERROR_DMA:
				HAL_UART_Transmit(&huart2, (uint8_t*)"ERR_Callbck - DMA transfer error\n", 33, 1000);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			default:
			break;
		}
	}

}**/



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
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, Rx_data, 1);
  HAL_UART_Receive_IT(&huart2, Rx_data2, 1);
  //__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
   // HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buff, BUFSIZE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	  //printf("Hello World\r\n");
	  //printf("%s", Rx_Buffer);
	  //sprintf(test_buff,"gia tri rx %s\r\n",rx_buff);
	  //HAL_UART_Transmit(&huart1, (uint8_t*)"Lee", 5, 1000);
	 HAL_UART_Transmit(&huart1, (uint8_t*)Rx_Buffer, sizeof(Rx_Buffer), 1000);
	 // HAL_UART_Transmit(&huart1, (uint8_t*)Rx_Buffer2, sizeof(Rx_Buffer2), 1000);
	  HAL_Delay(1000);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}



/**
  * @brief System Clock Configuration
  * @retval None
  */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;
   // printf("dang nhan \r\n");
    if (huart->Instance == USART1)  //Xét UART nhận dữ liệu
        {
        if (Rx_indx==0) {
        	for (i=0;i<100;i++) Rx_Buffer[i]=0;
        }   //clear Rx_buffer trước khi nhận dữ liệu mới
       // else if (sizeof(Rx_Buffer > 40)){
        //	for (i=0;i<100;i++) Rx_Buffer[i]=0;
     //   }


        /*HAL_UART_Receive(&huart1,Header,9,0xFFFF);
        if (Header[0] == 'R'){
        	if(Header[1]=='0'){
				 HAL_UART_Transmit(&huart1, (uint8_t*)"Oke\r\n", 10, 1000);
				 Rx_indx=0;
			}
			else{
				HAL_UART_Transmit(&huart1, (uint8_t*)"Fail\r\n", 10, 1000);
				Rx_indx=0;
			}
        }*/
        if (Rx_data[0] == 'R') {
        	HAL_UART_Receive(&huart1,Header,8,0xFFFF);
        	if(Header[0]=='0'){
				 HAL_UART_Transmit(&huart1, (uint8_t*)"Oke\r\n", 5, 1000);
				 Rx_indx=0;
			}
		else{
			HAL_UART_Transmit(&huart1, (uint8_t*)"Fail\r\n", 6, 1000);
			Rx_indx=0;
			}

        }
        if (Rx_data[0]!='\n' && Rx_data[0]!= 'R') //Nếu nhận dữ liệu là khác dấu xuống dòng
            {

            Rx_Buffer[Rx_indx++]=Rx_data[0];  //thêm dữ liệu vào Rx_Buffer
            char test_rx = Rx_data[0];
            HAL_UART_Transmit(&huart1,(uint8_t*)test_rx, 1,0xFFFF);
            }
        /*else if (Rx_data[0] == 'R'){
        	HAL_UART_Receive(&huart1,Rx_data2,1,100);
        	if(Rx_data2[0]=='0'){
        		 HAL_UART_Transmit(&huart1, (uint8_t*)"Oke\r\n", 10, 1000);
        		 Rx_indx=0;
        	}
        	else{
        		HAL_UART_Transmit(&huart1, (uint8_t*)"Fail\r\n", 10, 1000);
        		Rx_indx=0;
        	}
        }*/
        else            //nếu là dấu xuống dòng \n thì hoàn thành việc đọc 1 khung truyền
            {
            Rx_indx=0;
            Transfer_cplt=1;//Cờ báo hiệu đã chuyển dữ liệu xong và tiến hành đọc dữ liệu
            }
        //HAL_UART_Transmit(&huart1, (uint8_t*)Rx_data, sizeof(Rx_data), 5000);
        HAL_UART_Receive_IT(&huart1, Rx_data, 1);   //Kích hoạt ngắt UART mỗi data nhận được
        }

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
