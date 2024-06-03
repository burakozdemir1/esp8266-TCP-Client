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
#include <string.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
void UART_SendString(UART_HandleTypeDef *huart, const char *str);
void UART_ReceiveString(UART_HandleTypeDef *huart, char *buffer, uint16_t buffer_size);
void ESP8266_SendATCommand(UART_HandleTypeDef *huart, const char *cmd, char *response, uint16_t response_size);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rxBuffer[500] = {0};
uint8_t ATisOK;
char ATcommand[64];
char myData[64];
uint32_t timeout = 6000;
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
//  ESP8266_Init(&huart2);
 // ESP8266_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /*
__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE); //rx interrupt aktif edildi


HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_buffer, 1);
start:
	if(ESP_INIT()) goto start; //esp den istediğin cevap gelmediğinde bu start a yani ESP_INIT()' e tekrar döner

*/


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // AT komutu gönderme ve cevabı okuma
//  uint8_t rxBuffer[512] = {0};
//    uint8_t ATisOK;
//    int channel;
//    int onoff;
//    int led = 1;
//    char ATcommand[64];
//    char ATcommandB[1024];
//    char ATcommandN[100];
//    char ATcommandF[100];
//    char ATcommandT[16];
//    sprintf(ATcommandB,"<!DOCTYPE html><html>\n<head>\n\
//    <title>STM32 - ESP8266</title>\n<link href=\"data:image/x-icon;base64,\
//    A\" rel=\"icon\" type=\"image/x-icon\"><style>\nhtml {\
//    display: inline-block; margin: 0px auto; text-align: center;}\n\
//    body{margin-top: 50px;}\n.button {display: block;\n\
//    width: 70px;\nbackground-color: #008000;\nborder: none;\ncolor: white;\n\
//    padding: 14px 28px;\ntext-decoration: none;\nfont-size: 24px;\n\
//    margin: 0px auto 36px; \nborder-radius: 5px;}\n\
//    .button-on {background-color: #008000;}\n.button-on:active\
//    {background-color: #008000;}\n.button-off {background-color: #808080;}\n\
//    .button-off:active {background-color: #808080;}\n\
//    p {font-size: 14px;color: #808080;margin-bottom: 20px;}\n\
//    </style>\n</head>\n<body>\n<h1>STM32 - ESP8266</h1>");
//    sprintf(ATcommandN,"<p>Light is currently on\
//    </p><a class=\"button button-off\" href=\"/lightoff\">OFF</a>");
//    sprintf(ATcommandF,"<p>Light is currently off\
//    </p><a class=\"button button-on\" href=\"/lighton\">ON</a>");
//    sprintf(ATcommandT,"</body></html>");
//    int countB = strlen(ATcommandB);
//    int countN = strlen(ATcommandN);
//    int countF = strlen(ATcommandF);
//    int countT = strlen(ATcommandT);
//
//    sprintf(ATcommand,"AT+RST\r\n");
//    memset(rxBuffer,0,sizeof(rxBuffer));
//    HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//    HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
//    HAL_Delay(500);
//
//    ATisOK = 0;
//    while(!ATisOK){
//      sprintf(ATcommand,"AT+CWMODE_CUR=2\r\n");
//        memset(rxBuffer,0,sizeof(rxBuffer));
//        HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//        HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
//      if(strstr((char *)rxBuffer,"OK")){
//        ATisOK = 1;
//      }
//      HAL_Delay(500);
//    }
//
//    ATisOK = 0;
//    while(!ATisOK){
//      sprintf(ATcommand,"AT+CWSAP_CUR=\"STM32\",\"12345678\",1,3,4,0\r\n");
//        memset(rxBuffer,0,sizeof(rxBuffer));
//        HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//        HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
//      if(strstr((char *)rxBuffer,"OK")){
//        ATisOK = 1;
//      }
//      HAL_Delay(500);
//    }
//
//    ATisOK = 0;
//    while(!ATisOK){
//      sprintf(ATcommand,"AT+CIPAP_CUR=\"192.168.51.1\"\r\n");
//      memset(rxBuffer,0,sizeof(rxBuffer));
//      HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//      HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
//      if(strstr((char *)rxBuffer,"OK")){
//        ATisOK = 1;
//      }
//      HAL_Delay(500);
//    }
//
//    ATisOK = 0;
//    while(!ATisOK){
//      sprintf(ATcommand,"AT+CIPMUX=1\r\n");
//        memset(rxBuffer,0,sizeof(rxBuffer));
//        HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//        HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
//        if(strstr((char *)rxBuffer,"OK")){
//          ATisOK = 1;
//        }
//        HAL_Delay(500);
//    }
//
//    ATisOK = 0;
//    while(!ATisOK){
//      sprintf(ATcommand,"AT+CIPSERVER=1,80\r\n");
//      memset(rxBuffer,0,sizeof(rxBuffer));
//      HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//      HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
//      if(strstr((char *)rxBuffer,"OK")){
//          ATisOK = 1;
//      }
//      HAL_Delay(500);
//    }

  sprintf(ATcommand, "AT+RST\r\n");
  memset(rxBuffer, 0, sizeof(rxBuffer));
  HAL_UART_Transmit(&huart1, (uint8_t*)ATcommand, strlen(ATcommand), 1000);
  HAL_UART_Receive(&huart1, (uint8_t*)rxBuffer, sizeof(rxBuffer), 1000);
  HAL_Delay(500);  // Yanıtın gelmesi için bekleme

  // AT komutunu gönderme ve yanıtı kontrol etme
//  while (!ATisOK)
//  {
//    sprintf(ATcommand, "AT\r\n");
//    memset(rxBuffer, 0, sizeof(rxBuffer));
//
//    if (HAL_UART_Transmit(&huart1, (uint8_t*)ATcommand, strlen(ATcommand), 1000) != HAL_OK)
//    {
//      // Handle transmission error
//    }
//
//    uint32_t start_time = HAL_GetTick(); // Start timeout timer
//    while (HAL_UART_Receive(&huart1, rxBuffer, sizeof(rxBuffer), timeout) != HAL_OK)
//    {
//      if (HAL_GetTick() - start_time >= timeout) {
//        // Handle timeout error
//        break;
//      }
//    }
//
//    if (strstr((char*)rxBuffer, "OK"))
//    {
//      ATisOK = 1;
//    } else
//    {
//      // Handle unexpected response
//    }
//
//    HAL_Delay(500); // Or adjust based on your needs
//  }


  ATisOK = 0;
  while (!ATisOK)
  {
	//sprintf(ATcommand, "AT+CIPSTART=\"TCP\",\"192.168.1.103\",8080\r\n");
	sprintf(ATcommand, "AT+CWMODE=1\r\n");
    memset(rxBuffer, 0, sizeof(rxBuffer));

    if (HAL_UART_Transmit(&huart1, (uint8_t*)ATcommand, strlen(ATcommand), 1000) != HAL_OK)
    {
      // Handle transmission error
    }

    uint32_t start_time = HAL_GetTick(); // Start timeout timer
    while (HAL_UART_Receive(&huart1, rxBuffer, sizeof(rxBuffer), timeout) != HAL_OK)
    {
      if (HAL_GetTick() - start_time >= timeout) {
        // Handle timeout error
        break;
      }
    }

    if (strstr((char*)rxBuffer, "OK"))
    {
      ATisOK = 1;
    } else
    {
      // Handle unexpected response
    }

    HAL_Delay(500); // Or adjust based on your needs
  }


  ATisOK = 0;
  while (!ATisOK)
  {
    sprintf(ATcommand, "AT+CWJAP=\"TurkTelekom_T35F8\",\"6tZhxeGz\"\r\n");
    memset(rxBuffer, 0, sizeof(rxBuffer));

    if (HAL_UART_Transmit(&huart1, (uint8_t*)ATcommand, strlen(ATcommand), 1000) != HAL_OK)
    {
      // Handle transmission error
    }

    uint32_t start_time = HAL_GetTick(); // Start timeout timer
    while (HAL_UART_Receive(&huart1, rxBuffer, sizeof(rxBuffer), timeout) != HAL_OK)
    {
      if (HAL_GetTick() - start_time >= timeout)
      {
        // Handle timeout error
        break;
      }
    }

    // Check for expected response based on your module's documentation
    if (strstr((char*)rxBuffer, "WIFI CONNECTED"))
    {
    	if (strstr((char*)rxBuffer, "WIFI GOT IP"))
    	{
    		if (strstr((char*)rxBuffer, "OK"))
    		{
    			ATisOK = 1;
    		}

    	}





    } else
    {
      // Handle unexpected response (print rxBuffer for debugging)
    }

    HAL_Delay(500); // Or adjust based on your needs
  }




  ATisOK = 0;
  while (!ATisOK)
  {
	sprintf(ATcommand, "AT+CIPSTART=\"TCP\",\"192.168.1.103\",8080\r\n");
    memset(rxBuffer, 0, sizeof(rxBuffer));

    if (HAL_UART_Transmit(&huart1, (uint8_t*)ATcommand, strlen(ATcommand), 1000) != HAL_OK)
    {
      // Handle transmission error
    }

    uint32_t start_time = HAL_GetTick(); // Start timeout timer
    while (HAL_UART_Receive(&huart1, rxBuffer, sizeof(rxBuffer), timeout) != HAL_OK)
    {
      if (HAL_GetTick() - start_time >= timeout)
      {
        // Handle timeout error
        break;
      }
    }

    // Check for expected response based on your module's documentation
    if (strstr((char*)rxBuffer, "CONNECT"))
    {

    	if (strstr((char*)rxBuffer, "OK"))
    	{
    		ATisOK = 1;
    	}


    }
    else if (strstr((char*)rxBuffer, "CLOSED") || strstr((char*)rxBuffer, "ERROR")  ) // server closed
	{

	}
    else
    {
      // Handle unexpected response (print rxBuffer for debugging)
    }

    HAL_Delay(500); // Or adjust based on your needs
  }



  ATisOK = 0;
  while (!ATisOK)
  {

	sprintf(ATcommand, "AT+CIPMODE=1\r\n"); // Transparan mode on
    memset(rxBuffer, 0, sizeof(rxBuffer));

    if (HAL_UART_Transmit(&huart1, (uint8_t*)ATcommand, strlen(ATcommand), 1000) != HAL_OK)
    {
      // Handle transmission error
    }

    uint32_t start_time = HAL_GetTick(); // Start timeout timer
    while (HAL_UART_Receive(&huart1, rxBuffer, sizeof(rxBuffer), timeout) != HAL_OK)
    {
      if (HAL_GetTick() - start_time >= timeout) {
        // Handle timeout error
        break;
      }
    }

    if (strstr((char*)rxBuffer, "OK"))
    {
      ATisOK = 1;
    }
    else
    {
      // Handle unexpected response
    }

    HAL_Delay(500); // Or adjust based on your needs
  }

  ATisOK = 0;
  while (!ATisOK)
  {

	sprintf(ATcommand, "AT+CIPSEND\r\n"); // Transparan mode on

	sprintf(myData, "Hello Burak!\r\n");
    memset(rxBuffer, 0, sizeof(rxBuffer));

    if (HAL_UART_Transmit(&huart1, (uint8_t*)ATcommand, strlen(ATcommand), 1000) != HAL_OK)
    {
      // Handle transmission error
    }

    uint32_t start_time = HAL_GetTick(); // Start timeout timer
    while (HAL_UART_Receive(&huart1, rxBuffer, sizeof(rxBuffer), timeout) != HAL_OK)
    {
      if (HAL_GetTick() - start_time >= timeout) {
        // Handle timeout error
        break;
      }
    }

    if (strstr((char*)rxBuffer, "OK"))
    {
    	if (strstr((char*)rxBuffer, ">"))
    	{
    	    if (HAL_UART_Transmit(&huart1, (uint8_t*)myData, strlen(myData), 1000) != HAL_OK)
    	    {
    	      // Handle transmission error
    	    }

    		ATisOK = 1;
    	}

    }
    else
    {
      // Handle unexpected response
    }

    HAL_Delay(500); // Or adjust based on your needs
  }











  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

















//	    memset(rxBuffer,0,sizeof(rxBuffer));
//	    HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
//	    if(strstr((char *)rxBuffer,"+IPD,0")) channel = 0;
//	    else if(strstr((char *)rxBuffer,"+IPD,1")) channel = 1;
//	    else if(strstr((char *)rxBuffer,"+IPD,2")) channel = 2;
//	    else if(strstr((char *)rxBuffer,"+IPD,3")) channel = 3;
//	    else if(strstr((char *)rxBuffer,"+IPD,4")) channel = 4;
//	    else if(strstr((char *)rxBuffer,"+IPD,5")) channel = 5;
//	    else if(strstr((char *)rxBuffer,"+IPD,6")) channel = 6;
//	    else if(strstr((char *)rxBuffer,"+IPD,7")) channel = 7;
//	    else channel = 100;
//
//	    if(strstr((char *)rxBuffer,"GET /lighton")) onoff = 0;
//	    else if(strstr((char *)rxBuffer,"GET /lightoff")) onoff = 1;
//	    else onoff = led;
//
//	    if(channel<8 && onoff == 1)
//	    {
//	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	      led = 1;
//	      sprintf(ATcommand,"AT+CIPSEND=%d,%d\r\n",channel,countB+countF+countT);
//	      memset(rxBuffer,0,sizeof(rxBuffer));
//	      HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//	      HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
//	      if(strstr((char *)rxBuffer,">"))
//	      {
//	        memset(rxBuffer,0,sizeof(rxBuffer));
//	          HAL_UART_Transmit(&huart1,(uint8_t *)ATcommandB,countB,1000);
//	          HAL_UART_Transmit(&huart1,(uint8_t *)ATcommandF,countF,1000);
//	          HAL_UART_Transmit(&huart1,(uint8_t *)ATcommandT,countT,1000);
//	         HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
//	      }
//	      sprintf(ATcommand,"AT+CIPCLOSE=%d\r\n",channel);
//	      memset(rxBuffer,0,sizeof(rxBuffer));
//	      HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//	      HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
//	      channel=100;
//	    }
//	    else if(channel<8 && onoff == 0)
//	    {
//	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//	      led = 0;
//	      sprintf(ATcommand,"AT+CIPSEND=%d,%d\r\n",channel,countB+countN+countT);
//	      memset(rxBuffer,0,sizeof(rxBuffer));
//	      HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//	      HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
//	      if(strstr((char *)rxBuffer,">"))
//	      {
//	        memset(rxBuffer,0,sizeof(rxBuffer));
//	          HAL_UART_Transmit(&huart1,(uint8_t *)ATcommandB,countB,1000);
//	          HAL_UART_Transmit(&huart1,(uint8_t *)ATcommandN,countN,1000);
//	          HAL_UART_Transmit(&huart1,(uint8_t *)ATcommandT,countT,1000);
//	          HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
//	      }
//	      sprintf(ATcommand,"AT+CIPCLOSE=%d\r\n",channel);
//	      memset(rxBuffer,0,sizeof(rxBuffer));
//	      HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//	      HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
//	      channel=100;
//	    }

	//  ESP8266_SendATCommand(&huart1, "AT+RST\r\n", response, RX_BUFFER_SIZE);
	  //HAL_Delay(500); // ESP8266'nın resetlenmesi için bekleme
	 // ESP8266_SendATCommand(&huart1, "AT\r\n", response, RX_BUFFER_SIZE);
	/*  ESP8266_SendATCommand(&huart1, "AT+CWMODE_CUR=2\r\n", response, RX_BUFFER_SIZE);
	  ESP8266_SendATCommand(&huart1, "AT+CWSAP_CUR=\"STM32\",\"12345678\",1,3,4,0\r\n", response, RX_BUFFER_SIZE);
	  ESP8266_SendATCommand(&huart1, "AT+CIPAP_CUR=\"192.168.51.1\"\r\n", response, RX_BUFFER_SIZE);
	  ESP8266_SendATCommand(&huart1, "AT+CIPMUX=1\r\n", response, RX_BUFFER_SIZE);
	  ESP8266_SendATCommand(&huart1, "AT+CIPSERVER=1,80\r\n", response, RX_BUFFER_SIZE);


*/













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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
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
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
