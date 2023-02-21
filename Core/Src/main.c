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
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static cli_status_t help_func(int argc, char **argv);
cli_status_t clear_flash(int argc, char **argv);
cli_status_t load_over_xmodem(int argc, char **argv);
cli_status_t boot_app(int argc, char **argv);
void user_uart_println(char *string);
cmd_t cmd_tbl[] = {
    {
        .cmd = "help",
        .func = help_func
    },
    {
      .cmd = "clear_flash",
      .func = clear_flash
    },
    {
      .cmd = "xmodem_flash",
      .func = load_over_xmodem
    },
    {
      .cmd = "boot",
      .func = boot_app
    }
};

cli_t cli;
uint8_t rx_buffer;
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
  cli.println = user_uart_println;
  cli.cmd_tbl = cmd_tbl;
  cli.cmd_cnt = sizeof(cmd_tbl)/sizeof(cmd_t);
  cli_init(&cli);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  UART_Start_Receive_IT(&huart1, &rx_buffer, 1);
  cli.println("Welcome to simple bootloader! \n\r");
  cli.println("Press any key to stop at bootloader \r\n");
  int tick = 3;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    cli_process(&cli);

    // HAL_UART_Transmit(&huart1, "ahihi\n\r", 9, 1000);
    // HAL_Delay(1000);

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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  // HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // Do something with the received data
    // ...
    // uint8_t rxData;
    // char c = huart1.Instance->DR;

    // if (rxData == '\r') {
    //   HAL_UART_Transmit(&huart1, "\r\n", 2, 1000);
    // } else {
      HAL_UART_Transmit(&huart1, &rx_buffer, 1, 1000);    
    // }
    // HAL_UART_Receive_IT(huart, &rxData, 1);
    // HAL_UART_Transmit(huart, &rxData, 1, 10000);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // HAL_UART_Receive_IT(&huart1, &data, 1);
    // __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
    // data = huart1.Instance->DR;
    // int i = 0;
    cli_put(&cli, rx_buffer);

    // if (rxData == '\r') {
    // HAL_UART_Transmit(&huart1, "\r\n", 2, 1000);
    // } else {
    // HAL_UART_Transmit(&huart1, &data, 1, 1000);  
    HAL_UART_Receive_IT(&huart1, &rx_buffer, 1);
  }
}

void user_uart_println(char *string)
{
    /* For example.. */
    HAL_UART_Transmit(&huart1, string, strlen(string),1000);
}

cli_status_t help_func(int argc, char **argv)
{
    cli.println("\
    Current support command\n\r\                                     
    clear_flash: clear all application flash\n\r\
    xmodem_flash: flash new application use xmodem protocol\n\r\
    boot: jump to application\n\r");
    return CLI_OK;
}

cli_status_t clear_flash(int argc, char **argv)
{
  cli.println("Clear Flash start\n\r");
 	FLASH_EraseInitTypeDef hFlashErase;
	uint32_t sectorError;
	HAL_StatusTypeDef status;

  hFlashErase.TypeErase = FLASH_TYPEERASE_SECTORS;
  hFlashErase.Sector = 2; // start sector
  hFlashErase.NbSectors = 6;
  hFlashErase.Banks = FLASH_BANK_1;
  hFlashErase.VoltageRange = FLASH_VOLTAGE_RANGE_3;  
  HAL_FLASH_Unlock();
  status = (uint8_t) HAL_FLASHEx_Erase(&hFlashErase, &sectorError);
  HAL_FLASH_Lock();

  cli.println("Clear Flash successful\n\r");
}

cli_status_t program_flash(int argc, char **argv)
{

}


xmodem_status_t xmodem_callback(uint8_t *data,uint32_t data_length, uint8_t package_count) {
  HAL_FLASH_Unlock(); // TODO: check size bla bla
  for (uint32_t cnt =0; cnt < data_length; cnt ++){
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, APP_BASE_ADDR + 128 * (package_count - 1) + cnt, data[cnt] );
  }
  HAL_FLASH_Lock();
}
typedef void (*func_ptr_t)(void);

cli_status_t boot_app(int argc, char **argv) {

  func_ptr_t func_ptr;
  func_ptr = (func_ptr_t)(*(volatile uint32_t*) (APP_BASE_ADDR+4u));
  uint32_t MSP_val = *(volatile uint32_t*)APP_BASE_ADDR;
  HAL_DeInit();
  __set_MSP(MSP_val);
  func_ptr();
}

xmodem_status_t xmodem_send_data(uint8_t *data, uint32_t length) {
  HAL_UART_Transmit(&huart1, data, length, HAL_MAX_DELAY); 
}

xmodem_status_t xmodem_receive_data (uint8_t* data, uint32_t length) {
  HAL_UART_Receive(&huart1, data, length , 1000);
}

cli_status_t load_over_xmodem(int argc, char **argv)
{
  xmodem_t xmodem;
  xmodem.xmodem_receive_done_cb = xmodem_callback;
  xmodem.xmodem_send_data = xmodem_send_data;
  xmodem.xmodem_receive_data = xmodem_receive_data;
  init_xmodem(&xmodem);
  HAL_UART_AbortReceive_IT(&huart1);

  // start_receive_data(&xmodem);
  
  uint8_t buff;
  process_receive_data(&xmodem, buff);

  // after done ,restart to normal
  HAL_UART_Receive_IT(&huart1, &rx_buffer, 1);
  cli.println("Flash done\n\r");
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
