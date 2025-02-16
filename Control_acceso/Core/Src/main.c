/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdbool.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "keypad.h"
#include "ring_buffer.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_byte; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t key_pressed_tick = 0;
uint16_t column_pressed = 0;

uint32_t debounce_tick = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((HAL_GetTick() - debounce_tick) < 200) { // 150 ms timeout
    return;
  }
  debounce_tick = HAL_GetTick();
  key_pressed_tick = HAL_GetTick();
  column_pressed = GPIO_Pin;
}
// Command definitions --------------------------------
#define PASSWORD_LENGTH 5
#define COMMAND_LENGTH 1
#define PASSWORD "12345"
typedef enum {
  CMD_OPEN,
  CMD_CLOSE,
  CMD_STATUS,
  CMD_CLEAR
} command_t;

bool is_authenticated = false;  // Estado de autenticación

char password_entered[PASSWORD_LENGTH + 1];
uint8_t password_index = 0;

ring_buffer_t rx_buffer;
uint8_t rx_buffer_mem[64];
char current_cmd[COMMAND_LENGTH];
uint8_t cmd_index = 0;

// Function to check if the entered password is correct
bool check_password(const char *entered_password) {
  return strcmp(entered_password, PASSWORD) == 0;
}

// UART asociated Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
      ring_buffer_write(&rx_buffer, rx_byte);
      HAL_UART_Transmit(&huart2, &rx_byte, 1, 100); 
      HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }
}

// New helper function
void uart_send_string(const char *str) {
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

// Function to display the menu
void display_menu(void) {
  uart_send_string("\r\nMenu de opciones:\r\n");
  uart_send_string("A: Abrir puerta\r\n");
  uart_send_string("C: Cerrar puerta\r\n");
  uart_send_string("1: Estado de la puerta\r\n");
  uart_send_string("0: Bloquear sistema\r\n");
}


void process_commands(void) {
  uint8_t byte;
  while (ring_buffer_read(&rx_buffer, &byte)) {
      // Shift buffer left and append new byte
      memmove(password_entered, password_entered + 1, PASSWORD_LENGTH - 1);
      password_entered[PASSWORD_LENGTH - 1] = (char)byte;
      

      if (!is_authenticated) {
          if (check_password(password_entered)) {
            ring_buffer_reset(&rx_buffer);

              is_authenticated = true;
              uart_send_string("\r\nPassword correct. Access granted.\r\n");
              display_menu(); // Display the menu after successful authentication
          }
      } else {
          // Check for full command match
          ring_buffer_reset(&rx_buffer);
          switch (byte) {
              case 'A':
                  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
                  uart_send_string("\r\nSend command A\r\nDoor open (turn ON LD2)\r\n");
                  ring_buffer_reset(&rx_buffer);
                  break;
              case 'C':
                  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
                  uart_send_string("\r\nSend command C\r\nDoor closed (turn OFF LD2)\r\n");
                  ring_buffer_reset(&rx_buffer);
                  break;
              case '1':
                  uint8_t state = HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin);
                  uart_send_string(state ? "\r\nSend command 1\r\nDoor status: open (LD2 ON)\r\n" : "Send command #*1*#\r\nDoor status: closed (LD2 OFF)\r\n");
                  ring_buffer_reset(&rx_buffer);
                  break;
              case '0':
                  ring_buffer_reset(&rx_buffer);
                  uart_send_string("\r\nSend command 0\r\nRing buffer clear command: The buffer is empty\r\n");
                  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Close the door
                  is_authenticated = false; // Lock the system
                  uart_send_string("\r\nSystem locked. Please enter the password to unlock.\r\n");
                  break;
              default:
                
                  uart_send_string("\r\nUnknown command.\r\n");
                  break;
          }
      }
  }
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    keypad_init();
    ring_buffer_init(&rx_buffer, rx_buffer_mem, sizeof(rx_buffer_mem));
    memset(current_cmd, 0, COMMAND_LENGTH);
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1); // Start UART interrupt
    while (1) {
      if (column_pressed != 0 && (key_pressed_tick + 5) < HAL_GetTick() ) {
        uint8_t key = keypad_scan(column_pressed);
        ring_buffer_write(&rx_buffer, key);
        HAL_UART_Transmit(&huart2, &key, 1, 100);
        column_pressed = 0;
      }
      process_commands(); 
      
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|ROW_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ROW_2_Pin|ROW_4_Pin|ROW_3_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : COLUMN_1_Pin */
  GPIO_InitStruct.Pin = COLUMN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COLUMN_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : COLUMN_4_Pin */
  GPIO_InitStruct.Pin = COLUMN_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COLUMN_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COLUMN_2_Pin COLUMN_3_Pin */
  GPIO_InitStruct.Pin = COLUMN_2_Pin|COLUMN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ROW_1_Pin */
  GPIO_InitStruct.Pin = ROW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ROW_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW_2_Pin ROW_4_Pin ROW_3_Pin */
  GPIO_InitStruct.Pin = ROW_2_Pin|ROW_4_Pin|ROW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
