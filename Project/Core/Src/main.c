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
#define RB_CAPACITY 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
//UART_HandleTypeDef huart2;

UART_HandleTypeDef huart2;


/* -- Ring Buffer -- */
static uint8_t ring_buffer_memory[RB_CAPACITY];
static ring_buffer_t rb;

/* 2) Variable para almacenar temporalmente el byte recibido por UART (Rx) */
static uint8_t rx_char;

/* 3) Estado de la 'puerta' (LED) */
static uint8_t door_state = 0; // 0 = cerrada, 1 = abierta

/* -- Variables para Keypad -- */
uint32_t key_pressed_tick = 0;
uint16_t column_pressed = 0;
uint32_t debounce_tick = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
// Función que leerá del ring buffer y reconocerá comandos #*X*#
static void process_command(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Callback de EXTI (interrupción externa) para detectar la tecla presionada (Keypad)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((debounce_tick + 200) > HAL_GetTick()) {
    return;
  }
  debounce_tick = HAL_GetTick();
  key_pressed_tick = HAL_GetTick();
  column_pressed = GPIO_Pin;
}
/**
 * @brief Callback de UART (cuando recibe 1 byte)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // Guardamos el byte recibido en el ring buffer
        ring_buffer_write(&rb, rx_char);
        // Reactivamos la recepción para el siguiente byte
        HAL_UART_Receive_IT(&huart2, &rx_char, 1);
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

  
  ring_buffer_init(&rb, ring_buffer_memory, RB_CAPACITY);
  HAL_UART_Receive_IT(&huart2, &rx_char, 1);

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //ring_buffer_init(&rb, ring_buffer_memory, RB_CAPACITY);
  keypad_init();
  // Mensaje de bienvenida
  char start_msg[] = "Sistema de Control de Acceso Iniciado\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)start_msg, sizeof(start_msg)-1, 100);
  
  //HAL_UART_Transmit(&huart2, (uint8_t *)"Hello World\n", 12, 100);
  while (1) {
    if (column_pressed != 0 && (key_pressed_tick + 5) < HAL_GetTick() ) {
      uint8_t key = keypad_scan(column_pressed);
      HAL_UART_Transmit(&huart2, &key, 1, 100);

      ring_buffer_write(&rb, key); // Guardo el carácter en el ring buffer
     // process_command();  // Procesa los comandos en el ring buffer
      column_pressed = 0;
    }
    /* USER CODE END WHILE */
    process_command();  // Procesa los comandos en el ring buffer
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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ROW_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW_2_Pin ROW_4_Pin ROW_3_Pin */
  GPIO_InitStruct.Pin = ROW_2_Pin|ROW_4_Pin|ROW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void process_command(void)
{
    // Buffer auxiliar para "reconstruir" comandos
    static uint8_t command_buffer[8];
    // Índice que indica cuántos chars hemos acumulado en command_buffer
    static uint8_t idx = 0;

    uint8_t data; // Byte temporal para leer del ring buffer

    // Mientras haya algo en el ring buffer
    while (ring_buffer_read(&rb, &data))
    {
        // Imprimir el byte leído para ver qué llegó
        HAL_UART_Transmit(&huart2, &data, 1, 100);

        // Mostrar cuántos elementos quedarán en el buffer tras este read
        // (Debería ir bajando, a menos que entren más datos simultáneamente)
        char debug_msg[40];
        uint8_t current_size = ring_buffer_size(&rb);
        sprintf(debug_msg, "[DEBUG] Tam. buffer: %u\r\n", current_size);
        HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 100);

        // Almacenar el byte en command_buffer
        command_buffer[idx++] = data;

        // Evitar pasarnos de 'command_buffer'
        if (idx >= sizeof(command_buffer))
        {
            idx = 0;
        }

        // Verificar si hay 5 chars para detectar #*X*#
        if (idx >= 5)
        {
            if (command_buffer[idx-5] == '#' &&
                command_buffer[idx-4] == '*' &&
                command_buffer[idx-2] == '*' &&
                command_buffer[idx-1] == '#')
            {
                // Extraer comando central
                uint8_t cmd = command_buffer[idx-3];

                // Reiniciamos idx para buscar un nuevo comando
                idx = 0;

                // Ejecutar acción
                switch (cmd)
                {
                    case 'A': // #*A*#
                        door_state = 1; // Puerta abierta
                        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
                        HAL_UART_Transmit(&huart2,
                          (uint8_t*)"\r\nPuerta abierta\r\n", 18, 100);
                        break;

                    case 'C': // #*C*#
                        door_state = 0; // Puerta cerrada
                        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
                        HAL_UART_Transmit(&huart2,
                          (uint8_t*)"\r\nPuerta cerrada\r\n", 18, 100);
                        break;

                    case '1': // #*1*#
                        if (door_state)
                            HAL_UART_Transmit(&huart2,
                              (uint8_t*)"\r\nLa puerta esta abierta\r\n", 26, 100);
                        else
                            HAL_UART_Transmit(&huart2,
                              (uint8_t*)"\r\nLa puerta esta cerrada\r\n", 27, 100);
                        break;

                    case '0': // #*0*#
                        // Limpia el ring buffer
                        ring_buffer_reset(&rb);
                        HAL_UART_Transmit(&huart2,
                          (uint8_t*)"\r\nRing buffer limpiado\r\n", 25, 100);
                        break;

                    default:
                        HAL_UART_Transmit(&huart2,
                          (uint8_t*)"\r\nComando invalido\r\n", 20, 100);
                        break;
                } // end switch
            }
        }
    } // end while (ring_buffer_read(...))
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
