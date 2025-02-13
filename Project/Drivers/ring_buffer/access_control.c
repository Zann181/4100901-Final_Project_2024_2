#include "access_control.h"
#include "ring_buffer.h"
#include "main.h"   // Para acceder a huart2, pines del LED, etc.

// Declaración 'extern' del ring buffer definido en main.c
extern ring_buffer_t rb;
extern UART_HandleTypeDef huart2;

// Simulamos la "puerta" con el LED
static uint8_t door_state = 0;  // 0 = cerrada, 1 = abierta

/**
 * @brief Se encarga de leer continuamente del ring buffer y buscar
 *        comandos con el formato #*X*# (A, C, 1, 0).
 *        Cuando encuentra un comando válido, ejecuta la acción y
 *        envía un mensaje por UART.
 */
void process_command(void)
{
    static uint8_t command_buffer[8]; // buffer auxiliar
    static uint8_t idx = 0;           // índice dentro de command_buffer
    uint8_t data;

    // Leer todos los caracteres que tengamos pendientes en el ring buffer
    while (ring_buffer_read(&rb, &data))
    {
        // Guardamos el carácter en command_buffer
        command_buffer[idx++] = data;

        // Evitar sobreflow en command_buffer
        if (idx >= sizeof(command_buffer)) {
            idx = 0;
        }

        // Verificamos si ya tenemos un patrón #*X*#
        // Son 5 caracteres: '#' '*' X '*' '#'
        if (idx >= 5)
        {
            // Tomamos los últimos 5 chars
            if (command_buffer[idx-5] == '#' &&
                command_buffer[idx-4] == '*' &&
                command_buffer[idx-2] == '*' &&
                command_buffer[idx-1] == '#')
            {
                // El comando central es command_buffer[idx-3]
                uint8_t cmd = command_buffer[idx-3];

                // Reiniciamos idx para empezar a buscar un siguiente comando
                idx = 0;

                // Ejecutamos según 'cmd'
                switch (cmd)
                {
                    case 'A': // #*A*#
                        door_state = 1; 
                        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
                        HAL_UART_Transmit(&huart2,
                                         (uint8_t *)"Puerta abierta\r\n",
                                         16, 100);
                        break;

                    case 'C': // #*C*#
                        door_state = 0;
                        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
                        HAL_UART_Transmit(&huart2,
                                         (uint8_t *)"Puerta cerrada\r\n",
                                         16, 100);
                        break;

                    case '1': // #*1*#
                        if (door_state == 1)
                        {
                            HAL_UART_Transmit(&huart2,
                                             (uint8_t *)"Puerta esta abierta\r\n",
                                             21, 100);
                        }
                        else
                        {
                            HAL_UART_Transmit(&huart2,
                                             (uint8_t *)"Puerta esta cerrada\r\n",
                                             21, 100);
                        }
                        break;

                    case '0': // #*0*#
                        // Limpia el ring buffer
                        ring_buffer_reset(&rb);
                        HAL_UART_Transmit(&huart2,
                                         (uint8_t *)"Ring buffer limpiado\r\n",
                                         23, 100);
                        break;

                    default:
                        // Comando desconocido o error
                        HAL_UART_Transmit(&huart2,
                                         (uint8_t *)"Comando invalido\r\n",
                                         18, 100);
                        break;
                } // fin switch
            } // fin if (comando valido)
        } // fin if (idx >= 5)
    } // fin while (ring_buffer_read)
}
