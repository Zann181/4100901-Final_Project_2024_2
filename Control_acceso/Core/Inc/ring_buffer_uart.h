#ifndef RING_BUFFER_UART_H
#define RING_BUFFER_UART_H

#include <stdint.h>
#include <stdbool.h>

#define UART_BUFFER_CAPACITY 100  // Capacidad del buffer para UART

typedef struct {
    uint8_t buffer[UART_BUFFER_CAPACITY];  // Memoria del buffer
    uint16_t head;                         // Índice de escritura
    uint16_t tail;                        // Índice de lectura
    bool is_full;                         // Indicador de buffer lleno
} ring_buffer_uart_t;

// Funciones públicas
void ring_buffer_uart_init(ring_buffer_uart_t *rb);
void ring_buffer_uart_write(ring_buffer_uart_t *rb, uint8_t data);
bool ring_buffer_uart_read(ring_buffer_uart_t *rb, uint8_t *data);
uint16_t ring_buffer_uart_size(ring_buffer_uart_t *rb);
bool ring_buffer_uart_is_full(ring_buffer_uart_t *rb);
bool ring_buffer_uart_is_empty(ring_buffer_uart_t *rb);

#endif // RING_BUFFER_UART_H