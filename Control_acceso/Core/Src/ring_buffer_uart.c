#include "ring_buffer_uart.h"

// Inicializa el ring buffer
void ring_buffer_uart_init(ring_buffer_uart_t *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->is_full = false;
}

// Escribe un byte en el buffer
void ring_buffer_uart_write(ring_buffer_uart_t *rb, uint8_t data) {
    if (rb->is_full) {
        // Si el buffer está lleno, sobrescribe el dato más antiguo
        rb->tail = (rb->tail + 1) % UART_BUFFER_CAPACITY;
    }
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % UART_BUFFER_CAPACITY;
    rb->is_full = (rb->head == rb->tail);
}

// Lee un byte del buffer
bool ring_buffer_uart_read(ring_buffer_uart_t *rb, uint8_t *data) {
    if (ring_buffer_uart_is_empty(rb)) {
        return false;  // No hay datos para leer
    }
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % UART_BUFFER_CAPACITY;
    rb->is_full = false;
    return true;  // Lectura exitosa
}

// Devuelve el tamaño actual del buffer
uint16_t ring_buffer_uart_size(ring_buffer_uart_t *rb) {
    if (rb->is_full) {
        return UART_BUFFER_CAPACITY;
    }
    return (rb->head >= rb->tail) ? (rb->head - rb->tail) : (UART_BUFFER_CAPACITY - rb->tail + rb->head);
}

// Verifica si el buffer está lleno
bool ring_buffer_uart_is_full(ring_buffer_uart_t *rb) {
    return rb->is_full;
}

// Verifica si el buffer está vacío
bool ring_buffer_uart_is_empty(ring_buffer_uart_t *rb) {
    return (!rb->is_full && (rb->head == rb->tail));
}