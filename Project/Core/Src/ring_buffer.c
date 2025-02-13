/*
 * ring_buffer.c
 *
 *  Created on: Feb 11, 2025
 *      Author: saaci
 */

 #include "ring_buffer.h"

 /**
  * @brief Inicializa el ring buffer, asociándolo con la memoria dada.
  *
  * @param rb        Puntero a la instancia del ring buffer.
  * @param mem_add   Array donde se almacenarán los datos.
  * @param capacity  Tamaño máximo del buffer.
  */
 void ring_buffer_init(ring_buffer_t *rb, uint8_t *mem_add, uint8_t capacity)
 {
     rb->buffer   = mem_add;
     rb->capacity = capacity;
     ring_buffer_reset(rb);
 }
 
 /**
  * @brief Deja el buffer vacío (resetea índices y bandera).
  *
  * @param rb  Puntero a la instancia del ring buffer.
  */
 void ring_buffer_reset(ring_buffer_t *rb)
 {
     rb->head    = 0;
     rb->tail    = 0;
     rb->is_full = 0;
 }
 
 /**
  * @brief Devuelve la cantidad de elementos presentes actualmente en el buffer.
  *
  * @param rb  Puntero a la instancia del ring buffer.
  * @return Número de bytes almacenados.
  */
 uint8_t ring_buffer_size(ring_buffer_t *rb)
 {
     if (rb->is_full)
     {
         // Si está lleno, hay 'capacity' elementos
         return rb->capacity;
     }
     else if (rb->head >= rb->tail)
     {
         return (rb->head - rb->tail);
     }
     else
     {
         return (rb->capacity - rb->tail + rb->head);
     }
 }
 
 /**
  * @brief Indica si el buffer está completamente lleno.
  *
  * @param rb  Puntero a la instancia del ring buffer.
  * @return 1 si está lleno, 0 en caso contrario.
  */
 uint8_t ring_buffer_is_full(ring_buffer_t *rb)
 {
     return rb->is_full;
 }
 
 /**
  * @brief Indica si el buffer está vacío.
  *
  * @param rb  Puntero a la instancia del ring buffer.
  * @return 1 si está vacío, 0 en caso contrario.
  */
 uint8_t ring_buffer_is_empty(ring_buffer_t *rb)
 {
     // Vacío si (head == tail) y no está lleno
     return ((rb->head == rb->tail) && (rb->is_full == 0));
 }
 
 /**
  * @brief Escribe un byte en el ring buffer; sobrescribe el más antiguo si está lleno.
  *
  * @param rb   Puntero a la instancia del ring buffer.
  * @param data Byte a almacenar.
  */
 void ring_buffer_write(ring_buffer_t *rb, uint8_t data)
 {
     // Escribimos en la posición 'head'
     rb->buffer[rb->head] = data;
 
     if (rb->is_full)
     {
         // Si ya estaba lleno, avanzamos 'tail' para sobrescribir
         rb->tail = (rb->tail + 1) % rb->capacity;
     }
 
     // Avanzamos 'head' de modo circular
     rb->head = (rb->head + 1) % rb->capacity;
 
     // Si head alcanza tail, significa que el buffer se llenó
     if (rb->head == rb->tail)
     {
         rb->is_full = 1;
     }
     else
     {
         rb->is_full = 0;
     }
 }
 
 /**
  * @brief Lee un byte desde la cola del ring buffer.
  *
  * @param rb    Puntero a la instancia del ring buffer.
  * @param byte  Variable donde se devolverá el dato.
  * @return 1 si se leyó con éxito, 0 si estaba vacío.
  */
 uint8_t ring_buffer_read(ring_buffer_t *rb, uint8_t *byte)
 {
     // Si está vacío, no hay datos para leer
     if (ring_buffer_is_empty(rb))
     {
         return 0;
     }
 
     // Extraemos el dato desde 'tail'
     *byte = rb->buffer[rb->tail];
 
     // Avanzamos 'tail' de forma circular
     rb->tail = (rb->tail + 1) % rb->capacity;
 
     // Al haber quitado un elemento, ya no está lleno
     rb->is_full = 0;
 
     return 1;
 }
 