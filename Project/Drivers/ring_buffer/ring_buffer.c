/*
 * ring_buffer.c
 *
 *  Created on: Feb 11, 2025
 *      Author: saaci
 */

 #include "ring_buffer.h"

 /**
  * @brief Inicializa el ring buffer con la memoria y capacidad indicadas.
  * 
  * @param rb       Puntero a la instancia de ring_buffer_t.
  * @param mem_add  Puntero al array (uint8_t) donde se almacenarán los datos.
  * @param capacity Cantidad máxima de elementos que puede almacenar el buffer.
  */
 void ring_buffer_init(ring_buffer_t *rb, uint8_t *mem_add, uint8_t capacity)
 {
     rb->buffer   = mem_add;
     rb->capacity = capacity;
     ring_buffer_reset(rb);
 }
 
 /**
  * @brief Reinicia el estado del buffer, dejándolo vacío.
  * 
  * @param rb Puntero a la instancia de ring_buffer_t.
  */
 void ring_buffer_reset(ring_buffer_t *rb)
 {
     rb->head    = 0;
     rb->tail    = 0;
     rb->is_full = 0;
 }
 
 /**
  * @brief Retorna el número de elementos almacenados actualmente en el ring buffer.
  *
  * @param rb Puntero a la instancia de ring_buffer_t.
  * @return uint8_t Número de elementos en el buffer.
  */
 uint8_t ring_buffer_size(ring_buffer_t *rb)
 {
     if (rb->is_full)
     {
         // Si el buffer está lleno, hay 'capacity' elementos
         return rb->capacity;
     }
     // Si no está lleno, la cantidad depende de la posición de head y tail
     if (rb->head >= rb->tail)
     {
         return (rb->head - rb->tail);
     }
     else
     {
         return (rb->capacity - rb->tail + rb->head);
     }
 }
 
 /**
  * @brief Indica si el buffer está lleno.
  * 
  * @param rb Puntero a la instancia de ring_buffer_t.
  * @return 1 si está lleno, 0 en caso contrario.
  */
 uint8_t ring_buffer_is_full(ring_buffer_t *rb)
 {
     return rb->is_full;
 }
 
 /**
  * @brief Indica si el buffer está vacío.
  * 
  * @param rb Puntero a la instancia de ring_buffer_t.
  * @return 1 si está vacío, 0 en caso contrario.
  */
 uint8_t ring_buffer_is_empty(ring_buffer_t *rb)
 {
     // El buffer está vacío si head == tail y no está lleno
     return ((rb->head == rb->tail) && (rb->is_full == 0));
 }
 
 /**
  * @brief Escribe un dato en el ring buffer, sobrescribiendo el más antiguo si está lleno.
  * 
  * @param rb   Puntero a la instancia de ring_buffer_t.
  * @param data Byte a almacenar en el buffer.
  */
 void ring_buffer_write(ring_buffer_t *rb, uint8_t data)
 {
     // Se escribe el nuevo dato en la posición 'head'
     rb->buffer[rb->head] = data;
 
     // Si el buffer está lleno, al escribir se sobrescribe el más viejo (avanzar tail)
     if (rb->is_full)
     {
         rb->tail = (rb->tail + 1) % rb->capacity;
     }
 
     // Avanzamos head de forma circular
     rb->head = (rb->head + 1) % rb->capacity;
 
     // Si al avanzar head coincide con tail, significa que el buffer se llenó
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
  * @brief Lee un dato del ring buffer y lo almacena en *byte.
  * 
  * @param rb   Puntero a la instancia de ring_buffer_t.
  * @param byte Puntero para retornar el dato leído.
  * @return 1 si se leyó exitosamente, 0 si el buffer está vacío.
  */
 uint8_t ring_buffer_read(ring_buffer_t *rb, uint8_t *byte)
 {
     // Verificar si hay datos (buffer no vacío)
     if (ring_buffer_is_empty(rb))
     {
         // No hay datos por leer
         return 0;
     }
 
     // Obtener el dato en 'tail'
     *byte = rb->buffer[rb->tail];
     // Avanzar tail de forma circular
     rb->tail = (rb->tail + 1) % rb->capacity;
 
     // Al leer, el buffer deja de estar "lleno"
     rb->is_full = 0;
 
     return 1;
 }
 