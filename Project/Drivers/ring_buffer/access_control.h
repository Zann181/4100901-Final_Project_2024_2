#ifndef ACCESS_CONTROL_H
#define ACCESS_CONTROL_H

#include <stdint.h>

/**
 * @brief Procesa los datos en el ring buffer para detectar comandos y
 *        ejecuta la acción correspondiente (encender/apagar LED, etc.).
 */
void process_command(void);

#endif
