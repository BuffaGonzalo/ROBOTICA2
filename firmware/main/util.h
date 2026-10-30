#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include <stdint.h>

/* Identificadores conservados del protocolo UNER de referencia. */
enum { FALSE = 0, TRUE = 1 };
typedef enum {
    ACK = 0x0D,
    ALIVE = 0xA0
} _eCmd;

#endif
