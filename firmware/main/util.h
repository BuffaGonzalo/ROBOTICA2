#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include <stdint.h>

/* Identificadores conservados del protocolo UNER de referencia. */
enum { FALSE = 0, TRUE = 1 };
typedef enum {
    ACK = 0x0D,
    ALIVE = 0xA0,
    SETPWM = 0xA4,
    MOVE = 0xD0,
    STOP = 0xD1,
    MOTION_STATUS = 0xD2,
    SETGAINS = 0xD4,
    SETLIMITS = 0xD5,
    NACK = 0x0E
} _eCmd;

#endif
