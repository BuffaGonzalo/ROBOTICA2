#pragma once
#include <stdint.h>
// Register stand-ins for the host-only motion integration test.
inline uint8_t PINK=0,DDRK=0,PORTK=0,DIDR2=0,PCIFR=0,PCMSK2=0,PCICR=0;
#define _BV(bit) (1u<<(bit))
#define PCIF2 2
#define PCIE2 2
