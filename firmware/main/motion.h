#ifndef MOTION_H
#define MOTION_H
#include "unerPrtcl.h"
#ifdef __cplusplus
extern "C" {
#endif
void Motion_Init(void (*apply)(const int16_t *));
uint8_t Motion_Command(const _sComm *frame, uint32_t now);
void Motion_Task(uint32_t now);
#define MOTION_TELEMETRY_SIZE 93
void Motion_RecordPwm(const int16_t *powers);
uint8_t Motion_CoastMask(void);
/* 93 data bytes: previous 92-byte status plus direction:u8 (0 longitudinal,
   1 lateral, 2 diagonal left, 3 diagonal right). Distance is chassis path
   length along the selected direction; wheel ticks stay signed. */
uint8_t Motion_Telemetry(uint8_t *payload);
#ifdef __cplusplus
}
#endif
#endif
