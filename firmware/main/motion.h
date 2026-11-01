#ifndef MOTION_H
#define MOTION_H
#include "unerPrtcl.h"
#ifdef __cplusplus
extern "C" {
#endif
void Motion_Init(void (*apply)(const int16_t *));
uint8_t Motion_Command(const _sComm *frame, uint32_t now);
void Motion_Task(uint32_t now);
#define MOTION_TELEMETRY_SIZE 123
void Motion_RecordPwm(const int16_t *powers);
uint8_t Motion_CoastMask(void);
/* 123 data bytes: status 0..91; mode:u8 at 92 (0..3 legacy, 4 vector, 5 arc pose, 6 straight pose, 7 circular arc),
   requested forward/left:i32 um at 93/97; measured forward/left:i32 um
   at 101/105; movement generation:u16 at 109; requested/measured turn:i32
   microradians at 111/115, relative to the start of this MOVE. Modes 5/6
   measured XY is integrated in the starting body frame. Circular sweep:i32
   microradians at 119 (mode 7 only). Wheel order FR/FL/RR/RL. */
uint8_t Motion_Telemetry(uint8_t *payload);
#ifdef __cplusplus
}
#endif
#endif
