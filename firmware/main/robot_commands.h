#ifndef ROBOT_COMMANDS_H
#define ROBOT_COMMANDS_H
#include "unerPrtcl.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Orden FR, FL, RR, RL. PWM con signo, -255..255. */
void RobotCommands_Init(void (*apply)(const int16_t *));
void RobotCommands_AttachMotion(uint8_t (*handler)(const _sComm *, uint32_t));
uint8_t RobotCommands_Handle(const _sComm *frame, uint32_t now_ms);
void RobotCommands_Task(uint32_t now_ms);
#ifdef __cplusplus
}
#endif
#endif
