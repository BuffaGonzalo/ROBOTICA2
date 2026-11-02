#include "robot_commands.h"
#include "util.h"

static void (*apply_pwm)(const int16_t *);
static uint32_t last_pwm_ms;
static uint8_t moving;
static uint8_t (*motion_command)(const _sComm *, uint32_t);
void RobotCommands_AttachMotion(uint8_t (*handler)(const _sComm *, uint32_t)) { motion_command=handler; }

void RobotCommands_Init(void (*apply)(const int16_t *)) {
    apply_pwm = apply;
    moving = 0;
    last_pwm_ms = 0;
}

uint8_t RobotCommands_Handle(const _sComm *frame, uint32_t now_ms) {
    uint8_t id = frame->buff[frame->indexData];
    int16_t powers[4];
    uint8_t i, any_power = 0;
    if (id == ALIVE) {
        if(frame->frameLength != 2) return NACK;
        if(motion_command) motion_command(frame,now_ms);
        return ACK;
    }
    if(id==MOVE || id==STOP || id==SETGAINS) {
        if(id==SETGAINS && moving) return NACK;
        uint8_t result=motion_command ? motion_command(frame,now_ms) : NACK;
        if(result==ACK) moving=0;
        return result;
    }
    if(id==SETLIMITS) return motion_command ? motion_command(frame,now_ms) : NACK;
    if (id != SETPWM) return 0; /* No hay comandos históricos. */
    if (frame->frameLength != 10 || !apply_pwm) return NACK;
    for (i = 0; i < 4; ++i) {
        uint8_t index = (frame->indexData + 1u + i * 2u) & frame->mask;
        uint16_t raw = frame->buff[index];
        raw |= (uint16_t)frame->buff[(index + 1u) & frame->mask] << 8;
        /* Conversión explícita del complemento a dos de la trama. */
        int32_t value = raw < 0x8000u ? (int32_t)raw : (int32_t)raw - 65536L;
        if (value < -255 || value > 255) return NACK;
        powers[i] = (int16_t)value;
        any_power |= value != 0;
    }
    if(motion_command && motion_command(frame,now_ms)!=ACK) return NACK;
    apply_pwm(powers); /* Aplicar sólo después de validar las cuatro ruedas. */
    moving = any_power;
    last_pwm_ms = now_ms;
    return ACK;
}

void RobotCommands_Task(uint32_t now_ms) {
    if (moving && (uint32_t)(now_ms - last_pwm_ms) >= 1500u) {
        const int16_t stopped[4] = {0, 0, 0, 0};
        apply_pwm(stopped);
        moving = 0;
    }
}
