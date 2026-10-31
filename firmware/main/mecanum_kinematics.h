#ifndef FLEXIROVER_MECANUM_KINEMATICS_H
#define FLEXIROVER_MECANUM_KINEMATICS_H

#include <stdint.h>
#include <math.h>
#include "config.h"

namespace Mecanum {
constexpr float WHEEL_RADIUS_MM = 40.295f;
// Arduino.h ya define TWO_PI como macro; usar un nombre propio evita la colisión.
constexpr float MECANUM_TWO_PI = 6.2831853071795864769f;
constexpr float HALF_WHEELBASE_MM = MECANUM_HALF_WHEELBASE_MM;
constexpr float HALF_TRACK_MM = MECANUM_HALF_TRACK_MM;
constexpr float TICKS_PER_RAD = ENCODER_TICKS_PER_WHEEL_REV / MECANUM_TWO_PI;

// Orden fisico del firmware y de los cuatro lazos PI (no orden de la matriz).
enum Wheel : uint8_t { FR = 0, FL = 1, RR = 2, RL = 3 };
struct WheelSetpoints {
    float radPerSecond[4];
    float ticksPerSecond[4];
};

// Modern Robotics 13.2.1, ecuacion 13.10: u = H(0) * [wz, vx, vy].
// x_b hacia delante, y_b a izquierda, z_b arriba; wz positivo antihorario.
// wz en rad/s; vx, vy, l y w en mm/s y mm. Salida en rad/s de RUEDA.
// La matriz indicada tiene filas FL, FR, RR, RL (no FL, RL, RR, FR):
//  [-l-w, 1,-1] FL     [+l+w, 1,+1] FR
//  [+l+w, 1,-1] RR     [-l-w, 1,+1] RL
// Para giro puro las dos ruedas de cada lado deben compartir signo.
// Supone la disposicion de rodillos +/-45 grados de esa matriz.
// Devuelve false y salidas cero ante entrada invalida o giro sin geometria.
// No acciona motores: entrega consignas firmadas para el PI de velocidad.
// DIR_INV y ENC_INV ya se aplican en el driver/ISR: NO aplicarlos otra vez.
inline bool inverse(float wz, float vx, float vy, WheelSetpoints &out,
                    float l = HALF_WHEELBASE_MM, float w = HALF_TRACK_MM) {
    out = WheelSetpoints{};
    if (!isfinite(wz) || !isfinite(vx) || !isfinite(vy) ||
        !isfinite(l) || !isfinite(w) || l < 0 || w < 0 ||
        (wz != 0 && (l <= 0 || w <= 0))) return false;

    const float rotation = (l + w) * wz;
    // Se preserva literalmente H(0); solo se reordenan sus cuatro filas.
    const float u[4] = {
        (-rotation + vx - vy) / WHEEL_RADIUS_MM, // fila 1: FL
        ( rotation + vx + vy) / WHEEL_RADIUS_MM, // fila 2: FR
        ( rotation + vx - vy) / WHEEL_RADIUS_MM, // fila 3: RR
        (-rotation + vx + vy) / WHEEL_RADIUS_MM  // fila 4: RL
    };
    const uint8_t matrixRow[4] = {1, 0, 2, 3}; // FR, FL, RR, RL
    WheelSetpoints result = {};
    for (uint8_t i = 0; i < 4; ++i) {
        result.radPerSecond[i] = u[matrixRow[i]];
        // Calibracion REAL vigente. No volver a multiplicar por 56 ni por 4.
        result.ticksPerSecond[i] = result.radPerSecond[i] * TICKS_PER_RAD;
        if (!isfinite(result.radPerSecond[i]) ||
            !isfinite(result.ticksPerSecond[i])) return false;
    }
    out = result;
    return true;
}

// Limite opcional COMUN: conserva la proporcion entre ruedas y el twist.
// Aplicar antes del PI si alguna consigna excede su velocidad permitida.
inline bool limitSpeed(WheelSetpoints &out, float maxTicksPerSecond) {
    if (!isfinite(maxTicksPerSecond) || maxTicksPerSecond < 0) {
        out = WheelSetpoints{};
        return false;
    }
    float peak = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        if (!isfinite(out.radPerSecond[i]) || !isfinite(out.ticksPerSecond[i])) {
            out = WheelSetpoints{};
            return false;
        }
        const float speed = fabsf(out.ticksPerSecond[i]);
        if (speed > peak) peak = speed;
    }
    if (peak > maxTicksPerSecond) {
        const float scale = maxTicksPerSecond / peak;
        for (uint8_t i = 0; i < 4; ++i) {
            out.radPerSecond[i] *= scale;
            out.ticksPerSecond[i] *= scale;
        }
    }
    return true;
}

/* Ejemplos (calcular consignas no inicia movimiento):
   Mecanum::WheelSetpoints sp;
   bool ok = Mecanum::inverse(0.0f, 100.0f, 0.0f, sp); // avance 100 mm/s
   ok = Mecanum::inverse(0.0f, 0.0f, 100.0f, sp);      // izquierda 100 mm/s
   // Configurar antes l,w medidos en config.h, o pasarlos como argumentos.
   ok = Mecanum::inverse(0.7853981634f, 0.0f, 0.0f, sp); // giro pi/4 rad/s
   // Si ok: sp.ticksPerSecond[FR/FL/RR/RL] es la referencia de cada PI.
   // No asignar controller.demand durante MOVE: el lazo externo de posicion
   // la recalcula. Un futuro modo twist debe seleccionar estas referencias
   // en lugar de las de posicion y conservar watchdog, rampa y parada.
*/
} // namespace Mecanum
#endif
