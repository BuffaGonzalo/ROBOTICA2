#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include "config.h"
#include "navigation_limits.h"

constexpr double PPR_MOTOR = 11.0;
constexpr double GEAR_RATIO = 56.0;
constexpr double WHEEL_DIAMETER_MM = 80.59;
constexpr double NOMINAL_TICKS_PER_REV = PPR_MOTOR * GEAR_RATIO * 4.0;
constexpr double TICKS_PER_REV = ENCODER_TICKS_PER_WHEEL_REV;
static_assert(TICKS_PER_REV>0 && TICKS_PER_REV<=1000000,"Invalid wheel encoder calibration");
static_assert(WHEEL_TRAVEL_SCALE>0 && WHEEL_TRAVEL_SCALE<=10,"Invalid wheel travel scale");
// Use the same calibrated conversion for goals, odometry and telemetry.
constexpr double TICKS_PER_MM = TICKS_PER_REV / (3.14159265358979323846 * WHEEL_DIAMETER_MM * WHEEL_TRAVEL_SCALE);
constexpr int32_t ARRIVAL_TOLERANCE_TICKS = NavigationLimits::ArrivalToleranceTicks;
constexpr double ARRIVAL_TOLERANCE_MM = ARRIVAL_TOLERANCE_TICKS/TICKS_PER_MM;
// Final route destinations use the same narrow encoder-derived tolerance.
// Automatic detour handoffs have a separate margin in NavigationLimits.
constexpr double ROUTE_ARRIVAL_MARGIN_MM = ARRIVAL_TOLERANCE_MM;
constexpr double ROUTE_ARRIVAL_ANGLE_RAD = 5.0*3.14159265358979323846/180.0;
// Wire: signed micrometres (0.001 mm). Limit keeps arithmetic and run time bounded.
constexpr int32_t MAX_DISTANCE_UM = 10000000L;
constexpr int PWM_LIMIT = 185;
// Compensacion de friccion, independiente de los limites de potencia.
// FR=57 medido; FL/RR/RL=45 recuperados del ajuste anterior, a calibrar bajo carga.
constexpr int PWM_DEADZONE_BY_WHEEL[4] = {57,45,45,45};
constexpr uint8_t DEFAULT_MOVE_PWM[4] = {170,170,175,185};
constexpr int32_t DEFAULT_DISTANCE_UM = 900000L; // 90 cm; no inicia movimiento.
constexpr float POSITION_KP = 2.0f;
constexpr float SYNC_KP = 1.5f;
constexpr float VELOCITY_KP = 0.07f;
constexpr float VELOCITY_KI = 0.5f;
// Preserve the previous physical speed and acceleration after changing count units.
constexpr float MAX_TICKS_S = 2200.0f * TICKS_PER_REV / NOMINAL_TICKS_PER_REV;
constexpr float ACCEL_TICKS_S2 = 3000.0f * TICKS_PER_REV / NOMINAL_TICKS_PER_REV;
constexpr float MIN_AVERAGE_TICKS_S = 300.0f * TICKS_PER_REV / NOMINAL_TICKS_PER_REV;
inline float bounded(float v, float lo, float hi) { return v<lo ? lo : (v>hi ? hi : v); }
inline int32_t distanceToTicks(double mm) { return (int32_t)lround(mm * TICKS_PER_MM); }
// 00 -> 01 -> 11 -> 10 -> 00 is positive; illegal transitions count zero.
inline int8_t quadratureDelta(uint8_t previous, uint8_t current) {
    static const int8_t lut[16] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};
    return lut[(previous << 2) | current];
}
inline uint8_t encoderState(uint8_t port, uint8_t wheel) {
    return ((port >> wheel) & 1u) | (((port >> (wheel+4)) & 1u) << 1);
}
enum MotionState : uint8_t { IDLE, RUNNING, ARRIVED, STOPPED, STALLED, LINK_LOST, TIMING_FAULT, MOVE_TIMEOUT, DIRECTION_FAULT, OVERSHOOT };
struct MotionController {
    float kpPosition=POSITION_KP, kpSpeed=VELOCITY_KP, kiSpeed=VELOCITY_KI, kpSync=SYNC_KP;
    MotionState state = IDLE;
    int32_t target = 0, position[4] = {}, previous[4] = {};
    int16_t pwm[4] = {};
    float velocity[4] = {}, integral[4] = {}, demand[4] = {};
    uint32_t last = 0, heartbeat = 0, started = 0, lastProgress[4] = {};
    uint32_t settledSince = 0;
    uint32_t moveBudgetMs = 0;
    uint8_t pwmLimit[4] = {170,170,175,185};
    bool settling = false;
    int32_t progressMark[4] = {};
    uint8_t faultMask = 0;
    uint8_t activeMask = 15; // Diagonales: solo el par motriz participa del control.
    uint8_t arrivedMask = 0;
    int32_t wheelTargets[4] = {};
    float wheelRatio[4] = {1,1,1,1};
    int32_t arrivalToleranceTicks(uint8_t wheel=0) const {
        // Evitar que una consigna corta quede completamente dentro de la banda.
        const int32_t quarter=labs(wheelTargets[wheel])/4;
        const int32_t scaled=(int32_t)(ARRIVAL_TOLERANCE_TICKS*wheelRatio[wheel]+0.5f);
        const int32_t cap=scaled>0 ? scaled : 1;
        return quarter<cap ? (quarter>0 ? quarter : 1) : cap;
    }
    bool setGains(const float *g) {
        if(state==RUNNING) return false;
        const float maximum[4]={50,2,5,20};
        for(uint8_t i=0;i<4;++i) if(!isfinite(g[i]) || g[i]<0 || g[i]>maximum[i]) return false;
        kpPosition=g[0]; kpSpeed=g[1]; kiSpeed=g[2]; kpSync=g[3];
        for(uint8_t i=0;i<4;++i) integral[i]=0;
        return true;
    }
    void requestStop() {
        // A subsequent STOP must not erase the original fault or confirmed arrival.
        stop(state==RUNNING || state==IDLE || state==STOPPED ? STOPPED : state);
    }
    void stop(MotionState reason) {
        state = reason;
        for (uint8_t i=0;i<4;++i) { pwm[i]=0; integral[i]=demand[i]=0; }
    }
    void start(int32_t goal, uint32_t now, uint8_t limit=PWM_LIMIT) {
        activeMask=15;
        arrivedMask=0;
        for(uint8_t i=0;i<4;++i) pwmLimit[i]=limit;
        stop(RUNNING); target=goal; last=heartbeat=started=now; settling=false; faultMask=0;
        // Independent of encoder activity: even false pulses cannot prolong a MOVE forever.
        // Keep the physical timeout equivalent when calibrating encoder count units.
        uint32_t budget=4000u+(uint32_t)((double)labs(goal)*1000.0/MIN_AVERAGE_TICKS_S);
        moveBudgetMs=budget<180000u ? budget : 180000u;
        for (uint8_t i=0;i<4;++i) {
            wheelTargets[i]=goal; wheelRatio[i]=1;
            position[i]=previous[i]=progressMark[i]=0; lastProgress[i]=now;
            velocity[i]=0; pwm[i]=0;
        }
        if(!limit) stop(STOPPED); // Zero power never energizes a wheel.
    }
    void startWithLimits(int32_t goal, uint32_t now, const uint8_t *limits, uint8_t mask=15) {
        start(goal,now);
        activeMask=mask&15;
        uint8_t any=0;
        for(uint8_t i=0;i<4;++i) {
            pwmLimit[i]=(activeMask&(1u<<i)) ? limits[i] : 0; any|=pwmLimit[i];

        }
        if(!any) stop(STOPPED);
    }
    // Entradas en modulo: Motion_Task normaliza los signos fisicos.
    // Una referencia comun normalizada preserva las proporciones de H(0).
    void startWheelGoals(const int32_t *goals,uint32_t now,const uint8_t *limits) {
        int32_t peak=0;
        for(uint8_t i=0;i<4;++i) if(goals[i]>peak) peak=goals[i];
        start(peak,now); activeMask=0;
        for(uint8_t i=0;i<4;++i) {
            wheelTargets[i]=goals[i];
            wheelRatio[i]=peak>0 ? (float)goals[i]/peak : 0;
            pwmLimit[i]=goals[i]>0 ? limits[i] : 0;
            if(goals[i]>0) activeMask|=1u<<i;
        }
        if(!activeMask) stop(ARRIVED);
    }
    // Called once per real 10 ms sample, never catch up with duplicate encoder samples.
    void update(const int32_t *counts, uint32_t now) {
        if (state != RUNNING) return;
        uint32_t elapsed=now-last;
        if (elapsed<10) return;
        if (elapsed>30) { stop(TIMING_FAULT); return; }
        last=now;
        if ((uint32_t)(now-heartbeat)>3000) { stop(LINK_LOST); return; }
        if ((uint32_t)(now-started)>moveBudgetMs) { stop(MOVE_TIMEOUT); return; }
        float dt=elapsed*0.001f, mean=0;
        float phaseWeight=0;
        bool inside=true,withinTolerance=true;
        bool crossed[4]={};
        for(uint8_t i=0;i<4;++i) {
            position[i]=counts[i];
            if(!(activeMask&(1u<<i))) {
                previous[i]=counts[i]; velocity[i]=integral[i]=demand[i]=0; pwm[i]=0;
                continue;
            }
            velocity[i]+=0.35f*((counts[i]-previous[i])/dt-velocity[i]);
            if(labs(counts[i]-progressMark[i])>=4) { lastProgress[i]=now; progressMark[i]=counts[i]; }
            const int32_t goal=wheelTargets[i];
            const int32_t oldError=goal-previous[i];
            const int32_t newError=goal-counts[i];
            if(labs(newError)>arrivalToleranceTicks(i)) withinTolerance=false;
            crossed[i]=(oldError>0 && newError<0) || (oldError<0 && newError>0);
            // Enclavar llegada por rueda: no volver a energizarla para corregir
            // pequeños residuos ni invertir el giro al cruzar el objetivo.
            if(labs(newError)<=arrivalToleranceTicks(i) || crossed[i]) arrivedMask|=1u<<i;
            // 2. Reset agresivo al cruzar la meta; frenar este ciclo completo.
            if(crossed[i]) { integral[i]=0; demand[i]=0; pwm[i]=0; }
            integral[i]=bounded(integral[i],-50.0f,50.0f);
            previous[i]=counts[i];
            if((goal>0 && counts[i]<-100) || (goal<0 && counts[i]>100)) {
                faultMask|=1u<<i; stop(DIRECTION_FAULT); return;
            }
            if(!(arrivedMask&(1u<<i))) inside=false;
        }
        // Fase comun por minimos cuadrados. No amplificar el ruido de una
        // rueda cuya referencia es casi cero ni sincronizar contra una ya frenada.
        for(uint8_t i=0;i<4;++i) if((activeMask&(1u<<i)) && !(arrivedMask&(1u<<i))) {
            mean+=counts[i]*wheelRatio[i]; phaseWeight+=wheelRatio[i]*wheelRatio[i];
        }
        mean=phaseWeight>0 ? mean/phaseWeight : target;
        if(inside) {
            for(uint8_t i=0;i<4;++i) { pwm[i]=0; integral[i]=demand[i]=0; }
            if(!settling) { settling=true; settledSince=now; }
            if((uint32_t)(now-settledSince)>=100) stop(withinTolerance ? ARRIVED : OVERSHOOT);
            return;
        }
        settling=false;
        for(uint8_t i=0;i<4;++i)
            if((activeMask&(1u<<i)) && !(arrivedMask&(1u<<i)) &&
               (uint32_t)(now-lastProgress[i])>1000u+(uint32_t)(4000.0f/(MAX_TICKS_S*wheelRatio[i]))) faultMask|=1u<<i;
        if(faultMask) { stop(STALLED); return; }
        for(uint8_t i=0;i<4;++i) {
            const int32_t error=wheelTargets[i]-counts[i];
            // 3. Banda de llegada: cero velocidad y memoria; pwm=0 ordena HARD BRAKE.
            if((arrivedMask&(1u<<i)) || !pwmLimit[i]) {
                pwm[i]=0; integral[i]=0; demand[i]=0; continue;
            }
            const float ratio=wheelRatio[i];
            float desired=bounded(kpPosition*error+kpSync*(mean*ratio-counts[i]),-MAX_TICKS_S*ratio,MAX_TICKS_S*ratio);
            if(desired*error<0) desired=0;
            demand[i]+=bounded(desired-demand[i],-ACCEL_TICKS_S2*ratio*dt,ACCEL_TICKS_S2*ratio*dt);
            const float ve=demand[i]-velocity[i];
            // 2. Integral en unidades PWM: aporte máximo ±50, con anti-windup por saturación.
            const float trial=bounded(integral[i]+kiSpeed*ve*dt,-50.0f,50.0f);
            const float ff=demand[i]>0.01f ? PWM_DEADZONE_BY_WHEEL[i] :
                           (demand[i]<-0.01f ? -PWM_DEADZONE_BY_WHEEL[i] : 0);
            const float base=demand[i]*0.035f+kpSpeed*ve+ff;
            const float candidate=base+trial;
            const float limit=pwmLimit[i];
            if(!((candidate>limit && ve>0) || (candidate<-limit && ve<0))) integral[i]=trial;
            pwm[i]=(int16_t)bounded(base+integral[i],-limit,limit);
        }
    }
};
#endif
