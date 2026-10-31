#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include "config.h"

constexpr double PPR_MOTOR = 11.0;
constexpr double GEAR_RATIO = 56.0;
constexpr double WHEEL_DIAMETER_MM = 80.59;
constexpr double NOMINAL_TICKS_PER_REV = PPR_MOTOR * GEAR_RATIO * 4.0;
constexpr double TICKS_PER_REV = ENCODER_TICKS_PER_WHEEL_REV;
static_assert(TICKS_PER_REV>0 && TICKS_PER_REV<=1000000,"Invalid wheel encoder calibration");
constexpr double TICKS_PER_MM = TICKS_PER_REV / (3.14159265358979323846 * WHEEL_DIAMETER_MM);
// Wire: signed micrometres (0.001 mm). Limit keeps arithmetic and run time bounded.
constexpr int32_t MAX_DISTANCE_UM = 10000000L;
constexpr int PWM_LIMIT = 185;
// Valores base solicitados, orden FR, FL, RR, RL (no calibración medida).
constexpr int PWM_DEADZONE_BY_WHEEL[4] = {170,170,175,185};
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
        for(uint8_t i=0;i<4;++i) pwmLimit[i]=limit;
        stop(RUNNING); target=goal; last=heartbeat=started=now; settling=false; faultMask=0;
        // Independent of encoder activity: even false pulses cannot prolong a MOVE forever.
        // Keep the physical timeout equivalent when calibrating encoder count units.
        uint32_t budget=4000u+(uint32_t)((double)labs(goal)*1000.0/MIN_AVERAGE_TICKS_S);
        moveBudgetMs=budget<180000u ? budget : 180000u;
        for (uint8_t i=0;i<4;++i) {
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
        uint8_t activeCount=0;
        bool inside=true;
        bool crossed[4]={};
        for(uint8_t i=0;i<4;++i) {
            position[i]=counts[i];
            if(!(activeMask&(1u<<i))) {
                previous[i]=counts[i]; velocity[i]=integral[i]=demand[i]=0; pwm[i]=0;
                continue;
            }
            mean+=counts[i]; ++activeCount;
            velocity[i]+=0.35f*((counts[i]-previous[i])/dt-velocity[i]);
            if(labs(counts[i]-progressMark[i])>=4) { lastProgress[i]=now; progressMark[i]=counts[i]; }
            const int32_t oldError=target-previous[i];
            const int32_t newError=target-counts[i];
            crossed[i]=(oldError>0 && newError<0) || (oldError<0 && newError>0);
            // 2. Reset agresivo al cruzar la meta; frenar este ciclo completo.
            if(crossed[i]) { integral[i]=0; demand[i]=0; pwm[i]=0; }
            integral[i]=bounded(integral[i],-50.0f,50.0f);
            previous[i]=counts[i];
            if((target>0 && counts[i]<-100) || (target<0 && counts[i]>100)) {
                faultMask|=1u<<i; stop(DIRECTION_FAULT); return;
            }
            if(labs(target-counts[i])>5) inside=false;
        }
        if(activeCount) mean/=activeCount;
        if(inside) {
            for(uint8_t i=0;i<4;++i) { pwm[i]=0; integral[i]=demand[i]=0; }
            if(!settling) { settling=true; settledSince=now; }
            if((uint32_t)(now-settledSince)>=100) stop(ARRIVED);
            return;
        }
        settling=false;
        for(uint8_t i=0;i<4;++i)
            if((activeMask&(1u<<i)) && labs(target-counts[i])>5 && (uint32_t)(now-lastProgress[i])>1000) faultMask|=1u<<i;
        if(faultMask) { stop(STALLED); return; }
        for(uint8_t i=0;i<4;++i) {
            const int32_t error=target-counts[i];
            // 3. Banda de llegada: cero velocidad y memoria; pwm=0 ordena HARD BRAKE.
            if(labs(error)<=5 || crossed[i] || !pwmLimit[i]) {
                pwm[i]=0; integral[i]=0; demand[i]=0; continue;
            }
            float desired=bounded(kpPosition*error+kpSync*(mean-counts[i]),-MAX_TICKS_S,MAX_TICKS_S);
            if(desired*error<0) desired=0;
            demand[i]+=bounded(desired-demand[i],-ACCEL_TICKS_S2*dt,ACCEL_TICKS_S2*dt);
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
