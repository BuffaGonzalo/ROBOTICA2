#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "motion.h"
#include "motion_controller.h"
#include "util.h"
#include "config.h"
#include "pose_kinematics.h"
#include "straight_pose_controller.h"
#include <string.h>

volatile long ticks[4] = {};
static volatile uint8_t previousPink;
static MotionController controller;
static StraightPoseController straightController;
static int32_t origin[4];
static volatile uint16_t edgesA[4], edgesB[4], invalidTransitions[4];
static uint16_t originA[4], originB[4], originInvalid[4];
static uint8_t cancellation; // 0 none, 1 STOP command, 2 manual PWM command
static int16_t appliedPwm[4];
static uint8_t motionAxis; // 0: x longitudinal; 1: y lateral (+ izquierda).
static int32_t requestedForwardUm,requestedLeftUm;
static int8_t vectorSign[4]={1,1,1,1};
static uint16_t movementGeneration;
static int32_t requestedTurnUrad;
static int32_t requestedArcSweepUrad;
static int32_t posePrevious[4];
static PoseKinematics::Pose measuredPose;
// Modern Robotics, ec. 13.10: rodillos +/-45 grados, orden FR, FL, RR, RL.
// +Y: FR/RL adelante, FL/RR atras. tan(+/-45 grados)=+/-1.
static int8_t travelSign(uint8_t wheel) {
    if(motionAxis==6 || motionAxis==7) return 1;
    if(motionAxis==4 || motionAxis==5) return vectorSign[wheel];
    if(motionAxis==2) return (wheel==0 || wheel==3) ? 1 : 0;
    if(motionAxis==3) return (wheel==1 || wheel==2) ? 1 : 0;
    return motionAxis==1 && (wheel==1 || wheel==2) ? -1 : 1;
}
static float wheelDistanceFactor() { return motionAxis==2 || motionAxis==3 ? 1.41421356237f : 1.0f; }
uint8_t Motion_CoastMask(void) {
    // Las ruedas sin consigna diagonal deben rodar libres. STOP/fallo/llegada
    // restauran el freno activo en las cuatro ruedas.
    return controller.state==RUNNING ? ((~controller.activeMask)&15u) : 0;
}
void Motion_RecordPwm(const int16_t *powers) {
    for(uint8_t i=0;i<4;++i) appliedPwm[i]=powers[i];
}
static void (*applyMotor)(const int16_t *);

ISR(PCINT2_vect) {
    uint8_t current_pink = PINK;
    uint8_t old = previousPink;
    previousPink=current_pink;
    // Unrolled constant shifts avoid variable-shift loops on the AVR.
#define DECODE_WHEEL(I) { \
    if ((old ^ current_pink) & ((1u << (I)) | (1u << ((I)+4)))) { \
    uint8_t before=encoderState(old,I); \
    uint8_t after=encoderState(current_pink,I); \
    if((old^current_pink)&(1u<<(I))) ++edgesA[I]; \
    if((old^current_pink)&(1u<<((I)+4))) ++edgesB[I]; \
    if((before^after)==3) ++invalidTransitions[I]; \
    int8_t delta=quadratureDelta(before,after)*ENC_INV[I]; \
    if(delta) ticks[I]=(long)((uint32_t)ticks[I]+delta); \
    } \
}
    DECODE_WHEEL(0); DECODE_WHEEL(1); DECODE_WHEEL(2); DECODE_WHEEL(3);
#undef DECODE_WHEEL
}
static void snapshot(int32_t *values) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { for(uint8_t i=0;i<4;++i) values[i]=ticks[i]; }
}
static void updatePose(const int32_t *values) {
    if(motionAxis<5) return;
    double delta[4];
    for(uint8_t i=0;i<4;++i) {
        delta[i]=(int32_t)((uint32_t)values[i]-(uint32_t)posePrevious[i])/TICKS_PER_MM;
        posePrevious[i]=values[i];
    }
    const double forward=(delta[0]+delta[1]+delta[2]+delta[3])/4;
    const double left=(delta[0]-delta[1]-delta[2]+delta[3])/4;
    const double turn=(delta[0]-delta[1]+delta[2]-delta[3])/(4*(MECANUM_HALF_WHEELBASE_MM+MECANUM_HALF_TRACK_MM));
    PoseKinematics::integrate(measuredPose,forward,left,turn);
}
void Motion_Init(void (*apply)(const int16_t *)) {
    applyMotor=apply;
    // Llamado desde setup(): configurar referencias sin energizar motores.
    controller=MotionController{};
    controller.target=distanceToTicks(DEFAULT_DISTANCE_UM/1000.0);
    motionAxis=0; // Longitudinal.
    controller.stop(IDLE);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        DDRK=0x00; PORTK=0xFF; // Hall inputs with pull-ups, A8..A15.
        DIDR2=0x00; // Ensure digital input buffers ADC8..ADC15 remain enabled.
        previousPink=PINK;
        PCIFR=_BV(PCIF2);
        PCMSK2=0xFF;
        PCICR |= (1 << PCIE2);
    }
}
uint8_t Motion_Command(const _sComm *frame, uint32_t now) {
    uint8_t id=frame->buff[frame->indexData];
    if(id==SETGAINS) {
        if(frame->frameLength!=18) return NACK;
        static_assert(sizeof(float)==4,"SETGAINS requires float32");
        float gains[4];
        for(uint8_t i=0;i<4;++i) {
            uint32_t raw=0;
            for(uint8_t b=0;b<4;++b) raw|=(uint32_t)frame->buff[(frame->indexData+1+i*4+b)&frame->mask]<<(8*b);
            memcpy(&gains[i],&raw,4);
        }
        return controller.setGains(gains) ? ACK:NACK;
    }
    if(id==ALIVE) { controller.heartbeat=now; return ACK; }
    if(id==SETPWM) {
        // Nonzero queued manual packets must not replace an active MOVE.
        if(controller.state==RUNNING)
            for(uint8_t i=1;i<=8;++i) if(frame->buff[(frame->indexData+i)&frame->mask]) return NACK;
        cancellation=2; controller.stop(STOPPED); return ACK;
    }
    if(id==STOP) {
        if(frame->frameLength!=2) return NACK;
        cancellation=1; controller.requestStop(); applyMotor(controller.pwm); return ACK;
    }
    if(id==MOVE && (frame->frameLength==14 || frame->frameLength==18 || frame->frameLength==19 || frame->frameLength==23)) {
        const bool arc=frame->frameLength==23;
        const bool straight=frame->frameLength==19 || arc;
        if(straight && frame->buff[(frame->indexData+17)&frame->mask]!=(arc ? 7 : 6)) return NACK;
        int32_t sweep=0;
        if(arc) {
            uint32_t raw=0; for(uint8_t b=0;b<4;++b) raw|=(uint32_t)frame->buff[(frame->indexData+18+b)&frame->mask]<<(8*b);
            sweep=(int32_t)raw;
            if((sweep>-17453L && sweep<17453L) || sweep < -6265732L || sweep>6265732L) return NACK;
        }
        const bool oriented=frame->frameLength>=18;
        int32_t turn=0;
        if(oriented) {
            uint32_t raw=0;
            for(uint8_t b=0;b<4;++b) raw|=(uint32_t)frame->buff[(frame->indexData+13+b)&frame->mask]<<(8*b);
            turn=(int32_t)raw;
            if(turn < -3141593L || turn > 3141593L || MECANUM_HALF_WHEELBASE_MM<=0 || MECANUM_HALF_TRACK_MM<=0) return NACK;
        }
        // MOVE vectorial: avance_um:i32, izquierda_um:i32, 4 limites:u8.
        int32_t components[2];
        for(uint8_t i=0;i<2;++i) {
            uint32_t bits=0;
            for(uint8_t b=0;b<4;++b) bits|=(uint32_t)frame->buff[(frame->indexData+1+i*4+b)&frame->mask]<<(8*b);
            components[i]=(int32_t)bits;
            if(components[i]<-MAX_DISTANCE_UM || components[i]>MAX_DISTANCE_UM) return NACK;
        }
        const double distance=sqrt((double)components[0]*components[0]+(double)components[1]*components[1]);
        if((distance<1 && !turn) || distance>MAX_DISTANCE_UM) return NACK;
        if(arc && (distance<1 || PoseKinematics::arcLength({components[0]/1000.0,components[1]/1000.0,0},sweep/1000000.0)>10000)) return NACK;
        double wheelMm[4];
        if(oriented && !PoseKinematics::wheels({components[0]/1000.0,components[1]/1000.0,turn/1000000.0},
            MECANUM_HALF_WHEELBASE_MM,MECANUM_HALF_TRACK_MM,wheelMm)) return NACK;
        int32_t signedGoals[4],goals[4]; uint8_t limits[4];
        for(uint8_t i=0;i<4;++i) {
            const int8_t lateralSign=(i==0 || i==3) ? 1 : -1;
            const double millimetres=oriented ? wheelMm[i] : (components[0]+lateralSign*components[1])/1000.0;
            if(!isfinite(millimetres) || fabs(millimetres)>30000) return NACK;
            signedGoals[i]=distanceToTicks(millimetres);
            goals[i]=labs(signedGoals[i]);
            limits[i]=frame->buff[(frame->indexData+9+i)&frame->mask];
            if((straight || goals[i]) && !limits[i]) return NACK;
        }
        controller.stop(STOPPED); applyMotor(controller.pwm);
        motionAxis=arc ? 7 : straight ? 6 : oriented ? 5 : 4; requestedForwardUm=components[0]; requestedLeftUm=components[1]; requestedTurnUrad=turn; requestedArcSweepUrad=sweep;
        for(uint8_t i=0;i<4;++i) vectorSign[i]=signedGoals[i]>0 ? 1 : signedGoals[i]<0 ? -1 : 0;
        snapshot(origin);
        measuredPose={}; for(uint8_t i=0;i<4;++i) posePrevious[i]=origin[i];
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            for(uint8_t i=0;i<4;++i) { originA[i]=edgesA[i]; originB[i]=edgesB[i]; originInvalid[i]=invalidTransitions[i]; }
        }
        cancellation=0; ++movementGeneration;
        if(straight) straightController.start(controller,{components[0]/1000.0,components[1]/1000.0,turn/1000000.0},MECANUM_HALF_WHEELBASE_MM,MECANUM_HALF_TRACK_MM,now,limits,sweep/1000000.0);
        else controller.startWheelGoals(goals,now,limits);
        applyMotor(controller.pwm);
        return ACK;
    }
    if(id!=MOVE || (frame->frameLength!=6 && frame->frameLength!=7 && frame->frameLength!=10 && frame->frameLength!=11)) return NACK;
    const uint8_t axis=frame->frameLength==11 ? frame->buff[(frame->indexData+9)&frame->mask] : 0;
    if(axis>3) return NACK;
    uint8_t limits[4];
    for(uint8_t i=0;i<4;++i) {
        limits[i]=frame->frameLength>=10
            ? frame->buff[(frame->indexData+5+i)&frame->mask]
            : (frame->frameLength==7 ? frame->buff[(frame->indexData+5)&frame->mask] : DEFAULT_MOVE_PWM[i]);
    }
    uint32_t raw=0;
    for(uint8_t b=0;b<4;++b) raw|=(uint32_t)frame->buff[(frame->indexData+1+b)&frame->mask]<<(8*b);
    int32_t um=(int32_t)raw;
    if(um < -MAX_DISTANCE_UM || um > MAX_DISTANCE_UM) return NACK;
    controller.stop(STOPPED); applyMotor(controller.pwm);
    motionAxis=axis;
    snapshot(origin);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for(uint8_t i=0;i<4;++i) { originA[i]=edgesA[i]; originB[i]=edgesB[i]; originInvalid[i]=invalidTransitions[i]; }
    }
    cancellation=0;
    ++movementGeneration;
    requestedForwardUm=axis==0 ? um : axis>=2 ? (int32_t)lround(um/1.41421356237) : 0;
    requestedLeftUm=axis==1 ? um : axis==2 ? requestedForwardUm : axis==3 ? -requestedForwardUm : 0;
    const uint8_t mask=axis==2 ? 9 : axis==3 ? 6 : 15;
    controller.startWithLimits(distanceToTicks(um/1000.0*wheelDistanceFactor()),now,limits,mask);
    applyMotor(controller.pwm); // Start stopped; the 10 ms controller builds the ramp.
    return ACK;
}
void Motion_Task(uint32_t now) {
    if(controller.state!=RUNNING) return;
    if((uint32_t)(now-controller.last)<10u) return;
    int32_t values[4]; snapshot(values); updatePose(values);
    if(motionAxis==4) {
        const double forward=((int32_t)((uint32_t)values[0]-(uint32_t)origin[0])+(int32_t)((uint32_t)values[1]-(uint32_t)origin[1])
            +(int32_t)((uint32_t)values[2]-(uint32_t)origin[2])+(int32_t)((uint32_t)values[3]-(uint32_t)origin[3]))/(4*TICKS_PER_MM);
        const double left=((int32_t)((uint32_t)values[0]-(uint32_t)origin[0])-(int32_t)((uint32_t)values[1]-(uint32_t)origin[1])
            -(int32_t)((uint32_t)values[2]-(uint32_t)origin[2])+(int32_t)((uint32_t)values[3]-(uint32_t)origin[3]))/(4*TICKS_PER_MM);
        const double length=hypot(requestedForwardUm/1000.0,requestedLeftUm/1000.0);
        if(hypot(requestedForwardUm/1000.0-forward,requestedLeftUm/1000.0-left)<=fmin(10.0,length/4)) controller.arrivedMask=controller.activeMask;
    }
    // Proyectar cada encoder al eje elegido: sincronismo y protecciones
    // comparan avances equivalentes, aunque dos ruedas giren al reves.
    for(uint8_t i=0;i<4;++i) values[i]=(int32_t)((uint32_t)values[i]-(uint32_t)origin[i])*travelSign(i);
    if(motionAxis==6 || motionAxis==7) straightController.update(controller,values,measuredPose,now);
    else controller.update(values,now);
    int16_t wheelPwm[4];
    for(uint8_t i=0;i<4;++i) wheelPwm[i]=controller.pwm[i]*travelSign(i);
    applyMotor(wheelPwm);
}
static void put32(uint8_t *out, int32_t value) {
    for(uint8_t b=0;b<4;++b) out[b]=(uint32_t)value>>(8*b);
}
uint8_t Motion_Telemetry(uint8_t *payload) {
    int32_t values[4]; snapshot(values); updatePose(values);
    float mean=0;
    float forwardTicks=0,leftTicks=0;
    uint8_t activeCount=0;
    payload[0]=controller.state;
    put32(payload+1,controller.target);
    for(uint8_t i=0;i<4;++i) {
        int32_t relative=(int32_t)((uint32_t)values[i]-(uint32_t)origin[i]);
        forwardTicks+=relative*0.25f;
        leftTicks+=relative*((i==0 || i==3) ? 0.25f : -0.25f);
        const int8_t sign=travelSign(i);
        if(sign) { mean+=relative*sign; ++activeCount; }
        put32(payload+9+i*4,relative);
    }
    if(activeCount) mean/=activeCount;
    const double forwardUm=motionAxis>=5 ? measuredPose.forward*1000 : forwardTicks/TICKS_PER_MM*1000.0;
    const double leftUm=motionAxis>=5 ? measuredPose.left*1000 : leftTicks/TICKS_PER_MM*1000.0;
    double projected=mean/TICKS_PER_MM/wheelDistanceFactor()*1000.0;
    if(motionAxis>=4) {
        const double length=sqrt((double)requestedForwardUm*requestedForwardUm+(double)requestedLeftUm*requestedLeftUm);
        projected=length>0 ? (forwardUm*requestedForwardUm+leftUm*requestedLeftUm)/length : 0;
    }
    put32(payload+5,(int32_t)lround(projected));
    payload[25]=PINK;
    payload[26]=cancellation;
    payload[27]=controller.faultMask;
    uint16_t a[4],b[4],bad[4];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for(uint8_t i=0;i<4;++i) { a[i]=edgesA[i]; b[i]=edgesB[i]; bad[i]=invalidTransitions[i]; }
    }
    for(uint8_t i=0;i<4;++i) {
        uint16_t values[3]={(uint16_t)(a[i]-originA[i]),(uint16_t)(b[i]-originB[i]),(uint16_t)(bad[i]-originInvalid[i])};
        for(uint8_t k=0;k<3;++k) { payload[28+i*6+k*2]=values[k]&255; payload[29+i*6+k*2]=values[k]>>8; }
    }
    for(uint8_t i=0;i<4;++i) {
        payload[52+i*2]=(uint16_t)appliedPwm[i]&255;
        payload[53+i*2]=(uint16_t)appliedPwm[i]>>8;
    }
    put32(payload+60,(int32_t)lround(TICKS_PER_MM*1000000.0));
    put32(payload+64,(int32_t)lround(WHEEL_DIAMETER_MM*1000.0));
    for(uint8_t i=0;i<4;++i) {
        const float sign=travelSign(i);
        const float values[3]={controller.demand[i]*sign,controller.velocity[i]*sign,controller.integral[i]*100.0f*sign};
        for(uint8_t k=0;k<3;++k) {
            const int16_t value=(int16_t)lroundf(bounded(values[k],-32767,32767));
            payload[68+i*6+k*2]=(uint16_t)value&255;
            payload[69+i*6+k*2]=(uint16_t)value>>8;
        }
    }
    payload[92]=motionAxis;
    put32(payload+93,requestedForwardUm); put32(payload+97,requestedLeftUm);
    put32(payload+101,(int32_t)lround(forwardUm)); put32(payload+105,(int32_t)lround(leftUm));
    payload[109]=movementGeneration&255; payload[110]=movementGeneration>>8;
    put32(payload+111,motionAxis>=5 ? requestedTurnUrad : 0);
    put32(payload+115,motionAxis>=5 ? (int32_t)lround(measuredPose.heading*1000000.0) : 0);
    put32(payload+119,motionAxis==7 ? requestedArcSweepUrad : 0);
    return MOTION_TELEMETRY_SIZE;
}
