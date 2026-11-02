// Actual firmware with synthetic encoder counts; no hardware is driven.
#include "../../firmware/main/motion.cpp"
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

static void check(bool ok,const char *why) {
    if(!ok) { std::fprintf(stderr,"FAIL: %s\n",why); std::exit(1); }
}
static int16_t output[4];
static void apply(const int16_t *pwm) {
    for(int i=0;i<4;++i) output[i]=pwm[i];
    Motion_RecordPwm(pwm);
}

static void distanceArrival(int mode,int sign) {
    Motion_Init(apply);
    uint8_t data[256]={MOVE}; put32(data+1,sign*500000);
    _sComm command={}; command.buff=data; command.mask=255;
    if(mode==0) {
        command.frameLength=10;
        for(int i=0;i<4;++i) data[5+i]=DEFAULT_MOVE_PWM[i];
    } else {
        command.frameLength=mode==4 ? 14 : mode==6 ? 19 : 20;
        data[17]=6; data[18]=2; // final leg of a continuous detour
        for(int i=0;i<4;++i) data[9+i]=DEFAULT_MOVE_PWM[i];
    }
    check(Motion_Command(&command,1000)==ACK,"accept 50 cm command");
    const int32_t goal=distanceToTicks(500);
    for(int i=0;i<4;++i) ticks[i]=origin[i]+sign*distanceToTicks(470);
    Motion_Task(1010);
    check(controller.state==RUNNING && !controller.settling,"50 cm cannot finish at 47 cm");
    for(int i=0;i<4;++i) ticks[i]=origin[i]+sign*(goal-6);
    Motion_Task(1020);
    check(controller.state==RUNNING && !controller.settling,"six ticks remaining is outside arrival band");
    for(int i=0;i<4;++i) ticks[i]=origin[i]+sign*(goal-4);
    for(uint32_t now=1030;now<=1140;now+=10) Motion_Task(now);
    check(controller.state==ARRIVED,"four ticks remaining confirms arrival after settling");
    for(int value:output) check(value==0,"arrival brakes every wheel");
}

static void frictionCompensation(int sign,bool pose) {
    MotionController c;
    StraightPoseController path;
    const uint8_t limits[4]={255,255,255,255};
    const int32_t counts[4]={};
    if(pose) path.start(c,{sign*500.0,0,0},47.5,105,1000,limits);
    else c.startWithLimits(sign*distanceToTicks(500),1000,limits);
    c.kpPosition=c.kpSync=c.kiSpeed=0; c.kpSpeed=1;
    if(pose) path.update(c,counts,{},1010);
    else {
        // After the acceleration limiter, PI effort is about 10 PWM.
        for(int i=0;i<4;++i) c.demand[i]=sign*(ACCEL_TICKS_S2*0.01f+10);
        c.update(counts,1010);
    }
    for(int i=0;i<4;++i) {
        const float effort=c.demand[i]*0.035f+c.kpSpeed*(c.demand[i]-c.velocity[i])+c.integral[i];
        const int16_t expected=(int16_t)bounded(effort+sign*PWM_DEADZONE_BY_WHEEL[i],-255,255);
        check(c.pwm[i]==expected,"actual motor output adds signed friction base to PI effort");
        check(sign*c.pwm[i]>=PWM_DEADZONE_BY_WHEEL[i],"small drive effort retains the configured startup base");
    }
}

static void settlingDrift(bool pose) {
    Motion_Init(apply);
    uint8_t data[256]={MOVE}; put32(data+1,500000); data[17]=6;
    for(int i=0;i<4;++i) data[9+i]=170;
    _sComm command={}; command.buff=data; command.mask=255; command.frameLength=pose ? 19 : 14;
    check(Motion_Command(&command,2000)==ACK,"start drift regression");
    for(int i=0;i<4;++i) ticks[i]=origin[i]+distanceToTicks(500);
    Motion_Task(2010);
    check(controller.settling,"brake when inside the narrow band");
    for(int i=0;i<4;++i) ticks[i]+=20;
    for(uint32_t now=2020;now<=2120;now+=10) Motion_Task(now);
    check(controller.state==OVERSHOOT,"drift beyond tolerance is not reported as accurate arrival");
    for(int value:output) check(value==0,"overshoot remains braked");
}

static int32_t read32(const uint8_t *data) {
    uint32_t value=0; for(int i=0;i<4;++i) value|=uint32_t(data[i])<<(8*i);
    return int32_t(value);
}
static void physicalDistanceCalibration() {
    // Independent reference from the reported tape measurement, not a plant
    // whose distance-per-tick is derived from the calibrated controller.
    const double nominal=1500.0/(3.14159265358979323846*80.59);
    const double physicalMmPerTick=(505.0/499.5)/nominal;
    const int32_t previousGoal=lround(500*nominal);
    const int32_t correctedGoal=distanceToTicks(500);
    check(std::abs(previousGoal*physicalMmPerTick-500)>5,"uncalibrated encoder model reproduces the observed overshoot");
    check(correctedGoal<previousGoal && std::abs(correctedGoal*physicalMmPerTick-500)<physicalMmPerTick,"corrected 50 cm goal matches the physical model within one tick");
    for(int sign:{-1,1}) {
        Motion_Init(apply);
        uint8_t data[256]={MOVE}; put32(data+1,sign*500000);
        for(int i=0;i<4;++i) data[5+i]=DEFAULT_MOVE_PWM[i];
        _sComm command={}; command.buff=data; command.mask=255; command.frameLength=10;
        check(Motion_Command(&command,1000)==ACK,"accept calibrated 50 cm command");
        const int32_t observed=lround(499.5*nominal);
        for(int i=0;i<4;++i) ticks[i]=origin[i]+sign*observed;
        uint8_t telemetry[MOTION_TELEMETRY_SIZE]; Motion_Telemetry(telemetry);
        check(std::abs(read32(telemetry+5)/1000.0-sign*505.0)<physicalMmPerTick,"encoder travel formerly displayed as 49.95 cm now reports the measured 50.5 cm");
        check(std::abs(read32(telemetry+60)/1000000.0-TICKS_PER_MM)<0.000001,"telemetry transmits the calibrated tick scale to Qt");
    }
    std::printf("Calibration: scale %.9f, 50 cm goal %ld -> %ld ticks, predicted physical travel %.3f mm\n",double(WHEEL_TRAVEL_SCALE),long(previousGoal),long(correctedGoal),correctedGoal*physicalMmPerTick);
}

int main() {
    physicalDistanceCalibration();
    check(ARRIVAL_TOLERANCE_TICKS==5,"arrival tolerance is five ticks, independent of wheel calibration");
    for(int mode:{0,4,6,20}) for(int sign:{-1,1}) distanceArrival(mode,sign);
    for(bool pose:{false,true}) {
        for(int sign:{-1,1}) frictionCompensation(sign,pose);
        settlingDrift(pose);
    }
    std::printf("PASS: 50 cm arrival rejects 47 cm; +/-5 ticks (%.3f mm), signed PWM base + PI, drift and final detour arrival\n",ARRIVAL_TOLERANCE_MM);
}
