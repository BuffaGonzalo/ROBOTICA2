// Compile with -Isoftware/tests/avr_stubs. Executes the actual motion module
// with synthetic encoder counts; no motor or physical transport is attached.
#include "../../firmware/main/motion.cpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
static void check(bool ok,const char *why) { if(!ok) { std::fprintf(stderr,"FAIL: %s\n",why); std::exit(1); } }
static int16_t output[4];
static void apply(const int16_t *pwm) { for(int i=0;i<4;++i) output[i]=pwm[i]; Motion_RecordPwm(pwm); }
static int32_t read32(const uint8_t *data) {
    uint32_t result=0; for(int i=0;i<4;++i) result|=uint32_t(data[i])<<(8*i); return int32_t(result);
}
int main() {
    Motion_Init(apply);
    check(MECANUM_HALF_WHEELBASE_MM==47.5f && MECANUM_HALF_TRACK_MM==105.0f,"measured chassis semidimensions");
    uint8_t data[256]={MOVE}; for(int i=0;i<4;++i) data[9+i]=120;
    const auto motion=PoseKinematics::logarithm(1000,0,PoseKinematics::POSE_PI/2);
    put32(data+1,lround(motion.forward*1000)); put32(data+5,lround(motion.left*1000)); put32(data+13,1570796);
    _sComm command={}; command.buff=data; command.mask=255; command.frameLength=18;
    check(Motion_Command(&command,1000)==ACK && motionAxis==5,"accept oriented MOVE on measured chassis");
    const uint16_t generation=movementGeneration;
    put32(data+13,3141594);
    check(Motion_Command(&command,1000)==NACK && movementGeneration==generation,"reject excessive rotation without replacing active move");
    put32(data+13,1570796);
    int32_t goals[4]; for(int i=0;i<4;++i) goals[i]=controller.wheelTargets[i]*vectorSign[i];
    for(int sample=1;sample<=100;++sample) {
        for(int i=0;i<4;++i) ticks[i]=lround(goals[i]*sample/100.0);
        Motion_Task(1000+sample*10);
    }
    for(int sample=101;sample<=115;++sample) Motion_Task(1000+sample*10);
    check(controller.state==ARRIVED,"coordinated wheel goals arrive");
    uint8_t status[MOTION_TELEMETRY_SIZE+8]; std::memset(status,0xA5,sizeof(status));
    check(Motion_Telemetry(status)==MOTION_TELEMETRY_SIZE && status[92]==5,"extended pose telemetry size and mode");
    check(std::abs(read32(status+101)-1000000)<1000 && std::abs(read32(status+105))<1000,"encoder integration reaches requested body-frame endpoint");
    check(std::abs(read32(status+115)-1570796)<2000 && read32(status+111)==1570796,"requested and measured yaw telemetry");
    for(unsigned i=MOTION_TELEMETRY_SIZE;i<sizeof(status);++i) check(status[i]==0xA5,"telemetry respects buffer boundary");
    // A pure rotation uses opposite signs on left and right motor pairs.
    put32(data+1,0); put32(data+5,0); put32(data+13,-1570796);
    check(Motion_Command(&command,3000)==ACK,"pure rotation accepted");
    check(vectorSign[0]<0 && vectorSign[1]>0 && vectorSign[2]<0 && vectorSign[3]>0,"pure rotation motor signs");
    data[0]=STOP; command.frameLength=2;
    check(Motion_Command(&command,3010)==ACK && controller.state==STOPPED,"STOP cancels oriented movement");
    for(int value:output) check(value==0,"STOP brakes every wheel");
    data[0]=MOVE; command.frameLength=14; put32(data+1,100000); put32(data+5,0);
    check(Motion_Command(&command,4000)==ACK && motionAxis==4,"legacy vector MOVE still supported");
    Motion_Telemetry(status); check(read32(status+111)==0 && read32(status+115)==0,"legacy mode has no yaw delta");
    std::puts("PASS: actual firmware MOVE/STOP validation, coordinated encoder arrival, pose telemetry and legacy vector mode");
}
