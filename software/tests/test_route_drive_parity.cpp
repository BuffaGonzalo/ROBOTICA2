// Compare real firmware outputs for Configuration/Longitudinal and a plain
// XY setpoint at (0,100), using exactly the same unequal encoder samples.
#include "../../firmware/main/motion.cpp"
#include <cstdio>
#include <cstdlib>

static void check(bool ok,const char *why) {
    if(!ok) { std::fprintf(stderr,"FAIL: %s\n",why); std::exit(1); }
}
static int16_t output[4];
static void apply(const int16_t *pwm) {
    for(int i=0;i<4;++i) output[i]=pwm[i];
    Motion_RecordPwm(pwm);
}
struct Sample { int16_t pwm[4]; float demand[4],integral[4]; };
static Sample manual[400];

static void run(bool vector) {
    Motion_Init(apply);
    uint8_t data[256]={MOVE}; put32(data+1,1000000);
    const uint8_t limits[4]={170,170,175,185};
    for(int i=0;i<4;++i) data[(vector ? 9 : 5)+i]=limits[i];
    _sComm command={}; command.buff=data; command.mask=255; command.frameLength=vector ? 14 : 10;
    check(Motion_Command(&command,1000)==ACK,"accept 100 cm comparison move");
    for(int step=1;step<=400;++step) {
        const uint32_t now=1000+step*10;
        for(int wheel=0;wheel<4;++wheel) {
            const int skew[4]={0,-3,2,-1};
            ticks[wheel]=origin[wheel]+9*step+skew[wheel]*(step%11);
        }
        controller.heartbeat=now; Motion_Task(now);
        check(controller.state==RUNNING,"comparison encoder trace remains in motion");
        for(int wheel=0;wheel<4;++wheel) {
            if(!vector) {
                manual[step-1].pwm[wheel]=output[wheel];
                manual[step-1].demand[wheel]=controller.demand[wheel];
                manual[step-1].integral[wheel]=controller.integral[wheel];
            } else {
                check(output[wheel]==manual[step-1].pwm[wheel],"same PWM at every wheel and sample");
                check(fabs(controller.demand[wheel]-manual[step-1].demand[wheel])<0.0001,"same acceleration and speed demand");
                check(fabs(controller.integral[wheel]-manual[step-1].integral[wheel])<0.0001,"same PI memory and synchronization response");
            }
        }
    }
}
int main() {
    run(false); run(true);
    std::puts("PASS: longitudinal 100 cm and plain (0,100) setpoint produce identical wheel PWM, ramp and PI for 400 unequal encoder samples at 170/170/175/185");
}
