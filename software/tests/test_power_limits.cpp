#include "../../firmware/main/motion.cpp"
#include "../../firmware/main/robot_commands.c"
#include <cstdio>
#include <cstdlib>
static void check(bool ok,const char *why) { if(!ok) { std::fprintf(stderr,"FAIL: %s\n",why); std::exit(1); } }
static int16_t output[4];
static void apply(const int16_t *powers) { for(int i=0;i<4;++i) output[i]=powers[i]; Motion_RecordPwm(powers); }
int main() {
    Motion_Init(apply); RobotCommands_Init(apply); RobotCommands_AttachMotion(Motion_Command);
    uint8_t data[256]={MOVE}; put32(data+1,1000000); put32(data+13,1570796); data[17]=6;
    for(int i=0;i<4;++i) data[9+i]=255;
    _sComm command={}; command.buff=data; command.mask=255; command.frameLength=19;
    check(RobotCommands_Handle(&command,1000)==ACK,"start real straight MOVE"); Motion_Task(1010);
    for(int i=0;i<4;++i) check(controller.pwmLimit[i]==255,"setpoint MOVE accepts 255 PWM without fixed 185 cap");
    const auto target=requestedForwardUm; const auto generation=movementGeneration; const auto progress=straightController.progress;
    for(int i=0;i<4;++i) controller.pwm[i]=(i%2 ? -1 : 1)*150;
    uint8_t limits[256]={SETLIMITS,20,40,60,80}; _sComm update={}; update.buff=limits; update.mask=255; update.frameLength=6;
    check(RobotCommands_Handle(&update,1020)==ACK,"dispatch and apply live limits");
    check(controller.state==RUNNING && generation==movementGeneration && requestedForwardUm==target && progress==straightController.progress,"limits preserve goal, generation and trajectory progress");
    for(int i=0;i<4;++i) check(controller.pwmLimit[i]==limits[1+i] && abs(output[i])<=limits[1+i],"new lower limits clamp motor output immediately");
    limits[1]=0; check(RobotCommands_Handle(&update,1030)==NACK,"zero active-wheel limit rejected; STOP required");
    check(controller.pwmLimit[0]==20 && controller.pwmLimit[3]==80,"invalid limits cannot partially modify control"); limits[1]=20;
    update.frameLength=7; check(RobotCommands_Handle(&update,1030)==NACK,"wrong SETLIMITS size rejected"); update.frameLength=6;
    data[0]=STOP; command.frameLength=2; check(RobotCommands_Handle(&command,1040)==ACK,"STOP after speed change");
    check(RobotCommands_Handle(&update,1050)==NACK && controller.state==STOPPED,"live limits cannot start an idle robot");
    for(int value:output) check(value==0,"STOP remains braked after rejected update");
    // Fixed-ratio vector moves store normalized PWM but drivers need wheel signs.
    data[0]=MOVE; command.frameLength=14; put32(data+1,0); put32(data+5,-1000000);
    check(RobotCommands_Handle(&command,2000)==ACK,"start signed lateral vector");
    for(int i=0;i<4;++i) controller.pwm[i]=100;
    check(RobotCommands_Handle(&update,2010)==ACK,"live limits in vector mode");
    for(int i=0;i<4;++i) check(output[i]==controller.pwm[i]*travelSign(i),"live update preserves physical wheel direction");
    for(int i=0;i<4;++i) limits[1+i]=255;
    check(RobotCommands_Handle(&update,2020)==ACK,"live speed update accepts full PWM range");
    for(int i=0;i<4;++i) check(controller.pwmLimit[i]==255,"live limits are not capped at the default PWM");
    std::puts("PASS: live PWM limits dispatch, immediate clamping, unchanged trajectory, atomic validation, STOP and vector signs");
}
