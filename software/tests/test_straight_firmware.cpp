#include "../../firmware/main/motion.cpp"
#include <cstdio>
#include <cstdlib>
static void check(bool ok,const char *why) { if(!ok) { std::fprintf(stderr,"FAIL: %s\n",why); std::exit(1); } }
static int16_t output[4];
static unsigned brakeWrites=0;
static void apply(const int16_t *pwm) {
    if(!pwm[0] && !pwm[1] && !pwm[2] && !pwm[3]) ++brakeWrites; for(int i=0;i<4;++i) output[i]=pwm[i]; Motion_RecordPwm(pwm); }
static void simulate(double forward,double left,double turn,double sweep=0) {
    Motion_Init(apply);
    uint8_t data[256]={MOVE};
    put32(data+1,lround(forward*1000)); put32(data+5,lround(left*1000)); put32(data+13,lround(turn*1000000)); data[17]=6;
    if(sweep!=0) { data[17]=7; put32(data+18,lround(sweep*1000000)); }
    const uint8_t powers[4]={90,90,95,105};
    for(int i=0;i<4;++i) data[9+i]=powers[i];
    _sComm command={}; command.buff=data; command.mask=255; command.frameLength=sweep==0 ? 19 : 23;
    check(Motion_Command(&command,1000)==ACK && motionAxis==(sweep==0 ? 6 : 7),"accept straight or circular pose MOVE");
    const uint16_t generation=movementGeneration;
    data[17]=8; check(Motion_Command(&command,1000)==NACK && generation==movementGeneration,"reject unknown trajectory selector"); data[17]=sweep==0 ? 6 : 7;
    double velocity[4]={},distance[4]={}; int32_t base[4];
    for(int i=0;i<4;++i) base[i]=ticks[i];
    const double response[4]={0.95,1.0,0.90,1.05};
    double maxCrossTrack=0;
    const double length=hypot(forward,left);
    const auto center=sweep==0 ? PoseKinematics::Pose() : PoseKinematics::arcCenter({forward,left,turn},sweep);
    const double radius=hypot(center.forward,center.left);
    int sample=0;
    for(sample=1;sample<15000 && controller.state==RUNNING;++sample) {
        const uint32_t now=1000+sample*10;
        if(sample%100==0) controller.heartbeat=now;
        for(int i=0;i<4;++i) {
            const int magnitude=abs(output[i])-PWM_DEADZONE_BY_WHEEL[i];
            const double target=magnitude>0 ? (output[i]>0 ? 1 : -1)*magnitude/0.035*response[i] : 0;
            velocity[i]+=0.2*(target-velocity[i]); distance[i]+=velocity[i]*0.01;
            ticks[i]=base[i]+lround(distance[i]);
        }
        Motion_Task(now);
        if(sweep!=0) maxCrossTrack=fmax(maxCrossTrack,fabs(hypot(measuredPose.forward-center.forward,measuredPose.left-center.left)-radius));
        else if(length>0) maxCrossTrack=fmax(maxCrossTrack,fabs(forward*measuredPose.left-left*measuredPose.forward)/length);
    }
    std::printf("Pose %.0f/%.0f mm, %.0f deg: state %d, error %.2f mm / %.2f deg, max cross-track %.2f mm, %d ms\n",forward,left,turn*180/PoseKinematics::POSE_PI,
        controller.state,hypot(forward-measuredPose.forward,left-measuredPose.left),fabs(turn-measuredPose.heading)*180/PoseKinematics::POSE_PI,maxCrossTrack,sample*10);
    check(controller.state==ARRIVED,"straight pose controller reaches arrival margin with different motor responses");
    check(hypot(forward-measuredPose.forward,left-measuredPose.left)<ROUTE_ARRIVAL_MARGIN_MM+0.001,"settled final XY stays within the five-tick equivalent");
    check(fabs(turn-measuredPose.heading)<ROUTE_ARRIVAL_ANGLE_RAD+0.2*PoseKinematics::POSE_PI/180,"final angle inside margin");
    check(maxCrossTrack<15,"closed loop follows a line while body turns");
    for(int value:output) check(value==0,"arrival brakes all motors");
}
int main() {
    simulate(500,0,0); // regression: 50 cm must not finish at 47 cm
    simulate(-500,0,0);
    simulate(1000,0,PoseKinematics::POSE_PI);
    simulate(400,-300,-PoseKinematics::POSE_PI/2);
    simulate(0,0,PoseKinematics::POSE_PI/2);
    simulate(10,0,0); // short move must not be skipped by the 1 cm margin.
    simulate(400,0,0,PoseKinematics::POSE_PI); // semicircle without rotating chassis
    simulate(400,200,PoseKinematics::POSE_PI/2,-PoseKinematics::POSE_PI/2);
    simulate(10,0,0,359*PoseKinematics::POSE_PI/180); // almost closed circle must not finish at start
    // Vector destinations must not accept the former 30 mm arrival radius.
    Motion_Init(apply); uint8_t data[256]={MOVE}; put32(data+1,1000000); for(int i=0;i<4;++i) data[9+i]=120;
    _sComm command={}; command.buff=data; command.mask=255; command.frameLength=14;
    check(Motion_Command(&command,1000)==ACK,"legacy vector route");
    for(int i=0;i<4;++i) ticks[i]=origin[i]+distanceToTicks(975);
    for(int i=1;i<=12;++i) Motion_Task(1000+i*10);
    check(controller.state==RUNNING && !controller.settling,"vector route rejects arrival with 25 mm remaining");
    for(int i=0;i<4;++i) ticks[i]=origin[i]+controller.wheelTargets[i]-4;
    for(int i=13;i<=24;++i) Motion_Task(1000+i*10);
    check(controller.state==ARRIVED,"vector route accepts arrival only within five encoder ticks");
    // Continuous detours use explicit flags; ordinary MOVE retains its stop.
    put32(data+1,1000000); put32(data+13,0); data[17]=6; data[18]=1; command.frameLength=20;
    check(Motion_Command(&command,2000)==ACK && straightController.passThrough,"accept continuous waypoint");
    for(int i=0;i<4;++i) ticks[i]=origin[i]+distanceToTicks(975);
    Motion_Task(2010);
    check(controller.state==RUNNING && !controller.settling,"intermediate waypoint does not latch arrival or brake");
    for(int i=0;i<4;++i) { controller.demand[i]=400; controller.velocity[i]=380; controller.integral[i]=8; controller.pwm[i]=85; }
    straightController.phaseSpeed=400/straightController.boundTicks;
    const unsigned beforeBlend=brakeWrites;
    data[18]=2; put32(data+1,500000); put32(data+5,200000);
    check(Motion_Command(&command,2020)==ACK && !straightController.passThrough,"last detour leg still stops at user destination");
    check(brakeWrites==beforeBlend,"continuous successor never applies an all-wheel brake pulse");
    for(int i=0;i<4;++i) check(controller.demand[i]==400 && controller.velocity[i]==380 && controller.integral[i]==8 && output[i]==85,"handoff preserves motor control state instead of restarting ramp");
    data[18]=4; check(Motion_Command(&command,2020)==NACK,"unknown continuity flags rejected");
    data[18]=1; put32(data+1,1000000); put32(data+5,0);
    check(Motion_Command(&command,3000)==ACK,"start isolated intermediate waypoint");
    for(int i=0;i<4;++i) ticks[i]=origin[i]+distanceToTicks(1000);
    for(uint32_t now=3010;now<=3450;now+=10) Motion_Task(now);
    check(controller.state==ARRIVED,"missing successor stops at bounded waypoint after handoff grace");
    for(int value:output) check(value==0,"missing successor brakes every wheel");
    command.frameLength=19;
    // The new controller must preserve STOP, stall and timing/link watchdogs.
    put32(data+1,1000000); put32(data+13,1570796); data[17]=6; command.frameLength=19;
    check(Motion_Command(&command,1000)==ACK,"restart straight MOVE for safety tests");
    for(uint32_t now=1010;now<5000 && controller.state==RUNNING;now+=10) { controller.heartbeat=now; Motion_Task(now); }
    check(controller.state==STALLED && controller.faultMask,"missing encoder progress triggers stall");
    check(Motion_Command(&command,1000)==ACK,"restart after stall"); Motion_Task(1040);
    check(controller.state==TIMING_FAULT,"control period watchdog retained");
    check(Motion_Command(&command,1000)==ACK,"restart for link watchdog"); controller.last=4000; Motion_Task(4010);
    check(controller.state==LINK_LOST,"heartbeat watchdog retained");
    check(Motion_Command(&command,1000)==ACK,"restart for STOP"); Motion_Task(1010);
    data[0]=STOP; command.frameLength=2;
    check(Motion_Command(&command,1020)==ACK && controller.state==STOPPED,"STOP cancels straight pose MOVE");
    for(int value:output) check(value==0,"STOP immediately brakes every wheel");
    std::puts("PASS: straight trajectories with changing heading, short moves, pure rotation and route arrival margin");
}
