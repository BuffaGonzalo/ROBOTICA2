#include "../../firmware/main/pose_kinematics.h"
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
static void check(bool ok,const char *why) { if(!ok) { std::fprintf(stderr,"FAIL: %s\n",why); std::exit(1); } }
int main() {
    using namespace PoseKinematics;
    const double turns[]={0,1e-9,POSE_PI/4,-POSE_PI/2,POSE_PI,-POSE_PI,3*POSE_PI/2};
    for(double angle:turns) for(double f:{-1000.0,0.0,1000.0}) for(double l:{-500.0,0.0,500.0}) {
        const auto motion=logarithm(f,l,angle);
        const auto result=exponential(motion);
        check(fabs(result.forward-f)<1e-6 && fabs(result.left-l)<1e-6,"SE2 log and exp reach target");
        double wheel[4]; check(wheels(motion,100,80,wheel),"valid measured geometry");
        Pose measured;
        for(int sample=0;sample<100;++sample) {
            const double df=(wheel[0]+wheel[1]+wheel[2]+wheel[3])/400;
            const double dl=(wheel[0]-wheel[1]-wheel[2]+wheel[3])/400;
            const double dt=(wheel[0]-wheel[1]+wheel[2]-wheel[3])/(400*180);
            integrate(measured,df,dl,dt);
        }
        check(fabs(measured.forward-f)<1e-6 && fabs(measured.left-l)<1e-6,"wheel odometry reaches XY during rotation");
        check(fabs(shortest(measured.heading-angle))<1e-6,"wheel odometry reaches heading");
    }
    double wheel[4];
    for(double sweep:{POSE_PI/2,-POSE_PI/2,POSE_PI,-POSE_PI,359*POSE_PI/180}) {
        const Motion target={100,50,POSE_PI/2};
        const auto center=arcCenter(target,sweep);
        const double radius=hypot(center.forward,center.left);
        for(int sample=0;sample<=100;++sample) {
            const double fraction=sample/100.0;
            const auto pose=pathPose(target,sweep,fraction);
            check(fabs(hypot(pose.forward-center.forward,pose.left-center.left)-radius)<1e-8,"arc stays on circle");
            check(fabs(arcProgress(target,sweep,pose)-fraction)<1e-8,"arc progress handles both directions and sweeps over 180 degrees");
            if(sample==100) check(fabs(pose.forward-100)<1e-8 && fabs(pose.left-50)<1e-8,"arc reaches exact endpoint");
        }
    }
    // Changing heading must not curve the fixed-frame XY path.
    for(double angle:turns) {
        Pose pose; double previous[4]={};
        for(int sample=1;sample<=1000;++sample) {
            check(lineWheelTravel({1000,250,shortest(angle)},sample/1000.0,47.5,105,wheel),"straight pose wheel references");
            double delta[4]; for(int i=0;i<4;++i) { delta[i]=wheel[i]-previous[i]; previous[i]=wheel[i]; }
            integrate(pose,(delta[0]+delta[1]+delta[2]+delta[3])/4,(delta[0]-delta[1]-delta[2]+delta[3])/4,
                (delta[0]-delta[1]+delta[2]-delta[3])/(4*152.5));
            const auto expected=linePose({1000,250,shortest(angle)},sample/1000.0);
            check(fabs(pose.forward-expected.forward)<0.01 && fabs(pose.left-expected.left)<0.01,"straight XY at every intermediate heading");
        }
    }
    check(!wheels({0,0,1},0,80,wheel) && !wheels({0,0,1},100,0,wheel),"missing dimensions reject physical rotation");
    check(wheels({0,0,POSE_PI/2},100,80,wheel),"pure rotation");
    check(wheel[0]>0 && wheel[1]<0 && wheel[2]>0 && wheel[3]<0,"rotation uses opposite sides");
    std::puts("PASS: SE2 targets, combined translation/rotation, signed wheel odometry and missing geometry");
}
