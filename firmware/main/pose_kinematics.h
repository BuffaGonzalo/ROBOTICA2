#ifndef POSE_KINEMATICS_H
#define POSE_KINEMATICS_H
#include <math.h>

// Body axes: forward, left; positive heading is counterclockwise, radians.
namespace PoseKinematics {
constexpr double POSE_PI=3.14159265358979323846;
struct Motion { double forward,left,turn; };
struct Pose {
    double forward,left,heading;
    Pose(double f=0,double l=0,double h=0):forward(f),left(l),heading(h) {}
};
inline double shortest(double angle) {
    angle=fmod(angle,2*POSE_PI);
    if(angle>POSE_PI) angle-=2*POSE_PI;
    if(angle < -POSE_PI) angle+=2*POSE_PI;
    return angle;
}
inline void coefficients(double angle,double &a,double &b) {
    if(fabs(angle)<0.01) {
        const double square=angle*angle;
        a=1-square/6+square*square/120;
        b=angle*(0.5-square/24+square*square/720);
    } else {
        a=sin(angle)/angle; b=(1-cos(angle))/angle;
    }
}
inline Motion logarithm(double forward,double left,double turn) {
    turn=shortest(turn);
    double a,b; coefficients(turn,a,b);
    const double denominator=a*a+b*b;
    return {(a*forward+b*left)/denominator,(-b*forward+a*left)/denominator,turn};
}
inline Pose exponential(Motion motion,double fraction=1) {
    const double angle=motion.turn*fraction;
    double a,b; coefficients(angle,a,b);
    return {fraction*(a*motion.forward-b*motion.left),fraction*(b*motion.forward+a*motion.left),angle};
}
inline bool wheels(Motion motion,double halfWheelbase,double halfTrack,double *out) {
    if(!isfinite(motion.forward) || !isfinite(motion.left) || !isfinite(motion.turn)
        || !isfinite(halfWheelbase) || !isfinite(halfTrack) || halfWheelbase<=0 || halfTrack<=0) return false;
    const double rotation=(halfWheelbase+halfTrack)*motion.turn;
    out[0]=motion.forward+motion.left+rotation;
    out[1]=motion.forward-motion.left-rotation;
    out[2]=motion.forward-motion.left+rotation;
    out[3]=motion.forward+motion.left-rotation;
    return true;
}
inline void integrate(Pose &pose,double forward,double left,double turn) {
    const Pose step=exponential({forward,left,turn});
    const double c=cos(pose.heading),s=sin(pose.heading);
    pose.forward+=c*step.forward-s*step.left;
    pose.left+=s*step.forward+c*step.left;
    pose.heading+=turn;
}
// Endpoint in the fixed body frame at MOVE start. XY follows a straight line;
// heading changes linearly with progress, so body-frame wheel ratios vary.
inline Pose linePose(Motion endpoint,double progress) {
    return {endpoint.forward*progress,endpoint.left*progress,endpoint.turn*progress};
}
inline double arcLength(Motion endpoint,double sweep) {
    const double chord=hypot(endpoint.forward,endpoint.left);
    return fabs(sweep)<1e-7 ? chord : chord*fabs(sweep)/(2*sin(fabs(sweep)/2));
}
inline Pose arcCenter(Motion endpoint,double sweep) {
    const double cot=cos(sweep/2)/sin(sweep/2);
    return {endpoint.forward/2-endpoint.left*cot/2,endpoint.left/2+endpoint.forward*cot/2,0};
}
inline Pose pathPose(Motion endpoint,double sweep,double progress) {
    if(fabs(sweep)<1e-7) return linePose(endpoint,progress);
    const Pose center=arcCenter(endpoint,sweep);
    const double a=sweep*progress,c=cos(a),s=sin(a);
    return {center.forward*(1-c)+center.left*s,center.left*(1-c)-center.forward*s,endpoint.turn*progress};
}
inline Motion pathDerivative(Motion endpoint,double sweep,double progress) {
    if(fabs(sweep)<1e-7) return endpoint;
    const Pose center=arcCenter(endpoint,sweep);
    const double a=sweep*progress,c=cos(a),s=sin(a);
    return {sweep*(center.forward*s+center.left*c),sweep*(center.left*s-center.forward*c),endpoint.turn};
}
inline double arcProgress(Motion endpoint,double sweep,Pose actual,double progressHint=-1) {
    if(fabs(sweep)<1e-7) return 0;
    const Pose center=arcCenter(endpoint,sweep);
    const double rf=actual.forward-center.forward,rl=actual.left-center.left;
    double angle=atan2(-center.forward*rl+center.left*rf,-center.forward*rf-center.left*rl);
    // Small backward deviations at the start are not almost a complete circle.
    if(progressHint>=0 && progressHint<0.5 && angle*sweep<0 && fabs(angle)<0.05) return 0;
    if(sweep>0 && angle<0) angle+=2*POSE_PI;
    if(sweep<0 && angle>0) angle-=2*POSE_PI;
    const double fraction=angle/sweep;
    return fraction<0 ? 0 : fraction>1 ? 1 : fraction;
}
inline bool lineWheelTravel(Motion endpoint,double progress,double halfWheelbase,double halfTrack,double *out) {
    const double angle=endpoint.turn*progress;
    double a,b; coefficients(angle,a,b);
    return wheels({progress*(a*endpoint.forward+b*endpoint.left),
        progress*(-b*endpoint.forward+a*endpoint.left),angle},halfWheelbase,halfTrack,out);
}
}
#endif
