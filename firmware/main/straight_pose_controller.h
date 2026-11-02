#ifndef STRAIGHT_POSE_CONTROLLER_H
#define STRAIGHT_POSE_CONTROLLER_H
#include "motion_controller.h"
#include "pose_kinematics.h"

// Tracks a line in the initial fixed frame while rotating the body. Wheel
// directions may change during a MOVE, unlike fixed-ratio vector/arc moves.
struct StraightPoseController {
    PoseKinematics::Motion endpoint={0,0,0};
    double halfWheelbase=0,halfTrack=0;
    double arcSweep=0;
    float progress=0,phaseSpeed=0,boundTicks=1;
    bool passThrough=false;
    uint32_t waypointSince=0;
    uint32_t wrongDirectionSince[4]={};
    void start(MotionController &c,PoseKinematics::Motion target,double l,double w,uint32_t now,const uint8_t *limits,double sweep=0,bool through=false,bool blend=false) {
        float oldVelocity[4],oldIntegral[4],oldDemand[4]; int16_t oldPwm[4];
        const float oldSpeed=phaseSpeed*boundTicks;
        for(uint8_t i=0;i<4;++i) {
            oldVelocity[i]=c.velocity[i]; oldIntegral[i]=c.integral[i];
            oldDemand[i]=c.demand[i]; oldPwm[i]=c.pwm[i];
        }
        passThrough=through; waypointSince=0;
        endpoint=target; halfWheelbase=l; halfTrack=w;
        arcSweep=sweep;
        progress=phaseSpeed=0;
        // Bounds every wheel's speed over the entire translation + rotation.
        boundTicks=(sqrt(2.0)*PoseKinematics::arcLength(target,sweep)+(l+w)*fabs(target.turn))*TICKS_PER_MM;
        if(boundTicks<1) boundTicks=1;
        c.startWithLimits((int32_t)ceil(boundTicks),now,limits);
        if(blend) {
            phaseSpeed=fmin(oldSpeed,MAX_TICKS_S)/boundTicks;
            for(uint8_t i=0;i<4;++i) {
                c.velocity[i]=oldVelocity[i]; c.integral[i]=oldIntegral[i]; c.demand[i]=oldDemand[i];
                c.pwm[i]=(int16_t)bounded(oldPwm[i],-limits[i],limits[i]);
            }
        }
        for(uint8_t i=0;i<4;++i) wrongDirectionSince[i]=0;
    }
    void update(MotionController &c,const int32_t *counts,const PoseKinematics::Pose &pose,uint32_t now) {
        if(c.state!=RUNNING) return;
        const uint32_t elapsed=now-c.last;
        if(elapsed<10) return;
        if(elapsed>30) { c.stop(TIMING_FAULT); return; }
        c.last=now;
        if((uint32_t)(now-c.heartbeat)>3000) { c.stop(LINK_LOST); return; }
        if((uint32_t)(now-c.started)>c.moveBudgetMs) { c.stop(MOVE_TIMEOUT); return; }
        const double length=PoseKinematics::arcLength(endpoint,arcSweep);
        const double positionMargin=length>0 ? fmin(ROUTE_ARRIVAL_MARGIN_MM,length/4) : ROUTE_ARRIVAL_MARGIN_MM;
        const double angularMargin=fabs(endpoint.turn)>0 ? fmin(ROUTE_ARRIVAL_ANGLE_RAD,fabs(endpoint.turn)/4) : ROUTE_ARRIVAL_ANGLE_RAD;
        const bool inside=(arcSweep==0 || progress>=0.9f) && hypot(endpoint.forward-pose.forward,endpoint.left-pose.left)<=positionMargin
            && fabs(endpoint.turn-pose.heading)<=angularMargin;
        if(c.settling) {
            for(uint8_t i=0;i<4;++i) c.pwm[i]=0;
            if((uint32_t)(now-c.settledSince)>=100) c.stop(inside ? ARRIVED : OVERSHOOT);
            return;
        }
        // Give Qt a telemetry period to replace an internal waypoint. If no
        // successor arrives, keep the reference bounded at the endpoint and stop.
        if(inside && !waypointSince) waypointSince=now;
        if(!inside) waypointSince=0;
        if(inside && (!passThrough || (uint32_t)(now-waypointSince)>=300u)) {
            c.settling=true; c.settledSince=now; c.arrivedMask=15;
            for(uint8_t i=0;i<4;++i) { c.pwm[i]=0; c.integral[i]=c.demand[i]=0; }
            return;
        }
        const float dt=elapsed/1000.0f;
        const float acceleration=ACCEL_TICKS_S2/boundTicks;
        const float remaining=1-progress;
        const float targetSpeed=passThrough ? MAX_TICKS_S/boundTicks
            : fmin(MAX_TICKS_S/boundTicks,sqrt(2*acceleration*(remaining>0 ? remaining : 0)));
        phaseSpeed+=bounded(targetSpeed-phaseSpeed,-acceleration*dt,acceleration*dt);
        float next=bounded(progress+phaseSpeed*dt,0,1);
        // Keep the reference close to the measured robot when power saturates.
        if(length>0) {
            const float measured=arcSweep==0 ? (pose.forward*endpoint.forward+pose.left*endpoint.left)/(length*length)
                : PoseKinematics::arcProgress(endpoint,arcSweep,pose,progress);
            const float lead=fmin(0.1,20.0/length);
            next=fmin(next,fmax(progress,measured+lead));
        }
        const float rate=(next-progress)/dt;
        progress=next;
        const auto reference=PoseKinematics::pathPose(endpoint,arcSweep,progress);
        const auto derivative=PoseKinematics::pathDerivative(endpoint,arcSweep,progress);
        const double fixedForward=derivative.forward*rate+c.kpPosition*(reference.forward-pose.forward);
        const double fixedLeft=derivative.left*rate+c.kpPosition*(reference.left-pose.left);
        const double turn=endpoint.turn*rate+c.kpPosition*(reference.heading-pose.heading);
        const double cosine=cos(pose.heading),sine=sin(pose.heading);
        double desired[4];
        if(!PoseKinematics::wheels({cosine*fixedForward+sine*fixedLeft,-sine*fixedForward+cosine*fixedLeft,turn},halfWheelbase,halfTrack,desired)) {
            c.stop(TIMING_FAULT); return;
        }
        float peak=0;
        for(uint8_t i=0;i<4;++i) { desired[i]*=TICKS_PER_MM; if(fabs(desired[i])>peak) peak=fabs(desired[i]); }
        const float scale=peak>MAX_TICKS_S ? MAX_TICKS_S/peak : 1;
        for(uint8_t i=0;i<4;++i) {
            c.position[i]=counts[i];
            c.velocity[i]+=0.35f*((counts[i]-c.previous[i])/dt-c.velocity[i]);
            c.previous[i]=counts[i];
            if(labs(counts[i]-c.progressMark[i])>=4) { c.progressMark[i]=counts[i]; c.lastProgress[i]=now; }
            c.demand[i]+=bounded(desired[i]*scale-c.demand[i],-ACCEL_TICKS_S2*dt,ACCEL_TICKS_S2*dt);
            if(fabs(c.demand[i])<60) c.lastProgress[i]=now;
            if((uint32_t)(now-c.lastProgress[i])>1500) { c.faultMask|=1u<<i; c.stop(STALLED); return; }
            if(fabs(c.demand[i])>100 && fabs(c.velocity[i])>50 && c.demand[i]*c.velocity[i]<0) {
                if(!wrongDirectionSince[i]) wrongDirectionSince[i]=now;
                if((uint32_t)(now-wrongDirectionSince[i])>500) { c.faultMask|=1u<<i; c.stop(DIRECTION_FAULT); return; }
            } else wrongDirectionSince[i]=0;
            const float error=c.demand[i]-c.velocity[i];
            const float trial=bounded(c.integral[i]+c.kiSpeed*error*dt,-50,50);
            const float friction=c.demand[i]>0.01f ? PWM_DEADZONE_BY_WHEEL[i] : c.demand[i]<-0.01f ? -PWM_DEADZONE_BY_WHEEL[i] : 0;
            const float base=c.demand[i]*0.035f+c.kpSpeed*error+friction;
            const float limit=c.pwmLimit[i];
            if(!((base+trial>limit && error>0) || (base+trial < -limit && error<0))) c.integral[i]=trial;
            c.pwm[i]=(int16_t)bounded(base+c.integral[i],-limit,limit);
        }
    }
};
#endif
