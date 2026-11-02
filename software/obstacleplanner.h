#ifndef OBSTACLEPLANNER_H
#define OBSTACLEPLANNER_H
#include <QPointF>
#include <QLineF>
#include <QList>
#include <QString>
#include <queue>
#include <vector>
#include <limits>
#include <cmath>
#include "../firmware/main/pose_kinematics.h"
#include "../firmware/main/navigation_limits.h"

struct MapObstacle {
    QPointF center;
    bool circular=true;
    double sizeCm=20; // diameter or square side
};
struct AvoidanceStep {
    QPointF start,end;
    double startHeading=0,endHeading=0,arcSweep=0;
    int completedCount=0; // zero for generated intermediate waypoints
    bool detour=false;
};
struct AvoidanceRoute {
    QList<AvoidanceStep> steps;
    bool valid=true;
    QString error;
};

class ObstaclePlanner {
public:
    QList<MapObstacle> obstacles;
    double robotLength=30,robotWidth=19.8,safetyDistance=10,reserve=(NavigationLimits::PassThroughMarginMm+NavigationLimits::TrackingReserveMm)/10;
    double clearance() const { return std::hypot(robotLength,robotWidth)/2+safetyDistance+reserve; }
    bool pointFree(QPointF point,double extra=0) const {
        if(std::abs(point.x())>1000 || std::abs(point.y())>1000) return false;
        for(const auto &o:obstacles) {
            const double extent=o.sizeCm/2+clearance()+extra;
            const QPointF d=point-o.center;
            if(o.circular ? std::hypot(d.x(),d.y())<=extent+1e-7
                          : std::abs(d.x())<=extent+1e-7 && std::abs(d.y())<=extent+1e-7) return false;
        }
        return true;
    }
    bool segmentFree(QPointF start,QPointF end,double extra=0) const {
        if(!pointFree(start,extra) || !pointFree(end,extra)) return false;
        const QPointF d=end-start;
        const double lengthSquared=QPointF::dotProduct(d,d);
        for(const auto &o:obstacles) {
            const double extent=o.sizeCm/2+clearance()+extra;
            if(o.circular) {
                const double projection=lengthSquared>0 ? qBound(0.0,QPointF::dotProduct(o.center-start,d)/lengthSquared,1.0) : 0;
                if(QLineF(start+d*projection,o.center).length()<=extent+1e-7) return false;
            } else {
                double enter=0,leave=1;
                bool intersects=true;
                const double origin[2]={start.x()-o.center.x(),start.y()-o.center.y()};
                const double direction[2]={d.x(),d.y()};
                for(int axis=0;axis<2;++axis) {
                    if(std::abs(direction[axis])<1e-12) {
                        if(std::abs(origin[axis])>extent) { intersects=false; break; }
                    } else {
                        const double a=(-extent-origin[axis])/direction[axis],b=(extent-origin[axis])/direction[axis];
                        enter=qMax(enter,qMin(a,b)); leave=qMin(leave,qMax(a,b));
                        if(enter>leave) { intersects=false; break; }
                    }
                }
                if(intersects) return false;
            }
        }
        return true;
    }
    static QPointF arcPoint(QPointF start,QPointF end,double sweep,double fraction) {
        const QPointF d=end-start;
        const auto pose=PoseKinematics::pathPose({d.y(),-d.x(),0},sweep*PoseKinematics::POSE_PI/180,fraction);
        return start+QPointF(-pose.left,pose.forward);
    }
    bool pathFree(QPointF start,QPointF end,double sweep) const {
        if(sweep==0) return segmentFree(start,end);
        const double chord=QLineF(start,end).length(),angle=std::abs(sweep)*PoseKinematics::POSE_PI/180;
        if(chord<1e-7 || angle<=0 || angle>=2*PoseKinematics::POSE_PI) return false;
        const double radius=chord/(2*std::sin(angle/2)),length=radius*angle;
        const int count=qMax(8,int(std::ceil(length/2)));
        if(count>10000) return false;
        // Inflate by the sagitta so checking the chords also protects the arc.
        const double sagitta=radius*(1-std::cos(angle/(2*count)))+0.001;
        QPointF previous=start;
        for(int i=1;i<=count;++i) {
            const QPointF at=arcPoint(start,end,sweep,double(i)/count);
            if(!segmentFree(previous,at,sagitta)) return false;
            previous=at;
        }
        return true;
    }
    QList<QPointF> findPath(QPointF start,QPointF end) const {
        if(!pointFree(start) || !pointFree(end)) return {};
        if(segmentFree(start,end)) return {start,end};
        QList<QPointF> nodes={start,end};
        for(const auto &o:obstacles) {
            const double inflated=o.sizeCm/2+clearance();
            if(o.circular) {
                // Circumscribed polygon: all edges stay outside the circle.
                const double radius=inflated/std::cos(PoseKinematics::POSE_PI/16)+2;
                for(int i=0;i<16;++i) {
                    const double angle=i*2*PoseKinematics::POSE_PI/16;
                    const QPointF point=o.center+QPointF(std::cos(angle),std::sin(angle))*radius;
                    if(pointFree(point)) nodes.append(point);
                }
            } else {
                for(int x:{-1,1}) for(int y:{-1,1}) {
                    const QPointF point=o.center+QPointF(x,y)*(inflated+2);
                    if(pointFree(point)) nodes.append(point);
                }
            }
        }
        struct Entry { double score; int index; bool operator<(const Entry &other) const { return score>other.score; } };
        const int count=nodes.size();
        std::vector<double> distance(count,std::numeric_limits<double>::infinity());
        std::vector<int> previous(count,-1);
        std::vector<bool> closed(count,false);
        std::priority_queue<Entry> open;
        distance[0]=0; open.push({QLineF(start,end).length(),0});
        while(!open.empty()) {
            const int current=open.top().index; open.pop();
            if(closed[current]) continue;
            if(current==1) {
                QList<QPointF> result;
                for(int at=1;at!=-1;at=previous[at]) result.prepend(nodes[at]);
                return result;
            }
            closed[current]=true;
            for(int next=0;next<count;++next) {
                if(closed[next]) continue;
                const double candidate=distance[current]+QLineF(nodes[current],nodes[next]).length();
                if(candidate+1e-8>=distance[next] || !segmentFree(nodes[current],nodes[next])) continue;
                distance[next]=candidate; previous[next]=current;
                open.push({candidate+QLineF(nodes[next],end).length(),next});
            }
        }
        return {};
    }
    AvoidanceRoute planSegment(QPointF start,QPointF end,double fromHeading,double toHeading,double sweep,int completed) const {
        AvoidanceRoute result;
        if(obstacles.isEmpty() || pathFree(start,end,sweep)) {
            result.steps.append({start,end,fromHeading,toHeading,sweep,completed,false}); return result;
        }
        const auto path=findPath(start,end);
        if(path.isEmpty()) {
            result.valid=false;
            result.error="No hay paso libre: el robot o el destino quedan dentro de la zona de seguridad, o los obstáculos bloquean la ruta.";
            return result;
        }
        double total=0; for(int i=1;i<path.size();++i) total+=QLineF(path[i-1],path[i]).length();
        const double turn=std::remainder(toHeading-fromHeading,360.0);
        double travelled=0,heading=fromHeading;
        for(int i=1;i<path.size();++i) {
            travelled+=QLineF(path[i-1],path[i]).length();
            const double nextHeading=fromHeading+(total>0 ? turn*travelled/total : turn);
            result.steps.append({path[i-1],path[i],heading,nextHeading,0,i==path.size()-1 ? completed : 0,true});
            heading=nextHeading;
        }
        return result;
    }
};
#endif
