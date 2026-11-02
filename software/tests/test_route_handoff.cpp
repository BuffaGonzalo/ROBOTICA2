#include <QApplication>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QUdpSocket>
#include <QtEndian>
#include "routeplanwidget.h"
#define private public
#include "mainwindow.h"
#undef private
#include <cstdio>
#include <cstdlib>
static void check(bool ok,const char *why) {
    if(!ok) { std::fprintf(stderr,"FAIL: %s\n",why); std::exit(1); }
}
static QByteArray frame(QByteArray payload) {
    QByteArray result("UNER"); result+=char(payload.size()+1); result+=':'; result+=payload;
    quint8 sum=0; for(char byte:result) sum^=quint8(byte); result+=char(sum); return result;
}
static void ack(MainWindow &w) { auto p=frame(QByteArray::fromHex("d00d")); w.processFrames(p,"Test"); }
static QByteArray status(MainWindow &w,QPointF position,int state=1,double heading=0) {
    QByteArray payload(124,'\0'); payload[0]=char(0xD2); payload[1]=char(state); payload[93]=char(w.mapMode);
    qToLittleEndian<qint32>(w.routeForwardUm,payload.data()+94); qToLittleEndian<qint32>(w.routeLeftUm,payload.data()+98);
    qToLittleEndian<quint16>(w.expectedGeneration<0 ? 0 : w.expectedGeneration,payload.data()+110);
    qToLittleEndian<qint32>(w.routeTurnUrad,payload.data()+112); qToLittleEndian<qint32>(w.routeArcSweepUrad,payload.data()+120);
    const QPointF d=position-w.mapOrigin;
    const double h=w.mapHeadingDeg*PoseKinematics::POSE_PI/180;
    qToLittleEndian<qint32>(qRound((d.y()*std::cos(h)-d.x()*std::sin(h))*10000),payload.data()+102);
    qToLittleEndian<qint32>(qRound((-d.x()*std::cos(h)-d.y()*std::sin(h))*10000),payload.data()+106);
    qToLittleEndian<qint32>(qRound((heading-w.mapHeadingDeg)*PoseKinematics::POSE_PI/180*1000000),payload.data()+116);
    return payload;
}
static void deliver(MainWindow &w,QByteArray payload) { auto p=frame(payload); w.processFrames(p,"Test"); }
int main(int argc,char **argv) {
    QApplication app(argc,argv); MainWindow window;
    window.findChild<QComboBox *>("comboBox_protocol")->setCurrentIndex(1);
    window.findChild<QLineEdit *>("lineEdit_device_ip")->setText("127.0.0.1");
    window.findChild<QLineEdit *>("lineEdit_device_port")->setText("39001");
    check(window.udpSocket->bind(QHostAddress::LocalHost,0),"local test transport"); window.updateConnectionUi();
    auto *map=static_cast<RoutePlanWidget *>(window.findChild<QWidget *>("routePlan"));
    const auto reset=[&]() {
        window.cancelRoute("test"); window.motionRunning=false; window.routePaused=false; window.stopPending=false;
        window.findChild<QTimer *>("stopRetryTimer")->stop(); window.updateCommandUi();
        window.findChild<QPushButton *>("pushButton_resetMap")->click();
        window.findChild<QPushButton *>("pushButton_routeOrientation")->setChecked(false);
        map->arcSweeps.clear(); map->headings.clear();
    };
    // Ordinary user nodes now pass through, just like generated avoidance nodes.
    map->points={{0,100},{0,200},{0,300}}; map->changed(); window.startRoute(); ack(window);
    deliver(window,status(window,{0,98.9}));
    check(window.routeIndex==0 && window.routeActive,"11 mm remaining is outside the pass-through band");
    const auto oldSample=status(window,{0,99}); deliver(window,oldSample);
    check(window.routeIndex==1 && map->completed==1 && window.routeActive && !window.routeAck,"10 mm remaining automatically sends the next MOVE");
    check(!window.routePauseTimer->isActive() && window.mapMode==0 && window.routeForwardUm==1010000,"no 500 ms pause; next longitudinal leg includes the 1 cm residual");
    deliver(window,oldSample);
    auto oldFault=oldSample; oldFault[1]=char(9); deliver(window,oldFault);
    check(window.routeActive && window.routeIndex==1,"late telemetry or overshoot from the previous generation cannot cancel or advance the new leg");
    ack(window); deliver(window,status(window,{0,199}));
    check(window.routeActive && window.routeIndex==2 && map->completed==2 && !window.routePauseTimer->isActive(),"second user node also chains without another Execute click");
    ack(window); deliver(window,status(window,{0,299}));
    check(window.routeActive && window.routeIndex==2 && map->completed==2,"last destination is not consumed 10 mm early");
    deliver(window,status(window,{0,300},2));
    check(map->completed==3 && window.routeIndex==3,"final destination still requires firmware ARRIVED");
    window.routePauseTimer->stop(); window.nextRouteLeg(); check(!window.routeActive,"route completes once");

    reset(); map->points={{0,100},{50,100}}; map->changed(); window.startRoute(); ack(window);
    deliver(window,status(window,{0,100},2));
    check(window.routeIndex==1 && window.routePauseTimer->interval()==0,"ARRIVED at a middle node also schedules immediate continuation");
    window.routePauseTimer->stop(); window.nextRouteLeg();
    check(window.routeActive && window.routeIndex==1,"confirmed corner advances without pressing Execute again");

    reset(); map->points={{0,100},{0,200}}; map->changed(); window.startRoute(); ack(window);
    deliver(window,status(window,{0,99},4));
    check(!window.routeActive && map->completed==0,"a current stalled-wheel fault is never converted into successful handoff");

    reset(); map->points={{0,2},{0,10}}; map->changed(); window.startRoute(); ack(window);
    deliver(window,status(window,{0,1})); check(window.routeIndex==0,"short-node band is capped at 25 percent of the leg");
    deliver(window,status(window,{0,1.5})); check(window.routeIndex==1 && window.routeActive,"short node passes only after sufficient actual travel");

    reset(); map->points={{0,100},{0,200}}; map->headings={90,90};
    window.findChild<QPushButton *>("pushButton_routeOrientation")->setChecked(true);
    map->changed(); window.startRoute(); ack(window);
    deliver(window,status(window,{0,99},1,80)); check(window.routeIndex==0,"orientation must be reached before passing an oriented node");
    deliver(window,status(window,{0,99},1,90)); check(window.routeIndex==1,"oriented intermediate node automatically continues when angle is reached");

    reset(); map->points={{0,1},{0,10}}; map->arcSweeps={359,0}; map->changed(); window.startRoute(); ack(window);
    deliver(window,status(window,{0,0}));
    check(window.routeIndex==0 && window.mapArcProgress<0.9,"almost closed arc cannot skip its middle node at the beginning");
    window.cancelRoute("test"); window.motionRunning=false;
    std::puts("PASS: automatic 10 mm user-node handoff, residual carry, no intermediate pause, stale-fault filtering, strict final arrival, short legs, orientation and arc guards");
}
