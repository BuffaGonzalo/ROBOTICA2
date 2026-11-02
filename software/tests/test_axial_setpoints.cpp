#include <QApplication>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QUdpSocket>
#include <QTimer>
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
static QByteArray takeMove(QUdpSocket &receiver,const char *context) {
    for(int attempt=0;attempt<12;++attempt) {
        if(!receiver.hasPendingDatagrams() && !receiver.waitForReadyRead(1000)) {
            std::fprintf(stderr,"Waiting for %s\n",context);
            check(false,"receive actual Qt UDP command");
        }
        QByteArray packet(receiver.pendingDatagramSize(),'\0'); receiver.readDatagram(packet.data(),packet.size());
        if(packet.size()>6 && quint8(packet[6])==0xD0) return packet;
    }
    check(false,"find MOVE datagram"); return {};
}
static void arrive(MainWindow &window) {
    auto packet=frame(QByteArray::fromHex("d00d")); window.processFrames(packet,"Test");
    QByteArray status(124,'\0'); status[0]=char(0xD2); status[1]=char(2); status[93]=char(window.mapMode);
    const bool route=window.routeActive;
    const qint32 forward=route ? window.routeForwardUm : window.mapMode==0 ? window.findChild<QSpinBox *>("spinBox_distance")->value()*10000 : 0;
    const qint32 left=route ? window.routeLeftUm : window.mapMode==1 ? window.findChild<QSpinBox *>("spinBox_distance")->value()*10000 : 0;
    qToLittleEndian<qint32>(forward ? forward : left,status.data()+6);
    qToLittleEndian<qint32>(forward,status.data()+94); qToLittleEndian<qint32>(left,status.data()+98);
    qToLittleEndian<qint32>(forward,status.data()+102); qToLittleEndian<qint32>(left,status.data()+106);
    const int generation=route ? window.expectedGeneration : (window.lastGeneration+1)&65535;
    qToLittleEndian<quint16>(generation,status.data()+110);
    packet=frame(status); window.processFrames(packet,"Test");
}
int main(int argc,char **argv) {
    QApplication app(argc,argv);
    MainWindow window; QUdpSocket receiver;
    check(receiver.bind(QHostAddress::LocalHost,0),"bind command receiver");
    window.findChild<QComboBox *>("comboBox_protocol")->setCurrentIndex(1);
    window.findChild<QLineEdit *>("lineEdit_device_ip")->setText("127.0.0.1");
    window.findChild<QLineEdit *>("lineEdit_device_port")->setText(QString::number(receiver.localPort()));
    check(window.udpSocket->bind(QHostAddress::LocalHost,0),"bind Qt test transport"); window.updateConnectionUi();
    auto *map=static_cast<RoutePlanWidget *>(window.findChild<QWidget *>("routePlan"));
    auto *tabs=window.findChild<QTabWidget *>("tabWidget_wifi");
    const char *names[]={"fr","fl","rr","rl"}; const int limits[]={170,170,175,185};
    for(int i=0;i<4;++i) window.findChild<QSpinBox *>(QString("spinBox_move_")+names[i])->setValue(limits[i]);
    for(int axis:{0,1}) for(int sign:{-1,1}) {
        tabs->setCurrentWidget(window.findChild<QWidget *>("tab_CONFIG"));
        window.findChild<QComboBox *>("comboBox_moveDirection")->setCurrentIndex(axis);
        window.findChild<QSpinBox *>("spinBox_distance")->setValue(sign*100);
        check(window.findChild<QPushButton *>("pushButton_move")->isEnabled(),"manual MOVE button enabled for comparison");
        window.findChild<QPushButton *>("pushButton_move")->click();
        check(window.motionRunning,"manual comparison command was sent");
        const auto manual=takeMove(receiver,"Configuration MOVE"); arrive(window);
        tabs->setCurrentWidget(window.findChild<QWidget *>("tab_MAIN"));
        window.findChild<QPushButton *>("pushButton_resetMap")->click();
        map->points={axis==0 ? QPointF(0,sign*100) : QPointF(-sign*100,0)}; map->changed();
        window.startRoute(); check(window.routeActive,"setpoint comparison command was sent"); const auto route=takeMove(receiver,"axial setpoint MOVE");
        check(route==manual && window.mapMode==axis,"normal axial setpoint sends the exact Configuration MOVE bytes and motor limits");
        arrive(window);
        check(window.routeIndex==1 && map->completed==1 && window.routePauseTimer->isActive(),"axial telemetry confirms and consumes the setpoint once");
        window.routePauseTimer->stop(); window.nextRouteLeg();
        check(!window.routeActive,"axial setpoint finishes normally");
    }
    window.findChild<QPushButton *>("pushButton_resetMap")->click();
    map->points={{30,100}}; map->changed(); window.startRoute();
    const auto diagonal=takeMove(receiver,"arbitrary-angle MOVE");
    check(window.mapMode==4 && quint8(diagonal[4])==14 && window.routeLeftUm==-300000,"genuine lateral component is retained rather than snapped to an axis");
    window.cancelRoute("test"); window.motionRunning=false;
    std::puts("PASS: exact Qt command parity for longitudinal/lateral +/-100 cm at 170/170/175/185, route telemetry/completion and arbitrary-angle preservation");
}
