#include <QApplication>
#include <QComboBox>
#include <QFontDatabase>
#include <QLineEdit>
#include <QPushButton>
#include <QTextBrowser>
#include <QtEndian>
#include "routeplanwidget.h"
#define private public
#include "mainwindow.h"
#undef private
#include <cstdio>
#include <cstdlib>
static void check(bool ok,const char *why) { if(!ok) { std::fprintf(stderr,"FAIL: %s\n",why); std::exit(1); } }
static QByteArray frame(QByteArray payload) {
    QByteArray result("UNER"); result+=char(payload.size()+1); result+=':'; result+=payload;
    quint8 checksum=0; for(char byte:result) checksum^=quint8(byte); result+=char(checksum); return result;
}
int main(int argc,char **argv) {
    QApplication app(argc,argv);
    QFontDatabase::addApplicationFont("C:/Windows/Fonts/segoeui.ttf"); app.setFont(QFont("Segoe UI",10));
    MainWindow window; window.resize(1100,780); window.show(); app.processEvents();
    auto *plan=static_cast<RoutePlanWidget *>(window.findChild<QWidget *>("routePlan")); auto *slider=plan->speedSlider;
    auto *panel=plan->findChild<QFrame *>("mapSpeedPanel");
    check(panel && panel->x()>148 && panel->geometry().right()<plan->width(),"speed overlay anchored at upper right without covering the axes");
    check(slider->minimum()==0 && slider->maximum()==100 && slider->value()==73,"percentage range and calibrated startup value");
    QMouseEvent click(QEvent::MouseButtonPress,{10,10},{10,10},Qt::LeftButton,Qt::LeftButton,Qt::NoModifier);
    QApplication::sendEvent(panel,&click); check(plan->points.isEmpty(),"clicking speed panel does not add a setpoint");
    const char *names[]={"fr","fl","rr","rl"};
    QSpinBox *spins[4]; for(int i=0;i<4;++i) spins[i]=window.findChild<QSpinBox *>(QString("spinBox_move_")+names[i]);
    const auto powers=[&](int fr,int fl,int rr,int rl) {
        const int expected[]={fr,fl,rr,rl}; for(int i=0;i<4;++i) check(spins[i]->value()==expected[i],"percentage preserves motor compensation and PWM bounds");
    };
    powers(170,170,175,185);
    slider->setValue(100); powers(234,234,241,255);
    slider->setValue(50); powers(118,118,121,128);
    slider->setValue(0); powers(0,0,0,0);
    slider->setValue(73); powers(171,171,176,186);
    const int calibration[]={80,100,120,160}; for(int i=0;i<4;++i) spins[i]->setValue(calibration[i]);
    slider->setValue(100); powers(128,159,191,255);
    slider->setValue(50); powers(64,80,96,128);
    slider->setValue(0); slider->setValue(100); powers(128,159,191,255);
    slider->setValue(50);
    window.findChild<QComboBox *>("comboBox_protocol")->setCurrentIndex(1);
    window.findChild<QLineEdit *>("lineEdit_device_ip")->setText("127.0.0.1"); window.findChild<QLineEdit *>("lineEdit_device_port")->setText("39001");
    check(window.udpSocket->bind(QHostAddress::LocalHost,0),"local test transport"); window.updateConnectionUi();
    plan->points={{0,100}}; plan->changed(); window.startRoute();
    check(window.routeActive && window.moveAwaitingAck,"route starts with current slider power");
    slider->setValue(75); powers(96,119,143,191);
    QMetaObject::invokeMethod(window.mapSpeedSendTimer,"timeout",Qt::DirectConnection);
    check(window.mapLimitsDirty && !window.mapLimitsPending,"live limits wait for MOVE acknowledgement");
    const auto reply=[&](const char *bytes) { auto packet=frame(QByteArray::fromHex(bytes)); window.processFrames(packet,"Test"); };
    reply("d00d"); QMetaObject::invokeMethod(window.mapSpeedSendTimer,"timeout",Qt::DirectConnection);
    check(window.mapLimitsPending && window.routeActive && window.routeIndex==0,"speed update does not restart route");
    auto *log=window.findChild<QTextBrowser *>("textBrowserUnProcessed");
    check(log->toPlainText().contains(QString::fromLatin1(frame(QByteArray::fromHex("d560778fbf")).toHex(' ').toUpper())),"live SETLIMITS frame contains compensated power values");
    slider->setValue(25); powers(32,40,48,64);
    check(window.mapLimitsDirty,"dragging while waiting keeps latest power request");
    reply("d50d"); QMetaObject::invokeMethod(window.mapSpeedSendTimer,"timeout",Qt::DirectConnection);
    check(log->toPlainText().contains(QString::fromLatin1(frame(QByteArray::fromHex("d520283040")).toHex(' ').toUpper())),"latest percentage applied after previous ACK"); reply("d50d");
    slider->setValue(0);
    check(!window.routeActive && window.routePaused && window.stopPending && !window.mapLimitsPending,"zero percent pauses route with STOP");
    reply("d10d");
    QByteArray status(124,'\0'); status[0]=char(0xD2); status[1]=char(3); status[27]=char(1); status[93]=char(window.mapMode);
    qToLittleEndian<qint32>(window.routeForwardUm,status.data()+94); qToLittleEndian<qint32>(window.routeLeftUm,status.data()+98);
    qToLittleEndian<qint32>(400000,status.data()+102); auto packet=frame(status); window.processFrames(packet,"Test");
    check(window.resumeReady && !window.findChild<QPushButton *>("pushButton_resumeRoute")->isEnabled(),"cannot resume at zero power");
    slider->setValue(100);
    check(!window.routeActive && !window.motionRunning && window.findChild<QPushButton *>("pushButton_resumeRoute")->isEnabled(),"raising percentage never auto-starts movement");
    window.resumeRoute(); reply("d00d"); slider->setValue(60); QMetaObject::invokeMethod(window.mapSpeedSendTimer,"timeout",Qt::DirectConnection);
    check(window.mapLimitsPending,"new live request"); reply("d50e");
    check(window.routePaused && window.stopPending && !window.routeActive,"rejected live limits stop the robot");
    reply("d10d"); window.startRoute(); reply("d00d"); slider->setValue(70); QMetaObject::invokeMethod(window.mapSpeedSendTimer,"timeout",Qt::DirectConnection);
    QMetaObject::invokeMethod(window.mapSpeedAckTimer,"timeout",Qt::DirectConnection);
    check(window.routePaused && window.stopPending && !window.routeActive,"missing speed ACK stops rather than silently using unknown power");
    reply("d10d"); window.routePaused=false; window.updateCommandUi();
    slider->setValue(41); plan->points={{30,40},{90,60}}; plan->changed(); app.processEvents();
    window.grab().save("map-speed.png");
    std::puts("PASS: map speed placement, 0–255 mapping, motor calibration, live ACK/coalescing, zero pause, safe resume and rejection/timeout STOP");
}
