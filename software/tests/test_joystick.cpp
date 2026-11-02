#include <QApplication>
#include <QComboBox>
#include <QFontDatabase>
#include <QLineEdit>
#include <QSpinBox>
#include <QTextBrowser>
#include <QTabWidget>
#include <QtEndian>
#include "joystickwidget.h"
#define private public
#include "mainwindow.h"
#undef private
#include <cstdio>
#include <cstdlib>
static void check(bool ok,const char *why) {
    if(!ok) { std::fprintf(stderr,"FAIL: %s\n",why); std::exit(1); }
}
static void key(JoystickWidget *dialog,int code,bool down,bool repeat=false) {
    QKeyEvent event(down?QEvent::KeyPress:QEvent::KeyRelease,code,Qt::NoModifier,QString(),repeat);
    QApplication::sendEvent(dialog,&event);
}
static QByteArray frame(QByteArray payload) {
    QByteArray result("UNER"); result+=char(payload.size()+1); result+=':'; result+=payload;
    quint8 checksum=0; for(char byte:result) checksum^=quint8(byte); result+=char(checksum); return result;
}
int main(int argc,char **argv) {
    QApplication app(argc,argv);
    QFontDatabase::addApplicationFont("C:/Windows/Fonts/segoeui.ttf"); app.setFont(QFont("Segoe UI",10));
    MainWindow window;
    auto *tabs=window.findChild<QTabWidget *>("tabWidget_wifi");
    auto *dialog=window.joystick;
    check(dialog->findChild<QSlider *>("joystickSpeed")->value()==41,"startup percentage reflects 105/255 PWM instead of a relative 100 percent");
    check(dialog->power(0)==90 && dialog->power(1)==90 && dialog->power(2)==95 && dialog->power(3)==105,"startup percentage leaves calibrated PWM unchanged");
    check(tabs->count()==4 && tabs->tabText(tabs->indexOf(dialog))=="Joystick" && !dialog->isWindow(),"joystick is an embedded fourth tab");
    check(!window.findChild<QPushButton *>("pushButton_joystick"),"popup launcher removed");
    window.resize(1100,780); window.show(); tabs->setCurrentWidget(dialog); app.processEvents();
    window.grab().save("joystick-startup.png");
    check(!dialog->axisLocked(),"joystick starts unlocked in tank mode");
    key(dialog,Qt::Key_Up,true); check(!window.joystickDriving,"offline keys cannot drive");
    window.findChild<QComboBox *>("comboBox_protocol")->setCurrentIndex(1);
    window.findChild<QLineEdit *>("lineEdit_device_ip")->setText("127.0.0.1");
    window.findChild<QLineEdit *>("lineEdit_device_port")->setText("39001");
    check(window.udpSocket->bind(QHostAddress::LocalHost,0),"test sender");
    window.updateConnectionUi();
    const char *names[]={"fr","fl","rr","rl"};
    const int limits[]={90,100,110,120};
    for(int i=0;i<4;++i) {
        window.findChild<QSpinBox *>(QString("joystickPower_")+names[i])->setValue(limits[i]);
        check(dialog->power(i)==limits[i],"independent joystick power controls");
    }
    const int routePower=window.findChild<QSpinBox *>("spinBox_move_fr")->value();
    const auto checkWheels=[&](int fr,int fl,int rr,int rl) {
        const int expected[]={fr,fl,rr,rl};
        check(window.joystickDriving && window.findChild<QTimer *>("pwmRefreshTimer")->isActive(),"PWM renewal active");
        check(window.joystickPayload.size()==9 && quint8(window.joystickPayload[0])==0xA4,"SETPWM payload");
        for(int i=0;i<4;++i) check(qFromLittleEndian<qint16>(window.joystickPayload.constData()+1+i*2)==expected[i],"mecanum wheel signs and configured power");
        check(window.findChild<QTextBrowser *>("textBrowserUnProcessed")->toPlainText().contains(QString::fromLatin1(frame(window.joystickPayload).toHex(' ').toUpper())),"transmitted frame and checksum");
    };
    const auto ackStop=[&]() { auto reply=frame(QByteArray::fromHex("d10d")); window.processFrames(reply,"Test"); };
    auto *speed=window.findChild<QSlider *>("joystickSpeed");
    check(speed && speed->minimum()==0 && speed->maximum()==100,"general motor speed percentage");
    check(speed->value()==47,"manual motor values synchronize percentage with 120/255 PWM");
    key(dialog,Qt::Key_Up,true); speed->setValue(50); checkWheels(96,107,117,128);
    check(dialog->power(0)==96 && dialog->power(3)==128,"visible power is applied once on the 0–255 scale");
    check(window.findChild<QSpinBox *>("spinBox_move_fr")->value()==routePower,"joystick speed does not change route power");
    speed->setValue(0); check(!window.joystickDriving && window.stopPending,"zero speed stops immediately"); ackStop();
    speed->setValue(100); check(!window.joystickDriving,"increasing speed never restarts movement");
    check(dialog->power(0)==191 && dialog->power(1)==213 && dialog->power(2)==234 && dialog->power(3)==255,"100 percent reaches 255 while retaining motor proportions after zero");
    window.findChild<QSpinBox *>("joystickPower_fr")->setValue(200);
    speed->setValue(50);
    check(dialog->power(0)==100,"manual tuning becomes the reference for proportional scaling");
    for(int i=0;i<4;++i) window.findChild<QSpinBox *>(QString("joystickPower_")+names[i])->setValue(limits[i]);
    key(dialog,Qt::Key_Left,true); checkWheels(90,-100,110,-120);
    key(dialog,Qt::Key_F,true); check(dialog->axisLocked(),"F enables axis lock"); checkWheels(90,-100,-110,120);
    key(dialog,Qt::Key_F,true,true); check(dialog->axisLocked(),"F autorepeat does not toggle again");
    key(dialog,Qt::Key_F,false);
    key(dialog,Qt::Key_F,true); check(!dialog->axisLocked(),"second F disables axis lock"); checkWheels(90,-100,110,-120);
    key(dialog,Qt::Key_F,false);
    key(dialog,Qt::Key_Left,false); ackStop();
    key(dialog,Qt::Key_Right,true); checkWheels(-90,100,-110,120);
    key(dialog,Qt::Key_Up,true); checkWheels(0,100,0,120);
    key(dialog,Qt::Key_Up,false); checkWheels(-90,100,-110,120);
    key(dialog,Qt::Key_Right,false); ackStop();
    dialog->findChild<QPushButton *>("joystickAxisLock")->click();
    check(dialog->axisLocked(),"mouse button can enable axis lock");
    key(dialog,Qt::Key_Up,true); checkWheels(90,100,110,120);
    auto *pwmTimer=window.findChild<QTimer *>("pwmRefreshTimer");
    window.findChild<QTextBrowser *>("textBrowserUnProcessed")->clear();
    QMetaObject::invokeMethod(pwmTimer,"timeout",Qt::DirectConnection);
    checkWheels(90,100,110,120);
    check(!window.findChild<QPushButton *>("pushButton_importRoute")->isEnabled(),"joystick locks route import");
    check(!window.findChild<QPushButton *>("pushButton_sendCommand")->isEnabled(),"joystick excludes other PWM source");
    key(dialog,Qt::Key_Right,true); checkWheels(0,100,110,0);
    key(dialog,Qt::Key_Right,false,true); check(window.joystickDriving,"ignore autorepeat release");
    key(dialog,Qt::Key_Right,false); checkWheels(90,100,110,120);
    key(dialog,Qt::Key_Left,true); checkWheels(90,0,0,120);
    key(dialog,Qt::Key_Up,false); checkWheels(90,-100,-110,120);
    key(dialog,Qt::Key_Down,true); checkWheels(0,-100,-110,0);
    key(dialog,Qt::Key_Left,false); checkWheels(-90,-100,-110,-120);
    key(dialog,Qt::Key_Right,true); checkWheels(-90,0,0,-120);
    key(dialog,Qt::Key_Down,false); checkWheels(-90,100,110,-120);
    key(dialog,Qt::Key_Right,false);
    check(!window.joystickDriving && window.stopPending,"release sends STOP");
    check(!window.findChild<QTimer *>("pwmRefreshTimer")->isActive(),"no PWM after release");
    key(dialog,Qt::Key_Up,true); check(!window.joystickDriving,"wait for STOP acknowledgement");
    ackStop(); key(dialog,Qt::Key_Up,true); checkWheels(90,100,110,120);
    QEvent deactivate(QEvent::WindowDeactivate); QApplication::sendEvent(&window,&deactivate);
    check(!window.joystickDriving && window.stopPending,"focus loss stops"); ackStop();
    key(dialog,Qt::Key_Up,true); key(dialog,Qt::Key_Space,true);
    check(!window.joystickDriving && window.stopPending,"space stops"); ackStop();
    key(dialog,Qt::Key_Up,true); tabs->setCurrentIndex(0);
    check(!window.joystickDriving && window.stopPending,"leaving joystick tab stops"); ackStop();
    key(dialog,Qt::Key_Up,true); check(!window.joystickDriving,"hidden joystick ignores keys");
    tabs->setCurrentWidget(dialog); app.processEvents();
    check(!dialog->axisLocked(),"entering joystick tab starts unlocked");
    key(dialog,Qt::Key_Up,true);
    auto rejected=frame(QByteArray::fromHex("a40e")); window.processFrames(rejected,"Test");
    check(!window.joystickDriving && window.stopPending && !pwmTimer->isActive(),"PWM rejection stops and clears joystick"); ackStop();
    window.routeActive=true; window.updateCommandUi(); key(dialog,Qt::Key_Up,true);
    check(!window.joystickDriving,"active route blocks joystick");
    window.routeActive=false; window.routePaused=true; window.updateCommandUi(); key(dialog,Qt::Key_Up,true);
    check(!window.joystickDriving,"paused route blocks joystick");
    window.routePaused=false; window.updateCommandUi();
    auto *powerInput=window.findChild<QSpinBox *>("joystickPower_fr");
    powerInput->setFocus(); app.processEvents();
    QKeyEvent edit(QEvent::KeyPress,Qt::Key_Up,Qt::NoModifier);
    QApplication::sendEvent(powerInput,&edit);
    check(!window.joystickDriving && powerInput->value()==91,"arrows in a power editor adjust power without driving");
    QKeyEvent enter(QEvent::KeyPress,Qt::Key_Return,Qt::NoModifier);
    QApplication::sendEvent(powerInput,&enter);
    check(dialog->hasFocus(),"Enter returns keyboard control to joystick");
    window.grab().save("joystick.png");
    std::puts("PASS: joystick tab, motor PWM and live speed, zero-speed STOP, tab-exit STOP, tank/locked directions and route exclusion");
}
