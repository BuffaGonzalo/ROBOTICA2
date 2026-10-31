#include <QApplication>
#include <QByteArray>
#include <QHostAddress>
#include <QSerialPort>
#include <QUdpSocket>
#include <QTimer>
#include <QMainWindow>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLineEdit>
#include <QComboBox>
#include <QLabel>
#include <QTextBrowser>
#include <QTabWidget>
#include <QFontDatabase>
#include <QPixmap>
#include <QtEndian>
#include <QTableWidget>
#include <QSpinBox>
#include <QScrollArea>
#include <QGroupBox>
#include <stdio.h>
#include <stdlib.h>
// Exercise the real binary decoder without a robot or a network transport.
#define private public
#include "mainwindow.h"
#undef private
static void check(bool ok, const char *why) { if(!ok) { fprintf(stderr,"FAIL: %s\n",why); exit(1); } }
static QByteArray frame(QByteArray payload) {
    QByteArray data("UNER"); data+=char(payload.size()+1); data+=':'; data+=payload;
    quint8 chk=0; for(char b:data) chk^=quint8(b); data+=char(chk); return data;
}
int main(int argc,char **argv) {
    QApplication app(argc,argv);
    QFontDatabase::addApplicationFont("C:/Windows/Fonts/segoeui.ttf"); app.setFont(QFont("Segoe UI",10));
    MainWindow w;
    auto *distance=w.findChild<QSpinBox *>("spinBox_distance");
    check(distance && distance->minimum()==-1000 && distance->maximum()==1000 && distance->suffix()==" cm","signed distance range");
    auto *protocol=w.findChild<QComboBox *>("comboBox_protocol"); protocol->setCurrentIndex(1);
    w.findChild<QLineEdit *>("lineEdit_device_ip")->setText("127.0.0.1");
    w.findChild<QLineEdit *>("lineEdit_device_port")->setText("39001");
    check(w.udpSocket->bind(QHostAddress::LocalHost,0),"local test socket"); w.updateConnectionUi();
    auto *kp=w.findChild<QDoubleSpinBox *>("doubleSpinBox_kpPosition");
    check(kp && w.findChild<QDoubleSpinBox *>("doubleSpinBox_kpSpeed") && w.findChild<QDoubleSpinBox *>("doubleSpinBox_kiSpeed"),"PID spinboxes exist");
    kp->setValue(3.5);
    w.findChild<QPushButton *>("pushButton_applyPid")->click();
    check(w.pidPending && !w.findChild<QPushButton *>("pushButton_move")->isEnabled(),"gain request waits for ACK before MOVE");
    check(w.findChild<QTextBrowser *>("textBrowserUnProcessed")->toPlainText().contains("12 3A D4 00 00 60 40"),"PID float32 little endian wire");
    auto gainsReply=frame(QByteArray::fromHex("d40d")); w.processFrames(gainsReply,"Test");
    check(!w.pidPending && w.findChild<QLabel *>("label_pidStatus")->text().contains("confirmado"),"PID confirmed only by ACK");
    distance->setValue(-10);
    const char *names[]={"fr","fl","rr","rl"};
    const int limits[]={57,80,120,255};
    const int defaults[]={90,90,95,105};
    for(int i=0;i<4;++i) {
        auto *power=w.findChild<QSpinBox *>(QString("spinBox_move_")+names[i]);
        check(power && power->minimum()==0 && power->maximum()==255,"four movement power fields");
        check(power->value()==defaults[i],"requested default motor powers");
        power->setValue(limits[i]);
    }
    check(w.findChildren<QSpinBox *>().size()==9,"exactly eight integer motor controls");
    check(!w.findChild<QTableWidget *>("tableWidget_motion"),"removed diagnostic block");
    check(w.findChild<QPushButton *>("pushButton_exportTxt"), "export txt button exists");
    check(w.findChild<QPushButton *>("pushButton_exportXls"), "export xls button exists");
    check(!w.findChild<QPushButton *>("pushButton_emergency"), "emergency stop button removed from communication tab");
    w.findChild<QPushButton *>("pushButton_move")->click();
    auto raw=w.findChild<QTextBrowser *>("textBrowserUnProcessed")->toPlainText();
    check(raw.contains("0A 3A D0 60 79 FE FF 39 50 78 FF"),"signed distance and four independent power bytes");
    check(w.motionRunning && !w.findChild<QTimer *>("pwmRefreshTimer")->isActive(),"manual PWM cancelled by MOVE");
    QByteArray status(26,'\0'); status[0]=char(0xD2); status[1]=char(2);
    qToLittleEndian<qint32>(-1202,status.data()+2); qToLittleEndian<qint32>(-123456,status.data()+6);
    auto data=frame(status); w.processFrames(data,"Test");
    check(!w.motionRunning && w.findChild<QTextBrowser *>("textBrowserProcessed")->toPlainText().contains("Finalizado"),"telemetry stays in communication log");
    auto *tabs=w.findChild<QTabWidget *>("tabWidget_wifi");
    check(tabs->count()==3 && tabs->tabText(2)==QString::fromUtf8("Visualización"),"third visualization tab");
    QByteArray visual(69,'\0'); visual[0]=char(0xD2); visual[1]=char(1);
    qToLittleEndian<qint32>(973,visual.data()+2); qToLittleEndian<qint32>(50000,visual.data()+6);
    qToLittleEndian<qint32>(9732000,visual.data()+61); qToLittleEndian<qint32>(80590,visual.data()+65);
    for(int i=0;i<4;++i) { qToLittleEndian<qint32>(487,visual.data()+10+i*4); qToLittleEndian<qint16>(70+i,visual.data()+53+i*2); }
    data=frame(visual); w.processFrames(data,"Test");
    check(w.findChild<QLabel *>("label_visualDistance")->text().contains("5.00 cm"),"encoder mean displayed in cm");
    check(w.findChild<QLabel *>("label_topDistance")->text().contains("5.00 cm"),"top bar shows general encoder distance");
    check(w.findChild<QTableWidget *>("tableWidget_visual")->item(0,4)->text()=="70","applied PWM shown");
    visual[1]=char(4); data=frame(visual); w.processFrames(data,"Test");
    check(w.findChild<QLabel *>("label_visualState")->text().contains("Fallo"),"failure is not arrival");
    visual[1]=char(9); data=frame(visual); w.processFrames(data,"Test");
    check(w.findChild<QLabel *>("label_visualState")->text().contains("exceso de recorrido"),"full-power overshoot is distinguished from arrival");
    visual[1]=char(2); qToLittleEndian<qint32>(99977,visual.data()+6);
    for(int i=0;i<4;++i) qToLittleEndian<qint32>(973,visual.data()+10+i*4);
    data=frame(visual); w.processFrames(data,"Test");
    check(w.findChild<QLabel *>("label_visualState")->text()=="Finalizado","arrival status visualized");
    const int cmValues[]={1,10,100};
    const char *wireValues[]={"10 27 00 00","A0 86 01 00","40 42 0F 00"};
    for(int i=0;i<3;++i) {
        distance->setValue(cmValues[i]);
        w.motionRunning=false; w.updateCommandUi();
        w.findChild<QPushButton *>("pushButton_move")->click();
        check(w.findChild<QTextBrowser *>("textBrowserUnProcessed")->toPlainText().contains(QString("0A 3A D0 ")+wireValues[i]),"different integer cm produce different wire distances");
    }
    w.findChild<QPushButton *>("pushButton_stop")->click();
    check(w.findChild<QTextBrowser *>("textBrowserUnProcessed")->toPlainText().contains("02 3A D1"),"STOP wire command");
    check(w.stopPending && w.findChild<QTimer *>("stopRetryTimer")->isActive(),"STOP repeats until ACK");
    data=frame(QByteArray::fromHex("d10d")); w.processFrames(data,"Test");
    check(!w.stopPending && !w.findChild<QTimer *>("stopRetryTimer")->isActive(),"STOP ACK cancels retries");
    w.findChild<QSpinBox *>("spinBox_fr")->setValue(-100);
    w.findChild<QPushButton *>("pushButton_sendCommand")->click();
    check(w.findChild<QTextBrowser *>("textBrowserUnProcessed")->toPlainText().contains("0A 3A A4 9C FF"),"manual PWM test sends signed values");
    w.show(); QApplication::processEvents(); w.grab().save("communication.png");
    w.findChild<QTabWidget *>("tabWidget_wifi")->setCurrentIndex(1); QApplication::processEvents(); w.grab().save("configuration.png");
    w.findChild<QScrollArea *>("scrollArea_config")->ensureWidgetVisible(w.findChild<QGroupBox *>("groupBox_pid"));
    QApplication::processEvents(); w.grab().save("pid-configuration.png");
    tabs->setCurrentIndex(2); QApplication::processEvents(); w.grab().save("visualization.png");
    puts("PASS: Qt MOVE encoding, PWM exclusion, signed telemetry, arrival and STOP");
}
