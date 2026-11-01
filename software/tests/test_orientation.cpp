#include <QApplication>
#include <QComboBox>
#include <QFontDatabase>
#include <QLineEdit>
#include <QPushButton>
#include <QTemporaryDir>
#include <QDir>
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
    MainWindow window;
    auto *plan=static_cast<RoutePlanWidget *>(window.findChild<QWidget *>("routePlan"));
    auto *toggle=window.findChild<QPushButton *>("pushButton_routeOrientation");
    check(toggle && toggle->isCheckable() && !toggle->isChecked(),"orientation is optional");
    plan->points={{0,100}}; plan->setHeading(0,90); toggle->click();
    check(plan->oriented && plan->headingAt(0)==90,"enable and edit arrival angle");
    QTemporaryDir directory(QDir::currentPath()+"/orientation-XXXXXX"); check(directory.isValid(),"temporary directory");
    QString error; const QString path=directory.filePath("route.json");
    check(window.exportRouteFile(path,error),"export pose route");
    plan->setHeading(0,0); toggle->click();
    check(window.importRouteFile(path,error) && plan->oriented && plan->headingAt(0)==90,"JSON preserves angles and toggle");
    // Drag the arrival arrow independently of the destination coordinates.
    plan->points={{10,10}}; plan->resize(800,600);
    const QPointF handle=plan->screen(plan->points[0])-plan->basisX()*34;
    QMouseEvent press(QEvent::MouseButtonPress,handle,handle,Qt::LeftButton,Qt::LeftButton,Qt::NoModifier);
    QApplication::sendEvent(plan,&press);
    const QPointF drag=plan->screen(plan->points[0])+plan->basisX()*34;
    QMouseEvent move(QEvent::MouseMove,drag,drag,Qt::NoButton,Qt::LeftButton,Qt::NoModifier);
    QApplication::sendEvent(plan,&move);
    QMouseEvent release(QEvent::MouseButtonRelease,drag,drag,Qt::LeftButton,Qt::NoButton,Qt::NoModifier);
    QApplication::sendEvent(plan,&release);
    check(plan->points[0]==QPointF(10,10) && std::abs(plan->headingAt(0)+90)<0.001,"drag arrival heading without moving XY");
    // The same editor exposes X/Y/heading even with orientation disabled.
    toggle->click();
    bool editorOpened=false;
    QTimer::singleShot(0,[&]() {
        auto *editor=qobject_cast<QDialog *>(QApplication::activeModalWidget());
        if(!editor) return;
        editorOpened=true;
        editor->findChild<QSpinBox *>("setpointX")->setValue(-25);
        editor->findChild<QSpinBox *>("setpointY")->setValue(35);
        editor->findChild<QDoubleSpinBox *>("setpointHeading")->setValue(120);
        editor->grab().save("setpoint-editor.png"); editor->accept();
    });
    QPointF editAt=plan->screen(plan->points[0]);
    QMouseEvent editEvent(QEvent::MouseButtonDblClick,editAt,editAt,Qt::LeftButton,Qt::LeftButton,Qt::NoModifier);
    QApplication::sendEvent(plan,&editEvent);
    check(editorOpened && plan->points[0]==QPointF(-25,35) && plan->headingAt(0)==120 && !plan->oriented,"double click edits XY and heading without enabling orientation");
    QTimer::singleShot(0,[&]() {
        auto *editor=qobject_cast<QDialog *>(QApplication::activeModalWidget());
        if(!editor) return;
        editor->findChild<QSpinBox *>("setpointX")->setValue(999); editor->reject();
    });
    editAt=plan->screen(plan->points[0]);
    QMouseEvent cancelEdit(QEvent::MouseButtonDblClick,editAt,editAt,Qt::LeftButton,Qt::LeftButton,Qt::NoModifier);
    QApplication::sendEvent(plan,&cancelEdit);
    check(plan->points[0]==QPointF(-25,35) && plan->headingAt(0)==120,"cancel preserves all setpoint fields");
    toggle->click();
    plan->points={{0,100}}; plan->setHeading(0,90);
    window.findChild<QComboBox *>("comboBox_protocol")->setCurrentIndex(1);
    window.findChild<QLineEdit *>("lineEdit_device_ip")->setText("127.0.0.1");
    window.findChild<QLineEdit *>("lineEdit_device_port")->setText("39001");
    check(window.udpSocket->bind(QHostAddress::LocalHost,0),"local test socket"); window.updateConnectionUi();
    window.startRoute();
    check(window.routeActive && window.mapMode==6 && !toggle->isEnabled(),"straight pose route locks editing");
    check(window.routeTurnUrad==1570796 && window.routeForwardUm==1000000 && window.routeLeftUm==0,"straight endpoint and 90-degree rotation without arc displacement");
    auto accepted=frame(QByteArray::fromHex("d00d")); window.processFrames(accepted,"Test");
    QByteArray status(120,'\0'); status[0]=char(0xD2); status[1]=char(1); status[93]=char(6);
    qToLittleEndian<qint32>(window.routeForwardUm,status.data()+94);
    qToLittleEndian<qint32>(window.routeLeftUm,status.data()+98);
    qToLittleEndian<qint32>(1000000,status.data()+102);
    qToLittleEndian<qint32>(window.routeTurnUrad,status.data()+112);
    qToLittleEndian<qint32>(window.routeTurnUrad,status.data()+116);
    auto telemetry=frame(status); window.processFrames(telemetry,"Test");
    check(QLineF(plan->robot,{0,100}).length()<0.01 && std::abs(plan->robotHeadingDeg-90)<0.001,"pose telemetry updates XY and physical heading");
    status[1]=char(2); telemetry=frame(status); window.processFrames(telemetry,"Test");
    check(window.routePauseTimer->isActive() && std::abs(window.plannedHeadingDeg-90)<0.001,"arrival preserves angle for next leg");
    window.routePauseTimer->stop(); window.nextRouteLeg();
    check(!window.routeActive && plan->completed==1,"pose route completes");
    // Pure rotation at the same XY must not be skipped.
    plan->points={{0,100}}; plan->setHeading(0,-90); window.startRoute();
    check(window.routeActive && std::abs(std::abs(window.routeTurnUrad)-3141593)<=2,"same-position setpoint can rotate");
    window.findChild<QPushButton *>("pushButton_stop")->click();
    check(window.routePaused && !toggle->isEnabled(),"pause preserves orientation route");
    window.routePaused=false; window.stopPending=false; window.motionRunning=false; window.updateCommandUi();
    // A margin-confirmed arrival must advance once and use the stopped pose
    // to plan the next segment rather than accumulating nominal residuals.
    plan->points={{0,100},{0,200}}; plan->headings={90,90}; plan->robot={0,0}; plan->robotHeadingDeg=0;
    window.startRoute();
    accepted=frame(QByteArray::fromHex("d00d")); window.processFrames(accepted,"Test");
    status.fill('\0'); status[0]=char(0xD2); status[1]=char(2); status[93]=char(6);
    qToLittleEndian<qint32>(window.routeForwardUm,status.data()+94); qToLittleEndian<qint32>(window.routeLeftUm,status.data()+98);
    qToLittleEndian<qint32>(992000,status.data()+102); qToLittleEndian<quint16>(window.expectedGeneration,status.data()+110);
    qToLittleEndian<qint32>(window.routeTurnUrad,status.data()+112); qToLittleEndian<qint32>(1535890,status.data()+116);
    telemetry=frame(status); window.processFrames(telemetry,"Test");
    check(window.routeIndex==1 && plan->completed==1 && std::abs(window.plannedPosition.y()-99.2)<0.001,"arrival margin consumes target and records measured pose");
    window.routePauseTimer->stop(); window.nextRouteLeg();
    check(window.routeActive && window.routeIndex==1 && std::abs(window.routeExpectedUm-1008000)<=1,"next target starts from measured position");
    window.cancelRoute("Fin de prueba"); window.motionRunning=false; window.mapTracking=false; window.updateCommandUi();
    plan->points={{30,40},{100,90}}; plan->headings={45,-90}; plan->selectedIndex=1;
    window.resize(1200,780); window.show(); app.processEvents();
    window.grab().save("orientation.png");
    std::puts("PASS: straight orientation, XY/angle editor and cancel, JSON, pose telemetry, arrival-margin advance, pure rotation and pause");
}
