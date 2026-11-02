#include <QApplication>
#include <QComboBox>
#include <QFontDatabase>
#include <QLineEdit>
#include <QPushButton>
#include <QTemporaryDir>
#include <QFile>
#include <QDir>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
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
static void click(RoutePlanWidget *plan,QPointF world,Qt::MouseButton button=Qt::LeftButton) {
    const QPointF at=plan->screen(world);
    QMouseEvent down(QEvent::MouseButtonPress,at,at,button,button,Qt::NoModifier);
    QApplication::sendEvent(plan,&down);
    QMouseEvent up(QEvent::MouseButtonRelease,at,at,button,Qt::NoButton,Qt::NoModifier);
    QApplication::sendEvent(plan,&up);
}
int main(int argc,char **argv) {
    QApplication app(argc,argv);
    QFontDatabase::addApplicationFont("C:/Windows/Fonts/segoeui.ttf"); app.setFont(QFont("Segoe UI",10));
    MainWindow window; window.resize(1100,780); window.show(); app.processEvents();
    auto *plan=static_cast<RoutePlanWidget *>(window.findChild<QWidget *>("routePlan"));
    auto *arc=window.findChild<QPushButton *>("pushButton_routeArc");
    check(arc && arc->isCheckable() && !arc->isChecked(),"optional Arco button");
    arc->click(); check(plan->arcMode && plan->points.isEmpty(),"Arco enables creation without adding a spurious point");
    click(plan,{80,0}); check(plan->points.size()==1 && plan->arcAt(0)==180,"new point is a semicircle");
    const auto middle=RoutePlanWidget::arcPoint({0,0},{80,0},180,0.5);
    check(QLineF(middle,{40,-40}).length()<0.001,"semicircle geometry");
    bool opened=false;
    QTimer::singleShot(0,[&]() {
        auto *editor=qobject_cast<QDialog *>(QApplication::activeModalWidget()); if(!editor) return;
        opened=true; editor->findChild<QSpinBox *>("setpointX")->setValue(100);
        editor->findChild<QSpinBox *>("setpointY")->setValue(20);
        editor->findChild<QDoubleSpinBox *>("setpointHeading")->setValue(30);
        editor->findChild<QDoubleSpinBox *>("setpointArcSweep")->setValue(90);
        editor->findChild<QComboBox *>("setpointArcDirection")->setCurrentIndex(1);
        auto *sweep=editor->findChild<QDoubleSpinBox *>("setpointArcSweep");
        check(sweep->value()==-90,"clockwise selector updates signed angle");
        sweep->setValue(-135);
        check(editor->findChild<QComboBox *>("setpointArcDirection")->currentIndex()==1,"typed negative angle keeps clockwise direction");
        sweep->setValue(0);
        check(!editor->findChild<QDialogButtonBox *>()->button(QDialogButtonBox::Ok)->isEnabled(),"zero arc angle cannot be saved");
        sweep->setValue(-90);
        editor->grab().save("arc-editor.png"); editor->accept();
    });
    click(plan,{80,0});
    check(opened && plan->points[0]==QPointF(100,20) && plan->headingAt(0)==30 && plan->arcAt(0)==-90,"single click edits arc endpoint, heading, sweep and direction");
    click(plan,{150,70}); check(plan->arcAt(1)==180,"Arco mode keeps creating arcs");
    arc->click(); click(plan,{0,80}); check(plan->arcAt(0)==-90 && plan->arcAt(1)==180 && plan->arcAt(2)==0,"switching to rectas preserves existing arcs");
    click(plan,{150,70},Qt::RightButton); check(plan->points.size()==2 && plan->arcSweeps==QList<double>({-90,0}),"deletion preserves segment metadata alignment");
    plan->points={{80,0},{80,100},{0,100}}; plan->headings={0,90,0}; plan->arcSweeps={180,-120,0}; arc->click(); plan->changed();
    QTemporaryDir directory(QDir::currentPath()+"/arcs-XXXXXX"); check(directory.isValid(),"temporary directory");
    const auto path=directory.filePath("arcs.json"); QString error;
    check(window.exportRouteFile(path,error),"export arcs"); plan->arcSweeps.clear(); arc->click();
    check(window.importRouteFile(path,error) && plan->arcSweeps==QList<double>({180,-120,0}) && plan->arcMode,"JSON preserves arc geometry and creation mode");
    QFile file(path); check(file.open(QIODevice::ReadOnly),"read arc fixture");
    const QByteArray saved=file.readAll(); file.close();
    const auto original=QJsonDocument::fromJson(saved).object();
    for(QJsonValue invalidSweep:{QJsonValue(0.5),QJsonValue(360),QJsonValue("bad"),QJsonValue(QJsonValue::Null)}) {
        auto invalid=original; auto entries=invalid.value("points").toArray(); auto point=entries[0].toObject();
        point.insert("arcSweepDeg",invalidSweep); entries[0]=point; invalid.insert("points",entries);
        check(file.open(QIODevice::WriteOnly|QIODevice::Truncate),"write invalid arc fixture"); file.write(QJsonDocument(invalid).toJson()); file.close();
        check(!window.importRouteFile(path,error) && plan->arcSweeps==QList<double>({180,-120,0}),"invalid arc import preserves route");
    }
    auto legacy=original; legacy.insert("version",2); legacy.remove("arcMode");
    check(file.open(QIODevice::WriteOnly|QIODevice::Truncate),"write version 2 fixture"); file.write(QJsonDocument(legacy).toJson()); file.close();
    check(window.importRouteFile(path,error) && plan->arcSweeps==QList<double>({0,0,0}) && !plan->arcMode,"version 2 remains a straight route");
    check(file.open(QIODevice::WriteOnly|QIODevice::Truncate),"restore arc fixture"); file.write(saved); file.close();
    check(window.importRouteFile(path,error),"restore arc route");
    window.findChild<QComboBox *>("comboBox_protocol")->setCurrentIndex(1);
    window.findChild<QLineEdit *>("lineEdit_device_ip")->setText("127.0.0.1"); window.findChild<QLineEdit *>("lineEdit_device_port")->setText("39001");
    check(window.udpSocket->bind(QHostAddress::LocalHost,0),"local test socket"); window.updateConnectionUi(); window.startRoute();
    check(window.routeActive && window.mapMode==7 && window.routeArcSweepUrad==3141593 && !arc->isEnabled(),"arc execution uses dedicated protocol and locks editing");
    auto ack=frame(QByteArray::fromHex("d00d")); window.processFrames(ack,"Test");
    QByteArray status(124,'\0'); status[0]=char(0xD2); status[1]=char(1); status[93]=char(7);
    qToLittleEndian<qint32>(window.routeForwardUm,status.data()+94); qToLittleEndian<qint32>(window.routeLeftUm,status.data()+98);
    qToLittleEndian<qint32>(-400000,status.data()+102); qToLittleEndian<qint32>(-400000,status.data()+106);
    qToLittleEndian<qint32>(window.routeArcSweepUrad,status.data()+120);
    auto telemetry=frame(status); window.processFrames(telemetry,"Test");
    check(QLineF(plan->robot,{40,-40}).length()<0.01 && std::abs(window.mapArcProgress-0.5)<0.001,"arc telemetry records midpoint and progress");
    window.findChild<QPushButton *>("pushButton_stop")->click();
    ack=frame(QByteArray::fromHex("d10d")); window.processFrames(ack,"Test");
    status[1]=char(3); status[27]=char(1); telemetry=frame(status); window.processFrames(telemetry,"Test");
    check(window.routePaused && window.resumeReady,"arc pause waits for stopped pose"); window.resumeRoute();
    check(window.routeActive && std::abs(window.routeArcSweepUrad-1570796)<=1,"arc resume uses only remaining sweep");
    window.cancelRoute("Fin de prueba"); window.motionRunning=false; window.mapTracking=false; window.updateCommandUi();
    plan->robot=plan->points.last(); window.startReverseRoute();
    check(window.executionArcSweeps==QList<double>({0,120,-180}),"reverse route reverses segment order and circular direction");
    window.cancelRoute("Fin de prueba"); window.motionRunning=false; window.mapTracking=false; window.updateCommandUi();
    plan->pixelsPerCm=2; plan->robot={0,0}; plan->trace.clear(); plan->completed=0; plan->update(); app.processEvents();
    window.grab().save("arcs.png");
    std::puts("PASS: arc/semicircle creation, single-click editing, mixed paths, JSON, dedicated protocol, pause/resume and reverse");
}
