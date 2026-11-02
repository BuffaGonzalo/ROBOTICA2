#include <QApplication>
#include <QComboBox>
#include <QFontDatabase>
#include <QLineEdit>
#include <QPushButton>
#include <QTextBrowser>
#include <QTemporaryDir>
#include <QFile>
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
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
    const auto at=plan->screen(world);
    QMouseEvent down(QEvent::MouseButtonPress,at,at,button,button,Qt::NoModifier); QApplication::sendEvent(plan,&down);
    QMouseEvent up(QEvent::MouseButtonRelease,at,at,button,Qt::NoButton,Qt::NoModifier); QApplication::sendEvent(plan,&up);
}
static void testPlanner() {
    ObstaclePlanner planner; planner.obstacles={{{0,70},true,20}};
    check(!planner.segmentFree({0,0},{0,140}),"circle blocks direct path after accounting for robot and safety distance");
    auto path=planner.findPath({0,0},{0,140}); check(path.size()>2,"planner detours around circle");
    const double required=10+std::hypot(30.0,19.8)/2+10+5;
    for(int i=1;i<path.size();++i) for(int sample=0;sample<=300;++sample) {
        const QPointF at=path[i-1]+(path[i]-path[i-1])*(sample/300.0);
        check(QLineF(at,{0,70}).length()>required-1e-6,"all sampled detour positions retain physical footprint and clearance");
    }
    planner.obstacles[0].circular=false; path=planner.findPath({0,0},{0,140}); check(path.size()>2,"square detour");
    for(int i=1;i<path.size();++i) check(planner.segmentFree(path[i-1],path[i]),"square corners are not cut diagonally");
    auto plan=planner.planSegment({0,0},{0,140},0,90,0,1);
    check(plan.valid && plan.steps.size()>1 && plan.steps.last().endHeading==90 && plan.steps.last().completedCount==1,"detour preserves goal and interpolates arrival orientation");
    for(int i=0;i<plan.steps.size()-1;++i) check(plan.steps[i].completedCount==0,"generated waypoints do not count as user destinations");
    planner.obstacles={{{-30,70},false,20},{{30,70},false,20}};
    check(!planner.segmentFree({0,0},{0,140}),"narrow gap cannot fit robot plus safety distance");
    check(!planner.findPath({0,0},{0,140}).isEmpty(),"planner can go around both sides of narrow gap");
    planner.obstacles.clear(); for(int y=-1000;y<=1000;y+=500) planner.obstacles.append({{0,double(y)},false,500});
    check(planner.findPath({-500,0},{500,0}).isEmpty(),"wall spanning map boundaries has no unsafe fallback");
    check(!planner.planSegment({-500,0},{500,0},0,0,0,1).valid,"unreachable destination is rejected");
    planner.obstacles={{{50,-50},true,10}};
    check(planner.segmentFree({0,0},{100,0}) && !planner.pathFree({0,0},{100,0},180),"arc collision is checked along curve rather than just its endpoints");
    plan=planner.planSegment({0,0},{100,0},0,90,180,1);
    check(plan.valid && plan.steps.last().arcSweep==0 && plan.steps.last().end==QPointF(100,0),"unsafe arc is replaced by a validated detour to the same endpoint");
}
int main(int argc,char **argv) {
    QApplication app(argc,argv); testPlanner();
    QFontDatabase::addApplicationFont("C:/Windows/Fonts/segoeui.ttf"); app.setFont(QFont("Segoe UI",10));
    MainWindow window; window.resize(1100,780); window.show(); app.processEvents();
    auto *plan=static_cast<RoutePlanWidget *>(window.findChild<QWidget *>("routePlan")); plan->pixelsPerCm=2;
    auto *button=window.findChild<QPushButton *>("pushButton_obstacles"); check(button && button->isCheckable(),"Obstáculos toolbar button");
    button->click(); click(plan,{0,60}); check(plan->obstacles.size()==1 && plan->points.isEmpty(),"obstacle mode adds obstacle instead of setpoint");
    bool opened=false;
    QTimer::singleShot(0,[&]() {
        auto *editor=qobject_cast<QDialog *>(QApplication::activeModalWidget()); if(!editor) return;
        opened=true; check(editor->findChild<QDoubleSpinBox *>("robotLength")->value()==30 && editor->findChild<QDoubleSpinBox *>("robotWidth")->value()==19.8,"real exterior robot dimensions");
        editor->findChild<QComboBox *>("obstacleShape")->setCurrentIndex(1);
        editor->findChild<QDoubleSpinBox *>("obstacleSize")->setValue(30);
        auto *safety=editor->findChild<QDoubleSpinBox *>("obstacleSafety");
        editor->findChild<QScrollArea *>()->ensureWidgetVisible(safety); app.processEvents();
        check(!safety->visibleRegion().isEmpty() && !editor->findChild<QDialogButtonBox *>()->visibleRegion().isEmpty(),"safety controls and save buttons remain accessible on small/high-DPI screens");
        editor->grab().save("obstacle-editor.png"); editor->accept();
    });
    const auto at=plan->screen({0,60}); QMouseEvent edit(QEvent::MouseButtonDblClick,at,at,Qt::LeftButton,Qt::LeftButton,Qt::NoModifier);
    QApplication::sendEvent(plan,&edit); check(opened && !plan->obstacles[0].circular && plan->obstacles[0].sizeCm==30,"double click edits obstacle shape and size");
    click(plan,{0,60},Qt::RightButton); check(plan->obstacles.isEmpty(),"right click removes obstacle");
    click(plan,{0,60}); plan->obstacles[0].circular=false; plan->obstacles[0].sizeCm=30; plan->changed();
    button->click(); click(plan,{0,120}); const auto preview=plan->routePreview();
    check(plan->points.size()==1 && preview.valid && preview.steps.size()>1,"point behind square produces avoidance route");
    window.findChild<QComboBox *>("comboBox_protocol")->setCurrentIndex(1);
    window.findChild<QLineEdit *>("lineEdit_device_ip")->setText("127.0.0.1"); window.findChild<QLineEdit *>("lineEdit_device_port")->setText("39001");
    check(window.udpSocket->bind(QHostAddress::LocalHost,0),"local test transport"); window.updateConnectionUi(); window.startRoute();
    check(window.routeActive && window.executionTargets.size()==preview.steps.size() && !button->isEnabled(),"execution uses expanded preview and blocks obstacle editing");
    for(int i=0;i<preview.steps.size();++i) check(window.executionTargets[i]==preview.steps[i].end,"drawn and executed waypoint sequence match");
    check(window.executionTargets[0]!=plan->points[0],"first command aims at safe intermediate waypoint rather than through obstacle");
    auto packet=frame(QByteArray::fromHex("d00d")); window.processFrames(packet,"Test");
    QByteArray status(124,'\0'); status[0]=char(0xD2); status[1]=char(1); status[93]=char(6);
    qToLittleEndian<qint32>(window.routeForwardUm,status.data()+94); qToLittleEndian<qint32>(window.routeLeftUm,status.data()+98);
    const auto endpoint=window.executionTargets[0];
    const QPointF toward=plan->obstacles[0].center-endpoint; const double norm=std::hypot(toward.x(),toward.y());
    const QPointF stopped=endpoint+toward*(5.0/norm);
    qToLittleEndian<qint32>(qRound(stopped.y()*10000),status.data()+102); qToLittleEndian<qint32>(qRound(-stopped.x()*10000),status.data()+106);
    packet=frame(status); window.processFrames(packet,"Test");
    check(window.routeIndex==1 && plan->completed==0,"running waypoint handoff does not mark original setpoint complete");
    check(window.routeActive && !window.routePauseTimer->isActive() && !window.routeAck,"handoff sends next MOVE immediately without waiting for ARRIVED or a pause");
    packet=frame(status); window.processFrames(packet,"Test");
    check(window.routeIndex==1,"stale previous-leg telemetry cannot consume another waypoint");
    packet=frame(QByteArray::fromHex("d00d")); window.processFrames(packet,"Test");
    status[1]=char(1); qToLittleEndian<qint32>(window.routeForwardUm,status.data()+94); qToLittleEndian<qint32>(window.routeLeftUm,status.data()+98);
    qToLittleEndian<quint16>(window.expectedGeneration,status.data()+110);
    const QPointF unsafe=plan->obstacles[0].center-window.mapOrigin;
    qToLittleEndian<qint32>(qRound(unsafe.y()*10000),status.data()+102); qToLittleEndian<qint32>(qRound(-unsafe.x()*10000),status.data()+106);
    packet=frame(status); window.processFrames(packet,"Test");
    check(window.routePaused && window.stopPending && !plan->editable,"telemetry entering obstacle clearance triggers STOP and locks continuation");
    window.routePaused=false; window.motionRunning=false; window.stopPending=false; window.findChild<QTimer *>("stopRetryTimer")->stop(); plan->robot={0,0}; window.updateCommandUi(); plan->changed();
    QTemporaryDir directory(QDir::currentPath()+"/obstacles-XXXXXX"); check(directory.isValid(),"temporary directory");
    const auto path=directory.filePath("map.json"); QString error;
    check(window.exportRouteFile(path,error),"export map with obstacles"); QFile file(path); check(file.open(QIODevice::ReadOnly),"read map fixture"); const auto saved=file.readAll(); file.close();
    plan->obstacles.clear(); plan->robotWidthCm=40;
    check(window.importRouteFile(path,error) && plan->obstacles.size()==1 && !plan->obstacles[0].circular && plan->robotWidthCm==19.8 && plan->safetyDistanceCm==10,"JSON persists shapes and robot safety settings");
    auto invalid=QJsonDocument::fromJson(saved).object(); invalid.insert("safetyDistanceCm",-1);
    check(file.open(QIODevice::WriteOnly|QIODevice::Truncate),"write invalid map"); file.write(QJsonDocument(invalid).toJson()); file.close();
    check(!window.importRouteFile(path,error) && plan->obstacles.size()==1 && plan->safetyDistanceCm==10,"invalid safety data cannot alter current map");
    invalid=QJsonDocument::fromJson(saved).object();
    auto entries=invalid.value("obstacles").toArray(); auto malformed=entries[0].toObject(); malformed.insert("shape","triangle"); entries[0]=malformed; invalid.insert("obstacles",entries);
    check(file.open(QIODevice::WriteOnly|QIODevice::Truncate),"write invalid obstacle"); file.write(QJsonDocument(invalid).toJson()); file.close();
    check(!window.importRouteFile(path,error) && !plan->obstacles[0].circular && plan->obstacles[0].sizeCm==30,"invalid obstacle geometry preserves map");
    auto legacy=QJsonDocument::fromJson(saved).object(); legacy.insert("version",3);
    check(file.open(QIODevice::WriteOnly|QIODevice::Truncate),"write legacy route"); file.write(QJsonDocument(legacy).toJson()); file.close();
    check(window.importRouteFile(path,error) && plan->obstacles.isEmpty() && plan->points.size()==1,"version 3 routes remain importable without obstacles");
    check(file.open(QIODevice::WriteOnly|QIODevice::Truncate),"restore map"); file.write(saved); file.close(); check(window.importRouteFile(path,error),"restore obstacle map");
    // Reverse execution must use the same clearance planner, not a direct shortcut.
    plan->robot={0,120}; plan->changed(); window.startReverseRoute();
    check(window.routeActive && window.executionTargets.size()>1 && window.executionTargets.last()==QPointF(),"reverse route also detours around obstacle");
    for(const auto &step:plan->activeRoute.steps) check(plan->planner().pathFree(step.start,step.end,step.arcSweep),"reverse path remains collision free");
    window.cancelRoute("Fin de prueba"); window.motionRunning=false; window.moveAwaitingAck=false; plan->robot={0,0}; plan->changed(); window.updateCommandUi();
    window.startRoute();
    const int legs=window.executionTargets.size();
    for(int leg=0;leg<legs;++leg) {
        check(window.routeIndex==leg && window.routeActive,"continuous detour visits each planned leg once");
        packet=frame(QByteArray::fromHex("d00d")); window.processFrames(packet,"Test");
        const bool intermediate=window.executionCompletedCounts[leg]==0;
        status.fill('\0'); status[0]=char(0xD2); status[1]=char(intermediate ? 1 : 2); status[93]=char(window.mapMode);
        qToLittleEndian<qint32>(window.routeForwardUm,status.data()+94);
        qToLittleEndian<qint32>(window.routeLeftUm,status.data()+98);
        qToLittleEndian<quint16>(window.expectedGeneration<0 ? window.lastGeneration+1 : window.expectedGeneration,status.data()+110);
        const QPointF relative=window.plannedLegEnd-window.mapOrigin;
        qToLittleEndian<qint32>(qRound(relative.y()*10000),status.data()+102);
        qToLittleEndian<qint32>(qRound(-relative.x()*10000),status.data()+106);
        packet=frame(status); window.processFrames(packet,"Test");
        if(intermediate) check(!window.routePauseTimer->isActive() && plan->completed==0,"generated points have no pause and never complete the user destination");
    }
    check(plan->completed==1 && window.routeIndex==legs && window.routePauseTimer->isActive() && window.routePauseTimer->interval()==500,"user destination retains confirmed arrival and normal pause after continuous detour");
    window.cancelRoute("Fin de prueba continua"); window.motionRunning=false; window.moveAwaitingAck=false; plan->robot={0,0}; window.updateCommandUi();
    plan->points={{0,60}}; plan->showActiveRoute=false; plan->changed(); check(!plan->routePreview().valid,"destination inside clearance is rejected");
    const auto before=window.findChild<QTextBrowser *>("textBrowserUnProcessed")->toPlainText(); window.startRoute();
    check(!window.routeActive && before==window.findChild<QTextBrowser *>("textBrowserUnProcessed")->toPlainText(),"blocked goal never sends direct MOVE");
    plan->points.clear(); plan->headings.clear(); plan->arcSweeps.clear(); plan->changed();
    check(window.exportRouteFile(path,error) && window.importRouteFile(path,error) && plan->points.isEmpty() && plan->obstacles.size()==1,"obstacle-only maps can be saved and loaded");
    plan->points={{0,120},{70,100}}; plan->headings={90,0}; plan->arcSweeps={0,0}; plan->changed();
    window.grab().save("obstacles.png");
    std::puts("PASS: circle/square avoidance, footprint/clearance, blocked goals, arc collision, map editing, execution parity, arrival reserve and JSON");
}
