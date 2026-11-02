#include <QApplication>
#include <QPushButton>
#include <QFontDatabase>
#include <QLabel>
#include "routeplanwidget.h"
#define private public
#include "mainwindow.h"
#undef private
#include <cstdio>
#include <cstdlib>

static void check(bool ok,const char *why) {
    if(!ok) { std::fprintf(stderr,"FAIL: %s\n",why); std::exit(1); }
}
static bool near(QPointF a,QPointF b) { return QLineF(a,b).length()<1e-7; }
static void testGeometry() {
    RoutePlanWidget map;
    map.points={{-50,0},{-50,50}};
    const auto outbound=map.plannedRoute({},0);
    auto back=map.returnRoute(outbound);
    check(back.valid && back.steps.size()==2 && near(back.steps[0].end,{-50,0}) && near(back.steps[1].end,{}),"normal L returns through the original corner");
    auto mirrored=map.returnRoute(outbound,true);
    check(mirrored.valid && near(mirrored.steps[0].start,{-50,50}) && near(mirrored.steps[0].end,{0,50}) && near(mirrored.steps.last().end,{}),"mirrored L uses the opposite corner and keeps both endpoints");
    check(map.points==QList<QPointF>({{-50,0},{-50,50}}),"return never edits the outbound setpoints");
    map.obstacles={{{0,50},true,5}};
    check(map.returnRoute(outbound).valid && !map.returnRoute(outbound,true).valid,"blocked reflection is rejected without substituting the original side");

    map.obstacles={{{0,70},false,20}}; map.points={{0,140},{100,140}};
    const auto detour=map.plannedRoute({},0);
    back=map.returnRoute(detour);
    check(detour.valid && back.valid && back.steps.size()==detour.steps.size() && detour.steps.size()>2,"obstacle return retains every expanded outbound leg");
    int completed=0;
    for(int i=0;i<back.steps.size();++i) {
        const auto &forward=detour.steps[detour.steps.size()-1-i];
        const auto &reverse=back.steps[i];
        check(near(reverse.start,forward.end) && near(reverse.end,forward.start) && reverse.arcSweep==-forward.arcSweep,"return reverses the exact geometry rather than replanning avoidance");
        if(reverse.completedCount) check(reverse.completedCount==++completed,"only original destinations count on return");
    }
    check(completed==2,"automatic waypoints do not add stops to the reversed route");

    map.obstacles.clear(); map.points={{0,100}}; map.arcSweeps={180}; map.oriented=true; map.headings={90};
    const auto arc=map.plannedRoute({},0);
    back=map.returnRoute(arc); mirrored=map.returnRoute(arc,true);
    check(back.steps[0].arcSweep==-180 && mirrored.steps[0].arcSweep==180,"reflection and reversal each flip the circular sweep");
    check(std::abs(mirrored.steps[0].startHeading+90)<1e-6 && std::abs(mirrored.steps[0].endHeading)<1e-6,"reflected orientation follows the mirrored geometry");
    for(int i=0;i<=20;++i) {
        const auto p=RoutePlanWidget::arcPoint({},map.points[0],180,1-i/20.0);
        check(near(RoutePlanWidget::arcPoint(back.steps[0].start,back.steps[0].end,-180,i/20.0),p),"all sampled reverse arc positions match outbound arc");
        check(near(RoutePlanWidget::arcPoint(mirrored.steps[0].start,mirrored.steps[0].end,180,i/20.0),{-p.x(),p.y()}),"all sampled mirror arc positions lie on reflected outbound arc");
    }
    map.points={{50,0},{0,0}}; map.arcSweeps={0,0};
    check(!map.returnRoute(map.plannedRoute({},0),true).valid,"closed routes reject an undefined mirror axis");
}

int main(int argc,char **argv) {
    QApplication app(argc,argv); testGeometry();
    QFontDatabase::addApplicationFont("C:/Windows/Fonts/segoeui.ttf"); app.setFont(QFont("Segoe UI",10));
    MainWindow window;
    auto *map=static_cast<RoutePlanWidget *>(window.findChild<QWidget *>("routePlan"));
    auto *mirror=window.findChild<QPushButton *>("pushButton_mirrorRoute");
    check(mirror && mirror->text()=="Espejo" && mirror->isCheckable() && !mirror->isChecked(),"toolbar has an initially inactive Espejo toggle");
    map->points={{-50,0},{-50,50}}; map->robot={}; map->changed();
    check(window.prepareRoute(false),"prepare outbound L");
    window.cancelRoute("test"); map->robot=map->points.last();
    check(window.prepareRoute(true) && near(window.executionTargets[0],{-50,0}),"Qt normal return uses the outbound corner");
    window.updateCommandUi(); check(!mirror->isEnabled(),"mirror choice is locked while a route executes");
    window.cancelRoute("test"); mirror->click();
    check(mirror->isChecked() && window.prepareRoute(true) && near(window.executionTargets[0],{0,50}),"Qt mirror toggle changes the executed return corner");
    check(near(map->activeRoute.steps[0].end,window.executionTargets[0]),"drawn return and executed return share the same route");
    window.cancelRoute("test"); mirror->click();

    map->points={{0,140}}; map->obstacles={{{0,70},false,20}}; map->robot={10,-10}; map->changed();
    check(window.prepareRoute(false),"prepare avoidance from a nonzero outbound origin");
    const auto saved=window.outboundRoute;
    window.cancelRoute("test"); map->robot=map->points.last(); map->changed();
    check(window.prepareRoute(true) && near(window.executionTargets.last(),{10,-10}),"return uses saved outbound origin even after the active drawing is cleared");
    for(int i=0;i<saved.steps.size();++i) check(near(window.executionTargets[i],saved.steps[saved.steps.size()-1-i].start),"Qt retains every saved avoidance waypoint in reverse order");
    window.cancelRoute("test");
    window.findChild<QPushButton *>("pushButton_resetMap")->click();
    check(window.outboundRoute.steps.isEmpty(),"reset discards the previous outbound origin");

    map->obstacles.clear(); map->points={{-50,0},{-50,50}}; map->robot=map->points.last(); map->changed(); mirror->click();
    map->obstacles={{{0,50},true,5}};
    check(!window.prepareRoute(true) && window.findChild<QLabel *>("label_routeStatus")->text().contains("espejo bloqueado"),"Qt refuses a blocked mirrored return");
    map->obstacles.clear(); map->changed();
    check(window.prepareRoute(true),"prepare mirror preview for layout verification"); window.cancelRoute("test"); map->showActiveRoute=true;
    window.resize(1100,780); window.show(); app.processEvents();
    check(mirror->isVisible() && mirror->geometry().width()>40,"mirror toggle remains visible in the toolbar");
    window.grab().save("return-mirror.png");
    std::puts("PASS: exact outbound reversal, mirrored L/arcs/headings, saved avoidance, destination counts, blocked reflection, reset, UI toggle and preview parity");
}
