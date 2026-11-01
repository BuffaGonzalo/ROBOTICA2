#include <QApplication>
#include <QFile>
#include <QDir>
#include <QFontDatabase>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QTemporaryDir>
#include "routeplanwidget.h"
#define private public
#include "mainwindow.h"
#undef private
#include <cstdio>
#include <cstdlib>
static void check(bool ok,const char *message) {
    if(!ok) { std::fprintf(stderr,"FAIL: %s\n",message); std::exit(1); }
}
int main(int argc,char **argv) {
    QApplication app(argc,argv);
    QFontDatabase::addApplicationFont("C:/Windows/Fonts/segoeui.ttf");
    app.setFont(QFont("Segoe UI",10));
    MainWindow window;
    auto *plan=static_cast<RoutePlanWidget *>(window.findChild<QWidget *>("routePlan"));
    auto *importButton=window.findChild<QPushButton *>("pushButton_importRoute");
    auto *exportButton=window.findChild<QPushButton *>("pushButton_exportRoute");
    check(importButton && exportButton,"JSON buttons exist");
    check(!window.findChild<QLabel *>("label_mapZoom"),"Zoom label removed");
    check(importButton->isEnabled() && !exportButton->isEnabled(),"offline import, no empty export");
    plan->points={{-1000,1000},{10,-20},{0,0}}; plan->frameRotationDeg=37.5; plan->changed();
    QTemporaryDir directory(QDir::currentPath()+"/route-json-XXXXXX"); check(directory.isValid(),"temporary directory");
    const QString path=directory.filePath("route.json"); QString error;
    if(!window.exportRouteFile(path,error)) { std::fprintf(stderr,"Export: %s\n",qPrintable(error)); return 1; }
    QFile file(path); check(file.open(QIODevice::ReadOnly),"read exported file");
    const auto original=QJsonDocument::fromJson(file.readAll()).object(); file.close();
    const QList<QPointF> expected=plan->points;
    plan->points.clear(); plan->frameRotationDeg=0;
    check(window.importRouteFile(path,error),"import exported route");
    check(plan->points==expected && plan->frameRotationDeg==37.5,"round trip coordinates, order and rotation");
    const auto reject=[&](const QByteArray &data) {
        check(file.open(QIODevice::WriteOnly|QIODevice::Truncate),"write fixture"); file.write(data); file.close();
        check(!window.importRouteFile(path,error),"reject invalid input");
        check(plan->points==expected && plan->frameRotationDeg==37.5,"failed import preserves route");
    };
    reject("{broken"); reject("[]");
    for(const auto &key:{QString("version"),QString("units"),QString("axes")}) {
        auto invalid=original; invalid.insert(key,"unsupported"); reject(QJsonDocument(invalid).toJson());
    }
    for(const auto &point:{QJsonObject{{"x",1001},{"y",0}},QJsonObject{{"x",1.5},{"y",0}},
                          QJsonObject{{"x","10"},{"y",0}},QJsonObject{{"x",0}}}) {
        auto invalid=original; invalid.insert("points",QJsonArray{QJsonObject{{"x",0},{"y",0}},point});
        reject(QJsonDocument(invalid).toJson());
    }
    for(int count:{0,101}) {
        QJsonArray points; for(int i=0;i<count;++i) points.append(QJsonObject{{"x",i},{"y",0}});
        auto invalid=original; invalid.insert("points",points); reject(QJsonDocument(invalid).toJson());
    }
    auto invalid=original; invalid.insert("frameRotationDeg","bad"); reject(QJsonDocument(invalid).toJson());
    invalid=original; invalid.insert("orientationEnabled","bad"); reject(QJsonDocument(invalid).toJson());
    for(QJsonValue heading:{QJsonValue("bad"),QJsonValue(181),QJsonValue(QJsonValue::Null)}) {
        invalid=original; invalid.insert("points",QJsonArray{QJsonObject{{"x",0},{"y",0},{"headingDeg",heading}}}); reject(QJsonDocument(invalid).toJson());
    }
    // Existing version 1 files remain importable without angle fields.
    auto legacy=original; legacy.insert("version",1); legacy.remove("orientationEnabled");
    QJsonArray legacyPoints;
    for(const QPointF &point:expected) legacyPoints.append(QJsonObject{{"x",point.x()},{"y",point.y()}});
    legacy.insert("points",legacyPoints);
    check(file.open(QIODevice::WriteOnly|QIODevice::Truncate),"write legacy fixture"); file.write(QJsonDocument(legacy).toJson()); file.close();
    check(window.importRouteFile(path,error) && plan->points==expected && !plan->oriented,"legacy routes default to no orientation");
    check(window.exportRouteFile(path,error),"restore valid file");
    window.motionRunning=true; window.updateCommandUi();
    check(!importButton->isEnabled() && exportButton->isEnabled(),"movement locks import only");
    check(!window.importRouteFile(path,error) && plan->points==expected,"cannot replace active movement");
    window.motionRunning=false; window.routePaused=true; window.updateCommandUi();
    check(!importButton->isEnabled() && !window.importRouteFile(path,error),"paused route blocks import");
    window.routePaused=false; window.updateCommandUi();
    check(importButton->isEnabled(),"import restored when idle");
    window.resize(1100,780); window.show(); app.processEvents();
    check(window.grab().save("route-json-visualization.png"),"save visualization preview");
    std::puts("PASS: route JSON round trip, validation, atomic import and movement locks");
}
