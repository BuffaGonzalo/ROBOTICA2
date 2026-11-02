#ifndef ROUTEPLANWIDGET_H
#define ROUTEPLANWIDGET_H
#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainterPath>
#include <QApplication>
#include <QDialog>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QScreen>
#include <QSignalBlocker>
#include <QSlider>
#include <QDataStream>
#include <QIODevice>
#include <QPolygonF>
#include <functional>
#include <cmath>
#include "../firmware/main/pose_kinematics.h"
#include "obstacleplanner.h"

class RoutePlanWidget : public QWidget {
public:
    explicit RoutePlanWidget(QWidget *parent=nullptr):QWidget(parent) {
        setMinimumSize(420,320);
        setToolTip("Cuadrícula mínima de 10 cm. Izquierdo en vacío: añadir. Sobre un punto: seleccionar y arrastrar. Derecho: eliminar.");
        speedPanel=new QFrame(this); speedPanel->setObjectName("mapSpeedPanel");
        speedPanel->setAttribute(Qt::WA_NoMousePropagation);
        speedPanel->setStyleSheet("QFrame#mapSpeedPanel { background: #142235; border: 1px solid #70bfd0; border-radius: 7px; } QLabel { background: transparent; border: none; } QSlider { background: transparent; border: none; } QSlider::groove:horizontal { height: 6px; background: #35445b; border-radius: 3px; } QSlider::handle:horizontal { width: 16px; margin: -5px 0; background: #70bfd0; border-radius: 8px; }");
        auto *speedLayout=new QVBoxLayout(speedPanel); speedLayout->setContentsMargins(12,10,12,10); speedLayout->setSpacing(7);
        speedCaption=new QLabel(speedPanel); speedCaption->setObjectName("label_mapSpeed");
        speedCaption->setWordWrap(true); speedLayout->addWidget(speedCaption);
        speedSlider=new QSlider(Qt::Horizontal,speedPanel); speedSlider->setObjectName("slider_mapSpeed");
        speedSlider->setRange(0,100); speedSlider->setValue(73); speedSlider->setFocusPolicy(Qt::NoFocus);
        speedSlider->setToolTip("0–100 % equivale a 0–255 PWM de referencia. Conserva las proporciones de los cuatro motores de Configuración. 0 % detiene el robot.");
        speedLayout->addWidget(speedSlider);
        motorPowerCaption=new QLabel("FR 170 · FL 170\nRR 175 · RL 185",speedPanel); motorPowerCaption->setObjectName("label_mapMotorPower");
        motorPowerCaption->setWordWrap(true); speedLayout->addWidget(motorPowerCaption);
        connect(speedSlider,&QSlider::valueChanged,this,[this]() { updateSpeedCaption(); });
        updateSpeedCaption(); positionSpeedPanel();
    }
    QSlider *speedSlider=nullptr;
    void showMotorPower(int fr,int fl,int rr,int rl) {
        motorPowerCaption->setText(QString("FR %1 · FL %2\nRR %3 · RL %4").arg(fr).arg(fl).arg(rr).arg(rl));
        updateSpeedCaption();
    }
    QList<QPointF> points;
    QList<double> headings;
    QList<double> arcSweeps;
    QList<MapObstacle> obstacles;
    bool obstacleMode=false;
    int selectedObstacle=-1;
    double robotLengthCm=30,robotWidthCm=19.8,safetyDistanceCm=10;
    AvoidanceRoute activeRoute;
    bool showActiveRoute=false;
    ObstaclePlanner planner(double reserve=(NavigationLimits::PassThroughMarginMm+NavigationLimits::TrackingReserveMm)/10) const {
        ObstaclePlanner result; result.obstacles=obstacles; result.robotLength=robotLengthCm;
        result.robotWidth=robotWidthCm; result.safetyDistance=safetyDistanceCm; result.reserve=reserve; return result;
    }
    QByteArray routeDefinitionKey() const {
        QByteArray key; QDataStream stream(&key,QIODevice::WriteOnly);
        stream<<points<<headings<<arcSweeps<<oriented<<robotLengthCm<<robotWidthCm<<safetyDistanceCm;
        for(const auto &o:obstacles) stream<<o.center<<o.circular<<o.sizeCm;
        return key;
    }
    AvoidanceRoute returnRoute(const AvoidanceRoute &outbound,bool mirror=false) const {
        AvoidanceRoute result;
        if(!outbound.valid || outbound.steps.isEmpty()) {
            result.valid=false; result.error="No hay un camino de ida válido para invertir."; return result;
        }
        const QPointF origin=outbound.steps.first().start;
        QPointF axis=outbound.steps.last().end-origin;
        const double length=std::hypot(axis.x(),axis.y());
        if(mirror && length<0.001) {
            result.valid=false; result.error="Espejo requiere que el inicio y el final sean distintos para definir el eje."; return result;
        }
        if(length>0) axis/=length;
        const auto reflect=[&](QPointF p) {
            if(!mirror || p==origin || p==outbound.steps.last().end) return p;
            const QPointF d=p-origin;
            return origin+axis*(2*QPointF::dotProduct(d,axis))-d;
        };
        const auto reflectHeading=[&](double degrees) {
            if(!mirror) return degrees;
            const double h=degrees*PoseKinematics::POSE_PI/180;
            const QPointF v(-std::sin(h),std::cos(h));
            const QPointF reflected=axis*(2*QPointF::dotProduct(v,axis))-v;
            return std::atan2(-reflected.x(),reflected.y())*180/PoseKinematics::POSE_PI;
        };
        const auto planning=planner();
        int completed=0;
        for(int i=outbound.steps.size()-1;i>=0;--i) {
            const auto &step=outbound.steps[i];
            const int count=(i==0 || outbound.steps[i-1].completedCount>0) ? ++completed : 0;
            AvoidanceStep reversed={reflect(step.end),reflect(step.start),reflectHeading(step.endHeading),
                reflectHeading(step.startHeading),mirror ? step.arcSweep : -step.arcSweep,count,step.detour};
            // Validate this exact path. Replanning here could silently select
            // the opposite side of an obstacle and violate the return choice.
            if(!planning.pathFree(reversed.start,reversed.end,reversed.arcSweep)) {
                result.valid=false; result.steps.clear();
                result.error=mirror ? "Camino espejo bloqueado por un obstáculo o fuera del plano. Revisá el mapa."
                    : "El camino de ida está bloqueado y no se puede recorrer en sentido inverso. Revisá el mapa.";
                return result;
            }
            result.steps.append(reversed);
        }
        return result;
    }
    AvoidanceRoute plannedRoute(QPointF start,double heading,bool reverse=false) const {
        // Build avoidance only in the outbound direction. Its exact expanded
        // sequence, including arcs and generated waypoints, defines the return.
        if(reverse) return returnRoute(plannedRoute(QPointF(),0));
        AvoidanceRoute result; const auto planning=planner();
        for(int ordinal=0;ordinal<points.size();++ordinal) {
            const int index=ordinal;
            const QPointF target=points[index];
            const double nextHeading=oriented ? headingAt(index) : heading;
            const double sweep=arcAt(index);
            const auto segment=planning.planSegment(start,target,heading,nextHeading,sweep,ordinal+1);
            if(!segment.valid) { result.valid=false; result.steps.clear(); result.error=QString("Destino %1: %2").arg(ordinal+1).arg(segment.error); return result; }
            result.steps.append(segment.steps); start=target; heading=nextHeading;
        }
        return result;
    }
    AvoidanceRoute routePreview() const {
        if(showActiveRoute) return activeRoute;
        QByteArray key; QDataStream stream(&key,QIODevice::WriteOnly);
        stream<<points<<headings<<arcSweeps<<oriented<<robotLengthCm<<robotWidthCm<<safetyDistanceCm;
        if(!obstacles.isEmpty()) stream<<robot<<robotHeadingDeg;
        for(const auto &o:obstacles) stream<<o.center<<o.circular<<o.sizeCm;
        if(key!=previewKey) {
            previewKey=key;
            previewRoute=plannedRoute(obstacles.isEmpty() ? QPointF() : robot,obstacles.isEmpty() ? 0 : robotHeadingDeg);
        }
        return previewRoute;
    }
    bool arcMode=false;
    double arcAt(int index) const { return index<arcSweeps.size() ? arcSweeps[index] : 0; }
    static QPointF arcPoint(QPointF start,QPointF end,double sweepDeg,double progress) {
        const QPointF delta=end-start;
        const auto pose=PoseKinematics::pathPose({delta.y(),-delta.x(),0},sweepDeg*PoseKinematics::POSE_PI/180,progress);
        return start+QPointF(-pose.left,pose.forward);
    }
    bool oriented=false;
    double robotHeadingDeg=0;
    double headingAt(int index) const {
        if(index<headings.size()) return headings[index];
        const QPointF delta=points[index]-(index>0 ? points[index-1] : QPointF());
        return std::atan2(-delta.x(),delta.y())*180/PoseKinematics::POSE_PI;
    }
    void setHeading(int index,double degrees) {
        while(headings.size()<points.size()) headings.append(headingAt(headings.size()));
        headings[index]=std::remainder(degrees,360.0);
        completed=0; update(); if(changed) changed();
    }
    QList<QPointF> trace;
    QPointF robot;
    int completed=0;
    bool editable=true;
    const bool diagonal=true; // Segmentos vectoriales directos a cualquier angulo.
    double pixelsPerCm=3;
    int selectedIndex=-1;
    QPointF viewOffset;
    double frameRotationDeg=0;
    bool orientationEditable=true;
    std::function<void()> changed;
    std::function<void(int)> zoomChanged;
    struct Leg { int axis; double distance; QPointF end; double forwardCm; double leftCm; };
    static Leg planLeg(QPointF from,QPointF to,bool=true) {
        const QPointF d=to-from;
        const double length=std::hypot(d.x(),d.y());
        const double scale=length>1000 ? 1000/length : 1;
        const QPointF step=d*scale;
        // Plano: +Y avance, +X derecha. Cuerpo Modern Robotics: +x avance, +y izquierda.
        return {4,length*scale,from+step,step.y(),-step.x()};
    }
    QPointF basisX() const { const double a=frameRotationDeg*3.141592653589793/180; return {std::cos(a),std::sin(a)}; }
    QPointF basisY() const { const double a=frameRotationDeg*3.141592653589793/180; return {std::sin(a),-std::cos(a)}; }
    QPointF screen(QPointF p) const { return QPointF(width()/2.0,height()/2.0)+viewOffset+(basisX()*p.x()+basisY()*p.y())*pixelsPerCm; }
    QPointF world(QPointF at) const {
        const QPointF d=(at-screen({0,0}))/pixelsPerCm;
        return {QPointF::dotProduct(d,basisX()),QPointF::dotProduct(d,basisY())};
    }
    void setRobot(QPointF p) {
        robot=p;
        if(trace.isEmpty() || QLineF(trace.last(),p).length()>0.05) {
            trace.append(p); if(trace.size()>5000) trace.removeFirst();
        }
        update();
    }
    void editPoint(int index) {
        if(!editable || index<0 || index>=points.size()) return;
        headingDragging=dragCandidate=dragging=false;
        QDialog dialog(this); dialog.setObjectName("setpointEditor");
        dialog.setWindowTitle(QString("Editar setpoint %1").arg(index+1));
        auto *outer=new QVBoxLayout(&dialog);
        auto *scroll=new QScrollArea(&dialog); scroll->setObjectName("setpointEditorScroll");
        scroll->setWidgetResizable(true); scroll->setFrameShape(QFrame::NoFrame);
        scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        auto *content=new QWidget(scroll); scroll->setWidget(content); outer->addWidget(scroll);
        auto *form=new QFormLayout(content);
        form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        form->setRowWrapPolicy(QFormLayout::WrapLongRows);
        form->setVerticalSpacing(10);
        QSpinBox x(&dialog),y(&dialog);
        x.setObjectName("setpointX"); y.setObjectName("setpointY");
        for(auto *spin:{&x,&y}) { spin->setRange(-1000,1000); spin->setSuffix(" cm"); }
        x.setValue(qRound(points[index].x())); y.setValue(qRound(points[index].y()));
        QDoubleSpinBox heading(&dialog); heading.setObjectName("setpointHeading");
        heading.setRange(-180,180); heading.setDecimals(1); heading.setSuffix("°"); heading.setValue(headingAt(index));
        heading.setToolTip("0° = +Y; los ángulos positivos apuntan hacia la izquierda. Se aplica al ejecutar con Orientación activa.");
        form->addRow("Posición X",&x); form->addRow("Posición Y",&y); form->addRow("Orientación de llegada",&heading);
        QComboBox type(&dialog),direction(&dialog);
        type.setObjectName("setpointPathType"); type.addItems({"Recta","Arco"}); type.setCurrentIndex(arcAt(index)!=0 ? 1 : 0);
        QDoubleSpinBox sweep(&dialog); sweep.setObjectName("setpointArcSweep"); sweep.setRange(-359,359); sweep.setDecimals(1); sweep.setSuffix("°");
        sweep.setToolTip("Ángulo positivo: antihorario. Ángulo negativo: horario. Usá una magnitud de 1° a 359°.");
        sweep.setValue(arcAt(index)==0 ? 180 : arcAt(index));
        direction.setObjectName("setpointArcDirection"); direction.addItems({"Antihorario","Horario"}); direction.setCurrentIndex(arcAt(index)<0 ? 1 : 0);
        form->addRow("Tramo de llegada",&type); form->addRow("Ángulo del arco",&sweep); form->addRow("Sentido del arco",&direction);
        QLabel radius(&dialog); radius.setObjectName("setpointArcRadius"); radius.setWordWrap(true);
        radius.setMinimumWidth(0); radius.setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred);
        radius.setToolTip("El radio se calcula al seleccionar Arco, según los extremos y el ángulo del arco.");
        form->addRow("Radio calculado",&radius);
        QPushButton semicircle("Semicircunferencia · 180°",&dialog); form->addRow(&semicircle);
        connect(&semicircle,&QPushButton::clicked,&dialog,[&]() { type.setCurrentIndex(1); sweep.setValue(direction.currentIndex()==1 ? -180 : 180); });
        connect(&sweep,&QDoubleSpinBox::valueChanged,&dialog,[&](double value) {
            if(value==0) return;
            const QSignalBlocker blocker(&direction); direction.setCurrentIndex(value<0 ? 1 : 0);
        });
        connect(&direction,&QComboBox::currentIndexChanged,&dialog,[&](int value) { sweep.setValue(std::abs(sweep.value())*(value==1 ? -1 : 1)); });
        const auto refresh=[&]() {
            const bool arc=type.currentIndex()==1; sweep.setEnabled(arc); direction.setEnabled(arc);
            const QPointF start=index>0 ? points[index-1] : QPointF();
            const double chord=QLineF(start,{double(x.value()),double(y.value())}).length();
            const bool nextInvalid=index+1<points.size() && arcAt(index+1)!=0 && QPointF(x.value(),y.value())==points[index+1];
            const bool validSweep=std::abs(sweep.value())>=1;
            radius.setText(arc && validSweep ? QString("%1 cm").arg(chord/(2*std::sin(std::abs(sweep.value())*PoseKinematics::POSE_PI/360)),0,'f',2) : QString("No aplica: tramo recto."));
            if(arc && !validSweep) radius.setText("Usá un ángulo entre ±1° y ±359°.");
            if(arc && chord==0) radius.setText("El arco requiere dos extremos distintos.");
            if(nextInvalid) radius.setText("El siguiente arco requiere extremos distintos.");
            if(auto *box=dialog.findChild<QDialogButtonBox *>()) box->button(QDialogButtonBox::Ok)->setEnabled((!arc || (chord>0 && validSweep)) && !nextInvalid);
        };
        connect(&type,&QComboBox::currentIndexChanged,&dialog,refresh);
        connect(&sweep,&QDoubleSpinBox::valueChanged,&dialog,refresh);
        connect(&x,&QSpinBox::valueChanged,&dialog,refresh); connect(&y,&QSpinBox::valueChanged,&dialog,refresh); refresh();
        QDialogButtonBox buttons(QDialogButtonBox::Ok|QDialogButtonBox::Cancel,&dialog);
        buttons.button(QDialogButtonBox::Ok)->setText("Guardar"); buttons.button(QDialogButtonBox::Cancel)->setText("Cancelar");
        outer->addWidget(&buttons);
        connect(&buttons,&QDialogButtonBox::accepted,&dialog,&QDialog::accept);
        connect(&buttons,&QDialogButtonBox::rejected,&dialog,&QDialog::reject);
        refresh();
        dialog.ensurePolished(); form->activate();
        const QRect available=dialog.screen()->availableGeometry();
        const int maxWidth=qMax(240,available.width()-48),maxHeight=qMax(200,available.height()-80);
        dialog.setMinimumSize(qMin(360,maxWidth),qMin(240,maxHeight));
        dialog.resize(qMin(560,maxWidth),qMin(content->sizeHint().height()+buttons.sizeHint().height()+48,maxHeight));
        if(dialog.exec()==QDialog::Accepted && editable && index<points.size()) {
            x.interpretText(); y.interpretText(); heading.interpretText();
            while(headings.size()<points.size()) headings.append(headingAt(headings.size()));
            while(arcSweeps.size()<points.size()) arcSweeps.append(0);
            points[index]={double(x.value()),double(y.value())};
            arcSweeps[index]=type.currentIndex()==1 ? sweep.value() : 0;
            selectedIndex=index; setHeading(index,heading.value());
        }
    }
    void editAxes() {
        if(!orientationEditable) return;
        orienting=headingDragging=dragCandidate=dragging=false; unsetCursor();
        QDialog dialog(this); dialog.setObjectName("axesEditor"); dialog.setWindowTitle("Rotación de los ejes");
        auto *form=new QFormLayout(&dialog); form->setRowWrapPolicy(QFormLayout::WrapLongRows);
        QDoubleSpinBox rotation(&dialog); rotation.setObjectName("axesRotation");
        rotation.setRange(-360,360); rotation.setDecimals(1); rotation.setSuffix("°"); rotation.setValue(frameRotationDeg);
        form->addRow("Ángulo de rotación",&rotation);
        QLabel help("0°: +Y hacia arriba y +X hacia la derecha. Ambos ejes giran juntos.",&dialog);
        help.setWordWrap(true); form->addRow(&help);
        QDialogButtonBox buttons(QDialogButtonBox::Ok|QDialogButtonBox::Cancel|QDialogButtonBox::Reset,&dialog);
        buttons.button(QDialogButtonBox::Ok)->setText("Guardar"); buttons.button(QDialogButtonBox::Cancel)->setText("Cancelar");
        buttons.button(QDialogButtonBox::Reset)->setText("Restablecer 0°");
        connect(buttons.button(QDialogButtonBox::Reset),&QPushButton::clicked,&dialog,[&]() { rotation.setValue(0); });
        connect(&buttons,&QDialogButtonBox::accepted,&dialog,&QDialog::accept);
        connect(&buttons,&QDialogButtonBox::rejected,&dialog,&QDialog::reject); form->addRow(&buttons);
        dialog.resize(qMin(440,dialog.screen()->availableGeometry().width()-48),dialog.sizeHint().height());
        if(dialog.exec()==QDialog::Accepted && orientationEditable) {
            rotation.interpretText(); frameRotationDeg=std::remainder(rotation.value(),360.0); update();
        }
    }
    void editObstacle(int index) {
        if(!editable || index<0 || index>=obstacles.size()) return;
        dragObstacle=-1;
        QDialog dialog(this); dialog.setObjectName("obstacleEditor"); dialog.setWindowTitle(QString("Editar obstáculo %1").arg(index+1));
        auto *outer=new QVBoxLayout(&dialog); auto *scroll=new QScrollArea(&dialog);
        scroll->setWidgetResizable(true); scroll->setFrameShape(QFrame::NoFrame); scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        auto *content=new QWidget(scroll); scroll->setWidget(content); outer->addWidget(scroll);
        auto *form=new QFormLayout(content); form->setRowWrapPolicy(QFormLayout::WrapLongRows); form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        QDoubleSpinBox x(&dialog),y(&dialog),size(&dialog),length(&dialog),width(&dialog),safety(&dialog);
        x.setObjectName("obstacleX"); y.setObjectName("obstacleY"); size.setObjectName("obstacleSize");
        length.setObjectName("robotLength"); width.setObjectName("robotWidth"); safety.setObjectName("obstacleSafety");
        for(auto *spin:{&x,&y,&size,&length,&width,&safety}) { spin->setDecimals(1); spin->setSuffix(" cm"); }
        x.setRange(-1000,1000); y.setRange(-1000,1000); size.setRange(1,500);
        length.setRange(1,200); width.setRange(1,200); safety.setRange(0,100);
        x.setValue(obstacles[index].center.x()); y.setValue(obstacles[index].center.y()); size.setValue(obstacles[index].sizeCm);
        length.setValue(robotLengthCm); width.setValue(robotWidthCm); safety.setValue(safetyDistanceCm);
        QComboBox shape(&dialog); shape.setObjectName("obstacleShape"); shape.addItems({"Circular","Cuadrado"}); shape.setCurrentIndex(obstacles[index].circular ? 0 : 1);
        form->addRow("Centro X",&x); form->addRow("Centro Y",&y); form->addRow("Forma",&shape);
        QLabel sizeLabel(&dialog); sizeLabel.setWordWrap(true); sizeLabel.setMinimumWidth(145); form->addRow(&sizeLabel,&size);
        const auto updateLabel=[&]() { sizeLabel.setText(shape.currentIndex()==0 ? "Diámetro" : "Lado del cuadrado"); };
        connect(&shape,&QComboBox::currentIndexChanged,&dialog,updateLabel); updateLabel();
        form->addRow("Largo exterior del robot",&length); form->addRow("Ancho exterior del robot",&width); form->addRow("Distancia de seguridad",&safety);
        QLabel note("Las medidas del robot y la distancia de seguridad se aplican a todos los obstáculos. La zona marcada incluye además reserva para la llegada y el seguimiento.",&dialog);
        note.setWordWrap(true); form->addRow(&note);
        QDialogButtonBox buttons(QDialogButtonBox::Ok|QDialogButtonBox::Cancel,&dialog); outer->addWidget(&buttons);
        buttons.button(QDialogButtonBox::Ok)->setText("Guardar"); buttons.button(QDialogButtonBox::Cancel)->setText("Cancelar");
        connect(&buttons,&QDialogButtonBox::accepted,&dialog,&QDialog::accept); connect(&buttons,&QDialogButtonBox::rejected,&dialog,&QDialog::reject);
        dialog.ensurePolished(); form->activate(); const QRect available=dialog.screen()->availableGeometry();
        dialog.resize(qMin(540,available.width()-48),qMin(content->sizeHint().height()+buttons.sizeHint().height()+48,available.height()-80));
        if(dialog.exec()==QDialog::Accepted && editable && index<obstacles.size()) {
            for(auto *spin:{&x,&y,&size,&length,&width,&safety}) spin->interpretText();
            obstacles[index]={{x.value(),y.value()},shape.currentIndex()==0,size.value()};
            robotLengthCm=length.value(); robotWidthCm=width.value(); safetyDistanceCm=safety.value(); selectedObstacle=index;
            completed=0; update(); if(changed) changed();
        }
    }
protected:
    void resizeEvent(QResizeEvent *event) override { QWidget::resizeEvent(event); positionSpeedPanel(); }
    void mouseDoubleClickEvent(QMouseEvent *event) override {
        if(event->button()!=Qt::LeftButton) return;
        if(QLineF(QPointF(78,78),event->position()).length()<=70) { editAxes(); event->accept(); return; }
        const int obstacle=hitObstacle(event->position());
        if(obstacle>=0) { editObstacle(obstacle); event->accept(); return; }
        editPoint(hitPoint(event->position())); event->accept();
    }
    void wheelEvent(QWheelEvent *event) override {
        wheelRemainder+=event->angleDelta().y();
        const int steps=wheelRemainder/120; wheelRemainder%=120;
        if(steps) {
            const int zoom=qBound(1,int(pixelsPerCm)+steps*2,40);
            pixelsPerCm=zoom; if(zoomChanged) zoomChanged(zoom); update();
        }
        event->accept();
    }
    void mousePressEvent(QMouseEvent *event) override {
        if(event->button()==Qt::LeftButton && QLineF(QPointF(78,78),event->position()).length()<=70) {
            if(orientationEditable) {
                const QPointF d=event->position()-QPointF(78,78);
                orientationStart=frameRotationDeg;
                orientationPressAngle=std::atan2(d.x(),-d.y())*180/3.141592653589793;
                orienting=true; setCursor(Qt::ClosedHandCursor);
            }
            event->accept(); return;
        }
        if(event->button()==Qt::MiddleButton) {
            panning=true; panPosition=event->position(); setCursor(Qt::ClosedHandCursor); event->accept(); return;
        }
        if(!editable) return;
        const QPointF pos=event->position();
        const int obstacle=hitObstacle(pos);
        if(event->button()==Qt::RightButton && obstacle>=0) {
            obstacles.removeAt(obstacle); selectedObstacle=-1; dragObstacle=-1; completed=0; update(); if(changed) changed(); return;
        }
        if(event->button()==Qt::LeftButton && obstacle>=0) {
            selectedObstacle=obstacle; selectedIndex=-1; dragObstacle=obstacle; obstacleOffset=world(pos)-obstacles[obstacle].center;
            update(); if(changed) changed(); return;
        }
        if(event->button()==Qt::LeftButton && obstacleMode) {
            const QPointF at=world(pos);
            if(obstacles.size()>=50 || std::abs(at.x())>1000 || std::abs(at.y())>1000) return;
            obstacles.append({{std::round(at.x()),std::round(at.y())},true,20}); selectedObstacle=obstacles.size()-1;
            selectedIndex=-1; completed=0; update(); if(changed) changed(); return;
        }
        selectedObstacle=-1;
        if(oriented && event->button()==Qt::LeftButton) {
            for(int i=0;i<points.size();++i) {
                const double angle=headingAt(i)*PoseKinematics::POSE_PI/180;
                const QPointF tip=screen(points[i])+(basisY()*std::cos(angle)-basisX()*std::sin(angle))*34;
                if(QLineF(pos,tip).length()<12) {
                    selectedIndex=i; headingDragging=true; update(); if(changed) changed();
                    event->accept(); return;
                }
            }
        }
        if(event->button()==Qt::LeftButton) {
            selectedIndex=hitPoint(pos);
            if(selectedIndex>=0) {
                pressPosition=pos;
                dragOffset=screen(points[selectedIndex])-pos;
                dragging=false; dragCandidate=true;
                setCursor(Qt::OpenHandCursor); update(); if(changed) changed();
                return;
            }
            const QPointF coordinate=world(pos);
            QPointF p(std::round(coordinate.x()),std::round(coordinate.y()));
            if(std::abs(p.x())>1000 || std::abs(p.y())>1000 || points.size()>=100) return;
            if(arcMode && p==(points.isEmpty() ? QPointF() : points.last())) return;
            if(points.isEmpty() || points.last()!=p) {
                while(headings.size()<points.size()) headings.append(headingAt(headings.size()));
                while(arcSweeps.size()<points.size()) arcSweeps.append(0);
                points.append(p); headings.append(headingAt(points.size()-1));
                arcSweeps.append(arcMode ? 180 : 0);
            }
            selectedIndex=points.size()-1;
        } else if(event->button()==Qt::RightButton) {
            const int best=hitPoint(pos);
            if(best>=0) {
                while(headings.size()<points.size()) headings.append(headingAt(headings.size()));
                while(arcSweeps.size()<points.size()) arcSweeps.append(0);
                points.removeAt(best); headings.removeAt(best); arcSweeps.removeAt(best); completed=0; selectedIndex=-1;
                if(best<points.size() && arcAt(best)!=0 && points[best]==(best>0 ? points[best-1] : QPointF())) arcSweeps[best]=0;
            }
        } else return;
        update(); if(changed) changed();
    }
    void mouseMoveEvent(QMouseEvent *event) override {
        if(editable && dragObstacle>=0 && dragObstacle<obstacles.size() && (event->buttons()&Qt::LeftButton)) {
            const QPointF at=world(event->position())-obstacleOffset;
            const QPointF next(qBound(-1000.0,std::round(at.x()),1000.0),qBound(-1000.0,std::round(at.y()),1000.0));
            if(next!=obstacles[dragObstacle].center) { obstacles[dragObstacle].center=next; completed=0; update(); if(changed) changed(); }
            return;
        }
        if(headingDragging && editable && (event->buttons()&Qt::LeftButton) && selectedIndex>=0 && selectedIndex<points.size()) {
            const QPointF delta=world(event->position())-points[selectedIndex];
            if(std::hypot(delta.x(),delta.y())>0.1) setHeading(selectedIndex,std::atan2(-delta.x(),delta.y())*180/PoseKinematics::POSE_PI);
            return;
        }
        if(orienting && orientationEditable && (event->buttons()&Qt::LeftButton)) {
            const QPointF d=event->position()-QPointF(78,78);
            if(QLineF({0,0},d).length()>8) {
                const double angle=std::atan2(d.x(),-d.y())*180/3.141592653589793;
                frameRotationDeg=std::remainder(orientationStart+angle-orientationPressAngle,360.0);
                update();
            }
            return;
        }
        if(panning && (event->buttons()&Qt::MiddleButton)) {
            viewOffset+=event->position()-panPosition; panPosition=event->position(); update(); event->accept(); return;
        }
        if(!editable || !dragCandidate || !(event->buttons()&Qt::LeftButton) || selectedIndex<0 || selectedIndex>=points.size()) return;
        if(!dragging && QLineF(pressPosition,event->position()).length()<QApplication::startDragDistance()) return;
        dragging=true; setCursor(Qt::ClosedHandCursor);
        const QPointF at=event->position()+dragOffset;
        const QPointF coordinate=world(at);
        QPointF next(qBound(-1000.0,std::round(coordinate.x()),1000.0),qBound(-1000.0,std::round(coordinate.y()),1000.0));
        if(arcAt(selectedIndex)!=0 && next==(selectedIndex>0 ? points[selectedIndex-1] : QPointF())) return;
        if(selectedIndex+1<points.size() && arcAt(selectedIndex+1)!=0 && next==points[selectedIndex+1]) return;
        if(points[selectedIndex]!=next) {
            points[selectedIndex]=next; completed=0; update(); if(changed) changed();
        }
    }
    void mouseReleaseEvent(QMouseEvent *event) override {
        if(event->button()==Qt::MiddleButton) { panning=false; unsetCursor(); event->accept(); return; }
        if(event->button()==Qt::LeftButton) {
            dragObstacle=-1;
            const bool edit=editable && arcMode && dragCandidate && !dragging;
            headingDragging=orienting=dragCandidate=dragging=false; unsetCursor();
            if(edit) editPoint(selectedIndex);
        }
    }
    void paintEvent(QPaintEvent *) override {
        QPainter p(this); p.setRenderHint(QPainter::Antialiasing);
        p.fillRect(rect(),QColor("#0e192b"));
        int step=10; // Nunca subdividir por debajo de 10 cm.
        while(step*pixelsPerCm<30) step*=2;
        p.setPen(QPen(QColor("#26384b"),1));
        double minX=0,maxX=0,minY=0,maxY=0;
        bool first=true;
        for(QPointF corner:{QPointF(0,0),QPointF(width(),0),QPointF(0,height()),QPointF(width(),height())}) {
            const auto at=world(corner);
            if(first) { minX=maxX=at.x(); minY=maxY=at.y(); first=false; }
            minX=qMin(minX,at.x()); maxX=qMax(maxX,at.x()); minY=qMin(minY,at.y()); maxY=qMax(maxY,at.y());
        }
        for(int x=int(std::floor(minX/step))*step;x<=maxX;x+=step) p.drawLine(screen({double(x),minY}),screen({double(x),maxY}));
        for(int y=int(std::floor(minY/step))*step;y<=maxY;y+=step) p.drawLine(screen({minX,double(y)}),screen({maxX,double(y)}));
        p.setPen(QPen(QColor("#7396b3"),2));
        p.drawLine(screen({minX,0}),screen({maxX,0}));
        p.drawLine(screen({0,minY}),screen({0,maxY}));
        p.setPen(QColor("#c0d9e7"));
        const int labelStep=step*qMax(1,int(std::ceil(55.0/(step*pixelsPerCm))));
        for(int x=int(std::floor(minX));x<=int(std::ceil(maxX));x+=1)
            if(x%labelStep==0) p.drawText(screen({double(x),0})+QPointF(3,16),QString::number(x));
        for(int y=int(std::floor(minY));y<=int(std::ceil(maxY));y+=1)
            if(y && y%step==0) p.drawText(screen({0,double(y)})+QPointF(5,-3),QString::number(y));
        for(int i=0;i<obstacles.size();++i) {
            const auto &o=obstacles[i];
            const auto shape=[&](double half) {
                if(o.circular) p.drawEllipse(screen(o.center),half*pixelsPerCm,half*pixelsPerCm);
                else {
                    QPolygonF polygon;
                    for(const auto &d:{QPointF(-half,-half),QPointF(half,-half),QPointF(half,half),QPointF(-half,half)}) polygon<<screen(o.center+d);
                    p.drawPolygon(polygon);
                }
            };
            p.setPen(QPen(QColor("#a57954"),1,Qt::DashLine)); p.setBrush(QColor(230,140,70,25)); shape(o.sizeCm/2+planner().clearance());
            p.setPen(QPen(QColor(i==selectedObstacle ? "#ffd3a1" : "#ed9965"),i==selectedObstacle ? 3 : 2)); p.setBrush(QColor("#753e37")); shape(o.sizeCm/2);
            p.setPen(QColor("#fff0df")); p.drawText(screen(o.center)+QPointF(-9,5),QString("O%1").arg(i+1));
        }
        // Drawing and execution share this obstacle-aware plan.
        const auto route=routePreview();
        if(route.valid) for(const auto &leg:route.steps) {
            p.setPen(QPen(QColor(leg.detour ? "#70d8c3" : "#e6b755"),2,Qt::DashLine)); p.setBrush(Qt::NoBrush);
            QPainterPath path; path.moveTo(screen(leg.start));
            if(leg.arcSweep!=0) {
                const int samples=qMax(32,int(std::abs(leg.arcSweep)));
                for(int sample=1;sample<=samples;++sample) path.lineTo(screen(arcPoint(leg.start,leg.end,leg.arcSweep,double(sample)/samples)));
                p.drawText(screen(arcPoint(leg.start,leg.end,leg.arcSweep,0.5))+QPointF(6,-8),QString("Arco %1°").arg(leg.arcSweep,0,'f',0));
            } else path.lineTo(screen(leg.end));
            p.drawPath(path);
            if(leg.detour && leg.completedCount==0) { p.setBrush(QColor("#70d8c3")); p.drawEllipse(screen(leg.end),4,4); }
        }
        else {
            p.setPen(QPen(QColor("#ef6775"),2,Qt::DashLine)); QPointF previous=robot;
            for(const auto &point:points) { p.drawLine(screen(previous),screen(point)); previous=point; }
        }
        p.setPen(QPen(QColor("#63daca"),3));
        for(int i=1;i<trace.size();++i) p.drawLine(screen(trace[i-1]),screen(trace[i]));
        for(int i=0;i<points.size();++i) {
            const QPointF at=screen(points[i]);
            if(oriented) {
                const double angle=headingAt(i)*PoseKinematics::POSE_PI/180;
                const QPointF direction=basisY()*std::cos(angle)-basisX()*std::sin(angle);
                const QPointF tip=at+direction*34,normal(-direction.y(),direction.x());
                p.setPen(QPen(QColor("#70d8c3"),2)); p.drawLine(at,tip);
                p.drawLine(tip,tip-direction*9+normal*5); p.drawLine(tip,tip-direction*9-normal*5);
                p.setBrush(QColor("#70d8c3")); p.drawEllipse(tip,4,4);
                p.drawText(tip+QPointF(6,-6),QString("%1°").arg(headingAt(i),0,'f',0));
            }
            if(i==selectedIndex) {
                p.setBrush(Qt::NoBrush); p.setPen(QPen(QColor("#ffffff"),2)); p.drawEllipse(at,14,14);
            }
            p.setBrush(QColor(i<completed ? "#338c73" : "#e6b755")); p.setPen(Qt::NoPen); p.drawEllipse(at,10,10);
            p.setPen(QColor("#0e192b"));
            p.drawText(QRectF(at-QPointF(10,10),QSizeF(20,20)),Qt::AlignCenter,QString::number(i+1));
            p.setPen(QColor("#e4ebf4"));
            const bool leftSide=at.x()<screen({0,0}).x();
            p.drawText(QRectF(at+QPointF(leftSide ? -116 : 16,-26-16*(i%2)),QSizeF(100,20)),
                       (leftSide ? Qt::AlignRight : Qt::AlignLeft)|Qt::AlignVCenter,
                       QString("(%1, %2)").arg(points[i].x(),0,'f',1).arg(points[i].y(),0,'f',1));
        }
        drawRobot(p);
        p.setPen(QColor("#a8c7d5"));
        p.drawText(12,height()-12,QString("Cuadrícula: %1 cm · Robot (%2, %3) cm · estimado por encoders").arg(step).arg(robot.x(),0,'f',2).arg(robot.y(),0,'f',2));
        // Referencia arrastrable: +X siempre a 90 grados a la derecha de +Y.
        p.setPen(QPen(QColor("#70bfd0"),2));
        p.setBrush(QColor("#142235"));
        p.drawEllipse(QPointF(78,78),70,70);
        for(int tick=0;tick<60;++tick) {
            const double angle=tick*2*PoseKinematics::POSE_PI/60;
            const QPointF direction(std::sin(angle),-std::cos(angle));
            const bool major=tick%5==0;
            p.setPen(QPen(QColor(major ? "#70bfd0" : "#496c83"),major ? 1.8 : 1.0,Qt::SolidLine,Qt::RoundCap));
            p.drawLine(QPointF(78,78)+direction*(major ? 60 : 64),QPointF(78,78)+direction*68);
        }
        auto arrow=[&](QPointF dir,QColor color,const QString &label) {
            const QPointF center(78,78), tip=center+dir*45;
            const QPointF normal(-dir.y(),dir.x());
            p.setPen(QPen(color,3)); p.drawLine(center,tip);
            p.drawLine(tip,tip-dir*11+normal*6); p.drawLine(tip,tip-dir*11-normal*6);
            p.drawText(tip+QPointF(-2,-5),label);
        };
        arrow(basisY(),QColor("#63daca"),"+Y"); arrow(basisX(),QColor("#e6b755"),"+X");
    }
private:
    mutable QByteArray previewKey;
    mutable AvoidanceRoute previewRoute;
    int dragObstacle=-1;
    QPointF obstacleOffset;
    int hitObstacle(QPointF at) const {
        const QPointF coordinate=world(at);
        for(int i=obstacles.size()-1;i>=0;--i) {
            const auto &o=obstacles[i]; const QPointF d=coordinate-o.center;
            const double half=qMax(o.sizeCm/2,8/pixelsPerCm);
            if(o.circular ? std::hypot(d.x(),d.y())<=half : std::abs(d.x())<=half && std::abs(d.y())<=half) return i;
        }
        return -1;
    }
    QFrame *speedPanel=nullptr;
    QLabel *speedCaption=nullptr,*motorPowerCaption=nullptr;
    void positionSpeedPanel() {
        if(!speedPanel) return;
        speedPanel->setFixedWidth(qMin(300,qMax(220,width()-176)));
        speedPanel->adjustSize(); speedPanel->move(width()-speedPanel->width()-12,12); speedPanel->raise();
    }
    void updateSpeedCaption() {
        speedCaption->setText(QString("Velocidad: %1 % · PWM %2/255").arg(speedSlider->value()).arg(qRound(speedSlider->value()*255.0/100)));
    }
    void drawRobot(QPainter &p) const {
        p.save();
        if(!obstacles.isEmpty()) {
            const double angle=robotHeadingDeg*PoseKinematics::POSE_PI/180;
            const QPointF right(std::cos(angle),std::sin(angle)),front(-std::sin(angle),std::cos(angle));
            QPolygonF footprint;
            for(const auto &corner:{QPointF(-1,-1),QPointF(1,-1),QPointF(1,1),QPointF(-1,1)}) footprint<<screen(robot+right*(corner.x()*robotWidthCm/2)+front*(corner.y()*robotLengthCm/2));
            p.setPen(QPen(QColor("#70bfd0"),1.5)); p.setBrush(QColor(112,191,208,35)); p.drawPolygon(footprint);
        }
        p.translate(screen(robot));
        // Indicador de posición con orientación +Y y tamaño visual constante.
        p.rotate(frameRotationDeg-robotHeadingDeg);
        p.setPen(QPen(Qt::black,1,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));
        p.setBrush(QColor("#ef4444"));
        QPainterPath arrow;
        arrow.moveTo(0,-18);
        arrow.lineTo(14,13);
        arrow.quadTo(15,15,12,14);
        arrow.lineTo(0,10);
        arrow.lineTo(-12,14);
        arrow.quadTo(-15,15,-14,13);
        arrow.closeSubpath();
        p.drawPath(arrow);
        p.restore();
    }
    bool panning=false;
    bool headingDragging=false;
    bool orienting=false;
    double orientationStart=0,orientationPressAngle=0;
    QPointF panPosition;
    int hitPoint(QPointF position) const {
        int best=-1; double nearest=13;
        for(int i=0;i<points.size();++i) {
            const double distance=QLineF(screen(points[i]),position).length();
            if(distance<nearest) { nearest=distance; best=i; }
        }
        return best;
    }
    bool dragCandidate=false, dragging=false;
    QPointF pressPosition,dragOffset;
    int wheelRemainder=0;
};
#endif
