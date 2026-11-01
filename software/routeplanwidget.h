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
#include <functional>
#include <cmath>
#include "../firmware/main/pose_kinematics.h"

class RoutePlanWidget : public QWidget {
public:
    explicit RoutePlanWidget(QWidget *parent=nullptr):QWidget(parent) {
        setMinimumSize(420,320);
        setToolTip("Cuadrícula mínima de 10 cm. Izquierdo en vacío: añadir. Sobre un punto: seleccionar y arrastrar. Derecho: eliminar.");
    }
    QList<QPointF> points;
    QList<double> headings;
    QList<double> arcSweeps;
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
        QDoubleSpinBox sweep(&dialog); sweep.setObjectName("setpointArcSweep"); sweep.setRange(1,359); sweep.setDecimals(1); sweep.setSuffix("°");
        sweep.setValue(arcAt(index)==0 ? 180 : std::abs(arcAt(index)));
        direction.setObjectName("setpointArcDirection"); direction.addItems({"Antihorario","Horario"}); direction.setCurrentIndex(arcAt(index)<0 ? 1 : 0);
        form->addRow("Tramo de llegada",&type); form->addRow("Ángulo del arco",&sweep); form->addRow("Sentido del arco",&direction);
        QLabel radius(&dialog); radius.setObjectName("setpointArcRadius"); radius.setWordWrap(true);
        radius.setMinimumWidth(0); radius.setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred);
        radius.setToolTip("El radio se calcula al seleccionar Arco, según los extremos y el ángulo del arco.");
        form->addRow("Radio calculado",&radius);
        QPushButton semicircle("Semicircunferencia · 180°",&dialog); form->addRow(&semicircle);
        connect(&semicircle,&QPushButton::clicked,&dialog,[&]() { type.setCurrentIndex(1); sweep.setValue(180); });
        const auto refresh=[&]() {
            const bool arc=type.currentIndex()==1; sweep.setEnabled(arc); direction.setEnabled(arc);
            const QPointF start=index>0 ? points[index-1] : QPointF();
            const double chord=QLineF(start,{double(x.value()),double(y.value())}).length();
            const bool nextInvalid=index+1<points.size() && arcAt(index+1)!=0 && QPointF(x.value(),y.value())==points[index+1];
            radius.setText(arc ? QString("%1 cm").arg(chord/(2*std::sin(sweep.value()*PoseKinematics::POSE_PI/360)),0,'f',2) : QString("No aplica: tramo recto."));
            if(arc && chord==0) radius.setText("El arco requiere dos extremos distintos.");
            if(nextInvalid) radius.setText("El siguiente arco requiere extremos distintos.");
            if(auto *box=dialog.findChild<QDialogButtonBox *>()) box->button(QDialogButtonBox::Ok)->setEnabled((!arc || chord>0) && !nextInvalid);
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
            arcSweeps[index]=type.currentIndex()==1 ? sweep.value()*(direction.currentIndex()==0 ? 1 : -1) : 0;
            selectedIndex=index; setHeading(index,heading.value());
        }
    }
protected:
    void mouseDoubleClickEvent(QMouseEvent *event) override {
        if(event->button()!=Qt::LeftButton) return;
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
        if(event->button()==Qt::LeftButton && QRectF(8,8,145,140).contains(event->position())) {
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
        // Misma función de planificación que usa la ejecución de la ruta.
        QPointF previous(0,0);
        p.setPen(QPen(QColor("#e6b755"),2,Qt::DashLine));
        for(int pointIndex=0;pointIndex<points.size();++pointIndex) {
            const QPointF point=points[pointIndex];
            if(arcAt(pointIndex)!=0) {
                const QPointF start=previous;
                const int samples=qMax(32,int(std::abs(arcAt(pointIndex))));
                for(int sample=1;sample<=samples;++sample) {
                    const QPointF at=arcPoint(start,point,arcAt(pointIndex),double(sample)/samples);
                    p.drawLine(screen(previous),screen(at)); previous=at;
                }
                const QPointF middle=screen(arcPoint(start,point,arcAt(pointIndex),0.5));
                p.drawText(middle+QPointF(6,-8),QString("Arco %1°").arg(arcAt(pointIndex),0,'f',0));
                previous=point; continue;
            }
            const QPointF segment=point-previous;
            const QPointF screenSegment=screen(point)-screen(previous);
            const double pixelLength=std::hypot(screenSegment.x(),screenSegment.y());
            if(pixelLength>100) {
                const double angle=std::atan2(-segment.x(),segment.y())*180/3.141592653589793;
                const QPointF offset(-screenSegment.y()/pixelLength*20,screenSegment.x()/pixelLength*20);
                const QPointF at=screen((previous+point)/2)+offset;
                p.drawText(QRectF(at-QPointF(40,10),QSizeF(80,20)),Qt::AlignCenter,QString("%1%2°").arg(angle>=0 ? "+" : "").arg(angle,0,'f',1));
            }
            for(int leg=0;leg<10 && QLineF(previous,point).length()>0.05;++leg) {
                const auto planned=planLeg(previous,point,diagonal);
                p.drawLine(screen(previous),screen(planned.end)); previous=planned.end;
            }
            previous=point;
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
        p.fillRect(QRectF(8,8,145,140),QColor("#142235"));
        auto arrow=[&](QPointF dir,QColor color,const QString &label) {
            const QPointF center(78,78), tip=center+dir*45;
            const QPointF normal(-dir.y(),dir.x());
            p.setPen(QPen(color,3)); p.drawLine(center,tip);
            p.drawLine(tip,tip-dir*11+normal*6); p.drawLine(tip,tip-dir*11-normal*6);
            p.drawText(tip+QPointF(4,-4),label);
        };
        arrow(basisY(),QColor("#63daca"),"+Y"); arrow(basisX(),QColor("#e6b755"),"+X");
        p.setPen(QColor("#e4ebf4")); p.drawText(16,138,"Arrastrar para rotar");
    }
private:
    void drawRobot(QPainter &p) const {
        p.save();
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
