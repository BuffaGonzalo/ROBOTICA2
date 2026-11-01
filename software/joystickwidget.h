#ifndef JOYSTICKWIDGET_H
#define JOYSTICKWIDGET_H

#include <QApplication>
#include <QWidget>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QSlider>
#include <QSpinBox>
#include <QLineEdit>
#include <QKeyEvent>
#include <QLabel>
#include <QMap>
#include <QPushButton>
#include <QSet>
#include <functional>

class JoystickWidget : public QWidget {
public:
    explicit JoystickWidget(QWidget *parent=nullptr):QWidget(parent) {
        setFocusPolicy(Qt::StrongFocus);
        setMinimumSize(460,500);
        auto *outer=new QVBoxLayout(this);
        auto *panel=new QWidget(this); panel->setMaximumWidth(760); panel->setFocusPolicy(Qt::StrongFocus);
        outer->addWidget(panel,0,Qt::AlignHCenter); outer->addStretch();
        auto *layout=new QGridLayout(panel);
        auto *help=new QLabel("↑ / ↓: avanzar o retroceder.\nSin bloqueo, ← / → giran como un tanque.\nCon F, fijá el eje para moverte lateralmente y en diagonal.\nSoltá las teclas o presioná Espacio para detener.",this);
        help->setWordWrap(true);
        layout->addWidget(help,0,0,1,3);
        const int keys[]={Qt::Key_Up,Qt::Key_Left,Qt::Key_Down,Qt::Key_Right};
        const char *names[]={"Adelante ↑","← Izquierda","Atrás ↓","Derecha →"};
        const int rows[]={1,2,3,2},columns[]={1,0,1,2};
        for(int i=0;i<4;++i) {
            auto *button=new QPushButton(QString::fromUtf8(names[i]),this);
            button->setObjectName(QString("joystickKey_%1").arg(keys[i]));
            button->setFocusPolicy(Qt::NoFocus);
            button->setMinimumHeight(46);
            buttons[keys[i]]=button;
            layout->addWidget(button,rows[i],columns[i]);
            connect(button,&QPushButton::pressed,this,[this,key=keys[i]](){ setFocus(Qt::MouseFocusReason); changeKey(key,true); });
            connect(button,&QPushButton::released,this,[this,key=keys[i]](){ changeKey(key,false); });
        }
        auto *stop=new QPushButton("Parada",this);
        stop->setFocusPolicy(Qt::NoFocus);
        layout->addWidget(stop,2,1);
        connect(stop,&QPushButton::clicked,this,[this](){ reset(); if(stopRequested) stopRequested(); });
        lockButton=new QPushButton(this);
        lockButton->setObjectName("joystickAxisLock");
        lockButton->setCheckable(true);
        lockButton->setFocusPolicy(Qt::NoFocus);
        lockButton->setToolTip("F alterna entre giro tipo tanque y desplazamiento lateral con eje fijo.");
        lockButton->setStyleSheet("QPushButton:checked { background: #338c73; border: 2px solid #70d8c3; }");
        layout->addWidget(lockButton,4,0,1,3);
        connect(lockButton,&QPushButton::pressed,this,[this]() { setFocus(Qt::MouseFocusReason); });
        connect(lockButton,&QPushButton::toggled,this,[this]() { updateMode(); if(!pressed.isEmpty()) publishDirection(); });
        updateMode();
        status=new QLabel(this); status->setWordWrap(true); layout->addWidget(status,5,0,1,3);
        auto *powerGroup=new QGroupBox("Velocidad de los motores",this);
        auto *powerLayout=new QGridLayout(powerGroup);
        speedLabel=new QLabel("Velocidad general: 100 %",this);
        powerLayout->addWidget(speedLabel,0,0,1,4);
        speedSlider=new QSlider(Qt::Horizontal,this);
        speedSlider->setObjectName("joystickSpeed");
        speedSlider->setRange(0,100); speedSlider->setValue(100);
        speedSlider->setFocusPolicy(Qt::NoFocus);
        speedSlider->setToolTip("Porcentaje de los valores PWM de cada motor. 0 % detiene el autito.");
        powerLayout->addWidget(speedSlider,1,0,1,4);
        const char *motorNames[]={"FR","FL","RR","RL"};
        const int defaults[]={90,90,95,105};
        for(int i=0;i<4;++i) {
            auto *label=new QLabel(QString::fromLatin1(motorNames[i])+" · PWM",this);
            powerLayout->addWidget(label,2,i);
            motorPower[i]=new QSpinBox(this);
            motorPower[i]->setObjectName(QString("joystickPower_")+QString::fromLatin1(motorNames[i]).toLower());
            motorPower[i]->setRange(0,255); motorPower[i]->setValue(defaults[i]);
            motorPower[i]->setKeyboardTracking(false);
            motorPower[i]->setToolTip("Potencia PWM del joystick. Enter vuelve al control con flechas.");
            powerLayout->addWidget(motorPower[i],3,i);
            connect(motorPower[i],&QSpinBox::valueChanged,this,[this]() { if(!pressed.isEmpty()) publishDirection(); });
        }
        connect(speedSlider,&QSlider::valueChanged,this,[this](int value) {
            speedLabel->setText(QString("Velocidad general: %1 %").arg(value));
            if(!pressed.isEmpty()) publishDirection();
        });
        connect(speedSlider,&QSlider::sliderPressed,this,[this]() { setFocus(Qt::MouseFocusReason); });
        auto *powerHelp=new QLabel("PWM: potencia de 0 a 255 por motor. El porcentaje general ajusta los cuatro.\nEstos valores pertenecen al joystick; no modifican la potencia de las rutas.",this);
        powerHelp->setWordWrap(true); powerLayout->addWidget(powerHelp,4,0,1,4);
        layout->addWidget(powerGroup,6,0,1,3);
        qApp->installEventFilter(this);
        setControlEnabled(false);
    }
    std::function<void(int,int)> directionChanged;
    std::function<void()> stopRequested;
    bool axisLocked() const { return lockButton->isChecked(); }
    int power(int wheel) const { return motorPower[wheel]->value()*speedSlider->value()/100; }
    void setControlEnabled(bool enabled) {
        ready=enabled;
        for(auto *button:buttons) button->setEnabled(enabled);
        status->setText(enabled ? "Listo · flechas o botones para mover." : "Conectá el robot y finalizá cualquier movimiento o ruta pausada para usar el joystick.");
        if(!enabled) reset();
    }
    void reset() {
        if(pressed.isEmpty()) return;
        pressed.clear();
        for(auto *button:buttons) button->setDown(false);
        if(directionChanged) directionChanged(0,0);
    }
protected:
    bool event(QEvent *event) override {
        if(event->type()==QEvent::Show) lockButton->setChecked(false);
        if(event->type()==QEvent::WindowDeactivate || event->type()==QEvent::Hide || event->type()==QEvent::Close) reset();
        return QWidget::event(event);
    }
    bool eventFilter(QObject *target,QEvent *event) override {
        if(event->type()==QEvent::ApplicationDeactivate || (event->type()==QEvent::WindowDeactivate && target==window())) reset();
        auto *widget=qobject_cast<QWidget *>(target);
        if(!isVisible() || !widget || (widget!=this && !isAncestorOf(widget))) return false;
        if(event->type()==QEvent::FocusIn && (qobject_cast<QAbstractSpinBox *>(widget) || qobject_cast<QLineEdit *>(widget))) reset();
        if(event->type()!=QEvent::KeyPress && event->type()!=QEvent::KeyRelease) return false;
        auto *key=static_cast<QKeyEvent *>(event);
        const bool editing=qobject_cast<QAbstractSpinBox *>(widget) || qobject_cast<QLineEdit *>(widget);
        if(editing && event->type()==QEvent::KeyPress && (key->key()==Qt::Key_Return || key->key()==Qt::Key_Enter)) {
            for(auto *spin:motorPower) spin->interpretText();
            setFocus(Qt::OtherFocusReason); return true;
        }
        if(key->key()==Qt::Key_F) {
            if(event->type()==QEvent::KeyPress && !key->isAutoRepeat()) lockButton->toggle();
            return true;
        }
        if(key->key()==Qt::Key_Space) {
            if(event->type()==QEvent::KeyPress && !key->isAutoRepeat()) {
                reset(); if(stopRequested) stopRequested();
            }
            return true;
        }
        if(!buttons.contains(key->key())) return false;
        if(editing && event->type()==QEvent::KeyPress) return false;
        if(!key->isAutoRepeat()) changeKey(key->key(),event->type()==QEvent::KeyPress);
        return true;
    }
private:
    bool ready=false;
    QSet<int> pressed;
    QMap<int,QPushButton *> buttons;
    QLabel *status=nullptr;
    QPushButton *lockButton=nullptr;
    QSpinBox *motorPower[4]={};
    QSlider *speedSlider=nullptr;
    QLabel *speedLabel=nullptr;
    void updateMode() {
        const bool locked=axisLocked();
        lockButton->setText(locked ? "Eje fijo · F · Activado" : "Eje libre · F · Desactivado");
        buttons[Qt::Key_Left]->setText(locked ? "← Lateral" : "← Girar");
        buttons[Qt::Key_Right]->setText(locked ? "Lateral →" : "Girar →");
    }
    void publishDirection() {
        const int forward=int(pressed.contains(Qt::Key_Up))-int(pressed.contains(Qt::Key_Down));
        const int left=int(pressed.contains(Qt::Key_Left))-int(pressed.contains(Qt::Key_Right));
        if(directionChanged) directionChanged(forward,left);
    }
    void changeKey(int key,bool down) {
        if(!ready || pressed.contains(key)==down) return;
        if(down) pressed.insert(key); else pressed.remove(key);
        buttons[key]->setDown(down);
        publishDirection();
    }
};
#endif
