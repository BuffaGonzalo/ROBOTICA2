#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "joystickwidget.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QIntValidator>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QMessageBox>
#include <QSerialPortInfo>
#include <QTextBrowser>
#include <QTextStream>
#include <QtEndian>
#include <cmath>
#include <cstring>

namespace {
int directionWheelSign(int direction,int wheel) {
    if(direction==2) return wheel==0 || wheel==3 ? 1 : 0;
    if(direction==3) return wheel==1 || wheel==2 ? 1 : 0;
    return direction==1 && (wheel==1 || wheel==2) ? -1 : 1;
}
QString movementName(int direction) {
    return QStringList({"Longitudinal (+Y)","Lateral (−X)","Diagonal izquierda","Diagonal derecha","Vectorial","Traslación y giro","Recta con orientación","Arco circular"}).value(direction,"Dirección desconocida");
}
struct Command {
    quint8 id;
    const char *name;
    const char *description;
};

// Los únicos comandos activos del robot.
const Command commands[] = {
    {0xA0, "ALIVE", "Consulta de estado"},
    {0xA4, "SETPWM", "PWM independiente de las cuatro ruedas"}
};

QByteArray pwmPayload(Ui::MainWindow *ui)
{
    QByteArray payload(1, static_cast<char>(0xA4));
    const int values[] = {ui->spinBox_fr->value(), ui->spinBox_fl->value(),
                          ui->spinBox_rr->value(), ui->spinBox_rl->value()};
    for (int value : values) {
        const quint16 raw = static_cast<quint16>(value);
        payload.append(static_cast<char>(raw & 0xFF));
        payload.append(static_cast<char>(raw >> 8));
    }
    return payload;
}

QString commandLabel(quint8 id)
{
    for (const Command &command : commands) {
        if (command.id == id)
            return QString("%1 (0x%2) · %3").arg(command.name)
                .arg(id, 2, 16, QLatin1Char('0')).arg(QString::fromUtf8(command.description));
    }
    if(id==0xD0) return "MOVE";
    if(id==0xD1) return "STOP";
    if(id==0xD2) return "MOTION_STATUS";
    if(id==0xD4) return "SETGAINS";
    return QString("Comando desconocido (0x%1)").arg(id, 2, 16, QLatin1Char('0'));
}

QString determineLogType(const QString &message, bool processed)
{
    if (message.contains("Error", Qt::CaseInsensitive)) return "ERROR";
    if (message.contains("Aviso", Qt::CaseInsensitive)) return "AVISO";
    if (message.contains(" RX")) return "RX";
    if (message.contains(" TX")) return "TX";
    if (processed) return "PROCESADO";
    return "INFO";
}

QString escapeXml(QString str)
{
    str.replace('&', "&amp;");
    str.replace('<', "&lt;");
    str.replace('>', "&gt;");
    str.replace('"', "&quot;");
    str.replace('\'', "&apos;");
    return str;
}
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      serialPort(new QSerialPort(this)),
      udpSocket(new QUdpSocket(this)),
      aliveTimer(new QTimer(this))
{
    ui->setupUi(this);
    joystick=ui->tab_JOYSTICK;
    joystick->directionChanged=[this](int forward,int left) { driveJoystick(forward,left); };
    joystick->stopRequested=[this]() { ui->pushButton_stop->click(); };
    ui->routePlan->setToolTip("Rueda: zoom. Mantener la rueda presionada y arrastrar: desplazar vista. Izquierdo: añadir o arrastrar puntos. Derecho: eliminar.");
    showTopStatus();
    ui->tabWidget_wifi->setCurrentIndex(0);
    ui->routePlan->changed=[this]() {
        QString status=QString("%1 setpoints · izquierdo añade/arrastra; derecho elimina. Coordenadas en cm.").arg(ui->routePlan->points.size());
        const int selected=ui->routePlan->selectedIndex;
        if(selected>=0 && selected<ui->routePlan->points.size()) {
            const auto point=ui->routePlan->points[selected];
            status+=QString(" Seleccionado: %1 (%2, %3).").arg(selected+1).arg(point.x()).arg(point.y());
            const QPointF start=selected>0 ? ui->routePlan->points[selected-1] : QPointF(0,0);
            const QPointF delta=point-start;
            status+=QString(" Segmento: %1° · %2 cm.").arg(std::atan2(-delta.x(),delta.y())*180/3.141592653589793,0,'f',1).arg(std::hypot(delta.x(),delta.y()),0,'f',1);
            if(ui->routePlan->oriented) status+=QString(" Ángulo de llegada: %1° · arrastrá la flecha o doble clic para editar.").arg(ui->routePlan->headingAt(selected),0,'f',1);
            if(ui->routePlan->arcAt(selected)!=0) status+=QString(" Arco de llegada: %1°.").arg(ui->routePlan->arcAt(selected),0,'f',1);
        }
        else if(ui->routePlan->oriented) status+=" Orientación activa: arrastrá las flechas de llegada o hacé doble clic en un punto.";
        if(ui->routePlan->arcMode) status+=" Arco activo: clic en vacío agrega un arco de 180°; clic en un setpoint edita sus datos.";
        ui->label_routeStatus->setText(status);
        updateCommandUi();
    };
    connect(ui->tabWidget_wifi,&QTabWidget::currentChanged,this,[this]() {
        if(ui->tabWidget_wifi->currentWidget()!=joystick) joystick->reset();
        else joystick->setFocus(Qt::OtherFocusReason);
        updateCommandUi();
    });
    connect(ui->pushButton_runRoute,&QPushButton::clicked,this,&MainWindow::startRoute);
    connect(ui->pushButton_reverseRoute,&QPushButton::clicked,this,&MainWindow::startReverseRoute);
    connect(ui->pushButton_resumeRoute,&QPushButton::clicked,this,&MainWindow::resumeRoute);
    ui->pushButton_routeOrientation->setStyleSheet("QPushButton:checked { background: #338c73; border: 2px solid #70d8c3; }");
    connect(ui->pushButton_routeOrientation,&QPushButton::toggled,this,[this](bool enabled) {
        ui->routePlan->oriented=enabled; ui->routePlan->update(); ui->routePlan->changed();
    });
    ui->pushButton_routeArc->setStyleSheet("QPushButton:checked { background: #338c73; border: 2px solid #70d8c3; }");
    connect(ui->pushButton_routeArc,&QPushButton::toggled,this,[this](bool enabled) {
        ui->routePlan->arcMode=enabled; ui->routePlan->update(); ui->routePlan->changed();
    });
    connect(ui->pushButton_importRoute,&QPushButton::clicked,this,[this]() {
        const QString path=QFileDialog::getOpenFileName(this,"Importar ruta",QString(),"Rutas JSON (*.json)");
        if(path.isEmpty()) return;
        QString error;
        if(!importRouteFile(path,error)) QMessageBox::warning(this,"No se pudo importar la ruta",error);
    });
    connect(ui->pushButton_exportRoute,&QPushButton::clicked,this,[this]() {
        QString path=QFileDialog::getSaveFileName(this,"Exportar ruta","ruta.json","Rutas JSON (*.json)");
        if(path.isEmpty()) return;
        if(!path.endsWith(".json",Qt::CaseInsensitive)) path+=".json";
        QString error;
        if(!exportRouteFile(path,error)) QMessageBox::warning(this,"No se pudo exportar la ruta",error);
        else ui->label_routeStatus->setText("Ruta exportada: "+QDir::toNativeSeparators(path));
    });
    connect(ui->pushButton_clearRoute,&QPushButton::clicked,this,[this]() {
        routePaused=resumeReady=false;
        ui->routePlan->points.clear(); ui->routePlan->headings.clear(); ui->routePlan->arcSweeps.clear(); ui->routePlan->selectedIndex=-1; ui->routePlan->completed=0; ui->routePlan->update(); ui->routePlan->changed();
    });
    connect(ui->pushButton_resetMap,&QPushButton::clicked,this,[this]() {
        routePaused=resumeReady=false;
        mapTracking=false; mapReset=true; ui->routePlan->trace.clear(); ui->routePlan->setRobot({0,0});
        mapOrigin=plannedPosition=plannedLegEnd=QPointF(0,0); lastDistanceCm=0;
        ui->routePlan->robotHeadingDeg=plannedHeadingDeg=plannedLegHeadingDeg=mapHeadingDeg=0;
        ui->routePlan->viewOffset={};
        haveDistance=true; showTopStatus("Esperando telemetría");
        ui->routePlan->completed=0; ui->label_routeStatus->setText("Reset: posición y recorrido en (0, 0). Ruta conservada. No se movió físicamente el robot.");
        updateCommandUi();
    });
    for(QSpinBox *spin : {ui->spinBox_fr,ui->spinBox_fl,ui->spinBox_rr,ui->spinBox_rl}) {
        spin->setRange(-255,255);
        spin->setToolTip("PWM de prueba: −255 a +255. Negativo invierte el giro; cero frena. Escribí el valor con su signo.");
    }
    ui->lineEdit_local_port->setValidator(new QIntValidator(1, 65535, this));
    ui->lineEdit_device_port->setValidator(new QIntValidator(1, 65535, this));
    ui->textBrowserUnProcessed->document()->setMaximumBlockCount(500);
    ui->textBrowserProcessed->document()->setMaximumBlockCount(500);
    auto *pwmTimer = new QTimer(this);
    pwmTimer->setObjectName("pwmRefreshTimer");
    pwmTimer->setInterval(250);
    auto *stopTimer=new QTimer(this);
    auto *pidAckTimer=new QTimer(this);
    pidAckTimer->setObjectName("pidAckTimer");
    pidAckTimer->setSingleShot(true);
    connect(pidAckTimer,&QTimer::timeout,this,[this]() {
        pidPending=false;
        ui->label_pidStatus->setText("Sin confirmación del firmware: no se puede confirmar que los valores estén aplicados.");
        updateCommandUi();
    });
    connect(ui->pushButton_applyPid,&QPushButton::clicked,this,[this,pidAckTimer]() {
        ui->doubleSpinBox_kpPosition->interpretText();
        ui->doubleSpinBox_kpSpeed->interpretText();
        ui->doubleSpinBox_kiSpeed->interpretText();
        ui->doubleSpinBox_kpSync->interpretText();
        const float gains[]={(float)ui->doubleSpinBox_kpPosition->value(),(float)ui->doubleSpinBox_kpSpeed->value(),
                             (float)ui->doubleSpinBox_kiSpeed->value(),(float)ui->doubleSpinBox_kpSync->value()};
        QByteArray payload(17,'\0'); payload[0]=static_cast<char>(0xD4);
        static_assert(sizeof(float)==4,"SETGAINS requires float32");
        for(int i=0;i<4;++i) { quint32 raw; std::memcpy(&raw,&gains[i],4); qToLittleEndian<quint32>(raw,payload.data()+1+i*4); }
        if(!sendPayload(payload)) { ui->label_pidStatus->setText("No se pudo enviar la configuración PID."); return; }
        pidPending=true; pidAckTimer->start(1500);
        ui->label_pidStatus->setText("PID enviado; esperando ACK del firmware.");
        updateCommandUi();
    });
    stopTimer->setObjectName("stopRetryTimer");
    stopTimer->setInterval(100);
    auto *routeWatchdog=new QTimer(this);
    routePauseTimer=new QTimer(this);
    routePauseTimer->setSingleShot(true);
    routePauseTimer->setInterval(500);
    connect(routePauseTimer,&QTimer::timeout,this,[this]() { if(routeActive) nextRouteLeg(); });
    routeWatchdog->setInterval(100);
    connect(routeWatchdog,&QTimer::timeout,this,[this]() {
        if(routeActive && !routePauseTimer->isActive() && QDateTime::currentMSecsSinceEpoch()-routeLastSampleMs>2000) {
            cancelRoute("Ruta cancelada: no se confirmó el movimiento o faltan muestras de telemetría.");
            ui->pushButton_stop->click();
        }
    });
    routeWatchdog->start();
    connect(stopTimer,&QTimer::timeout,this,[this]() {
        sendPayload(QByteArray(1,static_cast<char>(0xD1)));
    });
    connect(pwmTimer, &QTimer::timeout, this, [this, pwmTimer]() {
        if (!sendPayload(joystickDriving ? joystickPayload : pwmPayload(ui))) {
            pwmTimer->stop();
            if(joystickDriving) ui->pushButton_stop->click();
            updateCommandUi();
            log("No se pudo renovar PWM; el robot se detiene por timeout.", true);
        }
    });
    connect(ui->pushButton_stop, &QPushButton::clicked, this, [this, pwmTimer, stopTimer]() {
        joystickDriving=false;
        joystick->reset();
        showTopStatus("Parada de emergencia");
        const bool pause=routeActive;
        const bool between=routePauseTimer->isActive();
        cancelRoute("Ruta cancelada por Parada.");
        if(pause) {
            routePaused=true; resumeReady=false; pausedBetweenLegs=between;
            ui->label_routeStatus->setText("Ruta pausada · esperando confirmación de parada y posición del robot para continuar.");
        }
        if(routePaused) resumeReady=false;
        pwmTimer->stop();
        stopPending=true;
        stopTimer->start();
        ui->spinBox_fr->setValue(0);
        ui->spinBox_fl->setValue(0);
        ui->spinBox_rr->setValue(0);
        ui->spinBox_rl->setValue(0);
        if (!sendPayload(QByteArray(1, static_cast<char>(0xD1))))
            log("No se pudo enviar la parada; se aplica el timeout del firmware.", true);
        updateCommandUi();
    });
    connect(ui->pushButton_exportTxt, &QPushButton::clicked, this, &MainWindow::exportLogTxt);
    connect(ui->pushButton_exportXls, &QPushButton::clicked, this, &MainWindow::exportLogXls);
    auto refreshMovePreview=[this]() {
        const int mode=ui->comboBox_moveDirection->currentIndex();
        const bool lateral=mode==1;
        const int cm=ui->spinBox_distance->value();
        ui->label_distance->setText(lateral ? "Distancia en cm (+ izquierda / − derecha)"
                                           : "Distancia en cm (+ avance / − retroceso)");
        if(mode>=2) ui->label_distance->setText("Recorrido diagonal en cm (+ adelante / − atrás)");
        const QString direction=cm==0 ? "Sin desplazamiento" : lateral
            ? (cm>0 ? "Izquierda" : "Derecha") : (cm>0 ? "Adelante" : "Atrás");
        ui->label_movePreview->setText(QString("%1 · %2 · %3 cm")
            .arg(movementName(mode),direction).arg(std::abs(cm)));
        QStringList signs;
        const int powers[]={ui->spinBox_move_fr->value(),ui->spinBox_move_fl->value(),ui->spinBox_move_rr->value(),ui->spinBox_move_rl->value()};
        for(int i=0;i<4;++i) {
            const int sign=(cm>0 ? 1 : cm<0 ? -1 : 0)*directionWheelSign(mode,i)*(powers[i]>0 ? 1 : 0);
            signs << (sign>0 ? "+" : sign<0 ? "−" : "0");
        }
        ui->label_moveWheelPattern->setText("Sentidos FR / FL / RR / RL: "+signs.join(" / "));
        ui->label_lateralHelp->setVisible(mode!=0);
        ui->label_lateralHelp->setText(mode>=2
            ? "Diagonal de 45°: usa FR/RL; el otro par queda libre. Distancia positiva avanza y negativa recorre la misma línea en sentido contrario. El recorrido por encoders no mide deslizamiento."
            : "Lateral: + izquierda / − derecha. El recorrido por encoders no mide deslizamiento.");
        ui->pushButton_move->setText(mode>=2 ? "MOVER DIAGONAL" : lateral ? "MOVER LATERAL" : "MOVER DISTANCIA");
        updateCommandUi();
    };
    connect(ui->comboBox_moveDirection,QOverload<int>::of(&QComboBox::currentIndexChanged),this,refreshMovePreview);
    connect(ui->spinBox_distance,&QSpinBox::valueChanged,this,refreshMovePreview);
    for(QSpinBox *spin : {ui->spinBox_move_fr,ui->spinBox_move_fl,ui->spinBox_move_rr,ui->spinBox_move_rl}) {
        spin->setRange(0,255);
        spin->setToolTip("Límite de potencia: 0 a 255 PWM. El sentido lo define la cinemática según Dirección y el signo de la distancia.");
        connect(spin,&QSpinBox::valueChanged,this,refreshMovePreview);
    }
    refreshMovePreview();
    connect(ui->pushButton_move, &QPushButton::clicked, this, [this, pwmTimer]() {
        ui->spinBox_distance->interpretText();
        for(QSpinBox *spin : {ui->spinBox_move_fr,ui->spinBox_move_fl,ui->spinBox_move_rr,ui->spinBox_move_rl}) spin->interpretText();
        if(!ui->spinBox_distance->value()) return;
        pwmTimer->stop();
        const int mode=ui->comboBox_moveDirection->currentIndex();
        // La extension lateral agrega eje=1. Firmware antiguo la rechaza
        // por longitud en lugar de ejecutar accidentalmente un avance.
        QByteArray payload(mode!=0 ? 10 : 9, '\0');
        payload[0]=static_cast<char>(0xD0);
        const qint32 um=static_cast<qint32>(ui->spinBox_distance->value())*10000;
        qToLittleEndian<qint32>(um,payload.data()+1);
        const int limits[]={ui->spinBox_move_fr->value(),ui->spinBox_move_fl->value(),ui->spinBox_move_rr->value(),ui->spinBox_move_rl->value()};
        for(int i=0;i<4;++i) payload[5+i]=static_cast<char>(limits[i]);
        if(mode!=0) payload[9]=static_cast<char>(mode);
        if(!sendPayload(payload)) {
            log("No se pudo enviar MOVE.",true);
            return;
        }
        armMapMovement(mode);
        showTopStatus("Desplazamiento");
        motionRunning=true;
        log(QString("MOVE: %1 cm; dirección %6; PWM FR/FL/RR/RL: %2/%3/%4/%5.").arg(ui->spinBox_distance->value()).arg(limits[0]).arg(limits[1]).arg(limits[2]).arg(limits[3]).arg(movementName(mode)),true);
        updateCommandUi();
    });
    refreshSerialPorts();

    connect(ui->comboBox_protocol, &QComboBox::currentIndexChanged, this, &MainWindow::updateConnectionUi);
    connect(ui->pushButton_refreshPorts, &QPushButton::clicked, this, &MainWindow::refreshSerialPorts);
    connect(ui->pushButton_connect, &QPushButton::clicked, this, &MainWindow::toggleConnection);
    connect(ui->pushButton_sendCommand, &QPushButton::clicked, this, &MainWindow::sendSelectedCommand);
    connect(ui->lineEdit_device_ip, &QLineEdit::textChanged, this, &MainWindow::updateConnectionUi);
    connect(ui->lineEdit_device_port, &QLineEdit::textChanged, this, &MainWindow::updateConnectionUi);
    connect(ui->pushButton_clearLogs, &QPushButton::clicked, this, [this]() {
        ui->textBrowserUnProcessed->clear();
        ui->textBrowserProcessed->clear();
        logHistory.clear();
    });
    connect(serialPort, &QSerialPort::readyRead, this, &MainWindow::receiveSerial);
    connect(serialPort, &QSerialPort::bytesWritten, this, [this](qint64 count) {
        log(QString("Serial: %1 byte(s) entregados al controlador.").arg(count));
    });
    connect(serialPort, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error) {
        if (error != QSerialPort::NoError)
            log("Error Serial: " + serialPort->errorString(), true);
    });
    connect(udpSocket, &QUdpSocket::readyRead, this, &MainWindow::receiveUdp);
    aliveTimer->setInterval(1000);
    connect(aliveTimer, &QTimer::timeout, this, &MainWindow::sendAlive);
    connect(aliveTimer,&QTimer::timeout,this,[this]() {
        if(lastMotionTelemetryMs && QDateTime::currentMSecsSinceEpoch()-lastMotionTelemetryMs>1500) {
            if(topEvent!="Parada de emergencia" && topEvent!="Ruta terminada") showTopStatus("Esperando telemetría");
        }
    });
    aliveTimer->start();
    updateConnectionUi();
}

MainWindow::~MainWindow()
{
    findChild<QTimer *>("pwmRefreshTimer")->stop();
    if (serialPort->isOpen() || udpSocket->state() == QAbstractSocket::BoundState) {
        ui->pushButton_stop->click();
        if (serialPort->isOpen()) serialPort->waitForBytesWritten(100);
    }
    delete ui;
}

void MainWindow::cleanOldLogs()
{
    const QDateTime cutoff = QDateTime::currentDateTime().addSecs(-300);
    while (!logHistory.isEmpty() && logHistory.first().timestamp < cutoff) {
        logHistory.removeFirst();
    }
}

void MainWindow::log(const QString &message, bool processed)
{
    const QDateTime now = QDateTime::currentDateTime();
    const QString line = now.toString("hh:mm:ss.zzz  ") + message;
    (processed ? ui->textBrowserProcessed : ui->textBrowserUnProcessed)->append(line.toHtmlEscaped());

    LogEntry entry;
    entry.timestamp = now;
    entry.type = determineLogType(message, processed);
    entry.message = message;
    logHistory.append(entry);

    cleanOldLogs();
}

void MainWindow::exportLogTxt()
{
    cleanOldLogs();

    const QString folder = QFileDialog::getExistingDirectory(
        this,
        tr("Seleccionar carpeta para exportar"),
        QDir::homePath(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
    );

    if (folder.isEmpty())
        return;

    const QDateTime now = QDateTime::currentDateTime();
    const QString fileName = QString("log_comunicacion_%1.txt").arg(now.toString("yyyyMMdd_HHmmss"));
    const QString filePath = QDir(folder).filePath(fileName);

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, tr("Error al exportar"), tr("No se pudo crear el archivo:\n%1").arg(filePath));
        return;
    }

    QTextStream out(&file);
    out << "======================================================================\n";
    out << "           REGISTRO DE COMUNICACION - ROBOT MECANUM\n";
    out << "           Generado: " << now.toString("dd/MM/yyyy HH:mm:ss") << "\n";
    out << "           Ventana temporal: Ultimos 5 minutos de ejecucion\n";
    out << "           Total de registros: " << logHistory.size() << "\n";
    out << "======================================================================\n\n";

    out << QString("%1 | %2 | %3\n")
               .arg("HORA", -12)
               .arg("TIPO", -10)
               .arg("MENSAJE");
    out << QString("%1-+-%2-+-%3\n")
               .arg(QString(12, '-'))
               .arg(QString(10, '-'))
               .arg(QString(50, '-'));

    for (const LogEntry &entry : logHistory) {
        out << QString("%1 | %2 | %3\n")
                   .arg(entry.timestamp.toString("hh:mm:ss.zzz"), -12)
                   .arg(entry.type, -10)
                   .arg(entry.message);
    }

    out << "\n======================================================================\n";
    out << "                       FIN DEL REGISTRO\n";
    out << "======================================================================\n";

    file.close();

    QMessageBox::information(this, tr("Exportacion exitosa"),
        tr("Se exportaron %1 registros de los ultimos 5 minutos a:\n%2")
            .arg(logHistory.size())
            .arg(QDir::toNativeSeparators(filePath)));
}

void MainWindow::exportLogXls()
{
    cleanOldLogs();

    const QString folder = QFileDialog::getExistingDirectory(
        this,
        tr("Seleccionar carpeta para exportar"),
        QDir::homePath(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
    );

    if (folder.isEmpty())
        return;

    const QDateTime now = QDateTime::currentDateTime();
    const QString fileName = QString("log_comunicacion_%1.xls").arg(now.toString("yyyyMMdd_HHmmss"));
    const QString filePath = QDir(folder).filePath(fileName);

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, tr("Error al exportar"), tr("No se pudo crear el archivo:\n%1").arg(filePath));
        return;
    }

    QTextStream out(&file);
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<?mso-application progid=\"Excel.Sheet\"?>\n";
    out << "<Workbook xmlns=\"urn:schemas-microsoft-com:office:spreadsheet\"\n";
    out << " xmlns:o=\"urn:schemas-microsoft-com:office:office\"\n";
    out << " xmlns:x=\"urn:schemas-microsoft-com:office:excel\"\n";
    out << " xmlns:ss=\"urn:schemas-microsoft-com:office:spreadsheet\"\n";
    out << " xmlns:html=\"http://www.w3.org/TR/REC-html40\">\n";
    out << " <Styles>\n";
    out << "  <Style ss:ID=\"Default\" ss:Name=\"Normal\">\n";
    out << "   <Alignment ss:Vertical=\"Center\"/>\n";
    out << "   <Font ss:FontName=\"Segoe UI\" ss:Size=\"10\" ss:Color=\"#000000\"/>\n";
    out << "  </Style>\n";
    out << "  <Style ss:ID=\"TitleStyle\">\n";
    out << "   <Font ss:FontName=\"Segoe UI\" ss:Size=\"13\" ss:Bold=\"1\" ss:Color=\"#176979\"/>\n";
    out << "  </Style>\n";
    out << "  <Style ss:ID=\"MetaStyle\">\n";
    out << "   <Font ss:FontName=\"Segoe UI\" ss:Size=\"9\" ss:Italic=\"1\" ss:Color=\"#555555\"/>\n";
    out << "  </Style>\n";
    out << "  <Style ss:ID=\"HeaderStyle\">\n";
    out << "   <Font ss:FontName=\"Segoe UI\" ss:Size=\"10\" ss:Bold=\"1\" ss:Color=\"#FFFFFF\"/>\n";
    out << "   <Interior ss:Color=\"#176979\" ss:Pattern=\"Solid\"/>\n";
    out << "   <Alignment ss:Horizontal=\"Center\" ss:Vertical=\"Center\"/>\n";
    out << "   <Borders>\n";
    out << "    <Border ss:Position=\"Bottom\" ss:LineStyle=\"Continuous\" ss:Weight=\"1\" ss:Color=\"#0D3F49\"/>\n";
    out << "   </Borders>\n";
    out << "  </Style>\n";
    out << "  <Style ss:ID=\"CenterStyle\">\n";
    out << "   <Alignment ss:Horizontal=\"Center\" ss:Vertical=\"Center\"/>\n";
    out << "  </Style>\n";
    out << "  <Style ss:ID=\"TypeRX\">\n";
    out << "   <Alignment ss:Horizontal=\"Center\" ss:Vertical=\"Center\"/>\n";
    out << "   <Font ss:FontName=\"Segoe UI\" ss:Size=\"10\" ss:Bold=\"1\" ss:Color=\"#0284C7\"/>\n";
    out << "  </Style>\n";
    out << "  <Style ss:ID=\"TypeTX\">\n";
    out << "   <Alignment ss:Horizontal=\"Center\" ss:Vertical=\"Center\"/>\n";
    out << "   <Font ss:FontName=\"Segoe UI\" ss:Size=\"10\" ss:Bold=\"1\" ss:Color=\"#059669\"/>\n";
    out << "  </Style>\n";
    out << "  <Style ss:ID=\"TypeError\">\n";
    out << "   <Alignment ss:Horizontal=\"Center\" ss:Vertical=\"Center\"/>\n";
    out << "   <Font ss:FontName=\"Segoe UI\" ss:Size=\"10\" ss:Bold=\"1\" ss:Color=\"#DC2626\"/>\n";
    out << "  </Style>\n";
    out << " </Styles>\n";
    out << " <Worksheet ss:Name=\"Log_Comunicacion\">\n";
    out << "  <Table>\n";
    out << "   <Column ss:Width=\"110\"/>\n";
    out << "   <Column ss:Width=\"80\"/>\n";
    out << "   <Column ss:Width=\"550\"/>\n";
    out << "   <Row>\n";
    out << "    <Cell ss:StyleID=\"TitleStyle\"><Data ss:Type=\"String\">Registro de Comunicacion - Robot Mecanum</Data></Cell>\n";
    out << "   </Row>\n";
    out << "   <Row>\n";
    out << QString("    <Cell ss:StyleID=\"MetaStyle\"><Data ss:Type=\"String\">Exportado: %1 | Ventana: Ultimos 5 minutos de ejecucion | Registros: %2</Data></Cell>\n")
               .arg(now.toString("dd/MM/yyyy HH:mm:ss"))
               .arg(logHistory.size());
    out << "   </Row>\n";
    out << "   <Row ss:Index=\"4\" ss:StyleID=\"HeaderStyle\">\n";
    out << "    <Cell><Data ss:Type=\"String\">Fecha y Hora</Data></Cell>\n";
    out << "    <Cell><Data ss:Type=\"String\">Tipo</Data></Cell>\n";
    out << "    <Cell><Data ss:Type=\"String\">Mensaje</Data></Cell>\n";
    out << "   </Row>\n";

    for (const LogEntry &entry : logHistory) {
        QString typeStyle = "CenterStyle";
        if (entry.type == "RX") typeStyle = "TypeRX";
        else if (entry.type == "TX") typeStyle = "TypeTX";
        else if (entry.type == "ERROR") typeStyle = "TypeError";

        out << "   <Row>\n";
        out << QString("    <Cell ss:StyleID=\"CenterStyle\"><Data ss:Type=\"String\">%1</Data></Cell>\n")
                   .arg(entry.timestamp.toString("hh:mm:ss.zzz"));
        out << QString("    <Cell ss:StyleID=\"%1\"><Data ss:Type=\"String\">%2</Data></Cell>\n")
                   .arg(typeStyle)
                   .arg(escapeXml(entry.type));
        out << QString("    <Cell><Data ss:Type=\"String\">%1</Data></Cell>\n")
                   .arg(escapeXml(entry.message));
        out << "   </Row>\n";
    }

    out << "  </Table>\n";
    out << " </Worksheet>\n";
    out << "</Workbook>\n";

    file.close();

    QMessageBox::information(this, tr("Exportacion exitosa"),
        tr("Se exportaron %1 registros de los ultimos 5 minutos a:\n%2")
            .arg(logHistory.size())
            .arg(QDir::toNativeSeparators(filePath)));
}

void MainWindow::refreshSerialPorts()
{
    const QString selected = ui->comboBox_serialPort->currentData().toString();
    ui->comboBox_serialPort->clear();
    for (const QSerialPortInfo &port : QSerialPortInfo::availablePorts()) {
        const QString text = port.portName() + (port.description().isEmpty() ? QString() : " · " + port.description());
        ui->comboBox_serialPort->addItem(text, port.portName());
    }
    const int previous = ui->comboBox_serialPort->findData(selected);
    if (previous >= 0) ui->comboBox_serialPort->setCurrentIndex(previous);
    updateConnectionUi();
}

bool MainWindow::udpPeer(QHostAddress &address, quint16 &port) const
{
    bool validPort = false;
    const uint number = ui->lineEdit_device_port->text().toUInt(&validPort);
    port = static_cast<quint16>(number);
    return address.setAddress(ui->lineEdit_device_ip->text().trimmed())
           && validPort && number > 0 && number <= 65535;
}

void MainWindow::showTopStatus(const QString &event)
{
    if(!event.isEmpty()) topEvent=event;
    ui->label_topDistance->setText(QString("%1 · Recorrido: %2 cm").arg(topEvent,haveDistance ? QString::number(lastDistanceCm,'f',2) : QStringLiteral("—")));
}

void MainWindow::armMapMovement(int mode)
{
    if(!routeActive) routePaused=resumeReady=false;
    mapReset=false;
    mapOrigin=ui->routePlan->robot; mapHeadingDeg=ui->routePlan->robotHeadingDeg; mapMode=mode; mapTracking=true;
    mapArcProgress=0;
}

void MainWindow::cancelRoute(const QString &reason)
{
    if(!routeActive) return;
    routePauseTimer->stop();
    routeActive=false; routeAck=false; routeSeenRunning=false;
    ui->label_routeStatus->setText(reason); log(reason,true); updateCommandUi();
}

bool MainWindow::exportRouteFile(const QString &path, QString &error) const
{
    if(ui->routePlan->points.isEmpty()) { error="La ruta no tiene puntos para exportar."; return false; }
    QJsonArray points;
    for(int i=0;i<ui->routePlan->points.size();++i) {
        const auto point=ui->routePlan->points[i];
        points.append(QJsonObject{{"x",point.x()},{"y",point.y()},{"headingDeg",ui->routePlan->headingAt(i)},{"arcSweepDeg",ui->routePlan->arcAt(i)}});
    }
    const QJsonDocument document(QJsonObject{{"format","robotica2-route"},{"version",3},{"units","cm"},
        {"axes","x-right-y-forward"},{"orientationEnabled",ui->routePlan->oriented},
        {"arcMode",ui->routePlan->arcMode},
        {"frameRotationDeg",ui->routePlan->frameRotationDeg},{"points",points}});
    QSaveFile file(path);
    if(!file.open(QIODevice::WriteOnly)) { error=file.errorString(); return false; }
    const QByteArray data=document.toJson(QJsonDocument::Indented);
    if(file.write(data)!=data.size() || !file.commit()) { error=file.errorString(); return false; }
    return true;
}

bool MainWindow::importRouteFile(const QString &path, QString &error)
{
    if(routeActive || routePaused || motionRunning || stopPending || pidPending || findChild<QTimer *>("pwmRefreshTimer")->isActive()) {
        error="Detené el movimiento y descartá la ruta pausada antes de importar."; return false;
    }
    QFile file(path);
    if(!file.open(QIODevice::ReadOnly)) { error=file.errorString(); return false; }
    if(file.size()>1024*1024) { error="El archivo de ruta supera 1 MB."; return false; }
    QJsonParseError parseError;
    const QJsonDocument document=QJsonDocument::fromJson(file.readAll(),&parseError);
    if(parseError.error!=QJsonParseError::NoError) { error="JSON inválido: "+parseError.errorString(); return false; }
    const QJsonObject object=document.object();
    const bool version3=object.value("version")==QJsonValue(3);
    const bool version2=version3 || object.value("version")==QJsonValue(2);
    if(!document.isObject() || object.value("format")!="robotica2-route" || (!version2 && object.value("version")!=QJsonValue(1))
        || object.value("units")!="cm" || object.value("axes")!="x-right-y-forward" || !object.value("points").isArray()) {
        error="Formato de ruta no compatible. Se requiere robotica2-route versión 1, 2 o 3, en cm, con ejes +X derecha y +Y avance."; return false;
    }
    const QJsonArray entries=object.value("points").toArray();
    if(entries.isEmpty() || entries.size()>100) { error="La ruta debe contener entre 1 y 100 puntos."; return false; }
    QList<QPointF> points;
    QList<double> headings;
    QList<double> arcSweeps;
    if(version3 && !object.value("arcMode").isBool()) { error="arcMode debe ser verdadero o falso."; return false; }
    if(version2 && !object.value("orientationEnabled").isBool()) { error="orientationEnabled debe ser verdadero o falso."; return false; }
    for(const QJsonValue &entry:entries) {
        const QJsonObject point=entry.toObject();
        const QJsonValue x=point.value("x"),y=point.value("y");
        if(!entry.isObject() || !x.isDouble() || !y.isDouble() || !std::isfinite(x.toDouble()) || !std::isfinite(y.toDouble())
            || std::abs(x.toDouble())>1000 || std::abs(y.toDouble())>1000
            || std::round(x.toDouble())!=x.toDouble() || std::round(y.toDouble())!=y.toDouble()) {
            error=QString("Punto %1 inválido: X e Y deben ser enteros entre −1000 y 1000 cm.").arg(points.size()+1); return false;
        }
        points.append(QPointF(x.toDouble(),y.toDouble()));
        const QJsonValue heading=point.value("headingDeg");
        if(version2 && (!heading.isDouble() || !std::isfinite(heading.toDouble()) || std::abs(heading.toDouble())>180)) {
            error=QString("Orientación inválida en el punto %1: debe estar entre −180° y 180°.").arg(points.size()); return false;
        }
        headings.append(version2 ? heading.toDouble() : 0);
        const QJsonValue sweep=point.value("arcSweepDeg");
        if(version3 && (!sweep.isDouble() || !std::isfinite(sweep.toDouble()) || (sweep.toDouble()!=0 && (std::abs(sweep.toDouble())<1 || std::abs(sweep.toDouble())>359)))) {
            error=QString("Arco inválido en el punto %1: usar 0 para recta o de ±1° a ±359° para arco.").arg(points.size()); return false;
        }
        arcSweeps.append(version3 ? sweep.toDouble() : 0);
        if(arcSweeps.last()!=0 && points.last()==(points.size()>1 ? points[points.size()-2] : QPointF())) {
            error="Los extremos de un arco deben ser distintos."; return false;
        }
    }
    const QJsonValue rotation=object.value("frameRotationDeg");
    if(!rotation.isUndefined() && (!rotation.isDouble() || !std::isfinite(rotation.toDouble()))) {
        error="La orientación del plano debe ser un número válido en grados."; return false;
    }
    ui->routePlan->points=points;
    ui->routePlan->headings=headings;
    ui->routePlan->arcSweeps=arcSweeps;
    ui->pushButton_routeOrientation->setChecked(version2 && object.value("orientationEnabled").toBool());
    ui->pushButton_routeArc->setChecked(version3 && object.value("arcMode").toBool());
    ui->routePlan->frameRotationDeg=std::remainder(rotation.toDouble(),360.0);
    ui->routePlan->selectedIndex=-1;
    ui->routePlan->completed=0;
    ui->routePlan->update();
    ui->routePlan->changed();
    ui->label_routeStatus->setText(QString("Ruta importada: %1 puntos · %2").arg(points.size()).arg(QDir::toNativeSeparators(path)));
    return true;
}

void MainWindow::startRoute()
{
    if(routeActive || motionRunning || stopPending || pidPending || ui->routePlan->points.isEmpty()) return;
    reverseRoute=false; executionTargets=ui->routePlan->points;
    routeOriented=ui->routePlan->oriented;
    executionHeadings.clear(); for(int i=0;i<executionTargets.size();++i) executionHeadings.append(ui->routePlan->headingAt(i));
    executionArcSweeps.clear(); for(int i=0;i<executionTargets.size();++i) executionArcSweeps.append(ui->routePlan->arcAt(i));
    plannedHeadingDeg=ui->routePlan->robotHeadingDeg;
    routePaused=resumeReady=false;
    plannedPosition=ui->routePlan->robot;
    routeActive=true; routeIndex=0; ui->routePlan->completed=0;
    nextRouteLeg();
}

void MainWindow::startReverseRoute()
{
    if(routeActive || motionRunning || stopPending || pidPending || ui->routePlan->points.isEmpty()) return;
    // La inversa parte del extremo final de la ruta; no corrige su posición.
    executionTargets.clear();
    executionHeadings.clear(); routeOriented=ui->routePlan->oriented;
    executionArcSweeps.clear(); for(int i=ui->routePlan->points.size()-1;i>=0;--i) executionArcSweeps.append(-ui->routePlan->arcAt(i));
    routePaused=resumeReady=false;
    for(int i=ui->routePlan->points.size()-2;i>=0;--i) { executionTargets.append(ui->routePlan->points[i]); executionHeadings.append(ui->routePlan->headingAt(i)); }
    executionTargets.append(QPointF(0,0));
    executionHeadings.append(0); plannedHeadingDeg=ui->routePlan->robotHeadingDeg;
    plannedPosition=ui->routePlan->points.last();
    reverseRoute=true; routeActive=true; routeIndex=0; ui->routePlan->completed=0;
    nextRouteLeg();
}

void MainWindow::resumeRoute()
{
    if(!routePaused || !resumeReady || routeActive || motionRunning || stopPending || pidPending) return;
    if(!lastMotionTelemetryMs || QDateTime::currentMSecsSinceEpoch()-lastMotionTelemetryMs>1500) {
        ui->label_routeStatus->setText("Esperando una posición reciente del robot antes de continuar."); return;
    }
    // En mitad de un tramo, recalcular solo lo pendiente desde la posición
    // detenida. Entre tramos conservar el avance nominal ya confirmado.
    if(!pausedBetweenLegs) {
        if(mapMode==7 && routeIndex<executionArcSweeps.size()) {
            executionArcSweeps[routeIndex]*=1-mapArcProgress;
            if(std::abs(executionArcSweeps[routeIndex])<1) executionArcSweeps[routeIndex]=0;
        }
        plannedPosition=ui->routePlan->robot; plannedHeadingDeg=ui->routePlan->robotHeadingDeg;
    }
    routePaused=resumeReady=false; routeActive=true;
    nextRouteLeg();
}

void MainWindow::nextRouteLeg()
{
    if(!routeActive) return;
    QPointF delta;
    // Avanzar por el plan nominal. ARRIVED del firmware cierra cada tramo;
    // no crear micro-MOVE para corregir el residuo de la odometría de Qt.
    while(routeIndex<executionTargets.size()) {
        delta=executionTargets[routeIndex]-plannedPosition;
        if((routeIndex<executionArcSweeps.size() && executionArcSweeps[routeIndex]!=0) || std::abs(delta.x())>0.05 || std::abs(delta.y())>0.05 || (routeOriented && std::abs(std::remainder(executionHeadings[routeIndex]-plannedHeadingDeg,360.0))>0.1)) break;
        ui->routePlan->completed=++routeIndex; ui->routePlan->update();
    }
    if(routeIndex==executionTargets.size()) {
        showTopStatus("Ruta terminada");
        cancelRoute(reverseRoute ? "Ruta inversa finalizada." : "Ruta finalizada."); return;
    }
    const double heading=plannedHeadingDeg*PoseKinematics::POSE_PI/180;
    PoseKinematics::Motion leg={delta.y()*std::cos(heading)-delta.x()*std::sin(heading),
        -delta.x()*std::cos(heading)-delta.y()*std::sin(heading),
        routeOriented ? PoseKinematics::shortest((executionHeadings[routeIndex]-plannedHeadingDeg)*PoseKinematics::POSE_PI/180) : 0};
    const double length=std::hypot(leg.forward,leg.left);
    const double arcSweep=routeIndex<executionArcSweeps.size() ? executionArcSweeps[routeIndex] : 0;
    if(arcSweep!=0 && (length<0.05 || PoseKinematics::arcLength(leg,arcSweep*PoseKinematics::POSE_PI/180)>999)) {
        cancelRoute("Arco no ejecutable: los extremos deben ser distintos y la longitud del arco no debe superar 999 cm. Ajustá sus datos."); return;
    }
    const double fraction=length>999 ? 999/length : 1;
    leg.forward*=fraction; leg.left*=fraction; leg.turn*=fraction;
    plannedLegEnd=plannedPosition+delta*fraction;
    plannedLegHeadingDeg=plannedHeadingDeg+leg.turn*180/PoseKinematics::POSE_PI;
    const int axis=arcSweep!=0 ? 7 : routeOriented ? 6 : 4;
    const double distance=std::hypot(leg.forward,leg.left);
    routeExpectedUm=static_cast<qint32>(std::lround(distance*10000.0));
    routeForwardUm=static_cast<qint32>(std::lround(leg.forward*10000.0));
    routeLeftUm=static_cast<qint32>(std::lround(leg.left*10000.0));
    routeTurnUrad=static_cast<qint32>(std::lround(leg.turn*1000000.0));
    routeArcSweepUrad=static_cast<qint32>(std::lround(arcSweep*PoseKinematics::POSE_PI/180*1000000.0));
    // Trama vectorial rechazada por firmware antiguo: no degradar a 45° + recta.
    QByteArray payload(axis==7 ? 22 : routeOriented ? 18 : 13,'\0'); payload[0]=static_cast<char>(0xD0);
    if(axis>=6) { qToLittleEndian<qint32>(routeTurnUrad,payload.data()+13); payload[17]=char(axis); }
    if(axis==7) qToLittleEndian<qint32>(routeArcSweepUrad,payload.data()+18);
    qToLittleEndian<qint32>(routeForwardUm,payload.data()+1);
    qToLittleEndian<qint32>(routeLeftUm,payload.data()+5);
    const int limits[]={ui->spinBox_move_fr->value(),ui->spinBox_move_fl->value(),ui->spinBox_move_rr->value(),ui->spinBox_move_rl->value()};
    // Un eje necesita las cuatro ruedas; no ejecutar rutas con una deshabilitada.
    for(int i=0;i<4;++i) {
        if(limits[i]<=0) { cancelRoute("Configurá potencia mayor que cero en las cuatro ruedas antes de ejecutar la ruta."); return; }
        payload[9+i]=static_cast<char>(limits[i]);
    }
    if(!sendPayload(payload)) { cancelRoute("No se pudo enviar la ruta: revisá la conexión."); return; }
    armMapMovement(axis); motionRunning=true; routeAck=false; routeSeenRunning=false;
    showTopStatus("Desplazamiento");
    expectedGeneration=lastGeneration<0 ? -1 : (lastGeneration+1)&65535;
    routeSentMs=routeLastSampleMs=QDateTime::currentMSecsSinceEpoch();
    ui->label_routeStatus->setText(QString("Setpoint %1/%2: (%3, %4) cm · tramo %5 de %6 cm. Posición estimada por encoders.")
        .arg(routeIndex+1).arg(executionTargets.size()).arg(executionTargets[routeIndex].x()).arg(executionTargets[routeIndex].y())
        .arg(movementName(axis)).arg(distance,0,'f',2));
    if(routeOriented) ui->label_routeStatus->setText(ui->label_routeStatus->text()+QString(" Llegada: %1° · giro progresivo %2°.").arg(executionHeadings[routeIndex],0,'f',1).arg(leg.turn*180/PoseKinematics::POSE_PI,0,'f',1));
    updateCommandUi();
}

void MainWindow::updateCommandUi()
{
    const bool connected = serialPort->isOpen() || udpSocket->state() == QAbstractSocket::BoundState;
    QHostAddress address;
    quint16 port = 0;
    const bool canSend = connected && (serialPort->isOpen() || udpPeer(address, port));
    const bool manualRunning=findChild<QTimer *>("pwmRefreshTimer")->isActive();
    joystick->setControlEnabled(ui->tabWidget_wifi->currentWidget()==joystick && canSend && !routeActive && !routePaused && !motionRunning && !stopPending && !pidPending && (!manualRunning || joystickDriving));
    const bool routeEditable=!routeActive && !motionRunning && !stopPending && !manualRunning;
    const bool onVisualization=ui->tabWidget_wifi->currentWidget()==ui->tab_MAIN;
    const bool onConfiguration=ui->tabWidget_wifi->currentWidget()==ui->tab_CONFIG;
    ui->routePlan->editable=routeEditable && !routePaused && onVisualization;
    ui->routePlan->orientationEditable=routeEditable && !routePaused;
    ui->pushButton_routeOrientation->setEnabled(routeEditable && !routePaused && !pidPending);
    ui->pushButton_routeArc->setEnabled(routeEditable && !routePaused && !pidPending);
    ui->pushButton_importRoute->setEnabled(routeEditable && !routePaused && !pidPending);
    ui->pushButton_exportRoute->setEnabled(!ui->routePlan->points.isEmpty());
    ui->pushButton_resumeRoute->setEnabled(canSend && routeEditable && !pidPending && onVisualization && routePaused && resumeReady);
    ui->pushButton_runRoute->setEnabled(canSend && routeEditable && !pidPending && onVisualization && !ui->routePlan->points.isEmpty());
    ui->pushButton_reverseRoute->setEnabled(ui->pushButton_runRoute->isEnabled());
    ui->pushButton_clearRoute->setEnabled(routeEditable && !ui->routePlan->points.isEmpty());
    ui->pushButton_resetMap->setEnabled(routeEditable);
    ui->groupBox_pid->setEnabled(!motionRunning && !stopPending && !pidPending && !manualRunning);
    ui->pushButton_applyPid->setEnabled(canSend && !motionRunning && !stopPending && !pidPending && !manualRunning);
    ui->pushButton_sendCommand->setEnabled(canSend && !stopPending && !motionRunning && !pidPending && !joystickDriving);
    ui->groupBox_pwm->setEnabled(!motionRunning && !stopPending && !joystickDriving);
    ui->groupBox_moveSettings->setEnabled(!motionRunning && !stopPending && !pidPending && !joystickDriving);
    ui->pushButton_move->setEnabled(canSend && onConfiguration && !motionRunning && !stopPending && !pidPending && !joystickDriving && ui->spinBox_distance->value()!=0);
    ui->pushButton_stop->setEnabled(canSend);
    if(routeActive || routePaused) {
        ui->groupBox_pid->setEnabled(false); ui->groupBox_pwm->setEnabled(false);
        ui->groupBox_moveSettings->setEnabled(false); ui->pushButton_sendCommand->setEnabled(false);
    }
}

void MainWindow::updateConnectionUi()
{
    const bool serialMode = ui->comboBox_protocol->currentIndex() == 0;
    const bool connected = serialMode ? serialPort->isOpen() : udpSocket->state() == QAbstractSocket::BoundState;
    ui->comboBox_protocol->setEnabled(!connected);
    ui->comboBox_serialPort->setVisible(serialMode);
    ui->pushButton_refreshPorts->setVisible(serialMode);
    ui->label_serialPort->setVisible(serialMode);
    ui->lineEdit_local_port->setVisible(!serialMode);
    ui->label_local_port->setVisible(!serialMode);
    ui->lineEdit_device_ip->setVisible(!serialMode);
    ui->label_device_ip->setVisible(!serialMode);
    ui->lineEdit_device_port->setVisible(!serialMode);
    ui->label_device_port->setVisible(!serialMode);
    ui->comboBox_serialPort->setEnabled(!connected);
    ui->pushButton_refreshPorts->setEnabled(!connected);
    ui->lineEdit_local_port->setEnabled(!connected);
    ui->pushButton_connect->setEnabled(connected || !serialMode || ui->comboBox_serialPort->count() > 0);
    ui->pushButton_connect->setText(connected ? "DESCONECTAR" : "CONECTAR");

    if (!connected) {
        ui->label_statusMode->setText(serialMode ? "Serial desconectado" : "UDP desconectado");
    } else if (lastAliveMs > 0 && QDateTime::currentMSecsSinceEpoch() - lastAliveMs < 3000) {
        ui->label_statusMode->setText(serialMode ? "Serial · ALIVE confirmado" : "UDP · ALIVE confirmado");
    } else {
        ui->label_statusMode->setText(serialMode ? "Serial · esperando ALIVE" : "UDP · esperando ALIVE");
    }
    updateCommandUi();
}

void MainWindow::toggleConnection()
{
    findChild<QTimer *>("pidAckTimer")->stop();
    pidPending=false;
    ui->label_pidStatus->setText("Aplicar PID después de conectar; los valores del dispositivo no están confirmados.");
    const bool serialMode = ui->comboBox_protocol->currentIndex() == 0;
    if (serialMode && serialPort->isOpen()) {
        ui->pushButton_stop->click();
        serialPort->waitForBytesWritten(100);
        serialPort->close();
        serialBuffer.clear();
        lastAliveMs = 0;
        log("Puerto serial cerrado.");
        updateConnectionUi();
        return;
    }
    if (!serialMode && udpSocket->state() == QAbstractSocket::BoundState) {
        ui->pushButton_stop->click();
        udpSocket->close();
        lastAliveMs = 0;
        log("Puerto UDP cerrado.");
        updateConnectionUi();
        return;
    }
    if (serialMode) {
        serialPort->setPortName(ui->comboBox_serialPort->currentData().toString());
        serialPort->setBaudRate(115200);
        serialPort->setDataBits(QSerialPort::Data8);
        serialPort->setParity(QSerialPort::NoParity);
        serialPort->setStopBits(QSerialPort::OneStop);
        serialPort->setFlowControl(QSerialPort::NoFlowControl);
        if (!serialPort->open(QIODevice::ReadWrite)) {
            QMessageBox::warning(this, "Puerto serial", serialPort->errorString());
            return;
        }
        // El Mega2560 usa DTR para el reinicio automático al abrir el COM.
        if (!serialPort->setDataTerminalReady(true))
            log("Aviso Serial: no se pudo activar DTR: " + serialPort->errorString());
        log("Serial abierto: " + serialPort->portName() + " · 115200 8N1.");
        updateConnectionUi();
        sendAlive();
        return;
    }
    bool validPort = false;
    const uint localPort = ui->lineEdit_local_port->text().toUInt(&validPort);
    if (!validPort || localPort == 0 || localPort > 65535) {
        QMessageBox::warning(this, "Puerto local", "Ingresá un puerto entre 1 y 65535.");
        return;
    }
    if (!udpSocket->bind(QHostAddress::AnyIPv4, static_cast<quint16>(localPort))) {
        QMessageBox::warning(this, "UDP", udpSocket->errorString());
        return;
    }
    log(QString("UDP abierto en el puerto %1.").arg(localPort));
    updateConnectionUi();
    sendAlive();
}

void MainWindow::receiveSerial()
{
    const QByteArray data = serialPort->readAll();
    if (data.isEmpty()) return;
    log("Serial RX  " + QString::fromLatin1(data.toHex(' ').toUpper()));
    serialBuffer += data;
    processFrames(serialBuffer, "Serial");
}

void MainWindow::receiveUdp()
{
    while (udpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(static_cast<int>(udpSocket->pendingDatagramSize()));
        QHostAddress sender;
        quint16 senderPort = 0;
        if (udpSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort) < 0) {
            log("Error al leer UDP: " + udpSocket->errorString());
            continue;
        }
        log(QString("UDP RX %1:%2  %3").arg(sender.toString()).arg(senderPort)
            .arg(QString::fromLatin1(datagram.toHex(' ').toUpper())));
        QByteArray buffer = datagram;
        processFrames(buffer, "UDP");
        ui->lineEdit_device_ip->setText(QHostAddress(sender.toIPv4Address()).toString());
        ui->lineEdit_device_port->setText(QString::number(senderPort));
        updateConnectionUi();
    }
}

void MainWindow::processFrames(QByteArray &buffer, const QString &transport)
{
    const QByteArray header("UNER", 4);
    while (!buffer.isEmpty()) {
        const int start = buffer.indexOf(header);
        if (start < 0) {
            buffer = buffer.right(qMin(buffer.size(), 3));
            return;
        }
        if (start > 0) buffer.remove(0, start);
        if (buffer.size() < 6) return;
        const int length = static_cast<quint8>(buffer[4]);
        if (length < 2 || buffer[5] != ':') {
            buffer.remove(0, 1);
            continue;
        }
        const int frameSize = length + 6;
        if (buffer.size() < frameSize) return;
        quint8 checksum = 0;
        for (int i = 0; i < frameSize - 1; ++i)
            checksum ^= static_cast<quint8>(buffer[i]);
        if (checksum != static_cast<quint8>(buffer[frameSize - 1])) {
            log(transport + " · trama UNER con checksum inválido.", true);
            buffer.remove(0, frameSize);
            continue;
        }
        const QByteArray payload = buffer.mid(6, length - 1);
        const quint8 id = static_cast<quint8>(payload[0]);
        if(id==0xD2 && (payload.size()==26 || payload.size()==53 || payload.size()==69 || payload.size()==93 || payload.size()==94 || payload.size()==112 || payload.size()==120 || payload.size()==124)) {
            const int mode=payload.size()>=94 ? static_cast<quint8>(payload[93]) : 0;
            const bool hasVectorStatus=payload.size()>=112;
            const bool hasPoseStatus=payload.size()>=120;
            const qint32 requestedArc=payload.size()>=124 ? qFromLittleEndian<qint32>(payload.constData()+120) : 0;
            const qint32 requestedTurn=hasPoseStatus ? qFromLittleEndian<qint32>(payload.constData()+112) : 0;
            const qint32 actualTurn=hasPoseStatus ? qFromLittleEndian<qint32>(payload.constData()+116) : 0;
            const qint32 forward=hasVectorStatus ? qFromLittleEndian<qint32>(payload.constData()+94) : 0;
            const qint32 left=hasVectorStatus ? qFromLittleEndian<qint32>(payload.constData()+98) : 0;
            const qint32 actualForward=hasVectorStatus ? qFromLittleEndian<qint32>(payload.constData()+102) : 0;
            const qint32 actualLeft=hasVectorStatus ? qFromLittleEndian<qint32>(payload.constData()+106) : 0;
            const int generation=hasVectorStatus ? qFromLittleEndian<quint16>(payload.constData()+110) : -1;
            if(hasVectorStatus && ((!routeActive && !routePaused && quint8(payload[1])==0) || lastGeneration<0 || quint16(generation-lastGeneration)<32768)) lastGeneration=generation;
            const bool generationMatches=expectedGeneration<0 || generation==expectedGeneration;
            const QString axisName=mode==4 ? QString("Vectorial · %1°").arg(std::atan2(double(left),double(forward))*180/3.141592653589793,0,'f',1) : movementName(mode);
            const quint8 state=static_cast<quint8>(payload[1]);
            const qint32 target=qFromLittleEndian<qint32>(payload.constData()+2);
            const qint32 um=qFromLittleEndian<qint32>(payload.constData()+6);
            const QStringList states={"Inactivo","En movimiento","Finalizado","Detenido","Fallo: encoder detenido o rueda bloqueada","Fallo: enlace perdido","Fallo: período de control","Fallo: tiempo máximo","Fallo: polaridad de motor o encoder","Detenido con exceso de recorrido por inercia (fuera de tolerancia)"};
            QString status=state<states.size()?states[state]:"Estado desconocido";
            if(state==3) {
                const quint8 reason=payload.size()>=53 ? static_cast<quint8>(payload[27]) : 0;
                status=reason==1 ? "Cancelado por STOP (sin llegada)" : reason==2
                    ? "Cancelado por PWM manual (sin llegada)" : "Detenido; no es confirmación de llegada";
            }
            if((state==4 || state==8) && payload.size()>=53) {
                const quint8 mask=static_cast<quint8>(payload[28]);
                const QStringList names={"FR","FL","RR","RL"};
                QStringList affected;
                for(int i=0;i<4;++i) if(mask&(1u<<i)) affected << names[i];
                if(!affected.isEmpty()) status+=" · ruedas: "+affected.join(" / ");
                if(state==8) status+=" · revisar encoder y sentido según el eje solicitado; parada por seguridad, no llegada";
            }
            QStringList counts;
            for(int i=0;i<4;++i) counts << QString::number(qFromLittleEndian<qint32>(payload.constData()+10+i*4));
            const QString message=QString("%1 · recorrido %5 por encoders %2 cm · objetivo %3 ticks · FR/FL/RR/RL: %4").arg(status).arg(um/10000.0,0,'f',3).arg(target).arg(counts.join(" / ")).arg(axisName);
            log(message,true);
            const double cm=mapReset ? 0.0 : um/10000.0;
            lastDistanceCm=cm;
            haveDistance=true;
            if(topEvent!="Parada de emergencia") {
                if(findChild<QTimer *>("pwmRefreshTimer")->isActive() || state==1) topEvent="Desplazamiento";
                else if(state>=3) topEvent="Parada de emergencia";
                else if(state==2 && topEvent!="Ruta terminada") topEvent="Setpoint alcanzado";
                else if(state==0 && topEvent!="Ruta terminada") topEvent="Esperando telemetría";
            }
            showTopStatus();
            const bool routeSample=routeAck && mode==mapMode && hasVectorStatus && generationMatches && forward==routeForwardUm && left==routeLeftUm
                && (mapMode<5 || (hasPoseStatus && requestedTurn==routeTurnUrad)) && (mapMode!=7 || requestedArc==routeArcSweepUrad);
            if(!routeActive && !mapTracking && !mapReset && state==1) armMapMovement(mode);
            if(mapTracking && mode==mapMode && ((!routeActive && (!routePaused || generationMatches)) || routeSample)) {
                QPointF displacement;
                if(hasVectorStatus) {
                    const double h=mapHeadingDeg*PoseKinematics::POSE_PI/180;
                    const double f=actualForward/10000.0,l=actualLeft/10000.0;
                    if(mode==7) mapArcProgress=qMax(mapArcProgress,PoseKinematics::arcProgress({routeForwardUm/10000.0,routeLeftUm/10000.0,0},routeArcSweepUrad/1000000.0,{f,l,0},mapArcProgress));
                    displacement=QPointF(-f*std::sin(h)-l*std::cos(h),f*std::cos(h)-l*std::sin(h));
                    if(mode>=5 && hasPoseStatus) ui->routePlan->robotHeadingDeg=std::remainder(mapHeadingDeg+actualTurn/1000000.0*180/PoseKinematics::POSE_PI,360.0);
                }
                else {
                    const QPointF unit=mode==0 ? QPointF(0,1) : mode==1 ? QPointF(-1,0)
                        : QPointF((mode==2 ? -1 : 1)/std::sqrt(2.0),1/std::sqrt(2.0));
                    displacement=unit*cm;
                    if(!odometryWarningShown) { log("Firmware anterior: el plano solo proyecta el recorrido solicitado. Actualizar firmware/main para medir XY por las cuatro ruedas.",true); odometryWarningShown=true; }
                }
                ui->routePlan->setRobot(mapOrigin+displacement);
                if(state!=1 && !routePaused) mapTracking=false;
            }
            if(routePaused && payload.size()>=53 && generationMatches && mode==mapMode && (state==2 || state==3) && static_cast<quint8>(payload[27])==1) {
                resumeReady=true;
                ui->label_routeStatus->setText(QString("Ruta pausada en (%1, %2) cm · pulsá Continuar ruta para retomar los destinos pendientes.")
                    .arg(ui->routePlan->robot.x(),0,'f',2).arg(ui->routePlan->robot.y(),0,'f',2));
            }
            if(routeActive) {
                if(state>=3) cancelRoute("Ruta interrumpida: "+status);
                else if(routeSample) {
                    routeLastSampleMs=QDateTime::currentMSecsSinceEpoch();
                    if(state==1) routeSeenRunning=true;
                    if(state==2 && (routeSeenRunning || hasVectorStatus)) {
                        // ARRIVED accepts the firmware's arrival margin. Consume
                        // that target explicitly, then plan from the measured pose
                        // so the residual does not accumulate between setpoints.
                        if(routeIndex<executionTargets.size() && QLineF(plannedLegEnd,executionTargets[routeIndex]).length()<0.05
                            && (!routeOriented || std::abs(std::remainder(plannedLegHeadingDeg-executionHeadings[routeIndex],360.0))<0.1)) {
                            ui->routePlan->completed=++routeIndex; ui->routePlan->update();
                        }
                        plannedPosition=ui->routePlan->robot;
                        plannedHeadingDeg=ui->routePlan->robotHeadingDeg;
                        routeAck=false; routeSeenRunning=false;
                        ui->label_routeStatus->setText("Tramo finalizado · pausa de 500 ms antes de continuar.");
                        routePauseTimer->start();
                    }
                }
            }
            QStringList pidEfforts;
            if(payload.size()>=93 && state==1) {
                const QStringList wheels={"FR","FL","RR","RL"};
                for(int i=0;i<4;++i) {
                    const char *pid=payload.constData()+69+i*6;
                    pidEfforts << QString("%1=%2/%3/%4").arg(wheels[i])
                        .arg(qFromLittleEndian<qint16>(pid)).arg(qFromLittleEndian<qint16>(pid+2))
                        .arg(qFromLittleEndian<qint16>(pid+4)/100.0,0,'f',2);
                }
            }
            if(!pidEfforts.isEmpty()) log("PID (V objetivo/V medida en ticks/s; I en PWM) · "+pidEfforts.join(" | "),true);
            if(payload.size()>=53) {
                const QStringList wheels={"FR","FL","RR","RL"};
                QStringList health;
                for(int i=0;i<4;++i) {
                    const char *p=payload.constData()+29+i*6;
                    health << QString("%1: A=%2, B=%3, saltos inválidos=%4").arg(wheels[i])
                        .arg(qFromLittleEndian<quint16>(p)).arg(qFromLittleEndian<quint16>(p+2)).arg(qFromLittleEndian<quint16>(p+4));
                }
                const QString summary=health.join("\n");
                if(summary!=lastEncoderSummary) { log("Diagnóstico Hall · "+health.join(" | "),true); lastEncoderSummary=summary; }
            }
            lastMotionTelemetryMs=QDateTime::currentMSecsSinceEpoch();
            motionRunning=state==1;
            updateCommandUi();
            buffer.remove(0,frameSize);
            continue;
        }
        if(id==0xD4 && payload.size()==2) {
            findChild<QTimer *>("pidAckTimer")->stop(); pidPending=false;
            const bool accepted=static_cast<quint8>(payload[1])==0x0D;
            ui->label_pidStatus->setText(accepted ? "PID aplicado: confirmado por el firmware (RAM)." : "PID rechazado: comprobar rangos y que el auto esté detenido.");
            log(accepted ? "SETGAINS: ACK; PID aplicado." : "SETGAINS: NACK; PID rechazado.",true);
            updateCommandUi();
        }
        if((id==0xD0 || id==0xD1) && payload.size()==2) {
            const bool accepted=static_cast<quint8>(payload[1])==0x0D;
            log(commandLabel(id)+(accepted?" aceptado: ACK.":" rechazado: NACK."),true);
            if(id==0xD0 && !accepted) motionRunning=false;
            if(id==0xD0 && routeActive) {
                if(accepted) routeAck=true;
                else cancelRoute(routeOriented || mapMode==7 ? "Ruta cancelada: el firmware rechazó el recorrido. Actualizá firmware/main y revisá la geometría del chasis." : "Ruta cancelada: el Arduino rechazó MOVE.");
            }
            if(id==0xD1 && accepted) {
                showTopStatus("Parada de emergencia");
                motionRunning=false;
                stopPending=false;
                findChild<QTimer *>("stopRetryTimer")->stop();
            }
            updateCommandUi();
        }
        log(transport + " RX · " + commandLabel(id), true);
        if (id == 0xA0 && payload.size() >= 2 && static_cast<quint8>(payload[1]) == 0x0D) {
            lastAliveMs = QDateTime::currentMSecsSinceEpoch();
            log("ALIVE + ACK: robot respondió correctamente.", true);
            updateConnectionUi();
        }
        if (id == 0xA4 && payload.size() == 2) {
            const bool accepted = static_cast<quint8>(payload[1]) == 0x0D;
            log(accepted ? "PWM aplicado: ACK." : "PWM rechazado: NACK.", true);
            if (!accepted) {
                findChild<QTimer *>("pwmRefreshTimer")->stop();
                if(joystickDriving) ui->pushButton_stop->click();
                updateCommandUi();
            }
        }
        buffer.remove(0, frameSize);
    }
}

bool MainWindow::sendPayload(const QByteArray &payload)
{
    if (payload.isEmpty() || payload.size() > 254) return false;
    QByteArray frame("UNER", 4);
    frame.append(static_cast<char>(payload.size() + 1));
    frame.append(':');
    frame.append(payload);
    quint8 checksum = 0;
    for (char byte : frame) checksum ^= static_cast<quint8>(byte);
    frame.append(static_cast<char>(checksum));

    qint64 sent = -1;
    QString medium;
    if (serialPort->isOpen()) {
        sent = serialPort->write(frame);
        medium = "Serial";
        if (sent == frame.size() && !serialPort->flush())
            log("Aviso Serial: no se pudo vaciar el buffer de salida: " + serialPort->errorString(), true);
    } else if (udpSocket->state() == QAbstractSocket::BoundState) {
        QHostAddress address;
        quint16 port = 0;
        if (udpPeer(address, port)) {
            sent = udpSocket->writeDatagram(frame, address, port);
            medium = "UDP";
        }
    }
    if (sent != frame.size()) return false;
    log(medium + (medium == "Serial" ? " TX en cola  " : " TX  ")
        + QString::fromLatin1(frame.toHex(' ').toUpper()));
    log(medium + " TX · " + commandLabel(static_cast<quint8>(payload[0])), true);
    return true;
}

void MainWindow::sendSelectedCommand()
{
    if(motionRunning || stopPending || joystickDriving) return;
    const uint id=0xA4;
    const QByteArray payload=pwmPayload(ui);
    if (!sendPayload(payload)) {
        QMessageBox::warning(this, "Enviar comando", "No se pudo enviar el comando.");
        return;
    }
    if (id == 0xA4) {
        auto *timer = findChild<QTimer *>("pwmRefreshTimer");
        const bool moving = ui->spinBox_fr->value() || ui->spinBox_fl->value()
                            || ui->spinBox_rr->value() || ui->spinBox_rl->value();
        if (moving) { timer->start(); showTopStatus("Desplazamiento"); }
        else { timer->stop(); showTopStatus("Parada de emergencia"); }
    }
    updateCommandUi();
}

void MainWindow::driveJoystick(int forward,int left)
{
    auto *timer=findChild<QTimer *>("pwmRefreshTimer");
    if(!forward && !left) {
        if(joystickDriving) ui->pushButton_stop->click();
        return;
    }
    if(ui->tabWidget_wifi->currentWidget()!=joystick || routeActive || routePaused || motionRunning || stopPending || pidPending || (timer->isActive() && !joystickDriving)) return;
    const int limits[]={joystick->power(0),joystick->power(1),joystick->power(2),joystick->power(3)};
    // Eje fijo: traslación mecanum. Eje libre: giro diferencial entre lados.
    const int wheel[]={forward+left,forward-left,
        joystick->axisLocked() ? forward-left : forward+left,
        joystick->axisLocked() ? forward+left : forward-left};
    const int scale=qMax(1,qMax(std::abs(forward+left),std::abs(forward-left)));
    QByteArray payload(9,'\0'); payload[0]=static_cast<char>(0xA4);
    bool moving=false;
    for(int i=0;i<4;++i) moving|=wheel[i]*limits[i]!=0;
    if(!moving) {
        if(joystickDriving) ui->pushButton_stop->click();
        else joystick->reset();
        return;
    }
    for(int i=0;i<4;++i) qToLittleEndian<qint16>(qint16(wheel[i]*limits[i]/scale),payload.data()+1+2*i);
    if(!sendPayload(payload)) {
        if(joystickDriving) ui->pushButton_stop->click();
        else joystick->reset();
        updateCommandUi(); return;
    }
    joystickPayload=payload; joystickDriving=true; timer->start();
    showTopStatus("Desplazamiento"); updateCommandUi();
}

void MainWindow::sendAlive()
{
    if(stopPending) { updateConnectionUi(); return; }
    if (serialPort->isOpen() || udpSocket->state() == QAbstractSocket::BoundState)
        sendPayload(QByteArray(1, static_cast<char>(0xA0)));
    updateConnectionUi();
}
