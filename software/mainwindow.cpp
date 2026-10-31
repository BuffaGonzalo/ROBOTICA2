#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QIntValidator>
#include <QMessageBox>
#include <QSerialPortInfo>
#include <QTextBrowser>
#include <QTextStream>
#include <QtEndian>
#include <cmath>
#include <cstring>
#include <QHeaderView>
#include <QTableWidget>

namespace {
int directionWheelSign(int direction,int wheel) {
    if(direction==2) return wheel==0 || wheel==3 ? 1 : 0;
    if(direction==3) return wheel==1 || wheel==2 ? 1 : 0;
    return direction==1 && (wheel==1 || wheel==2) ? -1 : 1;
}
QString movementName(int direction) {
    return QStringList({"Longitudinal (X)","Lateral (Y)","Diagonal","Diagonal derecha"}).value(direction,"Dirección desconocida");
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
    for(QSpinBox *spin : {ui->spinBox_fr,ui->spinBox_fl,ui->spinBox_rr,ui->spinBox_rl}) {
        spin->setRange(-255,255);
        spin->setToolTip("PWM de prueba: −255 a +255. Negativo invierte el giro; cero frena. Escribí el valor con su signo.");
    }
    ui->tableWidget_visual->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->tableWidget_visual->verticalHeader()->setVisible(false);
    const QStringList wheels={"FR","FL","RR","RL"};
    for(int row=0;row<4;++row)
        for(int col=0;col<8;++col) ui->tableWidget_visual->setItem(row,col,new QTableWidgetItem(col==0?wheels[row]:QStringLiteral("—")));
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
    connect(stopTimer,&QTimer::timeout,this,[this]() {
        sendPayload(QByteArray(1,static_cast<char>(0xD1)));
    });
    connect(pwmTimer, &QTimer::timeout, this, [this, pwmTimer]() {
        if (!sendPayload(pwmPayload(ui))) {
            pwmTimer->stop();
            log("No se pudo renovar PWM; el robot se detiene por timeout.", true);
        }
    });
    connect(ui->pushButton_stop, &QPushButton::clicked, this, [this, pwmTimer, stopTimer]() {
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
        motionRunning=true;
        ui->label_visualState->setText("MOVE enviado; esperando estado del firmware. Los valores mostrados corresponden a la última muestra recibida.");
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
            ui->label_visualFreshness->setText("Sin telemetría reciente; los valores son la última muestra recibida.");
            ui->label_topDistance->setText(QString("Último recorrido: %1 cm (sin datos recientes)").arg(lastDistanceCm,0,'f',2));
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

void MainWindow::updateCommandUi()
{
    const bool connected = serialPort->isOpen() || udpSocket->state() == QAbstractSocket::BoundState;
    QHostAddress address;
    quint16 port = 0;
    const bool canSend = connected && (serialPort->isOpen() || udpPeer(address, port));
    const bool manualRunning=findChild<QTimer *>("pwmRefreshTimer")->isActive();
    ui->groupBox_pid->setEnabled(!motionRunning && !stopPending && !pidPending && !manualRunning);
    ui->pushButton_applyPid->setEnabled(canSend && !motionRunning && !stopPending && !pidPending && !manualRunning);
    ui->pushButton_sendCommand->setEnabled(canSend && !stopPending && !motionRunning && !pidPending);
    ui->groupBox_pwm->setEnabled(!motionRunning && !stopPending);
    ui->groupBox_moveSettings->setEnabled(!motionRunning && !stopPending && !pidPending);
    ui->pushButton_move->setEnabled(canSend && !motionRunning && !stopPending && !pidPending && ui->spinBox_distance->value()!=0);
    ui->pushButton_stop->setEnabled(canSend);
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
        if(id==0xD2 && (payload.size()==26 || payload.size()==53 || payload.size()==69 || payload.size()==93 || payload.size()==94)) {
            const int mode=payload.size()>=94 ? static_cast<quint8>(payload[93]) : 0;
            const double factor=mode>=2 ? std::sqrt(2.0) : 1.0;
            const QString axisName=movementName(mode);
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
            double scale=2464.0/(3.14159265358979323846*80.59);
            if(payload.size()>=69) {
                const double reported=qFromLittleEndian<qint32>(payload.constData()+61)/1000000.0;
                if(reported>0 && reported<100000) scale=reported;
            }
            const double cm=um/10000.0;
            lastDistanceCm=cm;
            ui->label_topDistance->setText(QString("Recorrido %2: %1 cm").arg(cm,0,'f',2).arg(axisName));
            const double goalCm=target/(scale*10.0*factor);
            ui->label_visualDistance->setText(QString("Recorrido %2 por encoders: %1 cm").arg(cm,0,'f',2).arg(axisName));
            ui->label_visualTarget->setText(QString("Objetivo recibido: %1 cm (%2 ticks) · restante medio: %3 cm")
                .arg(goalCm,0,'f',2).arg(target).arg(goalCm-cm,0,'f',2));
            ui->label_visualState->setText(status);
            ui->progressBar_distance->setValue(target ? (int)std::lround(qBound(0.0,cm/goalCm*100.0,100.0)) : (state==2?100:0));
            QStringList pidEfforts;
            for(int i=0;i<4;++i) {
                const qint32 tick=qFromLittleEndian<qint32>(payload.constData()+10+i*4);
                const double wheelCm=tick/(scale*10.0);
                const int sign=directionWheelSign(mode,i);
                const QStringList cells={QString::number(tick),QString::number(wheelCm,'f',2),sign ? QString::number(goalCm-wheelCm*sign/factor,'f',2) : QStringLiteral("Libre"),
                    payload.size()>=69?QString::number(qFromLittleEndian<qint16>(payload.constData()+53+i*2)):QStringLiteral("—")};
                for(int col=0;col<4;++col) ui->tableWidget_visual->item(i,col+1)->setText(cells[col]);
                for(int col=5;col<8;++col) ui->tableWidget_visual->item(i,col)->setText(QStringLiteral("—"));
                if(payload.size()>=93) {
                    const char *pid=payload.constData()+69+i*6;
                    ui->tableWidget_visual->item(i,5)->setText(QString::number(qFromLittleEndian<qint16>(pid)));
                    if(state==1) ui->tableWidget_visual->item(i,6)->setText(QString::number(qFromLittleEndian<qint16>(pid+2)));
                    ui->tableWidget_visual->item(i,7)->setText(QString::number(qFromLittleEndian<qint16>(pid+4)/100.0,'f',2));
                    if(state==1) pidEfforts << QString("%1=%2/%3/%4")
                        .arg(QStringList({"FR","FL","RR","RL"})[i])
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
                ui->label_visualEncoderHealth->setText(summary);
                if(summary!=lastEncoderSummary) { log("Diagnóstico Hall · "+health.join(" | "),true); lastEncoderSummary=summary; }
            } else ui->label_visualEncoderHealth->setText("Firmware sin diagnóstico de canales A/B.");
            ui->tableWidget_visual->horizontalHeaderItem(3)->setText("Error eje (cm)");
            ui->tableWidget_visual->setToolTip("Ticks, distancia de rueda, PWM y velocidades conservan su signo físico. Error eje compara cada rueda con su meta proyectada en X o Y.");
            ui->label_visualPid->setText(payload.size()>=93
                ? "PID observable: V objetivo y V medida en ticks/s; integral en PWM (límite ±50). V medida se muestra durante movimiento."
                : "Actualizar firmware/main para ver velocidad objetivo, velocidad medida y aporte integral del PID.");
            const double diameter=payload.size()>=69 ? qFromLittleEndian<qint32>(payload.constData()+65)/1000.0 : 80.59;
            ui->label_visualScale->setText(QString("Escala %1: %2 ticks/vuelta · %3 ticks/mm · tolerancia: ±%4 cm (5 ticks)")
                .arg(payload.size()>=69?"recibida del firmware":"nominal (firmware antiguo)")
                .arg(scale*3.14159265358979323846*diameter,0,'f',0).arg(scale,0,'f',6).arg(5.0/(scale*10),0,'f',3));
            lastMotionTelemetryMs=QDateTime::currentMSecsSinceEpoch();
            ui->label_visualFreshness->setText("Última muestra: "+QDateTime::currentDateTime().toString("hh:mm:ss.zzz"));
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
            if(id==0xD1 && accepted) {
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
            if (!accepted) findChild<QTimer *>("pwmRefreshTimer")->stop();
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
    if(motionRunning || stopPending) return;
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
        if (moving) timer->start();
        else timer->stop();
    }
    updateCommandUi();
}

void MainWindow::sendAlive()
{
    if(stopPending) { updateConnectionUi(); return; }
    if (serialPort->isOpen() || udpSocket->state() == QAbstractSocket::BoundState)
        sendPayload(QByteArray(1, static_cast<char>(0xA0)));
    updateConnectionUi();
}
