#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDateTime>
#include <QIntValidator>
#include <QMessageBox>
#include <QSerialPortInfo>
#include <QTextBrowser>

namespace {
struct Command {
    quint8 id;
    const char *name;
    const char *description;
};

// Identificadores y significados conservados de la interfaz anterior.
const Command commands[] = {
    {0xA0, "GETALIVE", "Consulta de estado (ALIVE)"},
    {0xA1, "GETFIRMWARE", "Versión del firmware"},
    {0xA2, "GETMPU", "Datos de la unidad inercial"},
    {0xA3, "GETADC", "Lecturas de sensores analógicos"},
    {0xA4, "SETPWML", "Potencia PWM del motor izquierdo"},
    {0xA5, "SETPWMR", "Potencia PWM del motor derecho"},
    {0xA6, "SETPWMMINR", "PWM mínimo del motor derecho"},
    {0xA7, "SETPWMMINL", "PWM mínimo del motor izquierdo"},
    {0xA8, "SETBALANCEKP", "Kp del balance"},
    {0xA9, "SETBALANCEKD", "Kd del balance"},
    {0xAA, "SETBALANCEKI", "Ki del balance"},
    {0xAB, "SETSETPOINT", "Ángulo objetivo de equilibrio"},
    {0xAC, "SETLINEKP", "Kp del seguidor de línea"},
    {0xAD, "SETLINEKD", "Kd del seguidor de línea"},
    {0xAE, "SETOFFSETL", "Compensación del motor izquierdo"},
    {0xAF, "SETOFFSETR", "Compensación del motor derecho"},
    {0xB0, "SETCUSTOMTURN", "Ganancia de giro en curvas"},
    {0xB1, "SETSPEED", "Ángulo de ataque para avance"},
    {0xB2, "SETBKANG", "Ángulo de frenado"},
    {0xB3, "SETFRONTDIST", "Distancia frontal de obstáculo"},
    {0xB4, "SETSIDEDIST", "Distancia lateral de pared"},
    {0xB5, "SETLOSTDIST", "Distancia de pérdida de pared"},
    {0xB6, "SETSTOPCYCLES", "Ciclos de frenado"},
    {0xB7, "SETCORNERDIST", "Distancia de anticipación en esquina"},
    {0xB8, "SETALIGNDIST", "Distancia de alineación con pared"},
    {0xB9, "SETPWMLROT", "PWM izquierdo en rotación"},
    {0xC0, "SETPWMRROT", "PWM derecho en rotación"},
    {0xC1, "SETSTATICOFF", "Compensación estática"},
    {0xC2, "SETMOVINGOFF", "Compensación dinámica"},
    {0xC3, "SET_KP_EXT", "Kp del lazo externo"},
    {0xC4, "SETLIMITANG", "Límite angular de seguridad"},
    {0xC5, "SET_KI_EXT", "Ki del lazo externo"},
    {0xC6, "SET_ALFA_LPF", "Filtro pasa-bajos"},
    {0xC7, "SETVELDAMPDIV", "Divisor de amortiguamiento"},
    {0xC8, "SETVELDAMPLIM", "Límite de amortiguamiento"},
    {0xC9, "SETTURNLIMIT", "Límite de giro"},
    {0xCA, "EXPORTIRCSV", "Exportar lecturas infrarrojas"},
    {0xCB, "SETWALLKP", "Kp de seguimiento de pared"},
    {0xCC, "SETWALLKD", "Kd de seguimiento de pared"},
    {0xCD, "SETFRONTKP", "Kp de aproximación frontal"},
    {0xCE, "SETFRONTKD", "Kd de aproximación frontal"},
    {0xCF, "SETDODGEMODE", "Dirección de evasión"},
    {0xD1, "SETSOFTAP", "Cambiar a modo SoftAP"},
    {0xD2, "SETROBOTMODE", "Seleccionar modo del robot"},
    {0xD3, "SETJOYSTICKTURN / SETGOTOTURN", "Giro manual o joystick"},
    {0xD4, "SETDODGEBIASTIME", "Tiempo de sesgo de giro"},
    {0xD5, "SETDODGEBIASMULT", "Multiplicador del sesgo de evasión"},
    {0xF0, "GETINTERNALDATA", "Datos internos del seguidor de línea"},
    {0xF1, "GETPIDBALANCE", "Términos PID del balance"}
};

QString commandLabel(quint8 id)
{
    for (const Command &command : commands) {
        if (command.id == id)
            return QString("%1 (0x%2) · %3").arg(command.name)
                .arg(id, 2, 16, QLatin1Char('0')).arg(QString::fromUtf8(command.description));
    }
    return QString("Comando desconocido (0x%1)").arg(id, 2, 16, QLatin1Char('0'));
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
    ui->lineEdit_local_port->setValidator(new QIntValidator(1, 65535, this));
    ui->lineEdit_device_port->setValidator(new QIntValidator(1, 65535, this));
    ui->textBrowserUnProcessed->document()->setMaximumBlockCount(500);
    ui->textBrowserProcessed->document()->setMaximumBlockCount(500);
    for (const Command &command : commands)
        ui->comboBox_command->addItem(commandLabel(command.id), command.id);
    refreshSerialPorts();

    connect(ui->comboBox_protocol, &QComboBox::currentIndexChanged, this, &MainWindow::updateConnectionUi);
    connect(ui->comboBox_command, &QComboBox::currentIndexChanged, this, &MainWindow::updateCommandUi);
    connect(ui->pushButton_refreshPorts, &QPushButton::clicked, this, &MainWindow::refreshSerialPorts);
    connect(ui->pushButton_connect, &QPushButton::clicked, this, &MainWindow::toggleConnection);
    connect(ui->pushButton_sendCommand, &QPushButton::clicked, this, &MainWindow::sendSelectedCommand);
    connect(ui->lineEdit_device_ip, &QLineEdit::textChanged, this, &MainWindow::updateConnectionUi);
    connect(ui->lineEdit_device_port, &QLineEdit::textChanged, this, &MainWindow::updateConnectionUi);
    connect(ui->pushButton_clearLogs, &QPushButton::clicked, this, [this]() {
        ui->textBrowserUnProcessed->clear();
        ui->textBrowserProcessed->clear();
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
    aliveTimer->start();
    updateConnectionUi();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::log(const QString &message, bool processed)
{
    const QString line = QDateTime::currentDateTime().toString("hh:mm:ss.zzz  ") + message;
    (processed ? ui->textBrowserProcessed : ui->textBrowserUnProcessed)->append(line.toHtmlEscaped());
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
    const quint8 id = static_cast<quint8>(ui->comboBox_command->currentData().toUInt());
    const bool supported = id == 0xA0;
    ui->label_commandSupport->setText(supported
        ? "Este firmware responde GETALIVE con ALIVE + ACK."
        : "Referencia histórica: este firmware todavía no implementa este comando.");
    const bool connected = serialPort->isOpen() || udpSocket->state() == QAbstractSocket::BoundState;
    QHostAddress address;
    quint16 port = 0;
    ui->pushButton_sendCommand->setEnabled(supported && connected
        && (serialPort->isOpen() || udpPeer(address, port)));
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
    const bool serialMode = ui->comboBox_protocol->currentIndex() == 0;
    if (serialMode && serialPort->isOpen()) {
        serialPort->close();
        serialBuffer.clear();
        lastAliveMs = 0;
        log("Puerto serial cerrado.");
        updateConnectionUi();
        return;
    }
    if (!serialMode && udpSocket->state() == QAbstractSocket::BoundState) {
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
        log(transport + " RX · " + commandLabel(id), true);
        if (id == 0xA0 && payload.size() >= 2 && static_cast<quint8>(payload[1]) == 0x0D) {
            lastAliveMs = QDateTime::currentMSecsSinceEpoch();
            log("ALIVE + ACK: robot respondió correctamente.", true);
            updateConnectionUi();
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
    if (ui->comboBox_command->currentData().toUInt() != 0xA0) return;
    if (!sendPayload(QByteArray(1, static_cast<char>(0xA0))))
        QMessageBox::warning(this, "Enviar comando", "No se pudo enviar GETALIVE.");
}

void MainWindow::sendAlive()
{
    if (serialPort->isOpen() || udpSocket->state() == QAbstractSocket::BoundState)
        sendPayload(QByteArray(1, static_cast<char>(0xA0)));
    updateConnectionUi();
}
