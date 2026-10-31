#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QByteArray>
#include <QDateTime>
#include <QHostAddress>
#include <QList>
#include <QSerialPort>
#include <QUdpSocket>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private:
    struct LogEntry {
        QDateTime timestamp;
        QString type;
        QString message;
    };

    void refreshSerialPorts();
    void toggleConnection();
    void receiveSerial();
    void receiveUdp();
    void processFrames(QByteArray &buffer, const QString &transport);
    void sendSelectedCommand();
    void sendAlive();
    bool sendPayload(const QByteArray &payload);
    void updateConnectionUi();
    void updateCommandUi();
    void log(const QString &message, bool processed = false);
    void cleanOldLogs();
    void exportLogTxt();
    void exportLogXls();
    bool udpPeer(QHostAddress &address, quint16 &port) const;

    Ui::MainWindow *ui;
    QSerialPort *serialPort;
    QUdpSocket *udpSocket;
    QTimer *aliveTimer;
    QByteArray serialBuffer;
    QList<LogEntry> logHistory;
    qint64 lastAliveMs = 0;
    qint64 lastMotionTelemetryMs = 0;
    bool motionRunning = false;
    bool stopPending = false;
    bool pidPending = false;
    double lastDistanceCm = 0;
    QString lastEncoderSummary;
};

#endif // MAINWINDOW_H
