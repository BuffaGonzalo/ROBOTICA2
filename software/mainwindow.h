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
#include <QPointF>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE
class JoystickWidget;

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
    void driveJoystick(int forward,int left);
    JoystickWidget *joystick=nullptr;
    bool joystickDriving=false;
    QByteArray joystickPayload;
    void sendAlive();
    bool sendPayload(const QByteArray &payload);
    void updateConnectionUi();
    void updateCommandUi();
    void log(const QString &message, bool processed = false);
    void cleanOldLogs();
    void exportLogTxt();
    void exportLogXls();
    bool udpPeer(QHostAddress &address, quint16 &port) const;
    void startRoute();
    bool importRouteFile(const QString &path, QString &error);
    bool exportRouteFile(const QString &path, QString &error) const;
    void startReverseRoute();
    void resumeRoute();
    void nextRouteLeg();
    void cancelRoute(const QString &reason);
    void armMapMovement(int mode);
    void showTopStatus(const QString &event=QString());
    QString topEvent="Esperando telemetría";
    bool haveDistance=false;
    bool routeActive=false, routeAck=false, routeSeenRunning=false, mapTracking=false;
    int routeIndex=0, mapMode=0;
    qint32 routeExpectedUm=0;
    qint64 routeSentMs=0, routeLastSampleMs=0;
    QPointF mapOrigin;
    QList<QPointF> executionTargets;
    QList<double> executionHeadings;
    QList<double> executionArcSweeps;
    qint32 routeArcSweepUrad=0;
    double mapArcProgress=0;
    bool routeOriented=false;
    double plannedHeadingDeg=0,plannedLegHeadingDeg=0,mapHeadingDeg=0;
    qint32 routeTurnUrad=0;
    QPointF plannedPosition,plannedLegEnd;
    bool reverseRoute=false;
    bool mapReset=false;
    bool routePaused=false, resumeReady=false, pausedBetweenLegs=false;
    QTimer *routePauseTimer=nullptr;
    qint32 routeForwardUm=0,routeLeftUm=0;
    int lastGeneration=-1,expectedGeneration=-1;
    bool odometryWarningShown=false;

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
