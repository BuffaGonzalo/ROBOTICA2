#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QByteArray>
#include <QHostAddress>
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
    bool udpPeer(QHostAddress &address, quint16 &port) const;

    Ui::MainWindow *ui;
    QSerialPort *serialPort;
    QUdpSocket *udpSocket;
    QTimer *aliveTimer;
    QByteArray serialBuffer;
    qint64 lastAliveMs = 0;
};

#endif // MAINWINDOW_H
