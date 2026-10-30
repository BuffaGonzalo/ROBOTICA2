/**
 * @file mainwindow.cpp
 * @brief Implementación de la lógica de control, comunicaciones y telemetría de MainWindow.
 * @author Gonzalo Gomez
 * @date 2026
 *
 * @details Este archivo implementa las funciones de negocio de la estación terrena:
 * - Configuración del entorno gráfico, gráficos QChart y enlace QML 3D.
 * - Despacho de paquetes mediante protocolo UNER a través de Serial, UDP y TCP.
 * - Decodificación de flujos de telemetría y actualización de indicadores en tiempo real.
 * - Gestión de atajos de teclado y eventos interactivos para el modo Joystick / GoTo.
 *
 * @see mainwindow.h
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "internal_data.h"
#include <limits>
#include <QSpinBox>
#include <QFile>
#include <QTextStream>
#include <QFileDialog>
#include <QDateTime>
#include <QtWidgets/QGraphicsLayout>
#include <QListWidget>

static bool isWaitingReply = false;
static int timeoutPatience = 0;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    myDebugDialog = new QDialog(this);
    myDebugDialog->setWindowTitle("WIFI");
    myDebugDialog->setWindowFlags(Qt::Window);
    myDebugDialog->setStyleSheet(this->styleSheet());
    ui->stackedWidget->removeWidget(ui->DEBUG_PAGE);
    QVBoxLayout *debugLayout = new QVBoxLayout(myDebugDialog);
    debugLayout->setContentsMargins(0, 0, 0, 0);
    debugLayout->addWidget(ui->DEBUG_PAGE);
    ui->DEBUG_PAGE->show();
    myDebugDialog->adjustSize();

    myHelpDialog = new QDialog(this);
    myHelpDialog->setWindowTitle("HELP - MCC25");
    myHelpDialog->setWindowFlags(Qt::Window);
    myHelpDialog->setStyleSheet(this->styleSheet());
    ui->stackedWidget->removeWidget(ui->HELP_PAGE);
    QVBoxLayout *helpLayout = new QVBoxLayout(myHelpDialog);
    helpLayout->setContentsMargins(0, 0, 0, 0);
    helpLayout->addWidget(ui->HELP_PAGE);
    ui->HELP_PAGE->show();
    myHelpDialog->resize(720, 560);

    myPasswordDialog = new QDialog(this);
    myPasswordDialog->setWindowTitle("ACCESO A CONFIGURACIÓN");
    myPasswordDialog->setWindowFlags(Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
    myPasswordDialog->setModal(true);
    myPasswordDialog->setStyleSheet(this->styleSheet());
    ui->stackedWidget->removeWidget(ui->PASSWORD_PAGE);
    QVBoxLayout *passwordLayout = new QVBoxLayout(myPasswordDialog);
    passwordLayout->setContentsMargins(0, 0, 0, 0);
    passwordLayout->addWidget(ui->PASSWORD_PAGE);
    ui->PASSWORD_PAGE->show();
    myPasswordDialog->resize(440, 260);

    connect(ui->lineEdit_configPassword, &QLineEdit::returnPressed, this, &MainWindow::on_btn_password_accept_clicked, Qt::UniqueConnection);
    connect(ui->lineEdit_configPassword, &QLineEdit::textEdited, this, &MainWindow::resetPasswordDialogStyle, Qt::UniqueConnection);
    connect(ui->btn_password_cancel, &QPushButton::clicked, this, &MainWindow::on_btn_password_cancel_clicked, Qt::UniqueConnection);
    connect(ui->btn_password_retry, &QPushButton::clicked, this, &MainWindow::on_btn_password_retry_clicked, Qt::UniqueConnection);
    connect(ui->btn_password_error_cancel, &QPushButton::clicked, this, &MainWindow::on_btn_password_cancel_clicked, Qt::UniqueConnection);

    ui->btn_password_cancel->setAutoDefault(false);
    ui->btn_password_cancel->setDefault(false);
    ui->btn_password_error_cancel->setAutoDefault(false);
    ui->btn_password_error_cancel->setDefault(false);
    ui->btn_password_accept->setAutoDefault(true);
    ui->btn_password_accept->setDefault(true);
    ui->btn_password_retry->setAutoDefault(true);
    ui->btn_password_retry->setDefault(true);

    myJoystickExitDialog = new QDialog(this);
    myJoystickExitDialog->setWindowTitle("MODO JOYSTICK ACTIVO");
    myJoystickExitDialog->setWindowFlags(Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
    myJoystickExitDialog->setModal(true);
    myJoystickExitDialog->setStyleSheet(this->styleSheet());
    ui->stackedWidget->removeWidget(ui->JOYSTICK_EXIT_PAGE);
    QVBoxLayout *joystickExitLayout = new QVBoxLayout(myJoystickExitDialog);
    joystickExitLayout->setContentsMargins(0, 0, 0, 0);
    joystickExitLayout->addWidget(ui->JOYSTICK_EXIT_PAGE);
    ui->JOYSTICK_EXIT_PAGE->show();
    myJoystickExitDialog->resize(440, 220);

    connect(ui->btn_joystick_exit_cancel, &QPushButton::clicked, this, &MainWindow::on_btn_joystick_exit_cancel_clicked, Qt::UniqueConnection);
    connect(ui->btn_joystick_exit_confirm, &QPushButton::clicked, this, &MainWindow::on_btn_joystick_exit_confirm_clicked, Qt::UniqueConnection);

    initHelpContent();

    connect(ui->actionHELP, &QAction::triggered, this, &MainWindow::on_actionDocu_triggered);

    // Limitar los logs a los últimos 50 comandos
    ui->textBrowserProcessed->document()->setMaximumBlockCount(50);
    ui->textBrowserUnProcessed->document()->setMaximumBlockCount(50);

    initPIDChart();

    // Conectar checkboxes para activar/desactivar curvas de la gráfica PID
    connect(ui->checkBox_P, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_I, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_D, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Out, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_P_line, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_D_line, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Out_line, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Pitch, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Roll, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Yaw, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Ax, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Ay, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Az, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Gx, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Gy, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Gz, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Ir1, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Ir2, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Ir3, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Ir4, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Ir5, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Ir6, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Ir7, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);
    connect(ui->checkBox_Ir8, &QCheckBox::toggled, this, &MainWindow::updatePIDChartRange);

    timer1 = new QTimer(this);
    timer2 = new QTimer(this);

        // Inicializar barras de progreso de sensores
    ui->bar_ir1->setValue(100); ui->bar_ir1->setFormat("Libre (100%)");
    ui->bar_ir3->setValue(100); ui->bar_ir3->setFormat("Libre (100%)");
    ui->bar_ir5->setValue(100); ui->bar_ir5->setFormat("Libre (100%)");
    ui->bar_ir7->setValue(100); ui->bar_ir7->setFormat("Libre (100%)");
    ui->bar_ir8->setValue(100); ui->bar_ir8->setFormat("Libre (100%)");
    ui->bar_ir2->setValue(0);   ui->bar_ir2->setFormat("0 / 4095");
    ui->bar_ir4->setValue(0);   ui->bar_ir4->setFormat("0 / 4095");
    ui->bar_ir6->setValue(0);   ui->bar_ir6->setFormat("0 / 4095");

    
    // Configurar foco seguro para MainWindow
    this->setFocusPolicy(Qt::StrongFocus);

    // Desactivar autoExclusive y foco en los botones de navegación y modos para que las flechas jamás cambien de pestaña
    ui->btn_nav_infrarrojos->setAutoExclusive(false);
    ui->btn_nav_visualizacion->setAutoExclusive(false);
    ui->btn_nav_tuning->setAutoExclusive(false);
    ui->btn_nav_goto->setAutoExclusive(false);
    ui->btn_mode_balance->setAutoExclusive(false);
    ui->btn_mode_line->setAutoExclusive(false);
    ui->btn_mode_dodge->setAutoExclusive(false);
    ui->btn_mode_goto->setAutoExclusive(false);

    ui->btn_nav_infrarrojos->setFocusPolicy(Qt::NoFocus);
    ui->btn_nav_visualizacion->setFocusPolicy(Qt::NoFocus);
    ui->btn_nav_tuning->setFocusPolicy(Qt::NoFocus);
    ui->btn_nav_goto->setFocusPolicy(Qt::NoFocus);
    ui->btn_mode_balance->setFocusPolicy(Qt::NoFocus);
    ui->btn_mode_line->setFocusPolicy(Qt::NoFocus);
    ui->btn_mode_dodge->setFocusPolicy(Qt::NoFocus);
    ui->btn_mode_goto->setFocusPolicy(Qt::NoFocus);

    // Conexiones de controles del Modo Joystick
    if (ui->btn_goto_up) connect(ui->btn_goto_up, &QPushButton::clicked, this, &MainWindow::on_btn_goto_up_clicked);
    if (ui->btn_goto_down) connect(ui->btn_goto_down, &QPushButton::clicked, this, &MainWindow::on_btn_goto_down_clicked);
    if (ui->btn_goto_left) {
        connect(ui->btn_goto_left, &QPushButton::pressed, this, &MainWindow::on_btn_goto_left_pressed);
        connect(ui->btn_goto_left, &QPushButton::released, this, &MainWindow::on_btn_goto_left_released);
    }
    if (ui->btn_goto_right) {
        connect(ui->btn_goto_right, &QPushButton::pressed, this, &MainWindow::on_btn_goto_right_pressed);
        connect(ui->btn_goto_right, &QPushButton::released, this, &MainWindow::on_btn_goto_right_released);
    }
    if (ui->btn_goto_center) connect(ui->btn_goto_center, &QPushButton::clicked, this, &MainWindow::on_btn_goto_center_clicked);
    if (ui->spinBox_gotoStep) {
        connect(ui->spinBox_gotoStep, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &MainWindow::on_spinBox_gotoStep_valueChanged);
    }
    if (ui->spinBox_gotoTurnIntensity) {
        connect(ui->spinBox_gotoTurnIntensity, QOverload<int>::of(&QSpinBox::valueChanged),
                this, &MainWindow::on_spinBox_gotoTurnIntensity_valueChanged);
    }
    if (ui->spinBox_gotoTurnDuration) {
        connect(ui->spinBox_gotoTurnDuration, QOverload<int>::of(&QSpinBox::valueChanged),
                this, &MainWindow::on_spinBox_gotoTurnDuration_valueChanged);
    }
    if (ui->btn_goto_reset_yaw) {
        connect(ui->btn_goto_reset_yaw, &QPushButton::clicked, this, &MainWindow::on_btn_goto_reset_yaw_clicked);
    }

    m_gotoTurnKeepAliveTimer = new QTimer(this);
    m_gotoTurnKeepAliveTimer->setInterval(100);
    connect(m_gotoTurnKeepAliveTimer, &QTimer::timeout, this, [this]() {
        if (m_isRotatingLeft) sendGoToTurn(m_gotoTurnIntensity, m_gotoTurnDuration);
        else if (m_isRotatingRight) sendGoToTurn(-m_gotoTurnIntensity, m_gotoTurnDuration);
    });

    connect(ui->stackedWidget, &QStackedWidget::currentChanged, this, &MainWindow::updateNavSelection);
    ui->stackedWidget->setCurrentIndex(0);
    updateNavSelection(0);

    ui->AutoWidget->setSource(QUrl(QStringLiteral("qrc:/Scene3D.qml")));
    ui->AutoWidget->setResizeMode(QQuickWidget::SizeRootObjectToView);
    connect(ui->AutoWidget, &QQuickWidget::statusChanged, this, [this](QQuickWidget::Status status) {
        if (status == QQuickWidget::Ready) {
            update3DModelConnectionState(isRobotConnected());
        }
    });

    // Integración de Brújula de Orientación
    m_compassWidget = new CompassWidget(this);
    if (ui->layout_compass_dial) {
        ui->layout_compass_dial->addWidget(m_compassWidget);
    }

    //comunicacion
    QSerialPort1 = new QSerialPort(this);
    QUdpSocket1 = new QUdpSocket(this);
    QTcpServer1 = new QTcpServer(this);
    QTcpSocketClient = nullptr;

    //debug de comandos


    ui->comboBox_PORT->installEventFilter(this);
    qApp->installEventFilter(this);

    //connects del puerto serial
    connect(QSerialPort1,&QSerialPort::readyRead,this,&MainWindow::dataReceived);
    //connects de los timers con las funciones
    connect(timer1,&QTimer::timeout,this,&MainWindow::timeOut);
    connect(timer2,&QTimer::timeout,this,&MainWindow::getData);

    //connects de udp/tcp
    connect(QUdpSocket1,&QUdpSocket::readyRead,this,&MainWindow::OnUdpRxData);
    connect(QTcpServer1,&QTcpServer::newConnection,this,&MainWindow::OnTcpNewConnection);

    //connect(ui->actionScanPorts, &QAction::triggered, settingPorts,&SettingsDialog::show);




    //inicializamos
    estadoProtocolo=START;
    rxData.timeOut=0;

    timer2->start(75);

    statusMode = ui->label_statusMode;
    ui->statusBar->addWidget(statusMode);
    statusMode->setText("ESTADO --> DESCONECTADO");
    statusMode->setStyleSheet("color: #fd5d93; font-weight: bold; font-size: 11px; padding-left: 5px;");

    runtimeTimer.start();

    timer1->start(100);
    timer2->start(35);

    // Permitir valores de hasta 10000 (y -10000) en todos los QSpinBox
    QList<QSpinBox *> spinBoxes = this->findChildren<QSpinBox *>();
    for (QSpinBox *spinBox : spinBoxes) {
        spinBox->setMinimum(-10000);
        spinBox->setMaximum(10000);
    }

    // Iniciar desconectado por defecto (requerimiento de usuario)
    ui->pushButton_connectUdp->setText("CONNECT");
    updateProtocolUI();

    // Inicializar lista de comandos individuales
    if (ui->comboBox_CMD) {
        ui->comboBox_CMD->clear();
        ui->comboBox_CMD->addItem("GETALIVE (0xA0)", GETALIVE);
        ui->comboBox_CMD->addItem("GETFIRMWARE (0xA1)", GETFIRMWARE);
        ui->comboBox_CMD->addItem("GETMPU (0xA2)", GETMPU);
        ui->comboBox_CMD->addItem("GETADC (0xA3)", GETADC);
        ui->comboBox_CMD->addItem("GETINTERNALDATA (0xF0)", GETINTERNALDATA);
        ui->comboBox_CMD->addItem("GETPIDBALANCE (0xF1)", GETPIDBALANCE);
        ui->comboBox_CMD->addItem("EXPORTIRCSV (0xD3)", EXPORTIRCSV);
        ui->comboBox_CMD->addItem("SETPWML (0xA4)", SETPWML);
        ui->comboBox_CMD->addItem("SETPWMR (0xA5)", SETPWMR);
        ui->comboBox_CMD->addItem("SETPWMMINL (0xA7)", SETPWMMINL);
        ui->comboBox_CMD->addItem("SETPWMMINR (0xA6)", SETPWMMINR);
        ui->comboBox_CMD->addItem("SETBALANCEKP (0xA8)", SETBALANCEKP);
        ui->comboBox_CMD->addItem("SETBALANCEKD (0xA9)", SETBALANCEKD);
        ui->comboBox_CMD->addItem("SETBALANCEKI (0xAA)", SETBALANCEKI);
        ui->comboBox_CMD->addItem("SETSETPOINT (0xAB)", SETSETPOINT);
        ui->comboBox_CMD->addItem("SETSPEED (0xB1)", SETSPEED);
        ui->comboBox_CMD->addItem("SETBKANG (0xB2)", SETBKANG);
        ui->comboBox_CMD->addItem("SETSTATICOFF (0xC1)", SETSTATICOFF);
        ui->comboBox_CMD->addItem("SETMOVINGOFF (0xC2)", SETMOVINGOFF);
        ui->comboBox_CMD->addItem("SETLIMITANG (0xC4)", SETLIMITANG);
        ui->comboBox_CMD->addItem("SET_KP_EXT (0xC3)", SET_KP_EXT);
        ui->comboBox_CMD->addItem("SET_KI_EXT (0xC5)", SET_KI_EXT);
        ui->comboBox_CMD->addItem("SET_ALFA_LPF (0xC6)", SET_ALFA_LPF);
        ui->comboBox_CMD->addItem("SETVELDAMPDIV (0xC7)", SETVELDAMPDIV);
        ui->comboBox_CMD->addItem("SETVELDAMPLIM (0xC8)", SETVELDAMPLIM);
        ui->comboBox_CMD->addItem("SETTURNLIMIT (0xC9)", SETTURNLIMIT);
        ui->comboBox_CMD->addItem("SETLINEKP (0xAC)", SETLINEKP);
        ui->comboBox_CMD->addItem("SETLINEKD (0xAD)", SETLINEKD);
        ui->comboBox_CMD->addItem("SETOFFSETL (0xAE)", SETOFFSETL);
        ui->comboBox_CMD->addItem("SETOFFSETR (0xAF)", SETOFFSETR);
        ui->comboBox_CMD->addItem("SETCUSTOMTURN (0xB0)", SETCUSTOMTURN);
        ui->comboBox_CMD->addItem("SETFRONTDIST (0xB3)", SETFRONTDIST);
        ui->comboBox_CMD->addItem("SETSIDEDIST (0xB4)", SETSIDEDIST);
        ui->comboBox_CMD->addItem("SETLOSTDIST (0xB5)", SETLOSTDIST);
        ui->comboBox_CMD->addItem("SETSTOPCYCLES (0xB6)", SETSTOPCYCLES);
        ui->comboBox_CMD->addItem("SETCORNERDIST (0xB7)", SETCORNERDIST);
        ui->comboBox_CMD->addItem("SETALIGNDIST (0xB8)", SETALIGNDIST);
        ui->comboBox_CMD->addItem("SETPWMLROT (0xB9)", SETPWMLROT);
        ui->comboBox_CMD->addItem("SETPWMRROT (0xC0)", SETPWMRROT);
        ui->comboBox_CMD->addItem("SETWALLKP (0xCB)", SETWALLKP);
        ui->comboBox_CMD->addItem("SETWALLKD (0xCC)", SETWALLKD);
        ui->comboBox_CMD->addItem("SETFRONTKP (0xCD)", SETFRONTKP);
        ui->comboBox_CMD->addItem("SETFRONTKD (0xCE)", SETFRONTKD);
        ui->comboBox_CMD->addItem("SETDODGEMODE (0xCF)", SETDODGEMODE);
        ui->comboBox_CMD->addItem("SETROBOTMODE (0xD2)", SETROBOTMODE);
        ui->comboBox_CMD->addItem("SETJOYSTICKTURN (0xD3)", SETGOTOTURN);
        ui->comboBox_CMD->addItem("SETDODGEBIASTIME (0xD4)", SETDODGEBIASTIME);
        ui->comboBox_CMD->addItem("SETDODGEBIASMULT (0xD5)", SETDODGEBIASMULT);

        connect(ui->comboBox_CMD, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &MainWindow::on_comboBox_CMD_currentIndexChanged);
        on_comboBox_CMD_currentIndexChanged(0);
    }

    if (ui->spinBox_cmdParam) {
        ui->spinBox_cmdParam->setSingleStep(1);
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::addLogEntry(const QString &data, const QString &type) {
    LogEntry entry;
    entry.time = QDateTime::currentDateTime();
    entry.data = data;
    entry.type = type;
    m_logHistory.append(entry);

    if (type == "UNKNOWN") m_countUnknown++;
    else if (type == "CHK_ERROR") m_countChecksumErrors++;

    cleanOldLogs();
}

void MainWindow::cleanOldLogs() {
    QDateTime fiveMinutesAgo = QDateTime::currentDateTime().addSecs(-300);
    while (!m_logHistory.isEmpty() && m_logHistory.first().time < fiveMinutesAgo) {
        m_logHistory.removeFirst();
    }
}

void MainWindow::on_pushButton_exportExcel_clicked() {
    QString fileName = QFileDialog::getSaveFileName(this, "Exportar Historial (Excel/CSV)", 
                                                    "log_robot_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".csv", 
                                                    "Archivos CSV (*.csv)");
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "Error", "No se pudo crear el archivo.");
        return;
    }

    QTextStream out(&file);
    int lost = (m_countSent > m_countReceived) ? (m_countSent - m_countReceived) : 0;

    out << "TIEMPO;TIPO;DATO;ESTADISTICAS\n";
    out << ";;;Enviados: " << m_countSent << "\n";
    out << ";;;Recibidos: " << m_countReceived << "\n";
    out << ";;;Perdidos: " << lost << "\n";
    out << ";;;Desconocidos: " << m_countUnknown << "\n";
    out << ";;;Errores Checksum: " << m_countChecksumErrors << "\n\n";
    
    out << "TIEMPO;TIPO;DATO\n";

    for (const auto &entry : m_logHistory) {
        QString cleanData = entry.data;
        cleanData.replace("\n", " ").replace(";", ",");
        out << entry.time.toString("hh:mm:ss.zzz") << ";" << entry.type << ";" << cleanData << "\n";
    }

    file.close();
    QMessageBox::information(this, "Exportación Exitosa", 
                            QString("Se han exportado %1 registros de los últimos 5 minutos.").arg(m_logHistory.size()));
}

void MainWindow::on_pushButton_exportTxt_clicked() {
    QString fileName = QFileDialog::getSaveFileName(this, "Exportar Historial (Texto Plano)", 
                                                    "log_robot_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".txt", 
                                                    "Archivos de Texto (*.txt)");
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "Error", "No se pudo crear el archivo.");
        return;
    }

    QTextStream out(&file);
    int lost = (m_countSent > m_countReceived) ? (m_countSent - m_countReceived) : 0;

    out << "====================================================\n";
    out << "        REPORTE DE ACTIVIDAD DEL ROBOT\n";
    out << "        Generado el: " << QDateTime::currentDateTime().toString("dd/MM/yyyy HH:mm:ss") << "\n";
    out << "====================================================\n\n";
    
    out << "ESTADISTICAS (Ultimos 5 minutos):\n";
    out << "----------------------------------------------------\n";
    out << " Comandos Enviados:    " << m_countSent << "\n";
    out << " Comandos Recibidos:   " << m_countReceived << "\n";
    out << " Comandos Perdidos:    " << lost << "\n";
    out << " Comandos Desconocidos: " << m_countUnknown << "\n";
    out << " Errores de Checksum:  " << m_countChecksumErrors << "\n";
    out << "----------------------------------------------------\n\n";

    out << "DETALLE DE LOGS:\n";
    out << "TIMESTAMP    | TIPO      | DATO\n";
    out << "-------------|-----------|--------------------------\n";

    for (const auto &entry : m_logHistory) {
        out << entry.time.toString("hh:mm:ss.zzz").leftJustified(12) << " | "
            << entry.type.leftJustified(9) << " | "
            << entry.data.simplified() << "\n";
    }

    out << "\n--- Fin del reporte ---";

    file.close();
    QMessageBox::information(this, "Exportación Exitosa", "Historial exportado correctamente a .txt");
}

void MainWindow::dataReceived(){
    unsigned char *incomingBuffer;
    int count;
    //uint8_t xId;

    count = QSerialPort1->bytesAvailable();

    if(count<=0)
        return;

    incomingBuffer = new unsigned char[count];

    QSerialPort1->read((char *)incomingBuffer,count);

    QString str="";

    for(int i=0; i<count; i++){
        if(isalnum(incomingBuffer[i]))
            str = str + QString("%1").arg((char)incomingBuffer[i]);
        else
            str = str +"{" + QString("%1").arg(incomingBuffer[i],2,16,QChar('0')) + "}";
    }

    addLogEntry("MBED-->SERIAL-->PC (" + str + ")", "RX");
    ui->textBrowserUnProcessed->append("MBED-->SERIAL-->PC (" + str + ")");

    //Cada vez que se recibe un dato reinicio el timeOut
    rxData.timeOut=6;
    m_lastRxTime = QDateTime::currentMSecsSinceEpoch();

    for(int i=0;i<count; i++){
        switch (estadoProtocolo) {
        case START:
            if (incomingBuffer[i]=='U'){
                estadoProtocolo=HEADER_1;
            }
            break;
        case HEADER_1:
            if (incomingBuffer[i]=='N')
                estadoProtocolo=HEADER_2;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case HEADER_2:
            if (incomingBuffer[i]=='E')
                estadoProtocolo=HEADER_3;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case HEADER_3:
            if (incomingBuffer[i]=='R')
                estadoProtocolo=NBYTES;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case NBYTES:
            rxData.nBytes=incomingBuffer[i];
            estadoProtocolo=TOKEN;
            break;
        case TOKEN:
            if (incomingBuffer[i]==':'){
                estadoProtocolo=PAYLOAD;
                rxData.cheksum='U'^'N'^'E'^'R'^ rxData.nBytes^':';
                rxData.payLoad[0]=rxData.nBytes;
                rxData.index=1;
            }
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case PAYLOAD:
            if (rxData.nBytes>1){
                rxData.payLoad[rxData.index++]=incomingBuffer[i];
                rxData.cheksum^=incomingBuffer[i];
            }
            rxData.nBytes--;
            if(rxData.nBytes==0){
                estadoProtocolo=START;
                if(rxData.cheksum==incomingBuffer[i]){
                    decodeData(&rxData.payLoad[0], SERIE);
                }else{
                    addLogEntry("Chk Calculado ** " +QString().number(rxData.cheksum,16) + " **", "CHK_ERROR");
                    ui->textBrowserUnProcessed->append("Chk Calculado ** " +QString().number(rxData.cheksum,16) + " **" );
                    addLogEntry("Chk recibido ** " +QString().number(incomingBuffer[i],16) + " **", "CHK_ERROR");
                    ui->textBrowserUnProcessed->append("Chk recibido ** " +QString().number(incomingBuffer[i],16) + " **" );

                }
            }
            break;
        default:
            estadoProtocolo=START;
            break;
        }
    }
    delete [] incomingBuffer;

}

static const uint16_t PC_LUT_Y_SCALE[16] = {0, 67, 133, 200, 267, 333, 400, 467, 533, 600, 667, 733, 800, 867, 933, 1000};

// (LUTs de PC para sensores superiores eliminadas para usar valores crudos directamente)

[[maybe_unused]] static uint16_t PC_LUT_Interpolate(const uint16_t *x, const uint16_t *lut_y, uint16_t raw) {
    if(raw <= x[0])
        return lut_y[0];

    if(raw >= x[15])
        return lut_y[15];

    for(int i = 0; i < 15; i++) {
        if(raw >= x[i] && raw <= x[i + 1]) {
            uint32_t diff_x = x[i + 1] - x[i];
            if (diff_x == 0) return lut_y[i];
            uint32_t diff_y = lut_y[i + 1] - lut_y[i];
            uint32_t offset_x = raw - x[i];
            uint32_t y = lut_y[i] + (((offset_x * diff_y) + (diff_x / 2)) / diff_x);
            return (uint16_t)y;
        }
    }
    return lut_y[15];
}

void MainWindow::decodeData(uint8_t *datosRx, uint8_t source){
    m_countReceived++; // Comando completo y válido recibido

    //int32_t length = sizeof(*datosRx)/sizeof(datosRx[0]);
    int32_t length = datosRx[0];
    uint8_t id = datosRx[1];
    QString str, strOut;
    _udat w;

    // --- SEMÁFORO PING-PONG ---
    // Solo liberamos el semáforo si la respuesta pertenece a un comando solicitado (MPU, ADC, etc).
    // El heartbeat ALIVE es autónomo del STM32, NO debe liberar el semáforo
    // porque no fue una respuesta a algo que nosotros pedimos.
    if (id != GETALIVE) {
        isWaitingReply = false;
    }

    for(int i = 1; i<length; i++){
        if(isalnum(datosRx[i]))
            str = str + QString("%1").arg(char(datosRx[i]));
        else
            str = str +QString("%1").arg(datosRx[i],2,16,QChar('0'));
    }
    addLogEntry("*(MBED-S->PC)->decodeData (" + str + ")", "RX");
    ui->textBrowserUnProcessed->append("*(MBED-S->PC)->decodeData (" + str + ")");

    str=QString().number(datosRx[0]);
    addLogEntry(str, "RX");
    ui->textBrowserProcessed->append(str);
    str=QString().number(datosRx[1]);
    addLogEntry(str, "RX");
    ui->textBrowserProcessed->append(str);

    switch (datosRx[1]) {
    case GETALIVE://     GETALIVE=0xF0,
        if(datosRx[2]==ACK){
            contadorAlive++;
            if(source)
                str="ALIVE BLUEPILL VIA *SERIE* RECIBIDO!!!";
            else{
                str="ALIVE BLUEPILL VIA *UDP* RECIBIDO N°: " + QString().number(contadorAlive,10);
            }
        }else{
            str= "ALIVE BLUEPILL VIA *SERIE*  NO ACK!!!";
        }
        addLogEntry(str, "RX");
        ui->textBrowserProcessed->append(str);
        break;
    case GETFIRMWARE://     GETFIRMWARE=0xF1
        str = "FIRMWARE: ";
        for(uint8_t i=0;i<(datosRx[0]-2);i++){ //datosRx[0] -> tamaño en bytes del mensaje.
            str += (QChar)datosRx[2+i];
        }
        addLogEntry(str, "RX");
        ui->textBrowserProcessed->append(str);
        break;
    case GETMPU:{

        //Datos acelerometro
        w.i8[0] = datosRx[2];
        w.i8[1] = datosRx[3];

        float ax = w.i16[0];
        str = QString("%1").arg(w.i16[0], 5, 10, QChar('0'));
        strOut = "Ax: " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->ax_data->display(str);
          ///  setText(str);

        w.i8[0] = datosRx[4];
        w.i8[1] = datosRx[5];

        float ay = w.i16[0];
        str = QString("%1").arg(w.i16[0], 5, 10, QChar('0'));
        strOut = "Ay: " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->ay_data->display(str);

        w.i8[0] = datosRx[6];
        w.i8[1] = datosRx[7];

        float az = w.i16[0];
        str = QString("%1").arg(w.i16[0], 5, 10, QChar('0'));
        strOut = "Az: " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->az_data->display(str);

        //Datos giroscopio
        w.i8[0] = datosRx[8];
        w.i8[1] = datosRx[9];

        float gx = w.i16[0];
        str = QString("%1").arg(w.i16[0], 5, 10, QChar('0'));
        strOut = "Gx: " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->gx_data->display(str);

        w.i8[0] = datosRx[10];
        w.i8[1] = datosRx[11];

        float gy = w.i16[0];
        str = QString("%1").arg(w.i16[0], 5, 10, QChar('0'));
        strOut = "Gy: " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->gy_data->display(str);

        w.i8[0] = datosRx[12];
        w.i8[1] = datosRx[13];

        float gz = w.i16[0];
        str = QString("%1").arg(w.i16[0], 5, 10, QChar('0'));
        strOut = "Gz: " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->gz_data->display(str);

        // 1. Delta-time real entre paquetes MPU
        static double lastMpuTimestamp = -1.0;
        double t = runtimeTimer.elapsed() / 1000.0;
        double dt = (lastMpuTimestamp >= 0.0) ? (t - lastMpuTimestamp) : 0.0;
        lastMpuTimestamp = t;

        // 2. Calcular ángulos
        float pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
        float roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

        // 3. Integración precisa de Yaw con el dt real medido
        float gz_grados_seg = gz / 131.0f; // Asumiendo escala de +/- 250deg/s (131 LSB/(°/s))
        if (std::abs(gz_grados_seg) > 1.2f && dt > 0.0 && dt < 0.5) {
            yawAcumulado += gz_grados_seg * static_cast<float>(dt);
        }
        m_gotoRelativeYaw = yawAcumulado - m_gotoStartYaw;
        updateGoToAngleDisplays();
        // Imprimir los ángulos finales en la consola de Qt Creator
        qDebug() << "Angulos Calculados -> Pitch:" << pitch << " | Roll:" << roll << " | Yaw:" << yawAcumulado << " | dt:" << dt;

        // ---- Enviar a la gráfica ----
        updateMPUChart(t, ax, ay, az, gx, gy, gz, pitch, roll, yawAcumulado);

        // 4. Enviar los ángulos a Qt Quick 3D
        if (ui->AutoWidget && ui->AutoWidget->rootObject()) {
            ui->AutoWidget->rootObject()->setProperty("carPitch", pitch);
            ui->AutoWidget->rootObject()->setProperty("carRoll", roll);
            ui->AutoWidget->rootObject()->setProperty("carYaw", yawAcumulado);
        }
        m_lastMpuTime = QDateTime::currentMSecsSinceEpoch();
        update3DModelConnectionState(true);

        // 5. Actualizar la Brújula de Orientación (rumbo náutico horario: Este = +90°, Oeste = 270°)
        if (m_compassWidget) {
            m_compassWidget->setHeading(-yawAcumulado);
        }
        break;
    }
    case GETADC: {
        int16_t sumLineSensors = 0;
        //Datos acelerometro
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        uint16_t ir1 = w.ui16[0];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "IR0 (Der 90°): " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->ir1_data->display(str);
        double dist1 = adcToDistanceCm(ir1);
        ui->ir1_cm_data->display(QString::number(dist1, 'f', 1));
        int pct1 = (dist1 >= 80.0) ? 100 : qBound(0, (int)((dist1 / 80.0) * 100.0), 100);
        ui->bar_ir1->setValue(pct1);
        ui->bar_ir1->setFormat((dist1 >= 80.0) ? QString("Libre (100%)") : QString("%1 cm (%2%)").arg(dist1, 0, 'f', 1).arg(pct1));

        w.ui8[0] = datosRx[4];
        w.ui8[1] = datosRx[5];
        uint16_t ir2 = w.ui16[0];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "IR1 (Línea Izq): " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->ir2_data->display(str);
        ui->bar_ir2->setValue(ir2);
        ui->bar_ir2->setFormat(QString("%1 / 4095").arg(ir2));
        sumLineSensors += qMax(0, 2400 - w.i16[0]);

        w.ui8[0] = datosRx[6];
        w.ui8[1] = datosRx[7];
        uint16_t ir3 = w.ui16[0];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "IR2 (Izq 90°): " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->ir3_data->display(str);
        double dist3 = adcToDistanceCm(ir3);
        ui->ir3_cm_data->display(QString::number(dist3, 'f', 1));
        int pct3 = (dist3 >= 80.0) ? 100 : qBound(0, (int)((dist3 / 80.0) * 100.0), 100);
        ui->bar_ir3->setValue(pct3);
        ui->bar_ir3->setFormat((dist3 >= 80.0) ? QString("Libre (100%)") : QString("%1 cm (%2%)").arg(dist3, 0, 'f', 1).arg(pct3));

        w.ui8[0] = datosRx[8];
        w.ui8[1] = datosRx[9];
        uint16_t ir4 = w.ui16[0];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "IR3 (Línea Cen): " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->ir4_data->display(str);
        ui->bar_ir4->setValue(ir4);
        ui->bar_ir4->setFormat(QString("%1 / 4095").arg(ir4));
        sumLineSensors += qMax(0, 2400 - w.i16[0]);

        w.ui8[0] = datosRx[10];
        w.ui8[1] = datosRx[11];
        uint16_t ir5 = w.ui16[0];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "IR4 (Izq 45°): " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->ir5_data->display(str);
        double dist5 = adcToDistanceCm(ir5);
        ui->ir5_cm_data->display(QString::number(dist5, 'f', 1));
        int pct5 = (dist5 >= 80.0) ? 100 : qBound(0, (int)((dist5 / 80.0) * 100.0), 100);
        ui->bar_ir5->setValue(pct5);
        ui->bar_ir5->setFormat((dist5 >= 80.0) ? QString("Libre (100%)") : QString("%1 cm (%2%)").arg(dist5, 0, 'f', 1).arg(pct5));

        w.ui8[0] = datosRx[12];
        w.ui8[1] = datosRx[13];
        uint16_t ir6 = w.ui16[0];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "IR5 (Línea Der): " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->ir6_data->display(str);
        ui->bar_ir6->setValue(ir6);
        ui->bar_ir6->setFormat(QString("%1 / 4095").arg(ir6));
        sumLineSensors += qMax(0, 2400 - w.i16[0]);
        str = QString("%1").arg(sumLineSensors, 5, 10, QChar('0'));


        w.ui8[0] = datosRx[14];
        w.ui8[1] = datosRx[15];
        uint16_t ir7 = w.ui16[0];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "IR6 (Frontal): " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->ir7_data->display(str);
        double dist7 = adcToDistanceCm(ir7);
        ui->ir7_cm_data->display(QString::number(dist7, 'f', 1));
        int pct7 = (dist7 >= 80.0) ? 100 : qBound(0, (int)((dist7 / 80.0) * 100.0), 100);
        ui->bar_ir7->setValue(pct7);
        ui->bar_ir7->setFormat((dist7 >= 80.0) ? QString("Libre (100%)") : QString("%1 cm (%2%)").arg(dist7, 0, 'f', 1).arg(pct7));

        w.ui8[0] = datosRx[16];
        w.ui8[1] = datosRx[17];
        uint16_t ir8 = w.ui16[0];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "IR7 (Der 45°): " + str;
        addLogEntry(strOut, "RX");
        ui->textBrowserProcessed->append(strOut);
        ui->ir8_data->display(str);
        double dist8 = adcToDistanceCm(ir8);
        ui->ir8_cm_data->display(QString::number(dist8, 'f', 1));
        int pct8 = (dist8 >= 80.0) ? 100 : qBound(0, (int)((dist8 / 80.0) * 100.0), 100);
        ui->bar_ir8->setValue(pct8);
        ui->bar_ir8->setFormat((dist8 >= 80.0) ? QString("Libre (100%)") : QString("%1 cm (%2%)").arg(dist8, 0, 'f', 1).arg(pct8));

        // ---- NUEVO: Enviar a la gráfica ----
        double t = runtimeTimer.elapsed() / 1000.0;
        updateIRChart(t, ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8);

        // --- Acumular muestra en buffer circular de IR superiores (último minuto) ---
        UpperIrSample upperSample;
        upperSample.timestamp = QDateTime::currentDateTime();
        upperSample.ir1 = ir1;
        upperSample.ir3 = ir3;
        upperSample.ir5 = ir5;
        upperSample.ir7 = ir7;
        upperSample.ir8 = ir8;
        m_upperIrBuffer.append(upperSample);
        if (m_upperIrBuffer.size() > IR_BUFFER_SIZE)
            m_upperIrBuffer.removeFirst();



        break;
    }
    case GETINTERNALDATA: {
        w.ui8[0] = datosRx[58]; w.ui8[1] = datosRx[59]; w.ui8[2] = datosRx[60]; w.ui8[3] = datosRx[61];
        float hr_angle = w.i32 / 10000.0f;
        ui->angle_hr_data->display(QString::number(hr_angle, 'f', 4));

        // Unpack raw IR values (IR1=Izq, IR3=Cen, IR5=Der)
        w.ui8[0] = datosRx[68]; w.ui8[1] = datosRx[69];
        uint16_t rawIr1 = w.ui16[0]; // Izquierda (IR1)

        w.ui8[0] = datosRx[70]; w.ui8[1] = datosRx[71];
        uint16_t rawIr3 = w.ui16[0]; // Centro (IR3)

        w.ui8[0] = datosRx[72]; w.ui8[1] = datosRx[73];
        uint16_t rawIr5 = w.ui16[0]; // Derecha (IR5)


        // Unpack calibrated IR values (IR1=Izq, IR3=Cen, IR5=Der)
        w.ui8[0] = datosRx[74]; w.ui8[1] = datosRx[75];
        uint16_t calIr1 = w.ui16[0]; // Izquierda (IR1)
        m_calIr1 = calIr1;

        w.ui8[0] = datosRx[76]; w.ui8[1] = datosRx[77];
        uint16_t calIr3 = w.ui16[0]; // Centro (IR3)
        m_calIr3 = calIr3;

        w.ui8[0] = datosRx[78]; w.ui8[1] = datosRx[79];
        uint16_t calIr5 = w.ui16[0]; // Derecha (IR5)
        m_calIr5 = calIr5;

        // --- Modo del robot en tiempo real ---
        uint8_t currentRobotMode = datosRx[87];
        updateRobotModeUI(currentRobotMode);

        // --- Acumular muestra en buffer circular IR (último minuto) ---
        IrSample sample;
        sample.timestamp = QDateTime::currentDateTime();
        sample.ir1 = rawIr1; // Guardar el valor sin procesar (raw)
        sample.ir3 = rawIr3; // Guardar el valor sin procesar (raw)
        sample.ir5 = rawIr5; // Guardar el valor sin procesar (raw)
        m_irBuffer.append(sample);
        if (m_irBuffer.size() > IR_BUFFER_SIZE)
            m_irBuffer.removeFirst();

        if (!paramsSynced) {
            // 1. PID Balancín (indices 2 a 11)
            w.ui8[0] = datosRx[2];  w.ui8[1] = datosRx[3];  ui->setBalanceKp->setValue(w.i16[0]);
            w.ui8[0] = datosRx[4];  w.ui8[1] = datosRx[5];  ui->setBalanceKi->setValue(w.i16[0]);
            w.ui8[0] = datosRx[6];  w.ui8[1] = datosRx[7];  ui->setBalanceKd->setValue(w.i16[0]);
            w.ui8[0] = datosRx[8];  w.ui8[1] = datosRx[9];  ui->setPWMMINR->setValue(w.ui16[0]);
            w.ui8[0] = datosRx[10]; w.ui8[1] = datosRx[11]; ui->setPWMMINL->setValue(w.ui16[0]);

            // 2. Setpoint (indices 12 a 15)
            w.ui8[0] = datosRx[12]; w.ui8[1] = datosRx[13];
            w.ui8[2] = datosRx[14]; w.ui8[3] = datosRx[15];
            ui->setSetpoint->setValue(w.i32);
            m_currentSetpoint = w.i32;
            updateGoToAngleDisplays();

            // 3. Extra (indices 16 a 29)
            w.ui8[0] = datosRx[16]; w.ui8[1] = datosRx[17]; ui->setLineKp->setValue(w.i16[0]);
            w.ui8[0] = datosRx[18]; w.ui8[1] = datosRx[19]; ui->setLineKd->setValue(w.i16[0]);
            w.ui8[0] = datosRx[20]; w.ui8[1] = datosRx[21]; ui->setOFFSETL->setValue(w.i16[0]);
            w.ui8[0] = datosRx[22]; w.ui8[1] = datosRx[23]; ui->setOFFSETR->setValue(w.i16[0]);
            w.ui8[0] = datosRx[24]; w.ui8[1] = datosRx[25]; ui->setCustomTurn->setValue(w.i16[0]);
            w.ui8[0] = datosRx[26]; w.ui8[1] = datosRx[27];
            ui->setAttackSetpoint->blockSignals(true);
            ui->setAttackSetpoint->setValue(w.i16[0]);
            ui->setAttackSetpoint->blockSignals(false);
            w.ui8[0] = datosRx[28]; w.ui8[1] = datosRx[29]; ui->setWallKp->setValue(w.i16[0]);

            // 4. Esquivador (indices 30 a 39)
            w.ui8[0] = datosRx[30]; w.ui8[1] = datosRx[31]; ui->setFrontDistance->setValue(w.ui16[0]);
            w.ui8[0] = datosRx[32]; w.ui8[1] = datosRx[33]; ui->setSideDistance->setValue(w.ui16[0]);
            w.ui8[0] = datosRx[34]; w.ui8[1] = datosRx[35]; ui->setLostDistance->setValue(w.ui16[0]);
            w.ui8[0] = datosRx[36]; w.ui8[1] = datosRx[37]; ui->setStopCycles->setValue(w.ui16[0]);
            w.ui8[0] = datosRx[38]; w.ui8[1] = datosRx[39]; ui->setCornerDist->setValue(w.ui16[0]);
            w.ui8[0] = datosRx[40]; w.ui8[1] = datosRx[41]; ui->setAlignDist->setValue(w.ui16[0]);
            
            // 5. Rotación (los controles de UI fueron reemplazados por la columna 5 de Sesgo de Enganche)
            // w.ui8[0] = datosRx[42]; w.ui8[1] = datosRx[43]; ui->setPWMLROT->setValue(w.ui16[0]);
            // w.ui8[0] = datosRx[44]; w.ui8[1] = datosRx[45]; ui->setPWMRROT->setValue(w.ui16[0]);
            // w.ui8[0] = datosRx[46]; w.ui8[1] = datosRx[47]; ui->setStaticOff->setValue(w.ui16[0]);
            // w.ui8[0] = datosRx[48]; w.ui8[1] = datosRx[49]; ui->setMovingOff->setValue(w.ui16[0]);
            // w.ui8[0] = datosRx[50]; w.ui8[1] = datosRx[51]; ui->setLimitAngle->setValue(w.i16[0]);
            w.ui8[0] = datosRx[52]; w.ui8[1] = datosRx[53]; ui->setKpCascada->setValue(w.i16[0]);
            w.ui8[0] = datosRx[54]; w.ui8[1] = datosRx[55]; ui->setKiCascada->setValue(w.i16[0]);
            w.ui8[0] = datosRx[56]; w.ui8[1] = datosRx[57]; ui->setFilterLPF->setValue(w.i16[0]);
            w.ui8[0] = datosRx[62]; w.ui8[1] = datosRx[63]; ui->setVelDampDiv->setValue(w.i16[0]);
            w.ui8[0] = datosRx[64]; w.ui8[1] = datosRx[65]; ui->setVelDampLim->setValue(w.i16[0]);
            w.ui8[0] = datosRx[66]; w.ui8[1] = datosRx[67]; ui->setTurnLimit->setValue(w.i16[0]);
            w.ui8[0] = datosRx[80]; w.ui8[1] = datosRx[81]; ui->setWallKd->setValue(w.i16[0]);
            w.ui8[0] = datosRx[82]; w.ui8[1] = datosRx[83]; ui->setFrontKp->setValue(w.i16[0]);
            w.ui8[0] = datosRx[84]; w.ui8[1] = datosRx[85]; ui->setFrontKd->setValue(w.i16[0]);
            ui->comboDodgeDir->setCurrentIndex(datosRx[86]);
            w.ui8[0] = datosRx[89]; w.ui8[1] = datosRx[90]; ui->setDodgeBiasTime->setValue(w.i16[0]);
            w.ui8[0] = datosRx[91]; w.ui8[1] = datosRx[92]; ui->setDodgeBiasMult->setValue(w.i16[0]);

            paramsSynced = true;
            addLogEntry("***PARÁMETROS SINCRONIZADOS DESDE STM32***", "RX");
            ui->textBrowserProcessed->append("***PARÁMETROS SINCRONIZADOS DESDE STM32***");
        }
        break;
    }
    case GETPIDBALANCE: {
        // 1. Extraer Error (arranca en datosRx[2])
        w.ui8[0] = datosRx[2]; w.ui8[1] = datosRx[3]; w.ui8[2] = datosRx[4]; w.ui8[3] = datosRx[5];
        int32_t current_error = w.i32;

        // 2. Extraer Integral
        w.ui8[0] = datosRx[6]; w.ui8[1] = datosRx[7]; w.ui8[2] = datosRx[8]; w.ui8[3] = datosRx[9];
        int32_t current_integral = w.i32;

        // 3. Extraer Derivada
        w.ui8[0] = datosRx[10]; w.ui8[1] = datosRx[11]; w.ui8[2] = datosRx[12]; w.ui8[3] = datosRx[13];
        int32_t current_derivative = w.i32;

        // 4. Extraer Output
        w.ui8[0] = datosRx[14]; w.ui8[1] = datosRx[15]; w.ui8[2] = datosRx[16]; w.ui8[3] = datosRx[17];
        int32_t current_output = w.i32;

        // 5. Extraer Angulo Real del STM32 (Nuevo)
        w.ui8[0] = datosRx[18]; w.ui8[1] = datosRx[19]; w.ui8[2] = datosRx[20]; w.ui8[3] = datosRx[21];
        float stm_angle = w.i32 / 100.0f; // Escala x100 -> real
        ui->angle_data->display(QString::number(stm_angle, 'f', 2));
        m_currentAngle = stm_angle;
        updateGoToAngleDisplays();

        // 5b. Extraer Turn Offset del STM32 (Nuevo)
        w.ui8[0] = datosRx[22]; w.ui8[1] = datosRx[23]; w.ui8[2] = datosRx[24]; w.ui8[3] = datosRx[25];
        int32_t current_turn_offset = w.i32;
        ui->turn_offset_data->display(current_turn_offset);

        // 5c. Extraer Delta-time Medido en milisegundos (Nuevo)
        w.ui8[0] = datosRx[26]; w.ui8[1] = datosRx[27]; w.ui8[2] = datosRx[28]; w.ui8[3] = datosRx[29];
        int32_t current_dt_ms = w.i32;
        ui->dt_ms_data->display(current_dt_ms);

        // 6. Leer constantes actuales desde la Interfaz de Qt
        // (Como paramsSynced se encarga de poblarlas, siempre tendremos el valor real aquí)
        int16_t kp = ui->setBalanceKp->value();
        int16_t ki = ui->setBalanceKi->value();
        int16_t kd = ui->setBalanceKd->value();

        // 6. Calcular los Términos Individuales (Misma matemática que el micro)
        double term_P = (kp * current_error) / 1000.0;
        double term_I = (ki * current_integral) / 1000.0;
        double term_D = (kd * current_derivative) / 1000.0;
        double term_Out = current_output;

        // Calcular los Términos de Seguimiento de Línea
        int32_t sum = m_calIr1 + m_calIr3 + m_calIr5;
        if (sum == 0) sum = 1;
        int32_t error_linea = ((-(1000 * (int32_t)m_calIr1) + (1000 * (int32_t)m_calIr5)) / sum) / 10;
        int32_t abs_error = (error_linea > 0) ? error_linea : -error_linea;

        int16_t kp_line = ui->setLineKp->value();
        int16_t kq_line = ui->setLineKd->value();

        double term_P_line = kp_line * error_linea;
        double term_D_line = (kq_line * error_linea * abs_error) / 1000.0;
        double term_Out_line = current_turn_offset;

        // 7. Enviar a la gráfica embebida en MainWindow
        double t = runtimeTimer.elapsed() / 1000.0;
        updatePIDChart(t, term_P, term_I, term_D, term_Out, term_P_line, term_D_line, term_Out_line);

        // Opcional: Imprimir en consola para depurar
        // qDebug() << "P:" << term_P << "I:" << term_I << "D:" << term_D << "Out:" << term_Out;
        break;
    }
    case EXPORTIRCSV:
        if (datosRx[2] == ACK) {
            // Leer número de exportación devuelto por el STM32 (big-endian)
            m_irExportCount = ((int)datosRx[3] << 8) | datosRx[4];
            exportIrCsvToFile();
        }
        break;
    case SETPWML:
    case SETPWMR:
    case SETPWMMINR:
    case SETPWMMINL:
    case SET_KI_EXT:
    case SET_ALFA_LPF:
    case SETBALANCEKP:
    case SETBALANCEKD:
    case SETBALANCEKI:
    case SETSETPOINT:
    case SETLINEKP:
    case SETLINEKD:
    case SETOFFSETL:
    case SETOFFSETR:
    case SETCUSTOMTURN:
    case SETSPEED:
    case SETBKANG:
    case SETFRONTDIST:
    case SETSIDEDIST:
    case SETLOSTDIST:
    case SETSTOPCYCLES:
    case SETCORNERDIST:
    case SETALIGNDIST:
    case SETPWMLROT:
    case SETPWMRROT:
    case SETSTATICOFF:
    case SETMOVINGOFF:
    case SET_KP_EXT:
    case SETLIMITANG:
    case SETVELDAMPDIV:
    case SETVELDAMPLIM:
    case SETTURNLIMIT:
    case SETWALLKP:
    case SETWALLKD:
    case SETFRONTKP:
    case SETFRONTKD:
    case SETDODGEMODE:
    case SETDODGEBIASTIME:
    case SETDODGEBIASMULT:
        if(datosRx[2]==ACK){
            str="COMANDO ACEPTADO Y GUARDADO (ACK)!!!";
            addLogEntry(str, "RX");
            ui->textBrowserProcessed->append(str);
        }
        break;

    case SETSOFTAP:
        ui->pushButton_setSoftAp->setText("MODO SOFTAP");
        ui->pushButton_setSoftAp->setEnabled(true);
        if(datosRx[2]==ACK){
            m_isSoftApMode = true;
            str="COMANDO ACEPTADO: Robot cambiando a Modo SoftAP (SSID: MICRO)";
            addLogEntry(str, "RX");
            ui->textBrowserProcessed->append(str);
            statusMode->setText("ESTADO --> ROBOT EN MODO SOFTAP");
            statusMode->setStyleSheet("color: #e14eca; font-weight: bold; font-size: 11px; padding-left: 5px;");
            QMessageBox::information(this, "Modo SoftAP Activado",
                                     "El robot aceptó el comando y está reiniciando su módulo Wi-Fi en Modo SoftAP.\n\n"
                                     "• Red Wi-Fi: MICRO\n"
                                     "• Contraseña: 12345678\n"
                                     "• Servidor TCP: 192.168.4.1 : 80\n\n"
                                     "Ya puedes conectarte a la red 'MICRO' desde tu dispositivo o Hercules para enviar nuevas credenciales.");
        } else {
            str="ERROR: Comando SETSOFTAP no aceptado por el robot";
            addLogEntry(str, "CHK_ERROR");
            ui->textBrowserProcessed->append(str);
            QMessageBox::warning(this, "Error de Comando",
                                 "El robot no pudo procesar la orden de cambio a Modo SoftAP.");
        }
        break;

    case SETROBOTMODE: {
        if (datosRx[2] == ACK) {
            uint8_t currentMode = datosRx[3];
            updateRobotModeUI(currentMode);
            QString modeStr;
            switch(currentMode) {
            case 1: modeStr = "BALANCE"; break;
            case 2: modeStr = "SEGUIR LINEA"; break;
            case 3: modeStr = "ESQUIVAR"; break;
            case 4: modeStr = "JOYSTICK"; break;
            case 5: modeStr = "3D SCREEN"; break;
            default: modeStr = QString::number(currentMode); break;
            }
            str = QString("***MODO DEL ROBOT CONFIRMADO (ACK): %1***").arg(modeStr);
            addLogEntry(str, "RX");
            ui->textBrowserProcessed->append(str);
        }
        break;
    }

    case SETGOTOTURN: {
        if (datosRx[2] == ACK) {
            str = "***GIRO JOYSTICK CONFIRMADO (ACK)***";
            addLogEntry(str, "RX");
            ui->textBrowserProcessed->append(str);
        }
        break;
    }

    default:
        str = str + "Comando DESCONOCIDO!!!!";
        addLogEntry(str, "UNKNOWN");
        ui->textBrowserProcessed->append(str);
    }
}

void MainWindow::sendSerial(uint8_t *buf, uint8_t length){
    uint8_t tx[24];
    uint8_t cks, i;
    QString strHex;
    _udat w;

    if(!QSerialPort1->isOpen())
        return;

    w.i32 = -1000;

    tx[7] = w.ui8[0];
    tx[8] = w.ui8[1];
    tx[9] = w.ui8[2];
    tx[10] = w.ui8[3];


    tx[0] = 'U';
    tx[1] = 'N';
    tx[2] = 'E';
    tx[3] = 'R';
    tx[4] = length + 1;
    tx[5] = ':';

    memcpy(&tx[6], buf, length);

    cks = 0;
    for (i=0; i<(length+6); i++) {
        cks ^= tx[i];
    }

    tx[i] = cks;

    strHex = "--> 0x";
    for (int i=0; i<length+7; i++) {
        strHex = strHex + QString("%1").arg(tx[i], 2, 16, QChar('0')).toUpper();
    }

    addLogEntry(strHex, "TX");
    ui->textBrowserUnProcessed->append(strHex);

    QSerialPort1->write((char *)tx, length+7);
}

void MainWindow::sendUdp(uint8_t *buf, uint8_t length){
    uint8_t tx[256];
    _udat w;
    unsigned char indice=0, cks=0;


    QString str;
    int puerto=0;
    // bool ok;

    if(!QUdpSocket1->isOpen())
        return;


    w.i32 = -1000;

    tx[7] = w.ui8[0];
    tx[8] = w.ui8[1];
    tx[9] = w.ui8[2];
    tx[10] = w.ui8[3];


    tx[0] = 'U';
    tx[1] = 'N';
    tx[2] = 'E';
    tx[3] = 'R';
    tx[4] = length + 1;
    tx[5] = ':';

    memcpy(&tx[6], buf, length);

    cks = 0;
    for (indice=0; indice<(length+6); indice++) {
        cks ^= tx[indice];
    }

    tx[indice] = cks;

    str = "--> 0x";
    for (int i=0; i<length+7; i++) {
        str = str + QString("%1").arg(tx[i], 2, 16, QChar('0')).toUpper();
    }

    puerto=ui->lineEdit_device_port->text().toInt();
    puertoremoto=puerto;

    clientAddress.setAddress(ui->lineEdit_device_ip->text());
    if(puertoremoto==0)
        puertoremoto=puerto;
    if(QUdpSocket1->isOpen()){
        //QUdpSocket1->writeDatagram(reinterpret_cast<const char *>(tx), (tx[4]+7), clientAddress, puertoremoto);
        QUdpSocket1->writeDatagram(reinterpret_cast<const char *>(tx), (tx[4]+6), clientAddress, puertoremoto);
    }

    for(int i=0; i<=indice; i++){
        if(isalnum(tx[i]))
            str = str + QString("%1").arg(char(tx[i]));
        else
            str = str +"{" + QString("%1").arg(tx[i],2,16,QChar('0')) + "}";
    }
    str=str + clientAddress.toString() + "  " +  QString().number(puertoremoto,10);

    addLogEntry("PC--UDP/TCP-->MBED ( " + str + " )", "TX");
    ui->textBrowserUnProcessed->append("PC--UDP/TCP-->MBED ( " + str + " )");
}

void MainWindow::sendTcp(uint8_t *txData, uint8_t length) {
    if (!QTcpSocketClient || !QTcpSocketClient->isOpen()) return;

    uint8_t tx[100];
    uint8_t indice = 0;
    uint8_t chk = 0;
    QString str = "";

    tx[0] = 'U';
    tx[1] = 'N';
    tx[2] = 'E';
    tx[3] = 'R';
    tx[4] = length + 1;
    tx[5] = ':';

    memcpy(&tx[6], txData, length);

    chk = 0;
    for (indice=0; indice<(length+6); indice++) {
        chk ^= tx[indice];
    }

    tx[indice] = chk;

    QTcpSocketClient->write(reinterpret_cast<const char *>(tx), (length + 7));
    QTcpSocketClient->flush();

    str = "--> 0x";
    for (int i = 0; i < length + 7; i++) {
        str = str + QString("%1").arg(tx[i], 2, 16, QChar('0')).toUpper();
    }

    addLogEntry("PC--TCP-->MBED ( " + str + " )", "TX");
    ui->textBrowserUnProcessed->append("PC--TCP-->MBED ( " + str + " )");
}

void MainWindow::sendCommand(uint8_t *buf, uint8_t length) {
    bool sent = false;

    if (m_isTcpMode) {
        if (QTcpSocketClient && QTcpSocketClient->isOpen()) {
            sendTcp(buf, length);
            sent = true;
            m_countSent++;
        }
    } else {
        if (QUdpSocket1->isOpen()) {
            sendUdp(buf, length);
            sent = true;
            m_countSent++;
        }
    }

    if (!sent && QSerialPort1->isOpen()) {
        sendSerial(buf, length);
        sent = true;
        m_countSent++;
    }

    if (!sent) {
        addLogEntry("***ERROR: NINGUNA CONEXIÓN ABIERTA***", "CHK_ERROR");
        ui->textBrowserProcessed->append("***ERROR: NINGUNA CONEXIÓN ABIERTA***");
    }
}



void MainWindow::timeOut(){
    if(rxData.timeOut){
        rxData.timeOut--;
        if(!rxData.timeOut){
            estadoProtocolo=START;
        }
    }
    //getData();
}

void MainWindow::OnUdpRxData(){
    qint64          count = 0;
    unsigned char   *incomingBuffer = NULL;

    // EL BUCLE PROCESA CADA DATAGRAMA PENDIENTE EN LA COLA
    while(QUdpSocket1->hasPendingDatagrams()){
        count = QUdpSocket1->pendingDatagramSize();
        if (count <= 0) continue;

        incomingBuffer = new unsigned char[count];
        QUdpSocket1->readDatagram(reinterpret_cast<char *>(incomingBuffer), count, &RemoteAddress, &RemotePort);
        m_lastRxTime = QDateTime::currentMSecsSinceEpoch();

        // Debug visual de los datos crudos en la interfaz
        QString str = "";
        for(int i = 0; i < count; i++){
            if(isalnum(incomingBuffer[i]))
                str = str + QString("%1").arg(char(incomingBuffer[i]));
            else
                str = str + "{" + QString("%1").arg(incomingBuffer[i], 2, 16, QChar('0')) + "}";
        }
        addLogEntry("MBED-->UDP-->PC (" + str + ")", "RX");
        ui->textBrowserUnProcessed->append("MBED-->UDP-->PC (" + str + ")");
        addLogEntry(" adr " + RemoteAddress.toString(), "RX");
        ui->textBrowserUnProcessed->append(" adr " + RemoteAddress.toString());

        // Actualizar la IP y el puerto detectado del robot
        QHostAddress cleanAddr(RemoteAddress.toIPv4Address());
        ui->lineEdit_device_ip->setText(cleanAddr.toString());
        ui->lineEdit_device_port->setText(QString().number(RemotePort, 10));
        if (cleanAddr.toString().startsWith("192.168.4.")) {
            m_isSoftApMode = true;
        } else if (!cleanAddr.isNull() && cleanAddr.toString() != "0.0.0.0") {
            m_isSoftApMode = false;
        }

        // --- RESET de la MeF al inicio de cada datagrama ---
        // Cada AT+CIPSEND del ESP01 genera exactamente un datagrama UDP completo.
        // Empezar en START por cada datagrama evita que un paquete corrupto
        // arrastre estado residual al siguiente datagrama válido.
        estadoProtocoloUdp = START;

        // MÁQUINA DE ESTADOS DEL PROTOCOLO UNER
        for(int i = 0; i < count; i++){
            switch (estadoProtocoloUdp) {
            case START:
                if (incomingBuffer[i] == 'U'){
                    estadoProtocoloUdp = HEADER_1;
                    rxDataUdp.cheksum = 0;
                }
                break;
            case HEADER_1:
                if (incomingBuffer[i] == 'N')
                    estadoProtocoloUdp = HEADER_2;
                else {
                    i--;
                    estadoProtocoloUdp = START;
                }
                break;
            case HEADER_2:
                if (incomingBuffer[i] == 'E')
                    estadoProtocoloUdp = HEADER_3;
                else {
                    i--;
                    estadoProtocoloUdp = START;
                }
                break;
            case HEADER_3:
                if (incomingBuffer[i] == 'R')
                    estadoProtocoloUdp = NBYTES;
                else {
                    i--;
                    estadoProtocoloUdp = START;
                }
                break;
            case NBYTES:
                rxDataUdp.nBytes = incomingBuffer[i];
                estadoProtocoloUdp = TOKEN;
                break;
            case TOKEN:
                if (incomingBuffer[i] == ':'){
                    estadoProtocoloUdp = PAYLOAD;
                    rxDataUdp.cheksum = 'U' ^ 'N' ^ 'E' ^ 'R' ^ rxDataUdp.nBytes ^ ':';
                    rxDataUdp.payLoad[0] = rxDataUdp.nBytes;
                    rxDataUdp.index = 1;
                }
                else {
                    i--;
                    estadoProtocoloUdp = START;
                }
                break;
            case PAYLOAD:
                if (rxDataUdp.nBytes > 1){
                    rxDataUdp.payLoad[rxDataUdp.index++] = incomingBuffer[i];
                    rxDataUdp.cheksum ^= incomingBuffer[i];
                }
                rxDataUdp.nBytes--;
                if(rxDataUdp.nBytes == 0){
                    estadoProtocoloUdp = START;

                    // Verificación de Checksum
                    if(rxDataUdp.cheksum == incomingBuffer[i]){
                        decodeData(&rxDataUdp.payLoad[0], UDP);
                    } else {
                        addLogEntry(" CHK DISTINTO!!!!! ", "CHK_ERROR");
                        ui->textBrowserProcessed->append(" CHK DISTINTO!!!!! ");
                        // --- LIBERAR SEMÁFORO en error de CHK ---
                        // Sin esto, getData() queda bloqueado ~600ms esperando
                        // el timeoutPatience, paralizando toda la telemetría.
                        isWaitingReply = false;
                    }
                }
                break;

            default:
                estadoProtocoloUdp = START;
                break;
            }
        }

        // IMPORTANTE: Liberar la memoria del buffer actual antes de leer el siguiente paquete
        delete [] incomingBuffer;
    }
}

bool MainWindow::isSoftApActive() const {
    if (m_isSoftApMode) return true;
    if (ui->lineEdit_device_ip && ui->lineEdit_device_ip->text().trimmed().startsWith("192.168.4.")) return true;
    if (RemoteAddress.toString().contains("192.168.4.")) return true;
    if (QTcpSocketClient && QTcpSocketClient->peerAddress().toString().contains("192.168.4.")) return true;
    return false;
}

void MainWindow::OnTcpNewConnection() {
    if(QTcpSocketClient) {
        QTcpSocketClient->disconnect();
        QTcpSocketClient->close();
        QTcpSocketClient->deleteLater();
        QTcpSocketClient = nullptr;
    }
    QTcpSocketClient = QTcpServer1->nextPendingConnection();
    if(QTcpSocketClient) {
        connect(QTcpSocketClient, &QTcpSocket::readyRead, this, &MainWindow::OnTcpRxData);
        connect(QTcpSocketClient, &QTcpSocket::disconnected, this, &MainWindow::OnTcpDisconnected);

        QHostAddress cleanAddr(QTcpSocketClient->peerAddress().toIPv4Address());
        ui->lineEdit_device_ip->setText(cleanAddr.toString());
        if (cleanAddr.toString().startsWith("192.168.4.")) {
            m_isSoftApMode = true;
        } else if (!cleanAddr.isNull() && cleanAddr.toString() != "0.0.0.0") {
            m_isSoftApMode = false;
        }

        addLogEntry("CLIENT CONNECTED VIA TCP (" + cleanAddr.toString() + ")", "RX");
        ui->textBrowserUnProcessed->append("CLIENT CONNECTED VIA TCP (" + cleanAddr.toString() + ")");

        ui->pushButton_connectUdp->setText("DISCONNECT");
        ui->pushButton_protocol_wifi->setEnabled(false);
        statusMode->setText(isSoftApActive() ? "ESTADO --> CONECTADO SOFTAP" : "ESTADO --> CONECTADO STATION TCP");
        statusMode->setStyleSheet("color: #00f2c3; font-weight: bold; font-size: 11px; padding-left: 5px;");
        m_lastRxTime = QDateTime::currentMSecsSinceEpoch();
        paramsSynced = false;
        uint8_t b = GETINTERNALDATA;
        sendCommand(&b, 1);
    }
}

void MainWindow::OnTcpDisconnected() {
    addLogEntry("CLIENT DISCONNECTED FROM TCP", "RX");
    ui->textBrowserUnProcessed->append("CLIENT DISCONNECTED FROM TCP");
    if (m_isTcpMode && QTcpServer1 && QTcpServer1->isListening()) {
        statusMode->setText("ESTADO --> ESCUCHANDO TCP...");
        statusMode->setStyleSheet("color: #ffd600; font-weight: bold; font-size: 11px; padding-left: 5px;");
    } else if(QUdpSocket1->isOpen()) {
        statusMode->setText("ESTADO --> CONECTANDO...");
        statusMode->setStyleSheet("color: #ffd600; font-weight: bold; font-size: 11px; padding-left: 5px;");
    } else {
        statusMode->setText("ESTADO --> DESCONECTADO");
        statusMode->setStyleSheet("color: #fd5d93; font-weight: bold; font-size: 11px; padding-left: 5px;");
    }
    if(QTcpSocketClient) {
        QTcpSocketClient->disconnect();
        QTcpSocketClient->deleteLater();
        QTcpSocketClient = nullptr;
    }
    update3DModelConnectionState(false);
}

void MainWindow::OnTcpRxData() {
    if(!QTcpSocketClient) return;

    QByteArray incomingData = QTcpSocketClient->readAll();
    int count = incomingData.size();
    if(count <= 0) return;
    m_lastRxTime = QDateTime::currentMSecsSinceEpoch();

    const unsigned char *incomingBuffer = reinterpret_cast<const unsigned char *>(incomingData.constData());

    QString str = "";
    for(int i = 0; i < count; i++){
        if(isalnum(incomingBuffer[i]))
            str = str + QString("%1").arg(char(incomingBuffer[i]));
        else
            str = str + "{" + QString("%1").arg(incomingBuffer[i], 2, 16, QChar('0')) + "}";
    }
    addLogEntry("MBED-->TCP-->PC (" + str + ")", "RX");
    ui->textBrowserUnProcessed->append("MBED-->TCP-->PC (" + str + ")");

    for(int i = 0; i < count; i++){
        switch (estadoProtocoloTcp) {
        case START:
            if (incomingBuffer[i] == 'U'){
                estadoProtocoloTcp = HEADER_1;
                rxDataTcp.cheksum = 0;
            }
            break;
        case HEADER_1:
            if (incomingBuffer[i] == 'N')
                estadoProtocoloTcp = HEADER_2;
            else {
                i--;
                estadoProtocoloTcp = START;
            }
            break;
        case HEADER_2:
            if (incomingBuffer[i] == 'E')
                estadoProtocoloTcp = HEADER_3;
            else {
                i--;
                estadoProtocoloTcp = START;
            }
            break;
        case HEADER_3:
            if (incomingBuffer[i] == 'R')
                estadoProtocoloTcp = NBYTES;
            else {
                i--;
                estadoProtocoloTcp = START;
            }
            break;
        case NBYTES:
            rxDataTcp.nBytes = incomingBuffer[i];
            estadoProtocoloTcp = TOKEN;
            break;
        case TOKEN:
            if (incomingBuffer[i] == ':'){
                estadoProtocoloTcp = PAYLOAD;
                rxDataTcp.cheksum = 'U' ^ 'N' ^ 'E' ^ 'R' ^ rxDataTcp.nBytes ^ ':';
                rxDataTcp.payLoad[0] = rxDataTcp.nBytes;
                rxDataTcp.index = 1;
            }
            else {
                i--;
                estadoProtocoloTcp = START;
            }
            break;
        case PAYLOAD:
            if (rxDataTcp.nBytes > 1){
                rxDataTcp.payLoad[rxDataTcp.index++] = incomingBuffer[i];
                rxDataTcp.cheksum ^= incomingBuffer[i];
            }
            rxDataTcp.nBytes--;
            if(rxDataTcp.nBytes == 0){
                estadoProtocoloTcp = START;
                if(rxDataTcp.cheksum == incomingBuffer[i]){
                    decodeData(&rxDataTcp.payLoad[0], UDP);
                } else {
                    addLogEntry(" CHK TCP DISTINTO!!!!! ", "CHK_ERROR");
                    ui->textBrowserProcessed->append(" CHK TCP DISTINTO!!!!! ");
                    isWaitingReply = false;
                }
            }
            break;
        default:
            estadoProtocoloTcp = START;
            break;
        }
    }
}

void MainWindow::getData(){
    // --- 1. GUARDIA DE SEGURIDAD (Frena el spam) ---
    bool isTcpActive = (m_isTcpMode && QTcpSocketClient && QTcpSocketClient->isOpen() && QTcpSocketClient->state() == QAbstractSocket::ConnectedState);
    bool isSerialActive = (QSerialPort1 && QSerialPort1->isOpen());
    bool isUdpActive = (!m_isTcpMode && QUdpSocket1 && QUdpSocket1->isOpen());

    if(!isSerialActive && !isUdpActive && !isTcpActive) {
        statusMode->setText("ESTADO --> DESCONECTADO");
        statusMode->setStyleSheet("color: #fd5d93; font-weight: bold; font-size: 11px; padding-left: 5px;");
        update3DModelConnectionState(false);
        return;
    }

    // --- SISTEMA DE ADAPTACION A REDES LENTAS (PING-PONG) ---
    if (isWaitingReply) {
        timeoutPatience++;
        if (timeoutPatience >= 8) {
            isWaitingReply = false;
        } else {
            return;
        }
    }

    isWaitingReply = true;
    timeoutPatience = 0;

    // --- 2. ACTUALIZAR ESTADO VISUAL ---
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    if(isTcpActive) {
        statusMode->setText(isSoftApActive() ? "ESTADO --> CONECTADO SOFTAP" : "ESTADO --> CONECTADO STATION TCP");
        statusMode->setStyleSheet("color: #00f2c3; font-weight: bold; font-size: 11px; padding-left: 5px;");
    } else if(isSerialActive) {
        statusMode->setText("ESTADO --> CONECTADO SERIE");
        statusMode->setStyleSheet("color: #1d8cf8; font-weight: bold; font-size: 11px; padding-left: 5px;");
    } else if (isUdpActive) {
        if (m_lastRxTime > 0 && (now - m_lastRxTime) < 3000) {
            statusMode->setText(isSoftApActive() ? "ESTADO --> CONECTADO SOFTAP" : "ESTADO --> CONECTADO STATION UDP");
            statusMode->setStyleSheet("color: #00f2c3; font-weight: bold; font-size: 11px; padding-left: 5px;");
        } else {
            statusMode->setText("ESTADO --> CONECTANDO...");
            statusMode->setStyleSheet("color: #ffd600; font-weight: bold; font-size: 11px; padding-left: 5px;");
        }
    }

    update3DModelConnectionState(isRobotConnected());

    // Bloqueo de telemetría para permitir testeo limpio de comandos individuales
    if (m_isCommBlocked) {
        if (isTcpActive) {
            statusMode->setText(isSoftApActive() ? "ESTADO --> CONECTADO SOFTAP [ENVÍO BLOQUEADO]"
                                                 : "ESTADO --> CONECTADO STATION TCP [ENVÍO BLOQUEADO]");
        } else if (isSerialActive) {
            statusMode->setText("ESTADO --> CONECTADO SERIE [ENVÍO BLOQUEADO]");
        } else if (isUdpActive) {
            statusMode->setText(isSoftApActive() ? "ESTADO --> CONECTADO SOFTAP [ENVÍO BLOQUEADO]"
                                                 : "ESTADO --> CONECTADO STATION UDP [ENVÍO BLOQUEADO]");
        }
        isWaitingReply = false;
        return;
    }

    // --- 3. MÁQUINA DE ESTADOS ---
    static uint8_t commMef = 1;
    uint8_t buf[1];

    switch (commMef){
    case 1: buf[0] = GETMPU; break;
    case 2: buf[0] = GETADC; break;
    case 3: buf[0] = GETMPU; break;
    case 4: buf[0] = GETADC; break;
    case 5: buf[0] = GETPIDBALANCE; break;
    case 6: buf[0] = GETINTERNALDATA; break;
    }

    commMef++;
    if (commMef > 6) {
        commMef = 1;
    }

    sendCommand(buf, 1);
}
bool MainWindow::eventFilter(QObject *watched, QEvent *event){
    if (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease || event->type() == QEvent::ShortcutOverride) {
        if (ui->stackedWidget && ui->stackedWidget->currentIndex() == 3) {
            QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
            int key = keyEvent->key();
            if (key == Qt::Key_Up || key == Qt::Key_Down || key == Qt::Key_Left || key == Qt::Key_Right || key == Qt::Key_R) {
                if (event->type() == QEvent::ShortcutOverride) {
                    event->accept();
                    return true;
                }
                if (event->type() == QEvent::KeyPress) {
                    keyPressEvent(keyEvent);
                } else {
                    keyReleaseEvent(keyEvent);
                }
                return true; // Consumir evento para evitar cualquier navegación o cambio de pestaña indirecto
            } else if (key == Qt::Key_Tab || key == Qt::Key_Backtab) {
                // Bloquear tabulación para no perder el foco ni saltar a botones de cabecera
                event->accept();
                return true;
            }
        }
    }

    if(watched == ui->comboBox_PORT) {
        if (event->type() == QEvent::MouseButtonPress) {
            ui->comboBox_PORT->clear();
            QSerialPortInfo SerialPortInfo1;

            for(int i=0;i<SerialPortInfo1.availablePorts().count();i++)
                ui->comboBox_PORT->addItem(SerialPortInfo1.availablePorts().at(i).portName());

            return QMainWindow::eventFilter(watched, event);
        }
        else {
            return false;
        }
    }
    else{
        return QMainWindow::eventFilter(watched, event);
    }
}

void MainWindow::on_pushButton_connectSerial_clicked()
{
    if(QSerialPort1->isOpen()){
        QSerialPort1->close();
        ui->pushButton_connectSerial->setText("CONNECT");
        resetInterface();
    }
    else{

        if(ui->comboBox_PORT->currentText() == "")
            return;

        QSerialPort1->setPortName(ui->comboBox_PORT->currentText());
        QSerialPort1->setBaudRate(115200);
        QSerialPort1->setParity(QSerialPort::NoParity);
        QSerialPort1->setDataBits(QSerialPort::Data8);
        QSerialPort1->setStopBits(QSerialPort::OneStop);
        QSerialPort1->setFlowControl(QSerialPort::NoFlowControl);

        if(QSerialPort1->open(QSerialPort::ReadWrite)){
            ui->pushButton_connectSerial->setText("DISCONNECT");
            paramsSynced = false;
            uint8_t b = GETINTERNALDATA;
            sendSerial(&b, 1);
        }
        else
            QMessageBox::information(this, "Serial PORT", "ERROR. Opening PORT");
    }
}

void MainWindow::on_pushButton_connectUdp_clicked()
{
    int Port;
    bool ok;

    if(QUdpSocket1->isOpen() || (QTcpServer1 && QTcpServer1->isListening()) || (QTcpSocketClient && QTcpSocketClient->isOpen())){
        if(QUdpSocket1->isOpen()) {
            QUdpSocket1->close();
        }
        if(QTcpServer1 && QTcpServer1->isListening()) {
            QTcpServer1->close();
        }
        if(QTcpSocketClient) {
            QTcpSocketClient->disconnect();
            QTcpSocketClient->close();
            QTcpSocketClient->deleteLater();
            QTcpSocketClient = nullptr;
        }
        ui->pushButton_connectUdp->setText("CONNECT");
        ui->pushButton_protocol_wifi->setEnabled(true);
        statusMode->setText("ESTADO --> DESCONECTADO");
        statusMode->setStyleSheet("color: #fd5d93; font-weight: bold; font-size: 11px; padding-left: 5px;");
        m_lastRxTime = 0;
        resetInterface();
        return;
    }

    Port=ui->lineEdit_local_port->text().toInt(&ok,10);
    if(!ok || Port<=0 || Port>65535){
        QMessageBox::information(this, tr("SERVER PORT"),tr("ERRRO. Number PORT."));
        return;
    }

    try{
        if (m_isTcpMode) {
            if (QUdpSocket1->isOpen()) QUdpSocket1->close();
            if (QTcpServer1->isListening()) QTcpServer1->close();
            QTcpServer1->listen(QHostAddress::Any, Port);
        } else {
            if (QTcpServer1->isListening()) QTcpServer1->close();
            if (QTcpSocketClient) {
                QTcpSocketClient->disconnect();
                QTcpSocketClient->close();
                QTcpSocketClient->deleteLater();
                QTcpSocketClient = nullptr;
            }
            QUdpSocket1->abort();
            QUdpSocket1->bind(Port);
            QUdpSocket1->open(QUdpSocket::ReadWrite);
        }
    }catch(...){
        QMessageBox::information(this, tr("SERVER PORT"),tr("Can't OPEN Port."));
        return;
    }

    if (ui->lineEdit_device_ip->text().trimmed().startsWith("192.168.4.")) {
        m_isSoftApMode = true;
    } else if (!ui->lineEdit_device_ip->text().trimmed().isEmpty()) {
        m_isSoftApMode = false;
    }

    ui->pushButton_connectUdp->setText("DISCONNECT");
    ui->pushButton_protocol_wifi->setEnabled(false);

    if (m_isTcpMode) {
        statusMode->setText("ESTADO --> ESCUCHANDO TCP...");
        statusMode->setStyleSheet("color: #ffd600; font-weight: bold; font-size: 11px; padding-left: 5px;");
        addLogEntry("***ESCUCHANDO CONEXIONES TCP EN PUERTO " + QString::number(Port) + "***", "INFO");
        ui->textBrowserUnProcessed->append("***ESCUCHANDO CONEXIONES TCP EN PUERTO " + QString::number(Port) + "***");
    } else {
        statusMode->setText("ESTADO --> CONECTANDO UDP...");
        statusMode->setStyleSheet("color: #ffd600; font-weight: bold; font-size: 11px; padding-left: 5px;");
        paramsSynced = false;
        uint8_t b = GETINTERNALDATA;
        sendCommand(&b, 1);
        if(QUdpSocket1->isOpen()){
            clientAddress.setAddress(ui->lineEdit_device_ip->text());
            if(puertoremoto==0)
                puertoremoto=ui->lineEdit_device_port->text().toInt();
            QUdpSocket1->writeDatagram("r", 1, clientAddress, puertoremoto);
        }
    }
}

void MainWindow::on_pushButton_protocol_wifi_clicked()
{
    if (QUdpSocket1->isOpen() || (QTcpServer1 && QTcpServer1->isListening()) || (QTcpSocketClient && QTcpSocketClient->isOpen())) {
        QMessageBox::warning(this, "Protocolo WiFi", "Debe desconectar la conexión actual antes de cambiar de protocolo.");
        ui->pushButton_protocol_wifi->setChecked(m_isTcpMode);
        return;
    }
    m_isTcpMode = ui->pushButton_protocol_wifi->isChecked();
    updateProtocolUI();
}

void MainWindow::updateProtocolUI()
{
    ui->pushButton_protocol_wifi->blockSignals(true);
    ui->pushButton_protocol_wifi->setChecked(m_isTcpMode);
    ui->pushButton_protocol_wifi->blockSignals(false);

    if (m_isTcpMode) {
        ui->pushButton_protocol_wifi->setText("PROTOCOLO: TCP");
        ui->label_udp->setText("WIFI - TCP");
    } else {
        ui->pushButton_protocol_wifi->setText("PROTOCOLO: UDP");
        ui->label_udp->setText("WIFI - UDP");
    }
}

void MainWindow::on_sendBalanceKp_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETBALANCEKP;
    w.i32 = ui->setBalanceKp->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***KP BALANCÍN ACTUALIZADO***");
}

void MainWindow::on_sendBalanceKi_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETBALANCEKI;
    w.i32 = ui->setBalanceKi->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***KI BALANCÍN ACTUALIZADO***");
}

void MainWindow::on_sendBalanceKd_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETBALANCEKD;
    w.i32 = ui->setBalanceKd->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***KD BALANCÍN ACTUALIZADO***");
}

void MainWindow::on_sendLineKp_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETLINEKP;
    w.i32 = ui->setLineKp->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***KP LÍNEA ACTUALIZADO***");
}

void MainWindow::on_sendLineKd_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETLINEKD;
    w.i32 = ui->setLineKd->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***KQ LÍNEA (CUADRÁTICO) ACTUALIZADO***");
}

void MainWindow::on_sendWallKp_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETWALLKP;
    w.i32 = ui->setWallKp->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***KP PARED ACTUALIZADO***");
}

void MainWindow::on_sendWallKd_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETWALLKD;
    w.i32 = ui->setWallKd->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***KD ANTICIPO PARED ACTUALIZADO***");
}

void MainWindow::on_sendFrontKp_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETFRONTKP;
    w.i32 = ui->setFrontKp->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***KP FRONTAL (CENTRAL) ACTUALIZADO***");
}

void MainWindow::on_sendFrontKd_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETFRONTKD;
    w.i32 = ui->setFrontKd->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***KD ANTICIPO FRONTAL ACTUALIZADO***");
}

void MainWindow::on_sendDodgeDir_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETDODGEMODE;
    w.i32 = ui->comboDodgeDir->currentIndex();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    QString modeName = ui->comboDodgeDir->currentText();
    ui->textBrowserProcessed->append("***MODO ESQUIVADO ACTUALIZADO: " + modeName + "***");
}

void MainWindow::on_sendDodgeBiasTime_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETDODGEBIASTIME;
    w.i32 = ui->setDodgeBiasTime->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append(QString("***TIEMPO SESGO ACTUALIZADO: %1 ms***").arg(ui->setDodgeBiasTime->value()));
}

void MainWindow::on_sendDodgeBiasMult_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETDODGEBIASMULT;
    w.i32 = ui->setDodgeBiasMult->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append(QString("***MULTIPLICADOR SESGO ACTUALIZADO: %1***").arg(ui->setDodgeBiasMult->value()));
}

void MainWindow::on_pushButton_setSoftAp_clicked() {
    bool isConnOpen = (m_isTcpMode && QTcpSocketClient && QTcpSocketClient->isOpen())
                   || (!m_isTcpMode && QUdpSocket1 && QUdpSocket1->isOpen())
                   || (QSerialPort1 && QSerialPort1->isOpen());

    if (!isConnOpen) {
        QMessageBox::warning(this, "Conexión Requerida",
                             "No hay ninguna conexión activa con el robot (TCP, UDP o Serial).\n"
                             "Conéctate primero para enviar la solicitud de cambio a SoftAP.");
        return;
    }

    ui->pushButton_setSoftAp->setText("ENVIANDO...");
    ui->pushButton_setSoftAp->setEnabled(false);

    // Watchdog de 3 segundos por si no llega respuesta del robot
    QTimer::singleShot(3000, this, [this]() {
        if (ui->pushButton_setSoftAp->text() == "ENVIANDO...") {
            ui->pushButton_setSoftAp->setText("MODO SOFTAP");
            ui->pushButton_setSoftAp->setEnabled(true);
        }
    });

    uint8_t payload[4];
    uint8_t index = 0;
    payload[index++] = SETSOFTAP;
    sendCommand(payload, index);

    addLogEntry("***COMANDO ENVIADO: CAMBIO A MODO SOFTAP***", "TX");
    ui->textBrowserProcessed->append("***SOLICITANDO CAMBIO A MODO SOFTAP EN EL ROBOT...***");
}

void MainWindow::on_sendPWMMINL_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETPWMMINL; // 0xA7
    w.i32 = ui->setPWMMINL->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***PWM MIN L ACTUALIZADO***");
}

void MainWindow::on_sendPWMMINR_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETPWMMINR; // 0xA6
    w.i32 = ui->setPWMMINR->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***PWM MIN R ACTUALIZADO***");
}

void MainWindow::on_sendSetpoint_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETSETPOINT; // 0xAB
    w.i32 = ui->setSetpoint->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1]; // Enviamos 2 bytes
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***SETPOINT ACTUALIZADO***");
}

void MainWindow::on_sendAttackSetpoint_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETSPEED; // 0xB1
    w.i32 = ui->setAttackSetpoint->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1]; // Enviamos 2 bytes
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***ATTACK SETPOINT ACTUALIZADO***");
}

void MainWindow::on_setAttackSetpoint_valueChanged(int arg1) {
    (void)arg1;
    on_sendAttackSetpoint_clicked();
}

void MainWindow::on_sendPWML_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETPWML;
    w.i32 = ui->setPWML->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***PWM L ACTUALIZADO***");
}

void MainWindow::on_sendPWMR_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETPWMR;
    w.i32 = ui->setPWMR->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***PWM R ACTUALIZADO***");
}

void MainWindow::on_sendOFFSETL_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETOFFSETL;
    w.i32 = ui->setOFFSETL->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***OFFSET L ACTUALIZADO***");
}

void MainWindow::on_sendOFFSETR_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETOFFSETR;
    w.i32 = ui->setOFFSETR->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***OFFSET R ACTUALIZADO***");
}

void MainWindow::on_sendCustomTurn_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETCUSTOMTURN;
    w.i32 = ui->setCustomTurn->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***CUSTOM TURN ACTUALIZADO***");
}

void MainWindow::on_sendKpCascada_clicked() {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SET_KP_EXT;
    w.i16[0] = ui->setKpCascada->value();
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);
    ui->textBrowserProcessed->append("***KP CASCADA ACTUALIZADO***");
}





void MainWindow::on_actionConn_triggered()
{
    ui->tabWidget_wifi->setCurrentWidget(ui->tab_CONN);
    ui->DEBUG_PAGE->show();
    if (!myDebugDialog->isVisible()) {
        myDebugDialog->adjustSize();
    }
    myDebugDialog->show();
    myDebugDialog->raise();
    myDebugDialog->activateWindow();
}


void MainWindow::updateNavSelection(int index) {
    ui->btn_nav_infrarrojos->setChecked(index == 0);
    ui->btn_nav_visualizacion->setChecked(index == 1);
    ui->btn_nav_tuning->setChecked(index == 2);
    if (ui->btn_nav_goto) ui->btn_nav_goto->setChecked(index == 3);
}

void MainWindow::sendRobotMode(uint8_t modeId) {
    uint8_t payload[4];
    uint8_t index = 0;
    payload[index++] = SETROBOTMODE;
    payload[index++] = modeId;
    sendCommand(payload, index);

    QString modeName;
    switch(modeId) {
    case 0: modeName = "APAGADO / STANDBY"; break;
    case 1: modeName = "BALANCE"; break;
    case 2: modeName = "SEGUIR LINEA"; break;
    case 3: modeName = "ESQUIVAR"; break;
    case 4: modeName = "JOYSTICK"; break;
    case 5: modeName = "3D SCREEN"; break;
    default: modeName = QString("MODO %1").arg(modeId); break;
    }

    addLogEntry("***COMANDO MODO ENVIADO: " + modeName + "***", "TX");
    ui->textBrowserProcessed->append("***ENVIANDO MODO AL ROBOT: " + modeName + "...***");

    if (modeId <= 4) {
        updateRobotModeUI(modeId);
    }
    if (modeId == 4) {
        ui->stackedWidget->setCurrentIndex(3);
        resetGoToYaw();
        this->setFocus();
    }
}

void MainWindow::updateRobotModeUI(uint8_t mode) {
    ui->btn_mode_balance->blockSignals(true);
    ui->btn_mode_line->blockSignals(true);
    ui->btn_mode_dodge->blockSignals(true);
    if (ui->btn_mode_goto) ui->btn_mode_goto->blockSignals(true);

    ui->btn_mode_balance->setChecked(mode == 1);
    ui->btn_mode_line->setChecked(mode == 2);
    ui->btn_mode_dodge->setChecked(mode == 3);
    if (ui->btn_mode_goto) ui->btn_mode_goto->setChecked(mode == 4);

    ui->btn_mode_balance->blockSignals(false);
    ui->btn_mode_line->blockSignals(false);
    ui->btn_mode_dodge->blockSignals(false);
    if (ui->btn_mode_goto) ui->btn_mode_goto->blockSignals(false);

    static uint8_t lastKnownMode = 255;
    if (mode == 4 && lastKnownMode != 4) {
        resetGoToYaw();
    }
    lastKnownMode = mode;

    if (ui->label_gotoStatusBadge) {
        if (mode == 4) {
            ui->label_gotoStatusBadge->setText("● JOYSTICK ACTIVO");
            ui->label_gotoStatusBadge->setStyleSheet("background-color: #0d381e; color: #00e676; border: 1.5px solid #00e676; border-radius: 12px; padding: 4px 14px; font-weight: bold;");
            if (ui->label_gotoTitle) {
                ui->label_gotoTitle->setStyleSheet("color: #00e676;");
            }
        } else {
            ui->label_gotoStatusBadge->setText("● JOYSTICK STANDBY");
            ui->label_gotoStatusBadge->setStyleSheet("background-color: #2a1217; color: #ff5252; border: 1px solid #ff5252; border-radius: 12px; padding: 4px 14px; font-weight: bold;");
            if (ui->label_gotoTitle) {
                ui->label_gotoTitle->setStyleSheet("color: #ff5252;");
            }
        }
    }
}

void MainWindow::on_btn_mode_balance_clicked() {
    if (ui->stackedWidget && ui->stackedWidget->currentIndex() == 3) {
        if (ui->btn_mode_balance && !ui->btn_mode_balance->underMouse()) {
            if (ui->btn_mode_goto) ui->btn_mode_goto->setChecked(true);
            return;
        }
    }
    sendRobotMode(1);
}

void MainWindow::on_btn_mode_line_clicked() {
    if (ui->stackedWidget && ui->stackedWidget->currentIndex() == 3) {
        if (ui->btn_mode_line && !ui->btn_mode_line->underMouse()) {
            if (ui->btn_mode_goto) ui->btn_mode_goto->setChecked(true);
            return;
        }
    }
    sendRobotMode(2);
}

void MainWindow::on_btn_mode_dodge_clicked() {
    if (ui->stackedWidget && ui->stackedWidget->currentIndex() == 3) {
        if (ui->btn_mode_dodge && !ui->btn_mode_dodge->underMouse()) {
            if (ui->btn_mode_goto) ui->btn_mode_goto->setChecked(true);
            return;
        }
    }
    sendRobotMode(3);
}

void MainWindow::on_actionOpenDebug_triggered()
{
    ui->tabWidget_wifi->setCurrentWidget(ui->tab_LOGS);
    ui->DEBUG_PAGE->show();
    if (!myDebugDialog->isVisible()) {
        myDebugDialog->adjustSize();
    }
    myDebugDialog->show();
    myDebugDialog->raise();
    myDebugDialog->activateWindow();
}

void MainWindow::on_actionDocu_triggered()
{
    ui->tabWidget_help->setCurrentWidget(ui->tab_DOCU);
    ui->HELP_PAGE->show();
    if (!myHelpDialog->isVisible()) {
        myHelpDialog->resize(720, 560);
    }
    myHelpDialog->show();
    myHelpDialog->raise();
    myHelpDialog->activateWindow();
}

void MainWindow::on_actionAcercaDe_triggered()
{
    ui->tabWidget_help->setCurrentWidget(ui->tab_ACERCA_DE);
    ui->HELP_PAGE->show();
    if (!myHelpDialog->isVisible()) {
        myHelpDialog->resize(720, 560);
    }
    myHelpDialog->show();
    myHelpDialog->raise();
    myHelpDialog->activateWindow();
}

void MainWindow::initHelpContent()
{
    ui->textBrowser_docu->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    ui->textBrowser_docu->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->textBrowser_acerca->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    ui->textBrowser_acerca->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    QString docuHtml = R"(
<div style="font-family: 'Segoe UI', sans-serif; color: #e2e2e2; line-height: 1.5; padding: 12px;">
    <h2 style="color: #00f2c3; border-bottom: 2px solid #e14eca; padding-bottom: 6px; margin-top: 0; font-size: 18px; font-weight: bold;">
        Funciones del Autito - Centro de Control MCC25
    </h2>
    <p style="color: #a0a5b5; font-size: 12px; margin-bottom: 14px;">
        Este software supervisa y comanda un autito robótico de tipo <b>péndulo invertido auto-balanceado de 2 ruedas</b>. A continuación se detallan todas sus funciones principales y modos de operación:
    </p>

    <div style="background-color: #1a1a28; border-left: 4px solid #00f2c3; padding: 10px 14px; margin-bottom: 12px; border-radius: 4px;">
        <h3 style="color: #00f2c3; margin: 0 0 6px 0; font-size: 14px; font-weight: bold;">1. Modo Balance (Péndulo Invertido)</h3>
        <p style="margin: 0; font-size: 12px; color: #d0d2dc;">
            <b>• Descripción:</b> Mantiene el equilibrio dinámico vertical del robot sobre sus dos ruedas sin volcarse.<br>
            <b>• Principio Físico:</b> La Unidad de Medición Inercial (IMU con giroscopio y acelerómetro) registra continuamente el ángulo de inclinación (Pitch) y la velocidad angular.<br>
            <b>• Algoritmo:</b> Lazo cerrado de control PID en cascada. El algoritmo calcula en tiempo real el torque y velocidad necesarios para que los motores DC contrarresten la gravedad y estabilicen el centro de masa.
        </p>
    </div>

    <div style="background-color: #1a1a28; border-left: 4px solid #e14eca; padding: 10px 14px; margin-bottom: 12px; border-radius: 4px;">
        <h3 style="color: #e14eca; margin: 0 0 6px 0; font-size: 14px; font-weight: bold;">2. Modo Seguir Línea</h3>
        <p style="margin: 0; font-size: 12px; color: #d0d2dc;">
            <b>• Descripción:</b> Navegación autónoma siguiendo pistas o circuitos demarcados en el piso mediante cinta reflectiva o línea negra.<br>
            <b>• Sensores:</b> Barra frontal compuesta por 8 sensores infrarrojos reflectivos (IR1 a IR8).<br>
            <b>• Lógica:</b> Mediante ponderación espacial de los sensores activos, se calcula la desviación de la línea (error lineal y cuadrático). Un controlador PD diferencial ajusta las velocidades relativas entre la rueda izquierda y derecha para mantener la trayectoria sin perder el auto-balance.
        </p>
    </div>

    <div style="background-color: #1a1a28; border-left: 4px solid #ff8d72; padding: 10px 14px; margin-bottom: 12px; border-radius: 4px;">
        <h3 style="color: #ff8d72; margin: 0 0 6px 0; font-size: 14px; font-weight: bold;">3. Modo Esquivar (Evasión de Obstáculos)</h3>
        <p style="margin: 0; font-size: 12px; color: #d0d2dc;">
            <b>• Descripción:</b> Detección frontal de obstáculos y ejecución de maniobras evasivas automáticas.<br>
            <b>• Sensores:</b> Sensores de distancia de ultrasonido / infrarrojos frontales.<br>
            <b>• Maniobra:</b> Cuando un obstáculo se encuentra a una distancia menor al umbral de seguridad, el autito frena suavemente, rota sobre su eje hacia la dirección configurada (izquierda o derecha), bordea el obstáculo y retoma la navegación normal.
        </p>
    </div>

    <div style="background-color: #1a1a28; border-left: 4px solid #00e676; padding: 10px 14px; margin-bottom: 12px; border-radius: 4px;">
        <h3 style="color: #00e676; margin: 0 0 6px 0; font-size: 14px; font-weight: bold;">4. Modo Control Joystick</h3>
        <p style="margin: 0; font-size: 12px; color: #d0d2dc;">
            <b>• Descripción:</b> Control manual y maniobra directa del autito mediante cruceta direccional o teclado en tiempo real.<br>
            <b>• Funcionalidad:</b> Permite comandar avance, retroceso (ajuste progresivo del setpoint de inclinación) y giros sobre su eje (rotación horaria y antihoraria con intensidad y duración configurables), conservando la estabilidad de auto-balance.
        </p>
    </div>

    <div style="background-color: #1a1a28; border-left: 4px solid #1d8cf8; padding: 10px 14px; margin-bottom: 12px; border-radius: 4px;">
        <h3 style="color: #1d8cf8; margin: 0 0 6px 0; font-size: 14px; font-weight: bold;">5. Gemelo Digital y Visualización 3D</h3>
        <p style="margin: 0; font-size: 12px; color: #d0d2dc;">
            <b>• Descripción:</b> Renderizado tridimensional en tiempo real del autito dentro de la interfaz.<br>
            <b>• Tecnología:</b> Desarrollado con Qt Quick 3D y QML. Refleja fielmente la inclinación espacial (Pitch, Roll y Yaw) del autito en vivo tal como se encuentra en el mundo físico.
        </p>
    </div>

    <div style="background-color: #1a1a28; border-left: 4px solid #ba54f5; padding: 10px 14px; margin-bottom: 12px; border-radius: 4px;">
        <h3 style="color: #ba54f5; margin: 0 0 6px 0; font-size: 14px; font-weight: bold;">6. Telemetría y Sintonización PID en Tiempo Real</h3>
        <p style="margin: 0; font-size: 12px; color: #d0d2dc;">
            <b>• Monitoreo:</b> Visualización en vivo de señales PWM, lecturas de los 8 sensores IR, aceleración, velocidades y estado de conexión.<br>
            <b>• Sintonización en Caliente:</b> Permite modificar las ganancias proporcional (Kp), integral (Ki) y derivativa (Kd) de los lazos de control desde la interfaz gráfica sin necesidad de reprogramar el microcontrolador.<br>
            <b>• Gráficos Dinámicos:</b> Gráficas de respuesta temporal para evaluar estabilidad y sobreimpulso.
        </p>
    </div>

    <div style="background-color: #1a1a28; border-left: 4px solid #ffd600; padding: 10px 14px; margin-bottom: 6px; border-radius: 4px;">
        <h3 style="color: #ffd600; margin: 0 0 6px 0; font-size: 14px; font-weight: bold;">7. Conectividad Dual (WiFi UDP / Serial)</h3>
        <p style="margin: 0; font-size: 12px; color: #d0d2dc;">
            <b>• Comunicación WiFi:</b> Transmisión inalámbrica de alta velocidad mediante datagramas UDP para telemetría y control sin cables.<br>
            <b>• Conexión Serie:</b> Puerto COM (USB/UART) para calibración en banco de pruebas y consola de logs (TX/RX).
        </p>
    </div>
</div>
)";
    ui->textBrowser_docu->setHtml(docuHtml);

    QString acercaHtml = R"(
<div style="font-family: 'Segoe UI', sans-serif; color: #e2e2e2; line-height: 1.6; padding: 14px;">
    <div style="text-align: center; margin-bottom: 16px;">
        <h1 style="color: #00f2c3; margin: 0; font-size: 22px; font-weight: bold; letter-spacing: 1px;">MICRO-Software</h1>
        <h3 style="color: #e14eca; margin: 4px 0 10px 0; font-size: 14px; font-weight: bold;">Centro de Control, Telemetría y Visualización 3D</h3>
        <span style="background-color: #27293d; color: #00f2c3; border: 1px solid #00f2c3; border-radius: 12px; padding: 3px 12px; font-size: 11px; font-weight: bold;">MCC25 · Versión 1.0.0</span>
    </div>

    <div style="background-color: #1a1a28; border: 1px solid #27293d; border-radius: 8px; padding: 14px; margin-bottom: 14px;">
        <h4 style="color: #00f2c3; margin-top: 0; margin-bottom: 6px; font-size: 13px; font-weight: bold; text-transform: uppercase;">Descripción del Software</h4>
        <p style="margin: 0; font-size: 12.5px; color: #c4c7d0; text-align: justify;">
            <b>MICRO-Software</b> es una aplicación de escritorio diseñada para servir como estación de monitoreo y control en tiempo real para un autito robótico tipo péndulo invertido de dos ruedas. Integra representación espacial en 3D (gemelo digital), sintonización en caliente de parámetros de control PID, adquisición de telemetría de sensores inerciales e infrarrojos, y canales de comunicación inalámbrica vía Wi-Fi (protocolo UDP) y cableada mediante interfaz Serie UART.
        </p>
    </div>

    <table width="100%" align="center" border="0" cellpadding="0" cellspacing="10" style="width: 100%; border-collapse: separate; border-spacing: 10px; margin-bottom: 12px;">
        <tr>
            <td width="50%" align="center" style="width: 50%; background-color: #1a1a28; border: 1px solid #27293d; border-radius: 8px; padding: 12px 14px; text-align: center;">
                <div style="color: #e14eca; font-size: 12px; font-weight: bold; text-transform: uppercase; text-align: center;">Materia</div>
                <div style="color: #ffffff; font-size: 14px; font-weight: bold; margin-top: 4px; text-align: center;">Microcontroladores (MCC)</div>
            </td>
            <td width="50%" align="center" style="width: 50%; background-color: #1a1a28; border: 1px solid #27293d; border-radius: 8px; padding: 12px 14px; text-align: center;">
                <div style="color: #00f2c3; font-size: 12px; font-weight: bold; text-transform: uppercase; text-align: center;">Estudiante</div>
                <div style="color: #ffffff; font-size: 14px; font-weight: bold; margin-top: 4px; text-align: center;">Gonzalo Martín Buffa</div>
            </td>
        </tr>
        <tr>
            <td width="50%" align="center" style="width: 50%; background-color: #1a1a28; border: 1px solid #27293d; border-radius: 8px; padding: 12px 14px; text-align: center;">
                <div style="color: #ff8d72; font-size: 12px; font-weight: bold; text-transform: uppercase; text-align: center;">Año</div>
                <div style="color: #ffffff; font-size: 14px; font-weight: bold; margin-top: 4px; text-align: center;">2025 - 2026</div>
            </td>
            <td width="50%" align="center" style="width: 50%; background-color: #1a1a28; border: 1px solid #27293d; border-radius: 8px; padding: 12px 14px; text-align: center;">
                <div style="color: #00e676; font-size: 12px; font-weight: bold; text-transform: uppercase; text-align: center;">Tecnologías</div>
                <div style="color: #ffffff; font-size: 14px; font-weight: bold; margin-top: 4px; text-align: center;">Qt 6.10.2</div>
            </td>
        </tr>
    </table>

    <div style="text-align: center; color: #6c7293; font-size: 11px; font-weight: bold; margin-top: 10px;">
        © 2025-2026 Gonzalo Martín Buffa · Proyecto de Microcontroladores
    </div>
</div>
)";
    ui->textBrowser_acerca->setHtml(acercaHtml);
}


// -----------------------------------------------------------------------
// Chart PID embebido en MainWindow (widget PIDchart promovido a QChartView)
// -----------------------------------------------------------------------
void MainWindow::initPIDChart()
{
    // Series para Balanceo
    pid_pSeries   = new QLineSeries(); pid_pSeries->setName("P Balance (Kp)");
    pid_iSeries   = new QLineSeries(); pid_iSeries->setName("I Balance (Ki)");
    pid_dSeries   = new QLineSeries(); pid_dSeries->setName("D Balance (Kd)");
    pid_outSeries = new QLineSeries(); pid_outSeries->setName("Output Balance");

    // Series para Seguimiento de Linea
    pid_pLineSeries   = new QLineSeries(); pid_pLineSeries->setName("P Linea (Kp)");
    pid_dLineSeries   = new QLineSeries(); pid_dLineSeries->setName("D Linea (Kq)");
    pid_outLineSeries = new QLineSeries(); pid_outLineSeries->setName("Giro Offset");

    // Series MPU y Angulos
    pid_axSeries = new QLineSeries(); pid_axSeries->setName("Ax");
    pid_aySeries = new QLineSeries(); pid_aySeries->setName("Ay");
    pid_azSeries = new QLineSeries(); pid_azSeries->setName("Az");
    pid_gxSeries = new QLineSeries(); pid_gxSeries->setName("Gx");
    pid_gySeries = new QLineSeries(); pid_gySeries->setName("Gy");
    pid_gzSeries = new QLineSeries(); pid_gzSeries->setName("Gz");

    pid_pitchSeries = new QLineSeries(); pid_pitchSeries->setName("Pitch");
    pid_rollSeries  = new QLineSeries(); pid_rollSeries->setName("Roll");
    pid_yawSeries   = new QLineSeries(); pid_yawSeries->setName("Yaw");

    // Series Sensores IR
    pid_ir1Series = new QLineSeries(); pid_ir1Series->setName("IR0 (Der 90°)");
    pid_ir2Series = new QLineSeries(); pid_ir2Series->setName("IR1 (Línea Izq)");
    pid_ir3Series = new QLineSeries(); pid_ir3Series->setName("IR2 (Izq 90°)");
    pid_ir4Series = new QLineSeries(); pid_ir4Series->setName("IR3 (Línea Cen)");
    pid_ir5Series = new QLineSeries(); pid_ir5Series->setName("IR4 (Izq 45°)");
    pid_ir6Series = new QLineSeries(); pid_ir6Series->setName("IR5 (Línea Der)");
    pid_ir7Series = new QLineSeries(); pid_ir7Series->setName("IR6 (Frontal)");
    pid_ir8Series = new QLineSeries(); pid_ir8Series->setName("IR7 (Der 45°)");

    // Colores y grosores para diferenciar curvas claramente
    QPen penPitch(QColor("#00f2c3")); penPitch.setWidth(2); pid_pitchSeries->setPen(penPitch);
    QPen penRoll(QColor("#ffd600"));  penRoll.setWidth(2);  pid_rollSeries->setPen(penRoll);
    QPen penYaw(QColor("#bd00ff"));   penYaw.setWidth(2);   pid_yawSeries->setPen(penYaw);

    QPen penP(QColor("#fd5d93"));     penP.setWidth(2);     pid_pSeries->setPen(penP);
    QPen penI(QColor("#00f2c3"));     penI.setWidth(2);     pid_iSeries->setPen(penI);
    QPen penD(QColor("#1d8cf8"));     penD.setWidth(2);     pid_dSeries->setPen(penD);
    QPen penOut(QColor("#ff8d72"));   penOut.setWidth(2);   pid_outSeries->setPen(penOut);

    QPen penPLine(QColor("#00f2c3")); penPLine.setWidth(2); pid_pLineSeries->setPen(penPLine);
    QPen penDLine(QColor("#e14eca")); penDLine.setWidth(2); pid_dLineSeries->setPen(penDLine);
    QPen penOutLine(QColor("#ff8d72")); penOutLine.setWidth(2); pid_outLineSeries->setPen(penOutLine);

    auto setupChartCommon = [](QChart *chart, const QString &title = QString()) {
        chart->setTheme(QChart::ChartThemeDark);
        if (!title.isEmpty()) {
            chart->setTitle(title);
            chart->setTitleBrush(QBrush(QColor("#e2e2e2")));
            QFont titleFont = chart->titleFont();
            titleFont.setBold(true);
            titleFont.setPointSize(10);
            chart->setTitleFont(titleFont);
        }
        chart->setBackgroundBrush(QBrush(QColor("#151522")));
        chart->setPlotAreaBackgroundBrush(QBrush(QColor("#10101a")));
        chart->setPlotAreaBackgroundVisible(true);
        chart->layout()->setContentsMargins(0, 0, 0, 0);
        chart->setBackgroundRoundness(0);
        chart->legend()->setLabelColor(QColor("#e2e2e2"));
        chart->legend()->setBackgroundVisible(false);
        chart->legend()->setVisible(true);
        chart->legend()->setAlignment(Qt::AlignTop);
    };

    auto setupAxisCommon = [](QValueAxis *axis, const QString &title, double minVal, double maxVal) {
        axis->setRange(minVal, maxVal);
        axis->setTitleText(title);
        axis->setLabelsColor(QColor("#a0a5b5"));
        axis->setTitleBrush(QBrush(QColor("#00f2c3")));
        axis->setGridLineColor(QColor("#27293d"));
        axis->setLinePenColor(QColor("#3d405b"));
    };

    // 1. Gráfica de Orientación
    chartOrientation_mw = new QChart();
    setupChartCommon(chartOrientation_mw, "");
    chartOrientation_mw->addSeries(pid_pitchSeries);
    chartOrientation_mw->addSeries(pid_rollSeries);
    chartOrientation_mw->addSeries(pid_yawSeries);
    chartOrientation_mw->addSeries(pid_axSeries);
    chartOrientation_mw->addSeries(pid_aySeries);
    chartOrientation_mw->addSeries(pid_azSeries);
    chartOrientation_mw->addSeries(pid_gxSeries);
    chartOrientation_mw->addSeries(pid_gySeries);
    chartOrientation_mw->addSeries(pid_gzSeries);

    orient_axisX = new QValueAxis();
    setupAxisCommon(orient_axisX, "Tiempo (s)", 0, 10);
    orient_axisY = new QValueAxis();
    setupAxisCommon(orient_axisY, "Angulo (deg)", -45, 45);

    chartOrientation_mw->addAxis(orient_axisX, Qt::AlignBottom);
    chartOrientation_mw->addAxis(orient_axisY, Qt::AlignLeft);
    pid_pitchSeries->attachAxis(orient_axisX); pid_pitchSeries->attachAxis(orient_axisY);
    pid_rollSeries->attachAxis(orient_axisX);  pid_rollSeries->attachAxis(orient_axisY);
    pid_yawSeries->attachAxis(orient_axisX);   pid_yawSeries->attachAxis(orient_axisY);
    pid_axSeries->attachAxis(orient_axisX);    pid_axSeries->attachAxis(orient_axisY);
    pid_aySeries->attachAxis(orient_axisX);    pid_aySeries->attachAxis(orient_axisY);
    pid_azSeries->attachAxis(orient_axisX);    pid_azSeries->attachAxis(orient_axisY);
    pid_gxSeries->attachAxis(orient_axisX);    pid_gxSeries->attachAxis(orient_axisY);
    pid_gySeries->attachAxis(orient_axisX);    pid_gySeries->attachAxis(orient_axisY);
    pid_gzSeries->attachAxis(orient_axisX);    pid_gzSeries->attachAxis(orient_axisY);

    if (ui->chartView_orientation) {
        ui->chartView_orientation->setChart(chartOrientation_mw);
        ui->chartView_orientation->setRenderHint(QPainter::Antialiasing);
    }

    // 2. Gráfica de Balanceo
    chartBalance_mw = new QChart();
    setupChartCommon(chartBalance_mw, "");
    chartBalance_mw->addSeries(pid_pSeries);
    chartBalance_mw->addSeries(pid_iSeries);
    chartBalance_mw->addSeries(pid_dSeries);
    chartBalance_mw->addSeries(pid_outSeries);

    bal_axisX = new QValueAxis();
    setupAxisCommon(bal_axisX, "Tiempo (s)", 0, 10);
    bal_axisY = new QValueAxis();
    setupAxisCommon(bal_axisY, "Amplitud", -200, 200);

    chartBalance_mw->addAxis(bal_axisX, Qt::AlignBottom);
    chartBalance_mw->addAxis(bal_axisY, Qt::AlignLeft);
    pid_pSeries->attachAxis(bal_axisX);   pid_pSeries->attachAxis(bal_axisY);
    pid_iSeries->attachAxis(bal_axisX);   pid_iSeries->attachAxis(bal_axisY);
    pid_dSeries->attachAxis(bal_axisX);   pid_dSeries->attachAxis(bal_axisY);
    pid_outSeries->attachAxis(bal_axisX); pid_outSeries->attachAxis(bal_axisY);

    chartPID_mw = chartBalance_mw;
    pid_axisX = bal_axisX;
    pid_axisY = bal_axisY;

    if (ui->chartView_balance) {
        ui->chartView_balance->setChart(chartBalance_mw);
        ui->chartView_balance->setRenderHint(QPainter::Antialiasing);
    }

    // 3. Gráfica de Seguimiento de Línea
    chartLine_mw = new QChart();
    setupChartCommon(chartLine_mw, "");
    chartLine_mw->addSeries(pid_pLineSeries);
    chartLine_mw->addSeries(pid_dLineSeries);
    chartLine_mw->addSeries(pid_outLineSeries);
    chartLine_mw->addSeries(pid_ir1Series);
    chartLine_mw->addSeries(pid_ir2Series);
    chartLine_mw->addSeries(pid_ir3Series);
    chartLine_mw->addSeries(pid_ir4Series);
    chartLine_mw->addSeries(pid_ir5Series);
    chartLine_mw->addSeries(pid_ir6Series);
    chartLine_mw->addSeries(pid_ir7Series);
    chartLine_mw->addSeries(pid_ir8Series);

    line_axisX = new QValueAxis();
    setupAxisCommon(line_axisX, "Tiempo (s)", 0, 10);
    line_axisY = new QValueAxis();
    setupAxisCommon(line_axisY, "Giro / Error", -100, 100);

    chartLine_mw->addAxis(line_axisX, Qt::AlignBottom);
    chartLine_mw->addAxis(line_axisY, Qt::AlignLeft);
    pid_pLineSeries->attachAxis(line_axisX);   pid_pLineSeries->attachAxis(line_axisY);
    pid_dLineSeries->attachAxis(line_axisX);   pid_dLineSeries->attachAxis(line_axisY);
    pid_outLineSeries->attachAxis(line_axisX); pid_outLineSeries->attachAxis(line_axisY);
    pid_ir1Series->attachAxis(line_axisX);     pid_ir1Series->attachAxis(line_axisY);
    pid_ir2Series->attachAxis(line_axisX);     pid_ir2Series->attachAxis(line_axisY);
    pid_ir3Series->attachAxis(line_axisX);     pid_ir3Series->attachAxis(line_axisY);
    pid_ir4Series->attachAxis(line_axisX);     pid_ir4Series->attachAxis(line_axisY);
    pid_ir5Series->attachAxis(line_axisX);     pid_ir5Series->attachAxis(line_axisY);
    pid_ir6Series->attachAxis(line_axisX);     pid_ir6Series->attachAxis(line_axisY);
    pid_ir7Series->attachAxis(line_axisX);     pid_ir7Series->attachAxis(line_axisY);
    pid_ir8Series->attachAxis(line_axisX);     pid_ir8Series->attachAxis(line_axisY);

    if (ui->chartView_line) {
        ui->chartView_line->setChart(chartLine_mw);
        ui->chartView_line->setRenderHint(QPainter::Antialiasing);
    }

    updatePIDChartRange();
}

void MainWindow::updatePIDChart(double time, double p, double i, double d, double out, double pLine, double dLine, double outLine)
{
    // Balance
    pid_pSeries->append(time, p);
    pid_iSeries->append(time, i);
    pid_dSeries->append(time, d);
    pid_outSeries->append(time, out);

    // Line
    pid_pLineSeries->append(time, pLine);
    pid_dLineSeries->append(time, dLine);
    pid_outLineSeries->append(time, outLine);

    // Limitar la cantidad de puntos
    const int maxPoints = 1000;
    while (pid_pSeries->count() > maxPoints) pid_pSeries->remove(0);
    while (pid_iSeries->count() > maxPoints) pid_iSeries->remove(0);
    while (pid_dSeries->count() > maxPoints) pid_dSeries->remove(0);
    while (pid_outSeries->count() > maxPoints) pid_outSeries->remove(0);

    while (pid_pLineSeries->count() > maxPoints) pid_pLineSeries->remove(0);
    while (pid_dLineSeries->count() > maxPoints) pid_dLineSeries->remove(0);
    while (pid_outLineSeries->count() > maxPoints) pid_outLineSeries->remove(0);

    // Scroll del eje X para balance y linea
    if (time > 10.0) {
        bal_axisX->setRange(time - 10.0, time);
        line_axisX->setRange(time - 10.0, time);
    } else {
        bal_axisX->setRange(0, 10.0);
        line_axisX->setRange(0, 10.0);
    }

    updatePIDChartRange();
}

void MainWindow::updateMPUChart(double time, double ax, double ay, double az, double gx, double gy, double gz, double pitch, double roll, double yaw)
{
    pid_axSeries->append(time, ax);
    pid_aySeries->append(time, ay);
    pid_azSeries->append(time, az);
    pid_gxSeries->append(time, gx);
    pid_gySeries->append(time, gy);
    pid_gzSeries->append(time, gz);
    pid_pitchSeries->append(time, pitch);
    pid_rollSeries->append(time, roll);
    pid_yawSeries->append(time, yaw);

    const int maxPoints = 1000;
    while (pid_axSeries->count() > maxPoints) pid_axSeries->remove(0);
    while (pid_aySeries->count() > maxPoints) pid_aySeries->remove(0);
    while (pid_azSeries->count() > maxPoints) pid_azSeries->remove(0);
    while (pid_gxSeries->count() > maxPoints) pid_gxSeries->remove(0);
    while (pid_gySeries->count() > maxPoints) pid_gySeries->remove(0);
    while (pid_gzSeries->count() > maxPoints) pid_gzSeries->remove(0);
    while (pid_pitchSeries->count() > maxPoints) pid_pitchSeries->remove(0);
    while (pid_rollSeries->count() > maxPoints) pid_rollSeries->remove(0);
    while (pid_yawSeries->count() > maxPoints) pid_yawSeries->remove(0);

    // Scroll del eje X para orientacion
    if (time > 10.0) {
        orient_axisX->setRange(time - 10.0, time);
    } else {
        orient_axisX->setRange(0, 10.0);
    }

    updatePIDChartRange();
}

void MainWindow::updateIRChart(double time, double ir1, double ir2, double ir3, double ir4, double ir5, double ir6, double ir7, double ir8)
{
    pid_ir1Series->append(time, ir1);
    pid_ir2Series->append(time, ir2);
    pid_ir3Series->append(time, ir3);
    pid_ir4Series->append(time, ir4);
    pid_ir5Series->append(time, ir5);
    pid_ir6Series->append(time, ir6);
    pid_ir7Series->append(time, ir7);
    pid_ir8Series->append(time, ir8);

    const int maxPoints = 1000;
    while (pid_ir1Series->count() > maxPoints) pid_ir1Series->remove(0);
    while (pid_ir2Series->count() > maxPoints) pid_ir2Series->remove(0);
    while (pid_ir3Series->count() > maxPoints) pid_ir3Series->remove(0);
    while (pid_ir4Series->count() > maxPoints) pid_ir4Series->remove(0);
    while (pid_ir5Series->count() > maxPoints) pid_ir5Series->remove(0);
    while (pid_ir6Series->count() > maxPoints) pid_ir6Series->remove(0);
    while (pid_ir7Series->count() > maxPoints) pid_ir7Series->remove(0);
    while (pid_ir8Series->count() > maxPoints) pid_ir8Series->remove(0);

    if (time > 10.0) {
        line_axisX->setRange(time - 10.0, time);
    } else {
        line_axisX->setRange(0, 10.0);
    }

    updatePIDChartRange();
}

void MainWindow::updatePIDChartRange()
{
    auto autoRangeChart = [](QChart *chart, QValueAxis *axisX, QValueAxis *axisY, const QList<QPair<QLineSeries*, bool>> &seriesList, double defaultSpan) {
        if (!chart || !axisX || !axisY) return;

        double time = axisX->max();
        double xMin = qMax(0.0, time - 10.0);
        double xMax = time;
        double yMin = std::numeric_limits<double>::max();
        double yMax = std::numeric_limits<double>::lowest();
        bool hasPoints = false;
        bool anyVisible = false;

        for (const auto &item : seriesList) {
            QLineSeries *s = item.first;
            bool visible = item.second;
            if (!s) continue;
            s->setVisible(visible);

            const auto markers = chart->legend()->markers(s);
            for (QLegendMarker *marker : markers) {
                marker->setVisible(visible);
            }

            if (visible) {
                anyVisible = true;
                const QList<QPointF> points = s->points();
                for (const QPointF &pt : points) {
                    if (pt.x() >= xMin && pt.x() <= xMax) {
                        if (pt.y() < yMin) yMin = pt.y();
                        if (pt.y() > yMax) yMax = pt.y();
                        hasPoints = true;
                    }
                }
            }
        }

        chart->legend()->setVisible(anyVisible);

        if (hasPoints) {
            double margin = (yMax - yMin) * 0.15;
            if (margin < 1.0) margin = 2.0;
            axisY->setRange(yMin - margin, yMax + margin);
        } else {
            axisY->setRange(-defaultSpan, defaultSpan);
        }
    };

    // 1. Orientacion (Pitch, Roll, Yaw, MPU)
    autoRangeChart(chartOrientation_mw, orient_axisX, orient_axisY, {
        {pid_pitchSeries, ui->checkBox_Pitch->isChecked()},
        {pid_rollSeries,  ui->checkBox_Roll->isChecked()},
        {pid_yawSeries,   ui->checkBox_Yaw->isChecked()},
        {pid_axSeries,    ui->checkBox_Ax->isChecked()},
        {pid_aySeries,    ui->checkBox_Ay->isChecked()},
        {pid_azSeries,    ui->checkBox_Az->isChecked()},
        {pid_gxSeries,    ui->checkBox_Gx->isChecked()},
        {pid_gySeries,    ui->checkBox_Gy->isChecked()},
        {pid_gzSeries,    ui->checkBox_Gz->isChecked()}
    }, 30.0);

    // 2. Balanceo (P, I, D, Out)
    autoRangeChart(chartBalance_mw, bal_axisX, bal_axisY, {
        {pid_pSeries,   ui->checkBox_P->isChecked()},
        {pid_iSeries,   ui->checkBox_I->isChecked()},
        {pid_dSeries,   ui->checkBox_D->isChecked()},
        {pid_outSeries, ui->checkBox_Out->isChecked()}
    }, 100.0);

    // 3. Seguimiento de Linea (P_line, D_line, Out_line, IRs)
    autoRangeChart(chartLine_mw, line_axisX, line_axisY, {
        {pid_pLineSeries,   ui->checkBox_P_line->isChecked()},
        {pid_dLineSeries,   ui->checkBox_D_line->isChecked()},
        {pid_outLineSeries, ui->checkBox_Out_line->isChecked()},
        {pid_ir1Series,     ui->checkBox_Ir1->isChecked()},
        {pid_ir2Series,     ui->checkBox_Ir2->isChecked()},
        {pid_ir3Series,     ui->checkBox_Ir3->isChecked()},
        {pid_ir4Series,     ui->checkBox_Ir4->isChecked()},
        {pid_ir5Series,     ui->checkBox_Ir5->isChecked()},
        {pid_ir6Series,     ui->checkBox_Ir6->isChecked()},
        {pid_ir7Series,     ui->checkBox_Ir7->isChecked()},
        {pid_ir8Series,     ui->checkBox_Ir8->isChecked()}
    }, 50.0);
}


void MainWindow::resetInterface() {
    paramsSynced = false;
    update3DModelConnectionState(false);
    
    // Resetear QSpinBoxes
    QList<QSpinBox *> spinBoxes = this->findChildren<QSpinBox *>();
    for (QSpinBox *spinBox : spinBoxes) {
        bool oldState = spinBox->blockSignals(true);
        spinBox->setValue(0);
        spinBox->blockSignals(oldState);
    }

    // Resetear QLCDNumbers (Sensores IR, Acelerómetro, Giroscopio, etc.)
    QList<QLCDNumber *> lcdNumbers = this->findChildren<QLCDNumber *>();
    for (QLCDNumber *lcd : lcdNumbers) {
        lcd->display(0);
    }
}

void MainWindow::on_sendFrontDistance_clicked() {
    uint8_t buf[3];
    _udat w;
    w.i16[0] = ui->setFrontDistance->value();
    buf[0] = SETFRONTDIST;
    buf[1] = w.ui8[0];
    buf[2] = w.ui8[1];
    sendCommand(buf, 3);
}

void MainWindow::on_sendSideDistance_clicked() {
    uint8_t buf[3];
    _udat w;
    w.i16[0] = ui->setSideDistance->value();
    buf[0] = SETSIDEDIST;
    buf[1] = w.ui8[0];
    buf[2] = w.ui8[1];
    sendCommand(buf, 3);
}

void MainWindow::on_sendLostDistance_clicked() {
    uint8_t buf[3];
    _udat w;
    w.i16[0] = ui->setLostDistance->value();
    buf[0] = SETLOSTDIST;
    buf[1] = w.ui8[0];
    buf[2] = w.ui8[1];
    sendCommand(buf, 3);
}

void MainWindow::on_sendStopCycles_clicked() {
    uint8_t buf[3];
    _udat w;
    w.i16[0] = ui->setStopCycles->value();
    buf[0] = SETSTOPCYCLES;
    buf[1] = w.ui8[0];
    buf[2] = w.ui8[1];
    sendCommand(buf, 3);
}

void MainWindow::on_sendCornerDist_clicked() {
    uint8_t buf[3];
    _udat w;
    w.i16[0] = ui->setCornerDist->value();
    buf[0] = SETCORNERDIST;
    buf[1] = w.ui8[0];
    buf[2] = w.ui8[1];
    sendCommand(buf, 3);
}

void MainWindow::on_sendAlignDist_clicked() {
    uint8_t buf[3];
    _udat w;
    w.i16[0] = ui->setAlignDist->value();
    buf[0] = SETALIGNDIST;
    buf[1] = w.ui8[0];
    buf[2] = w.ui8[1];
    sendCommand(buf, 3);
}

/*
void MainWindow::on_sendPWMLROT_clicked() {
    uint8_t buf[3];
    _udat w;
    w.i16[0] = ui->setPWMLROT->value();
    buf[0] = SETPWMLROT;
    buf[1] = w.ui8[0];
    buf[2] = w.ui8[1];
    sendCommand(buf, 3);
}

void MainWindow::on_sendPWMRROT_clicked() {
    uint8_t buf[3];
    _udat w;
    w.i16[0] = ui->setPWMRROT->value();
    buf[0] = SETPWMRROT;
    buf[1] = w.ui8[0];
    buf[2] = w.ui8[1];
    sendCommand(buf, 3);
}

void MainWindow::on_sendStaticOff_clicked() {
    uint8_t buf[3];
    _udat w;
    w.i16[0] = ui->setStaticOff->value();
    buf[0] = SETSTATICOFF;
    buf[1] = w.ui8[0];
    buf[2] = w.ui8[1];
    sendCommand(buf, 3);
}

void MainWindow::on_sendMovingOff_clicked() {
    uint8_t buf[3];
    _udat w;
    w.i16[0] = ui->setMovingOff->value();
    buf[0] = SETMOVINGOFF;
    buf[1] = w.ui8[0];
    buf[2] = w.ui8[1];
    sendCommand(buf, 3);
}

void MainWindow::on_sendLimitAngle_clicked() {
    uint8_t buf[3];
    buf[0] = SETLIMITANG;
    myWord.i16[0] = ui->setLimitAngle->value();
    buf[1] = myWord.ui8[0];
    buf[2] = myWord.ui8[1];
    sendCommand(buf, 3);
}
*/

void MainWindow::on_sendKiCascada_clicked() {
    uint8_t buf[3];
    buf[0] = SET_KI_EXT;
    myWord.i16[0] = ui->setKiCascada->value();
    buf[1] = myWord.ui8[0];
    buf[2] = myWord.ui8[1];
    sendCommand(buf, 3);
    ui->textBrowserProcessed->append("***KI CASCADA ACTUALIZADO***");
}

void MainWindow::on_sendFilterLPF_clicked() {
    uint8_t buf[3];
    buf[0] = SET_ALFA_LPF;
    myWord.i16[0] = ui->setFilterLPF->value();
    buf[1] = myWord.ui8[0];
    buf[2] = myWord.ui8[1];
    sendCommand(buf, 3);
    ui->textBrowserProcessed->append("***FILTRO LPF CASCADA ACTUALIZADO***");
}

void MainWindow::on_sendVelDampDiv_clicked() {
    uint8_t buf[3];
    buf[0] = SETVELDAMPDIV;
    myWord.i16[0] = ui->setVelDampDiv->value();
    buf[1] = myWord.ui8[0];
    buf[2] = myWord.ui8[1];
    sendCommand(buf, 3);
}

void MainWindow::on_sendVelDampLim_clicked() {
    uint8_t buf[3];
    buf[0] = SETVELDAMPLIM;
    myWord.i16[0] = ui->setVelDampLim->value();
    buf[1] = myWord.ui8[0];
    buf[2] = myWord.ui8[1];
    sendCommand(buf, 3);
}

void MainWindow::on_sendTurnLimit_clicked() {
    uint8_t buf[3];
    buf[0] = SETTURNLIMIT;
    myWord.i16[0] = ui->setTurnLimit->value();
    buf[1] = myWord.ui8[0];
    buf[2] = myWord.ui8[1];
    sendCommand(buf, 3);
    ui->textBrowserProcessed->append("***TURN LIMIT ACTUALIZADO***");
}

// ---------------------------------------------------------------------------
// Exportar CSV de Sensores IR (último minuto)
// ---------------------------------------------------------------------------

double MainWindow::adcToDistanceCm(uint16_t adc) {
    if (adc >= 4085 || adc < 50) {
        return 99.9; // Fuera de rango o sin obstaculo proximo
    }
    if (adc >= 3800) {
        // Calibracion lineal experimental (3906 -> 0 cm, ~0.1865 cm/punto)
        double dist = (adc - 3906.0) * 0.1865;
        if (dist < 2.0) dist = 2.0;
        if (dist > 80.0) dist = 99.9;
        return dist;
    } else {
        // Curva analógica inversa de sensor optico (para lecturas directas 200..3800)
        double dist = 8000.0 / (double)adc;
        if (dist < 2.0) dist = 2.0;
        if (dist > 80.0) dist = 99.9;
        return dist;
    }
}

void MainWindow::exportIrCsvToFile() {
    bool lowerOk = false;
    QString fileNameLower;
    if (!m_irBuffer.isEmpty()) {
        QString defaultNameLower = "ir_sensors_lower_" +
            QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".csv";
        fileNameLower = QFileDialog::getSaveFileName(
            this, "Exportar Sensores IR Inferiores - CSV", defaultNameLower,
            "Archivos CSV (*.csv)");

        if (!fileNameLower.isEmpty()) {
            QFile file(fileNameLower);
            if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                QTextStream out(&file);
                out << "ExportNum;" << m_irExportCount << "\n";
                out << "Timestamp;IR1 (Línea Izq);IR3 (Línea Cen);IR5 (Línea Der);Promedio\n";
                for (const IrSample &s : m_irBuffer) {
                    uint32_t avg = ((uint32_t)s.ir1 + s.ir3 + s.ir5) / 3;
                    out << s.timestamp.toString("hh:mm:ss.zzz") << ";"
                        << s.ir1 << ";"
                        << s.ir3 << ";"
                        << s.ir5 << ";"
                        << avg   << "\n";
                }
                file.close();
                lowerOk = true;
            } else {
                QMessageBox::critical(this, "Error", "No se pudo crear el archivo CSV de sensores inferiores.");
            }
        }
    }

    bool upperOk = false;
    QString fileNameUpper;
    if (!m_upperIrBuffer.isEmpty()) {
        QString defaultNameUpper = "ir_sensors_upper_" +
            QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".csv";
        fileNameUpper = QFileDialog::getSaveFileName(
            this, "Exportar Sensores IR Superiores (Esquivar Objeto) - CSV", defaultNameUpper,
            "Archivos CSV (*.csv)");

        if (!fileNameUpper.isEmpty()) {
            QFile file(fileNameUpper);
            if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                QTextStream out(&file);
                out << "ExportNum;" << m_irExportCount << "\n";
                out << "Timestamp;IR0 (Der 90°);IR2 (Izq 90°);IR4 (Izq 45°);IR6 (Frontal);IR7 (Der 45°);Promedio\n";
                for (const UpperIrSample &s : m_upperIrBuffer) {
                    uint32_t avg = ((uint32_t)s.ir1 + s.ir3 + s.ir5 + s.ir7 + s.ir8) / 5;
                    out << s.timestamp.toString("hh:mm:ss.zzz") << ";"
                        << s.ir1 << ";"
                        << s.ir3 << ";"
                        << s.ir5 << ";"
                        << s.ir7 << ";"
                        << s.ir8 << ";"
                        << avg   << "\n";
                }
                file.close();
                upperOk = true;
            } else {
                QMessageBox::critical(this, "Error", "No se pudo crear el archivo CSV de sensores superiores.");
            }
        }
    }

    if (lowerOk || upperOk) {
        QString msg = QString("Exportación N° %1 exitosa.\n").arg(m_irExportCount);
        if (lowerOk) msg += QString("- %1 muestras inferiores guardadas en:\n  %2\n").arg(m_irBuffer.size()).arg(fileNameLower);
        if (upperOk) msg += QString("- %1 muestras superiores guardadas en:\n  %2\n").arg(m_upperIrBuffer.size()).arg(fileNameUpper);
        QMessageBox::information(this, "Exportación IR exitosa", msg);
    }
}

void MainWindow::on_pushButton_exportIrCsv_clicked() {
    if (m_irBuffer.isEmpty() && m_upperIrBuffer.isEmpty()) {
        QMessageBox::warning(this, "Sin datos",
            "El buffer está vacío. Conectate al robot y esperá que lleguen datos de telemetría.");
        return;
    }
    // Enviar comando EXPORTIRCSV al STM32 para obtener el número de exportación
    uint8_t cmd[2] = { (uint8_t)EXPORTIRCSV, 0x00 };
    sendCommand(cmd, 2);
}

void MainWindow::toggleBlockAutoData(bool blocked) {
    m_isCommBlocked = blocked;

    if (ui->pushButton_blockAutoData) {
        ui->pushButton_blockAutoData->blockSignals(true);
        ui->pushButton_blockAutoData->setChecked(blocked);
        ui->pushButton_blockAutoData->setText(blocked ? "ENVÍO BLOQUEADO (Click para reanudar)" : "BLOQUEAR ENVÍO AUTOMÁTICO");
        ui->pushButton_blockAutoData->blockSignals(false);
    }
    if (ui->pushButton_blockAutoData_comm) {
        ui->pushButton_blockAutoData_comm->blockSignals(true);
        ui->pushButton_blockAutoData_comm->setChecked(blocked);
        ui->pushButton_blockAutoData_comm->setText(blocked ? "ENVÍO BLOQUEADO (Click para reanudar)" : "BLOQUEAR ENVÍO AUTOMÁTICO (TEST MANUAL)");
        ui->pushButton_blockAutoData_comm->blockSignals(false);
    }

    if (ui->label_comm_traffic_status) {
        if (blocked) {
            ui->label_comm_traffic_status->setText("Telemetría periódica: PAUSADA (Canal libre)");
            ui->label_comm_traffic_status->setStyleSheet("color: #ff4d6d; font-weight: bold; font-size: 8pt;");
        } else {
            ui->label_comm_traffic_status->setText("Telemetría periódica: ACTIVA");
            ui->label_comm_traffic_status->setStyleSheet("color: #00f2c3; font-weight: bold; font-size: 8pt;");
        }
    }

    if (blocked) {
        ui->textBrowserProcessed->append("*** TELEMETRÍA AUTOMÁTICA DETENIDA - CANAL DISPONIBLE PARA TEST INDIVIDUAL ***");
    } else {
        ui->textBrowserProcessed->append("*** TELEMETRÍA AUTOMÁTICA REANUDADA ***");
    }
}

void MainWindow::on_pushButton_blockAutoData_clicked(bool checked) {
    toggleBlockAutoData(checked);
}

void MainWindow::on_pushButton_blockAutoData_comm_clicked(bool checked) {
    toggleBlockAutoData(checked);
}

void MainWindow::on_comboBox_CMD_currentIndexChanged(int index) {
    Q_UNUSED(index);
    if (!ui->comboBox_CMD) return;

    if (ui->spinBox_cmdParam) {
        ui->spinBox_cmdParam->setEnabled(true);
    }
}

void MainWindow::on_pushButton_sendCommand_clicked() {
    if (!ui->comboBox_CMD) return;

    uint8_t cmdId = ui->comboBox_CMD->currentData().toUInt();
    uint8_t payload[16];
    uint8_t length = 0;
    _udat w;

    payload[length++] = cmdId;

    bool isGetCommand = (cmdId == GETALIVE || cmdId == GETFIRMWARE || cmdId == GETMPU ||
                         cmdId == GETADC || cmdId == GETINTERNALDATA || cmdId == GETPIDBALANCE ||
                         cmdId == EXPORTIRCSV);

    int paramVal = ui->spinBox_cmdParam ? ui->spinBox_cmdParam->value() : 0;

    if (cmdId == SETROBOTMODE) {
        payload[length++] = (uint8_t)paramVal;
    } else if (!isGetCommand || paramVal != 0) {
        w.i16[0] = (short)paramVal;
        payload[length++] = w.ui8[0];
        payload[length++] = w.ui8[1];
    }

    sendCommand(payload, length);

    QString cmdName = ui->comboBox_CMD->currentText();
    QString medium = "DESCONECTADO";
    if (QTcpSocketClient && QTcpSocketClient->isOpen()) medium = isSoftApActive() ? "SOFTAP TCP" : "STATION TCP";
    else if (QSerialPort1 && QSerialPort1->isOpen()) medium = "SERIAL";
    else if (QUdpSocket1 && QUdpSocket1->isOpen()) medium = isSoftApActive() ? "SOFTAP UDP" : "STATION UDP";

    QString logMsg = QString("*** COMANDO ENVIADO POR %1: %2 ***").arg(medium, cmdName);
    if (!isGetCommand || paramVal != 0) {
        logMsg += QString(" [Valor: %1]").arg(paramVal);
    }
    ui->textBrowserProcessed->append(logMsg);
    if (ui->label_cmd_result) {
        ui->label_cmd_result->setText(QString("Enviado [%1]: %2%3")
            .arg(medium, cmdName)
            .arg((!isGetCommand || paramVal != 0) ? QString(" (Valor: %1)").arg(paramVal) : ""));
    }

    // Historial de comandos enviados:
    // El último comando en color ROJO (#ff4d6d) y negrita,
    // comandos viejos en gris (#7f8599) y normal.
    if (ui->listWidget_cmdHistory) {
        for (int i = 0; i < ui->listWidget_cmdHistory->count(); ++i) {
            QListWidgetItem *oldItem = ui->listWidget_cmdHistory->item(i);
            if (oldItem) {
                oldItem->setForeground(QBrush(QColor("#7f8599")));
                QFont f = oldItem->font();
                f.setBold(false);
                oldItem->setFont(f);
            }
        }

        QString itemText = QString("[%1] [%2] %3%4")
            .arg(QTime::currentTime().toString("hh:mm:ss"))
            .arg(medium)
            .arg(cmdName)
            .arg((!isGetCommand || paramVal != 0) ? QString(" (Valor: %1)").arg(paramVal) : "");

        QListWidgetItem *newItem = new QListWidgetItem(itemText);
        newItem->setForeground(QBrush(QColor("#ff4d6d"))); // ROJO para el último comando enviado
        QFont boldFont = newItem->font();
        boldFont.setBold(true);
        newItem->setFont(boldFont);

        ui->listWidget_cmdHistory->addItem(newItem);
        ui->listWidget_cmdHistory->scrollToBottom();
    }
}


// =========================================================
// MODO JOYSTICK: CONTROL MANUAL / REMOTO DESDE PC
// =========================================================

void MainWindow::switchNavTab(int targetIndex) {
    int currentIndex = ui->stackedWidget->currentIndex();
    if (currentIndex == targetIndex) {
        updateNavSelection(targetIndex);
        return;
    }

    // Si actualmente estamos en el Modo Joystick (pestaña 3), proteger contra cambios accidentales
    if (currentIndex == 3) {
        QPushButton *senderBtn = qobject_cast<QPushButton*>(sender());
        // Si el evento fue indirecto (foco, teclado, etc.) y el cursor no está sobre el botón, cancelar
        if (senderBtn && !senderBtn->underMouse()) {
            ui->btn_nav_goto->setChecked(true);
            return;
        }

        // Preguntar al usuario antes de salir utilizando la ventana pop-up configurada en el .ui
        if (!confirmJoystickExit()) {
            ui->btn_nav_goto->setChecked(true);
            return;
        }

        // Al confirmar salida, detener giros activos por seguridad
        m_isRotatingLeft = false;
        m_isRotatingRight = false;
        if (m_gotoTurnKeepAliveTimer && m_gotoTurnKeepAliveTimer->isActive()) {
            m_gotoTurnKeepAliveTimer->stop();
        }
        sendGoToTurn(0, 0);
    }

    // Pestaña de Configuración (índice 2): proteger con contraseña en pop-up
    if (targetIndex == 2) {
        if (!requestConfigPassword()) {
            updateNavSelection(currentIndex);
            return;
        }
    }

    ui->stackedWidget->setCurrentIndex(targetIndex);
    updateNavSelection(targetIndex);
    if (targetIndex == 3) {
        resetGoToYaw();
        this->setFocus();
    }
}

void MainWindow::on_btn_nav_infrarrojos_clicked() {
    switchNavTab(0);
}

void MainWindow::on_btn_nav_visualizacion_clicked() {
    switchNavTab(1);
}

void MainWindow::on_btn_nav_tuning_clicked() {
    switchNavTab(2);
}

bool MainWindow::requestConfigPassword() {
    if (!myPasswordDialog) return true;

    ui->stackedWidget_password->setCurrentWidget(ui->page_password_input);
    ui->lineEdit_configPassword->clear();
    resetPasswordDialogStyle();

    myPasswordDialog->adjustSize();
    if (this->isVisible()) {
        QPoint center = this->geometry().center();
        myPasswordDialog->move(center.x() - myPasswordDialog->width() / 2,
                               center.y() - myPasswordDialog->height() / 2);
    }

    ui->lineEdit_configPassword->setFocus();

    int result = myPasswordDialog->exec();
    return (result == QDialog::Accepted);
}

void MainWindow::on_btn_password_accept_clicked() {
    if (!myPasswordDialog || !myPasswordDialog->isVisible()) return;

    QString entered = ui->lineEdit_configPassword->text().trimmed();
    if (entered.compare("mcc25", Qt::CaseInsensitive) == 0 ||
        entered == "1234" ||
        entered.compare("admin", Qt::CaseInsensitive) == 0) {
        myPasswordDialog->accept();
    } else {
        setPasswordDialogError();
    }
}

void MainWindow::resetPasswordDialogStyle() {
    ui->label_passwordTitle->setText("ACCESO RESTRINGIDO");
    ui->label_passwordTitle->setStyleSheet("color: #00f2c3; letter-spacing: 0.5px; font-weight: bold;");
    ui->frame_passwordContainer->setStyleSheet(
        "#frame_passwordContainer { "
        "background-color: #161827; "
        "border: 1.5px solid #2e3553; "
        "border-radius: 12px; "
        "}"
    );
    ui->lineEdit_configPassword->setStyleSheet(
        "QLineEdit#lineEdit_configPassword { "
        "background-color: #121422; "
        "border: 1.5px solid #3d4566; "
        "border-radius: 6px; "
        "color: #ffffff; "
        "padding: 4px 12px; "
        "} "
        "QLineEdit#lineEdit_configPassword:focus { "
        "border: 1.5px solid #00f2c3; "
        "}"
    );
    ui->lineEdit_configPassword->setPlaceholderText("Contraseña de acceso...");
    ui->btn_password_accept->setStyleSheet(
        "QPushButton#btn_password_accept { "
        "background-color: #00f2c3; "
        "color: #081a10; "
        "border: none; "
        "border-radius: 6px; "
        "font-weight: bold; "
        "padding: 6px 14px; "
        "} "
        "QPushButton#btn_password_accept:hover { "
        "background-color: #26ff8a; "
        "} "
        "QPushButton#btn_password_accept:pressed { "
        "background-color: #00c99a; "
        "}"
    );
    ui->btn_password_accept->setText("Ingresar");
    ui->btn_password_accept->setDefault(true);
    ui->btn_password_accept->setAutoDefault(true);
    ui->btn_password_cancel->setDefault(false);
    ui->btn_password_cancel->setAutoDefault(false);
    ui->label_passwordFeedback->clear();
}

void MainWindow::setPasswordDialogError() {
    ui->label_passwordTitle->setText("CONTRASEÑA INCORRECTA");
    ui->label_passwordTitle->setStyleSheet("color: #ff2a6d; letter-spacing: 0.5px; font-weight: bold;");
    ui->frame_passwordContainer->setStyleSheet(
        "#frame_passwordContainer { "
        "background-color: #1a131f; "
        "border: 1.5px solid #ff2a6d; "
        "border-radius: 12px; "
        "}"
    );
    ui->lineEdit_configPassword->setStyleSheet(
        "QLineEdit#lineEdit_configPassword { "
        "background-color: #1c1218; "
        "border: 1.5px solid #ff2a6d; "
        "border-radius: 6px; "
        "color: #ffffff; "
        "padding: 4px 12px; "
        "} "
        "QLineEdit#lineEdit_configPassword:focus { "
        "border: 1.5px solid #ff2a6d; "
        "}"
    );
    ui->lineEdit_configPassword->clear();
    ui->lineEdit_configPassword->setPlaceholderText("Contraseña incorrecta...");
    ui->btn_password_accept->setStyleSheet(
        "QPushButton#btn_password_accept { "
        "background-color: #ff2a6d; "
        "color: #ffffff; "
        "border: none; "
        "border-radius: 6px; "
        "font-weight: bold; "
        "padding: 6px 14px; "
        "} "
        "QPushButton#btn_password_accept:hover { "
        "background-color: #ff4d84; "
        "} "
        "QPushButton#btn_password_accept:pressed { "
        "background-color: #c2185b; "
        "}"
    );
    ui->btn_password_accept->setText("Reintentar");
    ui->btn_password_accept->setDefault(true);
    ui->btn_password_accept->setAutoDefault(true);
    ui->btn_password_cancel->setDefault(false);
    ui->btn_password_cancel->setAutoDefault(false);
    ui->label_passwordFeedback->setStyleSheet("color: #ff2a6d; font-weight: bold; font-size: 8.5pt;");
    ui->label_passwordFeedback->setText("⚠️ Contraseña incorrecta. Intente nuevamente.");
    if (myPasswordDialog) {
        myPasswordDialog->adjustSize();
    }
    ui->lineEdit_configPassword->setFocus();
}

void MainWindow::on_btn_password_cancel_clicked() {
    if (!myPasswordDialog) return;
    myPasswordDialog->reject();
}

void MainWindow::on_btn_password_retry_clicked() {
    if (!myPasswordDialog) return;
    ui->stackedWidget_password->setCurrentWidget(ui->page_password_input);
    ui->lineEdit_configPassword->clear();
    resetPasswordDialogStyle();
    if (myPasswordDialog) {
        myPasswordDialog->adjustSize();
    }
    ui->lineEdit_configPassword->setFocus();
}

bool MainWindow::confirmJoystickExit() {
    if (!myJoystickExitDialog) return true;

    myJoystickExitDialog->adjustSize();
    if (this->isVisible()) {
        QPoint center = this->geometry().center();
        myJoystickExitDialog->move(center.x() - myJoystickExitDialog->width() / 2,
                                   center.y() - myJoystickExitDialog->height() / 2);
    }

    ui->btn_joystick_exit_cancel->setFocus();

    int result = myJoystickExitDialog->exec();
    return (result == QDialog::Accepted);
}

void MainWindow::on_btn_joystick_exit_confirm_clicked() {
    if (!myJoystickExitDialog) return;
    myJoystickExitDialog->accept();
}

void MainWindow::on_btn_joystick_exit_cancel_clicked() {
    if (!myJoystickExitDialog) return;
    myJoystickExitDialog->reject();
}

void MainWindow::on_btn_nav_goto_clicked() {
    switchNavTab(3);
}

void MainWindow::on_btn_mode_goto_clicked() {
    sendRobotMode(4);
    switchNavTab(3);
}

void MainWindow::on_btn_goto_up_clicked() {
    m_currentSetpoint += m_gotoStep;
    sendGoToSetpoint(m_currentSetpoint);
    updateGoToAngleDisplays();
}

void MainWindow::on_btn_goto_down_clicked() {
    m_currentSetpoint -= m_gotoStep;
    sendGoToSetpoint(m_currentSetpoint);
    updateGoToAngleDisplays();
}

void MainWindow::on_btn_goto_left_pressed() {
    m_isRotatingLeft = true;
    m_isRotatingRight = false;
    sendGoToTurn(m_gotoTurnIntensity, m_gotoTurnDuration);
    if (m_gotoTurnKeepAliveTimer && !m_gotoTurnKeepAliveTimer->isActive()) {
        m_gotoTurnKeepAliveTimer->start();
    }
}

void MainWindow::on_btn_goto_left_released() {
    m_isRotatingLeft = false;
    if (!m_isRotatingRight) {
        if (m_gotoTurnKeepAliveTimer) m_gotoTurnKeepAliveTimer->stop();
        sendGoToTurn(0, 0);
    }
}

void MainWindow::on_btn_goto_right_pressed() {
    m_isRotatingRight = true;
    m_isRotatingLeft = false;
    sendGoToTurn(-m_gotoTurnIntensity, m_gotoTurnDuration);
    if (m_gotoTurnKeepAliveTimer && !m_gotoTurnKeepAliveTimer->isActive()) {
        m_gotoTurnKeepAliveTimer->start();
    }
}

void MainWindow::on_btn_goto_right_released() {
    m_isRotatingRight = false;
    if (!m_isRotatingLeft) {
        if (m_gotoTurnKeepAliveTimer) m_gotoTurnKeepAliveTimer->stop();
        sendGoToTurn(0, 0);
    }
}

void MainWindow::on_btn_goto_center_clicked() {
    m_currentSetpoint = 0;
    sendGoToSetpoint(0);
    updateGoToAngleDisplays();
}

void MainWindow::on_spinBox_gotoStep_valueChanged(double val) {
    m_gotoStep = qRound(val * 100.0);
}

void MainWindow::on_spinBox_gotoTurnIntensity_valueChanged(int val) {
    m_gotoTurnIntensity = (int16_t)val;
}

void MainWindow::on_spinBox_gotoTurnDuration_valueChanged(int val) {
    m_gotoTurnDuration = (uint16_t)val;
}

void MainWindow::on_btn_goto_reset_yaw_clicked() {
    resetGoToYaw();
}

void MainWindow::resetGoToYaw() {
    m_gotoStartYaw = yawAcumulado;
    m_gotoRelativeYaw = 0.0f;
    updateGoToAngleDisplays();
}

void MainWindow::sendGoToSetpoint(int32_t sp_val) {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w;
    payload[index++] = SETSETPOINT;
    w.i32 = sp_val;
    payload[index++] = w.ui8[0];
    payload[index++] = w.ui8[1];
    sendCommand(payload, index);

    if (ui->setSetpoint) {
        ui->setSetpoint->setValue(sp_val);
    }
    QString logMsg = QString("***SETPOINT JOYSTICK ENVIADO: %1 (%2°)***").arg(sp_val).arg(sp_val / 100.0, 0, 'f', 2);
    addLogEntry(logMsg, "TX");
    ui->textBrowserProcessed->append(logMsg);
}

void MainWindow::sendGoToTurn(int16_t turn_val, uint16_t turn_ms) {
    uint8_t payload[10];
    uint8_t index = 0;
    _udat w_val, w_ms;
    payload[index++] = SETGOTOTURN;
    w_val.i16[0] = turn_val;
    payload[index++] = w_val.ui8[0];
    payload[index++] = w_val.ui8[1];
    w_ms.ui16[0] = turn_ms;
    payload[index++] = w_ms.ui8[0];
    payload[index++] = w_ms.ui8[1];
    sendCommand(payload, index);
}

void MainWindow::updateGoToAngleDisplays() {
    if (ui->lbl_goto_setpoint_deg) {
        float sp_deg = m_currentSetpoint / 100.0f;
        ui->lbl_goto_setpoint_deg->setText(QString("%1%2°")
            .arg(sp_deg >= 0 ? "+" : "")
            .arg(QString::number(sp_deg, 'f', 2)));
    }
    if (ui->lbl_goto_setpoint_raw) {
        ui->lbl_goto_setpoint_raw->setText(QString("Valor crudo: %1").arg(m_currentSetpoint));
    }
    if (ui->lbl_goto_current_deg) {
        ui->lbl_goto_current_deg->setText(QString("%1%2°")
            .arg(m_currentAngle >= 0 ? "+" : "")
            .arg(QString::number(m_currentAngle, 'f', 2)));
    }
    if (ui->lbl_goto_error) {
        float err_deg = (m_currentSetpoint / 100.0f) - m_currentAngle;
        ui->lbl_goto_error->setText(QString("Error: %1%2°")
            .arg(err_deg >= 0 ? "+" : "")
            .arg(QString::number(err_deg, 'f', 2)));
    }
    if (ui->lbl_goto_yaw_deg) {
        ui->lbl_goto_yaw_deg->setText(QString("%1%2°")
            .arg(m_gotoRelativeYaw >= 0 ? "+" : "")
            .arg(QString::number(m_gotoRelativeYaw, 'f', 2)));
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
    if (ui->stackedWidget && ui->stackedWidget->currentIndex() == 3) {
        switch (event->key()) {
        case Qt::Key_Up:
            if (ui->btn_goto_up) ui->btn_goto_up->setDown(true);
            on_btn_goto_up_clicked();
            event->accept();
            return;
        case Qt::Key_Down:
            if (ui->btn_goto_down) ui->btn_goto_down->setDown(true);
            on_btn_goto_down_clicked();
            event->accept();
            return;
        case Qt::Key_Left:
            if (!event->isAutoRepeat()) {
                if (ui->btn_goto_left) ui->btn_goto_left->setDown(true);
                on_btn_goto_left_pressed();
            }
            event->accept();
            return;
        case Qt::Key_Right:
            if (!event->isAutoRepeat()) {
                if (ui->btn_goto_right) ui->btn_goto_right->setDown(true);
                on_btn_goto_right_pressed();
            }
            event->accept();
            return;
        case Qt::Key_R:
            if (!event->isAutoRepeat()) {
                if (ui->btn_goto_center) ui->btn_goto_center->setDown(true);
                on_btn_goto_center_clicked();
            }
            event->accept();
            return;
        default:
            break;
        }
    }
    QMainWindow::keyPressEvent(event);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event) {
    if (ui->stackedWidget && ui->stackedWidget->currentIndex() == 3) {
        switch (event->key()) {
        case Qt::Key_Up:
            if (ui->btn_goto_up) ui->btn_goto_up->setDown(false);
            event->accept();
            return;
        case Qt::Key_Down:
            if (ui->btn_goto_down) ui->btn_goto_down->setDown(false);
            event->accept();
            return;
        case Qt::Key_Left:
            if (!event->isAutoRepeat()) {
                if (ui->btn_goto_left) ui->btn_goto_left->setDown(false);
                on_btn_goto_left_released();
            }
            event->accept();
            return;
        case Qt::Key_Right:
            if (!event->isAutoRepeat()) {
                if (ui->btn_goto_right) ui->btn_goto_right->setDown(false);
                on_btn_goto_right_released();
            }
            event->accept();
            return;
        case Qt::Key_R:
            if (!event->isAutoRepeat()) {
                if (ui->btn_goto_center) ui->btn_goto_center->setDown(false);
            }
            event->accept();
            return;
        default:
            break;
        }
    }
    QMainWindow::keyReleaseEvent(event);
}

void MainWindow::on_btn_car_stop_clicked() {
    // 1. Detener temporizadores de giros continuos de Joystick
    if (m_gotoTurnKeepAliveTimer) {
        m_gotoTurnKeepAliveTimer->stop();
    }
    m_isRotatingLeft = false;
    m_isRotatingRight = false;

    // 2. Enviar comando de parada total / standby (Modo 0)
    sendRobotMode(0);

    // 3. Actualizar botones de modo en la interfaz (todos desmarcados)
    updateRobotModeUI(0);

    // 4. Notificación y registro
    addLogEntry("*** COMANDO STOP: APAGADO TOTAL DEL ROBOT (STANDBY) ***", "TX");
    ui->textBrowserProcessed->append("<span style='color: #ff2a6d; font-weight: bold;'>*** STOP ACTIVADO: EL AUTITO SE HA APAGADO COMPLETAMENTE (STANDBY) ***</span>");
}

void MainWindow::on_btn_set_compass_ref_clicked() {
    float refVal = -yawAcumulado;
    if (m_compassWidget) {
        m_compassWidget->setReference(refVal);
        refVal = m_compassWidget->reference();
    }
    addLogEntry(QString("*** BRÚJULA: REFERENCIA FIJADA EN %1° ***").arg(refVal, 0, 'f', 1), "INFO");
    ui->textBrowserProcessed->append(QString("<span style='color: #00f2c3; font-weight: bold;'>📍 REFERENCIA DE BRÚJULA FIJADA: %1°</span>").arg(refVal, 0, 'f', 1));
}

void MainWindow::on_btn_reset_compass_ref_clicked() {
    if (m_compassWidget) {
        m_compassWidget->resetReference();
    }
    addLogEntry("*** BRÚJULA: REFERENCIA REINICIADA A 0° ***", "INFO");
    ui->textBrowserProcessed->append("<span style='color: #c5c9d8;'>🔄 REFERENCIA DE BRÚJULA REINICIADA A 0°</span>");
}

bool MainWindow::isRobotConnected() const {
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    bool isTcpActive = (m_isTcpMode && QTcpSocketClient && QTcpSocketClient->isOpen() && QTcpSocketClient->state() == QAbstractSocket::ConnectedState);
    bool isSerialActive = (QSerialPort1 && QSerialPort1->isOpen());
    bool isUdpActive = (!m_isTcpMode && QUdpSocket1 && QUdpSocket1->isOpen());

    if (!isSerialActive && !isUdpActive && !isTcpActive) {
        return false;
    }

    if (m_isCommBlocked) {
        return true;
    }

    if (m_lastRxTime > 0 && (now - m_lastRxTime) < 3000) {
        return true;
    }

    return false;
}

void MainWindow::update3DModelConnectionState(bool connected) {
    if (m_is3DModelConnected == connected && ui->AutoWidget && ui->AutoWidget->rootObject()) {
        return;
    }
    m_is3DModelConnected = connected;

    if (ui->AutoWidget && ui->AutoWidget->rootObject()) {
        ui->AutoWidget->rootObject()->setProperty("isConnected", connected);
    }
}


