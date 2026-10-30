/**
 * @file mainwindow.h
 * @brief Ventana principal, orquestador de telemetría y centro de comando de la Estación Terrena.
 * @author Gonzalo Gomez
 * @date 2026
 *
 * @details Este archivo define la clase `MainWindow`, la entidad central de la aplicación de escritorio.
 * Es responsable de:
 * - Gestionar los canales de comunicación física y lógica (Serial RS-232/USB, Wi-Fi UDP y Wi-Fi TCP Sockets).
 * - Decodificar en tiempo real la trama binaria del protocolo UNER mediante una máquina de estados finita.
 * - Despachar los lazos de telemetría hacia las gráficas en tiempo real (`QChart`), el modelo 3D (`Scene3D.qml`)
 *   y los instrumentos virtuales (`CompassWidget`).
 * - Transmitir setpoints de control, ganancias PID ($K_p, K_i, K_d$), offsets y comandos de modo (Balance,
 *   Seguidor de Línea, Evasión de Obstáculos y Control Manual / GoTo Joystick).
 * - Mantener el historial de eventos (`LogEntry`) y exportación a archivos CSV/Excel.
 *
 * @see CompassWidget
 * @see internal_data
 * @see mainwindow.cpp
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QResizeEvent>
#include <QDialog>
#include <QVBoxLayout>

#include <QtSerialPort/QSerialPort>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QLabel>
#include <QInputDialog>
#include <QTimer>
#include <QTime>
#include <QSerialPortInfo>

#include <QQuickWidget>
#include <QQuickItem>

#include <QElapsedTimer>
#include <QKeyEvent>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QtCharts/QLegendMarker>
#include "compasswidget.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

/**
 * @ingroup group_gui
 * @class MainWindow
 * @brief Orquestador central de la estación terrena para el robot móvil de dos ruedas.
 * @details Integra la interfaz de usuario, la máquina de decodificación del protocolo binario,
 * los lazos de control manual por teclado/interfaz, y las vistas de telemetría 2D/3D.
 *
 * \startuml
 * class MainWindow {
 *   - ui : Ui::MainWindow*
 *   - QSerialPort1 : QSerialPort*
 *   - QUdpSocket1 : QUdpSocket*
 *   - QTcpServer1 : QTcpServer*
 *   - QTcpSocketClient : QTcpSocket*
 *   - m_compassWidget : CompassWidget*
 *   - AutoWidget : QQuickWidget*
 *   + MainWindow(parent : QWidget*)
 *   + ~MainWindow()
 *   + sendCommand(buf : uint8_t*, length : uint8_t) : void
 *   + sendRobotMode(modeId : uint8_t) : void
 *   + decodeData(datosRx : uint8_t*, source : uint8_t) : void
 * }
 * MainWindow --|> QMainWindow
 * MainWindow --> CompassWidget : "integra"
 * MainWindow --> "_eProtocolo" : "parser"
 * MainWindow --> "_eCmd" : "despacho"
 * \enduml
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief Constructor de la ventana principal.
     * @param parent Puntero al widget padre de la jerarquía Qt (por defecto nullptr).
     * @post Inicializa la interfaz gráfica, charts de telemetría, timers de sondeo y sockets de red.
     */
    MainWindow(QWidget *parent = nullptr);

    /**
     * @brief Destructor de la ventana principal.
     * @post Libera los recursos de red, puerto serie, timers y memoria dinámica asignada a la UI.
     */
    ~MainWindow();

protected:
    /**
     * @brief Captura la pulsación de teclas del teclado físico para el control GoTo/Joystick.
     * @param event Evento de teclado de Qt con información de la tecla pulsada.
     */
    void keyPressEvent(QKeyEvent *event) override;

    /**
     * @brief Captura la liberación de teclas físicas para detener giros continuos en modo Joystick.
     * @param event Evento de liberación de tecla de Qt.
     */
    void keyReleaseEvent(QKeyEvent *event) override;

private slots:

    /**
     * @ingroup group_comm_host
     * @brief Slot invocado ante la señal `readyRead` del puerto serie `QSerialPort1`.
     * @details Lee los bytes disponibles en el buffer de recepción UART y alimenta la máquina de estados
     * de decodificación del protocolo UNER. Reinicia el contador de timeout de comunicaciones.
     * @pre El puerto serie debe encontrarse en estado abierto (`isOpen() == true`).
     * @post Actualiza el timestamp `m_lastRxTime`.
     */
    void dataReceived();

    /**
     * @ingroup group_comm_host
     * @brief Decodifica el payload de una trama válida verificada por checksum.
     * @details Desempaqueta las variables de telemetría según el identificador de comando (`_eCmd`),
     * actualizando los indicadores numéricos, gráficos `QLineSeries` y orientación del modelo 3D.
     *
     * \startuml
     * title Flujo de Decodificación y Despacho de Telemetría (decodeData)
     * start
     * :Recibir datosRx y source (SERIAL/UDP/TCP);
     * :Leer ID de comando (datosRx[0]);
     * switch (ID de comando)
     * case (GETALIVE)
     *   :Actualizar latido de vida y firmware;
     * case (GETMPU)
     *   :Desempaquetar acelerómetros (ax, ay, az);
     *   :Desempaquetar giróscopos (gx, gy, gz);
     *   :Desempaquetar ángulos (pitch, roll, yaw);
     *   :Actualizar Chart Orientación y Scene3D.qml;
     *   :Actualizar brújula m_compassWidget;
     * case (GETADC)
     *   :Extraer 8 canales de sensores infrarrojos;
     *   :Almacenar en buffer circular m_irBuffer;
     *   :Actualizar serie de telemetría IR;
     * case (GETPIDBALANCE)
     *   :Extraer términos P, I, D y salida PWM;
     *   :Graficar curva de respuesta temporal PID;
     * case (GETINTERNALDATA)
     *   :Enviar métricas a diálogo internal_data;
     * endswitch
     * :Actualizar contador m_countReceived y timestamp;
     * stop
     * \enduml
     *
     * @param datosRx Puntero al buffer con el payload decodificado.
     * @param source Identificador del medio de origen (0: UDP, 1: Serial, 2: TCP).
     */
    void decodeData(uint8_t *datosRx, uint8_t source);

    /**
     * @ingroup group_comm_host
     * @brief Temporizador de watchdog periódico (ejecutado cada 100 ms).
     * @details Verifica si se ha superado el tiempo límite sin recibir tramas del robot.
     * Si expira el tiempo, conmuta el indicador de estado a "DESCONECTADO" y activa la rotación
     * lenta de exhibición en el modelo 3D.
     */
    void timeOut();

    /**
     * @ingroup group_comm_host
     * @brief Slot invocado al recibir datagramas UDP en el socket `QUdpSocket1`.
     */
    void OnUdpRxData();

    /**
     * @ingroup group_comm_host
     * @brief Slot invocado cuando un cliente TCP (el robot en modo Station/SoftAP) solicita conexión.
     */
    void OnTcpNewConnection();

    /**
     * @ingroup group_comm_host
     * @brief Slot invocado al recibir un flujo de bytes a través del socket cliente TCP activo.
     */
    void OnTcpRxData();

    /**
     * @ingroup group_comm_host
     * @brief Slot invocado al desconectarse el socket TCP del robot.
     */
    void OnTcpDisconnected();

    /**
     * @brief Conecta o desconecta el puerto serie seleccionado en el combo box.
     */
    void on_pushButton_connectSerial_clicked();

    /**
     * @brief Inicializa o cierra el socket UDP en el puerto local y remoto especificado.
     */
    void on_pushButton_connectUdp_clicked();

    /**
     * @brief Alterna entre modo de transporte Wi-Fi UDP y TCP.
     */
    void on_pushButton_protocol_wifi_clicked();

    /**
     * @ingroup group_telemetry
     * @brief Rutina periódica de interrogación (polling) de telemetría hacia el robot.
     */
    void getData();

    /**
     * @ingroup group_comm_host
     * @brief Envía un búfer de datos binarios a través del enlace serie UART.
     * @param buf Puntero al búfer de salida formateado con el protocolo UNER.
     * @param length Cantidad de bytes a transmitir.
     * @pre `QSerialPort1->isOpen()` debe ser `true`.
     */
    void sendSerial(uint8_t *buf, uint8_t length);

    /**
     * @ingroup group_comm_host
     * @brief Envía un búfer de datos binarios encapsulado en un datagrama UDP.
     * @param buf Puntero al búfer de salida.
     * @param length Longitud de la trama en bytes.
     */
    void sendUdp(uint8_t *buf, uint8_t length);

    /**
     * @ingroup group_comm_host
     * @brief Despachador polimórfico de comandos hacia el robot según el enlace activo.
     * @details Evalúa dinámicamente si la conexión activa es Serial, TCP o UDP,
     * y rutea la trama hacia el controlador correspondiente.
     *
     * \startuml
     * title Despacho Polimórfico de Comandos (sendCommand)
     * start
     * :Recibir buffer y longitud;
     * if (QSerialPort1->isOpen()?) then (si)
     *   :Llamar sendSerial();
     * else (no)
     *   if (m_isTcpMode && QTcpSocketClient conectado?) then (si)
     *     :Llamar sendTcp();
     *   else (no)
     *     if (QUdpSocket1 bound?) then (si)
     *       :Llamar sendUdp();
     *     else (no)
     *       :Notificar canal no disponible;
     *     endif
     *   endif
     * endif
     * stop
     * \enduml
     *
     * @param buf Puntero al búfer de la trama binaria.
     * @param length Cantidad total de bytes a transmitir.
     */
    void sendCommand(uint8_t *buf, uint8_t length);

    /**
     * @brief Filtro de eventos para interceptar clics y atajos en widgets secundarios.
     * @param watched Objeto monitoreado.
     * @param event Evento interceptado.
     * @return bool `true` si el evento fue consumido, `false` para propagar.
     */
    bool eventFilter(QObject *watched, QEvent *event);

    // Slots de configuración de ganancias PID de Balance
    void on_sendBalanceKp_clicked();
    void on_sendBalanceKi_clicked();
    void on_sendBalanceKd_clicked();

    // Slots de configuración de ganancias PID de Línea
    void on_sendLineKp_clicked();
    void on_sendLineKd_clicked();

    // Slots de límites de potencia PWM
    void on_sendPWMMINL_clicked();
    void on_sendPWMMINR_clicked();
    void on_sendSetpoint_clicked();
    void on_sendAttackSetpoint_clicked();
    void on_setAttackSetpoint_valueChanged(int arg1);

    void on_sendPWML_clicked();
    void on_sendPWMR_clicked();
    void on_sendOFFSETL_clicked();
    void on_sendOFFSETR_clicked();
    void on_sendCustomTurn_clicked();
    void on_sendKpCascada_clicked();

    // Slots de calibración de evasión de obstáculos
    void on_sendFrontDistance_clicked();
    void on_sendSideDistance_clicked();
    void on_sendLostDistance_clicked();
    void on_sendStopCycles_clicked();
    void on_sendCornerDist_clicked();
    void on_sendAlignDist_clicked();

    void on_sendKiCascada_clicked();
    void on_sendFilterLPF_clicked();
    void on_sendVelDampDiv_clicked();
    void on_sendVelDampLim_clicked();
    void on_sendTurnLimit_clicked();
    void on_sendWallKp_clicked();
    void on_sendWallKd_clicked();
    void on_sendFrontKp_clicked();
    void on_sendFrontKd_clicked();
    void on_sendDodgeDir_clicked();
    void on_sendDodgeBiasTime_clicked();
    void on_sendDodgeBiasMult_clicked();
    void on_pushButton_setSoftAp_clicked();

    // Control de bloqueo de telemetría y comandos directos
    void on_pushButton_blockAutoData_clicked(bool checked);
    void on_pushButton_blockAutoData_comm_clicked(bool checked);
    void on_pushButton_sendCommand_clicked();
    void on_comboBox_CMD_currentIndexChanged(int index);
    void toggleBlockAutoData(bool blocked);

    // Acciones de la barra de menú superior
    void on_actionConn_triggered();
    void on_actionOpenDebug_triggered();
    void on_actionDocu_triggered();
    void on_actionAcercaDe_triggered();

    // Conmutación de modos de control del robot
    void on_btn_mode_balance_clicked();
    void on_btn_mode_line_clicked();
    void on_btn_mode_dodge_clicked();

    /**
     * @ingroup group_navigation
     * @brief Transmite al robot la orden de cambio de modo operativo.
     * @param modeId Identificador del modo (0: STANDBY, 1: BALANCE, 2: LINE, 3: DODGE, 4: JOYSTICK).
     * @post Envía trama de comando SETROBOTMODE (0xD2).
     */
    void sendRobotMode(uint8_t modeId);

    /**
     * @brief Actualiza la apariencia visual de los botones de modo en la GUI.
     * @param mode Modo activo actual.
     */
    void updateRobotModeUI(uint8_t mode);

    /**
     * @brief Actualiza los estilos visuales de las pestañas de navegación principales.
     * @param index Índice de la pestaña seleccionada.
     */
    void updateNavSelection(int index);

    /**
     * @brief Conmuta ordenadamente la vista hacia una pestaña específica.
     * @param targetIndex Índice de destino.
     */
    void switchNavTab(int targetIndex);

    /**
     * @brief Despliega el diálogo modal de validación de contraseña para la pestaña Configuración.
     * @return bool `true` si la clave ingresada es correcta, `false` en caso contrario.
     */
    bool requestConfigPassword();

    void on_btn_password_accept_clicked();
    void on_btn_password_cancel_clicked();
    void on_btn_password_retry_clicked();
    void resetPasswordDialogStyle();
    void setPasswordDialogError();

    /**
     * @brief Solicita confirmación al usuario antes de salir del modo de control manual Joystick.
     * @return bool `true` si el usuario confirma la salida.
     */
    bool confirmJoystickExit();
    void on_btn_joystick_exit_confirm_clicked();
    void on_btn_joystick_exit_cancel_clicked();

    // Botones de navegación lateral
    void on_btn_nav_infrarrojos_clicked();
    void on_btn_nav_visualizacion_clicked();
    void on_btn_nav_tuning_clicked();
    void on_btn_nav_goto_clicked();
    void on_btn_mode_goto_clicked();

    // Control cruceta del modo Joystick / GoTo
    void on_btn_goto_up_clicked();
    void on_btn_goto_down_clicked();
    void on_btn_goto_left_pressed();
    void on_btn_goto_left_released();
    void on_btn_goto_right_pressed();
    void on_btn_goto_right_released();
    void on_btn_goto_center_clicked();

    void on_spinBox_gotoStep_valueChanged(double val);
    void on_spinBox_gotoTurnIntensity_valueChanged(int val);
    void on_spinBox_gotoTurnDuration_valueChanged(int val);
    void on_btn_goto_reset_yaw_clicked();

    /**
     * @ingroup group_navigation
     * @brief Restablece a cero la referencia de giro Yaw en el visor y en el robot.
     */
    void resetGoToYaw();

    /**
     * @ingroup group_navigation
     * @brief Transmite un comando de rotación diferencial sobre el eje vertical.
     * @param turn_val Intensidad de giro con signo (+: derecha, -: izquierda).
     * @param turn_ms Duración del pulso de giro en milisegundos.
     */
    void sendGoToTurn(int16_t turn_val, uint16_t turn_ms = 0);

    /**
     * @ingroup group_navigation
     * @brief Transmite un nuevo setpoint angular de inclinación para avance o retroceso.
     * @param sp_val Valor del setpoint escalado en centésimas de grado.
     */
    void sendGoToSetpoint(int32_t sp_val);

    /**
     * @brief Actualiza los displays numéricos HUD del modo Joystick.
     */
    void updateGoToAngleDisplays();

    // Exportación de telemetría a ficheros
    void on_pushButton_exportExcel_clicked();
    void on_pushButton_exportIrCsv_clicked();
    void on_pushButton_exportTxt_clicked();

    // Comandos de seguridad y emergencia
    void on_btn_car_stop_clicked();

    // Gestión de referencia de la brújula
    void on_btn_set_compass_ref_clicked();
    void on_btn_reset_compass_ref_clicked();

private:
    /**
     * @ingroup group_telemetry
     * @struct LogEntry
     * @brief Registro estructurado de eventos de comunicación para auditoría y diagnóstico.
     */
    struct LogEntry {
        QDateTime time;  ///< Estampa de tiempo exacta del mensaje
        QString data;     ///< Contenido textual o volcado hexadecimal del frame
        QString type;     ///< Tipo de evento ("RX", "TX", "UNKNOWN", "CHK_ERROR")
    };
    QList<LogEntry> m_logHistory; ///< Historial en memoria de las transacciones registradas

    static const int IR_BUFFER_SIZE = 3000; ///< Capacidad máxima del buffer circular (~1 min a 50 Hz)

    /**
     * @ingroup group_telemetry
     * @struct IrSample
     * @brief Muestra de los 3 sensores ópticos inferiores calibrados para el cálculo del seguidor.
     */
    struct IrSample {
        QDateTime timestamp; ///< Timestamp de adquisición
        uint16_t ir1;        ///< Sensor derecho calibrado
        uint16_t ir3;        ///< Sensor central calibrado
        uint16_t ir5;        ///< Sensor izquierdo calibrado
    };
    QVector<IrSample> m_irBuffer; ///< Buffer circular de reflectancia infrarroja inferior

    /**
     * @ingroup group_telemetry
     * @struct UpperIrSample
     * @brief Muestra cruda de los 5 sensores infrarrojos de la barra superior de evasión.
     */
    struct UpperIrSample {
        QDateTime timestamp; ///< Timestamp de adquisición
        uint16_t ir1;        ///< Sensor superior 1 (adcDataTx[0])
        uint16_t ir3;        ///< Sensor superior 2 (adcDataTx[2])
        uint16_t ir5;        ///< Sensor superior 3 (adcDataTx[4])
        uint16_t ir7;        ///< Sensor superior 4 (adcDataTx[6])
        uint16_t ir8;        ///< Sensor superior 5 (adcDataTx[7])
    };
    QVector<UpperIrSample> m_upperIrBuffer; ///< Buffer de telemetría del array superior

    int m_irExportCount = 0; ///< Conteo de exportaciones serializado

    void exportIrCsvToFile();
    double adcToDistanceCm(uint16_t adc);

    int m_countSent = 0;           ///< Contador de paquetes enviados satisfactoriamente
    int m_countReceived = 0;       ///< Contador de paquetes válidos recibidos
    int m_countUnknown = 0;        ///< Contador de tramas con comando desconocido
    int m_countChecksumErrors = 0; ///< Contador de tramas descartadas por error de paridad/XOR

    void addLogEntry(const QString &data, const QString &type);
    void cleanOldLogs();

    Ui::MainWindow *ui;            ///< Puntero a los elementos de interfaz generados por uic
    QSerialPort *QSerialPort1;     ///< Controlador del puerto serie físico / virtual
    QLabel *statusMode;            ///< Etiqueta de la barra de estado inferior
    qint64 m_lastRxTime = 0;       ///< Timestamp en ms del último paquete de datos recibido

    QDialog *myDebugDialog;        ///< Puntero al diálogo secundario de depuración
    QDialog *myHelpDialog;         ///< Puntero al diálogo modal de ayuda
    QDialog *myPasswordDialog;     ///< Diálogo de autenticación para configuración
    QDialog *myJoystickExitDialog; ///< Diálogo de confirmación de salida de modo Joystick

    QTimer *timer1;                ///< Timer de watchdog para timeout de recepción
    QTimer *timer2;                ///< Timer periódico de sondeo de telemetría (getData)

    // Gráficos y series temporales (QChart / QLineSeries)
    QChart *chartOrientation_mw;   ///< Gráfica de orientación angular 3D (Pitch, Roll, Yaw)
    QChart *chartBalance_mw;       ///< Gráfica del lazo PID de balance e inclinación
    QChart *chartLine_mw;          ///< Gráfica del lazo PID de seguimiento de línea
    QChart *chartPID_mw;           ///< Gráfica combinada de parámetros de control

    QValueAxis *orient_axisX;      ///< Eje X temporal para orientación
    QValueAxis *orient_axisY;      ///< Eje Y angular en grados [-180, +180]
    QValueAxis *bal_axisX;         ///< Eje X temporal para balance
    QValueAxis *bal_axisY;         ///< Eje Y de balance
    QValueAxis *line_axisX;        ///< Eje X temporal de seguidor de línea
    QValueAxis *line_axisY;        ///< Eje Y de error de línea

    QLineSeries *pid_pSeries;      ///< Serie de término proporcional de balance
    QLineSeries *pid_iSeries;      ///< Serie de término integral de balance
    QLineSeries *pid_dSeries;      ///< Serie de término derivativo de balance
    QLineSeries *pid_outSeries;    ///< Serie de salida PWM final de balance
    QLineSeries *pid_pLineSeries;  ///< Serie de término proporcional de línea
    QLineSeries *pid_dLineSeries;  ///< Serie de término derivativo de línea
    QLineSeries *pid_outLineSeries;///< Serie de corrección diferencial de línea

    // Series de acelerómetro y giróscopo
    QLineSeries *pid_axSeries;
    QLineSeries *pid_aySeries;
    QLineSeries *pid_azSeries;
    QLineSeries *pid_gxSeries;
    QLineSeries *pid_gySeries;
    QLineSeries *pid_gzSeries;

    // Series de ángulos de Euler
    QLineSeries *pid_pitchSeries;
    QLineSeries *pid_rollSeries;
    QLineSeries *pid_yawSeries;

    // Series de los 8 canales reflectivos IR
    QLineSeries *pid_ir1Series;
    QLineSeries *pid_ir2Series;
    QLineSeries *pid_ir3Series;
    QLineSeries *pid_ir4Series;
    QLineSeries *pid_ir5Series;
    QLineSeries *pid_ir6Series;
    QLineSeries *pid_ir7Series;
    QLineSeries *pid_ir8Series;
    QValueAxis *pid_axisX;
    QValueAxis *pid_axisY;

    uint16_t m_calIr1 = 0; ///< Valor IR1 calibrado
    uint16_t m_calIr3 = 0; ///< Valor IR3 calibrado
    uint16_t m_calIr5 = 0; ///< Valor IR5 calibrado

    // Variables de control de navegación GoTo
    int32_t m_currentSetpoint = -1000;    ///< Setpoint de ángulo de balance activo
    float m_currentAngle = 0.0f;          ///< Ángulo de cabeceo instantáneo medido
    float m_gotoStartYaw = 0.0f;          ///< Ángulo de guiñada tomado al ingresar a GoTo
    float m_gotoRelativeYaw = 0.0f;       ///< Giro relativo acumulado en la sesión
    int m_gotoStep = 25;                  ///< Incremento por pulsación en centésimas de grado
    int16_t m_gotoTurnIntensity = 250;    ///< Intensidad de torque de giro diferencial PWM
    uint16_t m_gotoTurnDuration = 200;    ///< Duración de activación de giro en milisegundos
    bool m_isRotatingLeft = false;        ///< Flag de giro continuo a izquierda
    bool m_isRotatingRight = false;       ///< Flag de giro continuo a derecha
    QTimer *m_gotoTurnKeepAliveTimer = nullptr; ///< Timer para mantener giros prolongados

    void initPIDChart();
    void updatePIDChart(double time, double p, double i, double d, double out, double pLine, double dLine, double outLine);
    void updateMPUChart(double time, double ax, double ay, double az, double gx, double gy, double gz, double pitch, double roll, double yaw);
    void updateIRChart(double time, double ir1, double ir2, double ir3, double ir4, double ir5, double ir6, double ir7, double ir8);
    void updatePIDChartRange();
    void resetInterface();
    void initHelpContent();

    // Subsistema de comunicaciones UDP
    QUdpSocket *QUdpSocket1;       ///< Socket UDP para datagramas de control y telemetría
    QHostAddress RemoteAddress;    ///< Dirección IP remota del robot (ej. 192.168.4.1)
    quint16 RemotePort;            ///< Puerto de enlace UDP en el robot
    QHostAddress clientAddress;    ///< Dirección IP del cliente detectado
    int puertoremoto;              ///< Número de puerto remoto

    // Subsistema de comunicaciones TCP
    QTcpServer *QTcpServer1;       ///< Servidor TCP para aceptar conexión del módulo Wi-Fi
    QTcpSocket *QTcpSocketClient = nullptr; ///< Socket cliente TCP conectado
    void sendTcp(uint8_t *tx, uint8_t length);
    bool m_isSoftApMode = false;   ///< Flag indicador de red SoftAP creada por el robot
    bool isSoftApActive() const;
    bool m_isTcpMode = false;      ///< Flag de selección de transporte TCP
    void updateProtocolUI();
    void update3DModelConnectionState(bool connected);
    bool isRobotConnected() const;
    bool m_is3DModelConnected = false;
    qint64 m_lastMpuTime = 0;      ///< Timestamp del último paquete inercial recibido

    // Banderas de estado operativo
    bool firExe;
    bool servoDir;
    bool firRadarExe;
    int contadorAlive = 0;
    int angle;
    bool paramsSynced = false;

    uint32_t lftEncData; ///< Lectura de pulsos del encoder izquierdo
    uint32_t rhtEncData; ///< Lectura de pulsos del encoder derecho

    QElapsedTimer runtimeTimer; ///< Cronómetro de alta resolución para ejes temporales de gráficas

    /**
     * @ingroup group_comm_host
     * @enum _eProtocolo
     * @brief Estados de la máquina de decodificación de la cabecera del protocolo binario UNER.
     *
     * \startuml
     * title Máquina de Estados del Decodificador de Trama UNER (_eProtocolo)
     * [*] --> START
     * START --> HEADER_1 : Byte == 'U'
     * HEADER_1 --> HEADER_2 : Byte == 'N'
     * HEADER_1 --> START : Byte != 'N'
     * HEADER_2 --> HEADER_3 : Byte == 'E'
     * HEADER_2 --> START : Byte != 'E'
     * HEADER_3 --> NBYTES : Byte == 'R'
     * HEADER_3 --> START : Byte != 'R'
     * NBYTES --> TOKEN : Longitud N bytes leída
     * TOKEN --> ID : Byte == ':'
     * TOKEN --> START : Byte != ':'
     * ID --> PAYLOAD : ID de comando válido
     * PAYLOAD --> PAYLOAD : Recibir payload y calcular Checksum XOR
     * PAYLOAD --> START : Trama completa -> Despachar decodeData()
     * \enduml
     */
    typedef enum{
        START,    ///< Espera del primer byte de cabecera ('U')
        HEADER_1, ///< Espera del segundo byte de cabecera ('N')
        HEADER_2, ///< Espera del tercer byte de cabecera ('E')
        HEADER_3, ///< Espera del cuarto byte de cabecera ('R')
        NBYTES,   ///< Lectura de la cantidad de bytes que componen la trama
        TOKEN,    ///< Verificación del delimitador ':'
        ID,       ///< Extracción del código de comando (_eCmd)
        PAYLOAD,  ///< Acumulación de bytes de datos y verificación de Checksum XOR
    }_eProtocolo;

    _eProtocolo estadoProtocolo;    ///< Estado del parser para la conexión Serie
    _eProtocolo estadoProtocoloUdp; ///< Estado del parser para la conexión UDP
    _eProtocolo estadoProtocoloTcp; ///< Estado del parser para la conexión TCP

    /**
     * @ingroup group_comm_host
     * @enum _eCmd
     * @brief Opcodes de comandos intercambiados entre la Estación Terrena y el Firmware del Robot.
     */
    typedef enum{
        UDP = 0,               ///< Identificador de transporte UDP
        SERIE = 1,             ///< Identificador de transporte Serie
        ACK = 0x0D,            ///< Byte de confirmación / Enter

        GETALIVE = 0xA0,        ///< Consulta periódica de estado de vida (Heartbeat)
        GETFIRMWARE = 0xA1,     ///< Consulta de versión de firmware compilada
        GETMPU = 0xA2,          ///< Telemetría inercial de la IMU MPU-6050
        GETADC = 0xA3,          ///< Lecturas analógicas de reflectancia infrarroja

        SETPWML = 0xA4,         ///< Configuración de potencia PWM para motor izquierdo
        SETPWMR = 0xA5,         ///< Configuración de potencia PWM para motor derecho

        SETPWMMINR = 0xA6,      ///< Zona muerta / PWM mínimo motor derecho
        SETPWMMINL = 0xA7,      ///< Zona muerta / PWM mínimo motor izquierdo

        SETBALANCEKP = 0XA8,    ///< Ganancia Proporcional Kp del lazo de balance
        SETBALANCEKD = 0xA9,    ///< Ganancia Derivativa Kd del lazo de balance
        SETBALANCEKI = 0xAA,    ///< Ganancia Integral Ki del lazo de balance

        SETSETPOINT = 0xAB,     ///< Ángulo setpoint de equilibrio deseado

        SETLINEKP = 0xAC,       ///< Ganancia Proporcional Kp del seguidor de línea
        SETLINEKD = 0xAD,       ///< Ganancia Derivativa Kd del seguidor de línea

        GETINTERNALDATA = 0xF0, ///< Consulta de métricas algebraicas internas de línea
        GETPIDBALANCE = 0xF1,   ///< Consulta de términos del lazo PID de balance

        SETOFFSETL = 0xAE,      ///< Offset de trim motor izquierdo
        SETOFFSETR = 0xAF,      ///< Offset de trim motor derecho

        SETCUSTOMTURN = 0xB0,   ///< Ganancia de rotación en curvas del seguidor
        SET_KP_EXT = 0xC3,      ///< Ganancia proporcional de lazo cascada externo
        SETSPEED = 0xB1,        ///< Ángulo de ataque para velocidad de avance
        SETBKANG = 0xB2,        ///< Ángulo de frenado contra inercia

        SETFRONTDIST = 0xB3,    ///< Distancia frontal de detección de obstáculo
        SETSIDEDIST = 0xB4,     ///< Distancia lateral de seguimiento de pared
        SETLOSTDIST = 0xB5,     ///< Distancia límite de pérdida de pared
        SETSTOPCYCLES = 0xB6,   ///< Ciclos de frenado estático
        SETCORNERDIST = 0xB7,   ///< Distancia de anticipación en esquinas
        SETALIGNDIST = 0xB8,    ///< Distancia de alineación con pared

        SETPWMLROT = 0xB9,      ///< Potencia de motor izquierdo durante rotación
        SETPWMRROT = 0xC0,      ///< Potencia de motor derecho durante rotación

        SETSTATICOFF = 0xC1,    ///< Offset estático de compensación
        SETMOVINGOFF = 0xC2,    ///< Offset dinámico en movimiento
        SETLIMITANG = 0xC4,     ///< Límite angular de seguridad para desconexión
        SET_KI_EXT = 0xC5,      ///< Ganancia integral de lazo cascada externo
        SET_ALFA_LPF = 0xC6,    ///< Coeficiente alfa del filtro pasa-bajos
        SETVELDAMPDIV = 0xC7,   ///< Divisor de amortiguamiento de velocidad
        SETVELDAMPLIM = 0xC8,   ///< Límite de amortiguamiento de velocidad
        SETTURNLIMIT = 0xC9,    ///< Límite de giro diferencial
        SETWALLKP = 0xCB,       ///< Ganancia Kp para seguimiento de pared
        SETWALLKD = 0xCC,       ///< Ganancia Kd para seguimiento de pared
        SETFRONTKP = 0xCD,      ///< Ganancia Kp de aproximación frontal
        SETFRONTKD = 0xCE,      ///< Ganancia Kd de aproximación frontal
        SETDODGEMODE = 0xCF,    ///< Dirección de giro de evasión (izq/der)
        SETSOFTAP = 0xD1,       ///< Conmutación entre modo Wi-Fi Station y SoftAP
        SETROBOTMODE = 0xD2,    ///< Selección de modo operativo del robot
        SETGOTOTURN = 0xD3,     ///< Comando de giro en modo GoTo / Joystick
        SETJOYSTICKTURN = 0xD3, ///< Alias para comando de giro diferencial
        SETDODGEBIASTIME = 0xD4,///< Tiempo de polarización de giro
        SETDODGEBIASMULT = 0xD5,///< Multiplicador de polarización de evasión
        EXPORTIRCSV = 0xCA,     ///< Comando para volcado de buffer IR en CSV

        UNKNOWCMD = 0xFF,       ///< Comando desconocido o error de trama
        OTHERS
    }_eCmd;

    /**
     * @ingroup group_comm_host
     * @struct _sDatos
     * @brief Estructura de almacenamiento temporal durante la decodificación de tramas.
     */
    typedef struct{
        uint8_t timeOut;      ///< Contador regresivo de tiempo límite
        uint8_t cheksum;      ///< Checksum acumulado mediante operación XOR
        uint8_t payLoad[256]; ///< Búfer contenedor de los datos del paquete
        uint8_t nBytes;       ///< Cantidad total de bytes especificados en la cabecera
        uint8_t index;        ///< Índice del byte actual que se está recibiendo
    }_sDatos;

    _sDatos rxData;    ///< Búfer de decodificación para el puerto Serie
    _sDatos rxDataUdp; ///< Búfer de decodificación para el socket UDP
    _sDatos rxDataTcp; ///< Búfer de decodificación para el socket TCP

    /**
     * @ingroup group_comm_host
     * @union _udat
     * @brief Unión de conversión binaria para tipado fuerte y serialización rápida en memoria.
     */
    typedef union {
        double d32;             ///< Punto flotante doble precisión (64 bits)
        float f32;              ///< Punto flotante simple precisión (32 bits IEEE 754)
        int i32;                ///< Entero de 32 bits con signo
        unsigned int ui32;      ///< Entero de 32 bits sin signo
        unsigned short ui16[2]; ///< Dos enteros de 16 bits sin signo
        short i16[2];           ///< Dos enteros de 16 bits con signo
        uint8_t ui8[4];         ///< Cuatro bytes sin signo (acceso byte a byte)
        char chr[4];            ///< Cuatro caracteres ASCII
        unsigned char uchr[4];  ///< Cuatro bytes unsigned char
        int8_t i8[4];           ///< Cuatro bytes con signo
    }_udat;

    _udat myWord; ///< Instancia de conversión para serialización/deserialización

    QQuickWidget *view3D;         ///< Contenedor para la escena QML 3D del robot
    float yawAcumulado = 0.0f;    ///< Ángulo integrado continuo de rotación sobre eje Z
    bool m_isCommBlocked = false; ///< Flag para bloquear peticiones automáticas durante pruebas

    CompassWidget *m_compassWidget = nullptr; ///< Puntero al widget de instrumentación brújula

};
#endif // MAINWINDOW_H
