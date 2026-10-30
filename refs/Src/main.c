/**
 * @file   main.c
 * @author Gonzalo M. Buffa
 * @brief  Punto de entrada principal, planificador temporal cooperativo y lazo de control del péndulo invertido.
 * @details Este archivo implementa el núcleo en tiempo real del robot autobalanceado sobre STM32F103C8T6 (72 MHz).
 *          Contiene el lazo crítico de control en cascada ejecutado a 200 Hz (5 ms) mediante TIM2,
 *          la máquina de estados finita principal (MEF) con 8 modos operativos, el seguidor de línea autónomo
 *          con calibración por LUTs, el algoritmo de evasión de obstáculos con control PD de pared,
 *          la teleoperación por Joystick remoto, el despachador de tramas binarias unerPrtcl y el planificador
 *          cooperativo de tareas periódicas (10 ms, 20 ms, 100 ms y 1 s).
 *
 * @defgroup group_main Núcleo y Planificador del Sistema (Core & Scheduler)
 * @brief Inicialización, bucle principal cooperativo y gestión de la MEF general.
 *
 * @defgroup group_control Lazos de Control, PID y Cinemática (Control & PID)
 * @brief Algoritmo PID de balance (200 Hz), compensador PI externo de esfuerzo motor y control de guiñada.
 *
 * @defgroup group_sensors Adquisición Inercial y Sensores Ópticos (Sensors & IMU)
 * @brief Drivers y filtros para sensor inercial MPU6050, barra infrarroja de línea y sensores de distancia.
 *
 * @defgroup group_comm Comunicaciones y Protocolos (Networking & Protocol)
 * @brief Control por comandos AT para ESP-01 Wi-Fi, sockets UDP/TCP, servidor Web y protocolo unerPrtcl.
 *
 * @defgroup group_ui_graphics Interfaz OLED, Gráficos 3D y Botón (Display & UI)
 * @brief Primitivas para display SSD1306, motor 3D/4D wireframe, pulsador multifunción y LED Heartbeat.
 *
 * @defgroup group_system Abstracción de Hardware y Sistema STM32 (System & HAL)
 * @brief Manejadores de interrupciones NVIC, configuración de periféricos HAL y soporte de bajo nivel.
 *
 * @ingroup group_main
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#include "util.h"
#include "img.h"
#include "fonts.h"
#include "wiregfx.h"

#include "ssd1306.h"
#include "mpu6050.h"
#include "esp01.h"

#include <stdio.h>
#include <unerPrtcl.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Las estructuras y enumeraciones del sistema han sido centralizadas en util.h */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// =========================================================
// // SISTEMA Y TEMPORIZACIÓN
// =========================================================
#define TO10MS                  40       // Umbral para tarea de 10ms (40 x 250us)
#define T100MS                  100      // Equivalente a 100ms
#define T1000MS                 1000     // Equivalente a 1000ms
#define T2000MS                 2000     // Equivalente a 2000ms
#define T3000MS                 3000     // Equivalente a 3000ms
#define ON                      1        // Banderas de estado encendido
#define OFF                     0        // Banderas de estado apagado

// =========================================================
// // BANDERAS DEL SISTEMA (FLAGS)
// =========================================================
#define ALLFLAGS                myFlags.bytes       // Acceso a bytes completos de las banderas
#define IS10MS                  myFlags.bits.bit0   // Bandera de ciclo de 10ms activo
#define IS20MS                  myFlags.bits.bit1   // Bandera de ciclo de 20ms activo
#define IS100MS                 myFlags.bits.bit2   // Bandera de ciclo de 100ms activo
#define RUN_PID                 myFlags.bits.bit4   // Bandera para ejecutar PID balancín

// =========================================================
// // BOTONES E INTERFAZ DE USUARIO
// =========================================================
#define DEBOUNCE                4        // Ciclos de antirrebote para botones
#define T400MS                  400      // Ventana de espera para multiclic de botones (400ms)

// =========================================================
// // COMUNICACIÓN WIFI Y SOFTAP TCP
// =========================================================
#define SOFTAP_BUF_SIZE         48       // Tamaño de buffer para recepción de credenciales desde Hercules (SSID;PASS o SSID;PASS;IP)
#define WIFI_CRED_BUF_SIZE      9        // Longitud máxima de buffer de credencial (8 caracteres + '\0')
#define SOFTAP_TCP_PORT         80       // Puerto de escucha TCP del servidor SoftAP para configuración
#define NUM_KNOWN_NETWORKS      (sizeof(knownNetworks) / sizeof(knownNetworks[0])) // Cantidad de redes registradas
#define SCANTIME                3000     // Tiempo en ms para escaneo de redes conocidas

// =========================================================
// // MOTORES Y TRACCIÓN PWM
// =========================================================
#define TIM3CP                  9999     // Período máximo del PWM de los motores (Timer 3)

// =========================================================
// // BALANCEO Y CONTROL PID
// =========================================================
#define PID_SCALE_FACTOR        100      // Factor de escala (x100) para evitar uso de floats en el PID
#define ANG45                   (45 * PID_SCALE_FACTOR)        // Ángulo de caída extrema (45.00°)
#define ANG20                   (20 * PID_SCALE_FACTOR)        // Ángulo de límite de integración PID (20.00°)
#define ANG18                   (18 * PID_SCALE_FACTOR)        // Ángulo de inclinación crítica delantera (18.00°)
#define ANG15                   (15 * PID_SCALE_FACTOR)        // Ángulo de umbral dinámico trasero en curva (15.00°)
#define ANG12                   (12 * PID_SCALE_FACTOR)        // Ángulo de setpoint de ataque en curva (12.00°)
#define ANG10                   (10 * PID_SCALE_FACTOR)        // Ángulo de setpoint de ataque estándar (10.00°)
#define ANG7_5                  (75 * PID_SCALE_FACTOR / 10)   // Ángulo de recuperación amortiguado (7.50°)
#define ANG2                    (2 * PID_SCALE_FACTOR)         // Ángulo de caída delantera (2.00°)

// =========================================================
// // MODO JOYSTICK (TELEOPERACIÓN)
// =========================================================
#define goto_turn_offset        joystick_turn_offset           // Alias de compatibilidad para offset de giro
#define goto_turn_timer         joystick_turn_timer            // Alias de compatibilidad para temporizador de giro
#define goto_start_yaw_hr       joystick_start_yaw_hr          // Alias de compatibilidad para ángulo Yaw inicial

// =========================================================
// // MODO SEGUIDOR DE LÍNEA
// =========================================================
#define SCALE_LINE              1000     // Factor de escala para el término cuadrático de error de línea
#define IR_WHITE                200      // Umbral analógico para considerar superficie blanca
#define LINE_LOST_PHASE0        35       // Duración de la primera fase de búsqueda en ciclos
#define LINE_LOST_PHASE1        70       // Duración de la segunda fase de búsqueda en ciclos

// Estados para la sub-MEF de pérdida de línea
#define LINE_LOST_ROT_90        0
#define LINE_LOST_WAIT_2S       1
#define LINE_LOST_ROT_180       2
#define LINE_LOST_STOPPED       3
#define LINE_LOST_TURN_SPEED    180      // Esfuerzo de giro reducido para rotación suave en pérdida de línea (90° / 180°)

// =========================================================
// // MODO ESQUIVAR OBSTÁCULOS (DODGE)
// =========================================================
#define IR_DODGE_LINE_THRESHOLD 200      // Umbral analógico para considerar cinta negra en modo esquivar
#define IR6_BOX_THRESHOLD       2000     // Umbral analógico para detección de caja (IR6)

// =========================================================
// // SENSORES IMU (MPU6050) E I2C
// =========================================================
#define MPU6050                 1        // Identificador de tarea del giroscopio en la Pila I2C
#define DT_MS                   5        // Delta de tiempo nominal en ms para integración de giroscopio (ajustado a 5ms)
#define DT_US                   5000     // Delta de tiempo nominal en us para el lazo PID (ajustado a 5ms)
#define ALPHA_GYRO              980      // Confianza en escala x1000 del filtro complementario en el giroscopio (98.0%)
#define ALPHA_ACC               20       // Confianza en escala x1000 del filtro complementario en el acelerómetro (2.0%)
#define AZ_MIN_VALID            4000     // Mínimo valor absoluto del acelerómetro Z para validar el ángulo
#ifndef MPU6050_ADDR
#define MPU6050_ADDR            (0x68 << 1) // Dirección I2C del giroscopio MPU6050
#endif
#define I2CSIZE                 16       // Tamaño del buffer de tareas I2C (Pila)

// =========================================================
// // SENSORES IR Y CALIBRACIÓN (LUT)
// =========================================================
#define LUT_SIZE                16       // Tamaño de las Look-Up Tables de calibración de sensores
#define lut_l1                  LUT_IR1_IZQ           // Alias de tabla de calibración izquierda
#define lut_l2                  LUT_IR3_CEN           // Alias de tabla de calibración central
#define lut_l3                  LUT_IR5_DER           // Alias de tabla de calibración derecha
#define lut_l4                  LUT_PROMEDIO          // Alias de tabla de calibración promedio
#define lut_y                   LUT_Y_SCALE           // Alias de escala de salida (0 a 1000)

#define lut_l1_x                LUT_IR1_IZQ         // Mapeo X de tabla de calibración izquierda
#define lut_l1_y                LUT_Y_SCALE         // Mapeo Y de tabla de calibración izquierda
#define lut_l2_x                LUT_IR3_CEN         // Mapeo X de tabla de calibración central
#define lut_l2_y                LUT_Y_SCALE         // Mapeo Y de tabla de calibración central
#define lut_l3_x                LUT_IR5_DER         // Mapeo X de tabla de calibración derecha
#define lut_l3_y                LUT_Y_SCALE         // Mapeo Y de tabla de calibración derecha
#define lut_l4_x                LUT_PROMEDIO        // Mapeo X de tabla de calibración promedio
#define lut_l4_y                LUT_Y_SCALE         // Mapeo Y de tabla de calibración promedio

// =========================================================
// // PANTALLA OLED (SSD1306)
// =========================================================
#define SSD1306                 0        // Identificador de tarea de pantalla en la Pila I2C
#define SSD1306_MAXADC          30       // Límite superior de ADC para gráficos en pantalla
#define SSD1306_MINADC          60       // Límite inferior de ADC para gráficos en pantalla
#define SSD1306_SNDCOL          40       // Segunda columna del display
#define SSD1306_TRDCOL          85       // Tercera columna del display
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// =========================================================
// // SISTEMA Y ESTADO GENERAL
// =========================================================
const char firmware[] = "EX100923v01\n";             // Versión actual del firmware del microcontrolador
volatile _eRobotMode robotMode = STATE_SWING;         // Modo operativo actual del robot
volatile _eRobotMode lastMode = (_eRobotMode)-1;      // Modo operativo anterior para detección de cambios

// Máscara de 20 ranuras de 100ms síncronas para el LED Heartbeat (Ciclo de 2.0 segundos)
uint32_t heartBeatMask[] = {
    0x00000001,  // Indice 0: STATE_SWING (1 parpadeo de 100ms)
    0x00000005,  // Indice 1: STATE_LINE_FOLLOWING (2 parpadeos de 100ms)
    0x00000015,  // Indice 2: STATE_DODGE (3 parpadeos de 100ms)
    0x00000055,  // Indice 3: STATE_JOYSTICK (4 parpadeos de 100ms)
    0x00000155,  // Indice 4: STATE_3D_SCREEN (5 parpadeos de 100ms)
    0x0000001F,  // Indice 5: STATE_FIRST_SCREEN (encendido 500ms, apagado 1.5s)
    0x000003FF,  // Indice 6: STATE_SECOND_SCREEN (encendido 1000ms, apagado 1s)
    0x00000000   // Indice 7: STATE_STANDBY (apagado)
};
uint8_t hbIndex = 0;                                  // Índice para seleccionar la máscara del LED (Heartbeat)

// Temporizadores base y divisores del lazo principal
uint8_t time10ms;                                     // Temporizador incremental de 250us para llegar a 10ms
uint8_t tmo100ms = 10;                                // Temporizador de paciencia para eventos de 100ms
uint8_t tmo20ms = 2;                                  // Temporizador de paciencia para eventos de 20ms
uint8_t tmo100 = 20;                                  // Divisor de ciclo para intercalar tareas en I2C (TIM2 a 5ms)

// =========================================================
// // BANDERAS DEL SISTEMA (FLAGS)
// =========================================================
volatile _uFlag myFlags;                              // Banderas de ciclo de tareas del sistema en tiempo real

// =========================================================
// // BOTONES E INTERFAZ DE USUARIO
// =========================================================
_sButton myButton;                                    // Estructura de estado físico del botón de configuración

// =========================================================
// // COMUNICACIÓN SERIE Y USB
// =========================================================
_sComm USBTx;                                         // Estructura de protocolo para el buffer de transmisión USB
_sComm USBRx;                                         // Estructura de protocolo para el buffer de recepción USB
volatile uint8_t buffUSBTx[RXBUFSIZE];                // Array de memoria física para buffer de transmisión USB
volatile uint8_t buffUSBRx[TXBUFSIZE];                // Array de memoria física para buffer de recepción USB
_uWord myWord;                                        // Unión de propósito general para conversión de tipos de datos (2/4 bytes)

// =========================================================
// // COMUNICACIÓN WIFI Y SERVIDOR WEB
// =========================================================
_sComm WiFiTx;                                        // Estructura de protocolo para el buffer de transmisión WiFi (ESP-01)
_sComm WiFiRx;                                        // Estructura de protocolo para el buffer de recepción WiFi (ESP-01)
volatile uint8_t buffWiFiTx[RXBUFSIZE];               // Array de memoria física para buffer de transmisión WiFi
volatile uint8_t buffWiFiRx[TXBUFSIZE];               // Array de memoria física para buffer de recepción WiFi
uint8_t byteUART_ESP01;                               // Byte de almacenamiento de interrupción UART para WiFi
_sESP01Handle esp01Handler;                           // Estructura de llamadas y control del módulo Wi-Fi

// Base de datos local de redes Wi-Fi a las cuales autoconectarse
static const _sWiFiNetwork knownNetworks[] = {
	{ "POCOX8",                  "12345678",              "10.120.40.213"  },
	{ "ARPANET",                 "1969-Apolo_11-2022",    "192.168.0.10"   },
	{ "FCAL-Personal",           "fcal-uner+2019",        "172.22.237.227" },
	{ "FCAL",                    "fcalconcordia.06-2019", "172.23.190.89"  },
	{ "InternetPlus_872f10_EXT", "wlan78d0ef",            "192.168.1.52"   },
};
static uint8_t currentNetworkIdx = 0;                 // Red actual en intento de conexión por el escáner
static uint8_t networkScanActive = 0;                 // Bandera indicadora de escáner de redes activo
static uint16_t networkScanTimer = SCANTIME;          // Temporizador de permanencia en escaneo de red en milisegundos

// Telemetría y comunicación por sockets UDP/TCP
uint8_t timerUDP = 0;                                 // Temporizador para vigilar transmisión periódica UDP
uint8_t udpSilenceCounter = 0;                        // Contador de silencio UDP en segundos (manda ALIVEs en ausencia de comandos)
static char udpTargetIP[16] = "192.168.0.10";         // Dirección IP de destino UDP/TCP para envío de telemetría
static uint16_t udpTargetPort = 30010;                // Puerto de destino UDP/TCP de la aplicación de escritorio
static uint8_t udpReadyToStart = 0;                   // Bandera que indica que el socket UDP/TCP está listo para despachar
static char udpTargetProto[4] = "UDP";                // Protocolo inicial por defecto ("UDP" inicial, conmute a TCP según necesidad)

// Modo SoftAP servidor TCP para configuración desde terminal Hercules (SSID;PASS)
static char softAPBuf[SOFTAP_BUF_SIZE];               // Buffer para acumular comando de configuración
static uint8_t softAPBufIdx = 0;                      // Índice actual en el buffer SoftAP
static uint8_t softAPBufReady = 0;                    // Bandera de comando completo recibido
static uint8_t softAPRxTimer = 0;                     // Temporizador de inactividad para recepción SoftAP (10ms ticks)
static uint8_t isSoftAPMode = 0;                      // Estado de modo SoftAP activo (1 = activo, 0 = estación)
static char softAP_TargetSSID[WIFI_CRED_BUF_SIZE];    // Almacenamiento persistente del SSID recibido (8 chars max + '\0')
static char softAP_TargetPASS[WIFI_CRED_BUF_SIZE];    // Almacenamiento persistente de la contraseña recibida (8 chars max + '\0')
static uint16_t softAPSwitchDelay = 0;                // Demora en ticks de 10ms antes del reset para que salga el mensaje OK
static uint8_t softAPSwitchPending = 0;               // Bandera de cambio a Station pendiente
static uint16_t softAPEnterDelay = 0;                 // Demora en ticks de 10ms antes de pasar a SoftAP para que salga el ACK
static uint8_t softAPEnterPending = 0;                // Bandera de cambio a modo SoftAP pendiente por comando remoto

// =========================================================
// // MOTORES Y TRACCIÓN PWM
// =========================================================
uint16_t lPulse1 = 0;                                 // Ancho de pulso PWM para Motor Izquierdo Adelante
uint16_t rPulse2 = 0;                                 // Ancho de pulso PWM para Motor Derecho Adelante
uint16_t lPulse3 = 0;                                 // Ancho de pulso PWM para Motor Izquierdo Atrás
uint16_t rPulse4 = 0;                                 // Ancho de pulso PWM para Motor Derecho Atrás

uint16_t maxPWM = 9999;                               // Ciclo de trabajo máximo permitido (100%)
uint16_t minPWM_Left = 800;                           // PWM mínimo que vence la fricción estática de la rueda izquierda
uint16_t minPWM_Right = 1025;                         // PWM mínimo que vence la fricción estática de la rueda derecha
uint16_t PWM_LRot = 880;                              // PWM estático de pivote de giro en búsqueda para rueda izquierda
uint16_t PWM_RRot = 800;                              // PWM estático de pivote de giro en búsqueda para rueda derecha
int16_t offset_left = 0;                              // Offset para compensación de deriva de tracción del motor izquierdo
int16_t offset_right = 0;                             // Offset para compensación de deriva de tracción del motor derecho

// =========================================================
// // BALANCEO Y CONTROL PID
// =========================================================
// Estimación angular y filtrado complementario
int32_t acc_angle_hr = 0;                             // Ángulo del acelerómetro de alta resolución
int32_t gyro_delta_hr = 0;                            // Incremento angular de alta resolución calculado del giroscopio
int32_t current_angle_hr = 0;                         // Ángulo complementario filtrado de alta resolución
int32_t current_angle = 0;                            // Ángulo complementario de salida del robot en escala x100
int32_t measured_dt_ms = 20;                          // Delta de tiempo real medido en ms de ejecución del bucle
int32_t last_angle = 0;                               // Ángulo previo para cálculo de la derivada sobre medición

// Lazo PID de balance longitudinal (Lazo Rápido)
int32_t error = 0;                                    // Diferencia entre setpoint y ángulo actual
int32_t last_error = 0;                               // Error del ciclo PID inmediatamente anterior para cálculo derivativo
int32_t derivative = 0;                               // Componente derivativo del lazo PID
int32_t integral = 0;                                 // Componente acumulativo integral del lazo PID
int32_t output = 0;                                   // Acción de control total del lazo de equilibrio inyectada a los motores

// Ganancias del PID de balanceo
int16_t Kp_stable = 85;                               // Ganancia proporcional de equilibrio estático
int16_t Kd_stable = 2;                                // Ganancia derivativa de equilibrio estático
int16_t Ki_stable = 0;                                // Ganancia integral de equilibrio estático

// Consignas angulares y recuperación de caída
int32_t setpoint = -1000;                             // Setpoint de equilibrio estático base (x100) ajustable por Qt
int32_t balance_setpoint_calib = -1000;               // Setpoint estático calibrado base para recuperación de balance
int16_t attack_setpoint = -1600;                      // Setpoint de inclinación frontal para ataque (configurable por Qt)
volatile uint8_t backwards_recovery_active = 0;       // Estado del gatillo de recuperación de caída trasera
volatile uint8_t forwards_recovery_active = 0;        // Estado del gatillo de recuperación de caída delantera

// Lazo Cascada Externo (Control de posición y deriva de velocidad)
int32_t pwm_filtrado = 0;                             // Esfuerzo de motor filtrado LPF (Lazo Lento)
int32_t integral_esfuerzo = 0;                        // Integral de error de esfuerzo para lazo externo
int16_t angulo_modificador_pi = 0;                    // Corrección de ángulo calculado por PI (x100)
int16_t Kp_ext = 40;                                  // Ganancia proporcional de lazo externo (x1000)
int16_t Ki_ext = 200;                                 // Ganancia integral de lazo externo (x10000)
int16_t alfa_lpf = 10;                                // Coeficiente alfa del filtro LPF (0-100)

// =========================================================
// // MODO JOYSTICK (TELEOPERACIÓN)
// =========================================================
volatile int16_t joystick_turn_offset = 0;            // Offset de giro para modo Joystick
volatile uint16_t joystick_turn_timer = 0;            // Watchdog de giro para modo Joystick en ms
volatile int32_t joystick_start_yaw_hr = 0;           // Ángulo Yaw de referencia al iniciar modo Joystick

// =========================================================
// // MODO SEGUIDOR DE LÍNEA
// =========================================================
_eLineState lineState = LINE_SEARCHING;               // Estado actual de la máquina del seguidor de línea
int16_t Kp_line = 275;                                // Ganancia proporcional de guiñada para corrección rápida sobre la línea
int16_t Kq_line = 25;                                 // Ganancia derivativa/cuadrática de guiñada para atenuar oscilaciones
int16_t Kp_line_backup = 275;                         // Respaldo de Kp_line al entrar a Swing
int32_t sum_sensors = 0;                              // Suma de lecturas normalizadas de los sensores de línea activos
int32_t error_linea = 0;                              // Desviación calculada de la línea (eje horizontal de error)
int32_t abs_error = 0;                                // Valor absoluto del error de línea
int32_t linear_term = 0;                              // Aporte proporcional del control de dirección
int32_t quad_term = 0;                                // Aporte cuadrático/derivativo del control de dirección
int32_t turn_offset = 0;                              // Fuerza de rotación mezclada con el PID y enviada a los motores (Yaw)
int32_t last_line_error = 0;                          // Error de línea del ciclo anterior
int16_t custom_turn = 350;                            // Intensidad de giro prefijada para fases ciegas de búsqueda
int16_t vel_damp_div = 500;                           // Divisor del término amortiguador de velocidad
int16_t vel_damp_limit = 100;                         // Límite del amortiguador de velocidad
int16_t turn_limit = 3500;                           // Límite superior absoluto del esfuerzo de giro motor (Yaw)
uint16_t line_lost_timer = 0;                         // Temporizador en ciclos transcurridos desde que se perdió la pista
uint8_t line_lost_phase = 0;                          // Fase de búsqueda secuencial actual (fase 0, 1 o 2)
int32_t line_lost_yaw = 0;                            // Ángulo yaw acumulado durante búsqueda de línea
int8_t search_direction = 1;                          // Dirección de búsqueda (-1: izq, +1: der)
uint8_t dodge_line_rotation_done = 0;                 // Flag: rotación de reenganche ejecutada 1 sola vez por esquive

// =========================================================
// // MODO ESQUIVAR OBSTÁCULOS (DODGE)
// =========================================================
volatile _eDodgeSubState dodgeState = DODGE_LINE_FOLLOWING; // Inicia en seguimiento de línea con esquivado activo
volatile int32_t dodge_yaw = 0;                       // Referencia o delta de Yaw durante maniobra de esquive
volatile uint32_t dodge_timer = 0;                    // Temporizador en milisegundos para fases de maniobra
int8_t dodge_direction = -1;                          // -1: Rotación a la DERECHA, 1: IZQUIERDA
int16_t dodge_bias_time = 1000;                        // Duración del sesgo tras encontrar la línea (1000 ms = 1.0s)
int16_t dodge_bias_mult = 500;                         // Multiplicador / esfuerzo del sesgo de rotación
uint8_t dodge_bias_active = 0;                         // Flag indicador de sesgo activo al recuperar la línea
uint32_t dodge_bias_timer = 0;                         // Temporizador acumulado del sesgo (ms)

// Distancias de referencia y umbrales de proximidad
uint16_t obs_detect_dist = 1000;                      // Distancia frontal de detección en mm
uint16_t obs_corner_dist = 800;                       // Distancia lateral del sensor de 45° para validar esquina
uint16_t obs_lost_dist = 400;                         // Distancia mínima lateral por debajo de la cual la pared terminó
uint16_t obs_side_dist = 1000;                        // Distancia lateral de referencia deseada para seguir la pared
uint16_t obs_stop_cycles = 10;                        // Ciclos de inmovilización previa antes de iniciar rotación evasiva
uint16_t obs_align_dist = 2500;                       // Distancia objetivo del sensor lateral tras rotación de 90°

// Ganancias de control de evasión y seguimiento de pared
int32_t Kp_pared = 10;                                // Fuerza principal para mantener distancia lateral (90°)
int32_t Kd_anticipo = 7;                              // Fuerza menor de ayuda/anticipación anticipada (45°)
int32_t Kd_frontal = 30;                              // Fuerza de protección frontal anticipada (IR6)

// =========================================================
// // SENSORES IMU (MPU6050) E I2C
// =========================================================
uint8_t Pila[I2CSIZE] = {};                           // Cola de tareas I2C pendientes de despacho
uint8_t i2cIndex = 0;                                 // Índice circular actual para despacho en la cola I2C
volatile uint8_t mpu6050_RxCplt = 0;                  // Bandera indicadora de recepción I2C por DMA de datos del giroscopio

// Lecturas crudas del acelerómetro y giroscopio
int16_t ax = 0;                                       // Lectura cruda del acelerómetro en el eje X
int16_t ay = 0;                                       // Lectura cruda del acelerómetro en el eje Y
int16_t az = 0;                                       // Lectura cruda del acelerómetro en el eje Z
int16_t gx = 0;                                       // Lectura cruda del giroscopio en el eje X
int16_t gy = 0;                                       // Lectura cruda del giroscopio en el eje Y
int16_t gz = 0;                                       // Lectura cruda del giroscopio en el eje Z

// Aceleraciones filtradas y cinemática estimada
int32_t ax_filt = 0;                                  // Aceleración filtrada en el eje X
int32_t az_filt = 0;                                  // Aceleración filtrada en el eje Z
volatile int32_t speed = 0;                           // Estimación física de la velocidad lineal en mm/s
volatile int32_t dynamic_accel = 0;                   // Aceleración lineal filtrada y compensada

// Calibración, offsets e integración Yaw
int16_t ax_offset = 0;                                // Offset calibrado de gravedad en reposo del acelerómetro X
volatile int16_t gz_offset = 0;                       // Offset calibrado del giróscopo en el eje Z
volatile uint16_t calib_cycle = 0;                    // Contador de ciclos de calibración inicial del MPU (para telemetría)
volatile int32_t total_yaw_hr = 0;                    // Ángulo Yaw total acumulado continuo en milígrados
volatile int32_t compass_ref_yaw = 0;                 // Punto de referencia de Yaw para la Brújula (STATE_SECOND_SCREEN)

// =========================================================
// // SENSORES ADC E INFRARROJOS (CALIBRACIÓN / LUT)
// =========================================================
// Buffers de conversión ADC
uint16_t adcData[8];                                  // Buffer de DMA que almacena los valores crudos del ADC del micro
uint16_t adcDataTx[8];                                // Copia segura del buffer de ADC para transmisión libre de colisiones

// Tablas de calibración Look-Up Tables (LUT)
const uint16_t LUT_IR1_IZQ[LUT_SIZE] = {78, 167, 315, 384, 520, 722, 807, 903, 1055, 1327, 1492, 1629, 2213, 2442, 2919, 3682}; // LUT del sensor IR izquierdo
const uint16_t LUT_IR3_CEN[LUT_SIZE] = {149, 298, 459, 622, 868, 1261, 1501, 1720, 2246, 2874, 3261, 3499, 3847, 3875, 3885, 3905}; // LUT del sensor IR central
const uint16_t LUT_IR5_DER[LUT_SIZE] = {102, 237, 375, 518, 705, 1092, 1255, 1490, 1931, 2501, 2865, 3140, 3826, 3853, 3881, 3892}; // LUT del sensor IR derecho
const uint16_t LUT_PROMEDIO[LUT_SIZE] = {109, 234, 383, 508, 697, 1025, 1187, 1371, 1744, 2234, 2539, 2756, 3295, 3390, 3561, 3826}; // LUT de calibración promedio
const uint16_t LUT_Y_SCALE[16] = {0, 67, 133, 200, 267, 333, 400, 467, 533, 600, 667, 733, 800, 867, 933, 1000}; // Escala normalizada de salida (0 = Blanco, 1000 = Negro)

// Lecturas calibradas de sensores inferiores (seguimiento de línea)
volatile int16_t cal_left_ir = 0;                     // Lectura calibrada del sensor IR izquierdo (Der-Raw en telemetría)
volatile int16_t cal_center_ir = 0;                   // Lectura calibrada del sensor IR central (Cen-Raw en telemetría)
volatile int16_t cal_right_ir = 0;                    // Lectura calibrada del sensor IR derecho (Izq-Raw en telemetría)

// Lecturas calibradas de sensores superiores (detección de obstáculos / esquivar)
volatile int16_t cal_ir0 = 0;
volatile int16_t cal_ir2 = 0;
volatile int16_t cal_ir4 = 0;
volatile int16_t cal_ir6 = 0;
volatile int16_t cal_ir7 = 0;

// =========================================================
// // PANTALLA OLED (SSD1306)
// =========================================================
volatile uint8_t ssd1306_TxCplt = 0;                  // Bandera indicadora de fin de transmisión I2C por DMA para pantalla
uint16_t staticOff = 400;                             // Offset estático para dibujo de gráficos
uint16_t movingOff = 300;                             // Offset móvil para dibujo de gráficos interactivos
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/**
 * @name Tareas de Comunicación Serie y USB
 * @{
 */
/**
 * @brief Gestiona la transmisión y recepción periódica sobre el canal USB CDC.
 * @ingroup group_comm
 */
void USBTask(void);

/**
 * @brief Callback invocado al recibir un paquete de datos por USB CDC desde la PC.
 * @param[in] buf Puntero al buffer con los bytes recibidos.
 * @param[in] len Cantidad de bytes recibidos en el paquete.
 * @ingroup group_comm
 */
void USBRxData(uint8_t *buf, uint32_t len);

/**
 * @brief Decodificador central de comandos del protocolo unerPrtcl.
 * @details Examina el identificador de comando (`_eCmd`), extrae el payload con `_uWord`
 *          y genera la respuesta correspondiente con código `ACK` o datos de telemetría.
 * @param[in,out] dataRx Puntero al descriptor de recepción con la trama validada.
 * @param[in,out] dataTx Puntero al descriptor de transmisión para armar la respuesta.
 * @ingroup group_comm
 */
void decodeCommand(_sComm *dataRx, _sComm *dataTx);
/** @} */

/**
 * @name Planificador Temporal y Temporizadores Cooperativos
 * @{
 */
/**
 * @brief Tarea periódica de 10 ms ejecutada en el bucle principal.
 * @details Gestiona los timeouts del ESP-01, antirrebote del botón y genera los ticks
 *          derivados de 20 ms, 100 ms y 1 segundo.
 * @ingroup group_main
 */
void do10ms(void);

/**
 * @brief Genera el patrón visual de parpadeo sincrónico en el LED PC13 (Heartbeat).
 * @details Modula el LED según la máscara de 20 ranuras de 100 ms (`heartBeatMask[]`)
 *          para identificar a simple vista el modo activo de la MEF.
 * @ingroup group_ui_graphics
 */
void heartBeatTask(void);
/** @} */

/**
 * @name Manejadores de Pantalla OLED e Interfaz I2C
 * @{
 */
void ssd1306Data(void);
void displayMemWrite(uint8_t address, uint8_t *data, uint8_t size, uint8_t type);
void displayMemWriteDMA(uint8_t address, uint8_t *data, uint8_t size, uint8_t type);
void mpuMemWrite(uint8_t address, uint8_t *data, uint8_t size, uint8_t type);
void mpuMemReadDMA(uint8_t address, uint8_t *data, uint8_t size, uint8_t type);
void i2cTask(void);
/** @} */

/**
 * @name Máquina de Estados del Pulsador de Usuario
 * @{
 */
/**
 * @brief Inicializa los descriptores de estado y temporización del botón de usuario.
 * @param[in,out] button Puntero a la estructura `_sButton`.
 * @ingroup group_ui_graphics
 */
void initButton(_sButton *button);

/**
 * @brief Actualiza la MEF de bajo nivel de antirrebote (Debounce) del botón.
 * @param[in,out] button Puntero a la estructura `_sButton`.
 * @return Estado activo o inactivo del botón tras el filtrado.
 * @ingroup group_ui_graphics
 */
uint8_t updateMefTask(_sButton *button);

/**
 * @brief Interpreta los eventos de alto nivel del botón (multiclic de 1 a 5 clics y pulsación larga).
 * @details Conmuta la variable global `robotMode` (`_eRobotMode`) según el número de clics registrados.
 * @param[in,out] button Puntero a la estructura `_sButton` con el estado del botón.
 * @ingroup group_ui_graphics
 */
void buttonTask(_sButton *button);

/**
 * @brief Decrementa el temporizador de ventana multiclic del botón cada 10 ms.
 * @param[in,out] button Puntero a la estructura `_sButton`.
 * @ingroup group_ui_graphics
 */
void buttonTimeout10ms(_sButton *button);
/** @} */

/**
 * @name Modo SoftAP TCP y Manejo de Conectividad Wi-Fi
 * @{
 */
void softAPTask(void);
void OnESP01ChangeState(_eESP01STATUS state);
/** @} */

/**
 * @name Lazos de Control Dinámico, PID y Navegación
 * @{
 */
/**
 * @brief Tarea orquestadora central de control PID y MEF de modos operativos (200 Hz).
 * @ingroup group_control
 */
void PIDTask(void);

/**
 * @brief Gestiona el modo operativo de balanceo estático en el lugar (Swing y 3D Screen).
 * @param[out] target_setpoint Puntero a la consigna de inclinación longitudinal del robot.
 * @ingroup group_control
 */
void Control_Balanceo(int32_t *target_setpoint);

/**
 * @brief Máquina de estados finita y control del seguidor de línea autónomo sobre la pista.
 * @param[in] left_ir Lectura calibrada del sensor infrarrojo izquierdo.
 * @param[in] center_ir Lectura calibrada del sensor infrarrojo central.
 * @param[in] right_ir Lectura calibrada del sensor infrarrojo derecho.
 * @param[out] target_setpoint Puntero a la consigna de inclinación longitudinal modulada por la curva.
 * @ingroup group_control
 */
void ControlSeguimiento(int32_t left_ir, int32_t center_ir, int32_t right_ir, int32_t *target_setpoint);

/**
 * @brief Gestiona la máquina de estados finita (MEF) de evasión de obstáculos y seguimiento de pared.
 * @param[in] left_ir Lectura normalizada del sensor infrarrojo izquierdo.
 * @param[in] center_ir Lectura normalizada del sensor infrarrojo central.
 * @param[in] right_ir Lectura normalizada del sensor infrarrojo derecho.
 * @param[out] target_setpoint Puntero a la consigna de inclinación longitudinal del robot.
 * @ingroup group_control
 */
void Control_Esquivar(int32_t left_ir, int32_t center_ir, int32_t right_ir, int32_t *target_setpoint);

/**
 * @brief Gestiona el modo de control remoto interactivo vía comandos de Joystick.
 * @param[out] target_setpoint Puntero a la consigna de inclinación longitudinal del robot.
 * @ingroup group_control
 */
void Control_Joystick(int32_t *target_setpoint);

/**
 * @brief Ejecuta el lazo PID central de balance longitudinal, la mezcla de tracción y el accionamiento PWM.
 * @param[in] target_setpoint Consigna de ángulo deseada calculada por la tarea del modo activo.
 * @ingroup group_control
 */
void PID_Calcular(int32_t target_setpoint);

/**
 * @brief Integra numéricamente la aceleración longitudinal para estimar la velocidad lineal (mm/s).
 * @param[in] dt_us Intervalo de tiempo transcurrido en microsegundos.
 * @ingroup group_control
 */
void Speed_IntegrationTask(uint32_t dt_us);

static void WiFi_ScanTick(void);
static void UART_EnforceReceiverActive(void);
static void WiFi_HeartbeatTick(void);

/**
 * @brief Gestiona la transición limpia de interfaces visuales al cambiar el modo operativo.
 * @details Silencia temporalmente los motores para evitar tirones durante escrituras I2C síncronas.
 * @ingroup group_ui_graphics
 */
void HandleModeScreenTransition(void);

/**
 * @brief Inyecta los ciclos de trabajo PWM calculados sobre los canales del temporizador TIM3.
 * @ingroup group_control
 */
void PWM_Control(void);
/** @} */

/**
 * @name Calibración y Normalización de Sensores Infrarrojos
 * @{
 */
static uint16_t LUT_Interpolate(const uint16_t *x, const uint16_t *lut_y, uint16_t raw);
void NormalizeLineSensors(const uint16_t *adcDataTx_ptr, uint16_t *norm);
/** @} */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//Revisar tiempos de ejecución de esta sección con respecto a la mpu
	for (int i = 0; i < 8; i++) {
		adcDataTx[i] = adcData[i];
	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){ //Pantalla
	ssd1306_TxCplt = 1;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){ //MPU
	mpu6050_RxCplt = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) { //250us
		time10ms++;
		if (time10ms == TO10MS) {
			time10ms = 0;
			IS10MS = TRUE;
		}
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcData, 8);
	}

	if (htim->Instance == TIM2) { // 5ms (ajustado para muestreo rápido)
		// En modos de movimiento, encolamos el MPU6050 en cada ciclo de 5ms
		if (robotMode == STATE_SWING || robotMode == STATE_LINE_FOLLOWING || robotMode == STATE_DODGE || robotMode == STATE_3D_SCREEN || robotMode == STATE_JOYSTICK) {
			Pila[i2cIndex] = MPU6050;
			i2cIndex++;
			i2cIndex&=(I2CSIZE-1);
		}

		tmo100--;
		if(!tmo100){
			tmo100=20; // 20 * 5ms = 100ms para mantener el intervalo de actualización de la pantalla
			// En modos de pantalla, actualizamos secuencialmente
			if (robotMode == STATE_FIRST_SCREEN || robotMode == STATE_SECOND_SCREEN || robotMode == STATE_3D_SCREEN) {
				if (robotMode == STATE_FIRST_SCREEN || robotMode == STATE_SECOND_SCREEN) {
					Pila[i2cIndex] = MPU6050;
					i2cIndex++;
					i2cIndex&=(I2CSIZE-1);
				}

				Pila[i2cIndex] = SSD1306;
				i2cIndex++;
				i2cIndex&=(I2CSIZE-1);
			}
		}
	}
}


void USBRxData(uint8_t *buf, uint32_t len) { //Recibimos datos -> Enviamos datos

	for (uint8_t nBytesRx = 0; nBytesRx < len; nBytesRx++) { //Guardamos los datos en el buffer de recepcion
		USBRx.buff[USBRx.indexW++] = buf[nBytesRx];
		USBRx.indexW &= USBRx.mask;
	}

}

void COMMTask(_sComm *dataRx, _sComm *dataTx, uint8_t source) {

	if (dataRx->indexR != dataRx->indexW) {
		uint8_t sendBuffer[TXBUFSIZE];

		if (unerPrtcl_DecodeHeader(dataRx)) {

			// Si recibimos un comando válido por WiFi, la PC está conectada.
			// Resetear el contador de silencio para suprimir los ALIVEs autónomos.
			if (source == WIFI) {
				udpSilenceCounter = 0;
			}

			decodeCommand(dataRx, dataTx);

			for (uint8_t i = 0; i < dataTx->nBytes; i++) { //Paso limpio, error ultima posición
				sendBuffer[i] = dataTx->buff[dataTx->indexData++];
				dataTx->indexData &= dataTx->mask;
			}

			if(source)
				ESP01_Send(ESP01_GetLastConnID(), sendBuffer, 0, dataTx->nBytes, TXBUFSIZE);
			else
				CDC_Transmit_FS(sendBuffer, dataTx->nBytes);
		}
	}
}

void CHPD_Control(uint8_t state);

/**
 * @brief Conmuta el modo operativo global del robot péndulo invertido.
 * @details Gestiona las transiciones seguras entre modos operativos: silencia pulsos PWM para evitar
 *          sacudidas mecánicas, reinicia acumuladores integrales y memorias derivativas,
 *          configura el patrón del LED Heartbeat (`hbIndex`) y ajusta periféricos (Wi-Fi, OLED).
 *
 * \startuml
 * title Máquina de Estados Finita: Modos del Robot (SetRobotMode)
 * [*] --> STATE_STANDBY : Inicio en frío
 *
 * state STATE_STANDBY : PWM apagado (0%)\nSistema en espera segura
 * state STATE_SWING : Balanceo estático en el lugar\nDisplay OLED apagado
 * state STATE_LINE_FOLLOWING : Seguidor de línea por IR\nControl proporcional Kp_line
 * state STATE_DODGE : Evasión de obstáculos\nGiro 90° con gyro y seguimiento pared PD
 * state STATE_JOYSTICK : Control remoto interactivo UDP\nWatchdog de seguridad activo
 * state STATE_3D_SCREEN : Gemelo digital wireframe OLED\nBalanceo activo
 * state STATE_FIRST_SCREEN : Diagnóstico RAW de sensores\nMotores desenergizados
 * state STATE_SECOND_SCREEN : Brújula de orientación y estado dinámico\nMotores desenergizados
 *
 * STATE_STANDBY --> STATE_SWING : 1 Clic SW0 / Comando
 * STATE_STANDBY --> STATE_LINE_FOLLOWING : 2 Clics SW0 / Comando
 * STATE_STANDBY --> STATE_DODGE : 3 Clics SW0 / Comando
 * STATE_STANDBY --> STATE_JOYSTICK : 4 Clics SW0 / Comando
 * STATE_STANDBY --> STATE_3D_SCREEN : 5 Clics SW0 / Comando
 * STATE_STANDBY --> STATE_FIRST_SCREEN : Pulsación 1s / Comando
 * STATE_STANDBY --> STATE_SECOND_SCREEN : Pulsación 2s / Comando
 *
 * STATE_SWING --> STATE_STANDBY : Caída / Clic SW0
 * STATE_LINE_FOLLOWING --> STATE_STANDBY : Caída / Clic SW0
 * STATE_DODGE --> STATE_STANDBY : Caída / Clic SW0
 * STATE_JOYSTICK --> STATE_STANDBY : Watchdog timeout / Clic SW0
 * STATE_3D_SCREEN --> STATE_STANDBY : Caída / Clic SW0
 * STATE_FIRST_SCREEN --> STATE_STANDBY : Clic SW0
 * STATE_SECOND_SCREEN --> STATE_STANDBY : Clic SW0
 * \enduml
 *
 * @param[in] newMode Modo de destino al que transiciona el sistema (`_eRobotMode`).
 * @pre El planificador cooperativo debe estar en ejecución.
 * @post `robotMode` adopta `newMode`, PWM silenciado, integral reseteada a 0, `hbIndex` actualizado.
 * @see _eRobotMode
 * @see SetRobotModeRemote
 * @see PIDTask
 * @see buttonTask
 */
void SetRobotMode(_eRobotMode newMode) {
	if (robotMode == newMode) {
		return;
	}

	// Si salimos del modo Joystick, restauramos inmediatamente el setpoint estático
	// y cancelamos los offsets y temporizadores de giro del joystick
	if (robotMode == STATE_JOYSTICK && newMode != STATE_JOYSTICK) {
		setpoint = balance_setpoint_calib;
		turn_offset = 0;
		joystick_turn_offset = 0;
		joystick_turn_timer = 0;
	}

	// Silenciado seguro de PWM durante el cambio de modo para evitar impulsos residuales
	lPulse1 = 0;
	lPulse3 = 0;
	rPulse2 = 0;
	rPulse4 = 0;
	PWM_Control();

	integral = 0;
	last_error = 0;
	backwards_recovery_active = 0;
	forwards_recovery_active = 0;

	robotMode = newMode;
	lastMode = (_eRobotMode)-1; // Forzar actualización de pantalla/transición

	switch (newMode) {
		case STATE_SWING:
			hbIndex = 0; // LED Swing (1 parpadeo de 100ms)
			CHPD_Control(1);
			if (Kp_line > 0) {
				Kp_line_backup = Kp_line;
			}
			Kp_line = 0;
			setpoint = balance_setpoint_calib;
			turn_offset = 0;
			break;

		case STATE_LINE_FOLLOWING:
			hbIndex = 1; // LED Line Following (2 parpadeos)
			CHPD_Control(1);
			if (Kp_line == 0) {
				Kp_line = (Kp_line_backup > 0) ? Kp_line_backup : 275;
			}
			turn_offset = 0;
			break;

		case STATE_DODGE:
			dodgeState = DODGE_LINE_FOLLOWING; // Inicia en seguimiento de línea
			dodge_timer = 2000;
			dodge_bias_active = 0;
			dodge_bias_timer = 0;
			dodge_line_rotation_done = 0;
			hbIndex = 2; // LED Dodge (3 parpadeos)
			CHPD_Control(1);
			if (Kp_line == 0) {
				Kp_line = (Kp_line_backup > 0) ? Kp_line_backup : 275;
			}
			turn_offset = 0;
			break;

		case STATE_JOYSTICK:
			hbIndex = 3; // LED Joystick (4 parpadeos)
			CHPD_Control(1);
			if (Kp_line > 0) {
				Kp_line_backup = Kp_line;
			}
			Kp_line = 0;
			setpoint = balance_setpoint_calib;
			turn_offset = 0;
			joystick_turn_offset = 0;
			joystick_turn_timer = 0;
			joystick_start_yaw_hr = total_yaw_hr;
			break;

		case STATE_3D_SCREEN:
			hbIndex = 4; // LED Figuras 3D (5 parpadeos)
			CHPD_Control(0); // Sin WiFi en este modo
			if (Kp_line > 0) {
				Kp_line_backup = Kp_line;
			}
			Kp_line = 0;
			WIREGFX_ResetCycle();
			turn_offset = 0;
			break;

		case STATE_FIRST_SCREEN:
			hbIndex = 5; // LED RAW (500ms encendido)
			CHPD_Control(1);
			ssd1306_ResetDMAState();
			ssd1306_SetDisplayOn(1);
			turn_offset = 0;
			break;

		case STATE_SECOND_SCREEN:
			hbIndex = 6; // LED Premium (1000ms encendido)
			CHPD_Control(1);
			ssd1306_ResetDMAState();
			ssd1306_SetDisplayOn(1);
			turn_offset = 0;
			compass_ref_yaw = total_yaw_hr; // Guardar referencia de orientación al pasar a modo Brújula
			break;

		case STATE_STANDBY:
		default:
			hbIndex = 7; // LED Reposo (Apagado)
			CHPD_Control(1);
			turn_offset = 0;
			break;
	}
}

void SetRobotModeRemote(uint8_t modeId) {
	switch (modeId) {
		case 0: SetRobotMode(STATE_STANDBY); break;
		case 1: SetRobotMode(STATE_SWING); break;
		case 2: SetRobotMode(STATE_LINE_FOLLOWING); break;
		case 3: SetRobotMode(STATE_DODGE); break;
		case 4: SetRobotMode(STATE_JOYSTICK); break;
		case 5: SetRobotMode(STATE_3D_SCREEN); break;
		case 6: SetRobotMode(STATE_FIRST_SCREEN); break;
		case 7: SetRobotMode(STATE_SECOND_SCREEN); break;
		default: SetRobotMode(STATE_STANDBY); break;
	}
}

/**
 * @brief Despacha y procesa los comandos recibidos a través del protocolo binario UNER.
 * @details Examina el identificador de comando situado en `dataRx->buff[dataRx->indexData]`,
 *          ejecuta la acción solicitada (calibración, modificación de ganancias PID, cambio de modo,
 *          consulta de telemetría de sensores) y construye la trama binaria de respuesta en `dataTx`.
 * @param[in,out] dataRx Descriptor del canal de recepción con la trama validada.
 * @param[in,out] dataTx Descriptor del canal de transmisión donde se serializa la respuesta.
 * @pre La trama debe haber sido previamente validada con checksum exitoso por `unerPrtcl_DecodeHeader`.
 * @post Puede alterar variables de calibración, ganancias de control, o encolar paquetes en `dataTx`.
 * @see _eCmd
 * @see _sComm
 * @see unerPrtcl_DecodeHeader
 * @see unerPrtcl_PutHeaderOnTx
 * @ingroup group_comm
 */
void decodeCommand(_sComm *dataRx, _sComm *dataTx) {

	switch (dataRx->buff[dataRx->indexData]) {
	case ALIVE:
		unerPrtcl_PutHeaderOnTx(dataTx, ALIVE, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	case FIRMWARE:
		unerPrtcl_PutHeaderOnTx(dataTx, FIRMWARE, 13);
		unerPrtcl_PutStrOntx(dataTx, firmware);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	case GETMPU:
		unerPrtcl_PutHeaderOnTx(dataTx, GETMPU, 13);
		myWord.i16[0] = ax;
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.i16[0] = ay;
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.i16[0] = az;
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.i16[0] = gx;
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.i16[0] = gy;
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.i16[0] = gz;
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	case GETADC:
		unerPrtcl_PutHeaderOnTx(dataTx, GETADC, 17);
		myWord.ui16[0] = adcDataTx[0];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[1];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[2];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[3];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[4];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[5];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[6];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[7];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	case SETPWML:
		unerPrtcl_PutHeaderOnTx(dataTx, SETPWML, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		{
			int16_t val_L = (int16_t)myWord.ui16[0];
			if (val_L >= 0) {
				rPulse4 = val_L;
				lPulse3 = 0;
			} else {
				lPulse3 = -val_L;
				rPulse4 = 0;
			}
		}
		break;
	case SETPWMR:
		unerPrtcl_PutHeaderOnTx(dataTx, SETPWMR, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		{
			int16_t val_R = (int16_t)myWord.ui16[0];
			if (val_R >= 0) {
				rPulse2 = val_R;
				lPulse1 = 0;
			} else {
				lPulse1 = -val_R;
				rPulse2 = 0;
			}
		}
		break;
	case SETBALANCEKP:
        unerPrtcl_PutHeaderOnTx(dataTx, SETBALANCEKP, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        Kp_stable = myWord.i16[0];
		break;
	case SETBALANCEKD:
        unerPrtcl_PutHeaderOnTx(dataTx, SETBALANCEKD, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        Kd_stable = myWord.i16[0];
		break;
	case SETBALANCEKI:
        unerPrtcl_PutHeaderOnTx(dataTx, SETBALANCEKI, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        Ki_stable = myWord.i16[0];
		break;
	case SETPWMMINR:
        unerPrtcl_PutHeaderOnTx(dataTx, SETPWMMINR, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        minPWM_Right = myWord.ui16[0];
		break;
	case SETPWMMINL:
        unerPrtcl_PutHeaderOnTx(dataTx, SETPWMMINL, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        minPWM_Left = myWord.ui16[0];
		break;
	case SETSETPOINT:
		unerPrtcl_PutHeaderOnTx(dataTx, SETSETPOINT, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.i8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.i8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		setpoint = (int32_t) myWord.i16[0];
		if (robotMode != STATE_JOYSTICK && setpoint != 1500) {
			balance_setpoint_calib = setpoint;
		}
		break;
	case SETBKANG:
		unerPrtcl_PutHeaderOnTx(dataTx, SETBKANG, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		break;
	case SETSTATICOFF:
		unerPrtcl_PutHeaderOnTx(dataTx, SETSTATICOFF, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		staticOff = myWord.ui16[0];
		break;
	case SETMOVINGOFF:
		unerPrtcl_PutHeaderOnTx(dataTx, SETMOVINGOFF, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		movingOff = myWord.ui16[0];
		break;
	case GETINTERNALDATA:
		// Estructura para sincronización de parámetros (91 bytes de datos + 1 chk = 92)
		unerPrtcl_PutHeaderOnTx(dataTx, GETINTERNALDATA, 92);

		// 1. Bloque PID Balancín (10 bytes: Kp, Ki, Kd, Max, Min)
		int16_t pid_bal[5] = { Kp_stable, Ki_stable, Kd_stable, (int16_t)minPWM_Right, (int16_t)minPWM_Left};
		for (int i = 0; i < 5; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (pid_bal[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((pid_bal[i] >> 8) & 0xFF));
		}

		// 2. Setpoint (4 bytes - int32)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (setpoint & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((setpoint >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((setpoint >> 16) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((setpoint >> 24) & 0xFF));

		// 3. Bloque Seguimiento y Offsets (14 bytes: KpL, KdL, OffL, OffR, Turn, Attack, Kp_pared)
		int16_t params_ext[7] = { Kp_line, Kq_line, offset_left, offset_right, custom_turn, attack_setpoint, (int16_t)Kp_pared };
		for (int i = 0; i < 7; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (params_ext[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((params_ext[i] >> 8) & 0xFF));
		}

		// 4. Bloque Esquivador de Obstáculos (12 bytes: Front, Side, Lost, Stop, Corner, Align)
		uint16_t params_obs[6] = { obs_detect_dist, obs_side_dist, obs_lost_dist, obs_stop_cycles, obs_corner_dist, obs_align_dist};
		for (int i = 0; i < 6; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (params_obs[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((params_obs[i] >> 8) & 0xFF));
		}

		// 5. Bloque Rotación PWM (8 bytes: PWM_LRot, PWM_RRot, staticOff, movingOff)
		uint16_t params_rot[4] = { PWM_LRot, PWM_RRot, staticOff, movingOff };
		for (int i = 0; i < 4; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (params_rot[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((params_rot[i] >> 8) & 0xFF));
		}

		// 6. Nuevos Parámetros (4 bytes: angle_limit, Kp_ext)
		int16_t new_params[2] = { 0, Kp_ext };
		for (int i = 0; i < 2; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (new_params[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((new_params[i] >> 8) & 0xFF));
		}

		// 7. Anti-collapse Setpoints (4 bytes: Ki_ext, alfa_lpf)
		int16_t save_params[2] = { Ki_ext, alfa_lpf };
		for (int i = 0; i < 2; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (save_params[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((save_params[i] >> 8) & 0xFF));
		}

		// 8. current_angle_hr (4 bytes)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (current_angle_hr & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((current_angle_hr >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((current_angle_hr >> 16) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((current_angle_hr >> 24) & 0xFF));

		// 9. Vel Damp Params (4 bytes: vel_damp_div, vel_damp_limit)
		int16_t damp_params[2] = { vel_damp_div, vel_damp_limit };
		for (int i = 0; i < 2; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (damp_params[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((damp_params[i] >> 8) & 0xFF));
		}

		// 10. Turn Limit (2 bytes: turn_limit)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (turn_limit & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((turn_limit >> 8) & 0xFF));

		// 11. Sensores de línea RAW (6 bytes: IR1, IR3, IR5)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (adcDataTx[1] & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((adcDataTx[1] >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (adcDataTx[3] & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((adcDataTx[3] >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (adcDataTx[5] & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((adcDataTx[5] >> 8) & 0xFF));

		// 12. Sensores de línea Calibrados (6 bytes: IR1, IR3, IR5)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (cal_left_ir & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((cal_left_ir >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (cal_center_ir & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((cal_center_ir >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (cal_right_ir & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((cal_right_ir >> 8) & 0xFF));

		// 13. Kd Pared / Anticipo (2 bytes: Kd_anticipo)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (Kd_anticipo & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((Kd_anticipo >> 8) & 0xFF));

		// 14. Front Kp dummy (2 bytes: indices 82..83 en Qt)
		unerPrtcl_PutByteOnTx(dataTx, 0);
		unerPrtcl_PutByteOnTx(dataTx, 0);

		// 15. Kd Frontal / IR6 (2 bytes: indices 84..85 en Qt)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (Kd_frontal & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((Kd_frontal >> 8) & 0xFF));

		// 16. Dirección de esquivado (1 byte: index 86 en Qt)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (dodge_direction == 1 ? 1 : 0));

		// 17. Modo del robot y estado de esquivado (2 bytes: indices 87 y 88 en Qt)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) robotMode);
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) dodgeState);

		// 18. Sesgo de Esquivado (4 bytes: indices 89..90 y 91..92 en Qt)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (dodge_bias_time & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((dodge_bias_time >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (dodge_bias_mult & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((dodge_bias_mult >> 8) & 0xFF));

		// Checksum final
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	case GETPIDBALANCE:
			// Tamaño: 1(cmd) + 7 variables * 4 bytes = 29 bytes
			unerPrtcl_PutHeaderOnTx(dataTx, GETPIDBALANCE, 29);

			int32_t pid_telemetry[7] = { error, integral, derivative, output, current_angle, turn_offset, measured_dt_ms };

			for (int i = 0; i < 7; i++) {
				unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (pid_telemetry[i] & 0xFF));
				unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((pid_telemetry[i] >> 8) & 0xFF));
				unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((pid_telemetry[i] >> 16) & 0xFF));
				unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((pid_telemetry[i] >> 24) & 0xFF));
			}

			unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
			break;
	case SETLINEKP:
        unerPrtcl_PutHeaderOnTx(dataTx, SETLINEKP, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK);
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
        myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
        Kp_line = myWord.i16[0];
        break;
	case SETLINEKD:
        unerPrtcl_PutHeaderOnTx(dataTx, SETLINEKD, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK);
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
        myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
        Kq_line = myWord.i16[0];
        break;
	case SETOFFSETL:
		unerPrtcl_PutHeaderOnTx(dataTx, SETOFFSETL, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		offset_left = myWord.i16[0];
		break;
	case SETOFFSETR:
		unerPrtcl_PutHeaderOnTx(dataTx, SETOFFSETR, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		offset_right = myWord.i16[0];
		break;
	case SETCUSTOMTURN:
		unerPrtcl_PutHeaderOnTx(dataTx, SETCUSTOMTURN, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		custom_turn = myWord.i16[0];
		break;
	case SETSPEED:
		unerPrtcl_PutHeaderOnTx(dataTx, SETSPEED, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		attack_setpoint = myWord.i16[0];
		break;
	case SETFRONTDIST:
		unerPrtcl_PutHeaderOnTx(dataTx, SETFRONTDIST, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		obs_detect_dist = myWord.ui16[0];
		break;
	case SETSIDEDIST:
		unerPrtcl_PutHeaderOnTx(dataTx, SETSIDEDIST, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		obs_side_dist = myWord.ui16[0];
		break;
	case SETLOSTDIST:
		unerPrtcl_PutHeaderOnTx(dataTx, SETLOSTDIST, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		obs_lost_dist = myWord.ui16[0];
		break;
	case SETSTOPCYCLES:
		unerPrtcl_PutHeaderOnTx(dataTx, SETSTOPCYCLES, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		obs_stop_cycles = myWord.ui16[0];
		break;
	case SETCORNERDIST:
		unerPrtcl_PutHeaderOnTx(dataTx, SETCORNERDIST, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		obs_corner_dist = myWord.ui16[0];
		break;
	case SETALIGNDIST:
		unerPrtcl_PutHeaderOnTx(dataTx, SETALIGNDIST, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		obs_align_dist = myWord.ui16[0];
		break;
	case SETPWMLROT:
		unerPrtcl_PutHeaderOnTx(dataTx, SETPWMLROT, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		PWM_LRot = myWord.ui16[0];
		break;
	case SETPWMRROT:
		unerPrtcl_PutHeaderOnTx(dataTx, SETPWMRROT, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		PWM_RRot = myWord.ui16[0];
		break;
	case SET_KP_EXT:
		unerPrtcl_PutHeaderOnTx(dataTx, SET_KP_EXT, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		Kp_ext = myWord.i16[0];
		break;
	case SETWALLKP:
		unerPrtcl_PutHeaderOnTx(dataTx, SETWALLKP, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		Kp_pared = myWord.i16[0];
		break;
	case SETWALLKD:
		unerPrtcl_PutHeaderOnTx(dataTx, SETWALLKD, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		Kd_anticipo = myWord.i16[0];
		break;
	case SETFRONTKD:
		unerPrtcl_PutHeaderOnTx(dataTx, SETFRONTKD, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		Kd_frontal = myWord.i16[0];
		break;
	case SETFRONTKP:
		unerPrtcl_PutHeaderOnTx(dataTx, SETFRONTKP, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		break;
	case SETDODGEMODE:
		unerPrtcl_PutHeaderOnTx(dataTx, SETDODGEMODE, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		if (myWord.ui16[0] == 0) dodge_direction = -1;
		else if (myWord.ui16[0] == 1) dodge_direction = 1;
		break;
	case SETDODGEBIASTIME:
		unerPrtcl_PutHeaderOnTx(dataTx, SETDODGEBIASTIME, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		dodge_bias_time = myWord.i16[0];
		break;
	case SETDODGEBIASMULT:
		unerPrtcl_PutHeaderOnTx(dataTx, SETDODGEBIASMULT, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		dodge_bias_mult = myWord.i16[0];
		break;
	case SETLIMITANG:
		unerPrtcl_PutHeaderOnTx(dataTx, SETLIMITANG, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		break;
	case SET_KI_EXT:
		unerPrtcl_PutHeaderOnTx(dataTx, SET_KI_EXT, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		Ki_ext = myWord.i16[0];
		break;
	case SET_ALFA_LPF:
		unerPrtcl_PutHeaderOnTx(dataTx, SET_ALFA_LPF, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		alfa_lpf = myWord.i16[0];
		break;
	case SETVELDAMPDIV:
		unerPrtcl_PutHeaderOnTx(dataTx, SETVELDAMPDIV, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		vel_damp_div = myWord.i16[0];
		break;
	case SETVELDAMPLIM:
		unerPrtcl_PutHeaderOnTx(dataTx, SETVELDAMPLIM, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		vel_damp_limit = myWord.i16[0];
		break;
	case SETTURNLIMIT:
		unerPrtcl_PutHeaderOnTx(dataTx, SETTURNLIMIT, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		turn_limit = myWord.i16[0];
		break;
	case EXPORTIRCSV: {
		// Incrementar contador de exportación (persistente en sesión)
		static uint16_t ir_csv_export_count = 0;
		ir_csv_export_count++;
		// Responder: EXPORTIRCSV + ACK + count_hi + count_lo
		unerPrtcl_PutHeaderOnTx(dataTx, EXPORTIRCSV, 4);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t)((ir_csv_export_count >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t)(ir_csv_export_count & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	}
	case SETROBOTMODE: {
		unerPrtcl_PutHeaderOnTx(dataTx, SETROBOTMODE, 4);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		uint8_t reqMode = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		SetRobotModeRemote(reqMode);
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) robotMode);
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) dodgeState);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	}
	case SETJOYSTICKTURN: {
		unerPrtcl_PutHeaderOnTx(dataTx, SETJOYSTICKTURN, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		joystick_turn_offset = myWord.i16[0];
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		uint16_t turn_ms = myWord.ui16[0];
		if (joystick_turn_offset == 0) {
			joystick_turn_timer = 0;
			turn_offset = 0;
		} else if (turn_ms > 0 && turn_ms <= 3000) {
			joystick_turn_timer = turn_ms;
		} else {
			joystick_turn_timer = 200;
		}
		break;
	}
	case SETSOFTAP: {
		unerPrtcl_PutHeaderOnTx(dataTx, SETSOFTAP, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		softAPEnterDelay = 30; // 300 ms de margen para que el ACK salga por WiFi/Serie hacia la PC
		softAPEnterPending = 1;
		break;
	}
	default:
		unerPrtcl_PutHeaderOnTx(dataTx, (_eCmd) dataRx->buff[dataRx->indexData],
				2);
		unerPrtcl_PutByteOnTx(dataTx, UNKNOWN);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	}
}

void do10ms() {
	if (!IS10MS) return;
	IS10MS = FALSE;

	// --- 1. Tareas Periódicas de 10ms ---
	if (robotMode != STATE_3D_SCREEN) {
		ESP01_Timeout10ms();
		UART_EnforceReceiverActive();
		WiFi_ScanTick();
	}
	buttonTimeout10ms(&myButton);

	// Timeout de inactividad para recepción de credenciales SoftAP (Hercules TCP)
	if (isSoftAPMode && softAPRxTimer > 0) {
		softAPRxTimer--;
		if (softAPRxTimer == 0 && softAPBufIdx > 0 && !softAPBufReady) {
			softAPBuf[softAPBufIdx] = '\0';
			softAPBufReady = 1;
		}
	}

	// Cuenta regresiva para dar tiempo al envío del mensaje "OK" antes de reiniciar el ESP01
	if (softAPSwitchDelay > 0) {
		softAPSwitchDelay--;
	}

	// Cuenta regresiva para dar tiempo al envío del mensaje ACK antes de reiniciar a SoftAP
	if (softAPEnterDelay > 0) {
		softAPEnterDelay--;
	}

	// --- 2. Divisor de Tiempo: 20ms ---
	tmo20ms--;
	if (!tmo20ms) {
		tmo20ms = 2;
		IS20MS = TRUE;
	}

	// --- 3. Divisor de Tiempo: 100ms ---
	tmo100ms--;
	if (!tmo100ms) {
		tmo100ms = 10;
		IS100MS = TRUE;

		// Tarea Periódica de 100ms
		heartBeatTask();

		// Tareas Periódicas de 1s (Encapsuladas en tick de 100ms)
		if (robotMode != STATE_3D_SCREEN) {
			WiFi_HeartbeatTick();
		}
	}
}

void heartBeatTask() {
	static uint8_t times = 0;

	if (~heartBeatMask[hbIndex] & (1 << times)) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}

	times++;
	if (times >= 20) {
		times = 0; // Acotado a ciclo síncrono de exactly 2.0 segundos (20 ranuras x 100ms)
	}
}

void displayMemWrite(uint8_t address, uint8_t *data, uint8_t size, uint8_t type){
	HAL_I2C_Mem_Write(&hi2c2, address , type, 1, data, size, HAL_MAX_DELAY);
}

void displayMemWriteDMA(uint8_t address, uint8_t *data, uint8_t size, uint8_t type){
	HAL_I2C_Mem_Write_DMA(&hi2c2, address , type, 1, data, size);
}

static int32_t fast_sin_x1000(int32_t deg) {
	while (deg < 0) deg += 360;
	while (deg >= 360) deg -= 360;
	if (deg == 0 || deg == 180) return 0;
	if (deg == 90) return 1000;
	if (deg == 270) return -1000;

	int32_t sign = 1;
	if (deg > 180) {
		deg -= 180;
		sign = -1;
	}
	int32_t num = 4 * deg * (180 - deg);
	int32_t den = 40500 - deg * (180 - deg);
	return sign * ((num * 1000) / den);
}

static int32_t fast_cos_x1000(int32_t deg) {
	return fast_sin_x1000(deg + 90);
}

void mpuMemWrite(uint8_t address, uint8_t *data, uint8_t size, uint8_t type){
	HAL_I2C_Mem_Write(&hi2c2, address , type, 1, data, size, HAL_MAX_DELAY);
}

void mpuMemReadDMA(uint8_t address, uint8_t *data, uint8_t size, uint8_t type){
	HAL_I2C_Mem_Read_DMA(&hi2c2, address , type, 1, data, size);
}

void ssd1306Data() {
	char data[32];
	uint8_t y = 0, x = 2;

	if (robotMode == STATE_FIRST_SCREEN) {
		ssd1306_Fill(White);

		ssd1306_FillRectangle(30, 0, 32, 64, Black);

		ssd1306_FillRectangle(0, 20, 128, 22, Black);

		ssd1306_FillRectangle(80, 0, 82, 64, Black);

		x = SSD1306_SNDCOL;
		y = 0;
		ssd1306_SetCursor(x, y);
		snprintf(data, sizeof(data), "ACC");
		ssd1306_WriteString(data, Font_11x18, Black);
		x = SSD1306_TRDCOL;
		ssd1306_SetCursor(x, y);
		snprintf(data, sizeof(data), "GYR");
		ssd1306_WriteString(data, Font_11x18, Black);

		x = SSD1306_SNDCOL;
		y += 25;
		ssd1306_SetCursor(x, y);
		snprintf(data, sizeof(data), "%d", ax);
		ssd1306_WriteString(data, Font_7x10, (SSD1306_COLOR)Black);
		x = SSD1306_TRDCOL;
		ssd1306_SetCursor(x, y);
		snprintf(data, sizeof(data), "%d", gx);
		ssd1306_WriteString(data, Font_7x10, (SSD1306_COLOR)Black);

		x = SSD1306_SNDCOL;
		y += 12;
		ssd1306_SetCursor(x, y);
		snprintf(data, sizeof(data), "%d", ay);
		ssd1306_WriteString(data, Font_7x10, (SSD1306_COLOR)Black);
		x = SSD1306_TRDCOL;
		ssd1306_SetCursor(x, y);
		snprintf(data, sizeof(data), "%d", gy);
		ssd1306_WriteString(data, Font_7x10, (SSD1306_COLOR)Black);

		x = SSD1306_SNDCOL;
		y += 12;
		ssd1306_SetCursor(x, y);
		snprintf(data, sizeof(data), "%d", az);
		ssd1306_WriteString(data, Font_7x10, (SSD1306_COLOR)Black);
		x = SSD1306_TRDCOL;
		ssd1306_SetCursor(x, y);
		snprintf(data, sizeof(data), "%d", gz);
		ssd1306_WriteString(data, Font_7x10, (SSD1306_COLOR)Black);

		ssd1306_Line(3, 60, 3,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[0] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(6, 60, 6,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[1] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(9, 60, 9,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[2] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(12, 60, 12,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[3] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(15, 60, 15,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[4] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(18, 60, 18,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[5] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(21, 60, 21,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[6] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(24, 60, 24,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[7] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
	} else if (robotMode == STATE_SECOND_SCREEN) {
		// Modo Brújula de Referencia OLED (Reemplazo de pantalla 2 segundos)
		ssd1306_Fill(Black);

		// 1. Centro y radio de la circunferencia de la brújula
		const int16_t cx = 36;
		const int16_t cy = 32;
		const int16_t r = 27;

		// 2. Circunferencia exterior
		ssd1306_DrawCircle(cx, cy, r, White);

		// Marcas cardinales en la circunferencia
		ssd1306_Line(cx, cy - r, cx, cy - r + 3, White);     // N (0°)
		ssd1306_Line(cx + r - 3, cy, cx + r, cy, White);     // E (90°)
		ssd1306_Line(cx, cy + r - 3, cx, cy + r, White);     // S (180°)
		ssd1306_Line(cx - r, cy, cx - r + 3, cy, White);     // W (270°)

		// 3. Flechita indicando la referencia fijada al entrar al modo
		// delta respecto al punto de referencia guardado (en grados enteros)
		int32_t delta_yaw_mdeg = total_yaw_hr - compass_ref_yaw;
		int32_t disp_deg = delta_yaw_mdeg / 1000;
		while (disp_deg > 180)  disp_deg -= 360;
		while (disp_deg <= -180) disp_deg += 360;

		// La flecha apunta a la referencia fijada (0° = 12 en punto / arriba)
		int32_t arrow_angle = -disp_deg;
		int32_t sin_a = fast_sin_x1000(arrow_angle);
		int32_t cos_a = fast_cos_x1000(arrow_angle);

		// Punta de la flecha
		int16_t tip_x = cx + (int16_t)(((r - 4) * sin_a) / 1000);
		int16_t tip_y = cy - (int16_t)(((r - 4) * cos_a) / 1000);

		// Cola posterior de la flecha
		int16_t tail_x = cx - (int16_t)((6 * sin_a) / 1000);
		int16_t tail_y = cy + (int16_t)((6 * cos_a) / 1000);

		// Aletas de la flecha (apertura angular ~28°)
		int16_t left_x = tip_x - (int16_t)((8 * fast_sin_x1000(arrow_angle + 28)) / 1000);
		int16_t left_y = tip_y + (int16_t)((8 * fast_cos_x1000(arrow_angle + 28)) / 1000);
		int16_t right_x = tip_x - (int16_t)((8 * fast_sin_x1000(arrow_angle - 28)) / 1000);
		int16_t right_y = tip_y + (int16_t)((8 * fast_cos_x1000(arrow_angle - 28)) / 1000);

		// Dibujar flechita
		ssd1306_Line(tail_x, tail_y, tip_x, tip_y, White);
		ssd1306_Line(tip_x, tip_y, left_x, left_y, White);
		ssd1306_Line(tip_x, tip_y, right_x, right_y, White);
		ssd1306_Line(left_x, left_y, right_x, right_y, White);

		// Eje central
		ssd1306_FillCircle(cx, cy, 2, White);

		// 4. Panel derecho con lecturas y punto de referencia
		ssd1306_Line(70, 4, 70, 60, White);

		ssd1306_SetCursor(75, 4);
		ssd1306_WriteString("BRUJULA", Font_7x10, White);

		ssd1306_SetCursor(75, 18);
		ssd1306_WriteString("REF: 0*", Font_7x10, White);

		char ang_str[16];
		if (disp_deg > 0) {
			snprintf(ang_str, sizeof(ang_str), "+%d*", (int)disp_deg);
		} else {
			snprintf(ang_str, sizeof(ang_str), "%d*", (int)disp_deg);
		}
		ssd1306_SetCursor(75, 33);
		ssd1306_WriteString(ang_str, Font_7x10, White);

		// Indicador de orientación respecto a referencia
		const char *cardinal = "NORTE";
		if (disp_deg >= -22 && disp_deg <= 22)        cardinal = "NORTE";
		else if (disp_deg > 22 && disp_deg < 67)      cardinal = "NE";
		else if (disp_deg >= 67 && disp_deg <= 112)   cardinal = "ESTE";
		else if (disp_deg > 112 && disp_deg < 157)    cardinal = "SE";
		else if (disp_deg >= 157 || disp_deg <= -157) cardinal = "SUR";
		else if (disp_deg > -157 && disp_deg < -112)  cardinal = "SO";
		else if (disp_deg >= -112 && disp_deg <= -67) cardinal = "OESTE";
		else                                          cardinal = "NO";

		ssd1306_SetCursor(75, 48);
		ssd1306_WriteString((char*)cardinal, Font_7x10, White);
	}
}

void i2cTask() {
	static uint8_t i = IDLE;
	static uint8_t j = 0;
	static uint32_t mpu_timeout = 0; // NUEVO: Contador de paciencia

	// DESBLOQUEO DE SEGURIDAD: Forzar retorno a IDLE si estamos en modo movimiento para no colgar la lectura MPU
	if (robotMode == STATE_SWING || robotMode == STATE_LINE_FOLLOWING || robotMode == STATE_DODGE) {
		if (i == DATA_DISPLAY || i == UPD_DISPLAY) {
			i = IDLE;
		}
	}

	switch (i) {
	case IDLE:
		if (j == i2cIndex) {
			break;
		}

		if (Pila[j]) { //mpu6050
			i = ONMPU;
			j++;
			j &= (I2CSIZE - 1);
			break;
		}
		if (!Pila[j]) {
			i = DATA_DISPLAY;
			j++;
			j &= (I2CSIZE - 1);
		}
		break;
	case DATA_DISPLAY:
		if (robotMode == STATE_3D_SCREEN) {
			WIREGFX_DisplayTask();
		} else {
			ssd1306Data();
		}
		i = UPD_DISPLAY;
		break;
	case UPD_DISPLAY:
		if (HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) {
			if (ssd1306_UpdateScreenDMA()) {
				ssd1306_TxCplt = FALSE;
				i = IDLE;
			}
		}
		break;
	case ONMPU:
		// 1. Si el hardware I2C detecta un error eléctrico, forzamos un reinicio inmediato
		if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_NONE) {
			mpu_timeout = 1000;
		}

		// 2. Intentamos leer el sensor
		if (HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) {
			if (mpu6050_Read()) {
				mpu6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
				mpu6050_RxCplt = FALSE;
				RUN_PID = TRUE;
				mpu_timeout = 0; // Lectura exitosa: el sensor vive, reseteamos timeout
				i = IDLE;
			} else {
				// Está esperando que el DMA conteste
				mpu_timeout++;
			}
		} else {
			// El bus I2C está BUSY (Ocupado/Trancado)
			mpu_timeout++;
		}

		// 3. --- EL DESFIBRILADOR ---
		// Si pasó mucho tiempo atascado esperando el DMA o en estado BUSY
		if (mpu_timeout > 500) {
			// Apagamos el hardware I2C para limpiar los registros corruptos
			HAL_I2C_DeInit(&hi2c2);

			// Le damos tiempo a los voltajes de los cables para estabilizarse
			HAL_Delay(1);

			// Lo volvemos a encender
			HAL_I2C_Init(&hi2c2);

			// Reconfiguramos los registros del MPU y reseteamos la máquina de estados
			mpu6050_Init();
			mpu6050_Reset_State();

			mpu_timeout = 0;
			i = IDLE; // Volvemos al bucle principal
		}
		break;
	default:
		i = IDLE;
		break;
	}
}

void PWM_Control(){

	// +-------------------------------------------------------+
	// | TABLA DE ESTADOS - CONTROLADOR L9110S                 |
	// +------------+------------+--------------+--------------+
	// | Entrada IA | Entrada IB | Salida Motor | Estado       |
	// +------------+------------+--------------+--------------+
	// |    LOW     |    LOW     |     OFF      | Frenado/Stop |
	// |    HIGH    |    LOW     |    AVANCE    | Giro Horario |
	// |    LOW     |    HIGH    |  RETROCESO   | Giro Antihor.|
	// |    HIGH    |    HIGH    |     OFF      | Frenado/Stop |
	// +------------+------------+--------------+--------------+
	// | * Nota: Las salidas quedan en estado "flotante" si las |
	// |   entradas son iguales (ambas HIGH o ambas LOW).      |
	// +-------------------------------------------------------+

	//Rueda izquierda
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,lPulse1);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,lPulse3);

	//Rueda derecha
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,rPulse2);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,rPulse4);

}

void CHPD_Control(uint8_t state)
{
    /* Assuming CH_PD is on GPIOB Pin 0 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  Send one byte out over UART to ESP-01.
 * @param  byte  The data byte to transmit.
 */

int USART_SendByte(uint8_t byte)
{
    /* Assuming huart2 is configured for the ESP01 */
    if(HAL_UART_Transmit_IT(&huart1, &byte, 1) == HAL_OK)
    	return 1;
    else
    	return 0;
}

/**
 * @brief  Forward one received byte into the ESP01 driver’s rx buffer.
 * @param  byte  The byte received from UART ISR.
 */
void FeedRxBuf(uint8_t byte)
{
    ESP01_WriteRX(byte);
}

void DebugESP01_To_USB(const char *msg) {
    // strlen requiere #include <string.h>
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

// Esta función la llama el driver cuando tiene un byte de datos limpio
void WiFi_Data_Callback(uint8_t byte)
{
    if(isSoftAPMode){
        /*
         * Modo SoftAP: captura de credenciales enviadas desde software Hercules (TCP Client).
         * Formato de recepción: "SSID;PASS\r\n", "SSID;PASS\n" o "SSID;PASS" directo.
         */
        if(softAPBufReady)
            return; /* Trama previa pendiente de procesar en softAPTask */

        if(byte == '\r' || byte == '\n'){
            if(softAPBufIdx > 0){
                softAPBuf[softAPBufIdx] = '\0';
                softAPBufReady = 1;
                softAPRxTimer = 0;
            }
        } else if(softAPBufIdx < (SOFTAP_BUF_SIZE - 1)){
            softAPBuf[softAPBufIdx++] = (char)byte;
            softAPBuf[softAPBufIdx]   = '\0';
            softAPRxTimer = 5; /* 50ms de inactividad para disparo automático si no hay \r o \n */
        }
    } else {
        /* Modo station UDP/TCP: protocolo UNER */
        WiFiRx.buff[WiFiRx.indexW++] = byte;
        WiFiRx.indexW &= WiFiRx.mask;
    }
}

/**
 * @brief Callback del driver ESP01: se llama cada vez que cambia el estado
 *
 * Cuando el WiFi se conecta (ESP01_WIFI_CONNECTED), arranca el socket UDP/TCP
 * automáticamente usando la IP y puerto guardados.
 */
void OnESP01ChangeState(_eESP01STATUS state)
{
    if(state == ESP01_WIFI_CONNECTED){
        /* ¡Éxito! Se conectó a la red */
        if(networkScanActive){
            strncpy(udpTargetIP, knownNetworks[currentNetworkIdx].targetIP, 15);
            udpTargetIP[15] = '\0';
            networkScanActive = 0; /* Detenemos el escaneo */
        }
        udpReadyToStart = 1;
        udpSilenceCounter = 0;
    }
    else if(state == ESP01_UDPTCP_CONNECTED){
        /* Socket UDP o TCP conectado con éxito: resetear silencio */
        udpSilenceCounter = 0;
    }
    else if(state == ESP01_WIFI_DISCONNECTED){
        /* Si perdemos la conexión en pleno uso, reactivamos la búsqueda */
        if(!isSoftAPMode && !networkScanActive){
            networkScanActive = 1;
            networkScanTimer = SCANTIME; /* Le damos tiempo a la red para recuperarse */
            ESP01_SetWIFI(knownNetworks[currentNetworkIdx].ssid,
                          knownNetworks[currentNetworkIdx].password);
        }
    }
}

/**
 * @brief Tarea de gestión de conexión y procesamiento de credenciales SoftAP (Hercules)
 */
void softAPTask(void)
{
    /* Iniciar UDP/TCP en cuanto el WiFi este listo (viene del callback) */
    if(udpReadyToStart){
        udpReadyToStart = 0;
        if(strcmp(udpTargetProto, "TCP") == 0){
            ESP01_StartTCP(udpTargetIP, udpTargetPort, 30001);
        } else {
            ESP01_StartUDP(udpTargetIP, udpTargetPort, 30001);
        }
        return;
    }

    /* Transición forzada a modo SoftAP solicitada por comando remoto SETSOFTAP */
    if(softAPEnterPending){
        if(softAPEnterDelay == 0){
            softAPEnterPending = 0;
            isSoftAPMode = 1;
            networkScanActive = 0;
            udpReadyToStart = 0;
            softAPBufIdx = 0;
            softAPBuf[0] = '\0';
            softAPBufReady = 0;
            softAPRxTimer = 0;
            softAPSwitchPending = 0;
            softAPSwitchDelay = 0;
            ESP01_SetSoftAP("MICRO", "12345678", 5, 3, SOFTAP_TCP_PORT);
        }
        return;
    }

    if(!isSoftAPMode)
        return;

    /* Si hay un cambio a modo Station programado tras enviar la confirmación */
    if(softAPSwitchPending){
        if(softAPSwitchDelay == 0){
            softAPSwitchPending = 0;
            isSoftAPMode = 0;
            ESP01_SetWIFI(softAP_TargetSSID, softAP_TargetPASS);
        }
        return;
    }

    if(!softAPBufReady)
        return;

    /* Procesar comando de configuración: "SSID;PASS" */
    char *delim = strchr(softAPBuf, ';');
    if(delim != NULL){
        *delim = '\0';
        char *newSSID = softAPBuf;
        char *newPASS = delim + 1;

        /* Limpiar retornos, saltos de línea y espacios en SSID y Contraseña */
        while(*newSSID == ' '){
            newSSID++;
        }
        int lenSSID = strlen(newSSID);
        while(lenSSID > 0 && (newSSID[lenSSID - 1] == '\r' || newSSID[lenSSID - 1] == '\n' || newSSID[lenSSID - 1] == ' ')){
            newSSID[--lenSSID] = '\0';
        }

        char *newIP = NULL;
        char *newProto = NULL;
        char *delim2 = strchr(newPASS, ';');
        if(delim2 != NULL){
            *delim2 = '\0';
            newIP = delim2 + 1;
            char *delim3 = strchr(newIP, ';');
            if(delim3 != NULL){
                *delim3 = '\0';
                newProto = delim3 + 1;
            }
        }

        while(*newPASS == ' '){
            newPASS++;
        }
        int lenPass = strlen(newPASS);
        while(lenPass > 0 && (newPASS[lenPass - 1] == '\r' || newPASS[lenPass - 1] == '\n' || newPASS[lenPass - 1] == ' ')){
            newPASS[--lenPass] = '\0';
        }

        if(newIP != NULL){
            while(*newIP == ' ') newIP++;
            int lenIP = strlen(newIP);
            while(lenIP > 0 && (newIP[lenIP - 1] == '\r' || newIP[lenIP - 1] == '\n' || newIP[lenIP - 1] == ' ')){
                newIP[--lenIP] = '\0';
            }
            if(strlen(newIP) >= 7 && strlen(newIP) < sizeof(udpTargetIP)){
                strncpy(udpTargetIP, newIP, sizeof(udpTargetIP) - 1);
                udpTargetIP[sizeof(udpTargetIP) - 1] = '\0';
            }
        } else {
            /* Si no se especificó IP en Hercules, buscar automáticamente en knownNetworks */
            for(int i = 0; i < NUM_KNOWN_NETWORKS; i++){
                if(strcmp(newSSID, knownNetworks[i].ssid) == 0){
                    strncpy(udpTargetIP, knownNetworks[i].targetIP, sizeof(udpTargetIP) - 1);
                    udpTargetIP[sizeof(udpTargetIP) - 1] = '\0';
                    break;
                }
            }
        }

        if(newProto != NULL){
            while(*newProto == ' ') newProto++;
            int lenProto = strlen(newProto);
            while(lenProto > 0 && (newProto[lenProto - 1] == '\r' || newProto[lenProto - 1] == '\n' || newProto[lenProto - 1] == ' ')){
                newProto[--lenProto] = '\0';
            }
            if(strncmp(newProto, "UDP", 3) == 0 || strncmp(newProto, "udp", 3) == 0){
                strcpy(udpTargetProto, "UDP");
            } else if(strncmp(newProto, "TCP", 3) == 0 || strncmp(newProto, "tcp", 3) == 0){
                strcpy(udpTargetProto, "TCP");
            }
        }

        if(strlen(newSSID) > 0){
            /* Guardar credenciales de forma permanente ANTES de tocar o limpiar softAPBuf */
            strncpy(softAP_TargetSSID, newSSID, sizeof(softAP_TargetSSID) - 1);
            softAP_TargetSSID[sizeof(softAP_TargetSSID) - 1] = '\0';
            strncpy(softAP_TargetPASS, newPASS, sizeof(softAP_TargetPASS) - 1);
            softAP_TargetPASS[sizeof(softAP_TargetPASS) - 1] = '\0';

            /* Responder confirmación al cliente Hercules */
            uint8_t connID = ESP01_GetLastConnID();
            const char msgOk[] = "OK: Conectando a red...\r\n";
            ESP01_Send(connID, (uint8_t*)msgOk, 0, sizeof(msgOk) - 1, sizeof(msgOk));

            /* Esperar 800ms (80 ticks de 10ms) para que el ESP-01 transmita
             * el mensaje completo por TCP a Hercules antes de reiniciar en modo Station */
            softAPSwitchDelay = 80;
            softAPSwitchPending = 1;
        } else {
            uint8_t connID = ESP01_GetLastConnID();
            const char msgErr[] = "ERROR: SSID vacio\r\n";
            ESP01_Send(connID, (uint8_t*)msgErr, 0, sizeof(msgErr) - 1, sizeof(msgErr));
        }

        /* Limpiar buffer de recepción */
        softAPBufIdx = 0;
        softAPBuf[0] = '\0';
        softAPBufReady = 0;
        softAPRxTimer = 0;
    } else {
        uint8_t connID = ESP01_GetLastConnID();
        const char msgErr[] = "ERROR: Formato esperado SSID;PASS\r\n";
        ESP01_Send(connID, (uint8_t*)msgErr, 0, sizeof(msgErr) - 1, sizeof(msgErr));

        softAPBufIdx = 0;
        softAPBuf[0] = '\0';
        softAPBufReady = 0;
        softAPRxTimer = 0;
    }
}

//BOTONES
void initButton(_sButton *button){
    button->currentState = BUTTON_UP;
    button->stateInput = NO_EVENT;
    button->isPressed = FALSE;
    button->time = 0;
    button->clickCount = 0;
    button->justReleased = FALSE;
}

/**
 * @brief Máquina de estados finita de antirrebote (Debouncing) para el pulsador de usuario.
 * @details Filtra transitorios electromecánicos validando transiciones en dos ciclos consecutivos:
 *          `BUTTON_UP` &rarr; `BUTTON_FALLING` &rarr; `BUTTON_DOWN` &rarr; `BUTTON_RISING` &rarr; `BUTTON_UP`.
 *
 * \startuml
 * title MEF Antirrebote de Botón (updateMefTask)
 * [*] --> BUTTON_UP
 * BUTTON_UP --> BUTTON_FALLING : stateInput == PRESSED
 * BUTTON_FALLING --> BUTTON_DOWN : stateInput == PRESSED\n(isPressed = TRUE)
 * BUTTON_FALLING --> BUTTON_UP : stateInput != PRESSED (Rebote)
 * BUTTON_DOWN --> BUTTON_RISING : stateInput == NOT_PRESSED
 * BUTTON_RISING --> BUTTON_UP : stateInput == NOT_PRESSED\n(justReleased = TRUE, retorna TRUE)
 * BUTTON_RISING --> BUTTON_DOWN : stateInput != NOT_PRESSED (Rebote)
 * \enduml
 *
 * @param[in,out] button Puntero a la estructura `_sButton`.
 * @return TRUE (1) si ocurrió una transición completa de pulsación y liberación; FALSE (0) en caso contrario.
 * @pre Estructura inicializada con `initButton()`.
 * @post Modifica `currentState`, `isPressed`, `justReleased`.
 * @see _sButton
 * @see _eButtonState
 * @see buttonTask
 * @ingroup group_ui_graphics
 */
uint8_t updateMefTask(_sButton *button){
    uint8_t action=FALSE;

    switch (button->currentState){
        case BUTTON_UP:
            if(button->stateInput==PRESSED)
                button->currentState=BUTTON_FALLING;
        break;
        case BUTTON_FALLING:
            if(button->stateInput==PRESSED){
                button->currentState=BUTTON_DOWN;
                button->isPressed=TRUE;
            }else{
                button->currentState=BUTTON_UP;
            }
        break;
        case BUTTON_DOWN:
            if(button->stateInput==NOT_PRESSED)
                button->currentState=BUTTON_RISING;
        break;
        case BUTTON_RISING:
            if(button->stateInput==NOT_PRESSED){
                button->currentState=BUTTON_UP;
                button->isPressed = FALSE;
                button->justReleased = TRUE; // Capturar flanco de subida de liberación
                action=TRUE;
            }else{
                button->currentState=BUTTON_DOWN;
            }
        break;
        default:
            button->currentState=BUTTON_UP;
        break;
    }
    return action;
}

void buttonTimeout10ms(_sButton *button){
    static uint8_t timeToDebounce = 0;
    static uint8_t release_ticks = 0;
    static uint8_t was_pressed = FALSE;

    // Leer el estado físico instantáneo del pin
    _eEvent current_pin = (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == GPIO_PIN_RESET) ? PRESSED : NOT_PRESSED;

    // Si el botón está actualmente presionado, aplicamos un filtro de liberación lento contra rebotes por vibración
    if (button->isPressed) {
        if (current_pin == NOT_PRESSED) {
            release_ticks++;
            if (release_ticks >= 8) { // Requiere 80ms estables de liberación física para confirmar soltado
                button->stateInput = NOT_PRESSED;
                release_ticks = 0;
            }
        } else {
            button->stateInput = PRESSED;
            release_ticks = 0;
        }
        timeToDebounce = 0; // Sincronizar
    } else {
        // Si no está presionado, se utiliza el debouncer rápido de 50ms original del proyecto
        release_ticks = 0;
        if(timeToDebounce > DEBOUNCE){
            timeToDebounce = 0;
            button->stateInput = current_pin;
        } else {
            timeToDebounce++;
        }
    }

    // Lógica unificada para el acumulador / temporizador "time"
    if (button->isPressed) {
        if (!was_pressed) {
            button->time = 0; // Reiniciar para empezar a cronometrar la pulsación actual desde cero
            was_pressed = TRUE;
        }
        button->time += 10;
    } else {
        was_pressed = FALSE;
        // Si no está presionado y hay clics acumulados, decrementamos la ventana de multiclic
        if (button->clickCount > 0 && button->time > 0) {
            if (button->time >= 10) {
                button->time -= 10;
            } else {
                button->time = 0;
            }
        } else if (button->clickCount == 0) {
            button->time = 0;
        }
    }
}

void buttonTask(_sButton *button) {
	// 1. Evaluar si la ventana de clics múltiples terminó (time llegó a 0) sin eventos de liberación pendientes
	if (!button->isPressed && button->time == 0 && button->clickCount > 0 && !button->justReleased) {
		// Retornar y reintentar en el próximo ciclo si el bus I2C está ocupado, previniendo bloqueos y jitter
		if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
			return;
		}

		switch (button->clickCount) {
			case 1: SetRobotMode(STATE_SWING); break;
			case 2: SetRobotMode(STATE_LINE_FOLLOWING); break;
			case 3: SetRobotMode(STATE_DODGE); break;
			case 4: SetRobotMode(STATE_JOYSTICK); break;
			case 5: SetRobotMode(STATE_3D_SCREEN); break;
			default: SetRobotMode(STATE_LINE_FOLLOWING); break;
		}
		button->clickCount = 0; // Resetear contador
	}

	// 2. Solo tomamos decisiones de temporización de pulsación ante un EVENTO DE LIBERACIÓN
	if (button->justReleased) {
		if (button->time >= T3000MS) {
			button->justReleased = FALSE; // Consumir el evento
			button->time = 0; // Sin acción por ahora, limpiar
		}
		else if (button->time >= T2000MS && button->time < T3000MS) {
			if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
				return;
			}
			button->justReleased = FALSE; // Consumir el evento
			SetRobotMode(STATE_SECOND_SCREEN);
			button->clickCount = 0;  // Abortar clics cortos pendientes
			button->time = 0;        // Limpiar
		}
		else if (button->time >= T1000MS && button->time < T2000MS) {
			if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
				return;
			}
			button->justReleased = FALSE; // Consumir el evento
			SetRobotMode(STATE_FIRST_SCREEN);
			button->clickCount = 0;  // Abortar clics cortos pendientes
			button->time = 0;        // Limpiar
		}
		else if (button->time < T1000MS) {
			button->justReleased = FALSE; // Consumir el evento
			// Clic corto detectado (duración < 1s) -> Sumar al contador y arrancar ventana de 400ms reutilizando 'time'
			button->clickCount++;
			button->time = T400MS; 
		}
	}
}

/**
 * @brief Algoritmo de seguimiento de línea con guiñada no lineal, cruce y navegación a ciegas (Dead Reckoning).
 * @details Implementa la cinemática de guiñada del robot sobre pista:
 *          1. **Filtro y Discriminación Óptica**: Binariza las lecturas analógicas normalizadas de los 3 fotosensores frontales (`IR_WHITE`).
 *          2. **Estimación de Centroide**: Calcula el error de desviación lateral $e_{line} \in [-100, +100]$ mediante el centro de masa óptico normalizado por la suma de reflectancia.
 *          3. **Control No Lineal P + Cuadrático**: Sintetiza el par de guiñada combinando una acción proporcional suave para rectas y un término cuadrático ($K_{q\_line} \cdot e_{line} \cdot |e_{line}|$) con alta autoridad correctiva en curvas cerradas.
 *          4. **Filtro de Desenganche**: Temporizador de rebote de 6 ciclos (30 ms) para ignorar huecos o discontinuidades menores en la línea.
 *          5. **Maniobra de Dead Reckoning en Pérdida**: En desenganche total, utiliza el giróscopo Z integrado para ejecutar un patrón de búsqueda angular (+90° $\rightarrow$ espera 2s $\rightarrow$ -180° $\rightarrow$ frenado estático a -2.5° en rampa).
 *          6. **Resolución de Cruces en Cruz**: En bifurcaciones/cruces de 3 sensores, aplica jitter estocástico con el registro SysTick para desempatar y pivotar sobre el propio eje.
 *
 * \startuml
 * skinparam backgroundColor white
 * skinparam arrowColor #2C3E50
 * skinparam activity {
 *   BackgroundColor #F4F6F7
 *   BorderColor #34495E
 *   FontColor #2C3E50
 * }
 * skinparam partition {
 *   BackgroundColor #EAEDED
 *   BorderColor #7F8C8D
 *   FontColor #1A5276
 * }
 *
 * start
 * partition "Filtro / Sensores (Adquisición y Normalización Óptica)" {
 *   :Lectura de reflectancia analógica:\n**left_ir**, **center_ir**, **right_ir**;
 *   :Binarización por umbral óptico (IR_WHITE):\n**ir1_active = (left_ir < IR_WHITE)**\n**ir3_active = (center_ir < IR_WHITE)**\n**ir5_active = (right_ir < IR_WHITE)**;
 *   :Conteo de sensores sobre línea reflectiva:\n**active_count = ir1_active + ir3_active + ir5_active**;
 * }
 *
 * partition "Controlador de Seguimiento y Guiñada No Lineal" {
 *   switch (lineState)
 *   case (LINE_SEARCHING)
 *     :Consigna longitudinal de exploración: ***target_setpoint = attack_setpoint**;
 *     if (¿Sensor central detecta línea? (ir3_active)) then (Sí)
 *       :Línea enganchada: transición a **LINE_FOLLOWING**;
 *     else (No)
 *       :Aplicar giro de barrido:\n**turn_offset = -active_turn**;
 *     endif
 *
 *   case (LINE_FOLLOWING)
 *     if (¿Cruce ortogonal o línea transversal?\n(active_count == 3)) then (Sí: Cruce Detectado)
 *       :Resolución de giro pseudoaleatorio con jitter SysTick:\n**cross_direction = ((SysTick->VAL & 1) != 0) ? 1 : -1**\n**lineState = LINE_CROSS**;
 *     else (Pista Continua)
 *       if (¿Pérdida temporal de línea?\n(active_count == 0)) then (Sí: Pérdida)
 *         :Incrementar contador de debounce:\n**line_lost_debounce_count++**;
 *         if (¿line_lost_debounce_count >= 6? [30 ms]) then (Pérdida Confirmada)
 *           :Iniciar rutina de navegación a ciegas (Dead Reckoning):\n**lineState = LINE_LOST**\n**line_lost_phase = LINE_LOST_ROT_90**\n**search_direction = (last_line_error >= 0) ? -1 : 1**;
 *         else (Filtro Transitorio)
 *           :Mantener último error y guiñada:\n**error_linea = last_line_error**\n**turn_offset = last_turn_offset**;
 *         endif
 *       else (No: Línea Detectada)
 *         :Resetear contador: **line_lost_debounce_count = 0**;
 *         
 *         :Cálculo del Error de Posición Lateral (Centroide):\n**error_linea = (((-1000 * left_ir) + (1000 * right_ir)) / sum_sensors) / 10**\n**abs_error = |error_linea|**;
 *         
 *         :Cálculo del Término Proporcional Lineal:\n**linear_term = (Kp_line * error_linea) / 100**;
 *         
 *         :Cálculo del Término Cuadrático No Lineal (Mayor autoridad en curvas):\n**quad_term = (Kq_line * error_linea * abs_error) / SCALE_LINE**;
 *         
 *         :Suma de Esfuerzo de Guiñada Total:\n**turn_offset = linear_term + quad_term**;
 *         
 *         :Saturación de Guiñada (Clamping):\n**turn_offset = clamp(turn_offset, -turn_limit, +turn_limit)**;
 *         
 *         :Actualizar memoria de estado:\n**last_turn_offset = turn_offset**\n**last_line_error = error_linea**;
 *       endif
 *     endif
 *     :Consigna longitudinal de avance:\n***target_setpoint = attack_setpoint**;
 *
 *   case (LINE_LOST)
 *     if (¿Cualquier sensor recupera línea?\n(active_count > 0)) then (Re-enganche Inmediato)
 *       :Retorno instantáneo a **LINE_FOLLOWING**;
 *     else (Dead Reckoning con Giroscopio)
 *       :Integración giroscópica de guiñada:\n**gz_cal = gz - gz_offset**\n**line_lost_yaw += (gz_cal * DT_US) / 131000**;
 *       if (line_lost_phase == LINE_LOST_ROT_90) then
 *         :Rotar 90° hacia el último lado conocido con **LINE_LOST_TURN_SPEED**;
 *       elseif (line_lost_phase == LINE_LOST_WAIT_2S) then
 *         :Pausa estática de observación (2 segundos), **turn_offset = 0**;
 *       elseif (line_lost_phase == LINE_LOST_ROT_180) then
 *         :Rotar 180° en sentido opuesto con **LINE_LOST_TURN_SPEED**;
 *       else (LINE_LOST_STOPPED)
 *         :Frenado estático en rampa:\n***target_setpoint = -250**, **turn_offset = 0**;
 *       endif
 *     endif
 *
 *   case (LINE_CROSS)
 *     :Consigna de giro erguido: ***target_setpoint = +350**;
 *     if (¿Queda exactamente un sensor sobre la línea? (active_count == 1)) then (Línea Tomada)
 *       :Finalizar cruce: **turn_offset = 0**, **lineState = LINE_FOLLOWING**;
 *     else (En Rotación)
 *       :Rotar sobre su propio eje:\n**turn_offset = cross_direction * 350**;
 *     endif
 *   endswitch
 * }
 *
 * partition "Actuación (PWM) y Despacho a PID" {
 *   :Exportar consignas calculadas: ***target_setpoint** y **turn_offset**;
 *   :Despacho al mezclador de motores en **PID_Calcular(target_setpoint)**\npara modulación y accionamiento PWM en puente H;
 * }
 * stop
 * \enduml
 *
 * @param[in] left_ir Lectura analógica normalizada del fotosensor infrarrojo izquierdo.
 * @param[in] center_ir Lectura analógica normalizada del fotosensor infrarrojo central.
 * @param[in] right_ir Lectura analógica normalizada del fotosensor infrarrojo derecho.
 * @param[out] target_setpoint Puntero a la consigna de inclinación longitudinal para el lazo PID.
 * @pre ADC1 DMA completado, sensores calibrados, `robotMode == STATE_LINE_FOLLOWING`.
 * @post Actualiza `turn_offset`, `lineState`, `last_line_error` y `*target_setpoint`.
 * @see PID_Calcular
 * @see PIDTask
 * @ingroup group_control
 */
void ControlSeguimiento(int32_t left_ir, int32_t center_ir, int32_t right_ir, int32_t *target_setpoint) {
	static uint16_t line_lost_debounce_count = 0;
	static int32_t last_turn_offset = 0;
	static int8_t cross_direction = 1;

	uint8_t ir1_active = (left_ir < IR_WHITE);
	uint8_t ir3_active = (center_ir < IR_WHITE);
	uint8_t ir5_active = (right_ir < IR_WHITE);
	uint8_t active_count = ir1_active + ir3_active + ir5_active;

	switch (lineState) {
	case LINE_SEARCHING:
		*target_setpoint = attack_setpoint;
		if (ir3_active) {
			lineState = LINE_FOLLOWING;
		} else {
			int16_t active_turn = (custom_turn > 50) ? custom_turn : 350;
			turn_offset = -active_turn;
			last_turn_offset = turn_offset;
		}
		break;

	case LINE_FOLLOWING:
		// Detección de los 3 sensores en negro: activo EXCLUSIVAMENTE en modo seguimiento de línea (no en esquivar)
		if (robotMode == STATE_LINE_FOLLOWING && active_count == 3 && ir1_active && ir3_active && ir5_active) {
			// Sentido de giro aleatorio (+1: derecha, -1: izquierda) usando el jitter del SysTick
			cross_direction = ((SysTick->VAL & 1) != 0) ? 1 : -1;
			lineState = LINE_CROSS;
			break;
		}

		if (active_count == 0) {
			line_lost_debounce_count++;
			if (line_lost_debounce_count >= 6) {
				line_lost_debounce_count = 0;
				line_lost_timer = 0;
				line_lost_phase = LINE_LOST_ROT_90;
				line_lost_yaw = 0;
				search_direction = (last_line_error >= 0) ? -1 : 1;
				lineState = LINE_LOST;
				break;
			}
			error_linea = last_line_error;
			turn_offset = last_turn_offset;
		} else {
			line_lost_debounce_count = 0;

			error_linea = ((-(1000 * left_ir) + (1000 * right_ir)) / sum_sensors) / 10;
			abs_error = (error_linea > 0) ? error_linea : -error_linea;

			int32_t linear_term = (Kp_line * error_linea) / 100;
			int32_t quad_term = (Kq_line * error_linea * abs_error) / SCALE_LINE;

			turn_offset = linear_term + quad_term;
			if (turn_offset > turn_limit)        turn_offset = turn_limit;
			else if (turn_offset < -turn_limit)  turn_offset = -turn_limit;

			last_turn_offset = turn_offset;
		}
		last_line_error = error_linea;

		*target_setpoint = attack_setpoint;
		break;

	case LINE_LOST:
		// Verificar constantemente en cada ciclo si CUALQUIERA de los 3 sensores toca la línea
		if (active_count > 0) {
			lineState = LINE_FOLLOWING;
			break;
		}

		// Integración del ángulo de yaw usando el giroscopio Z (gz) y su offset
		{
			int32_t gz_cal = gz - gz_offset;
			line_lost_yaw += ((int64_t)gz_cal * DT_US) / 131000LL;
		}

		switch (line_lost_phase) {
		case LINE_LOST_ROT_90:
			*target_setpoint = attack_setpoint; // Mantener setpoint mientras rota 90°
			// Rotar 90 grados hacia el lado donde se perdió la línea a menor velocidad
			turn_offset = (search_direction > 0) ? LINE_LOST_TURN_SPEED : -LINE_LOST_TURN_SPEED;
			{
				int32_t abs_yaw = (line_lost_yaw < 0) ? -line_lost_yaw : line_lost_yaw;
				if (abs_yaw >= 90000) { // 90 grados = 90,000 milígrados
					line_lost_yaw = 0;
					line_lost_timer = 0;
					line_lost_phase = LINE_LOST_WAIT_2S;
				}
			}
			break;

		case LINE_LOST_WAIT_2S:
			*target_setpoint = attack_setpoint; // Mantener el mismo ángulo de rotación durante la espera de 2s
			turn_offset = 0;
			line_lost_yaw = 0;       // Limpiar acumulación de yaw mientras está en espera
			line_lost_timer += DT_MS;
			if (line_lost_timer >= 2000) { // Esperar 2000 ms (2 segundos)
				line_lost_timer = 0;
				line_lost_yaw = 0;
				line_lost_phase = LINE_LOST_ROT_180;
			}
			break;

		case LINE_LOST_ROT_180:
			*target_setpoint = attack_setpoint; // Mantener setpoint mientras rota 180°
			// Rotar 180 grados en sentido opuesto a menor velocidad
			turn_offset = (search_direction > 0) ? -LINE_LOST_TURN_SPEED : LINE_LOST_TURN_SPEED;
			{
				int32_t abs_yaw = (line_lost_yaw < 0) ? -line_lost_yaw : line_lost_yaw;
				if (abs_yaw >= 180000) { // 180 grados = 180,000 milígrados
					line_lost_yaw = 0;
					line_lost_timer = 0;
					line_lost_phase = LINE_LOST_STOPPED;
				}
			}
			break;

		case LINE_LOST_STOPPED:
		default:
			// Frenado estático estricto con setpoint -250 para sostenerse en mesa inclinada
			*target_setpoint = -250;
			turn_offset = 0;
			break;
		}
		break;

	case LINE_CROSS:
		*target_setpoint = 350; // Setpoint erguido (+3.50°) para rotar sobre su propio eje

		// Salida: rotar hasta que quede SOLAMENTE UN sensor midiendo
		if (active_count == 1) {
			turn_offset = 0;
			lineState = LINE_FOLLOWING;
			break;
		}

		// Rotar sobre su eje en sentido aleatorio
		turn_offset = cross_direction * 350;
		break;

	default:
		lineState = LINE_SEARCHING;
		break;
	}
}

/**
 * @brief Control supervisor de equilibrio estático longitudinal con detección por histéresis de caída inminente.
 * @details Gestiona el modo estacionario de balanceo puro (`STATE_SWING` y `STATE_3D_SCREEN`):
 *          1. **Neutralización de Guiñada**: Cancela cualquier par de rotación (`turn_offset = 0`) para aislar el lazo de balance longitudinal.
 *          2. **Consigna Nominal**: Carga la consigna estática calibrada `setpoint` como referencia de equilibrio erguido.
 *          3. **Detector Schmitt Trigger de Caída Frontal**: Monitorea el ángulo pitch estimado `current_angle`:
 *             - Si la inclinación frontal supera los $-15.00^\circ$ (`current_angle < -ANG15`), activa `forwards_recovery_active = 1`.
 *             - Aplica una histéresis de $5.00^\circ$, desactivando la recuperación únicamente cuando el robot retorna a $\ge -10.00^\circ$ (`current_angle >= -ANG10`).
 *          4. **Clamping de Consigna en Caída**: Cuando la bandera de recuperación está activa, fuerza `*target_setpoint = 0` para suprimir la consigna de avance y permitir que el par reactivo levante el chasis.
 *          5. **Accionamiento en Cascada**: El lazo rápido `PID_Calcular()` detecta esta bandera e inyecta un pulso directo de tracción delantera al 30% de ciclo de trabajo (`pwm = +3000`) reiniciando el integrador a cero.
 *
 * \startuml
 * skinparam backgroundColor white
 * skinparam arrowColor #2C3E50
 * skinparam activity {
 *   BackgroundColor #F4F6F7
 *   BorderColor #34495E
 *   FontColor #2C3E50
 * }
 * skinparam partition {
 *   BackgroundColor #EAEDED
 *   BorderColor #7F8C8D
 *   FontColor #1A5276
 * }
 *
 * start
 * partition "Filtro / Sensores (Estimación de Estado)" {
 *   :Lectura de ángulo de inclinación: **current_angle**\n(Estimación por Filtro Complementario IMU MPU-6050);\nLectura de consigna base de equilibrio: **setpoint**;
 * }
 *
 * partition "Controlador de Balanceo Estático y Supervisor de Caída" {
 *   :Anular esfuerzo de guiñada:\n**turn_offset = 0**;
 *   :Asignación de consigna nominal:\n***target_setpoint = setpoint**;
 *
 *   if (¿Inclinación frontal crítica?\ncurrent_angle < -ANG15 [-15.00°]) then (Sí: Caída Delantera Inminente)
 *     :Activar bandera de recuperación de emergencia:\n**forwards_recovery_active = 1**;
 *   elseif (¿Retorno seguro a zona de estabilidad?\ncurrent_angle >= -ANG10 [-10.00°]) then (Sí: Histéresis de Seguridad)
 *     :Desactivar bandera de recuperación:\n**forwards_recovery_active = 0**;
 *   else (Banda de Histéresis [-15° a -10°])
 *     :Mantener estado previo de **forwards_recovery_active**;
 *   endif
 *
 *   if (¿forwards_recovery_active == 1?) then (Sí)
 *     :Clamping de Consigna Anti-Picada:\n***target_setpoint = 0**;
 *     note right: Anula la consigna inclinada hacia adelante\npara permitir que el par motor levante el chasis.
 *   endif
 * }
 *
 * partition "Actuación (PWM) y Despacho a PID" {
 *   :Transferencia de consigna ***target_setpoint** y bandera de recuperación;
 *   :Despacho al lazo rápido **PID_Calcular(target_setpoint)**:;
 *   if (¿forwards_recovery_active == 1?) then (Bypass de Emergencia)
 *     :Inyección de pulso directo hacia adelante:\n**pwm_left = +3000**, **pwm_right = +3000**\nReset acumulador: **integral = 0**, **last_error = 0**;
 *   else (Lazo Normal)
 *     :Cálculo PID longitudinal y modulación de PWM;
 *   endif
 *   :Mapeo final a registros de comparación de TIM1;
 * }
 * stop
 * \enduml
 *
 * @param[out] target_setpoint Puntero a la consigna de inclinación longitudinal modulada por el supervisor.
 * @pre IMU calibrada, filtro complementario activo en `PIDTask()`.
 * @post Modifica `*target_setpoint`, `turn_offset` y `forwards_recovery_active`.
 * @see PID_Calcular
 * @see PIDTask
 * @ingroup group_control
 */
void Control_Balanceo(int32_t *target_setpoint) {
	turn_offset = 0;
	*target_setpoint = setpoint;

	// Control de prevención de caída delantera específico en Modo Swing
	if (current_angle < -ANG15) { // Si la inclinación delantera en Swing excede -15.00° (-1500)
		forwards_recovery_active = 1;
	} else if (current_angle >= -ANG10) { // Histéresis: se desactiva al volver a -10.00° (-1000)
		forwards_recovery_active = 0;
	}

	if (forwards_recovery_active) {
		*target_setpoint = 0; // Elimina el setpoint de avance en picada
	}
}

/**
 * @brief Gestiona el modo de control remoto interactivo vía comandos de Joystick.
 * @param[out] target_setpoint Puntero a la consigna de inclinación longitudinal del robot.
 * @ingroup group_control
 */
void Control_Joystick(int32_t *target_setpoint) {
	*target_setpoint = setpoint;
	if (joystick_turn_timer > 0) {
		if (joystick_turn_timer >= DT_MS) {
			joystick_turn_timer -= DT_MS;
		} else {
			joystick_turn_timer = 0;
		}
		turn_offset = joystick_turn_offset;
	} else {
		turn_offset = 0;
		joystick_turn_offset = 0;
	}
}

/**
 * @brief Algoritmo supervisor de evasión de obstáculos con frenado longitudinal en 3 etapas y control PD multivariable de pared.
 * @details Implementa la máquina de estados jerárquica y el control continuo para esquivado autónomo:
 *          1. **Filtro y Fusión Sensorial**:
 *             - Detección de proximidad frontal con sensor de rango infrarrojo `cal_ir6`.
 *             - Enrutamiento cinemático de sensores laterales según sentido de giro (`dodge_direction`):
 *               - Lateral 90° (`cal_ir0` / `cal_ir2`): distancia perpendicular al obstáculo.
 *               - Diagonal 45° (`cal_ir7` / `cal_ir4`): anticipación predictiva de curvatura.
 *               - Frontal (`cal_ir6`): protección activa anticolisión.
 *             - Odometría inercial de guiñada: integración numérica del giróscopo Z calibrado $\Delta \psi = \frac{(g_z - g_{z\_offset}) \cdot \Delta t}{131000}$.
 *          2. **Perfil Longitudinal de Frenado (`DODGE_STANDBY`)**:
 *             - Etapa 1 ($0 \le t < 250$ ms): Frenado reactivo brusco con pitch hacia atrás a $+10.00^\circ$.
 *             - Etapa 2 ($250 \le t < 750$ ms): Estabilización de inercia a $+3.50^\circ$.
 *             - Etapa 3 ($750 \le t < 1500$ ms): Inclinación preparatoria hacia adelante a $-2.50^\circ$.
 *          3. **Rotación Guiada por Giroscopio Z (`DODGE_ROTATING`)**:
 *             - Giro pivot de 90° con supervisión de estabilidad de balance: si el error pitch excede $4.50^\circ$ ($|error| > 450$), anula el par de giro (`turn_offset = 0`, `integral = 0`) para priorizar la estabilidad dinámica vertical sobre el giro.
 *          4. **Lazo de Control PD Multivariable de Pared (`DODGE_WALL_FOLLOWING`)**:
 *             - Consigna de avance de $-14.00^\circ$ para mantener velocidad constante de crucero.
 *             - Ley de control PD acoplada:
 *               $$u_{PD} = \frac{(s_{90} - 800) \cdot K_{p\_pared} + (s_{45} - 800) \cdot K_{d\_anticipo} + \max(s_{front} - 500, 0) \cdot K_{d\_frontal}}{100}$$
 *             - Saturación de salida (clamping) en $\pm 500$ para impedir pérdida de adherencia.
 *          5. **Re-enganche a la Pista (`DODGE_RETURN_ROTATING`)**:
 *             - Filtro de despeje previo de línea y re-alineación temporizada sobre la trayectoria óptica.
 *
 * \startuml
 * skinparam backgroundColor white
 * skinparam arrowColor #2C3E50
 * skinparam activity {
 *   BackgroundColor #F4F6F7
 *   BorderColor #34495E
 *   FontColor #2C3E50
 * }
 * skinparam partition {
 *   BackgroundColor #EAEDED
 *   BorderColor #7F8C8D
 *   FontColor #1A5276
 * }
 *
 * start
 * partition "Filtro / Sensores (Adquisición y Fusión)" {
 *   :Lectura de reflectancia de línea: **left_ir**, **center_ir**, **right_ir**;
 *   :Lectura de proximidad frontal: **cal_ir6**;
 *   :Selección de sensores de pared según **dodge_direction**:\n**sensor_90** (cal_ir0 / cal_ir2: Lateral 90°)\n**sensor_45** (cal_ir7 / cal_ir4: Diagonal 45° Anticipación)\n**sensor_front** = cal_ir6 (Protección Frontal);
 *   :Filtro e integración giroscópica de guiñada:\n**gz_cal = gz - gz_offset**\n**dodge_yaw += (gz_cal * DT_US) / 131000**;
 * }
 *
 * partition "Controlador de Evasión (MEF y Control PD Multivariable)" {
 *   switch (dodgeState)
 *   case (DODGE_LINE_FOLLOWING)
 *     :Ejecuta **ControlSeguimiento()**;
 *     :Acumula temporizador: **dodge_timer += DT_MS**;
 *     if (¿cal_ir6 >= 500 && dodge_timer >= 1000?) then (Obstáculo frontal detectado)
 *       :Inicializar variables de maniobra:\n**turn_offset = 0**, **dodge_timer = 0**\n**standby_next_state = DODGE_ROTATING**\n**dodgeState = DODGE_STANDBY**;
 *     endif
 *
 *   case (DODGE_STANDBY)
 *     :Frenado longitudinal en 3 etapas sin guiñada (**turn_offset = 0**):;
 *     if (dodge_timer < 250 ms) then (Frenado brusco)
 *       :Consigna de frenado:\n***target_setpoint = +1000** (+10.0° pitch);
 *     elseif (dodge_timer < 750 ms) then (Estabilización)
 *       :Consigna intermedia:\n***target_setpoint = +350** (+3.5° pitch);
 *     elseif (dodge_timer < 1500 ms) then (Inclinación frontal previa)
 *       :Consigna preparatoria:\n***target_setpoint = -250** (-2.5° pitch);
 *     else (Fin 1.5s)
 *       :Transición a **standby_next_state** (DODGE_ROTATING)\n**dodge_timer = 0**, **dodge_yaw = 0**;
 *     endif
 *
 *   case (DODGE_ROTATING)
 *     :Consigna de rotación con leve avance: ***target_setpoint = -250** (-2.5° pitch);
 *     :Atenuación de memoria inercial: **integral = (integral * 7) / 10**;
 *     if (¿abs_yaw >= 90000 milígrados [90°]?) then (Giro completado)
 *       :Detener giro y pasar a seguimiento de pared:\n**turn_offset = 0**, **dodge_yaw = 0**\n**dodgeState = DODGE_WALL_FOLLOWING**;
 *     else (En rotación)
 *       if (¿|error_pitch| > 450 milígrados [4.5°]?) then (Prioridad de Balance)
 *         :Cancelar par de giro para prevenir caída:\n**turn_offset = 0**, **integral = 0**;
 *       else (Estable)
 *         :Aplicar par de rotación firme:\n**turn_offset = 350 * dodge_direction**;
 *       endif
 *     endif
 *
 *   case (DODGE_WALL_FOLLOWING)
 *     :Consigna de avance continuo crucero: ***target_setpoint = -1400** (-14.0°);
 *     :1. Error de distancia lateral 90°:\n**error_distancia = sensor_90 - 800**;
 *     :2. Error de anticipación diagonal 45°:\n**error_anticipo = sensor_45 - 800**;
 *     :3. Error de protección frontal:\n**error_frontal = max(sensor_front - 500, 0)**;
 *     
 *     :Lazo de Control PD Multivariable:\n**u_PD = ((error_distancia * Kp_pared) + (error_anticipo * Kd_anticipo) + (error_frontal * Kd_frontal)) / 100**;
 *     :Cálculo de esfuerzo de guiñada:\n**turn_offset = u_PD * dodge_direction**;
 *     
 *     :Limitación de esfuerzo (Clamping):\n**turn_offset = clamp(turn_offset, -500, +500)**;
 *
 *     if (¿t >= 4000 ms && línea despejada && línea detectada?) then (Sí)
 *       :Transición de reenganche inmediato:\n***target_setpoint = -1400**, **turn_offset** activo\n**dodgeState = DODGE_RETURN_ROTATING**;
 *     endif
 *
 *   case (DODGE_RETURN_ROTATING)
 *     :Rotación directa sin espera manteniendo avance: ***target_setpoint = -1400**;
 *     :Rotación angular hacia la línea con **turn_offset** según **dodge_bias_mult**;
 *     if (¿Fin de tiempo de reingreso?) then (Sí)
 *       :Retorno a **DODGE_LINE_FOLLOWING** (o búsqueda **LINE_LOST**);
 *     endif
 *   endswitch
 * }
 *
 * partition "Actuación (PWM) y Despacho a PID" {
 *   :Exportación de consignas dinámicas:\n***target_setpoint** y **turn_offset**;
 *   :Despacho al lazo interno **PID_Calcular(target_setpoint)**\npara modulación y accionamiento PWM en puente H;
 * }
 * stop
 * \enduml
 *
 * @param[in] left_ir Lectura analógica normalizada del sensor de línea izquierdo.
 * @param[in] center_ir Lectura analógica normalizada del sensor de línea central.
 * @param[in] right_ir Lectura analógica normalizada del sensor de línea derecho.
 * @param[out] target_setpoint Puntero a la consigna de inclinación longitudinal del robot.
 * @pre Modo `STATE_DODGE` activo, IMU y sensores analógicos calibrados.
 * @post Actualiza `target_setpoint`, `turn_offset`, `dodgeState`, `dodge_timer` y `dodge_yaw`.
 * @see _eDodgeSubState
 * @see ControlSeguimiento
 * @see PID_Calcular
 * @see PIDTask
 * @ingroup group_control
 */
void Control_Esquivar(int32_t left_ir, int32_t center_ir, int32_t right_ir, int32_t *target_setpoint) {
	// Variables estáticas locales para el control de esquivado y pared
	static uint8_t line_cleared = 0;
	static _eDodgeSubState standby_next_state = DODGE_ROTATING;

	switch (dodgeState) {
	case DODGE_LINE_FOLLOWING:
		// 1. Seguimiento de línea normal continuo gobernado por ControlSeguimiento
		ControlSeguimiento(left_ir, center_ir, right_ir, target_setpoint);

		dodge_timer += DT_MS;

		// Cuando se detecta el obstáculo a la distancia adecuada
		if (cal_ir6 >= 500 && dodge_timer >= 1000) {
			dodge_line_rotation_done = 0; // Habilitar la rotación para el nuevo ciclo de esquive
			standby_next_state = DODGE_ROTATING;
			turn_offset = 0;
			dodge_timer = 0;
			dodgeState = DODGE_STANDBY;
		}
		break;

	case DODGE_STANDBY:
		// Standby unificado de frenado en 3 etapas: 250ms (+1000), 500ms (+350), 750ms (-250)
		dodge_timer += DT_MS;
		turn_offset = 0; // Frenado recto y balanceo quieto en el lugar

		if (dodge_timer < 250) {
			*target_setpoint = 1000; // Etapa 1: Frenado brusco (+10.00°) por 250ms
		} else if (dodge_timer < 750) {
			*target_setpoint = 350;  // Etapa 2: Estabilización (+3.50°) por 500ms (hasta 750ms)
		} else if (dodge_timer < 1500) {
			*target_setpoint = -250; // Etapa 3: Inclinación frontal leve (-2.50°) por 750ms (hasta 1500ms)
		} else {
			// Finalizados los 1.5s de standby: pasar al siguiente estado configurado
			dodge_timer = 0;
			dodge_yaw = 0;
			if (standby_next_state == DODGE_LINE_FOLLOWING) {
				lineState = LINE_FOLLOWING;
			}
			dodgeState = standby_next_state;
		}
		break;

	case DODGE_ROTATING: {
		// Rotación directa de 90° con giroscopio (la espera y frenado previo se realizaron en DODGE_STANDBY)
		*target_setpoint = -250; // Inclinación suave frontal (-2.50°) durante la rotación
		integral = (integral * 7) / 10; // Atenuación de memoria inercial

		int32_t gz_calibrated = gz - gz_offset;
		dodge_yaw += ((int64_t)gz_calibrated * DT_US) / 131000LL;
		int32_t abs_yaw = (dodge_yaw < 0) ? -dodge_yaw : dodge_yaw;

		if (abs_yaw >= 90000) { // Fin de giro (90°)
			turn_offset = 0;
			dodge_yaw = 0;
			dodge_timer = 0;
			line_cleared = 0; // Resetear validación de liberación de línea
			dodge_line_rotation_done = 0;
			dodgeState = DODGE_WALL_FOLLOWING;
		} else {
			// Prioridad de balanceo dinámica con par de rotación firme
			int32_t base_turn = 350;
			int32_t abs_error = (error < 0) ? -error : error;

			if (abs_error > 450) {
				turn_offset = 0;
				integral = 0;
			} else {
				turn_offset = base_turn * dodge_direction;
			}
		}
		break;
	}

	case DODGE_WALL_FOLLOWING: {
		*target_setpoint = -1400; // Setpoint de avance de -14.00° para seguimiento de pared

		// Sensores de pared: 90° para control principal de distancia, 45° para anticipación anticipada, IR6 para protección frontal
		int16_t sensor_90 = (dodge_direction == 1) ? cal_ir0 : cal_ir2;
		int16_t sensor_45 = (dodge_direction == 1) ? cal_ir7 : cal_ir4;
		int16_t sensor_front = cal_ir6;

		// Distancia objetivo principal a la pared
		int16_t wall_target = 800;

		// 1. Error de distancia del sensor lateral 90° (Control Principal)
		int32_t error_distancia = sensor_90 - wall_target;

		// 2. Error de anticipación del sensor diagonal 45° (Ayuda anticipada)
		int32_t error_anticipo = sensor_45 - wall_target;

		// 3. Error de protección frontal del sensor IR6 (Anticipación frontal)
		int32_t error_frontal = (sensor_front > 500) ? (sensor_front - 500) : 0;

		// Seguimiento PD normal de pared continuo con protección frontal
		int32_t calculo_pd = ((error_distancia * Kp_pared) + (error_anticipo * Kd_anticipo) + (error_frontal * Kd_frontal)) / 100;

		turn_offset = calculo_pd * dodge_direction;

		int16_t wall_turn_limit = 500;

		if (turn_offset > wall_turn_limit)        turn_offset = wall_turn_limit;
		else if (turn_offset < -wall_turn_limit)  turn_offset = -wall_turn_limit;

		// Medida de seguridad: Validar que el robot primero haya salido completamente de la línea previa
		if (center_ir >= IR_DODGE_LINE_THRESHOLD && left_ir >= IR_DODGE_LINE_THRESHOLD && right_ir >= IR_DODGE_LINE_THRESHOLD) {
			line_cleared = 1;
		}

		dodge_timer += DT_MS;
		// Re-enganche a la línea tras un tiempo mínimo de 4.0 segundos de avance siguiendo la pared
		if (dodge_timer >= 4000 && line_cleared && !dodge_line_rotation_done) {
			uint8_t d_ir1 = (left_ir < IR_DODGE_LINE_THRESHOLD);
			uint8_t d_ir3 = (center_ir < IR_DODGE_LINE_THRESHOLD);
			uint8_t d_ir5 = (right_ir < IR_DODGE_LINE_THRESHOLD);
			uint8_t d_active_count = d_ir1 + d_ir3 + d_ir5;

			if (d_active_count > 0) {
				// Guardar el estado inicial del IR que detectó la línea
				int32_t sum_sensors = left_ir + center_ir + right_ir;
				if (sum_sensors > 0) {
					last_line_error = ((-(1000 * left_ir) + (1000 * right_ir)) / sum_sensors) / 10;
				} else {
					if (d_ir1)      last_line_error = 100;
					else if (d_ir5) last_line_error = -100;
					else            last_line_error = 0;
				}
				dodge_line_rotation_done = 1; // La rotación solo se ejecuta 1 vez por esquive
				*target_setpoint = -1400;     // Mantener setpoint de avance (-14.00°)
				int32_t rot_turn = dodge_direction * dodge_bias_mult;
				int32_t max_turn = (turn_limit > 3500) ? turn_limit : 3500;
				if (rot_turn > max_turn)        rot_turn = max_turn;
				else if (rot_turn < -max_turn)  rot_turn = -max_turn;
				turn_offset = rot_turn;       // Rotación directa inmediata sin espera
				dodge_yaw = 0;
				dodge_timer = 0;
				dodgeState = DODGE_RETURN_ROTATING;
			}
		}
		break;
	}

	case DODGE_RETURN_ROTATING: {
		// Rotación directa buscando la línea manteniendo setpoint de avance (-1400) sin espera ni frenado previo
		dodge_timer += DT_MS;

		uint8_t d_ir1 = (left_ir < IR_DODGE_LINE_THRESHOLD);
		uint8_t d_ir3 = (center_ir < IR_DODGE_LINE_THRESHOLD);
		uint8_t d_ir5 = (right_ir < IR_DODGE_LINE_THRESHOLD);
		uint8_t d_active_count = d_ir1 + d_ir3 + d_ir5;

		// Guardar continuamente el estado del último IR que vio la línea en la rotación:
		if (d_active_count > 0) {
			int32_t sum_sensors = left_ir + center_ir + right_ir;
			if (sum_sensors > 0) {
				last_line_error = ((-(1000 * left_ir) + (1000 * right_ir)) / sum_sensors) / 10;
			} else {
				if (d_ir1)      last_line_error = 100;
				else if (d_ir5) last_line_error = -100;
				else            last_line_error = 0;
			}
		}

		if (dodge_timer < (uint32_t)dodge_bias_time) {
			// Rotación directa continua con setpoint -1400 buscando la línea
			*target_setpoint = -1400;
			int32_t rot_turn = dodge_direction * dodge_bias_mult;
			int32_t max_turn = (turn_limit > 3500) ? turn_limit : 3500;
			if (rot_turn > max_turn)        rot_turn = max_turn;
			else if (rot_turn < -max_turn)  rot_turn = -max_turn;
			turn_offset = rot_turn;
		} else {
			// Fin del tiempo configurado de rotación: pasar a seguimiento o activar el sistema de búsqueda
			turn_offset = 0;
			dodge_yaw = 0;
			dodge_timer = 0;
			dodgeState = DODGE_LINE_FOLLOWING;

			if (d_active_count > 0) {
				// Si al finalizar la rotación algún sensor está sobre la línea, seguirla directo
				lineState = LINE_FOLLOWING;
			} else {
				// Si no está sobre la línea, activar el sistema de búsqueda hacia el último IR que la vio
				search_direction = (last_line_error >= 0) ? -1 : 1;
				line_lost_phase = LINE_LOST_ROT_90;
				line_lost_timer = 0;
				line_lost_yaw = 0;
				lineState = LINE_LOST;
			}
		}
		break;
	}

	default:
		dodgeState = DODGE_WALL_FOLLOWING;
		break;
	}
}

/**
 * @brief Lazo PID central de balance longitudinal a 200 Hz con Derivative-on-Measurement, Anti-Windup con Fuga, Mezclador Diferencial y PWM.
 * @details Ejecuta el cómputo crítico en tiempo real del péndulo invertido cada 5 ms ($DT\_US = 5000\ \mu\text{s}$):
 *          1. **Filtro y Estado Angular**: Adquiere el ángulo pitch `current_angle` estimado por el filtro complementario inercial (giroscopio MPU-6050 + acelerómetro).
 *          2. **Cálculo del Error**: $e = target\_setpoint - current\_angle$.
 *          3. **Acción Proporcional (P)**: $P = K_{p\_stable} \cdot e$.
 *          4. **Acción Derivativa sobre Medición (D)**:
 *             - Implica derivar la medición angular en lugar del error: $\frac{d}{dt}(last\_angle - current\_angle)$ para erradicar por completo el choque o "Derivative Kick" ante cambios escalón de consigna:
 *               $$D = K_{d\_stable} \cdot \left(\frac{(last\_angle - current\_angle) \cdot 10^6}{DT\_US}\right)$$
 *          5. **Acción Integral con Clamping y Decaimiento Exponencial (Anti-Windup Leaky)**:
 *             - En zona lineal ($|e| < 1.50^\circ$ [150 milígrados]), acumula: $integral \mathrel{+}= \frac{e \cdot DT\_US}{1000}$ con saturación estricta a $\pm(20 \cdot ANG20)$ ($\pm 40.0^\circ$ equiv).
 *             - Fuera de la zona lineal ($|e| \ge 1.50^\circ$), aplica una fuga exponencial del 20% por ciclo ($integral = \frac{integral \cdot 8}{10}$) para atenuar instantáneamente la memoria inercial y prevenir sobrepasos descontrolados al estabilizarse.
 *             - Término integral: $I = \frac{K_{i\_stable} \cdot integral}{1000}$.
 *          6. **Suma Total del Esfuerzo Longitudinal**:
 *             $$output = \frac{P + I + D}{10000}$$
 *          7. **Mezclador de Tracción y Guiñada**:
 *             - En giros pivot (búsqueda de línea 90°/180° o joystick), ejecuta cancelación algebraica de la rueda interior ($turn\_offset = \pm \max(output, 150)$) frenándola a 0 PWM mientras la rueda exterior entrega el doble de tracción.
 *             - En translación normal, suma diferencialmente el par de guiñada: $pwm_L = base_L - turn\_offset$ y $pwm_R = base_R + turn\_offset$.
 *          8. **Compensación de Zona Muerta (Deadband)**: Inyecta el par mínimo de arranque estático ($\pm minPWM \pm offset$) acorde al sentido de tracción.
 *          9. **Protección Absoluta y Clamping**:
 *             - Kill Switch de caída: si $|current\_angle| > 45.0^\circ$, desenergiza inmediatamente los motores ($PWM = 0$, $integral = 0$, $speed = 0$).
 *             - Saturación final en $[ -maxPWM, +maxPWM ]$ ($\pm 4095$).
 *          10. **Mapeo al Hardware**: Transfiere las magnitudes a los registros del temporizador TIM1 para modulación H-Bridge (`rPulse4`/`lPulse3` izquierda, `rPulse2`/`lPulse1` derecha).
 *
 * \startuml
 * skinparam backgroundColor white
 * skinparam arrowColor #2C3E50
 * skinparam activity {
 *   BackgroundColor #F4F6F7
 *   BorderColor #34495E
 *   FontColor #2C3E50
 * }
 * skinparam partition {
 *   BackgroundColor #EAEDED
 *   BorderColor #7F8C8D
 *   FontColor #1A5276
 * }
 *
 * start
 * partition "Filtro / Sensores (Estimación de Estado)" {
 *   :Entrada de consigna: **target_setpoint**;
 *   :Lectura de ángulo de inclinación: **current_angle**\n(Estimado por Filtro Complementario IMU MPU-6050);
 *   :Recuperación de estados previos:\n**last_angle**, **last_error**, **integral**;
 * }
 *
 * partition "Controlador PID (Balance Longitudinal)" {
 *   :Cálculo del Error angular:\n**error = target_setpoint - current_angle**;
 *   
 *   :Cálculo del Término Proporcional (P):\n**P_term = Kp_stable * error**;
 *   
 *   :Cálculo de derivada sobre la medición (DoM):\n**d_angle = last_angle - current_angle**\n**derivative = (d_angle * 1000000) / DT_US**;
 *   :Cálculo del Término Derivativo (D):\n**D_term = Kd_stable * derivative**;
 *   note right: Derivada sobre la medición (current_angle)\nen lugar del error para evitar 'Derivative Kick'\nante cambios abruptos de consigna.
 *
 *   if (¿Error dentro de la banda lineal de balance?\n(|error| < 150 milígrados [±1.5°])) then (Sí: Integración Activa)
 *     :Integración discreta (Euler):\n**integral += (error * DT_US) / 1000**;
 *     if (¿integral > 20 * ANG20?) then (Sí)
 *       :Clamping Anti-Windup Superior:\n**integral = 20 * ANG20 (+40.0° equiv)**;
 *     elseif (¿integral < -20 * ANG20?) then (Sí)
 *       :Clamping Anti-Windup Inferior:\n**integral = -20 * ANG20 (-40.0° equiv)**;
 *     else (En rango)
 *       :Conservar acumulador integral;
 *     endif
 *   else (No: Transitorio / Perturbación Grande)
 *     :Fuga / Decaimiento Exponencial (Anti-Windup Leaky):\n**integral = (integral * 8) / 10**;
 *     note right: Reduce la memoria inercial un 20% por ciclo\npara impedir sobreoscilación al recuperar estabilidad.
 *   endif
 *   
 *   :Cálculo del Término Integral (I):\n**I_term = (Ki_stable * integral) / 1000**;
 *   
 *   :Suma Total de Esfuerzo de Control Longitudinal:\n**output = (P_term + I_term + D_term) / 10000**;
 *   
 *   :Actualización de variables de memoria de estado:\n**last_error = error**\n**last_angle = current_angle**;
 * }
 *
 * partition "Actuación (PWM) y Mezcla de Motores" {
 *   if (¿Rotación pivot sobre eje propio activa?\n(is_rotating_pivot)) then (Sí)
 *     if (¿Giro de 90° o 180° por pérdida de línea?) then (Sí)
 *       :Cancelación algebraica de rueda interna:\n**balance_effort = max(output, 150)**\n**turn_offset = ±balance_effort**;
 *       note right: La rueda interna se frena a PWM=0\nmientras la externa entrega el doble de par.
 *     endif
 *     :Cálculo de esfuerzos brutos rotacionales:\n**raw_L = output ± turn_offset**\n**raw_R = output ∓ turn_offset**;
 *     :Compensación de zona muerta rotacional:\n**pwm_left = raw_L ± rot_min_L ± offset_left**\n**pwm_right = raw_R ± rot_min_R ± offset_right**;
 *   else (No: Translación y Guiñada Diferencial)
 *     :Compensación de zona muerta longitudinal:\n**base_L = output ± minPWM_Left ± offset_left**\n**base_R = output ± minPWM_Right ± offset_right**;
 *     if (¿turn_offset != 0?) then (Sí: Mezcla Diferencial)
 *       :Inyección diferencial de giro:\n**pwm_left = base_L - turn_offset**\n**pwm_right = base_R + turn_offset**;
 *     else (No)
 *       :Tracción longitudinal simétrica:\n**pwm_left = base_L**\n**pwm_right = base_R**;
 *     endif
 *   endif
 *
 *   if (¿Recuperación directa activa?\n(forwards/backwards_recovery_active)) then (Sí)
 *     :Bypass de balanceo:\n**pwm_left = ±3000**, **pwm_right = ±3000**\n**integral = 0**, **last_error = 0**;
 *   endif
 *
 *   if (¿Caída inminente o modo inactivo?\n(|current_angle| > 45.0° o calib < 150)) then (Sí)
 *     :Desconexión de seguridad (Kill Switch):\n**pwm_left = 0**, **pwm_right = 0**\n**integral = 0**, **speed = 0**;
 *   endif
 *
 *   :Saturación final de salidas (Clamping):\n**pwm_left = clamp(pwm_left, -maxPWM, +maxPWM)**\n**pwm_right = clamp(pwm_right, -maxPWM, +maxPWM)**;
 *
 *   :Mapeo a registros del Temporizador TIM1 (Puente H):\n**rPulse4 / lPulse3 <= pwm_left**\n**rPulse2 / lPulse1 <= pwm_right**;
 * }
 * stop
 * \enduml
 *
 * @param[in] target_setpoint Consigna de ángulo deseada calculada por la tarea del modo activo (en milígrados).
 * @pre MPU-6050 y filtro complementario calibrados, período nominal de interrupción 5 ms.
 * @post Actualiza variables globales de telemetría (`error`, `integral`, `derivative`, `output`), y modula registros PWM de TIM1 (`rPulse4`, `lPulse3`, `rPulse2`, `lPulse1`).
 * @see PIDTask
 * @see Speed_IntegrationTask
 * @ingroup group_control
 */
void PID_Calcular(int32_t target_setpoint) {
	// =========================================================
	// --- 4. LAZO PID CENTRAL (Equilibrio Balancín Puro) ---
	// =========================================================
	error = target_setpoint - current_angle;
	// Derivada sobre la medición (Derivative on Measurement) para evitar el Derivative Kick
	derivative = (int32_t)(((int64_t)(last_angle - current_angle) * 1000000LL) / DT_US);

	if (error > -150 && error < 150) {
		integral += (error * (int32_t)DT_US) / 1000;
		if (integral > (ANG20 * 20))  integral = (ANG20 * 20);
		if (integral < -(ANG20 * 20)) integral = -(ANG20 * 20);
	} else {
		integral = (integral * 8) / 10;
	}

	output = (Kp_stable * error + (Ki_stable * integral) / 1000
			+ (Kd_stable * derivative)) / 10000;
	last_error = error;
	last_angle = current_angle;

	// =========================================================
	// --- 5. MEZCLA DE MOTORES (Mezclador de Velocidades y PID) ---
	// =========================================================
	int32_t pwm_left = 0;
	int32_t pwm_right = 0;

	// Detectamos si el robot está en búsqueda pivot sobre propio eje o en Modo Joystick con giro activo
	uint8_t is_rotating_pivot = ((robotMode == STATE_LINE_FOLLOWING && (lineState == LINE_LOST || lineState == LINE_SEARCHING || lineState == LINE_CROSS)) ||
	                             (robotMode == STATE_DODGE && (lineState == LINE_LOST || lineState == LINE_SEARCHING)) ||
	                             (robotMode == STATE_JOYSTICK && turn_offset != 0));

	if (is_rotating_pivot) {
		// --- ROTACIÓN PIVOT SOBRE PROPIO EJE (Búsqueda de línea y Modo Joystick) ---
		// Cancelación algebraica (Opción B): En rotación de 90° y 180° por pérdida de línea,
		// igualamos |turn_offset| al balance (output) para que la rueda interior se anule algebraicamente a 0 PWM
		// mientras la rueda exterior proporciona el doble de tracción de avance alrededor del pivote.
		if (lineState == LINE_LOST && (line_lost_phase == LINE_LOST_ROT_90 || line_lost_phase == LINE_LOST_ROT_180)) {
			int32_t balance_effort = (output > 150) ? output : 150;
			turn_offset = (turn_offset >= 0) ? balance_effort : -balance_effort;
		}

		int32_t raw_L, raw_R;
		if (robotMode == STATE_JOYSTICK) {
			// En Modo Joystick: polaridad directa (turn_offset > 0 gira a la izquierda: rueda izq atrás, rueda der adelante)
			raw_L = output - turn_offset;
			raw_R = output + turn_offset;
		} else {
			raw_L = output + turn_offset;
			raw_R = output - turn_offset;
		}

		uint16_t rot_min_L = (robotMode == STATE_JOYSTICK) ? 450 : ((lineState == LINE_LOST || lineState == LINE_CROSS) ? 770 : PWM_LRot);
		uint16_t rot_min_R = (robotMode == STATE_JOYSTICK) ? 450 : ((lineState == LINE_LOST || lineState == LINE_CROSS) ? 750 : PWM_RRot);

		if (raw_L > 0)       pwm_left = raw_L + rot_min_L + offset_left;
		else if (raw_L < 0)  pwm_left = raw_L - rot_min_L - offset_left;

		if (raw_R > 0)       pwm_right = raw_R + rot_min_R + offset_right;
		else if (raw_R < 0)  pwm_right = raw_R - rot_min_R - offset_right;

	} else {
		// --- MEZCLADOR GENERAL DE VELOCIDADES DE MOTORES (DODGE_ROTATING_90, DODGE_CORNER_ROTATING y Translación) ---
		// Vincula la rotación con el mezclador de velocidades para mantener la respuesta del PID de balanceo (output) alrededor del setpoint (+350)
		uint16_t active_minPWM_Left = minPWM_Left;
		uint16_t active_minPWM_Right = minPWM_Right;

		int32_t base_L = 0;
		int32_t base_R = 0;

		if (output > 0) {
			base_L = output + active_minPWM_Left + offset_left;
			base_R = output + active_minPWM_Right + offset_right;
		} else if (output < 0) {
			base_L = output - active_minPWM_Left - offset_left;
			base_R = output - active_minPWM_Right - offset_right;
		}

		// Mezcla diferencial uniforme de giro sin invertir dirección ni perder el setpoint PID
		if (turn_offset != 0) {
			if (base_L == 0) base_L = (turn_offset < 0) ? active_minPWM_Left : -active_minPWM_Left;
			if (base_R == 0) base_R = (turn_offset < 0) ? -active_minPWM_Right : active_minPWM_Right;

			pwm_left = base_L - turn_offset;
			pwm_right = base_R + turn_offset;
		} else {
			pwm_left = base_L;
			pwm_right = base_R;
		}
	}

	// --- IMPULSO DIRECTO DE RECUPERACIÓN (Bypass del Balanceo) ---
	if (backwards_recovery_active) {
		pwm_left = -3000;  // Impulso directo marcha atrás controlado (30% duty cycle)
		pwm_right = -3000;
		integral = 0;      // Resetear integrador para evitar descontrol al volver a balancear
		last_error = 0;
	} else if (forwards_recovery_active) {
		pwm_left = 3000;   // Impulso directo marcha adelante en modo SWING (30% duty cycle)
		pwm_right = 3000;
		integral = 0;      // Resetear integrador para evitar descontrol al volver a balancear
		last_error = 0;
	}

	// Integración de velocidad (estimación interna de telemetría extraída)
	Speed_IntegrationTask(DT_US);

	// =========================================================
	// --- 6. PROTECCIÓN ABSOLUTA Y APAGADO POR CAÍDA (> 45°) ---
	// =========================================================
	if (current_angle > ANG45 || current_angle < -ANG45) {
		pwm_left = 0;
		pwm_right = 0;
		integral = 0;
		speed = 0;
	}

	// Silenciado de motores durante calibración inicial (primeros 3 segundos)
	if (calib_cycle < 150) {
		pwm_left = 0;
		pwm_right = 0;
		integral = 0;
	}

	// Saturación final al límite de PWM
	if (pwm_left > (int32_t) maxPWM)  pwm_left = (int32_t) maxPWM;
	if (pwm_left < -(int32_t) maxPWM) pwm_left = -(int32_t) maxPWM;
	if (pwm_right > (int32_t) maxPWM)  pwm_right = (int32_t) maxPWM;
	if (pwm_right < -(int32_t) maxPWM) pwm_right = -(int32_t) maxPWM;

	if (robotMode != STATE_SWING && robotMode != STATE_LINE_FOLLOWING && robotMode != STATE_DODGE && robotMode != STATE_3D_SCREEN && robotMode != STATE_JOYSTICK) {
		pwm_left = 0;
		pwm_right = 0;
		integral = 0;
		speed = 0;
	}

	// =========================================================
	// --- 7. MAPEO AL HARDWARE ---
	// =========================================================
	if (pwm_left > 0) {
		rPulse4 = (uint16_t) pwm_left;
		lPulse3 = 0;
	} else {
		lPulse3 = (uint16_t) (-pwm_left);
		rPulse4 = 0;
	}

	if (pwm_right > 0) {
		rPulse2 = (uint16_t) pwm_right;
		lPulse1 = 0;
	} else {
		lPulse1 = (uint16_t) (-pwm_right);
		rPulse2 = 0;
	}
}

/**
 * @brief Tarea orquestadora central de control PID y MEF de modos operativos a 200 Hz (5 ms).
 * @details Implementa el lazo crítico de control en cascada del robot péndulo invertido:
 *          1. **Filtro Inercial**: Fusión sensorial complementaria de acelerómetro y giróscopo del MPU-6050.
 *          2. **Lectura Óptica**: Adquisición y normalización de reflectancia de línea IR y distancias por ADC1 DMA.
 *          3. **Despacho de MEF**: Ejecuta la lógica del modo activo (`Control_Balanceo`, `ControlSeguimiento`, `Control_Esquivar`, `Control_Joystick`).
 *          4. **Lazo PID Interno**: Controla el ángulo de inclinación longitudinal con Derivative-on-Measurement y Anti-Windup.
 *          5. **Lazo PI Externo**: Compensador de velocidad y esfuerzo motor para anular derivas de traslación.
 *          6. **Mezcla y Accionamiento**: Suma diferencial de guiñada (`turn_offset`), compensa zonas muertas y satura PWM a TIM1.
 *
 * \startuml
 * title Lazo de Control Crítico en Cascada 200 Hz (PIDTask)
 * start
 * if (¿Bandera RUN_PID activa (TIM2 5ms)?) then (Sí)
 *   :RUN_PID = FALSE;
 * else (No)
 *   stop
 * endif
 * :Normalizar sensores de línea y distancia ADC;
 * :Filtrar aceleraciones Ax, Az y velocidad Gy, Gz;
 * :Calcular ángulo estimado (Filtro Complementario);
 * :Acumular ángulo continuo de guiñada Yaw;
 *
 * partition "Control de Modo Operativo" {
 *   if (robotMode == STATE_LINE_FOLLOWING) then
 *     :ControlSeguimiento();
 *   elseif (robotMode == STATE_DODGE) then
 *     :Control_Esquivar();
 *   elseif (robotMode == STATE_JOYSTICK) then
 *     :Control_Joystick();
 *   else
 *     :Control_Balanceo();
 *   endif
 * }
 *
 * partition "Lazo PID Interno de Balance" {
 *   :error = target_setpoint - current_angle;
 *   :P = (error * Kp) / 100;
 *   :D = ((current_angle - last_angle) * Kd) / 100;
 *   :I = integral += (error * Ki);
 *   :pid_balance = P + I - D;
 * }
 *
 * partition "Lazo PI Externo y Generación PWM" {
 *   :Compensar esfuerzo motor (outer speed loop);
 *   :Mezclar giro: pwm_l = pid + turn_offset, pwm_r = pid - turn_offset;
 *   :Compensar umbrales mínimos de fricción (deadband);
 *   :Saturar a maxPWM (4095);
 *   :Actualizar registros de hardware TIM1->CCR1..CCR4;
 * }
 * stop
 * \enduml
 *
 * @pre Temporizador TIM2 habilitado generando interrupciones a 200 Hz que setean `RUN_PID = TRUE`.
 * @post Modifica registros PWM de TIM1, variables de estado `current_angle`, `error`, `integral`, `speed`.
 * @see mpu6050_Read
 * @see SetRobotMode
 * @see ControlSeguimiento
 * @see Control_Esquivar
 * @see Control_Joystick
 * @ingroup group_control
 */
void PIDTask(void) {
	if (RUN_PID == FALSE)
		return;
	RUN_PID = FALSE;

	// Variables estáticas persistentes de estado
	static _eRobotMode prev_pid_mode = STATE_STANDBY;

	if (robotMode != prev_pid_mode) {
		prev_pid_mode = robotMode;
		last_angle = current_angle; // Previene el Derivative Kick en el cambio de modo
		integral = 0;
		last_error = 0;
		backwards_recovery_active = 0;
		forwards_recovery_active = 0;
	}

	measured_dt_ms = DT_MS;

	// =========================================================
	// --- 1. LECTURA DIRECTA DE SENSORES DE LÍNEA Y DISTANCIA ---
	// =========================================================
	uint16_t raw_left   = adcDataTx[1];
	uint16_t raw_center = adcDataTx[3];
	uint16_t raw_right  = adcDataTx[5];

	// Normalización e interpolación directa por LUT
	uint16_t raw_sensors[4] = {
		raw_left,
		raw_center,
		raw_right,
		(uint16_t)((raw_left + raw_center + raw_right) / 3)
	};
	
	uint16_t norm_sensors[4];
	NormalizeLineSensors(raw_sensors, norm_sensors);

	int32_t left_ir   = norm_sensors[0];
	int32_t center_ir = norm_sensors[1];
	int32_t right_ir  = norm_sensors[2];

	cal_left_ir   = (int16_t)left_ir;
	cal_center_ir = (int16_t)center_ir;
	cal_right_ir  = (int16_t)right_ir;

	sum_sensors = left_ir + center_ir + right_ir;
	if (sum_sensors == 0)
		sum_sensors = 1;

	// Lectura directa de sensores de distancia superiores (esquivar objeto) sin normalización
	cal_ir0 = (int16_t)adcDataTx[0];
	cal_ir2 = (int16_t)adcDataTx[2];
	cal_ir4 = (int16_t)adcDataTx[4];
	cal_ir6 = (int16_t)adcDataTx[6];
	cal_ir7 = (int16_t)adcDataTx[7];

	// =========================================================
	// --- 2. FILTROS Y CÁLCULO DE ÁNGULO (IMU) ---
	// =========================================================
	if (ax_filt == 0 && az_filt == 0) {
		ax_filt = ax;
		az_filt = az;
	} else {
		ax_filt = (ax * 5 + ax_filt * 95) / 100;
		az_filt = (az * 5 + az_filt * 95) / 100;
	}

	if (az_filt > AZ_MIN_VALID || az_filt < -AZ_MIN_VALID) {
		acc_angle_hr = (int32_t) ax_filt * 35;
	}

	gyro_delta_hr = (-(int32_t) gy * 50) / 131;

	// Acumulación continua de Yaw
	int32_t gz_calibrated = gz - gz_offset;
	total_yaw_hr += ((int64_t)gz_calibrated * DT_US) / 131000LL;

	// Restauramos el filtro complementario puro. El acelerómetro DEBE
	// permanecer activo para corregir la deriva cruzada del giroscopio.
	current_angle_hr = (ALPHA_GYRO * (current_angle_hr + gyro_delta_hr)
			+ ALPHA_ACC * acc_angle_hr) / 1000;

	// Límites de seguridad extremos por software (+/- 90 grados)
	if (current_angle_hr > 900000)  current_angle_hr = 900000;
	if (current_angle_hr < -900000) current_angle_hr = -900000;

	current_angle = current_angle_hr / 100;

	// =========================================================
	// --- 3. MÁQUINA DE ESTADOS FINITA (MEF) DE MODOS OPERATIVOS ---
	// =========================================================
	int32_t target_setpoint = setpoint; 
	turn_offset = 0;
	forwards_recovery_active = 0;

	switch (robotMode) {
	case STATE_SWING:
	case STATE_3D_SCREEN:
		// Modo Balanceo Estático
		Control_Balanceo(&target_setpoint);
		break;

	case STATE_JOYSTICK:
		// Modo Control Remoto por Joystick
		Control_Joystick(&target_setpoint);
		break;

	case STATE_LINE_FOLLOWING:
		// Modo Seguidor de Línea
		ControlSeguimiento(left_ir, center_ir, right_ir, &target_setpoint);
		break;

	case STATE_DODGE:
		// Modo Evasión de Obstáculos y Seguimiento de Pared
		Control_Esquivar(left_ir, center_ir, right_ir, &target_setpoint);
		break;

	default:
		// Para otros modos o estados no definidos
		turn_offset = 0;
		target_setpoint = setpoint;
		break;
	}

	// --- CONTROL DE PREVENCIÓN DE CAÍDA TRASERA (Lógica del Gatillo) ---
	if (robotMode == STATE_SWING || robotMode == STATE_LINE_FOLLOWING || robotMode == STATE_3D_SCREEN) {
		if (current_angle > 0) { // Si la inclinación trasera supera 0.00° (0)
			backwards_recovery_active = 1;
		} else if (current_angle <= 0) {
			backwards_recovery_active = 0;
		}

		if (backwards_recovery_active) {
			target_setpoint = 0; // Desactivar avance agresivo instantáneamente
		}
	} else {
		backwards_recovery_active = 0;
	}

	// =========================================================
	// --- LAZO DE CONTROL EN CASCADA EXTERNO (Cada 40ms) ---
	// =========================================================
	if (robotMode == STATE_LINE_FOLLOWING || robotMode == STATE_DODGE) {
		static uint8_t slow_loop_counter = 0;
		slow_loop_counter++;
		if (slow_loop_counter >= 8) { // 8 * 5ms = 40ms
			slow_loop_counter = 0;

			// 1. Filtro pasa-bajos sobre la acción de control del motor (output)
			// Nota: 'output' no incluye el minPWM, por lo que su valor base es bajito.
			pwm_filtrado = (alfa_lpf * output + (100 - alfa_lpf) * pwm_filtrado) / 100;

			// 2. Error de esfuerzo (Velocidad deseada es attack_setpoint, esfuerzo real es pwm_filtrado)
			int32_t effort_error = (int32_t)attack_setpoint - pwm_filtrado;

			// 3. Acumulación con límites anti-windup estrictos (+/- 50000)
			integral_esfuerzo += effort_error;
			if (integral_esfuerzo > 50000)  integral_esfuerzo = 50000;
			if (integral_esfuerzo < -50000) integral_esfuerzo = -50000;

			// 4. Salida PI (escalada a x100 para coincidir con el setpoint)
			int32_t prop_term = (Kp_ext * effort_error) / 1000;
			int32_t int_term = (Ki_ext * integral_esfuerzo) / 10000;
			int32_t out_pi = prop_term + int_term;

			// Saturación del modificador a un límite de +/- 15.00 grados (1500 centésimas)
			if (out_pi > 1500)  out_pi = 1500;
			if (out_pi < -1500) out_pi = -1500;

			angulo_modificador_pi = (int16_t)out_pi;
		}

		// Sumar el modificador dinámico al target_setpoint
		target_setpoint += angulo_modificador_pi;
	} else {
		// Reiniciar variables si no estamos en modo seguidor de línea
		pwm_filtrado = 0;
		integral_esfuerzo = 0;
		angulo_modificador_pi = 0;
	}

	// Si se inyecta el pulso de destrabe manual (+1500), forzar la consigna de inclinación
	if (setpoint == 1500) {
		target_setpoint = 1500;
	}

	// =========================================================
	// --- 4. LAZO PID CENTRAL Y CONTROL DE MOTORES ---
	// =========================================================
	PID_Calcular(target_setpoint);
}


static uint16_t LUT_Interpolate(const uint16_t *x, const uint16_t *lut_y, uint16_t raw)
{
    if(raw <= x[0])
        return lut_y[0];

    // Se asume que LUT_SIZE está definido globalmente (ej. #define LUT_SIZE 16)
    if(raw >= x[LUT_SIZE - 1])
        return lut_y[LUT_SIZE - 1];

    for(int i = 0; i < LUT_SIZE - 1; i++)
    {
        if(raw >= x[i] && raw <= x[i + 1])
        {
            uint32_t diff_x = x[i + 1] - x[i];
            uint32_t diff_y = lut_y[i + 1] - lut_y[i];
            uint32_t offset_x = raw - x[i];

            // Aritmética entera pura.
            // Se suma (diff_x / 2) antes de la división para emular el redondeo (+0.5)
            uint32_t y = lut_y[i] + (((offset_x * diff_y) + (diff_x / 2)) / diff_x);

            return (uint16_t)y;
        }
    }

    return lut_y[LUT_SIZE - 1];
}

/**
 * @brief Normaliza los sensores de línea usando las Look-Up Tables.
 * @param adcDataTx_ptr Puntero al buffer DMA seguro (adcDataTx).
 * @param norm Puntero al array de valores normalizados de salida.
 */
void NormalizeLineSensors(const uint16_t *adcDataTx_ptr, uint16_t *norm)
{
    // Se añade lut_y como parámetro para independizar la función
    norm[0] = LUT_Interpolate(lut_l1_x, lut_l1_y, adcDataTx_ptr[0]);
    norm[1] = LUT_Interpolate(lut_l2_x, lut_l2_y, adcDataTx_ptr[1]);
    norm[2] = LUT_Interpolate(lut_l3_x, lut_l3_y, adcDataTx_ptr[2]);
    norm[3] = LUT_Interpolate(lut_l4_x, lut_l4_y, adcDataTx_ptr[3]);
}



void HandleModeScreenTransition(void) {
	if (robotMode != lastMode) {
		lastMode = robotMode;

		if (robotMode == STATE_SWING || robotMode == STATE_LINE_FOLLOWING || robotMode == STATE_DODGE || robotMode == STATE_STANDBY || robotMode == STATE_JOYSTICK) {
			// Esperar a que el bus I2C esté listo
			while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
				// Espera activa segura
			}

			ssd1306_SetDisplayOn(1);
			ssd1306_Fill(Black);

			if (robotMode == STATE_STANDBY) {
				// Centrar ">>>MODO 0 - STANDBY<<<"
				ssd1306_SetCursor((128 - 12 * 7) / 2, 20);
				ssd1306_WriteString(">>>MODO 0<<<", Font_7x10, White);
				ssd1306_SetCursor((128 - 7 * 7) / 2, 34);
				ssd1306_WriteString("STANDBY", Font_7x10, White);
			} else if (robotMode == STATE_SWING) {
				// Centrar ">>>MODO 1 -  BALANCEO<<<"
				// Primera línea: ">>>MODO 1<<<" (12 caracteres)
				ssd1306_SetCursor((128 - 12 * 7) / 2, 20);
				ssd1306_WriteString(">>>MODO 1<<<", Font_7x10, White);
				// Segunda línea: "BALANCEO" (8 caracteres)
				ssd1306_SetCursor((128 - 8 * 7) / 2, 34);
				ssd1306_WriteString("BALANCEO", Font_7x10, White);
			} else if (robotMode == STATE_LINE_FOLLOWING) {
				// Centrar ">>>MODO 2 - SEGUIMIENTO LINEA<<<"
				// Primera línea: ">>>MODO 2<<<" (12 caracteres)
				ssd1306_SetCursor((128 - 12 * 7) / 2, 20);
				ssd1306_WriteString(">>>MODO 2<<<", Font_7x10, White);
				// Segunda línea: "SEGUIMIENTO LINEA" (17 caracteres)
				ssd1306_SetCursor((128 - 17 * 7) / 2, 34);
				ssd1306_WriteString("SEGUIMIENTO LINEA", Font_7x10, White);
			} else if (robotMode == STATE_DODGE) {
				// Centrar ">>>MODO 3 - ESQUIVAR<<<"
				// Primera línea: ">>>MODO 3<<<" (12 caracteres)
				ssd1306_SetCursor((128 - 12 * 7) / 2, 20);
				ssd1306_WriteString(">>>MODO 3<<<", Font_7x10, White);
				// Segunda línea: "ESQUIVAR" (8 caracteres)
				ssd1306_SetCursor((128 - 8 * 7) / 2, 34);
				ssd1306_WriteString("ESQUIVAR", Font_7x10, White);
			} else if (robotMode == STATE_JOYSTICK) {
				// Centrar ">>>MODO 4 - JOYSTICK<<<"
				// Primera línea: ">>>MODO 4<<<" (12 caracteres)
				ssd1306_SetCursor((128 - 12 * 7) / 2, 20);
				ssd1306_WriteString(">>>MODO 4<<<", Font_7x10, White);
				// Segunda línea: "JOYSTICK" (8 caracteres)
				ssd1306_SetCursor((128 - 8 * 7) / 2, 34);
				ssd1306_WriteString("JOYSTICK", Font_7x10, White);
			}

			// Silenciado de motores durante el refresco síncrono I2C
			lPulse1 = 0;
			lPulse3 = 0;
			rPulse2 = 0;
			rPulse4 = 0;
			PWM_Control();

			ssd1306_UpdateScreen();

			RUN_PID = FALSE;
			integral = 0;
			last_error = 0;
		} else if (robotMode == STATE_3D_SCREEN) {
			while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
				// Espera activa segura
			}
			ssd1306_ResetDMAState();
			ssd1306_SetDisplayOn(1);
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  CDC_Attach_Rx(USBRxData); //Attach a la función que tenia en el .C

  	HAL_TIM_Base_Start_IT(&htim1);
  	HAL_TIM_Base_Start_IT(&htim2);
  	HAL_TIM_Base_Start_IT(&htim3);

  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); //Apagamos el LED

  	//Display

  	ssd1306_ADC_ConfCpltCallback(&ssd1306_TxCplt);
  	ssd1306_Attach_MemWrite(displayMemWrite);
  	ssd1306_Attach_MemWriteDMA(displayMemWriteDMA);
  	ssd1306_Init();
	ssd1306_SetDisplayOn(0); // Apagar pantalla inicialmente (modo sigue línea activo)

  	//mpu6050

  	mpu6050_ADC_ConfCpltCallback(&mpu6050_RxCplt);
  	mpu6050_Attach_MemWrite(mpuMemWrite);
  	mpu6050_Attach_MemReadDMA(mpuMemReadDMA);
  	mpu6050_Init();

  	//esp01

  	esp01Handler.DoCHPD = CHPD_Control;
 	esp01Handler.WriteUSARTByte = USART_SendByte;
 	esp01Handler.WriteByteToBufRX = WiFi_Data_Callback;

  	ESP01_Init(&esp01Handler);

  	ESP01_AttachDebugStr(DebugESP01_To_USB);
  	ESP01_AttachChangeState(OnESP01ChangeState); /* Inicia UDP automaticamente al conectar */

  	HAL_UART_Receive_IT(&huart1, &byteUART_ESP01, 1); //non blocking


  	/* ---- MODO SOFTAP TCP HERCULES (Activo al arranque) ----
  	 * Inicia emitiendo la red Wi-Fi "MICRO" (contraseña: 12345678) y levanta un servidor TCP en puerto 80.
  	 * Desde el software Hercules conectar a 192.168.4.1:80 y enviar "SSID;PASS".
  	 * Tras recibir las credenciales, el robot se conectará automáticamente a esa red. */
  	isSoftAPMode = 1;
  	ESP01_SetSoftAP("MICRO", "12345678", 5, 3, SOFTAP_TCP_PORT);

  	/* ---- MODO STATION DIRECTO (Desactivado de inicio; se usará tras recibir credenciales) ---- */
  	currentNetworkIdx = 0;
  	networkScanTimer = SCANTIME;
  	networkScanActive = 0;
  	// Para iniciar directamente conectando a knownNetworks sin pasar por SoftAP, descomentar:
  	// isSoftAPMode = 0;
  	// ESP01_SetWIFI(knownNetworks[currentNetworkIdx].ssid, knownNetworks[currentNetworkIdx].password);

  	//Inicializacion de protocolo
  	unerPrtcl_Init(&USBRx, &USBTx, buffUSBRx, buffUSBTx);
  	unerPrtcl_Init(&WiFiRx, &WiFiTx, buffWiFiRx, buffWiFiTx);
  	//Variables
  	ALLFLAGS = RESET;

  	lPulse1=0;
  	lPulse3=0;
  	rPulse2=0;
  	rPulse4=0;

    //INICIALIZAMOS BOTONES
    initButton(&myButton);

    // Habilitar contador de ciclos DWT para medición inercial de alta resolución (microsegundos)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	do10ms();
	if (robotMode != STATE_3D_SCREEN) {
		ESP01_Task();
		softAPTask();
		COMMTask(&WiFiRx, &WiFiTx, WIFI);
	}

	COMMTask(&USBRx, &USBTx, SERIE);

	PWM_Control();
	i2cTask();

	PIDTask();

	updateMefTask(&myButton);
	buttonTask(&myButton);
	HandleModeScreenTransition();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 249;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESP01_EN_GPIO_Port, ESP01_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW0_Pin */
  GPIO_InitStruct.Pin = SW0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP01_EN_Pin */
  GPIO_InitStruct.Pin = ESP01_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESP01_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Verificamos que la interrupción venga del UART1 (ESP01)
    if (huart->Instance == USART1)
    {
        // 1. Alimentamos al driver ESP01
        FeedRxBuf(byteUART_ESP01);

        // 2. Volvemos a activar la escucha para el siguiente byte
        HAL_UART_Receive_IT(&huart1, &byteUART_ESP01, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // En la familia STM32F1, leer el registro SR y luego el DR limpia el error ORE
        uint32_t er = huart->Instance->SR;
        uint32_t dr = huart->Instance->DR;
        (void)er;
        (void)dr;

        // Forzamos el reinicio de la escucha
        huart->RxState = HAL_UART_STATE_READY;
        HAL_UART_Receive_IT(&huart1, &byteUART_ESP01, 1);
    }
}

/**
 * @brief Tarea de integración y estimación inercial de la velocidad y aceleración lineal (Telemetría interna).
 * @param dt_us Delta-time medido en microsegundos de la ejecución del bucle inercial.
 */
void Speed_IntegrationTask(uint32_t dt_us) {
    static int32_t ax_calib_sum = 0;
    static int32_t gz_calib_sum = 0;
    static int32_t last_gz_filt = 0;
    static int32_t gx_filt = 0, gy_filt = 0, gz_filt = 0;
    static int32_t ax_tele_filt = 0;

    // Filtro pasa-bajos para las velocidades angulares (giroscopio)
    if (gx_filt == 0 && gy_filt == 0 && gz_filt == 0) {
        gx_filt = gx;
        gy_filt = gy;
        gz_filt = gz;
    } else {
        gx_filt = (gx * 10 + gx_filt * 90) / 100;
        gy_filt = (gy * 10 + gy_filt * 90) / 100;
        gz_filt = (gz * 10 + gz_filt * 90) / 100;
    }

    // Filtro pasa-bajos rápido de baja latencia para el acelerómetro (telemetría fina)
    if (ax_tele_filt == 0) {
        ax_tele_filt = ax;
    } else {
        ax_tele_filt = (ax * 50 + ax_tele_filt * 50) / 100;
    }

    // Compensación trigonométrica de la aceleración de gravedad debido al ángulo de inclinación
    int32_t ang = current_angle;
    int32_t linear_part = (ang * 286) / 100;
    int64_t ang64 = ang;
    int32_t cubic_part = (int32_t)((ang64 * ang64 * ang64) / 68880962LL);
    int32_t gravity_comp = linear_part - cubic_part;

    // FASE DE CALIBRACIÓN INICIAL E INTEGRACIÓN INERCIAL (Primeros 3 segundos)
    if (calib_cycle < 150) {
        if (calib_cycle < 40) {
            // Acoplamiento rápido inicial de filtros para eliminar el retardo (lag)
            ax_filt = ax;
            az_filt = az;
            gx_filt = gx;
            gy_filt = gy;
            gz_filt = gz;
            last_gz_filt = gz;
            current_angle_hr = (int32_t)ax_filt * 35;
            current_angle = current_angle_hr / 100;
            ang = current_angle;
            linear_part = (ang * 286) / 100;
            ang64 = ang;
            cubic_part = (int32_t)((ang64 * ang64 * ang64) / 68880962LL);
            gravity_comp = linear_part - cubic_part;
            ax_tele_filt = ax;
        }

        if (calib_cycle >= 50) {
            // Promedio móvil para calibración del offset en reposo
            ax_calib_sum += (ax_tele_filt - gravity_comp);
            gz_calib_sum += gz;
        }
        calib_cycle++;
        if (calib_cycle == 150) {
            ax_offset = ax_calib_sum / 100; // Offset guardado
            gz_offset = gz_calib_sum / 100; // Offset de giroscopio Z guardado
        }
        dynamic_accel = 0;
        speed = 0;
        last_gz_filt = gz_filt;
    } else {
        // MODO OPERACIÓN NORMAL (Cálculo tridimensional de fuerzas centrífugas y tangenciales)
        int32_t rot_Y_sq = (int32_t)gy_filt * gy_filt;
        int32_t rot_Z_sq = (int32_t)gz_filt * gz_filt;
        int32_t rot_XY   = (int32_t)gx_filt * gy_filt;
        int32_t alpha_z = gz_filt - last_gz_filt;
        last_gz_filt = gz_filt;

        // Compensación de fuerzas parásitas por desplazamiento físico del MPU
        int32_t rx_centrifugal = (rot_Y_sq + rot_Z_sq) / 2249000;
        int32_t ry_centrifugal = rot_XY / 2249000;
        int32_t ry_tangential  = alpha_z / 6;

        // Aceleración lineal resultante
        dynamic_accel = ax_tele_filt - ax_offset - gravity_comp - rx_centrifugal + ry_centrifugal + ry_tangential;

        // Puerta de ruido (deadband) para evitar deriva inercial remanente
        int32_t accel_for_integration = dynamic_accel;
        if (accel_for_integration > -1300 && accel_for_integration < 1300) {
            accel_for_integration = 0;
        }

        // Integración física en velocidad mm/s
        speed = (speed * 98) / 100 + (int32_t)(((int64_t)accel_for_integration * dt_us * 3LL) / 5000000LL);
    }
}

static void WiFi_ScanTick(void) {
	if (!networkScanActive) return;

	if (networkScanTimer > 0) {
		networkScanTimer--;
	} else {
		/* Se acabó el tiempo (15 segs). Pasamos a la siguiente red en la lista */
		currentNetworkIdx++;
		if (currentNetworkIdx >= NUM_KNOWN_NETWORKS) {
			currentNetworkIdx = 0; /* Volvemos al inicio de la lista */
		}

		networkScanTimer = SCANTIME; /* Reiniciamos la paciencia: 15 segundos */

		/* Forzamos al ESP01 a probar la nueva red */
		ESP01_SetWIFI(knownNetworks[currentNetworkIdx].ssid,
				knownNetworks[currentNetworkIdx].password);
	}
}

static void UART_EnforceReceiverActive(void) {
	if (huart1.RxState != HAL_UART_STATE_BUSY_RX) {
		uint32_t er = huart1.Instance->SR;
		uint32_t dr = huart1.Instance->DR;
		(void) er;
		(void) dr;
		huart1.RxState = HAL_UART_STATE_READY;
		HAL_UART_Receive_IT(&huart1, &byteUART_ESP01, 1);
	}
}

static void WiFi_HeartbeatTick(void) {
	timerUDP++;
	if (timerUDP >= 10) { // Entrar cada 10 ciclos de 100ms (1000ms o 1s)
		timerUDP = 0;

		// Incrementar contador de silencio WiFi (saturar en 60)
		if (udpSilenceCounter < 60)
			udpSilenceCounter++;

		if (!isSoftAPMode) {
			const char *currentProto = ESP01_GetProtocol();

			if (strcmp(currentProto, "UDP") == 0) {
				/* En modo UDP:
				 * Si hay silencio entre 2s y 4s, enviar ALIVE para avisar a Qt */
				if (ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED
						&& udpSilenceCounter >= 2 && udpSilenceCounter < 5) {
					static uint8_t bufferTx[9] = { 'U', 'N', 'E', 'R', 0x03, ':', ALIVE, ACK, 0x98 };
					ESP01_Send(0, bufferTx, 0, 9, TXBUFSIZE);
				}

				/* Si hay silencio continuo >= 5s (la PC no respondió a los ALIVEs UDP),
				 * probar conectar TCP cada 5 segundos por si el usuario en Qt cambió a TCP */
				static uint8_t tcpProbeTimer = 0;
				if (udpSilenceCounter >= 5) {
					if (tcpProbeTimer == 0) {
						ESP01_StartTCP(udpTargetIP, udpTargetPort, 30001);
					}
					tcpProbeTimer++;
					if (tcpProbeTimer >= 5) {
						tcpProbeTimer = 0;
					}
				} else {
					tcpProbeTimer = 0;
				}
			} else {
				/* En modo TCP:
				 * Si hay silencio entre 2s y 4s, enviar ALIVE por TCP para verificar conexión */
				if (ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED
						&& udpSilenceCounter >= 2 && udpSilenceCounter < 5) {
					static uint8_t bufferTx[9] = { 'U', 'N', 'E', 'R', 0x03, ':', ALIVE, ACK, 0x98 };
					ESP01_Send(0, bufferTx, 0, 9, TXBUFSIZE);
				}
			}
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
