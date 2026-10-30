/**
 * @file   util.h
 * @author Gonzalo M. Buffa
 * @date   24/05/2025
 * @brief  Declaración de tipos fundamentales, enumeraciones, estructuras y uniones del proyecto.
 * @details Este archivo centraliza los identificadores de comandos del protocolo binario,
 *          las máquinas de estado para botones y periféricos DMA, y las estructuras auxiliares
 *          empleadas transversalmente en todo el firmware del péndulo invertido.
 * @ingroup group_main
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include <stdint.h>

/**
 * @name Constantes Booleanas
 * @{
 */
enum {
	FALSE, /*!< Estado lógico Falso (0) */
	TRUE   /*!< Estado lógico Verdadero (1) */
};
/** @} */

/**
 * @name Canales de Comunicación Seleccionables
 * @{
 */
enum {
	SERIE, /*!< Canal de comunicación por puerto Serie / USB CDC */
	WIFI   /*!< Canal de comunicación inalámbrico Wi-Fi (ESP-01) */
};
/** @} */

/**
 * @brief Unión de mapa de bits para banderas de tiempo real y eventos del sistema.
 * @details Permite la consulta y modificación atómica de flags individuales de tareas o
 *          el reinicio/lectura global en formato de byte completo.
 */
typedef union{
    struct{
        uint8_t bit7 : 1; /*!< Bandera reservada bit 7 */
        uint8_t bit6 : 1; /*!< Bandera reservada bit 6 */
        uint8_t bit5 : 1; /*!< Bandera reservada bit 5 */
        uint8_t bit4 : 1; /*!< RUN_PID: Habilita la ejecución del lazo de equilibrio en 5ms */
        uint8_t bit3 : 1; /*!< Bandera reservada bit 3 */
        uint8_t bit2 : 1; /*!< IS100MS: Ciclo de temporización periódica de 100 ms activo */
        uint8_t bit1 : 1; /*!< IS20MS: Ciclo de temporización periódica de 20 ms activo */
        uint8_t bit0 : 1; /*!< IS10MS: Ciclo de temporización periódica de 10 ms activo */
    }bits;
    uint8_t bytes; /*!< Acceso directo a los 8 bits de banderas en un único byte */
} _uFlag;

/**
 * @brief Identificadores de comandos binarios del protocolo unerPrtcl.
 * @details Códigos de operación empleados en las tramas binarias transmitidas y recibidas
 *          entre el robot péndulo invertido y el software de telemetría/control.
 */
typedef enum {
	ALIVE = 0xA0,            /*!< Ping / Keep-alive de verificación de enlace de comunicación */
	FIRMWARE = 0xA1,         /*!< Solicitud de la cadena de versión compilada del firmware */
	GETMPU = 0xA2,           /*!< Solicitud de lectura de datos inerciales RAW del sensor MPU6050 */
	GETADC = 0xA3,           /*!< Solicitud de lecturas analógicas de los 8 canales ADC */

	SETPWML = 0xA4,          /*!< Calibración / test directo del ciclo de trabajo del motor izquierdo */
	SETPWMR = 0xA5,          /*!< Calibración / test directo del ciclo de trabajo del motor derecho */

	SETPWMMINR = 0xA6,       /*!< Configuración del umbral mínimo de PWM para motor derecho (fricción) */
	SETPWMMINL = 0xA7,       /*!< Configuración del umbral mínimo de PWM para motor izquierdo (fricción) */

	SETBALANCEKP = 0xA8,     /*!< Ajuste de la ganancia proporcional (Kp) del lazo PID de equilibrio */
	SETBALANCEKD = 0xA9,     /*!< Ajuste de la ganancia derivativa (Kd) del lazo PID de equilibrio */
	SETBALANCEKI = 0xAA,     /*!< Ajuste de la ganancia integral (Ki) del lazo PID de equilibrio */

	SETSETPOINT = 0xAB,      /*!< Ajuste del ángulo de consigna (setpoint) de balance en centésimas de grado */

	SETLINEKP = 0xAC,        /*!< Ajuste de la ganancia proporcional (Kp) del seguidor de línea */
	SETLINEKD = 0xAD,        /*!< Ajuste de la ganancia derivativa (Kd) del seguidor de línea */

	GETINTERNALDATA = 0xF0,  /*!< Solicitud de paquete compacto de telemetría interna para graficado en PC */
	GETPIDBALANCE = 0xF1,    /*!< Solicitud de parámetros actuales del controlador PID de balance */

	SETOFFSETL = 0xAE,       /*!< Ajuste del offset de compensación de potencia en motor izquierdo */
	SETOFFSETR = 0xAF,       /*!< Ajuste del offset de compensación de potencia en motor derecho */

	SETCUSTOMTURN = 0xB0,    /*!< Ajuste de la velocidad angular para giros prefijados de búsqueda */
	SETSPEED = 0xB1,         /*!< Ajuste de la velocidad nominal de traslación */
	SETBKANG = 0xB2,         /*!< Ajuste del umbral de ángulo de recuperación trasera */

	SETFRONTDIST = 0xB3,     /*!< Ajuste de la distancia de detección frontal para evasión de obstáculos */
	SETSIDEDIST = 0xB4,      /*!< Ajuste de la distancia lateral de referencia en seguimiento de pared */
	SETLOSTDIST = 0xB5,      /*!< Ajuste del umbral de pérdida de pared */
	SETSTOPCYCLES = 0xB6,    /*!< Número de ciclos de detención previa antes de girar en evasión */
	SETCORNERDIST = 0xB7,    /*!< Distancia de anticipación en esquinas para sensor a 45° */
	SETALIGNDIST = 0xB8,     /*!< Distancia de alineación con respecto al obstáculo */

	SETPWMLROT = 0xB9,       /*!< PWM estático asignado a la rueda izquierda en giro pivot de búsqueda */
	SETPWMRROT = 0xC0,       /*!< PWM estático asignado a la rueda derecha en giro pivot de búsqueda */

	SETSTATICOFF = 0xC1,     /*!< Offset estático de visualización en display OLED */
	SETMOVINGOFF = 0xC2,     /*!< Offset móvil de visualización en display OLED */
	SET_KP_EXT = 0xC3,       /*!< Ajuste de la ganancia proporcional (Kp) del lazo externo de velocidad */
	SETLIMITANG = 0xC4,      /*!< Límite angular máximo permitido antes del apagado de seguridad */
	SET_KI_EXT = 0xC5,       /*!< Ajuste de la ganancia integral (Ki) del lazo externo de velocidad */
	SET_ALFA_LPF = 0xC6,     /*!< Coeficiente alfa del filtro pasa-bajos del lazo de esfuerzo motor */
	SETVELDAMPDIV = 0xC7,    /*!< Divisor para el término de amortiguamiento por velocidad en giros */
	SETVELDAMPLIM = 0xC8,    /*!< Límite de saturación del amortiguador de velocidad */
	SETTURNLIMIT = 0xC9,     /*!< Límite superior del esfuerzo de guiñada (Yaw) */

	SETWALLKP = 0xCB,        /*!< Ganancia proporcional para seguimiento de pared en evasión */
	SETWALLKD = 0xCC,        /*!< Ganancia derivativa de anticipación para seguimiento de pared */
	SETFRONTKP = 0xCD,       /*!< Ganancia de protección frontal en evasión de obstáculos */
	SETFRONTKD = 0xCE,       /*!< Ganancia derivativa frontal en evasión de obstáculos */
	SETDODGEMODE = 0xCF,     /*!< Conmutación de sub-modos de esquivado de obstáculos */
	SETSOFTAP = 0xD1,        /*!< Conmutación forzada a modo Access Point Wi-Fi */
	SETROBOTMODE = 0xD2,     /*!< Comando de cambio del modo operativo principal del robot */
	SETJOYSTICKTURN = 0xD3,  /*!< Comando de control interactivo para modo Joystick (giro y watchdog) */
#define SETGOTOTURN SETJOYSTICKTURN /*!< Macro de compatibilidad hacia atrás para SETJOYSTICKTURN */
	SETDODGEBIASTIME = 0xD4, /*!< Ajuste de la duración del sesgo tras esquivar (ms) */
	SETDODGEBIASMULT = 0xD5, /*!< Ajuste del multiplicador de esfuerzo del sesgo tras esquivar */

	ACK = 0x0D,              /*!< Respuesta de confirmación positiva (Acknowledge) */
	EXPORTIRCSV = 0xCA,      /*!< Disparo de exportación de calibraciones IR a formato CSV */
	UNKNOWN = 0xFF           /*!< Código de comando no reconocido / error de sintaxis */
} _eCmd;

/**
 * @brief Tareas de multiplexado en la cola del bus I2C (Pila[]).
 */
typedef enum{
	IDLE = 0,         /*!< Bus I2C libre / sin operaciones en cola */
	DATA_DISPLAY = 1, /*!< Envío de comandos o página de configuración a la pantalla OLED */
	UPD_DISPLAY = 2,  /*!< Envío del buffer completo de video (128x64) a la pantalla OLED por DMA */
	ONMPU = 3         /*!< Lectura por DMA de los registros del sensor inercial MPU6050 */
} _eDMA;

/**
 * @brief Estados físicos internos de la MEF de lectura del botón de usuario.
 */
typedef enum{
    BUTTON_DOWN,    /*!< Botón presionado de manera sostenida (nivel lógico activo) */
    BUTTON_UP,      /*!< Botón en reposo / no presionado */
    BUTTON_RISING,  /*!< Flanco ascendente detectado (transición de no presionado a presionado) */
    BUTTON_FALLING  /*!< Flanco descendente detectado (transición de presionado a no presionado) */
} _eButtonState;

/**
 * @brief Eventos de entrada procesados por la máquina de estados del botón.
 * @note En configuración Pull-Down: NOT_PRESSED = 0 y PRESSED = 1.
 */
typedef enum{
    PRESSED,     /*!< Nivel lógico activo detectado en el pin del botón */
    NOT_PRESSED, /*!< Nivel lógico inactivo detectado en el pin del botón */
    NO_EVENT     /*!< Sin cambios de estado en el intervalo de muestreo */
} _eEvent;

/**
 * @brief Estructura de control y temporización del pulsador multifunción.
 * @details Gestiona el filtrado antirrebote, la cuenta de pulsaciones consecutivas (multiclic)
 *          y la detección de pulsaciones sostenidas de larga duración.
 */
typedef struct
{
    _eButtonState   currentState; /*!< Estado actual en la máquina de estados de debouncing */
    _eEvent         stateInput;   /*!< Lectura cruda del pin GPIO filtrada */
    uint8_t        isPressed;    /*!< Bandera booleana que indica si el botón está efectivamente oprimido */
    uint16_t        time;         /*!< Contador de permanencia temporal en milisegundos */
    uint8_t         clickCount;   /*!< Contador de pulsaciones detectadas en la ventana temporal */
    uint8_t         justReleased; /*!< Bandera de evento generada inmediatamente al soltar el botón */
} _sButton;

/**
 * @brief Estructura de credenciales para redes Wi-Fi preconfiguradas.
 * @ingroup group_comm
 */
typedef struct {
	const char *ssid;     /*!< Nombre de la red Wi-Fi (SSID) */
	const char *password; /*!< Contraseña de acceso */
	const char *targetIP; /*!< Dirección IP del host remoto para envío de telemetría UDP/TCP */
} _sWiFiNetwork;

/**
 * @brief Sub-estados de la máquina de seguimiento de línea (Line Following MEF).
 * @ingroup group_control
 */
typedef enum {
	LINE_SEARCHING, /*!< Estado 0: Pérdida total inicial, ejecutando patrón oscilante de búsqueda */
	LINE_FOLLOWING, /*!< Estado 1: Seguimiento lineal/cuadrático normal sobre la pista negra */
	LINE_LOST,      /*!< Estado 2: Desvío reciente de la línea, aplicando maniobra de rotación de 90°/180° */
	LINE_CROSS      /*!< Estado 3: Cruce perpendicular o bifurcación en T detectada (todos los sensores activos) */
} _eLineState;

/**
 * @brief Sub-estados de la máquina de esquivado y evasión de obstáculos frontales.
 * @ingroup group_control
 */
typedef enum {
	OBS_IDLE,      /*!< Estado 0: Trayectoria frontal libre de obstáculos */
	OBS_APPROACH,  /*!< Estado 1: Detección inminente de obstáculo, frenando e iniciando rotación */
	OBS_CORNER,    /*!< Estado 2: Maniobrando esquinas cerradas del obstáculo mediante sensor a 45° */
	OBS_WALL       /*!< Estado 3: Seguimiento paralelo a la pared del obstáculo con control PD */
} _eObsState;

/**
 * @brief Modos operativos principales del robot péndulo invertido gobernados por robotMode.
 * @ingroup group_main
 */
typedef enum {
    STATE_STANDBY = 0,        /*!< Modo Reposo: Motores desenergizados (PWM 0%), sistema en espera segura */
    STATE_SWING = 1,          /*!< Modo Swing (1 clic): Balanceo estático en el lugar, display OLED apagado */
    STATE_LINE_FOLLOWING = 2, /*!< Modo Seguidor de Línea (2 clics): Navegación autónoma por pista con sensores IR */
    STATE_DODGE = 3,          /*!< Modo Evasión de Obstáculos (3 clics): Detección frontal, giro 90° y seguimiento de pared */
    STATE_JOYSTICK = 4,       /*!< Modo Joystick (4 clics): Control interactivo remoto desde PC/App vía UDP con watchdog */
    STATE_3D_SCREEN = 5,      /*!< Modo 3D Screen (5 clics): Gemelo digital wireframe en OLED (Cubo->Teseracto->Pirámide) con balance activo */
    STATE_FIRST_SCREEN = 6,   /*!< Modo Diagnóstico RAW (Pulsación 1s): Muestra lecturas crudas de ADC y MPU6050, motores apagados */
    STATE_SECOND_SCREEN = 7   /*!< Modo Diagnóstico Dinámico (Pulsación 2s): Muestra velocidad, ángulo y salida PID, motores apagados */
} _eRobotMode;
#define STATE_GOTO STATE_JOYSTICK /*!< Alias de compatibilidad hacia atrás */

/**
 * @brief Sub-estados de la máquina de evasión de obstáculos (Dodge Rotation).
 * @ingroup group_control
 */
typedef enum {
    DODGE_LINE_FOLLOWING,  /*!< Seguimiento de línea con rampa de desaceleración */
    DODGE_ROTATING,        /*!< Rotación de 90° con giroscopio */
    DODGE_WALL_FOLLOWING,  /*!< Evasión PD continua */
    DODGE_RETURN_ROTATING, /*!< Rotación sobre el lugar para re-enganchar la línea */
    DODGE_STANDBY          /*!< Standby unificado de frenado (1.5s a +1000, 1.5s a +350) */
} _eDodgeSubState;

#endif /* INC_UTIL_H_ */
