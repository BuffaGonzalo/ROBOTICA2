/**
 * @file   esp01.c
 * @author Germán E. Hachmann / Gonzalo M. Buffa
 * @date   04/08/2024
 * @brief  Implementación del driver de control por comandos AT para módulo Wi-Fi ESP-01.
 * @details Implementa la máquina de estados AT para asociar el microcontrolador STM32F103
 *          a redes inalámbricas, habilitar el punto de acceso SoftAP con servidor HTTP,
 *          y transferir paquetes de telemetría a través de sockets UDP/TCP.
 * @ingroup group_comm
 */

#include "ESP01.h"
#include <stddef.h>
#include <string.h>
#include <stdlib.h>


static enum {
	ESP01ATIDLE,
	ESP01ATAT,
	ESP01ATRESPONSE,
	ESP01ATCWMODE,
	ESP01CWMODERESPONSE,
	ESP01ATCWAUTOCONN,
	ESP01CWAUTOCONNRESPONSE,
	ESP01ATCIPMUX,
	ESP01CIPMUXRESPONSE,
	ESP01ATCWSAP,        /* Nuevo: configura SoftAP */
	ESP01CWSAPRESPONSE,  /* Respuesta AT+CWSAP_CUR */
	ESP01ATCWDHCP,       /* Nuevo: habilita DHCP del AP */
	ESP01CWDHCPRESPONSE, /* Respuesta AT+CWDHCP_CUR */
	ESP01ATCIPSERVER,    /* Nuevo: inicia servidor HTTP puerto 80 */
	ESP01CIPSERVERRESPONSE, /* Respuesta AT+CIPSERVER */
	ESP01ATCWJAP,
	ESP01CWJAPRESPONSE,
	ESP01ATCIFSR,
	ESP01CIFSRRESPONSE,
	ESP01ATCIPCLOSE,
	ESP01ATCIPSTART,
	ESP01CIPSTARTRESPONSE,
	ESP01ATCONNECTED,
	ESP01ATHARDRST0,
	ESP01ATHARDRST1,
	ESP01ATHARDRSTSTOP,
} esp01ATSate = ESP01ATIDLE;

static union{
	struct{
		uint8_t WAITINGSYMBOL: 1;
		uint8_t WIFICONNECTED: 1;
		uint8_t TXCIPSEND: 1;
		uint8_t SENDINGDATA: 1;
		uint8_t HRDRESETON: 1;
		uint8_t ATRESPONSEOK: 1;
		uint8_t UDPTCPCONNECTED: 1;
		uint8_t WAITINGRESPONSE: 1;
	}bit;
	uint8_t byte;
} esp01Flags;

static void ESP01ATDecode();
static void ESP01DOConnection();
static void ESP01SENDData();
static void ESP01StrToBufTX(const char *str);
static void ESP01ByteToBufTX(uint8_t value);

static uint32_t esp01TimeoutTask = 0;
static uint32_t esp01TimeoutDataRx = 0;
static uint32_t esp01TimeoutTxSymbol = 0;
static uint32_t esp01TimeoutSendOk = 0;
static void (*ESP01ChangeState)(_eESP01STATUS esp01State);
static void (*ESP01DbgStr)(const char *dbgStr);

#define ESP01_CRED_BUF_SIZE     9        /* 8 caracteres max + '\0' */
static char esp01SSID[ESP01_CRED_BUF_SIZE] = {0};
static char esp01PASSWORD[ESP01_CRED_BUF_SIZE] = {0};
static char esp01RemoteIP[16] = {0};
static char esp01PROTO[4] = "UDP";
static char esp01RemotePORT[6] = {0};
static char esp01LocalIP[16] = {0};
static char esp01LocalPORT[6] = {0};

/* Variables para el modo SoftAP / Servidor TCP */
static char    esp01AP_SSID[ESP01_CRED_BUF_SIZE] = {0};
static char    esp01AP_PASS[ESP01_CRED_BUF_SIZE] = {0};
static char    esp01AP_CH[3]    = "5";
static char    esp01AP_ENC[2]   = "3"; /* 3 = WPA2 */
static char    esp01AP_PORT[6]  = "80";
static uint8_t esp01SoftAPMode  = 0; /* 1 = modo SoftAP servidor TCP activo */
static uint8_t esp01LastConnID  = 0; /* ID de conexion del ultimo +IPD */

static uint8_t esp01HState = 0;
static uint16_t	esp01nBytes = 0;
static uint8_t esp01RXATBuf[ESP01RXBUFAT];
static uint8_t esp01TXATBuf[ESP01TXBUFAT];
static uint16_t	esp01iwRXAT = 0;
static uint16_t	esp01irRXAT = 0;
static uint16_t esp01irTX = 0;
static uint16_t esp01iwTX = 0;

static uint8_t esp01TriesAT = 0;
static uint8_t tcpFailCount = 0;

//static _sESP01Handle esp01Handle = {.DoCHPD = NULL, .WriteUSARTByte = NULL,
//									.bufRX = NULL, .iwRX = NULL, .sizeBufferRX = 0};
static _sESP01Handle esp01Handle = {.DoCHPD = NULL, .WriteUSARTByte = NULL, .WriteByteToBufRX = NULL};

const char ATAT[] = "AT\r\n";
const char ATCIPMUX[] = "AT+CIPMUX=1\r\n";
const char ATCWQAP[] = "AT+CWQAP\r\n";
const char ATCWMODE[] = "AT+CWMODE=3\r\n";
const char ATCWJAP[] = "AT+CWJAP=";
const char ATCIFSR[] = "AT+CIFSR\r\n";
const char ATCIPSTART[] = "AT+CIPSTART=";
const char ATCIPCLOSE[] = "AT+CIPCLOSE\r\n";
const char ATCIPSEND[]   = "AT+CIPSEND=";
const char ATCWSAP[]     = "AT+CWSAP_CUR=";
const char ATCWDHCP[]    = "AT+CWDHCP_CUR=2,1\r\n";
const char ATCIPSERVER[] = "AT+CIPSERVER=1,";

const char respAT[] = "0302AT\r";
const char respATp[] = "0302AT+";
const char respOK[] = "0402OK\r\n";
const char respERROR[] = "0702ERROR\r\n";
const char respWIFIGOTIP[] = "1302WIFI GOT IP\r\n";
const char respWIFICONNECTED[] = "1602WIFI CONNECTED\r\n";
const char respWIFIDISCONNECT[] = "1702WIFI DISCONNECT\r\n";
const char respWIFIDISCONNECTED[] = "1902WIFI DISCONNECTED\r\n";
const char respDISCONNECTED[] = "1402DISCONNECTED\r\n";
const char respSENDOK[] = "0902SEND OK\r\n";
const char respCONNECT[] = "0902CONNECT\r\n";
const char respCLOSED[] = "0802CLOSED\r\n";
const char respCIFSRAPIP[] = "1205+CIFSR:STAIP";
const char respBUSY[] = "0602busy .";
const char respIPD[] = "0410+IPD";
const char respReady[] = "0702ready\r\n";
const char respBUSYP[] = "0602busy p";
const char respBUSYS[] = "0602busy s";
// 	  const char respCIFSRAPIP[] = "1102+CIFSR:APIP";
//    const char respCIFSRAPMAC[] = "1202+CIFSR:APMAC";
//    const char respCIFSRSTAIP[] = "1205+CIFSR:STAIP";
//    const char respCIFSRSTAMAC[] = "1302+CIFSR:STAMAC";

const char *const responses[] = {respAT, respATp, respOK, respERROR, respWIFIGOTIP, respWIFICONNECTED,
								 respWIFIDISCONNECT, respWIFIDISCONNECTED, respDISCONNECTED, respSENDOK, respCONNECT, respCLOSED,
								 respCIFSRAPIP, respBUSY, respIPD, respReady, respBUSYP, respBUSYS, NULL};

static uint8_t indexResponse = 0;
static uint8_t indexResponseChar = 0;

//const char _DNSFAIL[] = "DNS FAIL\r";
//const char _ATCIPDNS[] = "AT+CIPDNS_CUR=1,\"208.67.220.220\",\"8.8.8.8\"\r\n";
//const char CIFSRAPIP[] = "+CIFSR:APIP\r";
//const char CIFSRAPMAC[] = "+CIFSR:APMAC\r";
//const char CIFSRSTAIP[] = "+CIFSR:STAIP\r";
//const char CIFSRSTAMAC[] = "+CIFSR:STAMAC\r";


void ESP01_SetWIFI(const char *ssid, const char *password){
	esp01ATSate = ESP01ATIDLE;
	esp01Flags.byte = 0;
	esp01SoftAPMode = 0;   /* Salir del modo SoftAP al conectar como Station */

	strncpy(esp01SSID, ssid, ESP01_CRED_BUF_SIZE - 1);
	esp01SSID[ESP01_CRED_BUF_SIZE - 1] = '\0';
	strncpy(esp01PASSWORD, password, ESP01_CRED_BUF_SIZE - 1);
	esp01PASSWORD[ESP01_CRED_BUF_SIZE - 1] = '\0';

	esp01TimeoutTask = 50;
	esp01ATSate = ESP01ATHARDRST0;

	esp01TriesAT = 0;
}

/**
 * @brief Configura el ESP01 en modo SoftAP + inicia servidor TCP en el puerto especificado
 *
 * El cliente TCP (ej. Hercules) podrá conectarse a la IP del SoftAP (192.168.4.1) y puerto
 * para enviar "SSID;PASS", permitiendo asociarse como Station.
 *
 * @param apSSID   Nombre de la red WiFi a crear
 * @param apPass   Contraseña del AP (mínimo 8 chars). NULL o "" para red abierta
 * @param ch       Canal WiFi (1-13)
 * @param enc      Encriptación: 0=abierta, 2=WPA, 3=WPA2, 4=WPA/WPA2
 * @param port     Puerto TCP (por defecto 80 si es 0)
 */
void ESP01_SetSoftAP(const char *apSSID, const char *apPass, uint8_t ch, uint8_t enc, uint16_t port){
	esp01ATSate = ESP01ATIDLE;
	esp01Flags.byte = 0;
	esp01SoftAPMode = 1;

	strncpy(esp01AP_SSID, apSSID, ESP01_CRED_BUF_SIZE - 1);
	esp01AP_SSID[ESP01_CRED_BUF_SIZE - 1] = '\0';

	if(apPass && apPass[0] != '\0'){
		strncpy(esp01AP_PASS, apPass, ESP01_CRED_BUF_SIZE - 1);
		esp01AP_PASS[ESP01_CRED_BUF_SIZE - 1] = '\0';
	} else {
		esp01AP_PASS[0] = '\0';
		enc = 0; /* Sin contraseña → red abierta */
	}

	itoa(ch,  esp01AP_CH,  10);
	itoa(enc, esp01AP_ENC, 10);
	if(port == 0) port = 80;
	itoa(port, esp01AP_PORT, 10);

	esp01TimeoutTask = 50;
	esp01ATSate = ESP01ATHARDRST0;
	esp01TriesAT = 0;
}

/**
 * @brief Devuelve el ID de la ultima conexion recibida por +IPD
 *
 * Usar este valor para pasarlo a ESP01_Send() cuando se esta en modo webserver
 */
uint8_t ESP01_GetLastConnID(){
	return esp01LastConnID;
}


_eESP01STATUS ESP01_StartUDP(const char *RemoteIP, uint16_t RemotePORT, uint16_t LocalPORT){
	if(esp01Handle.WriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(LocalPORT == 0)
		LocalPORT = 30000;

	strcpy(esp01PROTO, "UDP");

	strncpy(esp01RemoteIP, RemoteIP, 15);
	esp01RemoteIP[15] = '\0';

	itoa(RemotePORT, esp01RemotePORT, 10);
	itoa(LocalPORT, esp01LocalPORT, 10);

	if(esp01SSID[0] == '\0')
		return ESP01_WIFI_NOT_SETED;

	if(esp01Flags.bit.WIFICONNECTED == 0)
		return ESP01_WIFI_DISCONNECTED;

	/* Limpiar transmisiones previas pendientes para evitar bloqueos del driver AT */
	esp01Flags.bit.SENDINGDATA = 0;
	esp01Flags.bit.WAITINGSYMBOL = 0;
	esp01Flags.bit.UDPTCPCONNECTED = 0;
	esp01Flags.bit.ATRESPONSEOK = 0;
	esp01irTX = esp01iwTX;

	esp01ATSate = ESP01ATCIPCLOSE;
	esp01TimeoutTask = 0;

	return ESP01_UDPTCP_CONNECTING;
}

_eESP01STATUS ESP01_StartTCP(const char *RemoteIP, uint16_t RemotePORT, uint16_t LocalPORT){
	if(esp01Handle.WriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(LocalPORT == 0)
		LocalPORT = 30000;

	strcpy(esp01PROTO, "TCP");

	strncpy(esp01RemoteIP, RemoteIP, 15);
	esp01RemoteIP[15] = '\0';

	itoa(RemotePORT, esp01RemotePORT, 10);
	itoa(LocalPORT, esp01LocalPORT, 10);

	if(esp01SSID[0] == '\0')
		return ESP01_WIFI_NOT_SETED;

	if(esp01Flags.bit.WIFICONNECTED == 0)
		return ESP01_WIFI_DISCONNECTED;

	/* Limpiar transmisiones previas pendientes para evitar bloqueos del driver AT */
	esp01Flags.bit.SENDINGDATA = 0;
	esp01Flags.bit.WAITINGSYMBOL = 0;
	esp01Flags.bit.UDPTCPCONNECTED = 0;
	esp01Flags.bit.ATRESPONSEOK = 0;
	esp01irTX = esp01iwTX;
	tcpFailCount = 0;

	esp01ATSate = ESP01ATCIPCLOSE;
	esp01TimeoutTask = 0;

	return ESP01_UDPTCP_CONNECTING;
}

const char *ESP01_GetProtocol(void){
	return esp01PROTO;
}

void ESP01_CloseUDPTCP(){
	if(esp01Handle.WriteUSARTByte == NULL)
		return;

	esp01ATSate = ESP01ATCIPCLOSE;
}

_eESP01STATUS ESP01_StateWIFI(){
	if(esp01Handle.WriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(esp01Flags.bit.WIFICONNECTED)
		return ESP01_WIFI_CONNECTED;
	else
		return ESP01_WIFI_DISCONNECTED;
}

char *ESP01_GetLocalIP(){
	if(esp01Flags.bit.WIFICONNECTED &&  esp01LocalIP[0]!='\0')
		return esp01LocalIP;

	return NULL;
}


_eESP01STATUS ESP01_StateUDPTCP(){
	if(esp01Handle.WriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(esp01Flags.bit.UDPTCPCONNECTED)
		return ESP01_UDPTCP_CONNECTED;
	else
		return ESP01_UDPTCP_DISCONNECTED;
}


/**
 * @brief Inyecta un byte recibido por interrupción USART en el buffer circular del driver.
 * @param[in] value Byte recibido por hardware USART.
 * @pre Interrupción de recepción USART habilitada.
 * @post Escribe `value` en `esp01RXATBuf` y avanza `esp01iwRXAT` con control de wrap-around.
 * @see USART1_IRQHandler
 */
void ESP01_WriteRX(uint8_t value){
	esp01RXATBuf[esp01iwRXAT++] = value;
	if(esp01iwRXAT == ESP01RXBUFAT)
		esp01iwRXAT = 0;
}

/**
 * @brief Transmite un bloque de datos del buffer circular a través del socket activo.
 * @param[in] connID Identificador del canal/conexión (0 en modo Station único).
 * @param[in] buf Puntero al buffer de memoria que almacena los datos a enviar.
 * @param[in] irRingBuf Índice de lectura actual en el buffer circular de origen.
 * @param[in] length Cantidad de bytes a transmitir.
 * @param[in] sizeRingBuf Tamaño total del buffer circular para gestión de wrap-around.
 * @return Estado de la solicitud de envío (`_eESP01STATUS`).
 * @pre Conexión establecida (`UDPTCPCONNECTED == 1` o `esp01SoftAPMode == 1`) y canal listo.
 * @post Carga el comando `AT+CIPSEND` en el buffer de transmisión TX y activa la bandera `SENDINGDATA`.
 * @see ESP01_Task
 * @see ESP01SENDData
 */
_eESP01STATUS ESP01_Send(uint8_t connID, uint8_t *buf, uint16_t irRingBuf, uint16_t length, uint16_t sizeRingBuf){
	if(esp01Handle.WriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(esp01Flags.bit.UDPTCPCONNECTED == 0 && !esp01SoftAPMode)
		return ESP01_UDPTCP_DISCONNECTED;

	if(esp01Flags.bit.SENDINGDATA == 0){
		char strInt[10];
		uint8_t l = 0;

		itoa(length, strInt, 10);
		l = strlen(strInt);
		if(l>4 || l==0)
			return ESP01_SEND_ERROR;

		ESP01StrToBufTX(ATCIPSEND);

		/* Con CIPMUX=1 (SoftAP) el formato es AT+CIPSEND=connID,length */
		/* Con CIPMUX=0 (UDP/TCP) el formato es AT+CIPSEND=length        */
		if(esp01SoftAPMode){
			char connStr[4];
			itoa(connID, connStr, 10);
			ESP01StrToBufTX(connStr);
			ESP01ByteToBufTX(',');
		}

		ESP01StrToBufTX(strInt);
		ESP01StrToBufTX("\r>");

		for(uint16_t i=0; i<length; i++){
			esp01TXATBuf[esp01iwTX++] = buf[irRingBuf++];
			if(esp01iwTX == ESP01TXBUFAT)
				esp01iwTX = 0;
			if(irRingBuf == sizeRingBuf)
				irRingBuf = 0;
		}

		esp01Flags.bit.TXCIPSEND = 1;
		esp01Flags.bit.SENDINGDATA = 1;

		if(ESP01DbgStr != NULL){
			ESP01DbgStr("+&DBGSENDING DATA ");
			ESP01DbgStr(strInt);
			ESP01DbgStr("\n");
		}

		return ESP01_SEND_READY;
	}

	if(ESP01DbgStr != NULL)
		ESP01DbgStr("+&DBGSENDING DATA BUSY\n");

	return ESP01_SEND_BUSY;
}

/**
 * @brief Inicializa las variables internas y estructuras del driver ESP-01.
 * @param[in] hESP01 Puntero a la estructura de manejadores de hardware `_sESP01Handle`.
 * @pre Estructura de hardware inicializada con los punteros válidos a GPIO y USART.
 * @post Configura buffers circulares, resetea banderas de conexión y programa el primer timeout.
 * @see _sESP01Handle
 */
void ESP01_Init(_sESP01Handle *hESP01){

	memcpy(&esp01Handle, hESP01, sizeof(_sESP01Handle));

	esp01ATSate = ESP01ATIDLE;
	esp01HState = 0;
	esp01irTX = 0;
	esp01iwTX = 0;
	esp01irRXAT = 0;
	esp01iwRXAT = 0;
	esp01Flags.byte = 0;
	esp01SoftAPMode = 0;
	esp01TimeoutTask = 50;
	ESP01ChangeState = NULL;
	ESP01DbgStr = NULL;
}

/**
 * @brief Base de tiempo periódica de 10 ms para la gestión de temporizadores del driver ESP-01.
 * @details Decrementa los contadores `esp01TimeoutTask`, `esp01TimeoutDataRx`, `esp01TimeoutTxSymbol` y `esp01TimeoutSendOk`.
 *          Destraba automáticamente envíos atascados ante la pérdida de confirmación `SEND OK`.
 * @pre Debe ser invocada en una rutina cíclica cada 10 ms (ej: planificador cooperativo).
 * @post Actualiza los temporizadores internos del driver.
 */
void ESP01_Timeout10ms(){
	if(esp01TimeoutTask)
		esp01TimeoutTask--;

	if(esp01TimeoutDataRx){
		esp01TimeoutDataRx--;
		if(!esp01TimeoutDataRx)
			esp01HState = 0;
	}

	if(esp01TimeoutTxSymbol)
		esp01TimeoutTxSymbol--;

	// --- NUEVO: WATCHDOG ANTI-CONGELAMIENTO ---
	if(esp01TimeoutSendOk){
		esp01TimeoutSendOk--;
		if(!esp01TimeoutSendOk && esp01Flags.bit.SENDINGDATA){
			// Destrabamos el envío descartando el paquete atascado.
			// NO reiniciamos la máquina de estados, solo limpiamos el canal.
			esp01Flags.bit.SENDINGDATA = 0;
			esp01irTX = esp01iwTX;
		}
	}
}

/**
 * @brief Tarea principal periódica de la máquina de estados AT del driver ESP-01.
 * @details Coordina tres subprocesos fundamentales de manera no bloqueante:
 *          1. `ESP01ATDecode()`: Decodifica cadenas entrantes por USART (`OK`, `ERROR`, `+IPD`, etc.).
 *          2. `ESP01DOConnection()`: Avanza la MEF de inicialización, asociación Wi-Fi y apertura de sockets.
 *          3. `ESP01SENDData()`: Gestiona el protocolo de despacho de paquetes (`AT+CIPSEND`).
 *
 * \startuml
 * title Despachador de Tareas ESP-01 (ESP01_Task)
 * start
 * if (¿Bytes pendientes en buffer RX AT?) then (Sí)
 *   :ESP01ATDecode();
 * endif
 * if (¿Temporizador de paso expirado (!esp01TimeoutTask)?) then (Sí)
 *   :ESP01DOConnection();
 * endif
 * :ESP01SENDData();
 * stop
 * \enduml
 *
 * @pre El driver debe estar inicializado mediante `ESP01_Init()` y ejecutarse en el super-loop.
 * @post Puede alterar el estado de conexión del robot (`ESP01ChangeState`), los buffers y flags AT.
 * @see ESP01ATDecode
 * @see ESP01DOConnection
 * @see ESP01SENDData
 */
void ESP01_Task(){

	if(esp01irRXAT != esp01iwRXAT)
		ESP01ATDecode();

	if(!esp01TimeoutTask)
		ESP01DOConnection();

	ESP01SENDData();
}

void ESP01_AttachChangeState(void (*aESP01ChangeState)(_eESP01STATUS esp01State)){
	ESP01ChangeState = aESP01ChangeState;
}

void ESP01_AttachDebugStr(void (*aESP01DbgStr)(const char *dbgStr)){
	ESP01DbgStr = aESP01DbgStr;
}

int ESP01_IsHDRRST(){
	if(esp01ATSate==ESP01ATHARDRST0 || esp01ATSate==ESP01ATHARDRST1 || esp01ATSate==ESP01ATHARDRSTSTOP)
		return 1;
	return 0;
}




/* Private Functions */
static void ESP01ATDecode(){
	uint16_t i;
	uint8_t value;

	if(esp01ATSate==ESP01ATHARDRST0 || esp01ATSate==ESP01ATHARDRST1 ||
	   esp01ATSate==ESP01ATHARDRSTSTOP){
		esp01irRXAT = esp01iwRXAT;
		return;
	}


	i = esp01iwRXAT;
	esp01TimeoutDataRx = 2;
	while(esp01irRXAT != i){
		value = esp01RXATBuf[esp01irRXAT];
		switch(esp01HState){
		case 0:
            indexResponse = 0;
            indexResponseChar = 4;
            while(responses[indexResponse] != NULL){
                if(value == responses[indexResponse][indexResponseChar]){
                    esp01nBytes = (responses[indexResponse][0] - '0');
                    esp01nBytes *= 10;
                    esp01nBytes += (responses[indexResponse][1] - '0');
                    esp01nBytes--;
                    break;
                }
                indexResponse++;
            }
            if(responses[indexResponse] != NULL){
                esp01HState = 1;
                indexResponseChar++;
            }
			else{
				esp01TimeoutDataRx = 0;
				if(esp01Flags.bit.WAITINGSYMBOL){
					if(value == '>'){
						esp01Flags.bit.WAITINGSYMBOL = 0;
						esp01TimeoutTxSymbol = 0;
						esp01TimeoutSendOk = 300;
					}
				}
			}
			break;
		case 1:
            if(value == responses[indexResponse][indexResponseChar]){
                esp01nBytes--;
                if(!esp01nBytes || value=='\r'){
                    esp01HState = (responses[indexResponse][2] - '0');
                    esp01HState *= 10;
                    esp01HState += (responses[indexResponse][3] - '0');
                    break;
                }
            }
            else{
                indexResponse = 0;
                while(responses[indexResponse] != NULL){
                    esp01nBytes = (responses[indexResponse][0] - '0');
                    esp01nBytes *= 10;
                    esp01nBytes += (responses[indexResponse][1] - '0');
                    esp01nBytes -= (indexResponseChar-3);
                    if(esp01nBytes<128 && value==responses[indexResponse][indexResponseChar]){
                        if(esp01nBytes == 0){
                            esp01HState = (responses[indexResponse][2] - '0');
                            esp01HState *= 10;
                            esp01HState += (responses[indexResponse][3] - '0');
                        }
                        break;
                    }
                    indexResponse++;
                }
                if(responses[indexResponse] == NULL){
                    esp01HState = 0;
                    esp01irRXAT--;
                    break;
                }
            }
			indexResponseChar++;
			break;
		case 2:
			if(value == '\n'){
				esp01HState = 0;
				switch(indexResponse){
				case 0://AT
				case 1:
					break;
				case 2://OK
					if (esp01ATSate == ESP01ATRESPONSE
							|| esp01ATSate == ESP01CWMODERESPONSE
							|| esp01ATSate == ESP01CWAUTOCONNRESPONSE
							|| esp01ATSate == ESP01CIPMUXRESPONSE
							|| esp01ATSate == ESP01CWSAPRESPONSE
							|| esp01ATSate == ESP01CWDHCPRESPONSE
							|| esp01ATSate == ESP01CIPSERVERRESPONSE
							|| esp01ATSate == ESP01CIPSTARTRESPONSE) {
						esp01TimeoutTask = 0;
						esp01Flags.bit.ATRESPONSEOK = 1;
					}
					break;
				case 3://ERROR
					if(esp01Flags.bit.SENDINGDATA){
						esp01Flags.bit.SENDINGDATA = 0;
						esp01Flags.bit.WAITINGSYMBOL = 0;
						esp01irTX = esp01iwTX;
						esp01TimeoutSendOk = 0;
					}
					if(esp01ATSate == ESP01CIPSTARTRESPONSE){
						esp01TimeoutTask = 0;
					}
					break;
				case 4://WIFI GOT IP
					/* SOLO aceptamos el éxito si estábamos esperando la respuesta de nuestro CWJAP */
					if (esp01ATSate == ESP01CWJAPRESPONSE) {
						esp01TimeoutTask = 0;
						esp01Flags.bit.ATRESPONSEOK = 1;
						esp01Flags.bit.WIFICONNECTED = 1;
						if (ESP01ChangeState != NULL)
							ESP01ChangeState(ESP01_WIFI_CONNECTED);
					}
					break;
				case 5://WIFI CONNECTED
					break;
				case 6:			//WIFI DISCONNECT
				case 7:			//WIFI DISCONNECTED
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					esp01Flags.bit.WIFICONNECTED = 0;

					/* Si el evento ocurre durante la configuración AT, lo ignoramos
					 * por completo para evitar abortar el escaneo. */
					if (esp01ATSate <= ESP01CWJAPRESPONSE
							|| esp01ATSate >= ESP01ATHARDRST0) {
						break;
					}

					/* Si estábamos verdaderamente conectados y se cayó la red, notificamos a main */
					if (ESP01ChangeState != NULL)
						ESP01ChangeState(ESP01_WIFI_DISCONNECTED);

					if (!esp01SoftAPMode) {
						esp01ATSate = ESP01ATHARDRSTSTOP;
					}
					break;
				case 8://DISCONNECTED
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					break;
				case 9://SEND OK
					esp01Flags.bit.SENDINGDATA = 0;
					esp01TimeoutSendOk = 0;
					if(ESP01ChangeState != NULL)
						ESP01ChangeState(ESP01_SEND_OK);
					break;
				case 10://CONNECT
					/* En CIPMUX=0: "CONNECT\r\n" indica conexion UDP/TCP establecida.
					 * En CIPMUX=1: "X,CONNECT\r\n" (con ID) → no matchea aqui.
					 * Solo actuamos si NO estamos en SoftAP para no interferir. */
					if(!esp01SoftAPMode){
						esp01TimeoutTask = 0;
						esp01Flags.bit.ATRESPONSEOK = 1;
						esp01Flags.bit.UDPTCPCONNECTED = 1;
						if(ESP01ChangeState != NULL)
							ESP01ChangeState(ESP01_UDPTCP_CONNECTED);
					}
					break;
				case 11://CLOSED
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					if(ESP01ChangeState != NULL)
						ESP01ChangeState(ESP01_UDPTCP_DISCONNECTED);
					break;
				case 13://busy
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					esp01Flags.bit.WIFICONNECTED = 0;
					break;
				case 15://ready
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					esp01Flags.bit.WIFICONNECTED = 0;
					esp01ATSate = ESP01ATHARDRSTSTOP;
					break;
				case 16:			//busy p
				case 17:			//busy s
					if (esp01Flags.bit.SENDINGDATA) {
						esp01Flags.bit.SENDINGDATA = 0;
						esp01Flags.bit.WAITINGSYMBOL = 0;
						esp01irTX = esp01iwTX;
					}
					/* Si el ESP está "ocupado" internamente con una red vieja, abortamos la
					 * espera de 15 segundos y forzamos a la librería a reintentar al instante. */
					if (esp01ATSate > ESP01ATIDLE
							&& esp01ATSate <= ESP01ATCIFSR) {
						esp01TimeoutTask = 0;
					}
					break;
				}
			}
			break;
		case 5://CIFR,STAIP
			if(value == ','){
				esp01HState = 6;
				if(ESP01DbgStr != NULL)
					ESP01DbgStr("+&DBGRESPONSE CIFSR\n");
			}
			else{
				esp01HState = 0;
				esp01irRXAT--;
				if(ESP01DbgStr != NULL)
					ESP01DbgStr("+&DBGERROR CIFSR 5\n");
			}
			break;
		case 6:
			if(value == '\"'){
				esp01HState = 7;
				esp01nBytes = 0;
			}
			break;
		case 7:
			if(value == '\"' || esp01nBytes==16)
				esp01HState = 8;
			else
				esp01LocalIP[esp01nBytes++] = value;
			break;
		case 8:
			if(value == '\n'){
				esp01HState = 0;
				if(esp01nBytes < 16){
					esp01LocalIP[esp01nBytes] = '\0';
					esp01Flags.bit.ATRESPONSEOK = 1;
					esp01TimeoutTask = 0;
				}
				else
					esp01LocalIP[0] = '\0';
				if(ESP01ChangeState != NULL)
					ESP01ChangeState(ESP01_WIFI_NEW_IP);
			}
			break;
		case 10://IPD - primera coma
			if(value == ','){
				if(esp01SoftAPMode){
					/* Con CIPMUX=1: +IPD,connID,N:data  →  leer connID primero */
					esp01HState = 13;
					esp01LastConnID = 0;
				} else {
					/* Con CIPMUX=0: +IPD,N:data  →  leer longitud directamente */
					esp01HState = 11;
					esp01nBytes = 0;
				}
			}
			else{
				esp01HState = 0;
				esp01irRXAT--;
			}
			break;
		case 13://IPD connID (solo CIPMUX=1): espera la segunda coma
			if(value == ','){
				esp01HState = 11;
				esp01nBytes = 0;
			}
			else if(value >= '0' && value <= '9'){
				esp01LastConnID = (uint8_t)(esp01LastConnID * 10 + (value - '0'));
			}
			else{
				esp01HState = 0;
				esp01irRXAT--;
			}
			break;
		case 11:
			if(value == ':')
				esp01HState = 12;
			else{
				if(value<'0' || value>'9'){
					esp01HState = 0;
					esp01irRXAT--;
				}
				else{
					esp01nBytes *= 10;
					esp01nBytes += (value - '0');
				}
			}
			break;
		case 12:
			if(esp01Handle.WriteByteToBufRX != NULL)
				esp01Handle.WriteByteToBufRX(value);
			esp01nBytes--;
			if(!esp01nBytes){
				esp01HState = 0;
				if(ESP01DbgStr != NULL)
					ESP01DbgStr("+&DBGRESPONSE IPD\n");
			}
			break;
		default:
			esp01HState = 0;
			esp01TimeoutDataRx = 0;
		}

		esp01irRXAT++;
		if(esp01irRXAT == ESP01RXBUFAT)
			esp01irRXAT = 0;
	}

}

static void ESP01DOConnection(){

	esp01TimeoutTask = 100;
	switch(esp01ATSate){
	case ESP01ATIDLE:
		esp01TimeoutTask = 0;
		break;
	case ESP01ATHARDRST0:
		esp01Handle.DoCHPD(0);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01HARDRESET0\n");
		esp01ATSate = ESP01ATHARDRST1;
		break;
	case ESP01ATHARDRST1:
		esp01Handle.DoCHPD(1);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01HARDRESET1\n");
		esp01ATSate = ESP01ATHARDRSTSTOP;
		esp01TimeoutTask = 500;
		break;
	case ESP01ATHARDRSTSTOP:
		esp01ATSate = ESP01ATAT;
		esp01TriesAT = 0;
		break;
	case ESP01ATAT:
		if(esp01TriesAT){
			esp01TriesAT--;
			if(!esp01TriesAT){
				esp01ATSate = ESP01ATHARDRST0;
				break;
			}
		}
		else
			esp01TriesAT = 4;

		esp01Flags.bit.ATRESPONSEOK = 0;
		ESP01StrToBufTX(ATAT);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01AT\n");
		esp01ATSate = ESP01ATRESPONSE;
		break;
	case ESP01ATRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK)
			esp01ATSate = ESP01ATCWMODE;
		else
			esp01ATSate = ESP01ATAT;
		break;
	case ESP01ATCWMODE:
			ESP01StrToBufTX(ATCWMODE);
			if(ESP01DbgStr != NULL)
				ESP01DbgStr("+&DBGESP01ATCWMODE\n");
			esp01Flags.bit.ATRESPONSEOK = 0;
			esp01ATSate = ESP01CWMODERESPONSE;
			break;
		case ESP01CWMODERESPONSE:
			if(esp01Flags.bit.ATRESPONSEOK)
				esp01ATSate = ESP01ATCWAUTOCONN;
			else
				esp01ATSate = ESP01ATAT;
			break;

		case ESP01ATCWAUTOCONN:
			ESP01StrToBufTX("AT+CWAUTOCONN=0\r\n"); // Evitar la conexión automatica
			esp01Flags.bit.ATRESPONSEOK = 0;
			esp01ATSate = ESP01CWAUTOCONNRESPONSE;
			break;
		case ESP01CWAUTOCONNRESPONSE:
			if(esp01Flags.bit.ATRESPONSEOK)
				esp01ATSate = ESP01ATCIPMUX;
			else
				esp01ATSate = ESP01ATAT;
			break;

		case ESP01ATCIPMUX:
			/* Enviar CIPMUX=1 para SoftAP TCP Server y CIPMUX=0 para UDP normal */
			if (esp01SoftAPMode) {
				ESP01StrToBufTX("AT+CIPMUX=1\r\n");
			} else {
				ESP01StrToBufTX("AT+CIPMUX=0\r\n");
			}
			if (ESP01DbgStr != NULL)
				ESP01DbgStr("+&DBGESP01ATCIPMUX\n");
			esp01Flags.bit.ATRESPONSEOK = 0;
			esp01ATSate = ESP01CIPMUXRESPONSE;
			break;
		case ESP01CIPMUXRESPONSE:
			if(esp01Flags.bit.ATRESPONSEOK)
				esp01ATSate = (esp01SoftAPMode) ? ESP01ATCWSAP : ESP01ATCWJAP;
			else
				esp01ATSate = ESP01ATAT;
			break;

	/* ---- NUEVOS ESTADOS: modo SoftAP + Webserver ---- */
	case ESP01ATCWSAP:
		/* AT+CWSAP_CUR="ssid","pass",canal,enc */
		ESP01StrToBufTX(ATCWSAP);
		ESP01ByteToBufTX('"');
		ESP01StrToBufTX(esp01AP_SSID);
		ESP01ByteToBufTX('"');
		ESP01ByteToBufTX(',');
		ESP01ByteToBufTX('"');
		ESP01StrToBufTX(esp01AP_PASS);
		ESP01ByteToBufTX('"');
		ESP01ByteToBufTX(',');
		ESP01StrToBufTX(esp01AP_CH);
		ESP01ByteToBufTX(',');
		ESP01StrToBufTX(esp01AP_ENC);
		ESP01ByteToBufTX('\r');
		ESP01ByteToBufTX('\n');
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCWSAP\n");
		esp01Flags.bit.ATRESPONSEOK = 0;
		esp01ATSate = ESP01CWSAPRESPONSE;
		esp01TimeoutTask = 300;
		break;
	case ESP01CWSAPRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK)
			esp01ATSate = ESP01ATCWDHCP;
		else
			esp01ATSate = ESP01ATAT;
		break;
	case ESP01ATCWDHCP:
		/* AT+CWDHCP_CUR=2,1  →  habilita DHCP del SoftAP */
		ESP01StrToBufTX(ATCWDHCP);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCWDHCP\n");
		esp01Flags.bit.ATRESPONSEOK = 0;
		esp01ATSate = ESP01CWDHCPRESPONSE;
		esp01TimeoutTask = 200;
		break;
	case ESP01CWDHCPRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK)
			esp01ATSate = ESP01ATCIPSERVER;
		else
			esp01ATSate = ESP01ATAT;
		break;
	case ESP01ATCIPSERVER:
		/* AT+CIPSERVER=1,<port>  →  inicia servidor TCP en el puerto configurado */
		ESP01StrToBufTX(ATCIPSERVER);
		ESP01StrToBufTX(esp01AP_PORT);
		ESP01StrToBufTX("\r\n");
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCIPSERVER\n");
		esp01Flags.bit.ATRESPONSEOK = 0;
		esp01ATSate = ESP01CIPSERVERRESPONSE;
		esp01TimeoutTask = 200;
		break;
	case ESP01CIPSERVERRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK)
			esp01ATSate = ESP01ATCONNECTED;
		else
			esp01ATSate = ESP01ATAT;
		break;
	/* ---- FIN NUEVOS ESTADOS ---- */
	case ESP01ATCWJAP:
		/*
		if(esp01Flags.bit.WIFICONNECTED){
			esp01ATSate = ESP01ATCIFSR;
			break;
		}
		*/
		if(esp01SSID[0] == '\0')
			break;
		ESP01StrToBufTX(ATCWJAP);
		ESP01ByteToBufTX('\"');
		ESP01StrToBufTX(esp01SSID);
		ESP01ByteToBufTX('\"');
		ESP01ByteToBufTX(',');
		ESP01ByteToBufTX('\"');
		ESP01StrToBufTX(esp01PASSWORD);
		ESP01ByteToBufTX('\"');
		ESP01ByteToBufTX('\r');
		ESP01ByteToBufTX('\n');
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCWJAP\n");
		esp01Flags.bit.ATRESPONSEOK = 0;
		esp01ATSate = ESP01CWJAPRESPONSE;
		esp01TimeoutTask = 1500;
		break;
	case ESP01CWJAPRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK){
			esp01ATSate = ESP01ATCIFSR;
			esp01TriesAT = 4;
		}
		else
			esp01ATSate = ESP01ATAT;
		break;
	case ESP01ATCIFSR:
		esp01LocalIP[0] = '\0';
		ESP01StrToBufTX(ATCIFSR);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01CIFSR\n");
		esp01Flags.bit.ATRESPONSEOK = 0;
		esp01ATSate = ESP01CIFSRRESPONSE;
		break;
	case ESP01CIFSRRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK)
			esp01ATSate = ESP01ATCIPCLOSE;
		else{
			esp01TriesAT--;
			if(esp01TriesAT == 0){
				esp01ATSate = ESP01ATAT;
				break;
			}
			esp01ATSate = ESP01ATCIFSR;
		}
		break;
	case ESP01ATCIPCLOSE:
		if(esp01RemoteIP[0] == '\0')
			break;
		ESP01StrToBufTX(ATCIPCLOSE);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCIPCLOSE\n");
		esp01ATSate = ESP01ATCIPSTART;
		esp01TimeoutTask = 20; /* 200ms para asegurar cierre previo en el ESP */
		break;
	case ESP01ATCIPSTART:
		ESP01StrToBufTX(ATCIPSTART);
		ESP01ByteToBufTX('\"');
		ESP01StrToBufTX(esp01PROTO);
		ESP01ByteToBufTX('\"');
		ESP01ByteToBufTX(',');
		ESP01ByteToBufTX('\"');
		ESP01StrToBufTX(esp01RemoteIP);
		ESP01ByteToBufTX('\"');
		ESP01ByteToBufTX(',');
		ESP01StrToBufTX(esp01RemotePORT);
		if(strcmp(esp01PROTO, "UDP") == 0){
			ESP01ByteToBufTX(',');
			ESP01StrToBufTX(esp01LocalPORT);
			ESP01ByteToBufTX(',');
			ESP01ByteToBufTX('0');
		}
		ESP01ByteToBufTX('\r');
		ESP01ByteToBufTX('\n');
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCIPSTART\n");
		esp01Flags.bit.ATRESPONSEOK = 0;
		esp01Flags.bit.UDPTCPCONNECTED = 0;
		esp01ATSate = ESP01CIPSTARTRESPONSE;
		esp01TimeoutTask = 150; /* 1.5s timeout de respuesta */
		break;
	case ESP01CIPSTARTRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK){
			esp01Flags.bit.UDPTCPCONNECTED = 1;
			tcpFailCount = 0;
			esp01ATSate = ESP01ATCONNECTED;
			if(ESP01ChangeState != NULL)
				ESP01ChangeState(ESP01_UDPTCP_CONNECTED);
		} else {
			esp01Flags.bit.UDPTCPCONNECTED = 0;
			if(esp01Flags.bit.WIFICONNECTED){
				/* Si falló TCP tras 2 intentos (ej: la PC aún no abrió TCP o está en UDP), regresar a UDP */
				if(strcmp(esp01PROTO, "TCP") == 0){
					tcpFailCount++;
					if(tcpFailCount >= 2){
						tcpFailCount = 0;
						strcpy(esp01PROTO, "UDP");
						if(esp01LocalPORT[0] == '\0') strcpy(esp01LocalPORT, "30001");
					}
				} else {
					tcpFailCount = 0;
				}
				esp01ATSate = ESP01ATCIPSTART;
				esp01TimeoutTask = 100; /* Reintentar en 1s */
			} else {
				esp01ATSate = ESP01ATAT;
			}
		}
		break;
	case ESP01ATCONNECTED:
		/* En modo SoftAP servidor TCP esperamos conexiones entrantes */
		if(esp01SoftAPMode){
			esp01TimeoutTask = 0;
			break;
		}
		if(esp01Flags.bit.WIFICONNECTED == 0){
			esp01ATSate = ESP01ATAT;
			break;
		}
		if(esp01Flags.bit.UDPTCPCONNECTED == 0){
			/* Si estábamos en TCP y el socket se cerró (DISCONNECT en Qt), regresar a UDP */
			if(strcmp(esp01PROTO, "TCP") == 0){
				strcpy(esp01PROTO, "UDP");
				if(esp01LocalPORT[0] == '\0') strcpy(esp01LocalPORT, "30001");
			}
			esp01ATSate = ESP01ATCIPCLOSE;
			esp01TimeoutTask = 0;
			break;
		}
		esp01TimeoutTask = 0;
		break;
	}
}

static void ESP01SENDData(){
	uint8_t value;

	if(esp01Flags.bit.WAITINGSYMBOL){
		if(!esp01TimeoutTxSymbol){
			esp01irTX = esp01iwTX;
			esp01Flags.bit.WAITINGSYMBOL = 0;
			esp01Flags.bit.SENDINGDATA   = 0;
			/* En modo UDP / webserver no reiniciamos el AT por timeout de envío */
			/* Simplemente abortamos la transmisión del paquete perdido */
		}
		return;
	}
	if(esp01irTX != esp01iwTX){
		value = esp01TXATBuf[esp01irTX];
		if(esp01Flags.bit.TXCIPSEND){
			if(value == '>')
				value = '\n';
		}
		if(esp01Handle.WriteUSARTByte(value)){
			if(esp01Flags.bit.TXCIPSEND){
				if(esp01TXATBuf[esp01irTX] == '>'){
					esp01Flags.bit.TXCIPSEND = 0;
					esp01Flags.bit.WAITINGSYMBOL = 1;
					esp01TimeoutTxSymbol = 50;
				}
			}
			esp01irTX++;
			if(esp01irTX == ESP01TXBUFAT)
				esp01irTX = 0;
		}
	}
}

static void ESP01StrToBufTX(const char *str){
	for(int i=0; str[i]; i++){
		esp01TXATBuf[esp01iwTX++] = str[i];
		if(esp01iwTX == ESP01TXBUFAT)
			esp01iwTX = 0;
	}
}

static void ESP01ByteToBufTX(uint8_t value){
	esp01TXATBuf[esp01iwTX++] = value;
	if(esp01iwTX == ESP01TXBUFAT)
		esp01iwTX = 0;
}


/* END Private Functions*/
