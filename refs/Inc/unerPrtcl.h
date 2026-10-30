/**
 * @file   unerPrtcl.h
 * @author Gonzalo M. Buffa
 * @date   24/05/2025
 * @brief  Definiciones y primitivas del protocolo de comunicación binario UNER (unerPrtcl).
 * @details Este módulo gestiona la construcción y decodificación de tramas binarias
 *          basadas en el protocolo estándar UNER, administrando buffers circulares con máscaras
 *          de tamaño potencia de dos ($2^N - 1$) y cálculo incremental de checksum por XOR.
 * @ingroup group_comm
 */

#ifndef INC_UNERPRTCL_H_
#define INC_UNERPRTCL_H_

#include <stdint.h>

/**
 * @defgroup Protocol_Config Parámetros de Configuración del Protocolo
 * @ingroup group_comm
 * @{
 */
#define RXBUFSIZE           256 /*!< Capacidad en bytes del buffer circular de recepción (debe ser potencia de 2) */
#define TXBUFSIZE           256 /*!< Capacidad en bytes del buffer circular de transmisión (debe ser potencia de 2) */
#define TXBYTES				7   /*!< Longitud base en bytes del encabezado estándar de transmisión ("UNER" + Len + ':' + ID) */
/** @} */

/**
 * @brief Unión multipropósito para conversión y serialización de tipos escalares (1, 2 y 4 bytes).
 * @details Facilita la descomposición de enteros de 16 y 32 bits a secuencias de bytes
 *          para su transmisión directa en la carga útil de la trama binaria sin recurrir a casting inseguro.
 * @see _sComm
 * @see _eCmd
 */
typedef union {
	uint32_t ui32;    /*!< Entero sin signo de 32 bits (4 bytes) */
	int32_t i32;      /*!< Entero con signo de 32 bits (4 bytes) */
	uint16_t ui16[2]; /*!< Vector de 2 enteros sin signo de 16 bits (Little-Endian / orden de bus) */
	int16_t i16[2];   /*!< Vector de 2 enteros con signo de 16 bits */
	uint8_t ui8[4];   /*!< Vector de 4 bytes sin signo directos */
	int8_t i8[4];     /*!< Vector de 4 bytes con signo */
} _uWord;

/**
 * @brief Estructura de control de buffer circular para canales de comunicación (Rx/Tx).
 * @details Encapsula la memoria física del buffer, los índices de lectura/escritura,
 *          la máscara de desbordamiento circular y las variables de estado de la MEF de recepción.
 * @see _eDecode
 * @see _uWord
 * @see unerPrtcl_Init
 * @see unerPrtcl_DecodeHeader
 */
typedef struct {
	uint8_t *buff;      /*!< Puntero al array de memoria física asignado al buffer */
	uint8_t indexR;     /*!< Índice de lectura (Read Index) en el buffer circular */
	uint8_t indexW;     /*!< Índice de escritura (Write Index) en el buffer circular */
	uint8_t indexData;  /*!< Puntero al byte inicial de datos útiles (Payload) dentro del buffer */
	uint8_t mask;       /*!< Máscara binaria para avance circular sin operación módulo ($2^N - 1$) */
	uint8_t chk;        /*!< Acumulador de Checksum por operación XOR para la trama activa */
	uint8_t nBytes;     /*!< Contador de longitud de bytes recibidos o pendientes de despacho */
	uint8_t header;     /*!< Estado actual en la máquina de estados de sincronización de cabecera (_eDecode) */
} _sComm;

/**
 * @brief Estados de la máquina de decodificación de tramas binarias entrantes.
 * @details La secuencia de sincronización valida rigurosamente la secuencia de preámbulo
 *          `'U'` &rarr; `'N'` &rarr; `'E'` &rarr; `'R'` antes de evaluar la longitud y el token.
 * @see unerPrtcl_DecodeHeader
 * @see _sComm
 */
typedef enum {
	HEADER_U, /*!< Esperando el carácter de sincronismo inicial 'U' */
	HEADER_N, /*!< Esperando el segundo carácter de sincronismo 'N' */
	HEADER_E, /*!< Esperando el tercer carácter de sincronismo 'E' */
	HEADER_R, /*!< Esperando el cuarto carácter de sincronismo 'R' */
	NBYTES,   /*!< Leyendo el byte que define la longitud de la trama */
	TOKEN,    /*!< Esperando el delimitador de comando ':' */
	ID,       /*!< Leyendo el identificador del comando (_eCmd) */
	PAYLOAD   /*!< Recibiendo el cuerpo de datos útiles y validando el checksum final */
} _eDecode;

/**
 * @brief Inicializa las estructuras de comunicación circular para transmisión y recepción.
 * @param[in,out] Rx Puntero a la estructura de recepción `_sComm`.
 * @param[in,out] Tx Puntero a la estructura de transmisión `_sComm`.
 * @param[in] buffRx Array de memoria física para el buffer de recepción.
 * @param[in] buffTx Array de memoria física para el buffer de transmisión.
 * @pre `buffRx` y `buffTx` deben ser punteros válidos con tamaño igual a `RXBUFSIZE` y `TXBUFSIZE`.
 * @post Reinicia a 0 todos los índices (`indexR`, `indexW`, `indexData`, `nBytes`, `chk`) y configura `mask`.
 * @see _sComm
 */
void unerPrtcl_Init(_sComm *Rx, _sComm *Tx, volatile uint8_t *buffRx, volatile uint8_t *buffTx);

/**
 * @brief Escribe el encabezado estándar del protocolo UNER en el buffer de transmisión.
 * @details Escribe la secuencia `'U'`, `'N'`, `'E'`, `'R'`, el campo de longitud ajustada,
 *          el carácter separador `':'` y el identificador de comando `ID`. Además, inicializa
 *          el cálculo de Checksum XOR de la trama.
 * @param[in,out] dataTx Puntero a la estructura de transmisión `_sComm`.
 * @param[in] ID Identificador del comando a despachar (definido en `_eCmd`).
 * @param[in] frameLength Longitud de los datos que componen el payload.
 * @pre El buffer `dataTx` debe contar con al menos `TXBYTES` (7) bytes libres.
 * @post Avanza `indexW` en 7 posiciones en anillo y actualiza `dataTx->chk` con el XOR de la cabecera.
 * @return Checksum XOR calculado hasta el byte de ID.
 * @see unerPrtcl_PutByteOnTx
 * @see unerPrtcl_PutStrOntx
 */
uint8_t unerPrtcl_PutHeaderOnTx(_sComm *dataTx, uint8_t ID, uint8_t frameLength);

/**
 * @brief Inserta un único byte en el buffer circular de transmisión y actualiza el Checksum.
 * @param[in,out] dataTx Puntero a la estructura de transmisión `_sComm`.
 * @param[in] byte Valor a encolar en el buffer de salida.
 * @pre Debe existir al menos un byte libre entre `indexW` e `indexR`.
 * @post Incrementa `nBytes`, avanza `indexW` y acumula `byte` en `dataTx->chk` mediante XOR.
 * @return Checksum acumulado resultante tras la operación XOR.
 * @see unerPrtcl_PutHeaderOnTx
 */
uint8_t unerPrtcl_PutByteOnTx(_sComm *dataTx, uint8_t byte);

/**
 * @brief Inserta una cadena de caracteres terminada en nulo (`\0`) en el buffer de transmisión.
 * @param[in,out] dataTx Puntero a la estructura de transmisión `_sComm`.
 * @param[in] str Cadena de caracteres ASCII a transmitir.
 * @pre `str` debe terminar en carácter nulo (`\0`) y el buffer debe tener espacio suficiente.
 * @post Copia byte a byte actualizando `indexW`, `nBytes` y `chk`.
 * @return Checksum acumulado resultante tras encolar la totalidad de la cadena.
 * @see unerPrtcl_PutByteOnTx
 */
uint8_t unerPrtcl_PutStrOntx(_sComm *dataTx, const char *str);

/**
 * @brief Extrae un byte del buffer de recepción aplicando offsets relativos a la posición de datos.
 * @param[in,out] dataRx Puntero a la estructura de recepción `_sComm`.
 * @param[in] start Desplazamiento inicial a sumar sobre `indexData` antes de la lectura.
 * @param[in] end Desplazamiento adicional a sumar sobre `indexData` tras la lectura.
 * @pre La trama debe haber sido decodificada con éxito mediante `unerPrtcl_DecodeHeader`.
 * @post Actualiza el índice interno `indexData` aplicando la máscara circular `mask`.
 * @return Valor del byte leído en la posición indexada resultante.
 * @see unerPrtcl_DecodeHeader
 */
uint8_t unerPrtcl_GetByteFromRx(_sComm *dataRx, uint8_t start, uint8_t end);

/**
 * @brief Ejecuta la máquina de estados de decodificación de cabeceras sobre los datos recibidos.
 * @details Procesa los bytes acumulados entre `indexR` e `indexW`. Verifica el preámbulo "UNER:",
 *          contabiliza los bytes recibidos y valida el Checksum al final del payload.
 *
 * \startuml
 * title Diagrama de Estados: Decodificación unerPrtcl (unerPrtcl_DecodeHeader)
 * [*] --> HEADER_U : Inicio / Reset
 * HEADER_U --> HEADER_N : Byte == 'U'
 * HEADER_N --> HEADER_E : Byte == 'N'
 * HEADER_N --> HEADER_U : Byte != 'N'
 * HEADER_E --> HEADER_R : Byte == 'E'
 * HEADER_E --> HEADER_U : Byte != 'E'
 * HEADER_R --> NBYTES : Byte == 'R'
 * HEADER_R --> HEADER_U : Byte != 'R'
 * NBYTES --> TOKEN : Guarda nBytes
 * TOKEN --> PAYLOAD : Byte == ':'\n(Inicializa Chk XOR)
 * TOKEN --> HEADER_U : Byte != ':'
 *
 * state PAYLOAD {
 *   [*] --> Recibiendo : nBytes > 0
 *   Recibiendo : Chk ^= Byte
 *   Recibiendo --> Recibiendo : nBytes > 0
 *   Recibiendo --> Validacion : nBytes == 0
 *   Validacion : ¿Byte == Chk?
 * }
 *
 * Validacion --> HEADER_U : Checksum OK (retorna TRUE)
 * Validacion --> HEADER_U : Checksum Error (retorna FALSE)
 * \enduml
 *
 * @param[in,out] dataRx Puntero a la estructura de recepción `_sComm`.
 * @pre El buffer debe contener bytes no leídos (`indexR != indexW`).
 * @post Modifica `indexR`, `header`, `indexData` y `chk`. Si retorna TRUE, `indexData` apunta al comando.
 * @retval 1 (TRUE) Trama completa y válida recibida con Checksum verificado.
 * @retval 0 (FALSE) Trama incompleta, datos en proceso o error de sincronización.
 * @see _sComm
 * @see _eDecode
 * @see decodeCommand
 */
uint8_t unerPrtcl_DecodeHeader(_sComm *dataRx);

#endif /* INC_UNERPRTCL_H_ */
