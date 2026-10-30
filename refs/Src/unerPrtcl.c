/**
 * @file   unerPrtcl.c
 * @author Gonzalo M. Buffa
 * @date   24/05/2025
 * @brief  Implementación de las rutinas del protocolo binario UNER (unerPrtcl).
 * @details Este archivo contiene las funciones de serialización, cálculo de Checksum por XOR,
 *          gestión de punteros circulares e implementación de la máquina de estados finita
 *          para la detección y validación de tramas de telemetría y telemando.
 * @ingroup group_comm
 */

#include "util.h"
#include <stdint.h>
#include <unerPrtcl.h>
#include "main.h"

/**
 * @brief Inicializa los descriptores de buffer circular de comunicación.
 * @details Asigna los arrays de memoria física, resetea índices de lectura/escritura a cero
 *          y calcula la máscara de truncamiento circular `(BUFFER_SIZE - 1)`.
 * @param[out] Rx Puntero al descriptor de recepción `_sComm`.
 * @param[out] Tx Puntero al descriptor de transmisión `_sComm`.
 * @param[in] buffRx Array de memoria física para recepción.
 * @param[in] buffTx Array de memoria física para transmisión.
 * @pre Los punteros a buffer deben apuntar a bloques contiguos de tamaño potencia de 2.
 * @post Todos los índices se resetean a cero y el parser se sitúa en `HEADER_U`.
 * @see _sComm
 * @see _eDecode
 */
void unerPrtcl_Init(_sComm *Rx, _sComm *Tx, volatile uint8_t *buffRx, volatile uint8_t *buffTx){
	Rx->buff = (uint8_t *)buffRx;
    Rx->indexR = 0;
    Rx->indexW = 0;
    Rx->indexData = 0;
    Rx->nBytes = 0;
    Rx->header = HEADER_U;
    Rx->mask = RXBUFSIZE - 1; // Control de buffer circular por máscara bit a bit (2^N - 1)
    Rx->chk = 0;

    Tx->buff = (uint8_t *)buffTx;
    Tx->indexR = 0;
    Tx->indexW = 0;
    Tx->indexData = 0;
    Tx->nBytes = 0;
    Tx->header = HEADER_U;
    Tx->mask = TXBUFSIZE - 1;
    Tx->chk = 0;
}

/**
 * @brief Escribe el encabezado estructurado del protocolo UNER en el buffer de transmisión.
 * @details Secuencia inyectada:
 *          1. Sync chars: 'U', 'N', 'E', 'R'
 *          2. Longitud total: frameLength + 1
 *          3. Delimitador: ':'
 *          4. Identificador de comando: ID
 *          Calcula el Checksum preliminar mediante XOR acumulado sobre todos los campos.
 * @param[in,out] dataTx Puntero a la estructura de transmisión `_sComm`.
 * @param[in] ID Identificador del comando según `_eCmd`.
 * @param[in] frameLength Cantidad de bytes en la carga útil (sin contar cabecera).
 * @return Checksum XOR calculado para la cabecera.
 * @pre El buffer `dataTx` debe estar inicializado y poseer al menos 7 bytes disponibles.
 * @post `indexW` avanza 7 bytes en el buffer circular y `dataTx->chk` contiene el XOR parcial.
 * @see unerPrtcl_PutByteOnTx
 * @see unerPrtcl_PutStrOntx
 */
uint8_t unerPrtcl_PutHeaderOnTx(_sComm  *dataTx, uint8_t ID, uint8_t frameLength)
{
	frameLength++;
    dataTx->chk = 0;
    dataTx->indexData = dataTx->indexW;

    dataTx->buff[dataTx->indexW++]='U';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='N';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='E';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='R';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=frameLength;
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=':';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=ID;
    dataTx->indexW &= dataTx->mask;

    dataTx->nBytes = TXBYTES;
    dataTx->chk ^= ('U' ^'N' ^'E' ^'R' ^frameLength ^':'^ID) ;

    return  dataTx->chk;
}

/**
 * @brief Inserta un byte de datos en el buffer de transmisión y actualiza el Checksum.
 * @param[in,out] dataTx Puntero a la estructura de transmisión `_sComm`.
 * @param[in] byte Valor numérico del byte a transmitir.
 * @return Checksum acumulado resultante tras la operación XOR con `byte`.
 * @pre Buffer `dataTx` con espacio libre.
 * @post `nBytes` se incrementa en 1, `indexW` avanza y `chk` acumula `byte`.
 * @see unerPrtcl_PutHeaderOnTx
 */
uint8_t unerPrtcl_PutByteOnTx(_sComm *dataTx, uint8_t byte)
{
	dataTx->nBytes++;
    dataTx->buff[dataTx->indexW++]=byte;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= byte;
    return dataTx->chk;
}

/**
 * @brief Inserta una cadena de texto en el buffer de salida hasta encontrar el terminador nulo.
 * @param[in,out] dataTx Puntero a la estructura de transmisión `_sComm`.
 * @param[in] str Cadena de caracteres a encolar.
 * @return Checksum final acumulado tras procesar la cadena completa.
 * @pre `str` debe ser un puntero no nulo terminado en `\0`.
 * @post Todos los caracteres de `str` son transferidos al buffer circular actualizando el checksum.
 * @see unerPrtcl_PutByteOnTx
 */
uint8_t unerPrtcl_PutStrOntx(_sComm *dataTx, const char *str)
{
    volatile uint8_t globalIndex=0;
    while(str[globalIndex]){
    	dataTx->nBytes++;
        dataTx->buff[dataTx->indexW++]=str[globalIndex];
        dataTx->indexW &= dataTx->mask;
        dataTx->chk ^= str[globalIndex++];
    }
    return dataTx->chk;
}

/**
 * @brief Lee un byte del buffer de recepción aplicando índices de desplazamiento seguro.
 * @param[in,out] dataRx Puntero a la estructura de recepción `_sComm`.
 * @param[in] start Desplazamiento inicial antes de la lectura.
 * @param[in] end Desplazamiento a aplicar luego de la lectura.
 * @return Byte leído de la posición solicitada en el buffer de recepción.
 * @pre Trama decodificada previamente mediante `unerPrtcl_DecodeHeader`.
 * @post `indexData` queda modificado sumándole `start + end` (con máscara circular).
 * @see unerPrtcl_DecodeHeader
 */
uint8_t unerPrtcl_GetByteFromRx(_sComm *dataRx, uint8_t start, uint8_t end) {
	uint8_t getByte;
	dataRx->indexData += start;
	dataRx->indexData &= dataRx->mask;
	getByte = dataRx->buff[dataRx->indexData];
	dataRx->indexData += end;
	dataRx->indexData &= dataRx->mask;
	return getByte;
}

/**
 * @brief Decodifica secuencialmente la cabecera y el payload de tramas entrantes.
 * @details Implementa una máquina de estados para detectar el preámbulo "UNER:".
 *          Una vez localizado el delimitador ':', extrae el payload restando bytes
 *          hasta llegar al último, el cual debe coincidir con el checksum acumulado.
 *
 * \startuml
 * title Proceso de Decodificación de Trama unerPrtcl (unerPrtcl_DecodeHeader)
 * start
 * :Obtener índice de escritura actual auxIndex = dataRx->indexW;
 * while (¿Hay bytes por procesar? (indexR != auxIndex)) is (Sí)
 *   :Leer byte de entrada dataRx->buff[indexR];
 *   :Evaluar estado del protocolo (dataRx->header);
 *   :Procesar transición de cabecera ('U' -> 'N' -> 'E' -> 'R' -> ':');
 *   :Calcular Checksum XOR acumulado sobre payload;
 *   :Avanzar indexR;
 * endwhile (Buffer vacío)
 * if (¿Checksum coincide y payload completo?) then (Sí)
 *   #palegreen:Retornar TRUE (1);
 * else (No)
 *   #salmon:Retornar FALSE (0);
 * endif
 * stop
 * \enduml
 *
 * @param[in,out] dataRx Puntero a la estructura de recepción `_sComm`.
 * @return 1 si se completó una trama con checksum válido, 0 en caso contrario.
 * @pre Bytes disponibles en el buffer circular.
 * @post `indexR` avanza hasta consumir la trama o los datos actuales. `indexData` apunta al comando.
 * @see _sComm
 * @see _eDecode
 * @see decodeCommand
 */
uint8_t unerPrtcl_DecodeHeader(_sComm *dataRx)
{
    uint8_t auxIndex=dataRx->indexW;

    // Procesar todos los bytes acumulados en el buffer circular
    while(dataRx->indexR != auxIndex){
        switch(dataRx->header)
        {
            // Sincronismo 1: Detección de 'U'
            case HEADER_U:
                if(dataRx->buff[dataRx->indexR] == 'U'){
                	dataRx->header = HEADER_N;
                }
            break;
            // Sincronismo 2: Detección de 'N'
            case HEADER_N:
                if(dataRx->buff[dataRx->indexR] == 'N'){
                	dataRx->header = HEADER_E;
                }else{
                    if(dataRx->buff[dataRx->indexR] != 'U'){
                    	dataRx->header = HEADER_U;
                        dataRx->indexR--; // Retroceder para no perder una 'U' contigua
                    }
                }
            break;
            // Sincronismo 3: Detección de 'E'
            case HEADER_E:
                if(dataRx->buff[dataRx->indexR] == 'E'){
                	dataRx->header = HEADER_R;
                }else{
                	dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            // Sincronismo 4: Detección de 'R'
            case HEADER_R:
                if(dataRx->buff[dataRx->indexR] == 'R'){
                	dataRx->header = NBYTES;
                }else{
                	dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            // Lectura de la cantidad de bytes que componen el paquete
            case NBYTES:
                dataRx->nBytes=dataRx->buff[dataRx->indexR];
                dataRx->header = TOKEN;
            break;
            // Delimitador de inicio de comando ':'
            case TOKEN:
                if(dataRx->buff[dataRx->indexR] == ':'){
                	dataRx->header = PAYLOAD;
                    dataRx->indexData = dataRx->indexR+1;
                    dataRx->indexData &= dataRx->mask;
                    // Inicializar acumulador de Checksum con el preámbulo validado
                    dataRx->chk = 0;
                    dataRx->chk ^= ('U' ^'N' ^'E' ^'R' ^ dataRx->nBytes ^':') ;
                }else{
                	dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            // Recepción del payload y validación de Checksum final
            case PAYLOAD:
            	dataRx->nBytes--;
                if(dataRx->nBytes>0){
                   dataRx->chk ^= dataRx->buff[dataRx->indexR];
                }else{
                	dataRx->header = HEADER_U;
                    // Comprobación de integridad
                    if(dataRx->buff[dataRx->indexR] == dataRx->chk)
                        return TRUE; // Trama completa y válida
                }
            break;
            default:
            	dataRx->header = HEADER_U;
            break;
        }
        dataRx->indexR++;
        dataRx->indexR &= dataRx->mask;
    }
    return FALSE; // Trama incompleta
}
