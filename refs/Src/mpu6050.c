/**
 * @file   mpu6050.c
 * @author Gonzalo M. Buffa
 * @date   24/05/2025
 * @brief  Implementación de las rutinas de lectura I2C por DMA del MPU-6050.
 * @details Este archivo implementa el puente de software entre la IMU MPU-6050 y el hardware
 *          I2C2 del STM32F103, gestionando las transferencias DMA para la lectura de 14 bytes
 *          y la reconstrucción de los enteros con signo de 16 bits sin bloquear la CPU.
 * @ingroup group_sensors
 */

#include "mpu6050.h"
#include "main.h"
#include <stdlib.h>

/**
 * @name Variables Estáticas Internas del Driver
 * @{
 */
static uint8_t *mpu6050_RxCplt = NULL; /*!< Puntero a la bandera de finalización de DMA Rx */
static void (*memWrite)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type) = NULL; /*!< Puntero a función de escritura en cola I2C */
static void (*memReadDMA)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type) = NULL; /*!< Puntero a función de lectura DMA */

static int32_t ax, ay, az, gx, gy, gz; /*!< Variables estáticas con los últimos valores inerciales reconstruidos */
static uint8_t mpu_state = 1;          /*!< Estado de la MEF de lectura DMA (1: Lanzar petición DMA, 2: Recomponer bytes) */
/** @} */

/**
 * @brief Asigna la bandera de interrupción DMA de recepción.
 * @param[in] PtrRx Puntero a la variable booleana que se activa en `HAL_I2C_MemRxCpltCallback`.
 * @pre El puntero debe apuntar a una variable con cualificador `volatile`.
 * @post Almacena internamente el puntero `mpu6050_RxCplt`.
 * @see HAL_I2C_MemRxCpltCallback
 */
void mpu6050_ADC_ConfCpltCallback(volatile uint8_t *PtrRx){
	mpu6050_RxCplt = (uint8_t *)PtrRx;
}

/**
 * @brief Asocia el manejador de escritura para registros I2C.
 * @param[in] PtrRx Puntero a la función de transmisión I2C bloqueante o encolada.
 * @post Configura `memWrite`.
 * @see mpu6050_WriteData
 */
void mpu6050_Attach_MemWrite(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type)){
	memWrite = PtrRx;
}

/**
 * @brief Asocia el manejador de lectura DMA sobre el bus I2C.
 * @param[in] PtrRx Puntero a la función de lectura por DMA (`HAL_I2C_Mem_Read_DMA`).
 * @post Configura `memReadDMA`.
 * @see mpu6050_ReadDataDMA
 */
void mpu6050_Attach_MemReadDMA(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type)){
	memReadDMA = PtrRx;
}

/**
 * @brief Envía un comando de escritura simple a un registro del sensor MPU-6050.
 * @param[in] byte Puntero al byte de datos a escribir.
 * @param[in] type Registro de destino en el mapa de memoria del sensor.
 * @pre `memWrite` debe estar configurado.
 * @see mpu6050_Attach_MemWrite
 */
static void mpu6050_WriteData(uint8_t *byte, uint8_t type) {
	memWrite(MPU6050_ADDR, byte, 1, type);
}

/**
 * @brief Inicia una lectura por DMA de longitud parametrizable.
 * @param[out] buffer Puntero de destino donde el controlador DMA volcará los bytes recibidos.
 * @param[in] size Cantidad de bytes a transferir.
 * @param[in] type Dirección del registro de inicio en el sensor.
 * @pre `memReadDMA` debe estar asignado y el canal DMA libre.
 * @see mpu6050_Attach_MemReadDMA
 */
static void mpu6050_ReadDataDMA(uint8_t* buffer, size_t size, uint8_t type) {
	memReadDMA(MPU6050_ADDR, buffer, size, type);
}

/**
 * @brief Inicializa los registros fundamentales del MPU-6050 para el lazo de 200 Hz.
 * @details Configuración aplicada sobre el sensor:
 *          - `PWR_MGMT_1` (0x6B) = 0x00: Despierta el oscilador interno.
 *          - `SMPLRT_DIV` (0x19) = 0x04: Tasa de muestreo = 1000Hz / (1 + 4) = 200 Hz (período de 5 ms).
 *          - `CONFIG` (0x1A) = 0x02: Filtro pasa-bajos DLPF ~98 Hz (retardo mínimo 2.8 ms).
 *          - `ACCEL_CONFIG` (0x1C) = 0x00: Rango de aceleración $\pm 2g$ (16384 LSB/g).
 *          - `GYRO_CONFIG` (0x1B) = 0x00: Rango de giróscopo $\pm 250^\circ/\text{s}$ (131 LSB/(°/s)).
 * @pre El bus I2C2 debe estar inicializado y `memWrite` enlazado.
 * @post El sensor queda activo y configurado a 200 Hz con DLPF anti-vibración a 98 Hz.
 * @see mpu6050_WriteData
 * @see mpu6050_Read
 */
void mpu6050_Init(void)
{
    uint8_t data;

    // 1. Salir del modo de bajo consumo (modo sleep)
    data = 0x00;
    mpu6050_WriteData(&data, PWR_MGMT_1_REG);

    // 2. Configurar Sample Rate Divider a 4 (Muestreo a 200 Hz / 5 ms sincronizado con STM32)
    data = 0x04;
    mpu6050_WriteData(&data, SMPLRT_DIV_REG);

    // 3. Activar filtro pasa-bajos DLPF a ~98Hz para filtrar vibraciones mecánicas de los motores
	data = 0x02;
	mpu6050_WriteData(&data, CONFIG_REG);

    // 4. Configurar acelerómetro con rango +-2g
    data = 0x00;
    mpu6050_WriteData(&data, ACCEL_CONFIG_REG);

    // 5. Configurar giróscopo con rango +-250 deg/s
    data = 0x00;
    mpu6050_WriteData(&data, GYRO_CONFIG_REG);
}

/**
 * @brief Resetea la secuencia de lectura DMA en caso de error o reinicio de estado.
 * @post `mpu_state` vuelve a 1 y si la bandera DMA Rx no es NULL, se limpia a 0.
 * @see mpu6050_Read
 */
void mpu6050_Reset_State(void) {
	mpu_state = 1;
	if (mpu6050_RxCplt != NULL) {
		*mpu6050_RxCplt = 0;
	}
}

/**
 * @brief Máquina de estados no bloqueante para la captura de 14 bytes inerciales continuos.
 * @details El proceso se divide en dos fases para sincronizarse con la interrupción DMA:
 *          - En Fase 1: Dispara la lectura de 14 bytes por DMA desde `ACCEL_XOUT_H_REG` (0x3B).
 *          - En Fase 2: Tras recibir la señal de `mpu6050_RxCplt`, recompone los registros:
 *            `Ax`, `Ay`, `Az`, `Temperatura`, `Gx`, `Gy`, `Gz` desplazando el byte alto 8 bits.
 * @return 1 si los datos fueron recibidos y ensamblados exitosamente; 0 si la transferencia sigue en curso.
 * @pre Función `memReadDMA` asociada y bus libre.
 * @post Al completarse (retorno 1), actualiza variables estáticas `ax, ay, az, gx, gy, gz`.
 * @see mpu6050_GetData
 * @see mpu6050_Reset_State
 * @see PIDTask
 */
char mpu6050_Read(void)
{
	static uint8_t Rec_Data[14];

	if (*mpu6050_RxCplt || mpu_state == 1) {
		*mpu6050_RxCplt = 0;  // Reset de la bandera de recepción DMA
		switch (mpu_state) {
		case 1:
			mpu_state = 2;
			// Lanzar solicitud DMA para leer los 14 registros de golpe (Ax, Ay, Az, Temp, Gx, Gy, Gz)
			mpu6050_ReadDataDMA(Rec_Data, 14, ACCEL_XOUT_H_REG);
			break;
		case 2:
			mpu_state = 1;
			// Combinar bytes MSB y LSB en enteros de 16 bits con signo (formato Big-Endian del sensor)
			ax = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
			ay = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
			az = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

			gx = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
			gy = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
			gz = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

			return 1; // Nuevos datos disponibles
			break;
		}
	}
	return 0; // Lectura en proceso
}

/**
 * @brief Entrega las variables crudas leídas al lazo principal de control.
 * @param[out] ax_out Puntero a variable receptora de aceleración en X.
 * @param[out] ay_out Puntero a variable receptora de aceleración en Y.
 * @param[out] az_out Puntero a variable receptora de aceleración en Z.
 * @param[out] gx_out Puntero a variable receptora de giróscopo en X.
 * @param[out] gy_out Puntero a variable receptora de giróscopo en Y.
 * @param[out] gz_out Puntero a variable receptora de giróscopo en Z.
 * @pre `mpu6050_Read()` debe haber retornado 1.
 * @post Vuelca los valores almacenados en memoria estática a los punteros proporcionados.
 * @see mpu6050_Read
 * @see PIDTask
 */
void mpu6050_GetData(int16_t *ax_out, int16_t *ay_out, int16_t *az_out, int16_t *gx_out, int16_t *gy_out, int16_t *gz_out) {
    if (ax_out) *ax_out = ax;
    if (ay_out) *ay_out = ay;
    if (az_out) *az_out = az;

    if (gx_out) *gx_out = gx;
    if (gy_out) *gy_out = gy;
    if (gz_out) *gz_out = gz;
}
