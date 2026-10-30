/**
 * @file   mpu6050.h
 * @author Gonzalo M. Buffa
 * @date   24/05/2025
 * @brief  Driver de configuración y adquisición para la Unidad de Medición Inercial MPU-6050.
 * @details Este módulo gestiona la inicialización de la IMU MPU-6050 sobre el bus I2C,
 *          configurando el muestreo interno a 200 Hz y el filtro digital pasabajos (DLPF) a 98 Hz.
 *          La lectura de los 14 registros continuos de aceleración y velocidad angular se realiza
 *          de forma no bloqueante mediante transferencias DMA directas a memoria.
 * @ingroup group_sensors
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include <stddef.h>

/**
 * @defgroup MPU6050_Registers Mapa de Registros del MPU-6050
 * @ingroup group_sensors
 * @{
 */
#define MPU6050_ADDR         0xD0  /*!< Dirección I2C del sensor en 8 bits (AD0 = 0 -> 0x68 << 1 = 0xD0) */
#define WHO_AM_I_REG         0x75  /*!< Registro de identidad del dispositivo (valor esperado: 0x68) */
#define PWR_MGMT_1_REG       0x6B  /*!< Registro de gestión de energía y modo sleep */
#define SMPLRT_DIV_REG       0x19  /*!< Divisor de la tasa de muestreo interno (Sample Rate Divider) */
#define CONFIG_REG           0x1A  /*!< Configuración del filtro digital pasa-bajos (DLPF) */
#define GYRO_CONFIG_REG      0x1B  /*!< Configuración de escala y rango del giróscopo */
#define ACCEL_CONFIG_REG     0x1C  /*!< Configuración de escala y rango del acelerómetro */
#define ACCEL_XOUT_H_REG     0x3B  /*!< Dirección base de lectura de aceleración (X_HIGH, 14 bytes continuos) */
#define GYRO_XOUT_H_REG      0x43  /*!< Dirección base de lectura de velocidad angular (X_HIGH) */
/** @} */

/**
 * @defgroup MPU6050_Calibration Offsets Digitales y Factores de Escala
 * @ingroup group_sensors
 * @{
 */
#define OFFSET_AX            450   /*!< Offset de calibración estática para acelerómetro eje X */
#define OFFSET_AY            450   /*!< Offset de calibración estática para acelerómetro eje Y */
#define OFFSET_AZ            450   /*!< Offset de calibración estática para acelerómetro eje Z */
#define OFFSET_GX            450   /*!< Offset de calibración estática para giróscopo eje X */
#define OFFSET_GY            350   /*!< Offset de calibración estática para giróscopo eje Y */
#define OFFSET_GZ            350   /*!< Offset de calibración estática para giróscopo eje Z */

#define MULTIPLICADORFLOAT   100   /*!< Factor de escala en punto fijo (x100) para dos cifras decimales enteras */
/** @} */

/**
 * @brief Inicializa el MPU-6050 con los parámetros de funcionamiento del robot.
 * @details Despierta al sensor sacándolo del modo sleep (PWR_MGMT_1 = 0), configura el DLPF
 *          a 98 Hz (CONFIG = 0x02), establece la tasa de muestreo interno a 200 Hz (SMPLRT_DIV = 4),
 *          y fija los rangos dinámicos en $\pm 2g$ para aceleración y $\pm 250^\circ/\text{s}$ para rotación.
 *
 * \startuml
 * title Secuencia de Inicialización MPU-6050 (mpu6050_Init)
 * participant "STM32 (I2C Master)" as MCU
 * participant "MPU-6050 (0xD0)" as IMU
 * MCU -> IMU : PWR_MGMT_1 (0x6B) = 0x00 (Wake up)
 * MCU -> IMU : SMPLRT_DIV (0x19) = 0x04 (200 Hz sample rate)
 * MCU -> IMU : CONFIG (0x1A) = 0x02 (DLPF ~98 Hz)
 * MCU -> IMU : ACCEL_CONFIG (0x1C) = 0x00 (+/- 2g)
 * MCU -> IMU : GYRO_CONFIG (0x1B) = 0x00 (+/- 250 deg/s)
 * \enduml
 *
 * @pre El periférico I2C2 del microcontrolador debe estar inicializado y la función `mpu6050_Attach_MemWrite` vinculada.
 * @post Configura registros del sensor; el MPU-6050 queda operativo entregando datos inerciales a 200 Hz.
 * @see mpu6050_Attach_MemWrite
 * @see mpu6050_Read
 */
void mpu6050_Init(void);

/**
 * @brief Ejecuta el ciclo de lectura no bloqueante de 14 bytes por DMA.
 * @details Implementa una máquina de dos estados:
 *          - Estado 1: Dispara la lectura DMA de 14 bytes contiguos desde `ACCEL_XOUT_H_REG`.
 *          - Estado 2: Recompone los pares de bytes recibidos (Big-Endian) en valores con signo de 16 bits.
 *
 * \startuml
 * title Pipeline de Adquisición No Bloqueante I2C DMA (mpu6050_Read)
 * start
 * if (mpu_state == 1?) then (Estado 1: Disparo DMA)
 *   :Disparar MemReadDMA(14 bytes, 0x3B);
 *   :mpu_state = 2;
 *   #salmon:Retornar 0 (Lectura en curso);
 * elseif (Flag RxCplt == 1?) then (Datos en RAM)
 *   :Limpiar flag RxCplt = 0;
 *   :Recomponer Ax, Ay, Az (Big-Endian);
 *   :Recomponer Gx, Gy, Gz (Big-Endian);
 *   :mpu_state = 1;
 *   #palegreen:Retornar 1 (Nuevos datos listos);
 * else (En espera de DMA)
 *   #salmon:Retornar 0 (Pendiente);
 * endif
 * stop
 * \enduml
 *
 * @pre Debe estar asignado el callback DMA mediante `mpu6050_ADC_ConfCpltCallback` y el despachador `mpu6050_Attach_MemReadDMA`.
 * @post Actualiza las variables estáticas internas `ax, ay, az, gx, gy, gz`.
 * @retval 1 Lectura completa, datos actualizados y listos.
 * @retval 0 Transferencia DMA en curso o bus no disponible.
 * @see mpu6050_GetData
 * @see mpu6050_Reset_State
 * @see PIDTask
 */
char mpu6050_Read(void);

/**
 * @brief Vincula la función de callback que indica la finalización de recepción DMA.
 * @param[in] PtrRx Puntero a la bandera volátil que se activa en la interrupción DMA Rx.
 * @pre Variable global o estática de bandera volátil instanciada en el módulo de control.
 * @post Almacena el puntero interno `mpu6050_RxCplt`.
 * @see HAL_I2C_MemRxCpltCallback
 */
void mpu6050_ADC_ConfCpltCallback(volatile uint8_t *PtrRx);

/**
 * @brief Conecta el puntero a la función de lectura por DMA del bus I2C del microcontrolador.
 * @param[in] PtrRx Puntero a la función despachadora de lectura I2C DMA implementada en el main.
 * @post Asigna el puntero de función `memReadDMA`.
 * @see mpu6050_Read
 */
void mpu6050_Attach_MemReadDMA(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type));

/**
 * @brief Conecta el puntero a la función de escritura en registros I2C.
 * @param[in] PtrRx Puntero a la función de encolado de escrituras I2C.
 * @post Asigna el puntero de función `memWrite`.
 * @see mpu6050_Init
 */
void mpu6050_Attach_MemWrite(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type));

/**
 * @brief Reinicia el estado interno del secuenciador de lectura DMA del sensor.
 * @post Restaura `mpu_state = 1` y limpia la bandera de finalización de DMA si está vinculada.
 * @see mpu6050_Read
 */
void mpu6050_Reset_State(void);

/**
 * @brief Copia los últimos datos crudos de 16 bits leídos del acelerómetro y giróscopo.
 * @param[out] ax Puntero donde se almacenará la aceleración cruda en eje X.
 * @param[out] ay Puntero donde se almacenará la aceleración cruda en eje Y.
 * @param[out] az Puntero donde se almacenará la aceleración cruda en eje Z.
 * @param[out] gx Puntero donde se almacenará la velocidad angular cruda en eje X.
 * @param[out] gy Puntero donde se almacenará la velocidad angular cruda en eje Y.
 * @param[out] gz Puntero donde se almacenará la velocidad angular cruda en eje Z.
 * @pre Haber llamado a `mpu6050_Read()` retornando valor 1.
 * @post Escribe los valores inerciales en los punteros provistos (si no son NULL).
 * @see mpu6050_Read
 * @see PIDTask
 */
void mpu6050_GetData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

#endif /* INC_MPU6050_H_ */
