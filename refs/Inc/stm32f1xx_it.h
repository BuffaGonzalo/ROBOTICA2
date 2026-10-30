/**
 * @file    stm32f1xx_it.h
 * @author  STMicroelectronics / Gonzalo M. Buffa
 * @brief   Declaración de manejadores de interrupción (ISRs) del microcontrolador STM32F103.
 * @details Prototipos de las rutinas de servicio de interrupción (ISR) para excepciones del núcleo
 *          Cortex-M3 (SysTick, HardFault, etc.) y periféricos integrados (DMA1, ADC1, TIM2, TIM3, USART1, USB).
 * @ingroup group_system
 */

#ifndef __STM32F1xx_IT_H
#define __STM32F1xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/**
 * @name Excepciones del Núcleo ARM Cortex-M3
 * @{
 */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);

/**
 * @brief Manejador del temporizador del sistema (SysTick).
 * @details Genera la base de tiempo de alta resolución (250 µs) para el planificador y los temporizadores HAL.
 */
void SysTick_Handler(void);
/** @} */

/**
 * @name Interrupciones de Periféricos Hardware STM32
 * @{
 */
/**
 * @brief ISR del Canal 1 de DMA1: Conversión continua multicanal de sensores IR (ADC1).
 */
void DMA1_Channel1_IRQHandler(void);

/**
 * @brief ISR del Canal 4 de DMA1: Transmisión asíncrona de datos I2C2 hacia el display OLED SSD1306.
 */
void DMA1_Channel4_IRQHandler(void);

/**
 * @brief ISR del Canal 5 de DMA1: Recepción asíncrona de datos I2C2 desde la IMU MPU-6050.
 */
void DMA1_Channel5_IRQHandler(void);

void ADC1_2_IRQHandler(void);
void USB_HP_CAN1_TX_IRQHandler(void);
void USB_LP_CAN1_RX0_IRQHandler(void);
void TIM1_BRK_IRQHandler(void);
void TIM1_UP_IRQHandler(void);
void TIM1_TRG_COM_IRQHandler(void);
void TIM1_CC_IRQHandler(void);

/**
 * @brief ISR del Temporizador TIM2: Lazo crítico determinístico de balance PID (5.0 ms / 200 Hz).
 */
void TIM2_IRQHandler(void);

/**
 * @brief ISR del Temporizador TIM3: Modulación por ancho de pulsos PWM para tracción de motores (10 kHz).
 */
void TIM3_IRQHandler(void);

void TIM4_IRQHandler(void);
void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);

/**
 * @brief ISR del puerto USART1: Recepción de bytes serie desde el módulo Wi-Fi ESP-01 (115200 bps).
 */
void USART1_IRQHandler(void);
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
