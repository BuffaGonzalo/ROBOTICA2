/**
 * @file   main.h
 * @author Gonzalo M. Buffa / STMicroelectronics
 * @brief  Cabecera principal de la aplicación, constantes de hardware y prototipos del sistema.
 * @details Define las asignaciones de pines GPIO (LED indicador de Heartbeat, botón de usuario,
 *          pin de habilitación del módulo Wi-Fi ESP-01) y prototipos de controladores de error.
 * @ingroup group_main
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/**
 * @brief Configura los canales PWM de los temporizadores tras la inicialización del hardware.
 * @param[in,out] htim Puntero a la estructura de manejador del temporizador TIM.
 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/**
 * @brief Manejador central de fallas críticas de inicialización de periféricos HAL.
 * @details Entra en un bucle infinito de bloqueo en caso de fallo irrecuperable de reloj o bus.
 */
void Error_Handler(void);

/**
 * @defgroup Hardware_Pinout Asignaciones de Pines GPIO de la Placa
 * @ingroup group_main
 * @{
 */
#define LED_Pin           GPIO_PIN_13   /*!< Pin del LED indicador integrado (Heartbeat activo a nivel bajo) */
#define LED_GPIO_Port     GPIOC         /*!< Puerto GPIO del LED indicador (Puerto C) */
#define SW0_Pin           GPIO_PIN_15   /*!< Pin del pulsador multifunción de usuario */
#define SW0_GPIO_Port     GPIOB         /*!< Puerto GPIO del pulsador de usuario (Puerto B) */
#define ESP01_EN_Pin      GPIO_PIN_9    /*!< Pin de habilitación (CH_PD / EN) del módulo Wi-Fi ESP-01 */
#define ESP01_EN_GPIO_Port GPIOB        /*!< Puerto GPIO de control del módulo ESP-01 (Puerto B) */
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
