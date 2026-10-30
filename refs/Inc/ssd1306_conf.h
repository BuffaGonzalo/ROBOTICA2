/**
 * @file   ssd1306_conf.h
 * @author Olivier Van den Eede / Aleksander Alekseev / Gonzalo M. Buffa
 * @brief  Archivo de configuración estática para el driver de pantalla OLED SSD1306.
 * @details Selecciona la familia de microcontrolador (STM32F1), el bus de comunicaciones (I2C a dirección 0x3C),
 *          y las tipografías habilitadas en memoria Flash (Font_7x10 y Font_11x18) para minimizar el uso de ROM.
 * @ingroup group_ui_graphics
 */

#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

// Selección de familia de microcontrolador
#define STM32F1

// Selección de bus de comunicación
#define SSD1306_USE_I2C

// Configuración de dirección I2C (7 bits desplazados: 0x3C << 1 = 0x78)
#define SSD1306_I2C_ADDR        (0x3C << 1)

// Inclusión selectiva de tipografías para optimizar memoria FLASH
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18

#endif /* __SSD1306_CONF_H__ */
