/**
 * @file   fonts.h
 * @author Olivier Van den Eede / Aleksander Alekseev / Gonzalo M. Buffa
 * @brief  Declaración de fuentes tipográficas matriciales para display OLED SSD1306.
 * @details Proporciona las estructuras de fuentes externas de ancho fijo (`Font_7x10` y `Font_11x18`)
 *          para su renderizado en pantalla sin consumo dinámico de memoria RAM.
 * @ingroup group_ui_graphics
 */

#ifndef __SSD1306_FONTS_H__
#define __SSD1306_FONTS_H__

#include "ssd1306.h"

#ifdef SSD1306_INCLUDE_FONT_6x8
extern const SSD1306_Font_t Font_6x8;
#endif

#ifdef SSD1306_INCLUDE_FONT_7x10
extern const SSD1306_Font_t Font_7x10; /*!< Tipografía estándar compacta de 7x10 píxeles */
#endif

#ifdef SSD1306_INCLUDE_FONT_11x18
extern const SSD1306_Font_t Font_11x18; /*!< Tipografía mediana de 11x18 píxeles para títulos y números */
#endif

#ifdef SSD1306_INCLUDE_FONT_16x26
extern const SSD1306_Font_t Font_16x26;
#endif

#ifdef SSD1306_INCLUDE_FONT_16x24
extern const SSD1306_Font_t Font_16x24;
#endif

#ifdef SSD1306_INCLUDE_FONT_16x15
extern const SSD1306_Font_t Font_16x15;
#endif

#endif // __SSD1306_FONTS_H__
