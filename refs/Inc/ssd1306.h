/**
 * @file   ssd1306.h
 * @author Olivier Van den Eede / Aleksander Alekseev / Gonzalo M. Buffa
 * @brief  Driver de control y primitivas gráficas para display OLED monocromático SSD1306 (128x64).
 * @details Este módulo gestiona el framebuffer de video en memoria RAM (1024 bytes), los comandos
 *          de inicialización y contraste, primitivas 2D (líneas, rectángulos, círculos, bitmaps)
 *          y la transferencia asíncrona no bloqueante de la imagen a través de DMA por bus I2C2.
 * @ingroup group_ui_graphics
 */

#ifndef __SSD1306_H__
#define __SSD1306_H__

#include <stddef.h>
#include <stdint.h>
#include <_ansi.h>

_BEGIN_STD_C

#include "ssd1306_conf.h"

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#else
#include "stm32f1xx_hal.h"
#endif

#ifdef SSD1306_X_OFFSET
#define SSD1306_X_OFFSET_LOWER (SSD1306_X_OFFSET & 0x0F)
#define SSD1306_X_OFFSET_UPPER ((SSD1306_X_OFFSET >> 4) & 0x07)
#else
#define SSD1306_X_OFFSET_LOWER 0
#define SSD1306_X_OFFSET_UPPER 0
#endif

/* Configuración I2C */
#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR        (0x3C << 1)
#endif

#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT          64  /*!< Alto de la pantalla OLED en píxeles */
#endif

#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           128 /*!< Ancho de la pantalla OLED en píxeles */
#endif

#ifndef SSD1306_BUFFER_SIZE
#define SSD1306_BUFFER_SIZE     (SSD1306_WIDTH * SSD1306_HEIGHT / 8) /*!< Tamaño en bytes del framebuffer (1024 bytes) */
#endif

/**
 * @brief Paleta de color binaria para pantalla OLED monocromática.
 */
typedef enum {
    Black = 0x00, /*!< Píxel apagado (Fondo negro) */
    White = 0x01  /*!< Píxel encendido (Color emisor blanco/azul según el display) */
} SSD1306_COLOR;

/**
 * @brief Códigos de error retornados por las operaciones gráficas del SSD1306.
 */
typedef enum {
    SSD1306_OK = 0x00, /*!< Operación completada exitosamente */
    SSD1306_ERR = 0x01 /*!< Error de parámetros, desbordamiento o timeout en bus */
} SSD1306_Error_t;

/**
 * @brief Estructura de estado interno del controlador de display OLED SSD1306.
 * @details Mantiene las coordenadas del cursor gráfico/texto y las banderas de inicialización y encendido.
 * @see SSD1306_COLOR
 * @see SSD1306_Font_t
 * @see ssd1306_Init
 * @see ssd1306_SetCursor
 */
typedef struct {
    uint16_t CurrentX;   /*!< Coordenada horizontal actual del cursor de texto en píxeles (0 a 127) */
    uint16_t CurrentY;   /*!< Coordenada vertical actual del cursor de texto en píxeles (0 a 63) */
    uint8_t Initialized; /*!< Bandera booleana de inicialización exitosa (1: Listo, 0: No inicializado) */
    uint8_t DisplayOn;   /*!< Estado de encendido de la matriz emisora OLED (1: ON, 0: OFF) */
} SSD1306_t;

/**
 * @brief Vértice de coordenadas bidimensionales para dibujo de polilíneas vectoriales.
 * @see ssd1306_Polyline
 */
typedef struct {
    uint8_t x; /*!< Coordenada horizontal del vértice (0 a 127) */
    uint8_t y; /*!< Coordenada vertical del vértice (0 a 63) */
} SSD1306_VERTEX;

/**
 * @brief Estructura descriptora de fuentes tipográficas matriciales almacenadas en FLASH.
 * @see ssd1306_WriteChar
 * @see ssd1306_WriteString
 */
typedef struct {
	const uint8_t width;                /*!< Ancho nominal del carácter en píxeles */
	const uint8_t height;               /*!< Alto nominal del carácter en píxeles */
	const uint16_t *const data;         /*!< Puntero a la tabla binaria de mapa de bits en memoria FLASH */
    const uint8_t *const char_width;    /*!< Tabla de ancho variable para glifos proporcionales (NULL para monoespaciadas) */
} SSD1306_Font_t;

/**
 * @name Funciones de Enlace y Callbacks del Driver
 * @{
 */
/**
 * @brief Asocia el manejador bloqueante de escritura en memoria I2C.
 * @param[in] PtrRx Puntero a la función de transmisión I2C del microcontrolador.
 * @post Configura el puntero de función interno de escritura.
 */
void ssd1306_Attach_MemWrite(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type));

/**
 * @brief Asocia el manejador no bloqueante de escritura DMA en bus I2C.
 * @param[in] PtrRx Puntero a la función de transmisión I2C DMA (`HAL_I2C_Mem_Write_DMA`).
 * @post Configura el puntero de función interno de DMA.
 */
void ssd1306_Attach_MemWriteDMA(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type));

/**
 * @brief Registra la bandera volátil que notifica la finalización de transferencias DMA Tx.
 * @param[in] PtrRx Puntero a la bandera modificada en la ISR del canal DMA.
 * @post Enlaza `SSD1306_TxCplt` con la bandera del sistema.
 */
void ssd1306_ADC_ConfCpltCallback(volatile uint8_t *PtrRx);
/** @} */

/**
 * @name Primitivas de Inicialización y Control
 * @{
 */
/**
 * @brief Inicializa el controlador SSD1306 con la secuencia de encendido y parámetros de display.
 * @details Configura la bomba de carga DC-DC (0x8D), el oscilador interno (0xD5), el multiplexado a 64 filas (0xA8)
 *          y limpia el buffer interno de memoria en RAM.
 * @pre I2C2 inicializado y funciones de enlace configuradas.
 * @post Enciende la pantalla y borra el framebuffer; `SSD1306.Initialized` pasa a 1.
 * @see ssd1306_Attach_MemWrite
 */
void ssd1306_Init(void);

/**
 * @brief Rellena el framebuffer completo de 1024 bytes con un color uniforme (Black o White).
 * @param[in] color Color de relleno (`Black`: 0x00, `White`: 0xFF).
 * @post Modifica los 1024 bytes del array `SSD1306_Buffer`.
 */
void ssd1306_Fill(SSD1306_COLOR color);

/**
 * @brief Vuelca síncronamente el framebuffer completo de video hacia la pantalla OLED.
 * @details Envía las 8 páginas secuencialmente de manera bloqueante por I2C.
 * @pre Display inicializado.
 * @post La imagen del display refleja fielmente el contenido de `SSD1306_Buffer`.
 */
void ssd1306_UpdateScreen(void);

/**
 * @brief Ejecuta el volcado asíncrono no bloqueante del framebuffer vía transferencias DMA.
 * @details Implementa una máquina de 4 estados para enviar las 8 páginas (1024 bytes)
 *          de forma segmentada intercalando comandos de direccionamiento y transferencias DMA de 128 bytes.
 *
 * \startuml
 * title Pipeline de Refresco No Bloqueante DMA OLED (ssd1306_UpdateScreenDMA)
 * start
 * if (¿Flag TxCplt activo O estado == 1?) then (Sí)
 *   :Limpiar bandera *SSD1306_TxCplt = 0;
 *   if (estado == 1) then
 *     :Comando DMA: Página (0xB0 + page);
 *     :estado = 2;
 *   elseif (estado == 2) then
 *     :Comando DMA: Columna baja (0x00 + offset);
 *     :estado = 3;
 *   elseif (estado == 3) then
 *     :Comando DMA: Columna alta (0x10 + offset);
 *     :estado = 4;
 *   elseif (estado == 4) then
 *     :Escribir 128 bytes por DMA de la página actual;
 *     :page++;
 *     :estado = 1;
 *     if (¿page > 7?) then (Fin de frame)
 *       :page = 0;
 *       #palegreen:Retornar 1 (Frame completado);
 *       stop
 *     endif
 *   endif
 * endif
 * #salmon:Retornar 0 (Transferencia en progreso);
 * stop
 * \enduml
 *
 * @return 1 cuando se transmitieron las 8 páginas completas (nuevo cuadro en OLED); 0 mientras continúe transfiriendo.
 * @pre Callback de DMA Tx configurado mediante `ssd1306_ADC_ConfCpltCallback`.
 * @post Actualiza `ssd1306_dma_state` y `ssd1306_dma_current_page`.
 * @see ssd1306_ResetDMAState
 */
char ssd1306_UpdateScreenDMA(void);

/**
 * @brief Reinicia la máquina de estados del secuenciador de volcado DMA.
 * @post Restablece el secuenciador DMA al estado 1 y página 0.
 * @see ssd1306_UpdateScreenDMA
 */
void ssd1306_ResetDMAState(void);

/**
 * @brief Ajusta el nivel de contraste electrónico de los emisores OLED.
 * @param[in] value Valor numérico de contraste (0 a 255).
 * @post Envía el comando 0x81 seguido de `value`.
 */
void ssd1306_SetContrast(const uint8_t value);

/**
 * @brief Enciende o apaga la matriz del panel OLED.
 * @param[in] on 1 para activar el panel (0xAF), 0 para suspenderlo en bajo consumo (0xAE).
 * @post Actualiza `SSD1306.DisplayOn`.
 */
void ssd1306_SetDisplayOn(const uint8_t on);

/**
 * @brief Consulta el estado actual de encendido del panel OLED.
 * @return 1 si el display está emitiendo luz, 0 si está en reposo.
 */
uint8_t ssd1306_GetDisplayOn(void);
/** @} */

/**
 * @name Primitivas Gráficas y Renderizado 2D
 * @{
 */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char ssd1306_WriteChar(char ch, SSD1306_Font_t Font, SSD1306_COLOR color);
char ssd1306_WriteString(char* str, SSD1306_Font_t Font, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
void ssd1306_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
void ssd1306_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color);
void ssd1306_DrawArcWithRadiusLine(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color);
void ssd1306_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR color);
void ssd1306_FillCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color);
void ssd1306_Polyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size, SSD1306_COLOR color);
void ssd1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
void ssd1306_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
SSD1306_Error_t ssd1306_InvertRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void ssd1306_DrawBitmap(uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, SSD1306_COLOR color);
/** @} */

/**
 * @name Primitivas de Comunicación de Bajo Nivel
 * @{
 */
void ssd1306_Reset(void);
void ssd1306_WriteCommand(uint8_t byte);
void ssd1306_WriteCommandDMA(uint8_t byte);
void ssd1306_WriteData(uint8_t* buffer, size_t buff_size);
void ssd1306_WriteDataDMA(uint8_t* buffer, size_t buff_size);
SSD1306_Error_t ssd1306_FillBuffer(uint8_t* buf, uint32_t len);
/** @} */

_END_STD_C

#endif // __SSD1306_H__
