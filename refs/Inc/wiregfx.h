/**
 * @file    wiregfx.h
 * @author  Gonzalo M. Buffa
 * @date    28/04/2026
 * @brief   Motor gráfico wireframe 3D/4D en punto fijo para pantalla OLED SSD1306.
 * @details Este módulo implementa la proyección geométrica en perspectiva y rotaciones
 *          tridimensionales y tetradimensionales (Hipercubo o Teseracto) sobre el framebuffer
 *          monocromático de 128x64 píxeles. Todas las transformaciones se computan mediante
 *          tablas de búsqueda de senos y cosenos (LUT) escaladas a 7 bits con divisiones
 *          por desplazamiento binario (`>> 7`) para prescindir de la FPU.
 * @ingroup group_ui_graphics
 */

#ifndef INC_WIREGFX_H_
#define INC_WIREGFX_H_

#include "ssd1306.h"

/**
 * @defgroup WireGFX_Config Parámetros de Centro y Perspectiva
 * @ingroup group_ui_graphics
 * @{
 */
#define CENTER_X 64 /*!< Coordenada X del centro geométrico en pantalla (128 / 2) */
#define CENTER_Y 32 /*!< Coordenada Y del centro geométrico en pantalla (64 / 2) */
/** @} */

/**
 * @brief Coordenada espacial tridimensional en punto fijo con enteros de 16 bits.
 * @details Modela las coordenadas de vértices para figuras tridimensionales (Cubo, Pirámide, Esfera).
 * @see Point2D
 * @see Point4D
 * @see WIREGFX_Graphics_DrawCube
 * @see WIREGFX_Graphics_DrawPyramid
 */
typedef struct {
    int16_t x; /*!< Coordenada espacial en eje horizontal X */
    int16_t y; /*!< Coordenada espacial en eje vertical Y */
    int16_t z; /*!< Coordenada espacial en eje de profundidad Z */
} Point3D;

/**
 * @brief Coordenada proyectada sobre el plano bidimensional del display OLED (128x64).
 * @details Almacena el resultado de la transformación afín y proyección en perspectiva.
 * @see Point3D
 * @see Point4D
 * @see ssd1306_Line
 */
typedef struct {
    int16_t x; /*!< Coordenada horizontal proyectada en píxeles (0 a 127) */
    int16_t y; /*!< Coordenada vertical proyectada en píxeles (0 a 63) */
} Point2D;

/**
 * @brief Coordenada espacial tetradimensional (4D) para geometrías hiperespaciales.
 * @details Modela vértices en un espacio $\mathbb{R}^4$ con un cuarto eje espacial $W$.
 * @see Point3D
 * @see Point2D
 * @see WIREGFX_Graphics_DrawTesseract
 */
typedef struct {
    int16_t x; /*!< Coordenada espacial en eje X */
    int16_t y; /*!< Coordenada espacial en eje Y */
    int16_t z; /*!< Coordenada espacial en eje Z */
    int16_t w; /*!< Coordenada hiperespacial en la cuarta dimensión W */
} Point4D;

/**
 * @brief Renderiza un cubo 3D rotando sobre los ejes X, Y y Z en la pantalla OLED.
 * @details Aplica matrices de rotación trigonométrica computadas mediante la tabla `sin_LUT`
 *          y proyecta los 8 vértices y 12 aristas en el framebuffer del SSD1306.
 *
 * \startuml
 * title Pipeline Gráfico 3D (WIREGFX_Graphics_DrawCube)
 * start
 * :Leer vértices locales cube_vertices[8];
 * :Consultar sin_x, cos_x, sin_y, cos_y, sin_z, cos_z de sin_LUT[];
 * :Borrar pantalla ssd1306_Fill(Black);
 * while (i < 8) is (Vértice pendiente)
 *   :Rotar en X: y' = (y*cos - z*sin) >> 7, z' = (y*sin + z*cos) >> 7;
 *   :Rotar en Y: x' = (x*cos + z*sin) >> 7, z' = (-x*sin + z*cos) >> 7;
 *   :Rotar en Z: x' = (x*cos - y*sin) >> 7, y' = (x*sin + y*cos) >> 7;
 *   :Proyectar a pantalla: projected[i].x = x + 64, projected[i].y = y + 32;
 *   :i++;
 * endwhile
 * while (k < 12) is (Arista pendiente)
 *   :Dibujar línea ssd1306_Line(projected[a], projected[b], White);
 *   :k++;
 * endwhile
 * :Incrementar ángulos de rotación;
 * stop
 * \enduml
 *
 * @pre El display SSD1306 debe estar inicializado mediante `ssd1306_Init()`.
 * @post Limpia y dibuja el nuevo cuadro sobre el framebuffer en RAM; incrementa ángulos de rotación.
 * @see ssd1306_Line
 * @see Point3D
 * @see Point2D
 */
void WIREGFX_Graphics_DrawCube(void);

/**
 * @brief Renderiza un teseracto (hipercubo 4D) con proyección en perspectiva estereográfica.
 * @details Realiza rotaciones en los planos hiperespaciales X-Z, Y-Z y X-W sobre 16 vértices
 *          y 32 aristas conectadas.
 * @pre El display SSD1306 debe estar inicializado.
 * @post Modifica el framebuffer de video del SSD1306 e incrementa `angle_xz`, `angle_yz`, `angle_xw`.
 * @see Point4D
 * @see Point2D
 */
void WIREGFX_Graphics_DrawTesseract(void);

/**
 * @brief Renderiza una pirámide de base cuadrada rotando en el espacio tridimensional.
 * @details Transforma 5 vértices y conecta 8 aristas en rotación continua sobre los tres ejes.
 * @pre Display SSD1306 inicializado.
 * @post Dibuja la pirámide en el framebuffer de RAM.
 * @see Point3D
 */
void WIREGFX_Graphics_DrawPyramid(void);

/**
 * @brief Renderiza una esfera alámbrica tridimensional de baja poligonización (Low-Poly).
 * @details Contiene 14 vértices distribuidos en dos polos y tres anillos latitudinales con 24 aristas.
 * @pre Display SSD1306 inicializado.
 * @post Actualiza el framebuffer del SSD1306.
 */
void WIREGFX_Graphics_DrawSphere(void);

/**
 * @brief Gestiona el ciclo secuencial de animación de figuras tridimensionales.
 * @details Conmuta automáticamente la figura visualizada cada 10 segundos:
 *          **Cubo 3D** &rarr; **Teseracto 4D** &rarr; **Pirámide 3D**.
 *
 * \startuml
 * title Máquina de Estados: Animación WireGFX (WIREGFX_DisplayTask)
 * [*] --> Cubo : Reset / Inicio
 * state Cubo : Renderiza WIREGFX_Graphics_DrawCube()
 * state Teseracto : Renderiza WIREGFX_Graphics_DrawTesseract()
 * state Piramide : Renderiza WIREGFX_Graphics_DrawPyramid()
 *
 * Cubo --> Teseracto : DeltaTick >= 10000 ms
 * Teseracto --> Piramide : DeltaTick >= 10000 ms
 * Piramide --> Cubo : DeltaTick >= 10000 ms
 * \enduml
 *
 * @pre Debe ser invocado periódicamente dentro del bucle cooperativo del sistema (ej: 20 ms).
 * @post Actualiza el framebuffer llamando a la rutina de renderizado correspondiente.
 * @see WIREGFX_ResetCycle
 * @see WIREGFX_Graphics_DrawCube
 * @see WIREGFX_Graphics_DrawTesseract
 * @see WIREGFX_Graphics_DrawPyramid
 */
void WIREGFX_DisplayTask(void);

/**
 * @brief Reinicia el ciclo de animación para comenzar inmediatamente desde el Cubo 3D.
 * @post Reinicia el timestamp `wiregfx_last_switch_tick` al tick actual y fija el índice en 0.
 * @see WIREGFX_DisplayTask
 */
void WIREGFX_ResetCycle(void);

#endif /* INC_WIREGFX_H_ */
