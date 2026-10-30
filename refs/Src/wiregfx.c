/**
 * @file   wiregfx.c
 * @author Gonzalo M. Buffa
 * @date   28/04/2026
 * @brief  Implementación del pipeline de renderizado alámbrico (Wireframe) 3D/4D en punto fijo.
 * @details Realiza proyecciones axonométricas y en perspectiva estereográfica sobre el display SSD1306.
 *          Aplica matrices de rotación trigonométrica computadas mediante una tabla precalculada (sin_LUT)
 *          de 256 elementos escalada a 7 bits ($1.0 \approx 127$), sustituyendo la división en coma flotante
 *          por desplazamientos aritméticos a la derecha (`>> 7`).
 * @ingroup group_ui_graphics
 */

#include "wiregfx.h"

/**
 * @name Variables Estáticas de Ángulos de Rotación
 * @{
 */
static uint8_t angle_x = 0;   /*!< Ángulo de rotación sobre el eje X (0 a 255 = 0° a 360°) */
static uint8_t angle_y = 0;   /*!< Ángulo de rotación sobre el eje Y */
static uint8_t angle_z = 0;   /*!< Ángulo de rotación sobre el eje Z */
static uint8_t angle_xz = 0;  /*!< Ángulo de rotación en el plano X-Z (para 4D) */
static uint8_t angle_yz = 0;  /*!< Ángulo de rotación en el plano Y-Z (para 4D) */
static uint8_t angle_xw = 0;  /*!< Ángulo de rotación en el plano X-W de la 4ta dimensión */
/** @} */

// Look-Up Table (LUT) de Seno. 256 valores precalculados.
// Está escalada de -127 a 127. (1.0 = 127). Esto nos permite dividir rápido con ">> 7".
const int8_t sin_LUT[256] = {
    0, 3, 6, 9, 12, 15, 18, 21, 24, 28, 31, 34, 37, 40, 43, 46,
    48, 51, 54, 57, 60, 63, 65, 68, 71, 73, 76, 78, 81, 83, 85, 88,
    90, 92, 94, 96, 98, 100, 102, 104, 106, 108, 109, 111, 112, 114, 115, 117,
    118, 119, 120, 121, 122, 123, 124, 124, 125, 126, 126, 126, 127, 127, 127, 127,
    127, 127, 127, 127, 126, 126, 126, 125, 124, 124, 123, 122, 121, 120, 119, 118,
    117, 115, 114, 112, 111, 109, 108, 106, 104, 102, 100, 98, 96, 94, 92, 90,
    88, 85, 83, 81, 78, 76, 73, 71, 68, 65, 63, 60, 57, 54, 51, 48,
    46, 43, 40, 37, 34, 31, 28, 24, 21, 18, 15, 12, 9, 6, 3, 0,
    -3, -6, -9, -12, -15, -18, -21, -24, -28, -31, -34, -37, -40, -43, -46, -48,
    -51, -54, -57, -60, -63, -65, -68, -71, -73, -76, -78, -81, -83, -85, -88, -90,
    -92, -94, -96, -98, -100, -102, -104, -106, -108, -109, -111, -112, -114, -115, -117, -118,
    -119, -120, -121, -122, -123, -124, -124, -125, -126, -126, -126, -127, -127, -127, -127, -127,
    -127, -127, -127, -126, -126, -126, -125, -124, -124, -123, -122, -121, -120, -119, -118, -117,
    -115, -114, -112, -111, -109, -108, -106, -104, -102, -100, -98, -96, -94, -92, -90, -88,
    -85, -83, -81, -78, -76, -73, -71, -68, -65, -63, -60, -57, -54, -51, -48, -46,
    -43, -40, -37, -34, -31, -28, -24, -21, -18, -15, -12, -9, -6, -3, 0
};

static const Point3D cube_vertices[8] = {
    {-16, -16, -16}, { 16, -16, -16}, { 16,  16, -16}, {-16,  16, -16},
    {-16, -16,  16}, { 16, -16,  16}, { 16,  16,  16}, {-16,  16,  16}
};

static const int cube_edges[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}
};

static const Point4D tesseract_vertices[16] = {
    {-16, -16, -16, -16}, { 16, -16, -16, -16}, { 16,  16, -16, -16}, {-16,  16, -16, -16},
    {-16, -16,  16, -16}, { 16, -16,  16, -16}, { 16,  16,  16, -16}, {-16,  16,  16, -16},
    {-16, -16, -16,  16}, { 16, -16, -16,  16}, { 16,  16, -16,  16}, {-16,  16, -16,  16},
    {-16, -16,  16,  16}, { 16, -16,  16,  16}, { 16,  16,  16,  16}, {-16,  16,  16,  16}
};

static const int tesseract_edges[32][2] = {
    {0,1}, {1,2}, {2,3}, {3,0}, {4,5}, {5,6}, {6,7}, {7,4}, {0,4}, {1,5}, {2,6}, {3,7},
    {8,9}, {9,10}, {10,11}, {11,8}, {12,13}, {13,14}, {14,15}, {15,12}, {8,12}, {9,13}, {10,14}, {11,15},
    {0,8}, {1,9}, {2,10}, {3,11}, {4,12}, {5,13}, {6,14}, {7,15}
};

// --- Vértices y Aristas de la Pirámide ---
static const Point3D pyramid_vertices[5] = {
    {  0, -20,   0}, // 0: Punta (Apex) - Y negativo va hacia "arriba" en la pantalla
    {-16,  16, -16}, // 1: Base Izquierda-Atrás
    { 16,  16, -16}, // 2: Base Derecha-Atrás
    { 16,  16,  16}, // 3: Base Derecha-Adelante
    {-16,  16,  16}  // 4: Base Izquierda-Adelante
};

static const int pyramid_edges[8][2] = {
    {0, 1}, {0, 2}, {0, 3}, {0, 4}, // Lineas de la punta a la base
    {1, 2}, {2, 3}, {3, 4}, {4, 1}  // Lineas que forman el cuadrado de la base
};

// --- Vértices y Aristas de la Esfera (Low-Poly) ---
// Tiene un Polo Norte, un Polo Sur, y dos anillos (Ecuador y zonas templadas)
static const Point3D sphere_vertices[14] = {
    {  0, -20,   0}, // 0: Polo Norte
    {  0,  20,   0}, // 1: Polo Sur

    // Anillo Superior (Y = -10)
    { 14, -10,   0}, {  0, -10,  14}, {-14, -10,   0}, {  0, -10, -14},

    // Anillo Central (Ecuador, Y = 0)
    { 20,   0,   0}, {  0,   0,  20}, {-20,   0,   0}, {  0,   0, -20},

    // Anillo Inferior (Y = 10)
    { 14,  10,   0}, {  0,  10,  14}, {-14,  10,   0}, {  0,  10, -14}
};

static const int sphere_edges[24][2] = {
    // Conexiones del Polo Norte al anillo superior
    {0, 2}, {0, 3}, {0, 4}, {0, 5},
    // Conexiones Anillo Sup -> Ecuador
    {2, 6}, {3, 7}, {4, 8}, {5, 9},
    {2, 7}, {3, 8}, {4, 9}, {5, 6}, // Diagonales para dar volumen
    // Conexiones Ecuador -> Anillo Inf
    {6, 10}, {7, 11}, {8, 12}, {9, 13},
    {7, 10}, {8, 11}, {9, 12}, {6, 13}, // Diagonales
    // Conexiones Anillo Inf -> Polo Sur
    {10, 1}, {11, 1}, {12, 1}, {13, 1}
};
// --- Implementación de Funciones ---

/**
 * @brief Renderiza un cubo 3D rotando sobre los ejes X, Y y Z en la pantalla OLED.
 * @details Aplica rotaciones tridimensionales mediante la tabla trigonométrica `sin_LUT`
 *          y proyecta los vértices calculados sobre el framebuffer de memoria del SSD1306.
 * @pre El controlador SSD1306 debe haber sido inicializado y configurado.
 * @post Limpia el buffer con `ssd1306_Fill(Black)` y traza las 12 aristas con `ssd1306_Line`.
 *       Incrementa los ángulos de rotación `angle_x`, `angle_y` y `angle_z`.
 * @see sin_LUT
 * @see cube_vertices
 * @see cube_edges
 * @see ssd1306_Line
 */
void WIREGFX_Graphics_DrawCube(void) {
    Point2D projected[8];
    int32_t sin_x = sin_LUT[angle_x];
    int32_t cos_x = sin_LUT[(uint8_t)(angle_x + 64)];
    int32_t sin_y = sin_LUT[angle_y];
    int32_t cos_y = sin_LUT[(uint8_t)(angle_y + 64)];
    int32_t sin_z = sin_LUT[angle_z];
    int32_t cos_z = sin_LUT[(uint8_t)(angle_z + 64)];

    ssd1306_Fill(Black);

    for (int i = 0; i < 8; i++) {
        int32_t x = cube_vertices[i].x;
        int32_t y = cube_vertices[i].y;
        int32_t z = cube_vertices[i].z;

        // Rotaciones (X, Y, Z) aplicando corrimiento >> 7 para punto fijo
        int32_t xy = (y * cos_x - z * sin_x) >> 7;
        int32_t xz = (y * sin_x + z * cos_x) >> 7;
        y = xy; z = xz;

        int32_t yx = (x * cos_y + z * sin_y) >> 7;
        int32_t yz = (-x * sin_y + z * cos_y) >> 7;
        x = yx; z = yz;

        int32_t zx = (x * cos_z - y * sin_z) >> 7;
        int32_t zy = (x * sin_z + y * cos_z) >> 7;
        x = zx; y = zy;

        projected[i].x = x + CENTER_X;
        projected[i].y = y + CENTER_Y;
    }

    for (int i = 0; i < 12; i++) {
        ssd1306_Line(projected[cube_edges[i][0]].x, projected[cube_edges[i][0]].y,
                     projected[cube_edges[i][1]].x, projected[cube_edges[i][1]].y, White);
    }

    angle_x += 2; angle_y += 1; angle_z += 3;
}

/**
 * @brief Renderiza un teseracto (hipercubo 4D) con proyección en perspectiva estereográfica.
 * @details Realiza rotaciones en los planos hiperespaciales X-Z, Y-Z y X-W sobre 16 vértices
 *          y 32 aristas conectadas, escalando por el factor de perspectiva del eje $W$.
 * @pre Display SSD1306 inicializado.
 * @post Modifica el framebuffer de video del SSD1306; avanza los ángulos `angle_xz`, `angle_yz`, `angle_xw`.
 * @see tesseract_vertices
 * @see tesseract_edges
 * @see ssd1306_Line
 */
void WIREGFX_Graphics_DrawTesseract(void) {
    Point2D projected[16];
    int32_t sin_xz = sin_LUT[angle_xz];
    int32_t cos_xz = sin_LUT[(uint8_t)(angle_xz + 64)];
    int32_t sin_yz = sin_LUT[angle_yz];
    int32_t cos_yz = sin_LUT[(uint8_t)(angle_yz + 64)];
    int32_t sin_xw = sin_LUT[angle_xw];
    int32_t cos_xw = sin_LUT[(uint8_t)(angle_xw + 64)];

    ssd1306_Fill(Black);

    for (int i = 0; i < 16; i++) {
        int32_t x = tesseract_vertices[i].x;
        int32_t y = tesseract_vertices[i].y;
        int32_t z = tesseract_vertices[i].z;
        int32_t w = tesseract_vertices[i].w;

        int32_t xw_x = (x * cos_xw - w * sin_xw) >> 7;
        int32_t xw_w = (x * sin_xw + w * cos_xw) >> 7;
        x = xw_x; w = xw_w;

        int32_t xz_x = (x * cos_xz - z * sin_xz) >> 7;
        int32_t xz_z = (x * sin_xz + z * cos_xz) >> 7;
        x = xz_x; z = xz_z;

        int32_t yz_y = (y * cos_yz - z * sin_yz) >> 7;
        int32_t yz_z = (y * sin_yz + z * cos_yz) >> 7;
        y = yz_y; z = yz_z;

        int32_t w_factor = 48 - w;
        x = (x * w_factor) >> 6;
        y = (y * w_factor) >> 6;

        projected[i].x = x + CENTER_X;
        projected[i].y = y + CENTER_Y;
    }

    for (int i = 0; i < 32; i++) {
        ssd1306_Line(projected[tesseract_edges[i][0]].x, projected[tesseract_edges[i][0]].y,
                     projected[tesseract_edges[i][1]].x, projected[tesseract_edges[i][1]].y, White);
    }

    angle_xz += 1; angle_yz += 1; angle_xw += 2;
}

/**
 * @brief Renderiza una pirámide de base cuadrada rotando en el espacio tridimensional.
 * @details Transforma 5 vértices y conecta 8 aristas en rotación continua sobre los tres ejes espaciales.
 * @pre Display SSD1306 inicializado.
 * @post Actualiza el framebuffer en memoria RAM con las aristas de la pirámide.
 * @see pyramid_vertices
 * @see pyramid_edges
 * @see ssd1306_Line
 */
void WIREGFX_Graphics_DrawPyramid(void) {
    Point2D projected[5];
    int32_t sin_x = sin_LUT[angle_x];
    int32_t cos_x = sin_LUT[(uint8_t)(angle_x + 64)];
    int32_t sin_y = sin_LUT[angle_y];
    int32_t cos_y = sin_LUT[(uint8_t)(angle_y + 64)];
    int32_t sin_z = sin_LUT[angle_z];
    int32_t cos_z = sin_LUT[(uint8_t)(angle_z + 64)];

    ssd1306_Fill(Black);

    // 1. Rotar y proyectar los 5 vértices
    for (int i = 0; i < 5; i++) {
        int32_t x = pyramid_vertices[i].x;
        int32_t y = pyramid_vertices[i].y;
        int32_t z = pyramid_vertices[i].z;

        int32_t xy = (y * cos_x - z * sin_x) >> 7;
        int32_t xz = (y * sin_x + z * cos_x) >> 7;
        y = xy; z = xz;

        int32_t yx = (x * cos_y + z * sin_y) >> 7;
        int32_t yz = (-x * sin_y + z * cos_y) >> 7;
        x = yx; z = yz;

        int32_t zx = (x * cos_z - y * sin_z) >> 7;
        int32_t zy = (x * sin_z + y * cos_z) >> 7;
        x = zx; y = zy;

        projected[i].x = x + CENTER_X;
        projected[i].y = y + CENTER_Y;
    }

    // 2. Dibujar las 8 aristas
    for (int i = 0; i < 8; i++) {
        ssd1306_Line(projected[pyramid_edges[i][0]].x, projected[pyramid_edges[i][0]].y,
                     projected[pyramid_edges[i][1]].x, projected[pyramid_edges[i][1]].y, White);
    }

    // Incremento de ángulos
    angle_x += 2; angle_y += 3; angle_z += 1;
}

/**
 * @brief Renderiza una esfera alámbrica tridimensional de baja poligonización (Low-Poly).
 * @details Proyecta 14 vértices distribuidos en anillos latitudinales y dibuja 24 aristas.
 * @pre Display SSD1306 inicializado.
 * @post Actualiza el framebuffer del SSD1306 con la geometría esférica.
 * @see sphere_vertices
 * @see sphere_edges
 * @see ssd1306_Line
 */
void WIREGFX_Graphics_DrawSphere(void) {
    Point2D projected[14]; // 14 vértices

    int32_t sin_x = sin_LUT[angle_x];
    int32_t cos_x = sin_LUT[(uint8_t)(angle_x + 64)];
    int32_t sin_y = sin_LUT[angle_y];
    int32_t cos_y = sin_LUT[(uint8_t)(angle_y + 64)];
    int32_t sin_z = sin_LUT[angle_z];
    int32_t cos_z = sin_LUT[(uint8_t)(angle_z + 64)];

    ssd1306_Fill(Black);

    for (int i = 0; i < 14; i++) {
        int32_t x = sphere_vertices[i].x;
        int32_t y = sphere_vertices[i].y;
        int32_t z = sphere_vertices[i].z;

        // Rotaciones matriciales
        int32_t xy = (y * cos_x - z * sin_x) >> 7;
        int32_t xz = (y * sin_x + z * cos_x) >> 7;
        y = xy; z = xz;

        int32_t yx = (x * cos_y + z * sin_y) >> 7;
        int32_t yz = (-x * sin_y + z * cos_y) >> 7;
        x = yx; z = yz;

        int32_t zx = (x * cos_z - y * sin_z) >> 7;
        int32_t zy = (x * sin_z + y * cos_z) >> 7;
        x = zx; y = zy;

        projected[i].x = x + CENTER_X;
        projected[i].y = y + CENTER_Y;
    }

    for (int i = 0; i < 24; i++) { // 24 aristas
        ssd1306_Line(projected[sphere_edges[i][0]].x, projected[sphere_edges[i][0]].y,
                     projected[sphere_edges[i][1]].x, projected[sphere_edges[i][1]].y, White);
    }

    angle_x += 1; angle_y += 2; angle_z += 1; // Giro suave
}

static uint32_t wiregfx_last_switch_tick = 0;
static uint8_t wiregfx_figure_index = 0; // 0: Cubo, 1: Teseracto, 2: Pirámide

/**
 * @brief Reinicia el temporizador y fija el índice para renderizar inmediatamente el Cubo 3D.
 * @post `wiregfx_last_switch_tick` toma el valor de `HAL_GetTick()` y `wiregfx_figure_index` pasa a 0.
 * @see WIREGFX_DisplayTask
 */
void WIREGFX_ResetCycle(void) {
    wiregfx_last_switch_tick = HAL_GetTick();
    wiregfx_figure_index = 0;
}

/**
 * @brief Gestiona el ciclo secuencial de animación de figuras tridimensionales.
 * @details Evalúa el delta de tiempo con `HAL_GetTick()`. Cada 10 segundos conmuta cíclicamente
 *          la figura geométrica: Cubo (0) &rarr; Teseracto (1) &rarr; Pirámide (2).
 * @pre Debe invocarse periódicamente desde la tarea de interfaz de usuario (ej. ciclo de 20 ms).
 * @post Ejecuta la función de renderizado de la figura correspondiente sobre el framebuffer del SSD1306.
 * @see WIREGFX_Graphics_DrawCube
 * @see WIREGFX_Graphics_DrawTesseract
 * @see WIREGFX_Graphics_DrawPyramid
 * @see WIREGFX_ResetCycle
 */
void WIREGFX_DisplayTask(void) {
    uint32_t now = HAL_GetTick();
    if (wiregfx_last_switch_tick == 0) {
        wiregfx_last_switch_tick = now;
    }

    // Cada 10 segundos (10,000 ms) se cambia de figura
    if ((now - wiregfx_last_switch_tick) >= 10000) {
        wiregfx_last_switch_tick = now;
        wiregfx_figure_index = (wiregfx_figure_index + 1) % 3;
    }

    // Orden de renderizado: Cubo -> Teseracto -> Pirámide
    switch (wiregfx_figure_index) {
        case 0:
            WIREGFX_Graphics_DrawCube();
            break;
        case 1:
            WIREGFX_Graphics_DrawTesseract();
            break;
        case 2:
            WIREGFX_Graphics_DrawPyramid();
            break;
        default:
            wiregfx_figure_index = 0;
            WIREGFX_Graphics_DrawCube();
            break;
    }
}
