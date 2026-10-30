/**
 * @file compasswidget.h
 * @brief Widget de instrumentación virtual para representación vectorial de brújula y orientación angular.
 * @author Gonzalo Gomez
 * @date 2026
 *
 * @details Proporciona un instrumento gráfico interactivo de alta fidelidad estética para el monitoreo
 * en tiempo real de la orientación espacial (Yaw/Heading) del robot móvil.
 * Soporta fijación de rumbo de referencia relativo (puesta a cero o setpoint de navegación),
 * normalización matemática angular en rangos $[-180^\circ, +180^\circ]$ y $[0^\circ, 360^\circ)$,
 * proyección cardinal (N, NE, E, SE, S, SO, O, NO), e interacción directa por clic de ratón.
 *
 * @see MainWindow
 * @see compasswidget.cpp
 */

#ifndef COMPASSWIDGET_H
#define COMPASSWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QtMath>

/**
 * @ingroup group_widgets
 * @class CompassWidget
 * @brief Widget personalizado para visualización de brújula náutica/aeronáutica digital.
 * @details Renderiza una rosa de los vientos estilizada con aguja indicadora de rumbo,
 * marca visual de referencia angular y telemetría numérica integrada tipo Head-Up Display (HUD).
 *
 * \startuml
 * class CompassWidget {
 *   - m_heading : float
 *   - m_reference : float
 *   - m_hasReference : bool
 *   + CompassWidget(parent : QWidget*)
 *   + heading() : float
 *   + reference() : float
 *   + hasReference() : bool
 *   + relativeAngle() : float
 *   + setHeading(deg : float) : void
 *   + setReference(deg : float) : void
 *   + resetReference() : void
 *   # paintEvent(event : QPaintEvent*) : void
 *   # mousePressEvent(event : QMouseEvent*) : void
 *   - getCardinal(deg : float) : QString
 *   - normalize180(deg : float) : float
 *   - normalize360(deg : float) : float
 * }
 * CompassWidget --|> QWidget
 * \enduml
 *
 * @see MainWindow
 */
class CompassWidget : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief Constructor explícito del widget de brújula.
     * @param parent Puntero al contenedor padre de Qt (nullptr por defecto).
     * @post Configura la política de tamaño expansiva y el cursor interactivo de selección.
     */
    explicit CompassWidget(QWidget *parent = nullptr);

    /**
     * @brief Obtiene el rumbo absoluto actual en grados.
     * @return float Ángulo de orientación actual $[0^\circ, 360^\circ)$.
     */
    float heading() const { return m_heading; }

    /**
     * @brief Obtiene el rumbo de referencia fijado en grados.
     * @return float Ángulo de referencia angular.
     */
    float reference() const { return m_reference; }

    /**
     * @brief Consulta si existe una referencia angular activa fijada.
     * @return bool `true` si la referencia está activa, `false` de lo contrario.
     */
    bool hasReference() const { return m_hasReference; }

    /**
     * @brief Calcula la desviación angular relativa respecto al rumbo de referencia.
     * @details Realiza la diferencia modular entre el rumbo medido y la referencia,
     * normalizando el resultado en el intervalo $[-180^\circ, +180^\circ]$.
     * @return float Desvío angular relativo en grados con signo.
     * @see normalize180
     */
    float relativeAngle() const;

public slots:
    /**
     * @brief Establece el nuevo ángulo de rumbo absoluto y actualiza la visualización.
     * @details Si el cambio excede un umbral de histéresis ($0.01^\circ$), actualiza el estado interno,
     * emite la señal `headingChanged` y solicita un refresco asíncrono con `update()`.
     *
     * \startuml
     * title Flujo de Actualización Angular (setHeading)
     * start
     * :Recibir deg;
     * if (|m_heading - deg| > 0.01) then (si)
     *   :m_heading = deg;
     *   :emit headingChanged(m_heading);
     *   :update() [Repintado QPainter];
     * else (no)
     *   :Filtrar por banda muerta;
     * endif
     * stop
     * \enduml
     *
     * @param deg Nuevo ángulo en grados proveniente del sensor MPU/Magnetómetro.
     * @post Se agenda el repintado de la aguja y lecturas del widget.
     */
    void setHeading(float deg);

    /**
     * @brief Fija un nuevo ángulo de referencia arbitrario.
     * @param deg Rumbo deseado para establecer como nuevo punto cero relativo.
     * @post Emite la señal `referenceChanged`.
     */
    void setReference(float deg);

    /**
     * @brief Reinicia el ángulo de referencia a $0.0^\circ$ (Norte absoluto).
     * @post `m_reference` se establece en 0.0f y se emite la señal `referenceChanged`.
     */
    void resetReference();

signals:
    /**
     * @brief Señal emitida cuando el rumbo de referencia es modificado.
     * @param refDeg Nuevo valor de referencia angular en grados.
     */
    void referenceChanged(float refDeg);

    /**
     * @brief Señal emitida cuando el rumbo actual medido cambia significativamente.
     * @param headingDeg Nuevo rumbo absoluto en grados.
     */
    void headingChanged(float headingDeg);

protected:
    /**
     * @brief Manejador del evento de repintado vectorial.
     * @details Dibuja mediante `QPainter` con antialiasing la base circular, graduaciones
     * de grados, puntos cardinales, aguja de rumbo con gradiente y panel HUD de telemetría digital.
     * @param event Puntero al objeto de evento de repintado de Qt.
     */
    void paintEvent(QPaintEvent *event) override;

    /**
     * @brief Manejador de eventos de interacción con el ratón.
     * @details Al hacer clic izquierdo sobre cualquier punto de la brújula, toma el rumbo actual
     * instantáneo (`m_heading`) y lo fija automáticamente como nuevo rumbo de referencia (`setReference`).
     * @param event Puntero al evento de ratón de Qt.
     */
    void mousePressEvent(QMouseEvent *event) override;

private:
    /**
     * @brief Ángulo de orientación actual del robot medido en grados.
     * @see setHeading()
     */
    float m_heading = 0.0f;

    /**
     * @brief Ángulo de referencia tomado como cero relativo en grados.
     * @see setReference()
     */
    float m_reference = 0.0f;

    /**
     * @brief Indicador de presencia de una referencia válida activa.
     */
    bool m_hasReference = true;

    /**
     * @brief Convierte un ángulo en grados a su denominación cardinal textual.
     * @param deg Ángulo en grados.
     * @return QString Cadena con el punto cardinal ("N", "NE", "E", "SE", "S", "SO", "O", "NO").
     */
    QString getCardinal(float deg) const;

    /**
     * @brief Normaliza cualquier valor angular al rango simétrico $[-180^\circ, +180^\circ]$.
     * @param deg Ángulo en grados arbitrario.
     * @return float Ángulo normalizado en $[-180, +180]$.
     */
    float normalize180(float deg) const;

    /**
     * @brief Normaliza cualquier valor angular al rango positivo $[0^\circ, 360^\circ)$.
     * @param deg Ángulo en grados arbitrario.
     * @return float Ángulo normalizado en $[0, 360)$.
     */
    float normalize360(float deg) const;
};

#endif // COMPASSWIDGET_H
