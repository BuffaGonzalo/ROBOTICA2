/**
 * @file compasswidget.cpp
 * @brief Implementación del motor de renderizado vectorial y manejo de eventos de la brújula.
 * @author Gonzalo Gomez
 * @date 2026
 *
 * @details Este archivo implementa el renderizado por software con antialiasing de la brújula (`CompassWidget`).
 * Utiliza primitivas geométricas de `QPainter`, transformaciones afines de rotación y traslación,
 * gradientes radiales y lineales, y lógica de proyección trigonométrica angular.
 *
 * @see compasswidget.h
 */

#include "compasswidget.h"
#include <QFont>
#include <QFontMetrics>
#include <QPen>
#include <QBrush>
#include <QRadialGradient>
#include <QLinearGradient>

/**
 * @brief Constructor de la clase CompassWidget.
 * @param parent Widget contenedor de la jerarquía Qt.
 * @post Establece el tamaño mínimo en 120x120 px y cursor de puntero.
 */
CompassWidget::CompassWidget(QWidget *parent)
    : QWidget(parent) {
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMinimumSize(120, 120);
    setCursor(Qt::PointingHandCursor);
    setToolTip("Brújula de Orientación: Clic para fijar rumbo actual como referencia (0°)");
}

/**
 * @brief Obtiene el ángulo relativo actual normalizado a [-180, +180].
 * @return float Desvío respecto al setpoint/referencia fijada.
 */
float CompassWidget::relativeAngle() const {
    return normalize180(m_heading - m_reference);
}

/**
 * @brief Asigna el rumbo actual en grados.
 * @param deg Ángulo absoluto en grados.
 * @post Si la variación supera 0.01°, emite headingChanged y agenda update().
 */
void CompassWidget::setHeading(float deg) {
    deg = normalize360(deg);
    if (qAbs(m_heading - deg) > 0.01f) {
        m_heading = deg;
        emit headingChanged(m_heading);
        update();
    }
}

/**
 * @brief Fija un nuevo ángulo de referencia arbitrario.
 * @param deg Ángulo en grados para actuar como nuevo origen 0°.
 * @post Emite referenceChanged y agenda repintado.
 */
void CompassWidget::setReference(float deg) {
    m_reference = normalize360(deg);
    m_hasReference = true;
    emit referenceChanged(m_reference);
    update();
}

/**
 * @brief Restablece la referencia al Norte absoluto (0.0°).
 */
void CompassWidget::resetReference() {
    m_reference = 0.0f;
    m_hasReference = true;
    emit referenceChanged(m_reference);
    update();
}

/**
 * @brief Captura clics del ratón para fijar la orientación instantánea como referencia.
 * @param event Evento de ratón Qt.
 */
void CompassWidget::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        // Al hacer clic sobre la brújula, fijar el rumbo actual como referencia
        setReference(m_heading);
    }
    QWidget::mousePressEvent(event);
}

/**
 * @brief Normaliza un ángulo al rango continuo (-180, +180].
 * @param deg Ángulo en grados.
 * @return float Ángulo normalizado simétrico.
 */
float CompassWidget::normalize180(float deg) const {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

/**
 * @brief Normaliza un ángulo al rango positivo [0, 360).
 * @param deg Ángulo en grados.
 * @return float Ángulo normalizado positivo.
 */
float CompassWidget::normalize360(float deg) const {
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

/**
 * @brief Mapea un ángulo numérico al octante cardinal correspondiente.
 * @param deg Ángulo en grados.
 * @return QString Etiqueta cardinal ("N", "NE", "E", "SE", "S", "SO", "O", "NO").
 */
QString CompassWidget::getCardinal(float deg) const {
    float norm = normalize360(deg);
    if (norm >= 337.5f || norm < 22.5f) return "N";
    if (norm >= 22.5f && norm < 67.5f) return "NE";
    if (norm >= 67.5f && norm < 112.5f) return "E";
    if (norm >= 112.5f && norm < 157.5f) return "SE";
    if (norm >= 157.5f && norm < 202.5f) return "S";
    if (norm >= 202.5f && norm < 247.5f) return "SO";
    if (norm >= 247.5f && norm < 292.5f) return "O";
    return "NO";
}

/**
 * @brief Función de renderizado del componente visual brújula.
 * @details Dibuja el marco HUD superior, disco de brújula con gradiente radial, marcas
 * graduadas cada 30°, arco cian de desviación angular relativa y aguja facetada 3D.
 * @param event Evento de pintura suministrado por Qt.
 */
void CompassWidget::paintEvent(QPaintEvent *) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);

    const int w = width();
    const int h = height();

    // Fondo oscuro que se integra fluidamente con la interfaz general
    p.fillRect(rect(), QColor("#0c0d16"));

    // ── 1. Telemetría Digital Superior (Minimalista HUD) ──────────────────────
    const float normHead = normalize360(m_heading);
    const QString cardinal = getCardinal(normHead);
    const QString headingNumStr = QString("%1°").arg(qRound(normHead));

    QFont fontNum("Segoe UI", 16, QFont::Bold);
    QFont fontBadge("Segoe UI", 8, QFont::Bold);
    QFont fontSub("Segoe UI", 8, QFont::Normal);

    QFontMetrics fmNum(fontNum);
    QFontMetrics fmBadge(fontBadge);

    const int numW = fmNum.horizontalAdvance(headingNumStr);
    const int badgeW = fmBadge.horizontalAdvance(cardinal) + 12;
    const int badgeH = 17;
    const int gap = 7;
    const int totalW = numW + gap + badgeW;
    const int startX = (w - totalW) / 2;

    // Número de rumbo principal en blanco puro
    p.setFont(fontNum);
    p.setPen(QColor("#ffffff"));
    p.drawText(QRect(startX, 3, numW, 26), Qt::AlignVCenter | Qt::AlignLeft, headingNumStr);

    // Badge cardinal minimalista cian neón
    QRectF badgeRect(startX + numW + gap, 8, badgeW, badgeH);
    p.setBrush(QColor("#101524"));
    p.setPen(QPen(QColor("#00f2c3"), 1.0));
    p.drawRoundedRect(badgeRect, 3.5, 3.5);

    p.setFont(fontBadge);
    p.setPen(QColor("#00f2c3"));
    p.drawText(badgeRect, Qt::AlignCenter, cardinal);

    // Subtítulo con punto de referencia y desviación angular Δ
    const float relAng = relativeAngle();
    const QString subText = QString("REF: %1°   •   Δ: %2%3°")
                                .arg(qRound(normalize360(m_reference)))
                                .arg(relAng >= 0 ? "+" : "")
                                .arg(QString::number(relAng, 'f', 1));
    p.setFont(fontSub);
    p.setPen(QColor("#8c92b2"));
    p.drawText(QRect(0, 27, w, 16), Qt::AlignCenter, subText);

    // ── 2. Geometría del Cuadrante ───────────────────────────────────────────
    const double headerHeight = 44.0;
    const QPointF center(w / 2.0, headerHeight + (h - headerHeight) / 2.0);

    double radius = qMin((w - 20.0) / 2.0, (h - headerHeight - 12.0) / 2.0);
    if (radius < 35.0) radius = 35.0;

    // Disco base con sutil degradado radial oscuro
    QRadialGradient dialGrad(center, radius);
    dialGrad.setColorAt(0.0, QColor("#131524"));
    dialGrad.setColorAt(0.75, QColor("#0f111c"));
    dialGrad.setColorAt(1.0, QColor("#0a0b12"));
    p.setBrush(dialGrad);
    p.setPen(QPen(QColor("#24283f"), 1.2));
    p.drawEllipse(center, radius, radius);

    // Anillo concéntrico interno de precisión aeronáutica
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(QColor("#1b1e30"), 1.0));
    p.drawEllipse(center, radius * 0.72, radius * 0.72);

    // Cruz central sutil
    p.setPen(QPen(QColor("#1e2236"), 1.0));
    p.drawLine(QPointF(center.x() - 7, center.y()), QPointF(center.x() + 7, center.y()));
    p.drawLine(QPointF(center.x(), center.y() - 7), QPointF(center.x(), center.y() + 7));

    // ── 3. Graduaciones y Puntos Cardinales ──────────────────────────────────
    p.save();
    p.translate(center);

    // Marcas de graduación minimalistas (solo cada 30 grados)
    for (int deg = 0; deg < 360; deg += 30) {
        p.save();
        p.rotate(deg);

        if (deg == 0) {
            // Marca Norte (Rojo Coral destacado)
            p.setPen(QPen(QColor("#ff5252"), 2.2, Qt::SolidLine, Qt::RoundCap));
            p.drawLine(QPointF(0, -radius), QPointF(0, -radius + 8));
        } else if (deg % 90 == 0) {
            // Marcas cardinales mayores (E, S, W)
            p.setPen(QPen(QColor("#6d759b"), 1.6, Qt::SolidLine, Qt::RoundCap));
            p.drawLine(QPointF(0, -radius), QPointF(0, -radius + 6));
        } else {
            // Marcas intermedias cada 30°
            p.setPen(QPen(QColor("#2d334e"), 1.0));
            p.drawLine(QPointF(0, -radius), QPointF(0, -radius + 3.5));
        }
        p.restore();
    }

    // Letras cardinales integradas elegantemente dentro del cuadrante
    // Norte
    p.setFont(QFont("Segoe UI", 9, QFont::Bold));
    p.setPen(QColor("#ff5252"));
    p.drawText(QRectF(-12, -radius + 10, 24, 15), Qt::AlignCenter, "N");

    // Este, Sur, Oeste
    p.setFont(QFont("Segoe UI", 8, QFont::Bold));
    p.setPen(QColor("#6b7294"));
    p.drawText(QRectF(radius - 23, -8, 20, 16), Qt::AlignCenter, "E");
    p.drawText(QRectF(-10, radius - 23, 20, 16), Qt::AlignCenter, "S");
    p.drawText(QRectF(-radius + 3, -8, 20, 16), Qt::AlignCenter, "O");

    // ── 4. Arco y Marcador de Referencia (Cian Neón) ─────────────────────────
    if (m_hasReference) {
        // Si hay una desviación angular apreciable, trazar un arco cian sutil
        const float spanDeg = -normalize180(m_heading - m_reference);
        if (qAbs(spanDeg) > 0.8f) {
            const float startQAngle = (90.0f - m_reference) * 16.0f;
            const float spanQAngle = spanDeg * 16.0f;

            QRectF arcRect(-radius + 1.5, -radius + 1.5, (radius - 1.5) * 2.0, (radius - 1.5) * 2.0);
            p.setBrush(Qt::NoBrush);
            p.setPen(QPen(QColor(0, 242, 195, 120), 2.2, Qt::SolidLine, Qt::RoundCap));
            p.drawArc(arcRect, qRound(startQAngle), qRound(spanQAngle));
        }

        // Marcador diamante minimalista en la posición de referencia fijada
        p.save();
        p.rotate(m_reference);

        QPolygonF refMarker;
        refMarker << QPointF(0, -radius)
                  << QPointF(-3.2, -radius + 5.5)
                  << QPointF(0, -radius + 8.5)
                  << QPointF(3.2, -radius + 5.5);

        p.setBrush(QColor("#00f2c3"));
        p.setPen(QPen(QColor("#061c16"), 0.6));
        p.drawPolygon(refMarker);
        p.restore();
    }

    // ── 5. Aguja Minimalista de Precisión (Estilo Telemetría) ─────────────────
    p.save();
    p.rotate(m_heading);

    const double needleLen = radius - 16.0;

    // Flecha delantera (Norte/Rumbo) facetada en coral y rojo para efecto 3D pulido
    QPolygonF leftFacet;
    leftFacet << QPointF(0, -needleLen)
              << QPointF(-3.2, 0)
              << QPointF(0, 0);
    p.setBrush(QColor("#ff5252"));
    p.setPen(Qt::NoPen);
    p.drawPolygon(leftFacet);

    QPolygonF rightFacet;
    rightFacet << QPointF(0, -needleLen)
               << QPointF(3.2, 0)
               << QPointF(0, 0);
    p.setBrush(QColor("#d32f2f"));
    p.setPen(Qt::NoPen);
    p.drawPolygon(rightFacet);

    // Arista central blanca sutil para brillo de precisión
    p.setPen(QPen(QColor(255, 255, 255, 170), 0.8));
    p.drawLine(QPointF(0, 0), QPointF(0, -needleLen));

    // Cola trasera corta / contrapeso estilizado
    const double tailLen = radius * 0.32;
    QPolygonF tailLeft;
    tailLeft << QPointF(0, 0)
             << QPointF(-2.2, 0)
             << QPointF(0, tailLen);
    p.setBrush(QColor("#272b42"));
    p.setPen(Qt::NoPen);
    p.drawPolygon(tailLeft);

    QPolygonF tailRight;
    tailRight << QPointF(0, 0)
              << QPointF(2.2, 0)
              << QPointF(0, tailLen);
    p.setBrush(QColor("#1b1d2e"));
    p.setPen(Qt::NoPen);
    p.drawPolygon(tailRight);

    // Eje de rotación central minimalista (anillo hueco con punto central)
    p.setBrush(QColor("#0e101a"));
    p.setPen(QPen(QColor("#ff5252"), 1.6));
    p.drawEllipse(QPointF(0, 0), 4.8, 4.8);

    p.setBrush(QColor("#ffffff"));
    p.setPen(Qt::NoPen);
    p.drawEllipse(QPointF(0, 0), 1.8, 1.8);

    p.restore(); // Fin rotación aguja
    p.restore(); // Fin traslación centro
}
