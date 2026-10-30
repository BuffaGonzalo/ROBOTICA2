/**
 * @file internal_data.h
 * @brief Ventana de diálogo para inspección y depuración de variables internas del algoritmo seguidor de línea.
 * @author Gonzalo Gomez
 * @date 2026
 *
 * @details Este archivo define la clase `internal_data`, un diálogo secundario (`QDialog`) diseñado para
 * la telemetría granular del algoritmo seguidor de línea ejecutado en el microcontrolador.
 * Permite visualizar métricas computadas en tiempo real tales como la suma ponderada de sensores, el error
 * proporcional instantáneo, valor absoluto del error, componentes lineal y cuadrática del lazo, desfases
 * y ganancias $K_p$ y $K_q$.
 *
 * @see MainWindow
 * @see internal_data.cpp
 */

#ifndef INTERNAL_DATA_H
#define INTERNAL_DATA_H

#include <QDialog>
#include <cstdint>

namespace Ui {
class internal_data;
}

/**
 * @ingroup group_widgets
 * @class internal_data
 * @brief Diálogo de telemetría interna y calibración del seguidor de línea.
 * @details Proporciona una interfaz visual complementaria para monitorear el comportamiento
 * algebraico interno del estimador de línea y los parámetros de control aplicados por el firmware.
 *
 * \startuml
 * class internal_data {
 *   - ui : Ui::internal_data*
 *   + internal_data(parent : QWidget*)
 *   + ~internal_data()
 *   + actualizarDatosLinea(sum, err, abs_err, lin, quad, offset, kp, kq) : void
 * }
 * internal_data --|> QDialog
 * \enduml
 *
 * @see MainWindow
 */
class internal_data : public QDialog
{
    Q_OBJECT

public:
    /**
     * @brief Constructor explícito del diálogo de datos internos.
     * @param parent Puntero al widget padre en la jerarquía de Qt (por defecto nullptr).
     */
    explicit internal_data(QWidget *parent = nullptr);

    /**
     * @brief Destructor de la clase, libera los recursos gráficos de la interfaz.
     */
    ~internal_data();

    /**
     * @brief Actualiza los campos numéricos de la interfaz con la última muestra recibida.
     * @details Desempaqueta y proyecta en los visores del diálogo los cálculos matemáticos
     * realizados por el algoritmo del robot seguidor de línea en cada ciclo de muestreo.
     *
     * \startuml
     * title Pipeline de Procesamiento de Telemetría de Línea (actualizarDatosLinea)
     * start
     * :Recepción de paquete GETINTERNALDATA (ID: 0xF0);
     * :Extracción de sumatoria (sum) y error (err);
     * :Cálculo de magnitud de desvío absoluto (abs_err);
     * :Extracción de componentes de corrección lineal (lin) y cuadrática (quad);
     * :Lectura de compensación lateral (offset) y coeficientes Kp / Kq;
     * :Formateo y actualización de widgets numéricos en la interfaz;
     * stop
     * \enduml
     *
     * @param sum Suma algebraica total de las lecturas de los reflectores IR.
     * @param err Error de posición respecto al centroide de la línea en píxeles/unidades discretas.
     * @param abs_err Valor absoluto del error de seguimiento.
     * @param lin Componente de control de respuesta lineal.
     * @param quad Componente de control de respuesta no lineal (cuadrática).
     * @param offset Desplazamiento dinámico o sesgo de centro calibrado.
     * @param kp Ganancia proporcional activa en el robot.
     * @param kq Ganancia cuadrática activa en el robot.
     *
     * @pre El puntero `ui` debe haber sido inicializado válidamente en el constructor.
     * @post Los indicadores numéricos del diálogo reflejan los nuevos valores.
     */
    void actualizarDatosLinea(int32_t sum, int32_t err, int32_t abs_err, int32_t lin, int32_t quad, int32_t offset, int16_t kp, int16_t kq);

private:
    /**
     * @brief Puntero a la estructura de interfaz generada automáticamente por uic.
     * @see Ui::internal_data
     */
    Ui::internal_data *ui;
};

#endif // INTERNAL_DATA_H
