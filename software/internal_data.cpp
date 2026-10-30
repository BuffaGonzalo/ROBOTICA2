/**
 * @file internal_data.cpp
 * @brief Implementación del diálogo de inspección interna para telemetría de línea.
 * @author Gonzalo Gomez
 * @date 2026
 *
 * @details Implementa la lógica de inicialización y actualización de datos de la clase
 * `internal_data`, vinculando los eventos de la interfaz con los datos procesados.
 *
 * @see internal_data.h
 */

#include "internal_data.h"
#include "ui_internal_data.h"

/**
 * @brief Constructor del diálogo internal_data.
 * @param parent Puntero al widget contenedor padre.
 */
internal_data::internal_data(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::internal_data)
{
    ui->setupUi(this);
}

/**
 * @brief Destructor del diálogo. Libera el recurso ui.
 */
internal_data::~internal_data()
{
    delete ui;
}

/**
 * @brief Método para actualizar los indicadores numéricos del diálogo.
 * @param sum Suma de los sensores ópticos.
 * @param err Error relativo respecto a la trayectoria deseada.
 * @param abs_err Valor absoluto del error actual.
 * @param lin Componente proporcional lineal calculada.
 * @param quad Componente cuadrática aplicada a curvas cerradas.
 * @param offset Corrección de desviación estática.
 * @param kp Ganancia proporcional activa.
 * @param kd Ganancia derivativa/cuadrática activa.
 */
void internal_data::actualizarDatosLinea(int32_t sum, int32_t err, int32_t abs_err, int32_t lin, int32_t quad, int32_t offset, int16_t kp, int16_t kd)
{
    Q_UNUSED(sum);
    Q_UNUSED(err);
    Q_UNUSED(abs_err);
    Q_UNUSED(lin);
    Q_UNUSED(quad);
    Q_UNUSED(offset);
    Q_UNUSED(kp);
    Q_UNUSED(kd);
}
