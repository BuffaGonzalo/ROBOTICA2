# MICRO-Software

Aplicación Qt para comunicar la PC con el Mega2560 por **Serial USB** o **Wi-Fi UDP**.

La pestaña **Comunicación** permite elegir el medio, seleccionar un COM o el puerto UDP local, conectar y consultar el estado con `GETALIVE`. En Serial usa **115200 baudios, 8N1, sin control de flujo**. En UDP escucha en el puerto 30010 y aprende la IP y el puerto del robot al recibir su primer datagrama. La aplicación envía `GETALIVE` al conectar y cada segundo mientras el enlace esté abierto y haya destino.

La pestaña **Comunicación** muestra el registro de comunicación (logs procesados), opciones para exportar los últimos 5 minutos de ejecución a `.txt` o `.xls` (Excel), y el botón para limpiar el registro. **Configuración** contiene PWM manual de las cuatro ruedas, distancia relativa firmada en mm, **MOVER DISTANCIA**, estado, ticks y el botón para detener todas las ruedas. El firmware admite ALIVE, SETPWM, MOVE y STOP; la llegada se confirma con telemetría, no con el ACK de aceptación. El control PWM manual queda excluido durante movimiento por distancia.

Consultar [movimiento y calibración](../firmware/MOVIMIENTO.md) para el protocolo binario, conexión de encoders, límites y puesta en marcha. El comando STOP se reintenta hasta ACK; durante esa espera Qt suspende los heartbeats.

El firmware debe cargarse con `WIFI_SERIAL_DIAGNOSTICS=0` para que el puerto USB transporte exclusivamente tramas UNER. El Monitor Serie debe estar cerrado antes de conectar desde Qt.

En el registro Serial, **TX en cola** indica que Qt aceptó los bytes; **bytes entregados al controlador** confirma la escritura en el COM. La respuesta del Mega aparece como **Serial RX** y luego **ALIVE + ACK**. Abrir el COM puede reiniciar el Mega2560 por DTR; las consultas periódicas permiten retomar el enlace tras el arranque.

## Compilación

Abrir `MCC25.pro` en Qt Creator o ejecutar `qmake MCC25.pro` y `mingw32-make` con Qt 6 y MinGW configurados.

La interfaz anterior se conserva en `legacy/`.


Configuración actual simplificada: cuatro PWM de testeo, cuatro potencias independientes para MOVE y distancia. La telemetría permanece en el registro de Comunicación; no hay tabla de movimiento ni selector de comandos en Configuración. Los nuevos MOVE requieren cargar el firmware que admite cuatro límites de potencia.

La consigna de distancia actual se ingresa en centímetros enteros mediante QSpinBox (-1000..1000 cm). Qt convierte cm a micrómetros (×10000) al enviar MOVE.


La pestaña **Visualización** muestra el recorrido medio firmado en cm calculado por los encoders, el objetivo recibido, el restante y progreso, estado de llegada/cancelación/fallo y una tabla por rueda con ticks, recorrido, restante y PWM aplicado. Usa la escala transmitida por el firmware cuando está disponible; los formatos antiguos usan la escala nominal. Marca las muestras sin actualizar después de 1,5 s. No modifica las ganancias ni la estrategia de frenado; el control de posición reduce velocidad al acercarse al objetivo. El estado Finalizado confirma llegada; un fallo o Detenido no confirma que se haya completado el recorrido.

Modo actual de MOVE: PWM directo por rueda, sin rampa ni frenado progresivo. La pestaña Visualización distingue llegada y exceso de recorrido por inercia (estado 9). Requiere cargar el firmware actualizado.

Modo actual restaurado: MOVE usa rampa de aceleración, sincronización y frenado por encoders. Las cuatro potencias de Configuración son máximos de PWM; se requiere compilar y cargar el firmware restaurado.

Configuración permite aplicar Kp de posición, Kp/Ki de velocidad y Kp de sincronismo mediante SETGAINS (D4), con ACK y sólo detenido. Los cambios duran hasta el reinicio del Mega. La barra superior muestra el recorrido medio firmado del último MOVE en cm y señala telemetría antigua. Se requiere compilar/cargar el firmware actualizado para aplicar los parámetros.
# Desplazamiento longitudinal y lateral

Dirección incluye una sola opción **Diagonal**. Usa FR/RL como par motriz y deja FL/RR libres: con distancia positiva avanza, con negativa recorre la misma línea en sentido contrario. Los cm indican longitud diagonal de trayectoria, con compensación sqrt(2) en la consigna de las ruedas. Visualización identifica las ruedas pasivas con «Libre». Requiere el firmware de `firmware/main` actualizado.

En Configuración, el combo **Dirección** selecciona **Longitudinal** o **Lateral**. La distancia permanece en cm enteros: longitudinal positivo avanza y negativo retrocede; lateral positivo va a izquierda y negativo a derecha, mirando hacia el frente del robot. Se muestran la dirección y los signos esperados FR/FL/RR/RL antes de ejecutar. Pulsar MOVER inicia el recorrido. Los cuatro límites PWM existentes se aplican a ambos ejes y los controles quedan bloqueados durante el movimiento.

La barra superior y Visualización indican X o Y según la telemetría recibida, con recorrido proyectado en ese eje. Los ticks de rueda mantienen su signo y la columna Error eje muestra el restante de cada rueda en el eje de movimiento. Para usar lateral hay que compilar y cargar el firmware actualizado de `firmware/main`. La medición por encoders no incluye el deslizamiento de los rodillos mecanum.

