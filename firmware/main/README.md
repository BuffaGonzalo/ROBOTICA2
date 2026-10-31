# Firmware de comunicación para Mega2560

Diagonales de 45° en MOVE: eje 2 = diagonal izquierda (FR/RL motrices), eje 3 = diagonal derecha (FL/RR motrices). Positivo avanza por esa diagonal y negativo recorre la misma línea hacia atrás. Para cambiar de par se cambia la opción del selector. La distancia ingresada es la longitud de trayectoria del chasis, no su componente X/Y: las ruedas motrices recorren sqrt(2) veces esa distancia, según H(0). Telemetría y Qt convierten nuevamente al recorrido del chasis. El PID, el sincronismo y las protecciones de encoder consideran únicamente el par activo. Las ruedas no motrices quedan libres (PWM=0, IN1=IN2=LOW) durante la maniobra; llegada, STOP y fallos restauran freno activo en las cuatro ruedas. No se modifica la calibración ni las ganancias del PI. Los límites PWM se aplican al par activo, sin multiplicar arbitrariamente su potencia por sqrt(2).

Movimiento por distancia conforme a Modern Robotics, ecuación 13.10: potencias como límites de magnitud 0..255; los sentidos se obtienen del eje y el signo de distancia. Para +Y, FR/FL/RR/RL = +/−/−/+. Se eliminó la inversión individual por potencia negativa en MOVE y su formato extendido de longitud 15. El PWM manual de pruebas conserva valores −255..255. Los rodillos de ±45 grados ya están considerados en H(0), sin multiplicadores arbitrarios sobre el PWM. El PI calcula el esfuerzo dentro de los límites configurados; determinar la potencia necesaria exige mediciones bajo carga.

## Cinemática inversa mecanum

### Desplazamiento lateral por distancia desde Qt

Qt selecciona con `comboBox_moveDirection` la dirección **Longitudinal** (X, + adelante / − atrás) o **Lateral** (Y, + izquierda / − derecha). El MOVE longitudinal conserva su formato anterior. El lateral usa el mismo ID 0xD0 y agrega un byte de eje después de las cuatro potencias: `ID | distancia_um:i32LE | FR:u8 | FL:u8 | RR:u8 | RL:u8 | eje:u8`. La longitud UNER es 11 (incluye checksum). Eje inválido se rechaza sin alterar el movimiento. El firmware anterior rechaza esta longitud, sin interpretar el pedido lateral como un avance.

Para Y positivo las ruedas FR/FL/RR/RL reciben signos `+ / − / − / +`, derivados de H(0). Los encoders se proyectan sobre el eje elegido antes de entrar al controlador existente; su salida vuelve a convertirse al sentido físico de cada rueda. Se conservan ganancias, límites PWM individuales, rampas, freno activo, sincronismo y protecciones. No se necesita l/w para traslación pura.

MOTION_STATUS agrega el eje en el byte 92 de datos (93 bytes de datos, 94 con ID). La distancia media se proyecta sobre X o Y, mientras que ticks, velocidades y PWM por rueda conservan sus signos físicos. Qt acepta la telemetría anterior como longitudinal. En lateral, el error por rueda se calcula contra su meta proyectada, evitando promediar a cero dos pares que giran en sentidos opuestos. La distancia por encoders es una estimación cinemática: no mide deslizamiento lateral del chasis.

`mecanum_kinematics.h`, incluido por `main.ino`, ofrece `Mecanum::inverse(wz, vx, vy, setpoints)`: recibe rad/s y mm/s, y devuelve velocidades de rueda en rad/s y ticks/s, siempre en orden **FR, FL, RR, RL**. Los ejemplos de avance, desplazamiento lateral izquierdo y giro están al final del archivo. Esta función calcula referencias; no agrega comandos Serial/UDP ni activa motores. El modo MOVE existente sigue usando su controlador de posición. Para un futuro modo de velocidad del chasis, las referencias calculadas deben alimentar el PI en lugar de la referencia generada por posición, conservando las protecciones y la rampa.

Se conserva la matriz H(0) solicitada y se corrige su asociación de filas: **FL, FR, RR, RL**. Asociar sus filas a FL, RL, RR, FR haría que un giro puro ordenase el mismo signo a las dos ruedas delanteras, lo cual no representa el giro sobre el centro. Ejes: x hacia delante, y a izquierda y giro positivo antihorario, visto desde arriba. Se supone el montaje de rodillos de la ecuación 13.10. Referencia: [Modern Robotics, capítulo 13.2](https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2/).

Medir y configurar `MECANUM_HALF_WHEELBASE_MM` (l) y `MECANUM_HALF_TRACK_MM` (w) en `config.h`: ambos son **semidistancias**, en mm. Inicialmente valen cero porque aún no se midieron; la función rechaza giros hasta disponer de ambas medidas y devuelve salidas cero con `false`. También se pueden pasar l y w como últimos argumentos. Las traslaciones no necesitan esas medidas. Radio: 40,295 mm. La conversión usa la calibración vigente de 1500 ticks/vuelta, sin modificar PID ni invertir de nuevo los signos de motor/encoder. `Mecanum::limitSpeed(setpoints, limiteTicksS)` permite escalar conjuntamente las cuatro referencias para conservar la dirección del movimiento cuando se excede el límite de velocidad.

## Calibración actual: 1500 ticks por vuelta de rueda

Se usa `ENCODER_TICKS_PER_WHEEL_REV=1500.0` en `config.h`, según la medición reportada por el usuario. La cifra teórica 11×56×4=2464 se conserva separada como nominal; no se vuelve a multiplicar la medición de rueda por 56 ni por 4. Con diámetro 80,59 mm, la escala es aproximadamente 5,924616 ticks/mm: 10 cm → 592 ticks y 45 cm → 2666 ticks. El firmware transmite esta escala a Qt para calcular los recorridos.

Es una calibración común provisional: comprobar el mismo conteo de vuelta completa en las cuatro ruedas y repetir a distintas velocidades antes de considerarla definitiva. Si el conteo cambia con la velocidad, investigar señal/lectura en lugar de compensar pérdidas mediante un factor fijo. Se mantienen los signos de los encoders, las ganancias configuradas, el freno activo y la banda de cinco ticks. Los límites de velocidad/aceleración y el tiempo máximo relativo se convierten a la nueva unidad de cuentas para conservar sus valores físicos previstos. Es necesario compilar y cargar el sketch de `firmware/main`.

El sketch `main.ino` intercambia tramas UNER por el USB `Serial` del Mega2560 a **115200 baudios, 8N1** y por Wi-Fi UDP mediante un ESP-01 conectado a `Serial1`. Reconoce `ALIVE (0xA0)`, `SETPWM (0xA4)`, `MOVE (0xD0)` y `STOP (0xD1)`, y publica `MOTION_STATUS (0xD2)`. Los comandos históricos no se procesan.

## Control de ruedas

`SETPWM` aplica cuatro valores **int16 little-endian entre -255 y 255**, en orden **FR, FL, RR, RL** (delantera derecha, delantera izquierda, trasera derecha, trasera izquierda). El valor absoluto es el duty PWM de 8 bits; el signo selecciona el sentido y cero desactiva PWM y ambas entradas del puente H. Es control de potencia en lazo abierto, sin regulación de RPM ni lectura de encoders.

Pines según `docs/pinout.pdf`:

| Rueda | IN1 | IN2 | PWM |
| --- | --- | --- | --- |
| FR | D22 | D24 | D9 |
| FL | D26 | D28 | D10 |
| RR | D5 | D6 | D11 |
| RL | D7 | D8 | D12 |

`DIR_INV` en `config.h` permite invertir el signo de cada motor (1 o -1) según el montaje. Inicialmente todos valen 1: positivo activa IN1 y desactiva IN2. Confirmar físicamente el sentido de cada rueda antes de asumir que cuatro valores positivos producen avance.

Qt muestra sólo ALIVE y SETPWM. Al enviar SETPWM, renueva las cuatro potencias cada **250 ms** y aplica los cambios de los controles mientras el envío esté activo. **DETENER TODAS LAS RUEDAS** envía STOP y cancela la renovación; desconectar o cerrar también intenta detener el robot. El firmware detiene todos los motores tras **1500 ms sin un SETPWM válido**, aunque siga recibiendo ALIVE. Arranca siempre detenido.

Petición PWM: `55 4E 45 52 0A 3A A4 <FRlo FRhi FLlo FLhi RRlo RRhi RLlo RLhi> <XOR>`. La trama completa tiene 16 bytes. Por ejemplo, cuatro valores +100: `55 4E 45 52 0A 3A A4 64 00 64 00 64 00 64 00 98`.

Respuesta: ID del comando y `ACK=0x0D` al aplicar; `NACK=0x0E` si la longitud o algún valor son inválidos. Ninguna rueda se actualiza si un valor es inválido. Una trama con checksum incorrecto se descarta sin respuesta. ALIVE requiere longitud 2 (comando y checksum), SETPWM longitud 10. Los comandos desconocidos se ignoran.

La respuesta Serial queda pendiente si el buffer USB de salida está ocupado y se reintenta en el siguiente ciclo. Hay que **volver a cargar este sketch** en el Mega2560 para que ese cambio y el ajuste de diagnósticos tengan efecto.

En `config.h`, `WIFI_SERIAL_DIAGNOSTICS` queda en **0** para que los mensajes de diagnóstico Wi-Fi no se mezclen con las tramas binarias del puerto USB. Para diagnosticar el ESP-01 en Monitor Serie se puede cambiar temporalmente a 1, pero hay que volver a 0 y cargar el sketch antes de usar Serial en la aplicación Qt. Cerrar el Monitor Serie para liberar el COM.

## Conexión

- **USB Serial:** conectar el Mega2560, elegir **Serial (USB)** y su COM en la aplicación Qt de `software/`, y abrir el puerto. La aplicación envía GETALIVE al conectar y cada segundo. El firmware devuelve ALIVE + ACK.
- **Wi-Fi UDP:** configurar `WIFI_SSID`, `WIFI_PASSWORD` y `PC_IP` en `config.h`. La PC escucha en `PC_UDP_PORT=30010`; el ESP-01 usa `ESP_UDP_PORT=30001`. La aplicación detecta la IP del robot a partir del primer datagrama.
- El ESP-01 se conecta a `Serial1`: D18/TX1 al RX del módulo con adaptación de 5 V a 3,3 V, TX del módulo a D19/RX1, masa común y alimentación estable de 3,3 V. `ESP_BAUD=115200` debe coincidir con el firmware AT del módulo.

## Tramas de referencia

UNER usa el encabezado `55 4E 45 52`, un byte de longitud que cuenta payload y checksum, el separador `3A` y XOR de todos los bytes anteriores.

- Consulta desde la PC: `55 4E 45 52 02 3A A0 94`.
- Respuesta del Mega2560: `55 4E 45 52 03 3A A0 0D 98`.

La verificación definitiva del enlace requiere el Mega2560 y el ESP-01 conectados físicamente.


## Diagnóstico de distancia física frente a odometría

Si Qt indica 10 cm y ARRIVED pero el chasis recorrió 15 cm, la lectura usada para cerrar el lazo subestima el desplazamiento físico. No demuestra por sí sola pérdida de pulsos: hay que distinguir escala, cuentas omitidas y deslizamiento. Las mediciones aproximadas 10→15 y 45→60 tienen factores diferentes (1,50 y 1,33), insuficientes para fijar una corrección única sin más mediciones.

La escala nominal sigue en 2464 ticks por vuelta de rueda y diámetro 80,59 mm. Antes de calibrar, medir la diferencia de ticks por cada vuelta completa y lenta de FR/FL/RR/RL, conservando sus signos. Repetir con mayor velocidad: si el conteo depende de la velocidad, investigar señales e interrupciones antes de cambiar la conversión. Una lectura de transiciones inválidas igual a cero no descarta que se hayan perdido ciclos completos. Registrar también distancia física y odometría final en ambos sentidos.

La telemetría actual conserva todo lo anterior y agrega velocidad objetivo, velocidad medida y aporte integral por rueda. Qt los muestra y registra durante movimiento, y muestra flancos A/B y transiciones inválidas. La integral está en unidades PWM, con límite ±50; las velocidades están en ticks/s. Cuando termina el movimiento no se presenta la velocidad del último ciclo como una medición nueva. No se cambiaron ganancias ni la escala con base en estas dos mediciones físicas aproximadas. Estas herramientas permiten comprobar si el PI sigue la referencia o si la salida permanece saturada en el límite de potencia.


Formato extendido MOTION_STATUS actual: 92 bytes de datos (93 incluyendo ID, 100 bytes de trama). Conserva los 68 bytes anteriores y agrega por rueda tres int16: velocidad objetivo (ticks/s), velocidad medida (ticks/s) e integral (PWM × 100). Serial drena la trama por fragmentos sin bloquear. Qt acepta también las versiones anteriores.
