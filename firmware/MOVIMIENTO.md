## Corrección de escala por distancia física

Se reportaron 49,95 cm indicados frente a aproximadamente 50,5 cm reales. `WHEEL_TRAVEL_SCALE` en `main/config.h` aplica el factor provisional `50.5/49.95` a los milímetros por tick, tanto al calcular objetivos como al integrar posición y enviar telemetría. Para 50 cm se solicitan 2930 ticks en lugar de 2962. Se mantienen las cuentas por vuelta medidas y la tolerancia de 5 ticks. Requiere compilar/cargar el firmware y validar varias repeticiones en avance y retroceso; el factor no elimina variaciones por deslizamiento.

## Corrección actual de frenado y sobrepaso

La salida lógica PWM=0 acciona freno activo en Model Y/PT5126: IN1=IN2=HIGH y EN/PWM=255, tanto por rueda como al parar todas. La integral de velocidad queda limitada a ±50 unidades PWM, con integración condicional que no aumenta una saturación. Al cambiar el signo del error de posición se borran integral y referencia de velocidad y se frena esa rueda durante el ciclo, sin reintegrar inmediatamente. Dentro de ±5 ticks se borran esos acumuladores y se mantiene freno activo. La confirmación global ARRIVED sigue requiriendo 100 ms con las cuatro ruedas en banda. Qt muestra la tolerancia actual de 5 ticks.

Las potencias iniciales para movimiento en Qt son límites máximos FR=170, FL=170, RR=175, RL=185; el PWM de testeo sigue en cero. El sketch ASCII también usa límite integral 50 y reinicio por cambio de signo. Las pruebas host verifican las lógicas y los pines del freno; la distancia real de frenado requiere verificación física y compilación/carga AVR.

## Ajustes PID desde Qt

Configuración incluye cuatro QDoubleSpinBox: Kp posición (0..50), Kp velocidad (0..2), Ki velocidad (0..5) y Kp sincronismo (0..20). APLICAR PID envía UNER D4 + cuatro float32 little-endian en ese orden, longitud 18 y 24 bytes de trama total. Se requiere ACK antes de indicarlos aplicados; NACK o timeout dejan el estado sin confirmar/rechazado. El firmware valida todo antes de actualizar, rechaza cambios durante MOVE o PWM manual activo y borra la integral al aceptar. No se modifica EEPROM. No hay Kd porque la cascada implementada es P exterior y PI interior.

Las potencias de las ruedas y la distancia siguen en sus campos separados. La barra superior contiene la distancia media firmada por encoders respecto al último MOVE, no una suma de todos los trayectos. Marca datos antiguos si faltan muestras durante más de 1,5 s.

## Modo actual restaurado: rampa y control por encoders

Se restauró el modo anterior de aceleración y frenado: P de posición, sincronismo entre ruedas, rampa de velocidad deseada de 3000 ticks/s² y PI de velocidad por rueda. Al aceptar MOVE las salidas empiezan en cero y el lazo de 10 ms construye la rampa; la velocidad deseada disminuye con el error de posición al acercarse al objetivo. La compensación de zona muerta vuelve a FR=57, FL/RR/RL=45.

Las cuatro potencias de Qt vuelven a ser **límites máximos independientes de PWM**, respetados incluso al sumar la zona muerta. No se aplican como potencia fija. Se conservan la consigna en centímetros enteros, conversión y escala, límites de tiempo, paro por falta de progreso, protección de enlace, signos de encoder calibrados y botón Parada. ARRIVED exige un error final de hasta ±5 ticks por rueda (menor en consignas cortas), confirmado tras 100 ms de frenado. Las ruedas quedan enclavadas al entrar en banda o cruzar el objetivo; si al confirmar quedaron fuera de tolerancia se informa OVERSHOOT. Los destinos rectos y de arco usan la distancia equivalente a 5 ticks para el error XY; se eliminó la aceptación anticipada a 30 mm. Los puntos automáticos de evasión conservan un margen de paso independiente para enlazar sin parar.

Las secciones del modo directo que siguen documentan el comportamiento anterior, sustituido por esta restauración. Las pruebas simulan avance y retroceso, aceleración limitada, reducción de la consigna de velocidad cerca del objetivo, límites por rueda y llegada dentro de tolerancia; no equivalen a calibración física.

## Modo anterior: PWM directo sin rampa

MOVE aplica inmediatamente a cada rueda el PWM configurado en Qt (0..255), con el signo de la distancia. Se mantiene ese valor hasta que la rueda alcanza el objetivo con el margen de 3 ticks; entonces se corta su potencia y no vuelve a energizarse durante ese MOVE. Se retiraron rampa de aceleración, frenado proporcional, PI de velocidad, corrección de sincronismo y compensación añadida de zona muerta: PWM 57 significa exactamente 57, no 57 más otra compensación. La conversión de distancia, encoders, polaridades y protecciones permanecen activos.

Sin frenado progresivo, la inercia puede llevar las ruedas más allá del objetivo. ARRIVED requiere que las cuatro queden dentro de ±3 ticks durante 100 ms después del corte. Si todas alcanzaron el umbral pero alguna quedó fuera de tolerancia, el estado **9 OVERSHOOT** informa «Detenido con exceso de recorrido por inercia» en Qt. No se fuerza una corrección a potencia plena en sentido contrario ni se declara llegada precisa. Las descripciones anteriores del control P/PI y los límites adaptativos quedan sustituidas por este modo.

## Estado actual: unidades y polaridad

Qt usa un QSpinBox de **centímetros enteros**, de -1000 a 1000 cm. Al enviar MOVE convierte `cm × 10000` a micrómetros; el formato de comunicación del firmware conserva sus unidades. Consignas 1, 10 y 100 cm generan respectivamente 97, 973 y 9732 ticks por rueda (redondeados). El registro muestra el recorrido medio en cm, con resolución decimal para diagnóstico; la consigna no tiene decimales.

El registro físico de avance `FR/FL/RR/RL = 123/-207/138/-133`, objetivo 973, confirmó un corte DIRECTION_FAULT por los encoders izquierdos invertidos. `ENC_INV` queda `{-1,1,-1,1}`; `DIR_INV` conserva `{-1,-1,-1,-1}`. La corrección se aplica sólo a FL/RL, sin desactivar la protección de polaridad. Debe compilarse y cargarse el firmware para aplicar los signos. El promedio negativo de aquel registro resultaba de sumar cuentas con signos inconsistentes y no era una medición válida del desplazamiento del chasis.

# Movimiento relativo con encoders

## Uso en Qt

Conectar en **Comunicación**. En **Configuración**, ingresar una distancia en mm y pulsar **MOVER DISTANCIA**. Se admiten valores positivos y negativos, tres decimales y un límite explícito de ±10000 mm por movimiento. El límite protege la conversión y el tiempo de ejecución; un valor fuera de rango se rechaza también en el firmware. No se usa una distancia fija en el código.

La referencia se captura respecto a los contadores actuales en cada MOVE. El envío manual de PWM se cancela antes de MOVE y queda deshabilitado mientras el robot informa RUNNING. Las cuatro consignas son iguales para traslación longitudinal de un montaje Mecanum correctamente orientado. No se utiliza un factor de diagonal para este desplazamiento.

La telemetría se publica a 5 Hz: distancia media firmada, objetivo en ticks, ticks relativos FR/FL/RR/RL y estado. ARRIVED se confirma únicamente después de que las cuatro ruedas permanezcan dentro de ±3 ticks durante 100 ms continuos. Los errores y la parada no se presentan como llegada.

STOP está disponible en ambas pestañas. Qt lo reenvía cada 100 ms hasta recibir ACK y suspende ALIVE durante la espera. Si no llegan paquetes, el controlador autónomo también se detiene por pérdida de heartbeat. Es una parada por software y enlace, no un corte eléctrico de emergencia.

## Encoders y cinemática

Mapa de `docs/pinout.pdf`, mirando en sentido de avance:

| Rueda | IN1 / IN2 / PWM | Hall A / B |
| --- | --- | --- |
| FR | D22 / D24 / D9 | A8=PK0 / A12=PK4 |
| FL | D26 / D28 / D10 | A9=PK1 / A13=PK5 |
| RR | D5 / D6 / D11 | A10=PK2 / A14=PK6 |
| RL | D7 / D8 / D12 | A11=PK3 / A15=PK7 |

Hall a +5 V regulados y masa común según el pinout; no a la alimentación de potencia. UART1 conserva D18/TX1 y D19/RX1. No se utiliza `attachInterrupt` ni `digitalRead` para los encoders: `ISR(PCINT2_vect)` captura `PINK` una sola vez, y usa los cuatro pares de bits y una tabla de cuadratura completa. Las transiciones inválidas cuentan cero. La copia al lazo usa `ATOMIC_RESTORESTATE` para preservar el estado previo de interrupciones.

`motion_controller.h` define las constantes constexpr tomadas de las referencias: anillo en eje motor 11 PPR por canal, relación 56:1 y rueda exterior 80,59 mm. Son **2464 ticks/vuelta de rueda**, perímetro `pi*80.59` mm y `round(mm*2464/(pi*80.59))` ticks. Los valores de la ficha `2024005800.pdf` y la imagen `2019018100.jpg` son nominales; la resolución de cuenta no equivale a exactitud sobre el suelo.

La configuración de interrupciones se basa en el [datasheet de Microchip ATmega2560](https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega640-1280-1281-2560-2561-Datasheet-DS40002211A.pdf) y sus registros PCICR, PCMSK2 y PINK.

## Control y calibración

El lazo muestrea cada 10 ms sin `delay`. Un retraso de más de 30 ms detiene el movimiento, sin ejecutar iteraciones ficticias de recuperación. El P de posición genera velocidad deseada, con corrección cruzada respecto a la posición media, límite de velocidad y rampa de aceleración. Un PI de velocidad por rueda, con filtrado y anti-windup, produce PWM firmado con compensación de zona muerta. Las ganancias iniciales están en `motion_controller.h`; son un punto de partida y requieren ajuste físico.

Protecciones: encoder sin cambios durante 1 s fuera de tolerancia, desplazamiento de más de 100 ticks en sentido opuesto al objetivo, heartbeat ALIVE ausente por más de 3 s, período excedido y movimiento de más de 180 s. Todos estos casos llevan PWM a cero y publican su motivo. SETPWM mantiene su timeout independiente de 1,5 s. Un MOVE válido reemplaza la consigna anterior; un comando inválido conserva el movimiento actual.

El límite temporal de MOVE se ajusta al objetivo: `min(180000, 4000 + abs(ticks_objetivo)*1000/300)` ms. Para 10 mm (97 ticks), corta a unos 4,33 s aunque lleguen pulsos espurios que renueven la supervisión de progreso. No representa una velocidad deseada ni sustituye el lazo cerrado; es un límite de fallo conservador, con margen de estabilización. El detector de falta de progreso sigue actuando antes, tras 1 s sin cuatro ticks netos por rueda.

Antes de probar en el piso:

1. Con las ruedas levantadas, verificar cada motor y cada contador. Ajustar **DIR_INV** en `config.h` para que la potencia positiva corresponda al avance del chasis y **ENC_INV** para que ese giro produzca ticks positivos. Ambos arreglos tienen orden FR, FL, RR, RL y admiten 1 o -1. La secuencia A/B de la tabla define el signo inicial, no el cableado físico.
2. Verificar que una vuelta de rueda produce aproximadamente 2464 cuentas: la definición real de PPR del encoder debe coincidir con 11 ciclos por canal y revolución de motor.
3. Ajustar PWM_DEADZONE, ganancias y límites con recorridos cortos en ambos sentidos. Confirmar que frena y estabiliza sin oscilación ni desvío.
4. Medir recorridos reales y calibrar el diámetro efectivo bajo carga. Los rodillos Mecanum, el suelo, la holgura y el deslizamiento afectan la distancia del chasis. Los encoders por sí solos no garantizan distancia exacta ni orientación absoluta; esa validación requiere medición externa.

## Protocolo UNER

Se conserva `UNER + longitud:u8 + ':' + ID + datos + XOR`. La longitud incluye ID, datos y checksum; todos los enteros multibyte van en little-endian. No se mezclan mensajes ASCII en el puerto binario.

| ID | Petición / respuesta |
| --- | --- |
| A0 ALIVE | Sin datos; respuesta ID + 0D ACK |
| A4 SETPWM | Cuatro int16 de -255 a 255, FR/FL/RR/RL; cancela posición |
| D0 MOVE | Un int32 en **micrómetros** (mm × 1000), seguido de cuatro u8 con límites PWM FR/FL/RR/RL 0..255. Longitud=10; 16 bytes totales. Se aceptan formatos anteriores: longitud 7 aplica un límite común, longitud 6 conserva 180 en las cuatro ruedas |
| D1 STOP | Sin datos. Longitud=2; 8 bytes totales. Cancela ambos modos y desactiva motores |
| D2 MOTION_STATUS | Sólo salida: estado u8, objetivo int32 ticks, distancia media int32 micrómetros, cuatro int32 ticks relativos; luego PINK u8, cancelación u8, máscara de ruedas con fallo u8 y, por rueda, flancos A u16, flancos B u16 y transiciones inválidas u16. Añade cuatro PWM int16 firmados, escala ticks/mm × 1000000 int32 y diámetro en micrómetros int32. Payload incluyendo ID=69; 76 bytes totales. Qt también acepta los formatos anteriores de 26 y 53 bytes |

MOVE/STOP/SETPWM responden ID + 0D ACK o 0E NACK. ACK de MOVE confirma aceptación, **no llegada**. El estado periódico confirma llegada aunque un datagrama de telemetría se pierda. No hay retransmisión automática de MOVE para evitar reiniciar la referencia por duplicados.

Estados: 0 inactivo, 1 RUNNING, 2 ARRIVED, 3 STOPPED, 4 STALLED, 5 LINK_LOST, 6 TIMING_FAULT, 7 MOVE_TIMEOUT, 8 DIRECTION_FAULT. No se deben interpretar como MOVE los datos de las versiones históricas que usaban estos IDs.

## Diagnóstico de actividad Hall y cancelaciones

El caso `10 mm → objetivo 97; 66/-1/0/-1 ticks → 1,644 mm` es aritméticamente consistente: el promedio es 16 ticks. Esa media no confirma el desplazamiento físico del chasis ni una llegada. STOPPED significa cancelación por STOP o PWM, no ARRIVED. La extensión identifica cancelación 1=STOP recibido y 2=PWM manual recibido. Un STOP posterior a un fallo conserva el fallo original hasta el siguiente MOVE. El PWM manual no nulo se rechaza durante RUNNING, para que paquetes manuales pendientes no reemplacen la consigna autónoma.

Con firmware actualizado, Qt muestra para cada rueda actividad A/B y transiciones de cuadratura inválidas. Los contadores de diagnóstico se referencian al último MOVE (antes del primero, al arranque) y se expresan módulo 65536. La máscara de fallo usa bits 0..3 para FR/FL/RR/RL. El promedio de encoders se rotula como tal y no como una medición independiente de la distancia del chasis.

- A cambia y B no: revisar el canal B de esa rueda. La situación inversa exige revisar A. Un solo canal puede hacer oscilar el contador entre valores adyacentes, aunque la rueda gire.
- Ninguno cambia: puede ser una rueda que no gira o falta de alimentación, masa o ambas señales. No es prueba suficiente de encoder averiado.
- Ambos cambian y hay muchas transiciones inválidas: revisar pareja de canales, conexión, ruido y pérdida de interrupciones. La tabla descarta cambios simultáneos de ambos bits; no inventa ticks.
- Ambos cambian con signo contrario al avance: verificar ENC_INV individualmente. Cambiar el signo no soluciona ausencia de pulsos.

El detector de atasco requiere un desplazamiento neto de al menos cuatro ticks para renovar el tiempo de progreso: el ruido de un tick no puede mantenerlo activo indefinidamente. Las entradas digitales del Puerto K se habilitan explícitamente mediante DIDR2=0. Se conservan pines, relación de reducción y decodificación 4×.

Para comprobar señales, levantar las ruedas y, sin activar el motor, girar manualmente cada rueda varios grados, observando actividad A y B en la fila correspondiente. Después verificar por separado si gira con PWM bajo. Confirmar masa común, alimentación Hall y continuidad hasta los ocho pines antes de volver a ajustar ganancias o la escala de distancia. Esta revisión distingue lectura de señal y accionamiento; requiere observación física.

## Verificación

`firmware/tests/test_motion.cpp` verifica cuadratura 4×, conversión, simulación de ruedas con respuestas distintas, avance/retroceso, tolerancia sostenida y fallos. Compilar con C++17 e incluir `firmware/main`. `test_commands.c` conserva pruebas del parser UNER y PWM.

`software/tests/test_motion_ui.pro` permite compilar la prueba Qt: comprueba unidades y signo del paquete MOVE, exclusión de PWM manual, telemetría de cuatro ruedas, llegada y STOP. Puede ejecutarse con `QT_QPA_PLATFORM=offscreen`, sin accionar el robot. Las simulaciones y pruebas de software no sustituyen las pruebas físicas de precisión, tasa de interrupciones ni carga del firmware.


## Indicadores de recorrido y mínimo de arranque de FR

Configuración presenta una vista previa de la consigna en mm/cm y ticks esperados, objetivo recibido redondeado a ticks, promedio recorrido por encoders en mm/cm, error medio, diferencia máxima entre ruedas y tolerancia. La tabla FR/FL/RR/RL muestra ticks, distancia calculada, error de posición, vueltas de rueda, PWM enviado al puente H y porcentaje de progreso (acotado a 0..100; no confirma llegada). Las distancias son netas y firmadas respecto a la referencia del último MOVE, no un acumulado de trayecto absoluto.

La escala y diámetro se transmiten desde el firmware; Qt no depende de una copia distinta de la calibración para interpretar el formato nuevo. El PWM mostrado es la orden lógica firmada que llega a la función de motores, tanto en manual como en automático, antes de DIR_INV; no mide corriente ni tensión real. Con firmware antiguo aparece «—» en la columna PWM. El objetivo recibido se deriva de los ticks redondeados, y puede diferir ligeramente de la distancia editada.

FR requiere PWM manual 57 para arrancar, según la prueba del usuario. `PWM_DEADZONE_BY_WHEEL` en `motion_controller.h` queda `{57,45,45,45}`: sólo cambia FR. No se modifican sus pines ni polaridades. Este ajuste puede resolver falta de arranque por potencia insuficiente, pero no valida los encoders ni garantiza funcionamiento físico bajo carga. El resto de ganancias y protecciones permanecen activos.

La trama extendida supera el buffer TX hardware del Mega: `main.ino` conserva el frame pendiente y escribe sólo los bytes disponibles en cada vuelta, sin mezclar ACK dentro de una trama ni detener la recepción o el controlador. El puente Wi-Fi conserva el envío del paquete completo mediante el buffer AT existente.


## Potencia configurable para MOVE

En Configuración, **Potencia máxima para mover distancia** admite 0..255 y empieza en 180. El valor se envía junto con la distancia al pulsar MOVER DISTANCIA; editarlo no altera una maniobra que ya esté en marcha. La magnitud del PWM aplicado por rueda no supera este límite, incluida la compensación de zona muerta, en ninguno de los dos sentidos. El controlador sigue ajustando cada motor para sincronizar y frenar: no es una potencia constante ni una consigna de velocidad.

Límite 0 acepta el comando, desactiva los motores y queda STOPPED, sin simular una llegada. Un límite menor al PWM necesario para arrancar (FR=57 actualmente) puede producir una parada por falta de progreso. El parámetro afecta sólo al modo de distancia, no al PWM manual independiente de cada rueda.


## Configuración simplificada actual

La pestaña Configuración contiene únicamente cuatro QSpinBox de PWM de testeo (-255..255), el botón ENVIAR PWM DE TESTEO, cuatro QSpinBox de potencia de movimiento (0..255, uno por rueda), el campo de distancia en mm y los botones MOVER DISTANCIA y DETENER. Se retiraron el bloque de telemetría, tabla, previsualización, diagnóstico y selector de comandos; ALIVE continúa automáticamente y la telemetría de estado/distancia/ticks se registra en Comunicación.

Las cuatro potencias de MOVE son límites individuales del lazo cerrado y se envían juntas con la distancia. No se fuerzan como PWM constante: la corrección y frenado siguen activos. Un límite cero mantiene esa rueda sin potencia; si las demás se accionan y no se alcanza la consigna de todas, se aplica la protección de falta de progreso. Con las cuatro potencias en cero el comando queda detenido inmediatamente. Se conserva el mínimo de arranque de FR=57, siempre subordinado al límite elegido para esa rueda.
