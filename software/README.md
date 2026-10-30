# MICRO-Software

Aplicación Qt para comunicar la PC con el Mega2560 por **Serial USB** o **Wi-Fi UDP**.

La pestaña **Comunicación** permite elegir el medio, seleccionar un COM o el puerto UDP local, conectar y consultar el estado con `GETALIVE`. En Serial usa **115200 baudios, 8N1, sin control de flujo**. En UDP escucha en el puerto 30010 y aprende la IP y el puerto del robot al recibir su primer datagrama. La aplicación envía `GETALIVE` al conectar y cada segundo mientras el enlace esté abierto y haya destino.

El selector de comandos recupera los identificadores y significados de la interfaz anterior. El firmware actual sólo implementa `GETALIVE (0xA0)`; los demás figuran como referencia y su envío permanece deshabilitado. La pestaña **Registro de comunicación** muestra bytes y comandos decodificados con sus nombres.

El firmware debe cargarse con `WIFI_SERIAL_DIAGNOSTICS=0` para que el puerto USB transporte exclusivamente tramas UNER. El Monitor Serie debe estar cerrado antes de conectar desde Qt.

En el registro Serial, **TX en cola** indica que Qt aceptó los bytes; **bytes entregados al controlador** confirma la escritura en el COM. La respuesta del Mega aparece como **Serial RX** y luego **ALIVE + ACK**. Abrir el COM puede reiniciar el Mega2560 por DTR; las consultas periódicas permiten retomar el enlace tras el arranque.

## Compilación

Abrir `MCC25.pro` en Qt Creator o ejecutar `qmake MCC25.pro` y `mingw32-make` con Qt 6 y MinGW configurados.

La interfaz anterior se conserva en `legacy/`.
