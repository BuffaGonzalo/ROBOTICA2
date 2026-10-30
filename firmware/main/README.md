# Firmware de comunicación para Mega2560

El sketch `main.ino` intercambia tramas UNER por el USB `Serial` del Mega2560 a **115200 baudios, 8N1** y por Wi-Fi UDP mediante un ESP-01 conectado a `Serial1`. El firmware actual sólo reconoce `ALIVE / GETALIVE (0xA0)` y responde con `ALIVE + ACK (0x0D)`. Todavía no procesa los demás comandos del catálogo histórico ni controla los motores.

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
