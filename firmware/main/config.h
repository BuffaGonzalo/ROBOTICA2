#ifndef FLEXIROVER_CONFIG_H
#define FLEXIROVER_CONFIG_H

/* Editar antes de cargar el sketch. La PC ejecuta la aplicación Qt de software/. */
#define WIFI_SSID       "LabPrototip"
#define WIFI_PASSWORD   "labproto"
#define PC_IP           "172.24.150.142"
#define PC_UDP_PORT     30010
#define ESP_UDP_PORT    30001

/* UART hardware del Mega2560: TX1=18, RX1=19. Debe coincidir con ESP-01 AT. */
#define ESP_BAUD        115200UL

/* USB Serial se reserva para tramas UNER binarias. Poner 1 sólo para
   diagnóstico con Monitor Serie; ese texto interfiere con el enlace Serial. */
#define WIFI_SERIAL_DIAGNOSTICS 0

/* -1: CH_PD/EN del ESP-01 conectado permanentemente a 3,3 V.
   Si está cableado a un pin del Mega mediante adaptación de nivel, indicar el pin. */
#define ESP_EN_PIN      (-1)

#endif
