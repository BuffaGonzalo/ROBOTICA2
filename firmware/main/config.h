#ifndef FLEXIROVER_CONFIG_H
#define FLEXIROVER_CONFIG_H

/* Editar antes de cargar el sketch. La PC ejecuta la aplicación Qt de software/. */
#define WIFI_SSID       "MEGAFIBRA-2.4G-exN4"
#define WIFI_PASSWORD   "A43yHNsa"
#define PC_IP           "192.168.18.4"
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

/* Calibración reportada: cuentas medidas por una vuelta completa de RUEDA.
   Valor común provisional; confirmar el mismo conteo en FR, FL, RR y RL.
   No multiplicar nuevamente por 56 ni por 4: la medición ya los incluye. */
#define ENCODER_TICKS_PER_WHEEL_REV 1500.0

/* Cinematica mecanum, en mm: medir antes de solicitar rotacion.
   l = mitad de distancia entre ejes; w = mitad de distancia entre ruedas.
   Cero significa sin medir (permite traslacion, rechaza rotacion). */
#define MECANUM_HALF_WHEELBASE_MM 0.0f
#define MECANUM_HALF_TRACK_MM     0.0f

/* FR, FL, RR, RL. Motores: positivo corresponde a avance físico.
   Encoders calibrados con avance: FR/RR ya contaban positivo;
   FL/RL contaban negativo y se invierten independientemente del motor. */
const int8_t DIR_INV[4] = {-1, -1, -1, -1};
const int8_t ENC_INV[4] = {-1, 1, -1, 1};

#endif
