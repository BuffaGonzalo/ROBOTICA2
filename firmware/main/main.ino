/*
 * OSOYOO FlexiRover 2024007400 / Arduino Mega 2560 / ESP-01 AT.
 * ALIVE UNER por Wi-Fi Station UDP y USB serial del Mega2560.
 */

#include <Arduino.h>
#include "config.h"
#include "flexirover_link.h"
#include "unerPrtcl.h"
#include "util.h"

/* Pinout de los motores tomado del firmware previo del proyecto.
   FR, FL, RR, RL. Se dejan inmóviles hasta implementar desplazamiento. */
static const uint8_t MOTOR_IN1[4] = {22, 26, 5, 7};
static const uint8_t MOTOR_IN2[4] = {24, 28, 6, 8};
static const uint8_t MOTOR_PWM[4] = {9, 10, 11, 12};

static uint32_t last_tick_ms;
static _sComm serial_rx, serial_tx;
static volatile uint8_t serial_rx_buffer[RXBUFSIZE], serial_tx_buffer[TXBUFSIZE];
static bool serial_alive_pending;

static bool sendSerialAlive() {
  if (Serial.availableForWrite() < 9) return false;
  serial_tx.indexW = serial_tx.indexR = 0;
  unerPrtcl_PutHeaderOnTx(&serial_tx, ALIVE, 2);
  unerPrtcl_PutByteOnTx(&serial_tx, ACK);
  unerPrtcl_PutByteOnTx(&serial_tx, serial_tx.chk);
  return Serial.write(serial_tx.buff, serial_tx.nBytes) == serial_tx.nBytes;
}

static void stopMotors() {
  for (uint8_t i = 0; i < 4; ++i) {
    analogWrite(MOTOR_PWM[i], 0);
    digitalWrite(MOTOR_IN1[i], LOW);
    digitalWrite(MOTOR_IN2[i], LOW);
  }
}

static void setEspEnabled(uint8_t on) {
#if ESP_EN_PIN >= 0
  digitalWrite(ESP_EN_PIN, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

static int writeEspByte(uint8_t byte) {
  if (Serial1.availableForWrite() < 1) return 0;
  return Serial1.write(byte) == 1 ? 1 : 0;
}

static void logWifi(const char *message) {
#if WIFI_SERIAL_DIAGNOSTICS
  if (message) Serial.println(message);
#else
  (void)message;
#endif
}

void setup() {
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
    pinMode(MOTOR_PWM[i], OUTPUT);
  }
  stopMotors();
#if ESP_EN_PIN >= 0
  pinMode(ESP_EN_PIN, OUTPUT);
  digitalWrite(ESP_EN_PIN, HIGH);
#endif
  Serial.begin(115200);
  Serial1.begin(ESP_BAUD);
  unerPrtcl_Init(&serial_rx, &serial_tx, serial_rx_buffer, serial_tx_buffer);
  FlexiLink_Init(setEspEnabled, writeEspByte, logWifi,
                 WIFI_SSID, WIFI_PASSWORD, PC_IP, PC_UDP_PORT, ESP_UDP_PORT);
#if WIFI_SERIAL_DIAGNOSTICS
  Serial.println(F("[WIFI] Iniciando ESP-01 por Serial1"));
  Serial.print(F("[UDP] Destino PC: "));
  Serial.print(PC_IP);
  Serial.print(':');
  Serial.println(PC_UDP_PORT);
#endif
  last_tick_ms = millis();
}

void loop() {
  while (Serial.available() > 0) {
    uint8_t byte = (uint8_t)Serial.read();
    uint8_t next = (uint8_t)((serial_rx.indexW + 1u) & serial_rx.mask);
    if (next != serial_rx.indexR) {
      serial_rx.buff[serial_rx.indexW] = byte;
      serial_rx.indexW = next;
    }
  }
  while (unerPrtcl_DecodeHeader(&serial_rx)) {
    if (serial_rx.buff[serial_rx.indexData] == ALIVE) {
      serial_alive_pending = true;
    }
  }
  // Si el buffer TX USB estaba lleno, conservar la respuesta y reintentar.
  if (serial_alive_pending && sendSerialAlive())
    serial_alive_pending = false;
  /* Primero drenar UART para evitar desbordes del buffer hardware de Serial1. */
  while (Serial1.available() > 0) {
    FlexiLink_UartRx((uint8_t)Serial1.read());
  }
  uint32_t now = millis();
  while ((uint32_t)(now - last_tick_ms) >= 10u) {
    last_tick_ms += 10u;
    FlexiLink_Tick10ms();
  }
  FlexiLink_Task(now);
}
