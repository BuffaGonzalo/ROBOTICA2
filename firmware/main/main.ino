/*
 * OSOYOO FlexiRover 2024007400 / Arduino Mega 2560 / ESP-01 AT.
 * ALIVE y PWM de cuatro ruedas por Wi-Fi UDP y USB serial del Mega2560.
 */

#include <Arduino.h>
#include "config.h"
#include "flexirover_link.h"
#include "unerPrtcl.h"
#include "util.h"
#include "robot_commands.h"
#include "motion.h"
#include "mecanum_kinematics.h"

/* Pinout docs/pinout.pdf, orden FR, FL, RR, RL. */
static const uint8_t MOTOR_IN1[4] = {22, 26, 5, 7};
static const uint8_t MOTOR_IN2[4] = {24, 28, 6, 8};
static const uint8_t MOTOR_PWM[4] = {9, 10, 11, 12};

static uint32_t last_tick_ms;
static _sComm serial_rx, serial_tx;
static volatile uint8_t serial_rx_buffer[RXBUFSIZE], serial_tx_buffer[TXBUFSIZE];
static bool serial_alive_pending;
static bool serial_pwm_pending;
static uint8_t serial_alive_status = ACK, serial_pwm_status = ACK;
static uint8_t serial_reply_id = SETPWM;
static bool serial_seen;
static uint32_t serial_telemetry_ms;
static uint8_t serial_status_remaining, serial_status_offset;

static bool sendSerialReply(uint8_t command, uint8_t status) {
  if (serial_status_remaining) return false; // Do not interleave binary frames.
  if (Serial.availableForWrite() < 9) return false;
  serial_tx.indexW = serial_tx.indexR = 0;
  unerPrtcl_PutHeaderOnTx(&serial_tx, command, 2);
  unerPrtcl_PutByteOnTx(&serial_tx, status);
  unerPrtcl_PutByteOnTx(&serial_tx, serial_tx.chk);
  return Serial.write(serial_tx.buff, serial_tx.nBytes) == serial_tx.nBytes;
}

static void applyMotorPwm(const int16_t *powers) {
  const uint8_t coastMask=Motion_CoastMask();
  Motion_RecordPwm(powers); // Esfuerzo lógico 0 = freno activo, no rueda libre.
  for (uint8_t i = 0; i < 4; ++i) {
    analogWrite(MOTOR_PWM[i], 0); // Deshabilitar antes de cambiar el puente.
    const int16_t value = powers[i] * DIR_INV[i];
    if (value == 0 && (coastMask & (1u<<i))) {
      digitalWrite(MOTOR_IN1[i], LOW);
      digitalWrite(MOTOR_IN2[i], LOW);
      // PWM ya esta en cero: rueda libre para el par diagonal no motriz.
    } else if (value == 0) {
      // 1. HARD BRAKE: IN1=IN2=HIGH, EN/PWM=255 para FR, FL, RR y RL.
      digitalWrite(MOTOR_IN1[i], HIGH);
      digitalWrite(MOTOR_IN2[i], HIGH);
      analogWrite(MOTOR_PWM[i], 255);
    } else {
      digitalWrite(MOTOR_IN1[i], value > 0 ? HIGH : LOW);
      digitalWrite(MOTOR_IN2[i], value < 0 ? HIGH : LOW);
      analogWrite(MOTOR_PWM[i], value < 0 ? -value : value);
    }
  }
}

static void stopMotors() {
  const int16_t stopped[4] = {0, 0, 0, 0};
  applyMotorPwm(stopped); // Frenado eléctrico de las cuatro ruedas.
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
  RobotCommands_Init(applyMotorPwm);
  Motion_Init(applyMotorPwm);
  RobotCommands_AttachMotion(Motion_Command);
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
    uint8_t status = RobotCommands_Handle(&serial_rx, millis());
    uint8_t id = serial_rx.buff[serial_rx.indexData];
    if(status) serial_seen=true;
    if (status && id == ALIVE) {
      serial_alive_pending = true;
      serial_alive_status = status;
    } else if (status && (id == SETPWM || id == MOVE || id == STOP || id == SETGAINS)) {
      serial_pwm_pending = true;
      serial_reply_id = id;
      serial_pwm_status = status;
    }
  }
  // Si el buffer TX USB estaba lleno, conservar la respuesta y reintentar.
  if (serial_alive_pending && sendSerialReply(ALIVE, serial_alive_status))
    serial_alive_pending = false;
  if (serial_pwm_pending && sendSerialReply(serial_reply_id, serial_pwm_status))
    serial_pwm_pending = false;
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
  RobotCommands_Task(now);
  Motion_Task(now);
  if(serial_seen && !serial_status_remaining && !serial_alive_pending && !serial_pwm_pending && (uint32_t)(now-serial_telemetry_ms)>=200u) {
    uint8_t payload[MOTION_TELEMETRY_SIZE];
    uint8_t length=Motion_Telemetry(payload);
    serial_tx.indexW=serial_tx.indexR=0;
    unerPrtcl_PutHeaderOnTx(&serial_tx,MOTION_STATUS,length+1);
    for(uint8_t i=0;i<length;++i) unerPrtcl_PutByteOnTx(&serial_tx,payload[i]);
    unerPrtcl_PutByteOnTx(&serial_tx,serial_tx.chk);
    serial_status_offset=0;
    serial_status_remaining=serial_tx.nBytes;
    serial_telemetry_ms=now;
  }
  // Telemetry exceeds the Mega UART TX capacity. Drain available space only.
  if(serial_status_remaining) {
    uint8_t count=(uint8_t)min((int)serial_status_remaining,Serial.availableForWrite());
    if(count) {
      uint8_t written=(uint8_t)Serial.write(serial_tx.buff+serial_status_offset,count);
      serial_status_offset+=written;
      serial_status_remaining-=written;
    }
  }
}
