#include <Arduino.h>

// Tabla de transición en cuadratura 4x: [estado_anterior << 2 | estado_actual]
const int8_t QEM[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// Polaridad de cada encoder: {FR, FL, RR, RL}
const int8_t ENC_INV[4] = {-1, 1, -1, 1};

volatile long ticks[4] = {0, 0, 0, 0}; // 0:FR, 1:FL, 2:RR, 3:RL
volatile uint8_t prev_ab[4] = {0, 0, 0, 0};

ISR(PCINT2_vect) {
  uint8_t pink = PINK; // Lectura directa de A8..A15 (Puerto K)
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t a = (pink >> i) & 0x01;       // Canal A: A8(PK0)..A11(PK3)
    uint8_t b = (pink >> (i + 4)) & 0x01; // Canal B: A12(PK4)..A15(PK7)
    uint8_t curr_ab = (a << 1) | b;
    int8_t delta = QEM[(prev_ab[i] << 2) | curr_ab];
    ticks[i] += delta * ENC_INV[i];
    prev_ab[i] = curr_ab;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Configurar A8..A15 como entradas con pull-up interno activo
  DDRK = 0x00;
  PORTK = 0xFF;

  // Inicializar estado previo real antes de activar interrupciones
  uint8_t pink = PINK;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t a = (pink >> i) & 0x01;
    uint8_t b = (pink >> (i + 4)) & 0x01;
    prev_ab[i] = (a << 1) | b;
  }

  // Habilitar interrupciones por cambio de pin en Puerto K (PCINT16..23)
  PCICR |= (1 << PCIE2);
  PCMSK2 = 0xFF;
}

void loop() {
  static uint32_t last_ms = 0;
  
  // Actualizar el Monitor Serie cada 150 ms
  if (millis() - last_ms >= 150) {
    last_ms = millis();
    
    long c[4]; // Arreglo local para guardar los ticks
    
    // Deshabilitar interrupciones brevemente para copiar los contadores de forma segura
    noInterrupts();
    for (uint8_t i = 0; i < 4; i++) {
      c[i] = ticks[i];
    }
    interrupts();

    // Imprimir exclusivamente los ticks por rueda
    Serial.print("TICKS -> FR: "); 
    Serial.print(c[0]);
    Serial.print(" | FL: "); 
    Serial.print(c[1]);
    Serial.print(" | RR: "); 
    Serial.print(c[2]);
    Serial.print(" | RL: "); 
    Serial.println(c[3]);
  }
}