/*
  ROBOT SUMO — ESP32-CAM + PS4 + BTS7960 + JGA25-370 170RPM 12V + SERVO DS3218MG
  ================================================================================
  Motor: JGA25-370  |  12V  |  170 RPM  |  Alto torque
  Driver: BTS7960 (43A pico) — R_EN y L_EN a 5V fijo
  Servo: DS3218MG  |  Rango: 0-180°  |  PWM 50Hz en IO14

  Pines:
    IO13 → RPWM Motor 1 — llanta izquierda
    IO12 → LPWM Motor 1
    IO2  → RPWM Motor 2 — llanta derecha
    IO4  → LPWM Motor 2
    IO14 → Servo DS3218MG (signal pin)
    IO15 → Libre

  Control joystick IZQUIERDO (movimiento):
    ↑          → avanzar (con rampa suave para no patinar)
    ↓          → retroceder
    ← / →      → giro sobre el eje
    diagonal   → curva proporcional
    R1         → embestida: rampa rápida a tope + FLIPPER AUTO UP + freno
    L2         → turbo (sube al PWM máximo)
    Círculo    → giro rápido 180° (evasión)
    X          → freno activo de emergencia

  Control ANTI-FLIPPER (botones):
    Cuadrado  → UN toque = abre rápido + espera + cierra automático
    R1        → EMBESTIDA automática con flipper AUTO integrado
    
    Parámetros ajustables (en microsegundos):
    - FLIPPER_TIEMPO_ABIERTO: 350ms (cuánto tiempo permanece abierto)
    - FLIPPER_VELOCIDAD_PASO: 18° (velocidad de movimiento, a 5V)
    
    NOTA: A 5V tendrá ~80% del torque máximo. Para más fuerza:
    - Aumenta voltaje a 6-8V si es posible (servo aguanta hasta 8V)
    - O aumenta FLIPPER_TIEMPO_ABIERTO para mejor enganche

  AJUSTES 2.0 (VELOCIDAD AUMENTADA + ANTI-FLIPPER):
    - VEL_NORMAL: 195 → 230 (90% PWM, +18% más velocidad)
    - VEL_TURBO: 245 → 255 (100% PWM, al máximo)
    - EMBESTIDA_MS: 320 → 600 ms (+87% duración)
    - FLIPPER_BAJADO: 20° (reposo/defensa)
    - FLIPPER_LEVANTADO: 160° (ataque)
    - Levantamiento automático durante R1 (embestida)
*/

#include <PS4Controller.h>
#include "esp_mac.h"

// ─── Pines BTS7960 ────────────────────────────────────────────
#define RPWM_M1  13
#define LPWM_M1  12
#define RPWM_M2   2
#define LPWM_M2   4

// ─── Servo DS3218MG anti-flipper ──────────────────────────────
#define SERVO_PIN  14      // IO14
#define CH_SERVO    4      // canal LEDC para servo
#define SERVO_FREQ   50    // 50 Hz para servo estándar
#define SERVO_RES   16     // 16 bits = 0-65535 para más precisión

// ─── Parámetros de activación del flipper ────────────────────
#define FLIPPER_BAJADO           175  // ángulo en reposo (MÁS RETRACTADO)
#define FLIPPER_LEVANTADO          5  // ángulo en ataque (MÁS ABIERTO - casi 180°)
#define FLIPPER_TIEMPO_ABIERTO  450  // ms que permanece abierto (máxima potencia)
#define FLIPPER_VELOCIDAD_PASO   22  // grados por ciclo (MÁS RÁPIDO)
#define FLIPPER_DELAY_PASO        8  // ms entre pasos (más rápido)

// ─── Canales LEDC ─────────────────────────────────────────────
#define CH_R1   0
#define CH_L1   1
#define CH_R2   2
#define CH_L2c  3
#define CH_SERVO    4
#define PWM_FREQ  1000
#define PWM_RES   8        // 0–255

// ─── Velocidades ajustadas para JGA25-370 170RPM 12V ─────────
// MEJORAS: +velocidad y aceleración más rápida
#define VEL_NORMAL    230  // velocidad base (90% PWM) — MÁS RÁPIDO
#define VEL_TURBO     255  // turbo con L2  (100% PWM) — A TOPE
#define VEL_GIRO      230  // giro puro sobre el eje — MÁS ÁGIL
#define VEL_EMBESTIDA 255  // embestida siempre al tope

// ─── Rampa de arranque (anti-patinaje) ───────────────────────
#define RAMPA_PASO     15  // PWM que sube por ciclo — ACELERACIÓN MÁS RÁPIDA (era 12)
#define RAMPA_MS        8  // ms entre pasos

// ─── Freno activo ─────────────────────────────────────────────
#define FRENO_PWM      85
#define FRENO_MS       25

// ─── Embestida ────────────────────────────────────────────────
#define EMBESTIDA_MS  600  // duración del golpe — DUPLICADO (era 320ms, ahora 600ms)

// ─── Zona muerta joystick ─────────────────────────────────────
#define DEADZONE       20

// ─── MAC del mando (dejar "" para cualquier mando) ────────────
const char* PS4_MAC = "4C:C3:82:CC:80:C2";

// ─── Estado de velocidad actual (para rampa) ──────────────────
int16_t velActualIzq = 0;
int16_t velActualDer = 0;

// ─── Posición del servo (anti-flipper) ─────────────────────────
uint16_t flipperAngle = FLIPPER_BAJADO;  // posición actual
bool flipperEnSecuencia = false;  // está ejecutando apertura/cierre
unsigned long flipperTiempoInicio = 0;  // para tracking de tiempo
unsigned long ultimaActualizacionServo = 0;  // para evitar updates muy rápidas

// ─── Helpers de motor ─────────────────────────────────────────
void m1(int r, int l) {
  ledcWrite(CH_R1,  constrain(r, 0, 255));
  ledcWrite(CH_L1,  constrain(l, 0, 255));
}
void m2(int r, int l) {
  ledcWrite(CH_R2,  constrain(r, 0, 255));
  ledcWrite(CH_L2c, constrain(l, 0, 255));
}

void stopMotors() {
  m1(0, 0);
  m2(0, 0);
  velActualIzq = 0;
  velActualDer = 0;
}

// ─── Control del anti-flipper DS3218MG ────────────────────────
// Convierte ángulo (0-180°) a valor LEDC (50Hz, 16 bits)
void setFlipperAngle(uint16_t angle) {
  angle = constrain(angle, 0, 180);
  uint32_t pulseWidth = 1000 + (angle * 1000 / 180);
  uint32_t duty = (pulseWidth * 65535) / 20000;
  ledcWrite(CH_SERVO, duty);
  flipperAngle = angle;
}

// Rampa suave de ángulo (para acelerar/desacelerar)
uint16_t rampaFlipperAngle(uint16_t actual, uint16_t objetivo) {
  if (actual < objetivo) 
    return min((uint16_t)(actual + FLIPPER_VELOCIDAD_PASO), objetivo);
  if (actual > objetivo) 
    return max((uint16_t)(actual - FLIPPER_VELOCIDAD_PASO), objetivo);
  return objetivo;
}

// Secuencia de activación: abre + cierra automático
void activarFlipperAutomatico() {
  if (flipperEnSecuencia) return;  // ya está en secuencia
  
  flipperEnSecuencia = true;
  flipperTiempoInicio = millis();
  ultimaActualizacionServo = 0;
  Serial.println(">> FLIPPER ACTIVADO");
}

// Gestionar secuencia en el loop (llamar cada iteración)
void actualizarFlipperSecuencia() {
  if (!flipperEnSecuencia) return;
  
  // NO actualizar el servo cada milisegundo
  if (millis() - ultimaActualizacionServo < 20) return;
  ultimaActualizacionServo = millis();
  
  unsigned long tiempoTranscurrido = millis() - flipperTiempoInicio;
  
  // TIMEOUT: si lleva más de 2 segundos, cancela (seguridad)
  if (tiempoTranscurrido > 2000) {
    flipperEnSecuencia = false;
    setFlipperAngle(FLIPPER_BAJADO);
    Serial.println("✗ Flipper TIMEOUT - reseteado a reposo");
    return;
  }
  
  // FASE 1: Abrir (primeros ~100ms)
  if (tiempoTranscurrido < 100) {
    if (flipperAngle > FLIPPER_LEVANTADO) {
      flipperAngle = rampaFlipperAngle(flipperAngle, FLIPPER_LEVANTADO);
      setFlipperAngle(flipperAngle);
    }
  }
  // FASE 2: Esperar abierto (100ms a 600ms)
  else if (tiempoTranscurrido < (100 + FLIPPER_TIEMPO_ABIERTO)) {
    // Solo espera, no actualiza
  }
  // FASE 3: Cerrar (después de 600ms)
  else {
    if (flipperAngle < FLIPPER_BAJADO) {
      flipperAngle = rampaFlipperAngle(flipperAngle, FLIPPER_BAJADO);
      setFlipperAngle(flipperAngle);
    } else {
      // Llegó al cierre final
      flipperEnSecuencia = false;
      Serial.println("✓ Flipper listo");
    }
  }
}

void frenoActivo() {
  // Pulso inverso breve para cortar inercia del JGA25-370
  if (velActualIzq >= 0) { m1(FRENO_PWM, 0); m2(FRENO_PWM, 0); }
  else                   { m1(0, FRENO_PWM); m2(0, FRENO_PWM); }
  delay(FRENO_MS);
  stopMotors();
}

// Aplica velocidad con signo a cada motor
void rawMotor1(int16_t v) {
  v = constrain(v, -255, 255);
  if (v >= 0) m1(v, 0);
  else        m1(0, -v);
}
void rawMotor2(int16_t v) {
  v = constrain(v, -255, 255);
  if (v >= 0) m2(v, 0);
  else        m2(0, -v);
}

// ─── Rampa suave ──────────────────────────────────────────────
// Mueve velActual hacia velObjetivo de a RAMPA_PASO por ciclo
int16_t rampa(int16_t actual, int16_t objetivo) {
  if (actual < objetivo) return min((int16_t)(actual + RAMPA_PASO), objetivo);
  if (actual > objetivo) return max((int16_t)(actual - RAMPA_PASO), objetivo);
  return objetivo;
}

// ─── Mixeo arcade un solo joystick ───────────────────────────
void mixJoystick(int8_t jx, int8_t jy, int velMax,
                 int16_t &velIzq, int16_t &velDer) {

  if (abs(jx) < DEADZONE) jx = 0;
  if (abs(jy) < DEADZONE) jy = 0;

  float fy = (float)jy / 127.0f * velMax;
  float fx = (float)jx / 127.0f * velMax;

  // Motor izquierdo: avance + giro
  // Motor derecho:   avance - giro
  float iz = fy + fx;
  float de = fy - fx;

  // Clip proporcional: mantiene la dirección aunque sature
  float maxVal = max(abs(iz), abs(de));
  if (maxVal > velMax) {
    float scale = (float)velMax / maxVal;
    iz *= scale;
    de *= scale;
  }

  velIzq = (int16_t)iz;
  velDer = (int16_t)de;
}

// ─── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  ledcSetup(CH_R1,  PWM_FREQ, PWM_RES);
  ledcSetup(CH_L1,  PWM_FREQ, PWM_RES);
  ledcSetup(CH_R2,  PWM_FREQ, PWM_RES);
  ledcSetup(CH_L2c, PWM_FREQ, PWM_RES);

  ledcAttachPin(RPWM_M1, CH_R1);
  ledcAttachPin(LPWM_M1, CH_L1);
  ledcAttachPin(RPWM_M2, CH_R2);
  ledcAttachPin(LPWM_M2, CH_L2c);

  // Configurar servo anti-flipper (SIN inicializar posición)
  ledcSetup(CH_SERVO, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_PIN, CH_SERVO);

  stopMotors();

  if (strlen(PS4_MAC) > 0) PS4.begin(PS4_MAC);
  else                      PS4.begin();

  Serial.println("SUMO listo — JGA25-370 + ANTI-FLIPPER DS3218MG (5V optimizado)");
  Serial.println("Square: abre+cierra automático | R1: embestida+flipper");
  Serial.println("Esperando mando PS4...");
}

// ─── Loop ─────────────────────────────────────────────────────
void loop() {

  if (!PS4.isConnected()) {
    stopMotors();
    delay(100);
    return;
  }

  // ── Parada de emergencia (X) ──────────────────────────────
  if (PS4.Cross()) {
    frenoActivo();
    Serial.println("! FRENO DE EMERGENCIA");
    delay(200);
    return;
  }

  // ── Embestida de ataque (R1) ──────────────────────────────
  // Solo embestida rápida (sin flipper automático)
  if (PS4.R1()) {
    Serial.println(">> EMBESTIDA (usa Square para flipper)");
    
    int v = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < EMBESTIDA_MS) {
      v = min(v + 20, VEL_EMBESTIDA);
      rawMotor1(v);
      rawMotor2(v);
      delay(6);
    }
    frenoActivo();
    return;
  }

  // ── Evasión / spin 180° (Círculo) ─────────────────────────
  if (PS4.Circle()) {
    Serial.println(">> GIRO 180 evasion");
    // Giro a tope durante ~400ms (ajustar según peso del robot)
    rawMotor1( VEL_GIRO);
    rawMotor2(-VEL_GIRO);
    delay(400);
    frenoActivo();
    return;
  }

  // ── Velocidad según turbo ─────────────────────────────────
  bool turbo  = PS4.L2();
  int  velMax = turbo ? VEL_TURBO : VEL_NORMAL;

  // ── Leer joystick izquierdo ───────────────────────────────
  int8_t jx = PS4.LStickX();   // −128 izq … +127 der
  int8_t jy = PS4.LStickY();   // −128 abajo … +127 arriba

  int16_t velObjIzq, velObjDer;
  mixJoystick(jx, jy, velMax, velObjIzq, velObjDer);

  // ── Control del anti-flipper ─────────────────────────────────
  // Cuadrado (Square) = activar secuencia automática (abre + cierra)
  static unsigned long lastSquarePress = 0;
  if (PS4.Square() && (millis() - lastSquarePress > 800)) {
    lastSquarePress = millis();
    activarFlipperAutomatico();
  }
  
  // Actualizar secuencia del flipper (IMPORTANTE: cada loop)
  actualizarFlipperSecuencia();
  
  // Levantamiento automático durante embestida (R1)
  // ya se maneja en la función R1 abajo

  // ── Aplicar rampa suave ───────────────────────────────────
  velActualIzq = rampa(velActualIzq, velObjIzq);
  velActualDer = rampa(velActualDer, velObjDer);

  rawMotor1(velActualIzq);
  rawMotor2(velActualDer);

  // ── Debug serie cada 200ms ────────────────────────────────
  static unsigned long tPrint = 0;
  if (millis() - tPrint > 200) {
    tPrint = millis();
    Serial.printf("JX:%4d JY:%4d | Izq:%4d Der:%4d | Turbo:%d | Flipper:%3d°\n",
                  jx, jy, velActualIzq, velActualDer, turbo ? 1 : 0, flipperAngle);
  }

  delay(RAMPA_MS);   // cadencia del loop = cadencia de la rampa
}
