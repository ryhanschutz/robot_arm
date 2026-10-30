/**
 * SIMULAÇÃO ISOLADA WOKWI — Braço Robótico 4-DOF (ESP32 Direto nos 4 Servos MG90, SEM PCA9685)
 *
 * Pinagem Direta no ESP32 (wokwi-esp32-devkit-v1):
 *   - Servo 1 (J1_Base)     -> Pino PWM: D25 (GPIO 25) | V+: VIN (5V) | GND: GND
 *   - Servo 2 (J2_Ombro)    -> Pino PWM: D26 (GPIO 26) | V+: VIN (5V) | GND: GND
 *   - Servo 3 (J3_Cotovelo) -> Pino PWM: D27 (GPIO 27) | V+: VIN (5V) | GND: GND
 *   - Servo 4 (J4_Garra)    -> Pino PWM: D14 (GPIO 14) | V+: VIN (5V) | GND: GND
 *
 * Por que usamos geração direta de pulso de 50 Hz (digitalWrite + delayMicroseconds)?
 *   O core Arduino ESP32 3.x (ESP-IDF 5.x) mudou os registradores de clock do periférico LEDC,
 *   o que faz o LEDC não gerar pulso no emulador do Wokwi. Gerando os pulsos de 544..2400 us
 *   diretamente nos GPIOs a cada 20 ms (50 Hz), os 4 `wokwi-servo` respondem com 100% de precisão!
 */

#include <Arduino.h>
#include <math.h>

// ======================= PINAGEM DIRETA ESP32 (WOKWI) =======================
static const uint8_t NUM_JOINTS = 4;

static const uint8_t SERVO_PINS[NUM_JOINTS] = {
    25, // J1_Base     -> pino D25
    26, // J2_Ombro    -> pino D26
    27, // J3_Cotovelo -> pino D27
    14  // J4_Garra    -> pino D14
};

static const char* JOINT_NAMES[NUM_JOINTS] = {
    "J1_Base",
    "J2_Ombro",
    "J3_Cotovelo",
    "J4_Garra"
};

// Parâmetros seguros do Servo MG90 / Wokwi (-90° a +90° na junta -> 544 us a 2400 us)
static const float MG90_MIN_DEG   = -90.0f;
static const float MG90_MAX_DEG   = +90.0f;
static const float MG90_MIN_US    = 544.0f;  // -90° da junta (0° no mostrador do wokwi-servo)
static const float MG90_CENTER_US = 1500.0f; //   0° da junta (90° no mostrador do wokwi-servo = view_mockup.py)
static const float MG90_MAX_US    = 2400.0f; // +90° da junta (180° no mostrador do wokwi-servo)

// Limites angulares operacionais seguros das juntas (calibrados contra colisão com o solo no MuJoCo):
//   J1 Base:     [-90.0°, +90.0°] -> [ 544 us .. 2400 us]
//   J2 Ombro:    [-72.0°, +60.0°] -> [ 735 us .. 2100 us]
//   J3 Cotovelo: [-35.0°, +65.0°] -> [1128 us .. 2150 us]
//   J4 Garra:    [  0.0°, +38.0°] -> [1500 us .. 1880 us]
static float jointMinDeg[NUM_JOINTS] = {-90.0f, -72.0f, -35.0f,  0.0f};
static float jointMaxDeg[NUM_JOINTS] = {+90.0f, +60.0f, +65.0f, +38.0f};

static float j3MinSafeForJ2(float j2Deg) {
  float val = 0.0051f * (j2Deg * j2Deg) - 0.520f * j2Deg - 35.2f;
  return constrain(val, jointMinDeg[2], jointMaxDeg[2]);
}

static float jointDir[NUM_JOINTS]    = {+1.0f, +1.0f, +1.0f, +1.0f};
static float jointOffset[NUM_JOINTS] = { 0.0f,  0.0f,  0.0f,  0.0f};

static float    currentDeg[NUM_JOINTS] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint16_t currentUs[NUM_JOINTS]  = {1500, 1500, 1500, 1500};

// ======================= CONVERSÃO ÂNGULO <-> PULSO =======================

static uint16_t jointDegToPulseUs(uint8_t j, float deg) {
  float clampedDeg = constrain(deg, jointMinDeg[j], jointMaxDeg[j]);
  float effectiveDeg = (jointDir[j] * clampedDeg) + jointOffset[j];
  effectiveDeg = constrain(effectiveDeg, MG90_MIN_DEG, MG90_MAX_DEG);

  float pulseUs = MG90_CENTER_US;
  if (effectiveDeg >= 0.0f) {
    pulseUs = MG90_CENTER_US + (effectiveDeg / 90.0f) * (MG90_MAX_US - MG90_CENTER_US);
  } else {
    pulseUs = MG90_CENTER_US + (effectiveDeg / 90.0f) * (MG90_CENTER_US - MG90_MIN_US);
  }
  return (uint16_t)lroundf(constrain(pulseUs, MG90_MIN_US, MG90_MAX_US));
}

static void setJointAngleDeg(uint8_t j, float deg) {
  if (j >= NUM_JOINTS) return;
  currentDeg[j] = constrain(deg, jointMinDeg[j], jointMaxDeg[j]);
  // Proteção cinemática acoplada (J2, J3) contra colisão com o chão:
  float j3FloorMin = j3MinSafeForJ2(currentDeg[1]);
  if (currentDeg[2] < j3FloorMin) {
    currentDeg[2] = j3FloorMin;
    currentUs[2]  = jointDegToPulseUs(2, currentDeg[2]);
  }
  currentUs[j] = jointDegToPulseUs(j, currentDeg[j]);
}

static void setJointPulseUs(uint8_t j, float pulseUs) {
  if (j >= NUM_JOINTS) return;
  float u = constrain(pulseUs, MG90_MIN_US, MG90_MAX_US);
  float rawDeg = 0.0f;
  if (u >= MG90_CENTER_US) {
    rawDeg = ((u - MG90_CENTER_US) / (MG90_MAX_US - MG90_CENTER_US)) * 90.0f;
  } else {
    rawDeg = ((u - MG90_CENTER_US) / (MG90_CENTER_US - MG90_MIN_US)) * 90.0f;
  }
  float deg = (rawDeg - jointOffset[j]) * jointDir[j];
  setJointAngleDeg(j, deg);
}

static void moveAllToZeroPose() {
  for (uint8_t j = 0; j < NUM_JOINTS; j++) {
    setJointAngleDeg(j, 0.0f);
  }
}

/**
 * Gera um trem de pulsos PWM de 50 Hz nos 4 pinos dos servos.
 * O componente `wokwi-servo` mede a duração do pulso HIGH em microssegundos (544..2400 us).
 */
static void sendServoPulses50Hz() {
  for (uint8_t j = 0; j < NUM_JOINTS; j++) {
    digitalWrite(SERVO_PINS[j], HIGH);
    delayMicroseconds(currentUs[j]);
    digitalWrite(SERVO_PINS[j], LOW);
  }
}

// ======================= SERIAL MONITOR & PROTOCOLO MUJOCO =======================

static void printStatusTable() {
  Serial.println(F("\n================== STATUS WOKWI (ESP32 PWM DIRETO 50 Hz) ================="));
  Serial.println(F("Junta         | Pino ESP32 | Junta (graus) | Servo Wokwi | Pulso (us)"));
  Serial.println(F("--------------------------------------------------------------------------"));
  for (uint8_t j = 0; j < NUM_JOINTS; j++) {
    float wokwiDeg = 90.0f + currentDeg[j];
    Serial.printf("%-13s |   GPIO %2u  |   %+7.2f deg |  %6.1f deg |    %4u us\n",
                  JOINT_NAMES[j],
                  SERVO_PINS[j],
                  currentDeg[j],
                  wokwiDeg,
                  currentUs[j]);
  }
  Serial.println(F("==========================================================================\n"));
}

static bool parseFourFloats(const char* str, float out[4]) {
  char buf[96];
  strncpy(buf, str, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  uint8_t idx = 0;
  char* token = strtok(buf, ",; ");
  while (token != nullptr && idx < 4) {
    out[idx++] = atof(token);
    token = strtok(nullptr, ",; ");
  }
  return (idx == 4);
}

static void processSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  String upper = line;
  upper.toUpperCase();

  if (upper == "ZERO" || upper == "0" || upper == "RESET") {
    moveAllToZeroPose();
    Serial.println(F("[OK] Todos os 4 servos em 0.0 deg (1500 us / 90 deg no Wokwi) — Pose view_mockup.py"));
    printStatusTable();
    return;
  }

  if (upper == "STATUS" || upper == "?" || upper == "HELP") {
    printStatusTable();
    return;
  }

  if (upper.startsWith("US:")) {
    float vals[4];
    if (parseFourFloats(line.substring(3).c_str(), vals)) {
      for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        setJointPulseUs(j, vals[j]);
      }
      printStatusTable();
    }
    return;
  }

  int sPos = upper.indexOf("S:");
  bool isStream = (sPos >= 0);
  const char* payload = isStream ? (line.c_str() + sPos + 2) : line.c_str();
  float degs[4];
  if (parseFourFloats(payload, degs)) {
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
      setJointAngleDeg(j, degs[j]);
    }
    if (!isStream) {
      Serial.printf("[MOV] J1:%+6.1f deg (%uus) | J2:%+6.1f deg (%uus) | J3:%+6.1f deg (%uus) | J4:%+6.1f deg (%uus)\n",
                    currentDeg[0], currentUs[0],
                    currentDeg[1], currentUs[1],
                    currentDeg[2], currentUs[2],
                    currentDeg[3], currentUs[3]);
    }
    return;
  }

  Serial.print(F("[Aviso] Comando nao reconhecido: "));
  Serial.println(line);
}

static String   serialBuffer = "";
static uint32_t lastPulseMs  = 0;

void setup() {
  Serial.begin(115200);

  for (uint8_t j = 0; j < NUM_JOINTS; j++) {
    pinMode(SERVO_PINS[j], OUTPUT);
    digitalWrite(SERVO_PINS[j], LOW);
  }

  // Posiciona imediatamente os 4 servos em 0.0° (1500 us) — Pose do view_mockup.py
  moveAllToZeroPose();
  for (int i = 0; i < 5; i++) {
    sendServoPulses50Hz();
    delay(14);
  }

  Serial.println(F("\n=========================================================================="));
  Serial.println(F("  WOKWI SIMULATOR — BRACO ROBOTICO 4-DOF (ESP32 PWM DIRETO 50 Hz)"));
  Serial.println(F("=========================================================================="));
  Serial.println(F("Pinagem ativa: J1=D25 | J2=D26 | J3=D27 | J4=D14"));
  Serial.println(F("Todos os servos iniciados em 0.0 deg (1500 us = centro 90 deg no Wokwi)."));
  Serial.println(F("Digite ZERO, STATUS ou angulos ex: 0,-45,20,25"));
  printStatusTable();
}

void loop() {
  // 1. Lê comandos recebidos via Serial (do Serial Monitor ou do MuJoCo via RFC2217)
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else if (serialBuffer.length() < 120) {
      serialBuffer += c;
    }
  }

  // 2. Envia os pulsos de 50 Hz (a cada 20 ms) para os 4 servos no Wokwi
  uint32_t nowMs = millis();
  if (nowMs - lastPulseMs >= 20) {
    lastPulseMs = nowMs;
    sendServoPulses50Hz();
  }
}
