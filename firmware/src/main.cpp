/**
 * Firmware ESP32 — Gêmeo Digital do Braço Robótico 4-DOF (Servos MG90 + PCA9685)
 *
 * Calibração de Montagem (Pose Zero idêntica ao sim/view_mockup.py):
 *   Ao ligar o ESP32, todos os 4 servos são posicionados automaticamente em 0.0°
 *   (pulso central de 1500 us / tick 307 a 50 Hz). Nessa posição de 0°, monte as
 *   peças impressas em 3D exatamente na pose exibida em `python sim/view_mockup.py`:
 *     - Canal 0 (J1 Base):     0° -> Tampa alinhada à frente
 *     - Canal 1 (J2 Ombro):    0° -> Braço 1 na inclinação neutra do CAD
 *     - Canal 2 (J3 Cotovelo): 0° -> Braço 2 horizontal apontando para frente
 *     - Canal 3 (J4 Garra):    0° -> Dedos da garra fechados (encostados no centro)
 *
 * Ligações I2C (ESP32 -> PCA9685):
 *   - SDA -> GPIO 21
 *   - SCL -> GPIO 22
 *   - VCC -> 3.3V (lógica)
 *   - V+  -> Fonte Externa 5V (2A a 3A para alimentar os servos MG90)
 *   - GND -> GND comum (ESP32 + Fonte 5V + PCA9685)
 *
 * Comandos aceitos no Serial Monitor (115200 baud, terminados por Enter / '\n'):
 *   - ZERO               -> Coloca todas as 4 juntas em 0.0° (1500 us) para calibração/montagem
 *   - j1,j2,j3,j4        -> Ex: 0,-45,20,25   (move as juntas para os ângulos em graus)
 *   - S:j1,j2,j3,j4      -> Ex: S:0.0,-72.8,26.9,1.0 (formato enviado em tempo real pelo MuJoCo)
 *   - US:u1,u2,u3,u4     -> Ex: US:1500,1200,1750,1800 (define pulsos diretos em microssegundos)
 *   - PWM:t1,t2,t3,t4    -> Ex: PWM:307,245,358,368   (define ticks brutos de 12 bits 0..4095)
 *   - FREQ:hz            -> Ex: FREQ:50 (altera a frequência PWM do PCA9685, padrão 50 Hz)
 *   - DIR:d1,d2,d3,d4    -> Ex: DIR:1,1,1,1 (usa +1 ou -1 para inverter sentido de algum servo)
 *   - OFFSET:o1,o2,o3,o4 -> Ex: OFFSET:0,2.5,-1.0,0 (ajuste fino do zero em graus por junta)
 *   - STATUS ou ?        -> Exibe a tabela completa de estado (graus, us, ticks PWM e frequência)
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ======================= CONFIGURAÇÃO DO HARDWARE =======================
static const uint8_t PIN_I2C_SDA = 21;
static const uint8_t PIN_I2C_SCL = 22;
static const uint8_t PCA9685_ADDR = 0x40;

// Registradores internos do PCA9685
static const uint8_t PCA9685_MODE1      = 0x00;
static const uint8_t PCA9685_PRESCALE   = 0xFE;
static const uint8_t PCA9685_LED0_ON_L  = 0x06;
static const uint32_t PCA9685_OSC_CLOCK = 25000000UL; // 25 MHz

// Parâmetros seguros do Servo MG90 (-90° a +90°)
// Usamos 544 us a 2400 us (padrão oficial Arduino Servo.h / Wokwi) com centro em 1500 us
// para evitar que o servo force o batente físico nas extremidades de -90° e +90°.
static const float MG90_MIN_DEG    = -90.0f;
static const float MG90_MAX_DEG    = +90.0f;
static const float MG90_MIN_US     = 544.0f;   // Pulso seguro para -90° (tick ~111)
static const float MG90_CENTER_US  = 1500.0f;  // Pulso para   0° (tick 307 — Pose do view_mockup.py)
static const float MG90_MAX_US     = 2400.0f;  // Pulso seguro para +90° (tick ~492)

// Canais dos 4 servos no PCA9685
static const uint8_t NUM_JOINTS = 4;
static const uint8_t JOINT_CHANNELS[NUM_JOINTS] = {0, 1, 2, 3};
static const char*   JOINT_NAMES[NUM_JOINTS]    = {
    "J1_Base",
    "J2_Ombro",
    "J3_Cotovelo",
    "J4_Garra"
};

// Limites angulares operacionais seguros de cada junta (em graus, calibrados contra colisão no MuJoCo):
//   J1 Base:     [-90.0°, +90.0°] -> [ 544 us .. 2400 us] -> [tick 111 .. 492]
//   J2 Ombro:    [-72.0°, +60.0°] -> [ 735 us .. 2100 us] -> [tick 151 .. 430]
//   J3 Cotovelo: [-35.0°, +65.0°] -> [1128 us .. 2150 us] -> [tick 231 .. 440]
//   J4 Garra:    [  0.0°, +38.0°] -> [1500 us .. 1880 us] -> [tick 307 .. 385]
static float jointMinDeg[NUM_JOINTS] = {-90.0f, -72.0f, -35.0f,  0.0f};
static float jointMaxDeg[NUM_JOINTS] = {+90.0f, +60.0f, +65.0f, +38.0f};

static float j3MinSafeForJ2(float j2Deg) {
  float val = 0.0051f * (j2Deg * j2Deg) - 0.520f * j2Deg - 35.2f;
  return constrain(val, jointMinDeg[2], jointMaxDeg[2]);
}

// Direção (+1 ou -1) e offset fino de calibração (em graus) por junta
static float jointDir[NUM_JOINTS]    = {+1.0f, +1.0f, +1.0f, +1.0f};
static float jointOffset[NUM_JOINTS] = { 0.0f,  0.0f,  0.0f,  0.0f};

// Estado atual das 4 juntas
static float    pwmFreqHz               = 50.0f;
static float    currentDeg[NUM_JOINTS]  = {0.0f, 0.0f, 0.0f, 0.0f};
static float    currentUs[NUM_JOINTS]   = {1500.0f, 1500.0f, 1500.0f, 1500.0f};
static uint16_t currentTick[NUM_JOINTS] = {307, 307, 307, 307};
static bool     pcaConnected            = false;
static bool     verboseTelemetry        = true;

// ======================= DRIVER I2C NATIVO PCA9685 =======================

static bool pcaWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static uint8_t pcaReadReg(uint8_t reg) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(PCA9685_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

static void pcaSetPWMFreq(float freqHz) {
  freqHz = constrain(freqHz, 24.0f, 1526.0f);
  pwmFreqHz = freqHz;

  float prescaleval = ((float)PCA9685_OSC_CLOCK / (4096.0f * freqHz)) - 1.0f;
  uint8_t prescale = (uint8_t)floorf(prescaleval + 0.5f);

  uint8_t oldmode = pcaReadReg(PCA9685_MODE1);
  uint8_t sleepmode = (oldmode & 0x7F) | 0x10; // Entra em modo Sleep para alterar PRESCALE
  pcaWriteReg(PCA9685_MODE1, sleepmode);
  pcaWriteReg(PCA9685_PRESCALE, prescale);
  pcaWriteReg(PCA9685_MODE1, oldmode);
  delay(5);
  pcaWriteReg(PCA9685_MODE1, oldmode | 0xA0);  // Auto-incremento ativo + Restart
}

static void pcaSetChannelPWM(uint8_t channel, uint16_t onTick, uint16_t offTick) {
  uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(onTick & 0xFF);
  Wire.write((onTick >> 8) & 0x0F);
  Wire.write(offTick & 0xFF);
  Wire.write((offTick >> 8) & 0x0F);
  pcaConnected = (Wire.endTransmission() == 0);
}

// ======================= CONVERSÃO ÂNGULO <-> PULSO <-> TICK =======================

/**
 * Converte largura de pulso (us) para tick de 12 bits (0..4095) do PCA9685
 * levando em consideração a frequência atual (padrão 50 Hz -> período 20.000 us).
 */
static uint16_t pulseUsToTick(float pulseUs) {
  float periodUs = 1000000.0f / pwmFreqHz; // 20.000 us em 50 Hz
  float tick = (pulseUs / periodUs) * 4096.0f;
  int32_t t = (int32_t)lroundf(tick);
  return (uint16_t)constrain(t, 0, 4095);
}

/**
 * Converte o ângulo da junta no MuJoCo (em graus, onde 0° = pose do view_mockup.py)
 * para largura de pulso em microssegundos (500..2500 us) do servo MG90.
 */
static float jointDegToPulseUs(uint8_t j, float deg) {
  float clampedDeg = constrain(deg, jointMinDeg[j], jointMaxDeg[j]);
  float effectiveDeg = (jointDir[j] * clampedDeg) + jointOffset[j];
  effectiveDeg = constrain(effectiveDeg, MG90_MIN_DEG, MG90_MAX_DEG);

  // Mapeamento linear assimétrico seguro em torno do centro 1500 us (0°):
  // -90° -> 544 us | 0° -> 1500 us | +90° -> 2400 us
  float pulseUs = MG90_CENTER_US;
  if (effectiveDeg >= 0.0f) {
    pulseUs = MG90_CENTER_US + (effectiveDeg / 90.0f) * (MG90_MAX_US - MG90_CENTER_US);
  } else {
    pulseUs = MG90_CENTER_US + (effectiveDeg / 90.0f) * (MG90_CENTER_US - MG90_MIN_US);
  }
  return constrain(pulseUs, MG90_MIN_US, MG90_MAX_US);
}

/**
 * Atualiza uma junta a partir do ângulo em graus e envia o sinal PWM ao PCA9685.
 */
static void setJointAngleDeg(uint8_t j, float deg) {
  if (j >= NUM_JOINTS) return;
  currentDeg[j] = constrain(deg, jointMinDeg[j], jointMaxDeg[j]);
  // Proteção cinemática acoplada (J2, J3) contra colisão com o chão:
  float j3FloorMin = j3MinSafeForJ2(currentDeg[1]);
  if (currentDeg[2] < j3FloorMin) {
    currentDeg[2]  = j3FloorMin;
    currentUs[2]   = jointDegToPulseUs(2, currentDeg[2]);
    currentTick[2] = pulseUsToTick(currentUs[2]);
    pcaSetChannelPWM(JOINT_CHANNELS[2], 0, currentTick[2]);
  }
  currentUs[j]   = jointDegToPulseUs(j, currentDeg[j]);
  currentTick[j] = pulseUsToTick(currentUs[j]);
  pcaSetChannelPWM(JOINT_CHANNELS[j], 0, currentTick[j]);
}

/**
 * Atualiza uma junta diretamente a partir da largura de pulso em microssegundos (us),
 * respeitando os limites seguros da junta e a restrição anti-colisão com o solo.
 */
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

/**
 * Atualiza uma junta diretamente a partir do tick PWM de 12 bits (0..4095),
 * respeitando os limites seguros da junta e a restrição anti-colisão com o solo.
 */
static void setJointTick(uint8_t j, uint16_t tick) {
  if (j >= NUM_JOINTS) return;
  uint16_t t = constrain(tick, 0, 4095);
  float periodUs = 1000000.0f / pwmFreqHz;
  float pulseUs  = ((float)t / 4096.0f) * periodUs;
  setJointPulseUs(j, pulseUs);
}

/**
 * Posiciona todas as 4 juntas em 0.0° (1500 us / tick 307) — Pose Zero do view_mockup.py.
 */
static void moveAllToZeroPose() {
  for (uint8_t j = 0; j < NUM_JOINTS; j++) {
    setJointAngleDeg(j, 0.0f);
  }
}

// ======================= RELATÓRIO E PARSER SERIAL =======================

static void printStatusTable() {
  Serial.println(F("\n==================== STATUS DO ROBO (ESP32 + PCA9685) ===================="));
  Serial.printf("I2C PCA9685 (0x40): %s | Frequencia PWM: %.1f Hz (Periodo: %.0f us)\n",
                pcaConnected ? "CONECTADO" : "SEM RESPOSTA (verifique SDA=21, SCL=22)",
                pwmFreqHz, 1000000.0f / pwmFreqHz);
  Serial.println(F("--------------------------------------------------------------------------"));
  Serial.println(F("Junta         | Canal | Angulo (graus) | Pulso (us) | Tick PWM (12b) | Dir/Off"));
  Serial.println(F("--------------------------------------------------------------------------"));
  for (uint8_t j = 0; j < NUM_JOINTS; j++) {
    Serial.printf("%-13s |  CH%u  |    %+7.2f deg |  %7.1f us |    %4u / 4095 | %+.0f / %+.1f\n",
                  JOINT_NAMES[j],
                  JOINT_CHANNELS[j],
                  currentDeg[j],
                  currentUs[j],
                  currentTick[j],
                  jointDir[j],
                  jointOffset[j]);
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

  // 1. Comando ZERO (calibração inicial igual ao view_mockup.py)
  if (upper == "ZERO" || upper == "0" || upper == "RESET") {
    moveAllToZeroPose();
    Serial.println(F("[OK] Todos os 4 servos posicionados em 0.0 deg (1500 us / tick 307) — Pose view_mockup.py"));
    printStatusTable();
    return;
  }

  // 2. Comando STATUS ou ?
  if (upper == "STATUS" || upper == "?" || upper == "HELP") {
    printStatusTable();
    return;
  }

  // 3. Comando FREQ:hz (ex: FREQ:50)
  if (upper.startsWith("FREQ:")) {
    float f = line.substring(5).toFloat();
    if (f >= 24.0f && f <= 200.0f) {
      pcaSetPWMFreq(f);
      // Recalcula os ticks de todas as juntas para a nova frequência mantendo os ângulos atuais
      for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        setJointAngleDeg(j, currentDeg[j]);
      }
      Serial.printf("[OK] Frequencia do PCA9685 ajustada para %.1f Hz e ticks recalculados.\n", pwmFreqHz);
      printStatusTable();
    }
    return;
  }

  // 4. Comando DIR:d1,d2,d3,d4 (ex: DIR:1,-1,1,1)
  if (upper.startsWith("DIR:")) {
    float vals[4];
    if (parseFourFloats(line.substring(4).c_str(), vals)) {
      for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        jointDir[j] = (vals[j] < 0.0f) ? -1.0f : +1.0f;
        setJointAngleDeg(j, currentDeg[j]);
      }
      Serial.println(F("[OK] Direcoes dos servos atualizadas."));
      printStatusTable();
    }
    return;
  }

  // 5. Comando OFFSET:o1,o2,o3,o4 (ex: OFFSET:0,2,-1.5,0)
  if (upper.startsWith("OFFSET:")) {
    float vals[4];
    if (parseFourFloats(line.substring(7).c_str(), vals)) {
      for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        jointOffset[j] = constrain(vals[j], -45.0f, 45.0f);
        setJointAngleDeg(j, currentDeg[j]);
      }
      Serial.println(F("[OK] Offsets de zero atualizados."));
      printStatusTable();
    }
    return;
  }

  // 6. Comando US:u1,u2,u3,u4 (pulsos diretos em microssegundos)
  if (upper.startsWith("US:")) {
    float vals[4];
    if (parseFourFloats(line.substring(3).c_str(), vals)) {
      for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        setJointPulseUs(j, vals[j]);
      }
      Serial.printf("[US->PWM] J1:%u(%0.fus) J2:%u(%.0fus) J3:%u(%.0fus) J4:%u(%.0fus)\n",
                    currentTick[0], currentUs[0],
                    currentTick[1], currentUs[1],
                    currentTick[2], currentUs[2],
                    currentTick[3], currentUs[3]);
    }
    return;
  }

  // 7. Comando PWM:t1,t2,t3,t4 (ticks diretos de 12 bits 0..4095)
  if (upper.startsWith("PWM:")) {
    float vals[4];
    if (parseFourFloats(line.substring(4).c_str(), vals)) {
      for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        setJointTick(j, (uint16_t)vals[j]);
      }
      Serial.printf("[PWM] Ticks aplicados: CH0=%u CH1=%u CH2=%u CH3=%u\n",
                    currentTick[0], currentTick[1], currentTick[2], currentTick[3]);
    }
    return;
  }

  // 8. Comando de streaming do Simulador MuJoCo: "S:j1,j2,j3,j4" ou direto "j1,j2,j3,j4"
  bool isStream = upper.startsWith("S:");
  const char* payload = isStream ? line.c_str() + 2 : line.c_str();
  float degs[4];
  if (parseFourFloats(payload, degs)) {
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
      setJointAngleDeg(j, degs[j]);
    }
    // Se foi digitado manualmente no Serial Monitor (sem prefixo S:), mostra a conversão completa
    if (!isStream && verboseTelemetry) {
      Serial.printf("[MOV] J1:%+6.1f deg (%4.0fus, tick %3u) | J2:%+6.1f deg (%4.0fus, tick %3u) | "
                    "J3:%+6.1f deg (%4.0fus, tick %3u) | J4:%+6.1f deg (%4.0fus, tick %3u)\n",
                    currentDeg[0], currentUs[0], currentTick[0],
                    currentDeg[1], currentUs[1], currentTick[1],
                    currentDeg[2], currentUs[2], currentTick[2],
                    currentDeg[3], currentUs[3], currentTick[3]);
    }
    return;
  }

  Serial.print(F("[Aviso] Comando nao reconhecido: "));
  Serial.println(line);
  Serial.println(F("Exemplos validos: ZERO | STATUS | 0,-30,15,20 | US:1500,1500,1500,1500 | PWM:307,307,307,307"));
}

// ======================= SETUP E LOOP PRINCIPAL =======================

static String serialBuffer = "";

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000); // I2C Fast Mode (400 kHz)

  // Inicializa o PCA9685 em 50 Hz e posiciona os 4 servos em 0.0° (Pose do view_mockup.py)
  pcaWriteReg(PCA9685_MODE1, 0x00);
  pcaSetPWMFreq(50.0f);
  moveAllToZeroPose();

  Serial.println(F("\n=========================================================================="));
  Serial.println(F("  BRACO ROBOTICO 4-DOF — FIRMWARE ESP32 + PCA9685 (CALIBRACAO & GEMEOS)"));
  Serial.println(F("=========================================================================="));
  Serial.println(F(">> MODO CALIBRACAO ATIVO: Todos os 4 servos foram colocados em 0.0 graus"));
  Serial.println(F("   (Pulso = 1500 us | Tick PCA9685 = 307 / 4095 @ 50 Hz)."));
  Serial.println(F(">> Monte as pecas exatamente como no `python sim/view_mockup.py`."));
  Serial.println(F(">> Comandos Serial (115200): ZERO | STATUS | j1,j2,j3,j4 | US:... | PWM:..."));
  printStatusTable();
}

void loop() {
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
}
