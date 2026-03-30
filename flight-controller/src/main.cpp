#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_task_wdt.h>

// ── Packet structure (must match glove transmitter) ───────────────────────────
typedef struct DronePacket {
  float pitch;
  float roll;
  float yaw;
  int   throttle;  // 0–100
} DronePacket;

// ── Motor GPIO pins (verify against your ESP32-CAM wiring) ───────────────────
// ESP32-CAM free GPIOs (camera not used): 12, 13, 14, 15
#define MOTOR_FL  12   // Front-Left
#define MOTOR_FR  13   // Front-Right
#define MOTOR_RL  14   // Rear-Left
#define MOTOR_RR  15   // Rear-Right

// ledc channels (one per motor)
#define CH_FL  0
#define CH_FR  1
#define CH_RL  2
#define CH_RR  3

#define PWM_FREQ_HZ    400
#define PWM_RESOLUTION 8          // 8-bit → 0–255
#define PWM_MIN        0
#define PWM_MAX        255
#define PWM_IDLE       0          // motors off when throttle = 0

// ── Safety ────────────────────────────────────────────────────────────────────
#define FAILSAFE_TIMEOUT_MS  500  // zero motors if no packet within this window
#define WDT_TIMEOUT_S        3

// ── PID ───────────────────────────────────────────────────────────────────────
struct PID {
  float kP, kI, kD;
  float integral  = 0.0f;
  float prevError = 0.0f;

  float compute(float setpoint, float actual, float dt) {
    float error   = setpoint - actual;
    integral     += error * dt;
    integral      = constrain(integral, -100.0f, 100.0f);  // anti-windup
    float deriv   = (error - prevError) / dt;
    prevError     = error;
    return kP * error + kI * integral + kD * deriv;
  }
};

// Starting gains — tune these during flight testing
PID pidPitch = {0.5f, 0.0f, 0.1f};
PID pidRoll  = {0.5f, 0.0f, 0.1f};
PID pidYaw   = {0.3f, 0.0f, 0.05f};

// ── State ─────────────────────────────────────────────────────────────────────
volatile DronePacket latestPacket = {0, 0, 0, 0};
volatile bool        newPacket    = false;
unsigned long        lastPacketMs = 0;
unsigned long        lastLoopMs   = 0;

// ── ESP-NOW receive callback ──────────────────────────────────────────────────
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(DronePacket)) {
    memcpy((void *)&latestPacket, data, sizeof(DronePacket));
    newPacket    = true;
    lastPacketMs = millis();
  }
}

// ── Motor helpers ─────────────────────────────────────────────────────────────
void motorsOff() {
  ledcWrite(CH_FL, PWM_IDLE);
  ledcWrite(CH_FR, PWM_IDLE);
  ledcWrite(CH_RL, PWM_IDLE);
  ledcWrite(CH_RR, PWM_IDLE);
}

void setMotors(float fl, float fr, float rl, float rr) {
  ledcWrite(CH_FL, (int)constrain(fl, PWM_MIN, PWM_MAX));
  ledcWrite(CH_FR, (int)constrain(fr, PWM_MIN, PWM_MAX));
  ledcWrite(CH_RL, (int)constrain(rl, PWM_MIN, PWM_MAX));
  ledcWrite(CH_RR, (int)constrain(rr, PWM_MIN, PWM_MAX));
}

void setup() {
  Serial.begin(115200);

  // Motor PWM
  ledcSetup(CH_FL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcSetup(CH_FR, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcSetup(CH_RL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcSetup(CH_RR, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_FL, CH_FL);
  ledcAttachPin(MOTOR_FR, CH_FR);
  ledcAttachPin(MOTOR_RL, CH_RL);
  ledcAttachPin(MOTOR_RR, CH_RR);
  motorsOff();

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed.");
    while (1) delay(10);
  }
  esp_now_register_recv_cb(onDataRecv);

  // Watchdog
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  Serial.println("Flight controller ready.");
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());

  lastLoopMs   = millis();
  lastPacketMs = millis();
}

void loop() {
  esp_task_wdt_reset();

  unsigned long now = millis();
  float dt = (now - lastLoopMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  lastLoopMs = now;

  // ── Failsafe ─────────────────────────────────────────────────────────────────
  if (now - lastPacketMs > FAILSAFE_TIMEOUT_MS) {
    motorsOff();
    pidPitch.integral = 0;
    pidRoll.integral  = 0;
    pidYaw.integral   = 0;
    return;
  }

  if (!newPacket) return;
  newPacket = false;

  // Snapshot volatile packet
  DronePacket pkt;
  noInterrupts();
  memcpy(&pkt, (const void *)&latestPacket, sizeof(DronePacket));
  interrupts();

  // ── Base throttle (0–100 → 0–PWM_MAX) ───────────────────────────────────────
  float base = map(pkt.throttle, 0, 100, PWM_IDLE, PWM_MAX);

  if (pkt.throttle == 0) {
    motorsOff();
    pidPitch.integral = 0;
    pidRoll.integral  = 0;
    pidYaw.integral   = 0;
    return;
  }

  // ── PID (setpoint = glove angle, actual = 0 for now; add IMU to drone later) ─
  float outPitch = pidPitch.compute(pkt.pitch, 0.0f, dt);
  float outRoll  = pidRoll.compute(pkt.roll,   0.0f, dt);
  float outYaw   = pidYaw.compute(pkt.yaw,     0.0f, dt);

  // ── Motor mixing (X-frame) ───────────────────────────────────────────────────
  // FL: CCW  FR: CW  RL: CW  RR: CCW
  float fl = base + outPitch - outRoll - outYaw;
  float fr = base + outPitch + outRoll + outYaw;
  float rl = base - outPitch - outRoll + outYaw;
  float rr = base - outPitch + outRoll - outYaw;

  setMotors(fl, fr, rl, rr);

  Serial.printf("P:%.1f R:%.1f Y:%.1f T:%d | FL:%.0f FR:%.0f RL:%.0f RR:%.0f\n",
                pkt.pitch, pkt.roll, pkt.yaw, pkt.throttle, fl, fr, rl, rr);
}
