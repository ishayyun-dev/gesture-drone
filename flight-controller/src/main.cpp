#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
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

// ── Onboard IMU I2C pins (remapped to save PSRAM for the Camera) ─────────────
#define IMU_SDA  14
#define IMU_SCL  13

// ── Motor GPIO pins (verify against your ESP32-CAM wiring) ───────────────────
// ESP32-CAM free GPIOs (Camera and PSRAM safely preserved): 12, 15, 2, 4
#define MOTOR_FL  12   // Front-Left
#define MOTOR_FR  15   // Front-Right
#define MOTOR_RL  2    // Rear-Left
#define MOTOR_RR  4    // Rear-Right (Note: also pulses the onboard Flash LED)

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
  float integral;
  float prevError;

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
PID pidPitch = {0.5f, 0.0f, 0.1f, 0.0f, 0.0f};
PID pidRoll  = {0.5f, 0.0f, 0.1f, 0.0f, 0.0f};
PID pidYaw   = {0.3f, 0.0f, 0.05f, 0.0f, 0.0f};

// ── Onboard IMU ───────────────────────────────────────────────────────────────
Adafruit_MPU6050 droneMpu;
Madgwick         droneFilter;
float            dronePitch = 0.0f;
float            droneRoll  = 0.0f;
float            droneYaw   = 0.0f;

// ── State ─────────────────────────────────────────────────────────────────────
volatile DronePacket latestPacket = {0, 0, 0, 0};
volatile bool        newPacket    = false;
unsigned long        lastPacketMs = 0;
unsigned long        lastLoopMs   = 0;
unsigned long        lastPrintMs  = 0;

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

  // Onboard IMU
  Wire.begin(IMU_SDA, IMU_SCL);
  if (!droneMpu.begin(0x68, &Wire)) {
    Serial.println("ERROR: Drone MPU6050 not found. Check wiring.");
    while (1) delay(10);
  }
  droneMpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  droneMpu.setGyroRange(MPU6050_RANGE_500_DEG);
  droneMpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  droneFilter.begin(500);  // 500Hz — tight stabilization loop
  Serial.println("Onboard IMU initialized.");

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

  // ── Read onboard IMU every iteration (decoupled from ESP-NOW rate) ──────────
  sensors_event_t da, dg, dtemp;
  droneMpu.getEvent(&da, &dg, &dtemp);
  droneFilter.updateIMU(
    dg.gyro.x, dg.gyro.y, dg.gyro.z,
    da.acceleration.x, da.acceleration.y, da.acceleration.z
  );
  dronePitch = droneFilter.getPitch();
  droneRoll  = droneFilter.getRoll();
  droneYaw   = droneFilter.getYaw();

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

  // ── PID (closed-loop: setpoint = glove angle, actual = drone IMU angle) ─────
  float outPitch = pidPitch.compute(pkt.pitch, dronePitch, dt);
  float outRoll  = pidRoll.compute(pkt.roll,   droneRoll,  dt);
  float outYaw   = pidYaw.compute(pkt.yaw,     droneYaw,   dt);

  // ── Motor mixing (X-frame) ───────────────────────────────────────────────────
  // FL: CCW  FR: CW  RL: CW  RR: CCW
  float fl = base + outPitch - outRoll - outYaw;
  float fr = base + outPitch + outRoll + outYaw;
  float rl = base - outPitch - outRoll + outYaw;
  float rr = base - outPitch + outRoll - outYaw;

  setMotors(fl, fr, rl, rr);

  if (now - lastPrintMs >= 100) {
    lastPrintMs = now;
    Serial.printf("SP P:%.1f R:%.1f Y:%.1f | ACT P:%.1f R:%.1f Y:%.1f | T:%d\n",
                  pkt.pitch, pkt.roll, pkt.yaw,
                  dronePitch, droneRoll, droneYaw, pkt.throttle);
  }
}
