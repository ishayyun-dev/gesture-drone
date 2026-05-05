#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <MadgwickAHRS.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

// ── Drone's MAC address ── update this with the actual ESP32-CAM MAC
// ──────────
uint8_t DRONE_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ── Packet structure (must match flight controller)
// ───────────────────────────
typedef struct DronePacket {
  float pitch;
  float roll;
  float yaw;
  int throttle; // 0–100 in fixed steps
} DronePacket;

// ── Tuning constants
// ──────────────────────────────────────────────────────────
#define BUMP_THRESHOLD 15.0f // m/s² spike to register a gesture
#define BUMP_COOLDOWN_MS 300 // ms between gesture triggers
#define THROTTLE_STEP 10     // % added/removed per bump
#define THROTTLE_MIN 0
#define THROTTLE_MAX 100
#define TX_INTERVAL_MS 20 // ~50Hz transmit rate

// ── Globals
// ───────────────────────────────────────────────────────────────────
Adafruit_MPU6050 mpu;
Madgwick filter;
DronePacket packet;

int throttle = 0;
float baselineZ = 0.0f;
bool baselineReady = false;
unsigned long lastBump = 0;
unsigned long lastTx = 0;
unsigned long lastLoop = 0;

// ── ESP-NOW send callback (optional debug)
// ────────────────────────────────────
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "TX OK" : "TX FAIL");
}

void setup() {
  Serial.begin(115200);

  // MPU6050
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found. Check wiring.");
    while (1)
      delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  Serial.println("MPU6050 initialized.");

  // Madgwick filter — sample rate matches TX_INTERVAL_MS (~50Hz)
  filter.begin(50);

  // WiFi (required for ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed.");
    while (1)
      delay(10);
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, DRONE_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ERROR: Failed to add ESP-NOW peer.");
    while (1)
      delay(10);
  }

  Serial.println("Glove transmitter ready.");
  lastLoop = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastLoop) / 1000.0f;
  lastLoop = now;

  // ── Read sensor
  // ─────────────────────────────────────────────────────────────
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  float gx = g.gyro.x * RAD_TO_DEG;
  float gy = g.gyro.y * RAD_TO_DEG;
  float gz = g.gyro.z * RAD_TO_DEG;

  // ── Madgwick filter
  // ──────────────────────────────────────────────────────────
  filter.updateIMU(gx, gy, gz, ax, ay, az);
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();

  // ── Baseline Z (settled after first 50 readings)
  // ────────────────────────────
  static int baselineCount = 0;
  static float baselineSum = 0.0f;
  if (!baselineReady) {
    baselineSum += az;
    baselineCount++;
    if (baselineCount >= 50) {
      baselineZ = baselineSum / baselineCount;
      baselineReady = true;
      Serial.printf("Baseline Z: %.2f m/s²\n", baselineZ);
    }
  }

  // ── Gesture detection
  // ────────────────────────────────────────────────────────
  if (baselineReady && (now - lastBump) > BUMP_COOLDOWN_MS) {
    float zDelta = az - baselineZ;
    if (zDelta > BUMP_THRESHOLD) {
      throttle =
          constrain(throttle + THROTTLE_STEP, THROTTLE_MIN, THROTTLE_MAX);
      lastBump = now;
      Serial.printf("UP bump → throttle: %d%%\n", throttle);
    } else if (zDelta < -BUMP_THRESHOLD) {
      throttle =
          constrain(throttle - THROTTLE_STEP, THROTTLE_MIN, THROTTLE_MAX);
      lastBump = now;
      Serial.printf("DOWN bump → throttle: %d%%\n", throttle);
    }
  }

  // ── Transmit at ~50Hz
  // ────────────────────────────────────────────────────────
  if (now - lastTx >= TX_INTERVAL_MS) {
    packet.pitch = pitch;
    packet.roll = roll;
    packet.yaw = yaw;
    packet.throttle = throttle;

    esp_now_send(DRONE_MAC, (uint8_t *)&packet, sizeof(packet));
    lastTx = now;

    Serial.printf("P:%.1f R:%.1f Y:%.1f T:%d%%\n", pitch, roll, yaw, throttle);
  }
}
