#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found. Check wiring.");
    while (1) delay(10);
  }

  Serial.println("MPU6050 initialized successfully.");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("X: "); Serial.print(a.acceleration.x);
  Serial.print("  Y: "); Serial.print(a.acceleration.y);
  Serial.print("  Z: "); Serial.println(a.acceleration.z);

  delay(100);
}
