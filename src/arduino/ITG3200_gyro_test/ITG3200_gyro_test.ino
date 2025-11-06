
#include <Arduino.h>
#include <Wire.h>
#include "ITG3200.h"

ITG3200 gyro;

// Integration & counting
double totalAngle = 0.0;      // continuous integrated angle in degrees (can grow + or -)
double z_offset = 0.0;        // library zeroCalibrate sets internal offsets, but keep local if needed
unsigned long lastTime = 0;

// Lap/step counting
long lapCount = 0;            // total full rotations (can be negative)
long prevRevs = 0;            // floor(totalAngle / 360) at previous step

// Filtering / noise suppression
const float FILTER_ALPHA = 0.3f;    // 0..1, higher = more responsive, lower = smoother
float gz_filtered = 0.0f;           // filtered angular rate (°/s)
const float MIN_RATE_DPS = 3.0f;    // ignore rates smaller than this (deg/s) to avoid drift count
const unsigned long MIN_STEP_INTERVAL_MS = 50; // safety debounce (not strictly needed with rev logic)

unsigned long lastStepTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(200);

  Serial.println("ITG3200 - robust lap counter starting...");
  gyro.init();

  // Let sensor warm up
  delay(200);

  // Library zeroCalibrate: samples, delay_ms
  Serial.println("Calibrating gyro - keep sensor still...");
  gyro.zeroCalibrate(400, 5); // increase samples for better bias estimate
  Serial.println("Calibration done.");

  // Initialize timing
  lastTime = millis();
  totalAngle = 0.0;
  prevRevs = (long)floor(totalAngle / 360.0);
  lastStepTime = 0;

  Serial.println("Ready. Rotations will be counted bidirectionally.");
}

void loop() {
  // Read gyro (deg/s) using library
  float gx, gy, gz;
  gyro.getAngularVelocity(&gx, &gy, &gz); // gz is degrees per second

  // Time delta
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f; // seconds
  if (dt <= 0) dt = 0.001f; // safety
  lastTime = now;

  // Simple low-pass filter on gz to remove jitter
  gz_filtered = FILTER_ALPHA * gz + (1.0f - FILTER_ALPHA) * gz_filtered;

  // Optionally ignore tiny rates (helps when stationary)
  float gz_use = gz_filtered;
  if (fabs(gz_use) < MIN_RATE_DPS) gz_use = 0.0f;

  // Integrate to get continuous total angle (note: positive gz increases totalAngle)
  totalAngle += (double)gz_use * dt;

  // Determine integer revolution count
  long currRevs = (long)floor(totalAngle / 360.0);

  // If revolutions changed since last update, update lapCount by the delta.
  // This handles multiple revolutions between updates (fast spins).
  if (currRevs != prevRevs) {
    long delta = currRevs - prevRevs;
    // Optional debounce: only accept if enough time since last step (helps against noisy flips)
    if ((now - lastStepTime) >= MIN_STEP_INTERVAL_MS) {
      lapCount += delta;           // delta may be positive (cw) or negative (ccw)
      lastStepTime = now;
      // debug message
      if (delta > 0) {
        Serial.print("Forward +");
        Serial.print(delta);
        Serial.println(" rev(s)");
      } else {
        Serial.print("Backward ");
        Serial.print(delta);
        Serial.println(" rev(s)");
      }
    }
    prevRevs = currRevs;
  }

  // For display: map totalAngle to 0..360
  double modAngle = fmod(totalAngle, 360.0);
  if (modAngle < 0) modAngle += 360.0;

  // Print status
  Serial.print("modAngle: ");
  Serial.print(modAngle, 2);
  Serial.print(" °  |  totalAngle: ");
  Serial.print(totalAngle, 2);
  Serial.print(" °  |  laps: ");
  Serial.println(lapCount);

  // small delay (adjust to your requirements; faster update for fast motors)
  delay(30);
}
