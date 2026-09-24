#include "imu.h"
#include "config.h"
#include <Wire.h>

#define GYRO_LSB_PER_DPS 65.5f   // +-500 dps full scale
#define CALIB_SAMPLES    400
#define CALIB_MAX_SPREAD 60      // raw LSB; larger means the robot was moving

static float offset = 0;
static float heading = 0;
static float rate = 0;
static unsigned long lastUs = 0;

static void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

static int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true);
  uint8_t high = Wire.read();  // Separate statements: evaluation order matters
  uint8_t low = Wire.read();
  return (int16_t)((high << 8) | low);
}

void imu_init() {
  writeReg(0x6B, 0x01);  // Wake, clock from gyro X PLL
  delay(50);
  writeReg(0x1A, 0x03);  // DLPF ~44 Hz
  writeReg(0x1B, 0x08);  // Gyro full scale +-500 dps
  lastUs = micros();
}

bool imu_calibrate() {
  long sum = 0;
  int16_t lo = 32767, hi = -32768;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    int16_t v = readGyroZRaw();
    sum += v;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
    delay(2);
  }
  if (hi - lo > CALIB_MAX_SPREAD) return false;
  offset = (float)sum / CALIB_SAMPLES;
  imu_reset_heading();
  return true;
}

void imu_update() {
  unsigned long now = micros();
  float dt = (now - lastUs) * 1e-6f;
  lastUs = now;
  if (dt > 0.05f) dt = 0;  // Not called for a while: don't integrate a stale rate
  rate = (readGyroZRaw() - offset) / GYRO_LSB_PER_DPS * GYRO_SCALE;
  if (GYRO_INVERT) rate = -rate;
  heading += rate * dt;
}

void imu_reset_heading() {
  heading = 0;
  lastUs = micros();
}

float imu_heading() { return heading; }
float imu_rate() { return rate; }
