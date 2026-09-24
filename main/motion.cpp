#include "motion.h"
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "imu.h"
#include "sensors.h"
#include <math.h>

static float targetHeading = 0;   // deg, absolute; a multiple of 90 between moves
static float prevFwdErr = 0;
static float steerDeg = 0;
static unsigned long lastTickUs = 0;

// Waits for the next control period, then updates the gyro and ToF. Returns dt in seconds.
static float tick() {
  unsigned long now = micros();
  while (now - lastTickUs < CONTROL_PERIOD_US) now = micros();
  float dt = (now - lastTickUs) * 1e-6f;
  lastTickUs = now;
  if (dt > 0.02f) dt = CONTROL_PERIOD_US * 1e-6f;  // First tick after being idle
  imu_update();
  sensors_poll();
  return dt;
}

// Forward + rotation controllers. Positive rotation output speeds up the right wheel (CCW).
// Returns false if tracking error is large enough to mean a collision or stall.
static bool drive(float dt, float posCmd, float vCmd, float headingCmd, float omegaCmd) {
  float fwdErr = posCmd - encoders_distance_mm();
  float uF = KFF_V * vCmd + KP_FWD * fwdErr + KD_FWD * (fwdErr - prevFwdErr) / dt;
  prevFwdErr = fwdErr;
  if (vCmd > 1) uF += KFF_S;
  else if (vCmd < -1) uF -= KFF_S;

  float rotErr = headingCmd - imu_heading();
  float uR = KFF_W * omegaCmd + KP_ROT * rotErr + KD_ROT * (omegaCmd - imu_rate());
  if (omegaCmd > 1) uR += KFF_S;
  else if (omegaCmd < -1) uR -= KFF_S;

  motors_set((int)(uF - uR), (int)(uF + uR));
  return fabs(fwdErr) < MAX_FWD_ERR_MM && fabs(rotErr) < MAX_ROT_ERR_DEG;
}

// Heading offset (deg, + = steer left) that pulls the robot back to the corridor centre.
// Side readings far from the calibrated centred value (gaps, posts) are ignored.
static float wallSteer() {
  uint16_t l = sensors_latest(TOF_L);
  uint16_t r = sensors_latest(TOF_R);
  int errL = (int)l - (int)cal.sideL;
  int errR = (int)r - (int)cal.sideR;
  bool hasL = l != TOF_NONE && abs(errL) < SIDE_TRACK_WINDOW_MM;
  bool hasR = r != TOF_NONE && abs(errR) < SIDE_TRACK_WINDOW_MM;

  float offsetMm = 0;  // + = robot is right of centre
  if (hasL && hasR) offsetMm = (errL - errR) * 0.5f;
  else if (hasL) offsetMm = errL;
  else if (hasR) offsetMm = -errR;

  float target = constrain(offsetMm * WALL_KP_DEG_PER_MM, -WALL_MAX_STEER_DEG, WALL_MAX_STEER_DEG);
  steerDeg += WALL_STEER_ALPHA * (target - steerDeg);
  return steerDeg;
}

void motion_reset() {
  imu_reset_heading();
  targetHeading = 0;
  steerDeg = 0;
}

bool motion_straight(float distanceMm, float vMax, float accel, bool alignFront) {
  float start = encoders_distance_mm();
  float target = distanceMm;  // Relative to start; may be corrected by the front wall
  float posCmd = 0, v = 0;
  prevFwdErr = 0;
  steerDeg = 0;
  uint8_t frontCount = sensors_count(TOF_F);
  unsigned long settleStart = 0;

  while (true) {
    float dt = tick();
    float pos = encoders_distance_mm() - start;

    // A fresh front reading that agrees with a wall at the end of the move pins the stop point
    if (alignFront && sensors_count(TOF_F) != frontCount) {
      frontCount = sensors_count(TOF_F);
      uint16_t f = sensors_latest(TOF_F);
      float expected = target - pos;
      if (f != TOF_NONE && expected < CELL_MM) {
        float measured = (float)f - cal.front - v * TOF_LATENCY_S;
        if (fabs(measured - expected) < FRONT_ALIGN_MAX_CORR_MM) target = pos + measured;
      }
    }

    if (settleStart == 0) {
      float remaining = target - posCmd;
      if (remaining > 0) {
        float vStop = sqrtf(2.0f * accel * remaining);
        v = min(min(v + accel * dt, vMax), vStop);
        posCmd = min(posCmd + v * dt, target);
      }
      if (posCmd >= target) { v = 0; settleStart = millis(); }
    }
    if (settleStart != 0) {
      // Follow late front-wall corrections gently so the error never looks like a crash
      float step = 200.0f * dt;
      posCmd += constrain(target - posCmd, -step, step);
    }

    if (!drive(dt, start + posCmd, v, targetHeading + wallSteer(), 0)) {
      motors_brake();
      return false;
    }

    if (settleStart != 0) {
      bool settled = fabs(target - pos) < 2.0f;
      if (settled || millis() - settleStart > SETTLE_MS) break;
    }
  }
  motors_brake();
  return true;
}

bool motion_turn(int8_t quarterTurnsCcw) {
  float angle = 90.0f * quarterTurnsCcw;
  float sign = angle > 0 ? 1.0f : -1.0f;
  float total = fabs(angle);
  float startHeading = targetHeading;
  targetHeading += angle;

  float holdAt = encoders_distance_mm();
  prevFwdErr = 0;
  float progress = 0, w = 0;
  unsigned long settleStart = 0;

  while (true) {
    float dt = tick();
    if (settleStart == 0) {
      float remaining = total - progress;
      w = min(min(w + TURN_ALPHA * dt, TURN_OMEGA), sqrtf(2.0f * TURN_ALPHA * remaining));
      progress += w * dt;
      if (progress >= total) { progress = total; w = 0; settleStart = millis(); }
    }

    if (!drive(dt, holdAt, 0, startHeading + sign * progress, sign * w)) {
      motors_brake();
      return false;
    }

    if (settleStart != 0) {
      bool settled = fabs(targetHeading - imu_heading()) < 1.0f && fabs(imu_rate()) < 20.0f;
      if (settled || millis() - settleStart > SETTLE_MS) break;
    }
  }
  motors_brake();
  return true;
}

void motion_wait(uint16_t ms) {
  motors_brake();
  unsigned long start = millis();
  while (millis() - start < ms) tick();
}
