#include "sensors.h"
#include "config.h"
#include <Wire.h>
#include <VL53L0X.h>

Calibration cal = { DEFAULT_SIDE_L_MM, DEFAULT_SIDE_R_MM, DEFAULT_FRONT_MM };

static VL53L0X dev[3];
static uint16_t history[3][3];
static uint8_t count[3];
static unsigned long lastMs[3];
static const uint8_t xshut[3] = { PIN_TOF_XSHUT_L, PIN_TOF_XSHUT_F, PIN_TOF_XSHUT_R };
static const uint8_t addr[3] = { TOF_ADDR_L, TOF_ADDR_F, TOF_ADDR_R };

bool sensors_init() {
  for (uint8_t i = 0; i < 3; i++) { pinMode(xshut[i], OUTPUT); digitalWrite(xshut[i], LOW); }
  delay(20);

  bool ok = true;
  for (uint8_t i = 0; i < 3; i++) {
    history[i][0] = history[i][1] = history[i][2] = TOF_NONE;
    digitalWrite(xshut[i], HIGH);
    delay(10);
    dev[i].setTimeout(50);
    if (!dev[i].init()) { ok = false; continue; }
    dev[i].setAddress(addr[i]);
    dev[i].setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
    dev[i].startContinuous();
  }
  return ok;
}

void sensors_poll() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < 3; i++) {
    // A new result can't be ready much sooner than the timing budget
    if (now - lastMs[i] < TOF_TIMING_BUDGET_US / 1000 - 8) continue;
    if ((dev[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) continue;
    uint16_t mm = dev[i].readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    dev[i].writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (mm == 0 || mm >= 8000) mm = TOF_NONE;
    history[i][2] = history[i][1];
    history[i][1] = history[i][0];
    history[i][0] = mm;
    lastMs[i] = now;
    count[i]++;
  }
}

uint16_t sensors_latest(uint8_t i) {
  if (millis() - lastMs[i] > TOF_STALE_MS) return TOF_NONE;
  return history[i][0];
}

uint16_t sensors_median(uint8_t i) {
  uint16_t a = history[i][0], b = history[i][1], c = history[i][2];
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;  // TOF_NONE sorts high, so one bad sample is rejected
}

uint8_t sensors_count(uint8_t i) { return count[i]; }

bool sensors_wall_left(uint16_t mm)  { return mm < cal.sideL + WALL_DETECT_MARGIN_MM; }
bool sensors_wall_front(uint16_t mm) { return mm < cal.front + WALL_DETECT_MARGIN_MM; }
bool sensors_wall_right(uint16_t mm) { return mm < cal.sideR + WALL_DETECT_MARGIN_MM; }
