#pragma once
#include <stdint.h>

enum { TOF_L = 0, TOF_F = 1, TOF_R = 2 };

// Centred readings (raw mm). Loaded from EEPROM, or config defaults.
struct Calibration {
  uint16_t sideL;
  uint16_t sideR;
  uint16_t front;
};
extern Calibration cal;

// Returns false if any sensor failed to initialise.
bool sensors_init();
// Non-blocking: fetches any new measurement. Call every control tick.
void sensors_poll();
// Latest reading in mm, or TOF_NONE if invalid or stale.
uint16_t sensors_latest(uint8_t i);
// Median of the last three readings (use after holding still), or TOF_NONE.
uint16_t sensors_median(uint8_t i);
// Number of measurements received since power-up (wraps).
uint8_t sensors_count(uint8_t i);

bool sensors_wall_left(uint16_t mm);
bool sensors_wall_front(uint16_t mm);
bool sensors_wall_right(uint16_t mm);
