#pragma once
#include "sensors.h"

// EEPROM layout (ATmega328P has 1024 bytes):
//   0        map magic 0xA9
//   1        start column
//   2..257   wall map (one byte per cell)
//   264      calibration magic 0x5C
//   265..    Calibration struct

// Loads the map into the maze module. Returns false if none is stored.
bool storage_load_map();
// Writes only bytes that changed, so it's cheap to call after every cell.
void storage_save_map();
void storage_clear_map();

bool storage_load_cal(Calibration &c);
void storage_save_cal(const Calibration &c);
