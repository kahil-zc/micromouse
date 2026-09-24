#include "storage.h"
#include "maze.h"
#include <EEPROM.h>

#define MAP_MAGIC     0xA9
#define CAL_MAGIC     0x5C
#define ADDR_MAP      0
#define ADDR_START_X  1
#define ADDR_WALLS    2
#define ADDR_CAL      264

bool storage_load_map() {
  if (EEPROM.read(ADDR_MAP) != MAP_MAGIC) return false;
  uint8_t startX = EEPROM.read(ADDR_START_X);
  if (startX != 0 && startX != MAZE_SIZE - 1) return false;
  mazeStartX = startX;
  for (uint16_t i = 0; i < MAZE_SIZE * MAZE_SIZE; i++) mazeWalls[i] = EEPROM.read(ADDR_WALLS + i);
  return true;
}

void storage_save_map() {
  EEPROM.update(ADDR_START_X, mazeStartX);
  for (uint16_t i = 0; i < MAZE_SIZE * MAZE_SIZE; i++) EEPROM.update(ADDR_WALLS + i, mazeWalls[i]);
  EEPROM.update(ADDR_MAP, MAP_MAGIC);
}

void storage_clear_map() {
  EEPROM.update(ADDR_MAP, 0xFF);
}

bool storage_load_cal(Calibration &c) {
  if (EEPROM.read(ADDR_CAL) != CAL_MAGIC) return false;
  EEPROM.get(ADDR_CAL + 1, c);
  return true;
}

void storage_save_cal(const Calibration &c) {
  EEPROM.put(ADDR_CAL + 1, c);
  EEPROM.update(ADDR_CAL, CAL_MAGIC);
}
