// ╔══════════════════════════════════════════════════════════════════╗
// ║  Micromouse — 16x16 flood-fill solver (SLIIT ROBOFEST 2026)      ║
// ║                                                                  ║
// ║  One button: short press = next mode, long press = start mode.   ║
// ║  The LED blinks the mode number:                                 ║
// ║    1 SEARCH       explore start -> goal -> start, save the map   ║
// ║    2 FAST 1       stored shortest path, safe speed, then home    ║
// ║    3 FAST 2       faster                                         ║
// ║    4 FAST 3       fastest                                        ║
// ║    5 CALIBRATE    learn ToF centred readings in the start cell   ║
// ║    6 SENSOR CHECK LED = front wall, readings on Serial           ║
// ║    7 HW CHECK     motor/encoder/gyro polarity, then a test move  ║
// ║    8 CLEAR MAP    forget the stored maze                         ║
// ║                                                                  ║
// ║  Place the robot in the start cell, back against the back wall.  ║
// ║  Error: fast blinks = code (see ErrorCode), press button to exit ║
// ╚══════════════════════════════════════════════════════════════════╝

#include <Wire.h>
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "imu.h"
#include "motion.h"
#include "maze.h"
#include "storage.h"

enum Mode {
  MODE_SEARCH, MODE_FAST1, MODE_FAST2, MODE_FAST3,
  MODE_CALIBRATE, MODE_SENSOR_CHECK, MODE_HW_CHECK, MODE_CLEAR_MAP,
  MODE_COUNT
};

enum ErrorCode {
  ERR_TOF_INIT = 1,   // A ToF sensor didn't respond
  ERR_GYRO_MOVING,    // Robot moved during gyro calibration
  ERR_CRASH,          // Tracking error too large: hit a wall or stalled
  ERR_NO_PATH,        // No route to the target (fast run: search first)
  ERR_CAL_RANGE,      // Calibration readings don't look like walls
  ERR_ENC_LEFT,       // Left encoder didn't count up driving forward
  ERR_ENC_RIGHT,      // Right encoder didn't count up driving forward
  ERR_GYRO_SIGN       // Heading didn't increase turning left
};

static uint8_t posX, posY, heading;
static uint8_t selectedMode = MODE_SEARCH;

// ---------------- UI ----------------

static bool buttonDown() { return digitalRead(PIN_BUTTON) == LOW; }

// `count` blinks, a pause, repeat.
static void showPattern(uint8_t count, unsigned long startMs) {
  unsigned long t = (millis() - startMs) % (count * 400UL + 1000);
  digitalWrite(PIN_LED, t < count * 400UL && (t % 400) < 150);
}

static uint8_t selectMode() {
  uint8_t mode = selectedMode;
  unsigned long patternStart = millis();
  while (true) {
    showPattern(mode + 1, patternStart);
    if (!buttonDown()) continue;
    delay(DEBOUNCE_MS);
    if (!buttonDown()) continue;

    unsigned long pressed = millis();
    bool longPress = false;
    while (buttonDown()) {
      if (!longPress && millis() - pressed >= LONG_PRESS_MS) {
        longPress = true;
        digitalWrite(PIN_LED, HIGH);  // Solid = release to start
      }
    }
    delay(DEBOUNCE_MS);
    digitalWrite(PIN_LED, LOW);

    if (longPress) {
      selectedMode = mode;
      return mode;
    }
    mode = (mode + 1) % MODE_COUNT;
    delay(400);
    patternStart = millis();
  }
}

static void showError(uint8_t code) {
  motors_brake();
  Serial.print(F("ERROR ")); Serial.println(code);
  unsigned long start = millis();
  while (!buttonDown()) {
    unsigned long t = (millis() - start) % (code * 200UL + 1200);
    digitalWrite(PIN_LED, t < code * 200UL && (t % 200) < 80);
  }
  while (buttonDown());
  delay(DEBOUNCE_MS);
  digitalWrite(PIN_LED, LOW);
}

static void flashLed(uint8_t times, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(PIN_LED, HIGH); delay(onMs);
    digitalWrite(PIN_LED, LOW); delay(offMs);
  }
}

// ---------------- Logging ----------------

static void printMap() {
  for (int8_t y = MAZE_SIZE - 1; y >= 0; y--) {
    for (uint8_t x = 0; x < MAZE_SIZE; x++) {
      Serial.print('+');
      Serial.print(!maze_known(x, y, NORTH) ? F(" . ") : maze_wall(x, y, NORTH) ? F("---") : F("   "));
    }
    Serial.println('+');
    for (uint8_t x = 0; x < MAZE_SIZE; x++) {
      Serial.print(!maze_known(x, y, WEST) ? ':' : maze_wall(x, y, WEST) ? '|' : ' ');
      if (maze_in_target(x, y, TARGET_GOAL)) Serial.print(F(" G "));
      else if (maze_in_target(x, y, TARGET_START)) Serial.print(F(" S "));
      else Serial.print(maze_visited(x, y) ? F("   ") : F(" ? "));
    }
    Serial.println('|');
  }
  for (uint8_t x = 0; x < MAZE_SIZE; x++) Serial.print(F("+---"));
  Serial.println('+');
}

// Optimistic == known-only length means the stored fast path is the true shortest path.
static void reportPathStatus() {
  uint8_t optimistic, knownOnly;
  maze_path_lengths(optimistic, knownOnly);
  Serial.print(F("Shortest path: ")); Serial.print(optimistic);
  Serial.print(F(" cells possible, ")); Serial.print(knownOnly); Serial.println(F(" cells proven"));
  if (optimistic == knownOnly) flashLed(1, 1500, 300);  // Long flash = proven shortest
  else flashLed(3, 150, 150);                            // Three blinks = search again to improve
}

// ---------------- Run building blocks ----------------

// Brake until every ToF has delivered three new readings (or a timeout).
static void waitForFreshReadings() {
  uint8_t before[3];
  for (uint8_t i = 0; i < 3; i++) before[i] = sensors_count(i);
  unsigned long start = millis();
  while (millis() - start < 400) {
    motion_wait(5);
    bool done = true;
    for (uint8_t i = 0; i < 3; i++) {
      if ((uint8_t)(sensors_count(i) - before[i]) < 3) done = false;
    }
    if (done) break;
  }
}

static void senseCell() {
  waitForFreshReadings();
  uint16_t l = sensors_median(TOF_L), f = sensors_median(TOF_F), r = sensors_median(TOF_R);
  bool wl = sensors_wall_left(l), wf = sensors_wall_front(f), wr = sensors_wall_right(r);
  if (maze_update(posX, posY, heading, wl, wf, wr)) {
    Serial.println(F("Opening to the west of column 0: start is the right-hand corner, map moved"));
  }
  storage_save_map();

  Serial.print(F("Cell (")); Serial.print(posX); Serial.print(','); Serial.print(posY);
  Serial.print(F(") h=")); Serial.print(heading);
  Serial.print(F(" L/F/R mm ")); Serial.print(l); Serial.print('/'); Serial.print(f); Serial.print('/'); Serial.print(r);
  Serial.print(F(" walls ")); Serial.print(wl); Serial.print(wf); Serial.println(wr);
}

static bool turnTo(uint8_t dir) {
  uint8_t rel = (dir - heading) & 3;
  bool ok = true;
  if (rel == 1) ok = motion_turn(-1);       // Right
  else if (rel == 3) ok = motion_turn(1);   // Left
  else if (rel == 2) ok = motion_turn(2);   // About-turn
  heading = dir;
  return ok;
}

// Hand-clear delay, gyro calibration, then from the back wall to the start cell centre.
static bool beginRun() {
  digitalWrite(PIN_LED, HIGH);
  delay(START_DELAY_MS);
  digitalWrite(PIN_LED, LOW);
  if (!imu_calibrate()) { showError(ERR_GYRO_MOVING); return false; }
  motion_reset();
  posX = mazeStartX;
  posY = 0;
  heading = NORTH;
  if (!motion_straight(START_TO_CENTER_MM, 100.0f, SEARCH_ACCEL, false)) { showError(ERR_CRASH); return false; }
  return true;
}

// Back in the start cell: face out again and reverse into the back wall to square up.
static void finishAtStart() {
  turnTo(NORTH);
  motors_set(-60, -60);
  delay(600);
  motors_brake();
}

// Stop-and-go search: sense each new cell, re-flood with unknown walls assumed open, and
// drive straight through already-visited cells without stopping.
static bool searchTo(Target t) {
  for (uint16_t steps = 0; steps < 1024; steps++) {
    senseCell();
    if (maze_in_target(posX, posY, t)) return true;

    maze_flood(t, false);
    int8_t dir = maze_best_dir(posX, posY, heading, false);
    if (dir < 0) { showError(ERR_NO_PATH); return false; }
    if (!turnTo(dir)) { showError(ERR_CRASH); return false; }

    uint8_t cells = maze_straight_cells(posX, posY, dir, t, false, true);
    if (!motion_straight(cells * CELL_MM, SEARCH_SPEED, SEARCH_ACCEL, true)) { showError(ERR_CRASH); return false; }
    for (uint8_t i = 0; i < cells; i++) {
      maze_set_wall(posX, posY, dir, false);  // We drove through it, so it's open
      maze_step(posX, posY, dir);
    }
  }
  showError(ERR_NO_PATH);
  return false;
}

// Follow the shortest path through known-open walls only, one straight segment at a time.
static bool followKnownPath(Target t, float speed, float accel) {
  maze_flood(t, true);
  while (!maze_in_target(posX, posY, t)) {
    int8_t dir = maze_best_dir(posX, posY, heading, true);
    if (dir < 0) { showError(ERR_NO_PATH); return false; }
    if (!turnTo(dir)) { showError(ERR_CRASH); return false; }
    uint8_t cells = maze_straight_cells(posX, posY, dir, t, true, false);
    if (!motion_straight(cells * CELL_MM, speed, accel, true)) { showError(ERR_CRASH); return false; }
    for (uint8_t i = 0; i < cells; i++) maze_step(posX, posY, dir);
  }
  return true;
}

// From the goal back to the start without a manual reset (+20 s). If the shortest path is
// already proven there is nothing left to learn, so go home along known cells without
// stopping; otherwise search on the way to explore more of the maze.
static bool returnHome() {
  uint8_t optimistic, knownOnly;
  maze_path_lengths(optimistic, knownOnly);
  if (optimistic == knownOnly) return followKnownPath(TARGET_START, FAST1_SPEED, FAST1_ACCEL);
  return searchTo(TARGET_START);
}

// ---------------- Modes ----------------

static void runSearch() {
  if (!beginRun()) return;
  if (!searchTo(TARGET_GOAL)) return;
  Serial.println(F("GOAL"));
  flashLed(2, 200, 200);
  if (!returnHome()) return;
  finishAtStart();
  printMap();
  reportPathStatus();
}

static void runFast(float speed, float accel) {
  maze_flood(TARGET_GOAL, true);
  if (maze_dist(mazeStartX, 0) == DIST_UNREACHABLE) { showError(ERR_NO_PATH); return; }
  if (!beginRun()) return;
  if (!followKnownPath(TARGET_GOAL, speed, accel)) return;
  Serial.println(F("GOAL"));
  flashLed(2, 200, 200);
  if (!returnHome()) return;
  finishAtStart();
  reportPathStatus();
}

// In the start cell (walls left and right, back wall behind): the centred side readings,
// then about-turn to read the back wall as the front-stop distance.
static void runCalibrate() {
  if (!beginRun()) return;
  waitForFreshReadings();
  uint16_t l = sensors_median(TOF_L);
  uint16_t r = sensors_median(TOF_R);
  if (!motion_turn(2)) { showError(ERR_CRASH); return; }
  waitForFreshReadings();
  uint16_t f = sensors_median(TOF_F);
  if (!motion_turn(2)) { showError(ERR_CRASH); return; }
  finishAtStart();

  Serial.print(F("Calibration L/F/R: ")); Serial.print(l); Serial.print('/');
  Serial.print(f); Serial.print('/'); Serial.println(r);
  if (l > 150 || r > 150 || f > 150) { showError(ERR_CAL_RANGE); return; }
  cal.sideL = l;
  cal.sideR = r;
  cal.front = f;
  storage_save_cal(cal);
  flashLed(1, 1500, 300);
}

static void runSensorCheck() {
  printMap();
  Serial.print(F("Calibration L/F/R: ")); Serial.print(cal.sideL); Serial.print('/');
  Serial.print(cal.front); Serial.print('/'); Serial.println(cal.sideR);
  while (!buttonDown()) {
    motion_wait(200);
    uint16_t l = sensors_latest(TOF_L), f = sensors_latest(TOF_F), r = sensors_latest(TOF_R);
    digitalWrite(PIN_LED, sensors_wall_front(f));
    Serial.print(F("L/F/R mm ")); Serial.print(l); Serial.print('/'); Serial.print(f); Serial.print('/'); Serial.print(r);
    Serial.print(F("  walls ")); Serial.print(sensors_wall_left(l)); Serial.print(sensors_wall_front(f));
    Serial.println(sensors_wall_right(r));
  }
  while (buttonDown());
  delay(DEBOUNCE_MS);
  digitalWrite(PIN_LED, LOW);
}

static void runOpenLoop(int left, int right, uint16_t ms) {
  unsigned long start = millis();
  motors_set(left, right);
  while (millis() - start < ms) { imu_update(); delay(5); }
  motors_brake();
  delay(300);
}

// Short open-loop pulses check wiring polarity; if all good, a closed-loop test move:
// one cell forward, four right turns, four left turns (should end square to where it started).
// If an encoder error appears: wheel turned backwards -> flip MOTOR_x_INVERT,
// wheel turned forwards -> flip ENC_x_INVERT.
static void runHardwareCheck() {
  delay(START_DELAY_MS);
  if (!imu_calibrate()) { showError(ERR_GYRO_MOVING); return; }
  motion_reset();

  long l0, r0, l1, r1;
  encoders_read(l0, r0);
  runOpenLoop(80, 80, 300);
  encoders_read(l1, r1);
  Serial.print(F("Forward counts L/R: ")); Serial.print(l1 - l0); Serial.print('/'); Serial.println(r1 - r0);
  if (l1 - l0 < 20) { showError(ERR_ENC_LEFT); return; }
  if (r1 - r0 < 20) { showError(ERR_ENC_RIGHT); return; }

  float h0 = imu_heading();
  runOpenLoop(-80, 80, 300);
  Serial.print(F("Left pivot heading change: ")); Serial.println(imu_heading() - h0);
  if (imu_heading() - h0 < 10) { showError(ERR_GYRO_SIGN); return; }

  motion_reset();
  if (!motion_straight(CELL_MM, SEARCH_SPEED, SEARCH_ACCEL, false)) { showError(ERR_CRASH); return; }
  for (uint8_t i = 0; i < 4; i++) if (!motion_turn(-1)) { showError(ERR_CRASH); return; }
  for (uint8_t i = 0; i < 4; i++) if (!motion_turn(1)) { showError(ERR_CRASH); return; }
  flashLed(1, 1500, 300);
}

static void runClearMap() {
  maze_init();
  storage_save_map();
  Serial.println(F("Map cleared"));
  flashLed(5, 80, 80);
}

// ---------------- Arduino entry points ----------------

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  motors_init();
  encoders_init();

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);  // Recover instead of hanging if motor noise upsets the bus

  bool tofOk = sensors_init();
  imu_init();
  if (!storage_load_cal(cal)) Serial.println(F("No stored calibration, using defaults"));
  if (!storage_load_map()) maze_init();

  Serial.println(F("\n=== MICROMOUSE ==="));
  printMap();
  if (!tofOk) showError(ERR_TOF_INIT);
}

void loop() {
  uint8_t mode = selectMode();
  switch (mode) {
    case MODE_SEARCH:       runSearch(); break;
    case MODE_FAST1:        runFast(FAST1_SPEED, FAST1_ACCEL); break;
    case MODE_FAST2:        runFast(FAST2_SPEED, FAST2_ACCEL); break;
    case MODE_FAST3:        runFast(FAST3_SPEED, FAST3_ACCEL); break;
    case MODE_CALIBRATE:    runCalibrate(); break;
    case MODE_SENSOR_CHECK: runSensorCheck(); break;
    case MODE_HW_CHECK:     runHardwareCheck(); break;
    case MODE_CLEAR_MAP:    runClearMap(); break;
  }
  motors_brake();
}
