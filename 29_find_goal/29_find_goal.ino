// ╔══════════════════════════════════════════════════════════════════╗
// ║  29_find_goal                                                    ║
// ║  Finds the goal room itself: no fixed goal, no fixed maze shape. ║
// ║  Driving and turning are 14_floodfill_final's, unchanged.        ║
// ║                                                                  ║
// ║  THE GOAL = a 2x2 room (36 x 36 cm) with no walls inside it.     ║
// ║  Nowhere else in a maze has that, so wherever it is, the robot   ║
// ║  recognises it once it has seen the 4 walls inside.              ║
// ║                                                                  ║
// ║  SEARCH: at every cell it reads the 3 walls (ToFs averaged over  ║
// ║  0.3 s), writes them on the map, and drives to the NEAREST cell  ║
// ║  it hasn't been in yet (shortest way on the map so far, straight ║
// ║  on preferred). So it sweeps the maze outward until it finds the ║
// ║  2x2 room, then drives home. The map is 19 x 19 cells with the   ║
// ║  start in the middle, so the maze can go any way from the start  ║
// ║  (left, right, ahead, behind) and it still fits.                 ║
// ║  FAST RUN: the shortest way on walls it has actually seen, full  ║
// ║  straights at speed, then home.                                  ║
// ║                                                                  ║
// ║  Clearing: every SEARCH starts with an empty map. Holding the    ║
// ║  button while switching on also wipes the saved map.             ║
// ║                                                                  ║
// ║  Button (START), robot in the start cell, back to the wall:      ║
// ║    short press = SEARCH       long press (1 s, LED on) = FAST RUN ║
// ║    held during a search = give up and drive home                 ║
// ║  LED: 5 quick blinks = goal room found / reached. LED stays on   ║
// ║  = 3 failed moves in a row: put it in the start cell, press.     ║
// ║                                                                  ║
// ║  USB (115200): it prints only the map (when it finds the goal    ║
// ║  and when it gets home) and real problems. While it waits:       ║
// ║  L F R readings with failed-reading counts ("L 35/0"); send 'm'  ║
// ║  to print the map. Map: ### wall, . unseen, ? not visited,       ║
// ║  S start, G goal room, ^ > v < robot.                            ║
// ║                                                                  ║
// ║  ToF readings: failed measurements (sensor status, or 0 mm) are  ║
// ║  thrown away; the emergency stop needs 2 close readings in a row.║
// ║  A wall the map didn't have (front ToF, or the side ToF just     ║
// ║  before a turn) is written down and it chooses again.            ║
// ║                                                                  ║
// ║  Movement (from 14): stop at a "decision point" in every cell    ║
// ║  (axle 80 mm before the centre), 90° turns are 80 mm arcs,       ║
// ║  U-turns shuffle so the long rear has room. Side ToFs steer to   ║
// ║  the corridor centre and correct the gyro; the front ToF stops   ║
// ║  it at walls.                                                    ║
// ╚══════════════════════════════════════════════════════════════════╝

#include <Wire.h>
#include <VL53L0X.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>

// --- PIN ASSIGNMENTS ---
#define MOTOR_STBY 12
#define MOTOR_A_PWM 5
#define MOTOR_A_IN1 8
#define MOTOR_A_IN2 9
#define MOTOR_B_PWM 6
#define MOTOR_B_IN1 10
#define MOTOR_B_IN2 11

#define ENCODER_LEFT_C1 2
#define ENCODER_LEFT_C2 4
#define ENCODER_RIGHT_C1 3
#define ENCODER_RIGHT_C2 7

#define TOF_XSHUT_LEFT   A0
#define TOF_XSHUT_FRONT  A1
#define TOF_XSHUT_RIGHT  A2

#define STATUS_LED 13
#define START_BUTTON A3
#define MPU_ADDR 0x68

// === MAZE LAYOUT ===
// Cells are (x, y). The robot starts in (START_X, START_Y) facing +y ("north"); +x is to its
// right. The map is MAP_SIZE x MAP_SIZE with the start in the middle, so a maze up to
// MAP_SIZE / 2 + 1 cells long (10) fits whichever way it goes from the start.
#define MAP_SIZE    19
#define START_X      9
#define START_Y      9

// === YOUR CALIBRATED TOF BIAS ===
// reading - bias = gap from that face of the robot to the wall
#define FRONT_BIAS_MM  37
#define LEFT_BIAS_MM   25
#define RIGHT_BIAS_MM  50

// === ROBOT GEOMETRY ===
#define ROBOT_WIDTH_MM     99.0f
#define ROBOT_LENGTH_MM   126.0f
#define AXLE_TO_FRONT_MM   47.0f   // front ToF sits at the front edge
#define SIDE_TOF_AHEAD_MM  32.0f   // side ToFs are this far in front of the axle
#define TRACK_WIDTH_MM     88.0f   // MEASURE: middle of left tyre -> middle of right tyre

// === MAZE DIMENSIONS ===
#define CELL_MM           180.0f
#define HALF_CORRIDOR_MM   84.0f   // cell centre -> wall face, (180 - 12 mm wall) / 2
#define POST_HALF_MM        6.0f

// === TURN GEOMETRY ===
#define TURN_RADIUS_MM     80.0f   // arc radius of 90° turns (axle path)
#define SAFETY_MM           3.0f   // extra room kept around the corners in U-turns

// Derived geometry. The robot's position is its axle midpoint.
// Decision point = axle TURN_RADIUS_MM before the cell centre. The robot stops there in every
// cell; an arc turn started there ends centred in the new corridor.
#define HALF_WIDTH_MM      (ROBOT_WIDTH_MM / 2.0f)
#define HALF_TRACK_MM      (TRACK_WIDTH_MM / 2.0f)
#define AXLE_TO_REAR_MM    (ROBOT_LENGTH_MM - AXLE_TO_FRONT_MM)
#define DECISION_BACK_MM   TURN_RADIUS_MM
#define SIDE_GAP_MM        (HALF_CORRIDOR_MM - HALF_WIDTH_MM)                         // centred side reading, ~34.5
#define FRONT_GAP_MM       (HALF_CORRIDOR_MM + DECISION_BACK_MM - AXLE_TO_FRONT_MM)   // front reading at decision point, ~117
#define ARC_RATIO          ((TURN_RADIUS_MM - HALF_TRACK_MM) / (TURN_RADIUS_MM + HALF_TRACK_MM))  // inner / outer wheel speed
#define ARC_OUTER_REACH_MM (sqrt(sq(TURN_RADIUS_MM + HALF_WIDTH_MM) + sq(AXLE_TO_REAR_MM)) - TURN_RADIUS_MM)
#define SPIN_R_FRONT_MM    (sqrt(sq(AXLE_TO_FRONT_MM) + sq(HALF_WIDTH_MM)))  // ~68
#define SPIN_R_REAR_MM     (sqrt(sq(AXLE_TO_REAR_MM) + sq(HALF_WIDTH_MM)))   // ~93
#define U_TURN_FRONT_GAP_MM (SPIN_R_REAR_MM - AXLE_TO_FRONT_MM + SAFETY_MM)  // rear swings through the front, ~49
#define BACK_TO_WALL_CARRY_MM ((AXLE_TO_REAR_MM - HALF_CORRIDOR_MM) + DECISION_BACK_MM)  // back against a wall

// === WALL DETECTION ===
#define WALL_THRESHOLD_MM        130                    // side wall present in this cell
#define FRONT_WALL_THRESHOLD_MM  (FRONT_GAP_MM + 70)    // front wall present in this cell (next one is 180 further)
#define FRONT_LOOK_MM            (FRONT_GAP_MM + 110)   // start braking for a front wall from here
#define SIDE_FOLLOW_MM           (SIDE_GAP_MM + 35)     // side wall close enough to centre on

// === SIDE-WALL EDGE CORRECTION ===
#define EDGE_BEAM_MM        5.0f   // CAL: how far past a wall end the side reading still sees the wall
#define EDGE_MIN_RUN_MM    25.0f   // wall/gap must last this long before its edge is trusted
#define EDGE_MAX_CORR_MM   30.0f   // ignore edges that disagree with the encoders by more than this

// === DRIVING ===
#define TICKS_PER_CELL      290
#define TICKS_PER_MM        (TICKS_PER_CELL / CELL_MM)
#define DRIVE_PWM           160
#define SPEED_RUN_PWM       220
#define MIN_DRIVE_PWM        80
#define ACCEL_PWM_PER_MM    2.0f   // ramp up over the first ~40 mm
#define DECEL_PWM_PER_MM    1.5f   // ramp down over the last ~50-90 mm
#define STALL_MS            500

// === STEERING (gyro holds heading, side walls nudge the heading target) ===
#define KP                  5.0f
#define KD                  0.2f
#define WALL_KP             0.5f   // degrees of heading nudge per mm of lateral error
#define MAX_WALL_NUDGE_DEG  8.0f
#define TOO_CLOSE_MM        15     // side gap that triggers a hard steer away and a slow-down
#define CLOSE_NUDGE_DEG     15.0f
#define CLOSE_PWM           110
#define FRONT_EMERGENCY_MM  20     // stop at once if the front gets this close

// === GYRO CORRECTION FROM SIDE WALLS ===
#define WALL_FIT_SPAN_MM    60.0f  // fit the wall angle over this much travel
#define WALL_FIT_MIN_N      6      // readings needed in one fit
#define HEADING_TRIM_GAIN   0.3f   // share of the measured gyro error removed per fit
#define MAX_TRIM_STEP_DEG   1.0f

// === TURNS ===
#define KP_ARC              3.0f
#define ARC_MIN_PWM         90
#define ARC_MAX_PWM         170
#define ARC_KT              4.0f   // extra inner-wheel PWM per tick it lags behind the arc
#define ARC_MIN_MARGIN_MM   6.0f   // shuffle toward the turn if the outer wall is closer than this
#define ARC_TARGET_MARGIN_MM 9.0f
#define KP_SPIN             2.5f
#define SPIN_MIN_PWM        60
#define SPIN_MAX_PWM        100
#define TURN_TOL_DEG        1.5f
#define TURN_SETTLE_DPS     20.0f
#define TURN_TIMEOUT_MS     3000
#define ALIGN_TOL_MM        2
#define SHIFT_TOL_MM        1.5f   // don't shuffle for less than this
#define SHIFT_ANGLE_DEG     12.0f  // heading used for the sideways shuffle
#define MAX_SHIFT_MM        15.0f

#define GYRO_SCALE          65.5f  // 500 deg/s range; bigger number = turns further
#define GYRO_CALIB_SAMPLES  200
#define MAX_FAILS           3      // consecutive failed moves before giving up
#define LONG_PRESS_MS       1000
#define BUTTON_HOLD_MS      150    // button must be held this long during a search (noise can't)
#define EEPROM_MAGIC        0xC9   // marks a saved map in EEPROM
#define WALL_SENSE_MS       300    // how long the ToFs are averaged to read the walls in a cell
#define SEARCH_PASSES       3      // explored everything without finding the room: clear, look again

#define DRIVE_STALL 0
#define DRIVE_DONE  1
#define DRIVE_WALL  2
#define DRIVE_CRASH 3   // front closer than FRONT_EMERGENCY_MM

#define TURN_OK      0
#define TURN_FAILED  1
#define TURN_BLOCKED 2  // the side ToF sees a wall where it wanted to turn

#define ST_WAIT         0
#define ST_SEARCH       1   // exploring until it finds the goal room
#define ST_HOME         2   // driving home (still reading walls)
#define ST_FAST         3   // fast run to the goal room

// --- GLOBALS ---
volatile long encoderLeftTicks = 0;
volatile long encoderRightTicks = 0;

float gyroZOffset = 0;
float gyroRate = 0;               // deg/s, + = counter-clockwise (left)
float absoluteHeading = 0.0f;
float targetHeading = 0.0f;       // always a multiple of 90 except during a shuffle
unsigned long lastGyroMicros = 0;
int16_t lastGyroRaw = 0;

int tofL = 999, tofF = 999, tofR = 999;
uint16_t badL = 0, badF = 0, badR = 0;  // failed readings thrown away (see readToF)
uint8_t frontCloseCount = 0;      // front readings in a row under FRONT_EMERGENCY_MM
float centreGap = SIDE_GAP_MM;    // learned side reading when centred, used when only one wall
uint8_t seqL = 0, seqR = 0;       // bump on every new side reading
long accL = 0, accF = 0, accR = 0;  // reading sums for averageAll()
uint16_t cntL = 0, cntF = 0, cntR = 0;

float carryMm = 0;                // axle position past this cell's decision point
float lastOvershootMm = 0;
float lastMovedMm = 0;            // distance of the last driveStraight, with edge corrections
int lastDriveResult = 0;          // DRIVE_... of the last moveCells
float odoMm = 0;                  // distance driven straight since the last turn
bool backToWall = false;          // rear is against a wall: never reverse
int failCount = 0;

int runState = ST_WAIT;
bool abortExploration = false;
unsigned long buttonDownSince = 0; // when the button went down during a search (0 = up)
uint8_t searchPass = 0;           // times it has explored everything without finding the room

VL53L0X sensorLeft;
VL53L0X sensorFront;
VL53L0X sensorRight;

// These two structs carry their own functions (member functions), so no free function takes
// them as a parameter: the Arduino IDE writes its own declarations of all free functions near
// the top of the sketch, above the structs, and would then not know these types.

// --- SIDE-WALL EDGE CORRECTION ---
// Posts sit on every cell boundary. When a side reading changes between wall and gap, the side
// sensor is at a known spot (a post edge), which fixes the distance driven so far.
struct SideEdge {
  bool known, wall, pending;
  float since, pendingAt;
  uint8_t pendingCount;

  void reset() { known = false; pending = false; }

  // corrMm: running correction added to the encoder distance. startAxleMm: axle position at the
  // start of the drive, measured from the centre of the cell the drive started in.
  void update(int tof, float done, float &corrMm, float startAxleMm, char side) {
    bool w = tof < SIDE_FOLLOW_MM;
    if (!known) { known = true; wall = w; since = done; pending = false; return; }
    if (w == wall) { pending = false; return; }
    if (!pending) { pending = true; pendingAt = done; pendingCount = 1; return; }
    if (++pendingCount < 2) return;  // two readings in a row: not noise

    bool longEnough = (pendingAt - since) >= EDGE_MIN_RUN_MM;
    wall = w; since = pendingAt; pending = false;
    if (!longEnough) return;

    // Boundary posts are centred on 180k + 90. A wall starts at the near face of a post and
    // ends at its far face; the beam width shifts both slightly.
    float edgeRel = w ? -(POST_HALF_MM + EDGE_BEAM_MM) : (POST_HALF_MM + EDGE_BEAM_MM);
    float sensorY = startAxleMm + corrMm + pendingAt + SIDE_TOF_AHEAD_MM;
    float k = round((sensorY - CELL_MM / 2 - edgeRel) / CELL_MM);
    float trueY = k * CELL_MM + CELL_MM / 2 + edgeRel;
    float c = trueY - sensorY;
    if (fabs(c) < EDGE_MAX_CORR_MM) corrMm += c;
  }
};

// --- GYRO CORRECTION FROM SIDE WALLS ---
// While a side wall is present, the reading against distance driven is a straight line whose
// slope is the robot's real angle to the corridor. Comparing that with the gyro's angle over
// the same stretch shows how far the gyro has drifted (turn scale error, bias drift), and the
// gyro is pulled back a little each time. Fits carry on across stops until the robot turns.
struct WallFit {
  uint8_t n;
  float x0, sx, sy, sxx, sxy, sPsi;

  void reset() { n = 0; }

  // x: distance driven straight (odoMm + this drive). sideSign: +1 = left wall, -1 = right wall.
  void update(int tof, float x, int sideSign) {
    if (tof >= SIDE_FOLLOW_MM) { n = 0; return; }  // no wall: start again
    if (n == 0) { x0 = x; sx = sy = sxx = sxy = sPsi = 0; }
    x -= x0;
    n++;
    sx += x; sy += tof; sxx += x * x; sxy += x * tof;
    sPsi += absoluteHeading - targetHeading;
    if (n < WALL_FIT_MIN_N || x < WALL_FIT_SPAN_MM) return;

    float den = n * sxx - sx * sx;
    if (den > 1) {
      float slope = (n * sxy - sx * sy) / den;
      // Turned toward the left wall = left reading shrinks and right reading grows.
      float wallPsi = -sideSign * asin(constrain(slope, -0.3f, 0.3f)) * RAD_TO_DEG;
      float gyroPsi = sPsi / n;
      float err = gyroPsi - wallPsi;
      if (fabs(err) < 8) {  // larger means a bad fit (post, open cell), not drift
        float step = constrain(err * HEADING_TRIM_GAIN, -MAX_TRIM_STEP_DEG, MAX_TRIM_STEP_DEG);
        absoluteHeading -= step;
      }
    }
    n = 0;
  }
};
WallFit fitL, fitR;
void resetFits() { fitL.reset(); fitR.reset(); }

// --- MAZE (no hardware calls) ---
// Tested on a PC by tests/micromouse_29_sim.sh on random mazes.
// Directions: 0 = north (+y), 1 = east (+x), 2 = south, 3 = west. Right of d is (d + 1) & 3.
const int8_t DX[4] = { 0, 1, 0, -1 };
const int8_t DY[4] = { 1, 0, -1, 0 };

// The map, half a byte per cell. A cell keeps only its north and east wall (its south wall is
// the north wall of the cell below it, its west wall the east wall of the cell to its left):
// bit 0 = wall north, bit 1 = wall east, bit 2 = north has been seen, bit 3 = east seen.
#define CELLS (MAP_SIZE * MAP_SIZE)
uint8_t wallMap[(CELLS + 1) / 2];
uint8_t visitedMap[(CELLS + 7) / 8];  // 1 bit per cell: the robot has read the walls there
uint8_t from[MAP_SIZE][MAP_SIZE];     // route search: the direction each cell was reached in
int8_t posX = START_X, posY = START_Y;
uint8_t facing = 0;
int8_t goalX = -1, goalY = -1;        // lower-left cell of the goal room; -1 = not found yet
int8_t candX = -1, candY = -1;        // a room that looks open, being checked; -1 = none
uint8_t routeDir = 0, routeRun = 0;   // found by findRoute: first direction, cells straight on

bool inMap(int x, int y) { return x >= 0 && y >= 0 && x < MAP_SIZE && y < MAP_SIZE; }

uint8_t nibble(int i) { return (wallMap[i >> 1] >> ((i & 1) * 4)) & 15; }
void setNibble(int i, uint8_t v) {
  uint8_t s = (i & 1) * 4;
  wallMap[i >> 1] = (wallMap[i >> 1] & ~(15 << s)) | (v << s);
}

// Where the wall between (x, y) and its neighbour d is kept: cell i, bit b (0 = N, 1 = E).
// False = that wall is the edge of the map.
bool edgeRef(int x, int y, uint8_t d, int &i, uint8_t &b) {
  int nx = x + DX[d], ny = y + DY[d];
  if (!inMap(x, y) || !inMap(nx, ny)) return false;
  if (d >= 2) { x = nx; y = ny; d -= 2; }
  i = x * MAP_SIZE + y;
  b = d;
  return true;
}
bool wallAt(int x, int y, uint8_t d) {
  int i; uint8_t b;
  return !edgeRef(x, y, d, i, b) || (nibble(i) & (1 << b));
}
bool seenAt(int x, int y, uint8_t d) {
  int i; uint8_t b;
  return !edgeRef(x, y, d, i, b) || (nibble(i) & (4 << b));
}
// Writes one wall (both cells share it).
void setWall(int x, int y, uint8_t d, bool present) {
  int i; uint8_t b;
  if (!edgeRef(x, y, d, i, b)) return;
  uint8_t v = nibble(i) | (4 << b);
  if (present) v |= 1 << b; else v &= ~(1 << b);
  setNibble(i, v);
}
bool visited(int x, int y) { int i = x * MAP_SIZE + y; return visitedMap[i >> 3] & (1 << (i & 7)); }
void markVisited(int x, int y) { int i = x * MAP_SIZE + y; visitedMap[i >> 3] |= 1 << (i & 7); }

void initMaze() {
  memset(wallMap, 0, sizeof(wallMap));
  memset(visitedMap, 0, sizeof(visitedMap));
  goalX = goalY = candX = candY = -1;
}

// The goal room: 2 x 2 cells, lower-left (x, y), with the 4 walls inside it seen and open.
bool seenOpen(int x, int y, uint8_t d) { return seenAt(x, y, d) && !wallAt(x, y, d); }
bool openRoom(int x, int y) {
  if (!inMap(x, y) || !inMap(x + 1, y + 1)) return false;
  return seenOpen(x, y, 0) && seenOpen(x, y, 1) && seenOpen(x + 1, y, 0) && seenOpen(x, y + 1, 1);
}
bool inRoom(int x, int y, int rx, int ry) { return rx >= 0 && x >= rx && x < rx + 2 && y >= ry && y < ry + 2; }
bool isGoal(int x, int y) { return inRoom(x, y, goalX, goalY); }

// After reading the walls of a cell: looks for the goal room. A room that looks open becomes a
// candidate; it counts as the goal once the robot has been in all 4 of its cells (so every
// wall inside it has been read from both sides) and it is still open. True = just found.
bool checkGoal() {
  if (goalX >= 0) return false;
  if (candX >= 0 && !openRoom(candX, candY)) candX = candY = -1;  // a wall after all
  for (int dx = -1; dx <= 0 && candX < 0; dx++)
    for (int dy = -1; dy <= 0 && candX < 0; dy++)
      if (openRoom(posX + dx, posY + dy)) { candX = posX + dx; candY = posY + dy; }
  if (candX < 0) return false;
  for (int i = 0; i < 4; i++)
    if (!visited(candX + (i & 1), candY + (i >> 1))) return false;
  goalX = candX; goalY = candY;
  return true;
}

// The three walls read here, as bits: 1 = left, 2 = front, 4 = right. Written down, and the
// cell is marked as visited. A reading of "open" doesn't remove a wall read before: only
// driving through it does (see moved). So a fake goal room needs the same wall misread every
// time; a wall misread the other way is caught by the ToFs when it tries to drive there.
void recordWalls(uint8_t walls) {
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t d = (facing + 3 + i) & 3;
    bool w = walls & (1 << i);
    if (w || !wallAt(posX, posY, d)) setWall(posX, posY, d, w);
  }
  markVisited(posX, posY);
}

// Drove k cells along d: those walls are open for sure.
void moved(uint8_t d, int k) {
  facing = d;
  for (int j = 0; j < k; j++) { setWall(posX, posY, d, false); posX += DX[d]; posY += DY[d]; }
}

#define TO_UNVISITED 0   // the nearest cell not visited yet (in the candidate room if there is one)
#define TO_START     1
#define TO_GOAL      2
bool isTarget(int x, int y, uint8_t to) {
  if (to == TO_UNVISITED) return !visited(x, y) && (candX < 0 || inRoom(x, y, candX, candY));
  if (to == TO_START) return x == START_X && y == START_Y;
  return isGoal(x, y);
}

// Shortest way on the map from the robot's cell to the nearest target (breadth-first search;
// straight on is tried first, so of equal routes the one with fewer turns tends to win).
// knownOnly: only walls it has seen open count as open (fast run); otherwise an unseen wall
// counts as open. Sets routeDir (first direction) and routeRun (cells straight on from here).
// False = no way there.
bool findRoute(uint8_t to, bool knownOnly) {
  static uint16_t queue[64];  // circular; the search front is never longer than ~40 on 19 x 19
  uint8_t head = 0, tail = 0;
  static const uint8_t order[4] = { 0, 1, 3, 2 };
  memset(from, 255, sizeof(from));
  from[posX][posY] = 4;
  queue[tail++] = posX * MAP_SIZE + posY;
  int gx = -1, gy = -1;
  while (head != tail) {
    uint16_t c = queue[head]; head = (head + 1) & 63;
    int x = c / MAP_SIZE, y = c % MAP_SIZE;
    if (isTarget(x, y, to) && from[x][y] != 4) { gx = x; gy = y; break; }
    for (uint8_t i = 0; i < 4; i++) {
      uint8_t d = (facing + order[i]) & 3;
      if (wallAt(x, y, d) || (knownOnly && !seenAt(x, y, d))) continue;
      int nx = x + DX[d], ny = y + DY[d];
      if (from[nx][ny] != 255) continue;
      from[nx][ny] = d;
      queue[tail] = nx * MAP_SIZE + ny; tail = (tail + 1) & 63;
      if (tail == head) return false;  // (can't happen) queue full
    }
  }
  if (gx < 0) return false;
  // Walk back from the target: the last step found is the first one to drive.
  routeDir = 255; routeRun = 0;
  for (int x = gx, y = gy; from[x][y] != 4; ) {
    uint8_t d = from[x][y];
    if (d == routeDir) routeRun++; else { routeDir = d; routeRun = 1; }
    x -= DX[d]; y -= DY[d];
  }
  return true;
}

// Turn needed to go from facing `from` to facing `to`.
char relMove(uint8_t from, uint8_t to) {
  switch ((to - from) & 3) {
    case 0: return 'S';
    case 1: return 'R';
    case 2: return 'U';
    default: return 'L';
  }
}
// --- END MAZE ---

// --- INTERRUPT SERVICE ROUTINES ---
void leftEncoderISR() {
  if (digitalRead(ENCODER_LEFT_C2) == HIGH) encoderLeftTicks++; else encoderLeftTicks--;
}
void rightEncoderISR() {
  if (digitalRead(ENCODER_RIGHT_C2) == HIGH) encoderRightTicks++; else encoderRightTicks--;
}

// 4-byte counters are updated by interrupts, so copy them with interrupts off.
void resetTicks() {
  noInterrupts(); encoderLeftTicks = 0; encoderRightTicks = 0; interrupts();
}
void readTicks(long &l, long &r) {
  noInterrupts(); l = labs(encoderLeftTicks); r = labs(encoderRightTicks); interrupts();
}
float travelledMm() {
  long l, r;
  readTicks(l, r);
  return (l + r) / 2.0f / TICKS_PER_MM;  // abs of each: works whichever way the encoders count
}

// --- HARDWARE INIT FUNCTIONS ---
void initToFSensors() {
  pinMode(TOF_XSHUT_LEFT, OUTPUT); digitalWrite(TOF_XSHUT_LEFT, LOW);
  pinMode(TOF_XSHUT_FRONT, OUTPUT); digitalWrite(TOF_XSHUT_FRONT, LOW);
  pinMode(TOF_XSHUT_RIGHT, OUTPUT); digitalWrite(TOF_XSHUT_RIGHT, LOW);
  delay(100);

  digitalWrite(TOF_XSHUT_LEFT, HIGH); delay(50);
  sensorLeft.setTimeout(500);
  if (sensorLeft.init()) {
    sensorLeft.setAddress(0x30);
    sensorLeft.setMeasurementTimingBudget(20000);
    sensorLeft.startContinuous();
  } else Serial.println(F("LEFT ToF NOT FOUND"));

  digitalWrite(TOF_XSHUT_FRONT, HIGH); delay(50);
  sensorFront.setTimeout(500);
  if (sensorFront.init()) {
    sensorFront.setAddress(0x31);
    sensorFront.setMeasurementTimingBudget(20000);
    sensorFront.startContinuous();
  } else Serial.println(F("FRONT ToF NOT FOUND"));

  digitalWrite(TOF_XSHUT_RIGHT, HIGH); delay(50);
  sensorRight.setTimeout(500);
  if (sensorRight.init()) {
    sensorRight.setAddress(0x32);
    sensorRight.setMeasurementTimingBudget(20000);
    sensorRight.startContinuous();
  } else Serial.println(F("RIGHT ToF NOT FOUND"));
}

void initMPU6050() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(true); // DLPF ~42Hz
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(true); // 500 deg/s
}

// Returns the previous value if the I2C read fails instead of returning garbage.
int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return lastGyroRaw;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true) != 2) return lastGyroRaw;
  uint8_t h = Wire.read();
  uint8_t l = Wire.read();
  lastGyroRaw = (int16_t)((h << 8) | l);
  return lastGyroRaw;
}

void calibrateGyro() {
  Serial.println(F("Calibrating Gyro... KEEP STILL!"));
  long sum = 0;
  for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
    sum += readGyroZRaw(); delay(3);
  }
  gyroZOffset = (float)sum / GYRO_CALIB_SAMPLES;
  absoluteHeading = 0.0f;
  targetHeading = 0.0f;
  gyroRate = 0.0f;
  lastGyroMicros = micros();
}

// Re-measures the gyro bias without touching the heading (robot must be still).
void recalGyroBias() {
  long sum = 0;
  for (int i = 0; i < 100; i++) { sum += readGyroZRaw(); delay(2); }
  gyroZOffset = (float)sum / 100;
  gyroRate = 0.0f;
  lastGyroMicros = micros();
}

void updateGyro() {
  unsigned long now = micros();
  float dt = (now - lastGyroMicros) / 1000000.0f;
  lastGyroMicros = now;
  gyroRate = (readGyroZRaw() - gyroZOffset) / GYRO_SCALE;
  absoluteHeading += gyroRate * dt;
}

// Non-blocking: only reads a sensor that has a new measurement ready. The sensor also reports
// whether the measurement worked (range status 11 = valid); a failed one, or a 0, is thrown
// away and counted in bad (the last good value stays). Nothing in range = 999.
// Returns: 0 = nothing new, 1 = new good reading, 2 = new failed reading.
uint8_t readToF(VL53L0X &sensor, int bias, int &tof, long &acc, uint16_t &cnt, uint16_t &bad) {
  if (!(sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07)) return 0;
  uint8_t status = (sensor.readReg(VL53L0X::RESULT_RANGE_STATUS) >> 3) & 0x0F;
  uint16_t r = sensor.readRangeContinuousMillimeters();
  if (r > 8000 || status == 4) { tof = 999; return 1; }  // 4 = nothing in range (open)
  if (status != 11 || r == 0) { bad++; return 2; }        // failed measurement
  tof = (int)r - bias;
  acc += tof; cnt++;
  return 1;
}

void updateToF() {
  if (readToF(sensorLeft, LEFT_BIAS_MM, tofL, accL, cntL, badL)) seqL++;
  if (readToF(sensorFront, FRONT_BIAS_MM, tofF, accF, cntF, badF) == 1)
    frontCloseCount = (tofF < FRONT_EMERGENCY_MM) ? (frontCloseCount < 3 ? frontCloseCount + 1 : 3) : 0;
  if (readToF(sensorRight, RIGHT_BIAS_MM, tofR, accR, cntR, badR)) seqR++;
}

// Something really close in front: two readings in a row, so one bad reading can't stop it.
bool frontCrash() { return frontCloseCount >= 2; }

// Called in every loop: keeps the gyro integrating and the ToF values fresh.
// The button only sets a flag (held down BUTTON_HOLD_MS without a break, so motor noise on the
// wire can't); the current cell always finishes first.
void sense() {
  updateGyro();
  updateToF();
  if (runState == ST_SEARCH && digitalRead(START_BUTTON) == LOW) {
    if (!buttonDownSince) buttonDownSince = millis() | 1;
    else if (millis() - buttonDownSince > BUTTON_HOLD_MS) abortExploration = true;
  } else {
    buttonDownSince = 0;
  }
}

void senseFor(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) { sense(); delay(2); }
}

// Averages all three good readings over ~150 ms while standing still. 999 = nothing in range
// (or no good reading at all).
void averageOver(unsigned long ms, float &l, float &f, float &r) {
  accL = accF = accR = 0; cntL = cntF = cntR = 0;
  senseFor(ms);
  l = cntL ? (float)accL / cntL : 999;
  f = cntF ? (float)accF / cntF : 999;
  r = cntR ? (float)accR / cntR : 999;
}
void averageAll(float &l, float &f, float &r) { averageOver(150, l, f, r); }

// --- MOTOR CONTROL FUNCTIONS ---
void setupMotors() {
  pinMode(MOTOR_STBY, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT); pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT); pinMode(MOTOR_B_IN1, OUTPUT); pinMode(MOTOR_B_IN2, OUTPUT);
  digitalWrite(MOTOR_STBY, LOW);
}

void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(MOTOR_STBY, HIGH);
  if (leftSpeed >= 0) { digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW); }
  else { digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, HIGH); leftSpeed = -leftSpeed; }
  analogWrite(MOTOR_A_PWM, constrain(leftSpeed, 0, 255));

  if (rightSpeed >= 0) { digitalWrite(MOTOR_B_IN1, HIGH); digitalWrite(MOTOR_B_IN2, LOW); }
  else { digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, HIGH); rightSpeed = -rightSpeed; }
  analogWrite(MOTOR_B_PWM, constrain(rightSpeed, 0, 255));
}

void stopMotors() { setMotors(0, 0); }  // PWM 0 on the TB6612 = short brake

// --- FAILURE HANDLING ---
// One bad move is not fatal: stop, count it, carry on. Give up only after MAX_FAILS in a row.
void noteResult(bool ok) {
  if (ok) { failCount = 0; return; }
  stopMotors();
  failCount++;
  Serial.print(F("Move failed (")); Serial.print(failCount); Serial.println(F(" in a row)"));
  if (failCount >= MAX_FAILS) {
    Serial.println(F("TOO MANY FAILURES - put the robot in the start cell and press START."));
    digitalWrite(STATUS_LED, HIGH);
    runState = ST_WAIT;
    failCount = 0;
  }
}

// --- STEERING ---
// Gyro holds targetHeading; side walls nudge that target toward the corridor centre.
float headingCorrection(bool useWalls) {
  float desired = targetHeading;
  if (useWalls) {
    bool hasL = tofL < SIDE_FOLLOW_MM;
    bool hasR = tofR < SIDE_FOLLOW_MM;
    float lateralErr = 0;                          // > 0: robot is right of centre
    if (hasL && hasR) {
      lateralErr = (tofL - tofR) / 2.0f;
      // (L + R) / 2 is the centred reading wherever the robot is; learn it so one-wall
      // stretches aim at the same line and the robot doesn't jump when a wall ends.
      centreGap += 0.02f * ((tofL + tofR) / 2.0f - centreGap);
      centreGap = constrain(centreGap, SIDE_GAP_MM - 10, SIDE_GAP_MM + 10);
    }
    else if (hasL) lateralErr = tofL - centreGap;
    else if (hasR) lateralErr = centreGap - tofR;
    // About to touch a wall: steer away twice as hard with a bigger limit.
    bool tooClose = (hasL && tofL < TOO_CLOSE_MM) || (hasR && tofR < TOO_CLOSE_MM);
    float limit = tooClose ? CLOSE_NUDGE_DEG : MAX_WALL_NUDGE_DEG;
    desired += constrain(lateralErr * WALL_KP * (tooClose ? 2 : 1), -limit, limit);
  }
  return KP * (desired - absoluteHeading) - KD * gyroRate;
}

// --- MOVEMENT BEHAVIOURS ---
// Drives distMm (negative = reverse) holding targetHeading.
//  center:      steer toward the corridor centre using the side walls
//  stopAtWall:  also stop at the decision point of a front wall
//  startAxleMm: if not NAN, correct the distance at side-wall edges (see SideEdge)
// Returns DRIVE_STALL, DRIVE_DONE, DRIVE_WALL or DRIVE_CRASH.
int driveStraight(float distMm, int maxPwm, bool center, bool stopAtWall, float startAxleMm) {
  bool reverse = distMm < 0;
  float goal = fabs(distMm);
  bool useEdges = !reverse && !isnan(startAxleMm);
  bool useFit = center && !reverse;
  if (!useFit) resetFits();
  int result = DRIVE_DONE;
  float corr = 0;
  SideEdge edgeL, edgeR;
  edgeL.reset(); edgeR.reset();
  uint8_t lastSeqL = seqL, lastSeqR = seqR;
  resetTicks();
  float lastDone = 0;
  unsigned long lastProgress = millis(), lastLoop = 0;

  while (true) {
    sense();
    float done = travelledMm();
    if (seqL != lastSeqL) {
      lastSeqL = seqL;
      if (useEdges) edgeL.update(tofL, done, corr, startAxleMm, 'L');
      if (useFit) fitL.update(tofL, odoMm + done, +1);
    }
    if (seqR != lastSeqR) {
      lastSeqR = seqR;
      if (useEdges) edgeR.update(tofR, done, corr, startAxleMm, 'R');
      if (useFit) fitR.update(tofR, odoMm + done, -1);
    }
    if (!reverse && frontCrash()) {
      Serial.println(F("FRONT TOO CLOSE - emergency stop"));
      result = DRIVE_CRASH;
      break;
    }
    float remaining = goal - done - corr;
    bool wallLimited = false;
    if (stopAtWall && !reverse && tofF < FRONT_LOOK_MM) {
      float toWall = tofF - FRONT_GAP_MM;
      if (toWall < remaining) { remaining = toWall; wallLimited = true; }
    }
    if (remaining <= 0) { result = wallLimited ? DRIVE_WALL : DRIVE_DONE; break; }

    if (done > lastDone + 0.5f) { lastDone = done; lastProgress = millis(); }
    else if (millis() - lastProgress > STALL_MS) {
      Serial.println(F("STALL: wheels not turning"));
      result = DRIVE_STALL;
      break;
    }

    if (millis() - lastLoop >= 5) {
      lastLoop = millis();
      float up = MIN_DRIVE_PWM + ACCEL_PWM_PER_MM * done;
      float down = MIN_DRIVE_PWM + DECEL_PWM_PER_MM * remaining;
      float pwm = up < down ? up : down;
      if (pwm > maxPwm) pwm = maxPwm;
      if (center && (tofL < TOO_CLOSE_MM || tofR < TOO_CLOSE_MM) && pwm > CLOSE_PWM) pwm = CLOSE_PWM;
      if (reverse) pwm = -pwm;
      float c = headingCorrection(center && !reverse);  // wall nudges steer the wrong way in reverse
      setMotors((int)(pwm - c), (int)(pwm + c));
    }
  }
  stopMotors();
  senseFor(80);
  float moved = travelledMm();
  if (useFit) odoMm += moved;
  lastMovedMm = moved + corr;
  lastOvershootMm = lastMovedMm - goal;
  return result;
}

// Brings the front of the robot to exactly gapMm from the front wall, holding heading.
// Never steers sideways, so it keeps any sideways shuffle.
void alignToFrontWall(float gapMm) {
  sense();
  if (tofF < FRONT_LOOK_MM + 60 && fabs(tofF - gapMm) > 20) {
    driveStraight(tofF - gapMm - (tofF > gapMm ? 10 : -10), 110, false, false, NAN);
  }
  unsigned long start = millis();
  while (millis() - start < 1000) {
    sense();
    if (tofF >= FRONT_LOOK_MM + 60) break;
    float err = tofF - gapMm;
    if (fabs(err) <= ALIGN_TOL_MM) break;
    float v = constrain(err * 4.0f, -60.0f, 60.0f);
    if (fabs(v) < 45) v = (v > 0) ? 45 : -45;  // enough to overcome friction
    float c = KP * (targetHeading - absoluteHeading) - KD * gyroRate;
    setMotors((int)(v - c), (int)(v + c));
    delay(3);
  }
  stopMotors();
  senseFor(60);
}

// Spins in place to targetHeading. Only safe for small angles, or after spinSafely() placed
// the robot.
bool rotateInPlace() {
  resetFits();
  unsigned long start = millis();
  while (true) {
    sense();
    float err = targetHeading - absoluteHeading;
    if (fabs(err) < TURN_TOL_DEG && fabs(gyroRate) < TURN_SETTLE_DPS) break;
    if (millis() - start > TURN_TIMEOUT_MS) {
      stopMotors();
      Serial.println(F("TURN TIMEOUT (hit a wall?)"));
      senseFor(80);
      return false;
    }
    int pwm = constrain((int)(KP_SPIN * fabs(err)), SPIN_MIN_PWM, SPIN_MAX_PWM);
    if (fabs(err) < TURN_TOL_DEG) pwm = 0;  // in tolerance: brake and let it settle
    int s = (err > 0) ? pwm : -pwm;         // + = rotate counter-clockwise
    setMotors(-s, s);
    delay(2);
  }
  stopMotors();
  senseFor(80);
  return true;
}

bool spinTurn(float angleDeg) {
  targetHeading += angleDeg;
  return rotateInPlace();
}

// 90° arc of TURN_RADIUS_MM. dir: +1 = left, -1 = right.
// The gyro drives the outer wheel; the inner wheel is held to ARC_RATIO of the outer wheel's
// distance with the encoders. Stops if the front ToF sees something very close.
bool arcTurn(int dir) {
  resetFits();
  odoMm = 0;
  targetHeading += 90.0f * dir;
  resetTicks();
  unsigned long start = millis();

  while (true) {
    sense();
    float err = targetHeading - absoluteHeading;
    if (fabs(err) < TURN_TOL_DEG && fabs(gyroRate) < TURN_SETTLE_DPS) break;
    if (millis() - start > TURN_TIMEOUT_MS) {
      stopMotors();
      Serial.println(F("TURN TIMEOUT (hit a wall?)"));
      senseFor(80);
      return false;
    }
    if (frontCrash()) {
      stopMotors();
      Serial.println(F("FRONT TOO CLOSE in turn - stopped"));
      senseFor(80);
      return false;
    }

    if (fabs(err) < TURN_TOL_DEG) {
      stopMotors();                          // in tolerance: brake and let it settle
    } else if (err * dir < 0) {
      int s = (err > 0) ? SPIN_MIN_PWM : -SPIN_MIN_PWM;  // overshot: nudge back in place
      setMotors(-s, s);
    } else {
      long l, r;
      readTicks(l, r);
      long outerTicks = (dir > 0) ? r : l;
      long innerTicks = (dir > 0) ? l : r;
      float outer = constrain(KP_ARC * fabs(err), (float)ARC_MIN_PWM, (float)ARC_MAX_PWM);
      float inner = outer * ARC_RATIO + ARC_KT * (ARC_RATIO * outerTicks - innerTicks);
      inner = constrain(inner, 0.0f, outer);
      if (dir > 0) setMotors((int)inner, (int)outer);
      else         setMotors((int)outer, (int)inner);
    }
    delay(2);
  }
  stopMotors();
  senseFor(80);
  return true;
}

// Moves the robot sideways: tilt, reverse, tilt back, then drive forward again.
// mm > 0 = move left. Ends at frontGapMm from the front wall if there is one.
void shiftSideways(float mm, float frontGapMm) {
  mm = constrain(mm, -MAX_SHIFT_MM, MAX_SHIFT_MM);
  float base = targetHeading;
  float back = fabs(mm) / sin(SHIFT_ANGLE_DEG * DEG_TO_RAD);
  spinTurn(mm > 0 ? -SHIFT_ANGLE_DEG : SHIFT_ANGLE_DEG);  // point the tail at the side we want
  driveStraight(-back, 90, false, false, NAN);
  targetHeading = base;
  rotateInPlace();
  sense();
  if (tofF < FRONT_WALL_THRESHOLD_MM) alignToFrontWall(frontGapMm);
  else driveStraight(back * cos(SHIFT_ANGLE_DEG * DEG_TO_RAD), 90, false, false, NAN);
}

// Spins in place by angleDeg (±90 or 180). The rear corners swing 93 mm and the corridor is
// 84 mm from centre to wall, so first pull up to the front wall and shuffle sideways toward the
// side the front corners sweep, giving the long rear the room. For 180° the direction needing
// the smaller shuffle is used.
bool spinSafely(float angleDeg) {
  sense();
  if (tofF < FRONT_WALL_THRESHOLD_MM) alignToFrontWall(U_TURN_FRONT_GAP_MM);

  float l, f, r;
  averageAll(l, f, r);
  bool hasL = l < WALL_THRESHOLD_MM, hasR = r < WALL_THRESHOLD_MM;
  float leftRoom = l + HALF_WIDTH_MM, rightRoom = r + HALF_WIDTH_MM;  // axle -> wall
  float width = (hasL && hasR) ? l + r + ROBOT_WIDTH_MM : 2 * HALF_CORRIDOR_MM;
  // Clockwise: rear corners sweep the left wall, front corners the right wall.
  float cwLeft  = (width + SPIN_R_REAR_MM - SPIN_R_FRONT_MM) / 2;  // balanced axle -> left wall
  float ccwLeft = (width - SPIN_R_REAR_MM + SPIN_R_FRONT_MM) / 2;

  bool cw;
  if (fabs(angleDeg) > 135) {
    if (hasL && hasR) cw = fabs(leftRoom - cwLeft) <= fabs(leftRoom - ccwLeft);
    else cw = !hasL;  // swing the rear toward the open side
  } else {
    cw = angleDeg < 0;
  }

  float needL = (cw ? SPIN_R_REAR_MM : SPIN_R_FRONT_MM) + SAFETY_MM;
  float needR = (cw ? SPIN_R_FRONT_MM : SPIN_R_REAR_MM) + SAFETY_MM;
  float shiftLeft = 0;  // > 0 = move left
  if (hasL && hasR)            shiftLeft = leftRoom - (cw ? cwLeft : ccwLeft);
  else if (hasL && leftRoom < needL)  shiftLeft = leftRoom - needL;
  else if (hasR && rightRoom < needR) shiftLeft = needR - rightRoom;
  if (fabs(shiftLeft) > SHIFT_TOL_MM && !backToWall) shiftSideways(shiftLeft, U_TURN_FRONT_GAP_MM);

  odoMm = 0;
  return spinTurn(cw ? -fabs(angleDeg) : fabs(angleDeg));
}

// Backs gently into the wall behind. Pressed flat against it the robot is exactly square to
// the maze and at a known distance, so the gyro heading, the gyro bias and the position along
// the corridor are all reset from it.
void squareOnBackWall() {
  setMotors(-70, -70);
  delay(500);
  setMotors(-45, -45);
  delay(300);
  stopMotors();
  delay(150);
  recalGyroBias();
  absoluteHeading = targetHeading;
  carryMm = BACK_TO_WALL_CARRY_MM;
  backToWall = true;
  odoMm = 0;
  resetFits();
}

// Before an arc: if the robot is too close to the wall the rear corner swings toward,
// shuffle toward the turn side. That shift only moves the end point along the new corridor,
// not sideways, so it costs nothing after the turn.
// Returns the side reading toward the turn: under WALL_THRESHOLD_MM = there is a wall there
// after all (then nothing is done).
float prepareArc(int dir) {
  float l, f, r;
  averageAll(l, f, r);
  float turnSide = (dir > 0) ? l : r;
  if (turnSide < WALL_THRESHOLD_MM || backToWall) return turnSide;
  float outerGap = (dir < 0) ? l : r;  // right turn: rear corner swings toward the left wall
  float innerGap = (dir < 0) ? r : l;
  if (outerGap >= WALL_THRESHOLD_MM) return turnSide;  // no outer wall
  float margin = outerGap + HALF_WIDTH_MM - ARC_OUTER_REACH_MM;
  if (margin >= ARC_MIN_MARGIN_MM) return turnSide;
  float shift = ARC_TARGET_MARGIN_MM - margin;
  if (innerGap < WALL_THRESHOLD_MM) shift = min(shift, innerGap - 12.0f);  // don't hit the inner wall
  if (shift <= SHIFT_TOL_MM) return turnSide;
  shiftSideways(dir < 0 ? -shift : shift, FRONT_GAP_MM);
  return turnSide;
}

// Puts the axle on the decision point before a turn: the front wall if there is one,
// otherwise the encoders.
void settleAtDecisionPoint() {
  sense();
  if (tofF < FRONT_WALL_THRESHOLD_MM) {
    alignToFrontWall(FRONT_GAP_MM);
    carryMm = 0;
  } else if (fabs(carryMm) > 5 && !(backToWall && carryMm > 0)) {
    driveStraight(-carryMm, 90, false, false, NAN);
    carryMm = 0;
  }
}

// Returns TURN_OK, TURN_FAILED, or TURN_BLOCKED (the side ToF sees a wall where the map had
// the opening: no arc, the robot stays where it is).
int executeTurn(char move) {
  if (move == 'S') return TURN_OK;
  bool ok;
  if (move == 'U') {
    sense();
    bool deadEnd = tofF < FRONT_WALL_THRESHOLD_MM;
    if (!deadEnd) settleAtDecisionPoint();
    ok = spinSafely(180.0f);
    if (deadEnd) {
      // Turned U_TURN_FRONT_GAP from the dead-end wall, which is now behind: back into it.
      carryMm = (U_TURN_FRONT_GAP_MM + AXLE_TO_FRONT_MM - HALF_CORRIDOR_MM) + DECISION_BACK_MM;
      if (ok) squareOnBackWall();
    } else {
      carryMm = 2 * DECISION_BACK_MM;  // turned on the decision point
    }
  } else {
    int dir = (move == 'L') ? 1 : -1;
    settleAtDecisionPoint();
    if (prepareArc(dir) < WALL_THRESHOLD_MM) return TURN_BLOCKED;  // side ToF: a wall after all
    ok = arcTurn(dir);
    carryMm = TURN_RADIUS_MM + DECISION_BACK_MM;  // arc ends TURN_RADIUS past the cell centre
    backToWall = false;
  }
  return ok ? TURN_OK : TURN_FAILED;
}

// Drives to the decision point numCells ahead. Returns the cells really driven (fewer if the
// front ToF found a wall the map didn't have, or it got stuck); lastDriveResult says why.
int moveCells(int numCells, int maxPwm) {
  float carry0 = carryMm;
  int r = driveStraight(numCells * CELL_MM - carry0, maxPwm, true, true, carry0 - DECISION_BACK_MM);
  backToWall = false;
  lastDriveResult = r;
  float axle = carry0 + lastMovedMm;  // from the start cell's decision point
  int k = constrain((int)round(axle / CELL_MM), 0, numCells);
  carryMm = (r == DRIVE_WALL) ? 0 : axle - k * CELL_MM;
  return k;
}

void startAlignment() {
  setMotors(-60, -60);
  delay(1500);
  stopMotors();
  delay(400);
  calibrateGyro();  // heading 0 = straight out of the start cell
  carryMm = BACK_TO_WALL_CARRY_MM;
  backToWall = true;
  odoMm = 0;
  resetFits();
  failCount = 0;
}

// --- MAP: WALL SENSING, SAVING, PRINTING ---
// Reads the 3 walls of the current cell from its decision point (ToFs averaged over
// WALL_SENSE_MS) and writes them on the map. (A wall missed here is caught by the ToFs when it
// tries to go there.)
void senseWallsHere() {
  float l, f, r;
  averageOver(WALL_SENSE_MS, l, f, r);
  recordWalls((l < WALL_THRESHOLD_MM) | ((f < FRONT_WALL_THRESHOLD_MM) << 1) | ((r < WALL_THRESHOLD_MM) << 2));
}

void saveMaze() {
  EEPROM.update(0, EEPROM_MAGIC);
  EEPROM.update(1, goalX);
  EEPROM.update(2, goalY);
  for (uint16_t i = 0; i < sizeof(wallMap); i++) EEPROM.update(3 + i, wallMap[i]);
  for (uint16_t i = 0; i < sizeof(visitedMap); i++) EEPROM.update(3 + sizeof(wallMap) + i, visitedMap[i]);
}

bool loadMaze() {
  if (EEPROM.read(0) != EEPROM_MAGIC) return false;
  goalX = (int8_t)EEPROM.read(1);
  goalY = (int8_t)EEPROM.read(2);
  for (uint16_t i = 0; i < sizeof(wallMap); i++) wallMap[i] = EEPROM.read(3 + i);
  for (uint16_t i = 0; i < sizeof(visitedMap); i++) visitedMap[i] = EEPROM.read(3 + sizeof(wallMap) + i);
  return true;
}

// The part of the map it has visited: +---+ / | = wall, . = not seen yet, ? = not visited,
// S = start, G = goal room, ^ > v < = robot.
void printMaze() {
  static const char arrow[4] = { '^', '>', 'v', '<' };
  int x0 = posX, x1 = posX, y0 = posY, y1 = posY;
  for (int x = 0; x < MAP_SIZE; x++)
    for (int y = 0; y < MAP_SIZE; y++)
      if (visited(x, y) || isGoal(x, y)) { x0 = min(x0, x); x1 = max(x1, x); y0 = min(y0, y); y1 = max(y1, y); }
  Serial.println();
  for (int y = y1; y >= y0 - 1; y--) {
    for (int x = x0; x <= x1; x++) {
      Serial.print('+');
      uint8_t d = y >= y0 ? 0 : 2;  // the last line is the south side of the bottom row
      int cy = y >= y0 ? y : y0;
      Serial.print(wallAt(x, cy, d) ? F("---") : seenAt(x, cy, d) ? F("   ") : F(" . "));
    }
    Serial.println('+');
    if (y < y0) break;
    for (int x = x0; x <= x1; x++) {
      Serial.print(wallAt(x, y, 3) ? '|' : seenAt(x, y, 3) ? ' ' : '.');
      Serial.print(' ');
      if (x == posX && y == posY) Serial.print(arrow[facing]);
      else if (isGoal(x, y)) Serial.print('G');
      else if (x == START_X && y == START_Y) Serial.print('S');
      else Serial.print(visited(x, y) ? ' ' : '?');
      Serial.print(' ');
    }
    Serial.println(wallAt(x1, y, 1) ? '|' : seenAt(x1, y, 1) ? ' ' : '.');
  }
  if (goalX >= 0) Serial.println(F("G = goal room found."));
  else Serial.println(F("Goal room not found yet."));
}

// --- RUN CONTROL ---
// Returns 0 = not pressed, 1 = short press, 2 = long press (LED lights once it counts as long).
int readButton() {
  if (digitalRead(START_BUTTON) != LOW) return 0;
  delay(30);
  if (digitalRead(START_BUTTON) != LOW) return 0;
  unsigned long t = millis();
  while (digitalRead(START_BUTTON) == LOW) {
    if (millis() - t > LONG_PRESS_MS) digitalWrite(STATUS_LED, HIGH);
  }
  digitalWrite(STATUS_LED, LOW);
  return (millis() - t > LONG_PRESS_MS) ? 2 : 1;
}

// Robot is in the start cell with its back to the start wall.
void beginRun(int state) {
  delay(500);  // let go of the robot
  startAlignment();
  posX = START_X; posY = START_Y; facing = 0;
  if (state == ST_SEARCH) { initMaze(); searchPass = 0; }  // every search starts with a clean map
  setWall(START_X, START_Y, 2, true);  // the start wall behind
  abortExploration = false;
  frontCloseCount = 0;
  runState = state;
}

// In the start cell after coming home: face north again and square up on the start wall.
void finishAtHome() {
  char m = relMove(facing, 0);
  if (m == 'U') spinSafely(180.0f);
  else if (m == 'R') spinSafely(-90.0f);
  else if (m == 'L') spinSafely(90.0f);
  facing = 0;
  startAlignment();
  saveMaze();
  printMaze();
  Serial.println(F("HOME. Short press = new search, long press = fast run."));
  runState = ST_WAIT;
}

// Turns to d and drives up to n cells; updates the position with the cells really driven.
// False = it didn't get into the next cell. lastDriveResult == DRIVE_WALL then means a wall was
// found there (now on the map): choose again.
bool driveTo(uint8_t d, int n, int pwm) {
  int t = executeTurn(relMove(facing, d));
  if (t == TURN_BLOCKED) {
    setWall(posX, posY, d, true);
    lastDriveResult = DRIVE_WALL;
    return false;
  }
  noteResult(t == TURN_OK);
  facing = d;
  if (runState == ST_WAIT) return false;
  int k = moveCells(n, pwm);
  noteResult(lastDriveResult != DRIVE_STALL && lastDriveResult != DRIVE_CRASH);
  moved(d, k);
  if (k < n && lastDriveResult == DRIVE_WALL) setWall(posX, posY, d, true);  // wall the map didn't have
  return k > 0;
}

void blink5() {
  for (int i = 0; i < 5; i++) { digitalWrite(STATUS_LED, HIGH); delay(80); digitalWrite(STATUS_LED, LOW); delay(80); }
}

// One cell of the search (or of the way home): read the walls, look for the goal room, then
// drive one cell toward the nearest unvisited cell (or home).
void searchStep() {
  senseWallsHere();
  if (checkGoal()) {
    Serial.println(F("\n*** GOAL ROOM FOUND *** heading home"));
    saveMaze();
    printMaze();
    blink5();
    runState = ST_HOME;
  }
  while (runState != ST_WAIT) {
    if (runState == ST_SEARCH && abortExploration) {
      Serial.println(F("Search stopped - heading home"));
      runState = ST_HOME;
    }
    if (runState == ST_HOME && posX == START_X && posY == START_Y) { finishAtHome(); return; }
    uint8_t to = (runState == ST_SEARCH) ? TO_UNVISITED : TO_START;
    if (!findRoute(to, false)) {
      if (to == TO_UNVISITED && candX >= 0) {
        candX = candY = -1;  // can't get into that room: forget it
      } else if (to == TO_UNVISITED && ++searchPass < SEARCH_PASSES) {
        // A misread wall may be hiding the room: clear the map (keeping where it is) and look again.
        Serial.println(F("Explored everything it can reach, no 2x2 room: clearing the map, looking again."));
        initMaze();
        setWall(START_X, START_Y, 2, true);
        return;  // read the walls here again
      } else if (to == TO_UNVISITED) {
        Serial.println(F("No 2x2 room found. Heading home."));
        runState = ST_HOME;
      } else {
        Serial.println(F("No way home on the map: forgetting the walls."));
        memset(wallMap, 0, sizeof(wallMap));
      }
      continue;
    }
    if (driveTo(routeDir, 1, DRIVE_PWM) || lastDriveResult != DRIVE_WALL) return;
  }  // a wall it just found blocks the way: choose again
}

// One straight of the fast run, on walls it has seen only (if a wall it didn't know blocks that
// way, it carries on cell by cell like the search).
void fastStep() {
  if (isGoal(posX, posY)) {
    Serial.println(F("\n*** GOAL *** heading home"));
    blink5();
    runState = ST_HOME;
    return;
  }
  if (goalX < 0) {
    Serial.println(F("No goal room on the map - do a search first (short press)."));
    runState = ST_WAIT;
    return;
  }
  if (findRoute(TO_GOAL, true)) {
    driveTo(routeDir, routeRun, SPEED_RUN_PWM);
  } else if (findRoute(TO_GOAL, false)) {
    // The known way is blocked (a wall it found on the way): go on carefully, reading walls.
    senseWallsHere();
    if (findRoute(TO_GOAL, false)) driveTo(routeDir, 1, DRIVE_PWM);
  } else {
    Serial.println(F("No way to the goal room on the map - do a new search (short press)."));
    runState = ST_WAIT;
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);  // a glitched I2C bus would otherwise freeze the robot forever
#endif

  pinMode(STATUS_LED, OUTPUT);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_C1, INPUT); pinMode(ENCODER_LEFT_C2, INPUT);
  pinMode(ENCODER_RIGHT_C1, INPUT); pinMode(ENCODER_RIGHT_C2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_C1), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_C1), rightEncoderISR, RISING);

  setupMotors();
  Serial.println(F("\nInitializing Sensors..."));  // seeing this mid-run = the board reset (brownout)
  initToFSensors();
  initMPU6050();
  calibrateGyro();

  if (digitalRead(START_BUTTON) == LOW) {
    initMaze();
    saveMaze();
    Serial.println(F("\nMap wiped."));
    while (digitalRead(START_BUTTON) == LOW);
    delay(50);
  } else if (loadMaze()) {
    Serial.print(F("\nSaved map:"));
    printMaze();
  } else {
    initMaze();
  }

  Serial.println(F("\n=== MICROMOUSE 29: FIND THE GOAL ROOM ==="));
  Serial.println(F("Short press = SEARCH, long press = FAST RUN, m = map."));
}

// --- MAIN LOOP ---
void loop() {
  if (runState == ST_WAIT) {
    // Print the sensors while waiting so the calibration can be checked.
    static unsigned long lastPrint = 0;
    sense();
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      Serial.print(F("L ")); Serial.print(tofL); Serial.print('/'); Serial.print(badL);
      Serial.print(F("  F ")); Serial.print(tofF); Serial.print('/'); Serial.print(badF);
      Serial.print(F("  R ")); Serial.print(tofR); Serial.print('/'); Serial.println(badR);
      badL = badF = badR = 0;
    }
    if (Serial.available() && Serial.read() == 'm') printMaze();
    int b = readButton();
    if (b == 1) {
      Serial.println(F("\n--- SEARCH ---"));
      beginRun(ST_SEARCH);
    } else if (b == 2) {
      Serial.println(F("\n--- FAST RUN ---"));
      beginRun(ST_FAST);
    }
  }
  else if (runState == ST_FAST) fastStep();
  else searchStep();
}
