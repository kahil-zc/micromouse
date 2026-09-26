// ╔══════════════════════════════════════════════════════════════════╗
// ║  15_micromouse_final                                             ║
// ║  Flood-fill micromouse for an 18 x 18 maze of 18 cm cells.       ║
// ║  Robot: 99 x 126 mm, axle 47 mm behind the front.                ║
// ║                                                                  ║
// ║  Button (START), robot in the start cell, back to the wall:      ║
// ║    1st press  = SEARCH: find the goal, explore until the         ║
// ║                 shortest path is proven, drive home              ║
// ║    next press = SPEED RUN to the goal and back home              ║
// ║                 (every press after that = another speed run)     ║
// ║    long press (1 s, LED on) = SEARCH again to map more           ║
// ║    press during a search = stop exploring and drive home         ║
// ║    held while powering on = forget the saved map                 ║
// ║  The map is saved in EEPROM, so it survives switching off.       ║
// ║                                                                  ║
// ║  The goal is found, not configured: it is the only place where   ║
// ║  four cells meet around a post with no wall touching it. The     ║
// ║  robot tries the centre first. It also works out which corner    ║
// ║  it started in.                                                  ║
// ║                                                                  ║
// ║  Search never wastes time on explored cells: it drives straight  ║
// ║  through mapped cells without stopping, and after the goal it    ║
// ║  only visits cells that could make the path shorter.             ║
// ║                                                                  ║
// ║  How the ToFs are used:                                          ║
// ║   * Sides, every 5 ms: steer toward the corridor centre.         ║
// ║   * Sides, every ~60 mm: measure the real angle to the walls     ║
// ║     and correct the gyro, so it stays straight in open cells.    ║
// ║   * Sides: wall start/end edges correct the distance driven.     ║
// ║   * Sides: closer than 15 mm = steer hard away and slow down.    ║
// ║   * Front: maps up to two cells ahead from each stop.            ║
// ║   * Front: checks the corridor before every straight and stops   ║
// ║     at the right spot; emergency stop under 20 mm.               ║
// ║                                                                  ║
// ║  Movement: stops at a "decision point" (axle 80 mm before the    ║
// ║  cell centre); 90° turns are 80 mm arcs; U-turns shuffle so the  ║
// ║  long rear has room, and square up on the wall behind.           ║
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

// === MAZE ===
// Cells are (x, y), y = 0 is the start row. The robot starts facing +y ("north"). It assumes it
// is in the left-hand corner (x = 0) and moves the map over if it finds it is in the right one.
#define MAZE_SIZE          18   // cells per side (up to 18)
#define MAX_IMPROVE_CELLS 150   // after the goal, map at most this many new cells looking for a shorter path

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
#define DRIVE_PWM           160    // into an unmapped cell
#define KNOWN_PWM           190    // through mapped cells while searching, and home after a speed run
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
#define EEPROM_MAGIC        (0xC0 ^ MAZE_SIZE)

#define DRIVE_STALL 0
#define DRIVE_DONE  1
#define DRIVE_WALL  2

#define ST_WAIT  0
#define ST_RUN   1

#if MAZE_SIZE > 18
#error "MAZE_SIZE must be 18 or less (RAM)"
#endif

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
float centreGap = SIDE_GAP_MM;    // learned side reading when centred, used when only one wall
uint8_t seqL = 0, seqR = 0;       // bump on every new side reading
long accL = 0, accF = 0, accR = 0;  // reading sums for averageAll()
uint16_t cntL = 0, cntF = 0, cntR = 0;

float carryMm = 0;                // axle position past this cell's decision point
float lastOvershootMm = 0;
float lastMovedMm = 0;            // distance of the last driveStraight, with edge corrections
bool lastDriveStalled = false;
float odoMm = 0;                  // distance driven straight since the last turn
bool backToWall = false;          // rear is against a wall: never reverse
int failCount = 0;

int runState = ST_WAIT;
bool abortExploration = false;

VL53L0X sensorLeft;
VL53L0X sensorFront;
VL53L0X sensorRight;

// Structs are defined before any function because the Arduino IDE puts its generated
// function prototypes above the first function.

// Tracks one side sensor switching between wall and gap (see edgeUpdate).
struct SideEdge {
  bool known, wall, pending;
  float since, pendingAt;
  uint8_t pendingCount;
};

// Straight-line fit of one side reading against distance driven (see fitUpdate).
struct WallFit {
  uint8_t n;
  float x0, sx, sy, sxx, sxy, sPsi;
};
WallFit fitL, fitR;

// --- MAZE (no hardware calls) ---
// This section is compiled on a PC by tests/micromouse_final_sim.sh and run on random mazes.
// Directions: 0 = north (+y), 1 = east (+x), 2 = south, 3 = west. Right of d is (d + 1) & 3.
#define PH_FIND     0   // looking for the goal
#define PH_IMPROVE  1   // goal reached: exploring cells that could give a shorter path
#define PH_HOME     2   // driving home, still mapping new cells on the way
#define PH_FAST     3   // speed run to the goal on seen walls only
#define PH_RETURN   4   // home after a speed run
#define PH_DONE     5   // in the start cell

#define MAX_TARGETS 12
#define MAX_RUN     17  // most cells driven in one straight

const int8_t DX[4] = { 0, 1, 0, -1 };
const int8_t DY[4] = { 1, 0, -1, 0 };

uint8_t maze[MAZE_SIZE][MAZE_SIZE];  // bits 0-3: wall N,E,S,W   bits 4-7: that wall has been seen
uint8_t dist[MAZE_SIZE][MAZE_SIZE];  // flood distance to the targets, 255 = unreachable
int8_t posX = 0, posY = 0;
uint8_t facing = 0;
int8_t startX = 0;           // MAZE_SIZE - 1 once the robot finds it started in the right corner
bool cornerKnown = false;
bool goalKnown = false;
int8_t goalX = 0, goalY = 0; // lower-left cell of the 2x2 goal
int8_t guessX = -1, guessY = -1;
uint8_t phase = PH_DONE;
uint16_t newCells = 0;       // cells mapped in the current phase
bool noRoute = false;        // the last plan found no way to its targets

int8_t tgX[MAX_TARGETS], tgY[MAX_TARGETS];
uint8_t tgN = 0;

bool inMaze(int x, int y) { return x >= 0 && y >= 0 && x < MAZE_SIZE && y < MAZE_SIZE; }
bool isStart(int x, int y) { return x == startX && y == 0; }
bool isGoal(int x, int y) { return goalKnown && x >= goalX && x <= goalX + 1 && y >= goalY && y <= goalY + 1; }
bool visited(int x, int y) { return (maze[x][y] & 0xF0) == 0xF0; }  // all four walls seen

// Records one wall on both cells that share it. The outer boundary always stays a wall.
// sticky = keep a wall that is already there (for less certain, long-range readings).
void putWall(int x, int y, uint8_t d, bool present, bool sticky) {
  int nx = x + DX[d], ny = y + DY[d];
  if (!inMaze(nx, ny)) present = true;
  if (sticky && (maze[x][y] & (1 << d))) return;
  uint8_t o = (d + 2) & 3;
  maze[x][y] |= 0x10 << d;
  if (present) maze[x][y] |= 1 << d; else maze[x][y] &= ~(1 << d);
  if (!inMaze(nx, ny)) return;
  maze[nx][ny] |= 0x10 << o;
  if (present) maze[nx][ny] |= 1 << o; else maze[nx][ny] &= ~(1 << o);
}

void initMaze() {
  memset(maze, 0, sizeof(maze));
  for (int i = 0; i < MAZE_SIZE; i++) {
    putWall(i, 0, 2, true, false);
    putWall(i, MAZE_SIZE - 1, 0, true, false);
    putWall(0, i, 3, true, false);
    putWall(MAZE_SIZE - 1, i, 1, true, false);
  }
  goalKnown = false;
  guessX = guessY = -1;
}

// pessimistic: unseen walls count as walls (speed run); otherwise as open (search).
bool blocked(int x, int y, uint8_t d, bool pessimistic) {
  uint8_t c = maze[x][y];
  if (c & (1 << d)) return true;
  return pessimistic && !(c & (0x10 << d));
}

void addTarget(int x, int y) {
  if (tgN < MAX_TARGETS) { tgX[tgN] = x; tgY[tgN] = y; tgN++; }
}
void targetGoal() {
  tgN = 0;
  for (int i = 0; i < 4; i++) addTarget(goalX + (i & 1), goalY + (i >> 1));
}
void targetStart() { tgN = 0; addTarget(startX, 0); }

// Distance from every cell to the nearest target, one layer at a time (no queue, so it fits
// in the Nano's RAM at 18 x 18).
void flood(bool pessimistic) {
  memset(dist, 255, sizeof(dist));
  for (uint8_t i = 0; i < tgN; i++) dist[tgX[i]][tgY[i]] = 0;
  for (uint8_t level = 0; level < 254; level++) {
    bool any = false;
    for (int x = 0; x < MAZE_SIZE; x++) {
      for (int y = 0; y < MAZE_SIZE; y++) {
        if (dist[x][y] != level) continue;
        any = true;
        for (uint8_t d = 0; d < 4; d++) {
          if (blocked(x, y, d, pessimistic)) continue;
          int nx = x + DX[d], ny = y + DY[d];
          if (dist[nx][ny] == 255) dist[nx][ny] = level + 1;
        }
      }
    }
    if (!any) break;
  }
}

// Open neighbour with the lowest distance, preferring straight, then right, left, back.
// 255 = no way out.
uint8_t bestDir(int x, int y, uint8_t f, bool pessimistic) {
  static const uint8_t order[4] = { 0, 1, 3, 2 };
  uint8_t best = 255, bestDist = 255;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t d = (f + order[i]) & 3;
    if (blocked(x, y, d, pessimistic)) continue;
    int nx = x + DX[d], ny = y + DY[d];
    if (dist[nx][ny] < bestDist) { bestDist = dist[nx][ny]; best = d; }
  }
  return best;
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

// Cells to drive straight from (x, y) along d (at least 1). Stops where the flood path turns,
// at a target, and (stopAtUnmapped) in the first cell that still has to be looked at.
uint8_t runLength(int x, int y, uint8_t d, bool pessimistic, bool stopAtUnmapped) {
  uint8_t n = 1;
  x += DX[d]; y += DY[d];
  while (n < MAX_RUN && dist[x][y] != 0 && (!stopAtUnmapped || visited(x, y)) &&
         bestDir(x, y, d, pessimistic) == d) {
    x += DX[d]; y += DY[d]; n++;
  }
  return n;
}

// The post at the top-right corner of cell (px, py): 0 = a wall touches it, 1 = it could be the
// middle of the goal, 2 = no wall touches it and all four walls are seen, so it IS the goal.
uint8_t postState(int px, int py) {
  uint8_t a = maze[px][py], b = maze[px + 1][py + 1];
  uint8_t walls = ((a >> 1) & 1) | ((a & 1) << 1) | (((b >> 3) & 1) << 2) | (((b >> 2) & 1) << 3);
  uint8_t seen  = ((a >> 5) & 1) | (((a >> 4) & 1) << 1) | (((b >> 7) & 1) << 2) | (((b >> 6) & 1) << 3);
  if (walls) return 0;
  return seen == 0x0F ? 2 : 1;
}

int centreDistance(int px, int py) {
  int c = MAZE_SIZE / 2 - 1;
  return abs(px - c) + abs(py - c);
}

// Looks for the goal post; the one closest to the centre wins.
void findGoal() {
  if (goalKnown) return;
  int best = 1000;
  for (int px = 0; px < MAZE_SIZE - 1; px++)
    for (int py = 0; py < MAZE_SIZE - 1; py++)
      if (postState(px, py) == 2 && centreDistance(px, py) < best) {
        best = centreDistance(px, py);
        goalX = px; goalY = py;
      }
  if (best < 1000) goalKnown = true;
}

// Picks the post to check next: close to the robot and close to the centre, keeping the
// current guess unless another is clearly better. False if no post could be the goal.
bool chooseGuess() {
  tgN = 0;
  addTarget(posX, posY);
  flood(false);
  int bestCost = 30000;
  int8_t bx = -1, by = -1;
  for (int px = 0; px < MAZE_SIZE - 1; px++) {
    for (int py = 0; py < MAZE_SIZE - 1; py++) {
      if (postState(px, py) != 1) continue;
      uint8_t m = 255;
      for (int i = 0; i < 4; i++) {
        uint8_t v = dist[px + (i & 1)][py + (i >> 1)];
        if (v < m) m = v;
      }
      if (m == 255) continue;
      int cost = m + 2 * centreDistance(px, py);
      if (px == guessX && py == guessY) cost -= 4;
      if (cost < bestCost) { bestCost = cost; bx = px; by = py; }
    }
  }
  guessX = bx; guessY = by;
  return bx >= 0;
}

// Targets while looking for the goal: the goal once known, else the unmapped cells around the
// guessed post (mapping them shows whether any wall touches it).
bool findTargets() {
  if (goalKnown) { targetGoal(); return true; }
  if (!chooseGuess()) return false;
  tgN = 0;
  for (int i = 0; i < 4; i++) {
    int x = guessX + (i & 1), y = guessY + (i >> 1);
    if (!visited(x, y)) addTarget(x, y);
  }
  return tgN > 0;
}

// Shortest start-to-goal length: optimistic (unseen walls open) and on seen walls only.
// Equal = the known path is proven shortest.
void pathLengths(uint8_t &optimistic, uint8_t &known) {
  targetGoal();
  flood(true);
  known = dist[startX][0];
  flood(false);
  optimistic = dist[startX][0];
}

// Targets after the goal: unmapped cells on the best possible start-to-goal path. None left
// (or the known path already as short as possible) = proven, go home.
bool improveTargets() {
  uint8_t optimistic, known;
  pathLengths(optimistic, known);  // leaves the optimistic flood in dist
  if (optimistic == 255 || known == optimistic) return false;
  int8_t cx[MAX_TARGETS], cy[MAX_TARGETS];
  uint8_t n = 0;
  int x = startX, y = 0;
  uint8_t f = 0;
  for (int steps = 0; steps < MAZE_SIZE * MAZE_SIZE && dist[x][y] != 0 && n < MAX_TARGETS; steps++) {
    uint8_t d = bestDir(x, y, f, false);
    if (d == 255) break;
    x += DX[d]; y += DY[d]; f = d;
    if (!visited(x, y)) { cx[n] = x; cy[n] = y; n++; }
  }
  tgN = 0;
  for (uint8_t i = 0; i < n; i++) addTarget(cx[i], cy[i]);
  return tgN > 0;
}

// Floods toward the current targets and picks the next straight. 0 = already on a target or
// no route (noRoute set).
uint8_t driveTo(bool pessimistic, bool stopAtUnmapped, uint8_t &dir) {
  flood(pessimistic);
  if (dist[posX][posY] == 255) { noRoute = true; return 0; }
  if (dist[posX][posY] == 0) return 0;
  dir = bestDir(posX, posY, facing, pessimistic);
  return runLength(posX, posY, dir, pessimistic, stopAtUnmapped);
}

// Decides the next move at a decision point (walls of this cell already recorded).
// Returns the cells to drive in `dir`, or 0 when the phase changed or there is no route.
uint8_t plan(uint8_t &dir) {
  noRoute = false;
  uint8_t n;
  switch (phase) {
    case PH_FIND:
      findGoal();
      if (goalKnown && isGoal(posX, posY)) { phase = PH_IMPROVE; newCells = 0; return 0; }
      if (!findTargets()) { phase = PH_HOME; return 0; }
      return driveTo(false, true, dir);
    case PH_IMPROVE:
      if (newCells >= MAX_IMPROVE_CELLS || !improveTargets()) { phase = PH_HOME; return 0; }
      return driveTo(false, true, dir);
    case PH_HOME:
      if (isStart(posX, posY)) { phase = PH_DONE; return 0; }
      targetStart();
      return driveTo(false, true, dir);
    case PH_FAST:
      if (isGoal(posX, posY)) { phase = PH_RETURN; return 0; }
      targetGoal();
      n = driveTo(true, false, dir);
      if (noRoute) { noRoute = false; phase = PH_FIND; }  // no fully seen path: search instead
      return n;
    case PH_RETURN:
      if (isStart(posX, posY)) { phase = PH_DONE; return 0; }
      targetStart();
      n = driveTo(true, false, dir);
      if (noRoute) { noRoute = false; phase = PH_HOME; }
      return n;
  }
  return 0;
}

// The robot assumed the left-hand corner but has just seen an opening to the west of column
// 0, so it started in the right-hand corner: everything seen so far is really column
// MAZE_SIZE - 1.
void shiftToRightCorner() {
  uint8_t col[MAZE_SIZE];
  for (int y = 0; y < MAZE_SIZE; y++) col[y] = maze[0][y];
  initMaze();
  for (int y = 0; y < MAZE_SIZE; y++)
    for (uint8_t d = 0; d <= 2; d += 2)
      if (col[y] & (0x10 << d)) putWall(MAZE_SIZE - 1, y, d, col[y] & (1 << d), false);
  posX += MAZE_SIZE - 1;
  startX = MAZE_SIZE - 1;
  cornerKnown = true;
}

// Records what the ToFs see from the decision point. aheadWall = k (1 or 2) when the front ToF
// sees the far wall of the k-th cell ahead; aheadOpen = it sees past the first cell ahead.
void recordWalls(bool wallL, bool wallF, bool wallR, uint8_t aheadWall, bool aheadOpen) {
  uint8_t left = (facing + 3) & 3, right = (facing + 1) & 3;
  if (!cornerKnown && posX == 0) {
    bool westOpen = (facing == 3 && !wallF) || (right == 3 && !wallR) || (left == 3 && !wallL);
    if (westOpen) shiftToRightCorner();
  }
  putWall(posX, posY, facing, wallF, false);
  putWall(posX, posY, right, wallR, false);
  putWall(posX, posY, left, wallL, false);
  if (wallF) return;
  int x1 = posX + DX[facing], y1 = posY + DY[facing];
  if (!inMaze(x1, y1)) return;
  if (aheadWall == 1) putWall(x1, y1, facing, true, true);
  else if (aheadOpen) {
    putWall(x1, y1, facing, false, true);
    int x2 = x1 + DX[facing], y2 = y1 + DY[facing];
    if (aheadWall == 2 && inMaze(x2, y2)) putWall(x2, y2, facing, true, true);
  }
}

// The robot drove n cells along d: those walls are open.
void advance(uint8_t d, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    putWall(posX, posY, d, false, false);
    posX += DX[d]; posY += DY[d];
  }
  if (posX > 0 && startX == 0) cornerKnown = true;
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

// Non-blocking: only reads a sensor that has a new measurement ready.
void updateToF() {
  if (sensorLeft.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
    uint16_t r = sensorLeft.readRangeContinuousMillimeters();
    tofL = (r > 8000) ? 999 : (int)r - LEFT_BIAS_MM;
    seqL++;
    if (tofL != 999) { accL += tofL; cntL++; }
  }
  if (sensorFront.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
    uint16_t r = sensorFront.readRangeContinuousMillimeters();
    tofF = (r > 8000) ? 999 : (int)r - FRONT_BIAS_MM;
    if (tofF != 999) { accF += tofF; cntF++; }
  }
  if (sensorRight.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
    uint16_t r = sensorRight.readRangeContinuousMillimeters();
    tofR = (r > 8000) ? 999 : (int)r - RIGHT_BIAS_MM;
    seqR++;
    if (tofR != 999) { accR += tofR; cntR++; }
  }
}

// Called in every loop: keeps the gyro integrating and the ToF values fresh.
// The button only sets a flag; the current cell always finishes first.
void sense() {
  updateGyro();
  updateToF();
  if (runState == ST_RUN && (phase == PH_FIND || phase == PH_IMPROVE) && digitalRead(START_BUTTON) == LOW)
    abortExploration = true;
}

void senseFor(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) { sense(); delay(2); }
}

// Averages all three readings over ~150 ms while standing still. 999 = nothing in range.
void averageAll(float &l, float &f, float &r) {
  accL = accF = accR = 0; cntL = cntF = cntR = 0;
  senseFor(150);
  l = cntL ? (float)accL / cntL : 999;
  f = cntF ? (float)accF / cntF : 999;
  r = cntR ? (float)accR / cntR : 999;
}

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
    phase = PH_DONE;
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

// --- SIDE-WALL EDGE CORRECTION ---
// Posts sit on every cell boundary. When a side reading changes between wall and gap, the side
// sensor is at a known spot (a post edge), which fixes the distance driven so far.
void resetEdge(SideEdge &e) { e.known = false; e.pending = false; }

// corrMm: running correction added to the encoder distance. startAxleMm: axle position at the
// start of the drive, measured from the centre of the cell the drive started in.
void edgeUpdate(SideEdge &e, int tof, float done, float &corrMm, float startAxleMm, char side) {
  bool w = tof < SIDE_FOLLOW_MM;
  if (!e.known) { e.known = true; e.wall = w; e.since = done; e.pending = false; return; }
  if (w == e.wall) { e.pending = false; return; }
  if (!e.pending) { e.pending = true; e.pendingAt = done; e.pendingCount = 1; return; }
  if (++e.pendingCount < 2) return;  // two readings in a row: not noise

  bool longEnough = (e.pendingAt - e.since) >= EDGE_MIN_RUN_MM;
  e.wall = w; e.since = e.pendingAt; e.pending = false;
  if (!longEnough) return;

  // Boundary posts are centred on 180k + 90. A wall starts at the near face of a post and
  // ends at its far face; the beam width shifts both slightly.
  float edgeRel = w ? -(POST_HALF_MM + EDGE_BEAM_MM) : (POST_HALF_MM + EDGE_BEAM_MM);
  float sensorY = startAxleMm + corrMm + e.pendingAt + SIDE_TOF_AHEAD_MM;
  float k = round((sensorY - CELL_MM / 2 - edgeRel) / CELL_MM);
  float trueY = k * CELL_MM + CELL_MM / 2 + edgeRel;
  float c = trueY - sensorY;
  if (fabs(c) < EDGE_MAX_CORR_MM) {
    corrMm += c;
    Serial.print(side); Serial.print(F(" edge, distance corrected by ")); Serial.println(c);
  }
}

// --- GYRO CORRECTION FROM SIDE WALLS ---
// While a side wall is present, the reading against distance driven is a straight line whose
// slope is the robot's real angle to the corridor. Comparing that with the gyro's angle over
// the same stretch shows how far the gyro has drifted (turn scale error, bias drift), and the
// gyro is pulled back a little each time. Fits carry on across stops until the robot turns.
void resetFit(WallFit &f) { f.n = 0; }
void resetFits() { resetFit(fitL); resetFit(fitR); }

// x: distance driven straight (odoMm + this drive). sideSign: +1 = left wall, -1 = right wall.
void fitUpdate(WallFit &f, int tof, float x, int sideSign) {
  if (tof >= SIDE_FOLLOW_MM) { f.n = 0; return; }  // no wall: start again
  if (f.n == 0) { f.x0 = x; f.sx = f.sy = f.sxx = f.sxy = f.sPsi = 0; }
  x -= f.x0;
  f.n++;
  f.sx += x; f.sy += tof; f.sxx += x * x; f.sxy += x * tof;
  f.sPsi += absoluteHeading - targetHeading;
  if (f.n < WALL_FIT_MIN_N || x < WALL_FIT_SPAN_MM) return;

  float den = f.n * f.sxx - f.sx * f.sx;
  if (den > 1) {
    float slope = (f.n * f.sxy - f.sx * f.sy) / den;
    // Turned toward the left wall = left reading shrinks and right reading grows.
    float wallPsi = -sideSign * asin(constrain(slope, -0.3f, 0.3f)) * RAD_TO_DEG;
    float gyroPsi = f.sPsi / f.n;
    float err = gyroPsi - wallPsi;
    if (fabs(err) < 8) {  // larger means a bad fit (post, open cell), not drift
      float step = constrain(err * HEADING_TRIM_GAIN, -MAX_TRIM_STEP_DEG, MAX_TRIM_STEP_DEG);
      absoluteHeading -= step;
      Serial.print(F("Gyro corrected from wall by ")); Serial.println(-step);
    }
  }
  f.n = 0;
}

// --- MOVEMENT BEHAVIOURS ---
// Drives distMm (negative = reverse) holding targetHeading.
//  center:      steer toward the corridor centre using the side walls
//  stopAtWall:  also stop at the decision point of a front wall
//  startAxleMm: if not NAN, correct the distance at side-wall edges (see edgeUpdate)
// Returns DRIVE_STALL, DRIVE_DONE or DRIVE_WALL.
int driveStraight(float distMm, int maxPwm, bool center, bool stopAtWall, float startAxleMm) {
  bool reverse = distMm < 0;
  float goal = fabs(distMm);
  bool useEdges = !reverse && !isnan(startAxleMm);
  bool useFit = center && !reverse;
  if (!useFit) resetFits();
  int result = DRIVE_DONE;
  float corr = 0;
  SideEdge edgeL, edgeR;
  resetEdge(edgeL); resetEdge(edgeR);
  uint8_t lastSeqL = seqL, lastSeqR = seqR;
  resetTicks();
  float lastDone = 0;
  unsigned long lastProgress = millis(), lastLoop = 0;

  while (true) {
    sense();
    float done = travelledMm();
    if (seqL != lastSeqL) {
      lastSeqL = seqL;
      if (useEdges) edgeUpdate(edgeL, tofL, done, corr, startAxleMm, 'L');
      if (useFit) fitUpdate(fitL, tofL, odoMm + done, +1);
    }
    if (seqR != lastSeqR) {
      lastSeqR = seqR;
      if (useEdges) edgeUpdate(edgeR, tofR, done, corr, startAxleMm, 'R');
      if (useFit) fitUpdate(fitR, tofR, odoMm + done, -1);
    }
    if (!reverse && tofF < FRONT_EMERGENCY_MM) {
      Serial.println(F("FRONT TOO CLOSE - emergency stop"));
      result = DRIVE_WALL;
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
  Serial.println(dir > 0 ? F("Arc LEFT") : F("Arc RIGHT"));
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
    if (tofF < FRONT_EMERGENCY_MM) {
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
  Serial.print(F("Shuffle sideways mm: ")); Serial.println(mm);
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

  Serial.println(cw ? F("Spin CW") : F("Spin CCW"));
  odoMm = 0;
  return spinTurn(cw ? -fabs(angleDeg) : fabs(angleDeg));
}

// Backs gently into the wall behind. Pressed flat against it the robot is exactly square to
// the maze and at a known distance, so the gyro heading, the gyro bias and the position along
// the corridor are all reset from it.
void squareOnBackWall() {
  Serial.println(F("Squaring on the wall behind"));
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
void prepareArc(int dir) {
  if (backToWall) return;
  float l, f, r;
  averageAll(l, f, r);
  float outerGap = (dir < 0) ? l : r;  // right turn: rear corner swings toward the left wall
  float innerGap = (dir < 0) ? r : l;
  if (outerGap >= WALL_THRESHOLD_MM) return;  // no outer wall
  float margin = outerGap + HALF_WIDTH_MM - ARC_OUTER_REACH_MM;
  if (margin >= ARC_MIN_MARGIN_MM) return;
  float shift = ARC_TARGET_MARGIN_MM - margin;
  if (innerGap < WALL_THRESHOLD_MM) shift = min(shift, innerGap - 12.0f);  // don't hit the inner wall
  if (shift <= SHIFT_TOL_MM) return;
  shiftSideways(dir < 0 ? -shift : shift, FRONT_GAP_MM);
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

bool executeTurn(char move) {
  if (move == 'S') return true;
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
    prepareArc(dir);
    ok = arcTurn(dir);
    carryMm = TURN_RADIUS_MM + DECISION_BACK_MM;  // arc ends TURN_RADIUS past the cell centre
    backToWall = false;
  }
  return ok;
}

// Drives to the decision point numCells ahead. Returns the cells actually advanced (fewer if a
// wall the map didn't know about stopped it, or it stalled).
uint8_t moveCells(uint8_t numCells, int maxPwm) {
  Serial.print(F("Forward cells: ")); Serial.println(numCells);
  float carry0 = carryMm;
  int r = driveStraight(numCells * CELL_MM - carry0, maxPwm, true, true, carry0 - DECISION_BACK_MM);
  float axle = carry0 + lastMovedMm;  // measured from the start cell's decision point
  int k = (int)round(axle / CELL_MM);
  k = constrain(k, 0, (int)numCells);
  carryMm = (r == DRIVE_WALL) ? 0 : axle - k * CELL_MM;
  lastDriveStalled = (r == DRIVE_STALL);
  backToWall = false;
  return k;
}

void startAlignment() {
  Serial.println(F("Squaring against back wall..."));
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
// Reads the walls of the current cell from its decision point, and with the front ToF up to
// two cells further along an open corridor.
void senseWallsHere() {
  float l, f, r;
  averageAll(l, f, r);
  bool wallF = f < FRONT_WALL_THRESHOLD_MM;
  uint8_t aheadWall = 0;
  bool aheadOpen = false;
  if (!wallF) {
    for (uint8_t k = 1; k <= 2; k++) {
      float gap = FRONT_GAP_MM + CELL_MM * k - carryMm;  // reading if the k-th cell ahead has a far wall
      if (f < gap + 35) { if (f > gap - 35) aheadWall = k; break; }
      if (k == 1 && f > gap + 70) aheadOpen = true;
    }
  }
  bool shifted = !cornerKnown;
  recordWalls(l < WALL_THRESHOLD_MM, wallF, r < WALL_THRESHOLD_MM, aheadWall, aheadOpen);
  if (shifted && startX != 0) Serial.println(F("Started in the RIGHT-hand corner - map moved over"));
}

// Before driving n cells: the front ToF must not see a wall the map says isn't there.
// Records any such wall and returns how many cells are really clear.
uint8_t checkAhead(uint8_t n) {
  senseFor(40);
  for (uint8_t k = 0; k < n && k < 3; k++) {
    float gap = FRONT_GAP_MM + CELL_MM * k - carryMm;  // reading if the far wall of cell k is there
    if (gap < 30) continue;                            // that wall line is already behind the nose
    if (tofF < gap + 25) {
      int x = posX + k * DX[facing], y = posY + k * DY[facing];
      putWall(x, y, facing, true, false);
      Serial.print(F("Front ToF found an unmapped wall ")); Serial.print(k); Serial.println(F(" cells ahead"));
      return k;
    }
  }
  return n;
}

void saveMaze() {
  EEPROM.update(0, EEPROM_MAGIC);
  const uint8_t *p = &maze[0][0];
  int i = 0;
  for (; i < MAZE_SIZE * MAZE_SIZE; i++) EEPROM.update(1 + i, p[i]);
  EEPROM.update(1 + i, goalKnown);
  EEPROM.update(2 + i, goalX);
  EEPROM.update(3 + i, goalY);
  EEPROM.update(4 + i, startX);
  EEPROM.update(5 + i, cornerKnown);
}

bool loadMaze() {
  if (EEPROM.read(0) != EEPROM_MAGIC) return false;
  uint8_t *p = &maze[0][0];
  int i = 0;
  for (; i < MAZE_SIZE * MAZE_SIZE; i++) p[i] = EEPROM.read(1 + i);
  goalKnown = EEPROM.read(1 + i);
  goalX = EEPROM.read(2 + i);
  goalY = EEPROM.read(3 + i);
  startX = EEPROM.read(4 + i);
  cornerKnown = EEPROM.read(5 + i);
  if (startX != 0 && startX != MAZE_SIZE - 1) return false;
  return true;
}

// ASCII map: robot ^ > v <, goal G, unseen walls '.'.
void printMaze() {
  static const char arrow[4] = { '^', '>', 'v', '<' };
  for (int y = MAZE_SIZE - 1; y >= 0; y--) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      Serial.print('+');
      uint8_t c = maze[x][y];
      Serial.print((c & 1) ? F("---") : (c & 0x10) ? F("   ") : F(" . "));
    }
    Serial.println('+');
    for (int x = 0; x < MAZE_SIZE; x++) {
      uint8_t c = maze[x][y];
      Serial.print((c & 8) ? '|' : (c & 0x80) ? ' ' : '.');
      Serial.print(' ');
      if (x == posX && y == posY) Serial.print(arrow[facing]);
      else if (isGoal(x, y)) Serial.print('G');
      else Serial.print(' ');
      Serial.print(' ');
    }
    Serial.println('|');
  }
  for (int x = 0; x < MAZE_SIZE; x++) Serial.print(F("+---"));
  Serial.println('+');
}

// Prints the shortest-path lengths. Equal = the known path is proven shortest.
void reportPath() {
  if (!goalKnown) { Serial.println(F("Goal not found yet.")); return; }
  uint8_t optimistic, known;
  pathLengths(optimistic, known);
  Serial.print(F("Goal at (")); Serial.print(goalX); Serial.print(','); Serial.print(goalY);
  Serial.print(F(")  shortest possible: ")); Serial.print(optimistic);
  Serial.print(F(" cells, known path: "));
  if (known == 255) Serial.println(F("none yet"));
  else { Serial.print(known); Serial.println(known == optimistic ? F(" (proven shortest)") : F(" (long press to search more)")); }
}

bool speedRunReady() {
  if (!goalKnown) return false;
  targetGoal();
  flood(true);
  return dist[startX][0] != 255;
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

void blink(uint8_t times) {
  for (uint8_t i = 0; i < times; i++) { digitalWrite(STATUS_LED, HIGH); delay(80); digitalWrite(STATUS_LED, LOW); delay(80); }
}

// Robot is in the start cell with its back to the start wall.
void beginRun(uint8_t firstPhase) {
  delay(500);  // let go of the robot
  startAlignment();
  posX = startX; posY = 0; facing = 0;
  putWall(startX, 0, 2, true, false);
  abortExploration = false;
  newCells = 0;
  phase = firstPhase;
  runState = ST_RUN;
}

// In the start cell: face north again and square up on the start wall.
void finishAtHome() {
  char m = relMove(facing, 0);
  if (m == 'U') spinSafely(180.0f);
  else if (m == 'R') spinSafely(-90.0f);
  else if (m == 'L') spinSafely(90.0f);
  facing = 0;
  startAlignment();
  saveMaze();
  printMaze();
  reportPath();
  Serial.println(speedRunReady() ? F("\nHOME. Press = SPEED RUN, long press = search more.")
                                 : F("\nHOME. Press = search."));
  runState = ST_WAIT;
}

// Says what happened when the plan moves to a new phase.
void announcePhase(uint8_t from) {
  if (from == PH_FIND && phase == PH_IMPROVE) {
    Serial.println(F("\n*** GOAL REACHED ***"));
    saveMaze();
    printMaze();
    blink(5);
    Serial.println(F("Checking cells that could give a shorter path..."));
  } else if (from == PH_FIND && phase == PH_HOME) {
    Serial.println(F("No post left that could be the goal - heading home"));
  } else if (from == PH_IMPROVE && phase == PH_HOME) {
    Serial.println(newCells >= MAX_IMPROVE_CELLS ? F("Exploration limit reached - heading home")
                                                 : F("Shortest path proven - heading home"));
    saveMaze();
  } else if (from == PH_FAST && phase == PH_RETURN) {
    Serial.println(F("\n*** SPEED RUN DONE *** heading home"));
    blink(5);
  } else if (from == PH_FAST && phase == PH_FIND) {
    Serial.println(F("No fully seen path to the goal - searching instead"));
  }
  if (phase == PH_DONE) finishAtHome();
}

// One move of a run: map this cell if new, plan, turn, check the corridor, drive.
void runStep() {
  bool searching = phase == PH_FIND || phase == PH_IMPROVE || phase == PH_HOME;
  if (searching && !visited(posX, posY)) {
    senseWallsHere();
    newCells++;
  }
  if (abortExploration && (phase == PH_FIND || phase == PH_IMPROVE)) {
    Serial.println(F("Search stopped - heading home"));
    abortExploration = false;
    phase = PH_HOME;
  }

  uint8_t oldPhase = phase;
  uint8_t dir = facing;
  uint8_t n = plan(dir);
  if (noRoute) {
    // A misread wall has closed every route. Start the map again and keep going.
    Serial.println(F("No route - a wall was misread. Clearing the map."));
    initMaze();
    putWall(startX, 0, 2, true, false);
    return;
  }
  if (phase != oldPhase) { announcePhase(oldPhase); return; }
  if (n == 0) return;

  noteResult(executeTurn(relMove(facing, dir)));
  facing = dir;
  if (runState != ST_RUN) return;

  n = checkAhead(n);
  if (n == 0) return;  // wall right ahead: plan again from here

  int pwm = (phase == PH_FAST) ? SPEED_RUN_PWM : (phase == PH_RETURN || n > 1) ? KNOWN_PWM : DRIVE_PWM;
  uint8_t done = moveCells(n, pwm);
  noteResult(!lastDriveStalled);
  advance(dir, done);
  if (done < n && !lastDriveStalled) putWall(posX, posY, dir, true, false);  // stopped by an unmapped wall
}

void printGeometryCheck() {
  float arcOuter = HALF_CORRIDOR_MM - ARC_OUTER_REACH_MM;
  float arcFront = HALF_CORRIDOR_MM - (sqrt(sq(TURN_RADIUS_MM + HALF_WIDTH_MM) + sq(AXLE_TO_FRONT_MM)) - DECISION_BACK_MM);
  float uTurnSide = (2 * HALF_CORRIDOR_MM - SPIN_R_REAR_MM - SPIN_R_FRONT_MM) / 2;

  Serial.println(F("\n--- GEOMETRY CHECK ---"));
  Serial.print(F("Side reading when centred:        ")); Serial.println(SIDE_GAP_MM);
  Serial.print(F("Front reading, axle at centre:    ")); Serial.println(HALF_CORRIDOR_MM - AXLE_TO_FRONT_MM);
  Serial.print(F("Front reading at decision point:  ")); Serial.println(FRONT_GAP_MM);
  Serial.print(F("Front reading for U-turns:        ")); Serial.println(U_TURN_FRONT_GAP_MM);
  Serial.print(F("Arc turn clearance, outer wall:   ")); Serial.println(arcOuter);
  Serial.print(F("Arc turn clearance, front wall:   ")); Serial.println(arcFront);
  Serial.print(F("U-turn clearance per side:        ")); Serial.println(uTurnSide);
  Serial.print(F("Arc inner/outer wheel ratio:      ")); Serial.println(ARC_RATIO);
  if (arcOuter < 6 || arcFront < 6) Serial.println(F("WARNING: arc turns are tight - raise TURN_RADIUS_MM"));
  if (uTurnSide < 2) Serial.println(F("WARNING: U-turns cannot clear the walls - shorten the rear overhang"));
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
  printGeometryCheck();

  if (digitalRead(START_BUTTON) == LOW) {
    initMaze();
    startX = 0; cornerKnown = false;
    saveMaze();
    Serial.println(F("\nMap cleared."));
    while (digitalRead(START_BUTTON) == LOW);
    delay(50);
  } else if (loadMaze()) {
    posX = startX; posY = 0; facing = 0;
    Serial.println(F("\nSaved map loaded:"));
    printMaze();
    reportPath();
  } else {
    initMaze();
    startX = 0; cornerKnown = false;
  }

  Serial.println(F("\n=== FLOOD FILL MICROMOUSE ==="));
  Serial.println(speedRunReady() ? F("Press = SPEED RUN, long press = search more.")
                                 : F("Press = SEARCH."));
}

// --- MAIN LOOP ---
void loop() {
  if (runState == ST_RUN) { runStep(); return; }

  // Waiting: print the sensors so the calibration can be checked.
  static unsigned long lastPrint = 0;
  sense();
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print(F("L ")); Serial.print(tofL);
    Serial.print(F("  F ")); Serial.print(tofF);
    Serial.print(F("  R ")); Serial.println(tofR);
  }
  int b = readButton();
  if (b == 1 && speedRunReady()) {
    Serial.println(F("\n--- SPEED RUN ---"));
    beginRun(PH_FAST);
  } else if (b != 0) {
    Serial.println(F("\n--- SEARCH ---"));
    beginRun(PH_FIND);
  }
}
