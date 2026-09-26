// ╔══════════════════════════════════════════════════════════════════╗
// ║  19_micromouse_final                                             ║
// ║  Flood-fill micromouse for a 5 x 10 maze of 18 cm cells, either  ║
// ║  way round, started in either corner. The goal is the corner     ║
// ║  diagonally opposite the start.                                  ║
// ║  Robot: 99 x 126 mm, axle 47 mm behind the front.                ║
// ║  Driving and turning are those of 14_floodfill_final.            ║
// ║                                                                  ║
// ║  Search logic follows ukmars/mazerunner-core:                    ║
// ║   * a wall is mapped the first time it is seen and never changed ║
// ║     after that (so the map can't flip and send the robot round   ║
// ║     in circles),                                                 ║
// ║   * flood fill to the goal, then flood fill back to the start,   ║
// ║     both treating unseen walls as open, so the way back tries    ║
// ║     other routes,                                                ║
// ║   * the speed run only uses walls that have been seen.           ║
// ║                                                                  ║
// ║  Button (START), robot in the start cell, back to the wall:      ║
// ║    1st press  = SEARCH to the goal and back to the start         ║
// ║    every press after that = SPEED RUN to the goal and back home  ║
// ║    long press (1 s, LED on) = search again (keeps the map, maps  ║
// ║                               more of it)                        ║
// ║    press while searching = stop and drive home                   ║
// ║    double click, or held while powering on = forget the map      ║
// ║  The map is saved in EEPROM, so it survives switching off.       ║
// ║                                                                  ║
// ║  LED: 5 blinks = reached the goal, 3 blinks = map cleared,       ║
// ║       on and staying on = 3 failed moves in a row, stopped.      ║
// ║                                                                  ║
// ║  How the ToFs are used:                                          ║
// ║   * Sides, every 5 ms: steer toward the corridor centre.         ║
// ║   * Sides, every ~60 mm: measure the real angle to the walls     ║
// ║     and correct the gyro, so it stays straight in open cells.    ║
// ║   * Sides: wall start/end edges correct the distance driven.     ║
// ║   * Sides: closer than 15 mm = steer hard away and slow down.    ║
// ║   * Front: stops at the right spot; emergency stop under 20 mm.  ║
// ║   * Every stop: a wall one cell ahead fixes the position along   ║
// ║     the corridor, and two side walls tune the centred reading.   ║
// ║   * Before going straight from a stop: more than 12 mm off the   ║
// ║     corridor centre = shuffle back to the centre first.          ║
// ║   * Before a turn: pull up to the front wall if there is one.    ║
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
// Cells are (x, y), y = 0 is the start row. The robot starts facing +y ("ahead"); +x is to its
// right. The map is a 10 x 10 box so the 5 x 10 maze fits whichever way round it is; the real
// outer walls are read like any other wall (and added to the map once the robot knows which
// way round the maze is). The goal is the corner diagonally opposite the start.
#define MAZE_SHORT   5   // cells on the short side of the maze
#define MAZE_LONG   10   // cells on the long side
#define MAZE_W      MAZE_LONG   // map size: fits the maze either way round
#define MAZE_H      MAZE_LONG
#define GOAL_SIZE    1   // 1 = the corner cell is the goal, 2 = the 2x2 block in that corner
// Which way round: 0 = work it out, 1 = long side across (10 across, 5 ahead),
// 2 = long side ahead (5 across, 10 ahead). Setting it saves some searching.
#define LONG_SIDE    0
// Start corner: 0 = work it out (the robot assumes left, and moves the map over the first time it
// sees an opening to the left of the start column), 1 = left corner (maze is to the robot's
// right), 2 = right corner (maze is to the robot's left). Setting it removes any chance of a
// wrong guess.
#define START_CORNER 0

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
#define DRIVE_PWM           160    // exploring and driving home, one cell at a time
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
#define SETTLE_TOL_MM       5.0f   // before a turn, drive back to the decision point if this far off
#define SHIFT_TOL_MM        1.5f   // don't shuffle for less than this
#define SHIFT_ANGLE_DEG     12.0f  // heading used for the sideways shuffle
#define MAX_SHIFT_MM        15.0f

#define GYRO_SCALE          65.5f  // 500 deg/s range; bigger number = turns further
#define GYRO_CALIB_SAMPLES  200
#define MAX_FAILS           3      // consecutive failed moves before giving up
#define SENSE_MS            150    // how long to average the ToFs when reading the walls
#define FRONT_FIX_MAX_MM    40.0f  // trust a front-wall position fix only this close to the encoders
#define CENTRE_TOL_MM       12.0f  // before a straight, shuffle to the centre if further off than this
#define LONG_PRESS_MS       1000
#define DOUBLE_CLICK_MS     400    // second press within this = double click
#define EEPROM_MAGIC        (0x50 ^ MAZE_SHORT ^ (MAZE_LONG << 3) ^ GOAL_SIZE)

#define DRIVE_STALL 0
#define DRIVE_DONE  1
#define DRIVE_WALL  2

#define ST_WAIT  0
#define ST_RUN   1

#if MAZE_W > 18 || MAZE_H > 18
#error "MAZE_W and MAZE_H must be 18 or less (RAM)"
#endif
#if GOAL_SIZE < 1 || GOAL_SIZE > 2
#error "GOAL_SIZE must be 1 or 2"
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
float lastL = 999, lastR = 999;   // side readings of the last wall reading at a stop
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
// This section is compiled on a PC by tests/micromouse_19_sim.sh and run on random mazes.
// Directions: 0 = ahead (+y), 1 = right (+x), 2 = back, 3 = left, as seen from the start.
// Right of d is (d + 1) & 3.
#define PH_SEARCH   0   // flood fill to the goal, unseen walls counted as open
#define PH_HOME     1   // flood fill back to the start, unseen walls counted as open
#define PH_FAST     2   // speed run to the goal on seen walls only
#define PH_RETURN   3   // home after a speed run, on seen walls only
#define PH_DONE     4   // in the start cell

#define MAX_TARGETS 12
#define MAX_REOPENS 8   // times walls are read again after a "no route" before starting the map again
#define MAX_RUN     17  // most cells driven in one straight

const int8_t DX[4] = { 0, 1, 0, -1 };
const int8_t DY[4] = { 1, 0, -1, 0 };

uint8_t maze[MAZE_W][MAZE_H];  // bits 0-3: wall N,E,S,W   bits 4-7: that wall has been seen
uint8_t dist[MAZE_W][MAZE_H];  // flood distance to the targets, 255 = unreachable
int8_t posX = 0, posY = 0;
uint8_t facing = 0;
int8_t startX = 0;             // MAZE_W - 1 once the robot finds it started in the right corner
bool cornerKnown = false;
uint8_t longSide = 0;          // 0 = not known yet, 1 = long side across, 2 = long side ahead
uint8_t phase = PH_DONE;
bool noRoute = false;          // the last plan found no way to its targets
uint8_t reopens = 0;           // see recoverRoute()

int8_t tgX[MAX_TARGETS], tgY[MAX_TARGETS];
uint8_t tgN = 0;

bool inMaze(int8_t x, int8_t y) { return x >= 0 && y >= 0 && x < MAZE_W && y < MAZE_H; }
bool isStart(int8_t x, int8_t y) { return x == startX && y == 0; }
bool visited(int8_t x, int8_t y) { return (maze[x][y] & 0xF0) == 0xF0; }  // all four walls seen

// Goal: the GOAL_SIZE x GOAL_SIZE block in the corner diagonally opposite the start. Until the
// robot knows which way round the maze is, both possible corners count; only the one inside the
// real maze can ever be reached.
bool isGoal(int8_t x, int8_t y) {
  int8_t a = (startX == 0) ? x : startX - x;  // cells across from the start column
  if (a < 0) return false;
  bool across = a >= MAZE_LONG - GOAL_SIZE && a < MAZE_LONG && y >= MAZE_SHORT - GOAL_SIZE && y < MAZE_SHORT;
  bool ahead = a >= MAZE_SHORT - GOAL_SIZE && a < MAZE_SHORT && y >= MAZE_LONG - GOAL_SIZE && y < MAZE_LONG;
  if (longSide == 1) return across;
  if (longSide == 2) return ahead;
  return across || ahead;
}

// Records one wall on both cells that share it. The outer boundary always stays a wall.
void putWall(int8_t x, int8_t y, uint8_t d, bool present) {
  if (!inMaze(x, y)) return;  // never write outside the map
  int8_t nx = x + DX[d], ny = y + DY[d];
  if (!inMaze(nx, ny)) present = true;
  uint8_t o = (d + 2) & 3;
  maze[x][y] |= 0x10 << d;
  if (present) maze[x][y] |= 1 << d; else maze[x][y] &= ~(1 << d);
  if (!inMaze(nx, ny)) return;
  maze[nx][ny] |= 0x10 << o;
  if (present) maze[nx][ny] |= 1 << o; else maze[nx][ny] &= ~(1 << o);
}

// Once the robot knows which way round the maze is and which corner it started in, the real
// outer walls on the far sides are known too: put them in the map.
void addOuterWalls() {
  if (!cornerKnown || longSide == 0) return;
  int8_t across = (longSide == 1) ? MAZE_LONG : MAZE_SHORT;
  int8_t ahead = (longSide == 1) ? MAZE_SHORT : MAZE_LONG;
  for (int8_t i = 0; i < ahead; i++) {  // far side wall, beside the last column
    if (startX == 0) putWall(across - 1, i, 1, true);
    else putWall(startX - across + 1, i, 3, true);
  }
  for (int8_t i = 0; i < across; i++)   // far end wall, beyond the last row
    putWall(startX == 0 ? i : startX - i, ahead - 1, 0, true);
}

// The robot has stood in (x, y): if that is further than the short side reaches, it shows which
// way round the maze is.
void noteShape(int8_t x, int8_t y) {
  if (longSide != 0) return;
  if (y >= MAZE_SHORT) longSide = 2;
  else if (cornerKnown && abs(x - startX) >= MAZE_SHORT) longSide = 1;
  if (longSide != 0) addOuterWalls();
}

void initMaze() {
  memset(maze, 0, sizeof(maze));
  for (int8_t x = 0; x < MAZE_W; x++) {
    putWall(x, 0, 2, true);
    putWall(x, MAZE_H - 1, 0, true);
  }
  for (int8_t y = 0; y < MAZE_H; y++) {
    putWall(0, y, 3, true);
    putWall(MAZE_W - 1, y, 1, true);
  }
  addOuterWalls();
}

// pessimistic: unseen walls count as walls (speed run); otherwise as open (search).
// A wall on either side counts (they only differ while a suspect wall is being read again).
bool blocked(int8_t x, int8_t y, uint8_t d, bool pessimistic) {
  uint8_t c = maze[x][y];
  if (c & (1 << d)) return true;
  int8_t nx = x + DX[d], ny = y + DY[d];
  if (inMaze(nx, ny) && (maze[nx][ny] & (1 << ((d + 2) & 3)))) return true;
  return pessimistic && !(c & (0x10 << d));
}

void addTarget(int8_t x, int8_t y) {
  if (tgN < MAX_TARGETS) { tgX[tgN] = x; tgY[tgN] = y; tgN++; }
}
void targetGoal() {
  tgN = 0;
  for (int8_t x = 0; x < MAZE_W; x++)
    for (int8_t y = 0; y < MAZE_H; y++)
      if (isGoal(x, y)) addTarget(x, y);
}
void targetStart() { tgN = 0; addTarget(startX, 0); }

// Distance from every cell to the nearest target, one layer at a time.
void flood(bool pessimistic) {
  memset(dist, 255, sizeof(dist));
  for (uint8_t i = 0; i < tgN; i++) dist[tgX[i]][tgY[i]] = 0;
  for (uint8_t level = 0; level < 254; level++) {
    bool any = false;
    for (int8_t x = 0; x < MAZE_W; x++) {
      for (int8_t y = 0; y < MAZE_H; y++) {
        if (dist[x][y] != level) continue;
        any = true;
        for (uint8_t d = 0; d < 4; d++) {
          if (blocked(x, y, d, pessimistic)) continue;
          int8_t nx = x + DX[d], ny = y + DY[d];
          if (dist[nx][ny] == 255) dist[nx][ny] = level + 1;
        }
      }
    }
    if (!any) break;
  }
}

// Open neighbour with the lowest distance, looking ahead, then right, left, back, so a tie
// goes straight on (as mazerunner-core's heading_to_smallest). 255 = no way out.
uint8_t bestDir(int8_t x, int8_t y, uint8_t f, bool pessimistic) {
  static const uint8_t order[4] = { 0, 1, 3, 2 };
  uint8_t best = 255, bestDist = 255;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t d = (f + order[i]) & 3;
    if (blocked(x, y, d, pessimistic)) continue;
    int8_t nx = x + DX[d], ny = y + DY[d];
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

// Cells to drive straight from (x, y) along d (at least 1). Stops where the flood path turns
// and at a target.
uint8_t runLength(int8_t x, int8_t y, uint8_t d, bool pessimistic) {
  uint8_t n = 1;
  x += DX[d]; y += DY[d];
  while (n < MAX_RUN && dist[x][y] != 0 && bestDir(x, y, d, pessimistic) == d) {
    x += DX[d]; y += DY[d]; n++;
  }
  return n;
}

// Speed-run length from the start to the goal on seen walls (255 = no route yet).
uint8_t knownPathLength() {
  targetGoal();
  flood(true);
  return dist[startX][0];
}

// Floods toward the current targets and picks the next move. 0 = already on a target or no
// route (noRoute set). Only the speed run drives several cells in one go; searching goes one
// cell at a time, stopping in each cell to read the walls (the 14_floodfill_final way).
uint8_t driveTo(bool pessimistic, bool multiCell, uint8_t &dir) {
  flood(pessimistic);
  if (dist[posX][posY] == 255) { noRoute = true; return 0; }
  if (dist[posX][posY] == 0) return 0;
  dir = bestDir(posX, posY, facing, pessimistic);
  return multiCell ? runLength(posX, posY, dir, pessimistic) : 1;
}

// Decides the next move at a decision point (walls of this cell already recorded).
// Returns the cells to drive in `dir`, or 0 when the phase changed or there is no route.
uint8_t plan(uint8_t &dir) {
  noRoute = false;
  uint8_t n;
  switch (phase) {
    case PH_SEARCH:
      if (isGoal(posX, posY)) { phase = PH_HOME; return 0; }
      targetGoal();
      return driveTo(false, false, dir);
    case PH_HOME:
      if (isStart(posX, posY)) { phase = PH_DONE; return 0; }
      targetStart();
      return driveTo(false, false, dir);
    case PH_FAST:
      if (isGoal(posX, posY)) { phase = PH_RETURN; return 0; }
      targetGoal();
      n = driveTo(true, true, dir);
      if (noRoute) { noRoute = false; phase = PH_SEARCH; }  // no seen route: search instead
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

// The robot assumed the left-hand corner but has just seen an opening to the left of column 0,
// so it started in the right-hand corner: everything seen so far is really column MAZE_W - 1.
void shiftToRightCorner() {
  uint8_t col[MAZE_H];
  for (int8_t y = 0; y < MAZE_H; y++) col[y] = maze[0][y];
  startX = MAZE_W - 1;
  cornerKnown = true;
  initMaze();
  for (int8_t y = 0; y < MAZE_H; y++) {
    bool wasVisited = (col[y] & 0xF0) == 0xF0;
    for (uint8_t d = 0; d < 4; d++) {
      if (!(col[y] & (0x10 << d))) continue;
      if (!wasVisited && d == 3) continue;  // the assumed left border was never really seen
      putWall(MAZE_W - 1, y, d, col[y] & (1 << d));
    }
  }
  posX += MAZE_W - 1;
  addOuterWalls();
}

// Called at the start of each run from the start cell.
void resetStart() {
  reopens = 0;
  if (START_CORNER == 1) { startX = 0; cornerKnown = true; }
  if (START_CORNER == 2) { startX = MAZE_W - 1; cornerKnown = true; }
  if (LONG_SIDE != 0) longSide = LONG_SIDE;
  addOuterWalls();
}

// Walls read from a decision point, as bits: 1 = left, 2 = front, 4 = right.
// Direction of bit i is (facing + 3 + i) & 3.

// True if these walls show an opening to the left of column 0, i.e. the robot started in the
// right-hand corner. That moves the whole map, so senseWallsHere() reads again before believing it.
bool showsRightCorner(uint8_t walls) {
  if (cornerKnown || posX != 0) return false;
  for (uint8_t i = 0; i < 3; i++) if (((facing + 3 + i) & 3) == 3 && !(walls & (1 << i))) return true;
  return false;
}

// Records the three walls the ToFs see from this cell's decision point. As in mazerunner-core,
// a wall is only written the first time it is seen; after that the map keeps it.
void recordWalls(uint8_t walls) {
  if (showsRightCorner(walls)) shiftToRightCorner();
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t d = (facing + 3 + i) & 3;
    if (!(maze[posX][posY] & (0x10 << d))) putWall(posX, posY, d, walls & (1 << i));
  }
}

// The robot drove n cells along d: those walls are open.
void advance(uint8_t d, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    putWall(posX, posY, d, false);
    posX += DX[d]; posY += DY[d];
  }
  if (posX > 0 && startX == 0 && !cornerKnown) { cornerKnown = true; addOuterWalls(); }
  noteShape(posX, posY);
}

// Wrongly seen walls have shut the robot off from its target. Forgets every wall between the
// cells the robot can reach and the cells it can't (except the one behind it, which it can't
// read from here), so it drives back and reads them again. If that keeps happening, the map
// starts again from scratch.
void recoverRoute() {
  tgN = 0;
  addTarget(posX, posY);
  flood(false);  // dist != 255: cells the robot can reach
  bool any = false;
  if (reopens < MAX_REOPENS) {
    for (int8_t x = 0; x < MAZE_W; x++) {
      for (int8_t y = 0; y < MAZE_H; y++) {
        if (dist[x][y] == 255) continue;
        for (uint8_t d = 0; d < 4; d++) {
          int8_t nx = x + DX[d], ny = y + DY[d];
          if (!inMaze(nx, ny) || dist[nx][ny] != 255 || !(maze[x][y] & (1 << d))) continue;
          if (x == posX && y == posY && d == ((facing + 2) & 3)) continue;
          uint8_t o = (d + 2) & 3;
          maze[x][y] &= ~((1 << d) | (0x10 << d));
          maze[nx][ny] &= ~((1 << o) | (0x10 << o));
          any = true;
        }
      }
    }
  }
  if (any) { reopens++; return; }
  // Start the map again, forgetting what it worked out about the maze's shape too (that may
  // have come from a misread). Still in the start column after deciding it started in the
  // right-hand corner? That may be the misreading, so forget it too.
  if (START_CORNER == 0 && startX != 0 && posX == startX) { posX = 0; startX = 0; cornerKnown = false; }
  longSide = 0;
  initMaze();
  resetStart();
  putWall(startX, 0, 2, true);
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
// Wakes one ToF (the others stay off, so it answers at the default address) and moves it to addr.
void initToF(VL53L0X &sensor, uint8_t xshutPin, uint8_t addr) {
  digitalWrite(xshutPin, HIGH); delay(50);
  sensor.setTimeout(500);
  if (sensor.init()) {
    sensor.setAddress(addr);
    sensor.setMeasurementTimingBudget(20000);
    sensor.startContinuous();
  }
}

void initToFSensors() {
  pinMode(TOF_XSHUT_LEFT, OUTPUT); digitalWrite(TOF_XSHUT_LEFT, LOW);
  pinMode(TOF_XSHUT_FRONT, OUTPUT); digitalWrite(TOF_XSHUT_FRONT, LOW);
  pinMode(TOF_XSHUT_RIGHT, OUTPUT); digitalWrite(TOF_XSHUT_RIGHT, LOW);
  delay(100);
  initToF(sensorLeft, TOF_XSHUT_LEFT, 0x30);
  initToF(sensorFront, TOF_XSHUT_FRONT, 0x31);
  initToF(sensorRight, TOF_XSHUT_RIGHT, 0x32);
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
  if (runState == ST_RUN && phase == PH_SEARCH && digitalRead(START_BUTTON) == LOW)
    abortExploration = true;
}

void senseFor(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) { sense(); delay(2); }
}

// Averages all three readings over SENSE_MS while standing still. 999 = nothing in range.
void averageAll(float &l, float &f, float &r) {
  accL = accF = accR = 0; cntL = cntF = cntR = 0;
  senseFor(SENSE_MS);
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
  if (failCount >= MAX_FAILS) {
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
void edgeUpdate(SideEdge &e, int tof, float done, float &corrMm, float startAxleMm) {
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
  if (fabs(c) < EDGE_MAX_CORR_MM) corrMm += c;
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
      if (useEdges) edgeUpdate(edgeL, tofL, done, corr, startAxleMm);
      if (useFit) fitUpdate(fitL, tofL, odoMm + done, +1);
    }
    if (seqR != lastSeqR) {
      lastSeqR = seqR;
      if (useEdges) edgeUpdate(edgeR, tofR, done, corr, startAxleMm);
      if (useFit) fitUpdate(fitR, tofR, odoMm + done, -1);
    }
    if (!reverse && tofF < FRONT_EMERGENCY_MM) {
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
      senseFor(80);
      return false;
    }
    if (tofF < FRONT_EMERGENCY_MM) {
      stopMotors();
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

// Before driving straight on from a stop: if the side walls show the robot is well off the
// corridor centre, shuffle back to it first. (Driving also steers toward the centre, but a big
// offset at the start of a straight would take most of a cell to remove.) Not against the wall
// behind (no room to reverse).
void centreBeforeStraight() {
  if (backToWall) return;
  bool hasL = lastL < SIDE_FOLLOW_MM, hasR = lastR < SIDE_FOLLOW_MM;
  float off;  // > 0: robot is right of centre, so move left
  if (hasL && hasR) off = (lastL - lastR) / 2;
  else if (hasL) off = lastL - centreGap;
  else if (hasR) off = centreGap - lastR;
  else return;
  if (fabs(off) > CENTRE_TOL_MM) shiftSideways(off, FRONT_GAP_MM);
}

// Puts the axle on the decision point before a turn: the front wall if there is one,
// otherwise the encoders.
void settleAtDecisionPoint() {
  sense();
  if (tofF < FRONT_WALL_THRESHOLD_MM) {
    alignToFrontWall(FRONT_GAP_MM);
    carryMm = 0;
  } else if (fabs(carryMm) > SETTLE_TOL_MM && !(backToWall && carryMm > 0)) {
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

// --- MAP: WALL SENSING AND SAVING ---
// One standing reading of the three walls (bits: 1 = left, 2 = front, 4 = right). The front
// threshold allows for the robot not being exactly on the decision point.
uint8_t readWalls(float &l, float &f, float &r) {
  averageAll(l, f, r);
  return (l < WALL_THRESHOLD_MM) | ((f < FRONT_WALL_THRESHOLD_MM - carryMm) << 1) | ((r < WALL_THRESHOLD_MM) << 2);
}

// Reads the three walls of the current cell from its decision point, at every stop. Walls not
// seen before go in the map (seen walls are never changed, see recordWalls).
// The standing reading is also used, with no extra moves, to fix the position and the centring:
//  * a front wall one cell ahead gives the axle position along the corridor,
//  * two side walls give the side reading when centred.
void senseWallsHere() {
  float l, f, r, l2, f2, r2;
  uint8_t walls = readWalls(l, f, r);
  if (showsRightCorner(walls)) walls |= readWalls(l2, f2, r2);  // an opening counts only if both readings see it
  recordWalls(walls);

  float fromFront = FRONT_GAP_MM + CELL_MM - f;  // position if the wall is one cell ahead
  if (!(walls & 2) && !backToWall && fabs(fromFront - carryMm) < FRONT_FIX_MAX_MM)
    carryMm = (carryMm + fromFront) / 2;  // the beam is wide that far: meet the encoders halfway
  if (l < SIDE_FOLLOW_MM && r < SIDE_FOLLOW_MM)
    centreGap = constrain(centreGap + 0.2f * ((l + r) / 2 - centreGap), SIDE_GAP_MM - 10, SIDE_GAP_MM + 10);

  lastL = l; lastR = r;
}

void saveMaze() {
  EEPROM.update(0, EEPROM_MAGIC);
  int addr = 1;
  for (int x = 0; x < MAZE_W; x++)
    for (int y = 0; y < MAZE_H; y++) EEPROM.update(addr++, maze[x][y]);
  EEPROM.update(addr++, startX);
  EEPROM.update(addr++, cornerKnown);
  EEPROM.update(addr++, longSide);
}

bool loadMaze() {
  if (EEPROM.read(0) != EEPROM_MAGIC) return false;
  int addr = 1;
  for (int x = 0; x < MAZE_W; x++)
    for (int y = 0; y < MAZE_H; y++) maze[x][y] = EEPROM.read(addr++);
  startX = EEPROM.read(addr++);
  cornerKnown = EEPROM.read(addr++);
  longSide = EEPROM.read(addr++);
  return (startX == 0 || startX == MAZE_W - 1) && longSide <= 2;
}

bool speedRunReady() { return knownPathLength() != 255; }

// --- RUN CONTROL ---
// Returns 0 = not pressed, 1 = short press, 2 = long press (LED lights once it counts as long),
// 3 = double click.
int readButton() {
  if (digitalRead(START_BUTTON) != LOW) return 0;
  delay(30);
  if (digitalRead(START_BUTTON) != LOW) return 0;
  unsigned long t = millis();
  while (digitalRead(START_BUTTON) == LOW) {
    if (millis() - t > LONG_PRESS_MS) digitalWrite(STATUS_LED, HIGH);
  }
  digitalWrite(STATUS_LED, LOW);
  if (millis() - t > LONG_PRESS_MS) return 2;
  delay(30);  // let the release bounce settle
  unsigned long released = millis();
  while (millis() - released < DOUBLE_CLICK_MS) {
    if (digitalRead(START_BUTTON) == LOW) {
      delay(30);
      while (digitalRead(START_BUTTON) == LOW);
      return 3;
    }
  }
  return 1;
}

// Forgets the saved map, start corner and which way round the maze is.
void clearMap() {
  startX = 0; cornerKnown = false; longSide = 0;
  initMaze();
  resetStart();
  posX = startX; posY = 0; facing = 0;
  saveMaze();
}

void blink(uint8_t times) {
  for (uint8_t i = 0; i < times; i++) { digitalWrite(STATUS_LED, HIGH); delay(80); digitalWrite(STATUS_LED, LOW); delay(80); }
}

// Robot is in the start cell with its back to the start wall.
void beginRun(uint8_t firstPhase) {
  delay(500);  // let go of the robot
  digitalWrite(STATUS_LED, LOW);
  startAlignment();
  resetStart();
  posX = startX; posY = 0; facing = 0;
  putWall(startX, 0, 2, true);
  abortExploration = false;
  phase = firstPhase;
  runState = ST_RUN;
}

// In the start cell: face ahead again and square up on the start wall.
void finishAtHome() {
  char m = relMove(facing, 0);
  if (m == 'U') spinSafely(180.0f);
  else if (m == 'R') spinSafely(-90.0f);
  else if (m == 'L') spinSafely(90.0f);
  facing = 0;
  startAlignment();
  saveMaze();
  runState = ST_WAIT;
}

// One move of a run (like searchStep in 14): read the walls, plan, turn, drive.
void runStep() {
  if (phase != PH_FAST) senseWallsHere();  // the speed run trusts the map and doesn't stop
  if (abortExploration && phase == PH_SEARCH) {
    abortExploration = false;
    phase = PH_HOME;
  }

  uint8_t oldPhase = phase;
  uint8_t dir = facing;
  uint8_t n = plan(dir);
  if (noRoute) { recoverRoute(); return; }
  if (phase != oldPhase) {
    if (oldPhase == PH_SEARCH && phase == PH_HOME) { saveMaze(); blink(5); }  // goal reached
    if (oldPhase == PH_FAST && phase == PH_RETURN) blink(5);                  // goal reached
    if (phase == PH_DONE) finishAtHome();
    return;
  }
  if (n == 0) return;

  char move = relMove(facing, dir);
  if (move == 'S' && phase != PH_FAST) centreBeforeStraight();
  noteResult(executeTurn(move));
  facing = dir;
  if (runState != ST_RUN) return;

  uint8_t done = moveCells(n, phase == PH_FAST ? SPEED_RUN_PWM : DRIVE_PWM);
  noteResult(!lastDriveStalled);
  advance(dir, done);
  if (done < n && !lastDriveStalled) putWall(posX, posY, dir, true);  // the front ToF found a wall the map didn't have
}

// --- SETUP ---
void setup() {
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
  initToFSensors();
  initMPU6050();
  calibrateGyro();

  if (digitalRead(START_BUTTON) == LOW) {
    clearMap();
    blink(3);
    while (digitalRead(START_BUTTON) == LOW);
    delay(50);
  } else if (loadMaze()) {
    posX = startX; posY = 0; facing = 0;
  } else {
    clearMap();  // nothing saved yet
  }
}

// --- MAIN LOOP ---
void loop() {
  if (runState == ST_RUN) { runStep(); return; }

  sense();
  int b = readButton();
  if (b == 3) {
    clearMap();
    blink(3);
  } else if (b == 1 && speedRunReady()) {
    beginRun(PH_FAST);
  } else if (b != 0) {
    beginRun(PH_SEARCH);
  }
}
