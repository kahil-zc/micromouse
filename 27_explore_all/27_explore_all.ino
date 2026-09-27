// ╔══════════════════════════════════════════════════════════════════╗
// ║  27_explore_all                                                  ║
// ║  Explore EVERY cell, record every wall, then drive the shortest  ║
// ║  route. 14_floodfill_final's driving and turning, unchanged.     ║
// ║                                                                  ║
// ║  SEARCH (depth-first): at every stop (one per cell) it reads the ║
// ║  3 walls and writes them down. Then it drives into a cell it     ║
// ║  hasn't been in yet (straight on first, then left, then right).  ║
// ║  If there's none, it backs up the way it came to the last cell   ║
// ║  that still has an unexplored way. When there is nothing left to ║
// ║  explore it is back in the start cell, with the whole maze       ║
// ║  recorded. Then it works out the shortest route to the goal      ║
// ║  (the 2x2 room in the far corner; either way round the maze) and ║
// ║  saves it. SPEED RUN = that route, several cells per straight,   ║
// ║  then home the same way.                                         ║
// ║  Walls: read twice at every stop, a wall counts only if both     ║
// ║  readings see it; a side it has driven through stays open. If a  ║
// ║  ToF finds a wall the record didn't have, it is recorded and the ║
// ║  route worked out again from there.                              ║
// ║                                                                  ║
// ║  ToF readings: every reading comes with a status from the        ║
// ║  sensor; failed measurements (bad status or 0 mm) are thrown     ║
// ║  away, so a flaky reading can't make a phantom wall or opening.  ║
// ║                                                                  ║
// ║  Button (START), robot in the start cell, back to the wall:      ║
// ║    short press = SEARCH if no route is saved yet, else SPEED RUN ║
// ║    long press (1 s, LED on) = SEARCH again                       ║
// ║    double click, or held while powering on = forget the route   ║
// ║  LED: 5 quick blinks = explored everything / goal reached. At    ║
// ║  power-on, 10 very quick blinks = the Nano restarted while       ║
// ║  driving (power problem).                                        ║
// ║  During a run, something wrong = PAUSE: the LED blinks SLOWLY a  ║
// ║  number of times, then stays on:                                 ║
// ║    1 = button   2 = front closer than 20 mm   3 = stuck          ║
// ║    4 = a turn didn't finish (hit something)                      ║
// ║  Put the robot in the middle of the cell it last stopped in,     ║
// ║  facing the way it was going, and press: it carries on. Long     ║
// ║  press = end the run.                                            ║
// ║                                                                  ║
// ║  USB (Serial Monitor, 115200), while it waits:                   ║
// ║    m = the recorded maze, the saved route and the last pause     ║
// ║    d = the log of the last run: every cell, what the 3 ToFs      ║
// ║        read, what it chose and what happened, and free RAM       ║
// ║    c = calibrate the side ToFs: first put the robot in the       ║
// ║        middle of a corridor with a wall on BOTH sides, straight  ║
// ║    x = forget that calibration (back to the numbers below)       ║
// ║  While it waits it also prints L F R and, after each, how many   ║
// ║  of that sensor's readings failed (e.g. "L 35/0"): a sensor that ║
// ║  keeps failing has a wiring or mounting problem.                 ║
// ║  Don't open the Serial Monitor while it drives: that restarts    ║
// ║  the Nano.                                                       ║
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
// Cells are (x, y). The robot starts in cell (0, 0) facing +y ("north", ahead); +x ("east") is
// to its right. The goal is the GOAL_SIZE x GOAL_SIZE room in the far corner of a
// MAZE_ACROSS x MAZE_AHEAD maze, or of the maze turned the other way round: the map is a
// MAP_SIZE x MAP_SIZE box, so both fit, and the real walls decide.
#define MAZE_ACROSS 10
#define MAZE_AHEAD   5
#define GOAL_SIZE    2
#define MAP_SIZE    10     // the map box (cells each way); must fit the maze either way round
#define MAX_PATH   100     // longest route (cells)

// === YOUR CALIBRATED TOF BIAS ===
// reading - bias = gap from that face of the robot to the wall. The side ones are replaced by
// the 'c' calibration (saved in EEPROM) once you have done it.
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
#define LONG_PRESS_MS       1000
#define DOUBLE_CLICK_MS     400    // second press within this = double click
#define BUTTON_PAUSE_MS     150    // button must be held this long to pause a run (noise can't)
#define SENSE_MS            150    // how long to average the ToFs when reading the walls
#define FRONT_FIX_MAX_MM    40.0f  // trust that wall for the position only this close to the encoders
#define EEPROM_MAGIC        0xA5   // marks a saved route in EEPROM

#define DRIVE_STALL 0
#define DRIVE_DONE  1
#define DRIVE_WALL  2
#define DRIVE_CRASH 3   // front closer than FRONT_EMERGENCY_MM

#define WHY_BUTTON   1   // pause reasons = number of slow blinks
#define WHY_CRASH    2
#define WHY_STUCK    3
#define WHY_TURN     4
#define EE_LOG     200  // EEPROM: last pause reason, x, y, facing, pauses this run, run flag
#define EE_CAL     210  // EEPROM: calibration marker, left bias, right bias (2 bytes each)
#define EE_MAP     220  // EEPROM: the recorded maze, MAP_SIZE x MAP_SIZE bytes
#define EE_RUN     330  // EEPROM: number of log entries, then the log, LOG_BYTES per cell
#define LOG_BYTES    7
#define LOG_MAX     95  // cells kept in the log (EEPROM is 1024 bytes)

#define TURN_OK      0
#define TURN_FAILED  1
#define TURN_BLOCKED 2

#define ST_WAIT         0
#define ST_SEARCH       1   // exploring every cell
#define ST_FAST         2   // speed run: the saved route to the goal


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
int leftBias = LEFT_BIAS_MM, rightBias = RIGHT_BIAS_MM;  // see calibrateSides()
float wallL = 999, wallF = 999, wallR = 999;  // averaged readings at the last stop (for the log)
uint16_t badL = 0, badF = 0, badR = 0;        // failed readings thrown away (see updateToF)
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
bool atCentre = false;            // put back by hand in the middle of a cell (after a pause)
float turnSideMm = 999;           // side reading toward the turn, just before an arc (prepareArc)

int runState = ST_WAIT;
bool pauseRequested = false;      // button pressed during a run
unsigned long buttonDownSince = 0; // when the button went down (0 = up)
uint8_t frontCloseCount = 0;      // front readings in a row under FRONT_EMERGENCY_MM

VL53L0X sensorLeft;
VL53L0X sensorFront;
VL53L0X sensorRight;

// These two structs carry their own functions (member functions), so no free function takes
// them as a parameter: the Arduino IDE writes its own declarations of all free functions near
// the top of the sketch, above the structs, and would then not know these types.

// --- SIDE-WALL EDGE CORRECTION ---
// Posts sit on every cell boundary. When a side reading changes between wall and gap, the side
// sensor is at a known spot (a post edge), which fixes the distance driven so far.
// Tracks one side sensor switching between wall and gap.
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
// Straight-line fit of one side reading against distance driven.
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
// Compiled on a PC by tests/micromouse_27_sim.sh and run on random mazes.
// Directions: 0 = north (+y), 1 = east (+x), 2 = south, 3 = west. Right of d is (d + 1) & 3.
const int8_t DX[4] = { 0, 1, 0, -1 };
const int8_t DY[4] = { 1, 0, -1, 0 };

// The recorded maze, one byte per cell: bits 0-3 = wall N, E, S, W; bit 4 = the robot has been
// in this cell (so its walls are known).
#define VISITED 0x10
uint8_t cellMap[MAP_SIZE][MAP_SIZE];
uint8_t drivenMap[MAP_SIZE][MAP_SIZE];  // bits 0-3: drove through that side, so it is open for sure
int8_t posX = 0, posY = 0;     // cell, counted from the start
uint8_t facing = 0;
uint8_t trail[MAX_PATH];       // the way it came, for backing up (depth-first search)
uint8_t trailLen = 0;
uint8_t pathDir[MAX_PATH];     // the saved shortest route from the start to the goal
uint8_t pathLen = 0;

bool inMap(int x, int y) { return x >= 0 && y >= 0 && x < MAP_SIZE && y < MAP_SIZE; }
bool inRoom(int x, int y, int w, int h) {
  return x >= w - GOAL_SIZE && x < w && y >= h - GOAL_SIZE && y < h;
}
bool isGoal(int x, int y) {
  return inRoom(x, y, MAZE_ACROSS, MAZE_AHEAD) || inRoom(x, y, MAZE_AHEAD, MAZE_ACROSS);
}

// A wall on both cells that share it (the map edge is always a wall). A side it has driven
// through stays open whatever is read later.
void setWall(int x, int y, uint8_t d, bool present) {
  int nx = x + DX[d], ny = y + DY[d];
  if (!inMap(nx, ny)) present = true;
  else if (present && (drivenMap[x][y] & (1 << d))) return;
  if (present) cellMap[x][y] |= 1 << d; else cellMap[x][y] &= ~(1 << d);
  if (!inMap(nx, ny)) return;
  uint8_t o = (d + 2) & 3;
  if (present) cellMap[nx][ny] |= 1 << o; else cellMap[nx][ny] &= ~(1 << o);
}

void resetSearch() {
  memset(cellMap, 0, sizeof(cellMap));
  memset(drivenMap, 0, sizeof(drivenMap));
  posX = 0; posY = 0; facing = 0;
  trailLen = 0;
  setWall(0, 0, 2, true);  // the start wall behind
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

// The three walls read here, as bits: 1 = left, 2 = front, 4 = right. Written down, and the
// cell is marked as explored. (The way it came in is open: it just drove through it.)
void recordWalls(uint8_t walls) {
  for (uint8_t i = 0; i < 3; i++) setWall(posX, posY, (facing + 3 + i) & 3, walls & (1 << i));
  cellMap[posX][posY] |= VISITED;
}

// Where to go next while exploring: an open neighbour not explored yet (straight on first, then
// left, right, back), or else back the way it came. 255 = everything reachable is explored
// (and it is back in the start cell).
uint8_t exploreDir() {
  static const uint8_t order[4] = { 0, 3, 1, 2 };
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t d = (facing + order[i]) & 3;
    int nx = posX + DX[d], ny = posY + DY[d];
    if (!(cellMap[posX][posY] & (1 << d)) && inMap(nx, ny) && !(cellMap[nx][ny] & VISITED)) return d;
  }
  if (trailLen == 0) return 255;
  return (trail[trailLen - 1] + 2) & 3;  // back up
}

// Drove one cell along d: forward into a new cell (remember the way back), or backing up.
void stepTo(uint8_t d) {
  facing = d;
  setWall(posX, posY, d, false);
  drivenMap[posX][posY] |= 1 << d;
  posX += DX[d]; posY += DY[d];
  drivenMap[posX][posY] |= 1 << ((d + 2) & 3);
  if (trailLen > 0 && d == ((trail[trailLen - 1] + 2) & 3) && (cellMap[posX][posY] & VISITED)) trailLen--;
  else if (trailLen < MAX_PATH) trail[trailLen++] = d;
}

// Shortest route from (sx, sy) to the goal (toGoal) or to the start cell, on the recorded maze
// (only explored cells; a cell nobody drove into counts as blocked). Breadth-first search;
// fills pathDir / pathLen. False = no route.
bool findRoute(int sx, int sy, bool toGoal) {
  static uint8_t from[MAP_SIZE][MAP_SIZE];  // 255 = not reached, else the direction it came in
  uint8_t queue[MAP_SIZE * MAP_SIZE], head = 0, tail = 0;
  memset(from, 255, sizeof(from));
  from[sx][sy] = 4;
  queue[tail++] = sx * MAP_SIZE + sy;
  int gx = -1, gy = -1;
  while (head < tail) {
    uint8_t c = queue[head++];
    int x = c / MAP_SIZE, y = c % MAP_SIZE;
    if (toGoal ? isGoal(x, y) : (x == 0 && y == 0)) { gx = x; gy = y; break; }
    for (uint8_t d = 0; d < 4; d++) {
      int nx = x + DX[d], ny = y + DY[d];
      if ((cellMap[x][y] & (1 << d)) || !inMap(nx, ny) || from[nx][ny] != 255) continue;
      if (!(cellMap[nx][ny] & VISITED)) continue;
      from[nx][ny] = d;
      queue[tail++] = nx * MAP_SIZE + ny;
    }
  }
  pathLen = 0;
  if (gx < 0) return false;
  uint8_t n = 0;
  for (int x = gx, y = gy; from[x][y] != 4; ) { uint8_t d = from[x][y]; x -= DX[d]; y -= DY[d]; n++; }
  if (n > MAX_PATH) return false;
  pathLen = n;
  for (int x = gx, y = gy; from[x][y] != 4; ) { uint8_t d = from[x][y]; pathDir[--n] = d; x -= DX[d]; y -= DY[d]; }
  return true;
}

// Cells to drive straight from step i of the route.
uint8_t runFrom(uint8_t i) {
  uint8_t n = 1;
  while (i + n < pathLen && pathDir[i + n] == pathDir[i]) n++;
  return n;
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
  if (readToF(sensorLeft, leftBias, tofL, accL, cntL, badL)) seqL++;
  if (readToF(sensorFront, FRONT_BIAS_MM, tofF, accF, cntF, badF) == 1)
    frontCloseCount = (tofF < FRONT_EMERGENCY_MM) ? (frontCloseCount < 3 ? frontCloseCount + 1 : 3) : 0;
  if (readToF(sensorRight, rightBias, tofR, accR, cntR, badR)) seqR++;
}

// Called in every loop: keeps the gyro integrating and the ToF values fresh.
// The button only sets a flag (held down BUTTON_PAUSE_MS without a break, so motor noise on the
// wire can't); the robot pauses when the current move ends.
void sense() {
  updateGyro();
  updateToF();
  if (runState != ST_WAIT && digitalRead(START_BUTTON) == LOW) {
    if (!buttonDownSince) buttonDownSince = millis() | 1;
    else if (millis() - buttonDownSince > BUTTON_PAUSE_MS) pauseRequested = true;
  } else {
    buttonDownSince = 0;
  }
}

// Something really close in front: two readings in a row, so one bad reading can't stop it.
bool frontCrash() { return frontCloseCount >= 2; }

void senseFor(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) { sense(); delay(2); }
}

// Averages all three good readings over ~150 ms while standing still. 999 = nothing in range
// (or no good reading at all).
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
    if (frontCrash()) {
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

// Sideways offset from the corridor centre from two side readings: > 0 = right of centre.
// NAN = no side wall to tell.
float lateralOffset(float l, float r) {
  bool hasL = l < SIDE_FOLLOW_MM, hasR = r < SIDE_FOLLOW_MM;
  if (hasL && hasR) return (l - r) / 2;
  if (hasL) return l - centreGap;
  if (hasR) return centreGap - r;
  return NAN;
}

// Before an arc: if the robot is too close to the wall the rear corner swings toward,
// shuffle toward the turn side. That shift only moves the end point along the new corridor,
// not sideways. Returns the sideways offset after any shuffle (> 0 = right of centre, NAN =
// unknown): it tells where along the new corridor the arc will end.
float prepareArc(int dir) {
  float l, f, r;
  averageAll(l, f, r);
  float off = lateralOffset(l, r);
  turnSideMm = (dir > 0) ? l : r;
  if (backToWall) return off;
  float outerGap = (dir < 0) ? l : r;  // right turn: rear corner swings toward the left wall
  float innerGap = (dir < 0) ? r : l;
  if (outerGap >= WALL_THRESHOLD_MM) return off;  // no outer wall
  float margin = outerGap + HALF_WIDTH_MM - ARC_OUTER_REACH_MM;
  if (margin >= ARC_MIN_MARGIN_MM) return off;
  float shift = ARC_TARGET_MARGIN_MM - margin;
  if (innerGap < WALL_THRESHOLD_MM) shift = min(shift, innerGap - 12.0f);  // don't hit the inner wall
  if (shift <= SHIFT_TOL_MM) return off;
  shiftSideways(dir < 0 ? -shift : shift, FRONT_GAP_MM);
  return off + (dir < 0 ? shift : -shift);  // moved right for a right turn
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
  if (atCentre && !(move == 'U' && tofF < FRONT_WALL_THRESHOLD_MM)) {
    // Put back by hand in the middle of the cell: no room for an arc, so spin on the spot.
    ok = spinSafely(move == 'L' ? 90.0f : move == 'R' ? -90.0f : 180.0f);
    carryMm = DECISION_BACK_MM;
    backToWall = false;
    return ok ? TURN_OK : TURN_FAILED;
  }
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
    if (turnSideMm < WALL_THRESHOLD_MM) return TURN_BLOCKED;  // side ToF: there's a wall after all
    ok = arcTurn(dir);
    carryMm = TURN_RADIUS_MM + DECISION_BACK_MM;  // arc ends TURN_RADIUS past the cell centre
    backToWall = false;
  }
  return ok ? TURN_OK : TURN_FAILED;
}

// Drives to the decision point numCells ahead. Returns the cells really advanced (fewer if the
// front ToF found a wall the map didn't have); -1 = stuck or about to crash.
int moveCells(int numCells, int maxPwm) {
  float carry0 = carryMm;
  int r = driveStraight(numCells * CELL_MM - carry0, maxPwm, true, true, carry0 - DECISION_BACK_MM);
  backToWall = false;
  lastDriveResult = r;
  if (r == DRIVE_STALL || r == DRIVE_CRASH) return -1;
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
}

// --- WALLS, SAVING, PRINTING ---
// The three walls here, read at the decision point (averaged ToFs): bit 0 = left, bit 1 =
// front, bit 2 = right. A wall one cell ahead also fixes the position along the corridor.
// Read twice: a wall counts only if both readings see it (a wall read as open by mistake is
// caught by the ToFs when it tries to go that way; a missed opening would hide a part of the maze).
uint8_t readWallsOnce() {
  float l, f, r;
  averageAll(l, f, r);
  wallL = l; wallF = f; wallR = r;
  bool frontWall = f < FRONT_WALL_THRESHOLD_MM - carryMm;
  float fromFront = FRONT_GAP_MM + CELL_MM - f;  // position if the wall is one cell ahead
  if (!frontWall && !backToWall && fabs(fromFront - carryMm) < FRONT_FIX_MAX_MM)
    carryMm = (carryMm + fromFront) / 2;
  if (l < SIDE_FOLLOW_MM && r < SIDE_FOLLOW_MM)
    centreGap = constrain(centreGap + 0.2f * ((l + r) / 2 - centreGap), SIDE_GAP_MM - 10, SIDE_GAP_MM + 10);
  return (l < WALL_THRESHOLD_MM) | (frontWall << 1) | ((r < WALL_THRESHOLD_MM) << 2);
}
uint8_t readWalls() { return readWallsOnce() & readWallsOnce(); }

void saveRoute() {
  EEPROM.update(0, EEPROM_MAGIC);
  EEPROM.update(1, pathLen);
  for (uint8_t i = 0; i < pathLen; i++) EEPROM.update(2 + i, pathDir[i]);
  const uint8_t *p = &cellMap[0][0];
  for (int i = 0; i < MAP_SIZE * MAP_SIZE; i++) EEPROM.update(EE_MAP + i, p[i]);
}

bool loadRoute() {
  if (EEPROM.read(0) != EEPROM_MAGIC) return false;
  pathLen = min(EEPROM.read(1), MAX_PATH);
  for (uint8_t i = 0; i < pathLen; i++) pathDir[i] = EEPROM.read(2 + i) & 3;
  uint8_t *p = &cellMap[0][0];
  for (int i = 0; i < MAP_SIZE * MAP_SIZE; i++) p[i] = EEPROM.read(EE_MAP + i);
  return pathLen > 0;
}

void printRoute() {
  Serial.print(F("Route (")); Serial.print(pathLen); Serial.print(F(" cells): "));
  for (uint8_t i = 0; i < pathLen; i++) Serial.print("NESW"[pathDir[i]]);
  Serial.println();
}

// The recorded maze: +---+ and | = wall, '#' = explored cell, blank = never been there, G = goal.
void printMaze() {
  for (int y = MAP_SIZE - 1; y >= 0; y--) {
    for (int x = 0; x < MAP_SIZE; x++) { Serial.print('+'); Serial.print((cellMap[x][y] & 1) ? F("---") : F("   ")); }
    Serial.println('+');
    for (int x = 0; x < MAP_SIZE; x++) {
      Serial.print((cellMap[x][y] & 8) ? '|' : ' ');
      Serial.print(' ');
      Serial.print(isGoal(x, y) ? 'G' : (cellMap[x][y] & VISITED) ? '#' : ' ');
      Serial.print(' ');
    }
    Serial.println('|');
  }
  for (int x = 0; x < MAP_SIZE; x++) Serial.print(F("+---"));
  Serial.println('+');
}

// --- RUN LOG (EEPROM) ---
// One entry per cell of a search: where it was, which way it faced, the walls it read, the
// averaged L / F / R readings, which way it chose and what happened. 'd' prints it.
#define LOG_MOVED    0   // drove the cell
#define LOG_BLOCKED  1   // the side ToF saw a wall where it wanted to turn: chose again
#define LOG_WALL     2   // the front ToF stopped it before the next cell
#define LOG_PAUSE    3   // paused: 3 + reason (1 button, 2 crash, 3 stuck, 4 turn)
uint8_t logCount = 0;
uint8_t lastPauseWhy = 0;

void logStep(uint8_t walls, uint8_t dir, uint8_t result) {
  if (logCount >= LOG_MAX) return;
  int a = EE_RUN + 1 + logCount * LOG_BYTES;
  EEPROM.update(a, posX); EEPROM.update(a + 1, posY);
  EEPROM.update(a + 2, (facing << 4) | walls);
  EEPROM.update(a + 3, (dir << 4) | result);
  EEPROM.update(a + 4, min(wallL, 510.0f) / 2);
  EEPROM.update(a + 5, min(wallF, 1020.0f) / 4);
  EEPROM.update(a + 6, min(wallR, 510.0f) / 2);
  EEPROM.update(EE_RUN, ++logCount);
}

// RAM check: at power-on the free RAM is filled with a pattern; the bytes still holding it were
// never needed. Near 0 = out of memory.
extern char __bss_end;
void paintRam() { char here; for (char *p = &__bss_end; p < &here - 40; p++) *p = 0x5A; }
int ramNeverUsed() { char *p = &__bss_end; while (*p == 0x5A) p++; return p - &__bss_end; }

void printLog() {
  uint8_t n = EEPROM.read(EE_RUN);
  if (n > LOG_MAX) n = 0;
  Serial.println(F("\nLOG (walls L F R, 1 = wall)"));
  Serial.println(F("  #   x  y face  walls  L mm  F mm  R mm  chose  result"));
  for (uint8_t i = 0; i < n; i++) {
    int a = EE_RUN + 1 + i * LOG_BYTES;
    uint8_t fw = EEPROM.read(a + 2), dr = EEPROM.read(a + 3), res = dr & 15;
    Serial.print(i); Serial.print(F("  ")); Serial.print((int8_t)EEPROM.read(a)); Serial.print(' ');
    Serial.print((int8_t)EEPROM.read(a + 1)); Serial.print(F("  ")); Serial.print("NESW"[(fw >> 4) & 3]);
    Serial.print(F("    ")); Serial.print(fw & 1); Serial.print((fw >> 1) & 1); Serial.print((fw >> 2) & 1);
    Serial.print(F("   ")); Serial.print(EEPROM.read(a + 4) * 2); Serial.print(F("  ")); Serial.print(EEPROM.read(a + 5) * 4);
    Serial.print(F("  ")); Serial.print(EEPROM.read(a + 6) * 2); Serial.print(F("   ")); Serial.print("NESW"[(dr >> 4) & 3]);
    Serial.print(F("  "));
    if (res == LOG_MOVED) Serial.println(F("moved"));
    else if (res == LOG_BLOCKED) Serial.println(F("BLOCKED"));
    else if (res == LOG_WALL) Serial.println(F("WALL AHEAD"));
    else { Serial.print(F("PAUSE ")); Serial.println(res - LOG_PAUSE); }
  }
  Serial.print(F("RAM never used: ")); Serial.print(ramNeverUsed()); Serial.println(F(" bytes (near 0 = out of memory)"));
}

// Side ToF calibration: the robot stands straight in the middle of a corridor with a wall on
// both sides, so each side should read SIDE_GAP_MM. The biases are changed so they do, and
// saved.
void calibrateSides() {
  float l = 0, f, r = 0, a, b;
  for (uint8_t i = 0; i < 6; i++) { averageAll(a, f, b); l += a; r += b; }
  l /= 6; r /= 6;
  if (l > 150 || r > 150) { Serial.println(F("Needs a wall on BOTH sides.")); return; }
  leftBias += (int)round(l - SIDE_GAP_MM);
  rightBias += (int)round(r - SIDE_GAP_MM);
  EEPROM.update(EE_CAL, 0xCA);
  EEPROM.put(EE_CAL + 1, (int16_t)leftBias); EEPROM.put(EE_CAL + 3, (int16_t)rightBias);
  Serial.print(F("Calibrated. Was L ")); Serial.print(l); Serial.print(F(" R ")); Serial.print(r);
  Serial.print(F(" (should be ")); Serial.print(SIDE_GAP_MM); Serial.print(F("). New biases L "));
  Serial.print(leftBias); Serial.print(F(" R ")); Serial.println(rightBias);
}

void loadCalibration() {
  if (EEPROM.read(EE_CAL) != 0xCA) return;
  int16_t a, b;
  EEPROM.get(EE_CAL + 1, a); EEPROM.get(EE_CAL + 3, b);
  if (abs(a - LEFT_BIAS_MM) < 100 && abs(b - RIGHT_BIAS_MM) < 100) { leftBias = a; rightBias = b; }
  Serial.print(F("Side ToF biases (calibrated): L ")); Serial.print(leftBias); Serial.print(F(" R ")); Serial.println(rightBias);
}

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
  delay(30);
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

void blink(uint8_t times) {
  for (uint8_t i = 0; i < times; i++) { digitalWrite(STATUS_LED, HIGH); delay(80); digitalWrite(STATUS_LED, LOW); delay(80); }
}

// Prints why the robot last paused (kept in EEPROM, so it survives switching off).
void printLastPause() {
  uint8_t why = EEPROM.read(EE_LOG);
  if (why < 1 || why > 4) return;
  Serial.print(F("Last pause: "));
  Serial.print(why == WHY_BUTTON ? F("button") : why == WHY_CRASH ? F("front closer than 20 mm")
               : why == WHY_STUCK ? F("stuck") : F("turn timeout"));
  Serial.print(F(" in cell (")); Serial.print(EEPROM.read(EE_LOG + 1)); Serial.print(',');
  Serial.print(EEPROM.read(EE_LOG + 2)); Serial.print(F(") facing ")); Serial.print("NESW"[EEPROM.read(EE_LOG + 3) & 3]);
  Serial.print(F(". Pauses that run: ")); Serial.println(EEPROM.read(EE_LOG + 4));
}

// Something went wrong (stuck, about to crash, a turn failed) or the button was pressed. Stop,
// blink `why` times slowly, LED on, and wait to be put back in the middle of the cell it last
// stopped in (posX, posY), facing `facing` (printed on USB). Short press = carry on from there, long press = end
// the run. Nothing from the failed move went into the map.
void pauseForHelp(uint8_t why) {
  stopMotors();
  lastPauseWhy = why;
  while (digitalRead(START_BUTTON) == LOW);
  delay(50);
  EEPROM.update(EE_LOG, why);
  EEPROM.update(EE_LOG + 1, posX); EEPROM.update(EE_LOG + 2, posY); EEPROM.update(EE_LOG + 3, facing);
  EEPROM.update(EE_LOG + 4, EEPROM.read(EE_LOG + 4) + 1);
  EEPROM.update(EE_LOG + 5, 0);  // not driving: switching off now is not a "restart while driving"
  Serial.println(F("\nPAUSED"));
  printLastPause();
  delay(400);
  for (uint8_t i = 0; i < why; i++) {  // slow, countable: the number = why
    digitalWrite(STATUS_LED, HIGH); delay(300); digitalWrite(STATUS_LED, LOW); delay(300);
  }
  delay(400);
  digitalWrite(STATUS_LED, HIGH);
  int b;
  do { b = readButton(); } while (b == 0);
  pauseRequested = false;
  frontCloseCount = 0;
  if (b != 1) { runState = ST_WAIT; return; }
  EEPROM.update(EE_LOG + 5, 1);  // driving again
  delay(500);  // let go of the robot
  recalGyroBias();
  targetHeading = -90.0f * facing;  // + = left, so east (1) is -90
  absoluteHeading = targetHeading;
  carryMm = DECISION_BACK_MM;       // in the middle of the cell
  backToWall = false;
  atCentre = true;
  odoMm = 0;
  resetFits();
}

// Robot is in the start cell with its back to the start wall.
void beginRun(int state) {
  delay(500);  // let go of the robot
  startAlignment();
  posX = 0; posY = 0; facing = 0;
  if (state == ST_SEARCH) { resetSearch(); logCount = 0; EEPROM.update(EE_RUN, 0); }
  pauseRequested = false;
  atCentre = false;
  EEPROM.update(EE_LOG, 0);
  EEPROM.update(EE_LOG + 4, 0);
  EEPROM.update(EE_LOG + 5, 1);  // driving: a restart before this is cleared = power problem
  runState = state;
}

// Turns to d and drives n cells. Returns the cells driven, -1 if it had to pause (the robot is
// then back in posX, posY facing `facing`, or the run was ended), -2 if the side ToF saw a wall
// where it wanted to turn (it hasn't moved).
int doMove(uint8_t d, int n, int pwm) {
  int t = executeTurn(relMove(facing, d));
  if (t == TURN_BLOCKED) return -2;
  if (t != TURN_OK) { pauseForHelp(pauseRequested ? WHY_BUTTON : WHY_TURN); return -1; }
  facing = d;
  if (pauseRequested) { pauseForHelp(WHY_BUTTON); return -1; }
  int k = moveCells(n, pwm);
  atCentre = false;
  if (k < 0) { pauseForHelp(lastDriveResult == DRIVE_CRASH ? WHY_CRASH : WHY_STUCK); return -1; }
  return k;
}

// Drives to the goal (toGoal) or back to the start on the recorded maze, the shortest way,
// several cells per straight. If the ToFs find a wall the record didn't have, it is recorded
// and the route worked out again from where the robot is. False = no route (or run ended).
bool driveRoute(bool toGoal, int pwm) {
  while (runState != ST_WAIT) {
    if (toGoal ? isGoal(posX, posY) : (posX == 0 && posY == 0)) return true;
    if (!findRoute(posX, posY, toGoal)) return false;
    uint8_t d = pathDir[0];
    int k = doMove(d, runFrom(0), pwm);
    if (k == -2 || k == 0) { if (k == 0) facing = d; setWall(posX, posY, d, true); continue; }
    for (int j = 0; j < k; j++) { posX += DX[d]; posY += DY[d]; }
  }
  return false;
}

// In the start cell: face ahead again and square up on the start wall.
void finishAtHome() {
  char m = relMove(facing, 0);
  if (m == 'U') spinSafely(180.0f);
  else if (m == 'R') spinSafely(-90.0f);
  else if (m == 'L') spinSafely(90.0f);
  facing = 0;
  startAlignment();
  EEPROM.update(EE_LOG + 5, 0);
  printLastPause();
  runState = ST_WAIT;
}

// Explored everything (back in the start cell): find the shortest route, save it, square up.
void explorationDone() {
  stopMotors();
  bool ok = findRoute(0, 0, true);
  saveRoute();
  Serial.println(F("\n*** EXPLORED EVERYTHING ***"));
  printMaze();
  if (ok) { printRoute(); blink(5); }
  else Serial.println(F("No route to the goal."));
  finishAtHome();
  Serial.println(ok ? F("\nHOME.") : F("\nHOME."));
}

// One cell of the search: read and record the walls, pick the next cell, drive there.
void searchStep() {
  uint8_t walls = readWalls();
  if (pauseRequested) { pauseForHelp(WHY_BUTTON); return; }
  recordWalls(walls);
  while (true) {
    uint8_t d = exploreDir();
    if (d == 255) { explorationDone(); return; }
    int k = doMove(d, 1, DRIVE_PWM);
    if (k == -2 || k == 0) {  // the ToFs saw a wall there after all: record it and choose again
      logStep(walls, d, k == -2 ? LOG_BLOCKED : LOG_WALL);
      if (k == 0) facing = d;
      setWall(posX, posY, d, true);
      continue;
    }
    logStep(walls, d, k == 1 ? LOG_MOVED : LOG_PAUSE + lastPauseWhy);
    if (k == 1) stepTo(d);
    return;
  }
}

// Speed run: the shortest recorded route to the goal, several cells per straight, then home.
void fastRun() {
  bool ok = driveRoute(true, SPEED_RUN_PWM);
  if (runState == ST_WAIT) return;
  if (!ok) { Serial.println(F("No route: search again.")); runState = ST_WAIT; return; }
  Serial.println(F("\n*** GOAL REACHED ***"));
  blink(5);
  driveRoute(false, DRIVE_PWM);
  if (runState == ST_WAIT) return;
  finishAtHome();
  Serial.println(F("\nHOME."));
}

// --- SETUP ---
void setup() {
  paintRam();
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
  loadCalibration();
  initToFSensors();
  initMPU6050();
  calibrateGyro();

  if (digitalRead(START_BUTTON) == LOW) {
    EEPROM.update(0, 0);  // forget the route
    blink(3);
    Serial.println(F("\nRoute cleared."));
    while (digitalRead(START_BUTTON) == LOW);
    delay(50);
  } else if (loadRoute()) {
    printRoute();
  }

  if (EEPROM.read(EE_LOG + 5) == 1) {
    EEPROM.update(EE_LOG + 5, 0);
    Serial.println(F("\n!!! RESTARTED while driving: power problem"));
    for (uint8_t i = 0; i < 10; i++) { digitalWrite(STATUS_LED, HIGH); delay(40); digitalWrite(STATUS_LED, LOW); delay(40); }
  }
  printLastPause();
  Serial.println(F("\n=== MICROMOUSE 27: EXPLORE ALL ==="));
  Serial.println(F("press=search/speed run, long=search, double=clear"));
  Serial.println(F("m=maze d=log c=calibrate x=uncalibrate"));
  Serial.print(F("RAM never used so far: ")); Serial.println(ramNeverUsed());
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
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'm') { printMaze(); printRoute(); printLastPause(); }
      else if (c == 'd') printLog();
      else if (c == 'c') calibrateSides();
      else if (c == 'x') {
        EEPROM.update(EE_CAL, 0);
        leftBias = LEFT_BIAS_MM; rightBias = RIGHT_BIAS_MM;
        Serial.println(F("Calibration forgotten."));
      }
    }
    int b = readButton();
    if (b == 3) {
      EEPROM.update(0, 0);
      pathLen = 0;
      blink(3);
      Serial.println(F("\nRoute cleared."));
    } else if (b == 1 && pathLen > 0) {
      Serial.println(F("\n--- SPEED RUN ---"));
      beginRun(ST_FAST);
    } else if (b != 0) {
      Serial.println(F("\n--- SEARCH ---"));
      beginRun(ST_SEARCH);
    }
  }
  else if (runState == ST_SEARCH) searchStep();
  else if (runState == ST_FAST) fastRun();
}
