// ╔══════════════════════════════════════════════════════════════════╗
// ║  24_micromouse_final                                             ║
// ║  14_floodfill_final's driving and turning, unchanged, with a     ║
// ║  simple flood fill that uses the ToFs all the time.              ║
// ║  Maze: 10 x 5 cells of 18 cm, start in the left corner, goal =   ║
// ║  the 2x2 room in the far corner. If you're not sure which way    ║
// ║  round the maze is, leave EITHER_WAY at 1: the map is 10 x 10    ║
// ║  and both possible goal corners count; the real outer walls,     ║
// ║  read by the ToFs, decide which one it reaches.                  ║
// ║                                                                  ║
// ║  Button (START), robot in the start cell, back to the wall:      ║
// ║    short press = SEARCH if no route is known yet, else SPEED RUN ║
// ║    long press (1 s, LED on) = SEARCH again (keeps the map)       ║
// ║    double click, or held while powering on = forget the map      ║
// ║  During a run: a press, a crash or getting stuck = PAUSE: the    ║
// ║  LED blinks twice and stays on. Put the robot in the middle of   ║
// ║  the cell shown by the arrow on the map (the cell it was in when ║
// ║  it last stopped), facing the way the arrow points, and press:   ║
// ║  it carries on from there. Long press instead = end the run.     ║
// ║                                                                  ║
// ║  The algorithm (classic flood fill):                             ║
// ║   1. At every stop read the 3 walls (ToFs, averaged). The newest ║
// ║      reading always replaces the old one, so a misread is fixed  ║
// ║      the next time it looks. Walls it has driven through stay    ║
// ║      open for good.                                              ║
// ║   2. The front ToF also looks one cell further: a wall at the    ║
// ║      end of the next cell goes in the map before it gets there,  ║
// ║      and it fixes the robot's position along the corridor.       ║
// ║   3. Flood fill: every cell gets its distance (in cells) to the  ║
// ║      goal, unseen walls counted as open.                         ║
// ║   4. Drive one cell to the open neighbour with the lowest        ║
// ║      number (straight on if it's a tie). Repeat.                 ║
// ║   5. At the goal: flood to the start and drive home the same     ║
// ║      way (it keeps mapping). If a misread ever closes every      ║
// ║      route, it forgets the walls it hasn't driven through and    ║
// ║      reads them again.                                           ║
// ║   6. Speed run: flood on seen walls only, several cells per      ║
// ║      straight.                                                   ║
// ║                                                                  ║
// ║  Also from the ToFs: after every turn it checks it's centred in  ║
// ║  the corridor (side ToFs) and shuffles back if it's more than    ║
// ║  6 mm off; before a straight, if more than 12 mm off.            ║
// ║                                                                  ║
// ║  USB (Serial Monitor, 115200): the map is printed at the goal,   ║
// ║  at home and at every pause; send 'm' while it waits to print it ║
// ║  again. Don't open the Serial Monitor while it drives: that      ║
// ║  restarts the Nano.                                              ║
// ║                                                                  ║
// ║  Movement (from 14):                                             ║
// ║   * The robot stops at a "decision point" in every cell, axle    ║
// ║     80 mm before the centre, reads the walls, runs flood fill.   ║
// ║   * 90° turns are 80 mm-radius arcs (spinning in place would     ║
// ║     hit: the rear corners swing 93 mm, the corridor is 84 mm).   ║
// ║   * U-turns shuffle sideways so the long rear gets the room,     ║
// ║     spin, and in a dead end back into the wall to square up.     ║
// ║                                                                  ║
// ║  How the ToFs are used while driving:                            ║
// ║   * Sides, every 5 ms: steer toward the corridor centre.         ║
// ║   * Sides, every ~60 mm: measure the real angle to the walls     ║
// ║     and correct the gyro, so it stays straight in open cells.    ║
// ║   * Sides: wall start/end edges correct the distance driven.     ║
// ║   * Sides: closer than 15 mm = steer hard away and slow down.    ║
// ║   * Front: stop at the right spot, emergency stop under 20 mm.   ║
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
// to its right. The maze is MAZE_ACROSS cells to the right and MAZE_AHEAD cells ahead; the goal
// is the GOAL_SIZE x GOAL_SIZE room in the far corner.
#define MAZE_ACROSS 10
#define MAZE_AHEAD   5
#define GOAL_SIZE    2
// 1 = not sure which way round: map a square box and also count the goal corner of the maze
// turned the other way (MAZE_AHEAD across, MAZE_ACROSS ahead). The outer walls it reads decide.
// 0 = the maze is exactly MAZE_ACROSS x MAZE_AHEAD.
#define EITHER_WAY   1

#if EITHER_WAY
#define MAP_W (MAZE_ACROSS > MAZE_AHEAD ? MAZE_ACROSS : MAZE_AHEAD)
#define MAP_H MAP_W
#else
#define MAP_W MAZE_ACROSS
#define MAP_H MAZE_AHEAD
#endif

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
#define LONG_PRESS_MS       1000
#define DOUBLE_CLICK_MS     400    // second press within this = double click
#define BUTTON_PAUSE_COUNT  5      // button must read pressed this many times in a row to pause
#define SENSE_MS            150    // how long to average the ToFs when reading the walls
#define AHEAD_TOL_MM        50     // a wall one cell ahead reads FRONT_GAP + 180, give or take this
#define FRONT_FIX_MAX_MM    40.0f  // trust that wall for the position only this close to the encoders
#define CENTRE_TOL_MM       12.0f  // before a straight, shuffle to the corridor centre if this far off
#define POST_TURN_TOL_MM     6.0f  // after a turn, shuffle to the corridor centre if this far off
#define EEPROM_MAGIC        (0xC0 ^ MAP_W ^ (MAP_H << 4) ^ GOAL_SIZE)

#define DRIVE_STALL 0
#define DRIVE_DONE  1
#define DRIVE_WALL  2
#define DRIVE_CRASH 3   // front closer than FRONT_EMERGENCY_MM

#define TURN_OK      0
#define TURN_FAILED  1
#define TURN_BLOCKED 2

#define ST_WAIT         0
#define ST_SEARCH_GOAL  1
#define ST_SEARCH_HOME  2
#define ST_FAST         3

#if MAP_W > 16 || MAP_H > 16
#error "the maze must be 16 x 16 or less"
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
float odoMm = 0;                  // distance driven straight since the last turn
bool backToWall = false;          // rear is against a wall: never reverse
bool atCentre = false;            // put back by hand in the middle of a cell (after a pause)
bool justTurned = false;          // the last move was a turn: centre at the next stop
float lastL = 999, lastR = 999;   // side readings at the last stop
float turnSideMm = 999;           // side reading toward the turn, just before an arc (prepareArc)

int runState = ST_WAIT;
bool pauseRequested = false;      // button pressed during a run
uint8_t buttonLowCount = 0;       // sense() calls in a row with the button down
uint8_t frontCloseCount = 0;      // front readings in a row under FRONT_EMERGENCY_MM

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
// Compiled on a PC by tests/micromouse_24_sim.sh and run on random mazes.
// Directions: 0 = north (+y), 1 = east (+x), 2 = south, 3 = west. Right of d is (d + 1) & 3.
const int8_t DX[4] = { 0, 1, 0, -1 };
const int8_t DY[4] = { 1, 0, -1, 0 };

uint8_t maze[MAP_W][MAP_H];    // bits 0-3: wall N,E,S,W   bits 4-7: that wall has been seen
uint8_t driven[MAP_W][MAP_H];  // bits 0-3: the robot drove through that side, so it is open
uint8_t dist[MAP_W][MAP_H];    // flood-fill distance to the target, 255 = unreachable
int8_t posX = 0, posY = 0;
uint8_t facing = 0;

bool inMaze(int x, int y) { return x >= 0 && y >= 0 && x < MAP_W && y < MAP_H; }

bool inRoom(int x, int y, int w, int h) {
  return x >= w - GOAL_SIZE && x < w && y >= h - GOAL_SIZE && y < h;
}
bool isGoal(int x, int y) {
  if (inRoom(x, y, MAZE_ACROSS, MAZE_AHEAD)) return true;
  return EITHER_WAY && inRoom(x, y, MAZE_AHEAD, MAZE_ACROSS);  // the maze the other way round
}

// Records one wall on both cells that share it. The newest reading wins, except that a wall
// the robot has driven through stays open, and the edge of the map is always a wall.
void setWall(int x, int y, uint8_t d, bool present) {
  int nx = x + DX[d], ny = y + DY[d];
  if (!inMaze(nx, ny)) present = true;
  else if (present && (driven[x][y] & (1 << d))) return;
  uint8_t o = (d + 2) & 3;
  maze[x][y] = (maze[x][y] | (0x10 << d)) & ~(1 << d);
  if (present) maze[x][y] |= 1 << d;
  if (!inMaze(nx, ny)) return;
  maze[nx][ny] = (maze[nx][ny] | (0x10 << o)) & ~(1 << o);
  if (present) maze[nx][ny] |= 1 << o;
}

void initMaze() {
  memset(maze, 0, sizeof(maze));
  memset(driven, 0, sizeof(driven));
  for (int x = 0; x < MAP_W; x++) { setWall(x, 0, 2, true); setWall(x, MAP_H - 1, 0, true); }
  for (int y = 0; y < MAP_H; y++) { setWall(0, y, 3, true); setWall(MAP_W - 1, y, 1, true); }
}

// pessimistic: unseen walls count as walls (speed run); otherwise as open (search).
bool blocked(int x, int y, uint8_t d, bool pessimistic) {
  uint8_t c = maze[x][y];
  if (c & (1 << d)) return true;
  return pessimistic && !(c & (0x10 << d));
}

// Breadth-first flood from the goal (toGoal) or the start cell.
void flood(bool toGoal, bool pessimistic) {
  static uint8_t queue[MAP_W * MAP_H];
  memset(dist, 255, sizeof(dist));
  uint8_t head = 0, tail = 0;
  for (int x = 0; x < MAP_W; x++) {
    for (int y = 0; y < MAP_H; y++) {
      if (toGoal ? isGoal(x, y) : (x == 0 && y == 0)) {
        dist[x][y] = 0;
        queue[tail++] = x * MAP_H + y;
      }
    }
  }
  while (head < tail) {
    uint8_t c = queue[head++];
    int x = c / MAP_H, y = c % MAP_H;
    for (uint8_t d = 0; d < 4; d++) {
      if (blocked(x, y, d, pessimistic)) continue;
      int nx = x + DX[d], ny = y + DY[d];
      if (!inMaze(nx, ny) || dist[nx][ny] != 255) continue;
      dist[nx][ny] = dist[x][y] + 1;
      queue[tail++] = nx * MAP_H + ny;
    }
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
    if (!inMaze(nx, ny)) continue;
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

// Cells to drive straight from (x, y) along d before the flood path turns (at least 1).
int straightRun(int x, int y, uint8_t d, bool pessimistic) {
  int n = 1;
  x += DX[d]; y += DY[d];
  while (dist[x][y] != 0 && bestDir(x, y, d, pessimistic) == d) { x += DX[d]; y += DY[d]; n++; }
  return n;
}

// The three walls read at a stop, as bits: 1 = left, 2 = front, 4 = right.
void recordWalls(uint8_t walls) {
  for (uint8_t i = 0; i < 3; i++) setWall(posX, posY, (facing + 3 + i) & 3, walls & (1 << i));
}

// What the front ToF sees beyond this cell's (open) front: 1 = the wall at the far end of the
// next cell, 2 = that wall is clearly not there, 0 = can't tell. A "not there" is only used
// for a wall not seen yet (a far reading is less sure than a close one).
void recordAhead(uint8_t ahead) {
  int nx = posX + DX[facing], ny = posY + DY[facing];
  if (!inMaze(nx, ny) || ahead == 0) return;
  if (ahead == 1) setWall(nx, ny, facing, true);
  else if (!(maze[nx][ny] & (0x10 << facing))) setWall(nx, ny, facing, false);
}

// Drove n cells along d: those walls are open for good.
void advance(uint8_t d, int n) {
  for (int i = 0; i < n; i++) {
    setWall(posX, posY, d, false);
    driven[posX][posY] |= 1 << d;
    posX += DX[d]; posY += DY[d];
    driven[posX][posY] |= 1 << ((d + 2) & 3);
  }
}

// A misread has closed every route: forget every wall the robot hasn't driven through (the map
// edge stays), so it reads them again on the way.
void forgetWalls() {
  for (int x = 0; x < MAP_W; x++)
    for (int y = 0; y < MAP_H; y++)
      for (uint8_t d = 0; d < 4; d++)
        if (inMaze(x + DX[d], y + DY[d]) && !(driven[x][y] & (1 << d)))
          maze[x][y] &= ~((1 << d) | (0x10 << d));
}

// Flood toward the target and pick the next heading from here (search: one cell at a time).
// 254 = on the target, 255 = no route (the map was then forgotten, see forgetWalls).
uint8_t nextDir(bool toGoal) {
  flood(toGoal, false);
  if (dist[posX][posY] == 0) return 254;
  if (dist[posX][posY] == 255) { forgetWalls(); return 255; }
  return bestDir(posX, posY, facing, false);
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
    frontCloseCount = (tofF < FRONT_EMERGENCY_MM) ? (frontCloseCount < 3 ? frontCloseCount + 1 : 3) : 0;
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
// The button only sets a flag (pressed for several calls in a row, so motor noise can't); the
// robot pauses when the current move ends.
void sense() {
  updateGyro();
  updateToF();
  if (runState != ST_WAIT && digitalRead(START_BUTTON) == LOW) {
    if (++buttonLowCount >= BUTTON_PAUSE_COUNT) pauseRequested = true;
  } else {
    buttonLowCount = 0;
  }
}

// Something really close in front: two readings in a row, so one bad reading can't stop it.
bool frontCrash() { return frontCloseCount >= 2; }

void senseFor(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) { sense(); delay(2); }
}

// Averages all three readings over ~150 ms while standing still. 999 = nothing in range.
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

// Shuffles back to the corridor centre (side ToFs, from the last stop's reading) if the robot
// is more than tol off it. Not with its back against a wall (no room to reverse).
void centreInCorridor(float tol) {
  if (backToWall) return;
  float off = lateralOffset(lastL, lastR);  // > 0: right of centre, so move left
  if (isnan(off) || fabs(off) <= tol) return;
  sense();
  bool front = tofF < FRONT_WALL_THRESHOLD_MM;
  shiftSideways(off, FRONT_GAP_MM);
  if (front) carryMm = 0;  // it pulled up to the front wall
  lastL -= off; lastR += off;
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
    justTurned = true;
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
    float e = prepareArc(dir);
    if (turnSideMm < WALL_THRESHOLD_MM) return TURN_BLOCKED;
    ok = arcTurn(dir);
    // The arc ends TURN_RADIUS past the cell centre; starting it off centre moves that end
    // along the new corridor (right of centre: further for a right turn, shorter for a left).
    if (isnan(e)) e = 0;
    carryMm = TURN_RADIUS_MM + DECISION_BACK_MM - dir * constrain(e, -25.0f, 25.0f);
    backToWall = false;
    justTurned = true;
  }
  return ok ? TURN_OK : TURN_FAILED;
}

// Drives to the decision point numCells ahead. Returns the cells really advanced (fewer if the
// front ToF found a wall the map didn't have); -1 = stuck or about to crash.
int moveCells(int numCells, int maxPwm) {
  Serial.print(F("Forward cells: ")); Serial.println(numCells);
  float carry0 = carryMm;
  int r = driveStraight(numCells * CELL_MM - carry0, maxPwm, true, true, carry0 - DECISION_BACK_MM);
  backToWall = false;
  if (r == DRIVE_STALL || r == DRIVE_CRASH) return -1;
  float axle = carry0 + lastMovedMm;  // from the start cell's decision point
  int k = constrain((int)round(axle / CELL_MM), 0, numCells);
  carryMm = (r == DRIVE_WALL) ? 0 : axle - k * CELL_MM;
  lastL = tofL; lastR = tofR;  // fresh side readings where it stopped (for centreInCorridor)
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
}

// --- MAP: WALL SENSING, SAVING, PRINTING ---
// Reads the walls of the current cell from its decision point (averaged ToFs), and what the
// front ToF sees one cell further on. A wall one cell ahead also gives the robot's position
// along the corridor (met halfway with the encoders).
void senseWallsHere() {
  float l, f, r;
  averageAll(l, f, r);
  float frontLimit = FRONT_WALL_THRESHOLD_MM - carryMm;
  bool frontWall = f < frontLimit;
  recordWalls((l < WALL_THRESHOLD_MM) | (frontWall << 1) | ((r < WALL_THRESHOLD_MM) << 2));
  if (!frontWall) {
    float nextWall = FRONT_GAP_MM + CELL_MM - carryMm;  // reading if the next cell ends in a wall
    uint8_t ahead = 0;
    if (fabs(f - nextWall) < AHEAD_TOL_MM) ahead = 1;
    else if (f > nextWall + 2 * AHEAD_TOL_MM) ahead = 2;
    recordAhead(ahead);
    float fromFront = FRONT_GAP_MM + CELL_MM - f;
    if (ahead == 1 && !backToWall && fabs(fromFront - carryMm) < FRONT_FIX_MAX_MM)
      carryMm = (carryMm + fromFront) / 2;
  }
  if (l < SIDE_FOLLOW_MM && r < SIDE_FOLLOW_MM)
    centreGap = constrain(centreGap + 0.2f * ((l + r) / 2 - centreGap), SIDE_GAP_MM - 10, SIDE_GAP_MM + 10);
  lastL = l; lastR = r;
}

void saveMaze() {
  EEPROM.update(0, EEPROM_MAGIC);
  const uint8_t *p = &maze[0][0];
  for (int i = 0; i < MAP_W * MAP_H; i++) EEPROM.update(1 + i, p[i]);
}

bool loadMaze() {
  if (EEPROM.read(0) != EEPROM_MAGIC) return false;
  uint8_t *p = &maze[0][0];
  for (int i = 0; i < MAP_W * MAP_H; i++) p[i] = EEPROM.read(1 + i);
  return true;
}

// ASCII map: +---+ and | = wall, blank = open, . and : = not seen yet. ^ > v < = the robot,
// G = goal.
void printMaze() {
  static const char arrow[4] = { '^', '>', 'v', '<' };
  for (int y = MAP_H - 1; y >= 0; y--) {
    for (int x = 0; x < MAP_W; x++) {
      Serial.print('+');
      uint8_t c = maze[x][y];
      Serial.print((c & 1) ? F("---") : (c & 0x10) ? F("   ") : F(" . "));
    }
    Serial.println('+');
    for (int x = 0; x < MAP_W; x++) {
      uint8_t c = maze[x][y];
      Serial.print((c & 8) ? '|' : (c & 0x80) ? ' ' : ':');
      Serial.print(' ');
      if (x == posX && y == posY) Serial.print(arrow[facing]);
      else if (isGoal(x, y)) Serial.print('G');
      else Serial.print(' ');
      Serial.print(' ');
    }
    Serial.println('|');
  }
  for (int x = 0; x < MAP_W; x++) Serial.print(F("+---"));
  Serial.println('+');
}

// Prints the shortest-path lengths. Equal = the known path is proven shortest.
void reportPath() {
  flood(true, false);
  uint8_t optimistic = dist[0][0];
  flood(true, true);
  uint8_t known = dist[0][0];
  Serial.print(F("Shortest possible: ")); Serial.print(optimistic);
  Serial.print(F(" cells, known path: "));
  if (known == 255) Serial.println(F("none yet"));
  else { Serial.print(known); Serial.println(known == optimistic ? F(" (proven shortest)") : F(" (search again to improve)")); }
}

bool speedRunReady() {
  flood(true, true);
  return dist[0][0] != 255;
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

// Something went wrong (stuck, about to crash, a turn failed) or the button was pressed. Stop,
// blink twice, LED on, and wait to be put back in the middle of the cell it last stopped in
// (posX, posY), facing `facing`. Short press = carry on from there, long press = end the run.
// Nothing from the failed move went into the map.
void pauseForHelp() {
  stopMotors();
  while (digitalRead(START_BUTTON) == LOW);
  delay(50);
  saveMaze();
  Serial.println(F("\nPAUSED: put it in the arrow's cell, middle, facing the arrow. Press = go on, long = end."));
  printMaze();
  blink(2);
  digitalWrite(STATUS_LED, HIGH);
  int b;
  do { b = readButton(); } while (b == 0);
  pauseRequested = false;
  frontCloseCount = 0;
  if (b != 1) { runState = ST_WAIT; return; }
  delay(500);  // let go of the robot
  recalGyroBias();
  targetHeading = -90.0f * facing;  // + = left, so east (1) is -90
  absoluteHeading = targetHeading;
  carryMm = DECISION_BACK_MM;       // in the middle of the cell
  backToWall = false;
  atCentre = true;
  justTurned = false;
  odoMm = 0;
  resetFits();
}

// Robot is in the start cell with its back to the start wall.
void beginRun(int state) {
  delay(500);  // let go of the robot
  startAlignment();
  posX = 0; posY = 0; facing = 0;
  memset(driven, 0, sizeof(driven));
  setWall(0, 0, 2, true);
  pauseRequested = false;
  atCentre = false;
  justTurned = false;
  lastL = lastR = 999;
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
  reportPath();
  Serial.println(F("\nHOME. Press = speed run, long press = search again."));
  runState = ST_WAIT;
}

// Turns to d and drives n cells. Returns false if it had to pause (the robot is then back in
// posX, posY facing `facing`, or the run was ended).
bool doMove(uint8_t d, int n, int pwm) {
  char m = relMove(facing, d);
  if (!atCentre) {
    if (justTurned && m != 'U') centreInCorridor(POST_TURN_TOL_MM);  // ToF check after every turn
    else if (m == 'S') centreInCorridor(CENTRE_TOL_MM);
  }
  justTurned = false;
  int t = executeTurn(m);
  if (t == TURN_BLOCKED) {  // the map was wrong: a wall there after all. Plan again.
    setWall(posX, posY, d, true);
    return true;
  }
  if (t != TURN_OK) { pauseForHelp(); return false; }  // turn failed: still facing the old way
  facing = d;
  if (pauseRequested) { pauseForHelp(); return false; }
  int k = moveCells(n, pwm);
  atCentre = false;
  if (k < 0) { pauseForHelp(); return false; }  // stuck or about to crash: last stop, facing d
  advance(d, k);
  if (k < n) setWall(posX, posY, d, true);  // the front ToF found a wall the map didn't have
  if (pauseRequested) { pauseForHelp(); return false; }  // stopped in the new cell: resume there
  return true;
}

// One cell of a search: read walls, flood, turn toward the lowest neighbour, drive one cell.
void searchStep(bool toGoal) {
  senseWallsHere();
  if (pauseRequested) { pauseForHelp(); return; }
  uint8_t d = nextDir(toGoal);
  if (d == 255) {
    Serial.println(F("No route - a wall was misread. Reading them again."));
    return;
  }
  if (d == 254) {
    if (toGoal) {
      Serial.println(F("\n*** GOAL REACHED *** heading home"));
      saveMaze();
      printMaze();
      blink(5);
      runState = ST_SEARCH_HOME;
    } else {
      finishAtHome();
    }
    return;
  }
  if (doMove(d, 1, DRIVE_PWM)) saveMaze();
}

// One straight of the fast run on known walls only.
void fastStep() {
  flood(true, true);
  if (dist[posX][posY] == 255) {
    Serial.println(F("No known path from here - searching instead."));
    runState = ST_SEARCH_GOAL;
    return;
  }
  if (dist[posX][posY] == 0) {
    Serial.println(F("\n*** FAST RUN DONE *** heading home"));
    blink(5);
    runState = ST_SEARCH_HOME;
    return;
  }
  uint8_t d = bestDir(posX, posY, facing, true);
  doMove(d, straightRun(posX, posY, d, true), SPEED_RUN_PWM);
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
    saveMaze();
    blink(3);
    Serial.println(F("\nMap cleared."));
    while (digitalRead(START_BUTTON) == LOW);
    delay(50);
  } else if (loadMaze()) {
    Serial.println(F("\nSaved map loaded:"));
    printMaze();
    reportPath();
  } else {
    initMaze();
  }

  Serial.println(F("\n=== MICROMOUSE 24 ==="));
  Serial.println(F("Press = search (speed run once a route is known), long = search, double = clear map, 'm' = map."));
}

// --- MAIN LOOP ---
void loop() {
  if (runState == ST_WAIT) {
    // Print the sensors while waiting so the calibration can be checked.
    static unsigned long lastPrint = 0;
    sense();
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      Serial.print(F("L ")); Serial.print(tofL);
      Serial.print(F("  F ")); Serial.print(tofF);
      Serial.print(F("  R ")); Serial.println(tofR);
    }
    if (Serial.available() && Serial.read() == 'm') { printMaze(); reportPath(); }
    int b = readButton();
    if (b == 3) {
      initMaze();
      saveMaze();
      blink(3);
      Serial.println(F("\nMap cleared."));
    } else if (b == 1 && speedRunReady()) {
      Serial.println(F("\n--- FAST RUN ---"));
      beginRun(ST_FAST);
    } else if (b != 0) {
      Serial.println(F("\n--- SEARCH ---"));
      beginRun(ST_SEARCH_GOAL);
    }
  }
  else if (runState == ST_SEARCH_GOAL) searchStep(true);
  else if (runState == ST_SEARCH_HOME) searchStep(false);
  else if (runState == ST_FAST)        fastStep();
}
