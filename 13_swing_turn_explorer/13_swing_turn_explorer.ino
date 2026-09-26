// ╔══════════════════════════════════════════════════════════════════╗
// ║  13_swing_turn_explorer                                          ║
// ║  Right-hand explore -> return home -> speed run, sized for a     ║
// ║  90 x 135 mm robot.                                              ║
// ║                                                                  ║
// ║  Turning in place needs a 162 mm circle for this body, and the   ║
// ║  corridor is 168 mm, so spins scrape the walls. Instead:         ║
// ║   * 90° turns pivot on the INNER wheel, started TURN_BACKOFF_MM  ║
// ║     before the cell centre. The pivot sits on the turning side   ║
// ║     (the "hug the turn side + back up" idea) without having to   ║
// ║     slide sideways, and the turn ends centred in the new         ║
// ║     corridor.                                                    ║
// ║   * U-turns (dead ends, nowhere to swing) shuffle to the exact   ║
// ║     centre first, then spin toward the side with more room.      ║
// ╚══════════════════════════════════════════════════════════════════╝

#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>

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

// === YOUR CALIBRATED TOF BIAS ===
// reading - bias = gap from that face of the robot to the wall
#define FRONT_BIAS_MM  37
#define LEFT_BIAS_MM   25
#define RIGHT_BIAS_MM  50

// === ROBOT GEOMETRY — MEASURE THE TWO MARKED VALUES ===
#define ROBOT_WIDTH_MM     90.0f
#define ROBOT_LENGTH_MM   135.0f
#define AXLE_TO_FRONT_MM   67.5f   // MEASURE: wheel axle centre line -> front edge of the robot
#define TRACK_WIDTH_MM     80.0f   // MEASURE: middle of left tyre -> middle of right tyre

// === MAZE ===
#define CELL_MM           180.0f
#define HALF_CORRIDOR_MM   84.0f   // cell centre -> wall face, (180 - 12 mm wall) / 2

// Derived geometry. The robot's position is its axle midpoint (the point it spins about).
// Decision point = axle TURN_BACKOFF_MM before the cell centre. The robot stops there in every
// cell; a swing turn started there ends centred in the new corridor.
#define HALF_WIDTH_MM      (ROBOT_WIDTH_MM / 2.0f)
#define HALF_TRACK_MM      (TRACK_WIDTH_MM / 2.0f)
#define AXLE_TO_REAR_MM    (ROBOT_LENGTH_MM - AXLE_TO_FRONT_MM)
#define TURN_BACKOFF_MM    HALF_TRACK_MM
#define SIDE_GAP_MM        (HALF_CORRIDOR_MM - HALF_WIDTH_MM)                       // centred side reading, ~39
#define FRONT_GAP_MM       (HALF_CORRIDOR_MM + TURN_BACKOFF_MM - AXLE_TO_FRONT_MM)  // front reading at decision point, ~56

// === WALL DETECTION ===
#define WALL_THRESHOLD_MM   130                 // wall present in this cell
#define SIDE_FOLLOW_MM      (SIDE_GAP_MM + 35)  // only centre on side walls closer than this

// === DRIVING ===
#define TICKS_PER_CELL      290
#define TICKS_PER_MM        (TICKS_PER_CELL / CELL_MM)
#define DRIVE_PWM           160
#define SPEED_RUN_PWM       220
#define MIN_DRIVE_PWM        80
#define ACCEL_PWM_PER_MM    2.0f   // ramp up over the first ~40 mm
#define DECEL_PWM_PER_MM    1.5f   // ramp down over the last ~50 mm
#define STALL_MS            500

// === STEERING (gyro holds heading, side walls nudge the heading target) ===
#define KP                  5.0f
#define KD                  0.2f
#define WALL_KP             0.25f  // degrees of heading nudge per mm of lateral error
#define MAX_WALL_NUDGE_DEG  5.0f

// === TURNS ===
#define KP_SWING            3.0f
#define SWING_MIN_PWM       70     // one wheel does all the work, needs more than a spin
#define SWING_MAX_PWM       150
#define KP_SPIN             2.5f
#define SPIN_MIN_PWM        60
#define SPIN_MAX_PWM        100
#define TURN_TOL_DEG        1.5f
#define TURN_SETTLE_DPS     20.0f
#define TURN_TIMEOUT_MS     2500
#define ALIGN_TOL_MM        2
#define CENTER_TOL_MM       1.5f   // re-centre before a U-turn if further off than this
#define SHIFT_ANGLE_DEG     12.0f  // heading used for the sideways shuffle
#define MAX_SHIFT_MM        15.0f

#define GYRO_SCALE          65.5f  // 500 deg/s range
#define GYRO_CALIB_SAMPLES  200
#define MAX_PATH            200
#define MAX_FAILS           3      // consecutive failed moves before giving up

#define DRIVE_STALL 0
#define DRIVE_DONE  1
#define DRIVE_WALL  2

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
long accL = 0, accR = 0;          // side reading sums for averageSides()
uint16_t cntL = 0, cntR = 0;

float carryMm = 0;                // axle position past this cell's decision point
float lastOvershootMm = 0;
bool atStartWall = false;         // back is against the start wall: never reverse
int failCount = 0;

int runState = 0; // 0=Wait, 1=Explore, 2=Return Home, 3=Wait Speedrun, 4=Speedrun
bool abortExploration = false;

char path[MAX_PATH];
int pathLength = 0;
char returnPath[MAX_PATH];

VL53L0X sensorLeft;
VL53L0X sensorFront;
VL53L0X sensorRight;

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
float travelledMm() {
  noInterrupts(); long l = encoderLeftTicks, r = encoderRightTicks; interrupts();
  return (labs(l) + labs(r)) / 2.0f / TICKS_PER_MM;  // abs of each: works whichever way the encoders count
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
    if (tofL != 999) { accL += tofL; cntL++; }
  }
  if (sensorFront.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
    uint16_t r = sensorFront.readRangeContinuousMillimeters();
    tofF = (r > 8000) ? 999 : (int)r - FRONT_BIAS_MM;
  }
  if (sensorRight.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
    uint16_t r = sensorRight.readRangeContinuousMillimeters();
    tofR = (r > 8000) ? 999 : (int)r - RIGHT_BIAS_MM;
    if (tofR != 999) { accR += tofR; cntR++; }
  }
}

// Called in every loop: keeps the gyro integrating and the ToF values fresh.
// The button only sets a flag; the current cell always finishes first.
void sense() {
  updateGyro();
  updateToF();
  if (runState == 1 && digitalRead(START_BUTTON) == LOW) abortExploration = true;
}

void senseFor(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) { sense(); delay(2); }
}

// Averages the side readings over ~150 ms while standing still.
void averageSides(float &l, float &r) {
  accL = accR = 0; cntL = cntR = 0;
  senseFor(150);
  l = cntL ? (float)accL / cntL : 999;
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
    Serial.println(F("TOO MANY FAILURES - stopping. Press START to restart."));
    digitalWrite(STATUS_LED, HIGH);
    runState = 0;
    failCount = 0;
  }
}

// --- STEERING ---
// Gyro holds targetHeading; side walls only nudge that target by a few degrees.
float headingCorrection(bool useWalls) {
  float desired = targetHeading;
  if (useWalls) {
    bool hasL = tofL < SIDE_FOLLOW_MM;
    bool hasR = tofR < SIDE_FOLLOW_MM;
    float lateralErr = 0;                          // > 0: robot is right of centre
    if (hasL && hasR) lateralErr = (tofL - tofR) / 2.0f;
    else if (hasL)    lateralErr = tofL - SIDE_GAP_MM;
    else if (hasR)    lateralErr = SIDE_GAP_MM - tofR;
    desired += constrain(lateralErr * WALL_KP, -MAX_WALL_NUDGE_DEG, MAX_WALL_NUDGE_DEG);
  }
  return KP * (desired - absoluteHeading) - KD * gyroRate;
}

// --- MOVEMENT BEHAVIOURS ---
// Drives distMm (negative = reverse) holding targetHeading. With stopAtWall it also stops at the
// decision point of a front wall. Returns DRIVE_STALL, DRIVE_DONE or DRIVE_WALL.
int driveStraight(float distMm, int maxPwm, bool stopAtWall) {
  bool reverse = distMm < 0;
  float goal = fabs(distMm);
  int result = DRIVE_DONE;
  resetTicks();
  float lastDone = 0;
  unsigned long lastProgress = millis(), lastLoop = 0;

  while (true) {
    sense();
    float done = travelledMm();
    float remaining = goal - done;
    bool wallLimited = false;
    if (stopAtWall && !reverse && tofF < WALL_THRESHOLD_MM) {
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
      if (reverse) pwm = -pwm;
      float c = headingCorrection(!reverse);  // wall nudges would steer the wrong way in reverse
      setMotors((int)(pwm - c), (int)(pwm + c));
    }
  }
  stopMotors();
  senseFor(80);
  lastOvershootMm = travelledMm() - goal;
  return result;
}

// Creeps to exactly FRONT_GAP_MM from the front wall, holding heading.
void alignToFrontWall() {
  unsigned long start = millis();
  while (millis() - start < 1000) {
    sense();
    if (tofF >= WALL_THRESHOLD_MM) break;
    float err = tofF - FRONT_GAP_MM;
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

// Rotates to targetHeading. pivot: 0 = spin in place, +1 = pivot on the left wheel (left turn),
// -1 = pivot on the right wheel (right turn).
bool rotateToTarget(int pivot) {
  float kp = pivot ? KP_SWING : KP_SPIN;
  int minPwm = pivot ? SWING_MIN_PWM : SPIN_MIN_PWM;
  int maxPwm = pivot ? SWING_MAX_PWM : SPIN_MAX_PWM;
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
    int pwm = constrain((int)(kp * fabs(err)), minPwm, maxPwm);
    if (fabs(err) < TURN_TOL_DEG) pwm = 0;  // in tolerance: brake and let it settle
    int s = (err > 0) ? pwm : -pwm;         // + = rotate counter-clockwise
    if (pivot == 0)     setMotors(-s, s);
    else if (pivot > 0) setMotors(0, s);    // left turn: right wheel drives around the left wheel
    else                setMotors(-s, 0);   // right turn: left wheel drives around the right wheel
    delay(2);
  }
  stopMotors();
  senseFor(80);
  return true;
}

bool spinTurn(float angleDeg) {
  targetHeading += angleDeg;
  return rotateToTarget(0);
}

// dir: +1 = left, -1 = right
bool swingTurn(int dir) {
  Serial.println(dir > 0 ? F("Swing LEFT") : F("Swing RIGHT"));
  targetHeading += 90.0f * dir;
  return rotateToTarget(dir);
}

// Moves the robot sideways without turning the body far: tilt, reverse, tilt back, return.
// mm > 0 = move left.
void shiftSideways(float mm) {
  mm = constrain(mm, -MAX_SHIFT_MM, MAX_SHIFT_MM);
  Serial.print(F("Shuffle sideways mm: ")); Serial.println(mm);
  float base = targetHeading;
  float back = fabs(mm) / sin(SHIFT_ANGLE_DEG * DEG_TO_RAD);
  spinTurn(mm > 0 ? -SHIFT_ANGLE_DEG : SHIFT_ANGLE_DEG);  // point the tail at the side we want
  driveStraight(-back, 90, false);
  targetHeading = base;
  rotateToTarget(0);
  sense();
  if (tofF < WALL_THRESHOLD_MM) alignToFrontWall();
  else driveStraight(back * cos(SHIFT_ANGLE_DEG * DEG_TO_RAD), 90, false);
}

// Dead end: spinning this body has only ~3 mm per side to spare, so centre exactly first,
// then spin the way that gives the longer end of the robot the bigger gap.
bool uTurn() {
  Serial.println(F("U-TURN"));
  float l, r;
  averageSides(l, r);
  if (l < WALL_THRESHOLD_MM && r < WALL_THRESHOLD_MM) {
    float offset = (l - r) / 2.0f;  // > 0: robot is right of centre, move left
    if (fabs(offset) > CENTER_TOL_MM) { shiftSideways(offset); averageSides(l, r); }
  }
  float leftRoom  = (l < WALL_THRESHOLD_MM ? l : 150) + HALF_WIDTH_MM;
  float rightRoom = (r < WALL_THRESHOLD_MM ? r : 150) + HALF_WIDTH_MM;
  float rf = sqrt(sq(AXLE_TO_FRONT_MM) + sq(HALF_WIDTH_MM));
  float rr = sqrt(sq(AXLE_TO_REAR_MM) + sq(HALF_WIDTH_MM));
  // Clockwise: front corners sweep the right wall, rear corners the left wall.
  float cwMargin  = min(rightRoom - rf, leftRoom - rr);
  float ccwMargin = min(leftRoom - rf, rightRoom - rr);
  return spinTurn(cwMargin >= ccwMargin ? -180.0f : 180.0f);
}

// Puts the axle on the decision point before a turn: the front wall if there is one,
// otherwise the encoders.
void settleAtDecisionPoint() {
  sense();
  if (tofF < WALL_THRESHOLD_MM) {
    alignToFrontWall();
    carryMm = 0;
  } else if (fabs(carryMm) > 5 && !(atStartWall && carryMm > 0)) {
    driveStraight(-carryMm, 90, false);
    carryMm = 0;
  }
}

bool executeTurn(char move) {
  if (move == 'S') return true;
  settleAtDecisionPoint();
  bool ok;
  if (move == 'R')      ok = swingTurn(-1);
  else if (move == 'L') ok = swingTurn(+1);
  else                  ok = uTurn();
  // Where the axle ended up, measured from the decision point in the new direction.
  carryMm = (move == 'U') ? 2 * TURN_BACKOFF_MM : HALF_TRACK_MM + TURN_BACKOFF_MM;
  return ok;
}

// Drives to the decision point numCells ahead.
bool moveCells(int numCells, int maxPwm) {
  Serial.print(F("Forward cells: ")); Serial.println(numCells);
  int r = driveStraight(numCells * CELL_MM - carryMm, maxPwm, true);
  carryMm = (r == DRIVE_WALL) ? 0 : lastOvershootMm;
  atStartWall = false;
  return r != DRIVE_STALL;
}

void startAlignment() {
  Serial.println(F("Squaring against back wall..."));
  setMotors(-60, -60);
  delay(1500);
  stopMotors();
  delay(400);
  calibrateGyro();  // heading 0 = straight out of the start cell
  // Back against the wall: axle is AXLE_TO_REAR from the wall face.
  carryMm = AXLE_TO_REAR_MM + TURN_BACKOFF_MM - HALF_CORRIDOR_MM;
  atStartWall = true;
  failCount = 0;
}

// --- PATH OPTIMIZATION ---
void simplifyPath() {
  if (pathLength < 3 || path[pathLength - 2] != 'U') return;

  int totalAngle = 0;
  for (int i = 1; i <= 3; i++) {
    switch (path[pathLength - i]) {
      case 'R': totalAngle += 90; break;
      case 'L': totalAngle += 270; break;
      case 'U': totalAngle += 180; break;
    }
  }

  totalAngle = totalAngle % 360;
  switch (totalAngle) {
    case 0: path[pathLength - 3] = 'S'; break;
    case 90: path[pathLength - 3] = 'R'; break;
    case 180: path[pathLength - 3] = 'U'; break;
    case 270: path[pathLength - 3] = 'L'; break;
  }
  pathLength -= 2;
}

char invertMove(char move) {
  if (move == 'R') return 'L';
  if (move == 'L') return 'R';
  return move;
}

// Each entry = turn at this cell's decision point, then drive to the next one.
// Straight entries are merged into one longer drive.
void executePath(const char* p, int len, int maxPwm) {
  for (int i = 0; i < len && runState != 0; i++) {
    noteResult(executeTurn(p[i]));
    if (runState == 0) return;
    int cells = 1;
    while (i + 1 < len && p[i + 1] == 'S') { cells++; i++; }
    noteResult(moveCells(cells, maxPwm));
  }
}

void printGeometryCheck() {
  float rf = sqrt(sq(AXLE_TO_FRONT_MM) + sq(HALF_WIDTH_MM));
  float rr = sqrt(sq(AXLE_TO_REAR_MM) + sq(HALF_WIDTH_MM));
  float reach = HALF_TRACK_MM + HALF_WIDTH_MM;
  float swingSide  = HALF_CORRIDOR_MM - (sqrt(sq(reach) + sq(AXLE_TO_REAR_MM)) - HALF_TRACK_MM);
  float swingFront = HALF_CORRIDOR_MM - (sqrt(sq(reach) + sq(AXLE_TO_FRONT_MM)) - TURN_BACKOFF_MM);
  float spinSide   = HALF_CORRIDOR_MM - max(rf, rr);

  Serial.println(F("\n--- GEOMETRY CHECK ---"));
  Serial.print(F("Centred side reading should be: ")); Serial.println(SIDE_GAP_MM);
  Serial.print(F("Front reading at decision point: ")); Serial.println(FRONT_GAP_MM);
  Serial.print(F("Swing turn clearance, outer wall: ")); Serial.println(swingSide);
  Serial.print(F("Swing turn clearance, front wall: ")); Serial.println(swingFront);
  Serial.print(F("U-turn clearance per side:        ")); Serial.println(spinSide);
  if (swingSide < 5 || swingFront < 5) Serial.println(F("WARNING: swing turns are tight - check AXLE_TO_FRONT_MM / TRACK_WIDTH_MM"));
  if (spinSide < 2) Serial.println(F("WARNING: U-turns will scrape - axle is far from the middle of the body"));
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

  Serial.println(F("\n=== EXPLORE THEN SPEEDRUN ==="));
  Serial.println(F("Press START to Begin Exploration!"));
}

// --- MAIN LOOP ---
void loop() {

  // STATE 0: wait for start, print sensors so you can check the calibration
  if (runState == 0) {
    static unsigned long lastPrint = 0;
    sense();
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      Serial.print(F("L ")); Serial.print(tofL);
      Serial.print(F("  F ")); Serial.print(tofF);
      Serial.print(F("  R ")); Serial.println(tofR);
    }
    if (digitalRead(START_BUTTON) == LOW) {
      delay(50); while (digitalRead(START_BUTTON) == LOW);
      digitalWrite(STATUS_LED, LOW);
      for (int i = 0; i < 3; i++) { digitalWrite(STATUS_LED, HIGH); delay(200); digitalWrite(STATUS_LED, LOW); delay(200); }
      abortExploration = false;
      pathLength = 0;
      Serial.println(F("\n--- EXPLORATION STARTED ---"));
      startAlignment();
      runState = 1;
    }
  }

  // STATE 1: explore (right-hand rule) until START is pressed
  if (runState == 1) {
    senseFor(60);  // fresh readings at the decision point
    bool wallFront = (tofF < WALL_THRESHOLD_MM);
    bool wallLeft  = (tofL < WALL_THRESHOLD_MM);
    bool wallRight = (tofR < WALL_THRESHOLD_MM);

    char move;
    if (!wallRight)      move = 'R';
    else if (!wallFront) move = 'S';
    else if (!wallLeft)  move = 'L';
    else                 move = 'U';

    noteResult(executeTurn(move));
    if (runState != 1) return;

    if (pathLength < MAX_PATH) {
      path[pathLength++] = move;
      simplifyPath();
    }

    // Always finish the cell, so the recorded path matches where the robot really is.
    noteResult(moveCells(1, DRIVE_PWM));
    if (runState == 1 && abortExploration) runState = 2;
  }

  // STATE 2: drive home along the reversed path (also used after each speed run)
  if (runState == 2) {
    stopMotors();
    while (digitalRead(START_BUTTON) == LOW);
    abortExploration = false;

    Serial.println(F("\n--- OPTIMIZED PATH ---"));
    for (int i = 0; i < pathLength; i++) { Serial.print(path[i]); Serial.print(' '); }
    Serial.println(F("\nReturning Home..."));

    if (pathLength > 0) {
      // Forward: at cell k turn path[k], drive to k+1.
      // Home: turn around, then at cell k turn invert(path[k]) for k = n-1 .. 1.
      int n = 0;
      returnPath[n++] = 'U';
      for (int k = pathLength - 1; k >= 1; k--) returnPath[n++] = invertMove(path[k]);
      executePath(returnPath, n, DRIVE_PWM);
      if (runState == 0) return;

      // In the start cell: face heading 0 again (start cell has walls, so spin, centred).
      settleAtDecisionPoint();
      float d = fmod(-targetHeading, 360.0f);
      if (d > 180) d -= 360;
      if (d < -180) d += 360;
      if (fabs(d) > 135) uTurn();
      else if (fabs(d) > 45) spinTurn(d);
    }

    Serial.println(F("\nArrived Home! Press START for the speed run."));
    startAlignment();
    runState = 3;
  }

  // STATE 3: wait for speed run
  if (runState == 3) {
    digitalWrite(STATUS_LED, millis() % 200 < 100);
    if (digitalRead(START_BUTTON) == LOW) {
      delay(50); while (digitalRead(START_BUTTON) == LOW);
      digitalWrite(STATUS_LED, HIGH); delay(1000); digitalWrite(STATUS_LED, LOW);
      runState = 4;
      Serial.println(F("\n--- SPEEDRUN STARTED ---"));
    }
  }

  // STATE 4: speed run, then drive home so it is ready for another one
  if (runState == 4) {
    executePath(path, pathLength, SPEED_RUN_PWM);
    if (runState == 4) {
      Serial.println(F("\n--- SPEEDRUN COMPLETE --- driving home"));
      runState = 2;
    }
  }
}
