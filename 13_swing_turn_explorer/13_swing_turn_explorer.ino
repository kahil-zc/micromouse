// ╔══════════════════════════════════════════════════════════════════╗
// ║  13_swing_turn_explorer                                          ║
// ║  Right-hand explore -> return home -> speed run, for a 99 x 126  ║
// ║  mm robot whose axle is 47 mm behind the front.                  ║
// ║                                                                  ║
// ║  The rear corners are 93 mm from the axle and the corridor is    ║
// ║  84 mm from centre to wall, so this robot cannot spin in place   ║
// ║  in the middle of a corridor. Instead:                           ║
// ║   * 90° turns are 80 mm-radius arcs, started 80 mm before the    ║
// ║     cell centre. They end centred in the new corridor with       ║
// ║     ~12 mm between the rear corner and the outer wall. If the    ║
// ║     robot is too close to the outer wall it first shuffles       ║
// ║     sideways toward the turn.                                    ║
// ║   * U-turns spin in place after shuffling ~12 mm toward the side ║
// ║     the front corners sweep, so the long rear gets the room.     ║
// ║   * Position along the corridor is corrected by the front wall   ║
// ║     and by side-wall edges (where a wall starts or ends).        ║
// ║   * The side walls also measure the robot's real angle to the    ║
// ║     corridor and correct the gyro, so it stays straight through  ║
// ║     open cells where there is nothing to centre on.              ║
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

// === ROBOT GEOMETRY ===
#define ROBOT_WIDTH_MM     99.0f
#define ROBOT_LENGTH_MM   126.0f
#define AXLE_TO_FRONT_MM   47.0f   // front ToF sits at the front edge
#define SIDE_TOF_AHEAD_MM  32.0f   // side ToFs are this far in front of the axle
#define TRACK_WIDTH_MM     88.0f   // MEASURE: middle of left tyre -> middle of right tyre

// === MAZE ===
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
float centreGap = SIDE_GAP_MM;    // learned side reading when centred, used when only one wall
uint8_t seqL = 0, seqR = 0;       // bump on every new side reading
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

// Tracks one side sensor switching between wall and gap (see edgeUpdate). Defined up here
// because the Arduino IDE puts its generated function prototypes above the first function.
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
// gyro is pulled back a little each time.
void resetFit(WallFit &f) { f.n = 0; }

// sideSign: +1 = left wall, -1 = right wall.
void fitUpdate(WallFit &f, int tof, float done, int sideSign) {
  if (tof >= SIDE_FOLLOW_MM) { f.n = 0; return; }  // no wall: start again
  if (f.n == 0) { f.x0 = done; f.sx = f.sy = f.sxx = f.sxy = f.sPsi = 0; }
  float x = done - f.x0;
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
  int result = DRIVE_DONE;
  float corr = 0;
  SideEdge edgeL, edgeR;
  resetEdge(edgeL); resetEdge(edgeR);
  WallFit fitL, fitR;
  resetFit(fitL); resetFit(fitR);
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
      if (useFit) fitUpdate(fitL, tofL, done, +1);
    }
    if (seqR != lastSeqR) {
      lastSeqR = seqR;
      if (useEdges) edgeUpdate(edgeR, tofR, done, corr, startAxleMm, 'R');
      if (useFit) fitUpdate(fitR, tofR, done, -1);
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
  lastOvershootMm = travelledMm() + corr - goal;
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
// distance with the encoders.
bool arcTurn(int dir) {
  Serial.println(dir > 0 ? F("Arc LEFT") : F("Arc RIGHT"));
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

  float l, r;
  averageSides(l, r);
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
  if (fabs(shiftLeft) > SHIFT_TOL_MM && !atStartWall) shiftSideways(shiftLeft, U_TURN_FRONT_GAP_MM);

  Serial.println(cw ? F("Spin CW") : F("Spin CCW"));
  return spinTurn(cw ? -fabs(angleDeg) : fabs(angleDeg));
}

// Before an arc: if the robot is too close to the wall the rear corner swings toward,
// shuffle toward the turn side. That shift only moves the end point along the new corridor,
// not sideways, so it costs nothing after the turn.
void prepareArc(int dir) {
  if (atStartWall) return;
  float l, r;
  averageSides(l, r);
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

// Puts the axle on the decision point before an arc: the front wall if there is one,
// otherwise the encoders.
void settleAtDecisionPoint() {
  sense();
  if (tofF < FRONT_WALL_THRESHOLD_MM) {
    alignToFrontWall(FRONT_GAP_MM);
    carryMm = 0;
  } else if (fabs(carryMm) > 5 && !(atStartWall && carryMm > 0)) {
    driveStraight(-carryMm, 90, false, false, NAN);
    carryMm = 0;
  }
}

bool executeTurn(char move) {
  if (move == 'S') return true;
  bool ok;
  if (move == 'U') {
    ok = spinSafely(180.0f);
    // The U-turn happens U_TURN_FRONT_GAP from the dead-end wall; measure the axle from the
    // decision point in the new direction.
    carryMm = (U_TURN_FRONT_GAP_MM + AXLE_TO_FRONT_MM - HALF_CORRIDOR_MM) + DECISION_BACK_MM;
  } else {
    int dir = (move == 'L') ? 1 : -1;
    settleAtDecisionPoint();
    prepareArc(dir);
    ok = arcTurn(dir);
    carryMm = TURN_RADIUS_MM + DECISION_BACK_MM;  // arc ends TURN_RADIUS past the cell centre
  }
  return ok;
}

// Drives to the decision point numCells ahead.
bool moveCells(int numCells, int maxPwm) {
  Serial.print(F("Forward cells: ")); Serial.println(numCells);
  float startAxle = carryMm - DECISION_BACK_MM;
  int r = driveStraight(numCells * CELL_MM - carryMm, maxPwm, true, true, startAxle);
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
  carryMm = (AXLE_TO_REAR_MM - HALF_CORRIDOR_MM) + DECISION_BACK_MM;
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
    bool wallFront = (tofF < FRONT_WALL_THRESHOLD_MM);
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

      // In the start cell: face heading 0 again, then back into the start wall.
      float d = fmod(-targetHeading, 360.0f);
      if (d > 180) d -= 360;
      if (d < -180) d += 360;
      if (fabs(d) > 45) spinSafely(fabs(d) > 135 ? 180.0f : d);
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
