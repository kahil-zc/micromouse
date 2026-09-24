// ╔══════════════════════════════════════════════════════════════════╗
// ║  12_maze_solver_speedrun — FULL ALGORITHM                        ║
// ║  Purpose: Explore the maze (right-hand rule) until the goal      ║
// ║           cell, optimize the path, return home, and run the      ║
// ║           shortest path at MAX SPEED.                            ║
// ╚══════════════════════════════════════════════════════════════════╝

#include <Wire.h>
#include <VL53L0X.h>

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

// === MAZE / GOAL SETTINGS ===
// Cell coordinates: the start cell is (0,0) and the robot starts facing "north" (+Y).
// +X is to the robot's RIGHT at the start. Set GOAL_X / GOAL_Y to your maze's goal cell.
#define GOAL_ENABLED       1    // 1 = stop exploring at the goal cell, 0 = explore MAX_EXPLORE_CELLS then stop
#define GOAL_X             2
#define GOAL_Y             2
#define MAX_EXPLORE_CELLS  40   // Safety limit: give up if the goal isn't reached within this many cells
#define MAX_PATH           200

// === YOUR CALIBRATED TOF UNCERTAINTIES (BIAS) ===
#define FRONT_BIAS_MM  30
#define LEFT_BIAS_MM   25
#define RIGHT_BIAS_MM  50

// Target to stop perfectly centered before a wall (22mm from front sensor = 90mm from center)
#define TARGET_WALL_STOP_MM 22
// Distance threshold to consider a wall "present"
#define WALL_THRESHOLD_MM   130

// Distance Tuning (Fallback if no front wall)
#define TICKS_PER_CELL    290
#define BASE_SPEED_LEFT   160
#define BASE_SPEED_RIGHT  160
#define SPEED_RUN_BASE_SPEED 220 // 🚀 MAX SPEED FOR SPEED RUN
#define SPEED_RAMP_TICKS  120    // Accelerate/decelerate over this many ticks in the speed run

// Turn Tuning
#define TURN_SPEED        75
#define TARGET_ANGLE      85.0f
#define TURN_FORWARD_OFFSET_TICKS 65 // ~4cm extra forward before turning to clear back wheels

// Gyro & ToF PID Tuning
#define KP                6.0f
#define KI                0.0f
#define KD                0.5f

#define WALL_KP                 0.3f   // Proportional control for wall centering (mostly rely on gyro)
#define TARGET_SIDE_WALL_DIST   40     // Ideal distance from side sensor to wall
#define TOF_ALPHA               0.2f   // Smoothing factor for ToF readings (lower = smoother)

#define GYRO_CALIB_SAMPLES 200

// --- GLOBALS ---
volatile long encoderLeftTicks = 0;
volatile long encoderRightTicks = 0;
float gyroZOffset = 0;
float currentHeading = 0;
bool hasRun = false;

// --- TOF AVERAGING GLOBALS ---
float avgLeft = 999;
float avgRight = 999;

VL53L0X sensorLeft;
VL53L0X sensorFront;
VL53L0X sensorRight;

// --- PATH MEMORY ---
char path[MAX_PATH];
int pathLength = 0;

// --- POSITION TRACKING (in cells) ---
int posX = 0;
int posY = 0;
uint8_t mazeHeading = 0; // 0 = North, 1 = East, 2 = South, 3 = West

// --- INTERRUPT SERVICE ROUTINES ---
void leftEncoderISR() {
  if (digitalRead(ENCODER_LEFT_C2) == HIGH) encoderLeftTicks++; else encoderLeftTicks--;
}
void rightEncoderISR() {
  if (digitalRead(ENCODER_RIGHT_C2) == HIGH) encoderRightTicks++; else encoderRightTicks--;
}

// 32-bit reads/writes are not atomic on AVR, so pause interrupts while touching the counters
long readLeftTicks() {
  noInterrupts();
  long ticks = encoderLeftTicks;
  interrupts();
  return ticks;
}
void resetEncoders() {
  noInterrupts();
  encoderLeftTicks = 0;
  encoderRightTicks = 0;
  interrupts();
}

// --- TOF HELPER FUNCTIONS ---
// All return 999 when no valid reading. Values can be <= 0 when very close to a wall.
int getTrueFront() {
  int raw = sensorFront.readRangeContinuousMillimeters();
  if (sensorFront.timeoutOccurred() || raw >= 8000 || raw == 0) return 999;
  return raw - FRONT_BIAS_MM;
}
int getTrueLeft() {
  int raw = sensorLeft.readRangeContinuousMillimeters();
  if (sensorLeft.timeoutOccurred() || raw >= 8000 || raw == 0) return 999;
  return raw - LEFT_BIAS_MM;
}
int getTrueRight() {
  int raw = sensorRight.readRangeContinuousMillimeters();
  if (sensorRight.timeoutOccurred() || raw >= 8000 || raw == 0) return 999;
  return raw - RIGHT_BIAS_MM;
}

// Exponential moving average that restarts cleanly when a wall reappears
float smoothSide(float avg, int raw) {
  if (raw >= WALL_THRESHOLD_MM) return 999;   // No wall
  if (avg >= WALL_THRESHOLD_MM) return raw;   // Wall just appeared: seed the average
  return (TOF_ALPHA * raw) + ((1.0f - TOF_ALPHA) * avg);
}

// --- HARDWARE INIT FUNCTIONS ---
void initToFSensors() {
  pinMode(TOF_XSHUT_LEFT, OUTPUT); digitalWrite(TOF_XSHUT_LEFT, LOW);
  pinMode(TOF_XSHUT_FRONT, OUTPUT); digitalWrite(TOF_XSHUT_FRONT, LOW);
  pinMode(TOF_XSHUT_RIGHT, OUTPUT); digitalWrite(TOF_XSHUT_RIGHT, LOW);
  delay(100);

  digitalWrite(TOF_XSHUT_LEFT, HIGH); delay(50);
  sensorLeft.setTimeout(500);
  if(sensorLeft.init()) { sensorLeft.setAddress(0x30); sensorLeft.startContinuous(); }
  else Serial.println("ERROR: Left ToF not found!");

  digitalWrite(TOF_XSHUT_FRONT, HIGH); delay(50);
  sensorFront.setTimeout(500);
  if(sensorFront.init()) { sensorFront.setAddress(0x31); sensorFront.startContinuous(); }
  else Serial.println("ERROR: Front ToF not found!");

  digitalWrite(TOF_XSHUT_RIGHT, HIGH); delay(50);
  sensorRight.setTimeout(500);
  if(sensorRight.init()) { sensorRight.setAddress(0x32); sensorRight.startContinuous(); }
  else Serial.println("ERROR: Right ToF not found!");
}

void initMPU6050() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
}

int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true);
  // Read high and low bytes in separate statements: the order of two Wire.read() calls
  // inside one expression is not guaranteed by C++ and could swap the bytes.
  uint8_t high = Wire.read();
  uint8_t low = Wire.read();
  return (int16_t)((high << 8) | low);
}

void calibrateGyro() {
  Serial.println("Calibrating Gyro... KEEP STILL!");
  long sum = 0;
  for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) { sum += readGyroZRaw(); delay(3); }
  gyroZOffset = (float)sum / GYRO_CALIB_SAMPLES;
}

float getGyroZRate() {
  return (readGyroZRaw() - gyroZOffset) / 131.0f;
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

void stopMotors() { setMotors(0, 0); }

// --- PATH LOGIC ---
// Each path entry means "turn (R/L/U/S) in this cell, then drive one cell".
void simplifyPath() {
  if (pathLength < 3 || path[pathLength-2] != 'U') return;

  int totalAngle = 0;
  for (int i = 1; i <= 3; i++) {
    switch(path[pathLength - i]) {
      case 'R': totalAngle += 90; break;
      case 'L': totalAngle += 270; break;
      case 'U': totalAngle += 180; break;
      case 'S': totalAngle += 0; break;
    }
  }

  totalAngle = totalAngle % 360;

  switch(totalAngle) {
    case 0: path[pathLength - 3] = 'S'; break;
    case 90: path[pathLength - 3] = 'R'; break;
    case 180: path[pathLength - 3] = 'U'; break;
    case 270: path[pathLength - 3] = 'L'; break;
  }
  pathLength -= 2; // Erased the dead end!
}

char invertMove(char m) {
  if (m == 'R') return 'L';
  if (m == 'L') return 'R';
  return m;
}

// --- POSITION TRACKING ---
void updateHeading(char move) {
  if (move == 'R') mazeHeading = (mazeHeading + 1) & 3;
  else if (move == 'L') mazeHeading = (mazeHeading + 3) & 3;
  else if (move == 'U') mazeHeading = (mazeHeading + 2) & 3;
}

void advancePosition() {
  switch (mazeHeading) {
    case 0: posY++; break;
    case 1: posX++; break;
    case 2: posY--; break;
    case 3: posX--; break;
  }
}

// --- MOVEMENT BEHAVIORS ---

void startAlignment() {
  Serial.println("Squaring against back wall...");
  setMotors(-60, -60);
  delay(2000); // Wait 2 seconds pushing backwards to square up
  stopMotors();
  delay(500);

  calibrateGyro(); // Reset heading to perfectly straight

  Serial.println("Moving 2.2cm to cell center...");
  resetEncoders();
  currentHeading = 0;
  unsigned long lastTime = millis();
  while(labs(readLeftTicks()) < 35) { // ~2.2cm
     unsigned long currentMillis = millis();
     if (currentMillis - lastTime >= 10) {
       float dt = (currentMillis - lastTime) / 1000.0f;
       lastTime = currentMillis;
       currentHeading += getGyroZRate() * dt;
       float correction = KP * (0.0f - currentHeading);
       setMotors(70 - correction, 70 + correction); // Move slow
     }
  }
  stopMotors();
  delay(300);
}

void moveForwardExtra(int targetTicks) {
  resetEncoders();
  currentHeading = 0;
  unsigned long lastLoopTime = millis();
  while (labs(readLeftTicks()) < targetTicks) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastLoopTime >= 10) {
      float dt = (currentMillis - lastLoopTime) / 1000.0f;
      lastLoopTime = currentMillis;
      currentHeading += getGyroZRate() * dt;
      float correction = KP * (0.0f - currentHeading);
      setMotors(BASE_SPEED_LEFT - correction, BASE_SPEED_RIGHT + correction);
    }
    // Safety check: Don't crash into front wall!
    if (getTrueFront() <= TARGET_WALL_STOP_MM) break;
  }
  stopMotors();
  delay(100);
}

// Drive straight for `cells` cells. In the speed run it ramps up and down between
// the exploration speed and SPEED_RUN_BASE_SPEED so long straights are fast but
// the robot still arrives at turns slowly.
void moveForwardCells(int cells, bool speedRun) {
  Serial.print("Moving: FORWARD "); Serial.print(cells); Serial.println(" CELL(S)");
  long targetTicks = (long)cells * TICKS_PER_CELL;
  int rightTrim = BASE_SPEED_RIGHT - BASE_SPEED_LEFT;
  int stopThreshold = speedRun ? TARGET_WALL_STOP_MM + 25 : TARGET_WALL_STOP_MM; // Hit brakes earlier if going fast!

  resetEncoders();
  currentHeading = 0;
  float previousError = 0, integralError = 0;
  bool firstLoop = true;
  unsigned long lastLoopTime = millis();

  avgLeft = 999;  // Seeded by the first valid reading
  avgRight = 999;

  while (true) {
    long traveled = labs(readLeftTicks());
    if (traveled >= targetTicks) break;

    unsigned long currentMillis = millis();
    if (currentMillis - lastLoopTime < 10) continue;
    float dt = (currentMillis - lastLoopTime) / 1000.0f;
    lastLoopTime = currentMillis;

    // Front ToF Verification: Stop EXACTLY in the center if there is a wall!
    // (No "> 0" check: a reading <= 0 means we are already very close and must stop.)
    int trueFront = getTrueFront();
    if (trueFront <= stopThreshold) {
      Serial.println("FRONT WALL VERIFIED! Stopping perfectly in center.");
      break;
    }

    currentHeading += getGyroZRate() * dt;
    float gyroError = 0.0f - currentHeading;

    avgLeft = smoothSide(avgLeft, getTrueLeft());
    avgRight = smoothSide(avgRight, getTrueRight());

    bool hasLeft = (avgLeft < WALL_THRESHOLD_MM);
    bool hasRight = (avgRight < WALL_THRESHOLD_MM);

    float tofError = 0.0f;
    if (hasLeft && hasRight) tofError = (avgLeft - avgRight) * WALL_KP;
    else if (hasLeft) tofError = (avgLeft - TARGET_SIDE_WALL_DIST) * WALL_KP;
    else if (hasRight) tofError = (TARGET_SIDE_WALL_DIST - avgRight) * WALL_KP;

    float totalError = gyroError + tofError;
    integralError += totalError * dt;
    float derivativeError = firstLoop ? 0.0f : (totalError - previousError) / dt;
    previousError = totalError;
    firstLoop = false;

    int base = BASE_SPEED_LEFT;
    if (speedRun) {
      long distToEnd = min(traveled, targetTicks - traveled);
      base = map(constrain(distToEnd, 0, SPEED_RAMP_TICKS), 0, SPEED_RAMP_TICKS,
                 BASE_SPEED_LEFT, SPEED_RUN_BASE_SPEED);
    }

    float correction = (KP * totalError) + (KI * integralError) + (KD * derivativeError);
    setMotors(base - correction, base + rightTrim + correction);
  }
  stopMotors();
  delay(300); // Wait for chassis to settle
}

void turnRight90() {
  Serial.println("Moving: TURN RIGHT 90");
  currentHeading = 0;
  unsigned long lastLoopTime = millis();
  setMotors(TURN_SPEED, -TURN_SPEED);
  while (abs(currentHeading) < TARGET_ANGLE) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastLoopTime >= 5) {
      float dt = (currentMillis - lastLoopTime) / 1000.0f;
      lastLoopTime = currentMillis;
      currentHeading += getGyroZRate() * dt;
    }
  }
  stopMotors(); delay(400);
}

void turnLeft90() {
  Serial.println("Moving: TURN LEFT 90");
  currentHeading = 0;
  unsigned long lastLoopTime = millis();
  setMotors(-TURN_SPEED, TURN_SPEED);
  while (abs(currentHeading) < TARGET_ANGLE) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastLoopTime >= 5) {
      float dt = (currentMillis - lastLoopTime) / 1000.0f;
      lastLoopTime = currentMillis;
      currentHeading += getGyroZRate() * dt;
    }
  }
  stopMotors(); delay(400);
}

void performTurn(char move) {
  if (move == 'R') { moveForwardExtra(TURN_FORWARD_OFFSET_TICKS); turnRight90(); }
  else if (move == 'L') { moveForwardExtra(TURN_FORWARD_OFFSET_TICKS); turnLeft90(); }
  else if (move == 'U') { turnRight90(); turnRight90(); }
}

void executePath(bool speedRun, bool backwards) {
  if (backwards) {
    // Recorded moves are "turn, then drive". To retrace them from the end we must
    // "drive, then do the opposite turn", in reverse order. Starts facing back the
    // way we came; ends in the start cell facing the back wall.
    for (int i = pathLength - 1; i >= 0; i--) {
      moveForwardCells(1, speedRun);
      performTurn(invertMove(path[i]));
    }
    return;
  }

  int i = 0;
  while (i < pathLength) {
    performTurn(path[i]);
    // In the speed run, merge the following straight moves into one long straight
    int cells = 1;
    if (speedRun) {
      while (i + cells < pathLength && path[i + cells] == 'S') cells++;
    }
    moveForwardCells(cells, speedRun);
    i += cells;
  }
}

// Right-hand wall follower. Returns true when exploration succeeded.
bool exploreMaze() {
  posX = 0; posY = 0; mazeHeading = 0;
  pathLength = 0;

  for (int cellsExplored = 0; cellsExplored < MAX_EXPLORE_CELLS; cellsExplored++) {
    int tf = getTrueFront();
    int tl = getTrueLeft();
    int tr = getTrueRight();

    bool wallFront = (tf < WALL_THRESHOLD_MM);
    bool wallLeft  = (tl < WALL_THRESHOLD_MM);
    bool wallRight = (tr < WALL_THRESHOLD_MM);

    char move;
    if (!wallRight) move = 'R';
    else if (!wallFront) move = 'S';
    else if (!wallLeft) move = 'L';
    else move = 'U';

    if (pathLength >= MAX_PATH) {
      Serial.println("ERROR: Path memory full!");
      return false;
    }

    performTurn(move);
    path[pathLength++] = move;
    simplifyPath(); // Optimize away dead ends!

    moveForwardCells(1, false);
    updateHeading(move);
    advancePosition();

    Serial.print("Now at cell ("); Serial.print(posX); Serial.print(",");
    Serial.print(posY); Serial.println(")");

    if (GOAL_ENABLED) {
      if (posX == GOAL_X && posY == GOAL_Y) {
        Serial.println("GOAL REACHED!");
        return true;
      }
      // Back at the start facing the original direction = the wall follower went all
      // the way around without finding the goal (e.g. goal on an island in the middle).
      if (posX == 0 && posY == 0 && mazeHeading == 0) {
        Serial.println("ERROR: Back at start - goal not reachable by wall following!");
        return false;
      }
    }
  }

  if (GOAL_ENABLED) {
    Serial.println("ERROR: Explore limit reached before the goal!");
    return false;
  }
  return true;
}

void failAndHalt() {
  stopMotors();
  Serial.println("Stopping. Check GOAL_X / GOAL_Y / MAX_EXPLORE_CELLS.");
  while (true) { // Fast blink forever = error
    digitalWrite(STATUS_LED, HIGH); delay(80);
    digitalWrite(STATUS_LED, LOW); delay(80);
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(STATUS_LED, OUTPUT);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_C1, INPUT); pinMode(ENCODER_LEFT_C2, INPUT);
  pinMode(ENCODER_RIGHT_C1, INPUT); pinMode(ENCODER_RIGHT_C2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_C1), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_C1), rightEncoderISR, RISING);

  setupMotors();
  Serial.println("\nInitializing Sensors...");
  initToFSensors();
  initMPU6050();

  Serial.println("\n=== MAZE SOLVER & SPEED RUNNER ===");
  Serial.println("Press START button (A3) to begin...");
}

// --- MAIN LOOP ---
void loop() {
  if (!hasRun) {
    if (digitalRead(START_BUTTON) == LOW) {
      delay(50);
      while(digitalRead(START_BUTTON) == LOW);

      for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED, HIGH); delay(200);
        digitalWrite(STATUS_LED, LOW); delay(200);
      }

      Serial.println("\n--- MAZE SOLVING STARTED ---");

      // Phase 0: Square Up against the wall!
      startAlignment();

      // Phase 1: EXPLORATION (until goal or MAX_EXPLORE_CELLS)
      bool explored = exploreMaze();

      Serial.println("Exploration finished. Path Optimized:");
      for(int i=0; i<pathLength; i++) { Serial.print(path[i]); Serial.print(" "); }
      Serial.println();

      if (!explored) failAndHalt();

      // Phase 2: RETURN HOME
      Serial.println("Returning Home...");
      turnRight90(); turnRight90(); // U-turn to face backwards
      executePath(false, true); // Navigate the path backward!

      // We are now back in the start cell, facing the back wall.
      Serial.println("Arrived Home! Preparing for Speed Run...");
      turnRight90(); turnRight90(); // U-turn to face forward again
      startAlignment(); // Square up perfectly for the speed run

      // Phase 3: SPEED RUN PREP (WAIT FOR USER)
      Serial.println("\n--- WAITING FOR USER TO START SPEED RUN ---");

      // 1. Blink LED for 10 seconds
      unsigned long blinkStart = millis();
      while(millis() - blinkStart < 10000) {
        digitalWrite(STATUS_LED, HIGH); delay(250);
        digitalWrite(STATUS_LED, LOW); delay(250);
      }

      // 2. Stay ON for 5 seconds
      digitalWrite(STATUS_LED, HIGH);
      delay(5000);
      digitalWrite(STATUS_LED, LOW);

      // 3. Wait for button click!
      Serial.println("Press START button (A3) to launch SPEED RUN!");
      while(digitalRead(START_BUTTON) == HIGH); // wait for press
      delay(50);
      while(digitalRead(START_BUTTON) == LOW); // wait for release

      // Quick countdown flashes
      for (int i=0; i<3; i++) {
        digitalWrite(STATUS_LED, HIGH); delay(100);
        digitalWrite(STATUS_LED, LOW); delay(100);
      }

      Serial.println("\n--- SPEED RUN ENGAGED ---");
      executePath(true, false); // BLAST IT!

      Serial.println("SPEED RUN COMPLETE!");
      hasRun = true;
    }
  }
}
