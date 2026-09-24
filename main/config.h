// ╔══════════════════════════════════════════════════════════════════╗
// ║  config.h — every pin, geometry value and tuning constant        ║
// ║  Values marked CAL must be checked on the real robot.            ║
// ╚══════════════════════════════════════════════════════════════════╝
#pragma once
#include <Arduino.h>

// ---------------- Pins (Arduino Nano) ----------------
#define PIN_MOTOR_STBY   12
#define PIN_MOTOR_L_PWM  5    // TB6612 channel A = left
#define PIN_MOTOR_L_IN1  8
#define PIN_MOTOR_L_IN2  9
#define PIN_MOTOR_R_PWM  6    // TB6612 channel B = right
#define PIN_MOTOR_R_IN1  10
#define PIN_MOTOR_R_IN2  11

#define PIN_ENC_L_A      2    // Must be an interrupt pin
#define PIN_ENC_L_B      4
#define PIN_ENC_R_A      3    // Must be an interrupt pin
#define PIN_ENC_R_B      7

#define PIN_TOF_XSHUT_L  A0
#define PIN_TOF_XSHUT_F  A1
#define PIN_TOF_XSHUT_R  A2
#define TOF_ADDR_L       0x30
#define TOF_ADDR_F       0x31
#define TOF_ADDR_R       0x32

#define PIN_BUTTON       A3   // To GND, internal pull-up
#define PIN_LED          13
#define MPU_ADDR         0x68

// ---------------- Polarity (use HARDWARE CHECK mode to verify) ----------------
#define MOTOR_L_INVERT   0
#define MOTOR_R_INVERT   0
#define ENC_L_INVERT     0    // CAL: counts must go UP when driving forward
#define ENC_R_INVERT     1    // CAL: mirrored motor usually needs inverting
#define GYRO_INVERT      0    // CAL: heading must go UP when turning LEFT (CCW)

// ---------------- Maze ----------------
// Maze size and goal position live in maze.h
#define CELL_MM          180.0f

// ---------------- Odometry ----------------
// Both edges of encoder channel A are counted (2x the old RISING-only count).
// The old sketch measured 290 rising edges per cell -> 580 counts per 180 mm.
#define COUNTS_PER_CELL  580.0f                       // CAL: drive 5 cells, adjust
#define COUNTS_PER_MM    (COUNTS_PER_CELL / CELL_MM)
#define GYRO_SCALE       1.000f                       // CAL: 4 turns in HW check must end square

// ---------------- Start pose ----------------
// Place the robot with its back touching the start cell's back wall.
// It drives this far forward to put the wheel axle over the cell centre.
#define START_TO_CENTER_MM 22.0f                      // CAL

// ---------------- ToF (raw mm, as reported by the sensor) ----------------
// Defaults below come from the old sketch's biases; CALIBRATE mode overwrites them in EEPROM.
#define DEFAULT_SIDE_L_MM   65   // Left reading with a wall, robot centred
#define DEFAULT_SIDE_R_MM   90   // Right reading with a wall, robot centred
#define DEFAULT_FRONT_MM    52   // Front reading facing a wall, axle at cell centre
#define WALL_DETECT_MARGIN_MM 90 // Wall present if reading < centred reading + margin
#define SIDE_TRACK_WINDOW_MM  35 // Side reading used for centring only if this close to expected
#define FRONT_ALIGN_MAX_CORR_MM 40 // Front-wall position correction only if within this of expected
#define TOF_TIMING_BUDGET_US  33000
#define TOF_LATENCY_S         0.02f
#define TOF_STALE_MS          100
#define TOF_NONE              0xFFFF

// ---------------- Control loop ----------------
#define CONTROL_PERIOD_US 5000   // 200 Hz
#define MAX_PWM           230

// Forward (translation) controller — PWM units
#define KFF_V    0.30f   // CAL: PWM per mm/s
#define KFF_S    25.0f   // CAL: PWM to overcome static friction
#define KP_FWD   3.0f    // PWM per mm of position error
#define KD_FWD   0.05f   // PWM per mm/s of position error rate

// Rotation controller — PWM units
#define KFF_W    0.24f   // CAL: PWM per deg/s (= KFF_V * track_mm * pi/360)
#define KP_ROT   4.0f    // PWM per degree
#define KD_ROT   0.15f   // PWM per deg/s

// Wall centring: heading offset (deg) per mm of lateral error
#define WALL_KP_DEG_PER_MM 0.15f
#define WALL_MAX_STEER_DEG 6.0f
#define WALL_STEER_ALPHA   0.3f

// Crash detection: abort the move if tracking error exceeds these
#define MAX_FWD_ERR_MM   40.0f
#define MAX_ROT_ERR_DEG  30.0f

// ---------------- Speed profiles ----------------
#define SEARCH_SPEED     250.0f  // mm/s
#define SEARCH_ACCEL     1500.0f // mm/s^2
#define FAST1_SPEED      400.0f
#define FAST1_ACCEL      2000.0f
#define FAST2_SPEED      550.0f
#define FAST2_ACCEL      2500.0f
#define FAST3_SPEED      700.0f
#define FAST3_ACCEL      3000.0f
#define TURN_OMEGA       360.0f  // deg/s
#define TURN_ALPHA       3000.0f // deg/s^2
#define SETTLE_MS        400     // Max time to settle at the end of a move

// ---------------- UI ----------------
#define LONG_PRESS_MS    800
#define DEBOUNCE_MS      30
#define START_DELAY_MS   1000    // Hand-clear time after committing a mode
