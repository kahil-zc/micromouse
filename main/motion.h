#pragma once
#include <stdint.h>

// Zero the gyro heading and the absolute heading target (robot square to the maze).
void motion_reset();

// Drive forward from rest to rest with a trapezoidal profile. alignFront uses a front wall,
// if one is seen near the end, to stop exactly at the cell centre. Returns false if the
// tracking error says the robot hit something (motors are braked).
bool motion_straight(float distanceMm, float vMax, float accel, bool alignFront);

// Pivot in place by 90-degree steps, positive = left (CCW). The heading target is absolute,
// so turn errors don't accumulate. Returns false on a crash.
bool motion_turn(int8_t quarterTurnsCcw);

// Brake and wait, keeping the gyro integrated and the sensors polled.
void motion_wait(uint16_t ms);
