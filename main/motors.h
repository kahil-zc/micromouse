#pragma once

void motors_init();
// Signed PWM, -255..255 (clamped to MAX_PWM). Positive drives the robot forward.
void motors_set(int left, int right);
// TB6612 short brake on both channels.
void motors_brake();
