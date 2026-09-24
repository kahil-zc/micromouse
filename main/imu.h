#pragma once

void imu_init();
// Averages the gyro bias while the robot is still. Returns false if it moved.
bool imu_calibrate();
// Integrates the gyro; call every control tick.
void imu_update();
void imu_reset_heading();
float imu_heading();  // degrees, CCW positive, unwrapped
float imu_rate();     // deg/s, CCW positive
