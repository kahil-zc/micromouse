#pragma once
#include <stdint.h>

void encoders_init();
// Atomic snapshot of both counters (positive = wheel moving forward).
void encoders_read(long &left, long &right);
// Mean distance travelled by the two wheels since power-up, in mm.
float encoders_distance_mm();
