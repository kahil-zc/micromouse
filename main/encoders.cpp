#include "encoders.h"
#include "config.h"

static volatile long countLeft = 0;
static volatile long countRight = 0;

// Both edges of channel A: A == B means one direction, A != B the other.
static void isrLeft() {
  bool forward = (digitalRead(PIN_ENC_L_A) == digitalRead(PIN_ENC_L_B)) != ENC_L_INVERT;
  if (forward) countLeft++; else countLeft--;
}
static void isrRight() {
  bool forward = (digitalRead(PIN_ENC_R_A) == digitalRead(PIN_ENC_R_B)) != ENC_R_INVERT;
  if (forward) countRight++; else countRight--;
}

void encoders_init() {
  pinMode(PIN_ENC_L_A, INPUT); pinMode(PIN_ENC_L_B, INPUT);
  pinMode(PIN_ENC_R_A, INPUT); pinMode(PIN_ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRight, CHANGE);
}

void encoders_read(long &left, long &right) {
  noInterrupts();
  left = countLeft;
  right = countRight;
  interrupts();
}

float encoders_distance_mm() {
  long l, r;
  encoders_read(l, r);
  return (l + r) * 0.5f / COUNTS_PER_MM;
}
