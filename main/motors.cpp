#include "motors.h"
#include "config.h"

static void setChannel(int pwmPin, int in1, int in2, int speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  if (speed >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else { digitalWrite(in1, LOW); digitalWrite(in2, HIGH); speed = -speed; }
  analogWrite(pwmPin, speed);
}

void motors_init() {
  pinMode(PIN_MOTOR_STBY, OUTPUT);
  pinMode(PIN_MOTOR_L_PWM, OUTPUT); pinMode(PIN_MOTOR_L_IN1, OUTPUT); pinMode(PIN_MOTOR_L_IN2, OUTPUT);
  pinMode(PIN_MOTOR_R_PWM, OUTPUT); pinMode(PIN_MOTOR_R_IN1, OUTPUT); pinMode(PIN_MOTOR_R_IN2, OUTPUT);
  motors_brake();
  digitalWrite(PIN_MOTOR_STBY, HIGH);
}

void motors_set(int left, int right) {
  setChannel(PIN_MOTOR_L_PWM, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, left, MOTOR_L_INVERT);
  setChannel(PIN_MOTOR_R_PWM, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, right, MOTOR_R_INVERT);
}

void motors_brake() {
  // IN1 = IN2 = HIGH is short brake regardless of PWM
  digitalWrite(PIN_MOTOR_L_IN1, HIGH); digitalWrite(PIN_MOTOR_L_IN2, HIGH);
  digitalWrite(PIN_MOTOR_R_IN1, HIGH); digitalWrite(PIN_MOTOR_R_IN2, HIGH);
  analogWrite(PIN_MOTOR_L_PWM, 0);
  analogWrite(PIN_MOTOR_R_PWM, 0);
}
