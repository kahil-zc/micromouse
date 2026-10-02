// ╔══════════════════════════════════════════════════════════════════╗
// ║  OLED DOOM — a Doom-style shooter for ESP32-S3                  ║
// ║  SH1106 128x64 I2C OLED + analog joystick (with push button)    ║
// ╚══════════════════════════════════════════════════════════════════╝
//
// Board: "ESP32S3 Dev Module". Libraries: Adafruit SH110X + Adafruit GFX (Library Manager).
// The game itself is in game.cpp; this file only reads the controls and drives the screen.
//
// Controls:  stick left/right = turn       stick up/down = walk forward/back
//            press the stick  = fire (hold for auto fire), start, restart after dying
//            BOOT button      = hold to show the map
//
// Doors open when you walk up to them. Find the red keycard for red doors. Walk into the
// EXIT switch to finish a level.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "game.h"

// ---------------------------------------------------------------- wiring
#define SDA_PIN    8
#define SCL_PIN    9
#define VRX_PIN    4      // joystick VRx (ADC1)
#define VRY_PIN    5      // joystick VRy (ADC1)
#define FIRE_PIN   6      // joystick SW -> pulled to GND when the stick is pressed
#define MAP_PIN    0      // BOOT button on the DevKit
#define BUZZER_PIN -1     // optional passive buzzer (e.g. 7); -1 = no sound

// Power the joystick from 3.3 V, not 5 V: the ESP32-S3 ADC pins only take 3.3 V.

// ---------------------------------------------------------------- display
#define OLED_ADDR  0x3C
// The SH1106 is specified for 400 kHz. Try 800000 for a faster frame rate if your display
// and wiring cope; drop back if you see glitches.
#define OLED_I2C_HZ 400000

// ---------------------------------------------------------------- joystick feel
// Flip one of these if that direction feels backwards with your joystick mounted as it is.
#define INVERT_TURN false
#define INVERT_MOVE false
#define DEADZONE    0.18f   // fraction of full travel ignored around the centre

Adafruit_SH1106G display(128, 64, &Wire, -1, OLED_I2C_HZ, OLED_I2C_HZ);

static uint8_t frame[FB_BYTES];
static int centerX = 2048, centerY = 2048;
static uint32_t lastMicros;

static int readAxis(int pin) {
  int s = 0;
  for (int i = 0; i < 4; i++) s += analogRead(pin);
  return s / 4;
}

// Raw 0..4095 -> -1..+1 with a dead zone. Each side is scaled separately because the
// resting value is rarely exactly in the middle.
static float axisValue(int raw, int center, bool invert) {
  float v = raw >= center ? (float)(raw - center) / (4095 - center) : (float)(raw - center) / center;
  if (invert) v = -v;
  float a = fabsf(v);
  if (a < DEADZONE) return 0;
  a = (a - DEADZONE) / (1.0f - DEADZONE);
  if (a > 1) a = 1;
  return v < 0 ? -a : a;
}

static int calibrate(int pin) {
  long s = 0;
  for (int i = 0; i < 32; i++) { s += analogRead(pin); delay(2); }
  int c = (int)(s / 32);
  return (c < 800 || c > 3300) ? 2048 : c;   // stick not centred / not connected
}

#if BUZZER_PIN >= 0
static void playSound(SoundId s) {
  static const uint16_t FREQ[] = { 0, 180, 120, 1400, 150, 900, 300, 700, 400, 500, 200, 250, 90, 100, 1000 };
  static const uint16_t MS[]   = { 0, 60, 150, 50, 30, 25, 60, 25, 60, 20, 120, 80, 200, 400, 300 };
  if (s != SND_NONE && s < sizeof(FREQ) / sizeof(FREQ[0])) tone(BUZZER_PIN, FREQ[s], MS[s]);
}
#endif

void setup() {
  Serial.begin(115200);
  pinMode(FIRE_PIN, INPUT_PULLUP);
  pinMode(MAP_PIN, INPUT_PULLUP);
  analogReadResolution(12);

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!display.begin(OLED_ADDR, true)) {
    Serial.println("OLED not found - check SDA/SCL wiring and the address (0x3C or 0x3D)");
    while (true) delay(1000);
  }
  display.setContrast(255);
  display.clearDisplay();
  display.display();

  // Leave the stick alone while the game starts: this reads its resting position.
  centerX = calibrate(VRX_PIN);
  centerY = calibrate(VRY_PIN);
  Serial.printf("Joystick centre: X=%d Y=%d\n", centerX, centerY);

  gameInit(micros() ^ ((uint32_t)analogRead(VRX_PIN) << 16) ^ analogRead(VRY_PIN));
  lastMicros = micros();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  GameInput in;
  float turn = axisValue(readAxis(VRX_PIN), centerX, INVERT_TURN);
  in.turn = turn * (0.35f + 0.65f * fabsf(turn));   // gentle near the centre for aiming
  in.move = -axisValue(readAxis(VRY_PIN), centerY, INVERT_MOVE);   // stick up reads low = forward
  in.fire = digitalRead(FIRE_PIN) == LOW;
  in.map = digitalRead(MAP_PIN) == LOW;

  gameUpdate(in, dt);
  gameRender(frame);

  display.clearDisplay();                          // marks the whole screen to be sent
  memcpy(display.getBuffer(), frame, FB_BYTES);
  display.display();

  SoundId s = gameTakeSound();
#if BUZZER_PIN >= 0
  playSound(s);
#else
  (void)s;
#endif
}
