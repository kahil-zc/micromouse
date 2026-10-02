# OLED DOOM — ESP32-S3 + SH1106 128×64 + joystick

A Doom-style shooter built for a 1.3" 128×64 monochrome OLED. It has three maps (E1M1–E1M3),
zombiemen, imps that throw fireballs, demons, a Baron of Hell, explosive barrels, doors, a red
keycard, pistol/shotgun/fist, pickups, a HUD, an automap, death/restart, level stats and a
victory screen.

## Wiring

| Part | Pin | ESP32-S3 DevKit |
|---|---|---|
| OLED SH1106 (I2C, 0x3C) | SDA / SCL | GPIO 8 / GPIO 9 |
| | VCC / GND | 3V3 / GND |
| Joystick | VRx / VRy | GPIO 4 / GPIO 5 |
| | SW (push button) | GPIO 6 |
| | +5V / GND | **3V3** / GND |
| Buzzer (optional, passive) | + | any free GPIO, set `BUZZER_PIN` |

Power the joystick from **3.3 V**: on 5 V it drives the ADC pins above 3.3 V.
The automap uses the DevKit's own **BOOT** button (GPIO 0), so there's nothing extra to wire.

## Build

1. Arduino IDE → Boards Manager → install **esp32 by Espressif**. Board: **ESP32S3 Dev Module**.
2. Library Manager → install **Adafruit SH110X** (it pulls in Adafruit GFX and BusIO).
3. Open `oled_doom/oled_doom.ino` (the folder holds `game.cpp`, `game.h`, `art.h`, `levels.h`) and upload.
4. Leave the stick centred while it boots: it reads the stick's resting position.

If you upload through the board's native **USB** port (not **UART**) and want Serial output, set
*Tools → USB CDC On Boot → Enabled*.

## Controls

| Input | Action |
|---|---|
| Stick left / right | Turn (gentle near the centre for aiming) |
| Stick up / down | Walk forward / back |
| Press stick | Fire. Hold for auto fire |
| Hold BOOT | Automap |
| BOOT + press stick | Quit to the title (same map selected, so fire again = restart the level) |
| Title: stick left / right | Choose the starting map |
| After dying | Press stick to restart the level |

Doors open when you walk up to them. Red doors need the red keycard. Walk into the **EXIT**
switch to finish a level. Weapons switch automatically: the shotgun once you have it, your fists
when you're out of ammo.

## Settings (top of `oled_doom.ino`)

| Setting | Use |
|---|---|
| `INVERT_TURN`, `INVERT_MOVE` | A direction feels backwards with your joystick |
| `DEADZONE` | The view drifts with the stick released → raise it |
| `OLED_I2C_HZ` | 400 kHz is the SH1106 spec. Try 800000 for a higher frame rate if the picture stays clean |
| `BUZZER_PIN` | Sound effects on a passive buzzer |

## Maps

Maps are strings in `levels.h`: `#` `T` `S` walls, `D` door, `L` red door, `E` exit switch,
`P` start, `z` `i` `d` `B` monsters, `b` barrel, `h` `a` `A` `s` `k` medikit, clip, ammo box,
shotgun, red keycard. The legend is at the top of the file.

## Tests

`tests/doom_sim.cpp` compiles the sketch's `game.cpp` on a PC. It checks every map is closed
and finishable, then a bot plays the whole game through joystick inputs only (with and
without god mode), plus death/restart, quit-to-title and a random-input soak, rendering every
frame:

```
cd tests
g++ -std=c++11 -O2 -Wall -I../oled_doom doom_sim.cpp ../oled_doom/game.cpp -o doom_sim && ./doom_sim
```

Pass a folder (`./doom_sim shots`) to save screenshots as `.pbm` images.
