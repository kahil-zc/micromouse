# Micromouse firmware (16×16 flood fill)

Arduino Nano (ATmega328P). Open `main/main.ino` in the Arduino IDE, select **Arduino Nano**
and install the **VL53L0X** library by Pololu. Measured build: 22.1 KB flash, 1347 B static RAM.

| File | Layer | Job |
|---|---|---|
| `sensors.*` | Perception | VL53L0X addressing, non-blocking reads, wall thresholds |
| `encoders.*`, `imu.*` | Estimation | Quadrature counts (both edges of A), gyro heading (±500 dps) |
| `maze.*` | Planning | Wall map, flood fill, next move — no hardware calls, tested on PC |
| `motion.*`, `motors.*` | Motion | Trapezoidal profiles, forward + rotation controllers, wall centring, front-wall stop |
| `storage.*` | — | EEPROM map (magic `0xA9`) and calibration (magic `0x5C`) |
| `config.h` | — | Every pin and constant; `CAL` marks values to check on the robot |

## Button

Short press = next mode, long press (0.8 s, LED goes solid) = start. The LED blinks the mode number.
Place the robot in the start cell with its back touching the back wall.

| # | Mode | What it does |
|---|---|---|
| 1 | SEARCH | Explore to the goal and back home; map saved to EEPROM after every cell |
| 2–4 | FAST 1/2/3 | Shortest known path at 400 / 550 / 700 mm/s, then drives itself home |
| 5 | CALIBRATE | In the start cell: learns centred side readings and front-stop reading |
| 6 | SENSOR CHECK | LED = front wall; readings and ASCII map on Serial; button exits |
| 7 | HW CHECK | Polarity checks, then 1 cell + 4 right + 4 left turns |
| 8 | CLEAR MAP | Forget the stored maze |

After a run home: **one long flash** = the stored path is proven shortest; **three blinks** = run
SEARCH again to try to find a shorter one.

Errors blink fast N times; press the button to go back to the menu.
1 ToF missing · 2 moved during gyro calibration · 3 crash/stall · 4 no path (search first) ·
5 calibration readings out of range · 6/7 left/right encoder wrong way · 8 gyro wrong way.
For 6/7: if the wheel turned backwards flip `MOTOR_x_INVERT`, otherwise flip `ENC_x_INVERT`.

## Bring-up order

1. **HW CHECK** on the floor until it finishes with a long flash. Tune `GYRO_SCALE` until the
   four turns each way end square.
2. **Distance**: drive a known distance, adjust `COUNTS_PER_CELL`.
3. **CALIBRATE** on your black test cells (and again on competition day).
4. **SENSOR CHECK** in a few cells to confirm walls are detected.
5. Tune `KFF_V`, `KFF_S`, `KFF_W` so moves track without large correction, then SEARCH, then FAST 1.

The robot works out which bottom corner it started in: it assumes the left-hand corner until it
sees an opening to the west of column 0.

## Tests

```
cd tests
g++ -std=c++11 -O2 -Wall -I../main maze_sim.cpp ../main/maze.cpp -o maze_sim && ./maze_sim
```

600 random competition-style mazes (both start corners, single-entry centre goal, loops). It runs
the same search / fast-run loop as `main.ino` and fails if the robot drives through a wall, if a
fast run uses an unobserved wall, or if a path marked proven isn't the true shortest.
