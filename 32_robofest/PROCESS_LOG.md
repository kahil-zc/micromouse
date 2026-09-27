# 32_robofest — process log

What was asked, what was done, how it was checked.

## 1. Request

- Drive with **14_floodfill_final's** methods and settings only.
- Centre in the cell around turns. "If you want to turn, just turn", with no 180°/reverse/forward
  moves, unless that would be unsafe.
- **Search**: strictly cell by cell (18 cm per move). **Speed run**: centre at all times; whole
  straights are allowed.
- Competition maze: *ROBOFEST 2026 University Preliminary Rounds* (PDF). The start can be **any of
  the 4 corners**.
- Have 10 agents review the code, log the process, and hand over the file.

## 2. The maze

- The PDF was rendered at 400 dpi. Every wall segment was classified by its line thickness: thick
  = 10 px, thin grid line = 1 px, with nothing in between. The result was saved as
  `tests/mazes/ROBOFEST2026.maz` (the same hex format as the classic contest mazes).
- Checks: every shared wall agrees on both sides, the goal is the open centre 2×2 (H8–I9), and the
  grey block H10–I11 is a closed box.
- Start corners: A16 opens East, P16 North, A1 South, P1 West. Each has exactly one opening. **From
  all four corners the maze lies on the robot's left.**
- The five classic contest mazes (APEC2002, HITEL01/02, SEOUL01/02) were copied from
  github.com/adam2392/ieee_micromouse (`Test Mazes/`) for extra tests. That repo's own robot uses
  point turns and a simple PID, which doesn't fit this robot's 79 mm rear overhang, so no driving
  code was taken from it.

## 3. Design

**Which corner.** The robot never knows its corner. It only needs to know whether the maze
extends to its right or its left.
- In the start column, one side wall of every cell is the outer wall. So the first cell with
  exactly one side open tells it which side the maze is on.
- If the maze is on the left, the map is kept as a mirror image: left and right readings and turns
  are swapped. The goal is then always the centre (7..8, 7..8) and 14's flood fill is unchanged.

**Search.** This is 14's `searchStep`: one cell per move.
- It stops at the decision point, reads the three walls, runs the flood fill, turns and drives
  exactly one cell.
- A new reading **replaces** the old one. In 14 walls were sticky, so one misread wall stayed
  forever and could send the robot the long way round (the "turns 180 instead of left" symptom).
- The position counts only **cells really driven**. 14 added the cell even after a stall or an
  early front-wall stop, which puts every later wall in the wrong cell.
- The map is saved at every cell.
- If the first 12 readings of a search disagree with the saved map twice, the saved map belongs to
  another corner and a new map is started.

**Speed run.** This is 14's `fastStep`: known walls only, whole straights at `SPEED_RUN_PWM`, and
`driveStraight(center = true)`, so it centres on the side walls all the time. If an unknown wall
blocks the path, it continues cell by cell like a search.

**Driving changes to 14.** Everything else is byte-identical.
1. **Arc turns**:
   - Before an arc, the side ToF reading taken in `prepareArc` must show the turn side open.
     Otherwise the robot doesn't turn (`TURN_BLOCKED`), the wall goes on the map and it replans.
   - It shuffles sideways before an arc only when the rear corner would come within 3 mm of the
     outer wall (14 used 6 mm, with a target of 9 → 6). Otherwise it just turns.
2. **U-turns** (`uTurn`, which replaces `spinSafely(180)` in `executeTurn`):
   - **Dead end:** as in 14, pull up to 49 mm from the wall, spin, and back into the wall to square
     up.
   - **No wall ahead:** spin on the cell centre. 14 spun at the decision point, where the rear
     corners (93 mm swing) pass 84.6 mm from the back posts of the cell.
   - **Walls on both sides:** the robot needs a 12.5 mm offset. It gets there by turning 12° and
     driving forward on the way to the spin spot, instead of 14's reverse-and-forward shuffle,
     which is kept only when there is no room ahead.

## 4. Checks

**Firmware build.** Built for the Nano with avr-gcc 7.3 and LTO:

| Flash | RAM |
|---|---|
| 30,334 B (under the 30,720 B old-bootloader limit) | 1,412 B |

**Simulator.** `tests/robofest_sim.sh` compiles the sketch's MAZE section (the real code). A
perfect robot drives in the true maze. At every move the simulator checks that the map position
matches the real one, including the mirroring.

| Test | Result |
|---|---|
| ROBOFEST2026, all 4 corners | maze on the left; fast run 45–46 cells = true shortest, proven after 2 searches |
| 5 classic contest mazes, all proper start corners | all pass, fast run = true shortest |
| 20 pairs: saved map from another corner, not wiped | mismatch noticed, new map, search and fast run still correct |
| 400 random mazes (1,276 starts) | all pass |

**Review.** 10 agents each reviewed one area of the code. Their findings and what was changed
because of them are listed in section 5.

## 5. Review findings and fixes

(filled in below)
