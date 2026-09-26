// Runs the maze section of 18_micromouse_corner.ino on random 10-across x 5-ahead mazes.
//   tests/micromouse_18_sim.sh
// The script copies the lines between "// --- MAZE (no hardware calls) ---" and
// "// --- END MAZE ---" into maze_18_section.inc, so this tests the exact planner the robot runs.
//
// The start is the left or right corner of the first row (the robot is not told which); the goal
// is the opposite corner of the far row. The robot searches with the same calls as runStep()
// (read the three walls of the cell it stands in, every cell, plan, drive one cell), then does a
// speed run and drives home.
//
// Part 1, perfect readings. Fails if it drives through a wall, loops, gets the start corner
// wrong, or the speed run is not the true shortest path.
// Part 2, noisy readings: every wall reading is wrong with some chance. A reading that disagrees
// with the map is read again, as senseWallsHere() does, and a wall the robot drives at stops it
// (the front ToF, as in driveStraight). Fails if it loops or never gets home.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef GOAL_SIZE
#define GOAL_SIZE 1
#endif
#define MAZE_W 10
#define MAZE_H 5
#define START_CORNER 0
#define MAX_IMPROVE_CELLS 60

#include "maze_18_section.inc"

static uint8_t truth[MAZE_W][MAZE_H];  // bits 0-3: real walls
static int trueStartX;
static bool inTrue(int x, int y) { return x >= 0 && y >= 0 && x < MAZE_W && y < MAZE_H; }
static bool inTrueGoal(int x, int y) {
  int gx = trueStartX == 0 ? MAZE_W - GOAL_SIZE : 0;
  return x >= gx && x < gx + GOAL_SIZE && y >= MAZE_H - GOAL_SIZE;
}

static void trueWall(int x, int y, int d, bool on) {
  int nx = x + DX[d], ny = y + DY[d];
  uint8_t o = (d + 2) & 3;
  if (on) { truth[x][y] |= 1 << d; if (inTrue(nx, ny)) truth[nx][ny] |= 1 << o; }
  else    { truth[x][y] &= ~(1 << d); if (inTrue(nx, ny)) truth[nx][ny] &= ~(1 << o); }
}

// Random perfect maze plus a few loops; start cell open only ahead.
static void makeMaze(bool rightCorner) {
  memset(truth, 0x0F, sizeof(truth));
  trueStartX = rightCorner ? MAZE_W - 1 : 0;
  static bool seen[MAZE_W][MAZE_H];
  memset(seen, 0, sizeof(seen));
  static int sx[MAZE_W * MAZE_H], sy[MAZE_W * MAZE_H];
  int top = 1;
  sx[0] = trueStartX; sy[0] = 1; seen[trueStartX][1] = true;
  seen[trueStartX][0] = true;  // start cell joined below
  while (top) {
    int x = sx[top - 1], y = sy[top - 1];
    int opts[4], n = 0;
    for (int d = 0; d < 4; d++) {
      int nx = x + DX[d], ny = y + DY[d];
      if (inTrue(nx, ny) && !seen[nx][ny]) opts[n++] = d;
    }
    if (!n) { top--; continue; }
    int d = opts[rand() % n];
    trueWall(x, y, d, false);
    int nx = x + DX[d], ny = y + DY[d];
    seen[nx][ny] = true; sx[top] = nx; sy[top] = ny; top++;
  }
  for (int i = 0; i < 6; i++) {
    int x = rand() % MAZE_W, y = 1 + rand() % (MAZE_H - 1), d = rand() % 4;
    int nx = x + DX[d], ny = y + DY[d];
    if (inTrue(nx, ny) && ny > 0) trueWall(x, y, d, false);
  }
  trueWall(trueStartX, 0, 0, false);
}

static int trueShortest() {
  static int d[MAZE_W][MAZE_H];
  for (int x = 0; x < MAZE_W; x++) for (int y = 0; y < MAZE_H; y++) d[x][y] = -1;
  static int qx[MAZE_W * MAZE_H], qy[MAZE_W * MAZE_H];
  int h = 0, t = 0;
  d[trueStartX][0] = 0; qx[t] = trueStartX; qy[t++] = 0;
  while (h < t) {
    int x = qx[h], y = qy[h++];
    if (inTrueGoal(x, y)) return d[x][y];
    for (int k = 0; k < 4; k++) {
      if (truth[x][y] & (1 << k)) continue;
      int nx = x + DX[k], ny = y + DY[k];
      if (d[nx][ny] >= 0) continue;
      d[nx][ny] = d[x][y] + 1; qx[t] = nx; qy[t++] = ny;
    }
  }
  return -1;
}

static int offX() { return trueStartX - startX; }
static bool trueWallAt(int x, int y, int d) {
  int tx = x + offX();
  if (!inTrue(tx, y)) return true;
  return truth[tx][y] & (1 << d);
}

static double noise = 0;  // chance that one wall reading is wrong
static bool readWall(int x, int y, int d) {
  bool w = trueWallAt(x, y, d);
  return (rand() < noise * RAND_MAX) ? !w : w;
}

// What senseWallsHere() does, from the true maze.
static int senses = 0, corrections = 0, resets = 0;
static uint8_t readWalls() {
  uint8_t w = 0;
  for (uint8_t i = 0; i < 3; i++) if (readWall(posX, posY, (facing + 3 + i) & 3)) w |= 1 << i;
  return w;
}
static void senseHere() {
  uint8_t walls = readWalls();
  if (!visited(posX, posY)) {
    senses++;
    if (showsRightCorner(walls)) walls |= readWalls();
  } else if (walls != mapWalls()) {
    uint8_t same = ~(walls ^ readWalls());
    walls = (walls & same) | (mapWalls() & ~same);
    if (walls != mapWalls()) corrections++;
  }
  recordWalls(walls);
}

// Same order as runStep(). Returns cells driven, or -1 on failure. reachedGoal: passed through it.
static bool reachedGoal;
static int run(uint8_t firstPhase) {
  resetStart();
  posX = startX; posY = 0; facing = 0;
  putWall(startX, 0, 2, true);
  phase = firstPhase;
  newCells = 0;
  reachedGoal = false;
  int cells = 0;
  for (int steps = 0; steps < 3000; steps++) {
    if (phase != PH_FAST) {
      bool wasNew = !visited(posX, posY);
      senseHere();
      if (wasNew && visited(posX, posY)) newCells++;
    }
    uint8_t dir = facing;
    uint8_t old = phase;
    uint8_t n = plan(dir);
    if (noRoute) {
      if (noise == 0) { printf("  no route (phase %d)\n", phase); return -1; }
      uint8_t before = reopens;
      recoverRoute();
      if (reopens == before) resets++;
      continue;
    }
    if (old == PH_SEARCH && phase == PH_IMPROVE) reachedGoal = true;
    if (phase == PH_DONE) return cells;
    if (n == 0) continue;
    if (phase != PH_FAST && n != 1) { printf("  drove %d cells without stopping\n", n); return -1; }
    facing = dir;
    uint8_t done = 0;
    for (; done < n; done++) {
      if (trueWallAt(posX, posY, dir)) break;
      advance(dir, 1);
    }
    if (done < n) {
      if (noise == 0) { printf("  drove at a wall at (%d,%d) dir %d, phase %d\n", posX + offX(), posY, dir, phase); return -1; }
      putWall(posX, posY, dir, true);  // the front ToF stops it
    }
    cells += done;
  }
  printf("  loops (phase %d)\n", phase);
  return -1;
}

// Speed run to the goal: cells driven, or -1 if it never got there.
static int speedRun() {
  resetStart();
  posX = startX; posY = 0; facing = 0; phase = PH_FAST;
  int fastCells = 0;
  for (int steps = 0; steps < 200; steps++) {
    uint8_t dir = facing;
    uint8_t n = plan(dir);
    if (phase != PH_FAST) break;
    if (n == 0) return -1;
    facing = dir;
    for (uint8_t i = 0; i < n; i++) {
      if (trueWallAt(posX, posY, dir)) {
        if (noise == 0) return -1;
        putWall(posX, posY, dir, true);
        break;
      }
      advance(dir, 1);
      fastCells++;
    }
  }
  return phase == PH_RETURN ? fastCells : -1;
}

int main() {
  const int N = 1000;
  int fails = 0;
  long searchCells = 0, mapped = 0, fastTotal = 0;
  for (int m = 0; m < N; m++) {
    srand(m + 1);
    makeMaze(m % 2 == 1);
    int shortest = trueShortest();
    initMaze();
    startX = 0; cornerKnown = false;
    senses = 0;
    int e = run(PH_SEARCH);
    if (e < 0) { printf("maze %d: search failed\n", m); fails++; continue; }
    if (!reachedGoal) { printf("maze %d: never reached the goal\n", m); fails++; continue; }
    if ((startX == 0) != (trueStartX == 0)) { printf("maze %d: start corner wrong\n", m); fails++; continue; }
    if (senses > MAZE_W * MAZE_H) { printf("maze %d: mapped %d cells\n", m, senses); fails++; continue; }
    searchCells += e; mapped += senses;
    int f = speedRun();
    if (f != shortest) { printf("maze %d: speed run %d cells, shortest %d\n", m, f, shortest); fails++; continue; }
    fastTotal += f;
    if (run(PH_RETURN) < 0) { printf("maze %d: return failed\n", m); fails++; continue; }
  }
  printf("GOAL_SIZE %d, PERFECT READINGS: %d mazes (10 x 5, start corner found by the robot): %d failures\n",
         GOAL_SIZE, N, fails);
  printf("  search + home: drove %.1f cells, mapped %.1f of 50 cells; speed run %.1f cells = always the shortest\n",
         (double)searchCells / N, (double)mapped / N, (double)fastTotal / N);

  static const double noises[] = { 0.01, 0.03 };
  for (int k = 0; k < 2; k++) {
    noise = noises[k];
    int stuck = 0, goalOk = 0, fastOk = 0, fastShortest = 0;
    corrections = 0; resets = 0;
    for (int m = 0; m < N; m++) {
      srand(m + 1);
      makeMaze(m % 2 == 1);
      int shortest = trueShortest();
      srand(7919 * (m + 1) + k);
      initMaze();
      startX = 0; cornerKnown = false;
      if (run(PH_SEARCH) < 0) { printf("noisy maze %d: search never got home\n", m); stuck++; continue; }
      if (!reachedGoal || (startX == 0) != (trueStartX == 0)) continue;
      goalOk++;
      int f = speedRun();
      if (f < 0) continue;
      fastOk++;
      if (f == shortest) fastShortest++;
      if (run(PH_RETURN) < 0) { printf("noisy maze %d: return never got home\n", m); stuck++; }
    }
    fails += stuck;
    printf("  %.0f%% OF WALL READINGS WRONG: %d stuck; reached the real goal %d / %d, speed run reached it %d (%d shortest); "
           "%d map corrections, %d map resets\n",
           noise * 100, stuck, goalOk, N, fastOk, fastShortest, corrections, resets);
  }
  return fails ? 1 : 0;
}
