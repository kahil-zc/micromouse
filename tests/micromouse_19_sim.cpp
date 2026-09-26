// Runs the maze section of 19_micromouse_final.ino on random 5 x 10 mazes.
//   tests/micromouse_19_sim.sh
// The script copies the lines between "// --- MAZE (no hardware calls) ---" and
// "// --- END MAZE ---" into maze_19_section.inc, so this tests the exact planner the robot runs.
//
// Each maze is 10 across x 5 ahead or 5 across x 10 ahead; the start is the left or right corner
// of the first row and the goal the diagonally opposite corner. The robot is told none of this.
// It searches with the same calls as runStep() (read the walls of the cell it stands in at
// every stop, plan, drive one cell): to the goal, then back to the start. Then a speed run.
//
// Perfect readings: fails if it drives through a wall, loops, gets the corner or the shape wrong,
// never reaches the goal, or the speed run doesn't reach the goal. Reports how often the speed
// run is the true shortest path, and how far straight the robot drives.
// Noisy readings: every wall reading is wrong with some chance; a wall the robot drives at stops
// it (the front ToF). Fails if it loops or never gets home.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef GOAL_SIZE
#define GOAL_SIZE 1
#endif
#define MAZE_SHORT 5
#define MAZE_LONG 10
#define MAZE_W MAZE_LONG
#define MAZE_H MAZE_LONG
#define START_CORNER 0
#define LONG_SIDE 0

#include "maze_19_section.inc"

static uint8_t truth[MAZE_LONG][MAZE_LONG];  // bits 0-3: real walls
static int TW, TH, trueStartX;
static bool inTrue(int x, int y) { return x >= 0 && y >= 0 && x < TW && y < TH; }
static bool inTrueGoal(int x, int y) {
  int gx = trueStartX == 0 ? TW - GOAL_SIZE : 0;
  return x >= gx && x < gx + GOAL_SIZE && y >= TH - GOAL_SIZE;
}

static void trueWall(int x, int y, int d, bool on) {
  int nx = x + DX[d], ny = y + DY[d];
  uint8_t o = (d + 2) & 3;
  if (on) { truth[x][y] |= 1 << d; if (inTrue(nx, ny)) truth[nx][ny] |= 1 << o; }
  else    { truth[x][y] &= ~(1 << d); if (inTrue(nx, ny)) truth[nx][ny] &= ~(1 << o); }
}

// Random perfect maze plus a few loops; start cell open only ahead. Some mazes get a long
// straight corridor so "drives to the end" is tested.
static void makeMaze(bool rightCorner) {
  memset(truth, 0x0F, sizeof(truth));
  trueStartX = rightCorner ? TW - 1 : 0;
  static bool seen[MAZE_LONG][MAZE_LONG];
  memset(seen, 0, sizeof(seen));
  static int sx[100], sy[100];
  int top = 1;
  sx[0] = trueStartX; sy[0] = 1; seen[trueStartX][1] = true;
  seen[trueStartX][0] = true;
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
    int x = rand() % TW, y = 1 + rand() % (TH - 1), d = rand() % 4;
    int nx = x + DX[d], ny = y + DY[d];
    if (inTrue(nx, ny) && ny > 0) trueWall(x, y, d, false);
  }
  if (rand() % 3 == 0)  // open the whole start column: a long straight to the far wall
    for (int y = 0; y < TH - 1; y++) trueWall(trueStartX, y, 0, false);
  trueWall(trueStartX, 0, 0, false);
}

static int trueShortest() {
  static int d[MAZE_LONG][MAZE_LONG];
  for (int x = 0; x < TW; x++) for (int y = 0; y < TH; y++) d[x][y] = -1;
  static int qx[100], qy[100];
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

// Map cell -> true cell. The map is 10 x 10; the true maze sits against the start corner.
static int offX() { return trueStartX - startX; }
static bool trueWallAt(int x, int y, int d) {
  int tx = x + offX();
  if (!inTrue(tx, y)) return true;
  return truth[tx][y] & (1 << d);
}

static double noise = 0;
static bool readWall(int x, int y, int d) {
  bool w = trueWallAt(x, y, d);
  return (rand() < noise * RAND_MAX) ? !w : w;
}

// What senseWallsHere() does, from the true maze.
static uint8_t readWalls() {
  uint8_t w = 0;
  for (uint8_t i = 0; i < 3; i++) if (readWall(posX, posY, (facing + 3 + i) & 3)) w |= 1 << i;
  return w;
}
static void senseHere() {
  uint8_t walls = readWalls();
  if (showsRightCorner(walls)) walls |= readWalls();
  recordWalls(walls);
}

static bool reachedGoal;
static long straightRuns[20];  // search: cells driven straight without turning, by length
static int resets = 0;
static int run(uint8_t firstPhase) {
  resetStart();
  posX = startX; posY = 0; facing = 0;
  putWall(startX, 0, 2, true);
  phase = firstPhase;
  reachedGoal = false;
  int cells = 0, straight = 0;
  for (int steps = 0; steps < 3000; steps++) {
    if (phase != PH_FAST) senseHere();
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
    if (old == PH_SEARCH && phase == PH_HOME) reachedGoal = true;
    if (phase == PH_DONE) { if (straight) straightRuns[straight < 19 ? straight : 19]++; return cells; }
    if (n == 0) continue;
    if (phase != PH_FAST && n != 1) { printf("  drove %d cells without stopping\n", n); return -1; }
    if (dir != facing && straight) { straightRuns[straight < 19 ? straight : 19]++; straight = 0; }
    facing = dir;
    uint8_t done = 0;
    for (; done < n; done++) {
      if (trueWallAt(posX, posY, dir)) break;
      advance(dir, 1);
    }
    if (done < n) {
      if (noise == 0) { printf("  drove at a wall at (%d,%d) dir %d, phase %d\n", posX + offX(), posY, dir, phase); return -1; }
      putWall(posX, posY, dir, true);
    }
    cells += done; straight += done;
  }
  printf("  loops (phase %d)\n", phase);
  return -1;
}

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

static void setup(int m) {
  srand(m + 1);
  if (m % 4 < 2) { TW = 10; TH = 5; } else { TW = 5; TH = 10; }
  makeMaze(m % 2 == 1);
  startX = 0; cornerKnown = false; longSide = 0;
  initMaze();
}

int main() {
  const int N = 1000;
  int fails = 0, optimal = 0;
  long searchCells = 0, fastTotal = 0, shortTotal = 0;
  for (int m = 0; m < N; m++) {
    setup(m);
    int shortest = trueShortest();
    int e = run(PH_SEARCH);
    if (e < 0) { printf("maze %d: search failed\n", m); fails++; continue; }
    if (!reachedGoal) { printf("maze %d: never reached the goal\n", m); fails++; continue; }
    if ((startX == 0) != (trueStartX == 0)) { printf("maze %d: start corner wrong\n", m); fails++; continue; }
    if (longSide != 0 && (longSide == 2) != (TH == 10)) { printf("maze %d: shape wrong\n", m); fails++; continue; }
    searchCells += e;
    int f = speedRun();
    if (f < 0) { printf("maze %d: speed run failed\n", m); fails++; continue; }
    fastTotal += f; shortTotal += shortest;
    if (f == shortest) optimal++;
    if (run(PH_RETURN) < 0) { printf("maze %d: return failed\n", m); fails++; continue; }
  }
  printf("GOAL_SIZE %d, PERFECT READINGS: %d mazes (5 x 10 both ways round, both corners): %d failures\n",
         GOAL_SIZE, N, fails);
  printf("  search to goal and back: %.1f cells; speed run %.2f cells vs shortest %.2f (shortest in %d / %d)\n",
         (double)searchCells / N, (double)fastTotal / N, (double)shortTotal / N, optimal, N);
  printf("  straights driven while searching, by length:");
  for (int i = 1; i < 10; i++) printf(" %d:%ld", i, straightRuns[i]);
  printf("\n");

  static const double noises[] = { 0.01, 0.03 };
  for (int k = 0; k < 2; k++) {
    noise = noises[k];
    int stuck = 0, goalOk = 0, fastOk = 0;
    resets = 0;
    for (int m = 0; m < N; m++) {
      setup(m);
      srand(7919 * (m + 1) + k);
      if (run(PH_SEARCH) < 0) { printf("noisy maze %d: search never got home\n", m); stuck++; continue; }
      if (!reachedGoal || (startX == 0) != (trueStartX == 0)) continue;
      goalOk++;
      if (speedRun() >= 0) fastOk++;
      if (run(PH_RETURN) < 0) { printf("noisy maze %d: return never got home\n", m); stuck++; }
    }
    fails += stuck;
    printf("  %.0f%% OF WALL READINGS WRONG: %d stuck; reached the real goal %d / %d, speed run reached it %d; %d map resets\n",
           noise * 100, stuck, goalOk, N, fastOk, resets);
  }
  return fails ? 1 : 0;
}
