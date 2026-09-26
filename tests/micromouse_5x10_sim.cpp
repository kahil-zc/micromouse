// Runs the maze section of 16_micromouse_5x10.ino on random 10-across x 5-ahead mazes.
//   tests/micromouse_5x10_sim.sh
// The script copies the lines between "// --- MAZE (no hardware calls) ---" and
// "// --- END MAZE ---" into maze_5x10_section.inc, so this tests the exact planner the robot runs.
//
// Each maze is 5 x 10 cells, either way round (the robot's map is a 10 x 10 box), has a 2x2 goal
// room with one entrance somewhere, every other post has a wall, and the start is in the left or
// right corner; the robot is told none of this. It explores (the same
// recordWalls / plan / advance calls as runStep, walls read from the true maze including the
// the robot only reads the three walls of the cell it stands in), then does a speed run and
// drives home.
// Fails if it drives through a wall, loops, reads a cell twice, picks the wrong goal, or the speed
// run is not the true shortest path.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAZE_W 10
#define MAZE_H 10
#define START_CORNER 0
#define MAX_IMPROVE_CELLS 60

#include "maze_5x10_section.inc"

#define MAXT 10
static uint8_t truth[MAXT][MAXT];  // bits 0-3: real walls
static int TW, TH;                 // real maze size
static int trueGoalX, trueGoalY, trueStartX;
static bool inTrue(int x, int y) { return x >= 0 && y >= 0 && x < TW && y < TH; }

static void trueWall(int x, int y, int d, bool on) {
  int nx = x + DX[d], ny = y + DY[d];
  uint8_t o = (d + 2) & 3;
  if (on) { truth[x][y] |= 1 << d; if (inTrue(nx, ny)) truth[nx][ny] |= 1 << o; }
  else    { truth[x][y] &= ~(1 << d); if (inTrue(nx, ny)) truth[nx][ny] &= ~(1 << o); }
}

static bool inTrueGoal(int x, int y) {
  return x >= trueGoalX && x <= trueGoalX + 1 && y >= trueGoalY && y <= trueGoalY + 1;
}

static bool postHasWall(int px, int py) {
  return (truth[px][py] & 2) || (truth[px][py] & 1) || (truth[px + 1][py + 1] & 8) || (truth[px + 1][py + 1] & 4);
}

static void makeMaze(bool rightCorner) {
  memset(truth, 0x0F, sizeof(truth));
  trueStartX = rightCorner ? TW - 1 : 0;
  do {
    trueGoalX = rand() % (TW - 1);
    trueGoalY = 1 + rand() % (TH - 2);
  } while (inTrueGoal(trueStartX, 0));

  static bool seen[MAXT][MAXT];
  memset(seen, 0, sizeof(seen));
  for (int x = trueGoalX; x <= trueGoalX + 1; x++)
    for (int y = trueGoalY; y <= trueGoalY + 1; y++) seen[x][y] = true;
  static int sx[MAXT * MAXT], sy[MAXT * MAXT];
  int top = 1;
  sx[0] = trueStartX; sy[0] = 0; seen[trueStartX][0] = true;
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
  // Goal room: open inside, one entrance to a cell outside it.
  for (int x = trueGoalX; x <= trueGoalX + 1; x++)
    for (int y = trueGoalY; y <= trueGoalY + 1; y++)
      for (int d = 0; d < 4; d++) trueWall(x, y, d, !inTrueGoal(x + DX[d], y + DY[d]));
  static const int ex[8][3] = { {0,0,2},{1,0,2},{1,0,1},{1,1,1},{1,1,0},{0,1,0},{0,1,3},{0,0,3} };
  for (;;) {
    const int *e = ex[rand() % 8];
    int x = trueGoalX + e[0], y = trueGoalY + e[1];
    if (!inTrue(x + DX[e[2]], y + DY[e[2]])) continue;
    trueWall(x, y, e[2], false);
    break;
  }
  // A few loops, never leaving a post with no wall (only the goal post may be bare).
  for (int i = 0; i < 8; i++) {
    int x = rand() % TW, y = rand() % TH, d = rand() % 4;
    int nx = x + DX[d], ny = y + DY[d];
    if (!inTrue(nx, ny) || inTrueGoal(x, y) || inTrueGoal(nx, ny) || !(truth[x][y] & (1 << d))) continue;
    trueWall(x, y, d, false);
    bool ok = true;
    for (int px = 0; px < TW - 1 && ok; px++)
      for (int py = 0; py < TH - 1 && ok; py++)
        if (!(px == trueGoalX && py == trueGoalY) && !postHasWall(px, py)) ok = false;
    if (!ok) trueWall(x, y, d, true);
  }
  // Start cell: only ahead open.
  trueWall(trueStartX, 0, 1, true);
  trueWall(trueStartX, 0, 3, true);
  trueWall(trueStartX, 0, 0, false);
}

static int reachable = 0;
static int trueShortest() {
  static int d[MAXT][MAXT];
  for (int x = 0; x < TW; x++) for (int y = 0; y < TH; y++) d[x][y] = -1;
  static int qx[MAXT * MAXT], qy[MAXT * MAXT];
  int h = 0, t = 0, best = -1;
  d[trueStartX][0] = 0; qx[t] = trueStartX; qy[t++] = 0;
  while (h < t) {
    int x = qx[h], y = qy[h++];
    if (inTrueGoal(x, y) && best < 0) best = d[x][y];
    for (int k = 0; k < 4; k++) {
      if (truth[x][y] & (1 << k)) continue;
      int nx = x + DX[k], ny = y + DY[k];
      if (d[nx][ny] >= 0) continue;
      d[nx][ny] = d[x][y] + 1; qx[t] = nx; qy[t++] = ny;
    }
  }
  reachable = t;
  return best;
}

static int offX() { return trueStartX - startX; }
static bool trueWallAt(int x, int y, int d) {
  int tx = x + offX();
  if (!inTrue(tx, y)) return true;
  return truth[tx][y] & (1 << d);
}

// What senseWallsHere() would report, from the true maze.
static int senses = 0;
static void senseHere() {
  uint8_t left = (facing + 3) & 3, right = (facing + 1) & 3;
  recordWalls(trueWallAt(posX, posY, left), trueWallAt(posX, posY, facing), trueWallAt(posX, posY, right));
  senses++;
}

// Same order as runStep(). Returns cells driven (stops counted separately), or -1 on failure.
static int run(uint8_t firstPhase, int &stops) {
  resetStart();
  posX = startX; posY = 0; facing = 0;
  putWall(startX, 0, 2, true, false);
  phase = firstPhase;
  exploreX = exploreY = -1;
  int cells = 0;
  stops = 0;
  for (int steps = 0; steps < 2000; steps++) {
    bool mapping = phase == PH_EXPLORE || phase == PH_IMPROVE || phase == PH_HOME;
    if (mapping && !visited(posX, posY)) { senseHere(); newCells++; }
    uint8_t dir = facing;
    uint8_t n = plan(dir);
    if (noRoute) { printf("  no route (phase %d)\n", phase); return -1; }
    if (phase == PH_DONE) return cells;
    if (n == 0) continue;
    for (uint8_t i = 0; i < n; i++) {
      if (trueWallAt(posX, posY, dir)) {
        printf("  drove through a wall at (%d,%d) dir %d, phase %d\n", posX + offX(), posY, dir, phase);
        return -1;
      }
      advance(dir, 1);
    }
    facing = dir;
    cells += n;
    stops++;
  }
  printf("  loops (phase %d)\n", phase);
  return -1;
}

int main() {
  const int N = 1000;
  int fails = 0;
  long exploreCells = 0, exploreStops = 0, reachableTotal = 0, sensedTotal = 0;
  for (int m = 0; m < N; m++) {
    srand(m + 1);
    int shortest;
    do {  // every cell must be reachable (a goal room next to the start can wall the rest off)
      if (m % 4 < 2) { TW = 10; TH = 5; } else { TW = 5; TH = 10; }
    makeMaze(m % 2 == 1);
      shortest = trueShortest();
    } while (reachable != TW * TH);
    if (shortest < 0) { printf("maze %d: generator made no path\n", m); fails++; continue; }

    initMaze();
    startX = 0; cornerKnown = false;
    int stops;
    senses = 0;
    int e = run(PH_EXPLORE, stops);
    if (e < 0) { printf("maze %d: explore failed\n", m); fails++; continue; }
    exploreCells += e; exploreStops += stops; reachableTotal += reachable;

    if (senses > TW * TH) { printf("maze %d: read %d cells for %d cells\n", m, senses, TW * TH); fails++; continue; }
    sensedTotal += senses;
    if ((startX == 0) != (trueStartX == 0)) { printf("maze %d: start corner wrong\n", m); fails++; continue; }
    if (!goalKnown || goalX + offX() != trueGoalX || goalY != trueGoalY) {
      printf("maze %d: goal wrong\n", m); fails++; continue;
    }

    // Speed run: count cells to the goal only.
    resetStart();
    posX = startX; posY = 0; facing = 0; phase = PH_FAST;
    int fastCells = 0;
    bool bad = false;
    for (int steps = 0; steps < 200; steps++) {
      uint8_t dir = facing;
      uint8_t n = plan(dir);
      if (phase != PH_FAST) break;
      if (n == 0) { bad = true; break; }
      for (uint8_t i = 0; i < n; i++) {
        if (trueWallAt(posX, posY, dir)) { bad = true; break; }
        advance(dir, 1);
      }
      facing = dir; fastCells += n;
    }
    if (bad || phase != PH_RETURN) { printf("maze %d: speed run failed\n", m); fails++; continue; }
    if (fastCells != shortest) { printf("maze %d: speed run %d cells, shortest %d\n", m, fastCells, shortest); fails++; continue; }
    int s2;
    if (run(PH_RETURN, s2) < 0) { printf("maze %d: return failed\n", m); fails++; continue; }
  }
  printf("%d mazes (5 x 10 both ways round, goal room and start corner found by the robot): %d failures\n", N, fails);
  printf("search + home: drove %.1f cells, read %.1f of %.1f cells, %.1f stops; no cell read twice\n",
         (double)exploreCells / N, (double)sensedTotal / N, (double)reachableTotal / N,
         (double)exploreStops / N);
  return fails ? 1 : 0;
}
