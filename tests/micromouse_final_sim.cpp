// Runs the maze section of 15_micromouse_final.ino on random 18x18 mazes.
//   tests/micromouse_final_sim.sh
// The script copies the lines between "// --- MAZE (no hardware calls) ---" and
// "// --- END MAZE ---" into maze_final_section.inc, so this tests the exact planner the robot runs.
//
// Each maze: the goal is a 2x2 room with one entrance, usually in the centre, sometimes
// elsewhere; every other post has a wall (competition rule); the start is in the left or right
// corner. The robot is not told where the goal or its corner is. It searches (the same
// recordWalls / plan / advance calls as runStep, with walls read from the true maze including
// the front ToF's look two cells ahead), then does a speed run and drives home.
// Fails if it drives through a wall, loops, picks the wrong goal, or the speed run needs a wall
// it never saw.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAZE_SIZE 18
#define MAX_IMPROVE_CELLS 150

#include "maze_final_section.inc"

static uint8_t truth[MAZE_SIZE][MAZE_SIZE];  // bits 0-3: real walls
static int trueGoalX, trueGoalY, trueStartX;

static void trueWall(int x, int y, int d, bool on) {
  int nx = x + DX[d], ny = y + DY[d];
  uint8_t o = (d + 2) & 3;
  if (on) { truth[x][y] |= 1 << d; if (inMaze(nx, ny)) truth[nx][ny] |= 1 << o; }
  else    { truth[x][y] &= ~(1 << d); if (inMaze(nx, ny)) truth[nx][ny] &= ~(1 << o); }
}

static bool inTrueGoal(int x, int y) {
  return x >= trueGoalX && x <= trueGoalX + 1 && y >= trueGoalY && y <= trueGoalY + 1;
}

// Does any wall touch the post at the top-right corner of (px, py)?
static bool postHasWall(int px, int py) {
  return (truth[px][py] & 2) || (truth[px][py] & 1) || (truth[px + 1][py + 1] & 8) || (truth[px + 1][py + 1] & 4);
}

static void makeMaze(bool centreGoal, bool rightCorner) {
  memset(truth, 0x0F, sizeof(truth));
  int c = MAZE_SIZE / 2 - 1;
  trueGoalX = centreGoal ? c : 2 + rand() % (MAZE_SIZE - 5);
  trueGoalY = centreGoal ? c : 2 + rand() % (MAZE_SIZE - 5);
  trueStartX = rightCorner ? MAZE_SIZE - 1 : 0;

  static bool seen[MAZE_SIZE][MAZE_SIZE];
  memset(seen, 0, sizeof(seen));
  for (int x = trueGoalX; x <= trueGoalX + 1; x++)
    for (int y = trueGoalY; y <= trueGoalY + 1; y++) seen[x][y] = true;
  static int sx[MAZE_SIZE * MAZE_SIZE], sy[MAZE_SIZE * MAZE_SIZE];
  int top = 1;
  sx[0] = trueStartX; sy[0] = 0; seen[trueStartX][0] = true;
  while (top) {
    int x = sx[top - 1], y = sy[top - 1];
    int opts[4], n = 0;
    for (int d = 0; d < 4; d++) {
      int nx = x + DX[d], ny = y + DY[d];
      if (inMaze(nx, ny) && !seen[nx][ny]) opts[n++] = d;
    }
    if (!n) { top--; continue; }
    int d = opts[rand() % n];
    trueWall(x, y, d, false);
    int nx = x + DX[d], ny = y + DY[d];
    seen[nx][ny] = true; sx[top] = nx; sy[top] = ny; top++;
  }
  // Goal room: open inside, one entrance.
  for (int x = trueGoalX; x <= trueGoalX + 1; x++)
    for (int y = trueGoalY; y <= trueGoalY + 1; y++)
      for (int d = 0; d < 4; d++) trueWall(x, y, d, !inTrueGoal(x + DX[d], y + DY[d]));
  static const int ex[8][3] = { {0,0,2},{1,0,2},{1,0,1},{1,1,1},{1,1,0},{0,1,0},{0,1,3},{0,0,3} };
  const int *e = ex[rand() % 8];
  trueWall(trueGoalX + e[0], trueGoalY + e[1], e[2], false);
  // Loops, but never leave a post with no wall (only the goal post may be bare).
  for (int i = 0; i < 40; i++) {
    int x = rand() % MAZE_SIZE, y = rand() % MAZE_SIZE, d = rand() % 4;
    int nx = x + DX[d], ny = y + DY[d];
    if (!inMaze(nx, ny) || inTrueGoal(x, y) || inTrueGoal(nx, ny) || !(truth[x][y] & (1 << d))) continue;
    trueWall(x, y, d, false);
    bool ok = true;
    for (int px = 0; px < MAZE_SIZE - 1 && ok; px++)
      for (int py = 0; py < MAZE_SIZE - 1 && ok; py++)
        if (!(px == trueGoalX && py == trueGoalY) && !postHasWall(px, py)) ok = false;
    if (!ok) trueWall(x, y, d, true);
  }
  // Start cell: only north open.
  trueWall(trueStartX, 0, 1, true);
  trueWall(trueStartX, 0, 3, true);
  trueWall(trueStartX, 0, 0, false);
}

static int trueShortest() {
  static int d[MAZE_SIZE][MAZE_SIZE];
  for (int x = 0; x < MAZE_SIZE; x++) for (int y = 0; y < MAZE_SIZE; y++) d[x][y] = -1;
  static int qx[MAZE_SIZE * MAZE_SIZE], qy[MAZE_SIZE * MAZE_SIZE];
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

// Robot coordinates -> true maze coordinates.
static int offX() { return trueStartX - startX; }
static bool trueWallAt(int x, int y, int d) {
  int tx = x + offX();
  if (tx < 0 || tx >= MAZE_SIZE) return true;
  return truth[tx][y] & (1 << d);
}

// What senseWallsHere() would report, from the true maze.
static void senseHere() {
  uint8_t left = (facing + 3) & 3, right = (facing + 1) & 3;
  bool wallF = trueWallAt(posX, posY, facing);
  uint8_t aheadWall = 0;
  bool aheadOpen = false;
  if (!wallF) {
    int x1 = posX + DX[facing], y1 = posY + DY[facing];
    if (trueWallAt(x1, y1, facing)) aheadWall = 1;
    else {
      aheadOpen = true;
      int x2 = x1 + DX[facing], y2 = y1 + DY[facing];
      if (inMaze(x2 + offX(), y2) && trueWallAt(x2, y2, facing)) aheadWall = 2;
    }
  }
  recordWalls(trueWallAt(posX, posY, left), wallF, trueWallAt(posX, posY, right), aheadWall, aheadOpen);
}

// Same order as runStep(). Returns cells driven, or -1 on failure.
static int run(uint8_t firstPhase, int &maxSteps) {
  posX = startX; posY = 0; facing = 0;
  putWall(startX, 0, 2, true, false);
  phase = firstPhase;
  newCells = 0;
  int cells = 0;
  for (int steps = 0; steps < 3000; steps++) {
    bool searching = phase == PH_FIND || phase == PH_IMPROVE || phase == PH_HOME;
    if (searching && !visited(posX, posY)) { senseHere(); newCells++; }
    uint8_t dir = facing;
    uint8_t n = plan(dir);
    if (noRoute) { printf("  no route (phase %d)\n", phase); return -1; }
    if (phase == PH_DONE) { if (steps > maxSteps) maxSteps = steps; return cells; }
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
  }
  printf("  loops (phase %d)\n", phase);
  return -1;
}

int main() {
  const int N = 400;
  int fails = 0, proven = 0, maxSteps = 0;
  long searchCells = 0;
  for (int m = 0; m < N; m++) {
    srand(m + 1);
    bool centre = m % 4 != 3;      // 3 in 4 mazes have the goal in the centre
    bool right = m % 2 == 1;       // half start in the right-hand corner
    makeMaze(centre, right);
    int shortest = trueShortest();
    if (shortest < 0) { printf("maze %d: generator made no path\n", m); fails++; continue; }

    initMaze();
    startX = 0; cornerKnown = false;
    int s = run(PH_FIND, maxSteps);
    if (s < 0) { printf("maze %d: search failed\n", m); fails++; continue; }
    searchCells += s;
    if (!goalKnown || goalX + offX() != trueGoalX || goalY != trueGoalY) {
      printf("maze %d: goal wrong (found %d, at %d,%d, true %d,%d)\n", m, goalKnown, goalX + offX(), goalY, trueGoalX, trueGoalY);
      fails++; continue;
    }
    if (startX != trueStartX) { printf("maze %d: start corner wrong\n", m); fails++; continue; }

    // Speed run: count cells to the goal only.
    posX = startX; posY = 0; facing = 0; phase = PH_FAST;
    int fastCells = 0;
    bool bad = false;
    for (int steps = 0; steps < 500; steps++) {
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
    if (run(PH_RETURN, maxSteps) < 0) { printf("maze %d: return failed\n", m); fails++; continue; }
    if (fastCells == shortest) proven++;
    else printf("maze %d: speed run %d cells, shortest %d\n", m, fastCells, shortest);
  }
  printf("%d mazes (18x18, goal found by the robot, both start corners): %d failures\n", N, fails);
  printf("speed run was the true shortest path in %d; average search %ld cells; most plan steps %d\n",
         proven, searchCells / N, maxSteps);
  return fails ? 1 : 0;
}
