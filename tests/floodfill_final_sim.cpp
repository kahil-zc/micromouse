// Runs the maze section of 14_floodfill_final.ino on random mazes.
//   cd tests && ./floodfill_final_sim.sh
// The script copies the lines between "// --- MAZE (no hardware calls) ---" and
// "// --- END MAZE ---" into maze_section.inc, so this tests the exact code the robot runs.
//
// For each maze: search to the goal and back home, reading only the walls a robot at the
// decision point would see, then do the fast run on known walls. Fails if the robot drives
// through a wall, gets stuck, or the fast run uses an unseen wall.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAZE_SIZE 16
#define GOAL_X 7
#define GOAL_Y 7
#define GOAL_SIZE 2
#define START_X 0
#define START_Y 0

#include "maze_section.inc"

static uint8_t truth[MAZE_SIZE][MAZE_SIZE];  // bits 0-3: real walls

static void trueWall(int x, int y, int d, bool on) {
  int nx = x + DX[d], ny = y + DY[d];
  uint8_t o = (d + 2) & 3;
  if (on) { truth[x][y] |= 1 << d; if (inMaze(nx, ny)) truth[nx][ny] |= 1 << o; }
  else    { truth[x][y] &= ~(1 << d); if (inMaze(nx, ny)) truth[nx][ny] &= ~(1 << o); }
}

// Random depth-first maze, then some walls knocked out for loops, competition-style centre
// with one entrance, start cell open only to the north.
static void makeMaze() {
  memset(truth, 0x0F, sizeof(truth));
  static bool seen[MAZE_SIZE][MAZE_SIZE];
  memset(seen, 0, sizeof(seen));
  int sx[MAZE_SIZE * MAZE_SIZE], sy[MAZE_SIZE * MAZE_SIZE], top = 0;
  for (int x = GOAL_X; x < GOAL_X + 2; x++)  // carve around the goal so the rest stays connected
    for (int y = GOAL_Y; y < GOAL_Y + 2; y++) seen[x][y] = true;
  sx[0] = 0; sy[0] = 0; seen[0][0] = true; top = 1;
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
  for (int i = 0; i < 25; i++) {  // loops
    int x = rand() % MAZE_SIZE, y = rand() % MAZE_SIZE, d = rand() % 4;
    if (inMaze(x + DX[d], y + DY[d])) trueWall(x, y, d, false);
  }
  // Goal: open inside, walled around except one entrance.
  for (int x = GOAL_X; x < GOAL_X + 2; x++)
    for (int y = GOAL_Y; y < GOAL_Y + 2; y++)
      for (int d = 0; d < 4; d++) {
        int nx = x + DX[d], ny = y + DY[d];
        trueWall(x, y, d, !isGoal(nx, ny));
      }
  static const int ex[8][3] = { {7,7,2},{8,7,2},{8,7,1},{8,8,1},{8,8,0},{7,8,0},{7,8,3},{7,7,3} };
  const int *e = ex[rand() % 8];
  trueWall(e[0], e[1], e[2], false);
  // Start cell: only north open.
  trueWall(0, 0, 1, true);
  trueWall(0, 0, 0, false);
}

static int trueShortest() {
  // BFS on the real maze from the start to any goal cell.
  static int d[MAZE_SIZE][MAZE_SIZE];
  for (int x = 0; x < MAZE_SIZE; x++) for (int y = 0; y < MAZE_SIZE; y++) d[x][y] = -1;
  int qx[256], qy[256], h = 0, t = 0;
  d[0][0] = 0; qx[t] = 0; qy[t++] = 0;
  while (h < t) {
    int x = qx[h], y = qy[h++];
    if (isGoal(x, y)) return d[x][y];
    for (int k = 0; k < 4; k++) {
      if (truth[x][y] & (1 << k)) continue;
      int nx = x + DX[k], ny = y + DY[k];
      if (d[nx][ny] >= 0) continue;
      d[nx][ny] = d[x][y] + 1; qx[t] = nx; qy[t++] = ny;
    }
  }
  return -1;
}

static void senseHere() {
  // Front, right, left of the current cell, as senseWallsHere() does.
  const uint8_t ds[3] = { facing, (uint8_t)((facing + 1) & 3), (uint8_t)((facing + 3) & 3) };
  for (int i = 0; i < 3; i++) setWall(posX, posY, ds[i], truth[posX][posY] & (1 << ds[i]));
}

// Same logic as searchStep(). Returns steps, or -1 on failure.
static int search(bool toGoal) {
  for (int steps = 0; steps < 2000; steps++) {
    senseHere();
    flood(toGoal, false);
    if (dist[posX][posY] == 255) return -1;
    if (dist[posX][posY] == 0) return steps;
    uint8_t d = bestDir(posX, posY, facing, false);
    if (truth[posX][posY] & (1 << d)) return -2;  // drove into a wall
    facing = d;
    posX += DX[d]; posY += DY[d];
  }
  return -3;
}

int main() {
  int fails = 0, proven = 0, unreachable = 0;
  const int N = 500;
  for (int m = 0; m < N; m++) {
    srand(m + 1);
    makeMaze();
    if (trueShortest() < 0) { unreachable++; continue; }  // generator cut the goal off
    initMaze();
    posX = START_X; posY = START_Y; facing = 0;
    setWall(START_X, START_Y, 2, true);

    int a = search(true);
    int b = a >= 0 ? search(false) : -9;
    if (a < 0 || b < 0) { printf("maze %d: search failed (%d, %d)\n", m, a, b); fails++; continue; }

    // Fast run: same as fastStep(), from the start facing north.
    posX = START_X; posY = START_Y; facing = 0;
    int cells = 0, moves = 0;
    bool bad = false;
    while (true) {
      flood(true, true);
      if (dist[posX][posY] == 255) { bad = true; printf("maze %d: no known path\n", m); break; }
      if (dist[posX][posY] == 0) break;
      uint8_t d = bestDir(posX, posY, facing, true);
      int n = straightRun(posX, posY, d, true);
      for (int i = 0; i < n; i++) {
        if (truth[posX][posY] & (1 << d)) { bad = true; printf("maze %d: fast run hits a wall\n", m); break; }
        posX += DX[d]; posY += DY[d];
      }
      if (bad) break;
      facing = d; cells += n; moves++;
      if (moves > 300) { bad = true; printf("maze %d: fast run loops\n", m); break; }
    }
    if (bad) { fails++; continue; }
    if (cells == trueShortest()) proven++;
  }
  printf("%d mazes (%d skipped: goal unreachable), %d failures, fast run was the true shortest path in %d\n",
         N, unreachable, fails, proven);
  return fails ? 1 : 0;
}
