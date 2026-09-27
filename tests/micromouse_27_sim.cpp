// Runs the explorer of 27_explore_all.ino on random 10 x 5 mazes (both ways round, with loops),
// with wrong wall readings, and checks it explores every cell, ends back at the start, and that
// the route it saves drives to the goal (and is the shortest one when the readings are right).
//   tests/micromouse_27_sim.sh
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "maze_27_config.inc"
#include "maze_27_section.inc"

static uint8_t truth[16][16];
static int W, H;
static bool inReal(int x, int y) { return x >= 0 && y >= 0 && x < W && y < H; }
static bool T(int x, int y, int d) { int nx = x + DX[d], ny = y + DY[d]; return !inReal(nx, ny) || (truth[x][y] & (1 << d)); }

static void makeMaze(int w, int h, int loops) {
  W = w; H = h;
  for (int x = 0; x < 16; x++) for (int y = 0; y < 16; y++) truth[x][y] = 15;
  static bool seen[16][16]; memset(seen, 0, sizeof(seen));
  int sx[256], sy[256], n = 0; sx[n] = 0; sy[n++] = 0; seen[0][0] = true;
  while (n) {
    int x = sx[n - 1], y = sy[n - 1], opts[4], k = 0;
    for (int d = 0; d < 4; d++) { int nx = x + DX[d], ny = y + DY[d]; if (inReal(nx, ny) && !seen[nx][ny]) opts[k++] = d; }
    if (!k) { n--; continue; }
    int d = opts[rand() % k], nx = x + DX[d], ny = y + DY[d];
    truth[x][y] &= ~(1 << d); truth[nx][ny] &= ~(1 << ((d + 2) & 3));
    seen[nx][ny] = true; sx[n] = nx; sy[n++] = ny;
  }
  for (int i = 0; i < loops; i++) {
    int x = rand() % w, y = rand() % h, d = rand() % 4, nx = x + DX[d], ny = y + DY[d];
    if (!inReal(nx, ny)) continue;
    truth[x][y] &= ~(1 << d); truth[nx][ny] &= ~(1 << ((d + 2) & 3));
  }
}
static bool realGoal(int x, int y) { return inRoom(x, y, W, H); }
static int shortest() {
  static int d[16][16]; for (int x = 0; x < 16; x++) for (int y = 0; y < 16; y++) d[x][y] = -1;
  int qx[256], qy[256], h = 0, t = 0; d[0][0] = 0; qx[t] = 0; qy[t++] = 0;
  while (h < t) { int x = qx[h], y = qy[h++]; if (realGoal(x, y)) return d[x][y];
    for (int k = 0; k < 4; k++) { if (T(x, y, k)) continue; int nx = x + DX[k], ny = y + DY[k]; if (d[nx][ny] >= 0) continue; d[nx][ny] = d[x][y] + 1; qx[t] = nx; qy[t++] = ny; } }
  return -1;
}
static double pWrong;
static bool rnd(double p) { return rand() < p * RAND_MAX; }

static uint8_t readOnce() {
  uint8_t w = 0;
  for (int i = 0; i < 3; i++) { bool r = T(posX, posY, (facing + 3 + i) & 3); if (rnd(pWrong)) r = !r; if (r) w |= 1 << i; }
  return w;
}

// Like searchStep(): returns cells driven, -1 if it never finished.
static int explore() {
  resetSearch();
  int cells = 0;
  for (int step = 0; step < 3000; step++) {
    uint8_t w = readOnce() & readOnce();  // like readWalls(): a wall only if both readings see it
    recordWalls(w);
    while (true) {
      uint8_t d = exploreDir();
      if (d == 255) return (posX == 0 && posY == 0) ? cells : -1;
      if (T(posX, posY, d)) { setWall(posX, posY, d, true); continue; }  // ToFs: a wall after all
      stepTo(d); cells++;
      break;
    }
  }
  return -1;
}
static int unexplored() {
  int n = 0;
  for (int x = 0; x < W; x++) for (int y = 0; y < H; y++) if (!(cellMap[x][y] & VISITED)) n++;
  return n;
}
// Like fastRun(): drives the shortest recorded route; a wall the record didn't have is recorded
// and the route worked out again. Returns cells driven, -1 if it can't get there.
static int speedRun() {
  posX = 0; posY = 0; facing = 0; int c = 0;
  for (int s = 0; s < 200; s++) {
    if (realGoal(posX, posY)) return c;
    if (!findRoute(posX, posY, true)) return -1;
    uint8_t d = pathDir[0]; int n = runFrom(0);
    if (T(posX, posY, d)) { setWall(posX, posY, d, true); continue; }
    facing = d;
    for (int i = 0; i < n; i++) { if (T(posX, posY, d)) { setWall(posX, posY, d, true); break; } posX += DX[d]; posY += DY[d]; c++; }
  }
  return -1;
}

static void trial(const char *name, int loops, double wrong) {
  pWrong = wrong;
  const int N = 2000; int fail = 0, missed = 0, noRoute = 0, bad = 0, optimal = 0; (void)bad; long cells = 0, extra = 0;
  for (int m = 0; m < N; m++) {
    srand(m + 1);
    bool turned = m & 1;
    makeMaze(turned ? MAZE_AHEAD : MAZE_ACROSS, turned ? MAZE_ACROSS : MAZE_AHEAD, loops);
    int c = explore();
    if (c < 0) { fail++; continue; }
    cells += c;
    missed += unexplored();
    int r = speedRun();
    if (r < 0) { noRoute++; continue; }
    int sh = shortest(); extra += r - sh; if (r == sh) optimal++;
  }
  int ok = N - fail - noRoute;
  printf("%-24s didn't finish %d/%d, cells missed %d | search %.1f cells | speed run: didn't get there %d, shortest in %d (%.2f cells longer on average)\n",
         name, fail, N, missed, (double)cells / (N - fail), noRoute, optimal, ok ? (double)extra / ok : 0);
}

int main() {
  trial("no loops, perfect", 0, 0);
  trial("some loops, perfect", 6, 0);
  trial("many loops, perfect", 20, 0);
  trial("some loops, 2% wrong", 6, 0.02);
  trial("some loops, 5% wrong", 6, 0.05);
  return 0;
}
