// Runs the wall follower of 25_wall_follower.ino on random 10 x 5 mazes (both ways round, with
// loops), with wrong wall readings, and checks it reaches the goal and that the saved route
// drives to the goal and back home on the real walls.
//   tests/micromouse_25_sim.sh
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "maze_25_config.inc"
#include "maze_25_section.inc"

static uint8_t truth[16][16];
static int W, H;
static bool inReal(int x, int y) { return x >= 0 && y >= 0 && x < W && y < H; }
static bool T(int x, int y, int d) { return truth[x][y] & (1 << d); }

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

// Search like searchStep(): returns cells driven, -1 if it never got there.
static int search() {
  resetSearch();
  int cells = 0;
  for (int step = 0; step < 2000; step++) {
    if (isGoal(posX, posY)) return realGoal(posX, posY) ? cells : -1;
    uint8_t w = 0;
    for (int i = 0; i < 3; i++) { bool r = T(posX, posY, (facing + 3 + i) & 3); if (rnd(pWrong)) r = !r; if (r) w |= 1 << i; }
    for (int tries = 0; tries < 4; tries++) {
      uint8_t d = chooseDir(w);
      if (T(posX, posY, d)) {  // a wall after all: side ToF check before a turn / front ToF stop
        if (d == facing) { w |= 2; }
        else if (d == ((facing + 3) & 3)) w |= 1;
        else if (d == ((facing + 1) & 3)) w |= 4;
        else break;  // (a U-turn is always possible: never a wall behind the way it came)
        beenHere[posX][posY] &= ~(1 << facing);
        continue;
      }
      stepTo(d); cells++;
      break;
    }
  }
  return -1;
}
// Drives the saved route on the real walls: forward from the start must end in the goal,
// backward from there must end at the start.
static bool routeWorks() {
  int x = 0, y = 0;
  for (int i = 0; i < pathLen; i++) { if (T(x, y, pathDir[i])) return false; x += DX[pathDir[i]]; y += DY[pathDir[i]]; }
  if (!realGoal(x, y)) return false;
  for (int i = pathLen - 1; i >= 0; i--) { int d = (pathDir[i] + 2) & 3; if (T(x, y, d)) return false; x += DX[d]; y += DY[d]; }
  return x == 0 && y == 0;
}

static void trial(const char *name, int loops, double wrong) {
  pWrong = wrong;
  const int N = 2000; int fail = 0, bad = 0, optimal = 0; long cells = 0, route = 0, extra = 0;
  for (int m = 0; m < N; m++) {
    srand(m + 1);
    bool turned = m & 1;
    makeMaze(turned ? MAZE_AHEAD : MAZE_ACROSS, turned ? MAZE_ACROSS : MAZE_AHEAD, loops);
    int c = search();
    if (c < 0) { fail++; continue; }
    cells += c;
    if (!routeWorks()) { bad++; continue; }
    route += pathLen; int sh = shortest(); extra += pathLen - sh; if (pathLen == sh) optimal++;
  }
  int ok = N - fail - bad;
  printf("%-26s never reached the goal %d/%d, bad route %d | search %.1f cells, route %.1f cells (shortest in %d, %.2f longer on average)\n",
         name, fail, N, bad, (double)cells / (N - fail), ok ? (double)route / ok : 0, optimal, ok ? (double)extra / ok : 0);
}

int main() {
  trial("no loops, perfect", 0, 0);
  trial("some loops, perfect", 6, 0);
  trial("many loops, perfect", 20, 0);
  trial("some loops, 2% wrong", 6, 0.02);
  trial("some loops, 5% wrong", 6, 0.05);
  return 0;
}
