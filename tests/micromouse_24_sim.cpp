// Runs the maze section of 24_micromouse_final.ino on random 10 x 5 mazes (both ways round,
// inside the 10 x 10 map), with wrong wall readings, and checks the robot reaches the goal,
// gets home, and that the speed run then reaches the goal on seen walls.
//   tests/micromouse_24_sim.sh
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "maze_24_config.inc"
#include "maze_24_section.inc"

static uint8_t truth[MAP_W][MAP_H];  // real walls, bits 0-3
static int realW, realH;

static bool inReal(int x, int y) { return x >= 0 && y >= 0 && x < realW && y < realH; }
static bool T(int x, int y, int d) { return truth[x][y] & (1 << d); }

// Random maze: depth-first carving plus a few extra openings (loops), inside realW x realH.
static void makeMaze(int w, int h) {
  realW = w; realH = h;
  for (int x = 0; x < MAP_W; x++) for (int y = 0; y < MAP_H; y++) truth[x][y] = 15;
  static bool seen[MAP_W][MAP_H]; memset(seen, 0, sizeof(seen));
  int sx[256], sy[256], n = 0;
  sx[n] = 0; sy[n++] = 0; seen[0][0] = true;
  while (n) {
    int x = sx[n - 1], y = sy[n - 1], opts[4], k = 0;
    for (int d = 0; d < 4; d++) { int nx = x + DX[d], ny = y + DY[d]; if (inReal(nx, ny) && !seen[nx][ny]) opts[k++] = d; }
    if (!k) { n--; continue; }
    int d = opts[rand() % k], nx = x + DX[d], ny = y + DY[d];
    truth[x][y] &= ~(1 << d); truth[nx][ny] &= ~(1 << ((d + 2) & 3));
    seen[nx][ny] = true; sx[n] = nx; sy[n++] = ny;
  }
  for (int i = 0; i < w * h / 8; i++) {  // loops
    int x = rand() % w, y = rand() % h, d = rand() % 4, nx = x + DX[d], ny = y + DY[d];
    if (!inReal(nx, ny)) continue;
    truth[x][y] &= ~(1 << d); truth[nx][ny] &= ~(1 << ((d + 2) & 3));
  }
}

static bool realGoal(int x, int y) { return inRoom(x, y, realW, realH); }
static int shortest() {
  static int d[MAP_W][MAP_H]; for (int x = 0; x < MAP_W; x++) for (int y = 0; y < MAP_H; y++) d[x][y] = -1;
  int qx[256], qy[256], h = 0, t = 0; d[0][0] = 0; qx[t] = 0; qy[t++] = 0;
  while (h < t) { int x = qx[h], y = qy[h++]; if (realGoal(x, y)) return d[x][y];
    for (int k = 0; k < 4; k++) { if (T(x, y, k)) continue; int nx = x + DX[k], ny = y + DY[k]; if (d[nx][ny] >= 0) continue; d[nx][ny] = d[x][y] + 1; qx[t] = nx; qy[t++] = ny; } }
  return -1;
}

static double pWrong, pShort;  // per reading: a wrong wall; per stop: stopped short (open sides read as walls)
static bool rnd(double p) { return rand() < p * RAND_MAX; }

static void senseHere() {
  bool shortStop = rnd(pShort);
  uint8_t w = 0;
  for (int i = 0; i < 3; i++) {
    int d = (facing + 3 + i) & 3;
    bool r = T(posX, posY, d);
    if (i != 1 && shortStop && !r) r = true;   // side ToF still sees the post
    else if (rnd(pWrong)) r = !r;
    if (r) w |= 1 << i;
  }
  recordWalls(w);
  if (!(w & 2) && !T(posX, posY, facing)) {
    int nx = posX + DX[facing], ny = posY + DY[facing];
    uint8_t a = T(nx, ny, facing) ? 1 : 2;
    if (rnd(pWrong)) a = 3 - a;
    recordAhead(a);
  }
}

static int cellsDriven;
// Search to the goal (toGoal) or home. Returns false if it gave up.
static bool search(bool toGoal) {
  for (int step = 0; step < 3000; step++) {
    senseHere();
    uint8_t d = nextDir(toGoal);
    if (d == 255) continue;
    if (d == 254) return true;
    if (T(posX, posY, d)) {  // the map was wrong: it can't go that way
      setWall(posX, posY, d, true);
      continue;
    }
    facing = d;
    advance(d, 1);
    cellsDriven++;
  }
  return false;
}

static long surprises;
static int speedRun() {
  posX = 0; posY = 0; facing = 0; int c = 0;
  for (int s = 0; s < 200; s++) {
    flood(true, true);
    if (dist[posX][posY] == 0) return realGoal(posX, posY) ? c : -1;
    if (dist[posX][posY] == 255) return -1;
    uint8_t d = bestDir(posX, posY, facing, true);
    int n = straightRun(posX, posY, d, true);
    // A wall the map had open: the front ToF (straight on) or the side ToF check before the
    // arc (a turn) sees it; the robot marks it and plans again from where it stands.
    if (T(posX, posY, d)) { setWall(posX, posY, d, true); surprises++; continue; }
    facing = d;
    for (int i = 0; i < n; i++) {
      if (T(posX, posY, d)) { setWall(posX, posY, d, true); surprises++; break; }
      posX += DX[d]; posY += DY[d]; c++;
    }
  }
  return -1;
}

static void trial(const char *name, double wrong, double shortP) {
  pWrong = wrong; pShort = shortP;
  const int N = 2000;
  int fail = 0, srOk = 0, optimal = 0; long cells = 0, extra = 0; surprises = 0;
  for (int m = 0; m < N; m++) {
    srand(m + 1);
    bool turned = m & 1;
    makeMaze(turned ? MAZE_AHEAD : MAZE_ACROSS, turned ? MAZE_ACROSS : MAZE_AHEAD);
    if (!EITHER_WAY && turned) continue;
    initMaze(); posX = posY = 0; facing = 0; setWall(0, 0, 2, true); cellsDriven = 0;
    bool ok = search(true) && realGoal(posX, posY) && search(false) && posX == 0 && posY == 0;
    if (!ok) { fail++; continue; }
    cells += cellsDriven;
    memset(driven, 0, sizeof(driven));
    int s = speedRun(), sh = shortest();
    if (s >= 0) { srOk++; extra += s - sh; if (s == sh) optimal++; }
  }
  printf("%-34s failed %d/%d, search to goal and home %.1f cells, speed run reached the goal %d (shortest %d, %.2f cells longer on average, %ld walls found on the way)\n",
         name, fail, N, (double)cells / (N - fail), srOk, optimal, srOk ? (double)extra / srOk : 0.0, surprises);
}

int main() {
  trial("perfect readings", 0, 0);
  trial("1% readings wrong", 0.01, 0);
  trial("3% readings wrong", 0.03, 0);
  trial("10% stops short (false side walls)", 0, 0.10);
  trial("10% short + 3% wrong", 0.03, 0.10);
  return 0;
}
