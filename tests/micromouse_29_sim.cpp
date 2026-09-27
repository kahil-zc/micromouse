// PC test of 29_find_goal's maze code (tests/micromouse_29_sim.sh extracts it from the sketch).
// Random 10x5 mazes placed so the start is one of their corners, whichever way round and
// whichever way the robot faces (back to a wall). The goal room is a 2x2 room at a random
// place. The robot reads its 3 walls (some wrong with probability ERR), drives one cell to the
// nearest unvisited cell (a wall it didn't know stops it, like the ToFs do), finds the room,
// goes home; then the fast run drives the known route.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <algorithm>
using std::min; using std::max;
#define MAP_SIZE 19
#define START_X 9
#define START_Y 9
#include "maze29.inc"

int W, H; uint8_t R[10][10];            // real maze, bits N E S W, in maze coordinates
int ox, oy, rot;                         // maze -> map placement
bool rin(int x, int y) { return x >= 0 && y >= 0 && x < W && y < H; }
// map cell -> maze cell
void toMaze(int mx, int my, int &x, int &y) {
  int dx = mx - START_X, dy = my - START_Y;
  for (int r = 0; r < rot; r++) { int t = dx; dx = dy; dy = -t; }  // rotate back
  x = ox + dx; y = oy + dy;
}
bool realWall(int mx, int my, int d) {
  int x, y; toMaze(mx, my, x, y);
  if (!rin(x, y)) return true;
  int md = (d + rot) & 3;  // map direction -> maze direction
  return R[x][y] & (1 << md);
}
bool vis[10][10];
void open(int x, int y, int d) { R[x][y] &= ~(1 << d); int nx = x + DX[d], ny = y + DY[d]; if (rin(nx, ny)) R[nx][ny] &= ~(1 << ((d + 2) & 3)); }
void carve(int x, int y) {
  vis[x][y] = 1; int o[4] = {0, 1, 2, 3};
  for (int i = 3; i > 0; i--) { int j = rand() % (i + 1); std::swap(o[i], o[j]); }
  for (int i = 0; i < 4; i++) { int d = o[i], nx = x + DX[d], ny = y + DY[d]; if (rin(nx, ny) && !vis[nx][ny]) { open(x, y, d); carve(nx, ny); } }
}
bool room2x2(int x, int y) {  // real open 2x2 with lower-left x,y
  if (!rin(x, y) || !rin(x + 1, y + 1)) return false;
  return !(R[x][y] & 1) && !(R[x][y] & 2) && !(R[x + 1][y] & 1) && !(R[x][y + 1] & 2);
}
int countRooms() { int n = 0; for (int x = 0; x < W; x++) for (int y = 0; y < H; y++) n += room2x2(x, y); return n; }
int trueDist(int sx, int sy, int gx, int gy) {  // shortest start -> goal room in the real maze
  int D[10][10]; memset(D, -1, sizeof D); int q[100], h = 0, t = 0; D[sx][sy] = 0; q[t++] = sx * 10 + sy;
  while (h < t) { int c = q[h++], x = c / 10, y = c % 10;
    if (x >= gx && x < gx + 2 && y >= gy && y < gy + 2) return D[x][y];
    for (int d = 0; d < 4; d++) { int nx = x + DX[d], ny = y + DY[d]; if (!(R[x][y] & (1 << d)) && rin(nx, ny) && D[nx][ny] < 0) { D[nx][ny] = D[x][y] + 1; q[t++] = nx * 10 + ny; } } }
  return -1;
}
int maxQueue = 0;
void measureQueue(uint8_t to, bool known) {  // same search as findRoute, counting the queue
  static uint8_t f[MAP_SIZE][MAP_SIZE]; memset(f, 255, sizeof f); f[posX][posY] = 4;
  int q[400], h = 0, t = 0; q[t++] = posX * MAP_SIZE + posY;
  while (h < t) { int c = q[h++], x = c / MAP_SIZE, y = c % MAP_SIZE;
    if (isTarget(x, y, to) && f[x][y] != 4) break;
    for (int d = 0; d < 4; d++) { if (wallAt(x, y, d) || (known && !seenAt(x, y, d))) continue;
      int nx = x + DX[d], ny = y + DY[d]; if (f[nx][ny] != 255) continue; f[nx][ny] = d; q[t++] = nx * MAP_SIZE + ny; maxQueue = max(maxQueue, t - h); } }
}

int main(int argc, char **argv) {
  double ERR = argc > 1 ? atof(argv[1]) : 0;
  int N = 3000, noGoal = 0, wrongGoal = 0, stuck = 0, fastFail = 0, fastLonger = 0; long cells = 0;
  for (int t = 0; t < N; t++) {
    srand(t);
    bool tall = t & 1; W = tall ? 5 : 10; H = tall ? 10 : 5;
    memset(R, 15, sizeof R); memset(vis, 0, sizeof vis);
    int gx, gy;
    do {
      memset(R, 15, sizeof R); memset(vis, 0, sizeof vis);
      carve(0, 0);
      gx = rand() % (W - 1); gy = rand() % (H - 1);
      if (gx == 0 && gy == 0) continue;
      open(gx, gy, 0); open(gx, gy, 1); open(gx + 1, gy, 0); open(gx, gy + 1, 1);
      for (int k = 0; k < W * H / 10; k++) { int x = rand() % W, y = rand() % H, d = rand() % 4; if (!rin(x + DX[d], y + DY[d])) continue;
        uint8_t save[10][10]; memcpy(save, R, sizeof R); open(x, y, d); if (countRooms() > 1) memcpy(R, save, sizeof R); }
    } while (countRooms() != 1 || (gx == 0 && gy == 0));
    // start: a corner of the maze, back to an outer wall
    int corner = rand() % 4;
    ox = (corner & 1) ? W - 1 : 0; oy = (corner & 2) ? H - 1 : 0;
    // robot's facing in maze directions: away from one of the two outer walls
    int faceChoices[2] = { (corner & 2) ? 2 : 0, (corner & 1) ? 3 : 1 };
    int face = faceChoices[rand() % 2];
    rot = face;  // map north = maze direction `face`
    initMaze(); posX = START_X; posY = START_Y; facing = 0; setWall(START_X, START_Y, 2, true);
    if (!realWall(START_X, START_Y, 2)) { t--; continue; }  // needs a wall behind (always true here)
    int state = 1, n = 0, pass = 0; bool ok = false;
    while (n < 900) {
      // sense
      uint8_t w = 0;
      for (int i = 0; i < 3; i++) { bool rw = realWall(posX, posY, (facing + 3 + i) & 3); if (rand() < ERR * RAND_MAX) rw = !rw; w |= rw << i; }
      recordWalls(w);
      if (checkGoal()) state = 2;
      bool done = false;
      while (true) {
        if (state == 2 && posX == START_X && posY == START_Y) { done = true; break; }
        uint8_t to = state == 1 ? TO_UNVISITED : TO_START;
        measureQueue(to, false);
        if (!findRoute(to, false)) {
          if (to == TO_UNVISITED && candX >= 0) { candX = candY = -1; continue; }
          if (to == TO_UNVISITED && ++pass < 3) { initMaze(); setWall(START_X, START_Y, 2, true); break; }
          if (to == TO_UNVISITED) { state = 2; continue; }
          memset(wallMap, 0, sizeof wallMap); continue; }
        uint8_t d = routeDir; facing = d;
        if (realWall(posX, posY, d)) { setWall(posX, posY, d, true); continue; }  // ToFs find it
        moved(d, 1); n++; break;
      }
      if (done) { ok = true; break; }
    }
    cells += n;
    if (!ok) { stuck++; continue; }
    if (goalX < 0) { noGoal++; continue; }
    int mx, my; toMaze(goalX, goalY, mx, my); int mx2, my2; toMaze(goalX + 1, goalY + 1, mx2, my2);
    if (min(mx, mx2) != gx || min(my, my2) != gy) { wrongGoal++; continue; }
    // fast run on known walls
    posX = START_X; posY = START_Y; facing = 0; int steps = 0, len = 0;
    while (!isGoal(posX, posY) && steps < 300) {
      measureQueue(TO_GOAL, true);
      bool known = findRoute(TO_GOAL, true);
      if (!known) {
        uint8_t w = 0;
        for (int i = 0; i < 3; i++) { bool rw = realWall(posX, posY, (facing + 3 + i) & 3); if (rand() < ERR * RAND_MAX) rw = !rw; w |= rw << i; }
        recordWalls(w);
        if (!findRoute(TO_GOAL, false)) break;
        routeRun = 1;
      }
      uint8_t d = routeDir; int k = 0; facing = d;
      for (; k < routeRun; k++) { if (realWall(posX, posY, d)) break; moved(d, 1); len++; }
      if (k < routeRun) setWall(posX, posY, d, true);
      steps++;
    }
    if (!isGoal(posX, posY)) { fastFail++; continue; }
    if (len > trueDist(ox, oy, gx, gy)) fastLonger++;
  }
  printf("wrong readings %.0f%%: %d mazes, avg %.0f cells driven (search + home); goal not found %d, wrong goal %d, stuck %d, fast run failed %d, fast run not shortest %d; longest search queue %d (fits 63)\n",
         ERR * 100, N, (double)cells / N, noGoal, wrongGoal, stuck, fastFail, fastLonger, maxQueue);
}
