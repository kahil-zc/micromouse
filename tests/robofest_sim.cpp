// Host-side test for 32_robofest/32_robofest.ino. robofest_sim.sh cuts the "MAZE CONFIG" and
// "MAZE" sections out of the sketch, so this runs the robot's own map, flood fill, corner /
// mirror logic and wall recording. searchStep, fastStep and driveTo are copied here with the
// hardware calls replaced by a perfect robot in a true maze.
//
// Mazes: tests/mazes/*.maz (ROBOFEST2026 from the preliminary-round PDF, and classic contest
// mazes), each started from every corner that is a proper start cell (one opening), plus random
// mazes. For each start it checks that:
//   - the robot's map position always matches where it really is (left/right mirroring right)
//   - it never turns or drives into a wall, reaches the centre and gets home
//   - a second search + fast run reach the goal; the fast run only uses walls it has seen
//   - "proven shortest" is really the shortest
//   - a saved map from another corner is noticed and replaced (then the search still works)

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <dirent.h>

#include "maze_config.h"
#include "maze_section.h"

static const int N = 16;
static const int WX[4] = { 0, 1, 0, -1 }, WY[4] = { 1, 0, -1, 0 };

struct World {
  std::string name;
  uint8_t c[N][N];  // bits N=1 E=2 S=4 W=8
  bool wall(int x, int y, int d) const { return c[x][y] & (1 << d); }
};

static bool loadMaz(const std::string &path, World &w) {
  FILE *f = fopen(path.c_str(), "r");
  if (!f) return false;
  int n = 0;
  char tok[16];
  while (n < 256 && fscanf(f, "%15s", tok) == 1) {
    unsigned v;
    if (sscanf(tok, "%4x", &v) != 1) continue;
    w.c[n / 16][n % 16] = v >> 8; n++;
    w.c[n / 16][n % 16] = v & 0xFF; n++;
  }
  fclose(f);
  return n == 256;
}

// Random 16 x 16 maze: centre 2x2 goal with one or two entrances, the four corners proper start
// cells (one opening), some loops.
static void randomMaze(World &w, int seed, double loops) {
  srand(seed);
  bool wl[N][N][4];
  for (int x = 0; x < N; x++) for (int y = 0; y < N; y++) for (int d = 0; d < 4; d++) wl[x][y][d] = true;
  auto set = [&](int x, int y, int d, bool v) {
    wl[x][y][d] = v;
    int nx = x + WX[d], ny = y + WY[d];
    if (nx >= 0 && ny >= 0 && nx < N && ny < N) wl[nx][ny][(d + 2) & 3] = v;
  };
  auto inGoal = [](int x, int y) { return x >= 7 && x <= 8 && y >= 7 && y <= 8; };
  // depth-first maze, the goal room is carved open and entered once
  set(7, 7, 0, false); set(7, 7, 1, false); set(8, 7, 0, false); set(7, 8, 1, false);
  std::vector<int> st; bool seen[N][N] = {};
  for (int x = 7; x <= 8; x++) for (int y = 7; y <= 8; y++) seen[x][y] = true;
  st.push_back(0);
  seen[0][0] = true;
  while (!st.empty()) {
    int x = st.back() / N, y = st.back() % N;
    int ds[4] = { 0, 1, 2, 3 };
    for (int i = 3; i > 0; i--) std::swap(ds[i], ds[rand() % (i + 1)]);
    bool went = false;
    for (int i = 0; i < 4 && !went; i++) {
      int d = ds[i], nx = x + WX[d], ny = y + WY[d];
      if (nx < 0 || ny < 0 || nx >= N || ny >= N || seen[nx][ny]) continue;
      seen[nx][ny] = true; set(x, y, d, false); st.push_back(nx * N + ny); went = true;
    }
    if (!went) st.pop_back();
  }
  // one entrance into the goal room
  int e = rand() % 8;
  static const int ex[8] = { 7, 8, 9, 9, 8, 7, 6, 6 }, ey[8] = { 6, 6, 7, 8, 9, 9, 8, 7 };
  static const int ed[8] = { 0, 0, 3, 3, 2, 2, 1, 1 };
  set(ex[e], ey[e], ed[e], false);
  // loops, no open 2x2 outside the goal
  for (int x = 0; x < N; x++)
    for (int y = 0; y < N; y++)
      for (int d = 0; d < 2; d++) {
        int nx = x + WX[d], ny = y + WY[d];
        if (nx >= N || ny >= N || !wl[x][y][d] || inGoal(x, y) != inGoal(nx, ny)) continue;
        if (rand() / (double)RAND_MAX >= loops) continue;
        set(x, y, d, false);
        bool bad = false;
        for (int bx = x - 1; bx <= x; bx++)
          for (int by = y - 1; by <= y; by++) {
            if (bx < 0 || by < 0 || bx + 1 >= N || by + 1 >= N || (bx == 7 && by == 7)) continue;
            if (!wl[bx][by][0] && !wl[bx][by][1] && !wl[bx + 1][by][0] && !wl[bx][by + 1][1]) bad = true;
          }
        if (bad) set(x, y, d, true);
      }
  // corners: exactly one opening each
  int cx[4] = { 0, N - 1, 0, N - 1 }, cy[4] = { 0, 0, N - 1, N - 1 };
  for (int i = 0; i < 4; i++) {
    int open = 0;
    for (int d = 0; d < 4; d++) if (!wl[cx[i]][cy[i]][d]) open++;
    if (open > 1) {  // keep the first opening
      bool kept = false;
      for (int d = 0; d < 4; d++) if (!wl[cx[i]][cy[i]][d]) { if (kept) set(cx[i], cy[i], d, true); kept = true; }
    }
  }
  char nm[32]; snprintf(nm, sizeof nm, "random%d", seed);
  w.name = nm;
  for (int x = 0; x < N; x++) for (int y = 0; y < N; y++) {
    uint8_t v = 0;
    for (int d = 0; d < 4; d++) if (wl[x][y][d]) v |= 1 << d;
    w.c[x][y] = v;
  }
}

// ---------------- The robot in the world ----------------
struct Robot {
  const World *w;
  int sx, sy, sh;   // start cell and heading (world)
  int x, y, h;      // where it really is
  long moves, cells, arcs, uturns, surprises, stops;
};
static Robot R;
static const char *why = 0;

// physical readings where the robot is
static void readWalls(bool &l, bool &f, bool &r) {
  l = R.w->wall(R.x, R.y, (R.h + 3) & 3);
  f = R.w->wall(R.x, R.y, R.h);
  r = R.w->wall(R.x, R.y, (R.h + 1) & 3);
}

// The map position the robot should have: its real position in the start's frame (x = to the
// right of the start direction, y = ahead), mirrored if the maze is on the left.
static bool mapMatches() {
  int rx = (R.x - R.sx) * WX[(R.sh + 1) & 3] + (R.y - R.sy) * WY[(R.sh + 1) & 3];
  int ry = (R.x - R.sx) * WX[R.sh] + (R.y - R.sy) * WY[R.sh];
  int rh = (R.h - R.sh) & 3;
  if (mirrored) { rx = -rx; rh = (4 - rh) & 3; }
  return rx == posX && ry == posY && rh == facing;
}

static void senseHere() {
  bool l, f, r;
  readWalls(l, f, r);
  recordReading(l, f, r);
  R.stops++;
}

// driveTo() of the sketch: physical turn (with the side-ToF check before an arc), then up to n
// cells; the front ToF stops it at a wall.
static bool wallStopSim = false;
static int failSim = 0;
static bool driveToSim(uint8_t d, int n) {
  wallStopSim = false;
  char pm = physMove(relMove(facing, d));
  if (pm == 'L' || pm == 'R') {
    int side = pm == 'L' ? (R.h + 3) & 3 : (R.h + 1) & 3;
    if (R.w->wall(R.x, R.y, side)) {  // TURN_BLOCKED
      setWall(posX, posY, d, true);
      wallStopSim = true;
      failSim++;
      R.surprises++;
      return false;
    }
    R.arcs++;
    R.h = side;
  } else if (pm == 'U') {
    R.uturns++;
    R.h = (R.h + 2) & 3;
  }
  facing = d;
  int k = 0;
  while (k < n && !R.w->wall(R.x, R.y, R.h)) { R.x += WX[R.h]; R.y += WY[R.h]; k++; }
  R.moves++; R.cells += k;
  if (k < n) { wallStopSim = true; R.surprises++; }
  moved(d, k);
  if (wallStopSim) setWall(posX, posY, d, true);
  failSim = k > 0 ? 0 : failSim + 1;
  return k > 0;
}

static void beginRunSim(int state) {
  R.x = R.sx; R.y = R.sy; R.h = R.sh;
  posX = START_X; posY = START_Y; facing = 0;
  mirrored = (mapSide == 2);
  sideChecked = (state == ST_FAST);
  mismatches = 0;
  runStops = (state == ST_FAST) ? CHECK_STOPS : 0;
  failSim = 0;
}

// searchStep() of the sketch, run to the goal and home. False = something went wrong (why).
static bool search() {
  beginRunSim(ST_SEARCH_GOAL);
  bool toGoal = true, reached = false;
  for (int guard = 0; guard < 3000; guard++) {
    senseHere();
    if (sideChecked && !mapMatches()) { why = "map position differs from the real one"; return false; }
    flood(toGoal, false);
    if (dist[posX][posY] == 255) { initMaze(); continue; }
    if (dist[posX][posY] == 0) {
      if (toGoal) {
        if (!(R.x >= 7 && R.x <= 8 && R.y >= 7 && R.y <= 8)) { why = "thinks it is at the goal but isn't"; return false; }
        toGoal = false; reached = true; continue;
      }
      if (R.x != R.sx || R.y != R.sy) { why = "thinks it is home but isn't"; return false; }
      return reached;
    }
    uint8_t d = bestDir(posX, posY, facing, false);
    driveToSim(d, 1);
    if (failSim >= MAX_FAILS) { why = "3 failed moves in a row"; return false; }
  }
  why = "search never finished";
  return false;
}

// fastStep() of the sketch. cells = cells driven to the goal.
static bool fastRun(int &cells) {
  beginRunSim(ST_FAST);
  long c0 = R.cells;
  for (int guard = 0; guard < 500; guard++) {
    if (!mapMatches()) { why = "fast run: map position differs"; return false; }
    if (isGoal(posX, posY)) { cells = R.cells - c0; return R.x >= 7 && R.x <= 8 && R.y >= 7 && R.y <= 8; }
    flood(true, true);
    if (dist[posX][posY] == 255) { why = "fast run: no known way"; return false; }
    uint8_t d = bestDir(posX, posY, facing, true);
    int n = straightRun(posX, posY, d, true);
    long s = R.surprises;
    driveToSim(d, n);
    if (R.surprises != s) { why = "fast run hit a wall it didn't know"; return false; }
  }
  why = "fast run never finished";
  return false;
}

// True shortest start -> centre in the world.
static int shortestWorld(const World &w, int sx, int sy) {
  int d[N][N]; memset(d, -1, sizeof d);
  std::vector<int> q(1, sx * N + sy); d[sx][sy] = 0;
  for (size_t i = 0; i < q.size(); i++) {
    int x = q[i] / N, y = q[i] % N;
    if (x >= 7 && x <= 8 && y >= 7 && y <= 8) return d[x][y];
    for (int k = 0; k < 4; k++) {
      if (w.wall(x, y, k)) continue;
      int nx = x + WX[k], ny = y + WY[k];
      if (d[nx][ny] >= 0) continue;
      d[nx][ny] = d[x][y] + 1; q.push_back(nx * N + ny);
    }
  }
  return -1;
}

// The proper start cells of a maze: corners with exactly one opening (facing it) that lead to the
// goal (a random maze can cut a corner off).
static std::vector<std::vector<int> > starts(const World &w) {
  std::vector<std::vector<int> > s;
  int cx[4] = { 0, N - 1, 0, N - 1 }, cy[4] = { 0, 0, N - 1, N - 1 };
  for (int i = 0; i < 4; i++) {
    int open = 0, od = 0;
    for (int d = 0; d < 4; d++) if (!w.wall(cx[i], cy[i], d)) { open++; od = d; }
    if (open == 1 && shortestWorld(w, cx[i], cy[i]) >= 0) s.push_back(std::vector<int>{ cx[i], cy[i], od });
  }
  return s;
}

static const char *cornerName(int x, int y) {
  // PDF naming for the Robofest maze: columns A..P left to right, rows 1..16 top to bottom
  static char b[16];
  snprintf(b, sizeof b, "%c%d", 'A' + x, 16 - y);
  return b;
}

// reportPath() of the sketch: shortest start -> goal on the map (knownOnly: seen walls only).
static uint8_t shortestPath(bool knownOnly) {
  flood(true, knownOnly);
  return dist[START_X][START_Y];
}

static int failures = 0;
static void fail(const World &w, const std::vector<int> &s, const char *what) {
  failures++;
  printf("  FAIL %s from %s: %s (%s) at real %d,%d map %d,%d\n", w.name.c_str(), cornerName(s[0], s[1]),
         what, why ? why : "", R.x, R.y, posX, posY);
}

// Everything for one maze and start. verbose: print a line.
static bool runStart(const World &w, const std::vector<int> &s, bool verbose, long *searchCells) {
  R = Robot(); R.w = &w; R.sx = s[0]; R.sy = s[1]; R.sh = s[2];
  initMaze(); mapSide = 0; why = 0;
  if (!search()) { fail(w, s, "first search"); return false; }
  long firstCells = R.cells, firstArcs = R.arcs, firstU = R.uturns, firstStops = R.stops;
  if (searchCells) *searchCells += firstCells;
  bool left = mapSide == 2;
  if (!search()) { fail(w, s, "second search"); return false; }
  uint8_t optimistic = shortestPath(false), known = shortestPath(true);
  int truth = shortestWorld(w, s[0], s[1]);
  if (known == optimistic && known != truth) { fail(w, s, "'proven shortest' is not the shortest"); return false; }
  int cells;
  if (!fastRun(cells)) { fail(w, s, "fast run"); return false; }
  if (cells != known) { fail(w, s, "fast run length differs from the known path"); return false; }
  if (verbose)
    printf("  %-13s from %-3s maze on the %-5s  search: %3ld cells %3ld stops %2ld arcs %2ld U-turns, "
           "fast run %2d cells (shortest %d)%s\n", w.name.c_str(), cornerName(s[0], s[1]),
           left ? "left" : "right", firstCells, firstStops, firstArcs, firstU, cells, truth,
           known == optimistic ? " proven" : "");
  return true;
}
int main(int argc, char **argv) {
  int randoms = argc > 1 ? atoi(argv[1]) : 200;
  std::vector<World> mazes;
  DIR *dir = opendir("mazes");
  std::vector<std::string> names;
  for (dirent *e; dir && (e = readdir(dir));) {
    std::string n = e->d_name;
    if (n.size() > 4 && n.substr(n.size() - 4) == ".maz") names.push_back(n);
  }
  if (dir) closedir(dir);
  std::sort(names.begin(), names.end());
  for (size_t i = 0; i < names.size(); i++) {
    World w; w.name = names[i].substr(0, names[i].size() - 4);
    if (loadMaz("mazes/" + names[i], w)) mazes.push_back(w);
  }

  printf("Contest mazes, every proper start corner:\n");
  for (size_t i = 0; i < mazes.size(); i++) {
    std::vector<std::vector<int> > s = starts(mazes[i]);
    for (size_t j = 0; j < s.size(); j++) runStart(mazes[i], s[j], true, 0);
  }

  // A saved map from one corner, then a search from another corner without wiping it.
  printf("Saved map from another corner (not wiped):\n");
  int pairs = 0;
  for (size_t i = 0; i < mazes.size(); i++) {
    std::vector<std::vector<int> > s = starts(mazes[i]);
    for (size_t a = 0; a < s.size(); a++)
      for (size_t b = 0; b < s.size(); b++) {
        if (a == b) continue;
        R = Robot(); R.w = &mazes[i]; R.sx = s[a][0]; R.sy = s[a][1]; R.sh = s[a][2];
        initMaze(); mapSide = 0; why = 0;
        if (!search()) { fail(mazes[i], s[a], "search"); continue; }
        R = Robot(); R.w = &mazes[i]; R.sx = s[b][0]; R.sy = s[b][1]; R.sh = s[b][2];
        why = 0;
        if (!search()) { fail(mazes[i], s[b], "search after another corner's map"); continue; }
        int cells;
        if (!fastRun(cells)) fail(mazes[i], s[b], "fast run after another corner's map");
        pairs++;
      }
  }
  printf("  %d start pairs done\n", pairs);

  printf("Random mazes: %d\n", randoms);
  long total = 0; int runs = 0;
  for (int i = 0; i < randoms; i++) {
    World w; randomMaze(w, 1000 + i, (i % 3) * 0.08);
    std::vector<std::vector<int> > s = starts(w);
    for (size_t j = 0; j < s.size(); j++) { runs++; runStart(w, s[j], false, &total); }
  }
  printf("  %d starts, first search %.1f cells on average\n", runs, runs ? total / (double)runs : 0.0);

  if (failures) { printf("%d FAILURES\n", failures); return 1; }
  printf("ALL OK\n");
  return 0;
}
