// Host-side test for the goal-room search of 31_find_goal/31_find_goal.ino. find_goal_sim.sh
// cuts the "MAZE CONFIG" and "MAZE" sections out of the sketch, so this runs the robot's own
// planner code.
//
// Random mazes (4..12 cells a side, as far as they fit the map from that start) with a
// 2x2 goal room and no other open 2x2, some loops, a start cell with walls on 3 sides. The
// robot sees the 3 walls of every cell it stops in and drives like the sketch's searchStep /
// fastStep. It checks that:
//   - the robot never drives through a wall and never gets stuck
//   - the search finds the right room and gets home
//   - a second search + fast run reach the goal, the fast run only on walls it has seen
//   - when the map says "proven shortest", the known path really is the shortest
// and prints how much driving and turning the search took.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>

#include "maze_config.h"
#include "maze_section.h"

// ---------------- True maze (on the robot's map grid) ----------------
struct Truth {
  bool wall[MAP_SIZE][MAP_SIZE][4];
  int x0, y0, w, h;       // maze area on the map
  int roomX, roomY;       // goal room lower-left

  bool inMaze(int x, int y) const { return x >= x0 && y >= y0 && x < x0 + w && y < y0 + h; }
  void set(int x, int y, int d, bool v) {
    wall[x][y][d] = v;
    int nx = x + DX[d], ny = y + DY[d];
    if (inMap(nx, ny)) wall[nx][ny][(d + 2) & 3] = v;
  }
  bool blockOpen(int x, int y) const {  // 2x2 with lower-left (x, y) has no inside walls
    if (!inMaze(x, y) || !inMaze(x + 1, y + 1)) return false;
    return !wall[x][y][0] && !wall[x][y][1] && !wall[x + 1][y][0] && !wall[x][y + 1][1];
  }
  // Shortest start -> room, -1 = none.
  int shortest() const {
    static int d[MAP_SIZE][MAP_SIZE];
    for (int x = 0; x < MAP_SIZE; x++) for (int y = 0; y < MAP_SIZE; y++) d[x][y] = -1;
    std::vector<int> q;
    q.push_back(START_X * MAP_SIZE + START_Y);
    d[START_X][START_Y] = 0;
    for (size_t i = 0; i < q.size(); i++) {
      int x = q[i] / MAP_SIZE, y = q[i] % MAP_SIZE;
      if (inRoom(x, y, roomX, roomY)) return d[x][y];
      for (int k = 0; k < 4; k++) {
        if (wall[x][y][k]) continue;
        int nx = x + DX[k], ny = y + DY[k];
        if (d[nx][ny] >= 0) continue;
        d[nx][ny] = d[x][y] + 1;
        q.push_back(nx * MAP_SIZE + ny);
      }
    }
    return -1;
  }
};

static int find(std::vector<int> &p, int a) { while (p[a] != a) a = p[a] = p[p[a]]; return a; }

// Where the start is along the bottom row: 0 = left corner, 1 = right corner, 2 = middle.
static int startColumn(int startCol, int w) { return startCol == 0 ? 0 : startCol == 1 ? w - 1 : w / 2; }
static bool fits(int startCol, int w, int h) {
  int x0 = START_X - startColumn(startCol, w);
  return x0 >= 0 && x0 + w <= MAP_SIZE && START_Y + h <= MAP_SIZE;
}

// A maze of 4..MAX_SIDE cells a side that fits the map.
#ifndef MAX_SIDE
#define MAX_SIDE 12
#endif
static void makeMaze(Truth &t, int startCol, double loops) {
  for (;;) {
    t.w = 4 + rand() % (MAX_SIDE - 3);
    t.h = 4 + rand() % (MAX_SIDE - 3);
    if (!fits(startCol, t.w, t.h)) continue;
    t.x0 = START_X - startColumn(startCol, t.w); t.y0 = START_Y;
    if (rand() % 3) { t.roomX = t.x0 + (t.w - 2) / 2; t.roomY = t.y0 + (t.h - 2) / 2; }
    else { t.roomX = t.x0 + rand() % (t.w - 1); t.roomY = t.y0 + rand() % (t.h - 1); }
    if (inRoom(START_X, START_Y, t.roomX, t.roomY)) continue;

    for (int x = 0; x < MAP_SIZE; x++) for (int y = 0; y < MAP_SIZE; y++) for (int d = 0; d < 4; d++) t.wall[x][y][d] = true;
    std::vector<int> par(MAP_SIZE * MAP_SIZE);
    for (size_t i = 0; i < par.size(); i++) par[i] = i;
    t.set(t.roomX, t.roomY, 0, false);      // the room's 4 inside walls are open
    t.set(t.roomX, t.roomY, 1, false);
    t.set(t.roomX + 1, t.roomY, 0, false);
    t.set(t.roomX, t.roomY + 1, 1, false);
    for (int i = 1; i < 4; i++)
      par[find(par, (t.roomX + (i & 1)) * MAP_SIZE + t.roomY + (i >> 1))] = find(par, t.roomX * MAP_SIZE + t.roomY);
    std::vector<int> edges;  // cell * 2 + (0 = north wall, 1 = east wall)
    for (int x = t.x0; x < t.x0 + t.w; x++)
      for (int y = t.y0; y < t.y0 + t.h; y++) {
        if (y + 1 < t.y0 + t.h) edges.push_back((x * MAP_SIZE + y) * 2);
        if (x + 1 < t.x0 + t.w) edges.push_back((x * MAP_SIZE + y) * 2 + 1);
      }
    std::random_shuffle(edges.begin(), edges.end());
    auto startSide = [&](int x, int y, int d) {  // the start cell's side walls stay
      return d == 1 && y == START_Y && (x == START_X || x + 1 == START_X);
    };
    std::vector<int> rest;
    for (size_t i = 0; i < edges.size(); i++) {
      int c = edges[i] / 2, d = edges[i] & 1, x = c / MAP_SIZE, y = c % MAP_SIZE;
      int n = (x + DX[d]) * MAP_SIZE + y + DY[d];
      if (startSide(x, y, d) || !t.wall[x][y][d]) continue;
      if (find(par, c) != find(par, n)) { par[find(par, c)] = find(par, n); t.set(x, y, d, false); }
      else rest.push_back(edges[i]);
    }
    for (size_t i = 0; i < rest.size(); i++) {  // loops, never making another open 2x2
      if (rand() / (double)RAND_MAX >= loops) continue;
      int c = rest[i] / 2, d = rest[i] & 1, x = c / MAP_SIZE, y = c % MAP_SIZE;
      t.set(x, y, d, false);
      bool bad = false;
      for (int bx = x - 1; bx <= x; bx++)
        for (int by = y - 1; by <= y; by++)
          if (t.blockOpen(bx, by) && !(bx == t.roomX && by == t.roomY)) bad = true;
      if (bad) t.set(x, y, d, true);
    }
    // Every cell connected (Kruskal saw to that) unless the start's side walls cut it off.
    if (t.shortest() < 0) continue;
    bool ok = true;
    for (int x = t.x0; x < t.x0 + t.w - 1; x++)
      for (int y = t.y0; y < t.y0 + t.h - 1; y++)
        if (t.blockOpen(x, y) != (x == t.roomX && y == t.roomY)) ok = false;
    if (ok) return;
  }
}

// ---------------- Robot ----------------
struct Stats { long stops, cells, arcs, deadUturns, openUturns, runs; double secs; };
static Stats st;
static int runStateSim;

static void senseHere(const Truth &t) {
  uint8_t bits = 0;
  for (int i = 0; i < 3; i++) if (t.wall[posX][posY][(facing + 3 + i) & 3]) bits |= 1 << i;
  recordWalls(bits);
  st.stops++;
  st.secs += 0.35;  // settle + 150 ms averaging + planning
}

// driveTo(): turn, then drive up to n cells; the front ToF stops it at a wall the map didn't
// have. Returns false if it didn't get into the next cell.
static bool driveToSim(const Truth &t, uint8_t d, int n, bool fast) {
  char m = relMove(facing, d);
  if (m == 'L' || m == 'R') { st.arcs++; st.secs += 0.9; }
  if (m == 'U') {
    if (t.wall[posX][posY][facing]) { st.deadUturns++; st.secs += 3.5; }
    else { st.openUturns++; st.secs += 3.0; }
  }
  facing = d;
  int k = 0;
  while (k < n && !t.wall[posX + k * DX[d]][posY + k * DY[d]][d]) k++;
  st.runs++;
  st.cells += k;
  st.secs += 0.25 + k * (fast ? 0.3 : 0.45);
  moved(d, k);
  if (k < n) setWall(posX, posY, d, true);
  return k > 0;
}

static bool fail(const char *why, int seed) {
  printf("FAIL seed %d: %s (robot at %d,%d)\n", seed, why, posX - START_X, posY - START_Y);
  return false;
}

// One search from the start cell until home. Mirrors searchStep() of the sketch.
static bool search(const Truth &t, int seed, bool &found) {
  runStateSim = ST_SEARCH_GOAL;
  posX = START_X; posY = START_Y; facing = 0;
  candX = candY = -1;
  int passes = 0;
  found = false;
  for (int guard = 0; guard < 5000; guard++) {
    senseHere(t);
    if (checkGoal()) {
      if (goalX != t.roomX || goalY != t.roomY) return fail("wrong goal room", seed);
      found = true;
      runStateSim = ST_SEARCH_HOME;
    }
    for (int inner = 0;; inner++) {
      if (inner > 50) return fail("keeps choosing without moving", seed);
      uint8_t next = nextStep(runStateSim);
      if (next == NEXT_AT_HOME) return true;
      if (next == NEXT_AT_GOAL) { found = true; runStateSim = ST_SEARCH_HOME; continue; }
      if (next == NEXT_EXPLORED) {
        if (++passes < SEARCH_PASSES) { initMaze(); break; }
        runStateSim = ST_SEARCH_HOME;
        continue;
      }
      if (next == NEXT_NO_WAY) continue;
      if (driveToSim(t, planDir, planRun, false)) break;
    }
  }
  return fail("search never got home", seed);
}

// Fast run to the goal: planned on seen walls only. Mirrors fastStep() of the sketch.
static bool fastRun(const Truth &t, int seed, int &cells) {
  posX = START_X; posY = START_Y; facing = 0;
  cells = 0;
  for (int guard = 0; guard < 500; guard++) {
    if (isGoal(posX, posY)) return true;
    if (!plan(TO_GOAL, true)) return fail("fast run: no known way", seed);
    for (int j = 0; j < planRun; j++)
      if (!seenAt(posX + j * DX[planDir], posY + j * DY[planDir], planDir))
        return fail("fast run on a wall it hasn't seen", seed);
    int before = st.cells;
    driveToSim(t, planDir, planRun, true);
    cells += st.cells - before;
  }
  return fail("fast run never got there", seed);
}

int main(int argc, char **argv) {
  int mazes = argc > 1 ? atoi(argv[1]) : 1000;
  double loops = argc > 2 ? atof(argv[2]) : 0.1;
  const char *corner[3] = { "start in the left corner (maze right)", "start in the right corner (maze left)",
                            "start in the middle of the bottom row" };
  int failures = 0;
  for (int sc = 0; sc < 3; sc++) {
    if (!fits(sc, 4, 4)) { printf("%s: doesn't fit the map, skipped\n", corner[sc]); continue; }
    Stats sum = {};
    double worst = 0;
    long proven = 0, fastExtra = 0;
    for (int i = 0; i < mazes; i++) {
      int seed = sc * 100000 + i;
      srand(seed);
      Truth t;
      makeMaze(t, sc, loops);
      initMaze();
      bool found;
      st = Stats();
      if (!search(t, seed, found)) { failures++; continue; }
      if (!found) { failures++; fail("goal room not found", seed); continue; }
      sum.stops += st.stops; sum.cells += st.cells; sum.arcs += st.arcs;
      sum.deadUturns += st.deadUturns; sum.openUturns += st.openUturns; sum.secs += st.secs;
      worst = std::max(worst, st.secs);

      bool again;
      if (!search(t, seed, again) || !again) { failures++; fail("second search failed", seed); continue; }
      uint8_t optimistic = shortestPath(false), known = shortestPath(true);
      int truth = t.shortest();
      if (known == optimistic && known != truth) { failures++; fail("'proven shortest' is not the shortest", seed); continue; }
      if (known == optimistic) proven++;
      int cells;
      if (!fastRun(t, seed, cells)) { failures++; continue; }
      if (cells != known) { failures++; fail("fast run length differs from the known path", seed); continue; }
      fastExtra += cells - truth;
    }
    printf("%s, %d mazes:\n", corner[sc], mazes);
    printf("  first search, start -> room -> home, per maze: %.1f stops, %.1f cells, %.1f arcs, "
           "%.2f dead-end U-turns, %.2f open U-turns, ~%.0f s (worst %.0f s)\n",
           sum.stops / (double)mazes, sum.cells / (double)mazes, sum.arcs / (double)mazes,
           sum.deadUturns / (double)mazes, sum.openUturns / (double)mazes, sum.secs / mazes, worst);
    printf("  after 2 searches: proven shortest in %.0f%%, fast run %.2f cells longer than the "
           "true shortest on average\n", 100.0 * proven / mazes, fastExtra / (double)mazes);
  }
  if (failures) { printf("%d FAILURES\n", failures); return 1; }
  printf("ALL OK\n");
  return 0;
}
