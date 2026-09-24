// Host-side regression for main/maze.cpp (the exact firmware file, compiled for the PC).
//
// Generates random 16x16 competition-style mazes (start cell in a bottom corner with three
// walls, 2x2 goal in the centre with a single entry, some loops), then drives the same
// search / fast-run loop as main.ino with simulated perfect sensing and checks that:
//   - the robot never drives through a real wall
//   - search reaches the goal and gets home, in both start corners
//   - when the map says the path is proven, the fast path equals the true shortest path
//   - the fast path never uses an unobserved wall
//
// Build & run:  g++ -std=c++11 -O2 -Wall -I../main maze_sim.cpp ../main/maze.cpp -o maze_sim && ./maze_sim

#include "maze.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cmath>

static const int N = MAZE_SIZE;
static const int DX[4] = { 0, 1, 0, -1 };
static const int DY[4] = { 1, 0, -1, 0 };

// ---------------- True maze ----------------
struct TrueMaze {
  bool wall[N][N][4];
  int startX;

  bool in(int x, int y) const { return x >= 0 && y >= 0 && x < N && y < N; }
  void set(int x, int y, int d, bool w) {
    wall[x][y][d] = w;
    int nx = x + DX[d], ny = y + DY[d];
    if (in(nx, ny)) wall[nx][ny][(d + 2) & 3] = w;
  }
  static bool isGoal(int x, int y) {
    return x >= GOAL_X0 && x <= GOAL_X0 + 1 && y >= GOAL_Y0 && y <= GOAL_Y0 + 1;
  }
  // Walls that must stay: goal perimeter (except the entry) and the start cell's side wall.
  bool locked(int x, int y, int d, int entryX, int entryY, int entryD) const {
    int nx = x + DX[d], ny = y + DY[d];
    if (!in(nx, ny)) return true;
    if (isGoal(x, y) != isGoal(nx, ny)) {
      if (x == entryX && y == entryY && d == entryD) return false;
      if (nx == entryX && ny == entryY && ((d + 2) & 3) == entryD) return false;
      return true;
    }
    int inner = startX == 0 ? 1 : N - 2;
    if (y == 0 && ny == 0 && ((x == startX && nx == inner) || (nx == startX && x == inner))) return true;
    return false;
  }
  // Shortest start -> goal distance (cells), -1 if unreachable.
  int shortest() const {
    int dist[N][N];
    memset(dist, -1, sizeof(dist));
    std::vector<std::pair<int, int>> q;
    q.push_back(std::make_pair(startX, 0));
    dist[startX][0] = 0;
    for (size_t h = 0; h < q.size(); h++) {
      int x = q[h].first, y = q[h].second;
      if (isGoal(x, y)) return dist[x][y];
      for (int d = 0; d < 4; d++) {
        int nx = x + DX[d], ny = y + DY[d];
        if (wall[x][y][d] || !in(nx, ny) || dist[nx][ny] >= 0) continue;
        dist[nx][ny] = dist[x][y] + 1;
        q.push_back(std::make_pair(nx, ny));
      }
    }
    return -1;
  }
};

static void generate(TrueMaze &m, unsigned seed, int startX) {
  srand(seed);
  m.startX = startX;
  for (int x = 0; x < N; x++) for (int y = 0; y < N; y++) for (int d = 0; d < 4; d++) m.wall[x][y][d] = true;

  // Perfect maze by randomized DFS
  bool seen[N][N] = {};
  std::vector<std::pair<int, int>> stack(1, std::make_pair(0, 0));
  seen[0][0] = true;
  while (!stack.empty()) {
    int x = stack.back().first, y = stack.back().second;
    int opts[4], n = 0;
    for (int d = 0; d < 4; d++) {
      int nx = x + DX[d], ny = y + DY[d];
      if (m.in(nx, ny) && !seen[nx][ny]) opts[n++] = d;
    }
    if (!n) { stack.pop_back(); continue; }
    int d = opts[rand() % n];
    m.set(x, y, d, false);
    seen[x + DX[d]][y + DY[d]] = true;
    stack.push_back(std::make_pair(x + DX[d], y + DY[d]));
  }

  // Goal: open inside, close the perimeter except one random entry
  for (int x = GOAL_X0; x <= GOAL_X0 + 1; x++)
    for (int y = GOAL_Y0; y <= GOAL_Y0 + 1; y++)
      for (int d = 0; d < 4; d++) {
        int nx = x + DX[d], ny = y + DY[d];
        m.set(x, y, d, !TrueMaze::isGoal(nx, ny));
      }
  int entries[8][3], ne = 0;
  for (int x = GOAL_X0; x <= GOAL_X0 + 1; x++)
    for (int y = GOAL_Y0; y <= GOAL_Y0 + 1; y++)
      for (int d = 0; d < 4; d++)
        if (!TrueMaze::isGoal(x + DX[d], y + DY[d])) { entries[ne][0] = x; entries[ne][1] = y; entries[ne][2] = d; ne++; }
  int e = rand() % ne;
  int ex = entries[e][0], ey = entries[e][1], ed = entries[e][2];
  m.set(ex, ey, ed, false);

  // Start cell: three walls, open to the north
  int inner = startX == 0 ? EAST : WEST;
  m.set(startX, 0, inner, true);
  m.set(startX, 0, NORTH, false);

  // Reconnect anything the forced walls cut off, then knock out some walls to make loops
  for (;;) {
    bool reach[N][N] = {};
    std::vector<std::pair<int, int>> q(1, std::make_pair(startX, 0));
    reach[startX][0] = true;
    for (size_t h = 0; h < q.size(); h++)
      for (int d = 0; d < 4; d++) {
        int x = q[h].first, y = q[h].second, nx = x + DX[d], ny = y + DY[d];
        if (!m.wall[x][y][d] && m.in(nx, ny) && !reach[nx][ny]) { reach[nx][ny] = true; q.push_back(std::make_pair(nx, ny)); }
      }
    bool fixed = false, allReached = true;
    for (int x = 0; x < N && !fixed; x++)
      for (int y = 0; y < N && !fixed; y++) {
        if (reach[x][y]) continue;
        allReached = false;
        for (int d = 0; d < 4; d++) {
          int nx = x + DX[d], ny = y + DY[d];
          if (m.in(nx, ny) && reach[nx][ny] && !m.locked(x, y, d, ex, ey, ed)) { m.set(x, y, d, false); fixed = true; break; }
        }
      }
    if (allReached) break;
    if (!fixed) {  // Pick any unlocked wall between two unreached-adjacent cells next time
      for (int x = 0; x < N && !fixed; x++)
        for (int y = 0; y < N && !fixed; y++)
          for (int d = 0; d < 4 && !fixed; d++)
            if (m.in(x + DX[d], y + DY[d]) && m.wall[x][y][d] && !m.locked(x, y, d, ex, ey, ed) && rand() % 4 == 0) { m.set(x, y, d, false); fixed = true; }
    }
  }
  for (int k = 0; k < 25; k++) {
    int x = rand() % N, y = rand() % N, d = rand() % 4;
    if (m.in(x + DX[d], y + DY[d]) && !m.locked(x, y, d, ex, ey, ed)) m.set(x, y, d, false);
  }
}

// ---------------- Robot simulation (mirrors main.ino) ----------------
// Rough timing (seconds) using the firmware's speed profiles
static const double SEARCH_V = 0.25, SEARCH_A = 1.5, RETURN_V = 0.40, RETURN_A = 2.0;
static const double CELL = 0.18, SENSE_S = 0.12, TURN_S = 0.5, SETTLE_S = 0.1;
static double straightTime(int cells, double v, double a) {
  double d = cells * CELL;
  if (d < v * v / a) return 2 * sqrt(d / a) + SETTLE_S;  // Never reaches top speed
  return d / v + v / a + SETTLE_S;
}

struct Robot {
  const TrueMaze *m;
  uint8_t posX, posY, heading;
  long cellsDriven, stops, turns;
  double seconds;

  int realX() const { return posX + (m->startX - mazeStartX); }

  void begin() { posX = mazeStartX; posY = 0; heading = NORTH; }

  void sense() {
    seconds += SENSE_S;
    int rx = realX();
    uint8_t l = (heading + 3) & 3, r = (heading + 1) & 3;
    maze_update(posX, posY, heading, m->wall[rx][posY][l], m->wall[rx][posY][heading], m->wall[rx][posY][r]);
    stops++;
  }

  bool turnTo(uint8_t dir) {
    if (dir != heading) { turns++; seconds += ((dir - heading) & 3) == 2 ? 2 * TURN_S : TURN_S; }
    heading = dir;
    return true;
  }

  // Returns false if it would drive through a real wall.
  bool drive(uint8_t dir, uint8_t cells, bool markOpen) {
    for (uint8_t i = 0; i < cells; i++) {
      int rx = realX();
      if (m->wall[rx][posY][dir]) return false;
      if (markOpen) maze_set_wall(posX, posY, dir, false);
      maze_step(posX, posY, dir);
      cellsDriven++;
    }
    return true;
  }

  bool searchTo(Target t) {
    for (int steps = 0; steps < 1024; steps++) {
      sense();
      if (maze_in_target(posX, posY, t)) return true;
      maze_flood(t, false);
      int8_t dir = maze_best_dir(posX, posY, heading, false);
      if (dir < 0) return false;
      turnTo(dir);
      uint8_t cells = maze_straight_cells(posX, posY, dir, t, false, true);
      seconds += straightTime(cells, SEARCH_V, SEARCH_A);
      if (!drive(dir, cells, true)) return false;
    }
    return false;
  }

  // Follows known-open walls only. Returns path length in cells, -1 on failure.
  int followKnown(Target t, double v, double a) {
    maze_flood(t, true);
    int len = 0;
    while (!maze_in_target(posX, posY, t)) {
      int8_t dir = maze_best_dir(posX, posY, heading, true);
      if (dir < 0) return -1;
      turnTo(dir);
      uint8_t cells = maze_straight_cells(posX, posY, dir, t, true, false);
      seconds += straightTime(cells, v, a);
      for (uint8_t i = 0; i < cells; i++) {
        if (!maze_known(posX, posY, dir)) return -1;  // Must only use observed walls
        if (!drive(dir, 1, false)) return -1;
      }
      len += cells;
    }
    return len;
  }

  // Same decision as main.ino's returnHome()
  bool returnHome() {
    uint8_t opt, known;
    maze_path_lengths(opt, known);
    if (opt == known) return followKnown(TARGET_START, RETURN_V, RETURN_A) >= 0;
    return searchTo(TARGET_START);
  }
};

int main() {
  const int MAZES = 600;
  int failures = 0, proven = 0, roundsHist[8] = {};
  long totalStops = 0, totalCells = 0;
  int maxStops = 0;
  std::vector<double> times;

  for (int i = 0; i < MAZES; i++) {
    TrueMaze m;
    int startX = (i % 2) ? N - 1 : 0;
    generate(m, 1000 + i, startX);
    int best = m.shortest();
    if (best < 0) { printf("maze %d: generator made an unsolvable maze\n", i); failures++; continue; }

    maze_init();
    Robot r = { &m, 0, 0, 0, 0, 0, 0, 0.0 };

    // Search round trips until the path is proven (the robot would do this with SEARCH mode)
    int rounds = 0;
    bool ok = true;
    uint8_t opt = 0, known = 255;
    long firstRoundStops = 0;
    double firstRoundSeconds = 0;
    while (rounds < 6) {
      r.begin();
      if (!r.searchTo(TARGET_GOAL) || !r.returnHome()) { ok = false; break; }
      if (r.realX() != m.startX || r.posY != 0) { ok = false; break; }
      rounds++;
      if (rounds == 1) { firstRoundStops = r.stops; firstRoundSeconds = r.seconds; }
      maze_path_lengths(opt, known);
      if (opt == known) break;
    }
    if (!ok) { printf("maze %d (start x=%d): search failed or drove through a wall\n", i, startX); failures++; continue; }
    if (mazeStartX != startX) { printf("maze %d: start corner not detected\n", i); failures++; continue; }

    r.begin();
    int len = r.followKnown(TARGET_GOAL, SEARCH_V, SEARCH_A);
    if (len < 0) { printf("maze %d: fast run failed\n", i); failures++; continue; }
    if (len != known) { printf("maze %d: fast run %d cells, map said %d\n", i, len, known); failures++; continue; }
    if (opt == known) {
      proven++;
      if (len != best) { printf("maze %d: PROVEN path %d but true shortest is %d\n", i, len, best); failures++; continue; }
    }
    if (len < best) { printf("maze %d: impossible path shorter than shortest\n", i); failures++; continue; }
    roundsHist[std::min(rounds, 7)]++;
    totalStops += firstRoundStops;
    totalCells += r.cellsDriven;
    maxStops = std::max(maxStops, (int)firstRoundStops);
    times.push_back(firstRoundSeconds);
  }

  printf("%d mazes (half with the start in each bottom corner)\n", MAZES);
  printf("failures: %d\n", failures);
  printf("proven shortest: %d\n", proven);
  printf("search round trips needed:");
  for (int k = 1; k < 7; k++) if (roundsHist[k]) printf("  %d:%d", k, roundsHist[k]);
  printf("\nfirst round trip stops: avg %.0f, max %d\n", (double)totalStops / (MAZES - failures), maxStops);
  if (!times.empty()) {
    std::sort(times.begin(), times.end());
    int over8 = 0;
    for (double t : times) if (t > 480) over8++;
    printf("first round trip est. time: median %.1f min, 90th pct %.1f min, max %.1f min, over 8 min: %d\n",
           times[times.size() / 2] / 60, times[times.size() * 9 / 10] / 60, times.back() / 60, over8);
  }
  return failures ? 1 : 0;
}
