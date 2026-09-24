#include "maze.h"
#include <string.h>

#if MAZE_SIZE > 16
#error "Cell indices are stored in uint8_t; MAZE_SIZE must be <= 16"
#endif

#define CELLS (MAZE_SIZE * MAZE_SIZE)

uint8_t mazeWalls[CELLS];
uint8_t mazeStartX = 0;

static uint8_t dist[CELLS];
static uint8_t queue[CELLS];

static const int8_t DX[4] = { 0, 1, 0, -1 };
static const int8_t DY[4] = { 1, 0, -1, 0 };

static inline uint8_t cellIndex(uint8_t x, uint8_t y) { return y * MAZE_SIZE + x; }

static inline bool inBounds(int x, int y) {
  return x >= 0 && y >= 0 && x < MAZE_SIZE && y < MAZE_SIZE;
}

void maze_set_wall(uint8_t x, uint8_t y, uint8_t dir, bool present) {
  int nx = x + DX[dir], ny = y + DY[dir];
  if (!inBounds(nx, ny)) present = true;  // The outer boundary is always a wall

  uint8_t bit = 1 << dir;
  uint8_t &cell = mazeWalls[cellIndex(x, y)];
  cell |= bit << 4;
  if (present) cell |= bit; else cell &= ~bit;

  if (inBounds(nx, ny)) {
    uint8_t opp = 1 << ((dir + 2) & 3);
    uint8_t &other = mazeWalls[cellIndex(nx, ny)];
    other |= opp << 4;
    if (present) other |= opp; else other &= ~opp;
  }
}

void maze_init() {
  memset(mazeWalls, 0, sizeof(mazeWalls));
  mazeStartX = 0;
  for (uint8_t i = 0; i < MAZE_SIZE; i++) {
    maze_set_wall(i, 0, SOUTH, true);
    maze_set_wall(i, MAZE_SIZE - 1, NORTH, true);
    maze_set_wall(0, i, WEST, true);
    maze_set_wall(MAZE_SIZE - 1, i, EAST, true);
  }
}

bool maze_wall(uint8_t x, uint8_t y, uint8_t dir) {
  return mazeWalls[cellIndex(x, y)] & (1 << dir);
}

bool maze_known(uint8_t x, uint8_t y, uint8_t dir) {
  return mazeWalls[cellIndex(x, y)] & (0x10 << dir);
}

bool maze_visited(uint8_t x, uint8_t y) {
  return (mazeWalls[cellIndex(x, y)] >> 4) == 0x0F;
}

// Until the robot proves otherwise it assumes the left-hand start corner, so it can only ever
// have travelled along column 0 (east of it is the real outer wall). Keep that column's
// north/south walls, move them to the last column, and forget the fake western boundary.
static void shiftToRightCorner() {
  uint8_t column[MAZE_SIZE];
  for (uint8_t y = 0; y < MAZE_SIZE; y++) column[y] = mazeWalls[cellIndex(0, y)];
  maze_init();
  mazeStartX = MAZE_SIZE - 1;
  for (uint8_t y = 0; y < MAZE_SIZE; y++) {
    const uint8_t dirs[2] = { NORTH, SOUTH };
    for (uint8_t k = 0; k < 2; k++) {
      uint8_t d = dirs[k];
      if (column[y] & (0x10 << d)) maze_set_wall(MAZE_SIZE - 1, y, d, column[y] & (1 << d));
    }
  }
}

bool maze_update(uint8_t &x, uint8_t y, uint8_t heading, bool wallLeft, bool wallFront, bool wallRight) {
  uint8_t left = (heading + 3) & 3;
  uint8_t right = (heading + 1) & 3;

  bool shifted = false;
  if (x == 0 && mazeStartX == 0) {
    bool westOpen = (left == WEST && !wallLeft) || (heading == WEST && !wallFront) ||
                    (right == WEST && !wallRight);
    if (westOpen) {
      shiftToRightCorner();
      x = MAZE_SIZE - 1;
      shifted = true;
    }
  }

  maze_set_wall(x, y, left, wallLeft);
  maze_set_wall(x, y, heading, wallFront);
  maze_set_wall(x, y, right, wallRight);
  return shifted;
}

bool maze_in_target(uint8_t x, uint8_t y, Target t) {
  if (t == TARGET_START) return x == mazeStartX && y == 0;
  return x >= GOAL_X0 && x <= GOAL_X0 + 1 && y >= GOAL_Y0 && y <= GOAL_Y0 + 1;
}

static bool passable(uint8_t x, uint8_t y, uint8_t dir, bool knownOnly) {
  uint8_t cell = mazeWalls[cellIndex(x, y)];
  if (cell & (1 << dir)) return false;
  if (knownOnly && !(cell & (0x10 << dir))) return false;
  return inBounds(x + DX[dir], y + DY[dir]);
}

void maze_flood(Target t, bool knownOnly) {
  memset(dist, DIST_UNREACHABLE, sizeof(dist));
  uint16_t head = 0, tail = 0;

  for (uint8_t y = 0; y < MAZE_SIZE; y++) {
    for (uint8_t x = 0; x < MAZE_SIZE; x++) {
      if (maze_in_target(x, y, t)) {
        dist[cellIndex(x, y)] = 0;
        queue[tail++] = cellIndex(x, y);
      }
    }
  }

  while (head < tail) {
    uint8_t c = queue[head++];
    uint8_t x = c % MAZE_SIZE, y = c / MAZE_SIZE;
    uint8_t next = dist[c] + 1;
    for (uint8_t dir = 0; dir < 4; dir++) {
      if (!passable(x, y, dir, knownOnly)) continue;
      uint8_t n = cellIndex(x + DX[dir], y + DY[dir]);
      if (dist[n] != DIST_UNREACHABLE) continue;
      dist[n] = next;
      queue[tail++] = n;
    }
  }
}

uint8_t maze_dist(uint8_t x, uint8_t y) { return dist[cellIndex(x, y)]; }

int8_t maze_best_dir(uint8_t x, uint8_t y, uint8_t heading, bool knownOnly) {
  uint8_t here = dist[cellIndex(x, y)];
  if (here == DIST_UNREACHABLE || here == 0) return -1;

  // Straight first (fewest turns), then right, left, back
  const uint8_t order[4] = { 0, 1, 3, 2 };
  for (uint8_t k = 0; k < 4; k++) {
    uint8_t dir = (heading + order[k]) & 3;
    if (!passable(x, y, dir, knownOnly)) continue;
    if (dist[cellIndex(x + DX[dir], y + DY[dir])] < here) return dir;
  }
  return -1;
}

uint8_t maze_straight_cells(uint8_t x, uint8_t y, uint8_t dir, Target t, bool knownOnly, bool stopAtUnvisited) {
  uint8_t n = 0;
  while (true) {
    maze_step(x, y, dir);
    n++;
    if (maze_in_target(x, y, t)) break;
    if (stopAtUnvisited && !maze_visited(x, y)) break;
    if (maze_best_dir(x, y, dir, knownOnly) != (int8_t)dir) break;
  }
  return n;
}

void maze_path_lengths(uint8_t &optimistic, uint8_t &knownOnly) {
  maze_flood(TARGET_GOAL, false);
  optimistic = maze_dist(mazeStartX, 0);
  maze_flood(TARGET_GOAL, true);
  knownOnly = maze_dist(mazeStartX, 0);
}
