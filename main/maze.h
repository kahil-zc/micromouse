#pragma once
#include <stdint.h>

// Pure logic (no hardware calls) so it can be compiled and tested on a PC.

// ---------------- Maze geometry (single source of truth) ----------------
#define MAZE_SIZE 16
#define GOAL_X0   7   // Lower-left cell of the 2x2 goal block
#define GOAL_Y0   7

enum Dir { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
enum Target { TARGET_GOAL, TARGET_START };

#define DIST_UNREACHABLE 255

// Per cell: bits 0-3 = wall present (N,E,S,W), bits 4-7 = that wall has been observed.
extern uint8_t mazeWalls[MAZE_SIZE * MAZE_SIZE];
// Column of the start cell (0, or MAZE_SIZE-1 if the robot started in the right-hand corner).
extern uint8_t mazeStartX;

// Clears everything except the outer boundary, start at (0,0).
void maze_init();

void maze_set_wall(uint8_t x, uint8_t y, uint8_t dir, bool present);
bool maze_wall(uint8_t x, uint8_t y, uint8_t dir);
bool maze_known(uint8_t x, uint8_t y, uint8_t dir);
bool maze_visited(uint8_t x, uint8_t y);

// Records the walls seen from (x, y) while facing `heading`. The robot starts believing it is
// in the left-hand corner; if it sees an opening to the west from column 0 it must be in the
// right-hand corner, so the map is moved over and x is updated. Returns true if that happened.
bool maze_update(uint8_t &x, uint8_t y, uint8_t heading, bool wallLeft, bool wallFront, bool wallRight);

bool maze_in_target(uint8_t x, uint8_t y, Target t);

// Breadth-first flood fill from the target. knownOnly = true treats unobserved walls as walls
// (safe fast-run path); false treats them as open (optimistic search).
void maze_flood(Target t, bool knownOnly);
uint8_t maze_dist(uint8_t x, uint8_t y);

// Best direction to leave (x, y) after maze_flood, preferring to keep `heading`. -1 if stuck.
int8_t maze_best_dir(uint8_t x, uint8_t y, uint8_t heading, bool knownOnly);

// How many cells to drive straight from (x, y) in `dir` before the next decision.
// With stopAtUnvisited, stops in the first cell that hasn't been fully observed.
uint8_t maze_straight_cells(uint8_t x, uint8_t y, uint8_t dir, Target t, bool knownOnly, bool stopAtUnvisited);

// Shortest start -> goal length in cells, optimistic and known-only. When equal, the
// fast-run path is proven optimal.
void maze_path_lengths(uint8_t &optimistic, uint8_t &knownOnly);

inline void maze_step(uint8_t &x, uint8_t &y, uint8_t dir) {
  if (dir == NORTH) y++;
  else if (dir == EAST) x++;
  else if (dir == SOUTH) y--;
  else x--;
}
