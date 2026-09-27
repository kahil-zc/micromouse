#!/bin/bash
# Runs the goal-room search of 31_find_goal on random mazes (see find_goal_sim.cpp).
#   tests/find_goal_sim.sh [mazes per start position] [loop density 0..1]
# Settings can be tried without editing the sketch, e.g.  PULL_RIGHT=4 UTURN_COST=0 tests/find_goal_sim.sh
# (also MAP_SIZE, START_X, START_Y, PULL_AHEAD, and MAX_SIDE = the biggest maze tried).
set -e
cd "$(dirname "$0")"
INO=../31_find_goal/31_find_goal.ino
B=build
mkdir -p $B
{
  sed -n '/^\/\/ --- MAZE CONFIG/,/^\/\/ --- END MAZE CONFIG ---/p' $INO
  grep -E '^#define (ST_[A-Z_]+|SEARCH_PASSES) ' $INO
} > $B/maze_config.h
for v in MAP_SIZE START_X START_Y PULL_RIGHT PULL_AHEAD UTURN_COST; do
  if [ -n "${!v}" ]; then sed -i "s/^#define $v .*/#define $v ${!v}/" $B/maze_config.h; fi
done
sed -n '/^\/\/ --- MAZE (no hardware calls)/,/^\/\/ --- END MAZE ---/p' $INO > $B/maze_section.h
g++ -std=c++11 -O2 -Wall -Wno-unused-function ${MAX_SIDE:+-DMAX_SIDE=$MAX_SIDE} -I$B find_goal_sim.cpp -o $B/find_goal_sim
$B/find_goal_sim "$@"
