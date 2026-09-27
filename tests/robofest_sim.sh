#!/bin/bash
# Runs the maze code of 32_robofest on the Robofest maze from all 4 corners, classic contest
# mazes and random ones (see robofest_sim.cpp).   tests/robofest_sim.sh [random mazes]
set -e
cd "$(dirname "$0")"
INO=../32_robofest/32_robofest.ino
B=build
mkdir -p $B
{
  sed -n '/^\/\/ --- MAZE CONFIG/,/^\/\/ --- END MAZE CONFIG ---/p' $INO
  grep -E '^#define (ST_[A-Z_]+|TURN_(OK|FAILED|BLOCKED)|MAX_FAILS) ' $INO
} > $B/maze_config.h
sed -n '/^\/\/ --- MAZE (no hardware calls)/,/^\/\/ --- END MAZE ---/p' $INO > $B/maze_section.h
g++ -std=c++11 -O2 -Wall -Wno-unused-function -I$B robofest_sim.cpp -o $B/robofest_sim
$B/robofest_sim "$@"
