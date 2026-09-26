#!/bin/sh
# Extracts the maze section of 18_micromouse_corner.ino and runs it on random 10 x 5 mazes,
# with the goal as the corner cell and as the 2x2 corner block.
set -e
cd "$(dirname "$0")"
sed -n '/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p' \
  ../18_micromouse_corner/18_micromouse_corner.ino > maze_18_section.inc
for g in 1 2; do
  g++ -std=c++11 -O2 -Wall -DGOAL_SIZE=$g micromouse_18_sim.cpp -o micromouse_18_sim
  ./micromouse_18_sim
done
