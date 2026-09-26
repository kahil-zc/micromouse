#!/bin/sh
# Extracts the maze section of 19_micromouse_final.ino and runs it on random 5 x 10 mazes,
# both ways round, from both corners, with the goal as the corner cell and as the 2x2 block.
set -e
cd "$(dirname "$0")"
sed -n '/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p' \
  ../19_micromouse_final/19_micromouse_final.ino > maze_19_section.inc
for g in 1 2; do
  g++ -std=c++11 -O2 -Wall -DGOAL_SIZE=$g micromouse_19_sim.cpp -o micromouse_19_sim
  ./micromouse_19_sim
done
