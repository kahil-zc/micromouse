#!/bin/sh
# Extracts the maze section of 21_micromouse_final.ino and runs it on random 5 x 10 mazes,
# both ways round, from both corners, with the goal as the corner cell and as the 2x2 block.
set -e
cd "$(dirname "$0")"
sed -n '/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p' \
  ../21_micromouse_final/21_micromouse_final.ino > maze_21_section.inc
for g in 1 2; do
  g++ -std=c++11 -O2 -Wall -DGOAL_SIZE=$g micromouse_21_sim.cpp -o micromouse_21_sim
  ./micromouse_21_sim
done
