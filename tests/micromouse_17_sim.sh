#!/bin/sh
# Extracts the maze section of 17_micromouse_final.ino and runs it on random 5 x 10 mazes,
# first with perfect wall readings, then with noisy ones.
set -e
cd "$(dirname "$0")"
sed -n '/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p' \
  ../17_micromouse_final/17_micromouse_final.ino > maze_17_section.inc
g++ -std=c++11 -O2 -Wall micromouse_17_sim.cpp -o micromouse_17_sim
./micromouse_17_sim
