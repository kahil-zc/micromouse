#!/bin/sh
# Extracts the maze section of 15_micromouse_final.ino and runs it on random 18x18 mazes.
set -e
cd "$(dirname "$0")"
sed -n '/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p' \
  ../15_micromouse_final/15_micromouse_final.ino > maze_final_section.inc
g++ -std=c++11 -O2 -Wall micromouse_final_sim.cpp -o micromouse_final_sim
./micromouse_final_sim
