#!/bin/sh
# Extracts the maze section of 14_floodfill_final.ino and runs it on random mazes.
set -e
cd "$(dirname "$0")"
sed -n '/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p' \
  ../14_floodfill_final/14_floodfill_final.ino > maze_section.inc
g++ -std=c++11 -O2 -Wall floodfill_final_sim.cpp -o floodfill_final_sim
./floodfill_final_sim
