#!/bin/sh
# Extracts the maze section of 16_micromouse_5x10.ino and runs it on random 10x5 mazes.
set -e
cd "$(dirname "$0")"
sed -n '/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p' \
  ../16_micromouse_5x10/16_micromouse_5x10.ino > maze_5x10_section.inc
g++ -std=c++11 -O2 -Wall micromouse_5x10_sim.cpp -o micromouse_5x10_sim
./micromouse_5x10_sim
