#!/bin/sh
# Extracts the maze layout and the maze section of 24_micromouse_final.ino and runs them on
# random 10 x 5 mazes (both ways round) with wrong wall readings.
set -e
cd "$(dirname "$0")"
F=../24_micromouse_final/24_micromouse_final.ino
sed -n '/^\/\/ === MAZE LAYOUT ===/,/^\/\/ === YOUR CALIBRATED TOF BIAS ===/p' $F > maze_24_config.inc
sed -n '/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p' $F > maze_24_section.inc
g++ -std=c++11 -O2 -Wall -Wno-unused-function micromouse_24_sim.cpp -o micromouse_24_sim
./micromouse_24_sim
