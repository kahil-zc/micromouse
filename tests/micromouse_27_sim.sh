#!/bin/sh
# Extracts the maze layout and the maze section of 27_explore_all.ino and runs the explorer
# on random 10 x 5 mazes (both ways round, with loops, with wrong wall readings).
set -e
cd "$(dirname "$0")"
F=../27_explore_all/27_explore_all.ino
sed -n '/^\/\/ === MAZE LAYOUT ===/,/^\/\/ === YOUR CALIBRATED TOF BIAS ===/p' $F > maze_27_config.inc
sed -n '/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p' $F > maze_27_section.inc
g++ -std=c++11 -O2 -Wall -Wno-unused-function micromouse_27_sim.cpp -o micromouse_27_sim
./micromouse_27_sim
