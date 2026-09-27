#!/bin/sh
# Extracts the maze layout and the maze section of 25_wall_follower.ino and runs the wall
# follower on random 10 x 5 mazes (both ways round, with loops, with wrong wall readings).
set -e
cd "$(dirname "$0")"
F=../25_wall_follower/25_wall_follower.ino
sed -n '/^\/\/ === MAZE LAYOUT ===/,/^\/\/ === YOUR CALIBRATED TOF BIAS ===/p' $F > maze_25_config.inc
sed -n '/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p' $F > maze_25_section.inc
g++ -std=c++11 -O2 -Wall -Wno-unused-function micromouse_25_sim.cpp -o micromouse_25_sim
./micromouse_25_sim
