#!/bin/sh
# Extracts the maze layout and the maze section of 26_wall_follower.ino and runs the wall
# follower on random 10 x 5 mazes (both ways round, with loops, with wrong wall readings).
set -e
cd "$(dirname "$0")"
F=../26_wall_follower/26_wall_follower.ino
sed -n '/^\/\/ === MAZE LAYOUT ===/,/^\/\/ === YOUR CALIBRATED TOF BIAS ===/p' $F > maze_26_config.inc
sed -n '/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p' $F > maze_26_section.inc
g++ -std=c++11 -O2 -Wall -Wno-unused-function micromouse_26_sim.cpp -o micromouse_26_sim
./micromouse_26_sim
