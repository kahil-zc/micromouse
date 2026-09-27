#!/bin/sh
# Runs 29_find_goal's maze code on random mazes (see micromouse_29_sim.cpp).
D=$(cd "$(dirname "$0")" && pwd)
sed -n '/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p' $D/../29_find_goal/29_find_goal.ino > $D/maze29.inc
g++ -O2 -I$D -o $D/sim29 $D/micromouse_29_sim.cpp || exit 1
for e in 0 0.02 0.05; do $D/sim29 $e; done
