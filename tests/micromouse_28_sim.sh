#!/bin/sh
# Runs 28's flood fill on 2000 random 10x5 / 5x10 mazes (search to the goal, home, fast-run path).
D=$(cd "$(dirname "$0")" && pwd)
sed -n "/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p" $D/../28_floodfill_fixed/28_floodfill_fixed.ino > $D/maze28.inc
g++ -O2 -I$D -o $D/sim28 $D/micromouse_28_sim.cpp && $D/sim28
