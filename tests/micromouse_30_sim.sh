#!/bin/sh
# Runs 30_find_goal's maze code on random mazes (see micromouse_30_sim.cpp), with the search
# pulled right (PULL_RIGHT 4, as in the sketch) and not pulled (0) for comparison.
D=$(cd "$(dirname "$0")" && pwd)
sed -n '/^\/\/ --- MAZE (no hardware calls) ---/,/^\/\/ --- END MAZE ---/p' $D/../30_find_goal/30_find_goal.ino > $D/maze30.inc
for p in 4 0; do
  g++ -O2 -DPULL_RIGHT=$p -I$D -o $D/sim30 $D/micromouse_30_sim.cpp || exit 1
  $D/sim30 0 left right; $D/sim30 0.02 left right; $D/sim30 0 left; $D/sim30 0
done
