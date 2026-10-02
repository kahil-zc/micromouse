// OLED DOOM — maps. One string per level, row after row.
//
//   # T S   walls (brick / tech / stone)      .  floor
//   D       door (opens when you walk up to it)
//   L       red door (needs the red keycard)   E  exit switch: walk into it to finish the level
//   P       player start
//   z i d B zombieman, imp, demon, baron of hell
//   b       explosive barrel
//   h a A   medikit, ammo clip, box of ammo
//   s k     shotgun, red keycard
//
// The border must be solid. tests/doom_sim.cpp checks every map is closed and finishable.

#pragma once

struct LevelDef {
  const char* id;
  const char* name;
  float angle;          // start facing, radians (0 = +x / right on the map, PI/2 = +y / down)
  uint8_t w;
  const char* rows;
};

static const char MAP_E1M1[] =
  "########################"
  "#....#.................#"
  "#.P..D..z...b.....z....#"
  "#....#.................#"
  "#.a..#......z..........#"
  "############D#######D###"
  "S....S...........T.....T"
  "S....S.....i.....T..z..T"
  "S.s..S...........T.....T"
  "S....D....b......T..i..T"
  "S..h.S.....z.....T.....T"
  "S....S...........T....hE"
  "S....S....a......T.....T"
  "SSSSSS###########TTTTTTT";

static const char MAP_E1M2[] =
  "########################"
  "#.....#...a...#........#"
  "#..P..D...z...D..d.....#"
  "#.....#.......#.....b..#"
  "#..a..#...b...#........#"
  "###D###...i...#####D####"
  "#.....#.......#....a...#"
  "#.z...####D####..z..h..#"
  "#.....#.......#........#"
  "#.....D...d...D....i...#"
  "#..h..#.......#........#"
  "#.....#...s...####L#####"
  "###D###.......T........T"
  "S.....S########...z....T"
  "S..k..S.......#.......dE"
  "S..z..D...A...#..i.....T"
  "S.....S..i....#.h...A..T"
  "SSSSSSSSSSSSSSSTTTTTTTTT";

static const char MAP_E1M3[] =
  "##########################"
  "#......#...a...#.........#"
  "#..P...D..z.b..D...i..z..#"
  "#......#.......#.........#"
  "#..A...#..i....#####D#####"
  "####D###.......T.........T"
  "#......####D####..d...b..T"
  "#..z...#.......T...A.....T"
  "#......D...i...D...z..h..T"
  "#..h...#..b.b..T.........T"
  "#......#.......TTTTTTTTTTT"
  "##D#####...d...S.........S"
  "#......#########A.i...i..S"
  "#..k.a.#.......S.........S"
  "#..d...D..z.A..L.........S"
  "#......#.......S....B....S"
  "#..i...#..h....S.........S"
  "#......#.......S.b.....h.E"
  "#......#..z....S.........S"
  "##########################";

static const LevelDef LEVELS[] = {
  { "E1M1", "HANGAR",         0.0f, 24, MAP_E1M1 },
  { "E1M2", "NUCLEAR PLANT",  0.0f, 24, MAP_E1M2 },
  { "E1M3", "TOXIN REFINERY", 0.0f, 26, MAP_E1M3 },
};
static const int NUM_LEVELS = sizeof(LEVELS) / sizeof(LEVELS[0]);
