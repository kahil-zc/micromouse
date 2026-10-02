// Host-side tests for oled_doom/game.cpp (the exact sketch file, compiled for the PC).
//
//   1. Every map: rectangular, closed border, one start, doors sit between two walls, the red
//      key is reachable before its door, the exit is reachable, nothing is walled in.
//   2. A bot plays the whole game in god mode using only joystick/fire input: it must kill
//      every monster on every level, reach each exit and see the victory screen.
//   3. Dying: standing still in a fight kills you; FIRE restarts the level with full health
//      and the monsters back. MAP + FIRE quits to the title with the same map selected.
//   4. Random input soak on every level, dying and restarting, with invariant checks every frame
//      (never inside a wall, no NaNs, health/ammo in range).
//   5. Without god mode the same bot must still be able to finish (prints deaths per level).
//
// Every frame is also rendered, so with -fsanitize=address,undefined the renderer is checked too.
//
// Build & run:
//   g++ -std=c++11 -O2 -Wall -I../oled_doom doom_sim.cpp ../oled_doom/game.cpp -o doom_sim && ./doom_sim
// Pass a folder name to also save screenshots there as .pbm images:  ./doom_sim shots

#include "game.h"
#include "levels.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const float DT = 1.0f / 30.0f;
static uint8_t fb[FB_BYTES];
static const char* shotDir = 0;

static void saveShot(const char* name) {
  if (!shotDir) return;
  char path[512];
  snprintf(path, sizeof(path), "%s/%s.pbm", shotDir, name);
  FILE* f = fopen(path, "w");
  if (!f) return;
  fprintf(f, "P1\n128 64\n");
  for (int y = 0; y < 64; y++) {
    for (int x = 0; x < 128; x++) fprintf(f, "%d ", (fb[x + (y / 8) * 128] >> (y & 7)) & 1);
    fprintf(f, "\n");
  }
  fclose(f);
}

static void frame(const GameInput& in) {
  gameUpdate(in, DT);
  gameRender(fb);
  gameTakeSound();
}

// ------------------------------------------------------------------------------------ maps
static bool isWallChar(char c) { return c == '#' || c == 'T' || c == 'S' || c == 'E'; }

static void checkMaps() {
  for (int li = 0; li < NUM_LEVELS; li++) {
    const LevelDef& L = LEVELS[li];
    int w = L.w, len = (int)strlen(L.rows), h = len / w;
    CHECK(len % w == 0, "%s: map length %d is not a multiple of width %d", L.id, len, w);
    CHECK(w <= MAX_MAP && h <= MAX_MAP, "%s: map too big", L.id);
    auto at = [&](int x, int y) -> char { return (x < 0 || y < 0 || x >= w || y >= h) ? '#' : L.rows[y * w + x]; };
    int starts = 0, exits = 0, keys = 0, locks = 0, sx = 0, sy = 0;
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        char c = at(x, y);
        CHECK(strchr("#TSDLE.PzidBbhaAsk", c) != 0, "%s: unknown map char '%c' at %d,%d", L.id, c, x, y);
        bool border = x == 0 || y == 0 || x == w - 1 || y == h - 1;
        if (border) CHECK(isWallChar(c), "%s: border open at %d,%d", L.id, x, y);
        if (c == 'P') { starts++; sx = x; sy = y; }
        if (c == 'k') keys++;
        if (c == 'L') locks++;
        if (c == 'E') {
          exits++;
          bool reachable = false;
          const int DX[4] = { 1, -1, 0, 0 }, DY[4] = { 0, 0, 1, -1 };
          for (int k = 0; k < 4; k++) if (!isWallChar(at(x + DX[k], y + DY[k])) && at(x + DX[k], y + DY[k]) != 'D') reachable = true;
          CHECK(reachable, "%s: exit at %d,%d has no floor next to it", L.id, x, y);
        }
        if (c == 'D' || c == 'L') {
          bool wallsX = isWallChar(at(x - 1, y)) && isWallChar(at(x + 1, y)) && !isWallChar(at(x, y - 1)) && !isWallChar(at(x, y + 1));
          bool wallsY = isWallChar(at(x, y - 1)) && isWallChar(at(x, y + 1)) && !isWallChar(at(x - 1, y)) && !isWallChar(at(x + 1, y));
          CHECK(wallsX || wallsY, "%s: door at %d,%d must sit between two walls with floor on both sides", L.id, x, y);
        }
      }
    }
    CHECK(starts == 1, "%s: %d player starts", L.id, starts);
    CHECK(exits >= 1, "%s: no exit", L.id);
    CHECK(locks == 0 || keys >= 1, "%s: red door without a red key", L.id);

    // Flood fill from the start, first without opening red doors, then with.
    std::vector<int> seen(w * h, 0);
    auto fill = [&](bool withKey) {
      std::fill(seen.begin(), seen.end(), 0);
      std::vector<int> q(1, sy * w + sx);
      seen[sy * w + sx] = 1;
      for (size_t i = 0; i < q.size(); i++) {
        int x = q[i] % w, y = q[i] / w;
        const int DX[4] = { 1, -1, 0, 0 }, DY[4] = { 0, 0, 1, -1 };
        for (int k = 0; k < 4; k++) {
          int nx = x + DX[k], ny = y + DY[k];
          char c = at(nx, ny);
          if (isWallChar(c) || (c == 'L' && !withKey) || seen[ny * w + nx]) continue;
          seen[ny * w + nx] = 1;
          q.push_back(ny * w + nx);
        }
      }
    };
    fill(false);
    for (int i = 0; i < w * h; i++) if (L.rows[i] == 'k') CHECK(seen[i], "%s: red key is behind its own door", L.id);
    fill(true);
    for (int i = 0; i < w * h; i++) {
      char c = L.rows[i];
      if (c != '#' && c != 'T' && c != 'S' && c != 'E') CHECK(seen[i], "%s: '%c' at %d,%d can't be reached", L.id, c, i % w, i / w);
    }
  }
}

// ------------------------------------------------------------------------------------ invariants
static bool cellBlocks(int x, int y) {
  if (x < 0 || y < 0 || x >= G.mapW || y >= G.mapH) return true;
  int8_t d = G.doorAt[y][x];
  if (d >= 0) return G.doors[d].open < 0.9f;
  return G.tiles[y][x] != 0;
}

static bool overlapsWall(float x, float y, float r) {
  for (int cy = (int)floorf(y - r); cy <= (int)floorf(y + r); cy++)
    for (int cx = (int)floorf(x - r); cx <= (int)floorf(x + r); cx++)
      if (cellBlocks(cx, cy)) {
        // Doors may close only on empty cells, but a monster may stand in a door that is open.
        int8_t d = (cx >= 0 && cy >= 0 && cx < G.mapW && cy < G.mapH) ? G.doorAt[cy][cx] : -1;
        if (d >= 0 && G.doors[d].state != 0) continue;
        return true;
      }
  return false;
}

static const char* invariantError() {
  const Player& p = G.p;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.angle)) return "player position not finite";
  if ((G.state == ST_PLAY || G.state == ST_DEAD) && overlapsWall(p.x, p.y, 0.2f)) return "player inside a wall";
  if (p.hp < 0 || p.hp > 100) return "health out of range";
  if (p.ammo < 0 || p.ammo > 200) return "ammo out of range";
  for (int i = 0; i < MAX_ENTS; i++) {
    const Ent& e = G.ents[i];
    if (e.type == ENT_NONE) continue;
    if (!std::isfinite(e.x) || !std::isfinite(e.y)) return "entity position not finite";
    bool alive = e.type >= ENT_ZOMBIE && e.type <= ENT_BARON && e.state != MS_DYING && e.state != MS_DEAD;
    if (alive && overlapsWall(e.x, e.y, 0.2f)) return "monster inside a wall";
  }
  if (G.kills > G.totalMonsters) return "more kills than monsters";
  return 0;
}

// ------------------------------------------------------------------------------------ bot
static float wrapAngle(float a) {
  while (a > 3.14159265f) a -= 6.2831853f;
  while (a < -3.14159265f) a += 6.2831853f;
  return a;
}

static bool sightClear(float x0, float y0, float x1, float y1) {
  float dx = x1 - x0, dy = y1 - y0;
  int n = (int)(sqrtf(dx * dx + dy * dy) / 0.05f) + 1;
  for (int i = 1; i < n; i++) {
    float t = (float)i / n;
    int cx = (int)floorf(x0 + dx * t), cy = (int)floorf(y0 + dy * t);
    if (cx < 0 || cy < 0 || cx >= G.mapW || cy >= G.mapH) return false;
    int8_t d = G.doorAt[cy][cx];
    if (d >= 0 ? G.doors[d].open < 0.95f : G.tiles[cy][cx] != 0) return false;
  }
  return true;
}

static bool barrelIn(int x, int y) {
  for (int i = 0; i < MAX_ENTS; i++)
    if (G.ents[i].type == ENT_BARREL && G.ents[i].state == 0 && (int)G.ents[i].x == x && (int)G.ents[i].y == y) return true;
  return false;
}

static bool avoidBarrels = true;

static bool passable(int x, int y) {
  if (x < 0 || y < 0 || x >= G.mapW || y >= G.mapH) return false;
  int8_t d = G.doorAt[y][x];
  if (d >= 0) return !G.doors[d].locked || G.p.hasKey;
  if (avoidBarrels && barrelIn(x, y)) return false;
  return G.tiles[y][x] == 0;
}

// BFS path from the player's cell; returns the first cell to walk to (or -1). Walks around
// barrels when it can.
static int pathStepOnce(int gx, int gy, int* lenOut);
static int pathStep(int gx, int gy, int* lenOut = 0) {
  avoidBarrels = true;
  int c = pathStepOnce(gx, gy, lenOut);
  if (c < 0) { avoidBarrels = false; c = pathStepOnce(gx, gy, lenOut); avoidBarrels = true; }
  return c;
}

static int pathStepOnce(int gx, int gy, int* lenOut) {
  int px = (int)G.p.x, py = (int)G.p.y;
  static int prev[MAX_MAP * MAX_MAP];
  for (int i = 0; i < MAX_MAP * MAX_MAP; i++) prev[i] = -2;
  std::vector<int> q(1, py * MAX_MAP + px);
  prev[py * MAX_MAP + px] = -1;
  for (size_t i = 0; i < q.size(); i++) {
    int c = q[i], x = c % MAX_MAP, y = c / MAX_MAP;
    if (x == gx && y == gy) {
      int len = 0;
      while (prev[c] >= 0 && prev[c] != py * MAX_MAP + px) { c = prev[c]; len++; }
      if (lenOut) *lenOut = len;
      return c;
    }
    const int DX[4] = { 1, -1, 0, 0 }, DY[4] = { 0, 0, 1, -1 };
    for (int k = 0; k < 4; k++) {
      int nx = x + DX[k], ny = y + DY[k];
      if (!passable(nx, ny) || prev[ny * MAX_MAP + nx] != -2) continue;
      prev[ny * MAX_MAP + nx] = c;
      q.push_back(ny * MAX_MAP + nx);
    }
  }
  return -1;
}

struct Bot {
  int target = -1;
  float targetTime = 0;
  float ignoreUntil[MAX_ENTS];
  float lastX = 0, lastY = 0, stuckTime = 0, wiggle = 0, clock = 0;
  bool fireToggle = false;
  Bot() { for (int i = 0; i < MAX_ENTS; i++) ignoreUntil[i] = 0; }

  GameInput think() {
    GameInput in = { 0, 0, false, false };
    clock += DT;
    if (G.state != ST_PLAY) {   // menus: tap fire
      fireToggle = !fireToggle;
      in.fire = fireToggle;
      return in;
    }
    const Player& p = G.p;

    // 1. Fight the closest monster we can see.
    int best = -1;
    float bestD = 1e9f;
    for (int i = 0; i < MAX_ENTS; i++) {
      const Ent& e = G.ents[i];
      bool alive = e.type >= ENT_ZOMBIE && e.type <= ENT_BARON && e.state != MS_DYING && e.state != MS_DEAD;
      if (!alive || ignoreUntil[i] > clock) continue;
      float d = hypotf(e.x - p.x, e.y - p.y);
      if (d < 12 && d < bestD && sightClear(p.x, p.y, e.x, e.y)) { best = i; bestD = d; }
    }
    if (best != target) { target = best; targetTime = 0; }
    if (best >= 0) {
      targetTime += DT;
      if (targetTime > 8) { ignoreUntil[best] = clock + 4; target = -1; }
      const Ent& e = G.ents[best];
      float diff = wrapAngle(atan2f(e.y - p.y, e.x - p.x) - p.angle);
      in.turn = fmaxf(-1, fminf(1, diff * 4));
      in.fire = fabsf(diff) < 0.07f;
      if (p.weapon == 0) in.move = fabsf(diff) < 0.3f ? 1 : 0;   // fists: close in
      return in;
    }

    // 2. Low on health or ammo: fetch the nearest useful pickup. Otherwise walk to the nearest
    //    monster we can reach, then the key, then the exit.
    int gx = -1, gy = -1, bestLen = 1 << 30;
    bool needHealth = p.hp < 60, needAmmo = p.ammo < 25;
    for (int i = 0; i < MAX_ENTS; i++) {
      const Ent& e = G.ents[i];
      bool want = (e.type == ENT_MEDKIT && needHealth) || ((e.type == ENT_CLIP || e.type == ENT_AMMOBOX || e.type == ENT_SHOTGUN) && needAmmo);
      int len;
      if (want && pathStep((int)e.x, (int)e.y, &len) >= 0 && len < bestLen) { bestLen = len; gx = (int)e.x; gy = (int)e.y; }
    }
    if (gx < 0) {
      for (int i = 0; i < MAX_ENTS; i++) {
        const Ent& e = G.ents[i];
        bool alive = e.type >= ENT_ZOMBIE && e.type <= ENT_BARON && e.state != MS_DYING && e.state != MS_DEAD;
        if (!alive || ignoreUntil[i] > clock) continue;
        int len;
        if (pathStep((int)e.x, (int)e.y, &len) >= 0 && len < bestLen) { bestLen = len; gx = (int)e.x; gy = (int)e.y; }
      }
    }
    bool pushExit = false;
    float exitX = 0, exitY = 0;
    if (gx < 0) {
      for (int i = 0; i < MAX_ENTS; i++)
        if (G.ents[i].type == ENT_KEY && !p.hasKey) { gx = (int)G.ents[i].x; gy = (int)G.ents[i].y; }
    }
    if (gx < 0) {
      for (int y = 0; y < G.mapH && gx < 0; y++)
        for (int x = 0; x < G.mapW && gx < 0; x++) {
          if (G.tiles[y][x] != 6) continue;   // T_EXIT
          const int DX[4] = { 1, -1, 0, 0 }, DY[4] = { 0, 0, 1, -1 };
          for (int k = 0; k < 4; k++)
            if (passable(x + DX[k], y + DY[k]) && G.doorAt[y + DY[k]][x + DX[k]] < 0) {
              gx = x + DX[k]; gy = y + DY[k]; exitX = x + 0.5f; exitY = y + 0.5f; pushExit = true; break;
            }
        }
    }
    if (gx < 0) return in;

    float tx, ty;
    if ((int)p.x == gx && (int)p.y == gy) {
      if (!pushExit) return in;
      tx = exitX; ty = exitY;   // in front of the exit: walk into it
    } else {
      int c = pathStep(gx, gy);
      if (c < 0) return in;
      tx = c % MAX_MAP + 0.5f; ty = c / MAX_MAP + 0.5f;
      if (barrelIn(c % MAX_MAP, c / MAX_MAP)) {   // a barrel in the only way through: shoot it
        float diff = wrapAngle(atan2f(ty - p.y, tx - p.x) - p.angle);
        in.turn = fmaxf(-1, fminf(1, diff * 4));
        in.fire = fabsf(diff) < 0.1f && !fireToggle;
        fireToggle = in.fire;
        return in;
      }
    }
    float diff = wrapAngle(atan2f(ty - p.y, tx - p.x) - p.angle);
    in.turn = fmaxf(-1, fminf(1, diff * 3));
    in.move = fabsf(diff) < 0.5f ? 1 : 0;

    // Unstick: if we haven't moved for a while, back up and turn.
    float moved = hypotf(p.x - lastX, p.y - lastY);
    if (moved > 0.4f) { lastX = p.x; lastY = p.y; stuckTime = 0; }
    else stuckTime += DT;
    if (stuckTime > 3) { wiggle = 0.6f; stuckTime = 0; }
    if (wiggle > 0) { wiggle -= DT; in.move = -1; in.turn = 1; }
    return in;
  }
};

// Plays until the victory screen. Returns false if it runs out of time.
static bool playThrough(bool god, int* deaths, float limitSeconds, const char* tag) {
  gameInit(7);
  Bot bot;
  int frames = (int)(limitSeconds / DT);
  int lastLevel = -1;
  bool wasDead = false;
  GameStateId prevState = G.state;
  for (int f = 0; f < frames; f++) {
    G.godMode = god;
    frame(bot.think());
    const char* err = invariantError();
    if (err) { CHECK(false, "%s: %s (level %d, frame %d)", tag, err, G.level, f); return false; }
    if (G.state == ST_PLAY && G.level != lastLevel) {
      lastLevel = G.level;
      char n[64];
      snprintf(n, sizeof(n), "%s_level%d_start", tag, G.level + 1);
      saveShot(n);
    }
    bool dead = G.state == ST_DEAD;
    if (dead && !wasDead) deaths[G.level]++;
    wasDead = dead;
    bool justFinished = G.state == ST_LEVEL_DONE && prevState != ST_LEVEL_DONE;
    prevState = G.state;
    if (justFinished) {
      printf("  %s: %s done at %.0fs  kills %d/%d  items %d/%d  hp %d  ammo %d\n", tag, LEVELS[G.level].id,
             f * DT, G.kills, G.totalMonsters, G.items, G.totalItems, G.p.hp, G.p.ammo);
      if (god) CHECK(G.kills == G.totalMonsters, "%s: %s finished with %d/%d kills", tag, LEVELS[G.level].id, G.kills, G.totalMonsters);
      char n[64];
      snprintf(n, sizeof(n), "%s_level%d_done", tag, G.level + 1);
      saveShot(n);
    }
    if (G.state == ST_VICTORY) {
      saveShot("victory");
      return true;
    }
  }
  return false;
}

// ------------------------------------------------------------------------------------ tests
static void testGodRun() {
  printf("God-mode bot run:\n");
  int deaths[NUM_LEVELS] = { 0 };
  bool won = playThrough(true, deaths, 30 * 60, "god");
  CHECK(won, "god-mode bot did not reach the victory screen (stuck on %s)", LEVELS[G.level].id);
}

static void testDeathAndRestart() {
  gameInit(3);
  GameInput none = { 0, 0, false, false }, fire = { 0, 0, true, false };
  for (int i = 0; i < 10; i++) frame(none);
  frame(fire);
  frame(none);
  CHECK(G.state == ST_PLAY && G.level == 0, "fire on the title screen should start E1M1");
  saveShot("title_to_play");
  // Walk into the big hall and stand there.
  G.p.x = 7.5f; G.p.y = 2.5f; G.p.angle = 0;
  int f = 0;
  while (G.state == ST_PLAY && f < 60 * 30) { frame(none); f++; }
  CHECK(G.state == ST_DEAD, "standing still among monsters for a minute should kill you");
  CHECK(G.p.hp == 0, "dead player should have 0 health");
  frame(fire);   // too soon after dying: ignored so you can't skip the death screen by accident
  frame(none);
  CHECK(G.state == ST_DEAD, "FIRE right after dying should be ignored");
  for (int i = 0; i < 40; i++) frame(none);
  saveShot("dead");
  frame(fire);
  CHECK(G.state == ST_PLAY, "FIRE after dying should restart the level");
  CHECK(G.level == 0 && G.p.hp == 100 && G.kills == 0, "restart should reset health and the level");
  int monsters = 0;
  for (int i = 0; i < MAX_ENTS; i++) if (G.ents[i].type >= ENT_ZOMBIE && G.ents[i].type <= ENT_BARON && G.ents[i].state == MS_IDLE) monsters++;
  CHECK(monsters == G.totalMonsters, "restart should bring every monster back (%d of %d)", monsters, G.totalMonsters);
  CHECK(fabsf(G.p.x - 2.5f) < 0.01f && fabsf(G.p.y - 2.5f) < 0.01f, "restart should put you back at the start");
}

static void testFireHeldThroughScreens() {
  // Holding fire through the title must not instantly shoot, and the death screen ignores a
  // fire button that was already held when you died.
  gameInit(5);
  GameInput none = { 0, 0, false, false }, fire = { 0, 0, true, false };
  for (int i = 0; i < 10; i++) frame(none);
  frame(fire);
  CHECK(G.state == ST_PLAY, "title -> play");
  int ammo = G.p.ammo;
  for (int i = 0; i < 10; i++) frame(fire);
  CHECK(G.p.ammo == ammo, "fire held from the title screen should not shoot until released");
  frame(none);
  frame(fire);
  CHECK(G.p.ammo == ammo - 1, "a fresh press should shoot");
}

// Picks a map on the title screen with the stick, like a player would, and starts it.
static void startFromTitle(int level, uint32_t seed) {
  gameInit(seed);
  GameInput none = { 0, 0, false, false }, right = { 1, 0, false, false }, fire = { 0, 0, true, false };
  for (int i = 0; i < 10; i++) frame(none);
  for (int k = 0; k < level; k++) { frame(right); frame(none); }
  frame(fire);
  frame(none);
}

static void testQuitToTitle() {
  // Holding the map button and pressing fire leaves the level without shooting, back to the
  // title with the same map selected, so fire again restarts it.
  startFromTitle(1, 21);
  CHECK(G.state == ST_PLAY && G.level == 1, "should be playing E1M2");
  GameInput none = { 0, 0, false, false }, map = { 0, 0, false, true }, mapFire = { 0, 0, true, true }, fire = { 0, 0, true, false };
  for (int i = 0; i < 30; i++) frame(none);
  for (int i = 0; i < 30; i++) frame(fire);
  int ammo = G.p.ammo;
  CHECK(ammo < 60, "holding fire should shoot");
  for (int i = 0; i < 30; i++) frame(mapFire);   // map opened while already firing
  CHECK(G.state == ST_PLAY && G.p.ammo == ammo, "no shooting (and no quitting) while the map is up");
  frame(map);
  saveShot("automap");
  frame(mapFire);
  CHECK(G.state == ST_TITLE && G.selectLevel == 1, "map + fire should quit to the title with E1M2 selected");
  for (int i = 0; i < 10; i++) frame(none);
  frame(fire);
  CHECK(G.state == ST_PLAY && G.level == 1 && G.kills == 0, "fire on the title should restart E1M2");
}

static void testSoak() {
  printf("Random input soak:\n");
  uint32_t r = 99;
  auto rnd = [&]() { r ^= r << 13; r ^= r >> 17; r ^= r << 5; return r; };
  for (int li = 0; li < NUM_LEVELS; li++) {
    for (int god = 0; god < 2; god++) {
      startFromTitle(li, 11 + li);
      CHECK(G.state == ST_PLAY && G.level == li, "title screen should start map %d (got state %d map %d)", li + 1, G.state, G.level);
      GameInput in = { 0, 0, false, false };
      int deaths = 0, restarts = 0;
      for (int f = 0; f < 30 * 60 * 4; f++) {
        if (f % 15 == 0) {
          in.turn = (int)(rnd() % 201 - 100) / 100.0f;
          in.move = (int)(rnd() % 201 - 100) / 100.0f;
          in.fire = rnd() % 3 == 0;
          in.map = rnd() % 25 == 0;
        }
        G.godMode = god;
        bool wasDead = G.state == ST_DEAD;
        frame(in);
        if (G.state == ST_DEAD && !wasDead) deaths++;
        if (wasDead && G.state == ST_PLAY) restarts++;
        const char* err = invariantError();
        if (err) { CHECK(false, "soak map %d god %d: %s at frame %d", li + 1, god, err, f); break; }
      }
      printf("  %s %s: %d deaths, %d restarts\n", LEVELS[li].id, god ? "god " : "real", deaths, restarts);
    }
  }
}

static void testFairRun() {
  printf("Bot run without god mode:\n");
  int deaths[NUM_LEVELS] = { 0 };
  bool won = playThrough(false, deaths, 60 * 60, "real");
  printf("  deaths per level:");
  for (int i = 0; i < NUM_LEVELS; i++) printf(" %s=%d", LEVELS[i].id, deaths[i]);
  printf("\n");
  CHECK(won, "the bot could not finish the game without god mode (stuck on %s)", LEVELS[G.level].id);
}

int main(int argc, char** argv) {
  if (argc > 1) shotDir = argv[1];
  checkMaps();
  testFireHeldThroughScreens();
  testDeathAndRestart();
  testQuitToTitle();
  testGodRun();
  testSoak();
  testFairRun();
  if (failures) { printf("%d FAILURES\n", failures); return 1; }
  printf("ALL PASS\n");
  return 0;
}
