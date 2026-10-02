// OLED DOOM — engine: raycaster, sprites, monsters, weapons, doors, HUD and screens.
// Plain C++ (no Arduino calls) so tests/doom_sim.cpp can run the exact same code on a PC.

#include "game.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "art.h"
#include "levels.h"

Game G;

// ============================================================================ tuning
static const int VIEW_H = 56;              // 3D view rows; the HUD uses the 8 rows below
static const int HORIZON = VIEW_H / 2;
static const float PLANE = 0.72f;          // camera plane half-width (about 72 degree view)
static const float VSCALE = 64.0f;         // wall height in pixels at distance 1
static const float PLAYER_R = 0.24f;
static const float TURN_RATE = 2.7f;       // rad/s at full stick
static const float MOVE_SPEED = 3.2f;      // cells/s at full stick
static const int MAX_HP = 100;
static const int MAX_AMMO = 200;
static const int START_AMMO = 40;
static const float DOOR_SPEED = 2.2f;      // fraction of the door per second
static const float DOOR_WAIT = 4.0f;       // seconds a door stays open

static Player& P = G.p;

// ============================================================================ small helpers
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static int imin(int a, int b) { return a < b ? a : b; }
static int imax(int a, int b) { return a > b ? a : b; }

static uint32_t rnd() {
  uint32_t x = G.rng;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  G.rng = x;
  return x;
}
static float frand() { return (rnd() >> 8) * (1.0f / 16777216.0f); }
static int irand(int lo, int hi) { return lo + (int)(rnd() % (uint32_t)(hi - lo + 1)); }

static void snd(SoundId s) { if (s > G.sound) G.sound = s; }

static void message(const char* m) {
  strncpy(G.msg, m, sizeof(G.msg) - 1);
  G.msg[sizeof(G.msg) - 1] = 0;
  G.msgTimer = 2.5f;
}

// ============================================================================ bitmaps
struct Bitmap {
  uint8_t w, h;
  uint32_t on[32];     // white pixels
  uint32_t solid[32];  // white + black pixels, plus the outline ring
};

enum BmpId {
  B_ZOMBIE_W1, B_ZOMBIE_W2, B_ZOMBIE_ATK, B_IMP_W1, B_IMP_W2, B_IMP_ATK,
  B_DEMON_W1, B_DEMON_W2, B_DEMON_ATK, B_BARON_W1, B_BARON_W2, B_BARON_ATK,
  B_DIE, B_CORPSE, B_BARREL, B_MEDKIT, B_CLIP, B_AMMOBOX, B_SHOTGUN_PICK, B_KEY,
  B_FIREBALL1, B_FIREBALL2, B_PLASMA, B_PUFF, B_BLAST1, B_BLAST2,
  B_PISTOL, B_SHOTGUN, B_FIST, B_FLASH,
  B_ICON_HEALTH, B_ICON_AMMO, B_ICON_SKULL, B_ICON_KEY, B_FACE_OK, B_FACE_HURT, B_FACE_DEAD,
  B_COUNT
};
static const int FIRST_ICON = B_ICON_HEALTH;   // icons get no outline

static const ArtDef ART[B_COUNT] = {
  { SPR_ZOMBIE_W1, 16 }, { SPR_ZOMBIE_W2, 16 }, { SPR_ZOMBIE_ATK, 16 },
  { SPR_IMP_W1, 16 }, { SPR_IMP_W2, 16 }, { SPR_IMP_ATK, 16 },
  { SPR_DEMON_W1, 16 }, { SPR_DEMON_W2, 16 }, { SPR_DEMON_ATK, 16 },
  { SPR_BARON_W1, 16 }, { SPR_BARON_W2, 16 }, { SPR_BARON_ATK, 16 },
  { SPR_DIE, 16 }, { SPR_CORPSE, 16 }, { SPR_BARREL, 10 }, { SPR_MEDKIT, 12 }, { SPR_CLIP, 8 },
  { SPR_AMMOBOX, 12 }, { SPR_SHOTGUN_PICK, 16 }, { SPR_KEY, 10 },
  { SPR_FIREBALL1, 8 }, { SPR_FIREBALL2, 8 }, { SPR_PLASMA, 8 }, { SPR_PUFF, 7 },
  { SPR_BLAST1, 16 }, { SPR_BLAST2, 16 },
  { WPN_PISTOL, 20 }, { WPN_SHOTGUN, 26 }, { WPN_FIST, 24 }, { WPN_FLASH, 14 },
  { ICON_HEALTH, 7 }, { ICON_AMMO, 3 }, { ICON_SKULL, 7 }, { ICON_KEY, 9 },
  { FACE_OK, 7 }, { FACE_HURT, 7 }, { FACE_DEAD, 7 },
};

static Bitmap bmp[B_COUNT];

enum TexId { TX_BRICK, TX_TECH, TX_STONE, TX_DOOR, TX_LOCKED, TX_EXIT, TX_COUNT };
static const char* const TEX_SRC[TX_COUNT] = { TEX_BRICK, TEX_TECH, TEX_STONE, TEX_DOOR, TEX_LOCKED, TEX_EXIT };
static uint16_t tex[TX_COUNT][16];

static void buildBitmap(Bitmap& b, const ArtDef& a, bool outline) {
  memset(&b, 0, sizeof(b));
  b.w = a.w;
  b.h = (uint8_t)(strlen(a.px) / a.w);
  if (b.h > 32) b.h = 32;
  for (int y = 0; y < b.h; y++) {
    for (int x = 0; x < b.w; x++) {
      char c = a.px[y * b.w + x];
      if (c == '#') { b.on[y] |= 1u << x; b.solid[y] |= 1u << x; }
      else if (c == '+') b.solid[y] |= 1u << x;
    }
  }
  if (!outline) return;
  uint32_t full = b.w >= 32 ? 0xFFFFFFFFu : ((1u << b.w) - 1);
  uint32_t grown[32];
  for (int y = 0; y < b.h; y++) {
    uint32_t m = b.solid[y];
    uint32_t g = m | (m << 1) | (m >> 1);
    if (y > 0) g |= b.solid[y - 1];
    if (y + 1 < b.h) g |= b.solid[y + 1];
    grown[y] = g & full;
  }
  for (int y = 0; y < b.h; y++) b.solid[y] = grown[y];
}

static void buildArt() {
  for (int i = 0; i < B_COUNT; i++) buildBitmap(bmp[i], ART[i], i < FIRST_ICON);
  for (int t = 0; t < TX_COUNT; t++)
    for (int y = 0; y < 16; y++) {
      uint16_t row = 0;
      for (int x = 0; x < 16; x++) if (TEX_SRC[t][y * 16 + x] == '#') row |= (uint16_t)(1u << x);
      tex[t][y] = row;
    }
}

// ============================================================================ frame buffer
static uint8_t* FB;

static inline void pset(int x, int y) { FB[x + (y >> 3) * SCREEN_W] |= (uint8_t)(1u << (y & 7)); }
static inline void pclr(int x, int y) { FB[x + (y >> 3) * SCREEN_W] &= (uint8_t)~(1u << (y & 7)); }
static inline void pput(int x, int y, bool on) { if (on) pset(x, y); else pclr(x, y); }
static inline void pxor(int x, int y) { FB[x + (y >> 3) * SCREEN_W] ^= (uint8_t)(1u << (y & 7)); }
static inline bool onScreen(int x, int y) { return x >= 0 && y >= 0 && x < SCREEN_W && y < SCREEN_H; }

static const uint8_t BAYER[4][4] = { { 0, 8, 2, 10 }, { 12, 4, 14, 6 }, { 3, 11, 1, 9 }, { 15, 7, 13, 5 } };
// level 0 = black .. 1 = white, as an ordered-dither pattern that reads as grey on the OLED.
static inline bool dither(int x, int y, float level) { return level * 16.0f > BAYER[y & 3][x & 3]; }

static void fillRect(int x, int y, int w, int h, bool on) {
  for (int yy = imax(y, 0); yy < imin(y + h, SCREEN_H); yy++)
    for (int xx = imax(x, 0); xx < imin(x + w, SCREEN_W); xx++) pput(xx, yy, on);
}

static void hline(int x0, int x1, int y) { for (int x = imax(x0, 0); x <= imin(x1, SCREEN_W - 1); x++) if (onScreen(x, y)) pset(x, y); }
static void vline(int x, int y0, int y1) { for (int y = imax(y0, 0); y <= imin(y1, SCREEN_H - 1); y++) if (onScreen(x, y)) pset(x, y); }

static void rect(int x, int y, int w, int h) {
  hline(x, x + w - 1, y); hline(x, x + w - 1, y + h - 1);
  vline(x, y, y + h - 1); vline(x + w - 1, y, y + h - 1);
}

// Unscaled blit. Sprites (with outline) draw black where solid and white where on.
static void blit(const Bitmap& b, int x0, int y0, int clipBottom = SCREEN_H) {
  for (int y = 0; y < b.h; y++) {
    int sy = y0 + y;
    if (sy < 0 || sy >= clipBottom) continue;
    for (int x = 0; x < b.w; x++) {
      int sx = x0 + x;
      if (sx < 0 || sx >= SCREEN_W) continue;
      if (b.on[y] >> x & 1) pset(sx, sy);
      else if (b.solid[y] >> x & 1) pclr(sx, sy);
    }
  }
}

// ============================================================================ text
static void drawChar(int x, int y, char c, int scale, bool white) {
  if (c >= 'a' && c <= 'z') c -= 32;
  if (c < ' ' || c > 'Z') c = '?';
  const uint8_t* g = &FONT5X7[(c - ' ') * 5];
  for (int col = 0; col < 5; col++)
    for (int row = 0; row < 8; row++)
      if (g[col] >> row & 1)
        for (int dy = 0; dy < scale; dy++)
          for (int dx = 0; dx < scale; dx++) {
            int px = x + col * scale + dx, py = y + row * scale + dy;
            if (onScreen(px, py)) pput(px, py, white);
          }
}

static int textWidth(const char* s, int scale) { int n = (int)strlen(s); return n ? n * 6 * scale - scale : 0; }

static void drawText(int x, int y, const char* s, int scale = 1, bool white = true) {
  for (; *s; s++, x += 6 * scale) drawChar(x, y, *s, scale, white);
}

static void drawTextCentered(int y, const char* s, int scale = 1) {
  drawText((SCREEN_W - textWidth(s, scale)) / 2, y, s, scale);
}

static void drawTextBox(int y, const char* s, int scale = 1) {
  int w = textWidth(s, scale);
  int x = (SCREEN_W - w) / 2;
  fillRect(x - 3, y - 2, w + 6, 7 * scale + 4, false);
  drawText(x, y, s, scale);
}

// ============================================================================ map
enum Tile : uint8_t { T_EMPTY, T_BRICK, T_TECH, T_STONE, T_DOOR, T_LOCKED, T_EXIT };

static inline uint8_t tileAt(int x, int y) {
  if (x < 0 || y < 0 || x >= G.mapW || y >= G.mapH) return T_BRICK;
  return G.tiles[y][x];
}
static inline bool isDoorTile(uint8_t t) { return t == T_DOOR || t == T_LOCKED; }

static bool blocksMove(int x, int y) {
  uint8_t t = tileAt(x, y);
  if (t == T_EMPTY) return false;
  if (isDoorTile(t)) return G.doors[G.doorAt[y][x]].open < 0.9f;
  return true;
}

static bool blocksSight(int x, int y) {
  uint8_t t = tileAt(x, y);
  if (t == T_EMPTY) return false;
  if (isDoorTile(t)) return G.doors[G.doorAt[y][x]].open < 0.6f;
  return true;
}

static bool hitsMap(float x, float y, float r) {
  int x0 = (int)floorf(x - r), x1 = (int)floorf(x + r);
  int y0 = (int)floorf(y - r), y1 = (int)floorf(y + r);
  for (int cy = y0; cy <= y1; cy++)
    for (int cx = x0; cx <= x1; cx++)
      if (blocksMove(cx, cy)) return true;
  return false;
}

static bool lineClear(float x0, float y0, float x1, float y1) {
  float dx = x1 - x0, dy = y1 - y0;
  int n = (int)(sqrtf(dx * dx + dy * dy) / 0.08f) + 1;
  for (int i = 1; i < n; i++) {
    float t = (float)i / n;
    if (blocksSight((int)floorf(x0 + dx * t), (int)floorf(y0 + dy * t))) return false;
  }
  return true;
}

struct RayHit { float dist, wallX; uint8_t tex, side; int16_t cx, cy; };

// DDA ray from (ox,oy) along (rdx,rdy). dist is in units of the ray vector, which for the
// camera rays is the perpendicular distance (no fisheye) and for unit vectors is plain distance.
static void castRay(float ox, float oy, float rdx, float rdy, RayHit& h) {
  int mx = (int)floorf(ox), my = (int)floorf(oy);
  float ddx = rdx == 0 ? 1e30f : fabsf(1.0f / rdx);
  float ddy = rdy == 0 ? 1e30f : fabsf(1.0f / rdy);
  int sx, sy;
  float sdx, sdy;
  if (rdx < 0) { sx = -1; sdx = (ox - mx) * ddx; } else { sx = 1; sdx = (mx + 1.0f - ox) * ddx; }
  if (rdy < 0) { sy = -1; sdy = (oy - my) * ddy; } else { sy = 1; sdy = (my + 1.0f - oy) * ddy; }

  for (int guard = 0; guard < 2 * MAX_MAP + 4; guard++) {
    float tEnter;
    uint8_t side;
    if (sdx < sdy) { tEnter = sdx; sdx += ddx; mx += sx; side = 0; }
    else           { tEnter = sdy; sdy += ddy; my += sy; side = 1; }
    uint8_t t = tileAt(mx, my);
    if (t == T_EMPTY) continue;
    h.cx = (int16_t)mx; h.cy = (int16_t)my;
    if (isDoorTile(t)) {
      // The door panel sits in the middle of its cell and slides sideways as it opens.
      const Door& d = G.doors[G.doorAt[my][mx]];
      float tExit = sdx < sdy ? sdx : sdy;
      float tp, u;
      if (d.planeX) { if (rdx == 0) continue; tp = (mx + 0.5f - ox) / rdx; u = oy + tp * rdy - my; }
      else          { if (rdy == 0) continue; tp = (my + 0.5f - oy) / rdy; u = ox + tp * rdx - mx; }
      if (tp < tEnter || tp > tExit || u < d.open) continue;
      h.dist = tp;
      h.wallX = clampf(u - d.open, 0, 0.999f);
      h.side = d.planeX ? 0 : 1;
      h.tex = t == T_DOOR ? TX_DOOR : TX_LOCKED;
      return;
    }
    h.dist = tEnter;
    float wx = side == 0 ? oy + tEnter * rdy : ox + tEnter * rdx;
    wx -= floorf(wx);
    // Keep textures reading left-to-right from every side (matters for the EXIT sign).
    if ((side == 0 && rdx < 0) || (side == 1 && rdy > 0)) wx = 1.0f - wx;
    h.wallX = clampf(wx, 0, 0.999f);
    h.side = side;
    h.tex = t == T_TECH ? TX_TECH : t == T_STONE ? TX_STONE : t == T_EXIT ? TX_EXIT : TX_BRICK;
    return;
  }
  h.dist = 64; h.wallX = 0; h.side = 0; h.tex = TX_BRICK; h.cx = h.cy = -1;
}

// ============================================================================ entities
enum AttackKind : uint8_t { ATK_HITSCAN, ATK_FIREBALL, ATK_MELEE, ATK_PLASMA };

struct MonInfo {
  int16_t hp;
  float speed, radius, height, range, windup, cdMin, cdMax;
  uint8_t attack;
  int16_t dmgMin, dmgMax;
  float painChance;
  uint8_t walk1, walk2, atk;
};

static const MonInfo MON[4] = {
  // hp   speed  radius height range windup cdMin cdMax  attack        dmg     pain
  {  20, 1.3f, 0.30f, 0.72f, 10.0f, 0.50f, 1.4f, 2.6f, ATK_HITSCAN,  4, 10, 0.80f, B_ZOMBIE_W1, B_ZOMBIE_W2, B_ZOMBIE_ATK },
  {  50, 1.5f, 0.30f, 0.78f, 11.0f, 0.55f, 1.6f, 3.0f, ATK_FIREBALL, 8, 14, 0.65f, B_IMP_W1, B_IMP_W2, B_IMP_ATK },
  {  90, 2.7f, 0.36f, 0.72f,  1.2f, 0.30f, 0.7f, 1.1f, ATK_MELEE,    8, 16, 0.50f, B_DEMON_W1, B_DEMON_W2, B_DEMON_ATK },
  { 400, 1.3f, 0.40f, 1.05f, 13.0f, 0.65f, 1.2f, 2.2f, ATK_PLASMA,  16, 26, 0.12f, B_BARON_W1, B_BARON_W2, B_BARON_ATK },
};

static inline bool isMonster(uint8_t t) { return t >= ENT_ZOMBIE && t <= ENT_BARON; }
static inline bool isPickup(uint8_t t) { return t >= ENT_MEDKIT && t <= ENT_KEY; }
static inline const MonInfo& monInfo(const Ent& e) { return MON[e.type - ENT_ZOMBIE]; }
static inline bool monAlive(const Ent& e) { return isMonster(e.type) && e.state != MS_DYING && e.state != MS_DEAD; }

static float entRadius(const Ent& e) {
  if (isMonster(e.type)) return monInfo(e).radius;
  if (e.type == ENT_BARREL) return 0.26f;
  return 0.2f;
}
static bool entSolid(const Ent& e) {
  if (isMonster(e.type)) return monAlive(e);
  return e.type == ENT_BARREL && e.state == 0;
}

static Ent* spawn(uint8_t type, float x, float y) {
  for (int i = 0; i < MAX_ENTS; i++) {
    Ent& e = G.ents[i];
    if (e.type != ENT_NONE) continue;
    memset(&e, 0, sizeof(e));
    e.type = type; e.x = x; e.y = y;
    if (isMonster(type)) { e.hp = monInfo(e).hp; e.state = MS_IDLE; e.lookTimer = frand() * 0.3f; }
    if (type == ENT_BARREL) e.hp = 20;
    return &e;
  }
  return 0;   // full: effects are simply skipped
}

static void spawnPuff(float x, float y) {
  Ent* p = spawn(ENT_PUFF, x, y);
  if (p) p->timer = 0.18f;
}

// Moving into another solid thing is blocked, but moving apart is always allowed so nothing
// can get wedged.
static bool blockedByEnts(float ox, float oy, float nx, float ny, float r, int self) {
  for (int i = 0; i < MAX_ENTS; i++) {
    if (i == self) continue;
    const Ent& e = G.ents[i];
    if (!entSolid(e)) continue;
    float rr = r + entRadius(e);
    float dn = (nx - e.x) * (nx - e.x) + (ny - e.y) * (ny - e.y);
    if (dn >= rr * rr) continue;
    float d0 = (ox - e.x) * (ox - e.x) + (oy - e.y) * (oy - e.y);
    if (dn < d0) return true;
  }
  if (self >= 0 && G.state == ST_PLAY) {
    float rr = r + PLAYER_R;
    float dn = (nx - P.x) * (nx - P.x) + (ny - P.y) * (ny - P.y);
    float d0 = (ox - P.x) * (ox - P.x) + (oy - P.y) * (oy - P.y);
    if (dn < rr * rr && dn < d0) return true;
  }
  return false;
}

static void moveBody(float& x, float& y, float dx, float dy, float r, int self) {
  float nx = x + dx;
  if (!hitsMap(nx, y, r) && !blockedByEnts(x, y, nx, y, r, self)) x = nx;
  float ny = y + dy;
  if (!hitsMap(x, ny, r) && !blockedByEnts(x, y, x, ny, r, self)) y = ny;
}

// ============================================================================ doors
static bool cellOccupied(int cx, int cy) {
  float x0 = (float)cx, y0 = (float)cy, x1 = cx + 1.0f, y1 = cy + 1.0f;
  if (P.x + PLAYER_R > x0 && P.x - PLAYER_R < x1 && P.y + PLAYER_R > y0 && P.y - PLAYER_R < y1) return true;
  for (int i = 0; i < MAX_ENTS; i++) {
    const Ent& e = G.ents[i];
    if (!entSolid(e)) continue;
    float r = entRadius(e);
    if (e.x + r > x0 && e.x - r < x1 && e.y + r > y0 && e.y - r < y1) return true;
  }
  return false;
}

static void openDoor(int idx) {
  Door& d = G.doors[idx];
  if (d.state == 0 || d.state == 3) { d.state = 1; snd(SND_DOOR); }
  else if (d.state == 2) d.timer = DOOR_WAIT;
}

static void updateDoors(float dt) {
  for (int i = 0; i < G.numDoors; i++) {
    Door& d = G.doors[i];
    switch (d.state) {
      case 1:
        d.open += DOOR_SPEED * dt;
        if (d.open >= 1) { d.open = 1; d.state = 2; d.timer = DOOR_WAIT; }
        break;
      case 2:
        d.timer -= dt;
        if (d.timer <= 0) {
          if (cellOccupied(d.x, d.y)) d.timer = 0.5f;
          else d.state = 3;
        }
        break;
      case 3:
        if (cellOccupied(d.x, d.y)) { d.state = 1; break; }
        d.open -= DOOR_SPEED * dt;
        if (d.open <= 0) { d.open = 0; d.state = 0; }
        break;
    }
  }
}

// ============================================================================ flow field
// Grid distance from the player. Monsters that can't see you walk down it to find their way:
// closed doors count as open for them (they open them), red doors only once open. Gunfire is
// heard only through doors that are open.
static int flowCellX = -1, flowCellY = -1;

static void distanceFromPlayer(int16_t (*dist)[MAX_MAP], bool hearing) {
  static uint16_t queue[MAX_MAP * MAX_MAP];
  for (int y = 0; y < MAX_MAP; y++) for (int x = 0; x < MAX_MAP; x++) dist[y][x] = -1;
  int px = (int)P.x, py = (int)P.y;
  if (px < 0 || py < 0 || px >= G.mapW || py >= G.mapH) return;
  int head = 0, tail = 0;
  dist[py][px] = 0;
  queue[tail++] = (uint16_t)(py * MAX_MAP + px);
  static const int8_t DX[4] = { 1, -1, 0, 0 }, DY[4] = { 0, 0, 1, -1 };
  while (head < tail) {
    int c = queue[head++];
    int cx = c % MAX_MAP, cy = c / MAX_MAP;
    for (int k = 0; k < 4; k++) {
      int nx = cx + DX[k], ny = cy + DY[k];
      if (nx < 0 || ny < 0 || nx >= G.mapW || ny >= G.mapH || dist[ny][nx] >= 0) continue;
      uint8_t t = G.tiles[ny][nx];
      bool pass = t == T_EMPTY;
      if (isDoorTile(t)) pass = G.doors[G.doorAt[ny][nx]].open > 0.5f || (t == T_DOOR && !hearing);
      if (!pass) continue;
      dist[ny][nx] = (int16_t)(dist[cy][cx] + 1);
      queue[tail++] = (uint16_t)(ny * MAX_MAP + nx);
    }
  }
}

static void computeFlow() {
  distanceFromPlayer(G.flow, false);
  flowCellX = (int)P.x; flowCellY = (int)P.y;
}

static void wakeMonster(Ent& e) {
  if (e.state != MS_IDLE) return;
  e.state = MS_CHASE;
  e.cooldown = 0.5f + frand() * 1.0f;   // reaction time before the first shot
}

// Gunfire wakes every monster that can hear it.
static void noiseAlert() {
  static int16_t heard[MAX_MAP][MAX_MAP];
  distanceFromPlayer(heard, true);
  for (int i = 0; i < MAX_ENTS; i++) {
    Ent& e = G.ents[i];
    if (!monAlive(e) || e.state != MS_IDLE) continue;
    int f = heard[(int)e.y][(int)e.x];
    if (f >= 0 && f <= 12) wakeMonster(e);
  }
}

// ============================================================================ damage
static void damagePlayer(int dmg) {
  if (G.state != ST_PLAY) return;
  P.hurtFlash = 0.35f;
  if (G.godMode) return;
  P.hp -= dmg;
  if (P.hp <= 0) {
    P.hp = 0;
    G.state = ST_DEAD;
    G.stateTimer = 0;
    G.fireBlocked = true;
    snd(SND_PLAYER_DIE);
  } else {
    snd(SND_PLAYER_HURT);
  }
}

static void damageEnt(Ent& e, int dmg) {
  if (e.type == ENT_BARREL) {
    if (e.state != 0) return;
    e.hp -= dmg;
    if (e.hp <= 0) { e.state = 1; e.timer = 0.15f; }   // short fuse: chains look better
    return;
  }
  if (!monAlive(e)) return;
  e.hp -= dmg;
  e.hitFlash = 0.12f;
  if (e.hp <= 0) {
    e.state = MS_DYING;
    e.timer = 0.45f;
    G.kills++;
    snd(SND_MON_DIE);
    return;
  }
  snd(SND_MON_HIT);
  if (frand() < monInfo(e).painChance) {
    e.state = MS_PAIN;      // flinch: also cancels an attack that was winding up
    e.timer = 0.22f;
  } else if (e.state == MS_IDLE) {
    wakeMonster(e);
  }
}

static void explode(float x, float y) {
  Ent* b = spawn(ENT_BLAST, x, y);
  if (b) b->timer = 0.45f;
  snd(SND_EXPLODE);
  const float R = 2.6f;
  for (int i = 0; i < MAX_ENTS; i++) {
    Ent& e = G.ents[i];
    if (!entSolid(e)) continue;
    float d = sqrtf((e.x - x) * (e.x - x) + (e.y - y) * (e.y - y));
    if (d < R && lineClear(x, y, e.x, e.y)) damageEnt(e, (int)(90 * (1 - d / R)) + 1);
  }
  float d = sqrtf((P.x - x) * (P.x - x) + (P.y - y) * (P.y - y));
  if (d < R && lineClear(x, y, P.x, P.y)) damagePlayer((int)(70 * (1 - d / R)) + 1);
}

// ============================================================================ player weapons
// One hitscan line from the player. Returns true if it hit something shootable.
static bool shootLine(float angleOffset, int dmg, float maxRange) {
  float a = P.angle + angleOffset;
  float dx = cosf(a), dy = sinf(a);
  RayHit h;
  castRay(P.x, P.y, dx, dy, h);
  float bestT = h.dist < maxRange ? h.dist : maxRange;
  Ent* best = 0;
  for (int i = 0; i < MAX_ENTS; i++) {
    Ent& e = G.ents[i];
    if (!entSolid(e)) continue;
    float ex = e.x - P.x, ey = e.y - P.y;
    float t = ex * dx + ey * dy;
    if (t <= 0.05f || t >= bestT) continue;
    float perp = fabsf(ex * dy - ey * dx);
    // A little horizontal aim assist, more at range where a target is only a few pixels wide.
    if (perp < entRadius(e) + 0.08f + 0.015f * t) { best = &e; bestT = t; }
  }
  if (best) {
    damageEnt(*best, dmg);
    spawnPuff(P.x + dx * (bestT - 0.3f), P.y + dy * (bestT - 0.3f));
    return true;
  }
  if (h.dist < maxRange) spawnPuff(P.x + dx * (h.dist - 0.06f), P.y + dy * (h.dist - 0.06f));
  return false;
}

static void selectWeapon() {
  if (P.ammo > 0) P.weapon = P.hasShotgun ? 2 : 1;
  else P.weapon = 0;
}

static void playerFire() {
  selectWeapon();
  if (P.fireCooldown > 0) return;
  switch (P.weapon) {
    case 0:   // fist
      P.fireCooldown = 0.45f; P.fireAnim = 0.25f;
      shootLine(0, irand(10, 20), 1.15f);
      snd(SND_PUNCH);
      return;
    case 1:   // pistol
      P.fireCooldown = 0.36f; P.fireAnim = 0.16f; P.ammo--;
      shootLine((frand() - 0.5f) * 0.02f, irand(10, 15), 40);
      snd(SND_PISTOL);
      break;
    default:  // shotgun: 7 pellets
      P.fireCooldown = 0.85f; P.fireAnim = 0.22f; P.ammo--;
      for (int i = 0; i < 7; i++) shootLine((frand() - 0.5f) * 0.17f, irand(6, 10), 40);
      snd(SND_SHOTGUN);
      break;
  }
  noiseAlert();
}

// ============================================================================ level setup
static void startLevel(int n, bool newGame);

static void loadLevel(int n) {
  const LevelDef& L = LEVELS[n];
  G.level = (uint8_t)n;
  G.mapW = L.w;
  G.mapH = (uint8_t)(strlen(L.rows) / L.w);
  memset(G.tiles, 0, sizeof(G.tiles));
  memset(G.doorAt, -1, sizeof(G.doorAt));
  memset(G.seen, 0, sizeof(G.seen));
  memset(G.ents, 0, sizeof(G.ents));
  G.numDoors = 0;
  G.kills = G.totalMonsters = G.items = G.totalItems = 0;
  for (int y = 0; y < G.mapH; y++) {
    for (int x = 0; x < G.mapW; x++) {
      char c = L.rows[y * L.w + x];
      float fx = x + 0.5f, fy = y + 0.5f;
      uint8_t t = T_EMPTY;
      Ent* e = 0;
      switch (c) {
        case '#': t = T_BRICK; break;
        case 'T': t = T_TECH; break;
        case 'S': t = T_STONE; break;
        case 'E': t = T_EXIT; break;
        case 'D': case 'L':
          t = c == 'D' ? T_DOOR : T_LOCKED;
          if (G.numDoors < MAX_DOORS) {
            Door& d = G.doors[G.numDoors];
            memset(&d, 0, sizeof(d));
            d.x = (uint8_t)x; d.y = (uint8_t)y; d.locked = c == 'L';
            G.doorAt[y][x] = (int8_t)G.numDoors++;
          } else {
            t = T_BRICK;
          }
          break;
        case 'P': P.x = fx; P.y = fy; break;
        case 'z': e = spawn(ENT_ZOMBIE, fx, fy); break;
        case 'i': e = spawn(ENT_IMP, fx, fy); break;
        case 'd': e = spawn(ENT_DEMON, fx, fy); break;
        case 'B': e = spawn(ENT_BARON, fx, fy); break;
        case 'b': spawn(ENT_BARREL, fx, fy); break;
        case 'h': e = spawn(ENT_MEDKIT, fx, fy); break;
        case 'a': e = spawn(ENT_CLIP, fx, fy); break;
        case 'A': e = spawn(ENT_AMMOBOX, fx, fy); break;
        case 's': e = spawn(ENT_SHOTGUN, fx, fy); break;
        case 'k': e = spawn(ENT_KEY, fx, fy); break;
      }
      G.tiles[y][x] = t;
      if (e && isMonster(e->type)) G.totalMonsters++;
      if (e && isPickup(e->type)) G.totalItems++;
    }
  }
  // A door's panel runs across the corridor: if the cells left and right of it are open you walk
  // through it along x, so the panel lies on the line x = cell + 0.5.
  for (int i = 0; i < G.numDoors; i++) {
    Door& d = G.doors[i];
    d.planeX = tileAt(d.x - 1, d.y) == T_EMPTY || tileAt(d.x + 1, d.y) == T_EMPTY;
  }
  P.angle = L.angle;
  computeFlow();
}

static void updateCamera() {
  P.dirX = cosf(P.angle); P.dirY = sinf(P.angle);
  P.planeX = -P.dirY * PLANE; P.planeY = P.dirX * PLANE;
}

static void startLevel(int n, bool newGame) {
  if (newGame) {
    // Starting on a later map gives you the shotgun you would have found on the way.
    P.hp = MAX_HP; P.ammo = n > 0 ? 60 : START_AMMO; P.hasShotgun = n > 0;
    G.totalKills = G.totalMonstersAll = 0;
    G.totalTime = 0;
  }
  G.startAmmo = P.ammo;
  G.startShotgun = P.hasShotgun;
  loadLevel(n);
  P.hasKey = false;
  P.fireCooldown = P.fireAnim = P.hurtFlash = P.pickupFlash = 0;
  P.bob = P.bobAmount = 0;
  P.camH = 0.5f;
  selectWeapon();
  updateCamera();
  G.state = ST_PLAY;
  G.stateTimer = 0;
  G.levelTime = 0;
  G.msgTimer = 0;
  G.noKeyTimer = 0;
  G.fireBlocked = true;
}

static void restartLevel() {
  // Back to how you entered the level, but never with less than a fresh start's ammo.
  int minAmmo = G.level > 0 ? 60 : START_AMMO;
  P.hp = MAX_HP;
  P.ammo = (int16_t)imax(G.startAmmo, minAmmo);
  P.hasShotgun = G.startShotgun || G.level > 0;
  startLevel(G.level, false);
}

static void finishLevel() {
  G.state = ST_LEVEL_DONE;
  G.stateTimer = 0;
  G.fireBlocked = true;
  G.totalKills += G.kills;
  G.totalMonstersAll += G.totalMonsters;
  G.totalTime += G.levelTime;
  snd(SND_LEVEL_DONE);
}

// ============================================================================ player update
static void usePushing(bool pushing) {
  // Whatever is right in front of you: doors open by walking up to them, the exit switch by
  // walking into it.
  for (int k = 0; k < 2; k++) {
    float reach = k == 0 ? 0.55f : 0.95f;
    int cx = (int)floorf(P.x + P.dirX * reach), cy = (int)floorf(P.y + P.dirY * reach);
    uint8_t t = tileAt(cx, cy);
    if (t == T_DOOR) { openDoor(G.doorAt[cy][cx]); return; }
    if (t == T_LOCKED) {
      if (P.hasKey) openDoor(G.doorAt[cy][cx]);
      else if (G.noKeyTimer <= 0) { message("YOU NEED A RED KEY"); snd(SND_NO_KEY); G.noKeyTimer = 2.5f; }
      return;
    }
    if (t == T_EXIT) { if (pushing && k == 0) finishLevel(); return; }
    if (t != T_EMPTY) return;
  }
}

static void pickups() {
  for (int i = 0; i < MAX_ENTS; i++) {
    Ent& e = G.ents[i];
    if (!isPickup(e.type)) continue;
    float dx = e.x - P.x, dy = e.y - P.y;
    if (dx * dx + dy * dy > 0.55f * 0.55f) continue;
    bool took = true;
    switch (e.type) {
      case ENT_MEDKIT:
        if (P.hp >= MAX_HP) { took = false; break; }
        P.hp = (int16_t)imin(MAX_HP, P.hp + 25); message("PICKED UP A MEDIKIT"); break;
      case ENT_CLIP:
        if (P.ammo >= MAX_AMMO) { took = false; break; }
        P.ammo = (int16_t)imin(MAX_AMMO, P.ammo + 10); message("PICKED UP A CLIP"); break;
      case ENT_AMMOBOX:
        if (P.ammo >= MAX_AMMO) { took = false; break; }
        P.ammo = (int16_t)imin(MAX_AMMO, P.ammo + 30); message("PICKED UP AMMO BOX"); break;
      case ENT_SHOTGUN:
        message(P.hasShotgun ? "PICKED UP SHELLS" : "YOU GOT THE SHOTGUN!");
        P.hasShotgun = true;
        P.ammo = (int16_t)imin(MAX_AMMO, P.ammo + 8);
        break;
      case ENT_KEY:
        P.hasKey = true; message("PICKED UP RED KEYCARD"); break;
    }
    if (!took) continue;
    e.type = ENT_NONE;
    G.items++;
    P.pickupFlash = 0.2f;
    snd(SND_PICKUP);
  }
}

static void updatePlayer(const GameInput& in, float dt) {
  float turn = clampf(in.turn, -1, 1), move = clampf(in.move, -1, 1);
  P.angle += turn * TURN_RATE * dt;
  if (P.angle > 6.2831853f) P.angle -= 6.2831853f;
  if (P.angle < 0) P.angle += 6.2831853f;
  updateCamera();

  float step = move * MOVE_SPEED * dt;
  moveBody(P.x, P.y, P.dirX * step, P.dirY * step, PLAYER_R, -1);
  float moving = fabsf(move);
  P.bob += dt * 9.0f * moving;
  P.bobAmount += (moving - P.bobAmount) * clampf(dt * 8, 0, 1);

  usePushing(move > 0.3f);
  if (G.state != ST_PLAY) return;   // reached the exit
  pickups();

  if (P.fireCooldown > 0) P.fireCooldown -= dt;
  if (P.fireAnim > 0) P.fireAnim -= dt;
  if (in.fire && !in.map && !G.fireBlocked) playerFire();
  else selectWeapon();
}

// ============================================================================ monsters
static void monsterAttack(Ent& e, const MonInfo& m) {
  float dx = P.x - e.x, dy = P.y - e.y;
  float d = sqrtf(dx * dx + dy * dy);
  bool see = lineClear(e.x, e.y, P.x, P.y);
  e.shotFlash = 0.18f;
  if (d < 1.25f && m.attack != ATK_HITSCAN) {   // close enough to claw / bite
    if (see) damagePlayer(irand(m.dmgMin, m.dmgMax));
    snd(SND_PUNCH);
    return;
  }
  if (!see) return;
  switch (m.attack) {
    case ATK_HITSCAN: {
      float chance = clampf(0.95f - d * 0.06f, 0.35f, 0.9f);
      if (frand() < chance) damagePlayer(irand(m.dmgMin, m.dmgMax));
      snd(SND_MON_SHOT);
      break;
    }
    case ATK_FIREBALL:
    case ATK_PLASMA: {
      float nx = dx / d, ny = dy / d;
      float speed = m.attack == ATK_FIREBALL ? 5.0f : 6.0f;
      Ent* f = spawn(m.attack == ATK_FIREBALL ? ENT_FIREBALL : ENT_PLASMA, e.x + nx * 0.4f, e.y + ny * 0.4f);
      if (f) { f->vx = nx * speed; f->vy = ny * speed; f->timer = 6; f->hp = (int16_t)irand(m.dmgMin, m.dmgMax); }
      snd(SND_FIREBALL);
      break;
    }
    default: break;   // melee out of reach: missed
  }
}

static void updateMonster(int idx, float dt) {
  Ent& e = G.ents[idx];
  const MonInfo& m = monInfo(e);
  if (e.hitFlash > 0) e.hitFlash -= dt;
  if (e.shotFlash > 0) e.shotFlash -= dt;
  switch (e.state) {
    case MS_DEAD: return;
    case MS_DYING:
      e.timer -= dt;
      if (e.timer <= 0) e.state = MS_DEAD;
      return;
    case MS_PAIN:
      e.timer -= dt;
      if (e.timer <= 0) e.state = MS_CHASE;
      return;
    case MS_IDLE: {
      e.lookTimer -= dt;
      if (e.lookTimer > 0) return;
      e.lookTimer = 0.25f;
      float dx = P.x - e.x, dy = P.y - e.y;
      if (G.state == ST_PLAY && dx * dx + dy * dy < 16 * 16 && lineClear(e.x, e.y, P.x, P.y)) wakeMonster(e);
      return;
    }
    case MS_ATTACK:
      e.timer -= dt;
      if (e.timer > 0) return;
      if (G.state == ST_PLAY) monsterAttack(e, m);
      e.state = MS_CHASE;
      e.cooldown = m.cdMin + frand() * (m.cdMax - m.cdMin);
      return;
    default: break;
  }

  // ---- chase
  if (e.cooldown > 0) e.cooldown -= dt;
  float dx = P.x - e.x, dy = P.y - e.y;
  float d = sqrtf(dx * dx + dy * dy);
  bool see = G.state == ST_PLAY && lineClear(e.x, e.y, P.x, P.y);

  if (see && e.cooldown <= 0 && d <= m.range + PLAYER_R) {
    e.state = MS_ATTACK;
    e.timer = m.windup;
    return;
  }

  float tx = P.x, ty = P.y;
  if (!see) {
    // Walk down the flow field towards the player.
    int cx = (int)e.x, cy = (int)e.y;
    int best = G.flow[cy][cx];
    static const int8_t DX[4] = { 1, -1, 0, 0 }, DY[4] = { 0, 0, 1, -1 };
    for (int k = 0; k < 4; k++) {
      int nx = cx + DX[k], ny = cy + DY[k];
      if (nx < 0 || ny < 0 || nx >= G.mapW || ny >= G.mapH) continue;
      int f = G.flow[ny][nx];
      if (f >= 0 && (best < 0 || f < best)) { best = f; tx = nx + 0.5f; ty = ny + 0.5f; }
    }
    if (best < 0) return;   // no way to reach the player (behind a locked door)
  }
  float mx = tx - e.x, my = ty - e.y;
  float ml = sqrtf(mx * mx + my * my);
  if (ml < 0.001f) return;
  mx /= ml; my /= ml;

  e.strafeTimer -= dt;
  if (e.strafeTimer <= 0) {
    e.strafe = see && m.attack != ATK_MELEE ? (frand() * 2 - 1) * 0.8f : 0;
    e.strafeTimer = 0.5f + frand();
  }
  float stopDist = m.attack == ATK_MELEE ? m.radius + PLAYER_R + 0.05f : 1.6f;
  float fwd = see && d < stopDist ? 0.0f : 1.0f;
  float vx = (mx * fwd - my * e.strafe) * m.speed, vy = (my * fwd + mx * e.strafe) * m.speed;
  moveBody(e.x, e.y, vx * dt, vy * dt, m.radius, idx);
  if (fwd > 0 || e.strafe != 0) e.anim += dt * m.speed * 2.2f;

  // Monsters open ordinary doors they walk into.
  int ax = (int)floorf(e.x + mx * (m.radius + 0.35f)), ay = (int)floorf(e.y + my * (m.radius + 0.35f));
  if (tileAt(ax, ay) == T_DOOR) openDoor(G.doorAt[ay][ax]);
}

static void updateProjectile(Ent& e, float dt) {
  e.timer -= dt;
  if (e.timer <= 0) { e.type = ENT_NONE; return; }
  const int steps = 4;
  for (int s = 0; s < steps; s++) {
    e.x += e.vx * dt / steps;
    e.y += e.vy * dt / steps;
    e.anim += dt / steps;
    if (blocksMove((int)floorf(e.x), (int)floorf(e.y))) {
      e.type = ENT_NONE;
      spawnPuff(e.x - e.vx * 0.02f, e.y - e.vy * 0.02f);
      return;
    }
    float pdx = P.x - e.x, pdy = P.y - e.y;
    if (G.state == ST_PLAY && pdx * pdx + pdy * pdy < (PLAYER_R + 0.15f) * (PLAYER_R + 0.15f)) {
      damagePlayer(e.hp);
      e.type = ENT_NONE;
      spawnPuff(e.x, e.y);
      return;
    }
    for (int i = 0; i < MAX_ENTS; i++) {
      Ent& b = G.ents[i];
      if (b.type != ENT_BARREL || b.state != 0) continue;
      float bdx = b.x - e.x, bdy = b.y - e.y;
      if (bdx * bdx + bdy * bdy < 0.4f * 0.4f) { damageEnt(b, 30); e.type = ENT_NONE; return; }
    }
  }
}

static void updateEntities(float dt) {
  for (int i = 0; i < MAX_ENTS; i++) {
    Ent& e = G.ents[i];
    if (isMonster(e.type)) { updateMonster(i, dt); continue; }
    switch (e.type) {
      case ENT_FIREBALL: case ENT_PLASMA: updateProjectile(e, dt); break;
      case ENT_PUFF: case ENT_BLAST:
        e.timer -= dt;
        if (e.timer <= 0) e.type = ENT_NONE;
        break;
      case ENT_BARREL:
        if (e.state == 1) {
          e.timer -= dt;
          if (e.timer <= 0) { e.type = ENT_NONE; explode(e.x, e.y); }
        }
        break;
      default: break;
    }
  }
}

// ============================================================================ main update
void gameInit(uint32_t seed) {
  memset(&G, 0, sizeof(G));
  G.rng = seed ? seed : 0x1234567u;
  buildArt();
  loadLevel(0);   // backdrop for the title screen
  P.hp = MAX_HP; P.ammo = START_AMMO; P.camH = 0.5f;
  selectWeapon();
  updateCamera();
  G.state = ST_TITLE;
  G.fireBlocked = true;
}

SoundId gameTakeSound() { SoundId s = G.sound; G.sound = SND_NONE; return s; }

void gameUpdate(const GameInput& in, float dt) {
  dt = clampf(dt, 0, 0.05f);
  if (!in.fire) G.fireBlocked = false;
  bool pressed = in.fire && !G.firePrev && !G.fireBlocked;
  G.firePrev = in.fire;
  G.stateTimer += dt;
  G.showMap = in.map;
  if (G.msgTimer > 0) G.msgTimer -= dt;
  if (G.noKeyTimer > 0) G.noKeyTimer -= dt;
  if (P.hurtFlash > 0) P.hurtFlash -= dt;
  if (P.pickupFlash > 0) P.pickupFlash -= dt;

  switch (G.state) {
    case ST_TITLE:
      P.angle += dt * 0.3f;
      updateCamera();
      // Stick left/right picks the starting map.
      if (fabsf(in.turn) < 0.5f) G.selectTimer = 0;
      else if ((G.selectTimer -= dt) <= 0) {
        G.selectLevel = (uint8_t)((G.selectLevel + (in.turn > 0 ? 1 : NUM_LEVELS - 1)) % NUM_LEVELS);
        G.selectTimer = 0.4f;
      }
      if (pressed && G.stateTimer > 0.3f) startLevel(G.selectLevel, true);
      return;
    case ST_LEVEL_DONE:
      if (pressed && G.stateTimer > 0.6f) {
        if (G.level + 1 < NUM_LEVELS) startLevel(G.level + 1, false);
        else { G.state = ST_VICTORY; G.stateTimer = 0; G.fireBlocked = true; }
      }
      return;
    case ST_VICTORY:
      if (pressed && G.stateTimer > 0.8f) {
        gameInit(rnd());
        G.stateTimer = 0;
      }
      return;
    case ST_DEAD:
      if (P.camH > 0.12f) P.camH -= dt * 0.7f;
      updateDoors(dt);
      updateEntities(dt);
      if (pressed && G.stateTimer > 1.0f) restartLevel();
      return;
    case ST_PLAY:
      if (in.map && pressed) {   // map + fire: back to the title, this map preselected
        uint8_t level = G.level;
        gameInit(rnd());
        G.selectLevel = level;
        return;
      }
      G.levelTime += dt;
      updatePlayer(in, dt);
      if (G.state != ST_PLAY) return;
      updateDoors(dt);
      G.flowTimer -= dt;
      if ((int)P.x != flowCellX || (int)P.y != flowCellY || G.flowTimer <= 0) {
        computeFlow();
        G.flowTimer = 0.5f;
      }
      updateEntities(dt);
      return;
  }
}

// ============================================================================ rendering
static float zbuf[SCREEN_W];

// Brightness of lit wall texels: full white up close, fading with distance. Walls facing
// north/south are a little darker so corners show.
static float wallLevel(float d, int side, float boost) {
  float l = 1.25f - d * 0.13f;
  if (side) l *= 0.6f;
  return clampf(l + boost, 0.1f, 1.0f);
}

static void renderWorld() {
  float boost = (P.fireAnim > 0.08f && P.weapon != 0) ? 0.3f : 0.0f;   // muzzle flash lights the room
  float camH = P.camH;

  // Floor rows: distance and world position of the left edge, plus the step per column.
  static float rowFX[VIEW_H], rowFY[VIEW_H], rowSX[VIEW_H], rowSY[VIEW_H], rowLevel[VIEW_H];
  for (int y = HORIZON; y < VIEW_H; y++) {
    float rowDist = camH * VSCALE / (y + 0.5f - HORIZON);
    float rx0 = P.dirX - P.planeX, ry0 = P.dirY - P.planeY;
    float rx1 = P.dirX + P.planeX, ry1 = P.dirY + P.planeY;
    rowSX[y] = rowDist * (rx1 - rx0) / SCREEN_W;
    rowSY[y] = rowDist * (ry1 - ry0) / SCREEN_W;
    rowFX[y] = P.x + rowDist * rx0 + rowSX[y] * 0.5f;
    rowFY[y] = P.y + rowDist * ry0 + rowSY[y] * 0.5f;
    rowLevel[y] = clampf(1.0f - rowDist * 0.2f + boost, 0, 1);
  }

  int prevTop = 0, prevBot = 0;
  float prevDist = 0;
  int prevSide = -1;
  for (int x = 0; x < SCREEN_W; x++) {
    float camX = 2.0f * (x + 0.5f) / SCREEN_W - 1.0f;
    float rdx = P.dirX + P.planeX * camX, rdy = P.dirY + P.planeY * camX;
    RayHit h;
    castRay(P.x, P.y, rdx, rdy, h);
    if (h.cx >= 0 && h.cy >= 0 && h.cx < G.mapW && h.cy < G.mapH) G.seen[h.cy][h.cx] = 1;
    float d = h.dist < 0.05f ? 0.05f : h.dist;
    zbuf[x] = d;
    float lh = VSCALE / d;
    float top = HORIZON - (1.0f - camH) * lh, bot = HORIZON + camH * lh;
    int y0 = (int)ceilf(top - 0.5f), y1 = (int)ceilf(bot - 0.5f) - 1;
    int c0 = imax(y0, 0), c1 = imin(y1, VIEW_H - 1);

    // Wall: 16x16 texture, lit texels dithered by distance; tiny far walls skip the texture
    // so they don't turn into noise.
    float lvl = wallLevel(d, h.side, boost);
    bool detail = lh >= (h.tex >= TX_DOOR ? 12.0f : 22.0f);   // doors and the exit stay readable further out
    int tx = (int)(h.wallX * 16) & 15;
    const uint16_t* T = tex[h.tex];
    float tstep = 16.0f / (bot - top);
    float tpos = (c0 + 0.5f - top) * tstep;
    for (int y = c0; y <= c1; y++, tpos += tstep) {
      bool lit = !detail || (T[(int)tpos & 15] >> tx & 1);
      if (lit && dither(x, y, lvl)) pset(x, y);
    }
    // Bright outline along the top and bottom of every wall: the room shape always reads.
    if (y0 >= 0 && y0 < VIEW_H) pset(x, y0);
    if (y1 >= 0 && y1 < VIEW_H) pset(x, y1);

    // Vertical edge where the wall jumps in depth or turns a corner.
    if (x > 0 && (fabsf(d - prevDist) > 0.12f * (d < prevDist ? d : prevDist) + 0.05f || (int)h.side != prevSide)) {
      if (d <= prevDist) vline(x, c0, c1);
      else vline(x - 1, imax(prevTop, 0), imin(prevBot, VIEW_H - 1));
    }
    prevTop = y0; prevBot = y1; prevDist = d; prevSide = h.side;

    // Floor: a tile grid that fades out with distance, gives a strong sense of motion.
    for (int y = imax(y1 + 1, HORIZON); y < VIEW_H; y++) {
      float fx = rowFX[y] + rowSX[y] * x, fy = rowFY[y] + rowSY[y] * x;
      float gx = fx - floorf(fx), gy = fy - floorf(fy);
      bool line = gx < 0.07f || gy < 0.07f;
      if (line && dither(x, y, rowLevel[y])) pset(x, y);
    }
  }
}

// Scaled sprite with per-column depth test. inv swaps white and black (hit flash).
static void drawSprite(const Bitmap& b, float cx, float bottom, float hpx, float depth, bool inv) {
  if (hpx < 1) return;
  float wpx = hpx * b.w / b.h;
  float left = cx - wpx * 0.5f, top = bottom - hpx;
  int x0 = imax((int)ceilf(left - 0.5f), 0), x1 = imin((int)ceilf(left + wpx - 0.5f) - 1, SCREEN_W - 1);
  int y0 = imax((int)ceilf(top - 0.5f), 0), y1 = imin((int)ceilf(bottom - 0.5f) - 1, VIEW_H - 1);
  for (int x = x0; x <= x1; x++) {
    if (depth >= zbuf[x]) continue;
    int sx = (int)((x + 0.5f - left) * b.w / wpx);
    if (sx < 0 || sx >= b.w) continue;
    for (int y = y0; y <= y1; y++) {
      int sy = (int)((y + 0.5f - top) * b.h / hpx);
      if (sy < 0 || sy >= b.h) continue;
      if (!(b.solid[sy] >> sx & 1)) continue;
      bool white = (b.on[sy] >> sx & 1) != 0;
      pput(x, y, white != inv);
    }
  }
}

struct SpriteRef { float depth, sx; uint8_t idx; };

static void renderSprites() {
  static SpriteRef list[MAX_ENTS];
  int n = 0;
  for (int i = 0; i < MAX_ENTS; i++) {
    const Ent& e = G.ents[i];
    if (e.type == ENT_NONE) continue;
    float dx = e.x - P.x, dy = e.y - P.y;
    float depth = dx * P.dirX + dy * P.dirY;
    if (depth < 0.2f) continue;
    float lateral = P.dirX * dy - P.dirY * dx;
    float sx = SCREEN_W * 0.5f * (1 + lateral / (PLANE * depth));
    if (sx < -64 || sx > SCREEN_W + 64) continue;
    list[n].depth = depth; list[n].sx = sx; list[n].idx = (uint8_t)i;
    n++;
  }
  for (int i = 1; i < n; i++) {   // far to near
    SpriteRef v = list[i];
    int j = i - 1;
    while (j >= 0 && list[j].depth < v.depth) { list[j + 1] = list[j]; j--; }
    list[j + 1] = v;
  }

  for (int i = 0; i < n; i++) {
    const Ent& e = G.ents[list[i].idx];
    float depth = list[i].depth;
    int b = -1;
    float h = 0.5f, z = 0;
    bool inv = false;
    if (isMonster(e.type)) {
      const MonInfo& m = monInfo(e);
      h = m.height;
      if (e.state == MS_DEAD) b = B_CORPSE;
      else if (e.state == MS_DYING) b = e.timer > 0.25f ? (int)m.walk1 : (int)B_DIE;
      else if (e.state == MS_ATTACK || e.shotFlash > 0) b = m.atk;
      else b = ((int)e.anim & 1) ? m.walk2 : m.walk1;
      inv = e.hitFlash > 0 || (e.state == MS_DYING && e.timer > 0.25f);
    } else {
      switch (e.type) {
        case ENT_BARREL:   b = B_BARREL; h = 0.5f; inv = e.state == 1; break;
        case ENT_MEDKIT:   b = B_MEDKIT; h = 0.27f; break;
        case ENT_CLIP:     b = B_CLIP; h = 0.26f; break;
        case ENT_AMMOBOX:  b = B_AMMOBOX; h = 0.24f; break;
        case ENT_SHOTGUN:  b = B_SHOTGUN_PICK; h = 0.16f; break;
        case ENT_KEY:      b = B_KEY; h = 0.22f; inv = ((int)(G.levelTime * 4)) & 1; break;   // blinks
        case ENT_FIREBALL: b = ((int)(e.anim * 12)) & 1 ? B_FIREBALL2 : B_FIREBALL1; h = 0.24f; z = 0.36f; break;
        case ENT_PLASMA:   b = B_PLASMA; h = 0.24f; z = 0.36f; inv = ((int)(e.anim * 12)) & 1; break;
        case ENT_PUFF:     b = B_PUFF; h = 0.14f; z = 0.42f; break;
        case ENT_BLAST:    b = e.timer > 0.25f ? B_BLAST1 : B_BLAST2; h = 1.0f; z = 0.0f; break;
      }
    }
    if (b < 0) continue;
    float bottom = HORIZON + (P.camH - z) * VSCALE / depth;
    drawSprite(bmp[b], list[i].sx, bottom, h * VSCALE / depth, depth, inv);
  }
}

static void renderWeapon() {
  if (G.state != ST_PLAY) return;
  int bobX = (int)lroundf(sinf(P.bob) * 3.0f * P.bobAmount);
  int bobY = (int)lroundf(fabsf(cosf(P.bob)) * 2.0f * P.bobAmount);
  const Bitmap* w;
  int x, y;
  bool flash = false;
  if (P.weapon == 0) {
    w = &bmp[B_FIST];
    int punch = P.fireAnim > 0 ? (int)(sinf(P.fireAnim / 0.25f * 3.14159f) * 10) : 0;
    x = SCREEN_W / 2 + 6 - punch + bobX;
    y = VIEW_H - w->h + 4 - punch + bobY;
  } else {
    w = &bmp[P.weapon == 1 ? B_PISTOL : B_SHOTGUN];
    float anim = P.weapon == 1 ? 0.16f : 0.22f;
    int recoil = P.fireAnim > 0 ? (int)(P.fireAnim / anim * (P.weapon == 1 ? 3 : 5)) : 0;
    flash = P.fireAnim > anim - 0.08f;
    x = SCREEN_W / 2 - w->w / 2 + bobX;
    y = VIEW_H - w->h + 1 + recoil + bobY;
  }
  blit(*w, x, y, VIEW_H);
  if (flash) blit(bmp[B_FLASH], x + w->w / 2 - bmp[B_FLASH].w / 2, y - bmp[B_FLASH].h + 1, VIEW_H);
}

static void renderCrosshair() {
  int cx = SCREEN_W / 2, cy = HORIZON;
  pxor(cx - 2, cy); pxor(cx - 3, cy); pxor(cx + 2, cy); pxor(cx + 3, cy);
  pxor(cx, cy - 2); pxor(cx, cy + 2);
}

static void renderHud() {
  fillRect(0, VIEW_H, SCREEN_W, SCREEN_H - VIEW_H, false);
  hline(0, SCREEN_W - 1, VIEW_H);
  char buf[12];
  int y = VIEW_H + 1;
  blit(bmp[B_ICON_HEALTH], 0, y);
  bool lowBlink = G.state == ST_PLAY && P.hp <= 25 && ((int)(G.levelTime * 4) & 1);
  if (!lowBlink) { snprintf(buf, sizeof(buf), "%3d", P.hp); drawText(9, y, buf); }
  blit(bmp[B_ICON_AMMO], 32, y);
  if (P.weapon == 0) drawText(37, y, " --");
  else { snprintf(buf, sizeof(buf), "%3d", P.ammo); drawText(37, y, buf); }
  blit(bmp[B_ICON_SKULL], 60, y);
  snprintf(buf, sizeof(buf), "%d/%d", G.kills, G.totalMonsters);
  drawText(69, y, buf);
  if (P.hasKey) blit(bmp[B_ICON_KEY], 106, y);
  int face = G.state == ST_DEAD ? B_FACE_DEAD : (P.hp < 40 || P.hurtFlash > 0) ? B_FACE_HURT : B_FACE_OK;
  blit(bmp[face], SCREEN_W - 7, y);
}

static void renderFlashes() {
  if (P.hurtFlash > 0.28f) {   // first frames of a hit: flash the whole view
    for (int y = 0; y < VIEW_H; y++) for (int x = 0; x < SCREEN_W; x++) pxor(x, y);
  } else if (P.hurtFlash > 0) {
    for (int t = 0; t < 2; t++) rect(t, t, SCREEN_W - 2 * t, VIEW_H - 2 * t);
  } else if (P.pickupFlash > 0) {
    rect(0, 0, SCREEN_W, VIEW_H);
  }
}

static void renderAutomap() {
  memset(FB, 0, FB_BYTES);
  int cs = imin(SCREEN_W / G.mapW, (SCREEN_H - 8) / G.mapH);
  if (cs < 2) cs = 2;
  int ox = (SCREEN_W - G.mapW * cs) / 2, oy = 8 + (SCREEN_H - 8 - G.mapH * cs) / 2;
  bool blink = ((int)(G.levelTime * 4)) & 1;
  for (int y = 0; y < G.mapH; y++) {
    for (int x = 0; x < G.mapW; x++) {
      if (!G.seen[y][x]) continue;
      uint8_t t = G.tiles[y][x];
      int px = ox + x * cs, py = oy + y * cs;
      if (isDoorTile(t)) {
        const Door& d = G.doors[G.doorAt[y][x]];
        bool dash = t == T_DOOR || blink;
        if (dash) {
          if (d.planeX) for (int k = 0; k < cs; k += 2) pset(px + cs / 2, py + k);
          else for (int k = 0; k < cs; k += 2) pset(px + k, py + cs / 2);
        }
        continue;
      }
      if (t == T_EXIT) { if (blink) fillRect(px, py, cs, cs, true); continue; }
      // Draw only the faces of a wall that border open floor: room outlines, like Doom's map.
      if (tileAt(x - 1, y) == T_EMPTY || isDoorTile(tileAt(x - 1, y))) vline(px, py, py + cs - 1);
      if (tileAt(x + 1, y) == T_EMPTY || isDoorTile(tileAt(x + 1, y))) vline(px + cs - 1, py, py + cs - 1);
      if (tileAt(x, y - 1) == T_EMPTY || isDoorTile(tileAt(x, y - 1))) hline(px, px + cs - 1, py);
      if (tileAt(x, y + 1) == T_EMPTY || isDoorTile(tileAt(x, y + 1))) hline(px, px + cs - 1, py + cs - 1);
    }
  }
  float pxf = ox + P.x * cs, pyf = oy + P.y * cs;
  for (int k = 0; k <= 4; k++) {
    int x = (int)(pxf + P.dirX * k), y = (int)(pyf + P.dirY * k);
    if (onScreen(x, y)) pset(x, y);
  }
  fillRect((int)pxf - 1, (int)pyf - 1, 3, 3, true);
  drawText(0, 0, LEVELS[G.level].id);
  if (G.state == ST_PLAY) drawText(SCREEN_W - textWidth("FIRE:QUIT", 1), 0, "FIRE:QUIT");
}

static void formatTime(char* buf, int n, float t) {
  unsigned s = t > 0 ? (unsigned)t % 6000 : 0;
  snprintf(buf, n, "%u:%02u", s / 60, s % 60);
}

static int percent(int a, int b) { return b > 0 ? a * 100 / b : 100; }

void gameRender(uint8_t* fb) {
  FB = fb;
  memset(FB, 0, FB_BYTES);
  char buf[32];
  bool blink = ((int)(G.stateTimer * 2.5f)) & 1;

  if (G.state == ST_LEVEL_DONE || G.state == ST_VICTORY) {
    if (G.state == ST_LEVEL_DONE) {
      snprintf(buf, sizeof(buf), "%s FINISHED", LEVELS[G.level].id);
      drawTextCentered(1, buf);
      drawTextCentered(11, LEVELS[G.level].name);
      hline(10, SCREEN_W - 11, 20);
      snprintf(buf, sizeof(buf), "KILLS  %3d%%", percent(G.kills, G.totalMonsters)); drawText(25, 24, buf);
      snprintf(buf, sizeof(buf), "ITEMS  %3d%%", percent(G.items, G.totalItems)); drawText(25, 33, buf);
      char t[12]; formatTime(t, sizeof(t), G.levelTime);
      snprintf(buf, sizeof(buf), "TIME  %5s", t); drawText(25, 42, buf);
    } else {
      drawTextCentered(2, "YOU WIN!", 2);
      drawTextCentered(20, "HELL IS SILENT.");
      snprintf(buf, sizeof(buf), "KILLS %d/%d", G.totalKills, G.totalMonstersAll); drawTextCentered(31, buf);
      char t[12]; formatTime(t, sizeof(t), G.totalTime);
      snprintf(buf, sizeof(buf), "TIME %s", t); drawTextCentered(40, buf);
    }
    if (G.stateTimer > 0.6f && blink) drawTextCentered(55, "PRESS FIRE");
    return;
  }

  if (G.showMap && G.state != ST_TITLE) {
    renderAutomap();
    return;
  }

  renderWorld();
  renderSprites();

  if (G.state == ST_TITLE) {
    fillRect(0, VIEW_H, SCREEN_W, SCREEN_H - VIEW_H, false);
    int w = textWidth("DOOM", 3);
    fillRect((SCREEN_W - w) / 2 - 4, 2, w + 8, 27, false);
    rect((SCREEN_W - w) / 2 - 4, 2, w + 8, 27);
    drawText((SCREEN_W - w) / 2, 5, "DOOM", 3);
    snprintf(buf, sizeof(buf), "< %s >", LEVELS[G.selectLevel].id);
    drawTextBox(35, buf);
    drawTextBox(46, LEVELS[G.selectLevel].name);
    if (blink) drawTextBox(56, "PRESS FIRE");
    return;
  }

  renderWeapon();
  if (G.state == ST_PLAY) renderCrosshair();
  renderFlashes();
  renderHud();

  if (G.state == ST_PLAY && G.levelTime < 2.5f) {
    snprintf(buf, sizeof(buf), "%s", LEVELS[G.level].id);
    int w = imax(textWidth(buf, 2), textWidth(LEVELS[G.level].name, 1));
    fillRect((SCREEN_W - w) / 2 - 4, 10, w + 8, 29, false);
    rect((SCREEN_W - w) / 2 - 4, 10, w + 8, 29);
    drawTextCentered(13, buf, 2);
    drawTextCentered(29, LEVELS[G.level].name);
  } else if (G.msgTimer > 0) {
    fillRect(0, 0, textWidth(G.msg, 1) + 3, 9, false);
    drawText(1, 1, G.msg);
  }

  if (G.state == ST_DEAD && G.stateTimer > 0.8f) {
    drawTextBox(12, "YOU DIED", 2);
    if (blink) drawTextBox(38, "FIRE: RESTART");
  }
}
