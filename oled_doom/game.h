// OLED DOOM — game engine (no Arduino dependencies, so it also builds on a PC for tests).
//
// The sketch feeds it joystick/button input once per frame and copies the 128x64 frame it
// renders straight into the display buffer.

#pragma once
#include <stdint.h>

#define SCREEN_W 128
#define SCREEN_H 64
#define FB_BYTES (SCREEN_W * SCREEN_H / 8)   // SH1106/SSD1306 page layout: fb[x + (y/8)*128], bit y&7

struct GameInput {
  float turn;   // -1 = full left .. +1 = full right
  float move;   // -1 = full back  .. +1 = full forward
  bool fire;    // held = auto fire; press = start / restart / continue
  bool map;     // held = automap
};

// Listed from least to most important: when several happen in one frame the most important wins.
enum SoundId : uint8_t {
  SND_NONE, SND_DOOR, SND_NO_KEY, SND_PICKUP, SND_PUNCH, SND_PISTOL, SND_SHOTGUN, SND_MON_SHOT,
  SND_FIREBALL, SND_MON_HIT, SND_MON_DIE, SND_PLAYER_HURT, SND_EXPLODE, SND_PLAYER_DIE,
  SND_LEVEL_DONE
};

void gameInit(uint32_t seed);
void gameUpdate(const GameInput& in, float dt);   // dt in seconds
void gameRender(uint8_t* fb);                      // fills FB_BYTES bytes
SoundId gameTakeSound();                           // most important sound since the last call

// ---------------------------------------------------------------------------------------------
// Game state (exposed so the PC test harness can inspect it; the sketch doesn't need it).
// ---------------------------------------------------------------------------------------------
#define MAX_MAP 32
#define MAX_ENTS 96
#define MAX_DOORS 32

enum GameStateId : uint8_t { ST_TITLE, ST_PLAY, ST_DEAD, ST_LEVEL_DONE, ST_VICTORY };

enum EntType : uint8_t {
  ENT_NONE,
  ENT_ZOMBIE, ENT_IMP, ENT_DEMON, ENT_BARON,                     // monsters
  ENT_BARREL,
  ENT_MEDKIT, ENT_CLIP, ENT_AMMOBOX, ENT_SHOTGUN, ENT_KEY,       // pickups
  ENT_FIREBALL, ENT_PLASMA,                                      // projectiles
  ENT_PUFF, ENT_BLAST                                            // effects
};

enum MonState : uint8_t { MS_IDLE, MS_CHASE, MS_ATTACK, MS_PAIN, MS_DYING, MS_DEAD };

struct Ent {
  uint8_t type;
  uint8_t state;
  float x, y;
  float vx, vy;        // projectiles
  float timer;         // state timer / lifetime
  float cooldown;      // monster attack cooldown
  float anim;          // walk animation clock
  float strafe;        // monster side-step (-1..1)
  float strafeTimer;
  float hitFlash;      // > 0: draw inverted (just got hit)
  float shotFlash;     // > 0: keep showing the attack frame after firing
  float lookTimer;     // idle monsters: time to next sight check
  int16_t hp;          // health, or damage for projectiles
};

struct Door {
  uint8_t x, y;
  bool planeX;         // true: panel lies on x = cell + 0.5 (you walk through it along x)
  bool locked;         // needs the red key
  uint8_t state;       // 0 closed, 1 opening, 2 open, 3 closing
  float open;          // 0 = closed .. 1 = fully open
  float timer;
};

struct Player {
  float x, y, angle;
  float dirX, dirY, planeX, planeY;
  int16_t hp, ammo;
  bool hasShotgun, hasKey;
  uint8_t weapon;      // 0 fist, 1 pistol, 2 shotgun
  float fireCooldown;
  float fireAnim;      // > 0 while the weapon is in its firing frame
  float bob;           // walk bob phase
  float bobAmount;
  float hurtFlash, pickupFlash;
  float camH;          // eye height (drops when dead)
};

struct Game {
  GameStateId state;
  uint8_t level;
  uint8_t selectLevel; // title screen choice
  float selectTimer;
  float stateTimer;
  float levelTime, totalTime;
  int kills, totalMonsters, items, totalItems;
  int totalKills, totalMonstersAll;
  uint8_t mapW, mapH;
  uint8_t tiles[MAX_MAP][MAX_MAP];
  int8_t doorAt[MAX_MAP][MAX_MAP];
  uint8_t seen[MAX_MAP][MAX_MAP];
  int16_t flow[MAX_MAP][MAX_MAP];
  float flowTimer;
  Door doors[MAX_DOORS];
  int numDoors;
  Ent ents[MAX_ENTS];
  Player p;
  // Inventory carried into the current level (restored when you die and restart).
  int16_t startAmmo;
  bool startShotgun;
  char msg[24];
  float msgTimer;
  float noKeyTimer;
  bool firePrev;
  bool fireBlocked;    // ignore fire until it is released (after a screen change)
  bool showMap;
  bool godMode;        // tests only
  uint32_t rng;
  SoundId sound;
};

extern Game G;
