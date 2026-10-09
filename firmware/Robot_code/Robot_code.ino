/*
  Desk Robot — Cute Eyes (Cozmo/Vector style) + Dual Servo
  Board   : Arduino Uno / Nano
  Display : SSD1306 128×64 OLED, I2C  (A4 = SDA, A5 = SCL)
  Servos  : D6 = HEAD (up / down)   HEAD_MIN 60° – HEAD_MAX 90°  rest 80°
            D9 = NECK (left / right) NECK_MIN  0° – NECK_MAX 70°  rest 35°

  Expression numbers used in drawFace():
    0 = Cute (default, no mask)
    1 = Mad  (V-brow triangles pointing down toward nose)
    2 = Cry  (Λ-brow triangles pointing up toward nose)
    3 = Glare (straight top eyelid rect, hooded)
    4 = Sleep / Blink (full rect covering oval, tiny bottom crescent)
    5 = Happy (circle mask on bottom, top crescent squint)

  Servo power model:
    - Both servos stay ATTACHED for the whole active window; only one ever
      MOVES at a time (moves are sequential). Holding still draws ~10 mA,
      moving draws the peak — so this avoids the re-attach slams that
      detach/attach cycling caused.
    - The HEAD servo is never released: with the hat on, an unpowered head
      drops under gravity, and both the drop and the re-attach snap back up
      caused brownout resets (video 2026-10-05). Only the neck is released,
      for the sleep window, via releaseNeck(), which forces the signal pin
      LOW after detach() (the AVR Servo lib leaves the pin stuck HIGH if
      detached mid-pulse → servo drives to an end stop).
    - Head moves are kept small and slow (HEAD_SOFT_MIN..HEAD_SOFT_MAX,
      HEAD_STEP_*), and Look Up uses the eyes only — no head motion.
    - write() is always called BEFORE attach() so the first pulse is the
      real position, not the library's 1500 µs (90°) default.
    - Moves ease in/out (slow first/last steps) to cut acceleration current.
  HEAD strictly 60–90° hard, HEAD_SOFT_MIN..HEAD_SOFT_MAX in practice.
  Boot: the head is assumed drooped (HEAD_DROOP) and lifted slowly to REST;
  the neck sweeps in from its last EEPROM-saved angle (see savePosition()).
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <EEPROM.h>

// ── Display ───────────────────────────────────────────────────────────────────
#define SCREEN_W   128
#define SCREEN_H    64
#define OLED_RESET  -1
#define OLED_ADDR 0x3C

// I2C at 400 kHz both during and after display() (lib default restores 100 kHz
// after each frame) — shorter bus activity = less ISR contention with Servo.
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RESET, 400000UL, 400000UL);

// ── Servo pins & hard limits ──────────────────────────────────────────────────
#define HEAD_PIN    6
#define HEAD_MIN   60
#define HEAD_MAX   90
#define HEAD_REST  80

// Head soft limits, minimum move and slower ramp — the hat makes the head a
// heavy lever, and every reset in the 2026-10-05 video was a head-servo event.
#define HEAD_SOFT_MIN       70
#define HEAD_SOFT_MAX       84
#define HEAD_MIN_MOVE_DEG    3
#define HEAD_STEP_SLOW_MS   75   // ms per 1° at the start/end of a head move
#define HEAD_STEP_FAST_MS   45   // ms per 1° mid head move
#define HEAD_DROOP          60   // where the unpowered head falls to (seen on video)

#define NECK_PIN    9
#define NECK_MIN    0
#define NECK_MAX   70
#define NECK_REST  35

// Neck soft limits + minimum move — video showed the body getting kicked (and
// walking/rotating) by small reversing neck moves held near the travel ends.
// moveNeck() never goes outside SOFT_MIN..SOFT_MAX and ignores moves < MIN_MOVE.
#define NECK_SOFT_MIN       8
#define NECK_SOFT_MAX      62
#define NECK_MIN_MOVE_DEG   4

// ── Power-safe timing ─────────────────────────────────────────────────────────
// Widen these if brownouts/reboots persist — they directly trade animation
// speed for capacitor recharge headroom.
#define STEP_SLOW_MS      50   // ms per 1° step at the start/end of a move (ease)
#define STEP_FAST_MS      30   // ms per 1° step in the middle of a move
#define EASE_STEPS         4   // steps over which speed ramps between SLOW and FAST
#define SETTLE_DELAY_MS   120  // ms to hold after a sweep finishes
#define ATTACH_GAP_MS     250  // ms between attaching head and neck (staggers inrush)

Servo headServo;
Servo neckServo;
int   headPos = HEAD_REST;
int   neckPos = NECK_REST;

// ── Last-settled-position persistence ────────────────────────────────────────
// Saved at the sleep pose only (see animSleepingFor()). Read back on boot so the
// initial move sweeps in from there instead of assuming REST. savePosition()
// skips unchanged bytes, so a fixed sleep pose costs no further EEPROM writes.
#define EEPROM_ADDR_HEAD 0   // unused since head boots from HEAD_DROOP; kept so addresses don't shift
#define EEPROM_ADDR_NECK 1

void savePosition(int addr, int pos) {
  if (EEPROM.read(addr) != (uint8_t)pos) EEPROM.write(addr, (uint8_t)pos);
}

int loadPosition(int addr, int lo, int hi, int fallback) {
  int v = EEPROM.read(addr);
  return (v >= lo && v <= hi) ? v : fallback;
}

// ── Reset diagnostics (breadcrumb) ───────────────────────────────────────────
// `crumb` lives in .noinit RAM: not cleared at startup, so it survives a
// brownout/reset but turns to garbage on a real cold power-up. setCrumb() is
// called at every behaviour/phase change; at boot a valid crumb = "the board
// RESET while it was doing <where>", shown on the OLED and kept in EEPROM
// (written only when such a reset happens — no wear in normal running).
#define CRUMB_MAGIC 0xB07C
enum : uint8_t {
  CRUMB_BOOT = 0, CRUMB_LOOK_L, CRUMB_LOOK_R, CRUMB_LOOK_UP, CRUMB_LOOK_DN,
  CRUMB_HAPPY, CRUMB_SURPRISED, CRUMB_MAD, CRUMB_CRY, CRUMB_GLARE, CRUMB_CURIOUS,
  CRUMB_CUTE, CRUMB_BLINKS, CRUMB_PAUSE, CRUMB_SLEEP_IN, CRUMB_SLEEP, CRUMB_WAKE
};
struct Crumb { uint16_t magic; uint8_t where; uint8_t phase; uint8_t check; };
Crumb crumb __attribute__((section(".noinit")));

// phase: 0 = moving into the pose, 1 = holding / idle
void setCrumb(uint8_t where, uint8_t phase) {
  crumb.magic = 0;                      // invalidate while updating
  crumb.where = where; crumb.phase = phase;
  crumb.check = where ^ phase ^ 0xA5;
  crumb.magic = CRUMB_MAGIC;
}
void crumbHold() { setCrumb(crumb.where, 1); }   // same behaviour, now holding

bool crumbValid() {
  return crumb.magic == CRUMB_MAGIC && crumb.check == (uint8_t)(crumb.where ^ crumb.phase ^ 0xA5)
         && crumb.where <= CRUMB_WAKE && crumb.phase <= 1;
}

const __FlashStringHelper *crumbName(uint8_t w) {
  switch (w) {
    case CRUMB_BOOT:      return F("BOOT");
    case CRUMB_LOOK_L:    return F("LOOK LEFT");
    case CRUMB_LOOK_R:    return F("LOOK RIGHT");
    case CRUMB_LOOK_UP:   return F("LOOK UP");
    case CRUMB_LOOK_DN:   return F("LOOK DOWN");
    case CRUMB_HAPPY:     return F("HAPPY");
    case CRUMB_SURPRISED: return F("SURPRISED");
    case CRUMB_MAD:       return F("MAD");
    case CRUMB_CRY:       return F("CRY");
    case CRUMB_GLARE:     return F("GLARE");
    case CRUMB_CURIOUS:   return F("CURIOUS");
    case CRUMB_CUTE:      return F("CUTE");
    case CRUMB_BLINKS:    return F("BLINKS");
    case CRUMB_PAUSE:     return F("PAUSE");
    case CRUMB_SLEEP_IN:  return F("SLEEP-IN");
    case CRUMB_SLEEP:     return F("SLEEP");
    case CRUMB_WAKE:      return F("WAKE");
  }
  return F("?");
}

// MCUSR captured before anything else runs. Some bootloaders (stock Uno
// optiboot) clear it first — then it reads 0 and the cause shows as "?".
uint8_t bootMcusr __attribute__((section(".noinit")));
void captureMcusr() __attribute__((naked, used, section(".init3")));
void captureMcusr() { bootMcusr = MCUSR; MCUSR = 0; }

// EEPROM record of the most recent unexpected reset (survives power-off)
#define EEPROM_ADDR_RST_COUNT 2
#define EEPROM_ADDR_RST_WHERE 3
#define EEPROM_ADDR_RST_PHASE 4
#define EEPROM_ADDR_RST_MCUSR 5

void printResetCause(uint8_t f) {
  if (f & _BV(BORF))       display.print(F("BROWNOUT"));
  else if (f & _BV(WDRF))  display.print(F("WATCHDOG"));
  else if (f & _BV(EXTRF)) display.print(F("RESET PIN"));
  else if (f & _BV(PORF))  display.print(F("POWER-ON"));
  else                     display.print(F("?"));
}

// Called once from setup() after display.begin(). Shows why we booted.
void reportBoot() {
  bool warm = crumbValid();
  uint8_t cnt = EEPROM.read(EEPROM_ADDR_RST_COUNT);
  if (cnt == 0xFF) cnt = 0;              // fresh EEPROM

  if (warm) {
    if (cnt < 254) cnt++;
    EEPROM.write(EEPROM_ADDR_RST_COUNT, cnt);
    EEPROM.write(EEPROM_ADDR_RST_WHERE, crumb.where);
    EEPROM.write(EEPROM_ADDR_RST_PHASE, crumb.phase);
    EEPROM.write(EEPROM_ADDR_RST_MCUSR, bootMcusr);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  if (warm) {
    display.setTextSize(2); display.print(F("RESET #")); display.println(cnt);
    display.setTextSize(1);
    display.print(F("in: ")); display.println(crumbName(crumb.where));
    display.print(F("    ")); display.println(crumb.phase ? F("(holding)") : F("(moving)"));
    display.print(F("cause: ")); printResetCause(bootMcusr);
  } else {
    display.setTextSize(2); display.println(F("POWER ON"));
    display.setTextSize(1);
    display.print(F("resets so far: ")); display.println(cnt);
    if (cnt > 0) {
      uint8_t w = EEPROM.read(EEPROM_ADDR_RST_WHERE);
      display.print(F("last in: ")); display.println(crumbName(w <= CRUMB_WAKE ? w : 0xFF));
    }
  }
  display.display();
  delay(warm ? 5000 : 1500);
  setCrumb(CRUMB_BOOT, 0);
}

// ── Eye geometry — shared constants ──────────────────────────────────────────
#define CX_L    40      // left  eye centre X
#define CX_R    88      // right eye centre X
#define CY_E    32      // both  eye centre Y
#define RX_E    14      // ellipse horizontal radius
#define RY_E    22      // ellipse vertical radius
#define SSH      8      // sclera horizontal shift for directional looks
#define SSV      4      // sclera vertical   shift for directional looks
#define SH       3      // pupil extra shift within sclera
#define SV       6      // pupil vertical extra shift

// ── Servo helpers ─────────────────────────────────────────────────────────────
// Both joints stay attached through the active window; moves are sequential so
// only one servo ever draws movement current. Detach happens only for sleep.
bool headAttached = false;
bool neckAttached = false;
uint32_t lastAttachMs = 0;    // millis() timestamp of the most recent attach (either servo)

// Staggers attaches so two re-attach inrush pulls never overlap.
void waitForAttachGap() {
  uint32_t elapsed = millis() - lastAttachMs;
  if (elapsed < ATTACH_GAP_MS) delay(ATTACH_GAP_MS - elapsed);
}

// write() BEFORE attach(): attach() starts Timer1 and the ISR emits the first
// pulse within µs using the stored value — writing after would let one 1500 µs
// (90°) default pulse escape on the first attach after boot.
void attachHead() {
  if (headAttached) return;
  waitForAttachGap();
  headServo.write(headPos);
  headServo.attach(HEAD_PIN);
  headAttached = true;
  lastAttachMs = millis();
  delay(SETTLE_DELAY_MS);   // let the horn pull in from any sag before ramping
}

void attachNeck() {
  if (neckAttached) return;
  waitForAttachGap();
  neckServo.write(neckPos);
  neckServo.attach(NECK_PIN);
  neckAttached = true;
  lastAttachMs = millis();
  delay(SETTLE_DELAY_MS);
}

// digitalWrite LOW after detach(): the Servo ISR only drives LOW for active
// channels, and detaching the last servo stops Timer1 — so a detach mid-pulse
// would leave the line stuck HIGH and the servo drives to an end stop.
void releaseNeck() { if (neckAttached) { neckServo.detach(); digitalWrite(NECK_PIN, LOW); neckAttached = false; } }
// No releaseHead(): the head must stay powered (see header — it drops when released).

// 1° steps with ease-in/out: first and last EASE_STEPS steps run at STEP_SLOW_MS,
// ramping to STEP_FAST_MS mid-move — lower acceleration = smaller current spikes.
void rampServo(Servo &s, int &pos, int target, int slowMs, int fastMs) {
  int n = abs(target - pos);
  if (n == 0) return;
  int d = (target > pos) ? 1 : -1;
  for (int i = 0; i < n; i++) {
    pos += d;
    s.write(pos);
    int edge = min(i, n - 1 - i);
    if (edge > EASE_STEPS) edge = EASE_STEPS;
    delay(fastMs + (slowMs - fastMs) * (EASE_STEPS - edge) / EASE_STEPS);
  }
  delay(SETTLE_DELAY_MS);
}

void moveHead(int target) {
  target = constrain(target, HEAD_SOFT_MIN, HEAD_SOFT_MAX);
  attachHead();
  if (abs(target - headPos) < HEAD_MIN_MOVE_DEG) return;   // skip tiny head twitches
  rampServo(headServo, headPos, target, HEAD_STEP_SLOW_MS, HEAD_STEP_FAST_MS);
}

void moveNeck(int target) {
  target = constrain(target, NECK_SOFT_MIN, NECK_SOFT_MAX);
  attachNeck();
  if (abs(target - neckPos) < NECK_MIN_MOVE_DEG) return;   // skip body-kicking twitches
  rampServo(neckServo, neckPos, target, STEP_SLOW_MS, STEP_FAST_MS);
}

// ── True ellipse via scanline ─────────────────────────────────────────────────
void fillEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color) {
  for (int16_t dy = -ry; dy <= ry; dy++) {
    float ratio = 1.0f - ((float)dy * dy) / ((float)ry * ry);
    if (ratio < 0.0f) continue;
    int16_t dx = (int16_t)(rx * sqrtf(ratio));
    display.drawFastHLine(x0 - dx, y0 + dy, 2 * dx + 1, color);
  }
}

// ── Core expression renderer ──────────────────────────────────────────────────
// expression: 0=cute  1=mad  2=cry  3=glare  4=sleep/blink  5=happy
void drawFace(int expression) {
  display.clearDisplay();

  const int cxL = CX_L, cyL = CY_E;
  const int cxR = CX_R, cyR = CY_E;
  const int rx  = RX_E, ry  = RY_E;

  // 1 — white ovals (sclera)
  fillEllipse(cxL, cyL, rx, ry, SSD1306_WHITE);
  fillEllipse(cxR, cyR, rx, ry, SSD1306_WHITE);

  // 2 — black pupils (cross-eyed for cuteness)
  display.fillCircle(cxL + 4, cyL, 6, SSD1306_BLACK);
  display.fillCircle(cxR - 4, cyR, 6, SSD1306_BLACK);

  // 3 — white sparkles (catchlights)
  display.fillCircle(cxL + 6, cyL - 3, 2, SSD1306_WHITE);
  display.fillCircle(cxR - 2, cyR - 3, 2, SSD1306_WHITE);

  // 4 — expression mask drawn OVER everything
  if (expression == 1) {
    // MAD — V-brows: inner corner of each eye drops toward nose
    display.fillTriangle(cxL-20, cyL-30, cxL+20, cyL-30, cxL+20, cyL+5, SSD1306_BLACK); // left  mask  '\'
    display.fillTriangle(cxR-20, cyR-30, cxR+20, cyR-30, cxR-20, cyR+5, SSD1306_BLACK); // right mask  '/'
  }
  else if (expression == 2) {
    // CRY — Λ-brows: outer corner of each eye drops toward ear
    display.fillTriangle(cxL-20, cyL-30, cxL+20, cyL-30, cxL-20, cyL+5, SSD1306_BLACK); // left  mask  '/'
    display.fillTriangle(cxR-20, cyR-30, cxR+20, cyR-30, cxR+20, cyR+5, SSD1306_BLACK); // right mask  '\'
  }
  else if (expression == 3) {
    // GLARE — straight top eyelid covers top 20px of each oval
    display.fillRect(cxL-20, cyL-30, 40, 20, SSD1306_BLACK);
    display.fillRect(cxR-20, cyR-30, 40, 20, SSD1306_BLACK);
  }
  else if (expression == 4) {
    // SLEEP / BLINK — covers almost all oval, leaves tiny bottom crescent
    display.fillRect(cxL-20, cyL-30, 40, 40, SSD1306_BLACK);
    display.fillRect(cxR-20, cyR-30, 40, 40, SSD1306_BLACK);
  }
  else if (expression == 5) {
    // HAPPY — circle wipes out bottom, leaves top crescent squint
    display.fillCircle(cxL, cyL+20, 18, SSD1306_BLACK);
    display.fillCircle(cxR, cyR+20, 18, SSD1306_BLACK);
  }
  // expression == 0: no mask → default cute face

  display.display();
}

// ── Directional-look renderer (shifts whole oval + pupil) ────────────────────
// Used only for look-left/right/up/down — sclera physically moves
void drawLook(int8_t scx, int8_t scy) {
  display.clearDisplay();

  const int16_t oxL = CX_L + scx, oyL = CY_E + scy;
  const int16_t oxR = CX_R + scx, oyR = CY_E + scy;

  fillEllipse(oxL, oyL, RX_E, RY_E, SSD1306_WHITE);
  fillEllipse(oxR, oyR, RX_E, RY_E, SSD1306_WHITE);

  // pupil leads a bit further than sclera
  int8_t pdx = (scx > 0) ? 7 : (scx < 0 ? -7 : 0);
  int8_t pdy = (scy > 0) ? 12 : (scy < 0 ? -12 : 0);

  int16_t pxL = constrain((int16_t)(oxL + pdx), oxL - RX_E + 7, oxL + RX_E - 7);
  int16_t pyL = constrain((int16_t)(oyL + pdy), oyL - RY_E + 8, oyL + RY_E - 8);
  int16_t pxR = constrain((int16_t)(oxR + pdx), oxR - RX_E + 7, oxR + RX_E - 7);
  int16_t pyR = constrain((int16_t)(oyR + pdy), oyR - RY_E + 8, oyR + RY_E - 8);

  display.fillCircle(pxL, pyL, 6, SSD1306_BLACK);
  display.fillCircle(pxL + 3, pyL - 3, 2, SSD1306_WHITE);
  display.fillCircle(pxR, pyR, 6, SSD1306_BLACK);
  display.fillCircle(pxR + 3, pyR - 3, 2, SSD1306_WHITE);

  display.display();
}

// ── Special-case face renderers ───────────────────────────────────────────────

void faceWide() {
  display.clearDisplay();
  fillEllipse(CX_L, CY_E, RX_E, RY_E, SSD1306_WHITE);
  fillEllipse(CX_R, CY_E, RX_E, RY_E, SSD1306_WHITE);
  // tiny centred pupils → lots of white showing = wide / surprised
  display.fillCircle(CX_L, CY_E, 3, SSD1306_BLACK);
  display.fillCircle(CX_R, CY_E, 3, SSD1306_BLACK);
  display.fillCircle(CX_L + 2, CY_E - 2, 2, SSD1306_WHITE);
  display.fillCircle(CX_R + 2, CY_E - 2, 2, SSD1306_WHITE);
  display.display();
}

void faceCrossed() {
  display.clearDisplay();
  fillEllipse(CX_L, CY_E, RX_E, RY_E, SSD1306_WHITE);
  fillEllipse(CX_R, CY_E, RX_E, RY_E, SSD1306_WHITE);
  // both pupils toward nose
  display.fillCircle(CX_L + SH + 2, CY_E, 6, SSD1306_BLACK);
  display.fillCircle(CX_R - SH - 2, CY_E, 6, SSD1306_BLACK);
  display.fillCircle(CX_L + SH + 4, CY_E - 3, 2, SSD1306_WHITE);
  display.fillCircle(CX_R - SH,     CY_E - 3, 2, SSD1306_WHITE);
  display.display();
}

void faceConfused() {
  display.clearDisplay();
  fillEllipse(CX_L, CY_E, RX_E, RY_E, SSD1306_WHITE);
  fillEllipse(CX_R, CY_E, RX_E, RY_E, SSD1306_WHITE);
  // left pupil up, right pupil down
  display.fillCircle(CX_L + 4, CY_E - 5, 6, SSD1306_BLACK);
  display.fillCircle(CX_R - 4, CY_E + 5, 6, SSD1306_BLACK);
  display.fillCircle(CX_L + 6, CY_E - 8, 2, SSD1306_WHITE);
  display.fillCircle(CX_R - 2, CY_E + 2, 2, SSD1306_WHITE);
  display.display();
}

void faceCurious() {
  display.clearDisplay();
  // scleras shift up slightly
  int16_t oyL = CY_E - SSV, oyR = CY_E - SSV;
  fillEllipse(CX_L, oyL, RX_E, RY_E, SSD1306_WHITE);
  fillEllipse(CX_R, oyR, RX_E, RY_E, SSD1306_WHITE);
  // pupils look upward
  display.fillCircle(CX_L + 4, oyL - SV, 6, SSD1306_BLACK);
  display.fillCircle(CX_R - 4, oyR - SV, 6, SSD1306_BLACK);
  display.fillCircle(CX_L + 6, oyL - SV - 3, 2, SSD1306_WHITE);
  display.fillCircle(CX_R - 2, oyR - SV - 3, 2, SSD1306_WHITE);
  // subtle inner-brow shadow on right eye (quizzical)
  int16_t ty = oyR - RY_E - 1;
  display.fillTriangle(CX_R - RX_E - 1, ty,
                       CX_R,             ty,
                       CX_R - RX_E - 1, ty + 8, SSD1306_BLACK);
  display.display();
}

// ── Sleeping face with animated Z's ──────────────────────────────────────────
// Call in a loop with phase 0..7 and ~200ms delay between phases.
// Draws minus-line closed eyes + up to 3 staggered Z's drifting diagonally up-right.
void drawZ(int16_t x, int16_t y, uint8_t s) {
  // top bar
  for (uint8_t i = 0; i < 5; i++) display.fillRect(x+i*s, y, s, s, SSD1306_WHITE);
  // diagonal top-right → bottom-left
  for (uint8_t i = 0; i < 5; i++) display.fillRect(x+(4-i)*s, y+i*s, s, s, SSD1306_WHITE);
  // bottom bar
  for (uint8_t i = 0; i < 5; i++) display.fillRect(x+i*s, y+4*s, s, s, SSD1306_WHITE);
}

// eyeOff: +1 eyes dip slightly, -1 eyes rise — creates gentle breathing bob
void faceSleeping(uint8_t phase, int8_t eyeOff) {
  display.clearDisplay();

  // Closed eyes — two short horizontal dashes, offset by eyeOff for bob effect
  const uint8_t eyeLen = 18;
  int16_t ey = CY_E + eyeOff;
  display.drawFastHLine(CX_L - eyeLen/2, ey,     eyeLen, SSD1306_WHITE);
  display.drawFastHLine(CX_L - eyeLen/2, ey + 1, eyeLen, SSD1306_WHITE);
  display.drawFastHLine(CX_R - eyeLen/2, ey,     eyeLen, SSD1306_WHITE);
  display.drawFastHLine(CX_R - eyeLen/2, ey + 1, eyeLen, SSD1306_WHITE);

  // Z1 — small (size 1), leads the float
  int16_t z1x = 70 + (int16_t)phase * 4;
  int16_t z1y = 20 - (int16_t)phase * 3;
  if (phase < 7 && z1x + 5 < SCREEN_W && z1y > 0) drawZ(z1x, z1y, 1);

  // Z2 — medium (size 2), lags 3 phases behind Z1
  if (phase >= 3) {
    uint8_t p2 = phase - 3;
    int16_t z2x = 72 + (int16_t)p2 * 4;
    int16_t z2y = 14 - (int16_t)p2 * 3;
    if (p2 < 7 && z2x + 10 < SCREEN_W && z2y > -8) drawZ(z2x, z2y, 2);
  }

  // Z3 — large (size 3), lags 6 phases
  if (phase >= 6) {
    uint8_t p3 = phase - 6;
    int16_t z3x = 74 + (int16_t)p3 * 4;
    int16_t z3y =  8 - (int16_t)p3 * 3;
    if (p3 < 4 && z3x + 15 < SCREEN_W && z3y > -16) drawZ(z3x, z3y, 3);
  }

  display.display();
}

// Set OLED contrast (brightness) — 0 = dimmest, 255 = full brightness
// Uses Wire directly — works on all Adafruit SSD1306 library versions.
void setContrast(uint8_t level) {
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x00);           // command mode
  Wire.write(0x81);           // SSD1306 Set Contrast command
  Wire.write(level);
  Wire.endTransmission();
}

// Run sleeping animation for durationMs milliseconds.
// Sleeps in the REST pose (caller already moved there) — no nod-down: the one
// observed brownout reset happened during the old moveHead(65) sleep nod.
// Servos are released one at a time, then only the OLED (Z's + eye bob) animates.
// Eyes bob up 1px on exhale (even phase), down 1px on inhale (odd phase).
void animSleepingFor(uint32_t durationMs) {
  savePosition(EEPROM_ADDR_NECK, neckPos);   // persist the neck pose we power down in
  releaseNeck();    // head stays attached — released, it drops under the hat's weight

  setCrumb(CRUMB_SLEEP, 1);
  uint32_t start = millis();
  uint8_t  phase = 0;
  while (millis() - start < durationMs) {
    int8_t eyeOff = (phase & 1) ? 1 : -1;   // odd=inhale dip, even=exhale rise
    faceSleeping(phase, eyeOff);
    delay(200);
    phase = (phase + 1) & 7;                 // wrap 0..7
  }
  // Re-attach the neck at its rest pose (head never let go)
  setCrumb(CRUMB_WAKE, 0);
  attachNeck();
}


void faceHello() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(34, 24);
  display.print(F("HELLO"));
  display.display();
}

void faceBye() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(3);
  display.setCursor(37, 20);
  display.print(F("BYE"));
  display.display();
}

// ── Convenience wrappers ──────────────────────────────────────────────────────
void faceCute()  { drawFace(0); }
void faceMad()   { drawFace(1); }
void faceCry()   { drawFace(2); }
void faceGlare() { drawFace(3); }
void faceSleep() { drawFace(4); }
void faceBlink() { drawFace(4); }
void faceHappy() { drawFace(5); }

// ── Animations ────────────────────────────────────────────────────────────────
void animBlink() {
  faceBlink();
  delay(80);
  faceCute();
}

void animHappyBlink() {
  faceHappy();
  delay(250);
  faceBlink();
  delay(80);
  faceHappy();
  delay(350);
  faceCute();
}

// ── Behaviour routines ────────────────────────────────────────────────────────
void behaveIdle() {
  faceCute();
  int sway = NECK_REST + (int)random(-8, 9);
  moveNeck(sway);
  delay(500);
  moveNeck(NECK_REST);
}

void behaveBlink() {
  animBlink();
  delay(150);
}

// Servo positions per expression spec:
//   Look Left:      Head 80, Neck  0
//   Look Right:     Head 80, Neck 70
//   Look Down:      Head 65, Neck 35
//   Look Up:        Head 87, Neck 35
//   Wide/Surprised: Head 85, Neck 35
//   Blink:          Head 80, Neck 35
//   Happy:          Head nod 70↔85, Neck 35
//   Mad:            Head 85, Neck 35
//   Cry:            Head 75, Neck 35
//   Glare:          look R then L, Neck 35
//   Sleep breathing: Head 65↔70, Neck 35 (handled in animSleepingFor)

// These functions SET UP the face + servo and return immediately.
// holdWithLife() holds for 4–7 s with emotion-aware micro-movements.

void behaveLookLeft() {
  moveHead(80);
  moveNeck(NECK_SOFT_MIN);   // was 5 — kept off the end stop
  drawLook(-SSH, 0);
}

void behaveLookRight() {
  moveHead(80);
  moveNeck(NECK_SOFT_MAX);   // was 70 — kept off the end stop
  drawLook(SSH, 0);
}

// Eyes only — holding the hat tilted back (87°) reset the board mid-hold.
// The head is already at REST here (interBlink() / bucket pause put it there).
void behaveLookUp() {
  drawLook(0, -SSV);
}

void behaveLookDown() {
  moveHead(HEAD_SOFT_MIN);   // was 65 — shallower dip
  moveNeck(35);
  drawLook(0, SSV);
}

void behaveHappy() {
  moveNeck(35);
  faceHappy();
  moveHead(74);   // two gentle nods (was three 70↔85 swings)
  moveHead(82);
  moveHead(74);
  moveHead(HEAD_REST);
  // face stays happy — no blink/reset here
}

void behaveSurprised() {
  moveHead(85);
  moveNeck(35);
  faceWide();
  // face stays wide — no blink/reset here
}

void behaveConfused() {
  moveNeck(35);
  faceConfused();
  moveNeck(NECK_MIN + 15);
  moveNeck(NECK_MAX - 15);
  moveNeck(NECK_REST);
  // face stays confused — no reset here
}

void behaveMad() {
  moveHead(85);
  moveNeck(35);
  faceMad();
  delay(200);
  moveNeck(30);
  moveNeck(40);
  moveNeck(35);
  // face stays mad — no blink/reset here
}

void behaveGlare() {
  moveNeck(35);
  faceGlare();
  drawLook(SSH,  0); delay(600);
  drawLook(-SSH, 0); delay(600);
  faceGlare();
  // face stays glare — no reset here
}

void behaveCry() {
  moveHead(75);
  moveNeck(35);
  faceCry();
  // face stays crying — no blink/reset here
}

void behaveCurious() {
  moveNeck(NECK_MAX - 15);
  faceCurious();
  // face stays curious — no blink/reset here
}

void behaveWord() {
  switch ((int)random(2)) {
    case 0:
      faceHello();
      moveNeck(NECK_MIN + 10);
      delay(300);
      moveNeck(NECK_MAX - 10);
      delay(300);
      moveNeck(NECK_REST);
      break;
    case 1:
      faceBye();
      moveNeck(NECK_MIN + 10);
      delay(300);
      moveNeck(NECK_MAX - 10);
      delay(300);
      moveNeck(NECK_REST);
      break;
  }
  delay(400);
  animBlink();
}

// ── Bucket runners ────────────────────────────────────────────────────────────
// Each behave*() sets the face+servo and returns.
// holdWithLife(id): holds for 4–7 s with emotion-aware micro-movements.
// interBlink():     resets to cute, blinks 2–3 times with 2–4 s gaps.

// Expression IDs passed to holdWithLife()
#define EXPR_CUTE       0
#define EXPR_LOOK_LEFT  1
#define EXPR_LOOK_RIGHT 2
#define EXPR_LOOK_UP    3
#define EXPR_LOOK_DOWN  4
#define EXPR_HAPPY      5
#define EXPR_SURPRISED  6
#define EXPR_MAD        7
#define EXPR_CRY        8
#define EXPR_GLARE      9
#define EXPR_CURIOUS    10

// Helper: shuffle an array of n bytes in-place (Fisher-Yates)
void shuffleBytes(uint8_t *arr, uint8_t n) {
  for (uint8_t i = n - 1; i > 0; i--) {
    uint8_t j = (uint8_t)random(i + 1);
    uint8_t tmp = arr[i]; arr[i] = arr[j]; arr[j] = tmp;
  }
}

// Small non-blocking wait — just delay in small slices to keep things readable
// (Arduino is single-threaded; all micro-movements are sequential servo steps)

void holdWithLife(uint8_t exprId) {
  crumbHold();
  uint32_t holdMs   = (uint32_t)random(3000, 5001);
  uint32_t deadline = millis() + holdMs;

  while (millis() < deadline) {
    uint32_t remaining = deadline - millis();

    switch (exprId) {

      // ── Mad: periodic neck twitches, slight head nod ──────────────────────
      case EXPR_MAD: {
        // random pause then twitch
        uint32_t wait = (uint32_t)random(500, 1200);
        if (wait > remaining) { delay(remaining); return; }
        delay(wait);
        if (millis() >= deadline) return;
        // tiny neck flick one way
        int dir = (random(2) == 0) ? -6 : 6;
        moveNeck(35 + dir);
        moveNeck(35);
        break;
      }

      // ── Cry: slow rhythmic head dips (sob pattern) ────────────────────────
      case EXPR_CRY: {
        uint32_t wait = (uint32_t)random(700, 1400);
        if (wait > remaining) { delay(remaining); return; }
        delay(wait);
        if (millis() >= deadline) return;
        // dip head down and back
        moveHead(72);
        moveHead(75);
        break;
      }

      // ── Glare: slow menacing side scan ────────────────────────────────────
      case EXPR_GLARE: {
        // glide head slightly, update pupil direction
        uint32_t wait = (uint32_t)random(800, 1600);
        if (wait > remaining) { delay(remaining); return; }
        delay(wait);
        if (millis() >= deadline) return;
        // alternate slow scan
        if (random(2) == 0) {
          moveHead(82); drawLook(SSH, 0);
        } else {
          moveHead(78); drawLook(-SSH, 0);
        }
        faceGlare();  // redraw mask/lids over new head pos
        break;
      }

      // ── Curious: gentle neck sway, slight head tilt ───────────────────────
      case EXPR_CURIOUS: {
        uint32_t wait = (uint32_t)random(600, 1300);
        if (wait > remaining) { delay(remaining); return; }
        delay(wait);
        if (millis() >= deadline) return;
        // sway neck around current 55° position
        int sway = (int)(NECK_MAX - 15) + (int)random(-5, 6);
        sway = constrain(sway, NECK_MIN, NECK_MAX);
        moveNeck((int8_t)sway);
        break;
      }

      // ── Happy: occasional soft double-nod ────────────────────────────────
      case EXPR_HAPPY: {
        uint32_t wait = (uint32_t)random(900, 2000);
        if (wait > remaining) { delay(remaining); return; }
        delay(wait);
        if (millis() >= deadline) return;
        // single gentle nod
        moveHead(75);
        moveHead(HEAD_REST);
        break;
      }

      // ── Surprised: slight head tremor then settle ─────────────────────────
      case EXPR_SURPRISED: {
        uint32_t wait = (uint32_t)random(600, 1500);
        if (wait > remaining) { delay(remaining); return; }
        delay(wait);
        if (millis() >= deadline) return;
        // eyes-only flicker — the old 87/83/85 head shiver sat at the top of travel
        faceBlink();
        delay(80);
        faceWide();
        break;
      }

      // ── Looks: eyes-only life (blink) — no neck / head micro-sway ────────
      // Neck sways at the travel ends kicked the body around; head sways
      // while looking up/down loaded the head servo (resets).
      case EXPR_LOOK_LEFT:
      case EXPR_LOOK_RIGHT:
      case EXPR_LOOK_UP:
      case EXPR_LOOK_DOWN: {
        uint32_t wait = (uint32_t)random(1000, 2200);
        if (wait > remaining) { delay(remaining); return; }
        delay(wait);
        if (millis() >= deadline) return;
        faceBlink();
        delay(80);
        if      (exprId == EXPR_LOOK_LEFT)  drawLook(-SSH, 0);
        else if (exprId == EXPR_LOOK_RIGHT) drawLook( SSH, 0);
        else if (exprId == EXPR_LOOK_UP)    drawLook(0, -SSV);
        else                                drawLook(0,  SSV);
        break;
      }

      // ── Cute / default: gentle idle sway ─────────────────────────────────
      default: {
        uint32_t wait = (uint32_t)random(1000, 2500);
        if (wait > remaining) { delay(remaining); return; }
        delay(wait);
        if (millis() >= deadline) return;
        int sway = NECK_REST + (int)random(-6, 7);
        moveNeck(constrain(sway, NECK_MIN + 5, NECK_MAX - 5));
        moveNeck(NECK_REST);
        break;
      }
    }
  }
}

void interBlink() {
  setCrumb(CRUMB_BLINKS, 0);
  faceCute(); moveHead(HEAD_REST); moveNeck(NECK_REST);
  // servos stay attached — idle holding draws little, re-attach slams draw a lot
  crumbHold();
  uint8_t n = (uint8_t)random(2, 4);   // 2 or 3 blinks
  for (uint8_t i = 0; i < n; i++) {
    delay((uint32_t)random(2000, 4001));
    animBlink();
  }
}

void runBucket1(uint32_t deadline) {
  // HAPPY / UPBEAT mood — looks (all 4) + happy + curious + surprised
  uint8_t seq[7] = {0, 1, 2, 3, 4, 5, 6};
  shuffleBytes(seq, 7);
  for (uint8_t i = 0; i < 7; i++) {
    if (millis() >= deadline) return;
    uint8_t exprId;
    switch (seq[i]) {
      case 0: setCrumb(CRUMB_LOOK_L, 0); behaveLookLeft();   exprId = EXPR_LOOK_LEFT;  break;
      case 1: setCrumb(CRUMB_LOOK_R, 0); behaveLookRight();  exprId = EXPR_LOOK_RIGHT; break;
      case 2: setCrumb(CRUMB_LOOK_UP, 0); behaveLookUp();     exprId = EXPR_LOOK_UP;    break;
      case 3: setCrumb(CRUMB_LOOK_DN, 0); behaveLookDown();   exprId = EXPR_LOOK_DOWN;  break;
      case 4: setCrumb(CRUMB_HAPPY, 0); behaveHappy();      exprId = EXPR_HAPPY;      break;
      case 5: setCrumb(CRUMB_CURIOUS, 0); behaveCurious();    exprId = EXPR_CURIOUS;    break;
      case 6: setCrumb(CRUMB_SURPRISED, 0); behaveSurprised();  exprId = EXPR_SURPRISED;  break;
      default: exprId = EXPR_CUTE; break;
    }
    holdWithLife(exprId);
    if (millis() >= deadline) return;
    interBlink();
  }
}

void runBucket2(uint32_t deadline) {
  // UPSET / NEGATIVE mood — looks (all 4) + mad + cry + glare
  uint8_t seq[7] = {0, 1, 2, 3, 4, 5, 6};
  shuffleBytes(seq, 7);
  for (uint8_t i = 0; i < 7; i++) {
    if (millis() >= deadline) return;
    uint8_t exprId;
    switch (seq[i]) {
      case 0: setCrumb(CRUMB_LOOK_L, 0); behaveLookLeft();   exprId = EXPR_LOOK_LEFT;  break;
      case 1: setCrumb(CRUMB_LOOK_R, 0); behaveLookRight();  exprId = EXPR_LOOK_RIGHT; break;
      case 2: setCrumb(CRUMB_LOOK_UP, 0); behaveLookUp();     exprId = EXPR_LOOK_UP;    break;
      case 3: setCrumb(CRUMB_LOOK_DN, 0); behaveLookDown();   exprId = EXPR_LOOK_DOWN;  break;
      case 4: setCrumb(CRUMB_MAD, 0); behaveMad();        exprId = EXPR_MAD;        break;
      case 5: setCrumb(CRUMB_CRY, 0); behaveCry();        exprId = EXPR_CRY;        break;
      case 6: setCrumb(CRUMB_GLARE, 0); behaveGlare();      exprId = EXPR_GLARE;      break;
      default: exprId = EXPR_CUTE; break;
    }
    holdWithLife(exprId);
    if (millis() >= deadline) return;
    interBlink();
  }
}

void runBucket3(uint32_t deadline) {
  // CALM / NEUTRAL mood — just looking around, no strong emotion overlay
  uint8_t seq[5] = {0, 1, 2, 3, 4};
  shuffleBytes(seq, 5);
  for (uint8_t i = 0; i < 5; i++) {
    if (millis() >= deadline) return;
    uint8_t exprId;
    switch (seq[i]) {
      case 0: setCrumb(CRUMB_LOOK_L, 0); behaveLookLeft();                                          exprId = EXPR_LOOK_LEFT;  break;
      case 1: setCrumb(CRUMB_LOOK_R, 0); behaveLookRight();                                         exprId = EXPR_LOOK_RIGHT; break;
      case 2: setCrumb(CRUMB_LOOK_UP, 0); behaveLookUp();                                            exprId = EXPR_LOOK_UP;    break;
      case 3: setCrumb(CRUMB_LOOK_DN, 0); behaveLookDown();                                          exprId = EXPR_LOOK_DOWN;  break;
      case 4: setCrumb(CRUMB_CUTE, 0); faceCute(); moveHead(HEAD_REST); moveNeck(NECK_REST); exprId = EXPR_CUTE; break;
      default: exprId = EXPR_CUTE; break;
    }
    holdWithLife(exprId);
    if (millis() >= deadline) return;
    interBlink();
  }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  randomSeed(analogRead(A3));

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (true) {}
  }
  display.clearDisplay();
  display.display();
  setContrast(150);  // conservative active brightness — reduces burn-in
  reportBoot();      // POWER ON, or RESET #n + which behaviour it happened in

  // Any boot follows a power-off or reset, so the unpowered head has already
  // dropped under the hat's weight. Attach it where it physically is
  // (HEAD_DROOP) and lift slowly — attaching at REST snapped the head up in
  // ~0.1 s and reset the board (RESET #12 in the 2026-10-05 video).
  // The neck doesn't fall, so it still sweeps in from its last saved angle.
  headPos = HEAD_DROOP;
  neckPos = loadPosition(EEPROM_ADDR_NECK, NECK_MIN, NECK_MAX, NECK_REST);

  moveHead(HEAD_REST);   // attach at HEAD_DROOP, slow eased lift to REST
  moveNeck(NECK_REST);   // waits ATTACH_GAP_MS, attaches at neckPos, sweeps to REST
  delay(200);            // both stay attached into the active window

  // Wake-up greeting
  faceHello(); delay(1200);
  faceWide();  delay(600);
  faceCute();  delay(400);
}

// ── loop — active buckets → Bye → 30s sleep → Hello → repeat ─────────────────
void loop() {
  // ── ACTIVE WINDOW: 60 seconds of expressions ─────────────────────────────────
  uint32_t activeDeadline = millis() + 60000UL;

  // Shuffled bucket order — keep cycling buckets until time runs out
  uint8_t order[3] = {0, 1, 2};
  for (uint8_t i = 2; i > 0; i--) {
    uint8_t j = (uint8_t)random(i + 1);
    uint8_t tmp = order[i]; order[i] = order[j]; order[j] = tmp;
  }

  for (uint8_t i = 0; i < 3; i++) {
    if (millis() >= activeDeadline) break;
    switch (order[i]) {
      case 0: runBucket1(activeDeadline); break;
      case 1: runBucket2(activeDeadline); break;
      case 2: runBucket3(activeDeadline); break;
    }
    // brief neutral pause between buckets (only if time remains)
    if (millis() < activeDeadline) {
      setCrumb(CRUMB_PAUSE, 0);
      faceCute();
      moveHead(HEAD_REST);
      moveNeck(NECK_REST);
      delay(1000);
    }
  }

  // ── SLEEP TRANSITION ─────────────────────────────────────────────────────────
  setCrumb(CRUMB_SLEEP_IN, 0);
  faceCute();
  moveHead(HEAD_REST);
  moveNeck(NECK_REST);
  delay(300);

  faceBye();
  delay(1500);
  setContrast(70);   // dim for sleep — reduces burn-in on sleeping pixels

  // ── SLEEP WINDOW (30 seconds) ─────────────────────────────────────────────────
  animSleepingFor(30000UL);
  setContrast(150); // restore active brightness on wake

  // ── WAKE TRANSITION ──────────────────────────────────────────────────────────
  faceCute();
  delay(300);
  faceHello();
  delay(1200);
  faceWide();  delay(500);
  faceCute();  delay(400);
}
