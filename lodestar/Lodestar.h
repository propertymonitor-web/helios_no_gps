// Lodestar - mobile-repeater uplink scoring for MeshCore.
// By FEAR REACTIVE.
//
// Header-only and free of Arduino / MeshCore dependencies so the logic can be
// unit-tested on a PC (see lodestar/test). The MeshCore glue lives in
// tools/apply_lodestar.py, which patches examples/simple_repeater.
//
// What it does
//   * Keeps a table of the repeaters this node can hear directly.
//   * Scores each one, in order of importance:
//       1. reliability  - implicit ACKs (did it re-flood what we sent?) blended
//                         with how consistently we hear it
//       2. link quality - smoothed SNR (relative to the LoRa floor for our SF)
//                         and RSSI
//       3. hop count    - how many hops beyond that repeater its traffic reaches
//   * Picks a best uplink with hysteresis so a moving node does not flap.
//   * Gives the glue two decisions to act on: a retransmit-delay factor
//     (better placed => transmit sooner) and a duplicate-cancel check.
//   * Never blocks forwarding. If no usable uplink exists it behaves like the
//     stock repeater (delay factor 1.0).
//   * Tracks the node's own position (fix, speed, course) from the GPS. For now
//     this is read-only plumbing: it does not change forwarding decisions.
#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lodestar {

#ifndef LODESTAR_MAX_UPLINKS
#define LODESTAR_MAX_UPLINKS 24      // table size (max 32: masks are 32-bit)
#endif
#define LODESTAR_TOPK        3       // size of the "current uplink set"
#define LODESTAR_MAX_PENDING 8       // flood packets awaiting an implicit ACK
#define LODESTAR_HASH_LEN    8       // == MeshCore MAX_HASH_SIZE
#define LODESTAR_MAX_PATH    64      // == MeshCore MAX_PATH_SIZE
#define LODESTAR_SLOT_MS     20000u  // one "was it heard" slot
#define LODESTAR_SLOTS       16      // slots in the sliding reliability window

struct Config {
  bool     enabled      = true;
  float    w_rel        = 0.5f;      // reliability
  float    w_lq         = 0.3f;      // link quality
  float    w_hop        = 0.2f;      // hop count
  float    floor_score  = 0.25f;     // below this an uplink is not usable
  float    switch_margin= 0.08f;     // challenger must beat the best by this
  uint8_t  switch_evals = 2;         // ...for this many evaluations in a row
  uint32_t stale_start_ms = 120000;  // full credit until this age
  uint32_t stale_end_ms   = 600000;  // zero credit at this age
  uint32_t expire_ms      = 1800000; // entry dropped after this age
  bool     cancel_dup   = true;      // drop our queued retransmit if a strong uplink already sent it
  float    cancel_min   = 0.60f;     // ...and its score is at least this
  float    cancel_min_lq= 0.50f;     // ...and the link to it is at least this good
  uint8_t  sf           = 10;        // spreading factor, for the SNR floor
  uint32_t gps_fresh_ms = 10000;     // a GPS fix older than this no longer counts
  float    min_move_mps = 1.5f;      // slower than this counts as stopped (GPS jitter at rest)
};

// Where this node is, fed from the GPS by the glue. Speed and course are derived
// from successive fixes, so no extra GPS driver support is needed. Position is
// kept separate from the uplink table: `lodestar reset` does not clear it.
struct Position {
  bool     valid;                    // a fix has been seen since the GPS last lost it
  int32_t  lat_e6, lon_e6;           // degrees * 1e6
  uint8_t  sats;
  uint32_t fix_ms;                   // device millis of the latest accepted fix
  float    speed_mps;                // smoothed, metres per second
  float    course_deg;               // 0 = north, 90 = east; held while stopped
  bool     course_valid;
};

struct Uplink {
  bool     used;
  uint8_t  key[3];                   // leading bytes of the repeater's public key
  uint8_t  key_len;                  // how many of them are known (1..3)
  uint32_t first_ms, last_ms;        // first / last time heard (device millis)
  float    snr, rssi;                // smoothed
  bool     lq_init;
  uint32_t slot_bits;                // bit 0 = most recent slot
  uint32_t last_slot;
  uint32_t slots_seen;               // slots since first heard (capped)
  float    ack;                      // EWMA implicit-ACK success, 0..1
  uint8_t  ack_n;                    // samples behind it (capped)
  uint32_t hits, misses;
  float    hop;                      // EWMA hops beyond this repeater
  uint8_t  hop_n;
  uint32_t heard_count;
};

struct Pending {                     // a flood packet we sent, waiting to be echoed
  bool     used;
  uint8_t  hash[LODESTAR_HASH_LEN];
  uint8_t  path[LODESTAR_MAX_PATH];
  uint8_t  count, sz;
  uint32_t tx_ms, window_ms;
  uint32_t cand_mask;                // uplinks that were in range when we sent
  uint32_t hit_mask;
};

class Lodestar {
public:
  Config cfg;

  Lodestar() { reset(); }

  void reset() {
    memset(_t, 0, sizeof(_t));
    memset(_p, 0, sizeof(_p));
    _best = -1; _best_score = 0; _challenger = -1; _chal_count = 0;
    _topn = 0; _last_eval = 0; _eval_valid = false;
    _cancels = 0; _tx_tracked = 0;
    for (int i = 0; i < LODESTAR_MAX_UPLINKS; i++) _score[i] = 0;
  }

  // ------------------------------------------------------------ inputs --

  // A zero-hop repeater advert was heard directly.
  int onAdvert(const uint8_t* pubkey, float snr, float rssi, uint32_t now) {
    int i = resolve(pubkey, 3, true, now);
    if (i < 0) return -1;
    touch(i, snr, rssi, now);
    return i;
  }

  // A flood packet arrived whose last hop was `hash` (we heard that repeater directly).
  int onHeardFrom(const uint8_t* hash, uint8_t sz, float snr, float rssi, uint32_t now) {
    int i = resolve(hash, sz, true, now);
    if (i < 0) return -1;
    touch(i, snr, rssi, now);
    return i;
  }

  // A flood advert from a far repeater arrived via uplink `idx` after `extra_hops`
  // hops beyond it. Fewer hops to the same mesh => better connected uplink.
  void noteReach(int idx, uint8_t extra_hops) {
    if (idx < 0 || idx >= LODESTAR_MAX_UPLINKS || !_t[idx].used) return;
    Uplink& u = _t[idx];
    u.hop = u.hop_n == 0 ? (float)extra_hops : 0.8f * u.hop + 0.2f * (float)extra_hops;
    if (u.hop_n < 255) u.hop_n++;
  }

  // We transmitted a flood packet. `path` is the path as sent (for a forward, it
  // already ends with our own hash; for our own packets it is empty).
  void onTx(const uint8_t* hash, const uint8_t* path, uint8_t count, uint8_t sz,
            uint32_t now, uint32_t window_ms) {
    if (!cfg.enabled || count >= LODESTAR_MAX_PATH / (sz ? sz : 1)) return;
    uint32_t mask = 0;
    for (int i = 0; i < LODESTAR_MAX_UPLINKS; i++) {
      if (_t[i].used && elapsed(now, _t[i].last_ms) <= cfg.stale_start_ms) mask |= (1u << i);
    }
    if (mask == 0) return;           // nobody to expect an echo from
    int slot = -1;
    uint32_t oldest = 0;
    for (int k = 0; k < LODESTAR_MAX_PENDING; k++) {
      if (!_p[k].used) { slot = k; break; }
      uint32_t age = elapsed(now, _p[k].tx_ms);
      if (slot < 0 || age > oldest) { slot = k; oldest = age; }
    }
    if (_p[slot].used) settle(slot);   // evicting an unsettled one counts it
    Pending& p = _p[slot];
    p.used = true;
    memcpy(p.hash, hash, LODESTAR_HASH_LEN);
    p.count = count; p.sz = sz;
    memcpy(p.path, path, (size_t)count * sz);
    p.tx_ms = now; p.window_ms = window_ms;
    p.cand_mask = mask; p.hit_mask = 0;
    _tx_tracked++;
  }

  // A flood packet was heard. `from` is the uplink index of its last hop (or -1).
  // Credits an implicit ACK when it is an echo of something we sent.
  // Returns true when the glue should cancel our own queued copy of this packet
  // (a strong uplink has already forwarded it).
  bool onFloodCopy(const uint8_t* hash, const uint8_t* path, uint8_t count, uint8_t sz,
                   int from, uint32_t now) {
    if (from < 0 || from >= LODESTAR_MAX_UPLINKS || !_t[from].used) return false;
    for (int k = 0; k < LODESTAR_MAX_PENDING; k++) {
      Pending& p = _p[k];
      if (!p.used || p.sz != sz || count != p.count + 1) continue;
      if (memcmp(p.hash, hash, LODESTAR_HASH_LEN) != 0) continue;
      if (p.count && memcmp(p.path, path, (size_t)p.count * sz) != 0) continue;
      uint32_t bit = 1u << from;
      if ((p.cand_mask & bit) && !(p.hit_mask & bit)) {
        p.hit_mask |= bit;
        ackSample(from, true);
      }
    }
    if (!cfg.enabled || !cfg.cancel_dup) return false;
    evaluate(now);
    return _score[from] >= cfg.cancel_min && linkQuality(from) >= cfg.cancel_min_lq;
  }

  void noteCancel() { _cancels++; }

  // The GPS reported its state. Call as often as you like (every loop is fine);
  // fixes are taken at most once a second. `fix` false means no usable fix.
  //   speed  - measured over ~3 s windows (steadier than fix-to-fix at 1 Hz) and
  //            lightly smoothed; the first window after a (re)start is taken as is
  //   course - only updates after moving at least 8 m from the last course point,
  //            so GPS jitter at rest does not invent a heading, while a slow
  //            walker still gets one every few seconds
  void onPosition(int32_t lat_e6, int32_t lon_e6, uint8_t sats, bool fix, uint32_t now) {
    if (!fix) {                                   // lost it: do not measure across the gap
      _pos.valid = false; _pos.speed_mps = 0; _pos.course_valid = false;
      _have_anchor = false; _have_speed = false;
      return;
    }
    bool restart = !_pos.valid || elapsed(now, _pos.fix_ms) > cfg.gps_fresh_ms;
    if (!restart && elapsed(now, _pos.fix_ms) < 1000) return;   // GPS is 1 Hz
    _pos.lat_e6 = lat_e6; _pos.lon_e6 = lon_e6; _pos.sats = sats; _pos.fix_ms = now;
    if (restart || !_have_anchor) {
      _pos.valid = true; _pos.speed_mps = 0; _pos.course_valid = false; _have_speed = false;
      _anchor_lat = lat_e6; _anchor_lon = lon_e6; _anchor_ms = now;
      _course_lat = lat_e6; _course_lon = lon_e6;
      _have_anchor = true;
      return;
    }
    double dx, dy;
    offsetM(_course_lat, _course_lon, lat_e6, lon_e6, dx, dy);
    if (sqrt(dx * dx + dy * dy) >= 8.0) {
      double b = atan2(dx, dy) * 180.0 / M_PI;
      if (b < 0) b += 360.0;
      _pos.course_deg = (float)b; _pos.course_valid = true;
      _course_lat = lat_e6; _course_lon = lon_e6;
    }
    uint32_t dt = elapsed(now, _anchor_ms);
    if (dt >= 3000) {
      offsetM(_anchor_lat, _anchor_lon, lat_e6, lon_e6, dx, dy);
      float v = (float)(sqrt(dx * dx + dy * dy) / ((double)dt / 1000.0));
      _pos.speed_mps = _have_speed ? 0.5f * _pos.speed_mps + 0.5f * v : v;
      _have_speed = true;
      _anchor_lat = lat_e6; _anchor_lon = lon_e6; _anchor_ms = now;
    }
  }

  // Housekeeping; call from loop().
  void loop(uint32_t now) {
    for (int k = 0; k < LODESTAR_MAX_PENDING; k++) {
      if (_p[k].used && elapsed(now, _p[k].tx_ms) > _p[k].window_ms) settle(k);
    }
    for (int i = 0; i < LODESTAR_MAX_UPLINKS; i++) {
      if (_t[i].used && elapsed(now, _t[i].last_ms) > cfg.expire_ms) {
        memset(&_t[i], 0, sizeof(Uplink));
        _eval_valid = false;
        if (_best == i) { _best = -1; _best_score = 0; }
      }
    }
    evaluate(now);
  }

  // --------------------------------------------------------- decisions --

  // Multiplier for the stock random retransmit delay.
  //   no usable uplink or disabled -> 1.0 (stock behaviour)
  //   best-placed                   -> 0.4  (go first)
  //   barely connected              -> up to 1.6 (yield to better-placed repeaters)
  float delayFactor(uint32_t now) {
    if (!cfg.enabled) return 1.0f;
    evaluate(now);
    if (_best < 0 || _best_score < cfg.floor_score) return 1.0f;
    return 0.4f + 1.2f * (1.0f - clamp01(_best_score));
  }

  // --------------------------------------------------------- inspection --

  int   best() const { return _best; }
  float bestScore() const { return _best_score; }
  int   topCount() const { return _topn; }
  int   topAt(int n) const { return (n >= 0 && n < _topn) ? _top[n] : -1; }
  uint32_t cancels() const { return _cancels; }
  const Uplink& entry(int i) const { return _t[i]; }
  float scoreOf(int i, uint32_t now) { evaluate(now); return _score[i]; }
  int   count() const { int n = 0; for (int i = 0; i < LODESTAR_MAX_UPLINKS; i++) n += _t[i].used; return n; }

  const Position& position() const { return _pos; }
  bool  hasFix(uint32_t now) const { return _pos.valid && elapsed(now, _pos.fix_ms) <= cfg.gps_fresh_ms; }
  bool  moving(uint32_t now) const { return hasFix(now) && _pos.speed_mps >= cfg.min_move_mps; }

  float reliability(int i, uint32_t now) const {
    const Uplink& u = _t[i];
    float rin = rIn(u, now);
    if (u.ack_n >= 3) return 0.6f * u.ack + 0.4f * rin;
    return rin;
  }
  float linkQuality(int i) const {
    const Uplink& u = _t[i];
    if (!u.lq_init) return 0;
    float floor_db = -7.5f - 2.5f * ((float)cfg.sf - 7.0f);
    float snrq  = clamp01((u.snr - floor_db) / 20.0f);
    float rssiq = clamp01((u.rssi + 130.0f) / 70.0f);
    return 0.7f * snrq + 0.3f * rssiq;
  }
  float hopScore(int i) const {
    const Uplink& u = _t[i];
    if (u.hop_n == 0) return 0.5f;        // nothing learned yet: neutral
    return 1.0f / (1.0f + u.hop);
  }
  float freshness(int i, uint32_t now) const {
    uint32_t age = elapsed(now, _t[i].last_ms);
    if (age <= cfg.stale_start_ms) return 1.0f;
    if (age >= cfg.stale_end_ms) return 0.0f;
    return 1.0f - (float)(age - cfg.stale_start_ms) / (float)(cfg.stale_end_ms - cfg.stale_start_ms);
  }

  // ------------------------------------------------------------ output --

  // One summary line for a remote-admin reply.
  void summary(char* out, size_t cap, uint32_t now) {
    evaluate(now);
    if (!cfg.enabled) { snprintf(out, cap, "Lodestar OFF (stock forwarding)"); return; }
    if (_best < 0) {
      snprintf(out, cap, "Lodestar: %d heard, no usable uplink (stock delay)", count());
      return;
    }
    const Uplink& u = _t[_best];
    snprintf(out, cap, "Lodestar: best %02X sc%d rel%d lq%d, %d heard, delay x%.2f, cancels %lu",
             u.key[0], pct(_best_score), pct(reliability(_best, now)), pct(linkQuality(_best)),
             count(), (double)delayFactor(now), (unsigned long)_cancels);
  }

  // GPS status for a reply. No coordinates: a reply can travel over the mesh to a
  // remote admin, and where you are is not for broadcasting.
  void gpsStatus(char* out, size_t cap, uint32_t now) const {
    if (!hasFix(now)) { snprintf(out, cap, "GPS: no fix"); return; }
    if (moving(now) && _pos.course_valid) {
      snprintf(out, cap, "GPS: fix, %d sats, moving %d km/h, course %d",
               (int)_pos.sats, (int)(_pos.speed_mps * 3.6f + 0.5f), (int)(_pos.course_deg + 0.5f) % 360);
    } else {
      snprintf(out, cap, "GPS: fix, %d sats, %s", (int)_pos.sats, moving(now) ? "moving" : "stopped");
    }
  }

  // Full position line, coordinates included. Only ever sent to the local serial port.
  bool gpsDetail(char* out, size_t cap, uint32_t now) const {
    if (!hasFix(now)) return false;
    snprintf(out, cap, "pos %.6f,%.6f sats %d fix age %lus speed %.1f m/s course %d",
             (double)_pos.lat_e6 / 1e6, (double)_pos.lon_e6 / 1e6, (int)_pos.sats,
             (unsigned long)(elapsed(now, _pos.fix_ms) / 1000), (double)_pos.speed_mps,
             _pos.course_valid ? (int)(_pos.course_deg + 0.5f) % 360 : -1);
    return true;
  }

  // One table row; returns false when the row is unused.
  bool row(int i, char* out, size_t cap, uint32_t now) {
    if (i < 0 || i >= LODESTAR_MAX_UPLINKS || !_t[i].used) return false;
    evaluate(now);
    const Uplink& u = _t[i];
    char tag = (i == _best) ? '*' : ' ';
    for (int k = 0; k < _topn; k++) if (_top[k] == i && i != _best) tag = '+';
    char id[8];
    if (u.key_len >= 3) snprintf(id, sizeof(id), "%02X%02X%02X", u.key[0], u.key[1], u.key[2]);
    else if (u.key_len == 2) snprintf(id, sizeof(id), "%02X%02X", u.key[0], u.key[1]);
    else snprintf(id, sizeof(id), "%02X", u.key[0]);
    snprintf(out, cap, "%c%-6s sc%3d rel%3d lq%3d hp%3d snr%5.1f rssi%4d age%4lus ack%lu/%lu",
             tag, id, pct(_score[i]), pct(reliability(i, now)), pct(linkQuality(i)),
             pct(hopScore(i)), (double)u.snr, (int)u.rssi,
             (unsigned long)(elapsed(now, u.last_ms) / 1000),
             (unsigned long)u.hits, (unsigned long)(u.hits + u.misses));
    return true;
  }

  // ---- saving settings across reboots --------------------------------------
  // The glue stores this one line in a file and hands it back at start-up. Only the
  // settings a person can change are saved; the uplink table is rebuilt from what is heard.
  bool settings_dirty = false;   // set by any command that changes a setting; the glue clears it after saving

  size_t saveSettings(char* out, size_t cap) const {
    int n = snprintf(out, cap, "LODESTAR1 %d %.4f %.4f %.4f %.4f %d %.4f\n", cfg.enabled ? 1 : 0,
                     (double)cfg.w_rel, (double)cfg.w_lq, (double)cfg.w_hop, (double)cfg.floor_score,
                     cfg.cancel_dup ? 1 : 0, (double)cfg.cancel_min);
    return (n < 0 || (size_t)n >= cap) ? 0 : (size_t)n;
  }

  // Returns false (and changes nothing) if the text is not a valid settings line.
  bool loadSettings(const char* text) {
    int en, cd; float wr, wl, wh, fl, cm; char tag[16];
    if (!text || sscanf(text, "%15s %d %f %f %f %f %d %f", tag, &en, &wr, &wl, &wh, &fl, &cd, &cm) != 8) return false;
    if (strcmp(tag, "LODESTAR1") != 0 || (en != 0 && en != 1) || (cd != 0 && cd != 1)) return false;
    if (!(wr >= 0 && wl >= 0 && wh >= 0 && wr + wl + wh > 0.5f && wr + wl + wh < 1.5f)) return false;
    if (!(fl >= 0 && fl <= 1 && cm > 0 && cm <= 1)) return false;
    cfg.enabled = en == 1; cfg.w_rel = wr; cfg.w_lq = wl; cfg.w_hop = wh;
    cfg.floor_score = fl; cfg.cancel_dup = cd == 1; cfg.cancel_min = cm;
    _eval_valid = false;
    return true;
  }

  // "lodestar ..." command handler. `args` is the text after the word "lodestar".
  // `print` receives full table lines (serial); `reply` gets a one-line answer.
  void runCommand(const char* args, char* reply, size_t cap, uint32_t now, void (*print)(const char*)) {
    while (*args == ' ') args++;
    if (*args == 0 || strncmp(args, "table", 5) == 0) {
      char line[128];
      int shown = 0;
      for (int i = 0; i < LODESTAR_MAX_UPLINKS; i++) {
        if (row(i, line, sizeof(line), now)) { if (print) print(line); shown++; }
      }
      if (print && !shown) print("(no repeaters heard yet)");
      summary(reply, cap, now);
    } else if (strncmp(args, "weights", 7) == 0) {
      char* e; const char* p = args + 7;
      float r = strtof(p, &e); if (e == p) { snprintf(reply, cap, "Err - lodestar weights <rel> <lq> <hop>"); return; }
      p = e; float l = strtof(p, &e); if (e == p) { snprintf(reply, cap, "Err - lodestar weights <rel> <lq> <hop>"); return; }
      p = e; float h = strtof(p, &e); if (e == p) { snprintf(reply, cap, "Err - lodestar weights <rel> <lq> <hop>"); return; }
      if (r < 0 || l < 0 || h < 0 || (r + l + h) <= 0) { snprintf(reply, cap, "Err - weights must be >= 0"); return; }
      float s = r + l + h;
      cfg.w_rel = r / s; cfg.w_lq = l / s; cfg.w_hop = h / s;
      _eval_valid = false; settings_dirty = true;
      snprintf(reply, cap, "OK - weights rel %.2f lq %.2f hop %.2f", (double)cfg.w_rel, (double)cfg.w_lq, (double)cfg.w_hop);
    } else if (strncmp(args, "floor", 5) == 0) {
      char* e; const char* p = args + 5;
      float f = strtof(p, &e);
      if (e == p || f < 0 || f > 1) { snprintf(reply, cap, "Err - lodestar floor <0..1>"); return; }   // a missing number is an error, not 0
      cfg.floor_score = f; _eval_valid = false; settings_dirty = true;
      snprintf(reply, cap, "OK - floor %.2f", (double)f);
    } else if (strncmp(args, "cancel", 6) == 0) {
      const char* a = args + 6; while (*a == ' ') a++;
      if (strncmp(a, "off", 3) == 0) cfg.cancel_dup = false;
      else if (strncmp(a, "on", 2) == 0) cfg.cancel_dup = true;
      else if (*a) { float f = atof(a); if (f <= 0 || f > 1) { snprintf(reply, cap, "Err - cancel on|off|0..1"); return; } cfg.cancel_min = f; cfg.cancel_dup = true; }
      settings_dirty = true;
      snprintf(reply, cap, "OK - cancel %s min %.2f", cfg.cancel_dup ? "on" : "off", (double)cfg.cancel_min);
    } else if (strncmp(args, "gps", 3) == 0) {
      gpsStatus(reply, cap, now);
      char line[128];
      if (print && gpsDetail(line, sizeof(line), now)) print(line);   // coordinates: serial only
    } else if (strncmp(args, "off", 3) == 0) {
      cfg.enabled = false; settings_dirty = true; snprintf(reply, cap, "OK - Lodestar off (stock forwarding)");
    } else if (strncmp(args, "on", 2) == 0) {
      cfg.enabled = true; settings_dirty = true; snprintf(reply, cap, "OK - Lodestar on");
    } else if (strncmp(args, "defaults", 8) == 0) {
      Config d; cfg.enabled = d.enabled; cfg.w_rel = d.w_rel; cfg.w_lq = d.w_lq; cfg.w_hop = d.w_hop;
      cfg.floor_score = d.floor_score; cfg.cancel_dup = d.cancel_dup; cfg.cancel_min = d.cancel_min;
      _eval_valid = false; settings_dirty = true;
      snprintf(reply, cap, "OK - settings back to defaults");
    } else if (strncmp(args, "reset", 5) == 0) {
      Config keep = cfg; reset(); cfg = keep; snprintf(reply, cap, "OK - uplink table cleared");
    } else {
      snprintf(reply, cap, "lodestar [table|gps|weights r l h|floor f|cancel on|off|f|on|off|defaults|reset]");
    }
  }

  // Re-score and pick the best uplink (rate limited to once per second).
  void evaluate(uint32_t now) {
    if (_eval_valid && elapsed(now, _last_eval) < 1000) return;
    _eval_valid = true; _last_eval = now;
    float wsum = cfg.w_rel + cfg.w_lq + cfg.w_hop;
    if (wsum <= 0) wsum = 1;
    int cand = -1; float cand_score = 0;
    for (int i = 0; i < LODESTAR_MAX_UPLINKS; i++) {
      _score[i] = 0;
      if (!_t[i].used) continue;
      float raw = (cfg.w_rel * reliability(i, now) + cfg.w_lq * linkQuality(i) + cfg.w_hop * hopScore(i)) / wsum;
      _score[i] = clamp01(raw) * freshness(i, now);
      if (cand < 0 || _score[i] > cand_score) { cand = i; cand_score = _score[i]; }
    }
    // top-K, best first
    _topn = 0;
    bool taken[LODESTAR_MAX_UPLINKS] = {false};
    for (int n = 0; n < LODESTAR_TOPK; n++) {
      int bi = -1;
      for (int i = 0; i < LODESTAR_MAX_UPLINKS; i++) {
        if (!_t[i].used || taken[i] || _score[i] < cfg.floor_score) continue;
        if (bi < 0 || _score[i] > _score[bi]) bi = i;
      }
      if (bi < 0) break;
      taken[bi] = true; _top[_topn++] = bi;
    }
    // hysteresis
    bool cur_ok = _best >= 0 && _t[_best].used && _score[_best] >= cfg.floor_score;
    if (!cur_ok) {
      _best = (cand >= 0 && cand_score >= cfg.floor_score) ? cand : -1;
      _chal_count = 0; _challenger = -1;
    } else if (cand != _best && cand_score > _score[_best] + cfg.switch_margin) {
      if (_challenger == cand) _chal_count++; else { _challenger = cand; _chal_count = 1; }
      if (_chal_count >= cfg.switch_evals) { _best = cand; _chal_count = 0; _challenger = -1; }
    } else {
      _chal_count = 0; _challenger = -1;
    }
    _best_score = _best >= 0 ? _score[_best] : 0;
  }

private:
  Uplink   _t[LODESTAR_MAX_UPLINKS];
  Pending  _p[LODESTAR_MAX_PENDING];
  float    _score[LODESTAR_MAX_UPLINKS];
  int      _best, _challenger, _top[LODESTAR_TOPK], _topn;
  float    _best_score;
  uint8_t  _chal_count;
  uint32_t _last_eval, _cancels, _tx_tracked;
  bool     _eval_valid;
  Position _pos{};                     // not cleared by reset(): it is not part of the uplink table
  int32_t  _anchor_lat = 0, _anchor_lon = 0;     // start of the current speed window
  uint32_t _anchor_ms = 0;
  int32_t  _course_lat = 0, _course_lon = 0;     // last point a course was taken from
  bool     _have_anchor = false, _have_speed = false;

  static uint32_t elapsed(uint32_t now, uint32_t then) { return now - then; }  // wrap-safe

  // East/north offset in metres from (lat0,lon0) to (lat1,lon1), both in degrees*1e6.
  // Flat-earth approximation: fine over the few metres between two GPS fixes.
  static void offsetM(int32_t lat0, int32_t lon0, int32_t lat1, int32_t lon1, double& east, double& north) {
    const double k = M_PI / 180.0 / 1e6;
    const double R = 6371000.0;
    double mid = ((double)lat0 + (double)lat1) * 0.5 * k;
    north = (double)(lat1 - lat0) * k * R;
    east  = (double)(lon1 - lon0) * k * R * cos(mid);
  }
  static float clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
  static int pct(float v) { return (int)(clamp01(v) * 100.0f + 0.5f); }

  static float ewma(float old, float sample, float a) { return old + a * (sample - old); }

  float rIn(const Uplink& u, uint32_t now) const {
    uint32_t slot = now / LODESTAR_SLOT_MS;
    uint32_t delta = slot - u.last_slot;
    uint32_t bits = delta >= 32 ? 0 : (u.slot_bits << delta);
    uint32_t mask = (LODESTAR_SLOTS >= 32) ? 0xFFFFFFFFu : ((1u << LODESTAR_SLOTS) - 1u);
    uint32_t seen = u.slots_seen + delta;          // slots elapsed since first heard
    uint32_t denom = seen + 1;
    if (denom > LODESTAR_SLOTS) denom = LODESTAR_SLOTS;
    if (denom < 4) denom = 4;                      // a brand-new entry is not "perfect"
    float r = (float)__builtin_popcount(bits & mask) / (float)denom;
    return clamp01(r);
  }

  void touch(int i, float snr, float rssi, uint32_t now) {
    Uplink& u = _t[i];
    uint32_t slot = now / LODESTAR_SLOT_MS;
    if (u.heard_count == 0) {
      u.first_ms = now; u.last_slot = slot; u.slot_bits = 1; u.slots_seen = 0;
    } else if (slot != u.last_slot) {
      uint32_t delta = slot - u.last_slot;
      u.slot_bits = delta >= 32 ? 0 : (u.slot_bits << delta);
      u.slot_bits |= 1;
      u.slots_seen += delta; if (u.slots_seen > 1000) u.slots_seen = 1000;
      u.last_slot = slot;
    }
    u.last_ms = now;
    u.heard_count++;
    if (!u.lq_init) { u.snr = snr; u.rssi = rssi; u.lq_init = true; }
    else { u.snr = ewma(u.snr, snr, 0.3f); u.rssi = ewma(u.rssi, rssi, 0.3f); }
    _eval_valid = false;
  }

  void ackSample(int i, bool hit) {
    Uplink& u = _t[i];
    if (u.ack_n == 0) u.ack = hit ? 1.0f : 0.0f;
    else u.ack = ewma(u.ack, hit ? 1.0f : 0.0f, 0.2f);
    if (u.ack_n < 255) u.ack_n++;
    if (hit) u.hits++; else u.misses++;
    _eval_valid = false;
  }

  void settle(int k) {                 // window over: unheard candidates count as misses
    Pending& p = _p[k];
    uint32_t miss = p.cand_mask & ~p.hit_mask;
    for (int i = 0; i < LODESTAR_MAX_UPLINKS; i++) {
      if ((miss & (1u << i)) && _t[i].used) ackSample(i, false);
    }
    p.used = false;
  }

  // Find (or create) the table entry for a hash prefix. -1 = ambiguous / table full.
  int resolve(const uint8_t* h, uint8_t sz, bool create, uint32_t now) {
    if (sz == 0) return -1;
    if (sz > 3) sz = 3;
    // exact full-key match first
    if (sz == 3) {
      for (int i = 0; i < LODESTAR_MAX_UPLINKS; i++) {
        if (_t[i].used && _t[i].key_len == 3 && memcmp(_t[i].key, h, 3) == 0) return i;
      }
    }
    int found = -1, n = 0;
    for (int i = 0; i < LODESTAR_MAX_UPLINKS; i++) {
      if (!_t[i].used) continue;
      uint8_t m = _t[i].key_len < sz ? _t[i].key_len : sz;
      if (memcmp(_t[i].key, h, m) == 0) { found = i; n++; }
    }
    if (n == 1) {
      Uplink& u = _t[found];
      if (sz > u.key_len) { memcpy(u.key, h, sz); u.key_len = sz; }   // learn more of the key
      return found;
    }
    if (n > 1) return -1;                                             // can't tell which
    if (!create) return -1;
    int slot = -1;
    for (int i = 0; i < LODESTAR_MAX_UPLINKS; i++) if (!_t[i].used) { slot = i; break; }
    if (slot < 0) {                                                   // evict the stalest
      uint32_t worst = 0;
      for (int i = 0; i < LODESTAR_MAX_UPLINKS; i++) {
        uint32_t age = elapsed(now, _t[i].last_ms);
        if (slot < 0 || age > worst) { slot = i; worst = age; }
      }
      if (_best == slot) { _best = -1; _best_score = 0; }
    }
    memset(&_t[slot], 0, sizeof(Uplink));
    _t[slot].used = true;
    memcpy(_t[slot].key, h, sz);
    _t[slot].key_len = sz;
    _t[slot].last_ms = now;
    _eval_valid = false;
    return slot;
  }
};

}  // namespace lodestar
