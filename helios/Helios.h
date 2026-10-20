// Helios.h - battery and solar watch for a solar-powered MeshCore repeater (header-only, no Arduino deps).
//
// HELIOS watches the battery of a solar repeater and reports it. It is read-only: it never changes
// forwarding, power or radio settings. Voltages are what the board's battery ADC reports (a Li-ion
// 18650 cell: about 4.2 V full, about 3.3 V empty), so percentages are APPROXIMATE and a loaded
// cell reads low.
//
// Host-side tests:  g++ -std=c++17 -Wall -Wextra -Ihelios helios/test/test_helios.cpp -o t && ./t
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace heliosfw {

enum State : uint8_t { ST_UNKNOWN = 0, ST_GOOD, ST_LOW, ST_CRITICAL };
enum Trend : uint8_t { TR_UNKNOWN = 0, TR_RISING, TR_STEADY, TR_FALLING };

struct Config {
  uint16_t good_mv = 3700;        // at or above: GOOD
  uint16_t low_mv = 3500;         // at or above (and below good): LOW; below: CRITICAL
  uint16_t hyst_mv = 50;          // must rise this far above a threshold to leave the worse state
  uint32_t interval_ms = 60000;   // one battery reading a minute
  int16_t trend_mv = 15;          // change over a trend window that counts as rising / falling
  uint32_t trend_ms = 600000;     // 10-minute trend window
};

struct HourSlot {
  uint16_t min_mv = 0;
  uint16_t max_mv = 0;
  bool used = false;
};

class Helios {
 public:
  static const int HOURS = 48;
  Config cfg;

  // True when a battery reading should be taken now. Safe across millis() wrap-around.
  bool due(uint32_t now) const {
    return !_started || (uint32_t)(now - _last_ms) >= cfg.interval_ms;
  }

  // Feed one reading in millivolts. Implausible readings are counted and ignored.
  void sample(uint16_t mv, uint32_t now) {
    _last_ms = now;
    if (!_started) { _started = true; _t0 = now; }
    if (mv < 2500 || mv > 5000) { _bad++; return; }
    _reads++;
    _raw = mv;
    _ema = (_reads == 1) ? (float)mv : _ema + ((float)mv - _ema) * 0.25f;
    uint16_t sm = smoothed();
    if (!_have_range) { _min = _max = mv; _have_range = true; }
    if (mv < _min) _min = mv;
    if (mv > _max) _max = mv;
    classify(sm);
    trendUpdate(sm, now);
    hourUpdate(mv, now);
  }

  bool haveReading() const { return _reads > 0; }
  uint16_t smoothed() const { return (uint16_t)(_ema + 0.5f); }
  uint16_t raw() const { return _raw; }
  uint16_t minMv() const { return _min; }
  uint16_t maxMv() const { return _max; }
  uint32_t reads() const { return _reads; }
  uint32_t badReads() const { return _bad; }
  State state() const { return _state; }
  Trend trend() const { return _trend; }
  uint32_t uptimeSec(uint32_t now) const { return _started ? (uint32_t)(now - _t0) / 1000 : 0; }

  // Approximate state of charge from a resting Li-ion cell voltage.
  static int percent(uint16_t mv) {
    static const uint16_t V[] = {3300, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200};
    static const uint8_t P[] = {0, 10, 25, 45, 60, 72, 85, 95, 100};
    if (mv <= V[0]) return 0;
    if (mv >= V[8]) return 100;
    for (int i = 1; i < 9; i++) {
      if (mv <= V[i]) {
        return P[i - 1] + (int)((uint32_t)(mv - V[i - 1]) * (P[i] - P[i - 1]) / (V[i] - V[i - 1]));
      }
    }
    return 100;
  }

  static const char* stateName(State s) {
    switch (s) {
      case ST_GOOD: return "GOOD";
      case ST_LOW: return "LOW";
      case ST_CRITICAL: return "CRITICAL";
      default: return "?";
    }
  }
  static const char* trendName(Trend t) {
    switch (t) {
      case TR_RISING: return "rising";
      case TR_STEADY: return "steady";
      case TR_FALLING: return "falling";
      default: return "trend ?";
    }
  }

  void status(char* out, size_t cap, uint32_t now) const {
    if (!haveReading()) { snprintf(out, cap, "HELIOS: no battery reading yet"); return; }
    uint16_t sm = smoothed();
    uint32_t up = uptimeSec(now);
    snprintf(out, cap, "HELIOS %u.%02uV ~%d%% %s, %s, min %u.%02uV max %u.%02uV, up %lud %luh, %lu reads",
             sm / 1000, (sm % 1000) / 10, percent(sm), stateName(_state), trendName(_trend),
             _min / 1000, (_min % 1000) / 10, _max / 1000, (_max % 1000) / 10,
             (unsigned long)(up / 86400), (unsigned long)((up % 86400) / 3600), (unsigned long)_reads);
  }

  // One line per hour since start-up, newest first (hour 0 is the current hour).
  void history(void (*print)(const char*), uint32_t now) const {
    if (!_started) { print("no history yet"); return; }
    uint32_t cur = (uint32_t)(now - _t0) / 3600000UL;
    int shown = 0;
    char line[64];
    for (uint32_t back = 0; back < (uint32_t)HOURS && back <= cur; back++) {
      const HourSlot& s = _slots[(cur - back) % HOURS];
      if (!s.used || _slot_hour[(cur - back) % HOURS] != cur - back) continue;
      snprintf(line, sizeof(line), "hour -%lu: %u.%02uV to %u.%02uV", (unsigned long)back,
               s.min_mv / 1000, (s.min_mv % 1000) / 10, s.max_mv / 1000, (s.max_mv % 1000) / 10);
      print(line);
      shown++;
    }
    if (!shown) print("no history yet");
  }

  // History as a short reply that fits over the air: 8 hours per page, newest first.
  // Each entry is "<hours ago>:<min>-<max>" in volts, e.g. "-3:3.78-3.80".
  void historyReply(int page, char* out, size_t cap, uint32_t now) const {
    const int PER = 8, PAGES = HOURS / PER;
    if (!_started) { snprintf(out, cap, "no history yet"); return; }
    if (page < 1 || page > PAGES) { snprintf(out, cap, "page 1-%d", PAGES); return; }
    uint32_t cur = (uint32_t)(now - _t0) / 3600000UL;
    size_t n = (size_t)snprintf(out, cap, "p%d/%d V min-max:", page, PAGES);
    int shown = 0;
    for (int k = 0; k < PER; k++) {
      uint32_t back = (uint32_t)((page - 1) * PER + k);
      if (back > cur) break;
      int i = (cur - back) % HOURS;
      if (!_slots[i].used || _slot_hour[i] != cur - back) continue;
      if (n >= cap) break;
      n += (size_t)snprintf(out + n, cap - n, " -%lu:%u.%02u-%u.%02u", (unsigned long)back,
                            _slots[i].min_mv / 1000, (_slots[i].min_mv % 1000) / 10,
                            _slots[i].max_mv / 1000, (_slots[i].max_mv % 1000) / 10);
      shown++;
    }
    if (!shown && n < cap) snprintf(out + n, cap - n, " none yet");
  }

  // Last 24 hours at a glance, plus how often the battery has gone LOW or CRITICAL since start-up.
  void stats(char* out, size_t cap, uint32_t now) const {
    if (!_started) { snprintf(out, cap, "no data yet"); return; }
    uint32_t cur = (uint32_t)(now - _t0) / 3600000UL;
    uint16_t lo = 0, hi = 0; int hrs = 0;
    for (uint32_t back = 0; back < 24 && back <= cur; back++) {
      int i = (cur - back) % HOURS;
      if (!_slots[i].used || _slot_hour[i] != cur - back) continue;
      if (hrs == 0 || _slots[i].min_mv < lo) lo = _slots[i].min_mv;
      if (hrs == 0 || _slots[i].max_mv > hi) hi = _slots[i].max_mv;
      hrs++;
    }
    snprintf(out, cap, "last %dh: %u.%02uV-%u.%02uV, LOW %lux, CRITICAL %lux, bad reads %lu",
             hrs, lo / 1000, (lo % 1000) / 10, hi / 1000, (hi % 1000) / 10,
             (unsigned long)_low_events, (unsigned long)_crit_events, (unsigned long)_bad);
  }

  // helios [history [page]|stats]
  void runCommand(const char* args, char* reply, size_t cap, uint32_t now, void (*print)(const char*)) {
    while (*args == ' ') args++;
    if (*args == 0) {
      status(reply, cap, now);
    } else if (strncmp(args, "history", 7) == 0 && (args[7] == 0 || args[7] == ' ')) {
      const char* a = args + 7;
      while (*a == ' ') a++;
      int page = *a ? atoi(a) : 1;
      if (page == 1 || *a == 0) history(print, now);   // the USB serial port also gets one line per hour
      historyReply(page, reply, cap, now);
    } else if (strcmp(args, "stats") == 0) {
      stats(reply, cap, now);
    } else {
      snprintf(reply, cap, "helios [history [1-6]|stats]");
    }
  }

 private:
  bool _started = false;
  uint32_t _t0 = 0, _last_ms = 0;
  float _ema = 0;
  uint16_t _raw = 0, _min = 0, _max = 0;
  bool _have_range = false;
  uint32_t _reads = 0, _bad = 0, _low_events = 0, _crit_events = 0;
  State _state = ST_UNKNOWN;
  Trend _trend = TR_UNKNOWN;
  uint16_t _mark_mv = 0;
  uint32_t _mark_ms = 0;
  bool _have_mark = false;
  HourSlot _slots[HOURS];
  uint32_t _slot_hour[HOURS] = {0};

  void classify(uint16_t sm) {
    State want;
    if (sm >= cfg.good_mv) want = ST_GOOD;
    else if (sm >= cfg.low_mv) want = ST_LOW;
    else want = ST_CRITICAL;
    // enum order is GOOD(1) < LOW(2) < CRITICAL(3), so a bigger value is a worse state
    if (_state == ST_UNKNOWN || want > _state) {
      _state = want;
      if (want == ST_LOW) _low_events++;
      if (want == ST_CRITICAL) _crit_events++;
      return;
    }   // first reading, or getting worse
    // getting better: only after clearing the threshold by the hysteresis margin
    if (want < _state) {   // improve one level at a time
      State up = (State)(_state - 1);
      uint16_t need = (up == ST_GOOD) ? cfg.good_mv : cfg.low_mv;
      if (sm >= (uint16_t)(need + cfg.hyst_mv)) _state = up;
    }
  }

  void trendUpdate(uint16_t sm, uint32_t now) {
    if (!_have_mark) { _mark_mv = sm; _mark_ms = now; _have_mark = true; return; }
    if ((uint32_t)(now - _mark_ms) >= cfg.trend_ms) {
      int d = (int)sm - (int)_mark_mv;
      _trend = d >= cfg.trend_mv ? TR_RISING : (d <= -cfg.trend_mv ? TR_FALLING : TR_STEADY);
      _mark_mv = sm;
      _mark_ms = now;
    }
  }

  void hourUpdate(uint16_t mv, uint32_t now) {
    uint32_t h = (uint32_t)(now - _t0) / 3600000UL;
    int i = h % HOURS;
    if (!_slots[i].used || _slot_hour[i] != h) {
      _slots[i].used = true; _slot_hour[i] = h; _slots[i].min_mv = _slots[i].max_mv = mv;
    } else {
      if (mv < _slots[i].min_mv) _slots[i].min_mv = mv;
      if (mv > _slots[i].max_mv) _slots[i].max_mv = mv;
    }
  }
};

}  // namespace heliosfw
