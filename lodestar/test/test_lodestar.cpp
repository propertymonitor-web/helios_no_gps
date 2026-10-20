// Host-side tests for Lodestar.h:  g++ -std=c++17 -Wall -Wextra -I.. test_lodestar.cpp -o t && ./t
#include <Lodestar.h>
#include <assert.h>
#include <stdio.h>

using lodestar::Lodestar;

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static const uint8_t PK_A[3] = {0xA1, 0x11, 0x11};
static const uint8_t PK_B[3] = {0xB2, 0x22, 0x22};
static const uint8_t PK_C[3] = {0xC3, 0x33, 0x33};

static void hear(Lodestar& L, const uint8_t* pk, float snr, float rssi, uint32_t t) {
  L.onAdvert(pk, snr, rssi, t);
}

static void pr(const char* s) { printf("    %s\n", s); }

static char g_cap[160];
static int  g_cap_calls = 0;
static void capture(const char* s) { snprintf(g_cap, sizeof(g_cap), "%s", s); g_cap_calls++; }

// Feed one GPS fix per second for `secs` seconds, moving (dlat, dlon) microdegrees each second.
// Returns the time after the last fix.
static uint32_t drive(Lodestar& L, uint32_t t, int secs, int32_t& lat, int32_t& lon,
                      int32_t dlat, int32_t dlon, uint8_t sats = 9) {
  for (int i = 0; i < secs; i++, t += 1000) {
    L.onPosition(lat, lon, sats, true, t);
    lat += dlat; lon += dlon;
  }
  return t;
}

int main() {
  // 1. strong neighbour outranks weak one
  {
    Lodestar L; L.cfg.sf = 10;
    uint32_t t = 100000;
    for (int i = 0; i < 12; i++, t += 20000) {
      hear(L, PK_A, 9.0f, -80, t);       // strong
      hear(L, PK_B, -12.0f, -118, t);    // weak, near the SF10 floor (-15 dB)
    }
    L.evaluate(t);
    CHECK(L.count() == 2);
    CHECK(L.best() >= 0 && L.entry(L.best()).key[0] == 0xA1);
    CHECK(L.bestScore() > 0.5f);
    float fa = L.delayFactor(t);
    CHECK(fa < 1.0f && fa >= 0.4f);      // well placed => transmits sooner
  }

  // 2. no uplinks => stock behaviour
  {
    Lodestar L;
    CHECK(L.delayFactor(5000) == 1.0f);
    CHECK(L.best() == -1);
    L.cfg.enabled = false;
    hear(L, PK_A, 9, -80, 1000);
    CHECK(L.delayFactor(2000) == 1.0f);
  }

  // 3. stale uplink decays and is dropped
  {
    Lodestar L; uint32_t t = 100000;
    for (int i = 0; i < 12; i++, t += 20000) hear(L, PK_A, 9, -80, t);
    L.evaluate(t);
    float fresh = L.scoreOf(0, t);
    float old = L.scoreOf(0, t + 400000);          // 400 s later: mid-decay
    CHECK(old < fresh && old > 0);
    CHECK(L.scoreOf(0, t + 700000) == 0);          // past stale_end
    L.loop(t + 2000000);                           // past expire
    CHECK(L.count() == 0);
  }

  // 4. implicit ACK: echo with our hash then theirs counts as a hit
  {
    Lodestar L; uint32_t t = 100000;
    for (int i = 0; i < 6; i++, t += 20000) { hear(L, PK_A, 5, -90, t); hear(L, PK_B, 5, -90, t); }
    uint8_t hash[8] = {1,2,3,4,5,6,7,8};
    uint8_t mine[1] = {0x77};                      // we are forwarding: path ends with our hash
    L.onTx(hash, mine, 1, 1, t, 8000);
    uint8_t echo[2] = {0x77, 0xA1};                // A re-flooded it
    int a = L.onHeardFrom(&echo[1], 1, 5, -90, t + 500);
    CHECK(a >= 0);
    L.onFloodCopy(hash, echo, 2, 1, a, t + 500);
    L.loop(t + 9000);                              // window over: B never echoed => miss
    int ib = -1; for (int i = 0; i < 24; i++) if (L.entry(i).used && L.entry(i).key[0] == 0xB2) ib = i;
    CHECK(L.entry(a).hits == 1 && L.entry(a).misses == 0);
    CHECK(L.entry(ib).hits == 0 && L.entry(ib).misses == 1);
  }

  // 5. echo with the wrong path prefix is not an ACK
  {
    Lodestar L; uint32_t t = 100000;
    for (int i = 0; i < 4; i++, t += 20000) hear(L, PK_A, 5, -90, t);
    uint8_t hash[8] = {9,9,9,9,9,9,9,9};
    uint8_t mine[1] = {0x77};
    L.onTx(hash, mine, 1, 1, t, 8000);
    uint8_t other[2] = {0x55, 0xA1};               // came a different way round
    int a = L.onHeardFrom(&other[1], 1, 5, -90, t + 200);
    L.onFloodCopy(hash, other, 2, 1, a, t + 200);
    CHECK(L.entry(a).hits == 0);
  }

  // 6. reliable uplink beats a stronger but unreliable one (reliability weighs most)
  {
    Lodestar L; uint32_t t = 100000;
    for (int i = 0; i < 10; i++, t += 20000) {
      hear(L, PK_A, 6, -95, t);                    // decent, always answers
      if (i % 4 == 0) hear(L, PK_B, 11, -75, t);   // strongest, but patchy
    }
    uint8_t mine[1] = {0x77};
    for (int k = 0; k < 8; k++) {
      uint8_t hash[8] = {(uint8_t)k, 1,1,1,1,1,1,1};
      L.onTx(hash, mine, 1, 1, t, 6000);
      uint8_t echo[2] = {0x77, 0xA1};
      int a = L.onHeardFrom(&echo[1], 1, 6, -95, t + 300);
      L.onFloodCopy(hash, echo, 2, 1, a, t + 300);
      t += 7000; L.loop(t);
    }
    L.evaluate(t + 1000);
    CHECK(L.entry(L.best()).key[0] == 0xA1);
  }

  // 7. hysteresis: a marginally better challenger does not steal the crown at once
  {
    Lodestar L; uint32_t t = 100000;
    for (int i = 0; i < 12; i++, t += 20000) { hear(L, PK_A, 8, -85, t); hear(L, PK_B, 7.9f, -85.5f, t); }
    L.evaluate(t);
    int first = L.best();
    for (int i = 0; i < 3; i++, t += 20000) { hear(L, PK_A, 8, -85, t); hear(L, PK_B, 8.3f, -84, t); L.evaluate(t); }
    CHECK(L.best() == first);
  }

  // 8. a clearly better challenger takes over after the confirm count
  {
    Lodestar L; uint32_t t = 100000;
    for (int i = 0; i < 12; i++, t += 20000) hear(L, PK_A, -8, -112, t);
    L.evaluate(t);
    CHECK(L.entry(L.best()).key[0] == 0xA1);
    for (int i = 0; i < 12; i++, t += 20000) { hear(L, PK_A, -8, -112, t); hear(L, PK_B, 10, -78, t); L.evaluate(t); }
    CHECK(L.entry(L.best()).key[0] == 0xB2);
  }

  // 9. ambiguous 1-byte hash is ignored, not mis-credited
  {
    Lodestar L; uint32_t t = 100000;
    uint8_t k1[3] = {0xD4, 1, 1}, k2[3] = {0xD4, 2, 2};
    hear(L, k1, 5, -90, t); hear(L, k2, 5, -90, t);
    CHECK(L.count() == 2);
    uint8_t h = 0xD4;
    CHECK(L.onHeardFrom(&h, 1, 5, -90, t + 10) == -1);
    CHECK(L.count() == 2);
  }

  // 10. a partial hash learned from traffic upgrades once the advert is heard
  {
    Lodestar L; uint32_t t = 100000;
    uint8_t h = 0xC3;
    int i = L.onHeardFrom(&h, 1, 4, -92, t);
    CHECK(i >= 0 && L.entry(i).key_len == 1);
    int j = L.onAdvert(PK_C, 4, -92, t + 1000);
    CHECK(j == i && L.entry(i).key_len == 3);
  }

  // 11. cancel only for strong uplinks
  {
    Lodestar L; uint32_t t = 100000;
    for (int i = 0; i < 14; i++, t += 20000) { hear(L, PK_A, 9, -80, t); hear(L, PK_B, -13, -119, t); }
    uint8_t hash[8] = {4,4,4,4,4,4,4,4};
    uint8_t echo[2] = {0x77, 0xA1};
    int a = L.onHeardFrom(&echo[1], 1, 9, -80, t);
    CHECK(L.onFloodCopy(hash, echo, 2, 1, a, t) == true);
    uint8_t echo2[2] = {0x77, 0xB2};
    int b = L.onHeardFrom(&echo2[1], 1, -13, -119, t);
    CHECK(L.onFloodCopy(hash, echo2, 2, 1, b, t) == false);
    L.cfg.cancel_dup = false;
    CHECK(L.onFloodCopy(hash, echo, 2, 1, a, t) == false);
  }

  // 12. table full evicts the stalest, never crashes
  {
    Lodestar L; uint32_t t = 100000;
    for (int n = 0; n < 40; n++, t += 1000) {
      uint8_t k[3] = {(uint8_t)(n + 1), (uint8_t)n, 7};
      hear(L, k, 3, -100, t);
    }
    CHECK(L.count() == LODESTAR_MAX_UPLINKS);
  }

  // 13. millis wrap-around is safe
  {
    Lodestar L; uint32_t t = 0xFFFFFFFFu - 30000u;
    for (int i = 0; i < 10; i++, t += 20000) hear(L, PK_A, 8, -85, t);
    L.evaluate(t);
    CHECK(L.count() == 1 && L.bestScore() > 0.3f);
  }

  // 14. CLI
  {
    Lodestar L; uint32_t t = 100000; char r[160];
    for (int i = 0; i < 12; i++, t += 20000) hear(L, PK_A, 8, -85, t);
    L.runCommand("", r, sizeof(r), t, pr);          printf("  reply: %s\n", r);
    CHECK(strstr(r, "best A1") != NULL);
    L.runCommand(" weights 5 3 2", r, sizeof(r), t, pr);
    CHECK(fabsf(L.cfg.w_rel - 0.5f) < 1e-4f && fabsf(L.cfg.w_hop - 0.2f) < 1e-4f);
    L.runCommand(" weights 1 0 0", r, sizeof(r), t, pr);
    CHECK(L.cfg.w_rel == 1.0f && L.cfg.w_lq == 0.0f);
    L.runCommand(" weights x", r, sizeof(r), t, pr);  CHECK(strncmp(r, "Err", 3) == 0);
    L.runCommand(" cancel off", r, sizeof(r), t, pr); CHECK(!L.cfg.cancel_dup);
    L.runCommand(" cancel 0.7", r, sizeof(r), t, pr); CHECK(L.cfg.cancel_dup && fabsf(L.cfg.cancel_min - 0.7f) < 1e-4f);
    L.runCommand(" off", r, sizeof(r), t, pr);        CHECK(!L.cfg.enabled);
    L.runCommand(" on", r, sizeof(r), t, pr);         CHECK(L.cfg.enabled);
    L.runCommand(" reset", r, sizeof(r), t, pr);      CHECK(L.count() == 0 && L.cfg.enabled);
    L.runCommand(" bogus", r, sizeof(r), t, pr);      CHECK(strncmp(r, "lodestar", 8) == 0);
  }

  // 15. table dump formatting
  {
    Lodestar L; uint32_t t = 100000; char line[128];
    for (int i = 0; i < 12; i++, t += 20000) { hear(L, PK_A, 8, -85, t); hear(L, PK_B, -2, -105, t); }
    for (int i = 0; i < 24; i++) if (L.row(i, line, sizeof(line), t)) printf("  %s\n", line);
  }

  // 16. no GPS => no fix, and nothing breaks
  {
    Lodestar L; char r[160];
    CHECK(!L.hasFix(5000) && !L.moving(5000));
    L.gpsStatus(r, sizeof(r), 5000);
    CHECK(strcmp(r, "GPS: no fix") == 0);
  }

  // 17. driving north at ~15 m/s: speed ~15, course ~0, moving
  {
    Lodestar L; char r[160]; int32_t lat = 32750000, lon = -97330000;
    uint32_t t = drive(L, 100000, 12, lat, lon, 135, 0);       // 135 microdeg/s ~ 15.0 m/s north
    CHECK(L.hasFix(t) && L.moving(t));
    CHECK(fabsf(L.position().speed_mps - 15.0f) < 1.5f);
    CHECK(L.position().course_valid && (L.position().course_deg < 2.0f || L.position().course_deg > 358.0f));
    L.gpsStatus(r, sizeof(r), t);
    printf("  %s\n", r);
    CHECK(strstr(r, "moving") != NULL && strstr(r, "km/h") != NULL && strstr(r, "32.75") == NULL);
  }

  // 18. driving east at ~15 m/s: course ~90
  {
    Lodestar L; int32_t lat = 32750000, lon = -97330000;
    uint32_t t = drive(L, 100000, 12, lat, lon, 0, 160);       // 160 microdeg/s at 32.75N ~ 15.0 m/s east
    CHECK(L.moving(t));
    CHECK(fabsf(L.position().course_deg - 90.0f) < 2.0f);
    CHECK(fabsf(L.position().speed_mps - 15.0f) < 1.5f);
  }

  // 18b. a hiker at ~1.7 m/s heading south-east still gets speed and a course
  {
    Lodestar L; int32_t lat = 32750000, lon = -97330000;
    uint32_t t = drive(L, 100000, 30, lat, lon, -11, 12);      // ~1.2 m/s south + ~1.1 m/s east
    CHECK(L.moving(t));
    CHECK(L.position().speed_mps > 1.4f && L.position().speed_mps < 2.0f);
    CHECK(L.position().course_valid && fabsf(L.position().course_deg - 137.0f) < 8.0f);
  }

  // 19. parked with GPS jitter (+-1.7 m): not moving, no course invented
  {
    Lodestar L; int32_t lat = 32750000, lon = -97330000; uint32_t t = 100000;
    for (int i = 0; i < 90; i++, t += 1000) {
      int32_t j = (i % 2) ? 15 : -15;
      L.onPosition(lat + j, lon - j, 8, true, t);
    }
    CHECK(L.hasFix(t) && !L.moving(t));
    CHECK(!L.position().course_valid);
  }

  // 20. fix lost then regained far away: no absurd speed measured across the gap
  {
    Lodestar L; int32_t lat = 32750000, lon = -97330000;
    uint32_t t = drive(L, 100000, 12, lat, lon, 135, 0);
    CHECK(L.moving(t));
    L.onPosition(0, 0, 0, false, t);
    CHECK(!L.hasFix(t) && !L.moving(t));
    t += 30000;
    lat += 30000; lon += 30000;                                // ~3 km away when it comes back
    L.onPosition(lat, lon, 6, true, t);
    CHECK(L.hasFix(t) && L.position().speed_mps == 0.0f && !L.moving(t));
  }

  // 21. feed goes quiet (GPS switched off): fix expires, and a late restart does not measure the gap
  {
    Lodestar L; int32_t lat = 32750000, lon = -97330000;
    uint32_t t = drive(L, 100000, 12, lat, lon, 135, 0);
    CHECK(L.hasFix(t));
    CHECK(!L.hasFix(t + 20000));                               // older than gps_fresh_ms
    lat += 50000;                                              // far away after a quiet spell
    L.onPosition(lat, lon, 7, true, t + 20000);
    CHECK(L.hasFix(t + 20000) && L.position().speed_mps == 0.0f);
  }

  // 22. polled every loop (50 ms) while the GPS only changes once a second: same answer
  {
    Lodestar L; int32_t lat0 = 32750000, lon = -97330000; uint32_t t = 100000;
    for (int ms = 0; ms < 12000; ms += 50, t += 50) {
      int32_t lat = lat0 + 135 * (int32_t)(ms / 1000);         // a GPS holds its last fix between updates
      L.onPosition(lat, lon, 9, true, t);
    }
    CHECK(L.moving(t));
    CHECK(fabsf(L.position().speed_mps - 15.0f) < 2.0f);
  }

  // 23. millis wrap-around while driving
  {
    Lodestar L; int32_t lat = 32750000, lon = -97330000;
    uint32_t t = drive(L, 0xFFFFFFFFu - 6000u, 14, lat, lon, 135, 0);
    CHECK(L.hasFix(t) && L.moving(t));
    CHECK(fabsf(L.position().speed_mps - 15.0f) < 1.5f);
  }

  // 24. CLI: the reply never carries coordinates; the serial line does, and only with a fix
  {
    Lodestar L; char r[160]; int32_t lat = 32750000, lon = -97330000;
    g_cap_calls = 0;
    L.runCommand(" gps", r, sizeof(r), 5000, capture);
    CHECK(strcmp(r, "GPS: no fix") == 0 && g_cap_calls == 0);
    uint32_t t = drive(L, 100000, 12, lat, lon, 135, 0);
    L.runCommand(" gps", r, sizeof(r), t, capture);
    printf("  reply: %s\n  serial: %s\n", r, g_cap);
    CHECK(strstr(r, "GPS: fix") != NULL && strstr(r, "32.") == NULL && strstr(r, "97.") == NULL);
    CHECK(g_cap_calls == 1 && strstr(g_cap, "32.75") != NULL && strstr(g_cap, "-97.33") != NULL);
    L.runCommand(" gps", r, sizeof(r), t, NULL);               // remote admin: no serial printer
    CHECK(strstr(r, "GPS: fix") != NULL);
  }

  // 25. `lodestar reset` clears the uplink table but keeps the position
  {
    Lodestar L; char r[160]; int32_t lat = 32750000, lon = -97330000;
    uint32_t t = drive(L, 100000, 12, lat, lon, 135, 0);
    hear(L, PK_A, 8, -85, t);
    L.runCommand(" reset", r, sizeof(r), t, pr);
    CHECK(L.count() == 0 && L.hasFix(t) && L.moving(t));
  }

  // 26. position is plumbing only: forwarding decisions are identical with or without it
  {
    Lodestar A, B; int32_t lat = 32750000, lon = -97330000; uint32_t t = 100000;
    for (int i = 0; i < 12; i++, t += 20000) {
      hear(A, PK_A, 9, -80, t); hear(B, PK_A, 9, -80, t);
      hear(A, PK_B, -2, -105, t); hear(B, PK_B, -2, -105, t);
    }
    drive(B, t, 12, lat, lon, 135, 0);
    CHECK(A.delayFactor(t) == B.delayFactor(t));
    CHECK(A.best() == B.best());
  }

  // 27. `lodestar floor` needs a number: missing or non-numeric input is rejected, not read as 0
  {
    Lodestar L; char r[160]; uint32_t t = 100000;
    CHECK(fabsf(L.cfg.floor_score - 0.25f) < 1e-4f);
    L.runCommand(" floor", r, sizeof(r), t, pr);       CHECK(strncmp(r, "Err", 3) == 0 && fabsf(L.cfg.floor_score - 0.25f) < 1e-4f);
    L.runCommand(" floor abc", r, sizeof(r), t, pr);   CHECK(strncmp(r, "Err", 3) == 0 && fabsf(L.cfg.floor_score - 0.25f) < 1e-4f);
    L.runCommand(" floor 1.5", r, sizeof(r), t, pr);   CHECK(strncmp(r, "Err", 3) == 0 && fabsf(L.cfg.floor_score - 0.25f) < 1e-4f);
    L.runCommand(" floor 0.4", r, sizeof(r), t, pr);   CHECK(strncmp(r, "OK", 2) == 0 && fabsf(L.cfg.floor_score - 0.4f) < 1e-4f);
    L.runCommand(" floor 0", r, sizeof(r), t, pr);     CHECK(strncmp(r, "OK", 2) == 0 && L.cfg.floor_score == 0.0f);   // an explicit 0 is still allowed
  }

  // 28. settings round-trip through saveSettings/loadSettings; bad text is rejected and changes nothing
  {
    Lodestar A; char r[160]; char buf[160]; uint32_t t = 100000;
    CHECK(!A.settings_dirty);
    A.runCommand(" weights 1 1 2", r, sizeof(r), t, pr);  CHECK(A.settings_dirty);
    A.settings_dirty = false;
    A.runCommand(" table", r, sizeof(r), t, pr);          CHECK(!A.settings_dirty);   // reading changes nothing
    A.runCommand(" floor abc", r, sizeof(r), t, pr);      CHECK(!A.settings_dirty);   // a rejected command changes nothing
    A.runCommand(" floor 0.4", r, sizeof(r), t, pr);
    A.runCommand(" cancel 0.8", r, sizeof(r), t, pr);
    A.runCommand(" off", r, sizeof(r), t, pr);
    size_t n = A.saveSettings(buf, sizeof(buf));
    CHECK(n > 0);
    Lodestar B;
    CHECK(B.loadSettings(buf));
    CHECK(!B.cfg.enabled && B.cfg.cancel_dup && fabsf(B.cfg.cancel_min - 0.8f) < 1e-3f);
    CHECK(fabsf(B.cfg.floor_score - 0.4f) < 1e-3f && fabsf(B.cfg.w_hop - 0.5f) < 1e-3f && fabsf(B.cfg.w_rel - 0.25f) < 1e-3f);
    Lodestar C;
    CHECK(!C.loadSettings("") && !C.loadSettings("garbage") && !C.loadSettings("LODESTAR2 1 .5 .3 .2 .25 1 .6")
          && !C.loadSettings("LODESTAR1 1 .5 .3 .2 7 1 .6") && !C.loadSettings("LODESTAR1 1 -1 .3 .2 .25 1 .6")
          && !C.loadSettings("LODESTAR1 1 0 0 0 .25 1 .6") && !C.loadSettings("LODESTAR1 1 .5 .3 .2 .25 1 0"));
    CHECK(C.cfg.enabled && fabsf(C.cfg.floor_score - 0.25f) < 1e-4f);   // still the defaults
    n = A.saveSettings(buf, 10);  CHECK(n == 0);                         // too small a buffer: nothing written
    A.runCommand(" defaults", r, sizeof(r), t, pr);
    CHECK(strncmp(r, "OK", 2) == 0 && A.cfg.enabled && fabsf(A.cfg.floor_score - 0.25f) < 1e-4f && A.settings_dirty);
  }

  printf(fails ? "\n%d FAILED\n" : "\nall tests passed\n", fails);
  return fails ? 1 : 0;
}
