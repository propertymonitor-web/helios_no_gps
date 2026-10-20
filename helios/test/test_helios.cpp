// g++ -std=c++17 -Wall -Wextra -Ihelios helios/test/test_helios.cpp -o helios_test && ./helios_test
#include <Helios.h>
#include <stdio.h>
#include <string.h>
using heliosfw::Helios;
static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)
static char g_lines[8][80]; static int g_n = 0;
static void cap(const char* s) { if (g_n < 8) snprintf(g_lines[g_n++], 80, "%s", s); }

int main() {
  { // percent is monotonic and bounded
    int last = -1;
    for (int mv = 3000; mv <= 4400; mv += 10) { int p = Helios::percent(mv); CHECK(p >= last && p >= 0 && p <= 100); last = p; }
    CHECK(Helios::percent(3200) == 0); CHECK(Helios::percent(4200) == 100); CHECK(Helios::percent(3700) == 45);
  }
  { // due(): first reading at once, then every interval, across millis() wrap
    Helios H; CHECK(H.due(0));
    H.sample(3900, 4000000000u); CHECK(!H.due(4000000000u + 59000)); CHECK(H.due(4000000000u + 60000));
    H.sample(3900, 4000000000u + 60000); uint32_t wrapped = 4000000000u + 60000 + 60000; CHECK(H.due(wrapped));
  }
  { // states and hysteresis
    Helios H; H.sample(3900, 0); CHECK(H.state() == heliosfw::ST_GOOD);
    for (int i = 1; i <= 30; i++) H.sample(3600, i * 60000);        // sinks to LOW
    CHECK(H.state() == heliosfw::ST_LOW);
    for (int i = 31; i <= 70; i++) H.sample(3720, i * 60000);       // above good_mv but inside hysteresis margin
    CHECK(H.state() == heliosfw::ST_LOW);
    for (int i = 71; i <= 110; i++) H.sample(3800, i * 60000);      // clears the margin
    CHECK(H.state() == heliosfw::ST_GOOD);
    for (int i = 111; i <= 160; i++) H.sample(3400, i * 60000);
    CHECK(H.state() == heliosfw::ST_CRITICAL);
  }
  { // implausible readings are ignored and counted
    Helios H; H.sample(3900, 0); H.sample(0, 60000); H.sample(9999, 120000);
    CHECK(H.reads() == 1 && H.badReads() == 2 && H.smoothed() == 3900);
  }
  { // trend over 10 minutes
    Helios H; uint32_t t = 0; H.sample(3800, t);
    CHECK(H.trend() == heliosfw::TR_UNKNOWN);
    for (int i = 1; i <= 10; i++) { t += 60000; H.sample(3800 + i * 10, t); }
    CHECK(H.trend() == heliosfw::TR_RISING);
    for (int i = 0; i < 40; i++) { t += 60000; H.sample(3900, t); }
    CHECK(H.trend() == heliosfw::TR_STEADY);
    for (int i = 1; i <= 40; i++) { t += 60000; H.sample(3900 - i * 8, t); }
    CHECK(H.trend() == heliosfw::TR_FALLING);
  }
  { // status line and command
    Helios H; char r[160]; H.runCommand("", r, sizeof(r), 0, cap); CHECK(strstr(r, "no battery reading"));
    H.sample(3920, 1000); H.runCommand("", r, sizeof(r), 90001000, cap);
    CHECK(strstr(r, "3.92V")); CHECK(strstr(r, "GOOD")); CHECK(strstr(r, "up 1d 1h")); CHECK(strlen(r) < 150);
    H.runCommand(" bogus", r, sizeof(r), 0, cap); CHECK(strstr(r, "helios [history"));
  }
  { // hourly history keeps min/max per hour, newest first
    Helios H; g_n = 0; char r[160];
    H.sample(3900, 0); H.sample(3850, 60000); H.sample(3950, 120000);
    H.sample(3800, 3600000UL + 1000); H.sample(3780, 3600000UL + 61000);
    H.runCommand("history", r, sizeof(r), 3600000UL + 62000, cap);
    CHECK(strstr(r, "p1/6")); CHECK(strstr(r, "-0:3.78-3.80")); CHECK(strstr(r, "-1:3.85-3.95")); CHECK(strlen(r) < 150); CHECK(g_n == 2);
    CHECK(strstr(g_lines[0], "hour -0: 3.78V to 3.80V")); CHECK(strstr(g_lines[1], "hour -1: 3.85V to 3.95V"));
    // an hour that wrapped the 48-slot ring is not shown as a stale entry
    Helios G; g_n = 0; G.sample(3900, 0); G.sample(3700, 49UL * 3600000UL);
    G.history(cap, 49UL * 3600000UL); CHECK(g_n == 1);
  }
  { // paged history fits a 150-byte reply even when full, and bad pages are refused
    Helios H; char r[150]; g_n = 0;
    for (int h = 0; h < 60; h++) { H.sample(3900 + h, (uint32_t)h * 3600000UL); }
    uint32_t now = 59UL * 3600000UL + 1000;
    for (int pg = 1; pg <= 6; pg++) { char a[16]; snprintf(a, sizeof(a), "history %d", pg); H.runCommand(a, r, sizeof(r), now, cap); CHECK(strlen(r) < 149); CHECK(strstr(r, "-")); }
    H.runCommand("history 1", r, sizeof(r), now, cap); CHECK(strstr(r, "-0:3.95-3.95"));
    H.runCommand("history 9", r, sizeof(r), now, cap); CHECK(strstr(r, "page 1-6"));
    H.runCommand("history 0", r, sizeof(r), now, cap); CHECK(strstr(r, "page 1-6"));
  }
  { // stats: 24h range and LOW/CRITICAL counts
    Helios H; char r[150]; g_n = 0;
    H.sample(3900, 0); for (int i = 0; i < 12; i++) H.sample(3400, 60000UL * (i + 1));
    H.runCommand("stats", r, sizeof(r), 800000, cap);
    CHECK(strstr(r, "last 1h")); CHECK(strstr(r, "CRITICAL 1x")); CHECK(strstr(r, "bad reads 0")); CHECK(strlen(r) < 149);
    Helios E; E.runCommand("stats", r, sizeof(r), 0, cap); CHECK(strstr(r, "no data"));
  }
  if (fails) { printf("%d FAILED\n", fails); return 1; }
  printf("all tests passed\n");
  return 0;
}
