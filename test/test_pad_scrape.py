#!/usr/bin/env python3
"""Run the production wipe control (Marlin/src/e3v3se/PadScrapeCtl.h: load servo and feed speed
profile) inside a timing model of PadHoming.cpp pad_wipe_pass(): Z and the feed axis step while
each HX711 conversion runs (12.5 ms), the control runs once per sample. Simulated strip: stiffness,
slope, deposits, a hand pressing the bed, drift, noise, spikes, one sample of HX711 lag.

Checks: never below the depth floor; the feed travels at most ~0.35 mm between samples (reaction
distance at 25 mm/s); the wipe reaches its speed; the feed stops quickly when the load is over the
band; the load is held in the band on a strip stiff enough for it; hand presses end the wipe in
time; the flick only moves X, from the strip to the flick X."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include "Marlin/src/e3v3se/PadScrapeCtl.h"

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { std::printf("FAIL %s: ", s.name); std::printf(__VA_ARGS__); std::printf("\n"); ++fails; } } while (0)

struct Sim {
  const char *name;
  double k_mm;                  // strip stiffness, counts per mm
  double bump_y, bump_mm, ramp; // deposit on the strip around bump_y: rises over 'ramp' mm, 2 mm flat
  double press_after_s, press;  // a hand presses the bed (load independent of Z) after this time
  double drift_per_s, noise;
  int expect_end;               // 0 = all passes + flick complete, 1 = the wipe ends
  double min_in_band;           // fraction of the samples on the strip that must be in the band
};

static double frand() { return std::rand() / double(RAND_MAX) * 2 - 1; }

// Firmware constants (PadHoming.cpp / Configuration_adv.h defaults)
static const double SPM = 400, SPM_A = 80, V0 = 5, ACCEL = 2000, VMAX = 25;
static const long SAMPLE_US = 12500, Z_US = 625;

static void run(const Sim &s) {
  const PadWipeCtl c = { 15000, 5000, 35000, -long(0.2 * SPM), long(1.0 * SPM), long(0.3 * SPM),
                         long(0.05 * SPM), long(0.01 * SPM), long(0.05 * SPM) };
  long z = long(5 * SPM);                  // the wipe starts at travel height over Y0
  double x = -5, y = 50, t = 0, prev = 0, fmax_true = 0, max_feed_per_sample = 0, vpeak = 0;
  double max_stop_mm = 0;
  int ended = 0, end_after_press = -1, samples_on = 0, in_band_on = 0;
  long samples_total = 0;
  bool x_ok = true;
  auto force_at = [&](long zz) {
    double surf = -0.001 * (50 - y);       // strip not level
    if (s.bump_mm > 0 && std::fabs(x + 5) < 1) {
      const double u = std::fabs(y - s.bump_y);
      if (u < 1) surf += s.bump_mm;
      else if (u < 1 + s.ramp) surf += s.bump_mm * (1 + s.ramp - u) / s.ramp;
    }
    if (x < -7) return 0.0;                // off the strip edge: nothing under the nozzle
    const double pen = surf - zz / SPM;
    return pen <= 0 ? 0.0 : pen * s.k_mm;
  };
  struct Pass { bool is_x; double to; };
  const Pass passes[] = { {false, 20}, {false, 50}, {false, 20}, {false, 50}, {true, -11} };
  long z_goal = z;
  bool feed = false, band = false;
  for (const Pass &p : passes) {
    if (ended) break;
    double &pos = p.is_x ? x : y;
    const double from = pos, y_before = y;
    long left = std::lround(std::fabs(p.to - from) * SPM_A);
    const double dir = p.to > from ? 1 : -1;
    double v = 0, us_next_a = 0, us_next_z = 0, stop_from = NAN;
    const float dv2 = float(2 * ACCEL / SPM_A);
    for (int i = 0; i < 100000; ++i) {
      // One conversion: Z and the feed step as in the firmware's wait loop
      const double t0 = t * 1e6, pos0 = pos;
      for (double us = t0; us < t0 + SAMPLE_US; us += 25) {
        if (z != z_goal && us >= us_next_z) { z += z_goal > z ? 1 : -1; us_next_z = us + Z_US; }
        if (left && us >= us_next_a) {
          v = pad_feed_speed(float(v), feed, left, float(VMAX), float(V0), dv2, [](float a) { return std::sqrt(a); });
          if (v > 0) { --left; pos += dir / SPM_A; us_next_a = us + 1e6 / (v * SPM_A); }
          else us_next_a = us + 1000;
          vpeak = std::max(vpeak, v);
        }
      }
      t += SAMPLE_US * 1e-6;
      max_feed_per_sample = std::max(max_feed_per_sample, std::fabs(pos - pos0));
      if (!feed && !std::isnan(stop_from) && v == 0) { max_stop_mm = std::max(max_stop_mm, std::fabs(pos - stop_from)); stop_from = NAN; }
      CHECK(z >= c.floor, "z %ld under the floor", z);
      if (p.is_x && (std::fabs(y - y_before) > 1e-9 || x < -11.0001 || x > -5 + 1e-9)) x_ok = false;
      double true_f = force_at(z);
      const bool pressing = s.press > 0 && t >= s.press_after_s;
      if (pressing) true_f += s.press;
      fmax_true = std::max(fmax_true, true_f);
      const double seen = 0.3 * true_f + 0.7 * prev;          // one sample of lag
      prev = true_f;
      const double reading = seen + s.drift_per_s * t + s.noise * frand() + (i % 37 == 5 ? 900 : 0);
      const int32_t f = int32_t(std::fabs(reading));
      ++samples_total;
      const bool was_feed = feed;
      if (pad_wipe_control(c, f, z, z_goal, feed, band) == WIPE_END) {
        ended = 1; z = long(5 * SPM);
        if (pressing) end_after_press = int((t - s.press_after_s) / (SAMPLE_US * 1e-6));
        break;
      }
      if (was_feed && !feed && v > 0) stop_from = pos;
      if (z <= 0 && !p.is_x) { ++samples_on; if (band) ++in_band_on; }
      if (!left) break;
    }
  }
  const double in_band = samples_on ? double(in_band_on) / samples_on : 0;
  std::printf("%-20s end=%d t=%5.2fs samples=%4ld feed/sample=%.3fmm vpeak=%4.1f stop=%.3fmm in_band=%3.0f%% max_true=%6.0f\n",
              s.name, ended, t, samples_total, max_feed_per_sample, vpeak, max_stop_mm, 100 * in_band, fmax_true);
  CHECK(ended == s.expect_end, "end=%d, expected %d", ended, s.expect_end);
  CHECK(max_feed_per_sample <= 0.35, "feed moved %.3f mm between two samples", max_feed_per_sample);
  CHECK(max_stop_mm <= 0.6, "feed took %.3f mm to stop after the load went over the band", max_stop_mm);
  CHECK(x_ok, "the flick left its path (x %.2f)", x);
  if (!s.expect_end) {
    CHECK(vpeak >= 0.95 * VMAX, "peak speed only %.1f mm/s", vpeak);
    CHECK(std::fabs(x + 11) < 0.02 && std::fabs(y - 50) < 0.02, "ended at X%.2f Y%.2f, expected the flick end X-11 Y50", x, y);
    CHECK(in_band >= s.min_in_band, "load in the band for only %.0f%% of the samples", 100 * in_band);
    CHECK(fmax_true < c.hard, "true load %.0f reached the hard limit", fmax_true);
  }
  if (s.press >= 35000) CHECK(end_after_press >= 0 && end_after_press <= 2, "hard press ended the wipe after %d samples", end_after_press);
  if (s.press > 0 && s.press < 35000) CHECK(end_after_press >= 0 && end_after_press <= 2 * int((c.ceil - c.floor) / c.up) + 6,
                                            "soft press ended the wipe after %d samples", end_after_press);
}

int main() {
  std::srand(1);
  const Sim sims[] = {
    // name                k_mm     bump_y bump  ramp  press_s press  drift noise end in_band
    { "strip ~1 kg/mm",    100000,  -1,    0,    0,    -1,     0,     0,    450,  0,  0.6 },
    { "stiff 1.4 kg/mm",   140000,  -1,    0,    0,    -1,     0,     0,    450,  0,  0.6 },
    { "soft 0.3 kg/mm",     30000,  -1,    0,    0,    -1,     0,     0,    450,  0,  0.0 },
    { "drift +300/s",      100000,  -1,    0,    0,    -1,     0,     300,  450,  0,  0.5 },
    { "drift -300/s",      100000,  -1,    0,    0,    -1,     0,    -300,  450,  0,  0.5 },
    { "deposit 0.3 ramp",  100000,  35,    0.3,  3,    -1,     0,     0,    450,  0,  0.4 },
    { "hand press 400 g",  100000,  -1,    0,    0,    1.5,    40000, 0,    450,  1,  0.0 },
    { "hand press 250 g",  100000,  -1,    0,    0,    1.5,    25000, 0,    450,  1,  0.0 },
    { "vertical deposit",  140000,  35,    0.6,  0.02, -1,     0,     0,    450,  1,  0.0 },
  };
  for (const Sim &s : sims) run(s);
  return fails ? 1 : 0;
}
'''

def main():
    with tempfile.TemporaryDirectory() as tmp:
        src = Path(tmp) / "wipe.cpp"
        exe = Path(tmp) / "wipe"
        src.write_text(HARNESS)
        subprocess.run(["g++", "-std=c++17", "-O1", "-Wall", "-Werror", "-I", str(ROOT), str(src), "-o", str(exe)], check=True)
        r = subprocess.run([str(exe)], capture_output=True, text=True)
        print(r.stdout, end="")
        assert r.returncode == 0, "wipe simulation failed"
    print("test_pad_scrape: OK")

if __name__ == "__main__":
    main()
