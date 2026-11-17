#!/usr/bin/env python3
"""Run the production wipe load watch (Marlin/src/e3v3se/PadScrapeCtl.h) inside a copy of the
PadHoming.cpp wipe loop against a simulated strip: soft strip, stiff surface, a blob on the
nozzle that wears off, deposits on the strip, thermal drift, noise, single-sample spikes and
one sample of HX711 lag. Checks: never below the depth floor, Y moves only at the target
height, soft limit -> jumps, hard limit -> the wipe ends."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "Marlin/src/e3v3se/PadScrapeCtl.h"

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { std::printf("FAIL %s: ", s.name); std::printf(__VA_ARGS__); std::printf("\n"); ++fails; } } while (0)

struct Sim {
  const char *name;
  double k_mm;             // strip stiffness, counts per mm
  double blob_mm, blob_k;  // plastic under the tip at the start, its stiffness (counts/mm)
  double wear;             // blob mm removed per (count * mm of Y)
  double bump_y, bump_mm, ramp; // deposit on the strip: rises over 'ramp' mm, 2 mm flat, falls
  double drift_per_s, noise;
  int expect_end;          // 0 = both segments complete, 1 = the wipe ends (hard limit)
  int min_jumps;
  double press_y, press;   // a hand presses the bed (load independent of Z) from this Y on
};

static double frand() { return std::rand() / double(RAND_MAX) * 2 - 1; }

static void run(const Sim &s) {
  const double spm = 400, spm_y = 80;                          // Z 400 steps/mm, Y 80 steps/mm
  const long travel = 5 * 400;
  // Rough contact (z = 0) is where the first thing touched: the blob if there is one
  const double contact_mm = s.blob_mm;
  const PadWipeCtl c = { 20000, 35000, long(0.1 * spm), -long(0.4 * spm), long(1.0 * spm) };
  long z = travel;                       // the wipe starts at travel height over Y0 = 50
  double y = 50, blob = s.blob_mm, t = 0, prev = 0, fmax = 0;
  int jumps = 0, ended = 0;
  auto force_at = [&](long zz) {
    const double tip = zz / spm + contact_mm - blob;              // nozzle tip + remaining blob, strip frame
    double surf = -0.001 * (50 - y);                              // strip not level
    if (s.bump_mm > 0) {
      const double u = s.bump_y - y, r = s.ramp;                  // the wipe moves towards -Y
      if (u > 0 && u < r) surf += s.bump_mm * u / r;
      else if (u >= r && u < r + 2) surf += s.bump_mm;
      else if (u >= r + 2 && u < 2 * r + 2) surf += s.bump_mm * (2 * r + 2 - u) / r;
    }
    const double pen = surf - tip;
    return pen <= 0 ? 0.0 : pen * (blob > 0.01 ? s.blob_k : s.k_mm);
  };
  const double depth[2] = { 0.1, 0.2 }, y_end[2] = { 40, 20 };
  // Timing as in PadHoming.cpp pad_wipe(): one HX711 conversion per 12.5 ms; while it converts, Z steps
  // towards this sample's goal (625 us per step = 4 mm/s) and, only with Z at its target, Y steps
  // (2500 us per step = 5 mm/s). Nothing else may block the loop.
  const long sample_us = 12500, z_us = 625, y_us = 2500;
  int press_start = -1, press_end_lat = -1;
  for (int seg = 0; seg < 2 && !ended; ++seg) {
    long target = -long(std::lround(depth[seg] * spm));
    if (target < c.floor) target = c.floor;
    uint8_t quiet = 0;
    long y_left = std::lround((y - y_end[seg]) * spm_y), y_wiped = 0, samples = 0, last_jump_z = 0;
    bool jumped = false;
    for (int i = 0; i < 20000; ++i) {
      const long z_goal = target >= z ? target : std::max(target, z - (z > long(0.3 * spm) ? 20L : 4L));
      for (long us = 0; us < sample_us; ) {
        if (z != z_goal) { z += z_goal > z ? 1 : -1; us += z_us; continue; }
        if (y_left && z == target) {
          --y_left; ++y_wiped; y -= 1 / spm_y;
          if (blob > 0) blob = std::fmax(0.0, blob - s.wear * prev / spm_y);
          us += y_us; continue;
        }
        break;
      }
      CHECK(z >= c.floor, "z %ld under the floor", z);
      // A jump above the current height is under way (20 steps = 0.05 mm per sample) by the next sample
      if (jumped) { CHECK(z >= last_jump_z, "jump not under way within one sample (z %ld, expected >= %ld)", z, last_jump_z); jumped = false; }
      double true_f = force_at(z);
      const bool pressing = s.press > 0 && y <= s.press_y;
      if (pressing) { true_f += s.press; if (press_start < 0) press_start = i; }
      if (true_f > fmax) fmax = true_f;
      const double seen = 0.3 * true_f + 0.7 * prev;             // one sample of lag
      prev = true_f;
      const double reading = seen + s.drift_per_s * t + s.noise * frand() + (i % 37 == 5 ? 900 : 0);
      const int32_t f = int32_t(std::fabs(reading));
      t += sample_us * 1e-6;
      ++samples;
      const long z_before = z;
      const PadWipeStep r = pad_wipe_control(c, f, target, quiet);
      CHECK(target >= c.floor && target <= c.ceil, "target %ld out of range", target);
      if (r == WIPE_END) { ended = 1; z = travel; if (press_start >= 0) press_end_lat = i - press_start; break; }
      if (r == WIPE_JUMP) { ++jumps; jumped = target > z_before; last_jump_z = std::min(target, z_before + 20); }
      if (!y_left && z == target) break;
    }
    // Samples are taken all along the wipe: at 5 mm/s and 80/s that is 16 per mm (jumps add more)
    if (!ended && y_wiped) {
      const double per_mm = samples / (y_wiped / spm_y);
      CHECK(per_mm >= 14, "only %.1f samples per mm of wipe", per_mm);
    }
  }
  if (s.press > 0) {
    CHECK(ended, "a hand press of %.0f did not end the wipe", s.press);
    // Hard press: within 2 samples (one of HX711 lag). Soft press: jumps up to the lift limit first.
    const int max_lat = s.press > c.hard ? 2 : 2 * int((c.ceil - c.floor) / c.jump) + 4;
    CHECK(press_end_lat >= 0 && press_end_lat <= max_lat, "press ended the wipe after %d samples (max %d)", press_end_lat, max_lat);
  }
  std::printf("%-20s end=%d y=%5.1f jumps=%2d max_true_load=%6.0f blob_left=%.3f\n", s.name, ended, y, jumps, fmax, blob);
  CHECK(ended == s.expect_end, "end=%d, expected %d", ended, s.expect_end);
  CHECK(jumps >= s.min_jumps, "jumps=%d, expected >= %d", jumps, s.min_jumps);
  if (!s.expect_end) {
    CHECK(y <= 20.01, "wipe stopped at y=%.1f", y);
    CHECK(fmax < c.hard, "true load %.0f reached the hard limit", fmax);
  }
}

int main() {
  std::srand(1);
  const Sim sims[] = {
    // name               k_mm    blob blob_k  wear  bump_y bump ramp drift noise end jumps press_y press
    { "clean soft strip",  30000, 0,   0,      0,    -1,    0,   0,   0,    450,  0,  0,  -1,    0 },
    { "stiff surface",    140000, 0,   0,      0,    -1,    0,   0,   0,    450,  0,  1,  -1,    0 },
    { "drift +300/s",      30000, 0,   0,      0,    -1,    0,   0,   300,  450,  0,  0,  -1,    0 },
    { "drift -300/s",      30000, 0,   0,      0,    -1,    0,   0,  -300,  450,  0,  0,  -1,    0 },
    { "blob wears off",    30000, 0.3, 300000, 2e-6, -1,    0,   0,   100,  450,  0,  1,  -1,    0 },
    { "ramped deposit",    30000, 0,   0,      0,    35,    0.5, 2,   0,    450,  0,  1,  -1,    0 },
    { "stiff deposit",    140000, 0,   0,      0,    35,    0.3, 3,   0,    450,  0,  1,  -1,    0 },
    { "hand press 400 g",  30000, 0,   0,      0,    -1,    0,   0,   0,    450,  1,  0,   30,    40000 },
    { "hand press 250 g",  30000, 0,   0,      0,    -1,    0,   0,   0,    450,  1,  1,   30,    25000 },
    { "vertical step",    140000, 0,   0,      0,    35,    0.5, 0.01, 0,   450,  1,  0,  -1,    0 },
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
