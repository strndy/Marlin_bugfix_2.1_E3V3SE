/**
 * Load control of the nozzle wipe (PAD_SCRAPE): one call per HX711 sample.
 *
 * Holds the load in target +- band: over the band Z goes up (and the feed stops), under it Z goes
 * back down towards the configured depth ('floor', never deeper). Over 'hard', or still over the
 * band at 'ceil', the wipe ends. Plain C++ with no Marlin dependency, so the host test
 * (test/test_pad_scrape.py) runs this exact code. Heights are Z steps, larger = higher, relative
 * to the rough contact.
 */
#pragma once

#include <stdint.h>

struct PadWipeCtl {
  int32_t target, band,  // load to hold, +- band (HX711 counts over the free reference)
          hard;          // load that ends the wipe at once
  long floor,            // lowest height: contact - depth (depth <= the 0.4 mm floor)
       ceil,             // over the band up here: the wipe ends
       near,             // above this the nozzle is still approaching: fast down, no feed
       up,               // Z per sample when over the band (what Z can do in one sample)
       down,             // Z per sample when under the band, at or below 'near'
       fast_down;        // Z per sample above 'near'
};

enum PadWipeStep : uint8_t { WIPE_OK, WIPE_END };

/**
 * f: load now (counts, >= 0). z: height now. z_goal: height to step to before the next sample.
 * feed: true = the wipe may move along (X/Y); false = stop the feed (approaching, or load too high).
 * in_band: the load is within target +- band.
 */
inline PadWipeStep pad_wipe_control(const PadWipeCtl &c, const int32_t f, const long z, long &z_goal, bool &feed, bool &in_band) {
  feed = false;
  in_band = false;
  z_goal = z;
  if (f > c.hard) return WIPE_END;
  if (f > c.target + c.band) {                         // too much: up, feed stops
    if (z >= c.ceil) return WIPE_END;
    z_goal = z + c.up < c.ceil ? z + c.up : c.ceil;
    return WIPE_OK;
  }
  if (f >= c.target - c.band) in_band = true;          // in the band: hold the height
  else {                                               // too little: down towards the depth
    const long d = z > c.near ? c.fast_down : c.down;
    z_goal = z - d > c.floor ? z - d : c.floor;
  }
  if (z_goal < c.floor) z_goal = c.floor;
  feed = z <= c.near;
  return WIPE_OK;
}

/**
 * Feed speed for the next step of a wipe pass (mm/s, 0 = stopped): accelerates from v0 to vmax
 * and slows down to v0 for the end of the pass ('left' steps) or, when 'feed' is off, to a stop.
 * dv2: change of v^2 per step (2 * acceleration / steps per mm). sqrtf: from the caller's math.
 */
template<typename SQRT_FN>
inline float pad_feed_speed(float v, const bool feed, const long left, const float vmax, const float v0, const float dv2, SQRT_FN sqrt_fn) {
  const float vcap = sqrt_fn(v0 * v0 + float(left) * dv2), vcmd = feed ? (vmax < vcap ? vmax : vcap) : 0.0f;
  if (v < vcmd) {
    if (v < v0) return v0;
    const float a = sqrt_fn(v * v + dv2);
    return a < vcmd ? a : vcmd;
  }
  if (v > vcmd) {
    const float d = v * v - dv2;
    if (d > v0 * v0) { const float a = sqrt_fn(d); return a > vcmd ? a : vcmd; }
    return vcmd > 0 ? v0 : 0.0f;
  }
  return v;
}
