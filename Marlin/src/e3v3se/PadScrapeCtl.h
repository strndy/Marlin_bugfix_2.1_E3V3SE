/**
 * Force watch of the nozzle wipe (PAD_SCRAPE): one call per HX711 sample.
 *
 * The wipe holds a set depth below the strip contact. Over the soft limit the nozzle jumps up
 * by 'jump' and carries on from there; over the hard limit (or jumped past 'ceil') the wipe ends.
 * Plain C++ with no Marlin dependency, so the host test (test/test_pad_scrape.py) runs this exact
 * code against a simulated strip. Heights are Z steps, larger = higher.
 */
#pragma once

#include <stdint.h>

struct PadWipeCtl {
  int32_t soft,   // load (counts over the free reference) that makes the nozzle jump up
          hard;   // load that ends the wipe at once
  long jump,      // jump height (steps)
       floor,     // lowest height ever held: strip contact - max depth
       ceil;      // jumped up to here and still over the soft limit: end the wipe
};

enum PadWipeStep : uint8_t { WIPE_OK, WIPE_JUMP, WIPE_END };

/**
 * f: load now (counts, >= 0). target: the height to hold, raised by 'jump' on WIPE_JUMP.
 * quiet: samples left in which the soft limit is not checked again; the sample right after a
 * jump was mostly converted before it (one sample of HX711 lag), so it would jump twice.
 * WIPE_END: hard limit, or still over the soft limit at 'ceil'. The caller lifts at once.
 */
inline PadWipeStep pad_wipe_control(const PadWipeCtl &c, const int32_t f, long &target, uint8_t &quiet) {
  if (f > c.hard) return WIPE_END;
  if (target < c.floor) target = c.floor;
  if (quiet) { --quiet; return WIPE_OK; }
  if (f <= c.soft) return WIPE_OK;
  if (target >= c.ceil) return WIPE_END;
  target += c.jump;
  if (target > c.ceil) target = c.ceil;
  quiet = 1;
  return WIPE_JUMP;
}
