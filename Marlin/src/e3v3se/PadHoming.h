/**
 * Pad Z homing (PAD_Z_HOMING)
 *
 * Home Z by touching the nozzle on the bed strain-gauge pad (HX711, the sensor
 * used by the automatic Z-offset) instead of homing with the CR Touch.
 * The probe stays configured (G29 still uses it) but is never deployed or read
 * while homing, so it can be unplugged for printing.
 *
 * See README "Pad Z homing".
 */
#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(PAD_SCRAPE) && DISABLED(PAD_Z_HOMING)
  #error "PAD_SCRAPE requires PAD_Z_HOMING."
#endif

#if ENABLED(PAD_Z_HOMING)

  #if DISABLED(USE_AUTOZ_TOOL_2)
    #error "PAD_Z_HOMING requires USE_AUTOZ_TOOL_2 (HX711 strain-gauge driver)."
  #elif !defined(Z_HOME_TO_MIN) || !HAS_Z_AXIS
    #error "PAD_Z_HOMING requires a Z axis that homes to MIN."
  #elif ENABLED(HOME_Z_FIRST)
    #error "PAD_Z_HOMING is incompatible with HOME_Z_FIRST."
  #elif ANY(DELTA, IS_SCARA, COREXZ, COREYZ, Z_MULTI_ENDSTOPS) || NUM_Z_STEPPERS > 1
    #error "PAD_Z_HOMING supports a cartesian machine with a single Z driver only."
  #endif

  // Called by G28 in place of the probe-based Z homing, after X/Y are homed.
  // true: Z is homed and the nozzle is about 1 mm above the pad (G28 then lifts it).
  // false: failed. The nozzle was lifted (at most 2 mm), heaters are off, Z is unhomed and
  // reads 0, the error is on serial and the LCD, an SD print is being aborted and the host
  // was told to cancel. G28 must then not move Z any further.
  [[nodiscard]] bool pad_home_z();

  // Called by G28 before X/Y homing when Z will be homed: remembers the current targets and
  // starts heating to the touch temperatures (print temperature if set, else the defaults).
  void pad_preheat();

  // M8015 T[seconds]: print live HX711 readings without moving anything.
  void pad_sensor_report(const uint8_t seconds);

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    // G29 makes the mesh relative to the pad spot (AUTOZ_TOOL_X/Y), the point Z is homed on, so
    // the probe's Z offset (M851 Z) cancels out. pad_mesh_probe: probe the bed at the pad spot after
    // the grid (NAN = not reachable; then the mesh is interpolated there). Returns false if the probe
    // failed (G29 then fails as for a grid point). pad_mesh_normalize: subtract the bed height at the
    // pad (probed, or the mesh there if ref is NAN) from the whole mesh.
    #define PAD_MESH_NORMALIZE 1
    [[nodiscard]] bool pad_mesh_probe(float &ref, const uint8_t verbose);
    void pad_mesh_normalize(float ref);
  #endif

  #if ENABLED(PAD_SCRAPE)
    // Nozzle wipe on the strip at X=PAD_SCRAPE_X (README "Pad Z homing", M8016).
    struct PadScrapeParams {
      float x, rough_y,        // strip X, rough touch Y
            y0, y1,            // wipe between these Y (starts at y0)
            depth,             // max wipe depth below the rough contact (mm, <= 0.4)
            speed,             // wipe speed (mm/s)
            flick_x,           // flick off the strip to this X at the end (= x: no flick)
            contact;           // M8016: expected strip height relative to the homed Z0 (mm)
      int32_t target, band,    // load held: target +- band (HX711 counts)
              hard;            // hard limit: lift, wipe ended
      uint8_t passes;          // wipe passes (y0->y1, y1->y0, ...)
    };
    // M8016: rough touch + wipe only (X/Y/Z homed). Never stops the machine; reports on serial.
    void pad_scrape_only(const PadScrapeParams &p);
    PadScrapeParams pad_scrape_defaults();
  #endif

#endif
