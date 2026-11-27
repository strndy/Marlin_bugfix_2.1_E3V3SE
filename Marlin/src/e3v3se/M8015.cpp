/**
 * @file M8015.cpp
 * @author creality
 *
 * @version 0.1
 * @date 2021-12-15
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "../gcode/gcode.h"
#include "../module/planner.h"
#include "../module/stepper.h"
#include "../module/probe.h"
// #include "../../../Configuration_adv.h"
#include "../feature/bedlevel/bedlevel.h"
#include "../feature/bedlevel/abl/bbl.h"
#include "AutoOffset.h"
#include "PadHoming.h"
#include "../lcd/dwin/creality/dwin.h"


/**
 *One key to obtain z-axis offset value
 */
void GcodeSuite::M8015()
{
  #if ENABLED(PAD_Z_HOMING)
    // M8015 T[seconds]: live strain-gauge readings for the pad Z homing self-test (no motion)
    if (parser.seen('T')) pad_sensor_report(parser.has_value() ? constrain(parser.value_int(), 1, 60) : 10);
    else SERIAL_ECHOLNPGM("M8015: automatic Z-offset is not available with PAD_Z_HOMING (M8015 T shows the sensor)");
    return; // The measuring routine assumes probe-homed Z; it must never run here
  #endif
  const bool wLevel = parser.seen('S') ? parser.value_bool() : true; // S0: Get Z offset without leveling; S1: Get Z offset with leveling
  // SERIAL_ECHOLNPGM("M8015: Trying to get Z offset value...");
  float zOffset = 0;
  if (motion.axis_is_trusted(Z_AXIS))
  {
    DISABLE_AXIS_Z();   // Release the z-axis motor. If not released, the second adjustment data will be abnormal and cannot be cleared.
  }
  // gcode.process_subcommands_now_P(PSTR("G28"));
  // probe.auto_get_offset(); //One-click high logic
  HMI_flag.leveling_offset_flag = true;
  HMI_flag.Pressure_Height_end = false;
  checkkey = ONE_HIGH;
  if (getZOffset(1, 1, 1, &zOffset))
  {
    for (int x = 0; x < GRID_MAX_POINTS_X; x++)
      for (int y = 0; y < GRID_MAX_POINTS_Y; y++)
        bedlevel.z_values[x][y] = 0;
    bedlevel.refresh_bed_level();
    settings.save(); // Commit the validated offset and the cleared mesh together.
    if (HMI_flag.Need_boot_flag) // Booting
    {
      HMI_flag.boot_step = Set_levelling; // Set the current step to the boot completion flag and save it
      // Save_Boot_Step_Value();//Save the boot boot steps
    }
    // else
    // {
    probe.offset.z = zOffset;
    // probe.offset.z -= NOZ_AUTO_OFT_MM; // Press down the final Z-axis compensation value by 0.02mm. Rock_20230516
    // TERN_(EEPROM_SETTINGS, settings.save());
    // TERN_(USE_AUTOZ_TOOL_2, DWIN_CompletedHeight());
    SERIAL_ECHOLNPGM("M8015 succeeded in getting Z offset.");
    SERIAL_ECHOLN("Z Offset: ", probe.offset.z);
    if (wLevel){

      HMI_flag.leveling_offset_flag = false;
      HMI_flag.Pressure_Height_end = true;

    }else{
    Goto_MainMenu();
    }
  }
  else // Failed against high
  {
    HMI_flag.leveling_offset_flag=false; //test failed
    DWIN_CalibrationFailed();
    SERIAL_ECHOLNPGM("M8015 failed to get Z offset.");
  }
}

#if ENABLED(PAD_SCRAPE)
/**
 * M8016: rough touch + nozzle wipe only, for tuning (pad Z homing; X/Y/Z must be homed).
 * Goes to X/R at least 5 mm over the expected strip height, takes the free reference there,
 * touches down (fast-change detection, at most 1 mm under the expected height), lifts, moves to
 * Y at travel height, takes a reference again, wipes P passes between Y and J holding the load at
 * S +- W (never deeper than D below the rough contact), flicks to K, lifts. Defaults from PAD_SCRAPE_*.
 *   X<mm> strip X    R<mm> rough touch Y    Y<mm> wipe start    J<mm> turning point    P<n> passes
 *   D<mm>            max depth below the rough contact (0..0.4)
 *   S<counts> W<counts>  load held and band (~100 counts per gram)
 *   L<counts>        hard limit: lift at once, wipe ended
 *   F<mm/s>          wipe speed (1..50)
 *   K<mm>            flick to this X at the end (K = X: no flick)
 *   A<mm>            zigzag: one way Y -> J in 45 degree diagonals, X between X and X+A (A0: back and forth with P passes)
*   E0 / E1          switch the wipe in G28 off / on (e.g. while a probe is clipped on); nothing else runs
 *   Z<mm>            expected strip height relative to homed Z0 (default 0). Z10 tests in the air:
 *                    no contact down to Z9, so it stops with "no strip contact in range".
 */
void GcodeSuite::M8016() {
  // M8016 E0 / E1: switch the wipe in G28 off / on (e.g. while a probe is clipped on). Not saved.
  if (parser.seenval('E')) {
    pad_scrape_enabled = parser.value_bool();
    SERIAL_ECHOLNPGM("PAD_SCRAPE: wipe in G28 ", pad_scrape_enabled ? "ON" : "OFF");
    return;
  }
  PadScrapeParams p = pad_scrape_defaults();
  if (parser.seenval('X')) p.x = parser.value_float();
  if (parser.seenval('R')) p.rough_y = parser.value_float();
  if (parser.seenval('Y')) p.y0 = parser.value_float();
  if (parser.seenval('J')) p.y1 = parser.value_float();
  if (parser.seenval('P')) p.passes = constrain(parser.value_int(), 1, 20);
  if (parser.seenval('D')) p.depth = parser.value_float();
  if (parser.seenval('S')) p.target = parser.value_long();
  if (parser.seenval('W')) p.band = parser.value_long();
  if (parser.seenval('L')) p.hard = parser.value_long();
  if (parser.seenval('F')) p.speed = parser.value_float();
  if (parser.seenval('K')) p.flick_x = parser.value_float();
  if (parser.seenval('A')) p.zigzag = parser.value_float();
  if (parser.seenval('Z')) p.contact = parser.value_float();
  if (!(p.hard <= 60000 && p.band > PAD_NOISE_MAX && p.target > p.band && p.target + p.band < p.hard)
    || !WITHIN(p.x, X_MIN_POS, X_MAX_POS) || !WITHIN(p.flick_x, X_MIN_POS, p.x) || !WITHIN(p.rough_y, Y_MIN_POS, Y_MAX_POS)
    || !WITHIN(p.y0, Y_MIN_POS, Y_MAX_POS) || !WITHIN(p.y1, Y_MIN_POS, Y_MAX_POS) || ABS(p.y1 - p.y0) < 5
    || !WITHIN(p.x + p.zigzag, X_MIN_POS, X_MAX_POS) || ABS(p.zigzag) > 10
    || !WITHIN(p.depth, 0.0f, 0.4f) || !WITHIN(p.speed, 1.0f, 50.0f) || !WITHIN(p.contact, -2.0f, 50.0f)) {
    SERIAL_ECHOLNPGM("M8016: bad parameter (need ", PAD_NOISE_MAX, " < W < S, S + W < L <= 60000, X/R/Y/J in range, |J-Y| >= 5,"
                     " K between X_MIN_POS and X, X+A in range, |A| <= 10, D 0..0.4, F 1..50, Z -2..50)");
    return;
  }
  pad_scrape_only(p);
}
#endif
