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
 * Y at travel height, takes a reference again, wipes Y->I at contact-A and I->J at contact-B with
 * the load watched live, lifts at J. Defaults from PAD_SCRAPE_*.
 *   X<mm> strip X    R<mm> rough touch Y    Y<mm> wipe start    I<mm> depth change    J<mm> wipe end
 *   A<mm> B<mm>      depths below the rough contact (0..0.4)
 *   S<counts>        soft limit: jump up by H<mm> and continue (~100 counts per gram)
 *   L<counts>        hard limit: lift at once, wipe ended
 *   F<mm/s>          Y speed while wiping (1..20, default 5)
 *   Z<mm>            expected strip height relative to homed Z0 (default 0). Z10 tests in the air:
 *                    no contact down to Z9, so it stops with "no strip contact in range".
 */
void GcodeSuite::M8016() {
  PadScrapeParams p = pad_scrape_defaults();
  if (parser.seenval('X')) p.x = parser.value_float();
  if (parser.seenval('R')) p.rough_y = parser.value_float();
  if (parser.seenval('Y')) p.y0 = parser.value_float();
  if (parser.seenval('I')) p.ymid = parser.value_float();
  if (parser.seenval('J')) p.y1 = parser.value_float();
  if (parser.seenval('A')) p.depth1 = parser.value_float();
  if (parser.seenval('B')) p.depth2 = parser.value_float();
  if (parser.seenval('H')) p.jump = parser.value_float();
  if (parser.seenval('S')) p.soft = parser.value_long();
  if (parser.seenval('L')) p.hard = parser.value_long();
  if (parser.seenval('F')) p.speed = parser.value_float();
  if (parser.seenval('Z')) p.contact = parser.value_float();
  if (!(p.hard <= 60000 && p.soft > PAD_SCRAPE_TRIGGER && p.soft < p.hard)
    || !WITHIN(p.x, X_MIN_POS, X_MAX_POS) || !WITHIN(p.rough_y, Y_MIN_POS, Y_MAX_POS) || !WITHIN(p.y0, Y_MIN_POS, Y_MAX_POS)
    || !WITHIN(p.ymid, Y_MIN_POS, Y_MAX_POS) || !WITHIN(p.y1, Y_MIN_POS, Y_MAX_POS) || (p.ymid - p.y0) * (p.y1 - p.ymid) <= 0
    || !WITHIN(p.depth1, 0.0f, 0.4f) || !WITHIN(p.depth2, 0.0f, 0.4f) || !WITHIN(p.jump, 0.02f, 0.5f)
    || !WITHIN(p.speed, 1.0f, 20.0f) || !WITHIN(p.contact, -2.0f, 50.0f)) {
    SERIAL_ECHOLNPGM("M8016: bad parameter (need ", PAD_SCRAPE_TRIGGER, " < S < L <= 60000, X/R/Y/I/J in range with I between Y and J,"
                     " A/B 0..0.4, H 0.02..0.5, F 1..20, Z -2..50)");
    return;
  }
  pad_scrape_only(p);
}
#endif
