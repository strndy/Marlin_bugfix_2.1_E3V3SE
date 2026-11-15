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
  // On success Z is homed and the nozzle is PAD_CLEARANCE_AFTER_MM above the pad.
  // On ANY failure it calls kill(): heaters off, printer halted until reset.
  void pad_home_z();

  // M8015 T[seconds]: print live HX711 readings without moving anything.
  void pad_sensor_report(const uint8_t seconds);

#endif
