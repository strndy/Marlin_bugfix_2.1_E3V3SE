/**
 * E3V3SE manual ("hand-turn") Z homing. See ManualZHome.h and the README.
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(MANUAL_Z_HOMING)

#include "ManualZHome.h"

#include "../MarlinCore.h"
#include "../gcode/gcode.h"
#include "../module/motion.h"
#include "../module/planner.h"
#include "../module/endstops.h"
#include "../module/temperature.h"
#include "../module/stepper/indirection.h"
#include "../sd/cardreader.h"

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../feature/host_actions.h"
#endif

#if DISABLED(DWIN_CREALITY_LCD)
  #error "MANUAL_Z_HOMING requires DWIN_CREALITY_LCD (the prompt is drawn by the Creality DWIN UI)."
#elif !AXIS_HAS_UART(Z)
  #error "MANUAL_Z_HOMING requires a TMC UART driver on Z. The enable pin is shared, so Z can only be released over UART."
#elif !HAS_HOTEND || !HAS_EXTRUDERS
  #error "MANUAL_Z_HOMING requires a hotend and an extruder."
#elif ANY(DELTA, AXEL_TPARA, HOME_Z_FIRST, FOAMCUTTER_XYUV) || !Z_HOME_TO_MIN
  #error "MANUAL_Z_HOMING requires a Cartesian machine that homes Z to MIN after X and Y."
#endif

static_assert(MANUAL_Z_HOME_TEMP == 0 || (MANUAL_Z_HOME_TEMP >= 100 && MANUAL_Z_HOME_TEMP <= (HEATER_0_MAXTEMP) - 15), "MANUAL_Z_HOME_TEMP must be 0 (no heating) or 100..(HEATER_0_MAXTEMP - 15).");
static_assert(WITHIN(MANUAL_Z_HOME_X_POINT, X_MIN_POS, X_MAX_POS) && WITHIN(MANUAL_Z_HOME_Y_POINT, Y_MIN_POS, Y_MAX_POS), "MANUAL_Z_HOME_X/Y_POINT must be reachable by the nozzle.");
static_assert(WITHIN(MANUAL_Z_HOME_RETRACT, 0, 10), "MANUAL_Z_HOME_RETRACT must be 0..10 mm.");
static_assert(WITHIN(MANUAL_Z_HOME_OFFSET, -2, 2), "MANUAL_Z_HOME_OFFSET must be -2..2 mm.");
static_assert(WITHIN(MANUAL_Z_HOME_CLEARANCE, 1, 50) && (MANUAL_Z_HOME_CLEARANCE) > (MANUAL_Z_HOME_OFFSET), "MANUAL_Z_HOME_CLEARANCE must be 1..50 mm and above MANUAL_Z_HOME_OFFSET.");
static_assert(WITHIN(MANUAL_Z_HOME_TIMEOUT_SEC, 30, 3600), "MANUAL_Z_HOME_TIMEOUT_SEC must be 30..3600.");

constexpr int16_t touch_temp = MANUAL_Z_HOME_TEMP;       // 0 = touch off cold
constexpr float touch_retract = MANUAL_Z_HOME_RETRACT;

ManualZHome manual_z;

bool ManualZHome::locked; // = false
int16_t ManualZHome::saved_target = -1;

//
// Z driver output stage. X/Y/Z/E share one enable pin on this board, so the only way
// to free Z alone is TOFF=0 in the Z driver's CHOPCONF (all bridges off, motor freewheels).
//
static bool z_toff_is(const uint8_t want) {
  #if ENABLED(MANUAL_Z_HOME_VERIFY_UART)
    const uint32_t chopconf = stepperZ.CHOPCONF();  // A missing or corrupt reply sets CRCerror
    return !stepperZ.CRCerror && (chopconf & 0x0F) == want;
  #else
    UNUSED(want);
    return true;
  #endif
}

static bool z_set_toff(const uint8_t toff) {
  for (uint8_t tries = 3; tries--;) {
    stepperZ.toff(toff);
    safe_delay(20);
    if (z_toff_is(toff)) return true;
  }
  return false;
}

static inline bool z_driver_responds() { return z_toff_is(chopper_timing_Z.toff); }
static inline bool z_driver_release()  { return z_set_toff(0); }
static inline bool z_driver_engage()   { return z_set_toff(chopper_timing_Z.toff); }

enum ManualZFail : uint8_t { MZF_NONE, MZF_XY, MZF_TIMEOUT, MZF_CANCEL, MZF_HOST, MZF_DRIVER, MZF_ENGAGE };

// Wait for the knob while keeping heaters, watchdog, UI and serial serviced.
// X/Y stay energised: idle(true) holds off the stepper idle timeout.
static ManualZFail wait_for_knob(const bool heating) {
  const millis_t timeout_ms = millis() + SEC_TO_MS(MANUAL_Z_HOME_TIMEOUT_SEC);
  millis_t next_report_ms = 0;
  marlin.heatup_start();                // M108 (emergency parser) clears this and aborts
  for (;;) {
    marlin.idle(true);
    const millis_t ms = millis();
    if (heating) {
      if (thermalManager.wholeDegHotend(0) >= touch_temp - 2) return MZF_NONE;
      if (ELAPSED(ms, next_report_ms)) {
        next_report_ms = ms + 5000UL;
        SERIAL_ECHOLNPGM("MANUAL_Z: heating ", thermalManager.wholeDegHotend(0), "/", touch_temp);
      }
    }
    const ManualZReply reply = DWIN_ManualZ_Reply();
    if (!heating && reply == MZR_CONFIRM) return MZF_NONE;
    if (reply == MZR_CANCEL) return MZF_CANCEL;
    if (!marlin.is_heating() || card.flag.abort_sd_printing) return MZF_HOST;  // M108 / M524
    if (ELAPSED(ms, timeout_ms)) return MZF_TIMEOUT;
  }
}

void ManualZHome::preheat() {
  saved_target = thermalManager.degTargetHotend(0);
  if (touch_temp > 0) {
    thermalManager.setTargetHotend(touch_temp, 0);
    SERIAL_ECHOLNPGM("MANUAL_Z: heating nozzle to ", touch_temp, " while homing XY");
  }
}

bool ManualZHome::home(const bool move_xy) {
  ManualZFail fail = MZF_NONE;
  bool released = false, engaged = true;

  const int16_t restore_target = saved_target < 0 ? thermalManager.degTargetHotend(0) : saved_target;
  saved_target = -1;

  // The probe / Z-min input may float with the probe unplugged. Nothing below
  // may be stopped by it. G28 restores the endstop state when it finishes.
  endstops.enable(false);

  planner.synchronize();
  motion.sync_plan_position();

  if (motion.homing_needed_error(_BV(X_AXIS) | _BV(Y_AXIS)))
    fail = MZF_XY;

  if (!fail && move_xy) {
    constexpr xy_pos_t touch_xy = { MANUAL_Z_HOME_X_POINT, MANUAL_Z_HOME_Y_POINT };
    motion.blocking_move(touch_xy, motion.homing_feedrate(X_AXIS));
    planner.synchronize();
  }

  if (!fail && touch_temp > 0) {
    // Normally set by preheat() already. Use the same temperature every time.
    thermalManager.setTargetHotend(touch_temp, 0);
    SERIAL_ECHOLNPGM("MANUAL_Z: heating, please wait (knob click or M108 cancels)");
    DWIN_ManualZ_Show(MZS_HEATING);
    fail = wait_for_knob(true);

    if (!fail && touch_retract > 0 && !thermalManager.tooColdToExtrude(0)) {
      // Pull the melt back a little. The logical E position is restored, so the
      // filament stays retracted for the print's own prime line / skirt to make up.
      const float e_before = motion.position.e;
      motion.position.e -= touch_retract;
      motion.goto_current_position(MMM_TO_MMS(1200));
      planner.synchronize();
      motion.position.e = e_before;
      motion.sync_plan_position_e();
    }
  }

  if (!fail) {
    planner.synchronize();
    if (!z_driver_responds())
      fail = MZF_DRIVER;                // Nothing was written: Z is still held where it was
    else {
      // From here the Z position is unknown until the knob is clicked
      motion.set_axis_never_homed(Z_AXIS);
      released = true;
      if (z_driver_release()) {
        SERIAL_ECHOLNPGM("MANUAL_Z: Z motor released. Clean the nozzle, lower Z by hand until it touches the bed, then click the knob (M108 cancels)");
        DWIN_ManualZ_Show(MZS_TOUCH);
        KEEPALIVE_STATE(PAUSED_FOR_USER);
        fail = wait_for_knob(false);
      }
      else
        fail = MZF_DRIVER;              // Written but not confirmed: switched back on below
    }
  }

  // Every path that touched the driver ends with it switched on again, and checked
  if (released) {
    engaged = z_driver_engage();
    safe_delay(250);                    // Let the rotor settle on its holding position
    if (!engaged) fail = MZF_ENGAGE;
  }

  if (!fail) {
    // The nozzle is touching the bed here
    motion.position.z = (MANUAL_Z_HOME_OFFSET) + TERN0(HAS_HOME_OFFSET, motion.home_offset.z);
    TERN_(HAS_WORKSPACE_OFFSET, motion.workspace_offset.z = 0);
    motion.sync_plan_position();
    motion.set_axis_trusted(Z_AXIS);
    motion.set_axis_homed(Z_AXIS);
    locked = false;
    SERIAL_ECHOLNPGM("MANUAL_Z: confirmed, Z set to ", motion.position.z);
    motion.blocking_move_z(MANUAL_Z_HOME_CLEARANCE, motion.homing_feedrate(Z_AXIS));
    planner.synchronize();
    // Back to the target from before G28. If the nozzle was off, it is off again.
    thermalManager.setTargetHotend(restore_target, 0);
    DWIN_ManualZ_End(true);
    SERIAL_ECHOLNPGM("MANUAL_Z: done, Z at ", motion.position.z, ", nozzle target ", restore_target);
    return true;
  }

  //
  // Failed: heater off, Z unknown, no further G-code moves until Z is homed
  //
  thermalManager.setTargetHotend(0, 0);
  motion.set_axis_never_homed(Z_AXIS);
  locked = true;

  FSTR_P const reason =
      fail == MZF_XY      ? F("X/Y are not homed")
    : fail == MZF_TIMEOUT ? F("timeout")
    : fail == MZF_CANCEL  ? F("cancelled at the knob")
    : fail == MZF_HOST    ? F("cancelled by host (M108/M524)")
    : fail == MZF_DRIVER  ? F("Z driver not responding")
    :                       F("Z DRIVER STILL OFF - POWER CYCLE");
  SERIAL_ECHOLN(F("MANUAL_Z: FAILED, "), reason, F(". Nozzle heater off, Z NOT homed, moves blocked until G28 Z succeeds"));

  // Z was moved by hand and is now unknown. Assume the lowest possible value, so the real
  // nozzle is never below where the firmware thinks it is and any later absolute Z move
  // (some LCD flows make one after G28) can only end up too high. Then raise by the
  // clearance so a hot nozzle isn't left sitting on the bed.
  if (released && engaged) {
    motion.position.z = 0;
    motion.sync_plan_position();
    motion.blocking_move_z(MANUAL_Z_HOME_CLEARANCE, motion.homing_feedrate(Z_AXIS));
    planner.synchronize();
  }

  const bool sd_job = card.isPrinting();

  DWIN_ManualZ_Show(MZS_FAILED, reason);

  // The Z driver could not be switched back on: nothing can be trusted to move Z.
  // Halt with the message on screen. A power cycle re-initialises the driver.
  if (fail == MZF_ENGAGE) marlin.kill(reason);

  for (const millis_t end_ms = millis() + 4000UL; PENDING(millis(), end_ms);) marlin.idle(true);
  DWIN_ManualZ_End(false);              // Also aborts a running SD print

  if (!sd_job) {
    // A host can't be stopped from here. Ask it to cancel; its moves are refused anyway.
    #ifdef ACTION_ON_CANCEL
      hostui.cancel();
    #endif
    if (marlin.printJobOngoing()) SERIAL_ERROR_MSG("Manual Z homing failed");
  }
  return false;
}

#endif // MANUAL_Z_HOMING
