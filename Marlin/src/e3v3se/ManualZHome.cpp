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
#elif ENABLED(SOFTWARE_DRIVER_ENABLE)
  #error "MANUAL_Z_HOMING can't be used with SOFTWARE_DRIVER_ENABLE (it would switch Z back on while it is turned by hand)."
#elif !HAS_HOTEND || !HAS_EXTRUDERS
  #error "MANUAL_Z_HOMING requires a hotend and an extruder."
#elif ANY(DELTA, AXEL_TPARA, HOME_Z_FIRST, FOAMCUTTER_XYUV) || !Z_HOME_TO_MIN
  #error "MANUAL_Z_HOMING requires a Cartesian machine that homes Z to MIN after X and Y."
#endif

static_assert(MANUAL_Z_HOME_TEMP == 0 || (MANUAL_Z_HOME_TEMP >= 100 && MANUAL_Z_HOME_TEMP <= (HEATER_0_MAXTEMP) - 15), "MANUAL_Z_HOME_TEMP must be 0 (no heating) or 100..(HEATER_0_MAXTEMP - 15).");
#if HAS_HEATED_BED
  static_assert(MANUAL_Z_HOME_BED_TEMP >= 0 && MANUAL_Z_HOME_BED_TEMP <= (BED_MAXTEMP) - 10, "MANUAL_Z_HOME_BED_TEMP must be 0..(BED_MAXTEMP - 10).");
#endif

constexpr int16_t touch_temp = MANUAL_Z_HOME_TEMP;  // 0 = touch off cold (no nozzle or bed heating)
constexpr int16_t nozzle_set_min = 50,              // A target below this counts as "not set"
                  bed_set_min = 30;

// Print temperature if one is set, else the default
static int16_t nozzle_temp_for(const int16_t target) { return target >= nozzle_set_min ? target : touch_temp; }
#if HAS_HEATED_BED
  static int16_t bed_temp_for(const int16_t target) { return target >= bed_set_min ? target : int16_t(MANUAL_Z_HOME_BED_TEMP); }
#endif
constexpr float touch_retract = 1.0f,               // (mm) Pulled back before the touch-off, not restored
                lift_z = 5.0f;                      // (mm) Height after the touch-off (relative on failure)
constexpr millis_t heat_limit_ms = SEC_TO_MS(900),  // Limit for heating (a 100 °C bed can take minutes)
                   wait_limit_ms = SEC_TO_MS(300),  // Limit for the click, nozzle hot on the bed
                   hands_off_ms = 2000;             // Warning before Z moves under power after the prompt

ManualZHome manual_z;

bool ManualZHome::locked; // = false
int16_t ManualZHome::saved_target = -1;
int16_t ManualZHome::saved_bed; // = 0

//
// Z driver output stage. X/Y/Z/E share one enable pin on this board, so the only way to free
// Z alone is TOFF=0 in the Z driver's CHOPCONF (all bridges off, the motor freewheels).
// Whole register values are written and every write is read back over UART.
//
static bool z_chopconf_is(const uint32_t want) {
  const uint32_t chopconf = stepperZ.CHOPCONF();  // A missing or corrupt reply sets CRCerror
  return !stepperZ.CRCerror && chopconf == want;
}

static bool z_write_chopconf(const uint32_t value) {
  for (uint8_t tries = 3; tries--;) {
    stepperZ.CHOPCONF(value);
    if (z_chopconf_is(value)) return true;
  }
  return false;
}

enum ManualZFail : uint8_t { MZF_NONE, MZF_XY, MZF_TIMEOUT, MZF_CANCEL, MZF_HOST, MZF_DRIVER, MZF_ENGAGE };

// Wait for the knob while keeping heaters, watchdog, UI and serial serviced.
// X/Y stay energised: idle(true) holds off the stepper idle timeout.
static ManualZFail wait_for_knob(const bool heating) {
  const millis_t timeout_ms = millis() + (heating ? heat_limit_ms : wait_limit_ms);
  millis_t next_report_ms = millis();
  ManualZFail result;
  marlin.heatup_start();                // M108 (emergency parser) clears this
  for (;;) {
    marlin.idle(true);
    const millis_t ms = millis();
    if (heating) {
      const int16_t nt = thermalManager.degTargetHotend(0);
      bool hot = thermalManager.wholeDegHotend(0) >= nt - 2;
      #if HAS_HEATED_BED
        const int16_t bt = thermalManager.degTargetBed();
        hot = hot && (bt == 0 || thermalManager.wholeDegBed() >= bt - 2);
      #endif
      if (hot) { result = MZF_NONE; break; }
      if (ELAPSED(ms, next_report_ms)) {
        next_report_ms = ms + 5000UL;
        SERIAL_ECHOLNPGM("MANUAL_Z: heating nozzle ", thermalManager.wholeDegHotend(0), "/", nt
          #if HAS_HEATED_BED
            , " bed ", thermalManager.wholeDegBed(), "/", bt
          #endif
        );
      }
    }
    const ManualZReply reply = DWIN_ManualZ_Reply();
    if (!heating && reply == MZR_CONFIRM) { result = MZF_NONE; break; }
    if (reply == MZR_CANCEL) { result = MZF_CANCEL; break; }
    if (!marlin.is_heating() || card.flag.abort_sd_printing) { result = MZF_HOST; break; }  // M108 / M524
    if (ELAPSED(ms, timeout_ms)) { result = MZF_TIMEOUT; break; }
  }
  marlin.heatup_done();                 // Left set, it would silence M155 temperature reports
  return result;
}

// Z holds again, but a hand may still be on the coupler: warn, wait, then lift
static void lift_after_touch(const float z) {
  DWIN_ManualZ_Show(MZS_LIFT);          // "Hands off" and a beep
  SERIAL_ECHOLNPGM("MANUAL_Z: hands off, Z lifts in 2 s");
  for (const millis_t end_ms = millis() + hands_off_ms; PENDING(millis(), end_ms);) marlin.idle(true);
  motion.blocking_move_z(z, motion.homing_feedrate(Z_AXIS));
  planner.synchronize();
}

// Set the touch-off targets: print temperature if set, else the defaults
static void set_touch_targets() {
  if (touch_temp <= 0) return;
  thermalManager.setTargetHotend(nozzle_temp_for(thermalManager.degTargetHotend(0)), 0);
  TERN_(HAS_HEATED_BED, thermalManager.setTargetBed(bed_temp_for(thermalManager.degTargetBed())));
}

void ManualZHome::preheat() {
  saved_target = thermalManager.degTargetHotend(0);
  saved_bed = TERN0(HAS_HEATED_BED, thermalManager.degTargetBed());
  set_touch_targets();
  if (touch_temp > 0)
    SERIAL_ECHOLNPGM("MANUAL_Z: heating nozzle to ", thermalManager.degTargetHotend(0)
      #if HAS_HEATED_BED
        , ", bed to ", thermalManager.degTargetBed()
      #endif
      , " while homing XY");
}

bool ManualZHome::home(const bool move_xy) {
  ManualZFail fail = MZF_NONE;
  bool released = false;
  uint32_t chopconf_on = 0;

  const bool had_preheat = saved_target >= 0;
  const int16_t restore_target = had_preheat ? saved_target : thermalManager.degTargetHotend(0);
  const int16_t restore_bed = had_preheat ? saved_bed : TERN0(HAS_HEATED_BED, thermalManager.degTargetBed());
  saved_target = -1;

  // The probe / Z-min input may float with the probe unplugged. Nothing below
  // may be stopped by it. G28 restores the endstop state when it finishes.
  endstops.enable(false);

  planner.synchronize();
  motion.sync_plan_position();

  if (motion.homing_needed_error(_BV(X_AXIS) | _BV(Y_AXIS)))
    fail = MZF_XY;

  if (!fail && move_xy) {
    constexpr xy_pos_t touch_xy = { X_CENTER, Y_CENTER };
    motion.blocking_move(touch_xy, motion.homing_feedrate(X_AXIS));
    planner.synchronize();
  }

  if (!fail && touch_temp > 0) {
    // Normally set by preheat() already
    set_touch_targets();
    SERIAL_ECHOLNPGM("MANUAL_Z: heating, please wait (knob click or M108 cancels)");
    DWIN_ManualZ_Show(MZS_HEATING);
    fail = wait_for_knob(true);

    if (!fail && !thermalManager.tooColdToExtrude(0)) {
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
    chopconf_on = stepperZ.CHOPCONF();
    if (stepperZ.CRCerror || !(chopconf_on & 0x0F))
      fail = MZF_DRIVER;                // Nothing was written: Z is still held where it was
    else {
      // From here the Z position is unknown until the knob is clicked
      motion.set_axis_never_homed(Z_AXIS);
      released = true;
      if (z_write_chopconf(chopconf_on & ~0x0FUL)) {
        SERIAL_ECHOLNPGM("MANUAL_Z: Z motor released. Clean the nozzle, lower Z by hand until it touches the bed, then click the knob (M108 cancels)");
        DWIN_ManualZ_Show(MZS_TOUCH);
        KEEPALIVE_STATE(PAUSED_FOR_USER);
        fail = wait_for_knob(false);
      }
      else
        fail = MZF_DRIVER;              // Written but not confirmed: switched back on below
    }
  }

  // Every path that touched the driver ends with the exact previous CHOPCONF, read back.
  // If that can't be confirmed, nothing can be trusted to move Z: halt below, no lift.
  const bool engaged = !released || z_write_chopconf(chopconf_on);
  if (!engaged) fail = MZF_ENGAGE;

  if (!fail) {
    // The nozzle is touching the bed here. M206 Z shifts this like a probe offset would.
    motion.position.z = TERN0(HAS_HOME_OFFSET, motion.home_offset.z);
    TERN_(HAS_WORKSPACE_OFFSET, motion.workspace_offset.z = 0);
    motion.sync_plan_position();
    motion.set_axis_trusted(Z_AXIS);
    motion.set_axis_homed(Z_AXIS);
    locked = false;
    SERIAL_ECHOLNPGM("MANUAL_Z: confirmed, Z set to ", motion.position.z);
    lift_after_touch(lift_z);
    // Back to the targets from before G28. What was off is off again.
    thermalManager.setTargetHotend(restore_target, 0);
    TERN_(HAS_HEATED_BED, thermalManager.setTargetBed(restore_bed));
    DWIN_ManualZ_End(true);
    SERIAL_ECHOLNPGM("MANUAL_Z: done, Z at ", motion.position.z, ", nozzle target ", restore_target, ", bed target ", restore_bed);
    return true;
  }

  //
  // Failed: heater off, Z unknown, no further G-code moves until Z is homed
  //
  thermalManager.setTargetHotend(0, 0);
  TERN_(HAS_HEATED_BED, thermalManager.setTargetBed(0));
  motion.set_axis_never_homed(Z_AXIS);
  locked = true;

  FSTR_P const reason =
      fail == MZF_XY      ? F("X/Y are not homed")
    : fail == MZF_TIMEOUT ? F("timeout")
    : fail == MZF_CANCEL  ? F("cancelled at the knob")
    : fail == MZF_HOST    ? F("cancelled by host (M108/M524)")
    : fail == MZF_DRIVER  ? F("Z driver not responding")
    :                       F("Z DRIVER STILL OFF - POWER CYCLE");
  SERIAL_ECHOLN(F("MANUAL_Z: FAILED, "), reason, F(". Heaters off, Z NOT homed, moves blocked until G28 Z succeeds"));

  // Z was moved by hand and is now unknown. Assume the lowest possible value, so the real
  // nozzle is never below where the firmware thinks it is and any later absolute Z move
  // (some LCD flows make one after G28) can only end up too high. Then raise, so a hot
  // nozzle isn't left sitting on the bed.
  if (released && engaged) {
    motion.position.z = 0;
    motion.sync_plan_position();
    lift_after_touch(lift_z);
  }

  const bool sd_job = card.isPrinting();

  DWIN_ManualZ_Show(MZS_FAILED, reason);

  // The Z driver could not be confirmed on again. Halt with the message on screen.
  // A power cycle re-initialises the driver.
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
