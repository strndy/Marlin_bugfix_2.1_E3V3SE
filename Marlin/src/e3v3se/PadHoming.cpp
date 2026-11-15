/**
 * Pad Z homing: home Z by touching the nozzle on the bed strain-gauge pad.
 *
 * Called from G28 once X/Y are homed (leveling off, FT Motion off, planner empty):
 *   1. planner move in X/Y only, to the pad (AUTOZ_TOOL_X / AUTOZ_TOOL_Y)
 *   2. HX711 check before any descent: answers, not stuck, not saturated, quiet
 *   3. coarse descent: direct Z stepping, one HX711 sample per COARSE_STEP_MM
 *   4. back off BACKOFF_MM, the sensor must relax
 *   5. FINE_PASSES slow passes from there (fresh baseline each), median + spread check
 *   6. Z := height above the contact + M206 Z. G28 then lifts to Z_AFTER_HOMING.
 *
 * Safety rules of this file:
 *   - Every downward step is preceded by a travel check and followed by a validated
 *     HX711 sample. No valid sample => no further step.
 *   - Contact = two samples over the threshold, the second taken without moving.
 *     Readings are compared as |delta|, so the gauge polarity does not matter.
 *   - Z moves by direct stepping only (DIR as the stepper ISR drives it, restored after),
 *     so endstop and probe pins, soft endstops and leveling cannot influence it.
 *   - Z is unhomed/untrusted until success (no babystepping meanwhile). Any failure
 *     lifts the nozzle (at most FAIL_LIFT_MM, never above the descent start) and kill()s.
 */
#include "../inc/MarlinConfig.h"

#if ENABLED(PAD_Z_HOMING)

#include "PadHoming.h"
#include "AutoOffset.h"            // HX711 driver and its pins
#include "../MarlinCore.h"
#include "../module/motion.h"
#include "../module/planner.h"
#include "../module/stepper.h"
#include "../module/temperature.h"
#include "../lcd/marlinui.h"
#include "../sd/cardreader.h"
#if ENABLED(DWIN_CREALITY_LCD)
  #include "../lcd/dwin/creality/dwin.h"
#endif

// Motion constants (mm). Only the force thresholds are user settings.
constexpr float Z_SPEED_MM_S      = 4,     // direct stepping speed, no acceleration
                COARSE_STEP_MM    = 0.05f, // coarse descent step, one sample per step
                FINE_STEP_MM      = 0.02f, // fine pass step
                BACKOFF_MM        = 1.0f,  // lift after the coarse contact; fine passes start here
                FINE_WINDOW_MM    = 0.5f,  // fine contact must be within this of the coarse contact
                MAX_SPREAD_MM     = 0.10f, // fine passes must agree within this
                KNOWN_Z_MARGIN_MM = 5,     // Z known: no contact this far below the expected point = halt
                FAIL_LIFT_MM      = 2;     // max lift before kill()
constexpr uint8_t FINE_PASSES     = 3;     // odd: the median is used
constexpr millis_t HX711_TIMEOUT_MS = 150; // 80 Hz => 12.5 ms per sample
constexpr uint8_t STUCK_COUNT     = 8;     // identical raw readings in a row = dead data line
#define PAD_RING 32                        // fine pass look-back (steps)

static_assert(PAD_NOISE_MAX > 0 && 2 * (PAD_NOISE_MAX) <= PAD_FINE_TRIGGER, "PAD_FINE_TRIGGER must be at least 2 x PAD_NOISE_MAX.");
static_assert(PAD_FINE_TRIGGER <= PAD_COARSE_TRIGGER && PAD_COARSE_TRIGGER <= 1000000, "Need PAD_FINE_TRIGGER <= PAD_COARSE_TRIGGER <= 1000000.");

static HX711 pad_hx;

// ---- Touch at print temperature ----
// A target already set (e.g. by the print's start G-code) is used; below these it counts as unset.
constexpr int16_t NOZZLE_SET_MIN = 50, BED_SET_MIN = 30;
constexpr millis_t HEAT_LIMIT_MS = SEC_TO_MS(900);   // a hot bed can take minutes
constexpr float TOUCH_RETRACT_MM = 1;                // pulled back against ooze, not restored
static int16_t saved_nozzle = -1, saved_bed;         // targets before G28 (-1 = pad_preheat not run)

static void pad_set_touch_targets() {
  if (PAD_HOME_NOZZLE_TEMP <= 0) return;
  const int16_t n = thermalManager.degTargetHotend(0);
  thermalManager.setTargetHotend(n >= NOZZLE_SET_MIN ? n : int16_t(PAD_HOME_NOZZLE_TEMP), 0);
  #if HAS_HEATED_BED
    const int16_t b = thermalManager.degTargetBed();
    thermalManager.setTargetBed(b >= BED_SET_MIN ? b : int16_t(PAD_HOME_BED_TEMP));
  #endif
}

void pad_preheat() {
  saved_nozzle = thermalManager.degTargetHotend(0);
  saved_bed = TERN0(HAS_HEATED_BED, thermalManager.degTargetBed());
  pad_set_touch_targets();
}
static int32_t pad_last_raw;
static uint8_t pad_same_count;

static inline float pad_spm() { return planner.settings.axis_steps_per_mm[Z_AXIS]; }
static inline long pad_steps(const float mm) { return _MAX(1L, lroundf(mm * pad_spm())); }
static inline int32_t pad_abs(const int32_t v) { return v < 0 ? -v : v; }

/**
 * Direct-step Z. up = Marlin's +Z: Z_DIR_WRITE() applies INVERT_Z_DIR exactly as the
 * stepper ISR's Z_APPLY_DIR does. The DIR pin is restored because the ISR only
 * rewrites it when its cached direction changes.
 */
static void pad_step_z(long steps, const bool up) {
  if (steps <= 0) return;
  const uint32_t half_us = _MAX(20UL, uint32_t(500000.0f / (pad_spm() * Z_SPEED_MM_S)));
  const bool old_dir = Z_DIR_READ();
  Z_DIR_WRITE(up);
  DELAY_US(100);
  while (steps--) {
    Z_STEP_WRITE(STEP_STATE_Z);
    DELAY_US(half_us);
    Z_STEP_WRITE(!STEP_STATE_Z);
    DELAY_US(half_us);
    if ((steps & 0x3F) == 0) hal.watchdog_refresh();
  }
  Z_DIR_WRITE(old_dir);
  DELAY_US(100);
}

static void pad_reader_reset() {
  pad_hx.init(HX711_SCK_PIN, HX711_SDO_PIN);
  pad_same_count = 0;
  pad_last_raw = INT32_MAX;
}

// One HX711 sample. Never moves anything, does not call idle(). nullptr = OK, else the error.
static FSTR_P pad_read(int32_t &val) {
  const millis_t start = millis();
  while (READ(HX711_SDO_PIN)) {                        // DOUT low = data ready
    if (millis() - start > HX711_TIMEOUT_MS) return F("HX711 no data");
    hal.watchdog_refresh();
  }
  const int32_t v = pad_hx.getVal(false);
  if (v >= 0x7FFFFF || v <= -0x800000) return F("HX711 saturated");
  if (v != pad_last_raw) { pad_same_count = 0; pad_last_raw = v; }
  else if (++pad_same_count >= STUCK_COUNT) return F("HX711 stuck");
  val = v;
  return nullptr;
}

// Lift (when the nozzle may be loaded), report on serial + LCD, halt. Never returns.
static void pad_fail(FSTR_P const why, const long lift_steps=0) {
  planner.quick_stop();                                // drop any move queued meanwhile (e.g. from the LCD)
  pad_step_z(_MIN(lift_steps, pad_steps(FAIL_LIFT_MM)), true);
  thermalManager.disable_all_heaters();
  SERIAL_ERROR_START();
  SERIAL_ECHOLN(F("PAD_HOME: FAILED: "), why);
  #if ENABLED(DWIN_CREALITY_LCD)
    char msg[61];
    strcpy(msg, "Pad Z homing FAILED: ");
    strncat(msg, FTOP(why), sizeof(msg) - strlen(msg) - 1);
    DWIN_Show_M117(msg);
    DWIN_UpdateLCD();
  #endif
  // Heaters off, halt. Steppers stay energized (kill's default) so the gantry holds.
  marlin.kill(why, F("Pad Z homing"));
  for (;;) hal.watchdog_refresh();                     // kill() does not return; be explicit
}

// A validated sample, or lift by lift_steps and halt
static int32_t pad_sample(const long lift_steps) {
  int32_t v = 0;
  const FSTR_P err = pad_read(v);
  if (err) pad_fail(err, lift_steps);
  return v;
}

// Called once per descent step: nothing else may have moved Z or stopped the machine
static void pad_check(const long lift_steps) {
  if (!marlin.isRunning() || planner.has_blocks_queued()) pad_fail(F("interrupted"), lift_steps);
}

// Heat to the touch targets and wait for both (nozzle not yet over the pad), then retract.
// M108, an SD abort or the time limit halts like any other failure (kill() turns heaters off).
static void pad_heat_and_wait() {
  pad_set_touch_targets();
  if (PAD_HOME_NOZZLE_TEMP <= 0) return;
  ui.set_status(F("Pad Z homing: heating"));
  const millis_t limit = millis() + HEAT_LIMIT_MS;
  millis_t next_report = 0;
  marlin.heatup_start();                               // M108 (emergency parser) clears this
  for (;;) {
    marlin.idle(true);                                 // true: keep the steppers energized
    const millis_t ms = millis();
    const int16_t nt = thermalManager.degTargetHotend(0);
    bool hot = thermalManager.wholeDegHotend(0) >= nt - 2;
    #if HAS_HEATED_BED
      const int16_t bt = thermalManager.degTargetBed();
      hot = hot && (bt == 0 || thermalManager.wholeDegBed() >= bt - 2);
    #endif
    if (hot) break;
    const bool cancelled = !marlin.is_heating() || card.flag.abort_sd_printing;
    if (cancelled || ELAPSED(ms, limit)) {
      marlin.heatup_done();
      pad_fail(cancelled ? F("cancelled while heating") : F("heating timeout"));
    }
    if (ELAPSED(ms, next_report)) {
      next_report = ms + 5000UL;
      SERIAL_ECHOLNPGM("PAD_HOME: heating nozzle ", thermalManager.wholeDegHotend(0), "/", nt
        #if HAS_HEATED_BED
          , " bed ", thermalManager.wholeDegBed(), "/", bt
        #endif
      );
    }
  }
  marlin.heatup_done();                                // left set, it would silence M155 reports

  if (!thermalManager.tooColdToExtrude(0)) {
    // Pull the melt back a little; logical E unchanged, the print's prime line makes it up
    const float e_before = motion.position.e;
    motion.position.e -= TOUCH_RETRACT_MM;
    motion.goto_current_position(MMM_TO_MMS(1200));
    planner.synchronize();
    motion.position.e = e_before;
    motion.sync_plan_position_e();
  }
}

struct PadStats { int32_t vmin, vmax, avg; };

// Baseline over n samples (after dropping the pending one). Halts on a bad or noisy sensor.
static PadStats pad_baseline(const uint8_t n, FSTR_P const noisy_msg) {
  int32_t vmin = INT32_MAX, vmax = INT32_MIN;
  int64_t sum = 0;
  (void)pad_sample(0);
  for (uint8_t i = 0; i < n; ++i) {
    const int32_t v = pad_sample(0);
    NOMORE(vmin, v); NOLESS(vmax, v); sum += v;
  }
  if (vmax - vmin > PAD_NOISE_MAX) {
    SERIAL_ECHOLNPGM("PAD_HOME: spread=", vmax - vmin, " limit=", PAD_NOISE_MAX);
    pad_fail(noisy_msg);
  }
  return { vmin, vmax, int32_t(sum / n) };
}

/**
 * One fine pass from the current position. Returns the travel (steps) from the start
 * to the first step of the force rise; the nozzle is back at the start. Halts on failure.
 */
static long pad_fine_pass(const uint8_t pass, const long step_n, const long max_steps) {
  const PadStats b = pad_baseline(16, F("sensor noisy before fine pass"));
  const int32_t knee = _MIN(int32_t(PAD_FINE_TRIGGER), _MAX(int32_t(PAD_FINE_TRIGGER) / 3, 2 * (b.vmax - b.vmin)));

  int32_t ring[PAD_RING];
  long down = 0;
  int32_t i = 0, d;
  uint8_t glitches = 0;
  for (;;) {
    pad_check(down);
    if (down + step_n > max_steps) pad_fail(F("fine pass: no contact in range"), down);
    pad_step_z(step_n, false);
    down += step_n;
    ++i;
    (void)pad_sample(down);                            // drop the sample converted during the step
    d = pad_abs(pad_sample(down) - b.avg);
    if (d > PAD_FINE_TRIGGER) {
      // Re-read without moving: a real load is still there, a glitch is not
      d = _MIN(d, pad_abs(pad_sample(down) - b.avg));
      if (d > PAD_FINE_TRIGGER) { ring[i % PAD_RING] = d; break; }
      if (++glitches > 3) pad_fail(F("sensor glitches in fine pass"), down);
    }
    ring[i % PAD_RING] = d;
    marlin.idle_no_sleep();
  }

  // First step of the uninterrupted rise above the knee (ring holds steps i-31..i)
  int32_t k = i;
  while (k > 1 && k - 1 > i - PAD_RING && ring[(k - 1) % PAD_RING] > knee) --k;

  pad_step_z(down, true);                              // back to the start position
  safe_delay(150);

  SERIAL_ECHOLNPGM("PAD_HOME: fine #", pass, " base=", b.avg, " noise=", b.vmax - b.vmin,
                   " knee=", knee, " trig_delta=", d,
                   " trig_mm=", p_float_t(float(down) / pad_spm(), 3),
                   " contact_mm=", p_float_t((float(k) - 0.5f) * float(step_n) / pad_spm(), 3));

  return k * step_n - step_n / 2;
}

void pad_home_z() {
  SERIAL_ECHOLNPGM("PAD_HOME: start");
  ui.set_status(F("Pad Z homing"));

  if (motion.axes_should_home(_BV(X_AXIS) | _BV(Y_AXIS)) || !motion.axis_is_trusted(X_AXIS) || !motion.axis_is_trusted(Y_AXIS))
    pad_fail(F("X/Y not homed"));

  const float spm = pad_spm(), z_shift = TERN0(HAS_HOME_OFFSET, motion.home_offset.z);
  if (!(spm >= 50 && spm <= 3200)) pad_fail(F("bad Z steps/mm"));
  if (!(z_shift >= -1 && z_shift <= 1)) pad_fail(F("M206 Z over 1mm"));   // a typo here would drive the nozzle into the bed

  // Was Z known? Then the contact must come at about the expected distance.
  // (G28 has disabled leveling, so position.z is the plain machine height.)
  const bool z_known = motion.axis_is_trusted(Z_AXIS) && !motion.axis_should_home(Z_AXIS);
  const float z_entry = motion.position.z;

  // Targets to restore afterwards: those from before G28 (off if they were off)
  const bool had_preheat = saved_nozzle >= 0;
  const int16_t restore_nozzle = had_preheat ? saved_nozzle : thermalManager.degTargetHotend(0),
                restore_bed = had_preheat ? saved_bed : TERN0(HAS_HEATED_BED, thermalManager.degTargetBed());
  saved_nozzle = -1;

  // Z is unknown until this succeeds. 'Unhomed' also refuses babysteps meanwhile.
  motion.set_axis_untrusted(Z_AXIS);
  motion.set_axis_unhomed(Z_AXIS);

  // Touch at print temperature. Before moving over the pad, so ooze doesn't drip on it.
  motion.sync_plan_position();
  pad_heat_and_wait();

  // X/Y only, at the current height (G28 has already raised Z)
  motion.sync_plan_position();
  motion.blocking_move_xy(float(AUTOZ_TOOL_X), float(AUTOZ_TOOL_Y), 80);
  planner.synchronize();
  pad_check(0);
  stepper.enable_axis(Z_AXIS);
  safe_delay(300);

  // ---- Sensor health, before any descent ----
  pad_reader_reset();
  const millis_t t0 = millis();
  const PadStats base = pad_baseline(32, F("sensor noisy or loaded"));
  SERIAL_ECHOLNPGM("PAD_HOME: sensor ok, baseline=", base.avg, " noise=", base.vmax - base.vmin,
                   " sample_ms=", (millis() - t0) / 33);

  // ---- Coarse descent ----
  const long coarse_n = pad_steps(COARSE_STEP_MM);
  float max_mm = Z_MAX_POS + 10;
  if (z_known) NOMORE(max_mm, _MAX(z_entry, 0.0f) + KNOWN_Z_MARGIN_MM);
  const long max_n = pad_steps(max_mm);
  SERIAL_ECHOLNPGM("PAD_HOME: descending, travel limit ", p_float_t(max_mm, 1), z_known ? " mm (Z was known)" : " mm (Z unknown)");
  ui.set_status(F("Pad Z homing: descending"));
  long travelled = 0;
  float track = base.avg;                              // follows slow (thermal) drift, at most one trigger level
  int32_t d;
  uint8_t glitches = 0;
  for (;;) {
    pad_check(travelled);
    if (travelled + coarse_n > max_n) pad_fail(F("no pad contact (max travel)"), travelled);
    pad_step_z(coarse_n, false);
    travelled += coarse_n;

    int32_t v = pad_sample(travelled);
    d = pad_abs(v - int32_t(track));
    if (d > PAD_COARSE_TRIGGER) {
      // Re-read without moving: a real load is still there, a glitch is not
      d = _MIN(d, pad_abs(pad_sample(travelled) - int32_t(track)));
      if (d > PAD_COARSE_TRIGGER) break;
      if (++glitches > 10) pad_fail(F("sensor glitches in descent"), travelled);
    }
    else if (d < (PAD_COARSE_TRIGGER) / 4) {
      track += float(v - int32_t(track)) * (1.0f / 32.0f);
      track = constrain(track, float(base.avg - (PAD_COARSE_TRIGGER)), float(base.avg + (PAD_COARSE_TRIGGER)));
    }
    marlin.idle_no_sleep();                            // heaters, watchdog, LCD
  }

  SERIAL_ECHOLNPGM("PAD_HOME: coarse contact after ", p_float_t(float(travelled) / spm, 2),
                   " mm, delta=", d, " drift=", int32_t(track) - base.avg, " glitches=", glitches);

  // ---- Back off; the sensor must relax ----
  const long backoff_n = pad_steps(BACKOFF_MM);
  pad_step_z(backoff_n, true);
  safe_delay(300);
  {
    const PadStats rs = pad_baseline(16, F("sensor noisy after contact"));
    const int32_t residual = rs.avg - int32_t(track);
    SERIAL_ECHOLNPGM("PAD_HOME: backed off ", p_float_t(BACKOFF_MM, 2), " mm, residual=", residual);
    if (pad_abs(residual) > (PAD_COARSE_TRIGGER) / 2) pad_fail(F("pad still loaded after backoff"), travelled - backoff_n);
  }

  // ---- Fine passes, from the backed-off position ----
  ui.set_status(F("Pad Z homing: measuring"));
  const long fine_n = pad_steps(FINE_STEP_MM), window_n = pad_steps(FINE_WINDOW_MM);
  long c[FINE_PASSES];
  for (uint8_t i = 0; i < FINE_PASSES; ++i) {
    c[i] = pad_fine_pass(i + 1, fine_n, backoff_n + window_n);
    if (c[i] < backoff_n - window_n) pad_fail(F("fine pass: early trigger"));   // not the coarse contact
    for (uint8_t j = i; j > 0 && c[j - 1] > c[j]; --j) { const long t = c[j]; c[j] = c[j - 1]; c[j - 1] = t; }
  }
  const float spread_mm = float(c[FINE_PASSES - 1] - c[0]) / spm,
              above_mm  = float(c[FINE_PASSES / 2]) / spm;   // median height above the contact
  SERIAL_ECHOLNPGM("PAD_HOME: median_mm=", p_float_t(above_mm, 3), " spread_mm=", p_float_t(spread_mm, 3));
  if (spread_mm > MAX_SPREAD_MM) pad_fail(F("fine passes disagree"));

  if (z_known)
    SERIAL_ECHOLNPGM("PAD_HOME: contact moved by ",
      p_float_t(z_entry - float(travelled - backoff_n) / spm - above_mm - z_shift, 3), " mm since the previous homing");

  // ---- The nozzle is above_mm over the contact: define Z (as set_axis_is_at_home does) ----
  motion.position.z = above_mm + z_shift;
  motion.sync_plan_position();
  motion.set_axis_trusted(Z_AXIS);
  motion.set_axis_homed(Z_AXIS);
  TERN_(BABYSTEP_DISPLAY_TOTAL, babystep.reset_total(Z_AXIS));
  TERN_(HAS_WORKSPACE_OFFSET, motion.workspace_offset.z = 0);

  // Back to the targets from before G28
  thermalManager.setTargetHotend(restore_nozzle, 0);
  TERN_(HAS_HEATED_BED, thermalManager.setTargetBed(restore_bed));

  SERIAL_ECHOLNPGM("PAD_HOME: done, Z=", p_float_t(motion.position.z, 3), " (M206 Z ", p_float_t(z_shift, 3),
                   "), nozzle target ", restore_nozzle, " bed target ", restore_bed);
  ui.reset_status();
}

void pad_sensor_report(const uint8_t seconds) {
  pad_reader_reset();
  int32_t v = 0, base = 0;
  FSTR_P err = pad_read(v);
  if (!err) err = pad_read(base);
  const millis_t end = millis() + 1000UL * seconds;
  millis_t next = 0;
  SERIAL_ECHOLNPGM("PAD_TEST: no motion. Press the bed at the front-left corner. baseline=", base);
  while (!err && PENDING(millis(), end)) {
    err = pad_read(v);
    if (!err && ELAPSED(millis(), next)) {
      next = millis() + 200;
      SERIAL_ECHOLNPGM("PAD_TEST: raw=", v, " delta=", v - base);
    }
    marlin.idle();
  }
  if (err) SERIAL_ECHOLN(F("PAD_TEST: FAILED: "), err);
  else SERIAL_ECHOLNPGM("PAD_TEST: end (trigger fine/coarse=", PAD_FINE_TRIGGER, "/", PAD_COARSE_TRIGGER, " noise max=", PAD_NOISE_MAX, ")");
}

#endif // PAD_Z_HOMING
