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
 *   - Z is unhomed/untrusted until success (no babystepping meanwhile).
 *
 * Failures are soft: every helper that can fail is [[nodiscard]] bool, records the reason
 * and the steps travelled down with pad_fail() and returns false; each caller returns at
 * once, so nothing moves between the failure and pad_abort() in pad_home_z(). That lifts
 * the nozzle (at most FAIL_LIFT_MM, never above the descent start), turns the heaters off,
 * leaves Z unhomed, reports, stops the job, and G28 ends without moving Z.
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
#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../feature/host_actions.h"
#endif
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
                KNOWN_Z_MARGIN_MM = 5,     // Z known: no contact this far below the expected point = fail
                FAIL_LIFT_MM      = 2;     // max lift after a failure
constexpr uint8_t FINE_PASSES     = 3;     // odd: the median is used
constexpr millis_t HX711_TIMEOUT_MS = 150; // 80 Hz => 12.5 ms per sample
constexpr uint8_t STUCK_COUNT     = 8;     // identical raw readings in a row = dead data line
// Measured on a C13 at 50 °C: ~100 counts/g, heated-bed drift up to ~300 counts/s while settling.
// Settling: one robust point (trimmed mean of 16 samples) per SETTLE_INTERVAL_MS, the drift is the
// least-squares slope over the last SETTLE_POINTS points (~10 s); it must stay under SETTLE_MAX_COUNTS
// per interval for SETTLE_CALM evaluations in a row. A difference of two single points was fooled
// by the point-to-point scatter while the bed still drifted ~300 counts per 2 s.
constexpr millis_t SETTLE_INTERVAL_MS = 2000, SETTLE_LIMIT_MS = SEC_TO_MS(180);
constexpr int32_t SETTLE_MAX_COUNTS = 200;  // per interval, i.e. ~1 g/s
constexpr uint8_t SETTLE_POINTS = 6, SETTLE_CALM = 3;
constexpr int32_t DRIFT_TRACK_MAX   = 30000; // descent may follow this much slow drift (~300 g)
#define PAD_RING 32                        // fine pass look-back (steps)
// Noise = range of the baseline window after removing its linear trend (slow drift) and
// dropping the NOISE_TRIM highest and lowest samples (isolated spikes: with the heaters
// regulating, single samples of +-900 were seen in an otherwise +-300 signal).
constexpr uint8_t NOISE_TRIM = 2;

static_assert(PAD_NOISE_MAX > 0 && 2 * (PAD_NOISE_MAX) <= PAD_FINE_TRIGGER, "PAD_FINE_TRIGGER must be at least 2 x PAD_NOISE_MAX.");
static_assert(PAD_FINE_TRIGGER <= PAD_COARSE_TRIGGER && PAD_COARSE_TRIGGER <= 1000000, "Need PAD_FINE_TRIGGER <= PAD_COARSE_TRIGGER <= 1000000.");

static HX711 pad_hx;

// ---- Failure state ----
static FSTR_P pad_err;           // reason of the failure (nullptr = none)
static long pad_err_down;        // steps travelled down from the descent start when it failed
static bool pad_err_stopped;     // the print was stopped elsewhere (LCD, M524, card out): no popup

// Record the failure. Always returns false so a helper can 'return pad_fail(...)'.
static bool pad_fail(FSTR_P const why, const long down=0) {
  pad_err = why;
  pad_err_down = down;
  return false;
}

// ---- Touch at print temperature ----
// A target already set (e.g. by the print's start G-code) is used; below these it counts as unset.
constexpr int16_t NOZZLE_SET_MIN = 50, BED_SET_MIN = 30;
constexpr millis_t HEAT_LIMIT_MS = SEC_TO_MS(900);   // a hot bed can take minutes
constexpr float TOUCH_RETRACT_MM = 1;                // pulled back against ooze, not restored
// The bed counts as reached at target - 1 °C, then BED_SOAK_MS more: the plate and the gauge keep
// moving after the controller reaches temperature (fine passes crept 0.06 mm per pass without it).
constexpr millis_t BED_SOAK_MS = SEC_TO_MS(30);
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

// A validated sample. false = failed (down: steps travelled down, lifted by pad_abort)
[[nodiscard]] static bool pad_sample(const long down, int32_t &v) {
  const FSTR_P err = pad_read(v);
  return err ? pad_fail(err, down) : true;
}

// Called once per descent step: nothing else may have moved Z or stopped the machine
[[nodiscard]] static bool pad_check(const long down) {
  if (TERN0(HAS_MEDIA, card.flag.abort_sd_printing)) {
    pad_err_stopped = true;
    return pad_fail(F("print stopped"), down);
  }
  if (!marlin.isRunning() || planner.has_blocks_queued()) return pad_fail(F("interrupted"), down);
  return true;
}

// Heat to the touch targets and wait for both (nozzle not yet over the pad), then retract.
// M108, an SD abort or the time limit fails like any other failure (heaters off).
[[nodiscard]] static bool pad_heat_and_wait() {
  pad_set_touch_targets();
  if (PAD_HOME_NOZZLE_TEMP <= 0) return true;
  ui.set_status(F("Pad Z homing: heating"));
  const millis_t limit = millis() + HEAT_LIMIT_MS;
  millis_t next_report = 0;
  marlin.heatup_start();                               // M108 (emergency parser) clears this
  TERN_(HAS_HEATED_BED, millis_t bed_since = 0);       // when the bed first reached its target (0 = not yet)
  for (;;) {
    marlin.idle(true);                                 // true: keep the steppers energized
    const millis_t ms = millis();
    const int16_t nt = thermalManager.degTargetHotend(0);
    bool hot = thermalManager.wholeDegHotend(0) >= nt - 2;
    #if HAS_HEATED_BED
      const int16_t bt = thermalManager.degTargetBed();
      if (bt > 0 && !bed_since && thermalManager.wholeDegBed() >= bt - 1) {
        bed_since = ms ?: 1;
        SERIAL_ECHOLNPGM("PAD_HOME: bed at target, soaking ", BED_SOAK_MS / 1000, " s");
      }
      hot = hot && (bt == 0 || (bed_since && ELAPSED(ms, bed_since + BED_SOAK_MS)));
    #endif
    if (hot) break;
    const bool cancelled = !marlin.is_heating() || TERN0(HAS_MEDIA, card.flag.abort_sd_printing);
    if (cancelled || ELAPSED(ms, limit)) {
      marlin.heatup_done();
      pad_err_stopped = TERN0(HAS_MEDIA, card.flag.abort_sd_printing);
      return pad_fail(cancelled ? F("cancelled while heating") : F("heating timeout"));
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
  return true;
}

// noise: detrended, trimmed min..max (see NOISE_TRIM); raw: plain min..max; avg: plain mean
struct PadStats { int32_t noise, raw, avg; };

// Sort a few samples in place (insertion sort; n <= 32)
static void pad_sort(int32_t * const v, const uint8_t n) {
  for (uint8_t i = 1; i < n; ++i)
    for (uint8_t j = i; j > 0 && v[j - 1] > v[j]; --j) { const int32_t t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
}

// Mean of 16 samples without the NOISE_TRIM highest and lowest (no noise check)
[[nodiscard]] static bool pad_trimmed_mean(int32_t &mean) {
  constexpr uint8_t n = 16;
  int32_t v[n];
  for (uint8_t i = 0; i < n; ++i) if (!pad_sample(0, v[i])) return false;
  pad_sort(v, n);
  int64_t sum = 0;
  for (uint8_t i = NOISE_TRIM; i < n - NOISE_TRIM; ++i) sum += v[i];
  mean = int32_t(sum / (n - 2 * NOISE_TRIM));
  return true;
}

// Wait until the drift (slope over the last SETTLE_POINTS points) is under SETTLE_MAX_COUNTS per
// SETTLE_INTERVAL_MS, SETTLE_CALM times in a row. Gives up after SETTLE_LIMIT_MS and continues:
// the descent tracks slow drift anyway.
[[nodiscard]] static bool pad_wait_settled() {
  ui.set_status(F("Pad Z homing: sensor settling"));
  const millis_t start = millis(), limit = start + SETTLE_LIMIT_MS;
  float pt[SETTLE_POINTS], pv[SETTLE_POINTS];          // ring: time (s since start), point (counts)
  uint8_t n = 0, calm = 0;
  for (;;) {
    pad_reader_reset();                                // idle() took longer than a sample period
    int32_t now;
    if (!pad_trimmed_mean(now)) return false;
    pt[n % SETTLE_POINTS] = float(millis() - start) * 0.001f;
    pv[n % SETTLE_POINTS] = float(now);
    ++n;
    if (n >= SETTLE_POINTS) {
      // Least-squares slope over the ring, in counts per SETTLE_INTERVAL_MS
      float mt = 0, mv = 0;
      for (uint8_t i = 0; i < SETTLE_POINTS; ++i) { mt += pt[i]; mv += pv[i]; }
      mt /= SETTLE_POINTS; mv /= SETTLE_POINTS;
      float sxy = 0, sxx = 0;
      for (uint8_t i = 0; i < SETTLE_POINTS; ++i) { sxy += (pt[i] - mt) * (pv[i] - mv); sxx += sq(pt[i] - mt); }
      const int32_t drift = int32_t(lroundf(sxy / sxx * (SETTLE_INTERVAL_MS / 1000.0f)));
      calm = pad_abs(drift) < SETTLE_MAX_COUNTS ? calm + 1 : 0;
      SERIAL_ECHOLNPGM("PAD_HOME: settling, drift=", drift, " per ", SETTLE_INTERVAL_MS, " ms (", calm, "/", SETTLE_CALM, ")");
      if (calm >= SETTLE_CALM) return true;
    }
    if (ELAPSED(millis(), limit)) { SERIAL_ECHOLNPGM("PAD_HOME: still drifting, continuing (drift is tracked)"); return true; }
    for (const millis_t next = millis() + SETTLE_INTERVAL_MS; PENDING(millis(), next);) {
      marlin.idle_no_sleep();
      if (!pad_check(0)) return false;
    }
  }
}

// Baseline over N samples (after dropping the pending one). Fails on a bad or noisy sensor.
template<uint8_t N>
[[nodiscard]] static bool pad_baseline(FSTR_P const noisy_msg, PadStats &st) {
  static_assert(N >= 4 * NOISE_TRIM + 4 && N <= 32, "pad_baseline: bad window size");
  int32_t v[N], vmin = INT32_MAX, vmax = INT32_MIN;
  int64_t sum = 0;
  if (!pad_sample(0, v[0])) return false;              // converted before the window started
  for (uint8_t i = 0; i < N; ++i) {
    if (!pad_sample(0, v[i])) return false;
    NOMORE(vmin, v[i]); NOLESS(vmax, v[i]); sum += v[i];
  }
  const int32_t avg = int32_t(sum / N);

  // Remove the least-squares linear trend (slow thermal drift), then sort the residuals
  constexpr float c = (N - 1) * 0.5f, sxx = float(N) * (float(N) * N - 1) / 12.0f;
  float sxy = 0;
  for (uint8_t i = 0; i < N; ++i) sxy += (float(i) - c) * float(v[i] - avg);
  const float slope = sxy / sxx;
  for (uint8_t i = 0; i < N; ++i) v[i] -= int32_t(lroundf(slope * (float(i) - c)));
  pad_sort(v, N);

  // A steady ramp is drift only while it is slow: thermal drift (<= ~300 counts/s) moves a few
  // hundred counts per window at most, a hand pressing or releasing the bed moves thousands.
  const int32_t drift = int32_t(lroundf(slope * (N - 1)));
  st = { v[N - 1 - NOISE_TRIM] - v[NOISE_TRIM], vmax - vmin, avg };
  if (st.noise > PAD_NOISE_MAX || pad_abs(drift) > PAD_NOISE_MAX) {
    SERIAL_ECHOLNPGM("PAD_HOME: noise=", st.noise, " drift=", drift, " limit=", PAD_NOISE_MAX, " (raw spread=", st.raw, ")");
    return pad_fail(noisy_msg);
  }
  return true;
}

/**
 * One fine pass from the current position. Gets the travel (steps) from the start to the
 * first step of the force rise; the nozzle is back at the start. false = failed.
 */
[[nodiscard]] static bool pad_fine_pass(const uint8_t pass, const long step_n, const long max_steps, long &contact) {
  PadStats b;
  if (!pad_baseline<16>(F("sensor noisy before fine pass"), b)) return false;
  const int32_t knee = _MIN(int32_t(PAD_FINE_TRIGGER), _MAX(int32_t(PAD_FINE_TRIGGER) / 3, 2 * b.noise));

  int32_t ring[PAD_RING];
  long down = 0;
  int32_t i = 0, d, v;
  uint8_t glitches = 0;
  for (;;) {
    if (!pad_check(down)) return false;
    if (down + step_n > max_steps) return pad_fail(F("fine pass: no contact in range"), down);
    pad_step_z(step_n, false);
    down += step_n;
    ++i;
    if (!pad_sample(down, v)) return false;            // drop the sample converted during the step
    if (!pad_sample(down, v)) return false;
    d = pad_abs(v - b.avg);
    if (d > PAD_FINE_TRIGGER) {
      // Re-read without moving: a real load is still there, a glitch is not
      if (!pad_sample(down, v)) return false;
      d = _MIN(d, pad_abs(v - b.avg));
      if (d > PAD_FINE_TRIGGER) { ring[i % PAD_RING] = d; break; }
      if (++glitches > 3) return pad_fail(F("sensor glitches in fine pass"), down);
    }
    ring[i % PAD_RING] = d;
    marlin.idle_no_sleep();
  }

  // First step of the uninterrupted rise above the knee (ring holds steps i-31..i)
  int32_t k = i;
  while (k > 1 && k - 1 > i - PAD_RING && ring[(k - 1) % PAD_RING] > knee) --k;

  pad_step_z(down, true);                              // back to the start position
  safe_delay(150);

  SERIAL_ECHOLNPGM("PAD_HOME: fine #", pass, " base=", b.avg, " noise=", b.noise, " raw=", b.raw,
                   " knee=", knee, " trig_delta=", d,
                   " trig_mm=", p_float_t(float(down) / pad_spm(), 3),
                   " contact_mm=", p_float_t((float(k) - 0.5f) * float(step_n) / pad_spm(), 3));

  contact = k * step_n - step_n / 2;
  return true;
}

// The homing itself. false = failed, pad_err/pad_err_down say why and where.
[[nodiscard]] static bool pad_home_run(const int16_t restore_nozzle, const int16_t restore_bed) {
  if (motion.axes_should_home(_BV(X_AXIS) | _BV(Y_AXIS)) || !motion.axis_is_trusted(X_AXIS) || !motion.axis_is_trusted(Y_AXIS))
    return pad_fail(F("X/Y not homed"));

  const float spm = pad_spm(), z_shift = TERN0(HAS_HOME_OFFSET, motion.home_offset.z);
  if (!(spm >= 50 && spm <= 3200)) return pad_fail(F("bad Z steps/mm"));
  if (!(z_shift >= -1 && z_shift <= 1)) return pad_fail(F("M206 Z over 1mm"));   // a typo here would drive the nozzle into the bed

  // Was Z known? Then the contact must come at about the expected distance.
  // (G28 has disabled leveling, so position.z is the plain machine height.)
  const bool z_known = motion.axis_is_trusted(Z_AXIS) && !motion.axis_should_home(Z_AXIS);
  const float z_entry = motion.position.z;

  // Z is unknown until this succeeds. 'Unhomed' also refuses babysteps meanwhile.
  motion.set_axis_untrusted(Z_AXIS);
  motion.set_axis_unhomed(Z_AXIS);

  // Touch at print temperature. Before moving over the pad, so ooze doesn't drip on it.
  motion.sync_plan_position();
  if (!pad_heat_and_wait()) return false;

  // X/Y only, at the current height (G28 has already raised Z)
  motion.sync_plan_position();
  motion.blocking_move_xy(float(AUTOZ_TOOL_X), float(AUTOZ_TOOL_Y), 80);
  planner.synchronize();
  if (!pad_check(0)) return false;
  stepper.enable_axis(Z_AXIS);
  safe_delay(300);

  // ---- Wait for the gauge to stop drifting (a freshly heated bed drifts up to ~300 counts/s) ----
  pad_reader_reset();
  if (!pad_wait_settled()) return false;

  // ---- Sensor health, before any descent ----
  const millis_t t0 = millis();
  PadStats base;
  if (!pad_baseline<32>(F("sensor noisy or loaded"), base)) return false;
  SERIAL_ECHOLNPGM("PAD_HOME: sensor ok, baseline=", base.avg, " noise=", base.noise, " raw=", base.raw,
                   " sample_ms=", (millis() - t0) / 33);

  // ---- Coarse descent ----
  const long coarse_n = pad_steps(COARSE_STEP_MM);
  float max_mm = Z_MAX_POS + 10;
  if (z_known) NOMORE(max_mm, _MAX(z_entry, 0.0f) + KNOWN_Z_MARGIN_MM);
  const long max_n = pad_steps(max_mm);
  SERIAL_ECHOLNPGM("PAD_HOME: descending, travel limit ", p_float_t(max_mm, 1), z_known ? " mm (Z was known)" : " mm (Z unknown)");
  ui.set_status(F("Pad Z homing: descending"));
  long travelled = 0;
  float track = base.avg;                              // follows slow (thermal) drift, bounded
  int32_t d, v;
  uint8_t glitches = 0;
  for (;;) {
    if (!pad_check(travelled)) return false;
    if (travelled + coarse_n > max_n) return pad_fail(F("no pad contact (max travel)"), travelled);
    pad_step_z(coarse_n, false);
    travelled += coarse_n;

    if (!pad_sample(travelled, v)) return false;
    d = pad_abs(v - int32_t(track));
    if (d > PAD_COARSE_TRIGGER) {
      // Re-read without moving: a real load is still there, a glitch is not
      int32_t v2;
      if (!pad_sample(travelled, v2)) return false;
      d = _MIN(d, pad_abs(v2 - int32_t(track)));
      if (d > PAD_COARSE_TRIGGER) break;
      if (++glitches > 10) return pad_fail(F("sensor glitches in descent"), travelled);
    }
    else if (d < (PAD_COARSE_TRIGGER) / 4) {
      track += float(v - int32_t(track)) * (1.0f / 32.0f);
      // Bounded so a broken sensor can't wander off forever; a real contact is a fast jump,
      // which this gate (d < trigger/4, gain 1/32 per sample) never follows.
      track = constrain(track, float(base.avg - DRIFT_TRACK_MAX), float(base.avg + DRIFT_TRACK_MAX));
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
    PadStats rs;
    if (!pad_baseline<16>(F("sensor noisy after contact"), rs)) return false;
    const int32_t residual = rs.avg - int32_t(track);
    SERIAL_ECHOLNPGM("PAD_HOME: backed off ", p_float_t(BACKOFF_MM, 2), " mm, residual=", residual);
    // A heated gauge keeps ~1% of a load for a while after it is removed (measured), so only a
    // residual as large as the trigger itself means the nozzle is still pressing. Each fine pass
    // takes its own fresh baseline.
    if (pad_abs(residual) > (PAD_COARSE_TRIGGER)) return pad_fail(F("pad still loaded after backoff"), travelled - backoff_n);
  }

  // ---- Fine passes, from the backed-off position ----
  ui.set_status(F("Pad Z homing: measuring"));
  const long fine_n = pad_steps(FINE_STEP_MM), window_n = pad_steps(FINE_WINDOW_MM);
  long c[FINE_PASSES];
  for (uint8_t i = 0; i < FINE_PASSES; ++i) {
    if (!pad_fine_pass(i + 1, fine_n, backoff_n + window_n, c[i])) return false;
    if (c[i] < backoff_n - window_n) return pad_fail(F("fine pass: early trigger"));   // not the coarse contact
    for (uint8_t j = i; j > 0 && c[j - 1] > c[j]; --j) { const long t = c[j]; c[j] = c[j - 1]; c[j - 1] = t; }
  }
  const float spread_mm = float(c[FINE_PASSES - 1] - c[0]) / spm,
              above_mm  = float(c[FINE_PASSES / 2]) / spm;   // median height above the contact
  SERIAL_ECHOLNPGM("PAD_HOME: median_mm=", p_float_t(above_mm, 3), " spread_mm=", p_float_t(spread_mm, 3));
  if (spread_mm > MAX_SPREAD_MM) return pad_fail(F("fine passes disagree"));

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
  return true;
}

/**
 * After a failure, with nothing moved since: make the machine safe, report, stop the job.
 * The nozzle is lifted by what was travelled down (at most FAIL_LIFT_MM). Its height is then
 * unknown: Z stays unhomed/untrusted and is called 0, so any later Z move (G28's own raise,
 * the SD-abort G-code, a print line still queued) can only go up from here.
 */
static void pad_abort() {
  planner.quick_stop();                                // drop any move queued meanwhile (e.g. from the LCD)
  pad_step_z(_MIN(pad_err_down, pad_steps(FAIL_LIFT_MM)), true);
  thermalManager.disable_all_heaters();
  const float e = motion.position.e;                   // logical E (the touch retract is not counted)
  motion.quickstop_stepper();                          // planner empty; X/Y from the steppers (a queued move may have been cut)
  motion.set_axis_untrusted(Z_AXIS);
  motion.set_axis_unhomed(Z_AXIS);
  motion.position.e = e;
  motion.position.z = 0;
  motion.sync_plan_position();

  // Stop the job: an SD print is aborted by the main loop before its next command
  // (as with M524: queue cleared, heaters off, EVENT_GCODE_SD_ABORT); a host is told to cancel.
  TERN_(HAS_MEDIA, card.abortFilePrintSoon());
  #ifdef ACTION_ON_CANCEL
    hostui.cancel();
  #endif

  SERIAL_ERROR_START();
  SERIAL_ECHOLN(F("PAD_HOME: FAILED: "), pad_err);

  char msg[64];
  strcpy(msg, "Pad Z homing FAILED: ");
  strncat(msg, FTOP(pad_err), sizeof(msg) - strlen(msg) - 1);
  ui.set_status(msg, true);
  #if ENABLED(DWIN_CREALITY_LCD)
    // Info screen; the knob returns to the main menu. Not when the print was stopped
    // elsewhere: the LCD's own stop flow is then showing and finishes when G28 returns.
    if (!pad_err_stopped) DWIN_Show_M117(msg);
    DWIN_UpdateLCD();
  #endif
}

bool pad_home_z() {
  SERIAL_ECHOLNPGM("PAD_HOME: start");
  ui.set_status(F("Pad Z homing"));
  pad_err = nullptr;
  pad_err_down = 0;
  pad_err_stopped = false;

  // Targets to restore afterwards: those from before G28 (off if they were off)
  const bool had_preheat = saved_nozzle >= 0;
  const int16_t restore_nozzle = had_preheat ? saved_nozzle : thermalManager.degTargetHotend(0),
                restore_bed = had_preheat ? saved_bed : TERN0(HAS_HEATED_BED, thermalManager.degTargetBed());
  saved_nozzle = -1;

  if (pad_home_run(restore_nozzle, restore_bed)) {
    ui.reset_status();
    return true;
  }
  if (!pad_err) pad_err = F("unknown");                // every failure path sets it; be safe
  pad_abort();
  return false;
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
