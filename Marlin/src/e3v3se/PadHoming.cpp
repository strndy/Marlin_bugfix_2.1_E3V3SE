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
#if ENABLED(PAD_SCRAPE)
  #include "PadScrapeCtl.h"
#endif
#if PAD_MESH_NORMALIZE
  #include "../module/probe.h"
#endif
#if PAD_MESH_NORMALIZE || (ENABLED(PAD_SCRAPE) && HAS_LEVELING)
  #include "../feature/bedlevel/bedlevel.h"
#endif

// Motion constants (mm). Only the force thresholds are user settings.
constexpr float Z_SPEED_MM_S      = 4,     // direct stepping speed, no acceleration
                COARSE_STEP_MM    = 0.05f, // coarse descent step, one sample per step
                FINE_STEP_MM      = 0.02f, // fine pass step
                BACKOFF_MM        = 1.0f,  // lift after the coarse contact; fine passes start here
                FINE_WINDOW_MM    = 0.5f,  // fine contact must be within this of the coarse contact
                MAX_SPREAD_MM     = 0.03f, // fine passes must agree within this (clean: <=0.007, dirty tip: 0.045..0.07)
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

#if ENABLED(PAD_SCRAPE)
  bool pad_scrape_enabled = true;
#endif
#if ENABLED(PAD_SCRAPE)
  static bool pad_scraped,       // the nozzle was scraped in this homing
              pad_err_dirty;     // the failure is one a dirty nozzle tip causes
  // A failure of the precise measurement that plastic on the tip explains
  static bool pad_fail_dirty(FSTR_P const why, const long down=0) {
    pad_err_dirty = true;
    return pad_fail(why, down);
  }
  #define PAD_FAIL_DIRTY pad_fail_dirty
#else
  #define PAD_FAIL_DIRTY pad_fail
#endif

// ---- Touch at print temperature ----
// A target already set (e.g. by the print's start G-code) is used; below these it counts as unset.
constexpr int16_t NOZZLE_SET_MIN = 50, BED_SET_MIN = 30;
constexpr millis_t HEAT_LIMIT_MS = SEC_TO_MS(900);   // a hot bed can take minutes
constexpr float TOUCH_RETRACT_MM = TERN(PAD_SCRAPE, PAD_SCRAPE_RETRACT, 1); // pulled back against ooze, not restored
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
  // A nozzle hot enough to extrude means the wipe runs first: the bed soaks meanwhile
  TERN_(PAD_SCRAPE, const bool soak = thermalManager.targetTooColdToExtrude(0));
  for (;;) {
    marlin.idle(true);                                 // true: keep the steppers energized
    const millis_t ms = millis();
    const int16_t nt = thermalManager.degTargetHotend(0);
    bool hot = thermalManager.wholeDegHotend(0) >= nt - 2;
    #if HAS_HEATED_BED
      const int16_t bt = thermalManager.degTargetBed();
      if (bt > 0 && !bed_since && thermalManager.wholeDegBed() >= bt - 1) {
        bed_since = ms ?: 1;
        #if ENABLED(PAD_SCRAPE)
          if (!soak) SERIAL_ECHOLNPGM("PAD_HOME: bed at target (no soak: it soaks during the wipe)");
          else
        #endif
        SERIAL_ECHOLNPGM("PAD_HOME: bed at target, soaking ", BED_SOAK_MS / 1000, " s");
      }
      hot = hot && (bt == 0 || (bed_since && (TERN0(PAD_SCRAPE, !soak) || ELAPSED(ms, bed_since + BED_SOAK_MS))));
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

  #if ENABLED(PAD_SCRAPE) && PAD_SCRAPE_OOZE_WAIT > 0
    // A freshly heated nozzle keeps oozing for a while: let it drip off the bed (parked at
    // PAD_SCRAPE_PARK_X) before the retract, the rough touch and the wipe.
    if (pad_scrape_enabled && !thermalManager.tooColdToExtrude(0)) {
      SERIAL_ECHOLNPGM("PAD_SCRAPE: letting the nozzle ooze off the bed for ", PAD_SCRAPE_OOZE_WAIT, " s");
      ui.set_status(F("Pad Z homing: oozing off"));
      for (const millis_t end = millis() + SEC_TO_MS(PAD_SCRAPE_OOZE_WAIT); PENDING(millis(), end);) {
        marlin.idle(true);
        if (TERN0(HAS_MEDIA, card.flag.abort_sd_printing) || !marlin.isRunning()) return pad_fail(F("cancelled while oozing"));
      }
    }
  #endif

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
// Taken only with the nozzle in the air, so a load means something else is touching the bed (a
// hand cleaning the strip, a bump): wait for it to go away (BASELINE_TRIES x BASELINE_WAIT_MS)
// before failing.
constexpr uint8_t BASELINE_TRIES = 6;
constexpr millis_t BASELINE_WAIT_MS = 2000;
template<uint8_t N>
[[nodiscard]] static bool pad_baseline(FSTR_P const noisy_msg, PadStats &st) {
  static_assert(N >= 4 * NOISE_TRIM + 4 && N <= 32, "pad_baseline: bad window size");
  for (uint8_t attempt = 1;; ++attempt) {
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
    if (attempt >= BASELINE_TRIES) return pad_fail(noisy_msg);
    SERIAL_ECHOLNPGM("PAD_HOME: bed disturbed? waiting (", attempt, "/", BASELINE_TRIES - 1, ")");
    for (const millis_t end = millis() + BASELINE_WAIT_MS; PENDING(millis(), end);) {
      marlin.idle_no_sleep();
      if (TERN0(HAS_MEDIA, card.flag.abort_sd_printing) || !marlin.isRunning()) return pad_fail(F("interrupted"));
    }
    pad_reader_reset();
    continue;
  }
  return true;
  }
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
    if (down + step_n > max_steps) return PAD_FAIL_DIRTY(F("fine pass: no contact in range"), down);
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

  // Contact = where the load crosses PAD_FINE_TRIGGER (a fixed force), interpolated between the last
  // step below it and the step that reached it. A fixed-force point repeats far better than the start
  // of the rise ("knee"), which drifted 0.03..0.23 mm from homing to homing on the real printer; the
  // constant force offset is calibrated out once with M206 Z. The knee is still logged for reference.
  int32_t k = i;
  while (k > 1 && k - 1 > i - PAD_RING && ring[(k - 1) % PAD_RING] > knee) --k;
  const int32_t d_prev = (i > 1 && i - 1 > i - PAD_RING) ? ring[(i - 1) % PAD_RING] : 0;
  long cross = down;
  if (d > d_prev && d_prev < int32_t(PAD_FINE_TRIGGER)) {
    const float frac = float(d - int32_t(PAD_FINE_TRIGGER)) / float(d - d_prev);   // 0..1 of a step back
    cross = down - lroundf(constrain(frac, 0.0f, 1.0f) * float(step_n));
  }

  pad_step_z(down, true);                              // back to the start position
  safe_delay(150);

  SERIAL_ECHOLNPGM("PAD_HOME: fine #", pass, " base=", b.avg, " noise=", b.noise, " raw=", b.raw,
                   " trig_delta=", d, " prev=", d_prev,
                   " trig_mm=", p_float_t(float(cross) / pad_spm(), 3),
                   " knee_mm=", p_float_t((float(k) - 0.5f) * float(step_n) / pad_spm(), 3));

  contact = cross;
  return true;
}

/**
 * Descend from the current height (where ref, the free reading, was just validated), one HX711
 * sample per COARSE_STEP_MM, until the reading jumps by more than trigger against a reference
 * that follows slow drift only (twice, the second sample without moving). travelled: steps down
 * (also on failure). track: the drift-followed reference at the contact. d: the trigger delta.
 */
[[nodiscard]] static bool pad_descend(const int32_t ref, const int32_t trigger, const long max_n, FSTR_P const no_contact,
                                      long &travelled, float &track, int32_t &d, uint8_t &glitches) {
  const long coarse_n = pad_steps(COARSE_STEP_MM);
  travelled = 0;
  track = ref;
  glitches = 0;
  int32_t v;
  for (;;) {
    if (!pad_check(travelled)) return false;
    if (travelled + coarse_n > max_n) return pad_fail(no_contact, travelled);
    pad_step_z(coarse_n, false);
    travelled += coarse_n;

    if (!pad_sample(travelled, v)) return false;
    d = pad_abs(v - int32_t(track));
    #if ENABLED(PAD_SCRAPE)
      if (pad_abs(v - ref) > (PAD_SCRAPE_FORCE_MAX)) break;   // loaded against the validated free reading: stop here
    #endif
    if (d > trigger) {
      // Re-read without moving: a real load is still there, a glitch is not
      int32_t v2;
      if (!pad_sample(travelled, v2)) return false;
      d = _MIN(d, pad_abs(v2 - int32_t(track)));
      if (d > trigger) break;
      if (++glitches > 10) return pad_fail(F("sensor glitches in descent"), travelled);
    }
    else if (d < trigger / 4) {
      track += float(v - int32_t(track)) * (1.0f / 32.0f);
      // Bounded so a broken sensor can't wander off forever; a real contact is a fast jump,
      // which this gate (d < trigger/4, gain 1/32 per sample) never follows.
      track = constrain(track, float(ref - DRIFT_TRACK_MAX), float(ref + DRIFT_TRACK_MAX));
    }
    marlin.idle_no_sleep();                            // heaters, watchdog, LCD
  }
  return true;
}

#if ENABLED(PAD_SCRAPE)

  constexpr float TRAVEL_MM         = 5,      // travel height above the highest contact known
                  SCRAPE_SEARCH_MM  = 1,      // M8016 rough descent: no contact this far below the expected height = skip
                  SCRAPE_FLOOR_MM   = 0.4f,   // never deeper than this below the strip contact
                  SCRAPE_CEIL_MM    = 1,      // up this far over the contact and still over the band: end the wipe
                  WIPE_NEAR_MM      = 0.3f,   // the approach to the wipe depth slows down from here over the contact
                  WIPE_DOWN_MM      = 0.01f,  // ... to this per HX711 sample
                  WIPE_V0_MM_S      = 5,      // feed start/stop speed (under the X/Y jerk)
                  WIPE_ACCEL        = 2000,   // feed acceleration (mm/s^2)
                  PRECISE_MARGIN_MM = 2;      // precise descent: no pad contact this far below the strip height = fail
  // Free reading at the wipe start vs the baseline before the rough touch: catches a reference taken while
  // loaded; allows the drift of an unsoaked bed over the ~10 s between them (up to ~300 counts/s)
  constexpr int32_t SCRAPE_REF_TOL = 2 * (PAD_COARSE_TRIGGER);

  static_assert(WITHIN(PAD_SCRAPE_X, X_MIN_POS, X_MAX_POS) && WITHIN(PAD_SCRAPE_PARK_X, X_MIN_POS, X_MAX_POS), "PAD_SCRAPE_X/PARK_X are out of the X range.");
  static_assert(WITHIN(PAD_SCRAPE_ROUGH_Y, Y_MIN_POS, Y_MAX_POS) && WITHIN(PAD_SCRAPE_Y0, Y_MIN_POS, Y_MAX_POS)
             && WITHIN(PAD_SCRAPE_Y1, Y_MIN_POS, Y_MAX_POS) && WITHIN(PAD_SCRAPE_PARK_Y, Y_MIN_POS, Y_MAX_POS), "PAD_SCRAPE_*Y are out of the Y range.");
  static_assert(ABS((PAD_SCRAPE_Y1) - (PAD_SCRAPE_Y0)) >= 5, "PAD_SCRAPE_Y0..Y1 must be at least 5 mm long.");
  static_assert(ABS((AUTOZ_TOOL_X) - (PAD_SCRAPE_X)) >= 20, "The pad spot must be at least 20 mm from the scrape strip.");
  static_assert(PAD_SCRAPE_PASSES >= 1 && PAD_SCRAPE_PASSES <= 20, "PAD_SCRAPE_PASSES must be 1..20.");
  static_assert(PAD_SCRAPE_SPEED >= 1 && PAD_SCRAPE_SPEED <= 50, "PAD_SCRAPE_SPEED must be 1..50 mm/s.");
  static_assert(PAD_SCRAPE_TRIGGER > PAD_NOISE_MAX, "Need PAD_SCRAPE_TRIGGER > PAD_NOISE_MAX.");
  static_assert(PAD_SCRAPE_BAND > PAD_NOISE_MAX && PAD_SCRAPE_FORCE > PAD_SCRAPE_BAND
             && PAD_SCRAPE_FORCE + PAD_SCRAPE_BAND < PAD_SCRAPE_FORCE_MAX && PAD_SCRAPE_FORCE_MAX <= 60000,
                "Need PAD_NOISE_MAX < PAD_SCRAPE_BAND < PAD_SCRAPE_FORCE, PAD_SCRAPE_FORCE + PAD_SCRAPE_BAND < PAD_SCRAPE_FORCE_MAX <= 60000.");
  static_assert(PAD_SCRAPE_DEPTH >= 0 && PAD_SCRAPE_DEPTH <= SCRAPE_FLOOR_MM, "PAD_SCRAPE_DEPTH must be 0..0.4 mm.");
  #ifdef PAD_SCRAPE_FLICK_X
    static_assert(PAD_SCRAPE_FLICK_X >= X_MIN_POS && PAD_SCRAPE_FLICK_X < PAD_SCRAPE_X, "PAD_SCRAPE_FLICK_X must be in X_MIN_POS..PAD_SCRAPE_X.");
  #endif
  static_assert(PAD_SCRAPE_RETRACT >= 0 && PAD_SCRAPE_RETRACT <= 10, "PAD_SCRAPE_RETRACT must be 0..10 mm.");

  PadScrapeParams pad_scrape_defaults() {
    PadScrapeParams p;
    p.x = PAD_SCRAPE_X; p.rough_y = PAD_SCRAPE_ROUGH_Y; p.y0 = PAD_SCRAPE_Y0; p.y1 = PAD_SCRAPE_Y1;
    p.depth = PAD_SCRAPE_DEPTH; p.speed = PAD_SCRAPE_SPEED; p.contact = 0;
    #ifdef PAD_SCRAPE_FLICK_X
      p.flick_x = PAD_SCRAPE_FLICK_X;
    #else
      p.flick_x = PAD_SCRAPE_X;                        // = x: no flick
    #endif
    p.target = PAD_SCRAPE_FORCE; p.band = PAD_SCRAPE_BAND; p.hard = PAD_SCRAPE_FORCE_MAX;
    p.passes = PAD_SCRAPE_PASSES;
    return p;
  }

  // Direct-step Z to height 'to' (steps). zs is the height now, in the same frame.
  static void pad_z_to(long &zs, const long to) {
    if (to > zs) pad_step_z(to - zs, true); else pad_step_z(zs - to, false);
    zs = to;
  }

  /**
   * One wipe pass along X or Y to 'to', at the load the control holds (Z between the depth and
   * the lift limit). Z and the feed axis are stepped directly *inside* the wait for each HX711
   * conversion, so the load is read at the full HX711 rate (~80/s) while moving, and every sample
   * sets Z and the feed for the very next steps. The feed accelerates from WIPE_V0_MM_S at
   * WIPE_ACCEL and slows down again for the end and whenever the control stops it. Nothing else
   * blocks the loop (only the heater task runs). Stepper ISR idle, nothing queued.
   * moved: steps done on that axis, not yet in motion.position. ended: the wipe ended (hard limit,
   * no relief); the nozzle is lifted to travel height. false = failed (pad_err set).
   */
  [[nodiscard]] static bool pad_wipe_pass(const PadScrapeParams &p, const PadWipeCtl &c, const int32_t ref, const uint8_t pass,
                                          const AxisEnum axis, const float to, long &zs, long &moved, bool &ended) {
    const bool is_x = axis == X_AXIS;
    const float spm = pad_spm(), spm_a = planner.settings.axis_steps_per_mm[axis],
                from = motion.position[axis] + float(moved) / spm_a;
    const long total = lroundf((to - from) * spm_a);
    const bool fwd = total > 0;
    long left = ABS(total);
    const uint32_t z_us = _MAX(20UL, uint32_t(1000000.0f / (spm * Z_SPEED_MM_S)));
    const float vmax = _MAX(p.speed, float(WIPE_V0_MM_S)), dv2 = 2.0f * WIPE_ACCEL / spm_a;   // v^2 change per step
    const millis_t start = millis(), limit = start + SEC_TO_MS(20) + millis_t(5000.0f * ABS(to - from) / WIPE_V0_MM_S);

    int32_t fmin = INT32_MAX, fmax = 0;
    int64_t fsum = 0;
    uint32_t samples = 0, in_band = 0, below = 0;
    long fed = 0;
    FSTR_P err = nullptr;

    const bool old_adir = is_x ? X_DIR_READ() : Y_DIR_READ(), old_zdir = Z_DIR_READ();
    if (is_x) X_DIR_WRITE(fwd); else Y_DIR_WRITE(fwd);
    bool z_up = false;
    Z_DIR_WRITE(false);
    DELAY_US(100);
    uint32_t next_a = micros(), next_z = next_a;
    float v = 0;                                       // feed speed now (0 = stopped)
    long z_goal = zs;
    bool feed = false, band = false;
    for (;;) {
      // Step Z towards the goal and the feed axis while the HX711 converts
      const millis_t t0 = millis();
      while (READ(HX711_SDO_PIN)) {
        if (millis() - t0 > HX711_TIMEOUT_MS) { err = F("HX711 no data"); break; }
        const uint32_t now = micros();
        if (zs != z_goal && int32_t(now - next_z) >= 0) {
          const bool up = z_goal > zs;
          if (up != z_up) { Z_DIR_WRITE(up); z_up = up; DELAY_US(20); }
          Z_STEP_WRITE(STEP_STATE_Z);
          DELAY_US(2);
          Z_STEP_WRITE(!STEP_STATE_Z);
          zs += up ? 1 : -1;
          next_z = now + z_us;
        }
        if (left && int32_t(now - next_a) >= 0) {
          // Speed for this step: up to vmax, down to stop for the end of the pass or when the feed is off
          v = pad_feed_speed(v, feed, left, vmax, float(WIPE_V0_MM_S), dv2, [](const float x) { return SQRT(x); });
          if (v > 0) {
            if (is_x) { X_STEP_WRITE(STEP_STATE_X); DELAY_US(2); X_STEP_WRITE(!STEP_STATE_X); }
            else      { Y_STEP_WRITE(STEP_STATE_Y); DELAY_US(2); Y_STEP_WRITE(!STEP_STATE_Y); }
            --left;
            ++fed;
            moved += fwd ? 1 : -1;
            next_a = now + uint32_t(1000000.0f / (v * spm_a));
          }
          else
            next_a = now + 1000;                       // stopped: look again in 1 ms
        }
        hal.watchdog_refresh();
      }
      int32_t val;
      if (err || (err = pad_read(val))) break;
      const int32_t f = pad_abs(val - ref);
      ++samples;
      NOMORE(fmin, f); NOLESS(fmax, f); fsum += f;
      if (zs <= 0) ++below;
      if (pad_wipe_control(c, f, zs, z_goal, feed, band) == WIPE_END) {
        Z_DIR_WRITE(old_zdir);                         // pad_step_z sets and restores DIR itself
        pad_z_to(zs, pad_steps(TRAVEL_MM));            // lift at once, then report
        SERIAL_ECHOPGM("PAD_SCRAPE: load ", f);
        SERIAL_ECHO(f > c.hard ? F(" over the hard limit") : F(" still over the band at the lift limit"));
        SERIAL_ECHOLNPGM(" (hard ", c.hard, "): lifted, wipe ended");
        ended = true;
        break;
      }
      if (band) ++in_band;
      if (!left) break;                                // pass done (Z keeps its height into the next one)
      if (!pad_check(0)) break;                        // pad_err set
      if (ELAPSED(millis(), limit)) { err = F("wipe timeout"); break; }
      if ((samples & 7) == 0) thermalManager.task();   // heaters only: idle() (LCD) would stall the loop
    }
    const millis_t took = millis() - start;
    if (is_x) X_DIR_WRITE(old_adir); else Y_DIR_WRITE(old_adir);
    Z_DIR_WRITE(old_zdir);
    DELAY_US(100);

    SERIAL_ECHOPGM("PAD_SCRAPE: pass ", pass, is_x ? " X" : " Y", p_float_t(from, 1), "->", p_float_t(from + float(fwd ? fed : -fed) / spm_a, 1),
                   ": ", took, " ms, samples=", samples, " (in band ", in_band, ", under the contact ", below, ")");
    if (samples) SERIAL_ECHOPGM(" load min/avg/max=", fmin, "/", int32_t(fsum / int64_t(samples)), "/", fmax);
    SERIAL_ECHOLNPGM(" (~100 counts/g), Z ", p_float_t(float(zs) / spm, 3), " from the contact");
    if (err) return pad_fail(err);
    return !pad_err;
  }

  /**
   * Over to the wipe start at travel height, free reference there, then the passes and the flick.
   * zs: height (steps) above the rough contact, kept exact. false = failed (pad_err set).
   */
  [[nodiscard]] static bool pad_scrape_run(const PadScrapeParams &p, const int32_t base, long &zs, long &x_moved, long &y_moved) {
    pad_z_to(zs, pad_steps(TRAVEL_MM));                // lift off the rough spot
    motion.blocking_move_xy(p.x, p.y0, 80);            // at travel height (planner: X/Y only)
    planner.synchronize();
    if (!pad_check(0)) return false;
    safe_delay(300);

    // ---- Free reference in the air, checked against the baseline taken before the rough touch ----
    pad_reader_reset();
    PadStats r;
    if (!pad_baseline<16>(F("reference noisy"), r)) return false;
    SERIAL_ECHOLNPGM("PAD_SCRAPE: reference=", r.avg, " baseline=", base, " noise=", r.noise);
    if (pad_abs(r.avg - base) > SCRAPE_REF_TOL) return pad_fail(F("reference does not match the baseline"));

    ui.set_status(F("Pad Z homing: wiping nozzle"));
    stepper.enable_axis(X_AXIS);
    stepper.enable_axis(Y_AXIS);
    const PadWipeCtl c = { p.target, p.band, p.hard, -pad_steps(_MIN(p.depth, SCRAPE_FLOOR_MM)), pad_steps(SCRAPE_CEIL_MM),
                           pad_steps(WIPE_NEAR_MM), pad_steps(COARSE_STEP_MM), pad_steps(WIPE_DOWN_MM), pad_steps(COARSE_STEP_MM) };
    bool ended = false;
    for (uint8_t i = 0; i < p.passes && !ended; ++i)
      if (!pad_wipe_pass(p, c, r.avg, i + 1, Y_AXIS, (i & 1) ? p.y0 : p.y1, zs, y_moved, ended)) return false;
    // Flick off the strip edge towards -X at the wipe height: material on the tip stays out there
    if (!ended && p.flick_x < p.x && !pad_wipe_pass(p, c, r.avg, 0, X_AXIS, p.flick_x, zs, x_moved, ended)) return false;
    return ended ? pad_fail(F("load limit, wipe ended early")) : true;
  }

  /**
   * The wipe, from the rough contact on the strip (zs = height above it, steps). Always ends at
   * least TRAVEL_MM over that contact (lifting off where the wipe ended: after the flick, off the
   * bed) with motion.position X/Y matching the steppers.
   * true = done or skipped (a warning; pad_err cleared). false = homing must fail (stopped,
   * interrupted); pad_err set, nothing left to lift.
   */
  [[nodiscard]] static bool pad_scrape(const PadScrapeParams &p, const int32_t base, long &zs) {
    SERIAL_ECHOLNPGM("PAD_SCRAPE: wipe X", p_float_t(p.x, 1), " Y", p_float_t(p.y0, 1), "<->", p_float_t(p.y1, 1), " x", p.passes,
                     " at ", p_float_t(p.speed, 0), " mm/s, load ", p.target, "+-", p.band, " hard ", p.hard,
                     ", depth max ", p_float_t(p.depth, 2), " mm, flick to X", p_float_t(p.flick_x, 1));
    long x_moved = 0, y_moved = 0;
    pad_err = nullptr;
    const bool ok = pad_scrape_run(p, base, zs, x_moved, y_moved);
    pad_z_to(zs, _MAX(zs, pad_steps(TRAVEL_MM)));      // lift-off where the wipe ended
    // Stopped, killed or something queued a move: homing must fail. Anything else only skips the wipe.
    const bool fatal = !ok && (pad_err_stopped || !marlin.isRunning() || planner.has_blocks_queued());

    // Direct X/Y steps into the position. Something queued meanwhile (pad_check failed) may have run:
    // drop it and take the steppers' counts first.
    if (planner.has_blocks_queued()) {
      planner.quick_stop();
      planner.synchronize();
      motion.set_current_from_steppers_for_axis(ALL_AXES_ENUM);
    }
    motion.position.x += float(x_moved) / planner.settings.axis_steps_per_mm[X_AXIS];
    motion.position.y += float(y_moved) / planner.settings.axis_steps_per_mm[Y_AXIS];
    motion.sync_plan_position();

    if (ok) { SERIAL_ECHOLNPGM("PAD_SCRAPE: done"); return true; }
    if (!pad_err) pad_err = F("unknown");
    pad_err_down = 0;                                  // already lifted
    if (fatal) return false;
    SERIAL_ECHOLN(F("PAD_SCRAPE: WARNING: wipe incomplete: "), pad_err);
    ui.set_status(F("Pad Z homing: wipe incomplete"));
    pad_err = nullptr;
    return true;
  }

#endif // PAD_SCRAPE

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
  #if ENABLED(PAD_SCRAPE)
    // The wipe will run (hot enough to extrude): wait off the bed, so ooze drips outside it
    if (PAD_HOME_NOZZLE_TEMP > 0 && !thermalManager.targetTooColdToExtrude(0)) {
      motion.blocking_move_xy(float(PAD_SCRAPE_PARK_X), float(PAD_SCRAPE_PARK_Y), 80);
      planner.synchronize();
      SERIAL_ECHOLNPGM("PAD_SCRAPE: heating parked at X", PAD_SCRAPE_PARK_X, " Y", PAD_SCRAPE_PARK_Y);
    }
  #endif
  if (!pad_heat_and_wait()) return false;

  #if ENABLED(PAD_SCRAPE)
    // Hot enough to extrude: the tip can carry a blob. Then the rough touch is on the scrape strip
    // (a squashed blob lands where it is scraped next) and the pad spot stays untouched until the
    // nozzle is clean.
    const bool scrape = pad_scrape_enabled && !thermalManager.tooColdToExtrude(0);
    const float first_x = scrape ? float(PAD_SCRAPE_X) : float(AUTOZ_TOOL_X),
                first_y = scrape ? float(PAD_SCRAPE_ROUGH_Y) : float(AUTOZ_TOOL_Y);
  #endif

  // X/Y only, at the current height (G28 has already raised Z)
  motion.sync_plan_position();
  motion.blocking_move_xy(TERN(PAD_SCRAPE, first_x, float(AUTOZ_TOOL_X)), TERN(PAD_SCRAPE, first_y, float(AUTOZ_TOOL_Y)), 80);
  planner.synchronize();
  if (!pad_check(0)) return false;
  stepper.enable_axis(Z_AXIS);
  safe_delay(300);

  // ---- Wait for the gauge to stop drifting (a freshly heated bed drifts up to ~300 counts/s) ----
  // (Not in the wipe sequence: every second hot lets new ooze form. The descents track drift, and the
  // baselines and fine passes check noise and drift themselves.)
  pad_reader_reset();
  if (TERN1(PAD_SCRAPE, !scrape) && !pad_wait_settled()) return false;

  // ---- Sensor health, before any descent ----
  const millis_t t0 = millis();
  PadStats base;
  if (!pad_baseline<32>(F("sensor noisy or loaded"), base)) return false;
  SERIAL_ECHOLNPGM("PAD_HOME: sensor ok, baseline=", base.avg, " noise=", base.noise, " raw=", base.raw,
                   " sample_ms=", (millis() - t0) / 33);

  // ---- Coarse descent ----
  float max_mm = Z_MAX_POS + 10;
  if (z_known) NOMORE(max_mm, _MAX(z_entry, 0.0f) + KNOWN_Z_MARGIN_MM);
  const long max_n = pad_steps(max_mm);
  SERIAL_ECHOLNPGM("PAD_HOME: descending, travel limit ", p_float_t(max_mm, 1), z_known ? " mm (Z was known)" : " mm (Z unknown)");
  ui.set_status(F("Pad Z homing: descending"));
  long travelled;
  float track;                                         // follows slow (thermal) drift, bounded
  int32_t d;
  uint8_t glitches;
  if (!pad_descend(base.avg, TERN(PAD_SCRAPE, scrape ? int32_t(PAD_SCRAPE_TRIGGER) : int32_t(PAD_COARSE_TRIGGER), PAD_COARSE_TRIGGER),
                   max_n, F("no pad contact (max travel)"), travelled, track, d, glitches)) return false;

  SERIAL_ECHOLNPGM("PAD_HOME: coarse contact after ", p_float_t(float(travelled) / spm, 2),
                   " mm, delta=", d, " drift=", int32_t(track) - base.avg, " glitches=", glitches);

  #if ENABLED(PAD_SCRAPE)
    long pre_n = 0,                                    // steps from the entry height down to the precise descent start
         zs = 0;                                       // height above the rough contact (steps)
    if (scrape) {
      // That was the rough touch, on the strip: scrape from here, then home on the clean pad spot
      SERIAL_ECHOLNPGM("PAD_SCRAPE: rough touch on the strip at X", p_float_t(first_x, 1), " Y", p_float_t(first_y, 1));
      if (!pad_scrape(pad_scrape_defaults(), base.avg, zs)) return false;
      pad_scraped = true;
      pre_n = travelled - zs;

      // Over the pad spot at travel height (from the wipe's far end; the pad spot is untouched so far)
      ui.set_status(F("Pad Z homing: descending"));
      motion.blocking_move_xy(float(AUTOZ_TOOL_X), float(AUTOZ_TOOL_Y), 80);
      planner.synchronize();
      if (!pad_check(0)) return false;
      // Straight on, before new ooze forms: no soak, no settle. A fresh baseline in the air (noise
      // and drift checked); each fine pass takes its own, and the spread check catches the rest.
      safe_delay(300);
      pad_reader_reset();
      if (!pad_baseline<32>(F("sensor noisy or loaded"), base)) return false;
      SERIAL_ECHOLNPGM("PAD_HOME: precise spot X", AUTOZ_TOOL_X, " Y", AUTOZ_TOOL_Y, ", baseline=", base.avg, " noise=", base.noise);
      if (!pad_descend(base.avg, PAD_COARSE_TRIGGER, zs + pad_steps(PRECISE_MARGIN_MM), F("no pad contact (max travel)"),
                       travelled, track, d, glitches)) return false;
      SERIAL_ECHOLNPGM("PAD_HOME: coarse contact after ", p_float_t(float(travelled) / spm, 2),
                       " mm, delta=", d, " drift=", int32_t(track) - base.avg, " glitches=", glitches);
    }
  #endif

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
    if (pad_abs(residual) > (PAD_COARSE_TRIGGER)) return PAD_FAIL_DIRTY(F("pad still loaded after backoff"), travelled - backoff_n);
  }

  // ---- Fine passes, from the backed-off position ----
  ui.set_status(F("Pad Z homing: measuring"));
  const long fine_n = pad_steps(FINE_STEP_MM), window_n = pad_steps(FINE_WINDOW_MM);
  long c[FINE_PASSES];
  for (uint8_t i = 0; i < FINE_PASSES; ++i) {
    if (!pad_fine_pass(i + 1, fine_n, backoff_n + window_n, c[i])) return false;
    if (c[i] < backoff_n - window_n) return PAD_FAIL_DIRTY(F("fine pass: early trigger"));   // not the coarse contact
    for (uint8_t j = i; j > 0 && c[j - 1] > c[j]; --j) { const long t = c[j]; c[j] = c[j - 1]; c[j - 1] = t; }
  }
  const float spread_mm = float(c[FINE_PASSES - 1] - c[0]) / spm,
              above_mm  = float(c[FINE_PASSES / 2]) / spm;   // median height above the contact
  SERIAL_ECHOLNPGM("PAD_HOME: median_mm=", p_float_t(above_mm, 3), " spread_mm=", p_float_t(spread_mm, 3));
  if (spread_mm > MAX_SPREAD_MM) return PAD_FAIL_DIRTY(F("fine passes disagree"));
  #if ENABLED(PAD_SCRAPE)
    // The nozzle is above_mm over the pad contact; the rough contact is travelled - backoff_n - zs steps above the nozzle
    if (scrape) SERIAL_ECHOLNPGM("PAD_SCRAPE: strip contact at the rough spot was Z",
                                 p_float_t(above_mm + float(travelled - backoff_n - zs) / spm, 3), " (relative to the pad contact)");
  #endif

  if (z_known)
    SERIAL_ECHOLNPGM("PAD_HOME: contact moved by ",
      p_float_t(z_entry - float(TERN0(PAD_SCRAPE, pre_n) + travelled - backoff_n) / spm - above_mm - z_shift, 3), " mm since the previous homing");

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
  #if ENABLED(PAD_SCRAPE)
    // The scrape did not clean the tip: say so rather than only what the measurement saw
    const bool dirty = pad_scraped && pad_err_dirty;
    if (dirty) SERIAL_ECHOLN(F("PAD_HOME: FAILED: nozzle still dirty after scrape ("), pad_err, F(")"));
    else
  #endif
  SERIAL_ECHOLN(F("PAD_HOME: FAILED: "), pad_err);

  char msg[64];
  strcpy(msg, "Pad Z homing FAILED: ");
  #if ENABLED(PAD_SCRAPE)
    strncat(msg, dirty ? "nozzle still dirty after scrape" : FTOP(pad_err), sizeof(msg) - strlen(msg) - 1);
  #else
    strncat(msg, FTOP(pad_err), sizeof(msg) - strlen(msg) - 1);
  #endif
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
  TERN_(PAD_SCRAPE, pad_scraped = pad_err_dirty = false);

  // Targets to restore afterwards: those from before G28 (off if they were off)
  const bool had_preheat = saved_nozzle >= 0;
  const int16_t restore_nozzle = had_preheat ? saved_nozzle : thermalManager.degTargetHotend(0),
                restore_bed = had_preheat ? saved_bed : TERN0(HAS_HEATED_BED, thermalManager.degTargetBed());
  saved_nozzle = -1;

  bool ok = pad_home_run(restore_nozzle, restore_bed);
  #if ENABLED(PAD_SCRAPE)
    // Dirty tip after the wipe (fine passes disagree etc.): lift, forget Z, and run the whole
    // sequence once more (rough touch, wipe, precise touch) before giving up.
    if (!ok && pad_scraped && pad_err_dirty && !pad_err_stopped) {
      SERIAL_ECHOLN(F("PAD_HOME: nozzle still dirty ("), pad_err, F("), wiping again"));
      planner.quick_stop();
      pad_step_z(_MIN(pad_err_down, pad_steps(FAIL_LIFT_MM)), true);
      motion.quickstop_stepper();
      motion.set_axis_untrusted(Z_AXIS);
      motion.set_axis_unhomed(Z_AXIS);
      motion.position.z = 0;
      motion.sync_plan_position();
      pad_err = nullptr;
      pad_err_down = 0;
      pad_scraped = pad_err_dirty = false;
      ok = pad_home_run(restore_nozzle, restore_bed);
    }
  #endif
  if (ok) {
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

#if ENABLED(PAD_SCRAPE)

  // M8016: rough touch + wipe alone, for tuning. Z homed; leveling off meanwhile; X/Y/Z stay exact.
  void pad_scrape_only(const PadScrapeParams &p) {
    if (motion.axes_should_home(_BV(X_AXIS) | _BV(Y_AXIS) | _BV(Z_AXIS))
      || !motion.axis_is_trusted(X_AXIS) || !motion.axis_is_trusted(Y_AXIS) || !motion.axis_is_trusted(Z_AXIS)) {
      SERIAL_ECHOLNPGM("PAD_SCRAPE: home first (G28)");
      return;
    }
    planner.synchronize();
    pad_err = nullptr;
    pad_err_down = 0;
    pad_err_stopped = false;
    if (thermalManager.tooColdToExtrude(0)) SERIAL_ECHOLNPGM("PAD_SCRAPE: note: the nozzle is cold");
    ui.set_status(F("Pad scrape"));
    #if HAS_LEVELING
      const bool leveling_was = planner.leveling_active;
      set_bed_leveling_enabled(false);
    #endif

    // Native Z of the expected strip contact: homed contact (+ M206 Z) + p.contact
    const float spm = pad_spm(), za = TERN0(HAS_HOME_OFFSET, motion.home_offset.z) + p.contact;
    if (motion.position.z < za + TRAVEL_MM) motion.blocking_move_z(za + TRAVEL_MM, motion.homing_feedrate(Z_AXIS));
    motion.blocking_move_xy(p.x, p.rough_y, 80);
    planner.synchronize();
    stepper.enable_axis(Z_AXIS);
    const float z_start = motion.position.z;
    const long zs0 = lroundf((z_start - za) * spm);
    long zs = zs0;                                     // height over the expected contact (steps)
    safe_delay(300);
    pad_reader_reset();

    // Free reference here, then the rough touch: down to SCRAPE_SEARCH_MM under the expected height
    bool ok = true;
    PadStats b;
    long down = 0;
    float track;
    int32_t d;
    uint8_t glitches;
    if (!pad_baseline<16>(F("sensor noisy or loaded"), b))
      SERIAL_ECHOLN(F("PAD_SCRAPE: WARNING: not started: "), pad_err);
    else {
      SERIAL_ECHOLNPGM("PAD_SCRAPE: baseline=", b.avg, " noise=", b.noise, ", rough touch at X", p_float_t(p.x, 1), " Y", p_float_t(p.rough_y, 1));
      const bool touched = pad_descend(b.avg, PAD_SCRAPE_TRIGGER, zs + pad_steps(SCRAPE_SEARCH_MM), F("no strip contact in range"), down, track, d, glitches);
      zs -= down;
      if (touched) {
        SERIAL_ECHOLNPGM("PAD_SCRAPE: rough contact at Z", p_float_t(float(zs) / spm + p.contact, 3), " delta=", d, " drift=", int32_t(track) - b.avg);
        long zr = 0;                                   // height over the rough contact
        ok = pad_scrape(p, b.avg, zr);
        zs += zr;
      }
      else {
        pad_z_to(zs, zs0);
        ok = !(pad_err_stopped || !marlin.isRunning() || planner.has_blocks_queued());
        if (ok) SERIAL_ECHOLN(F("PAD_SCRAPE: WARNING: not started: "), pad_err);
      }
    }

    if (planner.has_blocks_queued()) { planner.quick_stop(); planner.synchronize(); } // only after a failure; then unhomed below
    motion.position.z = z_start + float(zs - zs0) / spm;  // the direct Z steps
    motion.sync_plan_position();
    if (!ok) {
      SERIAL_ERROR_START();
      SERIAL_ECHOLN(F("PAD_SCRAPE: FAILED: "), pad_err, F(" (home again)"));
      motion.set_all_unhomed();
    }
    TERN_(HAS_LEVELING, set_bed_leveling_enabled(leveling_was));
    ui.reset_status();
    motion.report_position();
  }

#endif // PAD_SCRAPE

#if PAD_MESH_NORMALIZE

  bool pad_mesh_probe(float &ref, const uint8_t verbose) {
    ref = NAN;
    const xy_pos_t pad = { float(AUTOZ_TOOL_X), float(AUTOZ_TOOL_Y) };
    if (!probe.can_reach(pad)) {
      SERIAL_ECHOLNPGM("PAD_MESH: the probe can't reach the pad spot; the mesh is interpolated there");
      return true;
    }
    SERIAL_ECHOLNPGM("PAD_MESH: probing the pad spot X", AUTOZ_TOOL_X, " Y", AUTOZ_TOOL_Y);
    ref = probe.probe_at_point(pad, PROBE_PT_RAISE, verbose);
    return !isnan(ref);
  }

  void pad_mesh_normalize(float ref) {
    const bool probed = !isnan(ref);
    if (!probed) ref = bedlevel.get_z_correction(xy_pos_t({ float(AUTOZ_TOOL_X), float(AUTOZ_TOOL_Y) }));
    if (!isfinite(ref)) { SERIAL_ECHOLNPGM("PAD_MESH: no bed height at the pad spot, mesh left as probed"); return; }
    GRID_LOOP(x, y) bedlevel.z_values[x][y] -= ref;
    bedlevel.refresh_bed_level();
    SERIAL_ECHOPGM("PAD_MESH: bed at the pad spot ", p_float_t(ref, 3));
    SERIAL_ECHO(probed ? F(" mm (probed)") : F(" mm (from the mesh)"));
    SERIAL_ECHOLNPGM(", mesh shifted by ", p_float_t(-ref, 3), " mm: 0 at the pad, independent of M851 Z");
  }

#endif // PAD_MESH_NORMALIZE

#endif // PAD_Z_HOMING
