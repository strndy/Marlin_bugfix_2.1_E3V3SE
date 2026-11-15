/**
 * Pad Z homing: home Z by touching the nozzle on the bed strain-gauge pad.
 *
 * Sequence (called from G28 once X/Y are homed; leveling is already disabled):
 *   1. optional nozzle preheat (PAD_HOME_NOZZLE_TEMP)
 *   2. planner move in X/Y only, to the pad (PAD_HOME_X / PAD_HOME_Y)
 *   3. HX711 health check: answers, not stuck, not saturated, not noisy
 *   4. coarse descent: direct Z stepping, one HX711 sample per PAD_COARSE_STEP_MM,
 *      a sample over the threshold is re-read without moving before it counts
 *   5. back off PAD_BACKOFF_MM, the sensor must relax
 *   6. PAD_FINE_REPEATS fine passes in PAD_FINE_STEP_MM steps, each with a fresh
 *      baseline; contact is the first step of the force rise; median + spread check
 *   7. Z := (height above contact) + PAD_Z_COMPENSATION_MM + M206 Z, lift
 *
 * Safety rules of this file:
 *   - Every downward step is preceded by a travel check on the CURRENT position and
 *     followed by a fresh, validated HX711 sample. No sample (timeout) => no more steps.
 *   - All Z motion below is direct stepping, so endstop / probe pin state, software
 *     endstops and leveling cannot influence it. The probe is never deployed or read.
 *   - Any failure lifts the nozzle a little (when it may be loaded, and never above the
 *     height where the descent started) and calls kill(): heaters off, printer halted.
 */
#include "../inc/MarlinConfig.h"

#if ENABLED(PAD_Z_HOMING)

#include "PadHoming.h"
#include "AutoOffset.h"            // HX711 driver and its pins
#include "../MarlinCore.h"
#include "../gcode/gcode.h"
#include "../module/motion.h"
#include "../module/planner.h"
#include "../module/stepper.h"
#include "../module/temperature.h"
#include "../lcd/marlinui.h"
#if ENABLED(DWIN_CREALITY_LCD)
  #include "../lcd/dwin/creality/dwin.h"
#endif

// ---- Compile-time sanity of the tunables ----
static_assert(PAD_COARSE_STEP_MM >= 0.01 && PAD_COARSE_STEP_MM <= 0.1, "PAD_COARSE_STEP_MM must be 0.01..0.1 mm.");
static_assert(PAD_FINE_STEP_MM >= 0.005 && PAD_FINE_STEP_MM <= PAD_COARSE_STEP_MM, "PAD_FINE_STEP_MM must be 0.005 mm..PAD_COARSE_STEP_MM.");
static_assert(PAD_COARSE_MAX_TRAVEL_MM > 0 && PAD_COARSE_MAX_TRAVEL_MM <= (Z_MAX_POS) + 10, "PAD_COARSE_MAX_TRAVEL_MM must be 0..Z_MAX_POS+10.");
static_assert(PAD_BACKOFF_MM >= 0.5 && PAD_BACKOFF_MM <= 3, "PAD_BACKOFF_MM must be 0.5..3 mm.");
static_assert(PAD_FINE_MAX_DEPTH_MM >= 0.2 && PAD_FINE_MAX_DEPTH_MM < PAD_BACKOFF_MM && PAD_FINE_MAX_DEPTH_MM <= 1, "PAD_FINE_MAX_DEPTH_MM must be 0.2..1 mm and smaller than PAD_BACKOFF_MM.");
static_assert(PAD_FINE_REPEATS >= 1 && PAD_FINE_REPEATS <= 7 && (PAD_FINE_REPEATS & 1), "PAD_FINE_REPEATS must be odd, 1..7.");
static_assert(PAD_NOISE_MAX > 0 && 2 * (PAD_NOISE_MAX) <= PAD_FINE_TRIGGER, "PAD_FINE_TRIGGER must be at least 2 x PAD_NOISE_MAX.");
static_assert(PAD_FINE_TRIGGER <= PAD_COARSE_TRIGGER && PAD_COARSE_TRIGGER < PAD_FORCE_LIMIT, "Need PAD_FINE_TRIGGER <= PAD_COARSE_TRIGGER < PAD_FORCE_LIMIT.");
static_assert(PAD_FORCE_LIMIT <= 2000000, "PAD_FORCE_LIMIT is too large for a 24-bit sensor.");
static_assert(PAD_Z_COMPENSATION_MM >= -0.5 && PAD_Z_COMPENSATION_MM <= 0.5, "PAD_Z_COMPENSATION_MM must be -0.5..0.5 mm.");
static_assert(PAD_CLEARANCE_AFTER_MM >= 2 && PAD_CLEARANCE_AFTER_MM <= 20, "PAD_CLEARANCE_AFTER_MM must be 2..20 mm.");
static_assert(PAD_KNOWN_Z_MARGIN_MM >= 2 && PAD_KNOWN_Z_MARGIN_MM <= 50, "PAD_KNOWN_Z_MARGIN_MM must be 2..50 mm.");
static_assert(PAD_Z_SPEED_MM_S >= 0.5 && PAD_Z_SPEED_MM_S <= 8, "PAD_Z_SPEED_MM_S must be 0.5..8 mm/s.");
#ifdef PAD_HOME_NOZZLE_TEMP
  static_assert(PAD_HOME_NOZZLE_TEMP >= 50 && PAD_HOME_NOZZLE_TEMP <= 200, "PAD_HOME_NOZZLE_TEMP must be 50..200 C.");
#endif

#define PAD_HX711_TIMEOUT_MS 150    // 80 Hz => 12.5 ms per sample; no answer in this time = dead sensor
#define PAD_STUCK_COUNT        8    // this many identical raw readings in a row = stuck data line
#define PAD_FAIL_LIFT_MM       2.0f // blind lift before kill() when the nozzle may be loaded
#define PAD_RING              32    // fine pass look-back (steps)

enum PadRead : uint8_t { PAD_OK, PAD_TIMEOUT, PAD_SATURATED, PAD_STUCK };

static HX711 pad_hx;
static int32_t pad_last_raw;
static uint8_t pad_same_count;

static inline float pad_spm() { return planner.settings.axis_steps_per_mm[Z_AXIS]; }
static inline long pad_mm_to_steps(const float mm) { return _MAX(1L, lroundf(mm * pad_spm())); }

/**
 * Direct-step Z. 'up' is Marlin's logical +Z: Z_DIR_WRITE() applies INVERT_Z_DIR,
 * exactly as the stepper ISR does. The DIR pin is restored because the stepper
 * module caches the direction it last applied.
 */
static void pad_step_z(long steps, const bool up) {
  if (steps <= 0) return;
  const uint32_t half_us = _MAX(20UL, (uint32_t)(500000.0f / (pad_spm() * float(PAD_Z_SPEED_MM_S))));
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
  pad_last_raw = 0x7FFFFFFF;
}

// One validated HX711 sample. Never moves anything. Does not call idle().
static PadRead pad_read(int32_t &val) {
  const millis_t start = millis();
  while (READ(HX711_SDO_PIN)) {                       // DOUT low = data ready
    if (millis() - start > PAD_HX711_TIMEOUT_MS) return PAD_TIMEOUT;
    hal.watchdog_refresh();
  }
  const int32_t v = pad_hx.getVal(false);
  if (v >= 0x7FFFFF || v <= -0x800000) return PAD_SATURATED;
  if (v == pad_last_raw) { if (++pad_same_count >= PAD_STUCK_COUNT) return PAD_STUCK; }
  else { pad_same_count = 0; pad_last_raw = v; }
  val = v;
  return PAD_OK;
}

// Sample converted entirely after the last motion: drop the pending one first.
static PadRead pad_read_fresh(int32_t &val) {
  const PadRead r = pad_read(val);
  return r != PAD_OK ? r : pad_read(val);
}

static FSTR_P pad_read_error(const PadRead r) {
  switch (r) {
    case PAD_TIMEOUT:   return F("HX711 no data");
    case PAD_SATURATED: return F("HX711 saturated");
    default:            return F("HX711 stuck");
  }
}

// Lift (optional), report on serial + LCD, halt. Never returns.
static void pad_fail(FSTR_P const why, const long lift_steps=0) {
  if (lift_steps > 0) pad_step_z(_MIN(lift_steps, pad_mm_to_steps(PAD_FAIL_LIFT_MM)), true);
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
  // Heaters off, halt. Steppers stay energized (Marlin's default) so the gantry cannot sag onto the bed.
  marlin.kill(why, F("Pad Z homing"));
  for (;;) hal.watchdog_refresh();                     // kill() does not return; be explicit
}

static inline int32_t pad_abs(const int32_t v) { return v < 0 ? -v : v; }

struct PadStats { int32_t vmin, vmax, avg; };

// Baseline over n samples; kills on a bad sensor or when the spread is too large.
static PadStats pad_baseline(const uint8_t n, FSTR_P const noisy_msg) {
  int32_t v = 0, vmin = INT32_MAX, vmax = INT32_MIN;
  int64_t sum = 0;
  PadRead r = pad_read(v);                             // discard the pending sample
  for (uint8_t i = 0; r == PAD_OK && i < n; ++i) {
    r = pad_read(v);
    NOMORE(vmin, v); NOLESS(vmax, v); sum += v;
  }
  if (r != PAD_OK) pad_fail(pad_read_error(r));
  const PadStats s = { vmin, vmax, int32_t(sum / n) };
  if (s.vmin == s.vmax) pad_fail(F("HX711 stuck"));
  if (s.vmax - s.vmin > PAD_NOISE_MAX) {
    SERIAL_ECHOLNPGM("PAD_HOME: spread=", s.vmax - s.vmin, " limit=", PAD_NOISE_MAX);
    pad_fail(noisy_msg);
  }
  return s;
}

/**
 * One fine pass from the current position. Returns the travel (in steps) from the
 * start position to the first step of the force rise, and the nozzle is back at the
 * start position. Kills on any failure.
 */
static long pad_fine_pass(const uint8_t pass, const long step_n, const long max_steps) {
  const PadStats b = pad_baseline(16, F("sensor noisy before fine pass"));
  const int32_t knee = _MIN(int32_t(PAD_FINE_TRIGGER), _MAX(int32_t(PAD_FINE_TRIGGER) / 3, 2 * (b.vmax - b.vmin)));

  int32_t ring[PAD_RING];
  long down = 0;
  int32_t i = 0, d = 0;
  uint8_t spikes = 0;
  for (;;) {
    if (!marlin.isRunning()) pad_fail(F("stopped"), down);
    // Depth limit, checked against the current position before stepping
    if (down + step_n > max_steps) pad_fail(F("fine pass: no contact in range"), down);
    pad_step_z(step_n, false);
    down += step_n;
    ++i;

    int32_t v;
    PadRead r = pad_read_fresh(v);
    if (r != PAD_OK) pad_fail(pad_read_error(r), down);
    d = pad_abs(v - b.avg);
    if (d > PAD_FINE_TRIGGER) {
      // Re-read without moving: a real load is still there, a glitch is not
      r = pad_read(v);
      if (r != PAD_OK) pad_fail(pad_read_error(r), down);
      const int32_t d2 = pad_abs(v - b.avg);
      if (d2 > PAD_FINE_TRIGGER) { ring[i % PAD_RING] = _MIN(d, d2); break; }
      if (++spikes > 3) pad_fail(F("sensor glitches in fine pass"), down);
      d = d2;
    }
    ring[i % PAD_RING] = d;
    marlin.idle_no_sleep();
  }

  // First step of the uninterrupted rise above the knee
  int32_t k = i;
  while (k > 1 && k - 1 > i - PAD_RING && ring[(k - 1) % PAD_RING] > knee) --k;

  pad_step_z(down, true);                               // back to the start position
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

  // Refuse values that would put Z0 somewhere else than the contact point
  const float spm = pad_spm(), z_shift = float(PAD_Z_COMPENSATION_MM) + TERN0(HAS_HOME_OFFSET, motion.home_offset.z);
  if (!(spm >= 50 && spm <= 3200)) pad_fail(F("bad Z steps/mm"));
  if (!(z_shift >= -1 && z_shift <= 1)) pad_fail(F("M206 Z + compensation over 1mm"));

  // Was Z already known? Then the contact must come at about the expected distance.
  // (G28 has disabled leveling, so position.z is the plain machine height.)
  const bool z_known = motion.axis_is_trusted(Z_AXIS) && !motion.axis_should_home(Z_AXIS);
  const float z_entry = motion.position.z;

  // Until this function succeeds Z is unknown
  motion.set_axis_untrusted(Z_AXIS);

  #ifdef PAD_HOME_NOZZLE_TEMP
    const celsius_t old_target = thermalManager.degTargetHotend(0);
    if (old_target < PAD_HOME_NOZZLE_TEMP) {
      SERIAL_ECHOLNPGM("PAD_HOME: heating nozzle to ", PAD_HOME_NOZZLE_TEMP);
      thermalManager.setTargetHotend(PAD_HOME_NOZZLE_TEMP, 0);
    }
    (void)thermalManager.wait_for_hotend(0);
  #endif

  // X/Y only, at the current (unknown) height. G28 has already raised Z.
  motion.sync_plan_position();
  motion.blocking_move_xy(float(PAD_HOME_X), float(PAD_HOME_Y), 80);
  planner.synchronize();
  if (!marlin.isRunning()) pad_fail(F("stopped"));
  stepper.enable_axis(Z_AXIS);
  safe_delay(300);

  // ---- Sensor health, before any descent ----
  pad_reader_reset();
  const millis_t t0 = millis();
  const PadStats base = pad_baseline(32, F("sensor noisy or loaded"));
  const uint32_t dt = millis() - t0;
  SERIAL_ECHOLNPGM("PAD_HOME: sensor ok, baseline=", base.avg, " min=", base.vmin, " max=", base.vmax,
                   " noise=", base.vmax - base.vmin, " rate_hz=", dt ? 33000UL / dt : 0);

  // ---- Coarse descent ----
  const long coarse_n = pad_mm_to_steps(PAD_COARSE_STEP_MM);
  float max_mm = PAD_COARSE_MAX_TRAVEL_MM;
  if (z_known) NOMORE(max_mm, _MAX(z_entry, 0.0f) + float(PAD_KNOWN_Z_MARGIN_MM));
  const long max_n = pad_mm_to_steps(max_mm);
  SERIAL_ECHOLNPGM("PAD_HOME: descending, travel limit ", p_float_t(max_mm, 1), z_known ? " mm (Z was known)" : " mm (Z unknown)");
  long travelled = 0;
  float track = base.avg;                                // slow drift tracking (thermal)
  int32_t v = 0, d = 0;
  uint8_t spikes = 0;
  ui.set_status(F("Pad Z homing: descending"));
  for (;;) {
    // On failure: lift by what was descended, at most PAD_FAIL_LIFT_MM (pad_fail caps it)
    if (!marlin.isRunning()) pad_fail(F("stopped"), travelled);
    // Travel limit, checked against the current position before stepping
    if (travelled + coarse_n > max_n) pad_fail(F("no pad contact (max travel)"), travelled);
    pad_step_z(coarse_n, false);
    travelled += coarse_n;

    PadRead r = pad_read(v);
    if (r != PAD_OK) pad_fail(pad_read_error(r), travelled);
    d = pad_abs(v - int32_t(track));
    if (d > PAD_COARSE_TRIGGER || pad_abs(v - base.avg) > PAD_FORCE_LIMIT) {
      // Stop and re-read without moving: a real load is still there, a glitch is not
      r = pad_read(v);
      if (r != PAD_OK) pad_fail(pad_read_error(r), travelled);
      d = pad_abs(v - int32_t(track));
      if (d > PAD_COARSE_TRIGGER || pad_abs(v - base.avg) > PAD_FORCE_LIMIT) break;
      if (++spikes > 10) pad_fail(F("sensor glitches in descent"), travelled);
    }
    else if (d < (PAD_COARSE_TRIGGER) / 4) {
      // Follow slow drift only, and never further than one trigger level from the start
      track += float(v - int32_t(track)) * (1.0f / 32.0f);
      track = constrain(track, float(base.avg - (PAD_COARSE_TRIGGER)), float(base.avg + (PAD_COARSE_TRIGGER)));
    }

    marlin.idle_no_sleep();                             // heaters, watchdog, LCD; keep Z energized
  }

  SERIAL_ECHOLNPGM("PAD_HOME: coarse contact after ", p_float_t(float(travelled) / pad_spm(), 2),
                   " mm, delta=", d, " drift=", int32_t(track) - base.avg, " glitches=", spikes);

  // ---- Back off; the sensor must relax ----
  const long backoff_n = pad_mm_to_steps(PAD_BACKOFF_MM);
  pad_step_z(backoff_n, true);
  safe_delay(300);
  {
    const PadStats rs = pad_baseline(16, F("sensor noisy after contact"));
    SERIAL_ECHOLNPGM("PAD_HOME: backed off ", p_float_t(PAD_BACKOFF_MM, 2), " mm, residual=", rs.avg - int32_t(track));
    // Lift on failure, but never above the height where the descent started
    if (pad_abs(rs.avg - int32_t(track)) > (PAD_COARSE_TRIGGER) / 2) pad_fail(F("pad still loaded after backoff"), travelled - backoff_n);
  }

  // ---- Fine passes ----
  ui.set_status(F("Pad Z homing: measuring"));
  const long fine_n   = pad_mm_to_steps(PAD_FINE_STEP_MM),
             window_n = pad_mm_to_steps(PAD_FINE_MAX_DEPTH_MM),
             fine_max = backoff_n + window_n;
  long c[PAD_FINE_REPEATS];
  for (uint8_t i = 0; i < PAD_FINE_REPEATS; ++i) {
    c[i] = pad_fine_pass(i + 1, fine_n, fine_max);
    // Must agree with the coarse contact, else the trigger was not the pad
    if (c[i] < backoff_n - window_n) pad_fail(F("fine pass: early trigger"));
  }
  for (uint8_t i = 1; i < PAD_FINE_REPEATS; ++i)        // insertion sort
    for (uint8_t j = i; j > 0 && c[j - 1] > c[j]; --j) { const long t = c[j]; c[j] = c[j - 1]; c[j - 1] = t; }
  const float spread_mm = float(c[PAD_FINE_REPEATS - 1] - c[0]) / pad_spm(),
              above_mm  = float(c[PAD_FINE_REPEATS / 2]) / pad_spm();   // median height above contact
  SERIAL_ECHOLNPGM("PAD_HOME: median_mm=", p_float_t(above_mm, 3), " spread_mm=", p_float_t(spread_mm, 3));
  if (spread_mm > float(PAD_FINE_MAX_SPREAD_MM)) pad_fail(F("fine passes disagree"));

  // Repeatability: where the previous homing said this contact point would be
  if (z_known)
    SERIAL_ECHOLNPGM("PAD_HOME: contact moved by ",
      p_float_t(z_entry - float(travelled - backoff_n) / pad_spm() - above_mm - z_shift, 3), " mm since the previous homing");

  // ---- The nozzle is 'above_mm' over the contact point: define Z ----
  motion.position.z = above_mm + z_shift;
  motion.sync_plan_position();
  motion.set_axis_trusted(Z_AXIS);
  motion.set_axis_homed(Z_AXIS);
  TERN_(BABYSTEP_DISPLAY_TOTAL, babystep.reset_total(Z_AXIS));
  TERN_(HAS_WORKSPACE_OFFSET, motion.workspace_offset.z = 0);

  SERIAL_ECHOLNPGM("PAD_HOME: done, Z=", p_float_t(motion.position.z, 3),
                   " (compensation + M206 Z = ", p_float_t(z_shift, 3), ")");

  #ifdef PAD_HOME_NOZZLE_TEMP
    if (old_target < PAD_HOME_NOZZLE_TEMP) thermalManager.setTargetHotend(old_target, 0);
  #endif

  // Upward planner move. Z-min is only evaluated while probing, which never happens here.
  if (motion.position.z < PAD_CLEARANCE_AFTER_MM)
    motion.blocking_move_z(PAD_CLEARANCE_AFTER_MM, 5);
  ui.reset_status();
}

void pad_sensor_report(const uint8_t seconds) {
  pad_reader_reset();
  int32_t v = 0, base = 0;
  PadRead r = pad_read(v);
  if (r == PAD_OK) r = pad_read(base);
  const millis_t end = millis() + 1000UL * seconds;
  millis_t next = 0;
  SERIAL_ECHOLNPGM("PAD_TEST: no motion. Press the bed at the front-left corner. baseline=", base);
  while (r == PAD_OK && PENDING(millis(), end)) {
    r = pad_read(v);
    if (r == PAD_OK && ELAPSED(millis(), next)) {
      next = millis() + 200;
      SERIAL_ECHOLNPGM("PAD_TEST: raw=", v, " delta=", v - base);
    }
    marlin.idle();
  }
  if (r != PAD_OK) SERIAL_ECHOLN(F("PAD_TEST: FAILED: "), pad_read_error(r));
  else SERIAL_ECHOLNPGM("PAD_TEST: end (trigger=", PAD_FINE_TRIGGER, "/", PAD_COARSE_TRIGGER, " limit=", PAD_FORCE_LIMIT, " noise max=", PAD_NOISE_MAX, ")");
}

#endif // PAD_Z_HOMING
