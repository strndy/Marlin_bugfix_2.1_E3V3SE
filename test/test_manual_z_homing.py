#!/usr/bin/env python3
"""Run the real ManualZHome.cpp state machine on the host against stubbed hardware.

Checks the properties that matter when nothing can be tried on a printer first:
Z is only marked homed after a knob confirmation, the nozzle heater is off after
every failure, and the Z driver is never left switched off.
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

CONFIG = r'''
#pragma once
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>
#include <functional>
#include <stdexcept>
#include "Marlin/src/core/macros.h"

#define MANUAL_Z_HOMING
#define MANUAL_Z_HOME_TEMP        220
#define X_CENTER 110
#define Y_CENTER 110
#define DWIN_CREALITY_LCD
#define HOST_ACTION_COMMANDS
#define ACTION_ON_CANCEL "cancel"
#define HAS_HOTEND 1
#define HAS_EXTRUDERS 1
#define HAS_HOME_OFFSET 1
#define HAS_WORKSPACE_OFFSET 1
#define Z_HOME_TO_MIN 1
#define AXIS_HAS_UART(A) 1
#define HEATER_0_MAXTEMP 310
#define HAS_HEATED_BED 1
#define MANUAL_Z_HOME_BED_TEMP 50
#define BED_MAXTEMP 125

using millis_t = uint32_t;
using FSTR_P = const char*;
using feedRate_t = float;
#define F(S) S
#define SEC_TO_MS(N) millis_t((N) * 1000UL)
#define MMM_TO_MMS(N) ((N) / 60.0f)
#define PENDING(NOW,SOON) (int32_t((NOW) - (SOON)) < 0)
#define ELAPSED(NOW,SOON) (!PENDING(NOW,SOON))
#define KEEPALIVE_STATE(N) do{}while(0)
enum AxisEnum : uint8_t { X_AXIS, Y_AXIS, Z_AXIS, E_AXIS };

extern std::vector<std::string> serial_log;
inline std::string to_text(const char *s) { return s; }
inline std::string to_text(double v) { return std::to_string(v); }
template<typename... A> void serial_line(A... a) { std::string s; ((s += to_text(a)), ...); serial_log.push_back(s); }
#define SERIAL_ECHOLNPGM(...) serial_line(__VA_ARGS__)
#define SERIAL_ECHOLN(...)    serial_line(__VA_ARGS__)
#define SERIAL_ERROR_MSG(...) serial_line("Error:", __VA_ARGS__)

extern millis_t now_ms;
inline millis_t millis() { return now_ms; }
inline void safe_delay(const millis_t ms) { now_ms += ms; }

struct xy_pos_t { float x, y; };
struct Sim {
  // Printer state
  int target = 0, temp = 25;
  int bed_target = 0, bed_temp = 22;
  int touch_target = -1, touch_bed = -1;  // Targets seen when Z was released
  bool xy_homed = true, z_homed = false, z_trusted = false, heating_flag = true;
  bool endstops_on = true, sd_printing = false, abort_sd = false, job = false;
  uint32_t chopconf = 0x10000053;    // Driver register as configured at boot (TOFF=3)
  bool uart_ok = true, uart_corrupt_write = false;
  uint8_t toff() const { return chopconf & 0x0F; }
  float real_z = 12;                 // True nozzle height above the bed
  float lowest_powered_move = 1e9f;  // Lowest logical Z target commanded while Z is unknown
  // Observations
  int toff_zero_writes = 0, e_moves = 0, host_cancels = 0, ui_end = -1;
  millis_t hands_off_from = 0;       // When the "hands off" screen was shown
  bool lift_warned = false;          // A powered Z move after the prompt came >= 2 s after the warning
  std::vector<int> screens;
  bool killed = false, moved_xy = false, moved_with_driver_off = false;
  // Script
  int reply = 0;
  std::function<void(Sim&)> on_idle;
};
extern Sim sim;

struct StepperZ {
  bool CRCerror = false;
  uint32_t CHOPCONF() { CRCerror = !sim.uart_ok; return sim.uart_ok ? sim.chopconf : 0; }
  void CHOPCONF(const uint32_t v) {
    if (!(v & 0x0F)) ++sim.toff_zero_writes;
    sim.chopconf = sim.uart_corrupt_write ? (v ^ 0x00010000u) : v;  // e.g. a flipped MRES bit
  }
};
extern StepperZ stepperZ;

struct Marlin {
  void idle(bool no_sleep) { if (!no_sleep) throw std::logic_error("idle() may release steppers"); now_ms += 100; if (sim.on_idle) sim.on_idle(sim); }
  void heatup_start() { sim.heating_flag = true; }
  void heatup_done() { sim.heating_flag = false; }
  bool is_heating() { return sim.heating_flag; }
  bool printJobOngoing() { return sim.job; }
  void kill(FSTR_P) { sim.killed = true; sim.target = 0; throw std::runtime_error("kill"); }
};
extern Marlin marlin;

struct Thermal {
  int degTargetHotend(uint8_t) { return sim.target; }
  void setTargetHotend(int c, uint8_t) { sim.target = c; }
  int wholeDegHotend(uint8_t) { return sim.temp; }
  int degTargetBed() { return sim.bed_target; }
  void setTargetBed(int c) { sim.bed_target = c; }
  int wholeDegBed() { return sim.bed_temp; }
  bool tooColdToExtrude(uint8_t) { return sim.temp < 170; }
};
extern Thermal thermalManager;

struct Motion {
  struct { float x = 0, y = 0, z = 10, e = 7; } position;
  struct { float z = 0; } home_offset, workspace_offset;
  void sync_plan_position() {}
  void sync_plan_position_e() {}
  bool homing_needed_error(uint8_t) { return !sim.xy_homed; }
  feedRate_t homing_feedrate(AxisEnum) { return 10; }
  void blocking_move(const xy_pos_t &p, feedRate_t) { position.x = p.x; position.y = p.y; sim.moved_xy = true; }
  void blocking_move_z(float z, feedRate_t) {
    if (sim.toff() == 0) sim.moved_with_driver_off = true;
    sim.lift_warned = sim.hands_off_from && now_ms - sim.hands_off_from >= 2000;
    if (!sim.z_homed && z < sim.lowest_powered_move) sim.lowest_powered_move = z;
    sim.real_z += z - position.z; position.z = z;
  }
  void goto_current_position(feedRate_t) { ++sim.e_moves; }
  void set_axis_never_homed(AxisEnum) { sim.z_homed = sim.z_trusted = false; }
  void set_axis_trusted(AxisEnum) { sim.z_trusted = true; }
  void set_axis_homed(AxisEnum) { sim.z_homed = true; }
};
extern Motion motion;

struct Planner { void synchronize() {} };
extern Planner planner;
struct Endstops { void enable(bool on) { sim.endstops_on = on; } };
extern Endstops endstops;
struct Card { struct { bool abort_sd_printing; } flag; bool isPrinting() { flag.abort_sd_printing = sim.abort_sd; return sim.sd_printing; } };
extern Card card;
struct HostUI { void cancel() { ++sim.host_cancels; } };
extern HostUI hostui;
'''

MAIN = r'''
#include "src/inc/MarlinConfig.h"
#include "src/e3v3se/ManualZHome.h"
#include <cassert>
#include <cstdio>

std::vector<std::string> serial_log;
millis_t now_ms = 4294900000u;       // The 32-bit millis() wrap happens during the test on purpose
Sim sim; StepperZ stepperZ; Marlin marlin; Thermal thermalManager; Motion motion;
Planner planner; Endstops endstops; Card card; HostUI hostui;

void DWIN_ManualZ_Show(const ManualZScreen s, FSTR_P) { sim.screens.push_back(s); if (s == MZS_LIFT) sim.hands_off_from = now_ms; }
ManualZReply DWIN_ManualZ_Reply() { const int r = sim.reply; sim.reply = 0; return ManualZReply(r); }
void DWIN_ManualZ_End(const bool ok) { sim.ui_end = ok; }

static void reset(const int prior_target, const int prior_bed=0) {
  sim = Sim(); motion = Motion(); serial_log.clear(); stepperZ.CRCerror = false;
  sim.target = prior_target; sim.bed_target = prior_bed; manual_z.locked = false;
}

// The nozzle heats 5 C per idle tick; the user's hand lowers Z while the motor is off
static void physics(Sim &s) {
  card.flag.abort_sd_printing = s.abort_sd;
  if (s.target > s.temp) s.temp += 5;
  if (s.bed_target > s.bed_temp) s.bed_temp += 1;   // The bed is slower
  if (s.toff() == 0 && s.touch_target < 0) { s.touch_target = s.target; s.touch_bed = s.bed_target; }
}

static bool run() {
  manual_z.preheat();
  try { return manual_z.home(true); } catch (const std::runtime_error&) { return false; }
}

static void check_failed(const char *name) {
  if (!(sim.target == 0 && sim.bed_target == 0 && !sim.z_homed && !sim.z_trusted && manual_z.locked && sim.chopconf == 0x10000053u && !sim.moved_with_driver_off
         && (sim.toff_zero_writes == 0 || sim.lift_warned))) {
    printf("FAIL %s: target=%d homed=%d locked=%d chopconf=%x\n", name, sim.target, sim.z_homed, manual_z.locked, unsigned(sim.chopconf));
    for (auto &l : serial_log) printf("  %s\n", l.c_str());
    assert(false);
  }
}

int main() {
  // 1. Normal run, nozzle was at 200 before G28
  reset(200);
  sim.on_idle = [](Sim &s) {
    physics(s);
    static int ticks = 0;
    if (s.toff() == 0) { s.real_z = 0; if (++ticks == 50) s.reply = MZR_CONFIRM; }
  };
  if (!run()) { for (auto &l : serial_log) printf("  %s\n", l.c_str()); assert(false); }
  assert(sim.z_homed && sim.z_trusted && !manual_z.locked);
  assert(sim.chopconf == 0x10000053u && sim.toff_zero_writes == 1 && !sim.moved_with_driver_off);
  assert(sim.lift_warned);                             // Hands-off screen, then >= 2 s, then the lift
  assert(motion.position.z == 5 && sim.real_z == 5);   // Logical Z matches the real height
  assert(sim.target == 200);                           // Previous target restored
  assert(sim.touch_target == 200 && sim.touch_bed == 50); // Nozzle print temp kept, bed unset -> default
  assert(sim.bed_target == 0 && sim.bed_temp >= 48);    // Bed waited for, then back off
  assert(sim.e_moves == 1 && motion.position.e == 7);  // Retracted, logical E unchanged
  assert(sim.moved_xy && !sim.endstops_on && sim.ui_end == 1);
  assert(sim.screens == (std::vector<int>{ MZS_HEATING, MZS_TOUCH, MZS_LIFT }));
  assert(motion.position.e == 7 && !serial_log.empty());

  // 2. Nozzle was off before G28: it is off again afterwards
  reset(0);
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff() == 0) { s.real_z = 0; s.reply = MZR_CONFIRM; } };
  assert(run() && sim.target == 0 && sim.z_homed);
  assert(sim.touch_target == 220 && sim.touch_bed == 50 && sim.bed_target == 0);  // Defaults, then off again

  // 2b. Print start already set nozzle 240 / bed 70: touch off at those, restore them
  reset(240, 70);
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff() == 0) { s.real_z = 0; s.reply = MZR_CONFIRM; } };
  assert(run() && sim.z_homed);
  assert(sim.touch_target == 240 && sim.touch_bed == 70 && sim.target == 240 && sim.bed_target == 70);

  // 2c. Below the thresholds (nozzle 40, bed 25) counts as unset
  reset(40, 25);
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff() == 0) { s.real_z = 0; s.reply = MZR_CONFIRM; } };
  assert(run() && sim.touch_target == 220 && sim.touch_bed == 50 && sim.target == 40 && sim.bed_target == 25);

  // 3. Nobody clicks: timeout with the nozzle resting on the bed
  reset(210);
  sim.sd_printing = true;
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff() == 0) s.real_z = 0; };
  const millis_t t0 = now_ms;
  assert(!run());
  check_failed("timeout");
  assert(now_ms - t0 > 300000u && now_ms - t0 < 400000u);
  assert(sim.real_z == 5 && sim.ui_end == 0 && sim.host_cancels == 0);
  assert(sim.screens.back() == MZS_FAILED);

  // 4. Cancelled at the knob while heating: the driver is never touched
  reset(0);
  sim.on_idle = [](Sim &s) { physics(s); if (s.temp > 100) s.reply = MZR_CANCEL; };
  assert(!run());
  check_failed("cancel while heating");
  assert(sim.toff_zero_writes == 0 && sim.real_z == 12 && sim.host_cancels == 1);

  // 5. Cancel selected on the touch screen, with Z only partly lowered
  reset(0);
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff() == 0) { s.real_z = 3; s.reply = MZR_CANCEL; } };
  assert(!run());
  check_failed("cancel at touch");
  assert(sim.real_z >= motion.position.z);             // Real nozzle is never below the logical Z

  // 6. M108 from a host during the touch-off, host print job running
  reset(0);
  sim.job = true;
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff() == 0) s.heating_flag = false; };
  assert(!run());
  check_failed("M108");
  assert(sim.host_cancels == 1 && serial_log.back().rfind("Error:", 0) == 0);

  // 7. SD abort (M524 / card removed) during heating
  reset(0);
  sim.sd_printing = true;
  sim.on_idle = [](Sim &s) { physics(s); if (s.temp > 100) s.abort_sd = true; };
  assert(!run());
  check_failed("SD abort");

  // 8. Heater never gets there: timeout while heating
  reset(0);
  sim.on_idle = [](Sim &s) { if (s.temp < 150) s.temp += 5; };
  assert(!run());
  check_failed("heating timeout");
  assert(sim.toff_zero_writes == 0);

  // 9. Z driver UART dead from the start: fail without writing, without halting
  reset(0);
  sim.uart_ok = false;
  sim.on_idle = physics;
  assert(!run());
  check_failed("UART dead");
  assert(sim.toff_zero_writes == 0 && !sim.killed);

  // 10. UART dies while Z is released: re-enable can't be confirmed -> halt
  reset(0);
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff() == 0) { s.uart_ok = false; s.reply = MZR_CONFIRM; } };
  assert(!run());
  assert(sim.killed && sim.target == 0 && !sim.z_homed && manual_z.locked && !sim.moved_with_driver_off);
  assert(sim.chopconf == 0x10000053u);                 // The enable write itself was still sent

  // 10b. The re-enable write lands but reads back different (other CHOPCONF bits changed) -> halt
  reset(0);
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff() == 0) { s.uart_corrupt_write = true; s.reply = MZR_CONFIRM; } };
  assert(!run());
  assert(sim.killed && !sim.z_homed && manual_z.locked && !sim.moved_with_driver_off);

  // 10c. Driver answers but TOFF is already 0 (driver off): fail without writing
  reset(0);
  sim.chopconf = 0x10000050u;
  sim.on_idle = physics;
  assert(!run());
  assert(sim.toff_zero_writes == 0 && !sim.killed && sim.target == 0 && manual_z.locked && !sim.z_homed);

  // 10d. A previous G28 left the heat-up flag set? No: every exit clears it (M155 reports keep working)
  assert(!marlin.is_heating());

  // 11. X/Y not homed
  reset(0);
  sim.xy_homed = false;
  sim.on_idle = physics;
  assert(!run());
  check_failed("XY not homed");
  assert(!sim.moved_xy && sim.toff_zero_writes == 0);

  // 12. A later successful home clears the lock
  sim.xy_homed = true;
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff() == 0) { s.real_z = 0; s.reply = MZR_CONFIRM; } };
  assert(run() && !manual_z.locked && sim.z_homed && !marlin.is_heating());

  puts("PASS manual Z homing: confirm, restore, hands-off delay, timeout, cancel, M108, SD abort, UART faults, lock");
  return 0;
}
'''

HEADERS = ('inc/MarlinConfig.h', 'inc/MarlinConfigPre.h', 'MarlinCore.h', 'gcode/gcode.h', 'module/motion.h',
           'module/planner.h', 'module/endstops.h', 'module/temperature.h', 'module/stepper/indirection.h',
           'sd/cardreader.h', 'feature/host_actions.h')


PROBE_FREE = r'''
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <vector>
#include "Marlin/src/core/macros.h"
#define MANUAL_Z_HOMING
#define EEPROM_SETTINGS
#define IIC_BL24CXX_EEPROM
#define USE_AUTOZ_TOOL_2
#define USER_LEVEL_CHECK 1
#define PSTR(S) S
#define DWIN_BOOT_STEP_EEPROM_ADDRESS 1
#define DWIN_LANGUAGE_EEPROM_ADDRESS 2

// --- the real USER_LEVEL_CHECK override from dwin.h
@LEVEL_CHECK@
#if ENABLED(USER_LEVEL_CHECK)
  #error "Level calibration must be compiled out with MANUAL_Z_HOMING"
#endif

std::vector<std::string> lines;
#define SERIAL_ECHOLNPGM(S) lines.push_back(S)
// Anything that would move the printer or write the wizard state ends the test
[[noreturn]] static void forbidden(const char *what) { printf("FAIL reached %s\n", what); abort(); }

enum DC_Boot_Step { Set_language, Set_high, Set_levelling, Boot_Step_Max };
enum { Chinese = 2, English = 4, Language_Max = 12 };
enum { MainMenu, ONE_HIGH, POPUP_CONFIRM };
enum { Nozz_Start };
enum ENCODER_DiffState { ENCODER_DIFF_NO, ENCODER_DIFF_CW, ENCODER_DIFF_CCW, ENCODER_DIFF_ENTER };
struct { uint8_t boot_step, language; bool Need_boot_flag; } HMI_flag;
uint8_t checkkey = POPUP_CONFIRM, stored_step, stored_language;
int start_frames, main_menus;
struct BL24CXX { static void read(int address, uint8_t *v, size_t) { *v = address == DWIN_BOOT_STEP_EEPROM_ADDRESS ? stored_step : stored_language; } };
void HMI_StartFrame(bool) { ++start_frames; }
void Goto_MainMenu() { ++main_menus; checkkey = MainMenu; }
void Save_Boot_Step_Value() { forbidden("Save_Boot_Step_Value"); }
void Popup_Window_Height(int) { forbidden("the auto Z-offset page"); }
struct { void inject_P(const char *c) { forbidden(c); } } queue;
struct { void set(int) {} } select_page;
ENCODER_DiffState get_encoder_state() { return ENCODER_DIFF_ENTER; }
struct GcodeSuite { void M8015(); };

@FUNCTIONS@

int main() {
  // M8015 compiles with no probe, motion or settings code at all and only reports
  GcodeSuite().M8015();
  assert(lines.size() == 1 && lines[0] == "M8015 is not available with MANUAL_Z_HOMING");

  // Fresh, half-finished and finished wizard states all boot straight to the main menu
  for (int step : { 0xFF, int(Set_language), int(Set_high), int(Set_levelling), int(Boot_Step_Max) })
    for (int language : { 0xFF, 0, int(English), 7 }) {
      stored_step = step; stored_language = language; start_frames = 0;
      HMI_flag.Need_boot_flag = true;
      Read_Boot_Step_Value();
      assert(!HMI_flag.Need_boot_flag && start_frames == 1);
      assert(HMI_flag.language >= Chinese && HMI_flag.language < Language_Max);
      assert(HMI_flag.language == (language == 7 ? 7 : int(English)));

      // A leftover confirmation popup can only lead back to the main menu
      checkkey = POPUP_CONFIRM; main_menus = 0;
      HMI_Confirm();
      assert(main_menus == 1 && checkkey == MainMenu);
    }
  puts("PASS probe-free with MANUAL_Z_HOMING: M8015 inert, wizard skipped, level calibration compiled out");
  return 0;
}
'''


def function(source, prefix):
    """Return one function, with its preprocessor lines, by matching braces."""
    source = re.sub(r'/\*.*?\*/|//[^\n]*', '', source, flags=re.S)
    start = source.index(prefix)
    end, depth = source.index('{', start) + 1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'


def probe_free(tmp):
    dwin = (ROOT/'Marlin/src/lcd/dwin/creality/dwin.cpp').read_text()
    header = (ROOT/'Marlin/src/lcd/dwin/creality/dwin.h').read_text()
    first = header.index('#if ENABLED(MANUAL_Z_HOMING)')
    level_check = header[first:header.index('#endif', first) + 6]
    assert '#undef USER_LEVEL_CHECK' in level_check and first < header.index('enum processID')
    functions = function((ROOT/'Marlin/src/e3v3se/M8015.cpp').read_text(), 'void GcodeSuite::M8015(')
    functions += function(dwin, 'static void Read_Boot_Step_Value(') + function(dwin, 'void HMI_Confirm(')
    (tmp/'probe_free.cpp').write_text(PROBE_FREE.replace('@LEVEL_CHECK@', level_check).replace('@FUNCTIONS@', functions))
    exe = tmp/'probe_free'
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wno-unused-function', '-I', str(ROOT), str(tmp/'probe_free.cpp'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


def main():
    with tempfile.TemporaryDirectory(prefix='manual-z-') as folder:
        tmp = Path(folder)
        probe_free(tmp)
        for name in HEADERS:
            path = tmp/'src'/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#include "sim.h"\n')
        (tmp/'sim.h').write_text(CONFIG)
        (tmp/'src/e3v3se').mkdir()
        for name in ('ManualZHome.cpp', 'ManualZHome.h'):
            (tmp/'src/e3v3se'/name).write_bytes((ROOT/'Marlin/src/e3v3se'/name).read_bytes())
        (tmp/'main.cpp').write_text(MAIN)
        exe = tmp/'manual_z'
        subprocess.run(['g++', '-std=c++17', '-Wall', '-I', str(ROOT), '-I', str(tmp), str(tmp/'main.cpp'),
                        str(tmp/'src/e3v3se/ManualZHome.cpp'), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    main()
