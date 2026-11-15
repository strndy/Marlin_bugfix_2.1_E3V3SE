#!/usr/bin/env python3
"""Run the real ManualZHome.cpp state machine on the host against stubbed hardware.

Checks the properties that matter when nothing can be tried on a printer first:
Z is only marked homed after a knob confirmation, the nozzle heater is off after
every failure, and the Z driver is never left switched off.
"""
from pathlib import Path
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
#define MANUAL_Z_HOME_X_POINT     110
#define MANUAL_Z_HOME_Y_POINT     110
#define MANUAL_Z_HOME_RETRACT     1.0
#define MANUAL_Z_HOME_OFFSET      0.0
#define MANUAL_Z_HOME_CLEARANCE   5
#define MANUAL_Z_HOME_TIMEOUT_SEC 600
#define MANUAL_Z_HOME_VERIFY_UART
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
#define X_MIN_POS 0
#define X_MAX_POS 220
#define Y_MIN_POS 0
#define Y_MAX_POS 220

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
  bool xy_homed = true, z_homed = false, z_trusted = false, heating_flag = true;
  bool endstops_on = true, sd_printing = false, abort_sd = false, job = false;
  uint8_t toff = 3;
  bool uart_ok = true;
  float real_z = 12;                 // True nozzle height above the bed
  float lowest_powered_move = 1e9f;  // Lowest logical Z target commanded while Z is unknown
  // Observations
  int toff_zero_writes = 0, e_moves = 0, host_cancels = 0, ui_end = -1;
  std::vector<int> screens;
  bool killed = false, moved_xy = false, moved_with_driver_off = false;
  // Script
  int reply = 0;
  std::function<void(Sim&)> on_idle;
};
extern Sim sim;

struct StepperZ {
  bool CRCerror = false;
  uint32_t CHOPCONF() { CRCerror = !sim.uart_ok; return sim.uart_ok ? (0x10000050u | sim.toff) : 0; }
  void toff(const uint8_t v) { if (v == 0) ++sim.toff_zero_writes; sim.toff = v; }
};
extern StepperZ stepperZ;
struct chopper_timing_t { uint8_t toff; int8_t hend; uint8_t hstrt; };
static constexpr chopper_timing_t chopper_timing_Z = { 3, -1, 1 };

struct Marlin {
  void idle(bool no_sleep) { if (!no_sleep) throw std::logic_error("idle() may release steppers"); now_ms += 100; if (sim.on_idle) sim.on_idle(sim); }
  void heatup_start() { sim.heating_flag = true; }
  bool is_heating() { return sim.heating_flag; }
  bool printJobOngoing() { return sim.job; }
  void kill(FSTR_P) { sim.killed = true; sim.target = 0; throw std::runtime_error("kill"); }
};
extern Marlin marlin;

struct Thermal {
  int degTargetHotend(uint8_t) { return sim.target; }
  void setTargetHotend(int c, uint8_t) { sim.target = c; }
  int wholeDegHotend(uint8_t) { return sim.temp; }
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
    if (sim.toff == 0) sim.moved_with_driver_off = true;
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

void DWIN_ManualZ_Show(const ManualZScreen s, FSTR_P) { sim.screens.push_back(s); }
ManualZReply DWIN_ManualZ_Reply() { const int r = sim.reply; sim.reply = 0; return ManualZReply(r); }
void DWIN_ManualZ_End(const bool ok) { sim.ui_end = ok; }

static void reset(const int prior_target) {
  sim = Sim(); motion = Motion(); serial_log.clear(); stepperZ.CRCerror = false;
  sim.target = prior_target; manual_z.locked = false;
}

// The nozzle heats 5 C per idle tick; the user's hand lowers Z while the motor is off
static void physics(Sim &s) {
  card.flag.abort_sd_printing = s.abort_sd;
  if (s.target > s.temp) s.temp += 5;
}

static bool run() {
  manual_z.preheat();
  try { return manual_z.home(true); } catch (const std::runtime_error&) { return false; }
}

static void check_failed(const char *name) {
  if (!(sim.target == 0 && !sim.z_homed && !sim.z_trusted && manual_z.locked && sim.toff == 3 && !sim.moved_with_driver_off)) {
    printf("FAIL %s: target=%d homed=%d locked=%d toff=%d\n", name, sim.target, sim.z_homed, manual_z.locked, sim.toff);
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
    if (s.toff == 0) { s.real_z = 0; if (++ticks == 50) s.reply = MZR_CONFIRM; }
  };
  if (!run()) { for (auto &l : serial_log) printf("  %s\n", l.c_str()); assert(false); }
  assert(sim.z_homed && sim.z_trusted && !manual_z.locked);
  assert(sim.toff == 3 && sim.toff_zero_writes == 1 && !sim.moved_with_driver_off);
  assert(motion.position.z == 5 && sim.real_z == 5);   // Logical Z matches the real height
  assert(sim.target == 200);                           // Previous target restored
  assert(sim.e_moves == 1 && motion.position.e == 7);  // Retracted, logical E unchanged
  assert(sim.moved_xy && !sim.endstops_on && sim.ui_end == 1);
  assert(sim.screens == (std::vector<int>{ MZS_HEATING, MZS_TOUCH }));

  // 2. Nozzle was off before G28: it is off again afterwards
  reset(0);
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff == 0) { s.real_z = 0; s.reply = MZR_CONFIRM; } };
  assert(run() && sim.target == 0 && sim.z_homed);

  // 3. Nobody clicks: timeout with the nozzle resting on the bed
  reset(210);
  sim.sd_printing = true;
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff == 0) s.real_z = 0; };
  const millis_t t0 = now_ms;
  assert(!run());
  check_failed("timeout");
  assert(now_ms - t0 > 600000u && now_ms - t0 < 700000u);
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
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff == 0) { s.real_z = 3; s.reply = MZR_CANCEL; } };
  assert(!run());
  check_failed("cancel at touch");
  assert(sim.real_z >= motion.position.z);             // Real nozzle is never below the logical Z

  // 6. M108 from a host during the touch-off, host print job running
  reset(0);
  sim.job = true;
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff == 0) s.heating_flag = false; };
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
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff == 0) { s.uart_ok = false; s.reply = MZR_CONFIRM; } };
  assert(!run());
  assert(sim.killed && sim.target == 0 && !sim.z_homed && manual_z.locked && !sim.moved_with_driver_off);
  assert(sim.toff == 3);                               // The enable write itself was still sent

  // 11. X/Y not homed
  reset(0);
  sim.xy_homed = false;
  sim.on_idle = physics;
  assert(!run());
  check_failed("XY not homed");
  assert(!sim.moved_xy && sim.toff_zero_writes == 0);

  // 12. A later successful home clears the lock
  sim.xy_homed = true;
  sim.on_idle = [](Sim &s) { physics(s); if (s.toff == 0) { s.real_z = 0; s.reply = MZR_CONFIRM; } };
  assert(run() && !manual_z.locked && sim.z_homed);

  puts("PASS manual Z homing: confirm, restore, timeout, cancel, M108, SD abort, UART faults, lock");
  return 0;
}
'''

HEADERS = ('inc/MarlinConfig.h', 'inc/MarlinConfigPre.h', 'MarlinCore.h', 'gcode/gcode.h', 'module/motion.h',
           'module/planner.h', 'module/endstops.h', 'module/temperature.h', 'module/stepper/indirection.h',
           'sd/cardreader.h', 'feature/host_actions.h')


def main():
    with tempfile.TemporaryDirectory(prefix='manual-z-') as folder:
        tmp = Path(folder)
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
