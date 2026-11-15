/**
 * E3V3SE manual ("hand-turn") Z homing. Fork-owned, enabled by MANUAL_Z_HOMING.
 *
 * G28 homes X/Y normally. Instead of probing Z, the nozzle is heated and parked over the bed,
 * the Z driver alone is switched off (TMC UART, TOFF=0) and the user lowers the nozzle onto
 * the bed by turning the Z coupler by hand, then clicks the knob. The probe is never used.
 */
#pragma once

#include "../inc/MarlinConfigPre.h"

#if ENABLED(MANUAL_Z_HOMING)

class ManualZHome {
public:
  // Set when a manual Z home failed. G-code moves are refused until Z is homed again.
  static bool locked;

  // Start heating the nozzle for the touch-off without waiting. Call before homing X/Y.
  static void preheat();

  // Run the touch-off. Returns true if Z is now homed. On any failure the nozzle
  // heater is switched off, Z stays unhomed and 'locked' is set.
  static bool home(const bool move_xy);

private:
  static int16_t saved_target;  // Nozzle target before G28, or -1 if preheat() has not run
};

extern ManualZHome manual_z;

// Implemented by the Creality DWIN UI (lcd/dwin/creality/dwin.cpp)
enum ManualZScreen : uint8_t { MZS_HEATING, MZS_TOUCH, MZS_LIFT, MZS_FAILED };
enum ManualZReply  : uint8_t { MZR_NONE, MZR_CONFIRM, MZR_CANCEL };
void DWIN_ManualZ_Show(const ManualZScreen screen, FSTR_P const reason=nullptr);
ManualZReply DWIN_ManualZ_Reply();  // MZR_CANCEL as well if another screen took over the display
void DWIN_ManualZ_End(const bool ok);

#endif // MANUAL_Z_HOMING
