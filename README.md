# Marlin for the Creality Ender-3 V3 SE

An upstream-first Marlin 2.1.x bugfix fork for the **Creality Ender-3 V3 SE**.

The goal of this repository is simple: **pull upstream Marlin at any time without regressions**. All fork-specific behavior is isolated behind a small, auditable boundary, and the machine configuration is generated from pristine upstream files plus a reviewed profile.

> Status: the four supported variants (C13 and C14, each with SD card and OctoPrint) live in their own branches. The C13 profiles are verified on hardware; the C14 profiles build for STM32F401RE and are awaiting printer validation.

## Table of Contents

- [Why this architecture](#why-this-architecture)
- [Branches](#branches)
- [Project structure](#project-structure)
- [Configuration profiles](#configuration-profiles)
- [Enabling and disabling features by hand](#enabling-and-disabling-features-by-hand)
- [Pad Z homing (experimental)](#pad-z-homing-experimental)
- [Input shaping with FT_Motion (`M493`)](#input-shaping-with-ft_motion-m493)
- [Linear Advance: legacy vs FT_Motion](#linear-advance-legacy-vs-ft_motion)
- [Guardrails](#guardrails)
- [Merging upstream](#merging-upstream)
- [Host (OctoPrint) integration](#host-octoprint-integration)
- [Building](#building)
  - [Building with GitHub Actions](#building-with-github-actions)
- [Memory budget](#memory-budget)
- [Further documentation](#further-documentation)
- [Marlin upstream README](#marlin-3d-printer-firmware)

## Why this architecture

Previous versions of this fork edited upstream files directly (the DWIN UI, headers, `Configuration*.h`). Every upstream merge then produced conflicts and silent regressions. The current design follows three rules:

1. **Upstream owns the core.** Files outside the fork-owned paths stay as close to upstream as possible. Where a hook is unavoidable, it is a few lines that call into fork code.
2. **Fork code lives in fork-owned paths.** Custom features are implemented in `Marlin/src/e3v3se/` and `Marlin/src/lcd/dwin/creality/` and are compiled through the PlatformIO source filter.
3. **Configuration is derived, not hand-merged.** `Configuration.h` and `Configuration_adv.h` equal pristine upstream plus a profile (define overrides and small residual patches).

### How the pieces fit together

```mermaid
flowchart TB
    subgraph UP["Upstream Marlin (bugfix-2.1.x) - merged as-is"]
        UCFG["Configuration.h / Configuration_adv.h<br/>(pristine)"]
        UCORE["Core: gcode, planner, stepper,<br/>temperature, pause, settings"]
        UHAL["HAL / pins / sanity checks"]
    end

    subgraph PROF["Config profile (buildroot/share/e3v3se/profiles)"]
        OVR["overrides.txt<br/>define states"]
        RES["residual.*.patch<br/>fork-only options"]
    end

    subgraph FORK["Fork-owned code"]
        E3["Marlin/src/e3v3se/<br/>AutoOffset, ThumbRender, OctoHost"]
        DWIN["Marlin/src/lcd/dwin/creality/<br/>DWIN UI for the V3 SE"]
    end

    subgraph GUARD["Guardrails (check_profile.py)"]
        GB["boundary: upstream-file edits must not grow"]
        GC["config: files = upstream + profile"]
        GS["build: RAM / flash baseline"]
    end

    UCFG --> APPLY(["config_profile.py apply / derive"])
    OVR --> APPLY
    RES --> APPLY
    APPLY --> CFG["Machine Configuration*.h"]

    CFG --> BUILD(["PlatformIO build<br/>source filter in platformio.ini"])
    UCORE -- "few-line hooks" --> E3
    UCORE -- "hooks" --> DWIN
    UCORE --> BUILD
    UHAL --> BUILD
    E3 --> BUILD
    DWIN --> BUILD
    BUILD --> FW["Firmware (C13 SD / C13 OctoPrint)"]

    GB -. checks .-> UCORE
    GC -. checks .-> CFG
    GS -. checks .-> FW
```

Upstream code flows in untouched, the machine configuration is generated from it, and all fork behavior enters through a small number of hooks into fork-owned folders. The guardrails verify each of those three boundaries on every change.

## Branches

| Branch | Hardware | Purpose |
| --- | --- | --- |
| `c13_sd` | C13 (STM32F103RE), SD card | **This branch.** Main line and default branch. Upstream-first architecture with the C13 SD profile. |
| `c13_octoprint` | C13 (STM32F103RE), serial host (OctoPrint) | `c13_sd` plus the host layer, MeatPack and the OctoPrint plugin protocol; `ONE_CLICK_PRINT` is off. |
| `c14_sd` | C14 (STM32F401RE), SD card | C14 SD profile; uses `BOARD_CREALITY_F401RE` and the `STM32F401RE_creality` environment. |
| `c14_octoprint` | C14 (STM32F401RE), serial host (OctoPrint) | `c14_sd` plus the host layer, MeatPack and the OctoPrint plugin protocol; `ONE_CLICK_PRINT` is off. |
| `legacy/for_E3V3SE`, `legacy/for_Octoprint` | C13 | Pre-architecture branches, kept for reference only. |
| `legacy/for_STM32F401`, `legacy/for_Octoprint_F401` | C14 (STM32F401) | Pre-architecture branches, kept for reference only. |

`c13_sd` is the default branch. The `legacy/` branches are frozen and no longer receive upstream merges.

Each branch carries its own copy of the documentation, so notes follow the code they describe.

## Project structure

```
Marlin/
  Configuration.h, Configuration_adv.h   Derived from upstream + active profile
  src/
    e3v3se/                              Fork-owned features (isolated from upstream)
      AutoOffset.*, G212.cpp, M8015.cpp    Automatic Z-offset (load cell / CRTouch)
      ThumbRender.cpp                      G-code thumbnail renderer for the DWIN display
      OctoHost.*                           OctoPrint plugin layer (c13_octoprint)
    lcd/dwin/creality/                   Fork-owned DWIN UI for the V3 SE
    ...                                  Everything else follows upstream Marlin
buildroot/share/e3v3se/
  config_profile.py                      Derive / apply / check / stale for profiles
  check_profile.py                       Guardrails (boundary, config, build)
  baseline.json                          Reviewed upstream-boundary and size baseline
  active_profile                         Default profile name for the tools
  profiles/<name>/
    overrides.txt                        Define states (enabled / disabled / value)
    residual.*.patch                     Fork-only options and blocks
```

Fork-only functionality currently includes: DWIN thumbnail rendering, one-click printing, X/Delta automatic Z-offset routines, FT_Motion with input shaping, the HX711 load-cell path, CRTouch helpers, the compact 6x6 leveling grid, LCD dimmer and brightness controls, and filament runout handling.

## Configuration profiles

A profile is the difference between upstream's configuration files and the machine configuration:

- `overrides.txt` lists define states (for example `Configuration_adv.h E FT_MOTION`).
- `residual.*.patch` carries options and blocks that do not exist upstream.

```sh
# Regenerate the profile after editing Configuration*.h by hand
python3 buildroot/share/e3v3se/config_profile.py --profile c13_sd derive

# Verify the files equal upstream + profile
python3 buildroot/share/e3v3se/config_profile.py --profile c13_sd check

# Rebuild the files from upstream + profile (after taking upstream's versions)
python3 buildroot/share/e3v3se/config_profile.py --profile c13_sd apply

# List upstream options that the profile disables (review after a merge)
python3 buildroot/share/e3v3se/config_profile.py --profile c13_sd stale
```

Without `--profile`, the tools use the name stored in `buildroot/share/e3v3se/active_profile`.

This branch uses the `c13_sd` profile. The `c13_octoprint` branch adds `OCTOPRINT_PLUGIN` and `MEATPACK_ON_SERIAL_PORT_1`, and turns `ONE_CLICK_PRINT` off.

## Enabling and disabling features by hand

Features are plain Marlin defines. To toggle one, edit `Marlin/Configuration.h` or `Marlin/Configuration_adv.h`, then record the change in the profile so it survives the next upstream merge.

1. **Edit the define.** Comment it out (`//#define FEATURE`) to disable it, or remove the `//` to enable it. Options that carry a value keep it on the same line.
2. **Regenerate the profile** for the branch you are on:
   ```sh
   python3 buildroot/share/e3v3se/config_profile.py --profile c13_sd derive
   python3 buildroot/share/e3v3se/config_profile.py --profile c13_sd check
   ```
3. **Build and check size**, then commit and run the guardrails:
   ```sh
   pio run -e STM32F103RET6_creality
   python3 buildroot/share/e3v3se/check_profile.py build
   ```
4. If flash or RAM grew on purpose, update the baseline with `check_profile.py build --update` after reviewing the change.

Skipping step 2 is the usual cause of a failing `config` guardrail, because the profile no longer matches the files.

Some options depend on others. For example, `ONE_CLICK_PRINT` requires `NO_SD_AUTOSTART` (set automatically inside its own block) and is incompatible with `BROWSE_MEDIA_ON_INSERT`; the build stops with a sanity-check message if a dependency is missing.

### Common switches

| Feature | Define | File |
| --- | --- | --- |
| Auto Z-offset, delta (4 points) | `D_ROUTINE_AUTO_OFFSET` | `Configuration.h` |
| Auto Z-offset, X (5 points) | `X_ROUTINE_AUTO_OFFSET` | `Configuration.h` |
| Brightness and dimmer menu | `DWIN_DIMM_MENU` | `Configuration.h` |
| Custom extrude menu | `DWIN_CUSTOM_EXTRUDE` | `Configuration.h` |
| Extra preheat labels | `EXTRA_PREHEAT_LABELS` | `Configuration.h` |
| Filament runout sensor | `FILAMENT_RUNOUT_SENSOR` | `Configuration.h` |
| Filled 3D bed mesh viewer | `BED_LEVEL_VISUALIZER` | `Configuration_adv.h` |
| FT_Motion and input shaping | `FT_MOTION` | `Configuration_adv.h` |
| Ignore bed MINTEMP (off by default) | `E3V3SE_IGNORE_BED_MINTEMP` | `Configuration.h` |
| Input shaping menu (legacy X/Y) | `DWIN_INPUT_SHAPING_MENU` | `Configuration_adv.h` |
| Linear Advance | `LIN_ADVANCE` | `Configuration_adv.h` |
| MeatPack on serial port 1 | `MEATPACK_ON_SERIAL_PORT_1` | `Configuration_adv.h` (on in `c13_octoprint`) |
| OctoPrint plugin layer | `OCTOPRINT_PLUGIN` | `Configuration.h` (`c13_octoprint` only) |
| One-click print | `ONE_CLICK_PRINT` | `Configuration_adv.h` |
| Pad Z homing, experimental (off by default) | `PAD_Z_HOMING` | `Configuration.h`, thresholds `PAD_*` in `Configuration_adv.h` |
| Nozzle wipe before pad Z homing, experimental (off by default, needs `PAD_Z_HOMING`) | `PAD_SCRAPE` | `Configuration_adv.h`, settings `PAD_SCRAPE_*` |
| Power-loss recovery and LCD menu (off by default) | `POWER_LOSS_RECOVERY`, `DWIN_PLR_MENU` | `Configuration_adv.h` |
| Preheat completion alert | `PREHEAT_ALERT` | `Configuration.h` |
| Thumbnail rendering | `DWIN_RENDER_THUMBNAIL` | `Configuration.h` |
| Z height after homing menu | `DWIN_ZHOME_MENU` | `Configuration.h` |

The table lists switches, not a statement of the current state; check the define in the file to see whether it is on in your branch. Some menu options depend on a base feature (for example the input shaping menu needs the legacy shapers with `FT_MOTION` disabled).

`POWER_LOSS_RECOVERY` with `DWIN_PLR_MENU` adds **Control → Power-loss recovery**, a submenu for enabling SD recovery, heating the bed/nozzle while awaiting confirmation, bed temperature boost, automatic-resume temperature (0 = off), Z clearance, purge length and checkpoint Z interval. Use **Store settings** in the submenu or `M500` to persist all options; `M501` reloads them and `M502` restores defaults. `Configuration_adv.h` supplies the initial defaults. Recovery takes priority over One Click Print at startup. See the [recovery behavior and validation guide](docs/power-loss-recovery.md).

**Enhanced PLR is disabled by default in all four branches to save flash and RAM.** Enable both `POWER_LOSS_RECOVERY` and `DWIN_PLR_MENU` in `Configuration_adv.h`, or select **POWER_LOSS_RECOVERY + DWIN_PLR_MENU** in the GitHub Actions build dialog. This single workflow option is off by default and enables/disables recovery and the advanced LCD submenu together. The LCD switch and `M413` only control recovery when it was included at build time; they cannot enable a feature compiled out of the firmware. Recovery handles files printed from the printer's SD card, including on the OctoPrint branches; streamed host jobs are not recoverable with PLR.

`PREHEAT_ALERT` sounds three short beeps when the nozzle and bed reach the targets selected from the LCD's **Prepare → Preheat** menu (PLA/TPU, plus PETG/ABS with `EXTRA_PREHEAT_LABELS`). It is off by default and can be enabled in the GitHub Actions build dialog. Editing or saving presets does not arm the alert; cooling down, changing heater targets, or starting a print cancels it. The beeps run without blocking the LCD or temperature management.

`BED_LEVEL_VISUALIZER` adds **Leveling → Bed Level Map** to the Creality DWIN LCD. It displays the current bilinear bed mesh as a filled isometric surface with interpolated colors: blue below zero, green at zero, and red above zero. A mesh of 6×6 measured points forms 5×5 surface cells; all 36 measurements are used. The map title is centered in yellow in the header. The legend shows the color scale in millimeters; the header shows minimum, maximum, total height range, population standard deviation and the original bed-level assessment from `v1.0.9.8_1f`. Statistics use three decimal places and integer-based formatting, so they remain visible without floating-point printf support. The view omits the mesh-size banner and fixed origin label. Reference axes X, Y and Z are drawn behind the heatmap using the same projection, with their labels outside the colored surface. Vertical displacement is exaggerated automatically so the shape is visible. Single-pixel scan bands overlap at their far edge to avoid gaps on panels with exclusive rectangle endpoints. The QR links to the original bed-leveling help at `https://bit.ly/navaismo-bed-leveling`, preserving the URL's case and a white quiet zone. It is also available when there is no mesh.

The assessment selects the best rating that satisfies **both** original limits:

| LCD rating | Maximum height range (mm) | Maximum standard deviation (mm) |
| --- | --- | --- |
| PERFECT | 0.3 | 0.1 |
| GOOD | 0.5 | 0.2 |
| OK | 1.0 | 0.3 |
| BAD | 2.0 | 1.0 |
| HORRIBLE | Above either BAD limit | Above either BAD limit |

Missing measurements appear gray and receive `INCOMPLETE` instead of a quality rating; an uninitialized mesh shows a message to run bed leveling. The viewer sends one drawing command at a time with at least `BED_LEVEL_VISUALIZER_DRAW_DELAY_MS` between commands (10 ms by default, minimum 5 ms). This pacing covers the initial screen, QR, legend, surface rectangles and grid lines, preventing an uninterrupted flood of drawing commands. Increase the interval, for example to 20 ms, if a panel needs more time. Pressing the encoder returns to Leveling with Bed Level Map selected, even while drawing; temperature management continues between commands. The viewer reads a snapshot of the mesh without moving the printer or changing calibration. It requires `AUTO_BED_LEVELING_BILINEAR` and can be selected in the GitHub Actions build dialog (the workflow toggle defaults to off).

## Pad Z homing (experimental)

`PAD_Z_HOMING` (off by default) homes Z by touching the **nozzle** on the bed strain-gauge pad at the front-left bed mount (`AUTOZ_TOOL_X/Y`, the HX711 sensor of the automatic Z-offset) instead of homing with the CR Touch. The CR Touch stays configured, so `G29` still works when it is clipped on, but `G28` never deploys or reads it and works with the probe unplugged.

**Experimental: tested on one printer so far (homes repeatably within 0.02 mm). The thresholds come from that printer's sensor. Follow the first-test procedure.**

Enable it by uncommenting `#define PAD_Z_HOMING` in `Configuration.h` (then follow the profile steps above), or tick **PAD_Z_HOMING** in the GitHub Actions build dialog. EEPROM layout is unchanged.

What `G28` does with the option on: heating starts together with the X/Y homing, at print temperature: a nozzle target of 50 °C or more / bed target of 30 °C or more that is already set (e.g. by your start G-code before `G28`) is used, otherwise `PAD_HOME_NOZZLE_TEMP` (220 °C) and `PAD_HOME_BED_TEMP` (50 °C); `PAD_HOME_NOZZLE_TEMP 0` touches cold without heating. Both are waited for (15 min limit; `M108` cancels; the nozzle counts as reached at target - 2 °C, the bed at target - 1 °C and then 30 s more, because the plate and gauge keep moving after the bed controller reaches temperature) before the nozzle goes over the pad, then the filament is retracted 1 mm against ooze. After homing the targets from before `G28` are restored (off if they were off). **Wipe the hot nozzle clean** before homing: plastic on the tip or on the pad gives a wrong Z. Then the nozzle moves over the pad and waits until the gauge stops drifting (a freshly heated bed drifted up to ~300 counts/s on the first printer; every 2 s it takes a trimmed mean of 16 readings, fits the drift over the last 6 of them (~10 s) and waits until it is under 200 counts per 2 s three times in a row, at most 3 minutes, then continues because the descent tracks slow drift anyway), the sensor is checked (answers, not stuck, not saturated, quiet: noise is the min..max of 32 readings after removing slow drift and the 2 highest and 2 lowest readings, so the single-reading spikes seen while the heaters regulate do not count) before any descent, Z steps down 0.05 mm per HX711 sample until the force rises, backs off 1 mm, measures the contact three times in 0.02 mm steps (median, spread check), sets Z, and `G28` lifts to `Z_AFTER_HOMING`. `G28 Z` does the same after homing X/Y if needed; the `H` parameter is ignored. Every stage prints a `PAD_HOME:` line with its numbers. **A failure does not halt the printer.** The nozzle is lifted by what it moved down (at most 2 mm), all heaters are turned off, Z stays unhomed, the serial log gets `Error:PAD_HOME: FAILED: <reason>` and the LCD shows `Pad Z homing FAILED: <reason>` (press the knob to go to the main menu). A running SD print is aborted the same way as Stop (the `EVENT_GCODE_SD_ABORT` G-code then raises Z 10 mm and parks), and a host such as OctoPrint receives `//action:cancel`. `G28` ends without lifting Z or running the after-homing G-code. Because the real height is unknown, Z is set to 0 (plus the mesh correction when leveling is re-enabled) after a failure: every later Z move (the next `G28`, a queued print line) can only go up from there. Fix the cause and send `G28` again; there is no automatic retry. (OctoPrint may disconnect on the `Error:` line, depending on its firmware-error setting.)

### Hardware: fan shroud and probe mount

With pad Z homing the CR Touch is **only needed for bed leveling** (`G29` / LCD leveling). Homing, the nozzle wipe and printing work without it, so it can stay off for printing (it otherwise hangs next to the nozzle and limits the tilt of non-planar toolpaths). Clip it on, level, take it off.

A matching part-cooling fan shroud with a relocated probe mount is designed for this setup:

- **CAD (Onshape):** [Ender-3 V3 SE fan shroud + probe mount](https://cad.onshape.com/documents/c24d2ffa06007b6722483159/w/6306d1cafc16823eff7ef349/e/125d47a761374161794c4edb)
- **STL:** `hardware/fan-shroud/` (to be added)
- **Fans:** the shroud takes **4x 5015 blower fans** (part cooling). Check that your board's fan output can drive all four (wire them in parallel; 24 V fans on the stock 24 V fan header).
- **Non-planar clearance:** the shroud is designed around a 39° clearance cone; measured from the nozzle tip centre the lowest edge of the shroud is at **36° all around** (r 15 mm, 11 mm above the tip). Keep non-planar slopes at or below ~32-33° for margin (the stock CR Touch mount allowed about 5.5°). The gantry wheels sit ~48 mm above the tip at 80-100 mm from the nozzle.
- **Probe offset with this mount:** the probe moves 18 mm further out in X and 10 mm closer to the nozzle in Y than the stock mount (`M851 X-24.25 Y-15`): **`M851 X-42.24 Y-5`** (from CAD; check it with the dot test below), then `M500`. With the probe 42 mm left of the nozzle the rightmost ~40 mm of the bed cannot be probed; the mesh extrapolates there. the Z value of `M851` does not matter (`G29` normalizes the mesh to the pad spot). To get X/Y exactly on your printer: put a dot on paper on the bed, move the nozzle over it and note X/Y, deploy the pin (`M401`), move the pin over the same dot and note X/Y; the difference is the offset.
- With the probe clipped on, check that its body sits higher than the nozzle tip before running the wipe (`PAD_SCRAPE`), or switch the wipe off with `M8016 E0` while the probe is fitted.

### First test

1. Cold, clean nozzle tip (no blob), empty bed, serial terminal connected, **hand on the power switch**.
2. Send `M8015 T` (nothing moves; prints readings for 10 s). At rest `delta` should wander by much less than `PAD_NOISE_MAX` (1500). Press the bed at the front-left corner with a finger: `delta` must jump well past `PAD_COARSE_TRIGGER` (4000). Reference from the first printer: about 100-110 counts per gram at the front-left corner (a 206 g phone gave about -23000), idle noise about +-150 counts cold (peaks +-450); with the nozzle at 220 °C and the bed at 50 °C single readings jump by up to about 900, and the gauge drifts by 100-300 counts/s right after heating. A homing touch triggers at about 30-45 g (`PAD_FINE_TRIGGER` 3000). If it does not, **do not run `G28`**; adjust the `PAD_*` thresholds or check the sensor.
3. Abort test in the air: right after power-on (Z not homed yet) send `G1 Z50 F600` so the nozzle is 50 mm or more above the bed, then send `G28`. (Do not home first to jog: the LCD Move menu runs a full `G28` before it lets you move.) Check that Z goes **down** in the slow descent (power off if it goes up). While it descends, press the bed firmly at the front-left corner. The descent must stop at once and back off 1 mm; then homing fails with `fine pass: no contact in range` or `fine pass: early trigger` (or `pad still loaded after backoff` / `sensor noisy after contact` if you keep pressing). That failure is the expected result: heaters off, failure screen on the LCD; press the knob and continue without a power cycle.
4. Real test: nozzle about 20 mm above the bed, `G28`. Expect one touch, a 1 mm lift, three slow touches, then a lift. Power off if the nozzle keeps pushing into the bed. Afterwards send `M420 S0` and `G1 Z0.1 F300` and slide a sheet of paper under the nozzle: it should just drag.
5. Repeatability: send `G28` five more times. From the second homing on, the line `PAD_HOME: contact moved by ... mm since the previous homing` should stay within a few hundredths of a millimetre, and `spread_mm` at 0.02 or less.

### Nozzle wipe before homing (`PAD_SCRAPE`, experimental)

Plastic on the nozzle tip makes the pad touch wrong (`fine passes disagree`). With `PAD_SCRAPE` (in `Configuration_adv.h`, off by default; GitHub Actions switch **PAD_SCRAPE**) and a nozzle hot enough to extrude, `G28` does:

1. Heating starts; Y is homed, then X (`HOME_Y_BEFORE_X`); the nozzle waits for heat parked off the bed at `PAD_SCRAPE_PARK_X`/`PAD_SCRAPE_PARK_Y` (-11/10) so ooze drips outside it. Heat wait as above but **without the 30 s bed soak** (the bed soaks during the touch and wipe) and without the gauge settling (the descents track drift).
2. Retract `PAD_SCRAPE_RETRACT` (2 mm; replaces the 1 mm retract, logical E unchanged).
3. **Rough touch on the scrape strip** at `PAD_SCRAPE_X`/`PAD_SCRAPE_ROUGH_Y` (-5/10): the coarse descent only (free reference validated in the air first, contact by a fast change of more than `PAD_SCRAPE_TRIGGER` 2500 counts, confirmed by a second sample). A squashed blob lands on the strip, never on the pad.
4. Lift 5 mm, over to `PAD_SCRAPE_Y0` (50) at that height, new free reference (must match the first within 8000 counts, else the wipe is skipped).
5. **Wipe**: `PAD_SCRAPE_PASSES` (4) passes Y50→`PAD_SCRAPE_Y1` (20)→50→20→50 at `PAD_SCRAPE_SPEED` (25 mm/s, accelerating from 5 mm/s at 2000 mm/s²), holding the load at `PAD_SCRAPE_FORCE` ± `PAD_SCRAPE_BAND` (15000 ± 5000 counts, ~150 ± 50 g): over the band Z goes up (0.05 mm per sample) and the feed slows to a stop, under it Z goes back down (0.01 mm per sample) but never deeper than `PAD_SCRAPE_DEPTH` (0.2 mm, at most 0.4) below the rough contact. Z and Y are stepped directly while each HX711 conversion runs, so every sample (~80/s, i.e. every ~0.3 mm at 25 mm/s) sets Z and the feed for the next steps. Over `PAD_SCRAPE_FORCE_MAX` (35000, ~350 g), or still over the band 1 mm above the contact, the nozzle lifts 5 mm and the wipe ends.
6. **Flick**: at the wipe height, the nozzle moves sideways off the strip edge to `PAD_SCRAPE_FLICK_X` (-11; comment it out for no flick), same load control, so material on the tip stays out there, then lifts there (off the bed, away from the pad and the start point).
7. Travel 5 mm above the strip to the pad spot (25/28, untouched so far), then straight on, so no new ooze forms: no soak and no settle wait, only a fresh baseline in the air (noise and drift checked, a noisy or loaded gauge fails homing softly as usual), the coarse descent and the fine passes (each with its own baseline, spread limit 0.10 mm). Heat wait happens once per `G28`.

A wipe that cannot run (no contact in range, reference mismatch, noisy sensor, load limit) prints `PAD_SCRAPE: WARNING: wipe incomplete: <reason>` and homing goes on; it fails as usual if the tip is still dirty, then with `nozzle still dirty after scrape`. A stopped print or an interrupted move fails the homing as before. Log lines: `PAD_SCRAPE: reference=...`, one per pass `PAD_SCRAPE: pass 1 Y50.0->20.0: ... ms, samples=... (in band ..., under the contact ...) load min/avg/max=... Z ... from the contact` (pass 0 is the flick), and after homing `PAD_SCRAPE: strip contact at the rough spot was Z...` (relative to the pad contact).

`M8016` runs steps 3–6 alone (X/Y/Z homed, leveling off meanwhile, positions stay exact; nothing is heated): `M8016 [X] [R<rough Y>] [Y<start>] [J<turning point>] [P<passes>] [D<depth>] [S<load>] [W<band>] [L<hard>] [F<mm/s>] [K<flick X, = X for none>] [Z<expected strip height>]`. Defaults are the `PAD_SCRAPE_*` values. Tuning order:

1. `M8015 T` with heaters on, as above.
2. `M8016 Z10`: in the air. Goes to X-5 Y10 at Z15, takes the reference, descends to Z9 and stops with `no strip contact in range`. Check the direction and that nothing touches.
3. `M8016 D0.05 F5 P1 K-5`: one slow pass barely under the contact, no flick; watch the `load` numbers.
4. `M8016` with the defaults, then adjust `D`, `S`/`W` or `F`. Press the bed during a wipe: the nozzle must go up and the feed stop (and lift on a hard press).

### Z offset and mesh

- Z0 is the nozzle contact on the pad, plus `M206 Z` (stored with `M500`, applied at the next `G28`; positive = nozzle closer to the bed; limited to ±1 mm, a larger value makes `G28` fail).
- The probe Z offset (`M851 Z`) is **not** used for homing. Since `G29` makes the mesh 0 at the pad spot (below), it does not matter for the mesh either.
- The LCD **Z-Offset** (Prepare and Tune menus) shows and edits `M206 Z` in this mode, with the probe offset's sign: -0.45 means the nozzle 0.45 mm closer to the bed (`M206 Z0.45`). Turning the knob babysteps at once (also during a print); confirming saves it to EEPROM, and every later `G28` applies it. Range ±1 mm (pad homing refuses more). The probe Z offset is not shown on the LCD any more (`M851 Z` from the host still works but does not matter, see `G29` below). **Advanced → Home Offsets** shows the same value with the `M206` sign. Host `M290 Z` only babysteps until the next `G28` (with `BABYSTEP_ZPROBE_OFFSET` it also changes `M851 Z`, which pad homing ignores).
- `G29` (and **Start Bed Leveling**, which runs it) probes the pad spot (25/28) once more after the grid and subtracts that height from the whole mesh, so the mesh is 0 where `G28` homes and the probe Z offset (`M851 Z`) and the probe's clip-on height cancel out (`PAD_MESH: bed at the pad spot ... mesh shifted by ...`). If the probe cannot reach the spot, the mesh value there is used instead. The shifted mesh is what is saved (`G29` saves the settings at its end in this firmware). The dots drawn on the LCD while probing are the raw values.
- Making a mesh: clip the CR Touch on and plug it in (printer off), then use **Leveling → Start Bed Leveling** on the LCD or send `G28` + `G29`; the mesh is saved and is relative to the pad contact. Mesh view, mesh edit and `M420 S1` work as before. Power off, unplug the probe, print. `G28` re-enables the stored mesh by itself (`ENABLE_LEVELING_AFTER_G28`); `M420 S1` in the start G-code does no harm. Remove `G29` from the start G-code: with the probe unplugged it stops the printer with a probe error.

### Removed with the option on

Everything that would use the probe without an explicit leveling request is taken out, because the probe may be unplugged:

- **Automatic Z-offset**: the **Start Auto Z-Offset** row is gone from the Leveling menu and plain `M8015` only prints a notice (the routine assumes probe-homed Z and would drag the nozzle over the bed here). `M8015 T` still prints the sensor readings. The probe Z offset is therefore whatever is stored (`M851 Z`); `G29` cancels it out (see above).
- **First-boot wizard** (language, automatic Z-offset, leveling): skipped. A fresh EEPROM boots to the main menu in English; set the language under Prepare.
- **Level calibration before a print** (the toggle in the file preview): removed, a print start never probes.

Still probe-dependent, and only on request: `G29` / Start Bed Leveling, `G30`, `M48`, `M401`/`M402`, and **Probe Deploy / Probe Stow** in the Move menu. With the probe unplugged these end in a probe error that stops the printer.

### Not verified

- Nothing was tested on hardware: sensor polarity, noise, sample rate and the force per millimetre are unknown, and all thresholds are in raw HX711 counts.
- A strain gauge that looks alive but does not react to load cannot be detected before contact. The nozzle would then push into the bed until the travel limit (the full Z height on the first homing after power-on, expected height + 5 mm afterwards). Steps 2 and 3 exist to rule this out.
- Z is stepped directly at 4 mm/s without acceleration; lost steps on the way up would shift the result.
- Bed or nozzle heating disturbs the sensor (spikes, drift). The noise check ignores up to 2 spikes per side and slow drift; more than that still fails the homing (no power cycle needed).
- The failure screen, the print abort from the LCD, and the `G28` cleanup after a failure have not been tried on the printer yet.
- A hot nozzle can mark the bed surface at the pad, and ooze under the nozzle makes Z read high.
- The bed bends slightly under the trigger force, so the first layer may need a constant correction.
- The reduced Leveling menu, the skipped wizard and the file preview without the level-calibration toggle are untested on the display.
- With the probe unplugged its input reads "triggered" and the servo commands at boot and at `G28` go nowhere. From the code this is harmless because nothing reads the probe outside probing, but it has not been tried.

## Input shaping with FT_Motion (`M493`)

The default profile uses **FT_Motion advanced shapers** (`FTM_SHAPER_*`). **Configure FT_Motion with `M493`, using the commands in the table below.** The LCD **Input Shaping menu applies only to legacy standard motion (`M593`)** and cannot configure FT_Motion. It is automatically disabled whenever `FT_MOTION` is compiled in. In GitHub Actions, selecting FT_Motion forces the legacy menu off even if its checkbox is also selected.

To use **Control → Motion → Input Shaping**, disable `FT_MOTION` and enable `DWIN_INPUT_SHAPING_MENU` in `Configuration_adv.h` or the workflow. The menu enables legacy `INPUT_SHAPING_X/Y` together, using the configured frequency and damping defaults. It adjusts X/Y frequency (0 to disable an axis, otherwise up to 999.99 Hz within the timer's valid range) and damping ratio (0.00–1.00). Accepted values update the stepper's shaping delay and damping factors through the same setters as `M593`; no reboot is needed. `M593` reports the current values. Use **Control → Store Settings** or `M500` to persist them; `M501` reloads them. This LCD menu does not configure FT_Motion.

| Command | Effect |
| --- | --- |
| `M493` | Report the current FT_Motion and shaper settings |
| `M493 S1` / `M493 S0` | Enable / disable Fixed-Time Motion |
| `M493 X C<mode> A<Hz>` | Shaper mode and frequency for X |
| `M493 Y C<mode> A<Hz>` | Shaper mode and frequency for Y |
| `M493 X Y I<zeta> Q<vtol>` | Damping ratio and vibration tolerance (EI-family shapers) |

Modes for `C`: `0` None, `1` ZV, `2` ZVD, `3` ZVDD, `4` ZVDDD, `5` EI, `6` 2HEI, `7` 3HEI, `8` MZV. Set the mode for every axis you want shaped, then save with `M500`.

Example: 3-hump EI at 42 Hz on Y and ZV at 37 Hz on X.

```gcode
M493 Y C7 A42
M493 X C1 A37
M500
```

Factory defaults: X none at 37 Hz, Y 3HEI at 42 Hz. To change them, edit `FTM_DEFAULT_SHAPER_*` and `FTM_SHAPING_DEFAULT_FREQ_*` in `Configuration_adv.h` and follow the profile steps above.

## Linear Advance: legacy vs FT_Motion

There are two independent Linear Advance implementations, and this fork uses the FT_Motion one.

| | Legacy `LIN_ADVANCE` | FT_Motion built-in advance |
| --- | --- | --- |
| Enabled by | `#define LIN_ADVANCE` (commented out in this fork) | Automatically, whenever `FT_MOTION` is on and the machine has extruders |
| Where it runs | Stepper ISR, per planner block | Trajectory generator (1 kHz), added to the extruder position |
| Applies to | The standard-motion planner | Only moves executed by FT_Motion |
| K-factor | `M900 K<mm>` | The same `M900 K<mm>` (default `ADVANCE_K 0.0`, i.e. no effect until you set it) |

How the FT_Motion version behaves:

- For printing moves it adds `extruder velocity x K` to the extruder trajectory, so pressure is built up ahead of acceleration and relieved before deceleration.
- Retract and unretract moves are **not** advanced, so retractions behave as the slicer wrote them.
- `LIN_ADVANCE` stays commented out in this fork. FT_Motion does not need it, and enabling it only adds the legacy ISR code (more flash) without changing FT_Motion moves.
- `M493 P0` / `M493 P1` turns the advance term off or on at runtime without changing K (see the `M493` help in `Marlin/src/gcode/feature/ft_motion/M493.cpp`).

**Effect on extrusion.** Because K is shared through `M900`, a K value in your slicer start G-code is applied by FT_Motion even though `LIN_ADVANCE` is commented out. On this printer a start G-code carrying `M900 K0.10` coincided with under-extrusion after a number of retractions. The prints were good again after removing that line from the slicer start G-code, recalibrating E-steps and lowering the speed. Treat K as untuned: leave it at `0` until you have calibrated it (a pressure-advance pattern is the usual way), then set it with `M900 K<value>` and save with `M500`.

## Guardrails

The guardrails compare against `HEAD`, so commit before running them.

```sh
python3 buildroot/share/e3v3se/check_profile.py boundary   # edits to upstream-owned files must not grow
python3 buildroot/share/e3v3se/check_profile.py config     # config files match upstream + profile
python3 buildroot/share/e3v3se/check_profile.py build      # RAM/flash must not exceed the baseline
```

Use `--update` only after a deliberate, reviewed change. A new edit to an upstream-owned file has to be added to the boundary list in `baseline.json` on purpose.

## Merging upstream

The November integration and its compatibility checks are described in
[`docs/upstream-integration-2026-11.md`](docs/upstream-integration-2026-11.md).

1. Merge `bugfix-2.1.x` and resolve conflicts in favor of upstream, keeping the fork hooks.
2. Take upstream's `Configuration.h` and `Configuration_adv.h`, then run `config_profile.py apply`. It fails loudly if a residual patch no longer applies.
3. Run `config_profile.py stale` to review upstream options that the profile turns off.
4. Run the three guardrails and build every profile.
5. Flash and print a test object before releasing.

## Host (OctoPrint) integration

The `c13_octoprint` profile enables the host layer in `Marlin/src/e3v3se/OctoHost.*`, designed for the [OctoPrint-LCD_E3V3SE](https://github.com/navaismo/Octoprint-LCD_E3V3SE) plugin:

- `M9000` drives the print and finish screens and the print details (file name, time, progress).
- `M9001` streams the thumbnail (RGB565, 96 px wide) line by line.
- `M115` or `M9000 A1` marks the host as connected. `HOST_LINK_UP()` then enables host prompts for pause, resume and filament change; without the plugin, stock upstream behavior is kept.
- MeatPack is enabled and the emergency parser is kept in sync with it, so `M876`, `M108` and `M410` keep working.

## Bed minimum-temperature safety override

`E3V3SE_IGNORE_BED_MINTEMP` is disabled by default in all four branches, so the bed `MINTEMP` check remains active. Enable the override by uncommenting it in `Configuration.h` or selecting **E3V3SE_IGNORE_BED_MINTEMP** in the GitHub Actions build dialog (default: off). Enabling it suppresses only the bed `MINTEMP` shutdown; `BED_MAXTEMP`, thermal runaway protection, and the bed heating-failed (`WATCH_BED`) protection remain active.

This override should not be enabled when the bed sensor is required for safe
operation. A disconnected sensor can otherwise go unnoticed.

## Building

PlatformIO is the build system. Use a recent PlatformIO Core (6.2 or newer).

```sh
pio run -e STM32F103RET6_creality    # C13 (c13_sd)
```

Native unit tests and the upstream build tests are available through the Makefile (`make unit-test-all-local`, `make tests-all-local`).

### Building with GitHub Actions

The `c13_sd` branch (C13 SD card) ships a manual workflow, [`build-marlin.yml`](.github/workflows/build-marlin.yml), that compiles the firmware on GitHub so no local toolchain is needed. Fork the repository first; Actions run in your own fork.

1. Open your fork on GitHub, go to **Actions**, and select **Build E3V3SE Firmware**. If workflows are disabled in the fork, click **I understand my workflows, go ahead and enable them**.
2. Click **Run workflow** and choose the `c13_sd` branch.
3. Keep the environment at `STM32F103RET6_creality` and set the feature toggles you want (thumbnails, Z-offset routines, FT_Motion, DWIN menus, preheat labels and so on). The defaults match the committed configuration of this branch: `ONE_CLICK_PRINT` on; `OCTOPRINT_PLUGIN` and MeatPack are not offered.
4. Click **Run workflow** and wait for the job to finish. The log ends with a RAM and flash usage report.
5. Download the `firmware-c13_sd-STM32F103RET6_creality` artifact from the run summary. It contains the `firmware*.bin` file and is kept for 7 days.

Each toggle only overrides the matching define for that single build; nothing is committed back to the repository. To make a choice permanent, edit the profile as described in [Configuration profiles](#configuration-profiles). Check the reported memory usage against the [Memory budget](#memory-budget) before flashing, and enable `DWIN_INPUT_SHAPING_MENU` only with `FT_MOTION` disabled, and `DWIN_SKEW_MENU` only together with `SKEW_CORRECTION`. Selecting FT_Motion forces the legacy input shaping menu off for that build.

## Memory budget

Flash and RAM are hard limits on the STM32F103RE (512 KB flash, 64 KB RAM). The reviewed baseline is stored in `buildroot/share/e3v3se/baseline.json`; the `build` guardrail fails when firmware grows beyond it. Check usage after enabling or adding a feature.

## Further documentation

- [`AGENTS.E3V3SE.md`](AGENTS.E3V3SE.md): working rules for this fork.
- [`MEMORY.md`](MEMORY.md): branch map, history and technical decisions.
- [`.github/copilot-instructions.md`](.github/copilot-instructions.md): build, test and architecture notes for AI assistants.
- `docs/`: upstream Marlin technical documents.

---

The original Marlin README follows, as required by the upstream project's policy.

<br>
<br>
<hr>
<br>

<br>
<br>
<hr>
<br>
<p align="center"><img src="buildroot/share/pixmaps/logo/marlin-outrun-nf-500.png" height="250" alt="MarlinFirmware's logo" /></p>

<h1 align="center">Marlin 3D Printer Firmware</h1>

<p align="center">
    <a href="/LICENSE"><img alt="GPL-V3.0 License" src="https://img.shields.io/github/license/marlinfirmware/marlin.svg"></a>
    <a href="//github.com/MarlinFirmware/Marlin/graphs/contributors"><img alt="Contributors" src="https://img.shields.io/github/contributors/marlinfirmware/marlin.svg"></a>
    <a href="//github.com/MarlinFirmware/Marlin/releases"><img alt="Last Release Date" src="https://img.shields.io/github/release-date/MarlinFirmware/Marlin"></a>
    <a href="//github.com/MarlinFirmware/Marlin/actions/workflows/ci-build-tests.yml"><img alt="CI Status" src="https://github.com/MarlinFirmware/Marlin/actions/workflows/ci-build-tests.yml/badge.svg"></a>
    <a href="//github.com/sponsors/thinkyhead"><img alt="GitHub Sponsors" src="https://img.shields.io/github/sponsors/thinkyhead?color=db61a2"></a>
    <br />
    <a href="//bsky.app/profile/marlinfw.org"><img alt="Follow marlinfw.org on Bluesky" src="https://img.shields.io/badge/Follow%20@marlinfw.org-0085ff?logo=bluesky&logoColor=white"></a>
    <a href="//fosstodon.org/@marlinfirmware"><img alt="Follow MarlinFirmware on Mastodon" src="https://img.shields.io/mastodon/follow/109450200866020466?domain=https%3A%2F%2Ffosstodon.org&logoColor=%2300B&style=social"></a>
</p>

### 🌍 Translations

<table>
<tr>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=an">Aragonés</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=bg">Български</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=ca">Català</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=cs">Čeština</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=da">Dansk</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=de">Deutsch</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=el">Ελληνικά</a></td>
</tr>
<tr>
  <td><a href="//github.com/MarlinFirmware/Marlin">English</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=es">Español</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=eu">Euskara</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=fi">Suomi</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=fr">Français</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=gl">Galego</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=hr">Hrvatski</a></td>
</tr>
<tr>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=hu">Magyar</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=it">Italiano</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=ja">にほんご</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=ko">한국어</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=nl">Nederlands</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=pl">Polski</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=pt">Português</a></td>
</tr>
<tr>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=pt-BR">Português (Brasil)</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=ro">Română</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=ru">Русский</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=sk">Slovenčina</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=sv">Svenska</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=tr">Türkçe</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=uk">Українська</a></td>
</tr>
<tr>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=vi">Tiếng Việt</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=zh-CN">简体中文</a></td>
  <td><a href="//translate.google.com/translate?u=github.com/MarlinFirmware/Marlin&sl=auto&tl=zh-TW">繁體中文</a></td>
  <td></td>
  <td></td>
  <td></td>
  <td></td>
</tr>
</table>

Official documentation can be found at the [Marlin Home Page](//marlinfw.org/).

Please test this firmware and let us know if it misbehaves in any way. Volunteers are standing by!

---

## Marlin 2.1 Bugfix Branch

**Not for production use. Use with caution!**

Marlin 2.1 supports both 32-bit ARM and 8-bit AVR boards while adding support for up to 9 coordinated axes and to up to 8 extruders.

This branch is for patches to the latest 2.1.x release version. Periodically this branch will form the basis for the next minor 2.1.x release.

Download earlier versions of Marlin on the [Releases page](//github.com/MarlinFirmware/Marlin/releases).

## Example Configurations

Before you can build Marlin for your machine you'll need a configuration for your specific hardware. Upon request, your vendor will be happy to provide you with the complete source code and configurations for your machine, but you'll need to get updated configuration files if you want to install a newer version of Marlin. Fortunately, Marlin users have contributed hundreds of tested configurations to get you started. Visit the [MarlinFirmware/Configurations](//github.com/MarlinFirmware/Configurations) repository to find the right configuration for your hardware. Make sure to select a compatible branch! [The Marlin Download Page](//marlinfw.org/meta/download/) matches compatible software and configuration packages.

## Building Marlin 2.1

To build and upload Marlin you will use one of these tools:

- The free [Visual Studio Code](//code.visualstudio.com/download) using the [Auto Build Marlin](//marlinfw.org/docs/basics/auto_build_marlin.html) extension.
- Marlin is optimized to build with the [PlatformIO IDE](//platformio.org/) extension for Visual Studio Code.
- You can also use VSCode with devcontainer : See [Installing Marlin (VSCode devcontainer)](https://marlinfw.org/docs/basics/install_devcontainer_vscode.html).
- You can still build Marlin with [Arduino IDE](//www.arduino.cc/en/main/software) : See [Building Marlin with Arduino](//marlinfw.org/docs/basics/install_arduino.html). We hope to improve the Arduino build experience, but at this time, PlatformIO is the preferred choice.

## 32-bit ARM boards

Marlin is compatible with a plethora of 32-bit ARM boards, which offer ample computational power and memory and allows Marlin to deliver state-of-the-art performance and features we like to see in modern 3d printers. Some of the newer features in Marlin will require use of a 32-bit ARM board.

## 8-Bit AVR Boards

Marlin originates from the era of Arduino based 8-bit boards, and we aim to support 8-bit AVR boards in perpetuity. Both 32-bit and 8-bit boards are covered by a single code base that can apply to all machines. Our goal is to support casual hobbyists, tinkerers, and owners of older machines and boards, striving to allow them to benefit from the community's innovations just as much as those with fancier machines and newer boards. In addition, these venerable AVR-based machines are often the best for testing and feedback!

## Hardware Abstraction Layer (HAL)

Marlin's Hardware Abstraction Layer provides a common API for all the platforms it targets. This allows Marlin code to address the details of motion and user interface tasks at the lowest and highest levels with no system overhead, tying all events directly to the hardware clock.

Every new HAL opens up a world of hardware. Marlin currently has HALs for more than a dozen platforms. While AVR and STM32 are the most well known and popular ones, others like ESP32 and LPC1768 support a variety of less common boards. At this time, an HAL for RP2040 is available in beta; we would like to add one for the Duet3D family of boards. A HAL that wraps an RTOS is an interesting concept that could be explored.

Did you know that Marlin includes a Simulator that can run on Windows, macOS, and Linux? Join the Discord to help move these sub-projects forward!

### Supported Platforms

| Platform | MCU | Example Boards |
| --- | --- | --- |
| [Arduino AVR](//www.arduino.cc/) | ATmega | RAMPS, Melzi, RAMBo |
| [Teensy++ 2.0](//www.microchip.com/en-us/product/AT90USB1286) | AT90USB1286 | Printrboard |
| [Arduino Due](//www.arduino.cc/en/Guide/ArduinoDue) | SAM3X8E | RAMPS-FD, RADDS, RAMPS4DUE |
| [ESP32](//github.com/espressif/arduino-esp32) | ESP32 | FYSETC E4, E4d@BOX, MRR |
| [GD32](//www.gigadevice.com/) | GD32 ARM Cortex-M4 | Creality MFL GD32 V4.2.2 |
| [HC32](//www.huazhoucn.com/) | HC32 | Ender-2 Pro, Voxelab Aquila |
| [LPC1768](//www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc1700-cortex-m3/512-kb-flash-64-kb-sram-ethernet-usb-lqfp100-package:LPC1768FBD100) | ARM® Cortex-M3 | MKS SBASE, Re-ARM, Selena Compact |
| [LPC1769](//www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc1700-cortex-m3/512-kb-flash-64-kb-sram-ethernet-usb-lqfp100-package:LPC1769FBD100) | ARM® Cortex-M3 | Smoothieboard, Azteeg X5 mini, TH3D EZBoard |
| [Pico RP2040](//www.raspberrypi.com/documentation/microcontrollers/pico-series.html) | Dual Cortex M0+ | BigTreeTech SKR Pico |
| [STM32F103](//www.st.com/en/microcontrollers-microprocessors/stm32f103.html) | ARM® Cortex-M3 | Malyan M200, GTM32 Pro, MKS Robin, BTT SKR Mini |
| [STM32F401](//www.st.com/en/microcontrollers-microprocessors/stm32f401.html) | ARM® Cortex-M4 | ARMED, Rumba32, SKR Pro, Lerdge, FYSETC S6, Artillery Ruby |
| [STM32F7x6](//www.st.com/en/microcontrollers-microprocessors/stm32f7x6.html) | ARM® Cortex-M7 | The Borg, RemRam V1 |
| [STM32G0B1RET6](//www.st.com/en/microcontrollers-microprocessors/stm32g0x1.html) | ARM® Cortex-M0+ | BigTreeTech SKR mini E3 V3.0 |
| [STM32H743xIT6](//www.st.com/en/microcontrollers-microprocessors/stm32h743-753.html) | ARM® Cortex-M7 | BigTreeTech SKR V3.0, SKR EZ V3.0, SKR SE BX V2.0/V3.0 |
| [SAMD21P20A](//www.adafruit.com/product/4064) | ARM® Cortex-M0+ | Adafruit Grand Central M4 |
| [SAMD51P20A](//www.adafruit.com/product/4064) | ARM® Cortex-M4 | Adafruit Grand Central M4 |
| [Teensy 3.2/3.1](//www.pjrc.com/teensy/teensy31.html) | MK20DX256VLH7 ARM® Cortex-M4 |
| [Teensy 3.5](//www.pjrc.com/store/teensy35.html) | MK64FX512-VMD12 ARM® Cortex-M4 |
| [Teensy 3.6](//www.pjrc.com/store/teensy36.html) | MK66FX1MB-VMD18 ARM® Cortex-M4 |
| [Teensy 4.0](//www.pjrc.com/store/teensy40.html) | MIMXRT1062-DVL6B ARM® Cortex-M7 |
| [Teensy 4.1](//www.pjrc.com/store/teensy41.html) | MIMXRT1062-DVJ6B ARM® Cortex-M7 |
| Linux Native | x86 / ARM / RISC-V | Raspberry Pi GPIO |
| Simulator | Windows, macOS, Linux | Desktop OS |
| [All supported boards](//marlinfw.org/docs/hardware/boards.html#boards-list) | All platforms | All boards |

## Marlin Discord

The [Marlin Firmware Discord](//discord.gg/marlin-firmware-461605380783472640) is a great place to discuss issues with Marlin users and developers, get interactive help with troubleshooting, and build on your best ideas to improve to Marlin in tandem with the most active members of the development team.

<p align="center">
  <a target="_blank" href="https://discord.gg/marlin-firmware-461605380783472640">
    <img src="https://dcbadge.limes.pink/api/server/https://discord.gg/marlin-firmware-461605380783472640">
  </a>
</p>

## Other Marlin Support

The Issue Queue is reserved for Bug Reports and Feature Requests. Please use the following resources for help with configuration and troubleshooting:

- [Marlin Documentation](//marlinfw.org) - Official Marlin documentation
- ["Marlin Firmware"](//www.facebook.com/groups/1049718498464482/) Facebook Group
- [Marlin Forum](//forums.reprap.org/list.php?415) at RepRap.org
- ["Marlin Firmware for 3D Printers"](//www.facebook.com/groups/3Dtechtalk/) Facebook Group
- [Marlin Configuration](//www.youtube.com/results?search_query=marlin+configuration) playlist on YouTube

## Contributing Patches

Contribute your patches to Marlin by submitting a Pull Request to the ([bugfix-2.1.x](//github.com/MarlinFirmware/Marlin/tree/bugfix-2.1.x)) branch.

- We use the `bugfix-2.1.x` branch to fix bugs and integrate new features into the latest firmware. Use with caution!
- We maintain `lts-x.x.x` branches, mainly for vendors, so that older versions of Marlin can be patched as-needed to stay functional and fix bugs.
- Follow the [Coding Standards](//marlinfw.org/docs/development/coding_standards.html) to gain points with the maintainers.
- Please submit Feature Requests and Bug Reports to the [Issue Queue](//github.com/MarlinFirmware/Marlin/issues/new/choose). See above for user support.
- Whenever you add new features, be sure to add one or more build tests to `buildroot/tests`. Any tests added to a PR will be run within that PR on GitHub servers as soon as they are pushed. To minimize iteration be sure to run your new tests locally, if possible.
  - Local build tests:
    - All: `make tests-config-all-local`
    - Single: `make tests-config-single-local TEST_TARGET=...`
  - Local build tests in Docker:
    - All: `make tests-config-all-local-docker`
    - Single: `make tests-config-all-local-docker TEST_TARGET=...`
  - To run all unit test suites:
    - Using PIO: `platformio run -t test-marlin`
    - Using Make: `make unit-test-all-local`
    - Using Docker + make: `make unit-test-all-local-docker`
  - To run a single unit test suite:
    - Using PIO: `platformio run -t marlin_<test-suite-name>`
    - Using make: `make unit-test-single-local TEST_TARGET=<test-suite-name>`
    - Using Docker + make: `make unit-test-single-local-docker TEST_TARGET=<test-suite-name>`
- If your feature can be unit tested, add one or more unit tests. For more information see our documentation on [Unit Tests](test).

## Contributors

Marlin is constantly improving thanks to a huge number of contributors from all over the world bringing their specialties and talents. Huge thanks are due to [all the contributors](//github.com/MarlinFirmware/Marlin/graphs/contributors) who regularly patch up bugs, help direct traffic, and basically keep Marlin from falling apart. Marlin's continued existence would not be possible without them.

Marlin Firmware original logo design by Ahmet Cem TURAN [@ahmetcemturan](//github.com/ahmetcemturan).

## Project Leadership

| Name | Role | Link | Donate |
| --- | --- | --- | --- |
| 🇺🇸 Scott Lahteine | Project Lead | [[@thinkyhead](//github.com/thinkyhead)] | [❤️ Donate](//marlinfw.org/docs/development/contributing.html#donate) |
| 🇳🇿 Peter Ellens | Admin | [[@ellensp](//github.com/ellensp)] | [❤️ Donate](//ko-fi.com/ellensp) |
| 🇬🇧 Chris Pepper | Admin | [[@p3p](//github.com/p3p)] |
| 🇺🇸 Keith Bennett | Admin | [[@thisiskeithb](//github.com/thisiskeithb)] | [❤️ Donate](//github.com/sponsors/thisiskeithb) |
| 🇺🇸 Roxanne Neufeld | Admin | [[@Roxy-3D](//github.com/Roxy-3D)] |
| 🇺🇸 Jason Smith | Admin | [[@sjasonsmith](//github.com/sjasonsmith)] |
| 🇧🇷 Victor Oliveira | Admin | [[@rhapsodyv](//github.com/rhapsodyv)] |
| 🇺🇸 Bob Kuhn | Admin | [[@Bob-the-Kuhn](//github.com/Bob-the-Kuhn)] |
| 🇳🇱 Erik van der Zalm | Founder | [[@ErikZalm](//github.com/ErikZalm)] |

## Star History

<a id="starchart" href="//star-history.com/#MarlinFirmware/Marlin&Date">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=MarlinFirmware/Marlin&type=Date&theme=dark" />
    <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=MarlinFirmware/Marlin&type=Date" />
    <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=MarlinFirmware/Marlin&type=Date" />
  </picture>
</a>

## License

Marlin is published under the [GPL license](/LICENSE) because we believe in open development. The GPL comes with both rights and obligations. Whether you use Marlin firmware as the driver for your open or closed-source product, you must keep Marlin open, and you must provide your compatible Marlin source code to end users upon request. The most straightforward way to comply with the Marlin license is to make a fork of Marlin on GitHub, perform your modifications, and direct users to your modified fork.
