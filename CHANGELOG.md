# Changelog

All notable changes to FreeTheBeam. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/) (0.x: interfaces may still change
between minor versions).

## [Unreleased]

## [0.4.0] - 2026-11-08

The LiDAR's own camera: a webcam, pressure advance from photos of printed
lines, and the laser offset from photos. Plus module status, stream mode and
exposure that follows the plate. Camera background and results:
[docs/camera.md](docs/camera.md).

### Added
- **`LIDAR_PA_FLOW_CALIBRATE`** (macro `LIDAR_PA_FLOW_CALIBRATION`): one
  print, measured by the module's IR camera (line widths). The bare step
  (exposure, plate photo, bare LiDAR strips) runs inside START_PRINT before
  the purge (`_LIDAR_PRINT_SCAN` -> `LIDAR_PA_FLOW_STEP`); the helper plans
  the G-code (`helper/lidar_pa_flow.py`), takes and analyses the photos
  (`helper/lidar_camera.py`) and writes `pf-<job>.html`.
  - **PA** from the bulge/thinning after speed changes, saved as `lidar_pa`.
    K1 Max, white matte PLA on black smooth PEI: 0.0361 (r² 0.90), in use.
    Needs contrast between filament and plate; on a bright plate the PA
    falls back to LiDAR strips, otherwise nothing is saved.
  - **Flow (experimental, opt-in)**: from the flow lines' width x the gap
    under them. It came out ~4 % low against a printed A/B check (measured
    108.6 %, 104 % looked best), so it is only reported by default;
    `FLOW_SAVE=1` saves it per filament (`lidar_flow`), and the new
    `_LIDAR_FLOW_APPLY` hook applies it as M221 at print start (and remembers
    `last_filament_name`). Without a saved flow the hook sets M221 S100.
- **`LIDAR_OFFSET_CALIBRATE` with the camera**: a 1-layer cross (arms ±7 mm)
  photographed (fill light + laser line at the same pose) and scanned at Z1
  and Z2; the camera's offsets are saved only when camera and LiDAR agree
  within 0.15 mm.
- **Camera webcam**: `lidar-direct.py --camera-port N` serves `/` and
  `/snapshot`, only while the camera switch is on (`--camera-switch`,
  default the new `LiDAR_Camera` output pin, `none` = always) and the LiDAR
  has been idle for 10 s; the fill light is off for every LiDAR request.
  `--webcam NAME` enables/disables that Moonraker webcam with the switch.
  `k1max/lidar-start.sh` starts it on port 8081.
- **Exposure per job**: with the helper, every scan, mesh, first-layer check
  and calibration first picks the lowest exposure that sees the whole laser
  line (black smooth PEI needs ~4000, textured PEI 1000), once per print or
  command; the first-layer check's strip width follows the usable line.
- `LIDAR_STATUS`: firmware, serial, module temperature; `SELFTEST=1` (the
  module's laser/light test), `AMBIENT=1` (points with the laser off).
- `LIDAR_STREAM_MODE MODE=0|1` (`lidar-direct.py --stream-mode`): continuous
  capture, ~10.7 instead of ~5 frames/s, its own `LIDAR_TIMING`; off by
  default.
- Module temperature before/after meshes and calibrations (console, and
  `lidar_runs.jsonl` next to `klippy.log`).
- Driver requests `ir_photo`, `set/get_stream_mode`, `get_sensor_temp`,
  `get_module_info`, `get_timings`, `self_test`, `ambient_points`;
  `lidar_serial.py`: `capture_jpeg` (0x56 type 2 only, pipelined 0x57),
  `get_cal_param`, `self_test`, `set/get_stream_mode`, decoded temperature.
- `[lidar] camera:` (default True when `x_dir` is -1): the photo analysis
  assumes the K1 Max mounting; without the helper or the camera
  `LIDAR_PA_CALIBRATE` and `LIDAR_OFFSET_CALIBRATE` keep their LiDAR-only
  prints. `LIDAR_SETUP_CHECK` reports the camera state.
- Tests: driver camera/stream/status requests against the fake module,
  `klipper/tests/test_pa_flow.py` (calibration steps, flow opt-in, placement,
  exposure per job, camera dispatch), `test_flow_apply.py`,
  `test_reduce.py`, `test_cross_lidar.py`; helper `test_pa_flow_plan.py`,
  `test_camera.py` and `test_service_pf.py` on photos from the K1 Max
  (`helper/tests/fixtures/camera/`; need numpy + PIL, CI installs them).

### Changed
- `LIDAR_PA_CALIBRATE` runs `LIDAR_PA_FLOW_CALIBRATE` when the helper and the
  camera are available (same parameters; X=/Y= mean the new layout).
- Scan rows are reduced by the **median** per 0.05 mm bin instead of the
  mean (a stray point from room light or gloss no longer pulls a bin).
- `cross_fit` (LiDAR offset cross) accepts arms that read low: thin white
  lines show ~0.1 of 0.2 mm (threshold 0.25 instead of 0.5 x the height).
- The test prints (PA, PA/flow, offset cross, Creality pattern) no longer
  need the printer homed before they start; START_PRINT homes.
- The driver refuses 0x58 (watchdog time, written to flash) and 0xAB.
- The helper's install scripts copy `lidar_camera.py` and `lidar_pa_flow.py`
  and warn when numpy/PIL are missing (only the camera calibrations need
  them).

### Fixed
- The plugin reconnects once when the driver was restarted since the last
  request (broken pipe) instead of failing the command.

### Known issues
- Camera flow is ~4 % low (opt-in only); camera calibrations tested on one
  plate and one filament.
- Stop-mode `LIDAR_SCAN` measures the exposure before moving to the scan XY;
  `LIDAR_PA_FLOW_STEP` PHOTOS/ANALYZE don't refuse when BARE never ran; the
  first-layer strip width assumes the usable line is centred; the LiDAR PA
  fallback hides its reason and accepts 4 lines / r² 0.5; old plain-number
  `lidar_flow` entries aren't clamped; `FLOWS=` isn't validated; an error in
  a step aborts the print without END_PRINT.
- The LiDAR reads thin translucent lines and single layers low (laser light
  spreads in the plastic): first-layer thickness of thin white PLA reads
  low, which is why PA and flow use the camera.

### Upgrading
- Helper: re-run `helper/install.sh` (or `host/install.sh --helper`) and
  `apt install python3-numpy python3-pil` for the camera calibrations.
- Printer config: add `_LIDAR_FLOW_APPLY {rawparams}` to START_PRINT next
  to `_FIRST_LAYER_OFFSET`, and the new `_LIDAR_PRINT_SCAN`,
  `LIDAR_PA_FLOW_CALIBRATION` and (optional) `[output_pin LiDAR_Camera]`
  from `lidar.example.cfg`.
- Run `LIDAR_TIMING` again if you switch `LIDAR_STREAM_MODE`.

## [0.3.2] - 2026-11-02

### Fixed
- **Switch macros on mainline Klipper.** `_LIDAR_START_PRINT_MESH`,
  `_LIDAR_PRINT_SCAN` and `_FIRST_LAYER_OFFSET` in `lidar.example.cfg` read
  the `[output_pin]` switches unguarded, so without virtual pins (mainline
  Klipper) they stopped with an undefined-object error. Without a pin they
  now read the saved variable (`sw_lidar_mesh`, `sw_first_layer_check`,
  `sw_first_layer_learn`; set with `SAVE_VARIABLE`), off when unset.
  Offline test: `klipper/tests/test_example_macros.py` renders the macros
  with jinja2 (CI installs it; the test skips without it).

## [0.3.1] - 2026-11-02

### Fixed
- **First-layer check read the middle of the area too thin.** The plate's
  change between the two scans can be bowl-shaped (40-60 um, middle lower,
  seen on the bare plate between the pads of 9-pad test prints on the K1
  Max), and only a plane was taken off, so parts in the middle read 25-38 um
  thinner than they were. Gaps inside an object's outline now count as bare
  plate too, and with bare plate in all four quarters of the objects' area
  a quadratic surface comes off instead (console: "curved by N um"; report:
  `drift.surface`, `drift.bowl`). A single solid part keeps the plane.
  Re-analysed test prints: pad spread 16-23 -> 6-8 um rms, median 0.192 ->
  0.204 mm for 0.200. Learned Z offsets (`First_Layer_Learn`) from before
  may carry the old error: clear `fl_zoffset` to let them learn again.

### Added
- `FIRST_LAYER_SCAN STRIP_SHIFT=` (test option): moves the scan strips
  sideways, to tell the laser line's own errors from the plate's.

## [0.3.0] - 2026-11-02

Support for any Cartesian/CoreXY Klipper printer with the module mounted on
the toolhead. The K1 Max behaves as before (offline regression tests) once
its settings are in its config, see **Upgrading** below.

### Added
- **Laser area from the printer**: meshes and calibrations use the travel
  limits shifted by the laser offset instead of K1 Max constants; optional
  `mesh_min` / `mesh_max` narrow it, `twist_y` / `ytwist_x` place the
  calibration lines, `mesh_max_count` limits mesh points.
- **Touch methods** (`touch:` in `[lidar]`): `load_cell` (Kalico load cells,
  as before), `probe` (any probe, moved over the spot), `gcode` (your own
  nozzle-touch command, contact Z from `touch_result: object:key`), `none`.
  `touch_temp` and `touch_wipe_gcode` replace the Simple AF macro lookups.
- `LIDAR_SETUP_CHECK`: driver, offsets, timing, service, reachable area and
  touch method, with what is missing.
- `host/`: systemd unit, udev rule and installer for the driver on a Linux
  Klipper host; `--helper` also installs the helper on the same host
  (`lidar-helper`, 127.0.0.1:7132; a K1's `k1-lidar` service on the same
  computer is left alone).
- `lidar_service.py --socket`: the helper talks to the driver's unix socket
  directly (no bridge) when both run on one host.
- Printer configs: `config/generic.cfg` (every option), `config/ender3v3se.cfg`,
  `k1max/lidar-k1max.cfg`; guide [install-other-printer](docs/install-other-printer.md).
- Tests: laser area, calibration defaults, touch methods, setup check, the
  helper's socket mode (`klipper/tests/test_any_printer.py`,
  `helper/tests/`).

### Changed
- `data_dir` / `gcode_dir` default to Klipper's `[virtual_sdcard] path`
  (`.../gcodes` and `.../lidar`); the K1 paths are the fallback.
- `klipper/lidar.example.cfg` is printer-neutral; the K1's `[lidar]`,
  `LIDAR_DRIVER` and shell command moved to `k1max/lidar-k1max.cfg`.
- `MESH_BENCHMARK`'s default area is the laser area, narrowed to where the
  probe reaches when `probe` is one of the methods (K1 Max: X5-270
  Y18.8-265, was X20-265 Y25-265); it checks the touch method before the
  first mesh.
- The pressure advance, offset and Creality test prints keep the K1 Max
  positions where they fit and are centred in the laser area elsewhere;
  every position is checked against the travel before anything starts.
- Probe touches start 2 mm above the probe's trigger height (load cells and
  `gcode`: 2 mm above the bed, as before); probe-mode calibration lines stay
  where the probe can reach.
- Levels 4-5 extend the area on the side the laser line looks past (right
  on a K1 Max, left when the laser sits right of the nozzle).
- `LIDAR_SETUP_CHECK` also reports `zero_reference_position`.
- Zero reference fallback: the laser area's centre (was the K1's 153, 173.3).

### Fixed
- Moving-scan meshes on mainline Klipper (`chelper` was imported the
  Kalico way only).

### Upgrading (K1 Max)
Add `k1max/lidar-k1max.cfg` (or its `mesh_min`, `mesh_max`, `twist_y`,
`ytwist_x`, `touch`, `touch_temp`, `touch_wipe_gcode` lines) to your config.
Without them meshes would use the whole travel, into the keep-out zones at
the back.

## [0.2.0] - 2026-11-02

### Added
- **LiDAR bed mesh** `LIDAR_BED_MESH`, levels 1-5 (quick 7x7 in ~20 s up to
  30x30 fitted to nozzle touches), `ADAPTIVE=1` for the printed objects,
  `FALLBACK=1` to fall back to the probe inside START_PRINT. Moving scans
  placed with Klipper's trapq; skew-aware reach checks.
- **Mesh calibration against the nozzle**: `LIDAR_TWIST_CALIBRATE` (X-beam
  twist, error along Y) and `LIDAR_GRID_CALIBRATE` (whole grid, after Axis
  Twist Mesh by sterling5241). Drift-robust fits, the bed heater off only per
  touch, runs stored per bed temperature and averaged.
- `MESH_COMPARE` and `MESH_BENCHMARK` (every method judged by a nozzle-touch
  grid, HTML report on the helper).
- **First-layer check** `FIRST_LAYER_SCAN`: bare plate before the print, first
  layer at layer 2, thickness map and per-object values, HTML report on the
  helper; optional Z offset learning per filament (`First_Layer_Learn`).
- `SKEW_SCAN` / `SKEW_CALIBRATE`: XY skew from a printed square, scanned
  twice and turned between the scans, no calipers.
- **K1 Max extras**, all in use on the development printer:
  `keep_out.py` (keep-out zones), `nozzle_wipe.py` (`NOZZLE_WIPE`,
  `NOZZLE_SCRUB`), `twist_touch.py` (`AXIS_TWIST_TOUCH_CALIBRATE`: probe and
  LiDAR twist from the same nozzle touches), True Zero touch macros and skew
  profile loading in `k1max/k1max-extras.cfg`.
- [What has been tested](docs/tested.md), with dates and results.
- Example config: switches (kept across restarts), saved PA restored at
  boot, print hooks (`_LIDAR_PRINT_START`,
  `_LIDAR_START_PRINT_MESH`, `_FIRST_LAYER_OFFSET`, `_FIRST_LAYER_AFTER`,
  `_LIDAR_PRINT_END`, `_LIDAR_PRINT_CANCEL`), `LIDAR_MESH`,
  `LIDAR_MESH_CALIBRATION`, `LIDAR_TWIST_CALIBRATION`, `MESH_BENCHMARK_RUN`.
- Docs: [command reference](docs/commands.md), [bed mesh](docs/bed-mesh.md),
  [first-layer check](docs/first-layer.md).
- `lidar-direct.py --version`; `LIDAR_STATUS` reports the version.
- Offline tests for the grid calibration and the nozzle wipe
  (`klipper/tests/`), CI on GitHub Actions, issue and pull request
  templates, CONTRIBUTING and SECURITY.

### Changed
- The open driver is now the default: `lidar-start.sh` starts
  `lidar-direct.py` unless `lidar_driver = 'middleware'` is saved; Creality's
  middleware is an explicit fallback only. `LIDAR_DRIVER` defaults to
  `direct`.
- Helper `install.sh` restarts the service, so re-running it after an update
  loads the new code.
- README and docs rewritten in a more formal tone.

### Fixed
- `CREALITY_PA_TEST` in the example config was missing its command line.

## [0.1.0] - 2026-10-13

First public release.

- Open driver for the Creality LiDAR module: serial protocol, authentication,
  init, frame streaming with `CLOCK_MONOTONIC_RAW` time stamps, the
  middleware's JSON socket API; refuses commands that damage or hang the
  module.
- Klipper/Kalico plugin: `LIDAR_SCAN` (dwell scans locked to the frame
  clock), `LIDAR_TIMING`, LiDAR pressure advance calibration,
  `LIDAR_OFFSET_CALIBRATE`, Creality's PA pattern.
- Helper service for a second computer, K1 Max boot script, protocol, pinout
  and install docs, offline tests with a fake module.

[Unreleased]: https://github.com/PainfulDirt/FreeTheBeam/compare/v0.4.0...HEAD
[0.4.0]: https://github.com/PainfulDirt/FreeTheBeam/compare/v0.3.2...v0.4.0
[0.3.2]: https://github.com/PainfulDirt/FreeTheBeam/compare/v0.3.1...v0.3.2
[0.3.1]: https://github.com/PainfulDirt/FreeTheBeam/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/PainfulDirt/FreeTheBeam/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/PainfulDirt/FreeTheBeam/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/PainfulDirt/FreeTheBeam/releases/tag/v0.1.0
