# Changelog

All notable changes to FreeTheBeam. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/) (0.x: interfaces may still change
between minor versions).

## [Unreleased]

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

[Unreleased]: https://github.com/PainfulDirt/FreeTheBeam/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/PainfulDirt/FreeTheBeam/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/PainfulDirt/FreeTheBeam/releases/tag/v0.1.0
