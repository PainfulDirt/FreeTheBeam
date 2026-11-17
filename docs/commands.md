# G-code command reference

Commands registered by `klipper/lidar.py`. The authoritative, complete
parameter list is the comment block at the top of that file; this page is
the overview. Numbers in the descriptions are a K1 Max's; the defaults
are in the options table below.

**Needs** column: *service* = the helper service (`service:` in `[lidar]`,
see [install-helper.md](install-helper.md)); *touch* = a touch method
(`touch:` in `[lidar]`, below); *homed* = `G28` first.

## Status and timing

| Command | What it does | Needs |
|---|---|---|
| `LIDAR_STATUS [SELFTEST=1] [AMBIENT=1]` | Checks that the driver answers; prints the module's firmware, serial and temperature, the version, offsets, where the work runs (helper, the service on the printer itself, or the plugin), the scans in memory and the plate (`LIDAR_PLATE`; a warning when it has no calibration runs). `SELFTEST=1`: the module's laser/fill-light brightness test (run it at scan height over the plate: from far above, the laser reads weak; a failed test says so). `AMBIENT=1`: points found with the laser off (room light read as a line). | |
| `LIDAR_STREAM_MODE [MODE=0\|1]` | `MODE=1`: the module captures continuously (~10.7 instead of ~5 frames/s, frames ~0.1 s older); saved as `lidar_stream_mode`, each mode has its own `LIDAR_TIMING`. Not the default: on the K1 Max dwell scans got only ~15 % faster and meshes ~12 µm rms noisier. | lidar-direct.py 0.4.0 |
| `LIDAR_SETUP_CHECK` | What this printer's setup can do: driver, offsets measured or not, timing, service, the area the LiDAR reaches (levels 1-3 and 4-5), `zero_reference_position`, the touch method, skew/keep-out; ends with what is missing. | |
| `LIDAR_TIMING [Z=3] [DZ=1.5] [SPEED=5] [CYCLES=15] [LEVELS=0,1] [APPLY=1]` | Measures the frame period and the time window a frame covers (slow Z moves over a flat plate) and saves them as `lidar_timing` for dwell scans. Run once per driver. | service, homed |

## Scanning

| Command | What it does | Needs |
|---|---|---|
| `LIDAR_SCAN NAME= X= Y0= Y1= [STEP=0.4] [Z=3] [MODE=dwell\|stop] [CAPTURES=3]` | Scans a strip; X/Y are where the laser looks, one profile per Y row. `dwell`: one frame per row, rows locked to the frame clock (~0.2 s per row). `stop`: stop and average `CAPTURES` frames per row (~1.3 s per row). | homed; service for dwell |

## Bed mesh

See [bed-mesh.md](bed-mesh.md).

| Command | What it does | Needs |
|---|---|---|
| `LIDAR_BED_MESH [LEVEL=1-5 \| MODE=quick\|fast\|normal\|fine\|ultimate] [ADAPTIVE=0] [PROFILE=lidar] [FALLBACK=0]` | Bed mesh with the LiDAR. Levels 1-3 sample along columns while moving (or standing, `MOTION=dwell`), 4 scans strips with the whole laser line, 5 adds a 5x5 grid of nozzle touches. A moving column that got too few frames (a Wi-Fi or USB hiccup: under 30 % of the median column, at most 2 columns) is scanned again before the mesh is made. `ADAPTIVE=1` meshes only the printed objects (at least 20 mm a side, inside what the LiDAR sees, the zero reference included). `FALLBACK=1` runs `BED_MESH_CALIBRATE` instead of failing (also when the objects reach past the LiDAR's area). Uses the calibration runs of the current plate (`LIDAR_PLATE`). | service, homed; touch for level 5 |
| `LIDAR_TWIST_CALIBRATE [AXIS=X\|Y] [POINTS=] [PASSES=2] [APPLY=1]` | The LiDAR against nozzle touches on a line: `AXIS=X` the X-beam twist (`lidar_twist`), `AXIS=Y` the error along Y (`lidar_ytwist`). | service, homed, touch |
| `LIDAR_GRID_CALIBRATE [COUNT=7,5] [PASSES=2] [TOLERANCE=0.010] [APPLY=1]` | Nozzle taps (3 agreeing) and the LiDAR on every grid point; X curve and Y curve saved, the rest reported (`lidar_grid`). Method after [Axis Twist Mesh](https://github.com/sterling5241/axis-mesh). Runs are tagged with the current plate. | service, homed, touch |
| `LIDAR_PLATE [NAME=] [RESET_RUNS=1]` | Which build plate is on the bed (`NAME=`: 1-32 of a-z, 0-9, `_`, e.g. `black_pei`; saved as `lidar_plate`). Calibration runs (`LIDAR_TWIST_CALIBRATE`, `LIDAR_GRID_CALIBRATE`) are tagged with it and meshes use only its runs: runs made before plates were tagged are used where the plate has none, other plates' never. Without parameters it lists the plates and their runs (bed temperatures, count, newest). `RESET_RUNS=1` forgets the current plate's runs and the untagged ones, the other plates' stay (all runs when no plate is set; `LIDAR_MESH_CALIBRATION RESET=1`). Set the plate before calibrating a new one. | |
| `MESH_COMPARE A=<profile> [B=<profile>\|active] [STEP=10]` | A - B over the overlap: offset, plane, rms/max, worst points, provenance of each profile. | |
| `MESH_BENCHMARK [METHODS=probe,1,2,3,4,5,repeat2] [TRUTH=6]` | Runs several mesh methods on one area and judges them against a TRUTH x TRUTH grid of nozzle touches; JSON + HTML report on the helper. | service, homed, touch |

## First layer

See [first-layer.md](first-layer.md).

| Command | What it does | Needs |
|---|---|---|
| `FIRST_LAYER_SCAN PHASE=before\|after\|cancel` | `before`: scans the bare plate under the print's objects. `after` (at layer 2, or `FINAL=1` from END_PRINT): pauses, scans the same rows, reports the real first-layer thickness, then primes the nozzle (`TIP=5` mm to refill it, a `PRIME=20` mm purge line in front of the objects, else beside or behind them; `TIP=0 PRIME=0` = none) and resumes. The fitted line ramp is kept per filament (`fl_line_ramp`) for parts too small to fit it. `cancel`: forgets the pending check. When the helper or the driver fails the print goes on without the check. | service, `[exclude_object]` |
| `FIRST_LAYER_BIAS [VALUE=] [CLEAR=1] [PLATE=] [FILAMENT=]` | The first-layer check's LiDAR bias (LiDAR minus true height, mm, at most ±0.15) per plate and filament, taken off before the check reports or learns ([first-layer.md](first-layer.md#lidar-bias-per-platefilament)). `VALUE=` sets it for the current plate (`LIDAR_PLATE`) and the last filament (`last_filament_name`), `PLATE=` / `FILAMENT=` choose others, `CLEAR=1` removes it, no parameters: lists all. | a plate set |

## Calibration prints

| Command | What it does | Needs |
|---|---|---|
| `LIDAR_PA_CALIBRATE EXTRUDER_TEMP= BED_TEMP= [METHOD=IR\|LIDAR\|BOTH] [PA_START=0] [PA_STEP=0.01] [LINES=9] [X=130] [Y=100]` | With the helper (not in local mode): `LIDAR_PA_FLOW_CALIBRATE TESTS=PA PA_METHOD=<METHOD>` (default IR with the camera on, else LIDAR). Otherwise writes the LiDAR pressure advance test as a print and starts it: bare-plate scan inside START_PRINT, test lines, cool down, scan, analysis. Saves the result as `lidar_pa`. | `_LIDAR_PRINT_SCAN` in START_PRINT |
| `LIDAR_PA_FLOW_CALIBRATE [TESTS=PA,FLOW] [PA_METHOD=IR] [FLOW_METHOD=BOTH] EXTRUDER_TEMP= [BED_TEMP=60] [FILAMENT_NAME=] [FILAMENT_TYPE=] [X=100] [Y=35] [PA_START=0] [PA_STEP=0.01] [FLOWS=0.92,0.96,1.00,1.04,1.08,1.12] [SPEED=40]` | One print with PA lines and/or flow bands. PA: 9 lines, from line widths at speed changes (camera, `IR`) or the lines' cross-sections in LiDAR strips (`LIDAR`). Flow (**experimental**): 6 one-layer bands at 92..112 %, the first closed and smooth one, judged by the camera (gaps and ridge pattern) or LiDAR strips (bare before, printed after; provisional rule). `BOTH` measures both ways, saves nothing and asks (Fluidd/Mainsail prompt) which to save; `IR`/`LIDAR` saves `lidar_pa` / `lidar_flow` (per filament) directly. Bare step inside START_PRINT (`LIDAR_PA_FLOW_STEP`). [Details](camera.md). | service with numpy + PIL (not local mode), camera for `IR`, `_LIDAR_PRINT_SCAN` in START_PRINT |
| `LIDAR_PA_SAVE METHOD=IR\|LIDAR [FORCE=1]`, `LIDAR_FLOW_SAVE METHOD=IR\|LIDAR [FORCE=1]` | Save the PA / the flow of the last benchmark (`BOTH`) by that method. Refused unless `FORCE=1` when a later calibration saved that value directly, when it was already saved from this benchmark, or when the benchmark was for another filament than the last printed one; the prompt comes back with what is still unsaved. | a benchmark |
| `LIDAR_PA_FLOW_STEP STEP=BARE\|PHOTOS\|ANALYZE` | Internal steps of that print (only inside `lidar_pa_flow_test.gcode`). | |
| `LIDAR_PA_ANALYZE BASE= SCAN= [APPLY=1]` | Analyses a PA test scan against its bare-plate scan. | |
| `LIDAR_PA_CREALITY EXTRUDER_TEMP= BED_TEMP= [X=170] [Y=60]` | Prints Creality's PA pattern (20 values), read by eye. | |
| `LIDAR_OFFSET_CALIBRATE EXTRUDER_TEMP= BED_TEMP= [X=150] [Y=150] [Z1=3] [Z2=5] [APPLY=1]` | Prints a small cross and measures where the laser looks (X/Y offset and its change per mm of Z). With the helper and the camera: a 1-layer cross photographed and scanned at both heights, saved from the camera when camera and LiDAR agree within 0.15 mm, or, when the LiDAR can't see the thin cross, when the camera is within 1 mm of the current offsets. Saved in `variables.cfg`, overrides `[lidar]`. | |
| `SKEW_SCAN NAME= ROT= [X0= Y0= X1= Y1=]` | Scans a printed square; scan it twice, turned ~90° between the scans. | service, homed |
| `SKEW_CALIBRATE NAMES=a,b [APPLY=1]` | XY skew from two `SKEW_SCAN`s, without calipers; `APPLY=1` sets it via `[skew_correction]`. | service |

## Macros in `lidar.example.cfg`

| Macro | Wraps |
|---|---|
| `LIDAR_DRIVER [DRIVER=direct\|middleware]` | restarts the driver, the bridge and the local service (`k1max/lidar-start.sh restart`) |
| `LIDAR_MESH [LEVEL=]` | `LIDAR_BED_MESH` at a default level |
| `LIDAR_MESH_CALIBRATION [GRID=7,5 \| AXIS=X\|Y] [BED_TEMPS=60,70,100] [RESET=1]` | X twist + Y line (or a grid, or one line), optionally at several bed temperatures; `RESET=1` = `LIDAR_PLATE RESET_RUNS=1` (replaces 0.4.0's `LIDAR_TWIST_CALIBRATION`) |
| `MESH_BENCHMARK_RUN [BED_TEMP=70] [SOAK=10]` | heat, soak, `MESH_BENCHMARK` |
| `LIDAR_PA_CALIBRATION [METHOD=IR\|LIDAR\|BOTH]`, `LIDAR_FLOW_CALIBRATION [METHOD=BOTH\|IR\|LIDAR]`, `LIDAR_OFFSET_CALIBRATION`, `CREALITY_PA_TEST` | the calibration prints with default temperatures (PA/flow: FILAMENT_NAME defaults to the last printed filament; replace 0.4.0's `LIDAR_PA_FLOW_CALIBRATION`) |
| `_LIDAR_SWITCHES` (+ restore/save delayed G-code) | keeps the switches across restarts, restores `lidar_pa` at boot |
| `_LIDAR_PRINT_START`, `_LIDAR_START_PRINT_MESH`, `_LIDAR_PRINT_SCAN`, `_FIRST_LAYER_OFFSET`, `_LIDAR_FLOW_APPLY`, `_FIRST_LAYER_AFTER`, `_LIDAR_PRINT_END`, `_LIDAR_PRINT_CANCEL` | hooks for your START_PRINT, layer change, END_PRINT and CANCEL_PRINT |

## `[lidar]` options

Full list with comments: [`config/generic.cfg`](../config/generic.cfg).

| Option | Default | What |
|---|---|---|
| `x_offset`, `y_offset`, `offset_z`, `x_per_z`, `y_per_z` | -36, -21.2, 3, 0, 0 | where the laser line centre is relative to the nozzle (`LIDAR_OFFSET_CALIBRATE` saves measured values, which win) |
| `x_dir` | -1 | the profile runs towards printer -X (-1) or +X (1) |
| `scan_z` | 3 | nozzle height for scans |
| `service` | none | `host:port` of the helper service |
| `service_local` | False | True: the service runs on the printer itself without numpy (K1 Max local mode, [install-helper.md](install-helper.md#on-the-printer-itself-local-mode)): camera and flow-band calibrations refused, analysis timeouts x3 |
| `mesh_min`, `mesh_max` | none | narrow the laser area (default: what the laser reaches within the travel) |
| `twist_y`, `ytwist_x` | area Y centre; 1/3 and 2/3 of its X | `LIDAR_TWIST_CALIBRATE` lines |
| `mesh_max_count` | 30 | most mesh points per axis |
| `touch` | auto | `load_cell`, `probe`, `gcode` or `none`; auto: load_cell, else probe, else none |
| `touch_gcode`, `touch_result` | `NOZZLE_TOUCH`, none | `touch: gcode`: the command, and the status value `object:key` with the contact Z |
| `touch_temp`, `touch_wipe_gcode` | 150, none | nozzle temperature for touches; cleaning command before them |
| `data_dir`, `gcode_dir` | next to `[virtual_sdcard] path` | where scans and generated test prints go |
| `dwell_settle`, `dwell_margin` | 0.05, 0.015 | dwell scan timing (K1 Max: 0.03, 0.01) |
| `camera` | True when `x_dir` is -1 | camera calibrations with the helper ([camera.md](camera.md)); the analysis assumes the K1 Max mounting |

## Saved variables (`variables.cfg`)

| Variable | Written by |
|---|---|
| `lidar_driver` | `LIDAR_DRIVER` |
| `lidar_timing` | `LIDAR_TIMING` |
| `lidar_x_offset`, `lidar_y_offset`, `lidar_x_per_z`, `lidar_y_per_z` | `LIDAR_OFFSET_CALIBRATE` |
| `lidar_twist`, `lidar_ytwist`, `lidar_grid`, `lidar_cal_by_temp` (runs tagged with their plate), `lidar_zero_log` (data only) | `LIDAR_TWIST_CALIBRATE`, `LIDAR_GRID_CALIBRATE` |
| `lidar_plate` | `LIDAR_PLATE NAME=` |
| `lidar_pa` | `LIDAR_PA_CALIBRATE`, `LIDAR_PA_FLOW_CALIBRATE`, `LIDAR_PA_SAVE` / `LIDAR_PA_ANALYZE APPLY=1` |
| `lidar_flow` | `LIDAR_PA_FLOW_CALIBRATE FLOW_METHOD=IR\|LIDAR`, `LIDAR_FLOW_SAVE` (experimental), read by `_LIDAR_FLOW_APPLY` |
| `fl_line_ramp` | the first-layer check (fitted line ramp per filament) |
| `last_filament_name` | `_LIDAR_FLOW_APPLY` (default filament of the LiDAR calibrations) |
| `lidar_stream_mode` | `LIDAR_STREAM_MODE` |
| `sw_lidar_camera` | the `LiDAR_Camera` switch |
| `fl_zoffset` | the first-layer check with `First_Layer_Learn` on |
| `fl_bias` | `FIRST_LAYER_BIAS` |

Every run of `LIDAR_BED_MESH`, `LIDAR_TWIST_CALIBRATE`, `LIDAR_GRID_CALIBRATE`,
`LIDAR_TIMING`, `LIDAR_OFFSET_CALIBRATE`, `MESH_BENCHMARK` and
`SKEW_CALIBRATE` also prints the module temperature before and after and
appends a line to `lidar_runs.jsonl` next to `klippy.log` (warm-up drift
data; the module doesn't compensate for its temperature).
