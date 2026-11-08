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
| `LIDAR_STATUS [SELFTEST=1] [AMBIENT=1]` | Checks that the driver answers; prints the module's firmware, serial and temperature, the version, offsets, where the work runs and the scans in memory. `SELFTEST=1`: the module's laser/fill-light brightness test. `AMBIENT=1`: points found with the laser off (room light read as a line). | |
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
| `LIDAR_BED_MESH [LEVEL=1-5 \| MODE=quick\|fast\|normal\|fine\|ultimate] [ADAPTIVE=0] [PROFILE=lidar] [FALLBACK=0]` | Bed mesh with the LiDAR. Levels 1-3 sample along columns while moving (or standing, `MOTION=dwell`), 4 scans strips with the whole laser line, 5 adds a 5x5 grid of nozzle touches. `ADAPTIVE=1` meshes only the printed objects. `FALLBACK=1` runs `BED_MESH_CALIBRATE` instead of failing. | service, homed; touch for level 5 |
| `LIDAR_TWIST_CALIBRATE [AXIS=X\|Y] [POINTS=] [PASSES=2] [APPLY=1]` | The LiDAR against nozzle touches on a line: `AXIS=X` the X-beam twist (`lidar_twist`), `AXIS=Y` the error along Y (`lidar_ytwist`). | service, homed, touch |
| `LIDAR_GRID_CALIBRATE [COUNT=7,5] [PASSES=2] [TOLERANCE=0.010] [APPLY=1]` | Nozzle taps (3 agreeing) and the LiDAR on every grid point; X curve and Y curve saved, the rest reported (`lidar_grid`). Method after [Axis Twist Mesh](https://github.com/sterling5241/axis-mesh). | service, homed, touch |
| `MESH_COMPARE A=<profile> [B=<profile>\|active] [STEP=10]` | A - B over the overlap: offset, plane, rms/max, worst points, provenance of each profile. | |
| `MESH_BENCHMARK [METHODS=probe,1,2,3,4,5,repeat2] [TRUTH=6]` | Runs several mesh methods on one area and judges them against a TRUTH x TRUTH grid of nozzle touches; JSON + HTML report on the helper. | service, homed, touch |

## First layer

See [first-layer.md](first-layer.md).

| Command | What it does | Needs |
|---|---|---|
| `FIRST_LAYER_SCAN PHASE=before\|after\|cancel` | `before`: scans the bare plate under the print's objects. `after` (at layer 2, or `FINAL=1` from END_PRINT): pauses, scans the same rows, reports the real first-layer thickness, resumes. `cancel`: forgets the pending check. | service, `[exclude_object]` |

## Calibration prints

| Command | What it does | Needs |
|---|---|---|
| `LIDAR_PA_CALIBRATE EXTRUDER_TEMP= BED_TEMP= [PA_START=0] [PA_STEP=0.01] [LINES=9] [X=130] [Y=100]` | With the helper and the camera: `LIDAR_PA_FLOW_CALIBRATE`. Otherwise writes the LiDAR pressure advance test as a print and starts it: bare-plate scan inside START_PRINT, test lines, cool down, scan, analysis. Saves the result as `lidar_pa`. | `_LIDAR_PRINT_SCAN` in START_PRINT |
| `LIDAR_PA_FLOW_CALIBRATE EXTRUDER_TEMP= [BED_TEMP=60] [FILAMENT_NAME=] [FILAMENT_TYPE=] [X=100] [Y=35] [PA_START=0] [PA_STEP=0.01] [FLOWS=0.90,1.00,1.10] [SPEED=40] [FLOW_SAVE=0]` | One print, measured by the LiDAR's camera: PA from line widths at speed changes (saved, `lidar_pa`), flow from flow-line widths x the gap (**experimental**: reported; saved per filament as `lidar_flow` only with `FLOW_SAVE=1`). Bare step inside START_PRINT (`LIDAR_PA_FLOW_STEP`). [Details](camera.md). | service (numpy + PIL), camera, `_LIDAR_PRINT_SCAN` in START_PRINT |
| `LIDAR_PA_FLOW_STEP STEP=BARE\|PHOTOS\|ANALYZE` | Internal steps of that print (only inside `lidar_pa_flow_test.gcode`). | |
| `LIDAR_PA_ANALYZE BASE= SCAN= [APPLY=1]` | Analyses a PA test scan against its bare-plate scan. | |
| `LIDAR_PA_CREALITY EXTRUDER_TEMP= BED_TEMP= [X=170] [Y=60]` | Prints Creality's PA pattern (20 values), read by eye. | |
| `LIDAR_OFFSET_CALIBRATE EXTRUDER_TEMP= BED_TEMP= [X=150] [Y=150] [Z1=3] [Z2=5] [APPLY=1]` | Prints a small cross and measures where the laser looks (X/Y offset and its change per mm of Z). With the helper and the camera: a 1-layer cross photographed and scanned at both heights, saved from the camera only when camera and LiDAR agree within 0.15 mm. Saved in `variables.cfg`, overrides `[lidar]`. | |
| `SKEW_SCAN NAME= ROT= [X0= Y0= X1= Y1=]` | Scans a printed square; scan it twice, turned ~90° between the scans. | service, homed |
| `SKEW_CALIBRATE NAMES=a,b [APPLY=1]` | XY skew from two `SKEW_SCAN`s, without calipers; `APPLY=1` sets it via `[skew_correction]`. | service |

## K1 Max extras

From `klipper/keep_out.py`, `klipper/nozzle_wipe.py`, `klipper/twist_touch.py`
(config in `k1max/k1max-extras.cfg`).

| Command | What it does | Needs |
|---|---|---|
| `KEEP_OUT_STATUS` | Lists the keep-out zones. Moves into them are refused. | |
| `NOZZLE_WIPE [TEMP=] [AFTER_TEMP=]` | Cleans the nozzle on the strip at the plate's back edge. | homed, load cells |
| `NOZZLE_SCRUB` | Short wipe for `nozzle_scrubber_gcode` (bad taps), no touches. | a previous `NOZZLE_WIPE` |
| `AXIS_TWIST_TOUCH_CALIBRATE` | Probe vs nozzle touch along X, saved as `[axis_twist_compensation]`; with the service the LiDAR's X table too. | homed, probe, load cells |

## Macros in `lidar.example.cfg`

| Macro | Wraps |
|---|---|
| `LIDAR_DRIVER [DRIVER=direct\|middleware]` | restarts the driver (`k1max/lidar-start.sh`) |
| `LIDAR_MESH [LEVEL=]` | `LIDAR_BED_MESH` at a default level |
| `LIDAR_MESH_CALIBRATION [GRID=7,5] [BED_TEMPS=60,70,100] [RESET=1]` | X twist + Y line (or a grid), optionally at several bed temperatures |
| `LIDAR_TWIST_CALIBRATION AXIS=X\|Y` | one calibration line, with nozzle heating and wipe |
| `MESH_BENCHMARK_RUN [BED_TEMP=70] [SOAK=10]` | heat, soak, `MESH_BENCHMARK` |
| `LIDAR_PA_CALIBRATION`, `LIDAR_PA_FLOW_CALIBRATION [FLOW_SAVE=1]`, `LIDAR_OFFSET_CALIBRATION`, `CREALITY_PA_TEST` | the calibration prints with default temperatures (PA/flow: FILAMENT_NAME defaults to the last printed filament) |
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
| `lidar_twist`, `lidar_ytwist`, `lidar_grid`, `lidar_cal_by_temp`, `lidar_zero_log` (data only) | `LIDAR_TWIST_CALIBRATE`, `LIDAR_GRID_CALIBRATE` |
| `lidar_pa` | `LIDAR_PA_CALIBRATE`, `LIDAR_PA_FLOW_CALIBRATE` / `LIDAR_PA_ANALYZE APPLY=1` |
| `lidar_flow` | `LIDAR_PA_FLOW_CALIBRATE FLOW_SAVE=1` (experimental), read by `_LIDAR_FLOW_APPLY` |
| `last_filament_name` | `_LIDAR_FLOW_APPLY` (default filament of `LIDAR_PA_FLOW_CALIBRATION`) |
| `lidar_stream_mode` | `LIDAR_STREAM_MODE` |
| `sw_lidar_camera` | the `LiDAR_Camera` switch |
| `fl_zoffset` | the first-layer check with `First_Layer_Learn` on |

Every run of `LIDAR_BED_MESH`, `LIDAR_TWIST_CALIBRATE`, `LIDAR_GRID_CALIBRATE`,
`LIDAR_TIMING`, `LIDAR_OFFSET_CALIBRATE`, `MESH_BENCHMARK` and
`SKEW_CALIBRATE` also prints the module temperature before and after and
appends a line to `lidar_runs.jsonl` next to `klippy.log` (warm-up drift
data; the module doesn't compensate for its temperature).
