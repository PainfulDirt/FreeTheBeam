# G-code command reference

Commands registered by `klipper/lidar.py`. The authoritative, complete
parameter list is the comment block at the top of that file; this page is
the overview. Defaults are for a K1 Max.

**Needs** column: *service* = the helper service (`service:` in `[lidar]`,
see [install-helper.md](install-helper.md)); *load cells* = a
`[load_cell_probe]` for nozzle touches; *homed* = `G28` first.

## Status and timing

| Command | What it does | Needs |
|---|---|---|
| `LIDAR_STATUS` | Checks that the driver answers; prints the version, offsets, where the work runs and the scans in memory. | |
| `LIDAR_TIMING [Z=3] [DZ=1.5] [SPEED=5] [CYCLES=15] [LEVELS=0,1] [APPLY=1]` | Measures the frame period and the time window a frame covers (slow Z moves over a flat plate) and saves them as `lidar_timing` for dwell scans. Run once per driver. | service, homed |

## Scanning

| Command | What it does | Needs |
|---|---|---|
| `LIDAR_SCAN NAME= X= Y0= Y1= [STEP=0.4] [Z=3] [MODE=dwell\|stop] [CAPTURES=3]` | Scans a strip; X/Y are where the laser looks, one profile per Y row. `dwell`: one frame per row, rows locked to the frame clock (~0.2 s per row). `stop`: stop and average `CAPTURES` frames per row (~1.3 s per row). | homed; service for dwell |

## Bed mesh

See [bed-mesh.md](bed-mesh.md).

| Command | What it does | Needs |
|---|---|---|
| `LIDAR_BED_MESH [LEVEL=1-5 \| MODE=quick\|fast\|normal\|fine\|ultimate] [ADAPTIVE=0] [PROFILE=lidar] [FALLBACK=0]` | Bed mesh with the LiDAR. Levels 1-3 sample along columns while moving (or standing, `MOTION=dwell`), 4 scans strips with the whole laser line, 5 adds a 5x5 grid of nozzle touches. `ADAPTIVE=1` meshes only the printed objects. `FALLBACK=1` runs `BED_MESH_CALIBRATE` instead of failing. | service, homed; load cells for level 5 |
| `LIDAR_TWIST_CALIBRATE [AXIS=X\|Y] [POINTS=] [PASSES=2] [APPLY=1]` | The LiDAR against nozzle touches on a line: `AXIS=X` the X-beam twist (`lidar_twist`), `AXIS=Y` the error along Y (`lidar_ytwist`). | service, homed, load cells |
| `LIDAR_GRID_CALIBRATE [COUNT=7,5] [PASSES=2] [TOLERANCE=0.010] [APPLY=1]` | Nozzle taps (3 agreeing) and the LiDAR on every grid point; X curve and Y curve saved, the rest reported (`lidar_grid`). Method after [Axis Twist Mesh](https://github.com/sterling5241/axis-mesh). | service, homed, load cells |
| `MESH_COMPARE A=<profile> [B=<profile>\|active] [STEP=10]` | A - B over the overlap: offset, plane, rms/max, worst points, provenance of each profile. | |
| `MESH_BENCHMARK [METHODS=probe,1,2,3,4,5,repeat2] [TRUTH=6]` | Runs several mesh methods on one area and judges them against a TRUTH x TRUTH grid of nozzle touches; JSON + HTML report on the helper. | service, homed, load cells |

## First layer

See [first-layer.md](first-layer.md).

| Command | What it does | Needs |
|---|---|---|
| `FIRST_LAYER_SCAN PHASE=before\|after\|cancel` | `before`: scans the bare plate under the print's objects. `after` (at layer 2, or `FINAL=1` from END_PRINT): pauses, scans the same rows, reports the real first-layer thickness, resumes. `cancel`: forgets the pending check. | service, `[exclude_object]` |

## Calibration prints

| Command | What it does | Needs |
|---|---|---|
| `LIDAR_PA_CALIBRATE EXTRUDER_TEMP= BED_TEMP= [PA_START=0] [PA_STEP=0.01] [LINES=9] [X=130] [Y=100]` | Writes a pressure advance test as a print and starts it: bare-plate scan inside START_PRINT, test lines, cool down, scan, analysis. Saves the result as `lidar_pa`. | `_LIDAR_PRINT_SCAN` in START_PRINT |
| `LIDAR_PA_ANALYZE BASE= SCAN= [APPLY=1]` | Analyses a PA test scan against its bare-plate scan. | |
| `LIDAR_PA_CREALITY EXTRUDER_TEMP= BED_TEMP= [X=170] [Y=60]` | Prints Creality's PA pattern (20 values), read by eye. | |
| `LIDAR_OFFSET_CALIBRATE EXTRUDER_TEMP= BED_TEMP= [X=150] [Y=150] [Z1=3] [Z2=5] [APPLY=1]` | Prints a small cross and measures where the laser looks (X/Y offset and its change per mm of Z). Saved in `variables.cfg`, overrides `[lidar]`. | homed |
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
| `LIDAR_PA_CALIBRATION`, `LIDAR_OFFSET_CALIBRATION`, `CREALITY_PA_TEST` | the calibration prints with default temperatures |
| `_LIDAR_SWITCHES` (+ restore/save delayed G-code) | keeps the switches across restarts, restores `lidar_pa` at boot |
| `_LIDAR_PRINT_START`, `_LIDAR_START_PRINT_MESH`, `_LIDAR_PRINT_SCAN`, `_FIRST_LAYER_OFFSET`, `_FIRST_LAYER_AFTER`, `_LIDAR_PRINT_END`, `_LIDAR_PRINT_CANCEL` | hooks for your START_PRINT, layer change, END_PRINT and CANCEL_PRINT |

## Saved variables (`variables.cfg`)

| Variable | Written by |
|---|---|
| `lidar_driver` | `LIDAR_DRIVER` |
| `lidar_timing` | `LIDAR_TIMING` |
| `lidar_x_offset`, `lidar_y_offset`, `lidar_x_per_z`, `lidar_y_per_z` | `LIDAR_OFFSET_CALIBRATE` |
| `lidar_twist`, `lidar_ytwist`, `lidar_grid`, `lidar_cal_by_temp`, `lidar_zero_log` (data only) | `LIDAR_TWIST_CALIBRATE`, `LIDAR_GRID_CALIBRATE` |
| `lidar_pa` | `LIDAR_PA_CALIBRATE` / `LIDAR_PA_ANALYZE APPLY=1` |
| `fl_zoffset` | the first-layer check with `First_Layer_Learn` on |
