# First-layer check

The LiDAR measures the real thickness of the first layer of every print:
it scans the bare plate under the objects before the print, the same rows
again after layer 1, and the difference is the plastic. Test print on the
development printer: 0.194 mm measured for 0.200 mm sliced.

## How a check runs

1. **Before** (`FIRST_LAYER_SCAN PHASE=before`, from `_LIDAR_PRINT_SCAN` in
   START_PRINT, after the mesh and the nozzle touch, before the purge): the
   bare plate under the file's objects (from `exclude_object`) + an 8 mm
   margin, in strips 26 mm apart and rows 2 mm apart. Rows get coarser until
   one scan fits `MAX_TIME` (120 s). Bed mesh is off during the scan.
2. **After** (`PHASE=after`, from the slicer's layer-change G-code at layer 2,
   or `FINAL=1` from END_PRINT for one-layer prints): the print pauses,
   retracts, lets the nozzle cool by 30 °C (no ooze on the part), scans the
   same strips and rows, heats up, lays a short prime line 8 mm in front of
   the objects and resumes.
3. **Analysis** on the helper (`fl_analyze`): the thickness map (2 mm cells),
   median, p05-p95, tilt in X and Y, the plate drift between the two scans
   from the bare parts, and a value per object. Reported in the console, and
   as an HTML page on the helper (`firstlayer/fl-<id>.html`, history in
   `firstlayer/history.jsonl`).

Example console output:

```
First-layer check: 0.194mm measured for 0.200mm sliced (-0.006), p05-p95 0.171-0.214 over 8100 mm2; tilt X -3 Y -4 um/100mm; plate drift between the scans +2 um
  Z offset now +0.000; for 0.200mm it would be +0.006 (measurement clean)
```

A 100 mm part adds about 3 minutes to the print.

## Learning the Z offset

With the `First_Layer_Learn` switch on, each clean check moves the
filament's Z offset by half its error (at most 0.03 mm per print and 0.15 mm
in total). The offsets are stored per filament name in `fl_zoffset`
(`variables.cfg`) and applied by `_FIRST_LAYER_OFFSET` after the nozzle touch
at the next start. A check counts as clean with at least 100 cells, a p05-p95
spread of ≤ 0.15 mm, an error of ≤ 0.15 mm and a bare-plate drift of
≤ 30 µm rms.

## Setup

1. Helper service running and `service:` set in `[lidar]`.
2. From `klipper/lidar.example.cfg`: the switches `First_Layer_Check` /
   `First_Layer_Learn` and the hooks `_LIDAR_PRINT_START`, `_LIDAR_PRINT_SCAN`,
   `_FIRST_LAYER_OFFSET`, `_FIRST_LAYER_AFTER`, `_LIDAR_PRINT_END`,
   `_LIDAR_PRINT_CANCEL`. Where each one goes is in the comments there and in
   [install-k1max.md](install-k1max.md).
3. Slicer:
   - object labels on (`exclude_object`),
   - START_PRINT gets the first-layer height and the filament
     (OrcaSlicer: `FIRST_LAYER_HEIGHT=[initial_layer_print_height]
     FILAMENT_TYPE={filament_type[initial_extruder]}
     FILAMENT_NAME="{filament_settings_id[initial_extruder]}"`),
   - layer-change G-code: `{if layer_num == 1}_FIRST_LAYER_AFTER{endif}`
     (on its own line).

## Limitations

- Thin lines (parts under ~40 mm) read slightly low on PLA: the laser sheet
  has a thickness and a narrow line ramps the profile. Widths and areas are
  reliable, single peaks are not.
- The scan needs the objects inside the LiDAR's reach (see
  [bed-mesh.md](bed-mesh.md)); outside it the check is skipped and the print
  runs normally.
