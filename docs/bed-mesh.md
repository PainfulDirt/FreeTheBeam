# LiDAR bed mesh

`LIDAR_BED_MESH` builds a normal Klipper bed mesh profile from LiDAR
readings instead of probe touches. On the development printer it matches the
Microprobe's accuracy and is 5-10x faster, so it is that printer's
START_PRINT mesh since 2026-11-01. It needs the helper service.

## Levels

| Level | Name | How | Points | Time (full bed, K1 Max) |
|---|---|---|---|---|
| 1 | quick | frames along columns, moving 150 mm/s | 7x7 | ~20 s |
| 2 | fast | moving 120 mm/s | 12x12 | ~40 s |
| 3 | normal | moving 60 mm/s | 20x20 | ~1.8 min |
| 4 | fine | strips side by side, the whole 28 mm laser line | 30x30 | ~4.9 min |
| 5 | ultimate | level 4 + a 5x5 grid of nozzle touches, fitted to them | 30x30 | ~13.5 min |

Moving frames are kept only while the toolhead is at full speed (or
standing), and each is placed with Klipper's trapq over its exposure window.
`MOTION=dwell` stands at every point instead (levels 1-3). `ADAPTIVE=1`
meshes the printed objects only (+5 mm), always including the
`zero_reference_position`: the LiDAR measures distance, not absolute height,
so the mesh is made 0 where Z was homed or touched.

Reach on a K1 Max: X 5-269 x Y 5-265 (to X 283 with levels 4-5). The laser
looks 36 mm left of and 21 mm in front of the nozzle, so the right ~30 mm and
the back ~35 mm are out of view. With `FALLBACK=1` (the START_PRINT hook
uses it) anything the LiDAR can't do (objects out of reach, service down, a
failed scan) falls back to `BED_MESH_CALIBRATE` and the print goes on.

Every mesh is also kept on the helper (`meshes/`, `heightmap-*.json`).

## Calibration

The LiDAR sits in front of the nozzle, so anything that tips the toolhead
reads as height. Two corrections are measured against the nozzle itself
(load-cell touches) and applied to every LiDAR mesh:

- **X-beam twist** (`lidar_twist`): the gantry tips the toolhead forwards or
  backwards by an amount that changes along X. On the development printer:
  about +0.1 to -0.1 mm across the bed.
- **Error along Y** (`lidar_ytwist`): a curve rising ~50 µm front to back on a
  70 °C bed. The cause is not known; the correction is measured, not modelled.

`LIDAR_MESH_CALIBRATION` (macro, ~16 min) measures both with
`LIDAR_TWIST_CALIBRATE AXIS=X` and `AXIS=Y`. `GRID=7,5` measures a whole grid
instead (~25 min): at every point the nozzle taps until 3 readings agree,
then the LiDAR looks at the same spot; the differences split into an X curve,
a Y curve and a remainder (reported, with suspect points). The method follows
[Axis Twist Mesh](https://github.com/sterling5241/axis-mesh) by sterling5241,
used with the author's permission.

Both calibrations interleave passes in alternating direction and fit one
linear drift, because the toolhead warms up during the run (~6 µm/min seen).
The bed heater is switched off only for each touch, so the bed stays at print
temperature.

Every run is stored with its bed temperature in `lidar_cal_by_temp`; meshes
use the mean of all runs on a hot (≥ 45 °C) or cold bed. Grids at 60, 70 and
100 °C agreed within the run-to-run scatter (10-14 µm), so more runs give
better tables. `RESET=1` forgets the stored runs: do that after changing the
plate, the LiDAR mount or the gantry.

## Measured accuracy

One K1 Max, glass bed + textured PEI, 2026-10-29 to 2026-11-01.

**Against 36 nozzle touches** (`MESH_BENCHMARK`, bed 70 °C, grid tables):

| Method | rms vs nozzle | Time |
|---|---|---|
| LiDAR level 1 | 18.5 µm | 21 s |
| LiDAR level 2 | 18.3 µm | 35 s |
| LiDAR level 3 | 18.4 µm | 95 s |
| Microprobe 12x12 | 18.9 µm | 764 s |

Level 2 repeated: 8.3 µm rms between two meshes.

**Printed** (9 pads 30x30 mm at X/Y 60, 150, 240, PLA, bed 70 °C, first-layer
thickness measured by the first-layer check, same file, only the mesh
changed):

| Mesh | Mesh time | Pads, rms around their mean |
|---|---|---|
| LiDAR level 3, fresh tables | 80 s | 16.8 µm |
| Microprobe, fresh twist table | ~13 min | 17.7 µm |
| LiDAR level 3, 3-day-old X table | 80 s | 21.7 µm |

Stale tables show up (an 18 µm X shape with the old table): rerun the
calibration after hardware changes.

## Comparing and benchmarking

- `MESH_COMPARE A=lidar B=default` reports offset, plane, rms and the worst
  points between two profiles, and what made each of them.
- `MESH_BENCHMARK_RUN` (macro) heats and soaks the bed, then runs
  `MESH_BENCHMARK`: every method on one area, judged by a grid of nozzle
  touches, with a JSON + HTML report on the helper (`benchmarks/`).

## Limitations

- Needs the helper service; the printer alone is too slow for it.
- Needs load cells (`[load_cell_probe]`) for the calibrations and level 5.
  Without them levels 1-4 still run, but without twist/Y correction.
- Not absolute: Z=0 comes from homing / your nozzle touch.
- Textured PEI scatters ±13-20 µm per spot between what the nozzle touches
  and what the laser sees. That is why only smooth curves are fitted, never
  per-spot tables.
