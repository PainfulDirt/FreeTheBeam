# The LiDAR's camera

The module's image sensor (the one that sees the laser line) can also take
ordinary pictures under its own fill light: 1600x1200 JPEGs, ~19.4 µm per
pixel at scan height (Z3), a ~31 x 23 mm close-up next to the nozzle. Since
0.4.0 FreeTheBeam uses it in three ways:

1. a **webcam** for Fluidd/Mainsail (`lidar-direct.py --camera-port`),
2. **`LIDAR_PA_FLOW_CALIBRATE`**: pressure advance (and, experimentally,
   flow) from the width of printed lines,
3. **`LIDAR_OFFSET_CALIBRATE`**: where the laser looks, from photos of a
   1-layer cross, checked against a LiDAR scan.

Pictures are taken with 0x56 **type 2** (JPEG) only. Type 0 hangs the module
(see [protocol.md](protocol.md)); the driver never sends it.

## Why the camera

The LiDAR reads thin translucent lines too low: the laser light enters the
plastic, the whole line glows along its length and the spot becomes a ~45
row blob whose centre lands at or below the plate. On the development
printer a 0.2 mm white PLA line read 0.11-0.19 mm, a single white layer on
black PEI ~0.035 mm thin, and LiDAR PA fits of the same filament scattered
(0.0475 / 0.0302). The fill-light picture shows the same lines with sharp
edges, so their **width** repeats to 2-3 µm between pictures.

## Requirements

- The open driver, `lidar-direct.py` 0.4.0 (Creality's middleware has no
  `ir_photo`).
- The helper service with **numpy and PIL** (`apt install python3-numpy
  python3-pil`). The Klipper host never handles image bytes.
- `lidar_camera.py` and `lidar_pa_flow.py` next to `lidar_service.py` on the
  helper (both install scripts copy them).
- A module mounted like the K1 Max's: the analysis assumes picture columns
  along printer -X and rows along +Y, i.e. `x_dir: -1`. With another mount
  the camera calibrations are off by default; `camera: True` / `False` in
  `[lidar]` overrides it (`LIDAR_SETUP_CHECK` shows the state). Without the
  camera, `LIDAR_PA_CALIBRATE` prints the LiDAR-only PA test and
  `LIDAR_OFFSET_CALIBRATE` the LiDAR-only 2-layer cross, as in 0.3.x.
- **Contrast** between filament and plate for PA and flow. Tested: white
  matte PLA on black smooth PEI. When the lines are too faint (below 10 grey
  levels) the camera result is refused; on a bright plate the PA comes from
  LiDAR strips instead (see below), otherwise nothing is saved.

## Webcam

```
lidar-direct.py --camera-port 8081 [--camera-switch "output_pin LiDAR_Camera"]
                [--moonraker http://127.0.0.1:7125] [--webcam LiDAR]
```

- `http://<printer>:8081/` shows the camera, `/snapshot` returns one JPEG
  (`?res=0..3`, `light=0` without the fill light).
- Only while the switch is on: a Klipper object with a `value`, read from
  Moonraker every 3 s. The default is the `LiDAR_Camera` output pin from
  `lidar.example.cfg` (a virtual pin on Simple AF). Mainline Klipper without
  virtual pins: `--camera-switch none` (always on).
- A picture is only taken when nothing has used the LiDAR for 10 s;
  otherwise the last one comes back (header `X-LiDAR-Camera` says which). The
  fill light stays on while somebody watches and is off for every LiDAR
  request.
- To show it in Fluidd: *Settings -> Cameras -> Add*, service *Adaptive
  MJPEG-Streamer*, stream and snapshot URL `http://<printer>:8081/snapshot`,
  name `LiDAR`. With `--webcam LiDAR` the driver enables/disables that entry
  with the switch (add the entry first: Moonraker would otherwise create a
  new one).
- About one picture per second on the stock module firmware (resolution 0 =
  800x300).
- `k1max/lidar-start.sh` starts the driver with `--camera-port 8081`; on
  other hosts add the options to `host/lidar-direct.service`.

## Exposure per job

A dark or glossy sheet returns much less light: on black smooth PEI the
module found only -10..+9 mm of the 29 mm line at its default exposure
(1000), at 4000 all of it. With the helper, every LiDAR job (scan, mesh,
first-layer check, calibration; once per print or command) first has the
helper pick the lowest exposure of 1000/2000/4000/8000 that sees >= 95 % of
the line (`auto_exposure`). The measured height does not change with
exposure (bare plate, 1000-8000: within 2 µm). The first-layer check's strip
width follows the usable part of the line measured then.

## LIDAR_PA_FLOW_CALIBRATE

One print, started like the old PA test (`LIDAR_PA_FLOW_CALIBRATION` macro,
or `LIDAR_PA_CALIBRATE` when the camera is available):

1. Inside START_PRINT, after the mesh and nozzle touch and before the purge
   (`_LIDAR_PRINT_SCAN` -> `LIDAR_PA_FLOW_STEP STEP=BARE`): nozzle cooled to
   150 C, exposure for this plate, a plate photo (decides the photo exposure
   and whether the LiDAR fallback is possible), 4 bare LiDAR strips under the
   flow lines (their gap to the mesh).
2. 4 groups of 90/100/110 % flow lines (1.5 mm apart, 50 mm long), then 9 PA
   lines (PA_START + i * PA_STEP) with slow/fast segments.
3. After a cool-down the camera photographs the speed changes and the flow
   lines (`STEP=PHOTOS`), END_PRINT, then the helper analyses (`STEP=ANALYZE`,
   report `pf-<job>.html` in the helper's data folder).

- **PA** comes from how the lines bulge after slowing down and thin after
  speeding up: per line a score, a straight line through score vs PA crosses
  zero at the right PA (needs >= 7 lines, r² >= 0.6, inside the tested
  range). It is applied and saved (`lidar_pa`).
- **Flow** comes from the width of the flow lines x the gap under them
  (cross-section W*h - (1-pi/4)*h², h = 0.2 mm + the group's gap from the
  bare strips) against what was commanded. **Experimental, not saved by
  default**: it is reported, and only `FLOW_SAVE=1` saves it per filament
  (`lidar_flow`), which `_LIDAR_FLOW_APPLY` then applies as `M221` at print
  start (corrected when the first-layer check later learns a different Z
  offset for that filament). Keep the slicer's flow ratio at 1.0 for a
  filament with a saved flow.
- Without enough contrast the camera result is refused. On a bright plate
  (plate photo median > 60 grey levels) the bare step also scans the PA
  lines' area with the LiDAR, and the PA falls back to the LiDAR analysis.
- Placement: plate X-14..X+24, Y-2..Y+187 around `X=`/`Y=` (nozzle); with the
  laser's strips and photos the nozzle travels ~Y-15..Y+210. Without X/Y the
  K1 Max spot (X100 Y35) is used where it fits, else everything is centred in
  the travel; the console says where.

### Results on the development printer (K1 Max, 2026-11-08)

Black smooth PEI, white matte PLA at 210/60 C:

| | Result |
|---|---|
| Camera PA | **0.0361** (r² 0.90); trusted and in use. Two earlier test prints by hand: 0.040-0.048 (camera), LiDAR 0.0302 / 0.0475 |
| Camera flow | 0.921 (sd 0.019, 11 lines) -> M221 S108.6. A printed A/B check (three 20x20 one-layer pads at 100 / 104.3 / 108.6 %) looked best at **104 %**: the camera's absolute flow is ~4 % low. Hence experimental / opt-in |
| Width follows flow | +36..54 µm per +10 % flow inside a group (expected ~45) |
| Exposure | black smooth PEI 4000 (textured PEI 1000) |

## LIDAR_OFFSET_CALIBRATE with the camera

A 1-layer cross (0.2 mm, arms ±7 mm: taller steps confuse the module's line
finder, the line ends are dim on dark plates) is printed; at Z1 and Z2 the
camera takes a fill-light picture of the cross and a laser picture at the
same pose, and the helper finds where the profile's x = 0 lies on the laser
line relative to the cross. The LiDAR scans the cross too. The camera's
offsets are saved only when camera and LiDAR agree within 0.15 mm.
The LiDAR fit of the 1-layer cross accepts a cross that reads low (thin
white lines: ~0.08 of 0.2 mm).

## Known limitations (0.4.0)

- Tested on one K1 Max, one plate and one filament (white matte PLA on black
  smooth PEI).
- Camera flow is ~4 % low against a printed check: opt-in only.
- The camera analysis assumes the K1 Max mounting (`x_dir: -1`).
- Deferred review points, not fixed yet: stop-mode `LIDAR_SCAN` measures the
  exposure before moving to the scan XY; `LIDAR_PA_FLOW_STEP` PHOTOS/ANALYZE
  don't refuse when BARE never ran; the first-layer strip width assumes the
  usable line is centred; the LiDAR PA fallback hides its reason and accepts
  4 lines / r² 0.5; old plain-number `lidar_flow` entries aren't clamped;
  `FLOWS=` isn't validated; an error in a step aborts the print without
  END_PRINT.
