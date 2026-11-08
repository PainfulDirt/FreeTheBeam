# What has been tested

Everything in this release runs on the development printer: a **Creality K1
Max** with Simple AF (Pellcorp) + Kalico, Microprobe V2, load cells, glass bed
+ textured PEI (black smooth PEI for the camera work, since ~2026-11-03),
LiDAR firmware 1.0.13 (the camera work: the developer's own patched module
firmware, see below), helper service on a Raspberry Pi 4 (1 GB). Dates are when a feature was verified; "in use" means it has run in
normal prints since then.

## Driver and scanning

| Feature | Tested | Result |
|---|---|---|
| Open driver `lidar-direct.py` | 2026-10-12, in use; the only driver since 2026-11-01 | Same points as Creality's middleware, same scans; 5-min soak with 0 errors; ~5 % CPU, 12.5 MB; ~15 % more frames/s |
| Authentication and init without vendor software | 2026-10-12, at every boot since | |
| Frame streaming + `LIDAR_TIMING` | 2026-10-12 | Frame period 199 ms; frame window measured per driver |
| Dwell scans locked to the frames (`LIDAR_SCAN`) | 2026-10-12 | 51 rows in 12-14 s (was 20-21 s); 0 rows missed in 3 runs; matches a stop-mode scan within the scan noise |
| Helper service + bridge | 2026-10-07, in use | Meshes, first-layer checks and analysis run on the Pi |
| Boot script `lidar-start.sh` | in use | |
| Stream mode 1 (`LIDAR_STREAM_MODE`) | 2026-11-02 | 10.7 instead of 5 frames/s; dwell strip 11 vs 13 s (moves dominate), meshes ~12 µm rms noisier: left at 0 |
| `LIDAR_STATUS` firmware/serial/temperature, `SELFTEST=1`, `AMBIENT=1` | 2026-11-02 | ~615 points per frame with the laser off in room light |
| Camera webcam (`--camera-port`, `LiDAR_Camera` switch) | 2026-11-02, in use | ~1.5 pictures/s at 512x288 on the developer's patched module firmware 1.0.18 (stock 1.0.13: 800x300, ~1/s, not measured as carefully) |
| `ir_photo` (1600x1200 JPEG through the driver) | 2026-11-08 | 3.3 s (fill light) / 2.4 s (laser); exposure and stream restored |
| Exposure per job (`auto_exposure`) | 2026-11-08 | Black smooth PEI: 1000 -> 887 points, 4000 -> 1472 (full line); bare-plate height the same within 2 µm at 1000-8000 |
| Median per bin in scan rows | deployed 2026-11-08 | offline test; no separate before/after comparison |

## Calibrations

| Feature | Tested | Result |
|---|---|---|
| `LIDAR_OFFSET_CALIBRATION` (LiDAR, 2-layer cross) | 2026-10-06 | Laser line at nozzle X -36 / Y -21.2 (Creality's stock value: -23.2 in Y) |
| `LIDAR_OFFSET_CALIBRATION` with the camera (1-layer cross) | 2026-11-08: the cross is found in real photos at both heights (offline test on them); a full saved run not yet | camera vs LiDAR agreement check (0.15 mm) tested offline |
| `LIDAR_PA_CALIBRATION` (LiDAR) | by 2026-10-07 | PA 0.0431 for a PETG; on thin white PLA on black PEI the LiDAR reads the lines low and PA scattered (0.0475 / 0.0302) |
| `LIDAR_PA_FLOW_CALIBRATION` (camera), PA | 2026-11-08, white matte PLA on black smooth PEI | **PA 0.0361** (r² 0.90), saved and in use |
| `LIDAR_PA_FLOW_CALIBRATION` (camera), flow | 2026-11-08 | 0.921 -> M221 S108.6, but a printed A/B check (pads at 100 / 104.3 / 108.6 %) looked best at 104 %: **~4 % low**, so flow is experimental and only saved with `FLOW_SAVE=1` |
| `SKEW_SCAN` / `SKEW_CALIBRATE` | 2026-10-30, in use | -0.20° (-0.35 mm per 100 mm), saved as skew profile `lidar` |
| `AXIS_TWIST_TOUCH_CALIBRATE` (probe + LiDAR twist tables in one run) | 2026-10-29, 2026-11-01 | Probe and LiDAR see the X-beam twist with opposite signs; tables saved and used |
| `LIDAR_TWIST_CALIBRATE AXIS=X/Y` | 2026-11-01 | Y error +22 ± 4 µm/100 mm (cold bed), confirmed by A/B prints |
| `LIDAR_GRID_CALIBRATE` (`LIDAR_MESH_CALIBRATION GRID=7,5`) | 2026-11-01, at 60, 70 and 100 °C | Rest after the X and Y curves 11-13 µm rms; the three temperatures agree within run-to-run scatter |
| `CREALITY_PA_TEST` | prints | Read by eye; not scanned or analysed |

## Bed mesh

| Feature | Tested | Result |
|---|---|---|
| `LIDAR_BED_MESH` levels 1-5 | 2026-10-29 to 2026-11-01 | Agree with the Microprobe within 16-18 µm rms |
| `MESH_BENCHMARK` (36 nozzle touches as truth, bed 70 °C) | 2026-11-01 | LiDAR L1/L2/L3 18.5/18.3/18.4 µm in 21/35/95 s; Microprobe 18.9 µm in 764 s; L2 repeat 8.3 µm |
| LiDAR mesh in START_PRINT (`LiDAR_Mesh`, adaptive, probe fallback) | 2026-11-01, in use | 9-pad prints: 16.8 µm rms (LiDAR, 80 s) vs 17.7 µm (Microprobe, ~13 min); 16.2 µm with averaged grid tables (2026-11-02) |
| Skew-aware reach checks | 2026-11-01 | All levels 1-5 and dwell pass with the skew profile loaded |
| `MESH_COMPARE` | in use | |

## First layer

| Feature | Tested | Result |
|---|---|---|
| `FIRST_LAYER_SCAN` (`First_Layer_Check`) | 2026-10-29, in use | 0.194 mm measured for 0.200 mm sliced; per-pad values in 4 A/B prints |
| Prime line after the scan pause | 2026-11-01 | Layer 2 starts clean |
| Curved plate drift between the scans (0.3.1) | 2026-11-02, two 9-pad prints re-analysed | The bare plate between the pads showed a 40-60 um bowl; taken off as a quadratic surface the pads spread 6.1 / 7.5 um rms instead of 16.2 / 23.3, median 0.204 mm for 0.200 (a contact probe on the pads agreed); a strip-shift print ruled out the laser line itself |
| `First_Layer_Learn` (Z offset per filament) | in use since 2026-10-31 | |

## K1 Max extras (`k1max/k1max-extras.cfg`)

| Feature | Tested | Result |
|---|---|---|
| Keep-out zones (`keep_out.py`) | in use | Probe kept off the Z rod housings; LiDAR scans planned around the zones |
| Nozzle wipe (`NOZZLE_WIPE`, `NOZZLE_SCRUB`) | in use | Can leave a thin string on the side of the nozzle; the purge picks it up |
| True Zero touch (load-cell Z=0, checked by the probe) | in use | |

## Not tested

- Any other printer, LiDAR firmware or firmware stack. Since 0.3.0 the code
  supports other Klipper printers ([install-other-printer.md](install-other-printer.md)),
  and the K1 Max still behaves the same (offline regression tests), but no
  other machine has run it yet. The `probe` and `gcode` touch methods and the
  `host/` install are tested offline only.
- Running without the helper service beyond basic scans.
- The connector pinout ([pinout.md](pinout.md)).
- 2.5 Mbaud on the serial link (needs a setting stored in the module's flash).
- The camera calibrations on any other plate/filament than white matte PLA
  on black smooth PEI, on stock module firmware 1.0.13 (the 2026-11-08 runs
  were on the developer's patched 1.0.18 build; the 1600x1200 pictures the
  calibrations use are the same capture command), or with any module mount
  other than the K1 Max's.
- The camera webcam on mainline Klipper (`--camera-switch none`) and
  `--webcam` with Mainsail.
