# Install: K1 Max with Simple AF (Pellcorp) and Kalico

This is the setup FreeTheBeam is developed on. You need Kalico (for
`gcode_shell_command` and the plugin loader) and root SSH access to the
printer.

## 1. Files

Copy the repo to the printer, e.g. `/usr/data/FreeTheBeam` (a release
archive or `git clone`).

## 2. Plugin

Link the plugin into Klipper:

```
ln -sf /usr/data/FreeTheBeam/klipper/lidar.py /usr/data/klipper/klippy/plugins/lidar.py
```

Python plugins only reload on a full service restart
(`/etc/init.d/S55klipper_service restart`; `RESTART` keeps the old module
code). Re-create the link after every Simple AF update that touches
`klippy/plugins`.

## 3. Boot hook

Run `k1max/lidar-start.sh` at boot, in the background (on Simple AF: add
`/usr/data/FreeTheBeam/k1max/lidar-start.sh > /dev/null 2>&1 &` to your boot
script). It:

- creates `/tmp/load_done`, so Creality's `laser_status.sh` stops switching
  the LiDAR off,
- powers it with `laser_power.sh on`,
- waits for `/dev/serial/by-id/creality-laser`,
- starts the open driver `driver/lidar-direct.py`, or Creality's
  `cx_ai_middleware` if `variables.cfg` has `lidar_driver = 'middleware'`
  (fallback only; the open driver is faster and is the tested path),
- optionally starts `helper/lidar-bridge.py` for a helper computer: put its
  IP, alone, in `k1max/helper-ip`.

## 4. Config

Add `k1max/lidar-k1max.cfg` (the K1's `[lidar]` with its bed limits,
calibration lines and load-cell touch, and the `LIDAR_DRIVER` macro; adjust
the path in `[gcode_shell_command k1_lidar_start]`) and what you need from
`klipper/lidar.example.cfg` (macros) to your printer config. You need a
`[save_variables]` section.

> Upgrading from 0.2.0: the K1 values moved out of the code into
> `lidar-k1max.cfg`. Without `mesh_min`/`mesh_max` and `twist_y`/`ytwist_x`,
> meshes would use the whole travel (into the Z rod
> housings at the back) and calibrations other lines. Then hook the LiDAR into your print macros
(Simple AF names in brackets):

| Where | Add | For |
|---|---|---|
| START_PRINT, first thing (`_SAF_START_PRINT_START`) | `_LIDAR_PRINT_START` | first-layer check |
| START_PRINT, instead of `BED_MESH_CALIBRATE` (`_START_PRINT_BED_MESH`) | `_LIDAR_START_PRINT_MESH ADAPTIVE=1` | LiDAR mesh with the `LiDAR_Mesh` switch |
| START_PRINT, after the mesh and the nozzle touch (`_SAF_START_PRINT_AFTER_BED_MESH`) | `_FIRST_LAYER_OFFSET {rawparams}`, `_LIDAR_FLOW_APPLY {rawparams}`, then `_LIDAR_PRINT_SCAN {rawparams}` | learned Z offset, saved flow (experimental), PA and flow calibration prints, first-layer check |
| slicer layer change, layer 2 | `_FIRST_LAYER_AFTER` | first-layer check |
| END_PRINT, before moving away (`_SAF_END_PRINT_START`) | `_LIDAR_PRINT_END` | first-layer check of one-layer prints |
| CANCEL_PRINT (`_SAF_ON_CANCEL`) | `_LIDAR_PRINT_CANCEL` | first-layer check; PA and M221 back after a cancelled calibration print |

Only add the lines for the features you use. `LIDAR_PA_CALIBRATION` and
`LIDAR_FLOW_CALIBRATION` need `_LIDAR_PRINT_SCAN`. No helper computer: the
service can run on the K1 itself ([install-helper.md](install-helper.md#on-the-printer-itself-local-mode)). The driver serves the
LiDAR's camera on port 8081 while the `LiDAR_Camera` switch is on; how to
add it to Fluidd and the camera calibrations: [camera.md](camera.md).

## 5. First run

1. `LIDAR_STATUS`: the driver answers and prints the version.
2. `G28`, then `LIDAR_TIMING` (saved as `lidar_timing`; run it again after
   switching driver).
3. `LIDAR_OFFSET_CALIBRATION`: prints a small cross and saves where the laser
   looks. Repeat after any change of plate or mount.
4. With the helper and load cells: `LIDAR_SETUP_CHECK`, `LIDAR_PLATE NAME=<plate>`
   (e.g. `textured_pei`), then `LIDAR_MESH_CALIBRATION` (clear plate,
   bed at print temperature), then `LIDAR_MESH` to try a mesh and
   `MESH_COMPARE A=lidar B=<your probe profile>` to compare it. Switch on
   `LiDAR_Mesh` once you trust it.

## 6. XY skew

`SKEW_CALIBRATE ... APPLY=1` needs a `[skew_correction]` section. Klipper
doesn't load a saved skew profile at boot by itself; a `delayed_gcode` with
`SKEW_PROFILE LOAD=<name>` does.

## Things to avoid

- **Do not run Creality's sticker calibration** (`set_laser_cali_step`) with a
  raised or glass plate: the module stores the wrong geometry.
- **No heavy work on the K1** (~200 MB RAM, slow MIPS CPU): use the helper
  ([install-helper.md](install-helper.md)) for meshes and analysis.
- Check the printer is idle before restarting Klipper; scans need a homed
  printer.
- `/tmp` on the K1 is a ~100 MB RAM disk.
