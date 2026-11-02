# Install: the LiDAR on another Klipper printer

The Creality LiDAR module is a USB device, so it can be mounted on any
Cartesian or CoreXY printer that runs Klipper (or Kalico). This guide covers
mounting it, installing the driver on the Klipper host, and the first
calibrations. The K1 Max has its own guide: [install-k1max.md](install-k1max.md).

> Status: the K1 Max is the only machine tested so far
> ([tested.md](tested.md)). The code no longer assumes a K1 (bed size, touch
> method, paths and startup all come from your config), but the first
> printer of every other kind is a beta test. Please report results with the
> "Test report" issue template.

## 1. Mount the module

- **Working distance**: the module reads surfaces ~25-28 mm below it. Mount
  it so that with the nozzle 3 mm above the bed (`scan_z`, default 3) the
  bed is in that range; 3-5 mm nozzle height should all work.
- **Orientation**: the laser line must run along X (the profile is ~29 mm
  wide across X). Either way round works: `x_dir: -1` when the profile runs
  towards printer -X, `1` when it runs towards +X (a module turned by 180°
  flips it). Wrong `x_dir` shows up as a mirrored offset calibration.
- **Position**: anywhere next to the nozzle; the offsets are measured later.
  The further from the nozzle, the more of the bed the LiDAR can't reach
  (it can only look where the nozzle can take it).
- **Clearance**: nothing below the module may hang lower than the nozzle.
- **Cable**: USB to the Klipper host, with strain relief on the toolhead.
  The module's power comes over USB.

## 2. Install the driver on the Klipper host

On the host (Raspberry Pi, PC), as root, from a checkout of this repo, with
the module plugged in:

```
sh host/install.sh <klipper user> --helper
```

The udev rule is pinned to the module's USB serial number, because printer
boards can use the same USB chip (CH343, `1a86:55d3`) and the driver must
never open one of them. The installer stops if it finds no such device or
more than one; then give the module's serial as `SERIAL=... sh
host/install.sh ...` (`udevadm info -a /dev/ttyACMx | grep serial`).

This installs the udev rule (`/dev/serial/by-id/creality-laser`, group
`dialout`), the driver as the systemd service `lidar-direct` (socket
`/tmp/ai_server_uds`), and with `--helper` the helper service `lidar-helper`
on the same host (127.0.0.1:7132, talking to the driver's socket directly,
data in `/var/lib/lidar-helper`). It leaves a `k1-lidar` service alone, so a
Pi that is already a K1's helper can also be another printer's host. A Pi 4
with 1 GB runs both next to Klipper. Check with
`systemctl status lidar-direct` and `python3 driver/examples/dump_profile.py`
(with something flat under the module).

## 3. Klipper config

1. Link the plugin: `ln -sf <repo>/klipper/lidar.py <klipper>/klippy/extras/lidar.py`
   (mainline Klipper: `klippy/extras`; Kalico: `klippy/plugins`), then
   restart the Klipper service.
2. Add a `[lidar]` section: start from [`config/generic.cfg`](../config/generic.cfg)
   (every option, commented) or [`config/ender3v3se.cfg`](../config/ender3v3se.cfg)
   (an example with a strain-gauge nozzle touch).
3. Add the macros you want from [`klipper/lidar.example.cfg`](../klipper/lidar.example.cfg)
   and a `[save_variables]` section. Mainline Klipper has no virtual pins:
   leave its three `[output_pin]` switches out; the macros then read the
   saved variables instead (`SAVE_VARIABLE VARIABLE=sw_lidar_mesh VALUE=1`,
   likewise `sw_first_layer_check`, `sw_first_layer_learn`).
4. Set `zero_reference_position` in `[bed_mesh]` to where Z is homed or
   touched (see Limits below).
5. Run `LIDAR_SETUP_CHECK`: it lists the driver, offsets, timing, service,
   the area the LiDAR can reach, the zero reference and the touch method,
   and what is missing.

## 4. Touch method

Calibrations against the nozzle (`LIDAR_TWIST_CALIBRATE`,
`LIDAR_GRID_CALIBRATE`), mesh level 5 and `MESH_BENCHMARK` need a way to
touch the bed. `[lidar] touch:`

| Mode | Uses | Notes |
|---|---|---|
| `load_cell` | Kalico's `[load_cell_probe]` | the nozzle itself, the best reference |
| `probe` | any `[probe]` / `[bltouch]` | moved so the probe is over the spot; the reference is then the probe, which has its own twist: less accurate |
| `gcode` | your command (`touch_gcode`) + the status value with the contact Z (`touch_result: object:key`) | for nozzle-touch plugins, e.g. a strain gauge |
| `none` | | scans, meshes levels 1-4 and the first-layer check still work, without the twist/Y correction |

Unset, it picks `load_cell`, else `probe`, else `none`. `touch_temp`
(default 150 °C) is the nozzle temperature for touches, `touch_wipe_gcode`
an optional cleaning command run before a series of touches.

## 5. First calibrations

With the printer homed and the plate clear:

1. `LIDAR_TIMING`: the frame timing, saved.
2. `LIDAR_OFFSET_CALIBRATION EXTRUDER_TEMP= BED_TEMP=`: prints a small cross
   and measures where the laser looks. Its result overrides `x_offset` /
   `y_offset`. Repeat after changing the mount or the plate.
3. `LIDAR_SETUP_CHECK` again: the area should now cover most of the bed.
4. With a touch method: `LIDAR_MESH_CALIBRATION` (bed at print
   temperature).
5. `LIDAR_MESH`, then `MESH_COMPARE A=lidar B=<your probe mesh profile>`.
   Use the LiDAR mesh in prints (`LiDAR_Mesh` switch / `_LIDAR_START_PRINT_MESH`)
   only once the comparison looks right on your machine.

## Limits

- Cartesian and CoreXY only (the planning assumes X and Y move
  independently).
- `mesh_min` / `mesh_max` keep meshes and calibrations clear of clips,
  wipers or anything else on the bed edge; keep-out zones need
  `keep_out.py` ([install-k1max.md](install-k1max.md#6-k1-max-extras-optional)).
- The LiDAR has no absolute height: meshes are made 0 at `[bed_mesh]
  zero_reference_position`, which must be where Z is homed or touched.
  **Set it**: without it LiDAR meshes are only made relative to their
  median, and the first layer is off by the bed's height at the Z home spot.
  `LIDAR_SETUP_CHECK` reports it.
