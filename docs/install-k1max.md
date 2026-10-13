# Install: K1 Max with Simple AF (Pellcorp) and Kalico

This is the setup the repo was developed on. You need Kalico (for the
`gcode_shell_command` and plugin loader) and root SSH access to the printer.

1. **Copy the repo** to the printer, e.g. `/usr/data/FreeTheBeam`.
2. **Link the plugin** into Klipper:

   ```
   ln -sf /usr/data/FreeTheBeam/klipper/lidar.py /usr/data/klipper/klippy/plugins/lidar.py
   ```

   Python plugins only reload on a full service restart
   (`/etc/init.d/S55klipper_service restart`; `RESTART` keeps old module code).
   Re-create the link after every Simple AF update that touches `klippy/plugins`.
3. **Boot hook.** Run `k1max/lidar-start.sh` at boot, in the background (on
   Simple AF: add `/usr/data/FreeTheBeam/k1max/lidar-start.sh > /dev/null 2>&1 &`
   to your boot script). It:
   - creates `/tmp/load_done` so Creality's `laser_status.sh` stops switching
     the LiDAR off,
   - powers it with `laser_power.sh on`,
   - waits for `/dev/serial/by-id/creality-laser`,
   - starts `driver/lidar-direct.py` if `variables.cfg` has
     `lidar_driver = 'direct'`, otherwise Creality's `cx_ai_middleware`
     (still on the printer),
   - optionally starts `helper/lidar-bridge.py` for a helper computer: put its
     IP, alone, in `k1max/helper-ip`.
4. **Config.** Paste `klipper/lidar.example.cfg` into your printer config and
   adjust the path in `[gcode_shell_command k1_lidar_start]`. Add
   `_LIDAR_PRINT_SCAN` to your `START_PRINT` after the nozzle touch if you use
   `LIDAR_PA_CALIBRATION`. You need a `[save_variables]` section.
5. **Switch driver:** `LIDAR_DRIVER DRIVER=direct`, then `LIDAR_TIMING` (the
   timing is per driver and is saved as `lidar_timing`).
6. **Calibrate for your mount:** `LIDAR_OFFSET_CALIBRATION` prints a cross and
   saves where the laser looks. Do this after any change of plate or mount.

## Things that bite

- **Do not run Creality's sticker calibration** (`set_laser_cali_step`) with a
  raised or glass plate: the module stores the wrong geometry.
- **No heavy work on the K1**: use the helper (below) for analysis.
- Check the printer is idle before restarting Klipper; scans need a homed
  printer.
- `/tmp` on the K1 is a ~100 MB RAM disk.
