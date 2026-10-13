# Install: the driver on any Linux box

Requirements: Linux (the driver uses `termios` with 1 500 000 baud), Python 3.
No pip packages. The module needs to be powered (see [pinout.md](pinout.md));
on a plain USB port it simply is.

Tested only on a K1 Max (MIPS, BusyBox Python 3). The steps below are the same
thing done by hand; if your distro disagrees, believe your distro.

1. **Find the device.** Plug the module in and look for USB ID `1a86:55d3`
   (`lsusb`). It shows up as `/dev/ttyACM0` (or higher).
2. **Optional stable name.** Creality's K1 rule names it
   `/dev/serial/by-id/creality-laser`, which is the driver's default. On your
   machine, either pass `--port /dev/ttyACM0` or add a udev rule, e.g.
   `/etc/udev/rules.d/60-creality-laser.rules`:

   ```
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", SYMLINK+="serial/by-id/creality-laser", MODE="0660", GROUP="dialout"
   ```

   then `sudo udevadm control --reload && sudo udevadm trigger`.
3. **Permissions.** Add yourself to the `dialout` group (or whichever group
   owns the device) and log in again.
4. **Run the driver.**

   ```
   python3 driver/lidar-direct.py --port /dev/ttyACM0 --socket /tmp/ai_server_uds
   ```

   It authenticates with the module itself (no vendor software needed) and
   keeps retrying if the module isn't there yet.
5. **Get a profile.** Hold a flat surface in range (the surface reads ~25-28 mm
   from the module; on a printer that is the nozzle 3-5 mm above the bed):

   ```
   python3 driver/examples/dump_profile.py --out profile.json
   ```

   Output looks like `1500 points, x -14.40..14.80 mm, median z 25.5 mm` (your
   numbers will differ). Points are `[x, y, z]` in micrometres.

If it prints `is lidar-direct.py running?`, it isn't. If the profile is
empty (code 4), the surface is out of range. If the module goes quiet, power
cycle it and read the "Do NOT send these" box in [protocol.md](protocol.md)
before you try anything creative.

Run the offline tests with `python3 -m unittest discover -s driver/tests`;
they use a fake module and need no hardware.
