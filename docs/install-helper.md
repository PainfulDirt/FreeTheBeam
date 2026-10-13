# Install: the helper service (optional)

A second Linux computer (a Raspberry Pi 4 with 1 GB is plenty) runs the
captures and the analysis so the printer's CPU and RAM stay free for Klipper.
It only needs to be reachable over the network; only LiDAR data takes this
path, Klipper stays on the printer.

1. **On the printer**, start the bridge (it forwards TCP to the driver's socket,
   only for one allowed IP): put the helper's IP in `k1max/helper-ip` and run
   `k1max/lidar-start.sh`, or by hand:

   ```
   python3 helper/lidar-bridge.py 7130 HELPER_IP
   ```
2. **On the helper**, as root, from a checkout of this repo:

   ```
   sh helper/install.sh PRINTER_IP
   ```

   It copies `klipper/lidar.py` and `helper/lidar_service.py` to
   `/opt/k1-lidar/`, installs the `k1-lidar` systemd unit (listening on :7131,
   data in `/var/lib/k1-lidar`, accepting only `PRINTER_IP`) and starts it.
3. **In the printer config**, uncomment `service: HELPER_IP:7131` in `[lidar]`
   and restart Klipper.

Raw scans are kept on the helper as JSON in `/var/lib/k1-lidar`.
