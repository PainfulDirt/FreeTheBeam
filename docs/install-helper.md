# Install: the helper service

A second Linux computer (a Raspberry Pi 4 with 1 GB is plenty) does the
captures and the analysis, so the printer's CPU and RAM stay free for
Klipper. Only LiDAR data takes this path; Klipper stays on the printer.

When Klipper itself runs on a normal Linux host with the module on its USB
(not a K1), the helper runs on that same host instead:
`sh host/install.sh <klipper user> --helper` and `service: 127.0.0.1:7131`
([install-other-printer.md](install-other-printer.md)). The steps below are
for a K1 with a separate helper.

Optional for basic scans, **required** for bed meshes, the first-layer check,
dwell scans, skew calibration and benchmarks. The service is standard-library
Python 3 (no pip packages).

1. **On the printer**, start the bridge (it forwards TCP :7130 to the
   driver's socket, for one allowed IP only): put the helper's IP in
   `k1max/helper-ip` and run `k1max/lidar-start.sh`, or by hand:

   ```
   python3 helper/lidar-bridge.py 7130 HELPER_IP
   ```

2. **On the helper**, as root, from a checkout of this repo:

   ```
   sh helper/install.sh PRINTER_IP
   ```

   It copies `klipper/lidar.py` and `helper/lidar_service.py` to
   `/opt/k1-lidar/`, installs the `k1-lidar` systemd unit (listening on
   :7131, data in `/var/lib/k1-lidar`, accepting only `PRINTER_IP`) and
   starts it. Run it again after every update: the helper and the printer
   must run the same `lidar.py`.

3. **In the printer config**, set `service: HELPER_IP:7131` in `[lidar]` and
   restart Klipper. `LIDAR_STATUS` then says "work done by the service".

## What the helper keeps

Everything is under `/var/lib/k1-lidar`:

| Path | Contents |
|---|---|
| `<name>.json` | raw scans |
| `meshes/`, `heightmap-*.json` | every LiDAR mesh, full-resolution height maps (level 4-5) |
| `firstlayer/` | first-layer checks: `fl-<id>.json` + `.html`, `history.jsonl` |
| `benchmarks/` | `MESH_BENCHMARK` runs, JSON + HTML |
| `skew/` | `SKEW_SCAN` / `SKEW_CALIBRATE` data |

Logs: `journalctl -u k1-lidar`.
