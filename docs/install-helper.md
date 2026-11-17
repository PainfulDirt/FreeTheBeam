# Install: the helper service

A second Linux computer (a Raspberry Pi 4 with 1 GB is plenty) does the
captures and the analysis, so the printer's CPU and RAM stay free for
Klipper. Only LiDAR data takes this path; Klipper stays on the printer.

When Klipper itself runs on a normal Linux host with the module on its USB
(not a K1), the helper runs on that same host instead:
`sh host/install.sh <klipper user> --helper` and `service: 127.0.0.1:7132`
([install-other-printer.md](install-other-printer.md)). The steps below are
for a K1 with a separate helper.

No second computer? The same service can run on the K1 itself
([below](#on-the-printer-itself-local-mode)), without the camera and
flow-band calibrations.

Optional for basic scans, **required** for bed meshes, the first-layer check,
dwell scans, skew calibration and benchmarks. The service is standard-library
Python 3, except the camera and flow calibrations (`LIDAR_PA_CALIBRATE` /
`LIDAR_PA_FLOW_CALIBRATE` with the camera or LiDAR strips, the flow bands,
the camera part of `LIDAR_OFFSET_CALIBRATE`, [camera.md](camera.md)), which
need numpy and PIL on the helper: `apt install python3-numpy python3-pil`
(install.sh warns when they are missing). Everything else runs without them.

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

   It copies `klipper/lidar.py` and `helper/lidar_*.py` (the service, the
   camera, PA/flow planning and flow-band modules) to `/opt/k1-lidar/`, installs the `k1-lidar` systemd unit (listening on
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
| `photos/<job>/`, `pf-<job>.json` + `.html` | camera calibrations: the pictures (with pose and exposure) and the PA/flow reports |

Logs: `journalctl -u k1-lidar`.

## On the printer itself (local mode)

Without a second computer the K1 can run `lidar_service.py` itself
(since 0.5.0). Bed meshes, the first-layer check, dwell scans, the
LiDAR-only PA test, the LiDAR offset cross, skew and benchmarks work; the
analysis is ~3x slower (lidar.py waits up to 3x longer, at most 300 s), and
the service runs niced so Klipper keeps its CPU. Since 0.5.1 it also
paces the heavy analysis (fine mesh levels 4/5, first-layer check): after
~20 ms of work it rests ~20 ms. The K1 has no numpy, so
everything that analyses photos or flow bands is refused before anything is
printed: `LIDAR_PA_CALIBRATE` falls back to the LiDAR-only PA test,
`LIDAR_OFFSET_CALIBRATE` to the LiDAR cross, and `LIDAR_PA_FLOW_CALIBRATE` /
`LIDAR_FLOW_CALIBRATION` answer an error.

1. In `[lidar]` (k1max/lidar-k1max.cfg):

   ```
   service: 127.0.0.1:7131
   service_local: True
   ```

2. Run `LIDAR_DRIVER`, then restart Klipper. `k1max/lidar-start.sh` (at boot
   and from `LIDAR_DRIVER`, which runs it with `restart`) finds both lines
   in any `*.cfg` of the config folder and starts the service with
   `--socket /tmp/ai_server_uds --profile printer`: it talks to the driver's
   socket directly (no bridge), keeps at most 6 scans in memory, writes no
   raw capture files and trims its data folder
   (`/usr/data/printer_data/lidar`) to the newest 200 MB. Log:
   `/usr/data/printer_data/logs/lidar-service.log`.
3. `LIDAR_STATUS` says "work done by the service on this printer (local)";
   `LIDAR_SETUP_CHECK` lists the camera calibrations as off.

`service_local` is what makes the difference, not the address: a helper
with numpy on the same host as Klipper (`host/install.sh --helper`,
`service: 127.0.0.1:7132`) leaves it out.

