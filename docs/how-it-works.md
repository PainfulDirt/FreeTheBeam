# How it works

Short version: Creality put a perfectly good laser profiler on the toolhead,
then wrapped it in a closed-source middleware that only runs inside Creality's
own software stack. This repo talks to the module directly instead.

## The pieces

```
 toolhead                        K1 Max (or any Linux box)                     optional helper (Pi)
+---------+  USB   +--------+   +----------------+  unix socket   +----------+   TCP   +------------------+
| LiDAR   |<------>| CH343  |<->| lidar_serial.py|<-------------->| lidar.py |<------->| lidar_service.py |
| module  |        | serial |   | lidar-direct.py|  JSON + 0x03   | (Klipper |  :7130  | (bridge on the   |
| (laser  |        +--------+   +----------------+  /tmp/ai_      |  plugin) |  :7131  |  printer forwards|
|  line + |                                        server_uds     +----------+         |  the socket)     |
|  camera)|                                                                            +------------------+
+---------+
```

1. **The module** projects a laser line and images it. One capture is a height
   profile: ~1500 points `[x, y, z]` in micrometres across ~29 mm. Working
   range is narrow: the nozzle must be 3-5 mm above the surface (distance
   ~25-28 mm). Repeatability is 1-2 µm between captures.
2. **`driver/lidar_serial.py`** speaks the serial protocol (1 500 000 baud
   8N1, framed commands, a "challenge" that needs no secret) and does the
   handshake. See [protocol.md](protocol.md).
3. **`driver/lidar-direct.py`** serves Creality's JSON socket API on
   `/tmp/ai_server_uds` so existing clients don't notice they're talking to
   something else. It keeps one persistent connection per client and
   reinitialises the module if it is power-cycled.
4. **`klipper/lidar.py`** is a Klipper/Kalico plugin: it moves the toolhead,
   collects profiles, subtracts a bare-plate scan, and implements
   `LIDAR_SCAN`, `LIDAR_TIMING`, `LIDAR_PA_CALIBRATE` and friends.
5. **`helper/`** (optional) moves the heavy lifting off the printer.

## Frames and timing

The module produces ~5 frames per second at most (direct driver: 4.98/s).
It runs on a 28.4 ms sensor tick: back-to-back frames come every 7 ticks
(199 ms) at full resolution. A request that arrives late slips the frame by
a whole tick, so `lidar-direct.py` can stream frames back-to-back
(`get_point_cloud` with `"prefetch": 1`) and stamps each one with
`CLOCK_MONOTONIC_RAW`, the same clock Klipper's reactor uses.

A frame also covers the *past*: at level 0, the data is from ~197..163 ms
before it arrives (median 178 ms). `LIDAR_TIMING` measures that for your
setup.

## Dwell scans

To scan a strip, the toolhead stands still for one frame's window per row and
moves in between. Rows are planned just ahead of the toolhead and locked to
the frame clock, so each row costs one frame (0.2 s). 51 rows (20 mm in 0.4 mm
steps) take 12-14 s, down from 20-21 s with stop-and-average. A scan matches a
slower stop-mode scan of the same strip within the noise (p05..p95 of the
difference 18-31 µm).

Textured PEI changes by ~76 µm between rows only 0.1 mm apart, which is why
moving scans were dropped and why a bare-plate scan is subtracted from every
measurement.

## Geometry (K1 Max, glass bed + textured PEI)

- The laser line centre sits at nozzle **X -36, Y -21.2** and the profile's x
  runs towards printer -X. Creality's stock config says -23.2 in Y.
- The laser comes in at about 45° in Y, so the spot moves ~-0.99 mm in Y per
  mm of nozzle height; X barely moves (+0.035).
- Heights from a printed line are rough (the laser sheet has thickness):
  measure widths and areas, not peak heights.

`LIDAR_OFFSET_CALIBRATE` prints a small cross and finds these numbers for your
own mount.

## Why the Pi helper exists

The K1 has ~200 MB of RAM and a slow MIPS CPU. A numpy run on ~50 MB of scans
stalled Klipper ("MCU 'nozzle_mcu' shutdown: Rescheduled timer in the past").
With `service:` set in `[lidar]`, a helper computer fetches the point clouds
(through `helper/lidar-bridge.py` on the printer, which only copies bytes),
reduces them, stores the raw captures and runs the analysis.

## What happened to Creality's stack

Under Simple AF the stock LiDAR is dead: the udev hook `laser_status.sh`
switches it off right after it enumerates unless `/tmp/load_done` exists. See
[pinout.md](pinout.md) and [install-k1max.md](install-k1max.md).
