# FreeTheBeam

[![Release](https://img.shields.io/github/v/release/PainfulDirt/FreeTheBeam)](https://github.com/PainfulDirt/FreeTheBeam/releases)
[![Tests](https://github.com/PainfulDirt/FreeTheBeam/actions/workflows/tests.yml/badge.svg)](https://github.com/PainfulDirt/FreeTheBeam/actions/workflows/tests.yml)
[![License: GPL v3](https://img.shields.io/badge/license-GPLv3-blue.svg)](LICENSE)
![Python 3.8+](https://img.shields.io/badge/python-3.8%2B-blue.svg)

**Open-source driver and Klipper/Kalico integration for the Creality K1 Max
"AI LiDAR", on the K1 Max or mounted on any Klipper printer.** *Free the beam. Own your first layer.*

The K1 Max has a laser line profiler on its toolhead with micrometre
repeatability. Stock, it is driven by a closed-source middleware that only
works inside Creality's own software stack, so on community firmware the
LiDAR sits unused. FreeTheBeam replaces that middleware with an open driver
(reverse-engineered serial protocol, no vendor binaries) and builds the
printer features on top: fast bed meshes, a real first-layer thickness check
and LiDAR-measured calibrations.

## Features

- **Open driver** (`driver/`). Talks to the module over USB serial,
  performs the authentication handshake itself and serves the same JSON
  socket API as Creality's middleware, so it works as a drop-in replacement.
  On a K1 it uses ~5 % CPU and 12.5 MB, returns the same points as the
  middleware and delivers ~15 % more frames per second.
- **LiDAR bed mesh** (`LIDAR_BED_MESH`). Five levels from 7x7 in ~20 s to
  30x30 fitted to nozzle touches. On the development printer it is as
  accurate as the Microprobe (18.3 vs 18.9 µm rms against 36 nozzle touches)
  and 8-20x faster. Calibrated against the nozzle for X-beam twist and the
  error along Y, with automatic probe fallback inside START_PRINT.
  [Details](docs/bed-mesh.md).
- **First-layer check** (`FIRST_LAYER_SCAN`). Scans the bare plate before
  the print and the first layer at layer 2, and reports the real thickness
  map (test print: 0.194 mm measured for 0.200 mm sliced). It can learn the
  Z offset per filament. [Details](docs/first-layer.md).
- **The module's camera** (since 0.4.0, with the helper). Pressure advance
  from photos of printed lines (`LIDAR_PA_FLOW_CALIBRATE`; the LiDAR reads
  thin translucent lines low, the camera measures their width to a few µm),
  the laser offset from photos of a 1-layer cross, and a webcam for
  Fluidd/Mainsail. Flow from the same print is experimental (read ~4 % low
  against a printed check) and only saved on request.
  [Details](docs/camera.md).
- **Calibrations**: pressure advance measured from a printed test, the
  LiDAR's own position, XY skew without calipers, frame timing; the
  exposure follows the plate (dark or glossy sheets need more light).
- **K1 Max extras** (`k1max/k1max-extras.cfg`, `klipper/`): keep-out zones,
  nozzle wipe on the plate edge, True Zero touch (Z=0 from a load-cell nozzle
  touch, checked by the probe), and a twist calibration that measures the
  probe and the LiDAR against the same nozzle touches.
- **Helper service** (`helper/`). Runs captures and analysis on a second
  computer (e.g. a Raspberry Pi), so the K1's ~200 MB of RAM stay with
  Klipper.
- **Documentation** of the protocol, the timing and the hardware, with
  unknowns marked as unknown.

## Compatibility

| | Status |
|---|---|
| Creality K1 Max, Simple AF (Pellcorp) + Kalico | developed and tested here: [what was tested](docs/tested.md) |
| LiDAR module firmware 1.0.13 | tested (the 0.4.0 camera work ran on the developer's own patched build of it, 1.0.18; the calibrations' capture command is the stock one) |
| Any Cartesian/CoreXY Klipper or Kalico printer with the module mounted | supported, untested: [install-other-printer](docs/install-other-printer.md); reports welcome |
| The module on any Linux machine (driver only) | should work (standard-library Python 3.8+) |

Meshes, the first-layer check and dwell scans need the helper service (on a
second computer or on the Klipper host itself). Mesh calibration, level 5
and benchmarks need a way to touch the bed: load cells, a probe, or your own
nozzle-touch command. `LIDAR_SETUP_CHECK` shows what your setup can do.

## Installation

| You want to | Use | Guide |
|---|---|---|
| Read profiles from the module on any Linux machine | `driver/` | [install-generic](docs/install-generic.md) |
| Use the LiDAR on a K1 Max with Simple AF + Kalico | `driver/`, `klipper/`, `k1max/` | [install-k1max](docs/install-k1max.md) |
| Mount the module on another Klipper printer | `host/`, `driver/`, `klipper/`, `config/` | [install-other-printer](docs/install-other-printer.md) |
| Keep-out zones, nozzle wipe, True Zero touch, probe twist | `klipper/*.py`, `k1max/k1max-extras.cfg` | [install-k1max](docs/install-k1max.md#6-k1-max-extras-optional) |
| Bed meshes, first-layer check, analysis | add `helper/` | [install-helper](docs/install-helper.md) |

Quick start with only the module (no printer):

```
python3 driver/lidar-direct.py --port /dev/ttyACM0 &
python3 driver/examples/dump_profile.py --out profile.json
```

## Documentation

- [How it works](docs/how-it-works.md): architecture, frames and timing,
  dwell and moving scans
- [Command reference](docs/commands.md): every G-code command, macro and
  saved variable
- [Bed mesh](docs/bed-mesh.md), [first-layer check](docs/first-layer.md)
  and [the camera](docs/camera.md)
- [What has been tested](docs/tested.md), with results
- [Protocol](docs/protocol.md): socket API and serial protocol
- [Pinout](docs/pinout.md): what is verified about the hardware
- [Changelog](CHANGELOG.md)

## Safety

> [!WARNING]
> Some module commands **permanently change or hang the module**: sticker
> calibration (0x05), IR image capture with type 0 (0x56; the driver only
> sends type 2, JPEG), UART config (0x1B), watchdog time (0x58), firmware
> update and restore. The driver refuses all of them; do not work around it.
> The full list is at the top of [docs/protocol.md](docs/protocol.md).
>
> **Never run Creality's sticker calibration with a glass or raised plate.**
> The module stores the wrong geometry in its flash, and there is no known
> way to undo it.

Also: scans and meshes move the toolhead close to the plate. Home first, keep
the scanned area clear, and do not run heavy analysis on the K1 itself.

## Limitations

- Developed and tested on **one** K1 Max (glass bed, textured PEI; the
  camera calibrations on black smooth PEI with white matte PLA only).
- Camera flow is experimental (~4 % low against a printed check), and the
  camera analysis assumes the K1 Max's module mounting.
- The connector pinout is not measured yet ([pinout.md](docs/pinout.md) says
  how to help).
- The LiDAR reads thin translucent lines and single layers low (the laser
  light spreads inside the plastic): a 0.2 mm white PLA line read 0.11-0.19
  mm. That is why PA and flow use the camera.
- The module delivers at most ~5 frames/s and each frame is ~0.19 s old on
  arrival.

## Tests

The tests use a simulated module and printer, no hardware:

```
python3 -m unittest discover -s driver/tests
python3 -m unittest discover -s klipper/tests
python3 -m unittest discover -s helper/tests
```

The macro tests need jinja2, the camera tests numpy and PIL (they skip
without them).

## Contributing

Bug reports, measurements from other printers and pull requests are welcome;
see [CONTRIBUTING.md](CONTRIBUTING.md). Safety issues (anything that can
brick a module or crash a toolhead): see [SECURITY.md](SECURITY.md).

## Credits and licence

GPLv3, see [LICENSE](LICENSE). Built for [Simple AF](https://github.com/pellcorp/creality)
by Pellcorp and [Kalico](https://github.com/KalicoCrew/kalico). The grid
calibration follows [Axis Twist Mesh](https://github.com/sterling5241/axis-mesh)
by sterling5241, used with the author's permission.

FreeTheBeam is not affiliated with, endorsed by or supported by Creality.
"Creality", "K1" and "K1 Max" are trademarks of their owners. Use at your own
risk.
