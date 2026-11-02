# FreeTheBeam

[![Release](https://img.shields.io/github/v/release/PainfulDirt/FreeTheBeam)](https://github.com/PainfulDirt/FreeTheBeam/releases)
[![Tests](https://github.com/PainfulDirt/FreeTheBeam/actions/workflows/tests.yml/badge.svg)](https://github.com/PainfulDirt/FreeTheBeam/actions/workflows/tests.yml)
[![License: GPL v3](https://img.shields.io/badge/license-GPLv3-blue.svg)](LICENSE)
![Python 3.8+](https://img.shields.io/badge/python-3.8%2B-blue.svg)

**Open-source driver and Klipper/Kalico integration for the Creality K1 Max
"AI LiDAR".** *Free the beam. Own your first layer.*

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
- **Calibrations**: pressure advance measured from a printed test, the
  LiDAR's own position, XY skew without calipers, frame timing.
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
| LiDAR module firmware 1.0.13 | tested |
| Other K1-series printers with the same module, other firmware stacks | untested; reports welcome |
| The module on any Linux machine (driver only) | should work (standard-library Python 3.8+) |

Bed mesh calibration, level 5 and benchmarks need load cells
(`[load_cell_probe]`). Meshes, the first-layer check and dwell scans need the
helper service.

## Installation

| You want to | Use | Guide |
|---|---|---|
| Read profiles from the module on any Linux machine | `driver/` | [install-generic](docs/install-generic.md) |
| Use the LiDAR on a K1 Max with Simple AF + Kalico | `driver/`, `klipper/`, `k1max/` | [install-k1max](docs/install-k1max.md) |
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
- [Bed mesh](docs/bed-mesh.md) and [first-layer check](docs/first-layer.md)
- [What has been tested](docs/tested.md), with results
- [Protocol](docs/protocol.md): socket API and serial protocol
- [Pinout](docs/pinout.md): what is verified about the hardware
- [Changelog](CHANGELOG.md)

## Safety

> [!WARNING]
> Some module commands **permanently change or hang the module**: sticker
> calibration (0x05), IR image capture (0x56), UART config (0x1B), firmware
> update and restore. The driver refuses all of them; do not work around it.
> The full list is at the top of [docs/protocol.md](docs/protocol.md).
>
> **Never run Creality's sticker calibration with a glass or raised plate.**
> The module stores the wrong geometry in its flash, and there is no known
> way to undo it.

Also: scans and meshes move the toolhead close to the plate. Home first, keep
the scanned area clear, and do not run heavy analysis on the K1 itself.

## Limitations

- Developed and tested on **one** K1 Max (glass bed, textured PEI).
- The connector pinout is not measured yet ([pinout.md](docs/pinout.md) says
  how to help).
- Heights of thin printed lines are approximate (the laser sheet has a
  thickness): widths and areas are reliable, single peaks are not.
- The module delivers at most ~5 frames/s and each frame is ~0.19 s old on
  arrival.

## Tests

The tests use a simulated module and printer, no hardware:

```
python3 -m unittest discover -s driver/tests
python3 -m unittest discover -s klipper/tests
```

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
