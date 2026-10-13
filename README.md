# FreeTheBeam

**Free the beam.** Your K1 Max has a laser profiler on the toolhead with
micrometre repeatability. Creality's own software uses it to check your first
layer, and then, presumably out of embarrassment at how good it is, locks it
behind a closed-source middleware that refuses to run unless the whole Creality
stack is up. Install a custom firmware and the LiDAR just... sits there. Dead.
Staring. Judging.

This repo is the rescue: an open driver for the Creality "AI LiDAR" module
(reverse-engineered serial protocol, no vendor binary in the loop), a
Klipper/Kalico plugin that scans with it, and the paperwork to explain how it
works so the next person doesn't have to read MIPS disassembly at 2 a.m.

## What you get

- **A driver** that talks to the module directly over USB serial, does the
  "secret" handshake (spoiler: there is no secret, it's a SHA256 of a number
  the module tells you), and serves the same JSON socket API as Creality's
  middleware, so nothing else notices the swap. About 5 % CPU and 12.5 MB on a
  K1, same points as the middleware, ~15 % more frames per second.
- **A Klipper/Kalico plugin** (`LIDAR_SCAN`, `LIDAR_TIMING`, pressure-advance
  calibration measured by the LiDAR, laser-offset self-check). Scans lock to
  the module's frame clock: a 51-row scan takes 12-14 s instead of 20-21 s.
- **An optional Raspberry Pi helper** that does the heavy analysis, because
  a K1 with ~200 MB of RAM and a numpy job is how you get
  `MCU 'nozzle_mcu' shutdown: Rescheduled timer in the past` and a printer you
  have to unplug like it's 1999.
- **Docs**, including a pinout page that says "not measured yet" instead of
  inventing one. We have standards. Low ones, but standards.

## Which layer do I need?

| You want to | Use | Docs |
|---|---|---|
| Read profiles from the module on any Linux box | `driver/` | [install-generic](docs/install-generic.md) |
| Scan and calibrate from Klipper/Kalico | `driver/` + `klipper/` | [install-k1max](docs/install-k1max.md) |
| Revive the LiDAR on a K1 Max running Simple AF | `driver/` + `klipper/` + `k1max/` | [install-k1max](docs/install-k1max.md) |
| Keep your K1 from falling over during analysis | add `helper/` | [install-helper](docs/install-helper.md) |
| Understand what on earth is going on | read | [how-it-works](docs/how-it-works.md), [protocol](docs/protocol.md), [pinout](docs/pinout.md) |

## Quick start (no printer needed, just the module)

```
python3 driver/lidar-direct.py --port /dev/ttyACM0 &
python3 driver/examples/dump_profile.py --out profile.json
```

Tests, which use a fake module and no hardware:
`python3 -m unittest discover -s driver/tests`

## Warnings, in a font size you cannot ignore

**There are commands that permanently change or hang the module** (sticker
calibration, IR image capture, UART config, firmware update, restore). The
driver refuses to send them. Please don't go around the driver. The full list
is at the top of [docs/protocol.md](docs/protocol.md). Also: **don't run
Creality's sticker calibration with a glass or raised plate**; the module
stores wrong geometry inside itself, and there is no known way to take it back.

## Honesty section

- Developed and tested on **one** K1 Max with a glass bed and textured PEI,
  Simple AF + Kalico, module firmware 1.0.13. Other machines: you are the beta
  test.
- The connector pinout is unmeasured ([pinout.md](docs/pinout.md) says how to
  help).
- Heights from printed lines are rough (the laser sheet has thickness). Measure
  widths and areas, not peaks.
- The module delivers at most ~5 frames/s and each frame is ~0.19 s old when it
  arrives. Physics. We checked.

## Credits and licence

GPLv3 (see [LICENSE](LICENSE)). Built for [Simple AF](https://github.com/pellcorp/creality)
by Pellcorp and [Kalico](https://github.com/KalicoCrew/kalico). Not affiliated
with, endorsed by, or approved by Creality, who we assume are fine with this
and also assume are not reading it.
