# Pinout (the honest version)

## What is verified

| | |
|---|---|
| Interface | USB device, **CH343** USB-serial bridge, USB ID `1a86:55d3` |
| Device node | `/dev/ttyACM0` (K1 Max); Creality's udev rule adds the symlink `/dev/serial/by-id/creality-laser` |
| Serial | **1 500 000 baud, 8N1**, raw, no flow control |
| Not a camera | there is no UVC interface; all data goes over the serial protocol |
| Power | switched by a GPIO on the K1 Max mainboard: **PA14**, driven by `laser_power.sh on\|off\|restart` (the pin comes from `laser_info.json`, which depends on the USB port the module is on) |
| Firmware | `3D-CONTOUR-WQ-V1013-O.bin`, reports version 1.0.13 and a serial number |

On plug-in the K1's udev rules also run `cx_ai_crypto` on the port (sends an
"encryption key", SHA256) and `laser_status.sh`, which checks the firmware and
**switches the LiDAR off unless `/tmp/load_done` exists**. The open driver does
the key exchange itself.

## Connector pinout: not yet measured

The cable between the mainboard and the module has a connector, and so far
nobody (including us) has measured which wire is which. We don't print
guesses: wire colours and pin numbers copied from a forum post are how modules
get fried.

If you want to measure it with the printer **powered off**:

1. Photograph both connectors and note the pin count and keying.
2. With a multimeter in continuity mode, find the ground (the mainboard's
   ground plane, e.g. a USB shield or screw hole).
3. Find 5 V: power the printer, measure the module-side pins against ground
   *with the cable plugged into the mainboard and the module unplugged*.
4. D+ and D- are the remaining pair, which should be routed together towards
   the CH343 (differential pair, ~90 Ω).
5. Open a PR with the table and the photos.

Until then: if your module is on a USB port, it works like any USB serial
device; see [install-generic.md](install-generic.md).
