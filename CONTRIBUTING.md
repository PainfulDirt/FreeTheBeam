# Contributing to FreeTheBeam

Thanks for helping. The project runs on one printer so far, so reports from
other machines are the most valuable contribution there is.

## Reporting

- **Bugs**: open an issue with the bug template. Include the FreeTheBeam
  version (`LIDAR_STATUS` or `lidar-direct.py --version`), printer, firmware
  stack (Simple AF / Kalico version), whether the helper service is used, and
  the console output. The driver logs to stderr; on a K1 started by
  `lidar-start.sh`, run it by hand to see it.
- **Measurements from other printers** (meshes, benchmarks, first-layer
  checks, timing) are welcome as issues, with the numbers and the setup.
- **Safety issues** (anything that can damage a module or crash the
  toolhead): see [SECURITY.md](SECURITY.md).

## Pull requests

1. One topic per pull request; explain what changed and how you tested it
   (offline tests, and on which printer if you ran it on hardware).
2. Run the tests, which need no hardware:
   ```
   python3 -m unittest discover -s driver/tests
   python3 -m unittest discover -s klipper/tests
   ```
3. Keep the code style of the file you touch: standard-library Python 3.8
   (the K1's interpreter), no new dependencies on the printer side, comments
   that say *why*.
4. The printer has ~200 MB of RAM and a slow MIPS CPU. Anything heavy belongs
   in `helper/lidar_service.py`, not in the Klipper process.
5. **Never** add code that sends the commands listed at the top of
   [docs/protocol.md](docs/protocol.md) (sticker calibration, IR image, UART
   config, OTA, restore, 0xAA). The driver's `UNSAFE` list stays.
6. Protocol facts go into `docs/protocol.md` with how they were established
   (firmware, captured traffic, measurement). Unknowns stay marked as unknown.

## Licence

By contributing you agree that your contribution is licensed under the GPLv3,
like the rest of the project.

## Releasing (maintainers)

1. Bump the version in `driver/lidar_serial.py` (`__version__`) and
   `klipper/lidar.py` (`VERSION`).
2. Move the changes into a new section of `CHANGELOG.md`.
3. Commit, tag `vX.Y.Z`, push with tags, and create the GitHub release from
   the changelog section.
