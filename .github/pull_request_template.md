## What and why

## How it was tested

- [ ] `python3 -m unittest discover -s driver/tests`
- [ ] `python3 -m unittest discover -s klipper/tests`
- [ ] On hardware (printer, firmware stack):

## Checklist

- [ ] No new dependencies on the printer side (standard-library Python 3.8)
- [ ] Nothing heavy in the Klipper process (that goes to the helper)
- [ ] No command from the "Do NOT send these" list in docs/protocol.md
- [ ] CHANGELOG.md updated under "Unreleased"
