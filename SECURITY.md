# Security and safety policy

FreeTheBeam drives real hardware. For this project, "security" covers both
the usual software issues and anything that can damage hardware.

## What to report privately

- A way to make the driver send a command that permanently changes or hangs
  the module (see the list at the top of [docs/protocol.md](docs/protocol.md)),
  bypassing its `UNSAFE` list.
- Toolhead moves outside the configured limits, or into the plate or parts
  (for example through the skew correction or keep-out handling).
- Network exposure: the bridge (`helper/lidar-bridge.py`, TCP 7130) and the
  helper service (TCP 7131) accept only the configured address; a way around
  that is a vulnerability.

Report these through GitHub's **private vulnerability reporting** ("Report a
vulnerability" on the repository's Security tab), not in a public issue.
Expect an answer within a week. This is a hobby project without a bounty.

## Everything else

Ordinary bugs, wrong measurements and crashes without hardware risk: open a
normal issue.

## Supported versions

Only the latest release gets fixes.
