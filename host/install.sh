#!/bin/sh
# Install the FreeTheBeam LiDAR driver on a Linux Klipper host (the module on
# this computer's USB). Run from a checkout of this repo, as root:
#   sh host/install.sh KLIPPER_USER [--helper]
# --helper also installs the helper service on this host as lidar-helper
# (127.0.0.1:7132, talking to the driver's socket directly); then set
# `service: 127.0.0.1:7132` in [lidar]. It never touches a k1-lidar service
# (a K1's helper, possibly on the same computer).
# The module must be plugged in (and be the only 1a86:55d3 device), or its
# USB serial given as SERIAL=...: the udev rule names only that device.
# ROOT=, SYSTEMCTL=, UDEVADM= override the target and the tools (tests).
set -e
if [ -z "$1" ]; then
    echo "usage: sh host/install.sh KLIPPER_USER [--helper]" >&2
    exit 1
fi
U=$1
R=$(cd "$(dirname "$0")/.." && pwd)
D=${ROOT:-}
SYSTEMCTL=${SYSTEMCTL:-systemctl}
UDEVADM=${UDEVADM:-udevadm}
if [ -z "$SERIAL" ]; then
    n=0
    for d in "$D"/sys/bus/usb/devices/*; do
        [ "$(cat "$d/idVendor" 2>/dev/null)" = 1a86 ] || continue
        [ "$(cat "$d/idProduct" 2>/dev/null)" = 55d3 ] || continue
        SERIAL=$(cat "$d/serial" 2>/dev/null)
        n=$((n + 1))
    done
    if [ $n -ne 1 ] || [ -z "$SERIAL" ]; then
        echo "install.sh: found $n CH343 devices (1a86:55d3) with a serial" \
            "number; plug in the LiDAR module, or give its USB serial as" \
            "SERIAL=... (udevadm info -a /dev/ttyACM0 | grep serial)" >&2
        exit 1
    fi
fi
echo "install.sh: LiDAR module $SERIAL"
mkdir -p "$D/opt/freethebeam" "$D/etc/udev/rules.d" "$D/etc/systemd/system"
rm -rf "$D/opt/freethebeam/driver"
cp -r "$R/driver" "$D/opt/freethebeam/driver"
rm -rf "$D/opt/freethebeam/driver/tests"
sed "s/MODULE_SERIAL/$SERIAL/" "$R/host/60-creality-lidar.rules" \
    > "$D/etc/udev/rules.d/60-creality-lidar.rules"
$UDEVADM control --reload
$UDEVADM trigger --subsystem-match=tty
sed "s/KLIPPER_USER/$U/" "$R/host/lidar-direct.service" \
    > "$D/etc/systemd/system/lidar-direct.service"
if [ "$2" = "--helper" ]; then
    mkdir -p "$D/opt/freethebeam/helper" "$D/var/lib/lidar-helper"
    chown -R "$U" "$D/var/lib/lidar-helper"
    cp "$R/klipper/lidar.py" "$R/helper/lidar_service.py" \
        "$D/opt/freethebeam/helper/"
    sed "s/KLIPPER_USER/$U/" "$R/host/lidar-helper.service" \
        > "$D/etc/systemd/system/lidar-helper.service"
fi
$SYSTEMCTL daemon-reload
$SYSTEMCTL enable lidar-direct
# restart, so a re-install after an update runs the new code
$SYSTEMCTL restart lidar-direct
if [ "$2" = "--helper" ]; then
    $SYSTEMCTL enable lidar-helper
    $SYSTEMCTL restart lidar-helper
fi
$SYSTEMCTL --no-pager status lidar-direct | head -5
