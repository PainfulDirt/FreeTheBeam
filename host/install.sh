#!/bin/sh
# Install the FreeTheBeam LiDAR driver on a Linux Klipper host (the module on
# this computer's USB). Run from a checkout of this repo, as root:
#   sh host/install.sh KLIPPER_USER [--helper]
# --helper also installs the helper service on this host, talking to the
# driver's socket directly; then set `service: 127.0.0.1:7131` in [lidar].
set -e
if [ -z "$1" ]; then
    echo "usage: sh host/install.sh KLIPPER_USER [--helper]" >&2
    exit 1
fi
U=$1
R=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p /opt/freethebeam
rm -rf /opt/freethebeam/driver
cp -r "$R/driver" /opt/freethebeam/driver
rm -rf /opt/freethebeam/driver/tests
cp "$R/host/60-creality-lidar.rules" /etc/udev/rules.d/
udevadm control --reload
udevadm trigger --subsystem-match=tty
sed "s/KLIPPER_USER/$U/" "$R/host/lidar-direct.service" > /etc/systemd/system/lidar-direct.service
if [ "$2" = "--helper" ]; then
    mkdir -p /opt/k1-lidar /var/lib/k1-lidar
    chown "$U" /var/lib/k1-lidar
    cp "$R/klipper/lidar.py" "$R/helper/lidar_service.py" /opt/k1-lidar/
    sed -e "s#--printer PRINTER_IP:7130#--socket /tmp/ai_server_uds#" \
        -e "s#--listen 0.0.0.0:7131#--listen 127.0.0.1:7131#" \
        -e "s#--allow PRINTER_IP#--allow 127.0.0.1#" \
        -e "s#^\[Service\]#[Service]\nUser=$U#" \
        "$R/helper/k1-lidar.service" > /etc/systemd/system/k1-lidar.service
fi
systemctl daemon-reload
systemctl enable lidar-direct
# restart, so a re-install after an update runs the new code
systemctl restart lidar-direct
if [ "$2" = "--helper" ]; then
    systemctl enable k1-lidar
    systemctl restart k1-lidar
fi
systemctl --no-pager status lidar-direct | head -5
