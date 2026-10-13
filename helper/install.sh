#!/bin/sh
# Install the k1-lidar service on a helper computer (Debian / DietPi /
# Raspberry Pi OS). Run from a checkout of this repo, as root:
#   sh helper/install.sh PRINTER_IP
set -e
if [ -z "$1" ]; then
    echo "usage: sh helper/install.sh PRINTER_IP" >&2
    exit 1
fi
R=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p /opt/k1-lidar /var/lib/k1-lidar
cp "$R/klipper/lidar.py" "$R/helper/lidar_service.py" /opt/k1-lidar/
sed "s/PRINTER_IP/$1/g" "$R/helper/k1-lidar.service" > /etc/systemd/system/k1-lidar.service
systemctl daemon-reload
systemctl enable --now k1-lidar
systemctl --no-pager status k1-lidar | head -5
