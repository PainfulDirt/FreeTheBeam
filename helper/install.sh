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
# the service, the camera/PA/flow modules it imports and the shared analysis
cp "$R/klipper/lidar.py" "$R"/helper/lidar_*.py /opt/k1-lidar/
for m in lidar.py lidar_service.py lidar_camera.py lidar_pa_flow.py \
        lidar_bands.py; do
    [ -f "/opt/k1-lidar/$m" ] || { echo "install.sh: missing /opt/k1-lidar/$m" >&2; exit 1; }
done
# the camera calibrations and the PA / flow analysis (numpy + PIL)
python3 -c "import numpy, PIL" 2> /dev/null || echo "install.sh: the camera" \
    "and flow calibrations need numpy and PIL: apt install python3-numpy" \
    "python3-pil (scans and meshes work without them)" >&2
sed "s/PRINTER_IP/$1/g" "$R/helper/k1-lidar.service" > /etc/systemd/system/k1-lidar.service
systemctl daemon-reload
systemctl enable k1-lidar
# restart, so a re-install after an update runs the new code
systemctl restart k1-lidar
systemctl --no-pager status k1-lidar | head -5
