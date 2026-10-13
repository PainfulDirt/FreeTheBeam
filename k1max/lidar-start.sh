#!/bin/sh
# Bring up the K1 Max LiDAR under Simple AF. Start it at boot (background) and
# from the LIDAR_DRIVER macro; safe to run again.
#
# Optional: LIDAR_HELPER=<ip> in the environment, or that IP alone in a file
# k1max/helper-ip (the Klipper shell command has no environment), starts helper/lidar-bridge.py
# so a helper computer (helper/lidar_service.py) can reach the driver.
#
# The stock udev hook (laser_status.sh) switches the LiDAR off right after it
# enumerates unless /tmp/load_done exists, which the Creality stack would
# create. Then a driver serves /tmp/ai_server_uds for lidar.py: Creality's
# cx_ai_middleware (still ships on the printer) or lidar-direct.py, the open
# replacement (same JSON API). variables.cfg: lidar_driver = 'direct' or
# 'middleware' (default); switch with the LIDAR_DRIVER macro, which runs this.
touch /tmp/load_done
[ -x /usr/bin/laser_power.sh ] || exit 0
# fails harmlessly before the first enumeration (no laser_info.json yet); the
# LiDAR is powered at boot anyway and load_done now keeps it on
laser_power.sh on > /dev/null 2>&1
i=0
while [ ! -e /dev/serial/by-id/creality-laser ] && [ $i -lt 30 ]; do
    sleep 1; i=$((i + 1))
done
if [ ! -e /dev/serial/by-id/creality-laser ]; then
    echo "lidar-start: no LiDAR found" > /tmp/lidar-start.log
    exit 0
fi
sleep 2
C=$(dirname "$(readlink -f "$0")")
DRIVER=$(sed -n "s/^lidar_driver = '\(.*\)'/\1/p" /usr/data/printer_data/config/variables.cfg)
if [ "$DRIVER" = direct ]; then
    killall cx_ai_middleware > /dev/null 2>&1 && sleep 1
    pgrep -f lidar-direct.py > /dev/null ||
        setsid python3 -B $C/../driver/lidar-direct.py < /dev/null > /dev/null 2>&1 &
else
    DRIVER=middleware
    for p in $(pgrep -f lidar-direct.py); do kill $p; done
    if ! pidof cx_ai_middleware > /dev/null; then
        rm -f /tmp/ai_server_uds
        setsid cx_ai_middleware < /dev/null > /dev/null 2>&1 &
        # it only sets the module up when told it's plugged in (udev does
        # that at enumeration, which may have happened before it started)
        sleep 2
        ubus call laser set_state '{"laser_plugged": 1}' > /dev/null 2>&1
    fi
fi
# forward the driver to the helper computer that runs lidar_service.py
# (helper/): only that address may connect
[ -n "$LIDAR_HELPER" ] || LIDAR_HELPER=$(cat "$C/helper-ip" 2>/dev/null)
if [ -n "$LIDAR_HELPER" ] && ! ps | grep -v grep | grep -q lidar-bridge.py; then
    setsid python3 $C/../helper/lidar-bridge.py 7130 $LIDAR_HELPER < /dev/null > /dev/null 2>&1 &
fi
echo "lidar-start: up, driver $DRIVER" | tee /tmp/lidar-start.log
