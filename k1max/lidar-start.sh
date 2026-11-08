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
# create. Then a driver serves /tmp/ai_server_uds for lidar.py:
# lidar-direct.py, the open driver (default), or Creality's cx_ai_middleware
# (same JSON API, still ships on the printer) as a fallback.
# variables.cfg: lidar_driver = 'direct' (or unset) or 'middleware'; switch
# with the LIDAR_DRIVER macro, which runs this.
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
if [ "$DRIVER" != middleware ]; then
    DRIVER=direct
    killall cx_ai_middleware > /dev/null 2>&1 && sleep 1
    # module stream mode (variables.cfg lidar_stream_mode, LIDAR_STREAM_MODE):
    # 1 = ~10.7 instead of ~5 frames/s. The IR camera on http port 8081,
    # only while the LiDAR_Camera switch is on (docs/camera.md)
    MODE=$(sed -n 's/^lidar_stream_mode *= *\([01]\) *$/\1/p' \
        /usr/data/printer_data/config/variables.cfg 2> /dev/null)
    pgrep -f lidar-direct.py > /dev/null ||
        setsid python3 -B $C/../driver/lidar-direct.py --stream-mode ${MODE:-0} \
            --camera-port 8081 < /dev/null > /dev/null 2>&1 &
else
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
