#!/bin/sh
# Bring up the K1 Max LiDAR under Simple AF. Start it at boot (background) and
# from the LIDAR_DRIVER macro; safe to run again (starts what isn't running).
#   lidar-start.sh           start the driver (and bridge / service) if needed
#   lidar-start.sh restart   stop them first (LIDAR_DRIVER), then start them
#
# Optional: LIDAR_HELPER=<ip> in the environment, or that IP alone in a file
# k1max/helper-ip (the Klipper shell command has no environment), starts
# helper/lidar-bridge.py so a helper computer (helper/lidar_service.py) can
# reach the driver.
#
# No helper computer: with `service: 127.0.0.1:<port>` and
# `service_local: True` in the [lidar] section (any *.cfg in the config
# folder) helper/lidar_service.py runs here too, niced, talking to the
# driver's socket directly (local mode: no numpy on the K1, so no camera or
# flow-band calibrations; docs/install-helper.md). restart stops it too and
# starts it again only in that mode.
#
# The stock udev hook (laser_status.sh) switches the LiDAR off right after it
# enumerates unless /tmp/load_done exists, which the Creality stack would
# create. Then a driver serves /tmp/ai_server_uds for lidar.py:
# lidar-direct.py, the open driver (default), or Creality's cx_ai_middleware
# (same JSON API, still ships on the printer) as a fallback.
# variables.cfg: lidar_driver = 'direct' (or unset) or 'middleware'; switch
# with the LIDAR_DRIVER macro, which runs this.
CFG=/usr/data/printer_data/config

# PIDs of the python processes running script $1. pgrep -f also matches a
# shell whose command line merely contains the name (an SSH command, this
# script's caller): only python processes count, never this shell or its
# parent.
pids_of() {
    for p in $(pgrep -f "$1"); do
        [ "$p" = "$$" ] || [ "$p" = "$PPID" ] && continue
        tr '\000' ' ' < /proc/$p/cmdline 2> /dev/null | grep -q "python[^ ]* .*$1" &&
            echo $p
    done
}
# stop: SIGTERM, wait up to 5 s, then SIGKILL
stop() {
    P=$(pids_of "$1")
    [ -n "$P" ] || return 0
    kill $P 2> /dev/null
    n=0
    while [ -n "$(pids_of "$1")" ] && [ $n -lt 5 ]; do
        sleep 1; n=$((n + 1))
    done
    P=$(pids_of "$1")
    [ -n "$P" ] && kill -9 $P 2> /dev/null
    echo "lidar-start: stopped $1"
}
if [ "$1" = "restart" ]; then
    stop lidar-direct.py
    stop lidar-bridge.py
    stop lidar_service.py
fi

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
R=$(readlink -f "$C/..")
DRIVER=$(sed -n "s/^lidar_driver = '\(.*\)'/\1/p" $CFG/variables.cfg)
if [ "$DRIVER" != middleware ]; then
    DRIVER=direct
    killall cx_ai_middleware > /dev/null 2>&1 && sleep 1
    # module stream mode (variables.cfg lidar_stream_mode, LIDAR_STREAM_MODE):
    # 1 = ~10.7 instead of ~5 frames/s. The IR camera on http port 8081,
    # only while the LiDAR_Camera switch is on (docs/camera.md)
    MODE=$(sed -n 's/^lidar_stream_mode *= *\([01]\) *$/\1/p' \
        $CFG/variables.cfg 2> /dev/null)
    if [ -z "$(pids_of lidar-direct.py)" ]; then
        setsid python3 -B $R/driver/lidar-direct.py --stream-mode ${MODE:-0} \
            --camera-port 8081 < /dev/null > /dev/null 2>&1 &
    fi
else
    stop lidar-direct.py
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
if [ -n "$LIDAR_HELPER" ] && [ -z "$(pids_of lidar-bridge.py)" ]; then
    setsid python3 $R/helper/lidar-bridge.py 7130 $LIDAR_HELPER < /dev/null > /dev/null 2>&1 &
fi
# the analysis service on this printer: [lidar] service: 127.0.0.1:<port>
# with service_local: True
SVC=$(awk 'FNR == 1 { s = 0 } /^\[lidar\]/ { s = 1; next } /^\[/ { s = 0 }
    s && /^service[ \t]*[:=]/ { sub(/^service[ \t]*[:=][ \t]*/, "");
        sub(/[ \t#;].*/, ""); svc = $0 }
    s && /^service_local[ \t]*[:=][ \t]*([Tt]rue|1|[Yy]es)/ { loc = 1 }
    END { if (loc) print svc }' $CFG/*.cfg 2> /dev/null)
case $SVC in
    127.0.0.1:*|localhost:*)
        if [ -z "$(pids_of lidar_service.py)" ]; then
            # lidar_service.py imports lidar.py (klipper/) for the analysis
            PYTHONPATH=$R/klipper setsid nice -n 19 python3 -B \
                $R/helper/lidar_service.py --socket /tmp/ai_server_uds \
                --listen 127.0.0.1:${SVC##*:} --allow 127.0.0.1 \
                --data /usr/data/printer_data/lidar --profile printer \
                < /dev/null > /usr/data/printer_data/logs/lidar-service.log 2>&1 &
        fi
        echo "lidar-start: up, driver $DRIVER, service local" |
            tee /tmp/lidar-start.log ;;
    *)
        echo "lidar-start: up, driver $DRIVER" | tee /tmp/lidar-start.log ;;
esac
