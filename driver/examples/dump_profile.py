#!/usr/bin/env python3
# Grab one height profile from a running lidar-direct.py (or Creality's
# cx_ai_middleware: same socket API) and print a summary. No Klipper needed.
#
#   dump_profile.py [--socket /tmp/ai_server_uds] [--out profile.json]
#
# Points are [x, y, z] in micrometres; z is the distance to the surface.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import argparse
import json
import socket
import statistics
import sys

ETX = b"\x03"


class Client:
    """One persistent connection (the middleware leaks a descriptor per
    connection, so never open one per request)."""

    def __init__(self, path, timeout):
        self.sock = socket.socket(socket.AF_UNIX)
        self.sock.settimeout(timeout)
        self.sock.connect(path)

    def call(self, **msg):
        self.sock.sendall(json.dumps(msg).encode() + ETX)
        buf = b""
        while not buf.endswith(ETX):
            d = self.sock.recv(1 << 20)
            if not d:
                raise ConnectionError("socket closed by the daemon")
            buf += d
        return json.loads(buf[:-1])["result"]

    def close(self):
        self.sock.close()


def fetch_profile(socket_path, timeout=5.0):
    """Return one point cloud as [[x, y, z], ...] in micrometres."""
    try:
        c = Client(socket_path, timeout)
    except OSError as e:
        raise ConnectionError("cannot connect to %s (%s): is lidar-direct.py "
                              "running?" % (socket_path, e))
    try:
        for control in ("set_laser_open", "open_flow"):
            r = c.call(control=control)
            if r["code"] != 0:
                raise RuntimeError("%s failed: code %d" % (control, r["code"]))
        r = c.call(control="get_point_cloud")
        if r["code"] != 0:
            raise RuntimeError("get_point_cloud failed: code %d (4 = no data, "
                               "101 = LiDAR not ready)" % r["code"])
        pts = r["point_cloud"]
        if not pts:
            raise RuntimeError("get_point_cloud failed: code 4 (empty frame; "
                               "is the surface 3-5 mm below the nozzle?)")
        return pts
    finally:
        for control in ("close_flow", "set_laser_close"):
            try:
                c.call(control=control)
            except (OSError, ValueError, KeyError):
                pass
        c.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0] if __doc__ else None)
    ap.add_argument("--socket", default="/tmp/ai_server_uds")
    ap.add_argument("--out", help="also save the points as JSON")
    args = ap.parse_args()
    try:
        pts = fetch_profile(args.socket)
    except (ConnectionError, RuntimeError) as e:
        sys.exit("dump_profile: %s" % e)
    xs = [p[0] for p in pts]
    zs = [p[2] for p in pts]
    print("%d points, x %.2f..%.2f mm, median z %.3f mm" % (
        len(pts), min(xs) / 1000, max(xs) / 1000, statistics.median(zs) / 1000))
    if args.out:
        with open(args.out, "w") as f:
            json.dump(pts, f)


if __name__ == "__main__":
    main()
