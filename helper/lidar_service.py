#!/usr/bin/env python3
# k1-lidar service: the LiDAR work of lidar.py, on a helper computer.
#
# lidar.py (in Klipper on the K1) moves the toolhead; between rows it asks
# this service to capture. The service talks to cx_ai_middleware on the K1
# through lidar-bridge.py, reduces the point clouds, keeps the raw captures
# on disk and runs the analysis. The K1 then only forwards bytes.
#
#   lidar_service.py --printer PRINTER_IP:7130 [--listen 0.0.0.0:7131]
#                    [--data /var/lib/k1-lidar] [--allow PRINTER_IP]
#
# Requests and replies are one JSON object per line.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import argparse
import json
import os
import socket
import socketserver
import threading
import time

import lidar   # the shared reduction / analysis code (lidar.py)

ETX = b"\x03"


class Middleware:
    """One persistent connection to cx_ai_middleware via lidar-bridge.py."""

    def __init__(self, addr):
        self.addr = addr
        self.sock = None

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def call(self, msg, timeout=15.0):
        err = None
        for attempt in range(2):
            try:
                if self.sock is None:
                    self.sock = socket.create_connection(self.addr, 5.0)
                    self.sock.setsockopt(socket.IPPROTO_TCP,
                                         socket.TCP_NODELAY, 1)
                self.sock.settimeout(timeout)
                self.sock.sendall(json.dumps(msg).encode() + ETX)
                buf = b""
                while not buf.endswith(ETX):
                    d = self.sock.recv(1 << 20)
                    if not d:
                        raise OSError("connection closed")
                    buf += d
                return json.loads(buf[:-1].decode(errors="replace"))
            except (OSError, ValueError) as e:
                err = e
                self.close()
        raise RuntimeError("no answer from cx_ai_middleware (%s)" % (err,))


class Service:
    def __init__(self, mw, data_dir):
        self.mw = mw
        self.data_dir = data_dir
        self.scans = {}
        self.lock = threading.Lock()
        os.makedirs(data_dir, exist_ok=True)

    def laser(self, on, level=0):
        if on:
            self.mw.call({"control": "set_laser_open"})
            self.mw.call({"control": "open_flow"})
            self.mw.call({"control": "set_point_cloud_level", "level": level})
        else:
            for c in ("close_flow", "set_laser_close"):
                try:
                    self.mw.call({"control": c})
                except RuntimeError:
                    pass

    def handle(self, req):
        cmd = req.get("cmd")
        if cmd == "ping":
            return {"ok": True}
        if cmd == "middleware":
            return {"ok": True, "reply": self.mw.call(req["msg"])}
        if cmd == "begin":
            name = req["name"]
            self.laser(True, req.get("level", 0))
            raw = open(os.path.join(self.data_dir, name + ".raw.jsonl"), "w")
            raw.write(json.dumps({k: req[k] for k in (
                "x", "z", "x_dir", "x_offset", "y_offset")}) + "\n")
            # keep memory bounded
            while len(self.scans) >= 6:
                old = self.scans.pop(next(iter(self.scans)))
                if old.get("raw"):
                    old["raw"].close()
            self.scans[name] = {"x": req["x"], "z": req["z"],
                                "x_dir": req["x_dir"], "ys": [], "rows": [],
                                "captures": req.get("captures", 3),
                                "grid": req.get("grid"), "frames": [],
                                "raw": raw, "t0": time.time()}
            return {"ok": True}
        if cmd == "frame":
            # dwell scan: one frame for grid row "row", taken standing still
            s = self.scans[req["name"]]
            y0, y1, step = s["grid"]
            y = y0 + req["row"] * (step if y1 >= y0 else -step)
            pc = req["raw"].get("result", {}).get("point_cloud", [])
            s["raw"].write(json.dumps({"y": round(y, 4), "caps": [pc]})
                           + "\n")
            s["frames"].append((req["row"], lidar.reduce_caps([pc])))
            return None
        if cmd == "capture":
            s = self.scans[req["name"]]
            caps = []
            for i in range(s["captures"]):
                r = self.mw.call({"control": "get_point_cloud"})
                caps.append(r.get("result", {}).get("point_cloud", []))
            s["raw"].write(json.dumps({"y": req["y"], "caps": caps}) + "\n")
            s["ys"].append(round(req["y"], 4))
            s["rows"].append(lidar.reduce_caps(caps))
            return {"ok": True, "points": sum(len(c) for c in caps)}
        if cmd == "end":
            s = self.scans[req["name"]]
            self.laser(False)
            s["raw"].close()
            s["raw"] = None
            if s["grid"]:
                s["ys"], s["rows"] = fill_rows(s["frames"], *s["grid"])
                s["n_frames"] = len(s["frames"])
                s["frames"] = []
            self.save(req["name"], s)
            return {"ok": True, "rows": len(s["ys"]),
                    "frames": s.get("n_frames"),
                    "seconds": round(time.time() - s["t0"], 1)}
        if cmd == "pa_analyze":
            base = self.scans.get(req["base"])
            scan = self.scans.get(req["scan"])
            if base is None or scan is None:
                return {"ok": False, "error": "scan %s/%s not here" % (
                    req["base"], req["scan"])}
            if base["ys"] != scan["ys"] or base["x"] != scan["x"]:
                return {"ok": False,
                        "error": "base and scan don't cover the same rows"}
            test = req["test"]
            line_x = [test["x0"] + i * test["pitch"]
                      for i in range(len(test["pa"]))]
            A = [lidar.line_areas(b, r, scan["x"], scan["x_dir"], line_x)
                 for b, r in zip(base["rows"], scan["rows"])]
            res = lidar.pa_fit(scan["ys"], A, test)
            with open(os.path.join(self.data_dir, "pa_result.json"),
                      "w") as f:
                json.dump(dict(res, areas=A, ys=scan["ys"]), f)
            return {"ok": True, "result": res}
        if cmd == "offset_analyze":
            scan = self.scans.get(req["scan"])
            if scan is None:
                return {"ok": False, "error": "scan %s not here" % (
                    req["scan"],)}
            res = lidar.cross_fit(scan, req["test"])
            with open(os.path.join(self.data_dir, req["scan"] + ".fit.json"),
                      "w") as f:
                json.dump(res, f)
            return {"ok": True, "result": res}
        return {"ok": False, "error": "unknown command %r" % (cmd,)}

    def save(self, name, s):
        # same compact format as lidar.py writes on the printer
        with open(os.path.join(self.data_dir, name + ".json"), "w") as f:
            json.dump({"x": s["x"], "z": s["z"], "x_dir": s["x_dir"],
                       "ys": s["ys"], "bin": lidar.BIN,
                       "bin_min": lidar.BIN_MIN,
                       "rows": [[int(v * 1000) if v == v else -1 for v in r]
                                for r in s["rows"]]}, f)


def fill_rows(frames, y0, y1, step):
    """(row, profile) frames -> one profile per grid row; a row without a
    frame is interpolated from its neighbours."""
    n = int(round(abs(y1 - y0) / step))
    d = step if y1 >= y0 else -step
    ys = [round(y0 + i * d, 4) for i in range(n + 1)]
    nan = float("nan")
    by_row = {}
    for r, prof in frames:
        by_row.setdefault(r, []).append(prof)
    have = sorted(by_row)
    rows = []
    for i in range(len(ys)):
        if i in by_row:
            ps = by_row[i]
            rows.append([sum(v) / len(v) if all(x == x for x in v) else nan
                         for v in zip(*ps)])
            continue
        lo = max([r for r in have if r < i], default=None)
        hi = min([r for r in have if r > i], default=None)
        if lo is None or hi is None:
            rows.append([nan] * lidar.NBINS)
            continue
        w = (i - lo) / (hi - lo)
        a, b = by_row[lo][0], by_row[hi][0]
        rows.append([p + (q - p) * w for p, q in zip(a, b)])
    return ys, rows


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        if self.server.allow and self.client_address[0] not in \
                self.server.allow:
            return
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        for line in self.rfile:
            if not line.strip():
                continue
            try:
                req = json.loads(line)
                with self.server.svc.lock:
                    rep = self.server.svc.handle(req)
            except Exception as e:   # report, keep serving
                rep = {"ok": False, "error": "%s: %s" % (
                    type(e).__name__, e)}
            if rep is None:          # frames get no reply
                continue
            self.wfile.write((json.dumps(rep) + "\n").encode())
            self.wfile.flush()


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--printer", required=True,
                    help="host:port of lidar-bridge.py on the printer")
    ap.add_argument("--listen", default="0.0.0.0:7131")
    ap.add_argument("--data", default="/var/lib/k1-lidar")
    ap.add_argument("--allow", action="append", default=[])
    a = ap.parse_args()
    host, port = a.printer.rsplit(":", 1)
    lhost, lport = a.listen.rsplit(":", 1)
    srv = Server((lhost, int(lport)), Handler)
    srv.svc = Service(Middleware((host, int(port))), a.data)
    srv.allow = set(a.allow)
    print("k1-lidar service on %s, LiDAR via %s, data in %s" % (
        a.listen, a.printer, a.data), flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
