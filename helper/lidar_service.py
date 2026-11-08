#!/usr/bin/env python3
# k1-lidar service: the LiDAR work of lidar.py, on a helper computer.
#
# lidar.py (in Klipper on the K1) moves the toolhead; between rows it asks
# this service to capture. The service talks to the LiDAR driver on the K1
# through lidar-bridge.py, reduces the point clouds, keeps the raw captures
# on disk and runs the analysis. The K1 then only forwards bytes.
#
#   lidar_service.py --printer PRINTER_IP:7130 | --socket /tmp/ai_server_uds
#                    [--listen 0.0.0.0:7131]
#                    [--data /var/lib/k1-lidar] [--allow PRINTER_IP]
#
# Requests and replies are one JSON object per line.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import argparse
import array
import json
import os
import socket
import socketserver
import threading
import time

import lidar   # the shared reduction / analysis code (lidar.py)

ETX = b"\x03"
NAN = float("nan")


class Middleware:
    """One persistent connection to the LiDAR driver: (host, port) of
    lidar-bridge.py on the printer, or the driver's unix socket path when
    it runs on this host."""

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
                if self.sock is None and isinstance(self.addr, str):
                    self.sock = socket.socket(socket.AF_UNIX,
                                              socket.SOCK_STREAM)
                    self.sock.settimeout(5.0)
                    self.sock.connect(self.addr)
                elif self.sock is None:
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
        raise RuntimeError("no answer from the LiDAR driver (%s)" % (err,))


class Service:
    def __init__(self, mw, data_dir):
        self.mw = mw
        self.data_dir = data_dir
        self.scans = {}
        # fine bed mesh being collected: name -> [(x, ys, grouped rows)]
        self.strips = {}
        # first-layer checks: id -> {"before"/"after": {x: {y: groups}}}
        self.fl = {}
        # SKEW_SCAN strips by scan name, until skew_analyze
        self.skew = {}
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
                                "grid": req.get("grid"),
                                "path": req.get("path"), "frames": [],
                                "raw": raw, "t0": time.time()}
            return {"ok": True}
        if cmd == "frame":
            # dwell scan: one frame for grid row "row", taken standing still;
            # moving scan: the frame comes with where it looked (x, y: the
            # middle of its window, span: how far it moved meanwhile)
            s = self.scans[req["name"]]
            pc = req["raw"].get("result", {}).get("point_cloud", [])
            if "y" in req:
                s["path"].append([req["x"], req["y"]])
                s.setdefault("moves", []).append(
                    [req.get("span", 0.0), req.get("dir", 0)])
                s.setdefault("times", []).append(req.get("t"))
                req["row"] = len(s["path"]) - 1
                s["raw"].write(json.dumps({
                    "x": req["x"], "y": req["y"], "span": req.get("span"),
                    "dir": req.get("dir"), "t": req.get("t"),
                    "caps": [pc]}) + "\n")
            elif s["path"] is not None:
                x, y = s["path"][req["row"]]
                s["raw"].write(json.dumps({"x": x, "y": y, "caps": [pc]})
                               + "\n")
            else:
                y0, y1, step = s["grid"]
                y = y0 + req["row"] * (step if y1 >= y0 else -step)
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
            if s["path"] is not None:
                # sparse points: a missed one stays empty
                s["ys"] = [p[1] for p in s["path"]]
                s["rows"] = [array.array("f", [NAN]) * lidar.NBINS
                             for p in s["path"]]
                for r, prof in s["frames"]:
                    s["rows"][r] = prof
                s["n_frames"] = len(s["frames"])
                s["frames"] = []
                if s.get("moves"):
                    self.save_moving(req["name"], s)
            elif s["grid"]:
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
        if cmd == "heights":
            # line-centre heights of a scan, one per row (twist spots)
            scan = self.scans.get(req["scan"])
            if scan is None:
                return {"ok": False, "error": "scan %s not here" % (
                    req["scan"],)}
            hs = lidar.strip_column(scan["rows"], req.get("half_x", 3.0))
            return {"ok": True, "ys": scan["ys"],
                    "h": [round(h, 5) if h == h else None for h in hs]}
        if cmd == "mesh_fast":
            # sparse bed mesh: one path scan, a few frames per grid point
            scan = self.scans.get(req["scan"])
            if scan is None:
                return {"ok": False, "error": "scan %s not here" % (
                    req["scan"],)}
            res = lidar.mesh_points(scan["path"], scan["rows"], req["xs"],
                                    req["ys"], req.get("half_x", 3.0),
                                    req.get("twist"), req.get("half_y", 1.5))
            if res["ok"] and scan.get("moves"):
                res["lag"] = lidar.direction_lag(
                    scan["path"], scan["rows"], scan["moves"],
                    req.get("half_x", 3.0))
            self.save_mesh(req, res)
            return {"ok": True, "result": res}
        if cmd == "mesh_strip":
            # fine bed mesh: keep a strip as 0.5mm groups until mesh_fine
            scan = self.scans.get(req["scan"])
            if scan is None:
                return {"ok": False, "error": "scan %s not here" % (
                    req["scan"],)}
            if req.get("first"):
                self.strips[req["mesh"]] = []
            self.strips.setdefault(req["mesh"], []).append(
                (scan["x"], list(scan["ys"]),
                 [lidar.group_row(r) for r in scan["rows"]]))
            return {"ok": True}
        if cmd == "mesh_fine":
            strips = self.strips.get(req["mesh"])
            if not strips:
                return {"ok": False, "error": "no strips for %s" % (
                    req["mesh"],)}
            res, hmap = lidar.mesh_fine(
                strips, req["xs"], req["ys"], req.get("x_dir", -1.0),
                req.get("half_x", 3.0), req.get("half_y", 2.0),
                req.get("twist"), touches=req.get("touches"),
                max_order=req.get("max_order", 3))
            if res["ok"]:
                # the full-resolution height map, kept for looking at the
                # plate over time (dents, PEI wear)
                hmap.update(req.get("meta") or {})
                stamp = time.strftime("%Y%m%d-%H%M%S")
                for fn in ("heightmap-%s.json" % (stamp,),
                           "heightmap-latest.json"):
                    with open(os.path.join(self.data_dir, fn), "w") as f:
                        json.dump(hmap, f)
                res["map_file"] = os.path.join(
                    self.data_dir, "heightmap-%s.json" % (stamp,))
            self.save_mesh(req, res)
            return {"ok": True, "result": res}
        if cmd == "fl_begin":
            # one check at a time; an old one is dropped
            self.fl = {req["fl"]: {"before": {}, "after": {}}}
            return {"ok": True}
        if cmd == "fl_strip":
            # a strip of the first-layer check, kept as 0.5mm groups by row
            scan = self.scans.get(req["scan"])
            fl = self.fl.get(req["fl"])
            if scan is None or fl is None:
                return {"ok": False, "error": "scan %s / check %s not here"
                        % (req["scan"], req["fl"])}
            fl[req["phase"]][scan["x"]] = dict(
                (round(y, 3), lidar.group_row(r))
                for y, r in zip(scan["ys"], scan["rows"]))
            return {"ok": True}
        if cmd == "fl_analyze":
            fl = self.fl.get(req["fl"])
            if fl is None:
                return {"ok": False, "error": "check %s not here" % (
                    req["fl"],)}
            return {"ok": True, "result": fl_report(
                self.data_dir, req["fl"], req["plan"], fl, req.get("meta")
                or {})}
        if cmd == "skew_strip":
            # SKEW_SCAN: the strips of one scan, every bin (the sides along
            # Y are found along the profile)
            scan = self.scans.get(req["scan"])
            if scan is None:
                return {"ok": False, "error": "scan %s not here" % (
                    req["scan"],)}
            if req.get("first"):
                self.skew[req["sk"]] = []
            self.skew.setdefault(req["sk"], []).append(
                (scan["x"], list(scan["ys"]), [list(r) for r in scan["rows"]]))
            return {"ok": True}
        if cmd == "skew_analyze":
            strips = self.skew.pop(req["sk"], None)
            if not strips:
                return {"ok": False, "error": "no strips for %s" % (
                    req["sk"],)}
            res = lidar.skew_measure(strips, req.get("x_dir", -1.0))
            d = os.path.join(self.data_dir, "skew")
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, req["sk"] + ".json"), "w") as f:
                json.dump({"rot": req.get("rot", 0.0), "result": res,
                           "meta": req.get("meta") or {}}, f)
            return {"ok": True, "result": res}
        if cmd == "skew_solve":
            d = os.path.join(self.data_dir, "skew")
            scans = []
            for n in req["names"]:
                try:
                    with open(os.path.join(d, n + ".json")) as f:
                        s = json.load(f)
                except (OSError, ValueError):
                    return {"ok": False, "error": "no skew scan %s" % (n,)}
                if not s["result"].get("ok"):
                    return {"ok": False, "error": "skew scan %s failed: %s"
                            % (n, s["result"].get("reason"))}
                scans.append((s["rot"], s["result"]))
            res = lidar.skew_solve(scans, req.get("printed_skew", 0.0),
                                   req.get("length", 100.0))
            with open(os.path.join(d, "solve-%s.json" % (
                    time.strftime("%Y%m%d-%H%M%S"),)), "w") as f:
                json.dump({"names": req["names"], "result": res}, f)
            return {"ok": True, "result": res}
        if cmd == "benchmark_save":
            # MESH_BENCHMARK: the run as JSON + an HTML page, kept per run
            d = os.path.join(self.data_dir, "benchmarks")
            os.makedirs(d, exist_ok=True)
            stamp = req["run"].get("stamp") or time.strftime("%Y%m%d-%H%M%S")
            base = os.path.join(d, "bench-%s" % (stamp,))
            with open(base + ".json", "w") as f:
                json.dump(req["run"], f)
            with open(base + ".html", "w") as f:
                f.write(benchmark_html(req["run"]))
            return {"ok": True, "json": base + ".json",
                    "html": base + ".html"}
        # --- camera calibrations (numpy / PIL: imported only here)
        if cmd == "auto_exposure":
            return self.auto_exposure(req.get("min_cover", 0.95))
        if cmd == "photo":
            return self.photo(req)
        if cmd == "pf_plan":
            import lidar_pa_flow
            return dict(lidar_pa_flow.plan_pa_flow(req["params"]), ok=True)
        if cmd == "offset_plan":
            import lidar_pa_flow
            return dict(lidar_pa_flow.plan_offset_cross(req["params"]),
                        ok=True)
        if cmd == "pf_analyze":
            return self.pf_analyze(req)
        if cmd == "offset_cam":
            return self.offset_cam(req)
        return {"ok": False, "error": "unknown command %r" % (cmd,)}

    EXPOSURES = (1000, 2000, 4000, 8000)
    FULL_LINE = 1520          # level 0 points of a fully seen line

    def auto_exposure(self, min_cover=0.95):
        """The lowest exposure that sees (nearly) the whole laser line: dark
        or glossy plates need more light (black smooth PEI ~4000, textured
        PEI 1000). Left set in the module (RAM only)."""
        self.laser(True, 0)
        table, xr = [], {}
        try:
            for e in self.EXPOSURES:
                self.mw.call({"control": "set_laser_exposure", "exposure": e})
                best = []
                for _ in range(3):
                    r = self.mw.call({"control": "get_point_cloud"})
                    pc = r.get("result", {}).get("point_cloud", [])
                    best.append(len(pc))
                    if pc and len(pc) >= max(best):
                        xs = [p[0] for p in pc]
                        xr[e] = [round(min(xs) / 1000.0, 2),
                                 round(max(xs) / 1000.0, 2)]
                table.append([e, sorted(best)[1]])
            top = max(n for e, n in table)
            if top < 0.5 * self.FULL_LINE:
                return {"ok": False, "error": "LiDAR sees less than half of"
                        " the line at exposure %d (%d points): clean the"
                        " window? plate in place?" % (table[-1][0], top)}
            e = next(e for e, n in table if n >= min_cover * top)
            self.mw.call({"control": "set_laser_exposure", "exposure": e})
        finally:
            self.laser(False)
        return {"ok": True, "exposure": e, "points": dict(table)[e],
                "x_range": xr.get(e, [-14.0, 14.0]), "table": table}

    def photo(self, req):
        """One fill-light (or laser) picture through the driver, stored as
        photos/<job>/<name>.jpg with its pose."""
        import base64
        t0 = time.time()
        r = self.mw.call({"control": "ir_photo", "light": req.get("light", 1),
                          "laser": req.get("laser", 0),
                          "exposure": req.get("exposure", 1000), "res": 3},
                         timeout=40.0)
        res = r.get("result", {})
        if res.get("code") or "jpeg" not in res:
            return {"ok": False, "error": "photo %s failed (code %s)" % (
                req["name"], res.get("code"))}
        data = base64.b64decode(res["jpeg"])
        d = os.path.join(self.data_dir, "photos", str(req["job"]))
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, req["name"] + ".jpg"), "wb") as f:
            f.write(data)
        meta = dict((k, req.get(k)) for k in ("x", "y", "z", "exposure",
                                              "light", "laser"))
        meta.update(intr=res.get("intr"), time=time.time())
        with open(os.path.join(d, req["name"] + ".json"), "w") as f:
            json.dump(meta, f)
        try:
            import lidar_camera
            med = float(lidar_camera.np.median(lidar_camera.load_gray(data)))
        except Exception:     # the picture is stored; brightness is a hint
            med = None
        return {"ok": True, "file": os.path.join(d, req["name"] + ".jpg"),
                "bytes": len(data), "secs": round(time.time() - t0, 2),
                "intr": res.get("intr"), "median": med}

    def offset_cam(self, req):
        """LIDAR_OFFSET_CALIBRATE's camera part: the cross and the laser line
        in the pictures taken at each height -> offsets."""
        import lidar_camera as cam
        d = os.path.join(self.data_dir, "photos", str(req["job"]))
        per_z, intr = [], None
        for p in req["photos"]:
            imgs = []
            for kind in ("light", "laser"):
                with open(os.path.join(d, p[kind] + ".jpg"), "rb") as f:
                    imgs.append(cam.load_gray(f.read()))
                with open(os.path.join(d, p[kind] + ".json")) as f:
                    intr = json.load(f).get("intr") or intr
            per_z.append({"z": p["z"], "nozzle": p["nozzle"],
                          "cross": cam.cross_from_photos(*imgs)})
        if not intr:
            return {"ok": False, "error": "no camera intrinsics with the photos"}
        res = cam.offset_from_crosses(per_z, req["test"], intr[2],
                                      req.get("ref_z", 3.0))
        res["crosses"] = [p["cross"] for p in per_z]
        res["ok_cam"] = res.pop("ok")
        return dict(res, ok=True)

    def _raw_rows(self, name):
        """A saved scan's raw rows: (header, {y: points (N, 3) um})."""
        import numpy as np
        rows = {}
        with open(os.path.join(self.data_dir, name + ".raw.jsonl")) as f:
            hdr = json.loads(f.readline())
            for line in f:
                r = json.loads(line)
                caps = [np.array(c, float) for c in r.get("caps", []) if c]
                if caps:
                    rows[r["y"]] = np.concatenate(caps)
        return hdr, rows

    def pf_analyze(self, req):
        """LIDAR_PA_FLOW_CALIBRATE's analysis: PA from the photos at the
        speed changes, flow from the triplet photos and the bare strips."""
        import numpy as np
        import lidar_camera as cam
        test, job = req["test"], str(req["job"])
        d = os.path.join(self.data_dir, "photos", job)

        def photos(prefix):
            out = []
            for ph in test["photos"]:
                fn = os.path.join(d, ph["name"] + ".jpg")
                if ph["name"].startswith(prefix) and os.path.exists(fn):
                    with open(fn, "rb") as f:
                        out.append({"img": cam.load_gray(f.read()),
                                    "x": ph["x"], "y": ph["y"],
                                    "name": ph["name"]})
            return out
        tp, fp = photos("T"), photos("F")
        pa_xs = [test["pa"]["x0"] + i * test["pa"]["pitch"]
                 for i in range(len(test["pa"]["pa"]))]
        fl_xs = [ln["x"] for g in test["flow"]["groups"] for ln in g["lines"]]
        con = [cam.line_contrast(p["img"], cam.find_lines(p["img"], pa_xs, p["x"]))
               for p in tp] + \
            [cam.line_contrast(p["img"], cam.find_lines(p["img"], fl_xs, p["x"]))
             for p in fp]
        contrast = float(np.median(con)) if con else 0.0
        low = contrast < cam.MIN_CONTRAST
        # PA: camera, else the LiDAR scans when the bare step took them
        pa = cam.pa_from_widths(tp, test["pa"]) if tp and not low else {
            "ok": False, "reason": "camera contrast too low (%.0f grey"
            " levels)" % (contrast,), "report": []}
        method = "camera" if pa.get("ok") else "none"
        if not pa.get("ok") and "pf_pa_base" in self.scans \
                and "pf_pa_scan" in self.scans:
            lr = self.handle({"cmd": "pa_analyze", "base": "pf_pa_base",
                              "scan": "pf_pa_scan", "test": test["pa"]})
            if lr.get("ok") and lr["result"].get("ok"):
                pa, method = dict(lr["result"], camera_reason=pa.get(
                    "reason")), "lidar"
        # flow
        if low or len(fp) < len(test["photos"]) - len(tp) or not fp:
            flow = {"ok": False, "reason": "camera contrast too low (%.0f grey"
                    " levels)" % (contrast,) if low else "flow photos"
                    " missing", "report": []}
        else:
            fl = test["flow"]
            ys = list(np.arange(fl["y0"] + 8.0, fl["y1"] - 7.0 + 1e-6, 1.0))
            tables = req.get("tables") or {}
            samples, notes = {}, []
            for st in test["strips"]:
                k = int(st["name"].rsplit("g", 1)[1])
                ms = req.get("mesh", {}).get(st["name"])
                try:
                    hdr, rows = self._raw_rows(st["name"])
                except (OSError, ValueError):
                    notes.append("strip %s missing" % (st["name"],))
                    continue
                if not ms:
                    notes.append("no mesh samples for %s" % (st["name"],))
                    continue
                my, mz = np.array(ms, float).T
                samples[k] = cam.gap_pairs(
                    rows, hdr.get("x", st["x"]), ys,
                    lambda x, y: float(np.interp(y, my, mz)),
                    tables.get("twist"), tables.get("ytwist"))
            gaps = cam.group_gaps(samples)
            flow = cam.flow_fit(cam.flow_widths(fp, fl), gaps,
                                h0=fl.get("height", 0.2),
                                w_nom=fl.get("width", 0.45),
                                h_nom=fl.get("height", 0.2))
            flow["gaps_um"] = dict((str(k), round(v * 1000, 1))
                                   for k, v in gaps.items())
            flow["notes"] = notes
        res = {"pa": pa, "pa_method": method, "flow": flow,
               "contrast": round(contrast, 1), "job": job}
        base = os.path.join(self.data_dir, "pf-%s" % (job,))
        with open(base + ".json", "w") as f:
            json.dump(res, f)
        with open(base + ".html", "w") as f:
            f.write(pf_html(res, test, job))
        res.update(ok=True, html=base + ".html")
        return res

    def save_mesh(self, req, res):
        # the last mesh, and every mesh with a time stamp in meshes/: same
        # grid and coordinates as Klipper's (mesh xs/ys, rows = Y), so later
        # checks (first layer scans) can line up with it
        res = dict(res, request={k: v for k, v in req.items()
                                 if k not in ("cmd", "touches")})
        with open(os.path.join(self.data_dir, "mesh_result.json"), "w") as f:
            json.dump(res, f)
        d = os.path.join(self.data_dir, "meshes")
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "mesh-%s.json" % (
                time.strftime("%Y%m%d-%H%M%S"),)), "w") as f:
            json.dump(res, f)

    def save_moving(self, name, s, keep=20):
        """A small replay file per moving scan (meshes/moving-<stamp>.json):
        per frame where it was put, how far it moved, which way, when it
        arrived and the height at the line centre; the newest `keep`."""
        d = os.path.join(self.data_dir, "meshes")
        os.makedirs(d, exist_ok=True)
        hs = lidar.strip_column(s["rows"])
        fr = [[round(p[0], 3), round(p[1], 3), m[0], m[1], t,
               None if h != h else round(h, 5)]
              for p, m, t, h in zip(s["path"], s["moves"],
                                    s.get("times") or [None] * len(hs), hs)]
        stamp = time.strftime("%Y%m%d-%H%M%S")
        with open(os.path.join(d, "moving-%s.json" % (stamp,)), "w") as f:
            json.dump({"name": name, "time": stamp, "z": s.get("z"),
                       "fields": ["x", "y", "span", "dir", "t", "h"],
                       "frames": fr}, f)
        old = sorted(n for n in os.listdir(d) if n.startswith("moving-"))
        for n in old[:-keep]:
            os.remove(os.path.join(d, n))

    def save(self, name, s):
        # same compact format as lidar.py writes on the printer
        with open(os.path.join(self.data_dir, name + ".json"), "w") as f:
            json.dump({"x": s["x"], "z": s["z"], "x_dir": s["x_dir"],
                       "ys": s["ys"], "path": s.get("path"),
                       "bin": lidar.BIN,
                       "bin_min": lidar.BIN_MIN,
                       "rows": [[int(v * 1000) if v == v else -1 for v in r]
                                for r in s["rows"]]}, f)


def _bilinear(m, x, y):
    xs, ys, z = m["xs"], m["ys"], m["matrix"]
    if not (xs[0] <= x <= xs[-1] and ys[0] <= y <= ys[-1]):
        return None
    i = min(max(int((x - xs[0]) / (xs[-1] - xs[0]) * (len(xs) - 1)), 0),
            len(xs) - 2)
    j = min(max(int((y - ys[0]) / (ys[-1] - ys[0]) * (len(ys) - 1)), 0),
            len(ys) - 2)
    tx = (x - xs[i]) / (xs[i + 1] - xs[i])
    ty = (y - ys[j]) / (ys[j + 1] - ys[j])
    a = z[j][i] + (z[j][i + 1] - z[j][i]) * tx
    b = z[j + 1][i] + (z[j + 1][i + 1] - z[j + 1][i]) * tx
    return a + (b - a) * ty


def _heat_svg(vals, x0, y0, step, lim, title, dots=()):
    """Small SVG heat map: vals[j][i] (mm or None) on a step grid from
    (x0, y0), blue low / red high, +-lim; dots (x, y, mm) as circles."""
    px = 2.0 / step * 5
    ny, nx = len(vals), len(vals[0]) if vals else 0
    w, h = nx * step * px / 5, ny * step * px / 5
    out = ['<figure><svg width="%d" height="%d" viewBox="0 0 %d %d">' % (
        w, h, w, h)]

    def col(v):
        t = max(-1.0, min(1.0, v / lim))
        r = int(255 * (1 if t > 0 else 1 + t))
        b = int(255 * (1 if t < 0 else 1 - t))
        g = int(255 * (1 - abs(t)))
        return "rgb(%d,%d,%d)" % (r, g, b)
    cw = step * px / 5
    for j, row in enumerate(vals):
        for i, v in enumerate(row):
            if v is not None:
                out.append('<rect x="%.1f" y="%.1f" width="%.1f" height='
                           '"%.1f" fill="%s"><title>%.0f um</title></rect>'
                           % (i * cw, h - (j + 1) * cw, cw + 0.5, cw + 0.5,
                              col(v), v * 1000))
    for x, y, v in dots:
        out.append('<circle cx="%.1f" cy="%.1f" r="4" fill="%s" stroke='
                   '"black"><title>%.0f um</title></circle>' % (
                       (x - x0) / step * cw + cw / 2,
                       h - ((y - y0) / step * cw + cw / 2), col(v), v * 1000))
    out.append('</svg><figcaption>%s</figcaption></figure>' % (title,))
    return "".join(out)


def fl_report(data_dir, fid, plan, fl, meta):
    """Analyse a first-layer check, keep it (firstlayer/fl-<id>.json and
    .html, history.jsonl) and fold its residuals into the learned
    correction map (corrmap.json; not applied yet)."""
    xs = sorted(set(fl["before"]) & set(fl["after"]))
    ys = sorted(set(y for x in xs for y in fl["before"][x])
                & set(y for x in xs for y in fl["after"][x]))
    nan = [NAN] * lidar.NGROUPS
    before = dict((x, [fl["before"][x].get(y, nan) for y in ys]) for x in xs)
    after = dict((x, [fl["after"][x].get(y, nan) for y in ys]) for x in xs)
    plan = dict(plan, strips=xs, ys=ys)
    res, tmap = lidar.fl_analyze(plan, before, after)
    d = os.path.join(data_dir, "firstlayer")
    os.makedirs(d, exist_ok=True)
    rec = dict(meta, id=fid, height=plan["height"],
               objects=[o.get("name") for o in plan["objects"]],
               result=dict((k, v) for k, v in res.items() if k != "coarse"))
    with open(os.path.join(d, "history.jsonl"), "a") as f:
        f.write(json.dumps(rec) + "\n")
    if not res.get("ok"):
        return res
    with open(os.path.join(d, "fl-%s.json" % (fid,)), "w") as f:
        json.dump({"meta": meta, "plan": plan, "result": res,
                   "map": tmap}, f)
    html = os.path.join(d, "fl-%s.html" % (fid,))
    with open(html, "w") as f:
        f.write(fl_html(meta, plan, res, tmap))
    # learned correction map: per 10mm cell a running mean of
    # (thickness - median) over the prints, newest weighted 0.3
    cf = os.path.join(d, "corrmap.json")
    try:
        with open(cf) as f:
            cm = json.load(f)
    except (OSError, ValueError):
        cm = {"cell": 10.0, "cells": {}}
    for x, y, v, n in res["coarse"]:
        k = "%d,%d" % (x, y)
        old = cm["cells"].get(k)
        cm["cells"][k] = [round(v if old is None else
                                0.7 * old[0] + 0.3 * v, 4),
                          1 if old is None else old[1] + 1]
    cm["updated"] = meta.get("time")
    with open(cf, "w") as f:
        json.dump(cm, f)
    res["html"] = html
    return res


def fl_html(meta, plan, res, tmap):
    """First-layer check page: numbers and the thickness map (2mm cells,
    thickness - sliced height, +-0.1mm; the bare plate shows dark blue)."""
    h0 = plan["height"]
    t = tmap["t"]
    k = max(1, int(round(plan.get("step", 2.0) / tmap["dx"])))
    vals = []
    for row in t:
        out = []
        for i in range(0, len(row) - k + 1, k):
            v = [c for c in row[i:i + k] if c is not None]
            out.append(sum(v) / len(v) / 1000.0 - h0 if v else None)
        vals.append(out)
    step = tmap["dx"] * k
    rows = "".join("<tr><td>%s</td><td>%.3f</td><td>%d</td></tr>" % (
        o["name"], o["median"], o["cells"]) for o in res["objects"])
    return ("<!doctype html><meta charset='utf-8'><title>First layer %s"
            "</title><body style='font:14px sans-serif;max-width:900px;"
            "margin:20px auto'><h2>First layer: %.3f mm measured, %.3f mm"
            " sliced (%+.3f)</h2><p>%s, %s, %s. p05-p95 %.3f-%.3f mm over"
            " %.0f mm&sup2;; thickness tilt X %+.0f, Y %+.0f um/100mm;"
            " plate drift between the scans %+.0f um.</p><table border=1"
            " cellpadding=4><tr><th>object</th><th>median mm</th><th>cells"
            "</th></tr>%s</table>%s</body>" % (
                meta.get("time", ""), res["median"], h0, res["error"],
                meta.get("file") or "", meta.get("filament_name")
                or meta.get("filament") or "", meta.get("time", ""),
                res["p05"], res["p95"], res["area_mm2"],
                res["tilt_um_per_100mm"][0], res["tilt_um_per_100mm"][1],
                res["drift"]["plane"][0] * 1000.0, rows,
                _heat_svg(vals, tmap["x0"], plan["ys"][0], step, 0.1,
                          "thickness - %.2f mm (red thicker, blue thinner;"
                          " bare plate dark blue)" % (h0,))))


def pf_html(res, test, job):
    """LIDAR_PA_FLOW_CALIBRATE report: results, the PA scores against PA
    (inline SVG), the flow lines, links to the photos."""
    pa, fl = res["pa"], res["flow"]
    rows = []
    sc = [(p, s) for p, s in zip(test["pa"]["pa"], pa.get("scores") or [])
          if s is not None]
    svg = ""
    if sc:
        lo, hi = min(s for p, s in sc), max(s for p, s in sc)
        span = (hi - lo) or 1.0
        p0, p1 = test["pa"]["pa"][0], test["pa"]["pa"][-1]
        X = lambda p: 20 + 360 * (p - p0) / ((p1 - p0) or 1.0)
        Y = lambda s: 180 - 160 * (s - lo) / span
        pts = "".join('<circle cx="%.1f" cy="%.1f" r="4"/>' % (X(p), Y(s))
                      for p, s in sc)
        zero = '<line x1="20" x2="380" y1="%.1f" y2="%.1f" stroke="#888"/>' % (
            Y(0), Y(0)) if lo <= 0 <= hi else ""
        svg = ('<svg width="400" height="200" style="background:#fff">%s%s'
               '</svg>' % (zero, pts))
    for line in (pa.get("report") or []) + (fl.get("report") or []):
        rows.append("<div><code>%s</code></div>" % (line,))
    pics = "".join('<a href="photos/%s/%s.jpg">%s</a> ' % (job, p["name"],
                                                         p["name"])
                   for p in test["photos"])
    return ("<!doctype html><meta charset=utf-8><title>LiDAR PA + flow %s"
            "</title><body style='font-family:sans-serif'>"
            "<h2>PA %s (%s)</h2><h2>Flow %s</h2>"
            "<p>camera contrast %.0f grey levels; gap differences %s um</p>"
            "<h3>PA score per line (zero crossing = PA)</h3>%s<h3>Details</h3>"
            "%s<h3>Photos</h3>%s</body>" % (
                job, "%.4f" % pa["pa"] if pa.get("ok") else "not measured: %s"
                % pa.get("reason"), res["pa_method"],
                "%.3f (sd %.3f, dF/dh %.2f per mm)" % (fl["f"], fl["sd"],
                                                         fl["dfdh"])
                if fl.get("ok") else "not saved: %s" % (fl.get("reason"),),
                res["contrast"], fl.get("gaps_um", {}), svg, "".join(rows),
                pics))


def benchmark_html(run):
    """MESH_BENCHMARK report: a table and per method the difference to the
    reference mesh (+-50 um), the nozzle touches as dots."""
    rows = []
    for m in run["methods"]:
        def f(st, k):
            return "%.1f" % (st[k] * 1000) if st and k in st else "-"
        rows.append("<tr><td>%s</td><td>%.0f s</td><td>%dx%d</td>"
                    "<td>%s</td><td>%s</td><td>%s</td><td>%s</td>"
                    "<td>%s</td><td>%s</td></tr>" % (
                        m["name"], m.get("time", 0), len(m["xs"]),
                        len(m["ys"]), f(m.get("vs_nozzle"), "rms"),
                        f(m.get("vs_nozzle"), "max"),
                        f(m.get("vs_nozzle"), "rms_plane"),
                        f(m.get("vs_probe"), "rms"),
                        f(m.get("repeat"), "rms"), m.get("note", "")))
    ref = next((m for m in run["methods"] if m["name"] == run.get("ref")),
               None)
    maps = []
    step = 10.0
    for m in run["methods"]:
        x0, x1 = m["xs"][0], m["xs"][-1]
        y0, y1 = m["ys"][0], m["ys"][-1]
        gx = [x0 + step * i for i in range(int((x1 - x0) / step) + 1)]
        gy = [y0 + step * j for j in range(int((y1 - y0) / step) + 1)]
        vals = []
        for y in gy:
            row = []
            for x in gx:
                a = _bilinear(m, x, y)
                b = _bilinear(ref, x, y) if ref and ref is not m else 0.0
                row.append(None if a is None or b is None else a - b)
            vals.append(row)
        flat = [v for r in vals for v in r if v is not None]
        mu = sum(flat) / len(flat) if flat else 0.0
        vals = [[None if v is None else v - mu for v in r] for r in vals]
        dots = [(t[0], t[1], t[2]) for t in m.get("touch_res", [])]
        maps.append(_heat_svg(vals, x0, y0, step, 0.05, "%s %s" % (
            m["name"], "- " + ref["name"] if ref and ref is not m
            else "(own shape)"), dots))
    return """<!doctype html><html><head><meta charset="utf-8">
<title>Mesh benchmark %s</title><style>
body{font-family:sans-serif;margin:16px;background:#fff;color:#111}
table{border-collapse:collapse}td,th{border:1px solid #ccc;padding:3px 8px;
text-align:right}td:first-child,td:last-child{text-align:left}
figure{display:inline-block;margin:8px}figcaption{font-size:12px}
</style></head><body><h1>Mesh benchmark %s</h1><p>%s</p>
<table><tr><th>method</th><th>time</th><th>grid</th><th>rms vs nozzle um</th>
<th>max</th><th>rms -plane</th><th>rms vs probe</th><th>repeat rms</th>
<th>note</th></tr>%s</table>
<p>Maps: method minus %s, +-50 um (blue low, red high); dots: nozzle
touches minus the method (held out).</p>%s</body></html>""" % (
        run.get("stamp"), run.get("stamp"), run.get("summary", ""),
        "".join(rows), ref["name"] if ref else "-", "".join(maps))


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


def build_parser():
    ap = argparse.ArgumentParser()
    drv = ap.add_mutually_exclusive_group(required=True)
    drv.add_argument("--printer",
                     help="host:port of lidar-bridge.py on the printer")
    drv.add_argument("--socket",
                     help="the driver's unix socket, when it runs on this"
                          " host (e.g. /tmp/ai_server_uds)")
    ap.add_argument("--listen", default="0.0.0.0:7131")
    ap.add_argument("--data", default="/var/lib/k1-lidar")
    ap.add_argument("--allow", action="append", default=[])
    return ap


def main():
    a = build_parser().parse_args()
    if a.socket:
        addr = a.socket
    else:
        host, port = a.printer.rsplit(":", 1)
        addr = (host, int(port))
    lhost, lport = a.listen.rsplit(":", 1)
    srv = Server((lhost, int(lport)), Handler)
    srv.svc = Service(Middleware(addr), a.data)
    srv.allow = set(a.allow)
    print("k1-lidar service on %s, LiDAR via %s, data in %s" % (
        a.listen, a.socket or a.printer, a.data), flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
