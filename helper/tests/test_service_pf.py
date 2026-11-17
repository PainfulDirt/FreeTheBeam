"""lidar_service.py's camera commands (auto_exposure, photo, pf_plan,
pf_analyze, offset_cam) with a fake driver link answering from the
2026-11-08 fixtures. Skipped without numpy/PIL."""
import base64
import io
import json
import math
import os
import shutil
import sys
import tempfile
import unittest

try:
    import numpy as np
    import PIL  # noqa: F401
except ImportError:  # pragma: no cover (CI installs them)
    np = None

HERE = os.path.dirname(os.path.abspath(__file__))
FIX = os.path.join(HERE, "fixtures", "camera", "spike2")
sys.path[:0] = [os.path.join(HERE, ".."), os.path.join(HERE, "..", "..",
                                                     "klipper"), HERE]
import lidar_service as svc_mod  # noqa: E402
import test_bands as tb  # noqa: E402
import test_camera as tc  # noqa: E402

PHOTO = {"F1": "F47", "F2": "F60", "F3": "F73", "T1": "T115", "T2": "T135",
         "T3": "T150", "T4": "T170", "T5": "T185", "T6": "T205",
         "plate": "T150"}
BANDS = os.path.join(HERE, "fixtures", "camera", "bands")
BAND_PHOTO = dict(("B%d" % (i + 1), "band_%d_e1000.jpg" % f)
                  for i, f in enumerate((92, 96, 100, 104, 108, 112)))
POINTS = {1000: 1060, 2000: 1290, 4000: 1499, 8000: 1520}
P = {"tests": ["pa", "flow"], "flow_method": "ir",
     "extruder_temp": 210, "bed_temp": 60, "filament_name": "PLA Matte",
     "filament_type": "PLA", "x": 100.0, "y": 75.0, "pa_keep": 0.0475,
     "fil_area": math.pi * 1.75 ** 2 / 4, "max_accel": 20000.0}


class FakeMW:
    def __init__(self, dim=1.0):
        self.exposure, self.calls, self.next_photo, self.dim = 1000, [], None, dim

    def call(self, msg, timeout=15.0):
        c = msg["control"]
        self.calls.append(msg)
        if c == "set_laser_exposure":
            self.exposure = msg["exposure"]
        if c == "get_point_cloud":
            n = POINTS[self.exposure]
            pc = [[int((i - n / 2) * 19), 0, 25600] for i in range(n)]
            return {"control": c, "result": {"code": 0, "point_cloud": pc}}
        if c == "ir_photo":
            fn = os.path.join(BANDS, BAND_PHOTO[self.next_photo]) \
                if self.next_photo in BAND_PHOTO else \
                os.path.join(FIX, PHOTO[self.next_photo] + "_e1000_light.jpg")
            with open(fn, "rb") as f:
                data = f.read()
            if self.dim != 1.0:
                from PIL import Image
                img = np.asarray(Image.open(io.BytesIO(data)).convert("L"), float)
                out = io.BytesIO()
                Image.fromarray((img * self.dim).astype(np.uint8)).save(out, "JPEG", quality=95)
                data = out.getvalue()
            return {"control": c, "result": {
                "code": 0, "jpeg": base64.b64encode(data).decode(),
                "w": 1600, "h": 1200, "exposure": msg.get("exposure"),
                "intr": [1344.7, 1344.7, 789.6, 605.1], "secs": 3.0}}
        return {"control": c, "result": {"code": 0}}


def make_service(dim=1.0):
    d = tempfile.mkdtemp()
    s = svc_mod.Service(FakeMW(dim), d)
    return s, d


def take_photos(s, test, job):
    for ph in test["photos"] + [dict(test["plate_photo"], name="plate")]:
        s.mw.next_photo = ph["name"]
        r = s.handle({"cmd": "photo", "job": job, "name": ph["name"], "x": ph["x"],
                      "y": ph["y"], "z": 3.0, "exposure": 1000, "light": 1,
                      "laser": 0})
        assert r["ok"], r


def write_raw(d, name, x, rows):
    with open(os.path.join(d, name + ".raw.jsonl"), "w") as f:
        f.write(json.dumps({"x": x, "z": 3.0, "x_dir": -1.0}) + "\n")
        for y, pts in sorted(rows.items()):
            f.write(json.dumps({"y": y, "caps": [pts.astype(int).tolist()]}) + "\n")


def lidar_scans(d, test, ridges=(0.020, 0.015, 0.010, 0.006, 0.004, 0.003)):
    """Bare and printed strips of every band: ridges fall with flow."""
    for st, b, r in zip(test["strips"], test["flow"]["bands"], ridges):
        ys = [round(st["y0"] + 0.4 * i, 3)
              for i in range(int((st["y1"] - st["y0"]) / 0.4) + 1)]
        write_raw(d, st["name"], st["x"],
                  tb.synth_rows(ys, st["x"], -1.0, b, 0, 0, 1, bare=True))
        write_raw(d, st["name"] + "_after", st["x"],
                  tb.synth_rows(ys, st["x"], -1.0, b, 0.15, r, 2))


def analyze_req(s, d, test, job):
    return {"cmd": "pf_analyze", "job": job, "test": test}


@unittest.skipIf(np is None, "needs numpy and PIL (helper only)")
class ServiceTests(unittest.TestCase):
    def test_auto_exposure_picks_4000(self):
        s, _ = make_service()
        r = s.handle({"cmd": "auto_exposure", "min_cover": 0.95})
        self.assertTrue(r["ok"], r)
        self.assertEqual(r["exposure"], 4000)
        self.assertEqual(len(r["table"]), 4)
        sets = [m["exposure"] for m in s.mw.calls if m["control"] == "set_laser_exposure"]
        self.assertEqual(sets[-1], 4000)

    def test_photo_stored(self):
        s, d = make_service()
        s.mw.next_photo = "F1"
        r = s.handle({"cmd": "photo", "job": "j1", "name": "F1", "x": 110.5,
                      "y": 47.0, "z": 3.0, "exposure": 1000, "light": 1, "laser": 0})
        self.assertTrue(r["ok"], r)
        size = os.path.getsize(os.path.join(FIX, "F47_e1000_light.jpg"))
        self.assertEqual(r["bytes"], size)
        self.assertEqual(os.path.getsize(os.path.join(d, "photos", "j1", "F1.jpg")), size)

    def test_pf_plan(self):
        s, _ = make_service()
        r = s.handle({"cmd": "pf_plan", "params": P})
        self.assertTrue(r["ok"], r)
        self.assertIn("LIDAR_PA_FLOW_STEP STEP=PHOTOS", r["gcode"])

    def test_pf_analyze_spike2(self):
        s, d = make_service()
        test = s.handle({"cmd": "pf_plan", "params": P})["test"]
        take_photos(s, test, "j2")
        r = s.handle(analyze_req(s, d, test, "j2"))
        self.assertTrue(r["ok"], r)
        self.assertEqual(r["pa_method"], "camera")
        self.assertAlmostEqual(r["pa"]["pa"], 0.042, delta=0.006)
        self.assertTrue(r["flow"]["ok"], r["flow"].get("reason"))
        self.assertAlmostEqual(r["flow"]["flow"], 1.04)
        self.assertEqual(r["flow_method"], "ir")
        self.assertTrue(os.path.exists(r["html"]))

    def test_pa_only(self):
        s, d = make_service()
        test = s.handle({"cmd": "pf_plan", "params": dict(P, tests=["pa"], y=100.0)})["test"]
        take_photos(s, test, "j4")
        r = s.handle(analyze_req(s, d, test, "j4"))
        self.assertAlmostEqual(r["pa"]["pa"], 0.042, delta=0.006)
        self.assertEqual(r["pa_mode"], "ir")
        self.assertIsNone(r["flow"])

    def test_flow_only_skips_pa(self):
        s, d = make_service()
        test = s.handle({"cmd": "pf_plan", "params": dict(P, tests=["flow"])})["test"]
        take_photos(s, test, "j5")
        r = s.handle(analyze_req(s, d, test, "j5"))
        self.assertIsNone(r["pa"])
        self.assertEqual(r["pa_method"], "none")
        self.assertAlmostEqual(r["flow"]["flow"], 1.04)
        self.assertTrue(os.path.exists(r["html"]))

    def test_flow_lidar(self):
        s, d = make_service()
        test = s.handle({"cmd": "pf_plan", "params": dict(
            P, tests=["flow"], flow_method="lidar")})["test"]
        take_photos(s, test, "j6")          # only the plate photo
        lidar_scans(d, test)
        r = s.handle(analyze_req(s, d, test, "j6"))
        self.assertTrue(r["flow"]["ok"], r["flow"].get("reason"))
        self.assertAlmostEqual(r["flow"]["flow"], 1.04)
        self.assertEqual(r["flow_method"], "lidar")
        self.assertIn("lidar", r["flows"])

    def test_flow_lidar_scan_missing(self):
        s, d = make_service()
        test = s.handle({"cmd": "pf_plan", "params": dict(
            P, tests=["flow"], flow_method="lidar")})["test"]
        r = s.handle(analyze_req(s, d, test, "j7"))
        self.assertFalse(r["flow"]["ok"])
        self.assertIn("scan", r["flow"]["reason"])

    def flat_pa_scans(self, s, test):
        st = test["pa_strip"]
        ys = [round(st["y0"] + 0.4 * i, 3)
              for i in range(int((st["y1"] - st["y0"]) / 0.4) + 1)]
        for name in ("pf_pa_base", "pf_pa_scan"):
            # gen: scanned in this calibration job (since its pf_plan)
            s.scans[name] = {"x": st["x"], "z": 3.0, "x_dir": -1.0, "ys": ys,
                             "rows": [[25.0] * svc_mod.lidar.NBINS for _ in ys],
                             "gen": s.pf_gen}

    def test_pa_lidar_only(self):
        s, d = make_service()
        test = s.handle({"cmd": "pf_plan", "params": dict(
            P, tests=["pa"], pa_method="lidar")})["test"]
        self.flat_pa_scans(s, test)
        r = s.handle(analyze_req(s, d, test, "jl"))
        self.assertEqual(r["pa_mode"], "lidar")
        self.assertFalse(r["pa"]["ok"])
        self.assertIn("lines", r["pa"]["reason"])     # flat plate: no lines
        self.assertEqual(list(r["pas"]), ["lidar"])

    def test_pa_lidar_scans_missing(self):
        s, d = make_service()
        test = s.handle({"cmd": "pf_plan", "params": dict(
            P, tests=["pa"], pa_method="lidar")})["test"]
        r = s.handle(analyze_req(s, d, test, "jm"))
        self.assertFalse(r["pa"]["ok"])
        self.assertIn("missing", r["pa"]["reason"])

    def test_pa_flow_benchmark(self):
        s, d = make_service()
        test = s.handle({"cmd": "pf_plan", "params": dict(
            P, pa_method="both", flow_method="both")})["test"]
        take_photos(s, test, "jb")
        lidar_scans(d, test)
        self.flat_pa_scans(s, test)
        r = s.handle(analyze_req(s, d, test, "jb"))
        self.assertIsNone(r["pa"])
        self.assertIsNone(r["flow"])
        self.assertAlmostEqual(r["pas"]["ir"]["pa"], 0.042, delta=0.006)
        self.assertFalse(r["pas"]["lidar"]["ok"])
        self.assertEqual(sorted(r["flows"]), ["ir", "lidar"])
        html = open(r["html"]).read()
        self.assertIn("PA (camera)", html)
        self.assertIn("PA (LiDAR)", html)

    def test_flow_benchmark_both(self):
        s, d = make_service()
        test = s.handle({"cmd": "pf_plan", "params": dict(
            P, tests=["flow"], flow_method="both")})["test"]
        take_photos(s, test, "j8")
        lidar_scans(d, test)
        r = s.handle(analyze_req(s, d, test, "j8"))
        self.assertEqual(r["flow_method"], "both")
        self.assertAlmostEqual(r["flows"]["ir"]["flow"], 1.04)
        self.assertAlmostEqual(r["flows"]["lidar"]["flow"], 1.04)
        self.assertIsNone(r["flow"])                 # the user chooses
        html = open(r["html"]).read()
        self.assertIn("camera", html)
        self.assertIn("LiDAR", html)

    def test_dark_band_photos_refused(self):
        s, d = make_service(dim=0.125)
        test = s.handle({"cmd": "pf_plan", "params": dict(P, tests=["flow"])})["test"]
        take_photos(s, test, "j9")
        r = s.handle(analyze_req(s, d, test, "j9"))
        self.assertFalse(r["flow"]["ok"])
        self.assertIn("dark", r["flow"]["reason"])

    def test_low_contrast_falls_back(self):
        s, d = make_service(dim=0.125)
        test = s.handle({"cmd": "pf_plan", "params": P})["test"]
        take_photos(s, test, "j3")
        r = s.handle(analyze_req(s, d, test, "j3"))
        self.assertTrue(r["ok"], r)
        self.assertIn(r["pa_method"], ("lidar", "none"))
        self.assertIn("contrast", r["pa"]["reason"])


@unittest.skipIf(np is None, "needs numpy and PIL (helper only)")
class SafetyTests(unittest.TestCase):
    """QA 2026-11-16 P7/P2-03: names, begin order, frames, fl_html."""

    def test_bad_names_rejected(self):
        s, d = make_service()
        for req in ({"cmd": "photo", "job": "../x", "name": "F1"},
                    {"cmd": "photo", "job": "j", "name": ".hidden"},
                    {"cmd": "skew_solve", "names": ["a/b"]},
                    {"cmd": "fl_analyze", "fl": "..", "plan": {}}):
            r = s.handle(req)
            self.assertFalse(r["ok"])
            self.assertIn("bad", r["error"])
        self.assertEqual(svc_mod.safe_name("pf_b1_after.v2"), "pf_b1_after.v2")

    def test_begin_disk_error_leaves_laser_off(self):
        s, d = make_service()
        os.rmdir(d)                     # open() of the raw file fails now
        with self.assertRaises(OSError):
            s.handle({"cmd": "begin", "name": "x", "x": 1, "z": 3,
                      "x_dir": -1, "x_offset": 0, "y_offset": 0})
        self.assertNotIn("set_laser_open",
                         [m["control"] for m in s.mw.calls])

    def test_end_unknown_scan_turns_laser_off(self):
        s, d = make_service()
        r = s.handle({"cmd": "end", "name": "nothing"})
        self.assertFalse(r["ok"])
        self.assertIn("set_laser_close", [m["control"] for m in s.mw.calls])

    def test_bad_frames_never_answered(self):
        s, d = make_service()
        s.handle({"cmd": "begin", "name": "sc", "x": 1, "z": 3, "x_dir": -1,
                  "x_offset": 0, "y_offset": 0, "grid": [0, 1, 0.5]})
        self.assertIsNone(svc_mod.Handler.one(
            s, b'{"cmd": "frame", "name": "sc", "row": 0, "raw": {"x"\x03}\n'))
        self.assertIsNone(svc_mod.Handler.one(
            s, b'{"cmd": "frame", "name": "sc", "row": 0}\n'))  # KeyError
        self.assertIsNone(svc_mod.Handler.one(
            s, b'{"cmd": "frame", "name": "gone", "row": 0, "raw": {}}\n'))
        r = s.handle({"cmd": "end", "name": "sc"})
        self.assertEqual(r["bad_frames"], 2)
        self.assertFalse(svc_mod.Handler.one(s, b'{"cmd": "nope"\n')["ok"])

    def test_fl_html_without_tilt(self):
        res = {"median": 0.2, "error": 0.0, "p05": 0.19, "p95": 0.21,
               "area_mm2": 900.0, "tilt_um_per_100mm": None,
               "drift": {"plane": [None, 0, 0]}, "objects": []}
        tmap = {"t": [[200.0, None], [None, 201.0]], "dx": 1.0, "x0": 0.0}
        plan = {"height": 0.2, "ys": [0.0, 1.0], "step": 1.0}
        html = svc_mod.fl_html({}, plan, res, tmap)
        self.assertIn("X n/a, Y n/a", html)
        res["tilt_um_per_100mm"] = [None, 5.0]
        self.assertIn("Y +5", svc_mod.fl_html({}, plan, res, tmap))


@unittest.skipIf(np is None, "needs numpy and PIL (helper only)")
class OffsetCamTests(unittest.TestCase):
    def test_offset_cam_from_stored_photos(self):
        from PIL import Image
        s, d = make_service()
        job = "offset-1"
        jd = os.path.join(d, "photos", job)
        os.makedirs(jd)
        shots = []
        for i, (z, noz) in enumerate(((3.0, (186.0, 171.3)), (5.0, (186.0, 173.3)))):
            lx, ly = noz[0] - 36.0, noz[1] - 21.3 - 0.986 * (z - 3.0)   # laser on the plate: height z
            cxp = 789.6 - (150.0 - lx) / 0.0193
            cyp = 530.0 + (150.0 - ly) / 0.0245
            light, laser = tc.synth_cross(cxp, cyp, 0.0193, 0.0245, 530.0, seed=i)
            for kind, img in (("light", light), ("laser", laser)):
                name = "z%d_%s" % (i + 1, kind)
                Image.fromarray(img.astype(np.uint8)).save(
                    os.path.join(jd, name + ".jpg"), "JPEG", quality=95)
                with open(os.path.join(jd, name + ".json"), "w") as f:
                    json.dump({"intr": [1344.7, 1344.7, 789.6, 605.1]}, f)
            shots.append({"z": z, "light": "z%d_light" % (i + 1),
                          "laser": "z%d_laser" % (i + 1), "nozzle": list(noz)})
        r = s.handle({"cmd": "offset_cam", "job": job, "photos": shots,
                      "test": {"cx": 150.0, "cy": 150.0, "height": 0.2}, "ref_z": 3.0})
        self.assertTrue(r["ok"] and r["ok_cam"], r.get("reason"))
        self.assertAlmostEqual(r["x_offset"], -36.0, delta=0.03)
        self.assertAlmostEqual(r["y_offset"], -21.3, delta=0.03)


if __name__ == "__main__":
    unittest.main()
