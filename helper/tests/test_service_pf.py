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
import test_camera as tc  # noqa: E402

PHOTO = {"F1": "F47", "F2": "F60", "F3": "F73", "T1": "T115", "T2": "T135",
         "T3": "T150", "T4": "T170", "T5": "T185", "T6": "T205",
         "plate": "T150"}
POINTS = {1000: 1060, 2000: 1290, 4000: 1499, 8000: 1520}
P = {"extruder_temp": 210, "bed_temp": 60, "filament_name": "PLA Matte",
     "filament_type": "PLA", "x": 100.0, "y": 35.0, "pa_keep": 0.0475,
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
            with open(os.path.join(FIX, PHOTO[self.next_photo] + "_e1000_light.jpg"),
                      "rb") as f:
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


def analyze_req(s, d, test, job):
    at, v = tc.mesh_fn(), tc.load_vars()
    mesh = {}
    for st in test["strips"]:
        shutil.copy(os.path.join(FIX, "spike_g%s.raw.jsonl" % st["name"][-1]),
                    os.path.join(d, st["name"] + ".raw.jsonl"))
        mesh[st["name"]] = [[y, at(st["x"], y)] for y in
                            np.arange(st["y0"], st["y1"] + 0.01, 1.0)]
    return {"cmd": "pf_analyze", "job": job, "test": test, "mesh": mesh,
            "tables": {"twist": v.get("lidar_twist"), "ytwist": v.get("lidar_ytwist")}}


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
        self.assertTrue(r["flow"]["ok"], r["flow"]["reason"])
        self.assertAlmostEqual(r["flow"]["f"], 0.94, delta=0.02)
        self.assertTrue(os.path.exists(r["html"]))

    def test_low_contrast_falls_back(self):
        s, d = make_service(dim=0.125)
        test = s.handle({"cmd": "pf_plan", "params": P})["test"]
        take_photos(s, test, "j3")
        r = s.handle(analyze_req(s, d, test, "j3"))
        self.assertTrue(r["ok"], r)
        self.assertIn(r["pa_method"], ("lidar", "none"))
        self.assertFalse(r["flow"]["ok"])
        self.assertIn("contrast", r["flow"]["reason"])


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
