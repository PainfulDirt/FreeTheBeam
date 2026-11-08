"""Offline tests of helper/lidar_camera.py (helper: numpy + PIL) on photos
of the 2026-11-08 test prints on the K1 Max (fixtures/camera/: white matte
PLA on black smooth PEI; the strips' raw scans cut to the profile centre,
the only part the gap analysis uses). Skipped without numpy/PIL."""
import json
import os
import sys
import unittest

try:
    import numpy as np
    import PIL  # noqa: F401
except ImportError:  # pragma: no cover (CI installs them)
    np = None

HERE = os.path.dirname(os.path.abspath(__file__))
FIX = os.path.join(HERE, "fixtures", "camera")
sys.path.insert(0, os.path.join(HERE, ".."))
if np is not None:
    import lidar_camera as cam  # noqa: E402

S1_X = [190.0, 195.0, 200.0, 205.0, 210.0]
S1_W = [0.462, 0.454, 0.472, 0.436, 0.441]   # spike 1 flow lines, mm


def read(name):
    with open(os.path.join(FIX, name), "rb") as f:
        return f.read()


def s1_widths(name):
    img = cam.load_gray(read(name))
    cols = cam.find_lines(img, S1_X, photo_x=200.0)
    k = cam.scale_from_cols(cols, S1_X)
    return [float(np.median(cam.central_rows(cam.line_widths(img, c), img.shape[0])[:, 1])) * k
            for c in cols], k


def spike2():
    with open(os.path.join(FIX, "spike2", "spike2.json")) as f:
        return json.load(f)


def spike2_T_photos():
    return [{"img": cam.load_gray(read("spike2/T%d_e1000_light.jpg" % (y,))),
             "x": 110.0, "y": float(y)} for y in (115, 135, 150, 170, 185, 205)]


def spike2_F_photos():
    return [{"img": cam.load_gray(read("spike2/F%d_e1000_light.jpg" % (y,))),
             "x": 110.5, "y": float(y)} for y in (47, 60, 73)]


def load_vars():
    """lidar_twist / lidar_ytwist as saved on the printer at the time."""
    with open(os.path.join(FIX, "spike2", "tables.json")) as f:
        return json.load(f)


def mesh_fn():
    with open(os.path.join(FIX, "spike2", "mesh.json")) as f:
        m = json.load(f)["result"]["status"]["bed_mesh"]
    M = np.array(m["mesh_matrix"])
    (x0, y0), (x1, y1) = m["mesh_min"], m["mesh_max"]

    def at(x, y):
        fx = (x - x0) / (x1 - x0) * (M.shape[1] - 1)
        fy = (y - y0) / (y1 - y0) * (M.shape[0] - 1)
        i = int(min(max(np.floor(fy), 0), M.shape[0] - 2))
        j = int(min(max(np.floor(fx), 0), M.shape[1] - 2))
        ty, tx = fy - i, fx - j
        return float(M[i, j] * (1 - tx) * (1 - ty) + M[i, j + 1] * tx * (1 - ty)
                     + M[i + 1, j] * (1 - tx) * ty + M[i + 1, j + 1] * tx * ty)
    return at


def strip_rows(k):
    rows, hdr = {}, None
    with open(os.path.join(FIX, "spike2", "spike_g%d.raw.jsonl" % (k,))) as f:
        hdr = json.loads(f.readline())
        for line in f:
            r = json.loads(line)
            caps = [np.array(c, float) for c in r["caps"] if c]
            rows[r["y"]] = np.concatenate(caps)
    return hdr, rows


def spike2_samples():
    fl, v, at = spike2()["flow"], load_vars(), mesh_fn()
    ys = list(np.arange(fl["y0"] + 8, fl["y1"] - 7, 1.0))
    out = {}
    for g in fl["groups"]:
        hdr, rows = strip_rows(g["k"])
        out[g["k"]] = cam.gap_pairs(rows, hdr["x"], ys, at,
                                    v.get("lidar_twist"), v.get("lidar_ytwist"))
    return out


@unittest.skipIf(np is None, "needs numpy and PIL (helper only)")
class CoreTests(unittest.TestCase):
    def test_spike1_flow_line_widths(self):
        w, k = s1_widths("spike1/F_photo_e3000_light.jpg")
        for got, want in zip(w, S1_W):   # spike values were at 0.0194 mm/px
            self.assertAlmostEqual(got, want * k / 0.0194, delta=0.006)

    def test_repeatable_shift(self):
        a, _ = s1_widths("spike1/F_photo_e3000_light.jpg")
        b, _ = s1_widths("spike1/F_shift_e3000_light.jpg")
        for x, y in zip(a, b):
            self.assertAlmostEqual(x, y, delta=0.004)

    def test_scale(self):
        _, k = s1_widths("spike1/F_photo_e3000_light.jpg")
        self.assertTrue(0.0190 <= k <= 0.0198, k)

    def test_missing_line_is_none(self):
        img = cam.load_gray(read("spike1/F_photo_e3000_light.jpg"))
        cols = cam.find_lines(img, S1_X + [212.5], photo_x=200.0)
        self.assertIsNone(cols[-1])
        self.assertTrue(all(c is not None for c in cols[:-1]))


@unittest.skipIf(np is None, "needs numpy and PIL (helper only)")
class PaTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.photos = spike2_T_photos()

    def test_pa_spike2(self):
        r = cam.pa_from_widths(self.photos, spike2()["pa"])
        self.assertTrue(r["ok"], r.get("reason"))
        self.assertAlmostEqual(r["pa"], 0.042, delta=0.006)
        self.assertGreaterEqual(r["r2"], 0.6)
        self.assertEqual(r["lines_found"], 9)

    def test_pa_rejects_flat(self):
        pa = dict(spike2()["pa"])
        pa["pa"] = list(reversed(pa["pa"]))
        r = cam.pa_from_widths(self.photos, pa)
        self.assertFalse(r["ok"])
        self.assertIn("slope", r["reason"])


@unittest.skipIf(np is None, "needs numpy and PIL (helper only)")
class FlowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.photos = spike2_F_photos()
        cls.gaps = cam.group_gaps(spike2_samples())

    def test_flow_spike2(self):
        lines = cam.flow_widths(self.photos, spike2()["flow"])
        r = cam.flow_fit(lines, self.gaps)
        self.assertTrue(r["ok"], r.get("reason"))
        self.assertAlmostEqual(r["f"], 0.94, delta=0.02)
        for s in r["steps_um"].values():
            self.assertTrue(25 <= s <= 70, r["steps_um"])
        for f in (0.9, 1.0, 1.1):
            self.assertAlmostEqual(r["by_flow"][f], r["f"], delta=0.03)
        self.assertTrue(3.5 < r["dfdh"] < 5.5, r["dfdh"])

    def test_gaps_small_on_spike2(self):
        self.assertEqual(len(self.gaps), 4)
        for d in self.gaps.values():
            self.assertLessEqual(abs(d), 0.010)

    def test_flow_rejects_unordered(self):
        fl = json.loads(json.dumps(spike2()["flow"]))
        for g in fl["groups"]:               # claim each group's flows rotated
            fs = [ln["flow"] for ln in g["lines"]]
            for ln, f in zip(g["lines"], fs[1:] + fs[:1]):
                ln["flow"] = f
        r = cam.flow_fit(cam.flow_widths(self.photos, fl), self.gaps)
        self.assertFalse(r["ok"])


INTR_CX = 789.6


def synth_cross(cx_px, cy_px, sx, sy, laser_row, arm=7.0, width=0.6, seed=1):
    """Fill-light picture of a 1-layer cross (bright on dark speckle) and a
    laser picture (5-row line at laser_row, the cross glowing a little)."""
    rng = np.random.default_rng(seed)
    rows, cols = np.mgrid[0:1200, 0:1600].astype(float)
    bg = rng.gamma(1.2, 2.0, (1200, 1600))
    xmm, ymm = (cols - cx_px) * sx, (rows - cy_px) * sy
    def soft(d, half):          # 1 inside, 0 outside, ~1 px edge
        return np.clip(0.5 + (half - np.abs(d)) / (2 * min(sx, sy)), 0, 1)
    xarm = soft(ymm, width / 2) * (np.abs(xmm) <= arm)
    yarm = soft(xmm, width / 2) * (np.abs(ymm) <= arm) * (np.abs(ymm) >= 2.0)
    light = np.clip(bg + 110 * np.maximum(xarm, yarm), 0, 255)
    laser = np.clip(rng.gamma(1.2, 1.5, (1200, 1600))
                    + 200 * np.exp(-0.5 * ((rows - laser_row) / 1.5) ** 2)
                    + 40 * np.maximum(xarm, yarm), 0, 255)
    return light, laser


@unittest.skipIf(np is None, "needs numpy and PIL (helper only)")
class CrossTests(unittest.TestCase):
    def test_cross_synthetic(self):
        light, laser = synth_cross(812.4, 590.7, 0.019, 0.025, 530.0)
        r = cam.cross_from_photos(light, laser)
        self.assertTrue(r["ok"], r.get("reason"))
        self.assertAlmostEqual(r["cx_px"], 812.4, delta=0.3)
        self.assertAlmostEqual(r["cy_px"], 590.7, delta=0.3)
        self.assertAlmostEqual(r["sx"], 0.019, delta=0.019 * 0.01)
        self.assertAlmostEqual(r["sy"], 0.025, delta=0.025 * 0.01)
        self.assertAlmostEqual(cam.laser_row_at(r, INTR_CX), 530.0, delta=0.5)

    def test_cross_real_photos(self):
        # the first real 1-layer cross (2026-11-08): blobby, the Y halves a
        # few px apart, strings; found at both heights
        for z in ("z1", "z2"):
            light = cam.load_gray(read("offset1/%s_light.jpg" % (z,)))
            laser = cam.load_gray(read("offset1/%s_laser.jpg" % (z,)))
            r = cam.cross_from_photos(light, laser)
            self.assertTrue(r["ok"], "%s: %s" % (z, r.get("reason")))
            self.assertTrue(0.017 < r["sx"] < 0.022 and 0.015 < r["sy"] < 0.030, r)

    def test_offset_math(self):
        # planted: offset at Z3 X-36.00 Y-21.30, per mm X+0.035 Y-0.986
        def xo(z): return -36.0 + 0.035 * (z - 3.2)
        def yo(z): return -21.3 - 0.986 * (z - 3.2)
        cx, cy, sx, sy, rl = 150.0, 150.0, 0.0193, 0.0245, 530.0
        per_z = []
        for z, nozzle in ((3.0, (186.3, 171.1)), (5.0, (186.1, 173.2))):
            # the camera finds the laser line on the bare plate (height 0),
            # so it belongs to the offset at height z, not z - line height
            lx, ly = nozzle[0] + xo(z), nozzle[1] + yo(z)
            cxp = INTR_CX - (cx - lx) / sx
            cyp = rl + (cy - ly) / sy
            light, laser = synth_cross(cxp, cyp, sx, sy, rl, seed=int(z))
            per_z.append({"z": z, "nozzle": nozzle,
                          "cross": cam.cross_from_photos(light, laser)})
        r = cam.offset_from_crosses(per_z, {"cx": cx, "cy": cy, "height": 0.2},
                                    INTR_CX, ref_z=3.0)
        self.assertTrue(r["ok"], r.get("reason"))
        self.assertAlmostEqual(r["x_offset"], xo(3.0), delta=0.02)
        self.assertAlmostEqual(r["y_offset"], yo(3.0), delta=0.02)
        self.assertAlmostEqual(r["x_per_z"], 0.035, delta=0.01)
        self.assertAlmostEqual(r["y_per_z"], -0.986, delta=0.01)


if __name__ == "__main__":
    unittest.main()
