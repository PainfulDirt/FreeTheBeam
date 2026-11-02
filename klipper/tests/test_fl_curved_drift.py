"""fl_analyze (klipper/lidar.py, first-layer check) with a curved plate drift:
between the two scans the bare plate changes by a bowl (middle lower), as
seen on 9-pad test prints on a K1 Max. With bare plate between the islands
of one object the bowl must come off (quadratic surface) and the islands
read even; a single solid part has nothing to fit it on inside and keeps the
plane, as before."""
import math
import os
import random
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import lidar  # noqa: E402

H0 = 0.2          # sliced first layer (mm)
BOWL = 0.05       # mm, centre lower than the corners of the area
gx = [lidar.BIN_MIN + (g + 0.5) * lidar.GROUP * lidar.BIN
      for g in range(lidar.NGROUPS)]


def bowl(x, y):
    # 0 in the middle of X/Y 150, +BOWL at the area's corners (+-105 mm)
    return BOWL * (((x - 150) / 105.0) ** 2 + ((y - 150) / 105.0) ** 2) / 2.0


def scans(printed, seed=3):
    """before/after distances per strip (mm): the plate at 25 mm, the layer
    H0 high where printed(x, y), and the after scan reading the plate
    further away by the bowl (the drift between the scans)."""
    rnd = random.Random(seed)
    strips = [46.0 + 26.0 * i for i in range(9)]
    ys = [37.0 + 3.0 * j for j in range(77)]
    before, after = {}, {}
    for sx in strips:
        b_rows, a_rows = [], []
        for y in ys:
            b, a = [], []
            for g in gx:
                x = sx - g          # x_dir -1
                tex = rnd.gauss(0, 0.004)
                b.append(25.0 + tex)
                layer = H0 if printed(x, y) else 0.0
                a.append(25.0 + tex - layer + bowl(x, y)
                         + rnd.gauss(0, 0.002))
            b_rows.append(b)
            a_rows.append(a)
        before[sx], after[sx] = b_rows, a_rows
    return strips, ys, before, after


def plan(strips, ys, polys):
    return {"strips": strips, "ys": ys, "x_dir": -1.0, "z_before": 3.2,
            "z_after": 3.2, "height": H0,
            "objects": [{"name": "o%d" % i, "polygon": p}
                        for i, p in enumerate(polys)]}


def pad_median(tmap, cx, cy):
    v = sorted(t for j, y in enumerate(tmap["ys"]) if abs(y - cy) <= 12
               for i, t in enumerate(tmap["t"][j])
               if t is not None and abs(tmap["x0"] + i * tmap["dx"] - cx) <= 12)
    return v[len(v) // 2] / 1000.0


class CurvedDrift(unittest.TestCase):
    def test_islands_take_off_the_bowl(self):
        centres = [(cx, cy) for cx in (60, 150, 240) for cy in (60, 150, 240)]

        def printed(x, y):
            return any(abs(x - cx) <= 15 and abs(y - cy) <= 15
                       for cx, cy in centres)
        strips, ys, before, after = scans(printed)
        # one object (one STL): its outline is the hull of all the pads
        hull = [[45, 45], [255, 45], [255, 255], [45, 255], [45, 45]]
        res, tmap = lidar.fl_analyze(plan(strips, ys, [hull]), before, after)
        self.assertTrue(res["ok"], res.get("reason"))
        self.assertEqual(res["drift"]["surface"], "quadratic")
        self.assertGreater(res["drift"]["bowl"], 0.6 * BOWL)
        pads = [pad_median(tmap, cx, cy) for cx, cy in centres]
        mean = sum(pads) / len(pads)
        rms = math.sqrt(sum((p - mean) ** 2 for p in pads) / len(pads))
        self.assertLess(rms, 0.004, "pads %s" % (pads,))
        self.assertAlmostEqual(res["median"], H0, delta=0.005)

    def test_single_part_keeps_the_plane(self):
        def printed(x, y):
            return 100 <= x <= 200 and 100 <= y <= 200
        strips, ys, before, after = scans(printed)
        sq = [[100, 100], [200, 100], [200, 200], [100, 200], [100, 100]]
        res, tmap = lidar.fl_analyze(plan(strips, ys, [sq]), before, after)
        self.assertTrue(res["ok"], res.get("reason"))
        self.assertEqual(res["drift"]["surface"], "plane")
        self.assertEqual(res["drift"]["bowl"], 0.0)


if __name__ == "__main__":
    unittest.main()
