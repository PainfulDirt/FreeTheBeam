"""lidar.cross_fit on a synthetic 1-layer cross that reads low (thin white
lines: the LiDAR sees ~0.08 mm of a 0.2 mm line)."""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import lidar  # noqa: E402


def scan(seen_h, cx=150.0, cy=150.0, arm=7.0, width=0.6):
    x = 150.0
    ys = [cy - 15 + 0.2 * i for i in range(151)]
    rows = []
    for y in ys:
        row = lidar.array.array("f", [lidar.NAN]) * lidar.NBINS
        for b in range(lidar.NBINS):
            X = x - lidar.bin_center(b)            # x_dir -1
            d = 25.6
            on_y_arm = abs(X - cx) < width / 2 and 2.0 <= abs(y - cy) <= arm
            on_x_arm = abs(y - cy) < width / 2 and abs(X - cx) <= arm
            if on_y_arm or on_x_arm:
                d -= seen_h
            row[b] = d
        rows.append(row)
    return {"x": x, "x_dir": -1.0, "ys": ys, "rows": rows}


class CrossFitTests(unittest.TestCase):
    def test_low_reading_one_layer_cross_found(self):
        r = lidar.cross_fit(scan(0.08), {"cx": 150.0, "cy": 150.0, "arm": 7.0, "height": 0.2})
        self.assertTrue(r["ok"], r.get("reason"))
        self.assertAlmostEqual(r["x"], 150.0, delta=0.1)
        self.assertAlmostEqual(r["y"], 150.0, delta=0.1)


if __name__ == "__main__":
    unittest.main()
