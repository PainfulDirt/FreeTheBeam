"""The prime/purge before layer 2 of the first-layer check (fl_prime_plan).

After the scan pause (minutes, nozzle cooler) the nozzle has oozed: the old
prime (retract + 3 mm on a 25 mm line) only refilled it and nothing reached
the plate. Now: refill the tip, then a KAMP-like purge line (1 mm of line
per mm of filament, at KAMP's purge height and speed), in front of the
objects, else left, right or behind; None if nowhere fits.
"""
import unittest

import fake_klipper  # noqa: F401  (klipper/ on sys.path)
import lidar

PADS = [{"name": "pads", "polygon": [[45, 45], [255, 45], [255, 255],
                                     [45, 255], [45, 45]]}]


def box(x0, y0, x1, y1):
    return [{"name": "b", "polygon": [[x0, y0], [x1, y0], [x1, y1],
                                      [x0, y1], [x0, y0]]}]


def anywhere(segs):
    return True


class PrimePlan(unittest.TestCase):
    def plan(self, objects, fits=anywhere, **kw):
        args = dict(tip=5.0, purge=20.0, height=0.5, flow=12.0)
        args.update(kw)
        return lidar.fl_prime_plan(objects, fits=fits, **args)

    def test_front_of_the_pads(self):
        p = self.plan(PADS)
        self.assertEqual(p["side"], "front")
        # 8 mm in front of the objects, centred on them, 1 mm per mm
        self.assertEqual((p["x0"], p["x1"]), (140.0, 160.0))
        self.assertEqual((p["y0"], p["y1"]), (37.0, 37.0))
        self.assertEqual((p["tip"], p["purge"]), (5.0, 20.0))
        self.assertEqual(p["z"], 0.5)
        # KAMP's purge speed: flow_rate / 5 mm/s
        self.assertAlmostEqual(p["feed"], 12.0 / 5.0 * 60.0)

    def test_front_too_close_to_the_edge_goes_left(self):
        p = self.plan(box(60, 6, 120, 80))
        self.assertEqual(p["side"], "left")
        self.assertEqual((p["x0"], p["x1"]), (52.0, 52.0))
        # along Y, centred on the objects, same length
        self.assertEqual((p["y0"], p["y1"]), (33.0, 53.0))

    def test_order_front_left_right_back(self):
        tried = []

        def fits(segs):
            tried.append(segs)
            return len(tried) == 3          # only the third side fits
        p = self.plan(box(100, 100, 200, 200), fits=fits)
        self.assertEqual(p["side"], "right")
        self.assertEqual((p["x0"], p["x1"]), (208.0, 208.0))

        def back_only(segs):
            return segs[0][1] > 200 and segs[0][3] > 200
        p = self.plan(box(100, 100, 200, 200), fits=back_only)
        self.assertEqual(p["side"], "back")
        self.assertEqual((p["y0"], p["y1"]), (208.0, 208.0))

    def test_the_flick_is_checked_too(self):
        seen = []
        self.plan(PADS, fits=lambda segs: seen.append(segs) or True)
        segs = seen[0]
        # the line and the 10 mm flick past its end
        self.assertEqual(segs[0], (140.0, 37.0, 160.0, 37.0))
        self.assertEqual(segs[1], (160.0, 37.0, 170.0, 37.0))

    def test_nowhere_fits(self):
        self.assertIsNone(self.plan(PADS, fits=lambda segs: False))

    def test_no_objects(self):
        self.assertIsNone(self.plan([]))

    def test_short_purge_keeps_a_minimum_line(self):
        p = self.plan(PADS, purge=3.0)
        # very little filament still gets a 10 mm line (not a dot)
        self.assertEqual(p["x1"] - p["x0"], 10.0)

    def test_nothing_to_purge(self):
        self.assertIsNone(self.plan(PADS, tip=0.0, purge=0.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
