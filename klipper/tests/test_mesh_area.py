"""Regression (K1 bug hunt 2026-11-16 #1, #20): the adaptive LiDAR mesh area
(Lidar._area).

#1: when ONE side of the area was < 20 mm, BOTH sides were reset to ~20 mm
    around the centre, so a long thin part got a mesh covering only a
    fraction of it. The mesh must cover the objects and the
    zero_reference_position.
#20: ADAPTIVE=0 FALLBACK=1 with an object beyond the LiDAR's reach must
    raise (probe fallback).
"""
import unittest

import regress_fakes as F
import lidar

# a 300x300 bed: travel X-2..306.5 Y-2..306, laser 36 mm left of and
# 21.3 mm in front of the nozzle -> the LiDAR sees X0-270 Y0-284
AXIS_MIN, AXIS_MAX = (-2.0, -2.0), (306.5, 306.0)
ZERO_REF = (153.0, 173.3)


def make(polys):
    objs = {
        "toolhead": F.Obj(get_status=lambda e: {
            "homed_axes": "xyz",
            "axis_minimum": [AXIS_MIN[0], AXIS_MIN[1], -10.0, 0.0],
            "axis_maximum": [AXIS_MAX[0], AXIS_MAX[1], 302.0, 0.0]}),
        "exclude_object": F.Obj(get_status=lambda e: {"objects": [
            {"name": "o%d" % i, "polygon": p} for i, p in enumerate(polys)]}),
        "bed_mesh": F.Obj(bmc=F.Obj(zero_ref_pos=list(ZERO_REF))),
    }
    L = object.__new__(lidar.Lidar)
    L.printer = F.Obj(lookup_object=lambda n, d=None: objs.get(n, d),
                      command_error=F.Err)
    L.reactor = F.Obj(monotonic=lambda: 0.0)
    L.x_offset, L.y_offset, L.offset_z = -36.013, -21.336, 3.0
    L.x_per_z, L.y_per_z = 0.0354, -0.986
    return L


def rect(x0, y0, x1, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


class AdaptiveAreaCoversObjects(unittest.TestCase):
    CASES = (("bar X50-250 Y166-172", rect(50, 166, 250, 172)),
             ("bar X10-120 Y168-172", rect(10, 168, 120, 172)),
             ("8x8 cube at bed centre", rect(146, 146, 154, 154)),
             ("thin wall along Y X148-153 Y60-240",
              rect(148, 60, 153, 240)))

    def test_area_covers_object_and_zero_reference(self):
        for label, poly in self.CASES:
            with self.subTest(label):
                L = make([poly])
                x0, x1, y0, y1 = L._area(F.Gcmd(), {
                    "z": 3.0, "adaptive": 1, "fallback": 1}, False)
                bx0, bx1 = min(p[0] for p in poly), max(p[0] for p in poly)
                by0, by1 = min(p[1] for p in poly), max(p[1] for p in poly)
                self.assertTrue(
                    x0 <= bx0 and x1 >= bx1 and y0 <= by0 and y1 >= by1,
                    "mesh X%.1f-%.1f Y%.1f-%.1f does not cover the object"
                    " X%.0f-%.0f Y%.0f-%.0f" % (x0, x1, y0, y1,
                                                bx0, bx1, by0, by1))
                self.assertTrue(
                    x0 <= ZERO_REF[0] <= x1 and y0 <= ZERO_REF[1] <= y1,
                    "zero_reference_position %r outside the mesh"
                    " X%.1f-%.1f Y%.1f-%.1f" % (ZERO_REF, x0, x1, y0, y1))
                self.assertGreaterEqual(x1 - x0, 20.0)
                self.assertGreaterEqual(y1 - y0, 20.0)


class NonAdaptiveFallback(unittest.TestCase):
    def test_object_out_of_reach_falls_back(self):
        L = make([rect(275, 270, 298, 295)])   # beyond the LiDAR's reach
        with self.assertRaises(F.Err):
            L._area(F.Gcmd(), {"z": 3.0, "adaptive": 0, "fallback": 1},
                    False)


if __name__ == "__main__":
    unittest.main(verbosity=2)
