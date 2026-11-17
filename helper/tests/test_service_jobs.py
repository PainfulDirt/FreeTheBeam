"""Regression: lidar_service.py scans and job data, on the harness of
test_service_pf.py. K1 bug hunt 2026-11-16:

#11 one PA+FLOW (LiDAR) print keeps its own PA strips (pf_pa_base /
    pf_pa_scan are not evicted by the band strips).
#4  a new job never fits and saves an earlier job's LiDAR PA strips.
    Request names cannot write outside --data.
Skipped without numpy/PIL.
"""
import glob
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_service_pf as h  # noqa: E402  (the service harness)

_saved = []


def setUpModule():
    # the harness makes data dirs with tempfile.mkdtemp(): keep them in a
    # private directory that is removed afterwards
    d = tempfile.mkdtemp(prefix="ftb-regress-")
    _saved.append((tempfile.tempdir, d))
    tempfile.tempdir = d


def tearDownModule():
    old, d = _saved.pop()
    tempfile.tempdir = old
    shutil.rmtree(d, ignore_errors=True)


def printer_scan(s, name, x, y0, y1, step=0.4):
    """What klipper/lidar.py _scan_dwell sends: begin (grid), frames, end."""
    r = s.handle({"cmd": "begin", "name": name, "x": x, "z": 3.0,
                  "x_dir": -1.0, "x_offset": -36.5, "y_offset": -21.4,
                  "captures": 1, "level": 0, "grid": [y0, y1, step]})
    assert r["ok"], r
    pc = [[i * 19 - 14000, 0, 25600] for i in range(1500)]
    n = int(round(abs(y1 - y0) / step))
    for i in range(n + 1):
        assert s.handle({"cmd": "frame", "name": name, "row": i,
                         "raw": {"result": {"point_cloud": pc}}}) is None
    assert s.handle({"cmd": "end", "name": name})["ok"]


@unittest.skipIf(h.np is None, "needs numpy and PIL (helper only)")
class PfScans(unittest.TestCase):
    def setUp(self):
        self._pa_fit = h.svc_mod.lidar.pa_fit

    def tearDown(self):
        h.svc_mod.lidar.pa_fit = self._pa_fit

    def test_pa_flow_lidar_keeps_pa_strips(self):
        # LIDAR_PA_FLOW_CALIBRATE TESTS=PA,FLOW PA_METHOD=LIDAR
        # FLOW_METHOD=BOTH: the printer's BARE step scans pf_pa_base then the
        # 6 bare band strips, PHOTOS scans pf_pa_scan then the 6 "_after"
        s, d = h.make_service()
        test = s.handle({"cmd": "pf_plan", "params": dict(
            h.P, pa_method="lidar", flow_method="both")})["test"]
        ps = test["pa_strip"]
        printer_scan(s, "pf_pa_base", ps["x"], ps["y0"], ps["y1"])
        for st in test["strips"]:
            printer_scan(s, st["name"], st["x"], st["y0"], st["y1"])
        printer_scan(s, "pf_pa_scan", ps["x"], ps["y0"], ps["y1"])
        for st in test["strips"]:
            printer_scan(s, st["name"] + "_after", st["x"], st["y0"],
                         st["y1"])
        s.handle({"cmd": "pf_analyze", "job": "jx", "test": test})
        self.assertIn("pf_pa_base", list(s.scans), "PA base strip evicted")
        self.assertIn("pf_pa_scan", list(s.scans), "PA printed strip evicted")

    def test_stale_pa_strips_not_used_for_new_job(self):
        s, d = h.make_service(dim=0.125)   # camera contrast too low (job 2)
        # job 1: LIDAR_PA_CALIBRATION METHOD=LIDAR -> pf_pa_* on the helper
        t1 = s.handle({"cmd": "pf_plan", "params": dict(
            h.P, tests=["pa"], pa_method="lidar")})["test"]
        ps = t1["pa_strip"]
        printer_scan(s, "pf_pa_base", ps["x"], ps["y0"], ps["y1"])
        printer_scan(s, "pf_pa_scan", ps["x"], ps["y0"], ps["y1"])
        # stand-in for job 1 having printed real lines: the fit finds a PA
        h.svc_mod.lidar.pa_fit = lambda ys, A, test: {
            "ok": True, "pa": 0.0555, "report": ["job-1 data"]}
        # job 2: LIDAR_PA_CALIBRATION (IR) on a dark plate: the printer scans
        # no LiDAR strips, the camera fails
        t2 = s.handle({"cmd": "pf_plan", "params": dict(
            h.P, tests=["pa"], pa_method="ir", filament_name="Other PETG")
        })["test"]
        h.take_photos(s, t2, "job2")
        r = s.handle({"cmd": "pf_analyze", "job": "job2", "test": t2})
        # job 2 made no LiDAR strips -> no LiDAR PA, nothing saved
        self.assertNotEqual(r["pa_method"], "lidar",
                            "job 2 PA taken from job 1's LiDAR strips")
        self.assertFalse(r["pa"].get("ok"))


class PathNames(unittest.TestCase):
    def test_scan_name_stays_in_data_dir(self):
        # the data dir sits in a private parent, so an escape (the
        # regression) lands there and is removed with it
        parent = tempfile.mkdtemp()
        d = os.path.join(parent, "data")
        os.mkdir(d)
        try:
            s = h.svc_mod.Service(h.FakeMW(1.0), d)
            s.handle({"cmd": "begin", "name": "../escaped", "x": 1, "z": 3,
                      "x_dir": -1, "x_offset": 0, "y_offset": 0})
            s.handle({"cmd": "end", "name": "../escaped"})
            escaped = glob.glob(os.path.join(parent, "escaped*"))
            self.assertEqual(escaped, [], "written outside --data")
        finally:
            shutil.rmtree(parent, ignore_errors=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
