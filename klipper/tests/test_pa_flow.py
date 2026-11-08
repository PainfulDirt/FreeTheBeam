"""klipper/lidar.py: LIDAR_PA_FLOW_CALIBRATE, LIDAR_PA_FLOW_STEP and the
camera LIDAR_OFFSET_CALIBRATE with stand-in Klipper objects and a stub
helper service (planning by the real helper/lidar_pa_flow.py)."""
import json
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "..", "helper"))
sys.path.insert(0, HERE)
import lidar  # noqa: E402
import lidar_pa_flow  # noqa: E402
from fake_klipper import Err, Gcmd, Obj  # noqa: E402
import test_flow_apply  # noqa: E402

FIL = math.pi * 1.75 ** 2 / 4


def make(service="HELPER_IP:7131", state="standby", filename="",
         variables=None, svc_fail=None, analyze=None):
    L = lidar.Lidar.__new__(lidar.Lidar)
    d = tempfile.mkdtemp()
    ev = []
    stats = {"state": state, "filename": filename}
    L.ev, L.stats = ev, stats
    variables = dict(variables or {})
    L.printer = Obj(lookup_object=lambda name, default=None: {
        "print_stats": Obj(get_status=lambda t: dict(stats)),
        "toolhead": Obj(
            max_accel=20000.0,
            get_extruder=lambda: Obj(filament_area=FIL, get_status=lambda t: {
                "pressure_advance": 0.0475}),
            get_status=lambda t: {"homed_axes": "xyz",
                                  "axis_minimum": [-2.0, -2.0, -10.0],
                                  "axis_maximum": [306.5, 306.0, 302.0]}),
        "bed_mesh": Obj(get_mesh=lambda: Obj(calc_z=lambda x, y: 0.01)),
        "save_variables": Obj(allVariables=variables),
    }.get(name, default), command_error=Err)
    L.reactor = Obj(monotonic=lambda: 0.0, pause=lambda t: t)
    L.gcode = Obj(run_script_from_command=lambda s: ev.append(("gcode", s)),
                  respond_info=lambda m: ev.append(("info", m)))
    L.data_dir, L.gcode_dir, L.service = d, d, service
    L.pf = None
    L._xo = lambda z: -36.0
    L._yo = lambda z: -21.2
    L._lidar_tables = lambda: ({"xs": [0, 300], "comp": [0, 0]},
                               {"ys": [0, 300], "comp": [0, 0]}, "")
    L._check_reach = lambda gcmd, segs, z=None, need_homed=True: ev.append(
        ("reach", len(segs), z))
    L._scan_dwell = lambda name, x, y0, y1, step, z, **kw: (
        ev.append(("scan", name)) or (100, 10.0, 0))

    def svc(req, timeout=30.0):
        c = req["cmd"]
        ev.append(("svc", c))
        if svc_fail == c:
            raise Err("LiDAR service: boom")
        if c == "pf_plan":
            return dict(lidar_pa_flow.plan_pa_flow(req["params"]), ok=True)
        if c == "auto_exposure":
            return {"ok": True, "exposure": 4000, "x_range": [-14.0, 14.0],
                    "points": 1499, "table": []}
        if c == "photo":
            return {"ok": True, "file": "f", "bytes": 1,
                    "median": getattr(L, "photo_median", 3.0)}
        if c == "pf_analyze":
            L.analyze_req = req
            return dict(analyze or {
                "pa": {"ok": True, "pa": 0.042, "report": ["  pa line"]},
                "pa_method": "camera",
                "flow": {"ok": True, "f": 0.946, "dfdh": 4.26, "sd": 0.019,
                         "report": ["  flow line"]},
                "contrast": 50.0}, ok=True, html="/var/lib/lidar-helper/pf.html")
        return {"ok": True}
    L._svc = svc
    return L


def calibrate(L, **kw):
    p = dict(EXTRUDER_TEMP=210, BED_TEMP=60, FILAMENT_NAME="PLA Matte",
             FILAMENT_TYPE="PLA")
    p.update(kw)
    L.cmd_LIDAR_PA_FLOW_CALIBRATE(Gcmd(**p))


def saves(L):
    return [e[1] for e in L.ev if e[0] == "gcode" and "SAVE_VARIABLE" in e[1]]


class CalibrateTests(unittest.TestCase):
    def test_calibrate_writes_file_and_starts(self):
        L = make()
        calibrate(L)
        g = open(os.path.join(L.gcode_dir, "lidar_pa_flow_test.gcode")).read()
        self.assertIn("LIDAR_PA_FLOW_STEP STEP=PHOTOS", g)   # BARE runs in START_PRINT
        t = json.load(open(os.path.join(L.data_dir, "pa_flow_test.json")))
        self.assertEqual(t["job"], L.pf["job"])
        self.assertEqual([e[1] for e in L.ev if e[0] == "gcode"],
                         ["SDCARD_PRINT_FILE FILENAME=lidar_pa_flow_test.gcode"])
        self.assertEqual([e for e in L.ev if e[0] == "reach"][1][2], 3.0)

    def test_calibrate_needs_service(self):
        L = make(service=None)
        with self.assertRaises(Err):
            calibrate(L)
        self.assertEqual(L.ev, [])

    def test_calibrate_reach_refused(self):
        L = make()

        def refuse(gcmd, segs, z=None, need_homed=True):
            raise Err("LiDAR: the nozzle would go outside its travel")
        L._check_reach = refuse
        with self.assertRaises(Err):
            calibrate(L, X=290)
        self.assertFalse(os.path.exists(os.path.join(L.gcode_dir,
                                                     "lidar_pa_flow_test.gcode")))


class StepTests(unittest.TestCase):
    def started(self, flow_save=0, **kw):
        L = make(**kw)
        calibrate(L, FLOW_SAVE=flow_save)
        L.stats.update(state="printing", filename="lidar_pa_flow_test.gcode")
        del L.ev[:]
        return L

    def test_step_without_job_errors(self):
        L = self.started()
        L.stats["filename"] = "benchy.gcode"
        with self.assertRaises(Err):
            L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP="BARE"))
        L.stats["filename"] = "lidar_pa_flow_test.gcode"
        L.pf = dict(L.pf, job="19990101-000000")       # stale job in memory
        with self.assertRaises(Err):
            L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP="BARE"))
        self.assertFalse([e for e in L.ev if e[0] == "svc"])

    def test_bare_step_calls(self):
        L = self.started()
        L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP="BARE"))
        order = [e[1] if e[0] in ("svc", "scan") else e[1].split("\n")[0]
                 for e in L.ev if e[0] in ("svc", "scan", "gcode")]
        self.assertEqual(order[0], "M104 S150")
        i_ae = order.index("auto_exposure")
        i_ph = order.index("photo")
        scans = [e[1] for e in L.ev if e[0] == "scan"]
        self.assertEqual(scans, ["pf_g0", "pf_g1", "pf_g2", "pf_g3"])
        self.assertTrue(i_ae < i_ph < order.index("pf_g0"))
        self.assertEqual(L.pf["exposure"], 4000)
        self.assertEqual(sorted(L.pf["mesh"]), ["pf_g0", "pf_g1", "pf_g2", "pf_g3"])
        self.assertEqual(L.pf["mesh"]["pf_g0"][0], [33.0, 0.01])

    def test_bare_step_unreadable_photo(self):
        L = self.started()
        L.photo_median = None          # service couldn't read the brightness
        L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP="BARE"))
        self.assertEqual(L.pf["photo_exposure"], 2000)
        self.assertFalse(L.pf["lidar_pa"])

    def test_photos_step(self):
        L = self.started()
        L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP="BARE"))
        del L.ev[:]
        L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP="PHOTOS"))
        self.assertEqual(len([e for e in L.ev if e == ("svc", "photo")]), 9)

    def test_analyze_default_saves_pa_not_flow(self):
        # flow is experimental: measured and reported, saved only on request
        L = self.started(variables={"fl_zoffset": {"PLA_Matte": 0.005}})
        L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP="BARE"))
        L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP="ANALYZE"))
        s = saves(L)
        self.assertEqual(s, ["SET_PRESSURE_ADVANCE ADVANCE=0.0420\n"
                             "SAVE_VARIABLE VARIABLE=lidar_pa VALUE=0.0420"])
        info = [e[1] for e in L.ev if e[0] == "info"]
        self.assertTrue(any("NOT saved" in m and "FLOW_SAVE=1" in m
                            for m in info), info)

    def test_analyze_saves(self):
        L = self.started(flow_save=1,
                         variables={"fl_zoffset": {"PLA_Matte": 0.005}})
        L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP="BARE"))
        L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP="ANALYZE"))
        s = saves(L)
        self.assertIn("SAVE_VARIABLE VARIABLE=lidar_pa VALUE=0.0420", "\n".join(s))
        flow = [x for x in s if "lidar_flow" in x][0]
        val = eval(flow.split("VALUE=", 1)[1].strip('"'))
        self.assertEqual(val["PLA_Matte"]["f"], 0.946)
        self.assertEqual(val["PLA_Matte"]["dfdh"], 4.26)
        self.assertEqual(val["PLA_Matte"]["z"], 0.005)
        self.assertIn("t", val["PLA_Matte"])
        self.assertIsNone(L.pf)
        self.assertIn("tables", L.analyze_req)

    def test_analyze_failure_saves_nothing(self):
        L = self.started(svc_fail="pf_analyze")
        L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP="ANALYZE"))
        self.assertEqual(saves(L), [])
        self.assertTrue(any("analysis" in e[1] for e in L.ev if e[0] == "info"))
        self.assertIsNone(L.pf)
        L = self.started(analyze={"pa": {"ok": False, "reason": "noisy", "report": []},
                                  "pa_method": "none",
                                  "flow": {"ok": False, "reason": "contrast", "report": []},
                                  "contrast": 4.0})
        L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP="ANALYZE"))
        self.assertEqual(saves(L), [])

    @unittest.skipIf(test_flow_apply.jinja2 is None, "jinja2 not installed")
    def test_flow_key_roundtrip(self):
        name = 'PLA "Matte" @K1 Max'
        L = self.started(flow_save=1)
        L.pf["filament_name"] = name
        L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP="ANALYZE"))
        flow = [x for x in saves(L) if "lidar_flow" in x][0]
        val = eval(flow.split("VALUE=", 1)[1].strip('"'))
        # START_PRINT gets the name with " turned into ' (lidar_pa_flow._q)
        self.assertEqual(list(val), [test_flow_apply.key_of(
            name.replace('"', "'"))])


class GenericTests(unittest.TestCase):
    """FreeTheBeam: the camera is optional, the print placed on any bed."""

    def test_pa_calibrate_uses_camera_with_helper(self):
        L = make()
        L.cmd_LIDAR_PA_CALIBRATE(Gcmd(EXTRUDER_TEMP=210, BED_TEMP=60))
        self.assertTrue(os.path.exists(os.path.join(
            L.gcode_dir, "lidar_pa_flow_test.gcode")))

    def test_pa_calibrate_lidar_print_without_camera(self):
        for kw in ({"service": None}, {"camera": False}, {"x_dir": 1.0}):
            L = make(service=kw.get("service", "HELPER_IP:7131"))
            for k in ("camera", "x_dir"):
                if k in kw:
                    setattr(L, k, kw[k])
            called = []
            L._pa_lidar_print = lambda gcmd: called.append(gcmd)
            L.cmd_LIDAR_PA_CALIBRATE(Gcmd(EXTRUDER_TEMP=210, BED_TEMP=60))
            self.assertEqual(len(called), 1, kw)
            if "service" not in kw:
                with self.assertRaises(Err):
                    calibrate(L)

    def test_offset_lidar_only_without_camera(self):
        L = make(service=None)
        called = []
        L._offset_lidar = lambda gcmd: called.append("lidar")
        L._offset_camera = lambda gcmd: called.append("camera")
        L.cmd_LIDAR_OFFSET_CALIBRATE(Gcmd(EXTRUDER_TEMP=210, BED_TEMP=60))
        L.service = "HELPER_IP:7131"
        L.cmd_LIDAR_OFFSET_CALIBRATE(Gcmd(EXTRUDER_TEMP=210, BED_TEMP=60))
        self.assertEqual(called, ["lidar", "camera"])

    def test_placement_centred_when_the_k1_spot_does_not_fit(self):
        L = make()
        st = {"homed_axes": "", "axis_minimum": [0.0, 0.0, 0.0],
              "axis_maximum": [235.0, 235.0, 250.0]}
        th = L.printer.lookup_object("toolhead")
        th.get_status = lambda t: st
        look = L.printer.lookup_object
        L.printer.lookup_object = lambda name, default=None: (
            th if name == "toolhead" else look(name, default))
        seen = []

        def reach(gcmd, segs, z=None, need_homed=True):
            self.assertFalse(need_homed)     # START_PRINT homes later
            xs = [v for s in segs for v in (s[0], s[2])]
            ys = [v for s in segs for v in (s[1], s[3])]
            if z is not None:                # laser: nozzle = laser - offset
                xs = [x + 36.0 for x in xs]
                ys = [y + 21.2 for y in ys]
            seen.append((min(xs), max(xs), min(ys), max(ys)))
            if min(ys) < 0 or max(ys) > 235 or min(xs) < 0 or max(xs) > 235:
                raise Err("LiDAR: the nozzle would go outside its travel")
        L._check_reach = reach
        calibrate(L)
        t = json.load(open(os.path.join(L.data_dir, "pa_flow_test.json")))
        # the K1 Max spot (Y35 -> Y222 plus the laser strips) runs past the
        # 235 mm bed's back: centred in the laser area instead
        self.assertNotEqual(t["flow"]["y0"], 35.0)
        self.assertTrue(all(b[3] <= 235 for b in seen[-2:]), seen)


class OffsetTests(unittest.TestCase):
    def run_offset(self, cam_x):
        L = make()
        L.x_offset, L.y_offset, L.x_per_z, L.y_per_z = -36.0, -21.3, 0.03, -0.98
        L.offset_z, L.offset_source = 3.0, "test"
        L._skewed = lambda p: list(p)
        L._scan_any = lambda name, x, y0, y1, step, z: L.ev.append(("scan", name))
        L._cross_fit = lambda name, test: {"ok": True}
        svc0 = L._svc

        def svc(req, timeout=30.0):
            if req["cmd"] == "offset_plan":
                L.ev.append(("svc", "offset_plan"))
                return dict(lidar_pa_flow.plan_offset_cross(req["params"]), ok=True)
            if req["cmd"] == "offset_cam":
                L.ev.append(("svc", "offset_cam"))
                return {"ok": True, "x_offset": cam_x, "y_offset": -21.32,
                        "x_per_z": 0.031, "y_per_z": -0.985, "report": ["cam"]}
            return svc0(req, timeout)
        L._svc = svc
        old = lidar.offset_solve
        lidar.offset_solve = lambda fits, test, ref_z: {
            "ok": True, "x_offset": -36.0, "y_offset": -21.3, "x_per_z": 0.03,
            "y_per_z": -0.98, "report": ["lidar"]}
        try:
            L.cmd_LIDAR_OFFSET_CALIBRATE(Gcmd(EXTRUDER_TEMP=210, BED_TEMP=60))
        finally:
            lidar.offset_solve = old
        return L

    def test_offset_disagreement_not_saved(self):
        with self.assertRaises(Err):
            L = self.run_offset(-36.30)
        # nothing saved: run again to inspect the events
        L = make()

    def test_offset_agreement_saves_camera(self):
        L = self.run_offset(-36.05)
        sv = [e[1] for e in L.ev if e[0] == "gcode" and "SAVE_VARIABLE" in e[1]]
        self.assertEqual(len(sv), 4)
        self.assertIn("SAVE_VARIABLE VARIABLE=lidar_x_offset VALUE=-36.0500", sv)
        cross = [e[1] for e in L.ev if e[0] == "gcode" and "G1 E0.8" in e[1]]
        self.assertTrue(cross)
        photos = [e for e in L.ev if e == ("svc", "photo")]
        self.assertEqual(len(photos), 4)     # light + laser at Z1 and Z2


class ExposureTests(unittest.TestCase):
    def make_x(self, **kw):
        L = make(state="printing", filename="part.gcode", **kw)
        L.cmd_key = "LIDAR_SCAN:1.0"
        L._driver_msg = lambda msg, timeout=10.0: L.ev.append(("drv", msg)) or {"code": 0}
        return L

    def test_exposure_once_per_job(self):
        L = self.make_x()
        a = L._job_exposure()
        b = L._job_exposure()
        self.assertEqual(a["exposure"], 4000)
        self.assertEqual(b["exposure"], 4000)
        self.assertEqual([e for e in L.ev if e == ("svc", "auto_exposure")],
                         [("svc", "auto_exposure")])

    def test_exposure_without_service(self):
        L = self.make_x(service=None)
        self.assertIsNone(L._job_exposure())
        self.assertEqual(L.ev, [])

    def test_fl_after_reuses_exposure(self):
        L = self.make_x()
        L._job_exposure()
        del L.ev[:]
        L._job_exposure()                     # the first layer, same print
        self.assertEqual(L.ev, [("drv", {"control": "set_laser_exposure",
                                         "exposure": 4000})])

    def test_new_print_measures_again(self):
        L = self.make_x()
        L._job_exposure()
        L.stats["filename"] = "other.gcode"
        L._job_exposure()
        self.assertEqual(len([e for e in L.ev if e == ("svc", "auto_exposure")]), 2)

    def test_fl_plan_width(self):
        L = self.make_x()
        self.assertEqual(L._fl_width(), (28.0, 26.0))     # nothing measured yet
        L._job_exposure()
        L.exposure_job["x_range"] = [-9.0, 9.0]
        self.assertEqual(L._fl_width(), (16.0, 16.0))
        L.exposure_job["x_range"] = [-14.0, 14.0]
        self.assertEqual(L._fl_width(), (26.0, 24.0))


if __name__ == "__main__":
    unittest.main()
