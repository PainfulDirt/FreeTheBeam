"""First-layer check bias per plate and filament (fl_bias in variables.cfg).

The LiDAR read white PLA on black smooth PEI ~37 um LOW against a contact
probe (9 pads): the check said 0.166 for 0.200 while the layer was fine.
fl_bias[plate][filament] = LiDAR minus truth (mm); the check takes it off
before it reports or learns. FIRST_LAYER_BIAS sets/clears/shows it. Also:
LIDAR_STATUS SELFTEST far above the plate says so (laser "weak" at Z20, ok
at Z3)."""
import ast
import unittest

from fake_klipper import Err, Gcmd, make_lidar
import lidar


def make(saved):
    L, log, objs = make_lidar()
    L._saved = lambda: saved

    def run(s):
        log.append(s)
        if s.startswith("SAVE_VARIABLE VARIABLE=fl_bias VALUE="):
            saved["fl_bias"] = ast.literal_eval(
                s.split("VALUE=", 1)[1].strip('"'))
    L.gcode.run_script_from_command = run
    return L


def result():
    return {"ok": True, "median": 0.166, "error": -0.034, "p05": 0.123,
            "p95": 0.214, "objects": [{"name": "a", "median": 0.166,
                                       "cells": 10}]}


class BiasTests(unittest.TestCase):
    def test_lookup(self):
        L = make({"fl_bias": {"black_pei": {"PLA": -0.037}}})
        self.assertEqual(L._fl_bias_for("black_pei", "PLA"), -0.037)
        self.assertIsNone(L._fl_bias_for("textured_pei", "PLA"))
        self.assertIsNone(L._fl_bias_for("black_pei", "PETG"))
        self.assertIsNone(L._fl_bias_for(None, "PLA"))

    def test_applied_to_everything_reported(self):
        L = make({"fl_bias": {"black_pei": {"PLA": -0.037}}})
        res = result()
        b = L._fl_apply_bias(res, "black_pei", "PLA")
        self.assertEqual(b, -0.037)
        self.assertAlmostEqual(res["median"], 0.203)
        self.assertAlmostEqual(res["error"], 0.003)
        self.assertAlmostEqual(res["p05"], 0.160)
        self.assertAlmostEqual(res["p95"], 0.251)
        self.assertAlmostEqual(res["objects"][0]["median"], 0.203)
        self.assertEqual((res["bias"], res["median_raw"]), (-0.037, 0.166))

    def test_no_bias_no_change(self):
        L = make({})
        res = result()
        self.assertIsNone(L._fl_apply_bias(res, "black_pei", "PLA"))
        self.assertEqual(res, result())
        res = dict(result(), ok=False)
        L = make({"fl_bias": {"black_pei": {"PLA": -0.037}}})
        self.assertIsNone(L._fl_apply_bias(res, "black_pei", "PLA"))

    def test_filament_key_matches_check(self):
        L = make({})
        self.assertEqual(L._filament_key('PETG "Rapid"'), "PETG_Rapid")
        self.assertEqual(L._fl_key({"filament_name": "PLA Matte"}),
                         "PLA_Matte")

    def test_command_sets_for_current_plate_and_filament(self):
        saved = {"lidar_plate": "black_pei",
                 "last_filament_name": "PLA_Generic"}
        L = make(saved)
        L.cmd_FIRST_LAYER_BIAS(Gcmd(VALUE=-0.037))
        self.assertEqual(saved["fl_bias"],
                         {"black_pei": {"PLA_Generic": -0.037}})
        L.cmd_FIRST_LAYER_BIAS(Gcmd(VALUE=-0.02, PLATE="textured_pei",
                                    FILAMENT="PETG Rapid"))
        self.assertEqual(saved["fl_bias"]["textured_pei"],
                         {"PETG_Rapid": -0.02})
        L.cmd_FIRST_LAYER_BIAS(Gcmd(CLEAR=1))
        self.assertEqual(saved["fl_bias"], {"textured_pei":
                                            {"PETG_Rapid": -0.02}})

    def test_command_report_and_refusals(self):
        saved = {"fl_bias": {"black_pei": {"PLA": -0.037}}}
        L = make(saved)
        g = Gcmd()
        L.cmd_FIRST_LAYER_BIAS(g)
        self.assertTrue(any("black_pei" in m and "-0.037" in m
                            for m in g.info))
        with self.assertRaises(Err):     # no plate set
            L.cmd_FIRST_LAYER_BIAS(Gcmd(VALUE=-0.03, FILAMENT="PLA"))
        saved["lidar_plate"] = "black_pei"
        with self.assertRaises(Err):     # implausible
            L.cmd_FIRST_LAYER_BIAS(Gcmd(VALUE=0.3, FILAMENT="PLA"))
        with self.assertRaises(Err):     # no filament known
            L.cmd_FIRST_LAYER_BIAS(Gcmd(VALUE=-0.03))

    def test_selftest_hint(self):
        self.assertIn("Z3", lidar.Lidar._selftest_hint(20.0, 3.0))
        self.assertEqual(lidar.Lidar._selftest_hint(3.0, 3.0), "")

    def test_check_takes_the_bias_off(self):
        saved = {"lidar_plate": "black_pei",
                 "fl_bias": {"black_pei": {"PLA_Matte": -0.037}}}
        L = make(saved)
        L.x_dir = 1
        L.fl = {"id": "f1", "file": "a.gcode", "filament": "PLA",
                "filament_name": "PLA Matte", "strips": [], "y0": 0,
                "y1": 10, "step": 2, "z_before": 3, "z_after": 3.2,
                "height": 0.2, "objects": []}
        res = dict(result(), cells=500, area_mm2=900, height=0.2,
                   tilt_um_per_100mm=[1, 2],
                   drift={"plane": [0.001, 0, 0], "bare_rms": 0.005})
        L._svc = lambda msg, timeout=None: {"result": res}
        g = Gcmd()
        out = L._fl_analyze(g)
        self.assertAlmostEqual(out["median"], 0.203)
        self.assertIn("0.203mm measured", g.info[0])
        self.assertIn("LiDAR bias -0.037mm for PLA_Matte on black_pei"
                      " taken off (read 0.166mm", g.info[1])
        self.assertTrue(g.info[2].startswith("  a: 0.203mm"))
        saved["fl_bias"] = {}
        res = dict(res, median=0.166, objects=[{"name": "a",
                                                "median": 0.166,
                                                "cells": 10}])
        g = Gcmd()
        L._fl_analyze(g)
        self.assertFalse(any("LiDAR bias" in m for m in g.info))

    def test_registered(self):
        self.assertTrue(hasattr(lidar.Lidar, "desc_FIRST_LAYER_BIAS"))
        with open(lidar.__file__) as f:
            self.assertIn('"FIRST_LAYER_BIAS"', f.read())


if __name__ == "__main__":
    unittest.main(verbosity=2)
