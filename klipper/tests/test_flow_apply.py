"""lidar.example.cfg's _LIDAR_FLOW_APPLY and _LIDAR_PRINT_SCAN rendered like
Klipper does (jinja2 with { } delimiters): flow per filament from
LIDAR_PA_FLOW_CALIBRATE FLOW_SAVE=1, corrected by the first-layer check's
later learning (fl_zoffset); the PA/flow print's bare step in START_PRINT.
Skipped without jinja2."""
import configparser
import os
import re
import unittest

try:
    import jinja2
except ImportError:  # pragma: no cover
    jinja2 = None

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "..", "lidar.example.cfg")


def macro_source(name):
    """The macro's gcode as Klipper's config reader gets it (comment lines
    between sections dropped, indentation removed)."""
    cp = configparser.RawConfigParser(strict=False)
    cp.read(CFG)
    return cp.get("gcode_macro " + name, "gcode")


def render_flow_apply(variables, params, filename="part.gcode"):
    env = jinja2.Environment("{%", "%}", "{", "}", extensions=["jinja2.ext.do"])
    t = env.from_string(macro_source("_LIDAR_FLOW_APPLY"))
    out = t.render(params=params, printer={
        "print_stats": {"filename": filename},
        "save_variables": {"variables": variables}})
    return [l.strip() for l in out.splitlines() if l.strip()]


def m221(lines):
    return [l for l in lines if l.startswith("M221")]


def key_of(name):
    """The key _LIDAR_FLOW_APPLY looks up (from its SAVE_VARIABLE line)."""
    lines = render_flow_apply({}, {"FILAMENT_NAME": name})
    sv = [l for l in lines if "last_filament_name" in l][0]
    return re.search(r"VALUE=\"'(.*)'\"", sv).group(1)


def render_print_scan(filename, fl_check=0):
    env = jinja2.Environment("{%", "%}", "{", "}", extensions=["jinja2.ext.do"])
    t = env.from_string(macro_source("_LIDAR_PRINT_SCAN"))
    out = t.render(params={}, x=0, y0=0, y1=0, printer={
        "print_stats": {"filename": filename}, "extruder": {"target": 150.0},
        "output_pin First_Layer_Check": {"value": fl_check}})
    return [l.strip() for l in out.splitlines() if l.strip() and not l.strip().startswith("#")]


@unittest.skipIf(jinja2 is None, "jinja2 not installed")
class PrintScanTests(unittest.TestCase):
    def test_calibration_bare_step_in_start_print(self):
        self.assertEqual(render_print_scan("lidar_pa_flow_test.gcode", fl_check=1),
                         ["LIDAR_PA_FLOW_STEP STEP=BARE"])

    def test_other_print_first_layer_check(self):
        out = render_print_scan("part.gcode", fl_check=1)
        self.assertTrue(any(l.startswith("FIRST_LAYER_SCAN PHASE=before") for l in out))
        self.assertEqual(render_print_scan("part.gcode", fl_check=0), [])

    def test_lidar_pa_test_keeps_its_scan(self):
        env = jinja2.Environment("{%", "%}", "{", "}")
        t = env.from_string(macro_source("_LIDAR_PRINT_SCAN"))
        out = t.render(params={}, x=142.0, y0=97.0, y1=223.0, printer={
            "print_stats": {"filename": "lidar_pa_test.gcode"},
            "extruder": {"target": 220.0},
            "output_pin First_Layer_Check": {"value": 0}})
        lines = [l.strip() for l in out.splitlines() if l.strip()]
        self.assertEqual(lines, ["M104 S150",
                                 "LIDAR_SCAN NAME=pa_base X=142.0 Y0=97.0 Y1=223.0"])


@unittest.skipIf(jinja2 is None, "jinja2 not installed")
class FlowApplyTests(unittest.TestCase):
    def test_flow_apply_dict(self):
        v = {"lidar_flow": {"PLA_Matte": {"f": 0.94, "dfdh": 4.3, "z": 0.015,
                                          "t": "2026-11-08"}},
             "fl_zoffset": {"PLA_Matte": 0.005}}
        self.assertEqual(m221(render_flow_apply(v, {"FILAMENT_NAME": "PLA Matte"})),
                         ["M221 S101.7"])

    def test_flow_apply_old_entry(self):
        self.assertEqual(m221(render_flow_apply(
            {"lidar_flow": {"PLA": 0.97}}, {"FILAMENT_NAME": "PLA"})), ["M221 S103.1"])
        self.assertEqual(m221(render_flow_apply({}, {"FILAMENT_NAME": "PLA"})),
                         ["M221 S100"])
        v = {"lidar_flow": {"PLA": {"f": 0.95, "dfdh": 4.3, "z": 0.0, "t": "x"}}}
        self.assertEqual(m221(render_flow_apply(v, {"FILAMENT_NAME": "PLA"})),
                         ["M221 S105.3"])

    def test_flow_apply_test_print(self):
        v = {"lidar_flow": {"PLA": 0.9}}
        self.assertEqual(m221(render_flow_apply(v, {"FILAMENT_NAME": "PLA"},
                                                "lidar_pa_flow_test.gcode")),
                         ["M221 S100"])

    def test_flow_apply_clamped(self):
        v = {"lidar_flow": {"PLA": {"f": 0.70, "dfdh": 4.3, "z": 0.0, "t": "x"}}}
        self.assertEqual(m221(render_flow_apply(v, {"FILAMENT_NAME": "PLA"})),
                         ["M221 S125.0"])


if __name__ == "__main__":
    unittest.main()
