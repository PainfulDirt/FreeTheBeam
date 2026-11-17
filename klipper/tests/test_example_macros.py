"""The switch macros in klipper/lidar.example.cfg must work without the
[output_pin] switches (mainline Klipper has no virtual pins): then they read
the saved variables instead. Rendered with jinja2 the way Klipper does."""
import configparser
import os
import unittest

try:
    import jinja2
except ImportError:  # pragma: no cover
    jinja2 = None

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "..", "lidar.example.cfg")


class Status(dict):
    """printer[...] like Klipper's GetStatusWrapperJinja: a missing object
    raises KeyError (jinja2 makes it undefined), `in` works."""


def load_macros():
    cp = configparser.RawConfigParser(strict=False)
    cp.read(CFG)
    return dict((s, cp.get(s, "gcode")) for s in cp.sections()
                if cp.has_option(s, "gcode"))


def render(section, saved, pins=None, params=None, **variables):
    env = jinja2.Environment("{%", "%}", "{", "}")
    printer = Status({
        "save_variables": {"variables": saved},
        "gcode_macro LIDAR_MESH": {"print_level": 3},
        "print_stats": {"filename": variables.pop("filename", "part.gcode"),
                        "state": variables.pop("state", "printing")},
        "configfile": {"settings": {"extruder": {"pressure_advance": 0.04}}},
        "toolhead": {"homed_axes": "xyz"},
        "heater_bed": {"target": 60.0},
        "extruder": {"target": 220.0},
        "lidar": {"first_layer_armed": variables.pop("armed", False)},
    })
    for name, value in (pins or {}).items():
        printer["output_pin " + name] = {"value": value}
    ctx = dict(variables, printer=printer, params=params or {})
    out = env.from_string(load_macros()[section]).render(**ctx)
    return [ln.strip() for ln in out.splitlines() if ln.strip()]


@unittest.skipIf(jinja2 is None, "jinja2 not installed")
class SwitchesWithoutPins(unittest.TestCase):
    def test_mesh_switch_from_saved_variable(self):
        out = render("gcode_macro _LIDAR_START_PRINT_MESH",
                     {"sw_lidar_mesh": 1})
        self.assertEqual(out, ["LIDAR_BED_MESH LEVEL=3 ADAPTIVE=1 FALLBACK=1"])

    def test_mesh_switch_off_without_pin_or_variable(self):
        out = render("gcode_macro _LIDAR_START_PRINT_MESH", {})
        self.assertEqual(out, ["BED_MESH_CALIBRATE ADAPTIVE=1"])

    def test_pin_wins_over_saved_variable(self):
        out = render("gcode_macro _LIDAR_START_PRINT_MESH",
                     {"sw_lidar_mesh": 0}, pins={"LiDAR_Mesh": 1.0})
        self.assertEqual(out, ["LIDAR_BED_MESH LEVEL=3 ADAPTIVE=1 FALLBACK=1"])

    def test_first_layer_check_from_saved_variable(self):
        out = render("gcode_macro _LIDAR_PRINT_SCAN",
                     {"sw_first_layer_check": 1}, x=0, y0=0, y1=0)
        self.assertTrue(out[-1].startswith("FIRST_LAYER_SCAN PHASE=before"),
                        out)
        self.assertEqual(render("gcode_macro _LIDAR_PRINT_SCAN", {},
                                x=0, y0=0, y1=0), [])

    def test_first_layer_learn_from_saved_variable(self):
        saved = {"sw_first_layer_learn": 1, "fl_zoffset": {"PLA": 0.01}}
        out = render("gcode_macro _FIRST_LAYER_OFFSET", saved,
                     params={"FILAMENT_TYPE": "PLA"})
        self.assertEqual(out[0], "SET_GCODE_OFFSET Z=0.01")
        self.assertEqual(render("gcode_macro _FIRST_LAYER_OFFSET", {},
                                params={"FILAMENT_TYPE": "PLA"}), [])



@unittest.skipIf(jinja2 is None, "jinja2 not installed")
class CalibrationMacros(unittest.TestCase):
    def test_print_scan_only_in_a_file_print(self):
        # START_PRINT run from a command sees the last job's filename
        for fn in ("lidar_pa_flow_test.gcode", "part.gcode"):
            self.assertEqual(render("gcode_macro _LIDAR_PRINT_SCAN",
                                    {"sw_first_layer_check": 1}, x=0, y0=0,
                                    y1=0, filename=fn, state="standby"), [])
        out = render("gcode_macro _LIDAR_PRINT_SCAN", {}, x=0, y0=0, y1=0,
                     filename="lidar_pa_flow_test.gcode")
        self.assertEqual(out, ["LIDAR_PA_FLOW_STEP STEP=BARE"])

    def test_pa_calibration_method(self):
        out = render("gcode_macro LIDAR_PA_CALIBRATION", {},
                     params={"EXTRUDER_TEMP": "215", "METHOD": "both",
                             "FILAMENT_NAME": "PLA Matte", "PA_STEP": "0.005"})
        self.assertEqual(len(out), 1)
        self.assertTrue(out[0].startswith(
            'LIDAR_PA_CALIBRATE EXTRUDER_TEMP=215.0 BED_TEMP=60.0'
            ' FILAMENT_NAME="PLA Matte" FILAMENT_TYPE=""'), out)
        self.assertIn("PA_STEP=0.005", out[0])
        self.assertTrue(out[0].endswith("METHOD=BOTH"), out)
        # no METHOD: the plugin picks (camera if on, else LiDAR strips)
        out = render("gcode_macro LIDAR_PA_CALIBRATION", {})
        self.assertNotIn("METHOD", out[0])
        out = render("gcode_macro LIDAR_PA_CALIBRATION", {},
                     params={"METHOD": "foo"})
        self.assertTrue(out[0].startswith("RESPOND TYPE=error"), out)

    def test_flow_calibration_defaults_to_benchmark(self):
        out = render("gcode_macro LIDAR_FLOW_CALIBRATION", {})
        self.assertTrue(out[0].startswith(
            "LIDAR_PA_FLOW_CALIBRATE TESTS=FLOW FLOW_METHOD=BOTH"), out)
        out = render("gcode_macro LIDAR_FLOW_CALIBRATION", {},
                     params={"METHOD": "lidar"})
        self.assertIn("FLOW_METHOD=LIDAR", out[0])

    def test_mesh_calibration_axis_and_reset(self):
        out = render("gcode_macro LIDAR_MESH_CALIBRATION", {},
                     params={"AXIS": "y", "POINTS": "5", "RESET": "1"})
        self.assertIn("LIDAR_PLATE RESET_RUNS=1", out)
        self.assertIn("LIDAR_TWIST_CALIBRATE AXIS=Y POINTS=5", out)
        self.assertNotIn("LIDAR_TWIST_CALIBRATE AXIS=X", out)
        out = render("gcode_macro LIDAR_MESH_CALIBRATION", {})
        self.assertIn("LIDAR_TWIST_CALIBRATE AXIS=X", out)
        self.assertIn("LIDAR_TWIST_CALIBRATE AXIS=Y", out)
        out = render("gcode_macro LIDAR_MESH_CALIBRATION", {},
                     params={"AXIS": "X", "GRID": "7,5"})
        self.assertTrue(out[0].startswith("RESPOND TYPE=error"), out)

    def test_cancelled_calibration_restores_pa(self):
        out = render("gcode_macro _LIDAR_PRINT_CANCEL", {"lidar_pa": 0.036},
                     filename="lidar_pa_flow_test.gcode")
        self.assertIn("SET_PRESSURE_ADVANCE ADVANCE=0.036", out)
        self.assertIn("M221 S100", out)
        out = render("gcode_macro _LIDAR_PRINT_CANCEL", {},
                     filename="lidar_pa_flow_test.gcode")
        self.assertIn("SET_PRESSURE_ADVANCE ADVANCE=0.04", out)
        self.assertEqual(render("gcode_macro _LIDAR_PRINT_CANCEL", {}),
                         ["FIRST_LAYER_SCAN PHASE=cancel"])

    def test_first_layer_after_primes(self):
        self.assertEqual(render("gcode_macro _FIRST_LAYER_AFTER", {},
                                prime_tip=5, prime_purge=20), [])
        out = render("gcode_macro _FIRST_LAYER_AFTER", {}, prime_tip=5,
                     prime_purge=20, armed=True)
        self.assertEqual(out, ["FIRST_LAYER_SCAN PHASE=after TIP=5 PRIME=20"])


if __name__ == "__main__":
    unittest.main()
