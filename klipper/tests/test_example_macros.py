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
        "print_stats": {"filename": "part.gcode"},
        "extruder": {"target": 220.0},
        "lidar": {"first_layer_armed": False},
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


if __name__ == "__main__":
    unittest.main()
