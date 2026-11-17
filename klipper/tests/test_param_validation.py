"""Regression (K1 bug hunt 2026-11-16 #18): unchecked G-code parameters must
end as command errors, never as plain Python exceptions (in Klipper those
are "Internal error on command" -> printer shutdown, heaters off).

Lidar._grid(a, b, 1) and _scan_dwell STEP=0 (no ZeroDivisionError),
LIDAR_TIMING LEVELS=0,x, MESH_BENCHMARK TRUTH=1 and METHODS=dwell6.
"""
import unittest

import regress_fakes as F
import lidar


class Toolhead:
    def wait_moves(self):
        pass

    def get_last_move_time(self):
        return 0.0


class Mcu:
    def estimated_print_time(self, t):
        return t


class LidarParams(unittest.TestCase):
    def test_grid_with_one_point(self):
        L = F.make_lidar()
        try:
            g = L._grid(10.0, 20.0, 1)
        except ZeroDivisionError:
            self.fail("_grid(10, 20, 1): ZeroDivisionError")
        self.assertEqual(len(g), 1)

    def test_scan_dwell_step_zero(self):
        L = F.make_lidar()
        try:
            L._scan_dwell("s", 100.0, 10.0, 20.0, 0.0, 3.0)
        except ZeroDivisionError:
            self.fail("_scan_dwell STEP=0: ZeroDivisionError")
        except Exception:
            pass   # any other (command) error is fine

    def test_timing_bad_levels_is_a_command_error(self):
        L = F.make_lidar({"toolhead": Toolhead(), "mcu": Mcu(),
                          "motion_report": type("MR", (), {"trapqs": {
                              "toolhead": None}})()})
        try:
            L.cmd_LIDAR_TIMING(F.Gcmd(LEVELS="0,x"))
        except F.CmdError:
            return
        except Exception as e:
            self.fail("%s escapes LIDAR_TIMING LEVELS=0,x (Klipper:"
                      " internal error -> shutdown)" % (type(e).__name__,))


class BenchmarkParams(unittest.TestCase):
    def make(self):
        # a homed 300x300 printer with a load cell (touches possible), so
        # the parameter checks are what is reached
        objs = {"bed_mesh": F.Obj(), "axis_twist_compensation": None,
                "toolhead": F.Obj(get_status=lambda e: {
                    "homed_axes": "xyz",
                    "axis_minimum": [-2.0, -2.0, -10.0, 0.0],
                    "axis_maximum": [306.5, 306.0, 302.0, 0.0]}),
                "load_cell_probe": F.Obj(get_printer_probe=lambda: F.Obj(
                    last_z_result=0.0))}
        L = object.__new__(lidar.Lidar)
        L.printer = F.Obj(lookup_object=lambda n, d=None: objs.get(n, d),
                          command_error=F.Err)
        L.reactor = F.Obj(monotonic=lambda: 0.0)
        L.gcode = F.Obj(run_script_from_command=lambda s: None)
        L.service, L.scan_z, L.mesh_info = "HELPER_IP:7131", 3.0, {}
        L.x_offset, L.y_offset, L.offset_z = -36.0, -21.2, 3.0
        L.x_per_z = L.y_per_z = 0.0
        L._mesh = lambda gcmd, o: (lidar.Lidar.LEVELS[o["level"]], {})[1]
        return L

    def test_bad_params_are_command_errors(self):
        for label, gc in (("TRUTH=1", F.Gcmd(methods="probe", truth=1)),
                          ("METHODS=dwell6",
                           F.Gcmd(methods="dwell6", truth=0))):
            with self.subTest(label):
                L = self.make()
                try:
                    L.cmd_MESH_BENCHMARK(gc)
                except F.Err:
                    pass
                except Exception as e:
                    self.fail("MESH_BENCHMARK %s: %s %s (not a command"
                              " error)" % (label, type(e).__name__, e))


if __name__ == "__main__":
    unittest.main(verbosity=2)
