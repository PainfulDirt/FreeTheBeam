"""Regression (K1 bug hunt 2026-11-16 #8): a failed nozzle touch (here the
load cell: LOAD_CELL_PROBE) inside LIDAR_GRID_CALIBRATE /
LIDAR_TWIST_CALIBRATE must put the bed heater target back (touch_grid is
the control that always did).
"""
import unittest

import regress_fakes as F
import lidar


def make_lidar(fail_at):
    st = {"target": 70.0, "taps": 0}
    lc = F.Obj(last_z_result=0.0)

    def run(script):
        # like Klipper: lines run in order, an error aborts the rest
        for line in script.split("\n"):
            w = line.split()
            if not w:
                continue
            if w[0] == "SET_HEATER_TEMPERATURE":
                st["target"] = float(line.split("TARGET=")[1])
            elif w[0] == "LOAD_CELL_PROBE":
                st["taps"] += 1
                if st["taps"] == fail_at:
                    raise F.Err("Load cell probe: triggered prior to"
                                " movement")
                lc.last_z_result = 0.0

    mesh = [None]
    objs = {
        "toolhead": F.Obj(get_status=lambda e: {
            "homed_axes": "xyz", "axis_minimum": [-2.0, -2.0, -10.0, 0.0],
            "axis_maximum": [306.5, 306.0, 302.0, 0.0]},
            get_extruder=lambda: F.Obj(get_heater=lambda: F.Obj(
                get_status=lambda e: {"temperature": 150.0}))),
        "save_variables": F.Obj(allVariables={}),
        "bed_mesh": F.Obj(get_mesh=lambda: mesh[0],
                          set_mesh=lambda m: mesh.__setitem__(0, m),
                          bmc=F.Obj(zero_ref_pos=[153.0, 173.3])),
        "print_stats": F.Obj(get_status=lambda e: {"state": "standby"}),
        "gcode_move": F.Obj(get_status=lambda e: {
            "homing_origin": [0, 0, 0]}),
        "load_cell_probe": F.Obj(get_printer_probe=lambda: lc),
        "heater_bed": F.Obj(get_status=lambda e: {
            "temperature": 70.0, "target": st["target"]}),
    }
    L = object.__new__(lidar.Lidar)
    L.printer = F.Obj(lookup_object=lambda n, d=None: objs.get(n, d),
                      command_error=F.Err)
    L.reactor = F.Obj(monotonic=lambda: 0.0, pause=lambda w: w)
    L.gcode = F.Obj(run_script_from_command=run)
    L.service, L.scan_z = "HELPER_IP:7131", 3.0
    L.x_offset, L.y_offset, L.offset_z = -36.013, -21.336, 3.0
    L.x_per_z, L.y_per_z = 0.0354, -0.986
    L.twist_reading = lambda x, y, z=None: -25.6
    return L, st


class BedHeaterRestored(unittest.TestCase):
    def test_failed_touch_restores_bed_target(self):
        for name, call, gc in (
                ("LIDAR_GRID_CALIBRATE", "cmd_LIDAR_GRID_CALIBRATE",
                 F.Gcmd(count="3,3")),
                ("LIDAR_TWIST_CALIBRATE", "cmd_LIDAR_TWIST_CALIBRATE",
                 F.Gcmd(points="3")),
                ("touch_grid (control)", None, None)):
            with self.subTest(name):
                # tap 1 = throw-away, tap 3 = a real touch
                L, st = make_lidar(fail_at=3)
                try:
                    if call:
                        getattr(L, call)(gc)
                    else:
                        L.touch_grid(F.Gcmd(), [(100.0, 100.0),
                                                (150.0, 150.0)])
                except F.Err:
                    pass
                self.assertEqual(st["taps"], 3, "%s never reached the"
                                 " failing touch" % (name,))
                self.assertEqual(st["target"], 70.0,
                                 "%s left the bed heater at %.0f"
                                 % (name, st["target"]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
