"""Regression (K1 bug hunt 2026-11-16 #20): scans restore their state when
the setup fails (Lidar._scan_dwell / _scan): a service failure right after
SAVE_GCODE_STATE still runs RESTORE_GCODE_STATE; a failing open_flow still
closes the laser.
"""
import unittest

import regress_fakes as F


class Toolhead:
    def wait_moves(self):
        pass

    def get_last_move_time(self):
        return 0.0


class Mcu:
    def estimated_print_time(self, t):
        return t


class ScanStateRestore(unittest.TestCase):
    def test_scan_dwell_restores_gcode_state_when_service_fails(self):
        g = F.Gcode()
        L = F.make_lidar({"toolhead": Toolhead(), "mcu": Mcu()}, gcode=g,
                         service="HELPER_IP:7131")

        def down(*a, **k):
            raise F.CmdError("LiDAR: no answer from the lidar service")
        L._job_exposure = lambda: None
        L._svc = down
        try:
            L._scan_dwell("s", 100.0, 10.0, 20.0, 1.0, 3.0)
        except F.CmdError:
            pass
        self.assertTrue(any("RESTORE_GCODE_STATE" in s for s in g.scripts),
                        "scripts run: %r (no RESTORE_GCODE_STATE)"
                        % (g.scripts,))

    def test_scan_laser_off_when_open_flow_fails(self):
        g = F.Gcode()
        L = F.make_lidar({"toolhead": Toolhead()}, gcode=g)
        calls = []

        def call(msg, timeout=10.0):
            calls.append(msg["control"])
            if msg["control"] == "open_flow":
                raise F.CmdError("LiDAR: no answer from the driver")
            return {"result": {"code": 0}}
        L._call = call
        try:
            L._scan("s", 100.0, 10.0, 20.0, 1.0, 3.0, 1)
        except F.CmdError:
            pass
        self.assertIn("set_laser_close", calls,
                      "laser opened, never closed: %r" % (calls,))


if __name__ == "__main__":
    unittest.main(verbosity=2)
