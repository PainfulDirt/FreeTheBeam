"""lidar.py with the service on the printer itself ([lidar] service_local:
True, no helper computer): analysis timeouts x3 (max 300 s), LIDAR_STATUS
names the mode, the camera work (PA/flow print, camera offset cross) is not
attempted. A 127.0.0.1 service alone is not local mode (a helper with numpy
on the same host)."""
import unittest

import regress_fakes as F


def make(service, local=True):
    L = F.make_lidar(service=service)
    L.service_local = local
    L.offset_source = "camera"
    L._saved = lambda: {}
    L._plate_report = lambda *a, **kw: []
    return L


class SvcLocalTests(unittest.TestCase):
    def test_scale_local(self):
        for svc in ("127.0.0.1:7131", "localhost:7131"):
            L = make(svc)
            self.assertTrue(L.svc_local)
            self.assertEqual(L._svc_timeout(60), 180)
            self.assertEqual(L._svc_timeout(240), 300)
            self.assertEqual(L._svc_timeout(10), 10)
            self.assertEqual(L._svc_timeout(30.0), 90.0)

    def test_helper_unchanged(self):
        for svc, local in (("HELPER_IP:7131", False),
                           ("127.0.0.1:7132", False)):
            L = make(svc, local)
            self.assertFalse(L.svc_local, svc)
            self.assertEqual(L._svc_timeout(60), 60)
            self.assertEqual(L._svc_timeout(240), 240)
        L = make(None)          # service_local without a service
        self.assertFalse(L.svc_local)

    def test_svc_uses_scaled_deadline(self):
        """A request with timeout=240 waits 300 s in local mode, not 720
        (and not 240)."""
        L = make("127.0.0.1:7131")

        class Sock:
            def send(self, data):
                return len(data)

            def recv(self, n):
                raise BlockingIOError()

            def close(self):
                pass
        L.svc_sock = Sock()
        L._svc_connect = lambda *a, **kw: None
        t0 = L.reactor.monotonic()
        with self.assertRaises(F.CmdError):
            L._svc({"cmd": "fl_analyze"}, timeout=240)
        waited = L.reactor.monotonic() - t0
        self.assertGreater(waited, 299.0)
        self.assertLess(waited, 301.0)

    def status(self, service, local=True):
        L = make(service, local)
        L._driver_msg = lambda msg, timeout=10.0: (
            {"exposure": 4000} if msg["control"] == "get_laser_exposure"
            else {})
        g = F.Gcmd()
        L.cmd_LIDAR_STATUS(g)
        return "\n".join(g.out)

    def test_status_says_local(self):
        self.assertIn("service on this printer (local)",
                      self.status("127.0.0.1:7131"))
        out = self.status("HELPER_IP:7131", local=False)
        self.assertIn("service at HELPER_IP:7131", out)
        self.assertNotIn("(local)", out)


class HelperOnlyTests(unittest.TestCase):
    """Camera calibrations (PA/flow, offset cross) need numpy/PIL on the
    helper: in local mode they are not attempted."""

    def make(self):
        L = make("127.0.0.1:7131")
        L.printer = F.Printer({"print_stats": F.Obj(
            get_status=lambda t: {"state": "standby"})})
        return L

    def test_pa_flow_refused_up_front(self):
        L = self.make()
        g = F.Gcmd(EXTRUDER_TEMP=210, BED_TEMP=60)
        with self.assertRaises(F.CmdError) as e:
            L.cmd_LIDAR_PA_FLOW_CALIBRATE(g)
        self.assertIn("needs the helper", str(e.exception))
        self.assertEqual(L.gcode.scripts, [])   # nothing ran

    def test_offset_and_pa_skip_the_camera(self):
        L = self.make()
        self.assertFalse(L._camera_ok())
        # LIDAR_OFFSET_CALIBRATE / LIDAR_PA_CALIBRATE: the LiDAR-only paths
        seen = []
        L._offset_camera = lambda g: seen.append("camera")
        L._offset_lidar = lambda g: seen.append("lidar")
        L._pa_lidar_print = lambda g: seen.append("pa_lidar")
        L._pf_calibrate = lambda g, **kw: seen.append("pf")
        L.cmd_LIDAR_OFFSET_CALIBRATE(F.Gcmd())
        L.cmd_LIDAR_PA_CALIBRATE(F.Gcmd())
        self.assertEqual(seen, ["lidar", "pa_lidar"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
