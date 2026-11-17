"""FIRST_LAYER_SCAN in and outside prints, on the simulated printer of
regress_sim.py (K1 bug hunt 2026-11-16):

#6  PHASE=before inside START_PRINT skips the optional check when the
    helper service fails (fl_begin or fl_strip) instead of aborting the
    print.
#19 PHASE=after FINAL=1 learns no Z offset outside a real print (state
    standby, no filament name).
Skipped without numpy."""
import os
import shutil
import sys
import tempfile
import unittest

try:
    import numpy  # noqa: F401
except ImportError:  # pragma: no cover (CI installs it)
    raise unittest.SkipTest("the simulator needs numpy")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import regress_sim as S  # noqa: E402

TMP = None


def setUpModule():
    global TMP
    TMP = tempfile.mkdtemp(prefix="ftb-fl-check-")


def tearDownModule():
    shutil.rmtree(TMP, ignore_errors=True)


def before(L):
    L.cmd_FIRST_LAYER_SCAN(S.Gcmd(phase="before", height=0.2,
                                  filament_type="PLA",
                                  filament_name="PLA Test"))


class Control(unittest.TestCase):
    def test_print_measures_the_first_layer(self):
        # the harness works: the planted 0.23 mm layer (0.20 sliced) is seen
        env, world, st = S.fl_setup(TMP)
        before(env.L)
        self.assertTrue(env.L.get_status(0)["first_layer_armed"])
        world.printed = True
        res = env.L.cmd_FIRST_LAYER_SCAN(S.Gcmd(phase="after"))
        self.assertTrue(res and res.get("ok"), res)
        self.assertAlmostEqual(res["error"], 0.03, delta=0.006)


class BeforePhaseServiceDown(unittest.TestCase):
    def test_service_failure_skips_check(self):
        for bad in ("fl_begin", "fl_strip"):
            with self.subTest(failing=bad):
                env, world, st = S.fl_setup(TMP)
                L = env.L
                real = L._svc

                def svc(req, timeout=30.0, bad=bad, real=real, L=L):
                    if req.get("cmd") == bad:
                        raise L.printer.command_error(
                            "LiDAR: no answer from the lidar service at"
                            " HELPER_IP:7131 (timeout)")
                    return real(req, timeout)
                L._svc = svc
                g = S.Gcmd(phase="before", height=0.2, filament_type="PLA",
                           filament_name="PLA Test")
                try:
                    L.cmd_FIRST_LAYER_SCAN(g)
                except L.printer.command_error as e:
                    self.fail("%s failing: FIRST_LAYER_SCAN PHASE=before"
                              " raised %r -> START_PRINT would abort"
                              % (bad, str(e)))
                self.assertIsNone(L.fl)
                self.assertTrue(any("skipped" in m for m in g.msgs), g.msgs)


class NoLearningOutsidePrint(unittest.TestCase):
    def test_final_outside_print_learns_nothing(self):
        env, world, st = S.fl_setup(TMP, learn=1, state="standby")
        st["filename"] = ""
        L = env.L
        L.cmd_FIRST_LAYER_SCAN(S.Gcmd(phase="before", height=0.2,
                                      filament_type="", filament_name=""))
        world.printed = True
        L.cmd_FIRST_LAYER_SCAN(S.Gcmd(phase="after", final=1))
        z = env.saved.get("fl_zoffset", {})
        self.assertNotIn("unknown", z, "learned fl_zoffset['unknown']"
                         " outside a print: %r" % (z,))


if __name__ == "__main__":
    unittest.main(verbosity=2)
