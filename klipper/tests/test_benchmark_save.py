"""Regression (K1 bug hunt 2026-11-16 #13): LIDAR_PA_SAVE refuses a stale
benchmark. A PA benchmark for filament A, then a normal camera calibration
that saves filament B's PA: the old prompt's LIDAR_PA_SAVE METHOD=LIDAR must
be refused, not overwrite lidar_pa with A's value. On the harness of
test_pa_flow.py.
"""
import unittest

import regress_fakes as F
import test_pa_flow as T
from fake_klipper import Gcmd


def setUpModule():
    F.isolate_tmp()


def tearDownModule():
    F.restore_tmp()


def run_print(L, **cal):
    L.stats.update(state="standby", filename="")
    L.exposure_job = None   # fake clock is constant: a new job per print
    T.calibrate(L, **cal)
    L.stats.update(state="printing", filename=L.PF_FILE)
    for st in ("BARE", "PHOTOS", "ANALYZE"):
        L.cmd_LIDAR_PA_FLOW_STEP(Gcmd(STEP=st))


class StaleBenchmarkSave(unittest.TestCase):
    def test_pa_save_after_newer_calibration_is_refused(self):
        L = T.make()
        # 1. benchmark for PLA Matte (camera and LiDAR PA), nothing saved
        run_print(L, TESTS="PA", PA_METHOD="BOTH", FILAMENT_NAME="PLA Matte")
        self.assertEqual(T.saves(L), [], "the benchmark saved something")
        # 2. later: filament B loaded, a normal camera PA calibration saves
        del L.ev[:]
        run_print(L, TESTS="PA", PA_METHOD="IR", FILAMENT_NAME="PETG HF")
        self.assertTrue(T.saves(L), "the camera calibration saved nothing")
        # 3. the old prompt's button / a typed LIDAR_PA_SAVE
        del L.ev[:]
        with self.assertRaises(F.Err):
            L.cmd_LIDAR_PA_SAVE(Gcmd(METHOD="LIDAR"))
        self.assertEqual(T.saves(L), [], "LIDAR_PA_SAVE after B's"
                         " calibration saved the PLA Matte benchmark's PA")

    def test_force_saves_anyway(self):
        L = T.make()
        run_print(L, TESTS="PA", PA_METHOD="BOTH", FILAMENT_NAME="PLA Matte")
        run_print(L, TESTS="PA", PA_METHOD="IR", FILAMENT_NAME="PETG HF")
        del L.ev[:]
        L.cmd_LIDAR_PA_SAVE(Gcmd(METHOD="LIDAR", FORCE=1))
        self.assertTrue(any("lidar_pa" in s for s in T.saves(L)), T.saves(L))


if __name__ == "__main__":
    unittest.main(verbosity=2)
