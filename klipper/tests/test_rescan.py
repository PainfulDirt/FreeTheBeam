"""Continuous LiDAR mesh (levels 1-3) with a column that got no frames (a
driver prefetch timeout + module re-init during the first column): the
column is scanned once more inside the same scan and the mesh succeeds;
nothing on the rescan either -> the same error as before. On the simulated
printer of regress_sim.py; skipped without numpy."""
import json
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
    TMP = tempfile.mkdtemp(prefix="ftb-rescan-")


def tearDownModule():
    shutil.rmtree(TMP, ignore_errors=True)


def env_with_dead_column(times):
    """The simulator, whose driver answers with error frames (no points)
    while the toolhead is on the first column, the first `times` visits."""
    env = S.build(TMP)
    L, sim = env.L, env.sim
    first = {}
    orig_scan = L._scan_moving
    state = {"visits": 0, "on": False}

    def scan_moving(name, xs, y0, y1, z, *a, **kw):
        first["nx"] = xs[0] - L._xo(z)
        return orig_scan(name, xs, y0, y1, z, *a, **kw)
    L._scan_moving = scan_moving
    recv = L._recv

    def bad_recv(timeout=10.0):
        raw, t = recv(timeout)
        a = json.loads(raw.decode())["t"]
        (px, py, pz), v = sim.at(a)
        on = "nx" in first and abs(px - first["nx"]) < 0.01
        if on and not state["on"]:
            state["visits"] += 1
        state["on"] = on
        if on and state["visits"] <= times:
            # what the driver sends after a prefetch timeout
            return json.dumps({"t": a, "code": -2, "result": {}}).encode(), t
        return raw, t
    L._recv = bad_recv
    return env, first, state


class PoorColumns(unittest.TestCase):
    def test_rule(self):
        pc = S.lidar.poor_columns
        xs = [5.0, 30.0, 55.0, 80.0, 105.0]
        self.assertEqual(pc(xs, dict(zip(xs, [14, 15, 13, 16, 14]))), [])
        self.assertEqual(pc(xs, dict(zip(xs, [0, 15, 13, 16, 14]))), [5.0])
        # under 30 % of the median column
        self.assertEqual(pc(xs, dict(zip(xs, [4, 15, 13, 16, 14]))), [5.0])
        self.assertEqual(pc(xs, dict(zip(xs, [5, 15, 13, 16, 14]))), [])
        self.assertEqual(pc(xs, dict(zip(xs, [0, 15, 0, 16, 14]))),
                         [5.0, 55.0])
        # more than two: not a hiccup, no rescans
        self.assertEqual(pc(xs, dict(zip(xs, [0, 0, 0, 16, 14]))), [])


class Rescan(unittest.TestCase):
    def test_baseline_no_rescan(self):
        env, first, state = env_with_dead_column(0)
        g = S.Gcmd(level=2, profile="ok")
        env.L.cmd_LIDAR_BED_MESH(g)
        self.assertFalse(any("rescanned" in m for m in g.msgs))

    def test_one_dead_column_rescanned(self):
        env, first, state = env_with_dead_column(1)
        g = S.Gcmd(level=2, profile="r1")
        env.L.cmd_LIDAR_BED_MESH(g)
        msgs = [m for m in g.msgs if "rescanned" in m]
        self.assertEqual(len(msgs), 1, g.msgs)
        self.assertIn("LiDAR mesh: rescanned X", msgs[0])
        self.assertGreaterEqual(state["visits"], 2)
        # as good as the same mesh without the hiccup
        rms, mx = S.truth_vs(env, "r1")
        ok, first, state = env_with_dead_column(0)
        ok.L.cmd_LIDAR_BED_MESH(S.Gcmd(level=2, profile="ok"))
        self.assertLess(rms, S.truth_vs(ok, "ok")[0] + 3.0)

    def test_dead_twice_same_error(self):
        env, first, state = env_with_dead_column(99)
        g = S.Gcmd(level=2, profile="r2")
        with self.assertRaises(S.Err) as cm:
            env.L.cmd_LIDAR_BED_MESH(g)
        self.assertIn("the LiDAR saw nothing along X", str(cm.exception))
        self.assertEqual(state["visits"], 2)    # one rescan, no more

    def test_level1_and_3(self):
        for lvl in (1, 3):
            with self.subTest(level=lvl):
                env, first, state = env_with_dead_column(1)
                g = S.Gcmd(level=lvl, profile="l%d" % lvl)
                env.L.cmd_LIDAR_BED_MESH(g)
                self.assertEqual(
                    len([m for m in g.msgs if "rescanned" in m]), 1, g.msgs)


if __name__ == "__main__":
    unittest.main(verbosity=2)
