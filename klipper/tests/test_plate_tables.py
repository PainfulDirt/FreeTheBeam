"""Plate-tagged LiDAR calibration tables (klipper/lidar.py): LIDAR_PLATE
NAME= (lidar_plate), runs in lidar_cal_by_temp tagged with the plate, meshes
use the current plate's runs (untagged runs only where its group has none,
other plates never), legacy single tables only when untagged or the same
plate, RESET_RUNS=1 keeps the other plates' runs. No plate set: everything
as before."""
import ast
import os
import unittest

import fake_klipper  # noqa: F401  (klipper/ on sys.path)
import lidar
from fake_klipper import Err, Obj

HERE = os.path.dirname(os.path.abspath(__file__))


class Gcmd:
    def __init__(self, **p):
        self.p = dict((k.upper(), str(v)) for k, v in p.items())
        self.msgs = []

    def get(self, k, default=None):
        return self.p.get(k, default)

    def get_float(self, k, default=None, **kw):
        return float(self.p[k]) if k in self.p else default

    def get_int(self, k, default=None, **kw):
        return int(self.p[k]) if k in self.p else default

    def respond_info(self, m):
        self.msgs.append(m)

    def error(self, m):
        return Err(m)


def tab(v, key="xs"):
    return {key: [0.0, 300.0], "comp": [v, v]}


def run(v, plate="-", t="2026-11-17T10:00", yv=None):
    r = {"twist": tab(v), "time": t}
    if yv is not None:
        r["ytwist"] = tab(yv, "ys")
    if plate != "-":
        r["plate"] = plate
    return r


def make(saved, bed=70.0):
    def script(s):
        for line in s.split("\n"):
            w = line.split()
            if w and w[0] == "SAVE_VARIABLE":
                d = dict(a.split("=", 1) for a in w[1:])
                saved[d["VARIABLE"]] = ast.literal_eval(
                    d["VALUE"].strip('"'))
    objs = {"save_variables": Obj(allVariables=saved),
            "heater_bed": Obj(get_status=lambda e: {"temperature": bed,
                                                     "target": bed}),
            "toolhead": Obj(get_extruder=lambda: Obj(get_heater=lambda: Obj(
                get_status=lambda e: {"temperature": 150.0})))}
    L = object.__new__(lidar.Lidar)
    L.printer = Obj(lookup_object=lambda n, d=None: objs.get(n, d),
                    command_error=Err)
    L.reactor = Obj(monotonic=lambda: 0.0, pause=lambda w: w)
    L.gcode = Obj(run_script_from_command=script)
    return L


def val(t):
    return None if t is None else t["comp"][0]


class Selection(unittest.TestCase):
    STORE = {"70": {"runs": [run(0.01), run(0.02, "black"),
                             run(0.04, "textured")]},
             "100": {"runs": [run(0.08, "black")]}}

    def test_no_plate_uses_all_runs(self):
        tw, yt, note = lidar.tables_for_temp(self.STORE, 70.0)
        self.assertAlmostEqual(val(tw), (0.01 + 0.02 + 0.04 + 0.08) / 4)
        self.assertNotIn("untagged", note)

    def test_plate_uses_only_its_runs(self):
        tw, yt, note = lidar.tables_for_temp(self.STORE, 70.0, "black")
        self.assertAlmostEqual(val(tw), (0.02 + 0.08) / 2)
        self.assertNotIn("untagged", note)

    def test_other_plate_never_used(self):
        store = {"70": {"runs": [run(0.04, "textured")]}}
        tw, yt, note = lidar.tables_for_temp(store, 70.0, "black")
        self.assertIsNone(tw)
        self.assertIsNone(yt)

    def test_untagged_only_where_group_has_no_plate_run(self):
        store = {"70": {"runs": [run(0.01), run(0.04, "textured")]},
                 "25": {"runs": [run(0.03, "black")]}}
        # hot group: no black run -> the untagged one, said so
        tw, yt, note = lidar.tables_for_temp(store, 70.0, "black")
        self.assertAlmostEqual(val(tw), 0.01)
        self.assertIn("untagged", note)
        # cold group has a black run: the untagged hot one isn't mixed in
        tw, yt, note = lidar.tables_for_temp(store, 25.0, "black")
        self.assertAlmostEqual(val(tw), 0.03)
        self.assertNotIn("untagged", note)

    def test_plate_none_run_counts_as_untagged(self):
        store = {"70": {"runs": [run(0.01, None), run(0.04, "textured")]}}
        tw, yt, note = lidar.tables_for_temp(store, 70.0, "black")
        self.assertAlmostEqual(val(tw), 0.01)
        self.assertIn("untagged", note)

    def test_old_entry_without_runs_is_untagged(self):
        store = {"70": {"twist": tab(0.05), "time": "x"}}
        tw, yt, note = lidar.tables_for_temp(store, 70.0, "black")
        self.assertAlmostEqual(val(tw), 0.05)
        self.assertIn("untagged", note)


class LidarTables(unittest.TestCase):
    def test_no_plate_exactly_as_before(self):
        saved = {"lidar_cal_by_temp": Selection.STORE,
                 "lidar_twist": tab(0.5), "lidar_ytwist": tab(0.6, "ys")}
        tw, yt, note = make(saved)._lidar_tables()
        self.assertAlmostEqual(val(tw), 0.0375)
        self.assertAlmostEqual(val(yt), 0.6)    # no Y run: the last one
        self.assertTrue(note.startswith("bed 70C: "), note)
        saved = {"lidar_twist": tab(0.5), "lidar_ytwist": tab(0.6, "ys")}
        tw, yt, note = make(saved)._lidar_tables()
        self.assertEqual((val(tw), val(yt), note), (0.5, 0.6, ""))

    def test_plate_named_in_note(self):
        saved = {"lidar_cal_by_temp": Selection.STORE,
                 "lidar_plate": "black"}
        tw, yt, note = make(saved)._lidar_tables()
        self.assertAlmostEqual(val(tw), 0.05)
        self.assertIn("plate black", note)
        self.assertIn("bed 70C", note)

    def test_legacy_tables_untagged_or_same_plate(self):
        saved = {"lidar_plate": "black", "lidar_twist": tab(0.5),
                 "lidar_ytwist": dict(tab(0.6, "ys"), plate="black")}
        tw, yt, note = make(saved)._lidar_tables()
        self.assertEqual((val(tw), val(yt)), (0.5, 0.6))
        self.assertIn("plate black", note)

    def test_legacy_tables_of_other_plate_refused(self):
        saved = {"lidar_plate": "black",
                 "lidar_cal_by_temp": {"70": {"runs": [run(0.04, "tex")]}},
                 "lidar_twist": dict(tab(0.5), plate="tex"),
                 "lidar_ytwist": dict(tab(0.6, "ys"), plate="tex")}
        tw, yt, note = make(saved)._lidar_tables()
        self.assertEqual((tw, yt), (None, None))
        self.assertIn("no LiDAR calibration for plate black", note)


class Store(unittest.TestCase):
    def test_store_tags_plate(self):
        saved = {"lidar_plate": "black",
                 "lidar_cal_by_temp": {"70": {"runs": [run(0.01)]}}}
        make(saved)._store_by_temp({"twist": tab(0.02)})
        runs = saved["lidar_cal_by_temp"]["70"]["runs"]
        self.assertEqual(len(runs), 2)
        self.assertNotIn("plate", runs[0])
        self.assertEqual(runs[1]["plate"], "black")

    def test_store_without_plate(self):
        saved = {}
        make(saved)._store_by_temp({"twist": tab(0.02)})
        self.assertIsNone(saved["lidar_cal_by_temp"]["70"]["runs"][0][
            "plate"])

    def test_twist_save_tags_plate(self):
        saved = {"lidar_plate": "black"}
        L = make(saved)
        L.twist_save(Gcmd(), [10.0, 150.0, 290.0], [0.01, 0.0, -0.01],
                     145.0, 3.0, 0.01, [25.6, 25.6])
        self.assertEqual(saved["lidar_twist"]["plate"], "black")
        self.assertEqual(saved["lidar_cal_by_temp"]["70"]["runs"][0][
            "plate"], "black")


class PlateCommand(unittest.TestCase):
    def test_name_validation(self):
        for bad in ("Black", "black pei", "a-b", "", "x" * 33, "pei!"):
            saved = {}
            with self.assertRaises(Err):
                make(saved).cmd_LIDAR_PLATE(Gcmd(name=bad))
            self.assertNotIn("lidar_plate", saved)
        for good in ("black_pei", "x" * 32, "pei2"):
            saved = {}
            make(saved).cmd_LIDAR_PLATE(Gcmd(name=good))
            self.assertEqual(saved["lidar_plate"], good)

    def test_report(self):
        saved = {"lidar_plate": "black",
                 "lidar_cal_by_temp": {
                     "70": {"runs": [run(0.01, t="2026-11-01T10:00"),
                                     run(0.02, "black",
                                         t="2026-11-17T09:00")]},
                     "100": {"runs": [run(0.08, "black",
                                          t="2026-11-17T11:00")]}}}
        g = Gcmd()
        make(saved).cmd_LIDAR_PLATE(g)
        text = "\n".join(g.msgs)
        self.assertIn("black", text)
        self.assertIn("2026-11-17T11:00", text)
        self.assertIn("untagged", text)
        self.assertIn("100", text)

    def test_report_warns_without_runs(self):
        saved = {"lidar_plate": "smooth",
                 "lidar_cal_by_temp": {"70": {"runs": [run(0.02, "black")]}}}
        g = Gcmd()
        make(saved).cmd_LIDAR_PLATE(g)
        self.assertIn("no calibration runs", "\n".join(g.msgs))

    def test_reset_keeps_other_plates(self):
        saved = {"lidar_plate": "black",
                 "lidar_cal_by_temp": {
                     "70": {"runs": [run(0.01), run(0.02, "black"),
                                     run(0.04, "tex")]},
                     "100": {"runs": [run(0.08, "black")]},
                     "60": {"twist": tab(0.03), "time": "old"}}}
        make(saved).cmd_LIDAR_PLATE(Gcmd(reset_runs=1))
        st = saved["lidar_cal_by_temp"]
        self.assertEqual(list(st), ["70"])
        self.assertEqual([r.get("plate") for r in st["70"]["runs"]],
                         ["tex"])

    def test_reset_without_plate_clears_all(self):
        saved = {"lidar_cal_by_temp": {"70": {"runs": [run(0.04, "tex")]}}}
        make(saved).cmd_LIDAR_PLATE(Gcmd(reset_runs=1))
        self.assertEqual(saved["lidar_cal_by_temp"], {})


class Registered(unittest.TestCase):
    def test_command_and_description(self):
        with open(os.path.join(HERE, "..", "lidar.py")) as f:
            src = f.read()
        self.assertIn('"LIDAR_PLATE"', src)
        self.assertTrue(lidar.Lidar.desc_LIDAR_PLATE)


if __name__ == "__main__":
    unittest.main()
