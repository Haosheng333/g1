"""standing_store: recording, reload, preservation, atomicity and map mismatch.

Pure file logic -- no ROS, no robot, no TF.
"""
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from g1_slam import standing_store as store  # noqa: E402

IDENT = store.identity("g1_map_20261016_101500", "20261016_101500", "map", "pelvis")
OTHER = store.identity("g1_map_20261017_090000", "20261017_090000", "map", "pelvis")


class Naming(unittest.TestCase):
    def test_names_are_normalised_and_validated(self):
        self.assertEqual(store.normalize_name("  Handover_01 "), "handover_01")
        for bad in ("", "  ", "has space", "UPPER!", "x" * 41, "-starts-with-dash"):
            with self.assertRaises(store.StoreError, msg=repr(bad)):
                store.normalize_name(bad)

    def test_next_name_is_unique_and_sequential(self):
        self.assertEqual(store.next_name({}, "handover"), "handover_01")
        self.assertEqual(store.next_name({"handover_01": {}}, "handover"), "handover_02")
        taken = {"handover_%02d" % i: {} for i in range(1, 4)}
        self.assertEqual(store.next_name(taken, "handover"), "handover_04")

    def test_next_name_skips_gaps_without_reusing(self):
        self.assertEqual(store.next_name({"handover_01": {}, "handover_03": {}},
                                         "handover"), "handover_02")


class RecordAndReload(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "standing_waypoints.yaml")

    def _add(self, **kw):
        base = dict(x=1.25, y=-0.5, yaw=1.5707963,
                    recorded_at="2026-10-16T10:22:31", stamp=1792252951.284)
        base.update(kw)
        return store.add(self.path, IDENT, **base)

    def test_a_record_round_trips_through_the_file(self):
        name, rec = self._add()
        self.assertEqual(name, "handover_01")
        ident, waypoints = store.load(self.path)
        self.assertTrue(store.same_map(ident, IDENT))
        self.assertEqual(sorted(waypoints), ["handover_01"])
        got = waypoints["handover_01"]
        self.assertAlmostEqual(got["x"], 1.25, places=4)
        self.assertAlmostEqual(got["y"], -0.5, places=4)
        self.assertAlmostEqual(got["yaw"], 1.5708, places=4)
        self.assertEqual(got["map_frame"], "map")
        self.assertEqual(got["robot_frame"], "pelvis")
        self.assertEqual(got["recorded_at"], "2026-10-16T10:22:31")
        self.assertAlmostEqual(got["stamp"], 1792252951.284, places=3)

    def test_the_identity_block_is_written(self):
        self._add()
        ident, _ = store.load(self.path)
        self.assertEqual(ident["name"], "g1_map_20261016_101500")
        self.assertEqual(ident["created_at"], "20261016_101500")
        self.assertEqual(ident["map_frame"], "map")

    def test_existing_records_are_preserved(self):
        self._add()
        self._add(x=2.0, y=3.0, yaw=0.0)
        self._add(x=-1.0, y=0.25, yaw=-0.7)
        _ident, waypoints = store.load(self.path)
        self.assertEqual(sorted(waypoints),
                         ["handover_01", "handover_02", "handover_03"])
        # the first record is untouched
        self.assertAlmostEqual(waypoints["handover_01"]["x"], 1.25, places=4)

    def test_a_hand_edited_record_survives_a_later_append(self):
        self._add()
        ident, waypoints = store.load(self.path)
        waypoints["kitchen_door"] = {"x": 9.0, "y": 8.0, "yaw": 0.1,
                                     "recorded_at": "manual", "stamp": 1.0,
                                     "map_frame": "map", "robot_frame": "pelvis"}
        store.save(self.path, ident, waypoints)
        self._add(x=0.0, y=0.0, yaw=0.0)
        _i, after = store.load(self.path)
        self.assertIn("kitchen_door", after)
        self.assertAlmostEqual(after["kitchen_door"]["x"], 9.0, places=4)

    def test_an_explicit_name_is_honoured(self):
        name, _ = self._add(name="Handover_Spot")
        self.assertEqual(name, "handover_spot")

    def test_a_duplicate_name_is_refused_without_replace(self):
        self._add(name="spot")
        with self.assertRaises(store.StoreError):
            self._add(name="spot", x=5.0)
        _i, waypoints = store.load(self.path)
        self.assertAlmostEqual(waypoints["spot"]["x"], 1.25, places=4)

    def test_replace_overwrites_only_that_record(self):
        self._add(name="spot")
        self._add(name="other", x=7.0)
        self._add(name="spot", x=5.0, replace=True)
        _i, waypoints = store.load(self.path)
        self.assertAlmostEqual(waypoints["spot"]["x"], 5.0, places=4)
        self.assertAlmostEqual(waypoints["other"]["x"], 7.0, places=4)

    def test_a_snapshot_filename_round_trips(self):
        self._add(cloud_snapshot="handover_01_20261016_102231.pcd")
        _i, waypoints = store.load(self.path)
        self.assertEqual(waypoints["handover_01"]["cloud_snapshot"],
                         "handover_01_20261016_102231.pcd")

    def test_non_finite_coordinates_are_refused(self):
        for bad in ({"x": float("nan")}, {"y": float("inf")}, {"yaw": float("-inf")}):
            with self.assertRaises(store.StoreError):
                self._add(**bad)
        self.assertFalse(os.path.exists(self.path))

    def test_get_returns_the_identity_and_the_record(self):
        self._add(name="spot")
        ident, key, rec = store.get(self.path, "SPOT")
        self.assertEqual(key, "spot")
        self.assertTrue(store.same_map(ident, IDENT))
        self.assertAlmostEqual(rec["x"], 1.25, places=4)

    def test_get_on_a_missing_name_raises_with_the_known_names(self):
        self._add(name="spot")
        with self.assertRaises(KeyError) as ctx:
            store.get(self.path, "nope")
        self.assertIn("spot", str(ctx.exception))

    def test_remove_keeps_the_rest(self):
        self._add(name="a")
        self._add(name="b", x=2.0)
        store.remove(self.path, "a")
        _i, waypoints = store.load(self.path)
        self.assertEqual(sorted(waypoints), ["b"])


class MapMismatchIsRefused(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "standing_waypoints.yaml")
        store.add(self.path, IDENT, x=1.0, y=2.0, yaw=0.0,
                  recorded_at="t", stamp=1.0, name="first")

    def test_a_different_map_name_is_refused(self):
        with self.assertRaises(store.MapMismatch):
            store.add(self.path, OTHER, x=0.0, y=0.0, yaw=0.0,
                      recorded_at="t", stamp=2.0)

    def test_the_same_name_remade_at_a_different_time_is_refused(self):
        """A rebuilt map has a new origin even under the same name."""
        remade = store.identity("g1_map_20261016_101500", "20261016_180000")
        with self.assertRaises(store.MapMismatch):
            store.add(self.path, remade, x=0.0, y=0.0, yaw=0.0,
                      recorded_at="t", stamp=2.0)

    def test_the_refusal_names_both_maps(self):
        try:
            store.add(self.path, OTHER, x=0.0, y=0.0, yaw=0.0,
                      recorded_at="t", stamp=2.0)
            self.fail("expected MapMismatch")
        except store.MapMismatch as exc:
            text = str(exc)
            self.assertIn("g1_map_20261016_101500", text)
            self.assertIn("g1_map_20261017_090000", text)

    def test_nothing_is_written_on_a_mismatch(self):
        before = open(self.path, encoding="utf-8").read()
        with self.assertRaises(store.MapMismatch):
            store.add(self.path, OTHER, x=0.0, y=0.0, yaw=0.0,
                      recorded_at="t", stamp=2.0)
        self.assertEqual(open(self.path, encoding="utf-8").read(), before)

    def test_the_matching_map_still_appends(self):
        name, _ = store.add(self.path, IDENT, x=3.0, y=4.0, yaw=0.0,
                            recorded_at="t", stamp=3.0)
        _i, waypoints = store.load(self.path)
        self.assertIn(name, waypoints)
        self.assertIn("first", waypoints)

    def test_same_map_comparison(self):
        self.assertTrue(store.same_map(IDENT, dict(IDENT)))
        self.assertFalse(store.same_map(IDENT, OTHER))
        self.assertFalse(store.same_map(None, IDENT))
        self.assertFalse(store.same_map(IDENT, None))


class MalformedFilesAreRejected(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "w.yaml")

    def _write(self, text):
        with open(self.path, "w", encoding="utf-8") as fh:
            fh.write(text)

    def test_a_list_top_level_is_rejected(self):
        self._write("- 1\n- 2\n")
        with self.assertRaises(store.StoreError):
            store.load(self.path)

    def test_a_non_mapping_waypoints_block_is_rejected(self):
        self._write("waypoints: [1, 2]\n")
        with self.assertRaises(store.StoreError):
            store.load(self.path)

    def test_a_missing_coordinate_is_rejected(self):
        self._write("waypoints:\n  a:\n    x: 1.0\n    y: 2.0\n")
        with self.assertRaises(store.StoreError):
            store.load(self.path)

    def test_a_non_numeric_coordinate_is_rejected(self):
        self._write("waypoints:\n  a:\n    x: hello\n    y: 2.0\n    yaw: 0.0\n")
        with self.assertRaises(store.StoreError):
            store.load(self.path)

    def test_a_non_finite_coordinate_is_rejected(self):
        self._write("waypoints:\n  a:\n    x: .nan\n    y: 2.0\n    yaw: 0.0\n")
        with self.assertRaises(store.StoreError):
            store.load(self.path)

    def test_case_duplicate_names_are_rejected(self):
        self._write("waypoints:\n"
                    "  spot:\n    x: 1.0\n    y: 1.0\n    yaw: 0.0\n"
                    "  SPOT:\n    x: 2.0\n    y: 2.0\n    yaw: 0.0\n")
        with self.assertRaises(store.StoreError):
            store.load(self.path)

    def test_an_empty_file_loads_as_empty(self):
        self._write(store.empty_file_text(IDENT))
        ident, waypoints = store.load(self.path)
        self.assertEqual(waypoints, {})
        self.assertTrue(store.same_map(ident, IDENT))


class WritesAreAtomic(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "w.yaml")

    def test_no_temp_file_is_left_behind(self):
        store.add(self.path, IDENT, x=1.0, y=1.0, yaw=0.0, recorded_at="t", stamp=1.0)
        leftovers = [n for n in os.listdir(self.tmp.name) if n.startswith(".standing")]
        self.assertEqual(leftovers, [])

    def test_a_failed_write_leaves_the_previous_file_intact(self):
        store.add(self.path, IDENT, x=1.0, y=1.0, yaw=0.0, recorded_at="t",
                  stamp=1.0, name="good")
        before = open(self.path, encoding="utf-8").read()
        import unittest.mock as m
        with m.patch("os.replace", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                store.add(self.path, IDENT, x=2.0, y=2.0, yaw=0.0,
                          recorded_at="t", stamp=2.0, name="bad")
        self.assertEqual(open(self.path, encoding="utf-8").read(), before)
        leftovers = [n for n in os.listdir(self.tmp.name) if n.startswith(".standing")]
        self.assertEqual(leftovers, [], "a temp file was left behind")

    def test_writing_into_a_missing_directory_is_refused(self):
        with self.assertRaises(store.StoreError):
            store.save(os.path.join(self.tmp.name, "nope", "w.yaml"), IDENT, {})

    def test_the_header_explains_the_no_offset_rule(self):
        store.add(self.path, IDENT, x=1.0, y=1.0, yaw=0.0, recorded_at="t", stamp=1.0)
        text = open(self.path, encoding="utf-8").read()
        self.assertIn("/move_base_simple/goal", text)
        self.assertIn("No approach-distance offset", text)
        self.assertIn("WHERE THE ROBOT STOOD", text)


if __name__ == "__main__":
    unittest.main()
