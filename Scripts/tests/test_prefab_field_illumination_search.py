"""Exercise search configuration only; these tests never initialize the engine or run ray tracing."""

import copy
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from search_prefab_field_illumination import build_cases, select_run_ids


class SearchRoiTest(unittest.TestCase):
    def setUp(self):
        config_path = Path(__file__).resolve().parents[1] / "prefab_field_illumination_search.example.json"
        self.config = json.loads(config_path.read_text(encoding="utf-8"))

    def test_old_config_defaults_to_full_field(self):
        self.config["search"].pop("roi", None)
        cases = build_cases(self.config)
        self.assertEqual(len(cases), 594)
        self.assertTrue(all(case["roi"] == 1.0 for case in cases))

    def test_roi_is_an_independent_search_dimension(self):
        self.config["search"].update(rows=[8], columns=[20], roi=[1.0, 0.5])
        cases = build_cases(self.config)
        self.assertEqual(len(cases), 1188)
        self.assertEqual({case["roi"] for case in cases}, {1.0, 0.5})
        self.assertTrue(all(case["rows"] * case["columns"] == 160 for case in cases))

    def test_rejects_invalid_roi(self):
        for roi in [0, -1, 1.01, float("nan"), float("inf")]:
            with self.subTest(roi=roi):
                config = copy.deepcopy(self.config)
                config["search"]["roi"] = [roi]
                with self.assertRaises(ValueError):
                    build_cases(config)

    def test_selected_spacing_pairs_do_not_form_a_cartesian_grid(self):
        self.config["search"].update(spacing_pairs_inches=[[10, 1], [35, 5]], seed=[101], samples=[16, 256])
        cases = build_cases(self.config)
        self.assertEqual(len(cases), 8)
        self.assertEqual({(case["row_spacing_inches"], case["column_spacing_inches"]) for case in cases}, {(10, 1), (35, 5)})
        self.assertEqual({case["samples"] for case in cases}, {16, 256})

    def test_continuation_preserves_original_ids(self):
        self.assertEqual(select_run_ids({}, 3), [0, 1, 2])
        self.assertEqual(select_run_ids({"run_ids": [989, 890]}, 990), [890, 989])
        for values in [[], [0, 0], [-1], [990], [True], [1.0], "0"]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                select_run_ids({"run_ids": values}, 990)


if __name__ == "__main__":
    unittest.main()
