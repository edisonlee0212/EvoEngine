"""Synthetic checks of the scientific quantities, requiring no engine or GPU."""
import sys
import unittest
import json
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_soybean_experiments import analyze_experiment, analyze_study, assert_close, paired_interval, scalar


class AnalysisTest(unittest.TestCase):
    def test_excludes_unpaired_chunks_and_incomplete_attempts(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[2] / "out") as directory:
            root = Path(directory)
            chunk = root / "raw/reference_check"
            chunk.mkdir(parents=True)
            (chunk / "completion.json").write_text(json.dumps({"include_in_study_analysis": False}))
            (root / "assembled/incomplete").mkdir(parents=True)
            self.assertEqual(analyze_study(root, plots=False), {})

    def test_relative_rgb_scalar_preserves_white_and_channel_weights(self):
        frame = pd.DataFrame({"flux_r": [3, 1], "flux_g": [3, 0], "flux_b": [3, 0]})
        np.testing.assert_allclose(scalar(frame, "flux"), [3, .2126])

    def test_leaf_total_is_twice_area_times_face_mean(self):
        area, front, back = 2, 5, 1
        face_mean = (front + back) / 2
        assert_close(area * face_mean * 2, area * (front + back), "two leaf faces")
        with self.assertRaises(ValueError):
            assert_close(area * face_mean, area * (front + back), "wrong old convention")

    def test_intervals_are_deterministic_and_preserve_constant_difference(self):
        self.assertEqual(paired_interval([2] * 5), [2, 2])
        self.assertEqual(paired_interval([2]), [None, None])
        self.assertEqual(paired_interval([1, 2, 3]), paired_interval([1, 2, 3]))

    def test_complete_csv_reconciliation_and_leaf_whole_distinction(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[2] / "out") as directory:
            root = Path(directory)
            (root / "search_manifest.json").write_text(json.dumps({"organ_reporting": {"schema_version": 2}}))
            summaries, plants, organs = [], [], []
            for run, kind, area, irradiance in [(0, "broad", 2, 3), (1, "narrow", 1, 4)]:
                common = dict(run_id=run, asset_name=kind, rows=1, columns=1, roi=1,
                              row_spacing_inches=20, column_spacing_inches=3, samples=256, bounces=2, seed=11,
                              roi_row_start=0, roi_column_start=0, roi_rows=1, roi_columns=1,
                              row_spacing=.5, column_spacing=.1, measurement_ground_area=.05)
                whole = dict(total_area=area + 1, **{f"total_flux_{c}": area * irradiance * 2 + 2 for c in "rgb"})
                summaries.append({**common, **whole, "plant_count": 1})
                plants.append({**common, **whole, "model_index": 0, "generation_seed": 123, "row": 0, "column": 0})
                for organ, surface_area, mean, faces in [("leaf", area, irradiance, 2), ("stem", 1, 2, 1)]:
                    organs.append({**common, "model_index": 0, "organ": organ, "total_area": surface_area,
                                   **{f"total_flux_{c}": surface_area * mean * faces for c in "rgb"},
                                   **{f"average_flux_{c}": mean for c in "rgb"}})
            pd.DataFrame(summaries).to_csv(root / "field_summary.csv", index=False)
            pd.DataFrame(plants).to_csv(root / "plant_measurements.csv", index=False)
            pd.DataFrame(organs).to_csv(root / "plant_organ_measurements.csv", index=False)
            pd.DataFrame(organs).to_csv(root / "field_organ_summary.csv", index=False)
            runs, ranked, paired, facts = analyze_experiment(root, root / "analysis")
            np.testing.assert_allclose(runs.leaf_mean_irradiance, [3, 4])
            np.testing.assert_allclose(runs.leaf_incident_per_plant, [12, 8])
            np.testing.assert_allclose(runs.whole_incident_per_plant, [14, 10])
            self.assertTrue(runs.leaf_incident_per_ground_area.isna().all())
            self.assertEqual(facts["reconciliation"], "passed")
            comparison = {value["metric"]: value for value in facts["condition_comparisons"]}
            self.assertEqual(comparison["leaf_mean_irradiance"]["narrow_mean_higher"], 1)
            self.assertEqual(comparison["leaf_incident_per_plant"]["broad_mean_higher"], 1)
            self.assertEqual(comparison["leaf_incident_per_plant"]["ratio_min"], 1.5)
            self.assertEqual(len(paired), 1)
            summaries[0]["total_flux_r"] += 1
            pd.DataFrame(summaries).to_csv(root / "field_summary.csv", index=False)
            with self.assertRaisesRegex(ValueError, "plants to field"):
                analyze_experiment(root, root / "analysis")


if __name__ == "__main__":
    unittest.main()
