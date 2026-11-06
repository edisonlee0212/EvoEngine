"""Recovery joins preserve original bytes and require exactly one copy of every case."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import resume_soybean_study as recovery


class RecoveryTest(unittest.TestCase):
    def test_assembly_is_verbatim_and_rejects_overlapping_attempts(self):
        repo = Path(__file__).resolve().parents[2]
        config = json.loads((repo / "Scripts/prefab_field_illumination_search.example.json").read_text(encoding="utf-8"))
        config["search"].update(rows=[1], columns=[1], roi=[1], seed=[101], row_spacing_inches=[10], column_spacing_inches=[1])
        for overlap, first_complete in [(False, False), (True, False), (False, True), (True, True)]:
            with self.subTest(overlap=overlap, first_complete=first_complete), tempfile.TemporaryDirectory(dir=repo / "out") as directory:
                study = Path(directory)
                first, second = study / "raw/03_field_search", study / "raw/03_field_search_resume"
                for folder in [first, second, study / "configs", study / "timings"]:
                    folder.mkdir(parents=True)
                (study / "configs/03_field_search.json").write_text(json.dumps(config))
                (first / "search_manifest.json").write_text(json.dumps({"config": config}))
                (study / "timings/03_field_search.json").write_text(json.dumps({"started_at_utc": "2026-10-16T00:00:00+00:00", "elapsed_seconds": 10}))
                if first_complete:
                    (first / "completion.json").write_text((study / "timings/03_field_search.json").read_text())
                (second / "completion.json").write_text(json.dumps({"completed_at_utc": "2026-10-16T00:01:00+00:00", "elapsed_seconds": 20}))
                original = b"run_id,value\r\n0,0.125\r\n"
                resumed = f"run_id,value\r\n{0 if overlap else 1},0.375\r\n".encode()
                for name in ["field_summary.csv", "plant_measurements.csv", "field_organ_summary.csv", "plant_organ_measurements.csv", "case_timings.csv"]:
                    (first / name).write_bytes(original)
                    (second / name).write_bytes(resumed)
                with patch.object(recovery, "STUDY", study):
                    if overlap:
                        with self.assertRaisesRegex(ValueError, "overlap"):
                            recovery.assemble()
                        self.assertFalse((study / "assembled").exists())
                    else:
                        recovery.assemble()
                        assembled = study / "assembled/03_field_search"
                        self.assertEqual((assembled / "field_summary.csv").read_bytes(), original + resumed.split(b"\n", 1)[1])
                        self.assertEqual((first / "field_summary.csv").read_bytes(), original)
                        completion = json.loads((assembled / "completion.json").read_text())
                        self.assertEqual(completion["completed_cases"], 2)
                        self.assertEqual(completion["elapsed_seconds"], 30)
                        self.assertEqual(completion["wall_elapsed_seconds"], 60)


if __name__ == "__main__":
    unittest.main()
