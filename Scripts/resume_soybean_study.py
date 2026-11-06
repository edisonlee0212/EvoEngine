"""User-authorized continuation of the recorded CUDA failure; never overwrites the first attempt."""
import argparse
import copy
import csv
import hashlib
import json
import math
import shutil
import sys
from datetime import datetime
from pathlib import Path

from continue_soybean_study import STUDY, analyze, finish_study, stage
from run_soybean_experiments import ROOT, run_search, save_json
from search_prefab_field_illumination import build_cases


def rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def check_reference():
    """Compare geometry, selected seeds and every exported RGB mean/integral, not execution times."""
    checks = {}
    for name, keys in [("field_summary.csv", ["run_id"]), ("plant_measurements.csv", ["run_id", "model_index"]),
                       ("field_organ_summary.csv", ["run_id", "organ"]),
                       ("plant_organ_measurements.csv", ["run_id", "model_index", "organ"])]:
        original = {tuple(row[key] for key in keys): row for row in rows(STUDY / "raw/03_field_search" / name)}
        repeated = rows(STUDY / "raw/03_reference_check" / name)
        maximum_absolute, maximum_relative = 0.0, 0.0
        for actual in repeated:
            expected = original[tuple(actual[key] for key in keys)]
            for column, value in actual.items():
                if column in {"generation_seed", "source_obj", "asset_index", "row", "column"}:
                    if value != expected[column]:
                        raise ValueError(f"Reference geometry changed: {name} {column}")
                if column == "total_area" or column.startswith(("total_flux_", "average_flux_", "area_weighted_average_flux_", "mean_model_average_flux_")):
                    value, baseline = float(value), float(expected[column])
                    if not math.isclose(value, baseline, rel_tol=2e-4, abs_tol=2e-6):
                        raise ValueError(f"Reference numerical mismatch: {name} {column}: {value} vs {baseline}")
                    maximum_absolute = max(maximum_absolute, abs(value - baseline))
                    if abs(baseline) > 1e-6:
                        maximum_relative = max(maximum_relative, abs(value - baseline) / abs(baseline))
        checks[name] = {"rows": len(repeated), "maximum_absolute_difference": maximum_absolute,
                        "maximum_relative_difference_nonzero_reference": maximum_relative}
    save_json(STUDY / "provenance/repair_reference_check.json", {"status": "passed", "run_ids": [0, 885],
              "rtol": 2e-4, "atol": 2e-6, "checks": checks})


def assemble(name="03_field_search", parts=("raw/03_field_search", "raw/03_field_search_resume"), study=None):
    """Join nonoverlapping completed rows verbatim; raw attempts remain immutable."""
    study = study or STUDY
    folders = [study / part for part in parts]
    target = study / "assembled" / name
    if target.exists():
        raise ValueError("Assembled output exists; refusing to overwrite")
    config = json.loads((study / "configs" / f"{name}.json").read_text(encoding="utf-8"))
    expected = len(build_cases(config))
    identifiers = [int(row["run_id"]) for folder in folders for row in rows(folder / "case_timings.csv")]
    if sorted(identifiers) != list(range(expected)):
        raise ValueError("Attempts overlap or leave missing case IDs")
    target.mkdir(parents=True)
    hashes = {}
    for filename in ["field_summary.csv", "plant_measurements.csv", "field_organ_summary.csv", "plant_organ_measurements.csv", "case_timings.csv"]:
        with (target / filename).open("wb") as destination:
            header = None
            for folder in folders:
                with (folder / filename).open("rb") as source:
                    current = source.readline()
                    if header is None:
                        header = current
                        destination.write(header)
                    elif current != header:
                        raise ValueError(f"CSV header changed: {filename}")
                    shutil.copyfileobj(source, destination)
                with (folder / filename).open("rb") as source:
                    hashes[(folder / filename).relative_to(study).as_posix()] = hashlib.file_digest(source, "sha256").hexdigest()
    manifest = json.loads((folders[0] / "search_manifest.json").read_text(encoding="utf-8"))
    manifest["config"] = config
    manifest["assembled_from"] = [folder.relative_to(study).as_posix() for folder in folders]
    manifest["source_sha256"] = hashes
    manifest["executed_run_ids"] = list(range(expected))
    if (study / "provenance/repair_reference_check.json").exists():
        manifest["resource_fix_reference_check"] = "provenance/repair_reference_check.json"
    save_json(target / "search_manifest.json", manifest)
    receipts = [json.loads((folder / "completion.json" if (folder / "completion.json").exists()
                           else study / "timings" / f"{folder.name}.json").read_text(encoding="utf-8")) for folder in folders]
    first, last = receipts[0], receipts[-1]
    save_json(target / "completion.json", {"completed_cases": expected, "started_at_utc": first["started_at_utc"],
              "completed_at_utc": last["completed_at_utc"],
              "elapsed_seconds": sum(receipt["elapsed_seconds"] for receipt in receipts),
              "wall_elapsed_seconds": (datetime.fromisoformat(last["completed_at_utc"]) - datetime.fromisoformat(first["started_at_utc"])).total_seconds(),
              "timing_note": "Elapsed sums the recorded attempts/chunks, including cleanup or failures. Wall elapsed includes intervening approval/build waits where applicable. Reference-check time is separate.",
              "include_in_study_analysis": True, "assembled_not_new_measurement": True})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assemble-only", action="store_true")
    parser.add_argument("--study", type=Path, default=STUDY)
    parser.add_argument("--name", default="03_field_search")
    parser.add_argument("--parts", nargs="+", default=["raw/03_field_search", "raw/03_field_search_resume"])
    args = parser.parse_args()
    if args.assemble_only:
        assemble(args.name, args.parts, args.study.resolve())
        return
    first = json.loads((STUDY / "timings/03_field_search.json").read_text(encoding="utf-8"))
    if first["exit_code"] != 1:
        raise ValueError("Expected the recorded failed attempt, not a new or successful search")
    config = json.loads((STUDY / "configs/03_field_search.json").read_text(encoding="utf-8"))
    completed = [int(row["run_id"]) for row in rows(STUDY / "raw/03_field_search/case_timings.csv")]
    if completed != list(range(890)):
        raise ValueError("Original completed IDs differ from the approved 890-case recovery")
    check = copy.deepcopy(config)
    check.update(run_ids=[0, 885], include_in_study_analysis=False, capture_run_ids=[])
    check.pop("save_scene_prefix", None)
    run_search("03_reference_check", check, STUDY)
    check_reference()
    binding = ROOT / "out/install/vs2026-x64/python/PyEvoEngine.cp312-win_amd64.pyd"
    with binding.open("rb") as stream:
        save_json(STUDY / "provenance/repaired_engine.json", {"binding": str(binding),
                  "sha256": hashlib.file_digest(stream, "sha256").hexdigest(),
                  "reference_check": "repair_reference_check.json", "resource_stress_test": "../logs/interop_test_run.log",
                  "partial_initialization_stress_test": "../logs/interop_partial_test_run.log"})
    config.update(run_ids=list(range(890, 990)), include_in_study_analysis=False)
    config["capture_run_ids"] = [run_id for run_id in config.get("capture_run_ids", []) if run_id >= 890]
    run_search("03_field_search_resume", config, STUDY)
    stage("assemble_training", [sys.executable, str(ROOT / "Scripts/resume_soybean_study.py"), "--assemble-only"])
    analyze("analyze_after_resumed_field")
    finish_study()


if __name__ == "__main__":
    main()
