"""Verify HDR routing/scaling, unchanged plant geometry and the separate study's CSVs."""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from analyze_soybean_experiments import analyze_experiment
from run_soybean_hdr_experiments import SOURCE, STUDY
from run_soybean_experiments import save_json


def smoke(study):
    a = pd.read_csv(study / "raw/00_hdr_smoke_intensity1/plant_organ_measurements.csv")
    b = pd.read_csv(study / "raw/00_hdr_smoke_intensity2/plant_organ_measurements.csv")
    keys = ["asset_name", "run_id", "model_index", "organ"]
    joined = a.merge(b, on=keys, suffixes=("_one", "_two"), validate="one_to_one")
    assert len(joined) == len(a) == len(b) == 6
    np.testing.assert_array_equal(joined.total_area_one, joined.total_area_two)
    errors = []
    for name in [c for c in a if c.startswith(("average_flux_", "total_flux_"))]:
        expected, actual = 2 * joined[name + "_one"], joined[name + "_two"]
        np.testing.assert_allclose(actual, expected, rtol=1e-5, atol=1e-8)
        errors.append(float((abs(actual - expected) / np.maximum(abs(expected), 1e-9)).max()))
    report = {"status": "passed", "organ_rows": len(joined), "intensity2_vs_intensity1_max_relative_error": max(errors),
              "geometry_area": "identical", "hdr_multiplier": 2, "separate_sun": False}
    save_json(study / "provenance/hdr_smoke_validation.json", report)
    return report


def pilot(study):
    runs, _, _, _ = analyze_experiment(study / "raw/00_hdr_pilot", study / "provenance/pilot_analysis")
    keys = ["asset_name", "row_spacing_inches", "column_spacing_inches", "seed"]
    joined = runs[runs.samples == 16].merge(runs[runs.samples == 256], on=keys,
                                          suffixes=("_16", "_256"), validate="one_to_one")
    assert len(joined) == 4
    metrics = ["leaf_mean_irradiance", "leaf_incident_per_plant", "whole_incident_per_plant"]
    errors = {}
    for name in metrics:
        joined[name + "_relative_error"] = joined[name + "_16"] / joined[name + "_256"] - 1
        errors[name] = float(joined[name + "_relative_error"].abs().max())
    joined.to_csv(study / "provenance/hdr_sampling_check.csv", index=False)
    report = {"maximum_relative_errors": errors, "original_16_samples_retained": True,
              "reference_samples": 256, "passed_one_percent_check": max(errors.values()) <= .01,
              "scope": "two leaf types, dense and sparse spacing corners, original seed 101"}
    save_json(study / "provenance/hdr_sampling_check.json", report)
    return report


def verify_geometry(study, name):
    def data_root(root):
        assembled = root / "assembled" / name
        return assembled if assembled.exists() else root / "raw" / name
    actual_root, expected_root = data_root(study), data_root(SOURCE)
    counts = {}
    for filename, keys, values in [
        ("plant_measurements.csv", ["run_id", "model_index"], ["asset_index", "source_obj", "generation_seed", "row", "column", "total_area"]),
        ("plant_organ_measurements.csv", ["run_id", "model_index", "organ"], ["total_area"]),
    ]:
        actual = pd.read_csv(actual_root / filename).set_index(keys).sort_index()
        expected = pd.read_csv(expected_root / filename).set_index(keys).sort_index()
        pd.testing.assert_index_equal(actual.index, expected.index)
        for key in values:
            if key == "total_area":
                np.testing.assert_allclose(actual[key], expected[key], rtol=1e-6, atol=1e-8)
            else:
                np.testing.assert_array_equal(actual[key], expected[key])
        counts[filename] = len(actual)
    summary = pd.read_csv(actual_root / "field_summary.csv")
    assert (summary.environment_lighting_type == "hdr").all()
    assert (summary.skylight_intensity == 2).all() and (summary.sun_intensity == 0).all()
    assert summary.hdr_sha256.nunique() == 1
    return {"status": "passed", "geometry_and_selected_models": "match original Skydome study", **counts}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=["smoke", "pilot", "final"])
    parser.add_argument("--study", type=Path, default=STUDY)
    args = parser.parse_args()
    if args.phase == "smoke":
        report = smoke(args.study)
    elif args.phase == "pilot":
        report = pilot(args.study)
    else:
        report = {name: verify_geometry(args.study, name) for name in ["01_single_plant", "03_field_search", "06_edge_control"]}
        save_json(args.study / "provenance/matched_scene_validation.json", report)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
