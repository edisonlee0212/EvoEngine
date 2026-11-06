#!/usr/bin/env python3
"""Validate organ CSVs and produce the exact metrics used by the notebook and slides."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

LUMA = np.array([0.2126, 0.7152, 0.0722])
METRICS = ["leaf_mean_irradiance", "leaf_incident_per_plant", "whole_incident_per_plant",
           "leaf_area_per_plant_m2", "whole_surface_area_per_plant_m2",
           "leaf_incident_per_ground_area", "whole_incident_per_ground_area", "leaf_irradiance_cv"]
KEYS = ["asset_name", "rows", "columns", "roi", "row_spacing_inches", "column_spacing_inches", "samples", "bounces"]


def scalar(frame, prefix):
    return frame[[f"{prefix}_{channel}" for channel in "rgb"]].to_numpy() @ LUMA


def assert_close(actual, expected, label):
    if not np.allclose(actual, expected, rtol=2e-4, atol=2e-6):
        raise ValueError(f"Reconciliation failed: {label}")


def paired_interval(values, weights=None):
    """A descriptive bootstrap over paired runs, not over correlated plants within a field."""
    values = np.asarray(values, dtype=float)
    if len(values) < 2:
        return [None, None]
    rng = np.random.default_rng(44123)
    weights = np.ones(len(values)) if weights is None else np.asarray(weights, dtype=float)
    indices = rng.integers(0, len(values), (5000, len(values)))
    means = (values[indices] * weights[indices]).sum(axis=1) / weights[indices].sum(axis=1)
    return np.quantile(means, [0.025, 0.975]).tolist()


def analyze_experiment(source, destination):
    source, destination = Path(source), Path(destination)
    manifest = json.loads((source / "search_manifest.json").read_text(encoding="utf-8"))
    if manifest.get("organ_reporting", {}).get("schema_version") != 2:
        raise ValueError("This analysis requires incident-total schema 2; older face-mean totals are not comparable.")
    summary = pd.read_csv(source / "field_summary.csv")
    completion = source / "completion.json"
    if completion.exists() and json.loads(completion.read_text(encoding="utf-8"))["completed_cases"] != len(summary):
        raise ValueError("Completion record and CSV run count disagree")
    plants = pd.read_csv(source / "plant_measurements.csv")
    organs = pd.read_csv(source / "plant_organ_measurements.csv")
    field_organs = pd.read_csv(source / "field_organ_summary.csv")
    if summary["run_id"].duplicated().any() or plants.duplicated(["run_id", "model_index"]).any():
        raise ValueError("Duplicate run or plant identifiers")
    counts = plants.groupby("run_id").size().reindex(summary.run_id)
    assert_close(counts, summary.plant_count, "ROI plant counts")
    assert_close(summary.plant_count, summary.roi_rows * summary.roi_columns, "ROI dimensions")
    if not ((plants.row >= plants.roi_row_start) & (plants.row < plants.roi_row_start + plants.roi_rows)
            & (plants.column >= plants.roi_column_start)
            & (plants.column < plants.roi_column_start + plants.roi_columns)).all():
        raise ValueError("A measured plant lies outside the ROI")
    reuse_keys = ["asset_name", "seed", "rows", "columns", "roi", "model_index"]
    if (plants.groupby(reuse_keys).generation_seed.nunique() > 1).any():
        raise ValueError("The selected plant changed across spacing or sampling settings")
    totals = ["total_area", "total_flux_r", "total_flux_g", "total_flux_b"]
    assert_close(plants.groupby("run_id")[totals].sum().reindex(summary.run_id), summary[totals], "plants to field")
    organ_plant = organs.groupby(["run_id", "model_index"])[totals].sum()
    assert_close(organ_plant, plants.set_index(["run_id", "model_index"])[totals].reindex(organ_plant.index), "organs to plant")
    organ_field = organs.groupby(["run_id", "organ"])[totals].sum()
    assert_close(organ_field, field_organs.set_index(["run_id", "organ"])[totals].reindex(organ_field.index), "organs to field")
    for name, faces in [("leaf", 2), ("stem", 1), ("petiole", 1)]:
        selected = organs[organs.organ == name]
        assert_close(scalar(selected, "total_flux"), selected.total_area * scalar(selected, "average_flux") * faces,
                     f"{name} incident integral versus face mean")

    # Leaf scores come exclusively from the leaf group. Whole-plant area includes stem and petiole.
    leaf = organs[organs.organ == "leaf"].copy()
    leaf["leaf_mean_irradiance"] = scalar(leaf, "average_flux")
    leaf["leaf_incident"] = scalar(leaf, "total_flux")
    leaf = leaf.rename(columns={"total_area": "leaf_area_m2"})
    measured = plants.merge(leaf[["run_id", "model_index", "leaf_area_m2", "leaf_mean_irradiance", "leaf_incident"]],
                            on=["run_id", "model_index"], validate="one_to_one")
    if len(measured) != len(plants):
        raise ValueError("Missing leaf organ measurements")
    measured["whole_incident"] = scalar(measured, "total_flux")
    if not (measured.leaf_area_m2 > 0).all() or not np.isfinite(measured[["leaf_mean_irradiance", "leaf_incident", "whole_incident"]]).all().all():
        raise ValueError("Empty leaf geometry or non-finite illumination")
    measured["leaf_weighted_mean"] = measured.leaf_mean_irradiance * measured.leaf_area_m2
    per_run = measured.groupby("run_id").agg(
        leaf_mean_numerator=("leaf_weighted_mean", "sum"), leaf_area=("leaf_area_m2", "sum"),
        leaf_incident=("leaf_incident", "sum"), whole_incident=("whole_incident", "sum"),
        leaf_area_per_plant_m2=("leaf_area_m2", "mean"), whole_surface_area_per_plant_m2=("total_area", "mean"),
        leaf_incident_per_plant=("leaf_incident", "mean"), whole_incident_per_plant=("whole_incident", "mean"),
        leaf_irradiance_std=("leaf_mean_irradiance", "std"), leaf_irradiance_unweighted=("leaf_mean_irradiance", "mean"),
    ).reset_index()
    per_run["leaf_mean_irradiance"] = per_run.leaf_mean_numerator / per_run.leaf_area
    per_run["leaf_irradiance_cv"] = per_run.leaf_irradiance_std / per_run.leaf_irradiance_unweighted
    runs = summary.merge(per_run, on="run_id", validate="one_to_one")
    assert_close(runs.measurement_ground_area, runs.plant_count * runs.row_spacing * runs.column_spacing, "ROI ground area")
    isolated = (runs.rows == 1) & (runs["columns"] == 1)
    runs["leaf_incident_per_ground_area"] = (runs.leaf_incident / runs.measurement_ground_area).mask(isolated)
    runs["whole_incident_per_ground_area"] = (runs.whole_incident / runs.measurement_ground_area).mask(isolated)
    ranked = runs.groupby(KEYS)[METRICS].agg(["mean", "std", "count"]).reset_index()
    ranked.columns = [name if not statistic or statistic == "mean" else f"{name}_{statistic}"
                      for name, statistic in ranked.columns]
    ranked = ranked.rename(columns={"leaf_mean_irradiance_count": "repeats"})
    ranked = ranked.drop(columns=[name for name in ranked if name.endswith("_count")])

    # Match broad/narrow by field subset seed and every controlled setting.
    pair_keys = [name for name in KEYS if name != "asset_name"] + ["seed"]
    broad = runs[runs.asset_name == "broad"]
    narrow = runs[runs.asset_name == "narrow"]
    paired = broad.merge(narrow, on=pair_keys, suffixes=("_broad", "_narrow"), validate="one_to_one")
    if len(paired) * 2 != len(runs):
        raise ValueError("Unmatched broad/narrow runs")
    plant_pairs = measured[measured.asset_name == "broad"].merge(
        measured[measured.asset_name == "narrow"], on=pair_keys + ["model_index"],
        suffixes=("_broad", "_narrow"), validate="one_to_one")
    if not (plant_pairs.generation_seed_broad == plant_pairs.generation_seed_narrow).all():
        raise ValueError("Broad/narrow plant generation seeds do not match")
    if isolated.all():
        paired = paired.merge(plant_pairs[pair_keys + ["generation_seed_broad"]], on=pair_keys, validate="one_to_one")
    paired = paired.copy()
    for metric in METRICS:
        paired[f"{metric}_delta"] = paired[f"{metric}_broad"] - paired[f"{metric}_narrow"]
        paired[f"{metric}_ratio"] = paired[f"{metric}_broad"] / paired[f"{metric}_narrow"]

    destination.mkdir(parents=True, exist_ok=True)
    runs.to_csv(destination / "run_metrics.csv", index=False)
    measured.to_csv(destination / "plant_metrics.csv", index=False)
    ranked.to_csv(destination / "ranked_combinations.csv", index=False)
    paired.to_csv(destination / "paired_comparisons.csv", index=False)
    pair_summary = []
    for key, group in paired.groupby(pair_keys[:-1]):
        for metric in METRICS:
            values = group[f"{metric}_delta"].dropna()
            if values.empty:
                continue
            cluster_values = (group.groupby("generation_seed_broad")[f"{metric}_delta"].mean().dropna()
                              if "generation_seed_broad" in group else values)
            weights = (group.groupby("generation_seed_broad")[f"{metric}_delta"].count().reindex(cluster_values.index)
                       if "generation_seed_broad" in group else None)
            interval = paired_interval(cluster_values, weights)
            pair_summary.append({**dict(zip(pair_keys[:-1], key)), "metric": metric, "paired_runs": len(values),
                                 "independent_clusters": len(cluster_values),
                                 "broad_minus_narrow": values.mean(), "mean_broad_narrow_ratio": group[f"{metric}_ratio"].mean(),
                                 "delta_interval_low": interval[0], "delta_interval_high": interval[1]})
    pd.DataFrame(pair_summary).to_csv(destination / "paired_summary.csv", index=False)
    for metric in METRICS[:3] + ["leaf_incident_per_ground_area"]:
        ranked.sort_values(metric, ascending=False).to_csv(destination / f"ranking_{metric}.csv", index=False)
    facts = {"cases": len(runs), "measured_plants": len(measured), "paired_runs": len(paired),
             "unique_generation_seeds": int(measured.generation_seed.nunique()),
             "reconciliation": "passed", "quantity_schema": 2,
             "units": "Relative linear RGB projected to luminance. No W, PAR, absorption, yield, or time integration.",
             "isolated_ground_scores": "not applicable", "metrics": {}}
    for metric in METRICS:
        if not runs[metric].notna().any():
            continue
        clusters = paired.groupby("generation_seed_broad" if isolated.all() else "seed")[f"{metric}_delta"]
        facts["metrics"][metric] = {"broad_mean": float(broad[metric].mean()), "narrow_mean": float(narrow[metric].mean()),
            "broad_minus_narrow_mean": float(paired[f"{metric}_delta"].mean()),
            "cluster_bootstrap_95_interval": paired_interval(clusters.mean().dropna(), clusters.count())}
    facts["uncertainty_unit"] = "generation seed" if isolated.all() else "field subset seed, averaged over tested spacings"
    condition_means = ranked[ranked.asset_name == "broad"].merge(
        ranked[ranked.asset_name == "narrow"], on=[key for key in KEYS if key != "asset_name"],
        suffixes=("_broad", "_narrow"), validate="one_to_one")
    comparisons = []
    for metric in METRICS:
        selected = condition_means.dropna(subset=[metric + "_broad", metric + "_narrow"])
        if selected.empty:
            continue
        b, n = selected[metric + "_broad"], selected[metric + "_narrow"]
        ratios = (b / n).replace([np.inf, -np.inf], np.nan).dropna()
        comparisons.append({"metric": metric, "conditions": len(selected), "broad_mean_higher": int((b > n).sum()),
                            "narrow_mean_higher": int((n > b).sum()), "equal_means": int((b == n).sum()),
                            "ratio_min": float(ratios.min()) if len(ratios) else None,
                            "ratio_max": float(ratios.max()) if len(ratios) else None})
    facts["condition_comparisons"] = comparisons
    pd.DataFrame(comparisons).to_csv(destination / "shape_comparison.csv", index=False)
    (destination / "facts.json").write_text(json.dumps(facts, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return runs, ranked, paired, facts


def plot_experiment(runs, ranked, paired, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    destination = Path(destination)
    metrics = ["leaf_mean_irradiance", "leaf_incident_per_plant", "whole_incident_per_plant", "leaf_area_per_plant_m2"]
    labels = ["Leaf face-mean irradiance", "Leaf incident light / plant", "Whole-plant incident light / plant", "Leaf mesh area / plant (m²)"]
    if (runs.rows == 1).all() and (runs["columns"] == 1).all():
        fig, axes = plt.subplots(1, 4, figsize=(17, 4.5), constrained_layout=True)
        for ax, metric, label in zip(axes, metrics, labels):
            for _, row in paired.iterrows():
                ax.plot([0, 1], [row[f"{metric}_broad"], row[f"{metric}_narrow"]], alpha=.25, color="#47655d")
            ax.scatter(np.zeros(len(paired)), paired[f"{metric}_broad"], s=14, label="Broad", color="#28664f")
            ax.scatter(np.ones(len(paired)), paired[f"{metric}_narrow"], s=14, label="Narrow", color="#ad6b37")
            ax.set(xticks=[0, 1], xticklabels=["Broad", "Narrow"], title=label)
        fig.savefig(destination / "single_plant_pairs.png", dpi=180)
        plt.close(fig)
        return
    # Separate panels keep broad/narrow, ROI sizes, and sample settings from being silently pooled.
    plot_keys = ["asset_name", "rows", "columns", "roi", "samples"]
    for key, group in ranked.groupby(plot_keys):
        fig, axes = plt.subplots(1, 3, figsize=(16, 5), constrained_layout=True)
        for ax, metric in zip(axes, ["leaf_mean_irradiance", "leaf_incident_per_plant", "leaf_incident_per_ground_area"]):
            table = group.pivot(index="row_spacing_inches", columns="column_spacing_inches", values=metric).sort_index(ascending=False)
            comparison = ranked[(ranked.rows == key[1]) & (ranked["columns"] == key[2])
                                & (ranked.roi == key[3]) & (ranked.samples == key[4])]
            image = ax.imshow(table, aspect="auto", cmap="viridis", vmin=comparison[metric].min(), vmax=comparison[metric].max())
            ax.set(title=metric.replace("_", " "), xlabel="Plant spacing (inch)", ylabel="Row spacing (inch)")
            ax.set_xticks(range(len(table.columns)), [f"{v:g}" for v in table.columns])
            ax.set_yticks(range(len(table.index)), [f"{v:g}" for v in table.index])
            fig.colorbar(image, ax=ax)
        incomplete_grid = len(group) < group.row_spacing_inches.nunique() * group.column_spacing_inches.nunique()
        fig.suptitle(f"{key[0]}   {key[1]}×{key[2]}   ROI {key[3]:g}   samples {key[4]}"
                     + ("\nBlank cells: untested combinations" if incomplete_grid else ""))
        fig.savefig(destination / ("heatmaps_" + "_".join(map(str, key)) + ".png"), dpi=160)
        plt.close(fig)
    if ranked.asset_name.nunique() == 2 and len(ranked[["rows", "columns", "roi", "samples", "bounces"]].drop_duplicates()) == 1:
        from matplotlib.colors import TwoSlopeNorm
        fig, axes = plt.subplots(1, 3, figsize=(16, 5), constrained_layout=True)
        if len(ranked) < 2 * ranked.row_spacing_inches.nunique() * ranked.column_spacing_inches.nunique():
            fig.suptitle("Broad / narrow ratios of condition means. Blank cells: untested combinations")
        for ax, metric in zip(axes, ["leaf_mean_irradiance", "leaf_incident_per_plant", "leaf_incident_per_ground_area"]):
            values = ranked.pivot(index=["row_spacing_inches", "column_spacing_inches"], columns="asset_name", values=metric)
            ratio = (values.broad / values.narrow).unstack().sort_index(ascending=False)
            norm = TwoSlopeNorm(vmin=min(.99, ratio.min().min()), vcenter=1, vmax=max(1.01, ratio.max().max()))
            image = ax.imshow(ratio, aspect="auto", cmap="coolwarm", norm=norm)
            ax.set(title=metric.replace("_", " "), xlabel="Plant spacing (inch)", ylabel="Row spacing (inch)")
            ax.set_xticks(range(len(ratio.columns)), [f"{v:g}" for v in ratio.columns])
            ax.set_yticks(range(len(ratio.index)), [f"{v:g}" for v in ratio.index])
            fig.colorbar(image, ax=ax, label="Broad / narrow ratio of means")
        fig.savefig(destination / "broad_narrow_ratios.png", dpi=160)
        plt.close(fig)


def analyze_study(study, plots=True):
    study = Path(study)
    all_facts = {}
    deck_data = {"experiments": {}}
    experiment_times = []
    sources = list((study / "raw").iterdir())
    if (study / "assembled").exists():
        sources.extend((study / "assembled").iterdir())
    for source in sorted(sources, key=lambda path: path.name):
        if not (source / "completion.json").exists():
            continue
        completion = json.loads((source / "completion.json").read_text(encoding="utf-8"))
        if not completion.get("include_in_study_analysis", True):
            continue
        if source.name in all_facts:
            raise ValueError(f"Duplicate logical dataset: {source.name}")
        destination = study / "analysis" / source.name
        runs, ranked, paired, facts = analyze_experiment(source, destination)
        if plots:
            plot_experiment(runs, ranked, paired, destination)
        all_facts[source.name] = facts
        experiment_times.append({"experiment": source.name, "cases": len(runs), "elapsed_seconds": completion["elapsed_seconds"],
                                 "field_setup_seconds_sum": runs.setup_seconds.sum(),
                                 "illumination_api_seconds_sum": runs.sampling_seconds.sum(),
                                 "illumination_api_seconds_median": runs.sampling_seconds.median(),
                                 "illumination_api_seconds_p95": runs.sampling_seconds.quantile(.95),
                                 "completed_at_utc": completion["completed_at_utc"]})
        deck_data["experiments"][source.name] = {"facts": facts, "ranked": json.loads(ranked.to_json(orient="records"))}
    (study / "analysis").mkdir(exist_ok=True)
    pd.DataFrame(experiment_times).to_csv(study / "analysis/experiment_timings.csv", index=False)
    deck_data["experiment_timings"] = experiment_times
    library_split = study / "library_split.json"
    if library_split.exists():
        library_root = Path(json.loads(library_split.read_text(encoding="utf-8"))["root"])
        generation_times = []
        for kind in ["Broad", "Narrow"]:
            manifest = json.loads((library_root / "Library" / kind / "manifest.json").read_text(encoding="utf-8"))
            generation_times.extend({"asset_name": kind.lower(), "generation_seed": entry["seed"],
                                     "generation_seconds": entry["generation_seconds"], "obj_bytes": entry["obj_bytes"]}
                                    for entry in manifest["models"])
        pd.DataFrame(generation_times).to_csv(study / "analysis/generation_timings.csv", index=False)
    (study / "analysis/facts.json").write_text(json.dumps(all_facts, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    timings = [json.loads(path.read_text(encoding="utf-8")) for path in sorted((study / "timings").glob("*.json"))]
    pd.DataFrame(timings).to_csv(study / "analysis/stage_timings.csv", index=False)
    deck_data["timings"] = timings
    repair_check = study / "provenance/repair_reference_check.json"
    if repair_check.exists():
        deck_data["repair_reference"] = json.loads(repair_check.read_text(encoding="utf-8"))
    for subset in ["geometry_representative", "geometry_library"]:
        geometry_path = study / "analysis" / subset / "plant_geometry.csv"
        if not geometry_path.exists():
            continue
        geometry = pd.read_csv(geometry_path)
        geometry_columns = [column for column in geometry.select_dtypes("number") if column not in {"generation_seed", "seed"}]
        summary = geometry.groupby("asset_name")[geometry_columns].agg(["mean", "std", "min", "max"])
        summary.columns = [f"{metric}_{statistic}" for metric, statistic in summary.columns]
        summary = summary.reset_index()
        summary.to_csv(geometry_path.parent / "geometry_summary.csv", index=False)
        if subset == "geometry_representative":
            deck_data["representative_geometry"] = json.loads(geometry.to_json(orient="records"))
        else:
            deck_data["library_geometry_stats"] = json.loads(summary.to_json(orient="records"))
    convergence = []
    reference_path = study / "analysis/02_convergence/run_metrics.csv"
    if reference_path.exists():
        convergence_runs = pd.read_csv(reference_path)
        reference = convergence_runs[convergence_runs.samples == 256]
        match = ["asset_name", "rows", "columns", "roi", "row_spacing_inches", "column_spacing_inches", "seed"]
        for samples in [16, 64]:
            candidate = convergence_runs[convergence_runs.samples == samples]
            joined = candidate.merge(reference, on=match, suffixes=("", "_reference"), validate="one_to_one")
            for _, row in joined.iterrows():
                for metric in METRICS[:3]:
                    convergence.append({**{key: row[key] for key in match}, "samples": samples, "metric": metric,
                                        "relative_difference_from_256": row[metric] / row[f"{metric}_reference"] - 1})
        pd.DataFrame(convergence).to_csv(study / "analysis/sample_convergence.csv", index=False)
        deck_data["convergence"] = convergence
    baseline_path = study / "analysis/05_buffer_baseline/run_metrics.csv"
    sensitivity_path = study / "analysis/05_buffer_sensitivity/run_metrics.csv"
    if baseline_path.exists() and sensitivity_path.exists():
        baseline = pd.read_csv(baseline_path)
        sensitivity = pd.read_csv(sensitivity_path)
        joined = sensitivity.merge(baseline, on=["asset_name", "row_spacing_inches", "column_spacing_inches", "seed", "samples"],
                                   suffixes=("_large", "_baseline"), validate="one_to_one")
        for metric in METRICS[:3]:
            joined[f"{metric}_relative_change"] = joined[f"{metric}_large"] / joined[f"{metric}_baseline"] - 1
        joined.to_csv(study / "analysis/buffer_sensitivity.csv", index=False)
        changes = [f"{metric}_relative_change" for metric in METRICS[:3]]
        grouped = joined.groupby(["asset_name", "row_spacing_inches", "column_spacing_inches"])[changes].mean().reset_index()
        deck_data["buffer_sensitivity"] = json.loads(grouped.to_json(orient="records"))
    edge_path = study / "analysis/06_edge_control/plant_metrics.csv"
    if edge_path.exists():
        edge_plants = pd.read_csv(edge_path)
        primary_plants = pd.read_csv(study / "analysis/03_field_search/plant_metrics.csv")
        comparisons = []
        keys = ["asset_name", "row_spacing_inches", "column_spacing_inches", "seed", "samples"]
        for key, group in edge_plants.groupby(keys):
            central_mask = group.row.between(2, 5) & group.column.between(5, 14)
            centre, outer = group[central_mask], group[~central_mask]
            reference = primary_plants
            for name, value in zip(keys, key):
                reference = reference[reference[name] == value]
            centre, reference = centre.sort_values("model_index"), reference.sort_values("model_index")
            if len(centre) != 40 or len(outer) != 120 or not np.array_equal(centre.generation_seed, reference.generation_seed):
                raise ValueError("ROI control did not retain the same central 40 plants")
            def mean_irradiance(frame):
                return (frame.leaf_mean_irradiance * frame.leaf_area_m2).sum() / frame.leaf_area_m2.sum()
            central_mean, outer_mean, reference_mean = map(mean_irradiance, [centre, outer, reference])
            comparisons.append({**dict(zip(keys, key)), "central_leaf_mean": central_mean, "outer_leaf_mean": outer_mean,
                                "outer_to_central_ratio": outer_mean / central_mean,
                                "central_relative_difference_from_primary": central_mean / reference_mean - 1,
                                "matched_central_plants": 40})
        frame = pd.DataFrame(comparisons)
        frame.to_csv(study / "analysis/edge_control.csv", index=False)
        deck_data["edge_control"] = json.loads(frame.to_json(orient="records"))
    if "03_field_search" in deck_data["experiments"] and "04_holdout" in deck_data["experiments"]:
        training = pd.DataFrame(deck_data["experiments"]["03_field_search"]["ranked"])
        validation = pd.DataFrame(deck_data["experiments"]["04_holdout"]["ranked"])
        checks = []
        for metric in ["leaf_mean_irradiance", "leaf_incident_per_plant", "leaf_incident_per_ground_area"]:
            for kind in ["broad", "narrow"]:
                selected = training[training.asset_name == kind].sort_values(metric, ascending=False).iloc[0]
                candidates = validation[validation.asset_name == kind].sort_values(metric, ascending=False)
                match = candidates[(candidates.row_spacing_inches == selected.row_spacing_inches)
                                   & (candidates.column_spacing_inches == selected.column_spacing_inches)]
                if len(match) != 1:
                    raise ValueError("The training winner was not evaluated exactly once in the holdout rankings")
                observed, best = match.iloc[0], candidates.iloc[0]
                runner_up = candidates.iloc[1] if len(candidates) > 1 else None
                checks.append({"asset_name": kind, "metric": metric,
                               "selected_row_inches": selected.row_spacing_inches,
                               "selected_plant_inches": selected.column_spacing_inches,
                               "holdout_mean": observed[metric], "holdout_std": observed[metric + "_std"],
                               "holdout_rank_among_candidates": int((candidates[metric] > observed[metric]).sum()) + 1,
                               "candidate_count": len(candidates), "best_holdout_row_inches": best.row_spacing_inches,
                               "best_holdout_plant_inches": best.column_spacing_inches,
                               "best_holdout_mean": best[metric],
                               "runner_up_holdout_mean": None if runner_up is None else runner_up[metric],
                               "best_to_runner_up_relative_gap": None if runner_up is None else (best[metric] - runner_up[metric]) / best[metric],
                               "relative_gap_to_best_candidate": (best[metric] - observed[metric]) / best[metric]})
        frame = pd.DataFrame(checks)
        frame.to_csv(study / "analysis/holdout_validation.csv", index=False)
        deck_data["holdout_validation"] = json.loads(frame.to_json(orient="records"))
    (study / "analysis/presentation_data.json").write_text(json.dumps(deck_data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return all_facts


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", type=Path, required=True)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    print(json.dumps(analyze_study(args.study, not args.no_plots), indent=2))
