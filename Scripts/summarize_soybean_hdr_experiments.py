"""Prepare the retained slides' exact HDR values from validated CSVs, without tracing rays."""
import json
from pathlib import Path

import pandas as pd

from run_soybean_experiments import save_json
from run_soybean_hdr_experiments import SOURCE, STUDY


def summarize(study=STUDY):
    ranked = pd.read_csv(study / "analysis/03_field_search/ranked_combinations.csv")
    edge = pd.read_csv(study / "analysis/edge_control.csv")
    facts = json.loads((study / "analysis/facts.json").read_text())
    assert len(ranked) == 198 and (ranked.repeats == 5).all()
    objectives = ["leaf_mean_irradiance", "leaf_incident_per_plant", "whole_incident_per_plant",
                  "leaf_incident_per_ground_area", "leaf_irradiance_cv"]
    winners = []
    for metric in objectives:
        row = ranked.sort_values(metric, ascending=metric.endswith("cv"), kind="stable").iloc[0]
        winners.append({"metric": metric, "leaf_type": row.asset_name.capitalize(),
                        "row": float(row.row_spacing_inches), "seeding": float(row.column_spacing_inches),
                        "mean": float(row[metric]), "std": float(row[metric + "_std"]),
                        "mean_sd": f"{row[metric]:.4f} ± {row[metric + '_std']:.4f}"})
    outer = {kind: {"min": float(g.outer_to_central_ratio.min()), "max": float(g.outer_to_central_ratio.max()),
                    "range": f"{g.outer_to_central_ratio.min():.2f}–{g.outer_to_central_ratio.max():.2f}×"}
             for kind, g in edge.groupby("asset_name")}
    outer["all_above_one"] = bool((edge.outer_to_central_ratio > 1).all())
    assert edge.central_relative_difference_from_primary.abs().max() < 1e-5
    cases = {"single": facts["01_single_plant"]["cases"], "field": facts["03_field_search"]["cases"],
             "edge": facts["06_edge_control"]["cases"]}
    assert cases == {"single": 64, "field": 990, "edge": 8}
    result = {"cases": cases, "winners": winners, "outer": outer,
              "single": facts["01_single_plant"], "lighting": "G1706994 HDR, Scene mode, intensity 2, rotation 0 degrees",
              "units": "Relative linear RGB luminance scores, not calibrated W/m² or PAR"}
    save_json(study / "analysis/hdr_slide_results.json", result)
    timings = [json.loads(p.read_text()) for p in (study / "timings").glob("*.json")]
    assert all(r.get("exit_code") == 0 for r in timings), "A stage is incomplete or failed"
    completed = [r for r in timings if r.get("exit_code") == 0]
    retained = [r for r in completed if r["stage"] == "01_single_plant" or r["stage"].startswith(("03_field_search_", "06_edge_control_"))]
    experiment_times = pd.read_csv(study / "analysis/experiment_timings.csv")
    durations = {"retained_experiment_worker_seconds": sum(r["elapsed_seconds"] for r in retained),
                 "illumination_api_seconds": float(experiment_times.illumination_api_seconds_sum.sum()),
                 "field_setup_seconds": float(experiment_times.field_setup_seconds_sum.sum()),
                 "hdr_preflight_seconds": sum(r["elapsed_seconds"] for r in completed if r["stage"].startswith("00_hdr_")),
                 "separate_field_capture_seconds": sum(r["elapsed_seconds"] for r in completed if r["stage"].startswith("capture_field_")),
                 "post_install_validation_seconds": sum(r["elapsed_seconds"] for r in completed if r["stage"] == "00_post_install_check"),
                 "new_plant_generation_seconds": 0,
                 "note": "Worker time includes startup/import and in-case scene/image saves. Illumination API time excludes setup and captures, but includes acceleration/probe preparation, tracing and result collection. Preflight and separate captures are listed separately. generation_timings.csv describes the reused historical library, not new generation."}
    save_json(study / "analysis/experiment_totals.json", durations)
    lines = ["## Material Passport", "", "- Origin Skill: experiment-agent", "- Origin Mode: run / validate",
             "- Origin Date: 2026-10-17", "- Verification Status: ANALYZED", "- Version Label: G1706994_HDR", "",
             "# Matched-scene HDR study", "", "## Scope", "",
             "The original 22-slide deck and Skydome results remain unchanged. The copied deck uses the same soil, geometry,",
             "materials, field/subset seeds, spacing, ROI and camera setup. Only the sky changes to G1706994 HDR with",
             "the user-selected intensity multiplier 2. The existing 1,024 broad and 1,024 narrow models were reused.", "",
             "Retained experiments: 64 isolated-plant cases, 990 main-field cases, 8 full-field outer/central cases.",
             "The deleted holdout and field-size sensitivity experiments were not rerun.", "",
             "Fields keep 16 samples per enabled triangle face and 2 bounces. Isolated plants keep 256 samples per face.",
             "The HDR pilot's maximum 16-versus-256-sample metric deviation was 0.469% in the four tested corner cases.", "",
             "## Best tested combinations", "", "| Objective | Leaf type | Row (in) | Seeding (in) | Mean ± SD |",
             "|---|---|---:|---:|---:|"]
    lines += [f"| {w['metric']} | {w['leaf_type']} | {w['row']:g} | {w['seeding']:g} | {w['mean_sd']} |" for w in winners]
    lines += ["", "Means and SDs summarize five field subset seeds. These are in-sample grid winners, not validated global optima.",
              "", "## Isolated plants", "", "| Metric | Broad mean | Narrow mean | Broad minus narrow, 95% bootstrap CI |",
              "|---|---:|---:|---:|"]
    for metric, values in result["single"]["metrics"].items():
        lo, hi = values["cluster_bootstrap_95_interval"]
        lines.append(f"| {metric} | {values['broad_mean']:.6f} | {values['narrow_mean']:.6f} | [{lo:.6f}, {hi:.6f}] |")
    lines += ["", "The paired bootstrap clusters repeated draws by generation seed, using the unchanged original analysis method.",
              "", "## Outer versus central", ""]
    lines += [f"- {kind.capitalize()}: outer/central leaf mean irradiance {outer[kind]['range']} across four spacings."
              for kind in ["broad", "narrow"]]
    lines += ["- Central plant identities and values reconcile with the corresponding main-search cases.", "",
              "## Saved scenes", "",
              "Under Resources/DigitalAgricultureProject/Assets/GeneratedAssets/Scenes:", "",
              "- 20261017_G1706994_HDR_field_0245.evescene: broad, 8 × 20 plants, row 22.5 in, seeding 3 in, subset seed 101.",
              "- 20261017_G1706994_HDR_field_0740.evescene: the matched narrow field.",
              "- 20261017_G1706994_HDR_single_0000.evescene and _0032.evescene: the isolated examples.", "",
              "For OptiX viewing, select Ray Tracer Layer > Environment lighting > Scene. The scene's EnvironmentalLighting",
              "asset stores the HDR source, intensity 2 and rotation 0 degrees. The layer's lighting-mode selector is an application setting.", "",
              "## Timing", "", *[f"- {key}: {value:.1f} s" for key, value in durations.items() if isinstance(value, (int, float))],
              "", durations["note"], "", "## Validation and limitations", "",
              "- All raw organ/plant/field integrals and area-weighted means reconcile through the original analysis checks.",
              "- Selected model IDs, generation seeds, plant positions and organ areas match the original study.",
              "- The single-exposure HDR and multiplier 2 are not an absolute radiometric calibration. No W/m², PAR, absorption or yield claims follow.",
              "- The pilot emitted a nonfatal Vulkan validation error for a global-buffer descriptor range after both leaf types accumulated in one process. OptiX uses a separate geometry path. Formal large-field runs use separate processes per leaf type and seed. The original error remains in the pilot log.",
              "- The sampling check covers four cases, not a proof of convergence for every triangle or grid cell.", "",
              "## Runtime, tests and installation", "",
              "- Headless Python: C:/Users/sdjkl/AppData/Local/Programs/Python/Python312/python.exe.",
              "- Executed module: D:/Sorghum-Vulkan/out/install/vs2026-x64/python/PyEvoEngine.cp312-win_amd64.pyd.",
              "- Installed application: D:/Sorghum-Vulkan/out/install/vs2026-x64/bin/DigitalAgricultureApp.exe. This study used headless bindings, not a native UI smoke test.",
              "- 19 focused Python regression tests passed in the Demeter environment with the project-local temporary directory. The first test command used a runtime without Matplotlib and a restricted temporary directory; the corrected command passed all tests.",
              "- Install command: `cmake --install out/build/vs2026-x64 --config RelWithDebInfo` (exit 0). No C++ source changes were made for this HDR rerun.",
              "- Installation after all retained experiments replaced the installed Python module with the build-tree module. Both SHA-256 values are preserved in provenance. Two separate fixed HDR cases were then checked: all six organ rows matched the pre-install reference exactly (maximum relative error 0). No retained data was replaced.", "",
              "## Statistical interpretation audit (11/11 checks)", "",
              "1. Simpson's paradox: separate shape/spacing summaries retain the full grid rather than relying only on a pooled mean.",
              "2. Ecological inference: conclusions describe simulated field/plant light scores, not biological yield.",
              "3. Selection bias: the same fixed library subsets are reused. One library and one sky limit generalization.",
              "4. Collider bias: no outcome-conditioned model filtering or additional covariate adjustment.",
              "5. Base rates: no diagnostic classification probabilities are reported.",
              "6. Regression to the mean: no pre/post intervention claim. Grid maxima remain exploratory selections.",
              "7. Survivorship: completion checks require all 1,062 retained cases, without dropping failures.",
              "8. Multiple searches: all 198 shape/spacing cells and all objectives are exported. No multiple-testing significance claim.",
              "9. Researcher choices: original configs, subset seeds, metric definitions and bootstrap settings remain fixed.",
              "10. Causal scope: differences describe this simulator and controlled sky replacement, not established real-world effects.",
              "11. Reverse causality: light inputs are configured before tracing. No observational prediction claim.", ""]
    (study / "run_report.md").write_text("\n".join(lines), encoding="utf-8")
    return result


if __name__ == "__main__":
    print(json.dumps(summarize(), indent=2))
