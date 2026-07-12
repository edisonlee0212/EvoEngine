#!/usr/bin/env python3
"""Run and validate the three-capture M11 portable ray-camera benchmark."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SUITE = ROOT / "Scripts" / "raytracer_m11_suite.json"
M6_RUNNER = ROOT / "Scripts" / "run_raytracer_m6.py"
DEFAULT_OUTPUT = ROOT / "out" / "raytracer-m11"


def load_json(path: Path) -> dict[str, object]:
    with path.open("r", encoding="utf-8") as file:
        value = json.load(file)
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object: {path}")
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def acceptance_config(suite: dict[str, object]) -> dict[str, object]:
    config = suite.get("m11_acceptance")
    if not isinstance(config, dict) or config.get("schema") != 1:
        raise ValueError("M11 suite requires m11_acceptance schema 1")
    if suite.get("validation_mode") != "compact" or suite.get("capture_renderers") != ["evo"]:
        raise ValueError("M11 must use the compact fresh-Evo-only capture mode")
    techniques = suite.get("cross_renderer", {}).get("techniques")
    if techniques != ["rtx", "rq", "query-only"]:
        raise ValueError("M11 must contain exactly RTX, RayQuery, and forced query-only lanes")
    if suite.get("repetitions") != 1 or suite.get("samples_per_frame") != 4:
        raise ValueError("M11 must retain one repetition and four samples per frame")
    measure = suite.get("profiles", {}).get("measure", {})
    if measure.get("width") != 1280 or measure.get("height") != 720 or measure.get("candidate_spp") != [512]:
        raise ValueError("M11 must retain the approved 1280x720 512-SPP measure profile")
    return config


def add_gate(
    gates: list[dict[str, object]], name: str, passed: bool, actual: object, expected: object
) -> None:
    gates.append({"name": name, "pass": bool(passed), "actual": actual, "expected": expected})


def comparison_by_name(report: dict[str, object], name: str) -> dict[str, object]:
    for comparison in report.get("image_comparisons", []):
        if comparison.get("name") == name:
            return comparison
    raise RuntimeError(f"M11 report is missing image comparison {name}")


def record_for_row(output_root: Path, row: dict[str, object]) -> tuple[Path, dict[str, object]]:
    output = Path(str(row["output"])).resolve()
    record_path = output.with_suffix(".record.json")
    if output_root.resolve() not in record_path.parents:
        raise RuntimeError(f"M11 record escaped its evidence root: {record_path}")
    return record_path, load_json(record_path)


def load_approved_m6(config: dict[str, object], measured_spp: int) -> dict[str, object]:
    report_path = Path(str(config.get("report", "")))
    if not report_path.is_absolute():
        report_path = ROOT / report_path
    report_path = report_path.resolve()
    expected_hash = str(config.get("report_sha256", ""))
    actual_hash = sha256(report_path)
    if actual_hash != expected_hash:
        raise RuntimeError(f"Approved M6 report hash mismatch: {actual_hash} != {expected_hash}")

    report = load_json(report_path)
    rows = report.get("runs", [])
    rows_by_technique = {str(row.get("technique")): row for row in rows}
    if report.get("type") != "raytracer_m6_compact_report" or set(rows_by_technique) != {
        "rtx",
        "rq",
        "query-only",
    }:
        raise RuntimeError("Approved M6 report is not the pinned three-lane compact baseline")
    if report.get("gpu") != config.get("gpu"):
        raise RuntimeError("Approved M6 report GPU/driver differs from the committed baseline")

    derived_lanes: dict[str, object] = {}
    for technique, expected_lane in config.get("lanes", {}).items():
        row = rows_by_technique[technique]
        throughput = float(row["wall_throughput_msamples_per_second"])
        derived_lane = {
            "wall_throughput_msamples_per_second": throughput,
            "gpu_median_ms": float(row["gpu_median_ms"]),
            "accumulation_wall_seconds": 1280 * 720 * measured_spp / throughput / 1.0e6,
            "queue_submit_median_ms": float(row["cpu_median_ms"]),
        }
        for name, expected_value in expected_lane.items():
            if not math.isclose(float(derived_lane[name]), float(expected_value), rel_tol=0.0, abs_tol=1.0e-9):
                raise RuntimeError(f"Approved M6 {technique} {name} differs from the committed baseline")
        derived_lanes[technique] = derived_lane
    if set(derived_lanes) != {"rtx", "rq", "query-only"}:
        raise RuntimeError("Approved M6 baseline must bind all three techniques")
    return {
        "report": str(report_path),
        "report_sha256": actual_hash,
        "gpu": report["gpu"],
        "lanes": derived_lanes,
    }


def validate_report(suite_path: Path, output_root: Path) -> dict[str, object]:
    suite = load_json(suite_path)
    config = acceptance_config(suite)
    report_path = output_root / "report.json"
    report = load_json(report_path)
    rows = report.get("runs", [])
    if not isinstance(rows, list):
        raise RuntimeError("M11 compact report has no run rows")
    rows_by_technique = {str(row.get("technique")): row for row in rows}
    expected_techniques = {"rtx", "rq", "query-only"}
    if len(rows) != 3 or set(rows_by_technique) != expected_techniques:
        raise RuntimeError("M11 compact report must contain exactly three fresh Evo lanes")

    expected = config["expected"]
    targets = config["targets"]
    baseline = load_approved_m6(config["approved_m6"], int(expected["measured_spp"]))
    gates: list[dict[str, object]] = []
    add_gate(gates, "fresh_capture_count", report.get("run_count") == 3, report.get("run_count"), 3)
    add_gate(
        gates,
        "frozen_reference_is_historical",
        report.get("frozen_reference", {}).get("mode") == "frozen_historical"
        and report.get("frozen_reference", {}).get("binding_speedup_claims_allowed") is False,
        report.get("frozen_reference", {}).get("mode"),
        "frozen_historical",
    )
    add_gate(
        gates,
        "approved_m6_report_integrity",
        baseline["report_sha256"] == config["approved_m6"]["report_sha256"],
        baseline["report_sha256"],
        config["approved_m6"]["report_sha256"],
    )
    add_gate(
        gates,
        "approved_m6_gpu_driver",
        report.get("gpu") == baseline["gpu"],
        report.get("gpu"),
        baseline["gpu"],
    )

    lane_results: dict[str, object] = {}
    for technique in ("rtx", "rq", "query-only"):
        row = rows_by_technique[technique]
        record_path, record = record_for_row(output_root, row)
        summary = record.get("summary", {})
        variant = summary.get("ray_shader_variant", {})
        expected_key = expected["rtx_variant_key"] if technique == "rtx" else expected["rq_variant_key"]
        expected_backend = "ray-tracing-pipeline" if technique == "rtx" else "ray-query-compute"
        throughput = float(row["wall_throughput_msamples_per_second"])
        gpu_median = float(row["gpu_median_ms"])
        queue_median = float(row["cpu_median_ms"])
        wall_seconds = float(summary["accumulation_wall_seconds"])
        baseline_lane = baseline["lanes"][technique]

        add_gate(
            gates,
            f"{technique}_fixed_measure_window",
            summary.get("measured_frames") == expected["measured_frames"]
            and summary.get("measured_spp") == expected["measured_spp"],
            {"measured_frames": summary.get("measured_frames"), "measured_spp": summary.get("measured_spp")},
            {"measured_frames": expected["measured_frames"], "measured_spp": expected["measured_spp"]},
        )
        add_gate(gates, f"{technique}_ser_disabled", summary.get("ser_enabled") is False, summary.get("ser_enabled"), False)
        add_gate(
            gates,
            f"{technique}_active_backend",
            summary.get("active_ray_backend") == expected_backend,
            summary.get("active_ray_backend"),
            expected_backend,
        )
        add_gate(
            gates,
            f"{technique}_timing_sample_count",
            row.get("gpu_sample_count") == expected["measured_frames"]
            and row.get("cpu_sample_count") == expected["measured_frames"],
            {"gpu": row.get("gpu_sample_count"), "cpu": row.get("cpu_sample_count")},
            expected["measured_frames"],
        )
        add_gate(
            gates,
            f"{technique}_fresh_cold_evo_lane",
            row.get("renderer") == "evo" and row.get("cache_state") == "cold",
            {"renderer": row.get("renderer"), "cache_state": row.get("cache_state")},
            {"renderer": "evo", "cache_state": "cold"},
        )
        add_gate(
            gates,
            f"{technique}_specialized_material_variant",
            variant.get("active_mask") == expected["active_feature_mask"]
            and variant.get("active_key") == expected_key
            and variant.get("selection_mode") == "auto"
            and variant.get("fallback_active") is False,
            {
                "active_mask": variant.get("active_mask"),
                "active_key": variant.get("active_key"),
                "selection_mode": variant.get("selection_mode"),
                "fallback_active": variant.get("fallback_active"),
            },
            {"active_mask": expected["active_feature_mask"], "active_key": expected_key},
        )
        add_gate(
            gates,
            f"{technique}_cold_frontend_compile",
            variant.get("cache_source") == "compiled" and variant.get("ready") is True,
            {"cache_source": variant.get("cache_source"), "ready": variant.get("ready")},
            {"cache_source": "compiled", "ready": True},
        )
        add_gate(
            gates,
            f"{technique}_queue_submit_median",
            queue_median <= float(targets["maximum_queue_submit_median_ms"]),
            queue_median,
            {"maximum": targets["maximum_queue_submit_median_ms"]},
        )

        lane_target = targets[technique]
        add_gate(
            gates,
            f"{technique}_wall_throughput",
            throughput >= float(lane_target["minimum_wall_throughput_msamples_per_second"]),
            throughput,
            {"minimum": lane_target["minimum_wall_throughput_msamples_per_second"]},
        )
        if technique in ("rtx", "rq"):
            add_gate(
                gates,
                f"{technique}_gpu_median",
                gpu_median <= float(lane_target["maximum_gpu_median_ms"]),
                gpu_median,
                {"maximum": lane_target["maximum_gpu_median_ms"]},
            )
            add_gate(
                gates,
                f"{technique}_accumulation_wall",
                wall_seconds <= float(lane_target["maximum_accumulation_wall_seconds"]),
                wall_seconds,
                {"maximum": lane_target["maximum_accumulation_wall_seconds"]},
            )

        lane_results[technique] = {
            "record": str(record_path),
            "record_sha256": sha256(record_path),
            "wall_throughput_msamples_per_second": throughput,
            "gpu_median_ms": gpu_median,
            "accumulation_wall_seconds": wall_seconds,
            "queue_submit_median_ms": queue_median,
            "m6_delta_percent": {
                "wall_throughput":
                    (throughput / float(baseline_lane["wall_throughput_msamples_per_second"]) - 1.0) * 100.0,
                "gpu_median": (gpu_median / float(baseline_lane["gpu_median_ms"]) - 1.0) * 100.0,
                "accumulation_wall":
                    (wall_seconds / float(baseline_lane["accumulation_wall_seconds"]) - 1.0) * 100.0,
                "queue_submit_median":
                    (queue_median / float(baseline_lane["queue_submit_median_ms"]) - 1.0) * 100.0,
            },
            "recursion": {
                "pipeline": summary.get("ray_pipeline_max_recursion_depth"),
                "device_limit": summary.get("device_ray_tracing_max_recursion_depth"),
            },
        }

    rtx_summary = load_json(Path(lane_results["rtx"]["record"]))["summary"]
    add_gate(
        gates,
        "rtx_camera_recursion_depth",
        rtx_summary.get("ray_pipeline_max_recursion_depth") == expected["camera_pipeline_recursion_depth"]
        and int(rtx_summary.get("device_ray_tracing_max_recursion_depth", 0))
        >= int(expected["camera_pipeline_recursion_depth"]),
        {
            "pipeline": rtx_summary.get("ray_pipeline_max_recursion_depth"),
            "device_limit": rtx_summary.get("device_ray_tracing_max_recursion_depth"),
        },
        {"pipeline": expected["camera_pipeline_recursion_depth"], "device_limit_minimum": 1},
    )
    query_only_summary = load_json(Path(lane_results["query-only"]["record"]))["summary"]
    add_gate(
        gates,
        "query_only_independent_backend",
        query_only_summary.get("active_ray_backend") == "ray-query-compute"
        and query_only_summary.get("query_only") is True
        and query_only_summary.get("capabilities", {}).get("ray_tracing_pipeline") is False
        and query_only_summary.get("ray_pipeline_max_recursion_depth") is None,
        {
            "active_ray_backend": query_only_summary.get("active_ray_backend"),
            "query_only": query_only_summary.get("query_only"),
            "ray_tracing_capability": query_only_summary.get("capabilities", {}).get("ray_tracing_pipeline"),
            "pipeline_recursion": query_only_summary.get("ray_pipeline_max_recursion_depth"),
        },
        {
            "active_ray_backend": "ray-query-compute",
            "query_only": True,
            "ray_tracing_capability": False,
            "pipeline_recursion": None,
        },
    )

    rtx_rq = comparison_by_name(report, "evo-rtx-vs-rq")["metrics"]
    rq_query_only = comparison_by_name(report, "evo-rq-vs-query-only")["metrics"]
    add_gate(
        gates,
        "rtx_rq_image_parity",
        float(rtx_rq["relative_l2_error"]) <= float(targets["maximum_rtx_rq_relative_l2_error"]),
        rtx_rq["relative_l2_error"],
        {"maximum": targets["maximum_rtx_rq_relative_l2_error"]},
    )
    add_gate(
        gates,
        "rq_query_only_bit_exact",
        rq_query_only.get("exact_match") == bool(targets["require_rq_query_only_exact"]),
        rq_query_only.get("exact_match"),
        targets["require_rq_query_only_exact"],
    )

    result = {
        "schema": 1,
        "type": "raytracer_m11_acceptance_report",
        "pass": all(gate["pass"] for gate in gates),
        "suite": str(suite_path),
        "suite_sha256": sha256(suite_path),
        "source_report": str(report_path),
        "source_report_sha256": sha256(report_path),
        "approved_m6": baseline,
        "capture_accounting": {
            "fresh_precommit": 3,
            "postcommit_delivery": 1,
            "total_milestone_capture_count": 4,
            "reference_executed": False,
        },
        "lanes": lane_results,
        "image_comparisons": {
            "rtx_vs_rq": rtx_rq,
            "rq_vs_query_only": rq_query_only,
        },
        "gates": gates,
        "failed_gates": [gate["name"] for gate in gates if not gate["pass"]],
    }
    write_json(output_root / "m11-report.json", result)
    return result


def run_m6(args: argparse.Namespace, phase: str) -> None:
    command = [
        sys.executable,
        str(M6_RUNNER),
        "--phase",
        phase,
        "--suite-manifest",
        str(args.suite_manifest),
        "--output-dir",
        str(args.output_dir),
        "--editor",
        str(args.editor),
        "--evo-build-dir",
        str(args.evo_build_dir),
    ]
    subprocess.run(command, cwd=ROOT, check=True)


def write_m11_plan(suite_path: Path, output_root: Path) -> dict[str, object]:
    source_plan = load_json(output_root / "plan.json")
    if source_plan.get("total_runs") != 3 or source_plan.get("fresh_benchmark_capture_count") != 3:
        raise RuntimeError("M11 plan must contain exactly three fresh precommit captures")
    plan = {
        "schema": 1,
        "type": "raytracer_m11_plan",
        "suite": str(suite_path),
        "suite_sha256": sha256(suite_path),
        "fresh_precommit_capture_count": 3,
        "postcommit_delivery_capture_count": 1,
        "total_milestone_capture_count": 4,
        "reference_executed": False,
        "source_plan": source_plan,
    }
    write_json(output_root / "m11-plan.json", plan)
    return plan


def run_self_test(suite_path: Path) -> None:
    suite = copy.deepcopy(load_json(suite_path))
    config = acceptance_config(suite)
    expected = config["expected"]
    targets = config["targets"]
    with tempfile.TemporaryDirectory(prefix="raytracer-m11-self-test-") as temporary_directory:
        temporary_root = Path(temporary_directory)
        baseline_path = temporary_root / "m6-report.json"
        baseline_rows = []
        for technique, lane in config["approved_m6"]["lanes"].items():
            baseline_rows.append(
                {
                    "technique": technique,
                    "wall_throughput_msamples_per_second": lane["wall_throughput_msamples_per_second"],
                    "gpu_median_ms": lane["gpu_median_ms"],
                    "cpu_median_ms": lane["queue_submit_median_ms"],
                }
            )
        write_json(
            baseline_path,
            {
                "schema": 1,
                "type": "raytracer_m6_compact_report",
                "gpu": config["approved_m6"]["gpu"],
                "runs": baseline_rows,
            },
        )
        config["approved_m6"]["report"] = str(baseline_path)
        config["approved_m6"]["report_sha256"] = sha256(baseline_path)

        output_root = temporary_root / "m11"
        rows = []
        for technique in ("rtx", "rq", "query-only"):
            lane_target = targets[technique]
            throughput = float(lane_target["minimum_wall_throughput_msamples_per_second"]) + 1.0
            gpu_median = float(lane_target.get("maximum_gpu_median_ms", 51.0)) - 1.0
            wall_seconds = float(lane_target.get("maximum_accumulation_wall_seconds", 9.0)) - 0.1
            output_path = output_root / f"{technique}.hdr"
            summary = {
                "accumulation_wall_seconds": wall_seconds,
                "active_ray_backend": "ray-tracing-pipeline" if technique == "rtx" else "ray-query-compute",
                "capabilities": {"ray_tracing_pipeline": technique != "query-only"},
                "device_ray_tracing_max_recursion_depth": 31 if technique != "query-only" else None,
                "measured_frames": expected["measured_frames"],
                "measured_spp": expected["measured_spp"],
                "query_only": technique == "query-only",
                "ray_pipeline_max_recursion_depth": expected["camera_pipeline_recursion_depth"]
                if technique == "rtx"
                else None,
                "ray_shader_variant": {
                    "active_key": expected["rtx_variant_key"] if technique == "rtx" else expected["rq_variant_key"],
                    "active_mask": expected["active_feature_mask"],
                    "cache_source": "compiled",
                    "fallback_active": False,
                    "ready": True,
                    "selection_mode": "auto",
                },
                "ser_enabled": False,
            }
            write_json(output_path.with_suffix(".record.json"), {"summary": summary})
            rows.append(
                {
                    "cache_state": "cold",
                    "cpu_median_ms": 0.1,
                    "cpu_sample_count": expected["measured_frames"],
                    "gpu_median_ms": gpu_median,
                    "gpu_sample_count": expected["measured_frames"],
                    "output": str(output_path),
                    "renderer": "evo",
                    "technique": technique,
                    "wall_throughput_msamples_per_second": throughput,
                }
            )
        report = {
            "schema": 1,
            "type": "raytracer_m6_compact_report",
            "frozen_reference": {"binding_speedup_claims_allowed": False, "mode": "frozen_historical"},
            "gpu": config["approved_m6"]["gpu"],
            "image_comparisons": [
                {"name": "evo-rtx-vs-rq", "metrics": {"relative_l2_error": 0.005}},
                {"name": "evo-rq-vs-query-only", "metrics": {"exact_match": True}},
            ],
            "run_count": 3,
            "runs": rows,
        }
        write_json(output_root / "report.json", report)
        synthetic_suite_path = temporary_root / "suite.json"
        write_json(synthetic_suite_path, suite)
        passing = validate_report(synthetic_suite_path, output_root)
        if not passing["pass"] or passing["capture_accounting"]["total_milestone_capture_count"] != 4:
            raise AssertionError("Synthetic passing M11 evidence did not pass every gate")

        report["image_comparisons"][1]["metrics"]["exact_match"] = False
        write_json(output_root / "report.json", report)
        failing = validate_report(synthetic_suite_path, output_root)
        if failing["pass"] or "rq_query_only_bit_exact" not in failing["failed_gates"]:
            raise AssertionError("Synthetic M11 parity failure was not rejected")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("self-test", "plan", "measure", "analyze", "all"), default="plan")
    parser.add_argument("--suite-manifest", type=Path, default=DEFAULT_SUITE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--editor", type=Path, default=ROOT / "out" / "install" / "vs2026-x64" / "bin" / "EvoEngineEditor.exe"
    )
    parser.add_argument("--evo-build-dir", type=Path, default=ROOT / "out" / "build" / "vs2026-x64")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.suite_manifest = args.suite_manifest.resolve()
    args.output_dir = args.output_dir.resolve()
    config = acceptance_config(load_json(args.suite_manifest))
    if args.phase == "self-test":
        subprocess.run([sys.executable, str(M6_RUNNER), "--phase", "self-test"], cwd=ROOT, check=True)
        run_self_test(args.suite_manifest)
        print("M11 validator self-test passed")
        return 0
    load_approved_m6(config["approved_m6"], int(config["expected"]["measured_spp"]))
    if args.phase in ("plan", "measure", "analyze", "all"):
        run_m6(args, args.phase)
        plan = write_m11_plan(args.suite_manifest, args.output_dir)
        print(f"M11 plan: {plan['fresh_precommit_capture_count']} fresh captures -> {args.output_dir / 'm11-plan.json'}")
    if args.phase in ("analyze", "all"):
        report = validate_report(args.suite_manifest, args.output_dir)
        print(f"M11 acceptance: {'PASS' if report['pass'] else 'FAIL'} -> {args.output_dir / 'm11-report.json'}")
        return 0 if report["pass"] else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
