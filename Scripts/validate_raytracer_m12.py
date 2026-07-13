#!/usr/bin/env python3
"""Run and validate the three-capture M12 static-BLAS acceptance slice."""

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
DEFAULT_OUTPUT = ROOT / "out" / "raytracer-m12"
BUILD_HINT_BYTES = 512 * 1024 * 1024
MAX_BLAS_BUILD_GPU_MS = 425.0
MAX_COMPACTION_RATIO = 0.75
MAX_FINAL_DEVICE_LOCAL_BYTES = 6_000_000_000


def load_json(path: Path) -> dict[str, object]:
    with path.open("r", encoding="utf-8") as file:
        value = json.load(file)
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object: {path}")
    return value


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def suite_binding(suite_path: Path) -> dict[str, object]:
    suite = load_json(suite_path)
    config = suite.get("m11_acceptance")
    if not isinstance(config, dict) or config.get("schema") != 1:
        raise ValueError("M12 requires the schema-1 approved M11/M6 suite binding")
    approved = config.get("approved_m6")
    if not isinstance(approved, dict):
        raise ValueError("M12 suite is missing approved M6 provenance")
    report_path = Path(str(approved.get("report", "")))
    if not report_path.is_absolute():
        report_path = ROOT / report_path
    report_path = report_path.resolve()
    expected_hash = str(approved.get("report_sha256", ""))
    actual_hash = sha256(report_path)
    if actual_hash != expected_hash:
        raise RuntimeError(f"Approved M6 report hash mismatch: {actual_hash} != {expected_hash}")
    report = load_json(report_path)
    if report.get("type") != "raytracer_m6_compact_report" or report.get("gpu") != approved.get("gpu"):
        raise RuntimeError("Approved M6 report type or GPU/driver binding is invalid")
    return {
        "suite": str(suite_path.resolve()),
        "suite_sha256": sha256(suite_path),
        "approved_m6_report": str(report_path),
        "approved_m6_report_sha256": actual_hash,
        "gpu": approved["gpu"],
    }


def add_gate(gates: list[dict[str, object]], name: str, passed: bool, actual: object, expected: object) -> None:
    gates.append({"name": name, "pass": bool(passed), "actual": actual, "expected": expected})


def timing_section(summary: dict[str, object], group: str, name: str) -> dict[str, object] | None:
    for section in summary.get(group, []):
        if section.get("name") == name:
            return section
    return None


def device_local_bytes(summary: dict[str, object]) -> int | None:
    final = summary.get("gpu_memory", {}).get("final", {})
    heaps = [heap for heap in final.get("heaps", []) if heap.get("category") == "device_local"]
    if not heaps or any("allocation_bytes" not in heap or int(heap["allocation_bytes"]) < 0 for heap in heaps):
        return None
    return sum(int(heap["allocation_bytes"]) for heap in heaps)


def record_for_row(output_root: Path, row: dict[str, object]) -> tuple[Path, dict[str, object]]:
    output = Path(str(row["output"])).resolve()
    record_path = output.with_suffix(".record.json")
    if output_root.resolve() not in record_path.parents:
        raise RuntimeError(f"M12 record escaped its evidence root: {record_path}")
    return record_path, load_json(record_path)


def comparison(report: dict[str, object], name: str) -> dict[str, object]:
    for item in report.get("image_comparisons", []):
        if item.get("name") == name:
            return item.get("metrics", {})
    raise RuntimeError(f"M12 report is missing image comparison {name}")


def validate_report(output_root: Path, suite_path: Path) -> dict[str, object]:
    provenance = suite_binding(suite_path)
    report = load_json(output_root / "report.json")
    if report.get("type") != "raytracer_m6_compact_report":
        raise RuntimeError("M12 requires a compact M6 harness report")
    rows = report.get("runs", [])
    rows_by_technique = {str(row.get("technique")): row for row in rows}
    expected_techniques = {"rtx", "rq", "query-only"}
    if len(rows) != 3 or set(rows_by_technique) != expected_techniques:
        raise RuntimeError("M12 requires exactly one fresh RTX, RayQuery, and forced query-only capture")

    gates: list[dict[str, object]] = []
    add_gate(gates, "fresh_capture_count", report.get("run_count") == 3, report.get("run_count"), 3)
    add_gate(gates, "approved_gpu_driver", report.get("gpu") == provenance["gpu"], report.get("gpu"), provenance["gpu"])
    add_gate(
        gates,
        "reference_not_executed",
        report.get("frozen_reference", {}).get("mode") == "frozen_historical"
        and report.get("frozen_reference", {}).get("binding_speedup_claims_allowed") is False,
        report.get("frozen_reference"),
        "frozen historical reference only",
    )

    lanes: dict[str, object] = {}
    records: list[dict[str, str]] = []
    for technique in ("rtx", "rq", "query-only"):
        record_path, record = record_for_row(output_root, rows_by_technique[technique])
        records.append({"technique": technique, "path": str(record_path), "sha256": sha256(record_path)})
        summary = record.get("summary", {})
        add_gate(gates, f"{technique}_capture_schema", summary.get("schema") == 2, summary.get("schema"), 2)
        builder = summary.get("blas_builder")
        add_gate(gates, f"{technique}_builder_telemetry_present", isinstance(builder, dict), builder, "object")
        if not isinstance(builder, dict):
            continue

        hint = int(builder.get("fixed_hint_bytes", 0))
        passes = builder.get("passes", [])
        static_count = int(builder.get("static_eligible_count", -1))
        cumulative_built_count = int(builder.get("cumulative_built_static_count", -1))
        cumulative_uncompacted = int(builder.get("cumulative_uncompacted_bytes", -1))
        cumulative_compacted = int(builder.get("cumulative_compacted_bytes", -1))
        pass_invariants = isinstance(passes, list) and len(passes) == int(builder.get("pass_count", -1)) > 0
        expected_begin = 0
        planned_count = 0
        planned_destination = 0
        planned_waves = 0
        planned_scratch_peak = 0
        if pass_invariants:
            for build_pass in passes:
                count = int(build_pass.get("count", 0))
                begin = int(build_pass.get("begin", -1))
                destination = int(build_pass.get("destination_bytes", 0))
                scratch = int(build_pass.get("scratch_bytes", 0))
                waves = int(build_pass.get("scratch_wave_count", 0))
                oversized = bool(build_pass.get("oversized_singleton", False))
                destination_valid = destination <= hint or (count == 1 and oversized)
                scratch_valid = scratch <= hint or (count == 1 and oversized)
                pass_invariants = (
                    pass_invariants
                    and begin == expected_begin
                    and count > 0
                    and waves > 0
                    and destination_valid
                    and scratch_valid
                )
                expected_begin += count
                planned_count += count
                planned_destination += destination
                planned_waves += waves
                planned_scratch_peak = max(planned_scratch_peak, scratch)

        uncompacted = int(builder.get("eligible_static_uncompacted_bytes", 0))
        compacted = int(builder.get("eligible_static_compacted_bytes", 0))
        ratio = compacted / uncompacted if uncompacted else 1.0
        reported_ratio = float(builder.get("eligible_static_compaction_ratio", float("nan")))
        build_section = timing_section(summary, "startup_gpu_sections", "BLAS Build")
        compact_section = timing_section(summary, "startup_gpu_sections", "BLAS Compact")
        build_total = float(build_section.get("total_ms", float("inf"))) if build_section else float("inf")
        capture_as_sections = [
            section.get("name")
            for section in summary.get("gpu_sections", [])
            if str(section.get("name", "")).startswith(("BLAS ", "TLAS "))
        ]
        final_device_local = device_local_bytes(summary)
        build_timing_valid = (
            build_section is not None
            and math.isfinite(build_total)
            and build_total > 0.0
            and int(build_section.get("sample_count", -1)) == int(builder.get("pass_count", -2))
        )
        compact_total = float(compact_section.get("total_ms", float("nan"))) if compact_section else float("nan")
        compact_timing_valid = (
            compact_section is not None
            and math.isfinite(compact_total)
            and compact_total > 0.0
            and int(compact_section.get("sample_count", -1)) == int(builder.get("pass_count", -2))
        )
        pass_invariants = (
            pass_invariants
            and planned_count == cumulative_built_count
            and planned_destination == cumulative_uncompacted
            and cumulative_compacted >= compacted
            and planned_waves == int(builder.get("scratch_wave_count", -1))
            and planned_scratch_peak == int(builder.get("scratch_peak_bytes", -1))
        )

        add_gate(gates, f"{technique}_builder_complete", builder.get("complete") is True, builder.get("complete"), True)
        add_gate(gates, f"{technique}_builder_pending_zero", builder.get("pending_count") == 0, builder.get("pending_count"), 0)
        add_gate(gates, f"{technique}_fixed_hint", hint == BUILD_HINT_BYTES, hint, BUILD_HINT_BYTES)
        add_gate(gates, f"{technique}_pass_invariants", pass_invariants, passes, "budgeted nonempty passes")
        add_gate(
            gates,
            f"{technique}_shared_static_inputs",
            int(builder.get("shared_input_count", -1)) == static_count > 0,
            {"shared": builder.get("shared_input_count"), "static": builder.get("static_eligible_count")},
            "shared == static > 0",
        )
        add_gate(
            gates,
            f"{technique}_private_inputs_updateable_only",
            int(builder.get("private_input_count", -1)) == int(builder.get("updateable_count", -2)),
            {"private": builder.get("private_input_count"), "updateable": builder.get("updateable_count")},
            "private == updateable",
        )
        add_gate(gates, f"{technique}_blas_build_timing_valid", build_timing_valid, build_section, "positive and one sample per pass")
        add_gate(gates, f"{technique}_blas_build_gpu", build_timing_valid and build_total <= MAX_BLAS_BUILD_GPU_MS, build_total, f"<= {MAX_BLAS_BUILD_GPU_MS}")
        add_gate(
            gates,
            f"{technique}_compact_timestamp_present",
            compact_timing_valid,
            compact_section,
            "positive startup BLAS Compact with one sample per pass",
        )
        add_gate(gates, f"{technique}_capture_as_zero", not capture_as_sections, capture_as_sections, [])
        add_gate(
            gates,
            f"{technique}_static_compaction_ratio",
            uncompacted > 0 and compacted > 0 and ratio <= MAX_COMPACTION_RATIO,
            ratio,
            f"<= {MAX_COMPACTION_RATIO}",
        )
        add_gate(
            gates,
            f"{technique}_reported_compaction_ratio",
            math.isfinite(reported_ratio) and math.isclose(reported_ratio, ratio, rel_tol=0.0, abs_tol=1.0e-12),
            reported_ratio,
            ratio,
        )
        add_gate(
            gates,
            f"{technique}_final_compacted_storage",
            int(builder.get("final_compacted_storage_bytes", -1)) == compacted,
            builder.get("final_compacted_storage_bytes"),
            compacted,
        )
        add_gate(
            gates,
            f"{technique}_final_device_local",
            final_device_local is not None and final_device_local <= MAX_FINAL_DEVICE_LOCAL_BYTES,
            final_device_local,
            f"<= {MAX_FINAL_DEVICE_LOCAL_BYTES}",
        )
        add_gate(
            gates,
            f"{technique}_blas_count_accounting",
            int(builder.get("total_blas_count", -1)) == static_count + int(builder.get("updateable_count", -2)),
            builder.get("total_blas_count"),
            static_count + int(builder.get("updateable_count", -2)),
        )
        add_gate(
            gates,
            f"{technique}_report_only_telemetry",
            int(builder.get("scratch_peak_bytes", 0)) > 0
            and int(builder.get("transient_peak_bytes", 0)) > 0
            and float(builder.get("wall_milliseconds", 0.0)) > 0.0,
            {
                "scratch_peak_bytes": builder.get("scratch_peak_bytes"),
                "transient_peak_bytes": builder.get("transient_peak_bytes"),
                "wall_milliseconds": builder.get("wall_milliseconds"),
            },
            "positive values",
        )
        lanes[technique] = {
            "blas_build_gpu_ms": build_total,
            "compaction_gpu_ms": compact_total if compact_timing_valid else None,
            "compaction_ratio": ratio,
            "final_device_local_bytes": final_device_local,
            "builder": builder,
        }

    rtx_rq = comparison(report, "evo-rtx-vs-rq")
    rq_query_only = comparison(report, "evo-rq-vs-query-only")
    add_gate(gates, "rtx_rq_relative_l2", float(rtx_rq.get("relative_l2_error", 1.0)) <= 0.010, rtx_rq, "relative L2 <= 0.010")
    add_gate(gates, "rq_query_only_bit_exact", rq_query_only.get("exact_match") is True, rq_query_only, "exact match")

    result = {
        "schema": 1,
        "type": "raytracer_m12_acceptance_report",
        "pass": all(gate["pass"] for gate in gates),
        "capture_accounting": {
            "observed_fresh_precommit": 3,
            "planned_postcommit_delivery": 1,
            "planned_total_milestone_capture_count": 4,
            "reference_executed": False,
        },
        "provenance": {**provenance, "records": records},
        "targets": {
            "fixed_hint_bytes": BUILD_HINT_BYTES,
            "maximum_blas_build_gpu_ms": MAX_BLAS_BUILD_GPU_MS,
            "maximum_compaction_ratio": MAX_COMPACTION_RATIO,
            "maximum_final_device_local_bytes": MAX_FINAL_DEVICE_LOCAL_BYTES,
        },
        "lanes": lanes,
        "gates": gates,
        "failed_gates": [gate["name"] for gate in gates if not gate["pass"]],
    }
    write_json(output_root / "m12-report.json", result)
    return result


def synthetic_summary() -> dict[str, object]:
    builder = {
        "fixed_hint_bytes": BUILD_HINT_BYTES,
        "complete": True,
        "pending_count": 0,
        "static_eligible_count": 10,
        "updateable_count": 2,
        "total_blas_count": 12,
        "shared_input_count": 10,
        "private_input_count": 2,
        "cumulative_built_static_count": 10,
        "cumulative_uncompacted_bytes": 1000,
        "cumulative_compacted_bytes": 500,
        "pass_count": 1,
        "scratch_wave_count": 1,
        "scratch_peak_bytes": 1024,
        "eligible_static_uncompacted_bytes": 1000,
        "eligible_static_compacted_bytes": 500,
        "eligible_static_compaction_ratio": 0.5,
        "final_compacted_storage_bytes": 500,
        "transient_peak_bytes": 2000,
        "wall_milliseconds": 10.0,
        "passes": [
            {
                "begin": 0,
                "count": 10,
                "destination_bytes": 1000,
                "scratch_bytes": 1024,
                "scratch_wave_count": 1,
                "oversized_singleton": False,
            }
        ],
    }
    return {
        "schema": 2,
        "blas_builder": builder,
        "startup_gpu_sections": [
            {"name": "BLAS Build", "sample_count": 1, "total_ms": 100.0},
            {"name": "BLAS Compact", "sample_count": 1, "total_ms": 10.0},
        ],
        "gpu_sections": [],
        "gpu_memory": {
            "final": {"heaps": [{"category": "device_local", "allocation_bytes": 5_000_000_000}]}
        },
    }


def write_synthetic_report(
    output_root: Path, summaries: dict[str, dict[str, object]], gpu: dict[str, object]
) -> None:
    rows = []
    for technique, summary in summaries.items():
        output = output_root / f"{technique}.hdr"
        write_json(output.with_suffix(".record.json"), {"summary": summary})
        rows.append({"technique": technique, "output": str(output)})
    write_json(
        output_root / "report.json",
        {
            "type": "raytracer_m6_compact_report",
            "run_count": 3,
            "gpu": gpu,
            "frozen_reference": {"mode": "frozen_historical", "binding_speedup_claims_allowed": False},
            "runs": rows,
            "image_comparisons": [
                {"name": "evo-rtx-vs-rq", "metrics": {"relative_l2_error": 0.001}},
                {"name": "evo-rq-vs-query-only", "metrics": {"exact_match": True}},
            ],
        },
    )


def run_self_test(suite_path: Path) -> None:
    def remove_scratch_oversize_flag(value: dict[str, object]) -> None:
        builder = value["blas_builder"]
        builder.update(static_eligible_count=1, shared_input_count=1, total_blas_count=3)
        builder["scratch_peak_bytes"] = BUILD_HINT_BYTES + 1
        builder["passes"][0].update(count=1, scratch_bytes=BUILD_HINT_BYTES + 1)

    with tempfile.TemporaryDirectory(prefix="raytracer-m12-self-test-") as temporary_directory:
        output_root = Path(temporary_directory)
        suite = copy.deepcopy(load_json(suite_path))
        approved = suite["m11_acceptance"]["approved_m6"]
        gpu = approved["gpu"]
        baseline_path = output_root / "approved-m6.json"
        write_json(baseline_path, {"type": "raytracer_m6_compact_report", "gpu": gpu})
        approved["report"] = str(baseline_path)
        approved["report_sha256"] = sha256(baseline_path)
        synthetic_suite_path = output_root / "suite.json"
        write_json(synthetic_suite_path, suite)
        passing = {technique: synthetic_summary() for technique in ("rtx", "rq", "query-only")}
        write_synthetic_report(output_root, passing, gpu)
        if not validate_report(output_root, synthetic_suite_path)["pass"]:
            raise AssertionError("Synthetic passing M12 evidence was rejected")

        mutations = {
            "capture_as": lambda value: value["gpu_sections"].append({"name": "TLAS Update", "total_ms": 1.0}),
            "future_capture_as": lambda value: value["gpu_sections"].append(
                {"name": "BLAS Rebuild", "total_ms": 1.0}
            ),
            "build_time": lambda value: value["startup_gpu_sections"][0].update(total_ms=426.0),
            "compact_timing": lambda value: value["startup_gpu_sections"][1].update(total_ms=0.0),
            "hint": lambda value: value["blas_builder"].update(fixed_hint_bytes=123),
            "scratch_oversize_flag": remove_scratch_oversize_flag,
            "ratio": lambda value: value["blas_builder"].update(
                eligible_static_compacted_bytes=900,
                eligible_static_compaction_ratio=0.9,
                final_compacted_storage_bytes=900,
            ),
            "pass_accounting": lambda value: value["blas_builder"]["passes"][0].update(count=9),
            "cumulative_bytes": lambda value: value["blas_builder"].update(cumulative_uncompacted_bytes=999),
            "vram": lambda value: value["gpu_memory"]["final"]["heaps"][0].update(allocation_bytes=6_000_000_001),
            "missing_vram": lambda value: value.pop("gpu_memory"),
            "telemetry": lambda value: value.pop("blas_builder"),
        }
        for name, mutate in mutations.items():
            evidence = {technique: copy.deepcopy(synthetic_summary()) for technique in ("rtx", "rq", "query-only")}
            mutate(evidence["rtx"])
            write_synthetic_report(output_root, evidence, gpu)
            result = validate_report(output_root, synthetic_suite_path)
            if result["pass"]:
                raise AssertionError(f"Synthetic M12 {name} failure was accepted")

        write_synthetic_report(output_root, passing, {**gpu, "driver_version": 0})
        if validate_report(output_root, synthetic_suite_path)["pass"]:
            raise AssertionError("Synthetic M12 GPU/driver mismatch was accepted")


def run_m6(args: argparse.Namespace, phase: str) -> None:
    subprocess.run(
        [
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
        ],
        cwd=ROOT,
        check=True,
    )


def write_plan(output_root: Path, suite_path: Path) -> dict[str, object]:
    source = load_json(output_root / "plan.json")
    if source.get("total_runs") != 3 or source.get("fresh_benchmark_capture_count") != 3:
        raise RuntimeError("M12 plan must contain exactly three fresh precommit captures")
    plan = {
        "schema": 1,
        "type": "raytracer_m12_plan",
        "fresh_precommit_capture_count": 3,
        "postcommit_delivery_capture_count": 1,
        "total_milestone_capture_count": 4,
        "reference_executed": False,
        "provenance": suite_binding(suite_path),
        "source_plan": source,
    }
    write_json(output_root / "m12-plan.json", plan)
    return plan


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
    if args.phase == "self-test":
        subprocess.run([sys.executable, str(M6_RUNNER), "--phase", "self-test"], cwd=ROOT, check=True)
        run_self_test(args.suite_manifest)
        print("M12 validator self-test passed")
        return 0
    run_m6(args, args.phase)
    write_plan(args.output_dir, args.suite_manifest)
    if args.phase in ("analyze", "all"):
        report = validate_report(args.output_dir, args.suite_manifest)
        print(f"M12 acceptance: {'PASS' if report['pass'] else 'FAIL'} -> {args.output_dir / 'm12-report.json'}")
        return 0 if report["pass"] else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
