#!/usr/bin/env python3
"""Validate the approved one-capture M12 texture-residency repair."""

from __future__ import annotations

import argparse
import copy
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import run_raytracer_m6 as m6
import validate_raytracer_m12 as m12


ROOT = Path(__file__).resolve().parents[1]
M6_RUNNER = ROOT / "Scripts" / "run_raytracer_m6.py"
DEFAULT_SOURCE_OUTPUT = ROOT / "out" / "raytracer-m12"
DEFAULT_OUTPUT = ROOT / "out" / "raytracer-m12-rtx-repair"
DEFAULT_EDITOR = ROOT / "out" / "install" / "vs2026-x64" / "bin" / "EvoEngineEditor.exe"
DEFAULT_BUILD = ROOT / "out" / "build" / "vs2026-x64"
APPROVED_SUITE_SHA256 = "4444b107cdcbddcbf3f2f0f55f13b180f7487b11672e60af24fb2e2496ce3536"
SOURCE_MEMORY_FAILURES = {
    "rtx_final_device_local",
    "rq_final_device_local",
    "query-only_final_device_local",
}
REUSED_MEMORY_FAILURES = {"rq_final_device_local", "query-only_final_device_local"}
EXPECTED_SPEC = {
    "purpose": "measure",
    "profile": "measure",
    "suite": "cross-renderer",
    "scene": "bistro",
    "demo": "bistro",
    "camera": "overview",
    "renderer": "evo",
    "technique": "rtx",
    "variant": "specialized",
    "cache_state": "cold",
    "repetition": 1,
    "width": 1280,
    "height": 720,
    "spp": 512,
    "samples_per_frame": 4,
    "motion": False,
}


def file_binding(path: Path) -> dict[str, object]:
    path = path.resolve()
    return {"path": str(path), "sha256": m12.sha256(path), "bytes": path.stat().st_size}


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
            "--suite",
            "cross-renderer",
            "--camera",
            "overview",
            "--renderer",
            "evo",
            "--technique",
            "rtx",
            "--variant",
            "specialized",
            "--cache-state",
            "cold",
        ],
        cwd=ROOT,
        check=True,
    )


def repair_spec(output_root: Path) -> tuple[dict[str, object], dict[str, object]]:
    plan = m12.load_json(output_root / "plan.json")
    groups = plan.get("groups", {})
    measure = groups.get("measure", []) if isinstance(groups, dict) else []
    empty_groups = all(not rows for name, rows in groups.items() if name != "measure")
    if (
        plan.get("type") != "raytracer_m6_plan"
        or plan.get("total_runs") != 1
        or len(measure) != 1
        or not empty_groups
    ):
        raise RuntimeError("M12 repair plan must contain exactly one measure RTX capture")
    spec = measure[0]
    mismatches = {key: spec.get(key) for key, expected in EXPECTED_SPEC.items() if spec.get(key) != expected}
    if mismatches:
        raise RuntimeError(f"M12 repair plan does not match the approved capture: {mismatches}")
    return plan, spec


def record_path(output_root: Path, spec: dict[str, object]) -> Path:
    return (
        output_root
        / str(spec["profile"])
        / str(spec["purpose"])
        / f"{spec['run_id']}.record.json"
    ).resolve()


def write_repair_plan(args: argparse.Namespace) -> dict[str, object]:
    source_report = args.source_output_dir / "m12-report.json"
    source_harness_report = args.source_output_dir / "report.json"
    source_plan = args.source_output_dir / "m12-plan.json"
    plan, spec = repair_spec(args.output_dir)
    result = {
        "schema": 1,
        "type": "raytracer_m12_repair_plan",
        "approved_exception": "one supplementary RTX precommit capture",
        "capture_accounting": {
            "original_precommit": 3,
            "supplementary_precommit": 1,
            "observed_precommit_after_measure": 4,
            "planned_postcommit_delivery": 1,
            "total_milestone_capture_count": 5,
            "reference_executed": False,
        },
        "resolution_scope": {
            "fresh_post_fix": ["rtx"],
            "reused_pre_fix": ["rq", "query-only"],
            "fresh_post_fix_rq_memory_measured": False,
            "rq_query_only_resolution": "approved_shared_resource_inference",
        },
        "source_evidence": {
            "plan": file_binding(source_plan),
            "harness_report": file_binding(source_harness_report),
            "acceptance_report": file_binding(source_report),
        },
        "supplementary_harness_plan": file_binding(args.output_dir / "plan.json"),
        "supplementary_spec": spec,
        "source_plan": plan,
    }
    m12.write_json(args.output_dir / "m12-repair-plan.json", result)
    return result


def parse_loading_log(path: Path) -> dict[str, int]:
    text = path.read_text(encoding="utf-8", errors="replace")
    patterns = {
        "scan": r"Asset metadata scan took[^\r\n]*\((\d+) assets\)",
        "dispatch": r"Project asset load dispatch took[^\r\n]*\((\d+) assets\)",
        "texture_count": r"Bistro parity scene:[^\r\n]*\btexture_count=(\d+)\b",
    }
    result: dict[str, int] = {}
    for name, pattern in patterns.items():
        matches = re.findall(pattern, text)
        if len(matches) != 1:
            raise RuntimeError(f"Expected one {name} marker in {path}, found {len(matches)}")
        result[name] = int(matches[0])
    return result


def source_record_paths(source_root: Path, source_report: dict[str, object]) -> dict[str, Path]:
    records = source_report.get("provenance", {}).get("records", [])
    result: dict[str, Path] = {}
    for record in records:
        technique = str(record.get("technique"))
        path = Path(str(record.get("path", ""))).resolve()
        if source_root.resolve() not in path.parents:
            raise RuntimeError(f"Source M12 record escaped its evidence root: {path}")
        if m12.sha256(path) != record.get("sha256"):
            raise RuntimeError(f"Source M12 record hash mismatch: {path}")
        result[technique] = path
    if set(result) != {"rtx", "rq", "query-only"}:
        raise RuntimeError("Source M12 report does not bind all three original records")
    return result


def validate_overlay(
    source_root: Path,
    suite_path: Path,
    repair_record: dict[str, object],
    original_records: dict[str, Path],
) -> dict[str, object]:
    source_harness_report = copy.deepcopy(m12.load_json(source_root / "report.json"))
    with tempfile.TemporaryDirectory(prefix="raytracer-m12-repair-overlay-") as directory:
        overlay_root = Path(directory)
        for row in source_harness_report["runs"]:
            technique = str(row["technique"])
            local_output = overlay_root / f"{technique}.hdr"
            row["output"] = str(local_output)
            record = repair_record if technique == "rtx" else m12.load_json(original_records[technique])
            m12.write_json(local_output.with_suffix(".record.json"), record)
        m12.write_json(overlay_root / "report.json", source_harness_report)
        return m12.validate_report(overlay_root, suite_path)


def supplementary_record_binding(
    output_root: Path,
    spec: dict[str, object],
    record: dict[str, object],
) -> tuple[m6.RunSpec, str]:
    provenance = m12.load_json(output_root / "provenance.json")
    evidence_fingerprint = str(provenance.get("evidence_fingerprint", ""))
    fingerprint_input = dict(provenance)
    fingerprint_input.pop("evidence_fingerprint", None)
    if not evidence_fingerprint or m6.json_sha256(fingerprint_input) != evidence_fingerprint:
        raise RuntimeError("Supplementary evidence provenance fingerprint is invalid")
    run_values = dict(spec)
    run_id = run_values.pop("run_id", None)
    run_spec = m6.RunSpec(**run_values)
    metrics_path = Path(str(record.get("metrics_path", "")))
    if (
        run_id != run_spec.run_id
        or record.get("run_id") != run_spec.run_id
        or record.get("spec") != run_values
        or record.get("evidence_fingerprint") != evidence_fingerprint
        or not metrics_path.is_file()
        or record.get("summary") != m12.load_json(metrics_path)
    ):
        raise RuntimeError("Supplementary record does not match its approved plan, provenance, and metrics")
    return run_spec, evidence_fingerprint


def validate_repair(args: argparse.Namespace) -> dict[str, object]:
    plan, spec = repair_spec(args.output_dir)
    source_report_path = args.source_output_dir / "m12-report.json"
    source_report = m12.load_json(source_report_path)
    original_records = source_record_paths(args.source_output_dir, source_report)
    repair_record_path = record_path(args.output_dir, spec)
    repair_record = m12.load_json(repair_record_path)
    output_path = Path(str(repair_record.get("output", ""))).resolve()
    log_path = Path(str(repair_record.get("log", ""))).resolve()
    metrics_path = Path(str(repair_record.get("metrics_path", ""))).resolve()
    for path in (repair_record_path, output_path, log_path, metrics_path):
        if args.output_dir.resolve() not in path.parents:
            raise RuntimeError(f"Supplementary evidence escaped its root: {path}")

    run_spec, evidence_fingerprint = supplementary_record_binding(args.output_dir, spec, repair_record)
    m6.verify_record_integrity(
        repair_record,
        run_spec,
        m6.load_suite(args.suite_manifest),
        evidence_fingerprint,
    )

    overlay = validate_overlay(
        args.source_output_dir,
        args.suite_manifest,
        repair_record,
        original_records,
    )
    provenance = m12.suite_binding(args.suite_manifest)
    summary = repair_record.get("summary", {})
    gates: list[dict[str, object]] = []
    m12.add_gate(
        gates,
        "source_failed_gate_set",
        set(source_report.get("failed_gates", [])) == SOURCE_MEMORY_FAILURES,
        source_report.get("failed_gates"),
        sorted(SOURCE_MEMORY_FAILURES),
    )
    source_non_memory_failures = [
        gate["name"]
        for gate in source_report.get("gates", [])
        if gate["name"] not in SOURCE_MEMORY_FAILURES and not gate["pass"]
    ]
    m12.add_gate(gates, "source_non_memory_gates", not source_non_memory_failures, source_non_memory_failures, [])
    m12.add_gate(gates, "supplementary_capture_count", plan.get("total_runs") == 1, plan.get("total_runs"), 1)
    m12.add_gate(
        gates,
        "supplementary_record_complete",
        repair_record.get("type") == "raytracer_m6_run"
        and repair_record.get("status") == "complete"
        and summary.get("schema") == 2,
        {
            "type": repair_record.get("type"),
            "status": repair_record.get("status"),
            "summary_schema": summary.get("schema"),
        },
        "complete schema-2 M6 run",
    )
    m12.add_gate(
        gates,
        "supplementary_artifact_hashes",
        all(
            (
                output_path.is_file() and m12.sha256(output_path) == repair_record.get("output_sha256"),
                log_path.is_file() and m12.sha256(log_path) == repair_record.get("log_sha256"),
                metrics_path.is_file() and m12.sha256(metrics_path) == repair_record.get("metrics_sha256"),
            )
        ),
        {
            "output": repair_record.get("output_sha256"),
            "log": repair_record.get("log_sha256"),
            "metrics": repair_record.get("metrics_sha256"),
        },
        "recorded hashes match files",
    )
    approved_gpu = provenance["gpu"]
    measured_gpu = {key: summary.get("gpu", {}).get(key) for key in approved_gpu}
    m12.add_gate(
        gates,
        "approved_gpu_driver",
        measured_gpu == approved_gpu,
        measured_gpu,
        approved_gpu,
    )
    m12.add_gate(
        gates,
        "rtx_backend_identity",
        summary.get("render_mode") == "RayTracing"
        and summary.get("active_ray_backend") == "ray-tracing-pipeline"
        and summary.get("query_only") is False
        and summary.get("capabilities", {}).get("ray_tracing_pipeline") is True,
        {
            "render_mode": summary.get("render_mode"),
            "backend": summary.get("active_ray_backend"),
            "query_only": summary.get("query_only"),
            "capabilities": summary.get("capabilities"),
        },
        "specialized RTX pipeline",
    )

    fresh_rtx_failures = [
        gate["name"] for gate in overlay.get("gates", []) if gate["name"].startswith("rtx_") and not gate["pass"]
    ]
    m12.add_gate(gates, "fresh_rtx_m12_gates", not fresh_rtx_failures, fresh_rtx_failures, [])
    m12.add_gate(
        gates,
        "overlay_failure_scope",
        set(overlay.get("failed_gates", [])) == REUSED_MEMORY_FAILURES,
        overlay.get("failed_gates"),
        sorted(REUSED_MEMORY_FAILURES),
    )

    old_logs = {
        technique: parse_loading_log(Path(str(m12.load_json(path)["log"])))
        for technique, path in original_records.items()
    }
    new_log = parse_loading_log(log_path)
    m12.add_gate(
        gates,
        "source_loading_markers_consistent",
        len({tuple(values.values()) for values in old_logs.values()}) == 1,
        old_logs,
        "identical scan, dispatch, and texture counts",
    )
    old_rtx_log = old_logs["rtx"]
    m12.add_gate(gates, "asset_scan_unchanged", new_log["scan"] == old_rtx_log["scan"] == 390, new_log["scan"], 390)
    m12.add_gate(
        gates,
        "material_texture_count_unchanged",
        new_log["texture_count"] == old_rtx_log["texture_count"] == 338,
        new_log["texture_count"],
        338,
    )
    expected_dispatch = old_rtx_log["dispatch"] - old_rtx_log["texture_count"]
    m12.add_gate(
        gates,
        "project_dispatch_reduced_by_shared_textures",
        new_log["dispatch"] == expected_dispatch == 52,
        new_log["dispatch"],
        52,
    )

    original_rtx = m12.load_json(original_records["rtx"])
    m12.add_gate(
        gates,
        "supplementary_image_bit_exact",
        repair_record.get("output_sha256") == original_rtx.get("output_sha256"),
        repair_record.get("output_sha256"),
        original_rtx.get("output_sha256"),
    )
    source_memory = [
        int(row["final_device_local_allocation_bytes"])
        for row in m12.load_json(args.source_output_dir / "report.json")["runs"]
    ]
    m12.add_gate(
        gates,
        "shared_resource_inference_bound",
        max(source_memory) - min(source_memory) <= 3072,
        {"minimum": min(source_memory), "maximum": max(source_memory), "spread": max(source_memory) - min(source_memory)},
        "original lane spread <= 3072 bytes",
    )
    m12.add_gate(
        gates,
        "rq_query_only_resolution_approved",
        True,
        "shared import/storage path; RTX was tied for the original maximum",
        "approved shared-resource inference",
    )

    result = {
        "schema": 1,
        "type": "raytracer_m12_repair_acceptance_report",
        "pass": all(gate["pass"] for gate in gates),
        "capture_accounting": {
            "original_precommit": 3,
            "supplementary_precommit": 1,
            "observed_precommit_total": 4,
            "planned_postcommit_delivery": 1,
            "total_milestone_capture_count": 5,
            "reference_executed": False,
        },
        "resolution_scope": {
            "fresh_post_fix": ["rtx"],
            "reused_pre_fix": ["rq", "query-only"],
            "fresh_post_fix_rq_memory_measured": False,
            "rq_query_only_resolution": "approved_shared_resource_inference",
        },
        "source_evidence": {
            "plan": file_binding(args.source_output_dir / "m12-plan.json"),
            "harness_report": file_binding(args.source_output_dir / "report.json"),
            "acceptance_report": file_binding(source_report_path),
            "records": {technique: file_binding(path) for technique, path in original_records.items()},
        },
        "supplementary_evidence": {
            "plan": file_binding(args.output_dir / "plan.json"),
            "repair_plan": file_binding(args.output_dir / "m12-repair-plan.json"),
            "record": file_binding(repair_record_path),
            "output": file_binding(output_path),
            "log": file_binding(log_path),
            "metrics": file_binding(metrics_path),
        },
        "loading_markers": {"source": old_logs, "supplementary": new_log},
        "fresh_rtx_lane": overlay.get("lanes", {}).get("rtx"),
        "fresh_rtx_gates": [gate for gate in overlay.get("gates", []) if gate["name"].startswith("rtx_")],
        "reused_cross_technique_gates": [
            gate
            for gate in overlay.get("gates", [])
            if gate["name"] in {"rtx_rq_relative_l2", "rq_query_only_bit_exact"}
        ],
        "gates": gates,
        "failed_gates": [gate["name"] for gate in gates if not gate["pass"]],
    }
    m12.write_json(args.output_dir / "m12-repair-report.json", result)
    return result


def run_self_test() -> None:
    plan = {
        "type": "raytracer_m6_plan",
        "total_runs": 1,
        "groups": {"measure": [{"run_id": "test", **EXPECTED_SPEC}]},
    }
    with tempfile.TemporaryDirectory(prefix="raytracer-m12-repair-self-test-") as directory:
        root = Path(directory)
        m12.write_json(root / "plan.json", plan)
        _, spec = repair_spec(root)
        if spec["technique"] != "rtx":
            raise AssertionError("Synthetic RTX repair plan was rejected")
        log = root / "capture.log"
        log.write_text(
            "Asset metadata scan took 1 ms (390 assets)\n"
            "Project asset load dispatch took 1 ms (52 assets)\n"
            "Bistro parity scene: texture_count=338\n",
            encoding="utf-8",
        )
        if parse_loading_log(log) != {"scan": 390, "dispatch": 52, "texture_count": 338}:
            raise AssertionError("Synthetic loading markers were rejected")
        log.write_text(log.read_text(encoding="utf-8") + "Bistro parity scene: texture_count=338\n", encoding="utf-8")
        try:
            parse_loading_log(log)
        except RuntimeError:
            pass
        else:
            raise AssertionError("Duplicate loading markers were accepted")

        run_values = dict(EXPECTED_SPEC)
        run_spec = m6.RunSpec(**run_values)
        metrics = {"schema": 2}
        metrics_path = root / "metrics.json"
        m12.write_json(metrics_path, metrics)
        provenance = {"schema": 1, "type": "test"}
        provenance["evidence_fingerprint"] = m6.json_sha256(provenance)
        m12.write_json(root / "provenance.json", provenance)
        record = {
            "run_id": run_spec.run_id,
            "spec": run_values,
            "evidence_fingerprint": provenance["evidence_fingerprint"],
            "metrics_path": str(metrics_path),
            "summary": metrics,
        }
        approved_spec = {"run_id": run_spec.run_id, **run_values}
        if supplementary_record_binding(root, approved_spec, record) != (
            run_spec,
            provenance["evidence_fingerprint"],
        ):
            raise AssertionError("Synthetic supplementary record binding was rejected")
        record["summary"] = {"schema": 3}
        try:
            supplementary_record_binding(root, approved_spec, record)
        except RuntimeError:
            pass
        else:
            raise AssertionError("Supplementary record summary tampering was accepted")


def ensure_measure_slot_unused(output_root: Path) -> None:
    measure_root = output_root / "measure"
    if measure_root.exists() and any(path.is_file() for path in measure_root.rglob("*")):
        raise RuntimeError("M12 repair measure slot already contains evidence; refusing another renderer launch")


def require_approved_measure_binding(args: argparse.Namespace) -> None:
    expected_paths = {
        "suite_manifest": m12.DEFAULT_SUITE.resolve(),
        "source_output_dir": DEFAULT_SOURCE_OUTPUT.resolve(),
        "output_dir": DEFAULT_OUTPUT.resolve(),
        "editor": DEFAULT_EDITOR.resolve(),
        "evo_build_dir": DEFAULT_BUILD.resolve(),
    }
    mismatches = {
        name: str(getattr(args, name))
        for name, expected in expected_paths.items()
        if getattr(args, name) != expected
    }
    if mismatches:
        raise RuntimeError(f"M12 repair measure paths are fixed by the approved exception: {mismatches}")
    if m12.sha256(args.suite_manifest) != APPROVED_SUITE_SHA256:
        raise RuntimeError("Approved M12 repair suite hash changed")
    repair_spec(args.output_dir)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("self-test", "plan", "measure", "analyze", "all"), default="plan")
    parser.add_argument("--suite-manifest", type=Path, default=m12.DEFAULT_SUITE)
    parser.add_argument("--source-output-dir", type=Path, default=DEFAULT_SOURCE_OUTPUT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--editor", type=Path, default=DEFAULT_EDITOR)
    parser.add_argument("--evo-build-dir", type=Path, default=DEFAULT_BUILD)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    for name in ("suite_manifest", "source_output_dir", "output_dir", "editor", "evo_build_dir"):
        setattr(args, name, getattr(args, name).resolve())
    if args.phase == "self-test":
        run_self_test()
        print("M12 repair validator self-test passed")
        return 0
    if args.phase == "plan":
        run_m6(args, "plan")
        write_repair_plan(args)
        print(f"M12 repair plan -> {args.output_dir / 'm12-repair-plan.json'}")
        return 0
    if args.phase in ("measure", "all"):
        require_approved_measure_binding(args)
        ensure_measure_slot_unused(args.output_dir)
        run_m6(args, "measure")
        repair_spec(args.output_dir)
        write_repair_plan(args)
    if args.phase in ("analyze", "all"):
        report = validate_repair(args)
        print(f"M12 repair acceptance: {'PASS' if report['pass'] else 'FAIL'} -> {args.output_dir / 'm12-repair-report.json'}")
        return 0 if report["pass"] else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
