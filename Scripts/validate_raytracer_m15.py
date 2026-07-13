#!/usr/bin/env python3
"""Validate M15 camera-owned ray-history lifetime capture metrics."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def add_gate(gates: list[dict[str, Any]], name: str, passed: bool, actual: Any, expected: Any) -> None:
    gates.append({"name": name, "passed": bool(passed), "actual": actual, "expected": expected})


def load_metrics(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"Capture metrics must be an object: {path}")
    return payload


def validate_lane(name: str, metrics: dict[str, Any], technique: str) -> list[dict[str, Any]]:
    gates: list[dict[str, Any]] = []
    history = metrics.get("ray_camera_history", {})
    baseline_history = history.get("measurement_baseline", {})
    final_history = history.get("final", {})
    resources = metrics.get("resource_lifetime", {})
    descriptors = resources.get("descriptor_sets", {})
    ray_pipelines = resources.get("ray_tracing_pipelines", {})
    final_pipelines = ray_pipelines.get("final", {})
    variant = metrics.get("ray_shader_variant", {})
    width = int(metrics.get("width", 0))
    height = int(metrics.get("height", 0))
    expected_bytes = width * height * 32

    add_gate(gates, f"{name}_capture_schema", metrics.get("schema") == 2, metrics.get("schema"), 2)
    add_gate(
        gates,
        f"{name}_capture_type",
        metrics.get("type") == "evoengine_ray_capture",
        metrics.get("type"),
        "evoengine_ray_capture",
    )
    add_gate(
        gates,
        f"{name}_camera_owned_history",
        history.get("ownership") == "camera-owned-single-slot",
        history.get("ownership"),
        "camera-owned-single-slot",
    )
    add_gate(
        gates,
        f"{name}_single_history_slot",
        history.get("maximum_histories_per_camera") == 1,
        history.get("maximum_histories_per_camera"),
        1,
    )
    add_gate(
        gates,
        f"{name}_history_bytes_per_pixel",
        history.get("bytes_per_pixel_per_history") == 32,
        history.get("bytes_per_pixel_per_history"),
        32,
    )
    add_gate(
        gates,
        f"{name}_one_live_camera",
        final_history.get("live_camera_count") == 1,
        final_history.get("live_camera_count"),
        1,
    )
    add_gate(
        gates,
        f"{name}_one_live_history",
        final_history.get("live_history_count") == 1,
        final_history.get("live_history_count"),
        1,
    )
    add_gate(
        gates,
        f"{name}_valid_history",
        final_history.get("valid_history_count") == 1,
        final_history.get("valid_history_count"),
        1,
    )
    add_gate(
        gates,
        f"{name}_technique_history",
        final_history.get(
            "live_ray_tracing_history_count" if technique == "RayTracing" else "live_ray_query_history_count"
        )
        == 1,
        final_history,
        technique,
    )
    add_gate(
        gates,
        f"{name}_other_technique_absent",
        final_history.get(
            "live_ray_query_history_count" if technique == "RayTracing" else "live_ray_tracing_history_count"
        )
        == 0,
        final_history,
        0,
    )
    for field in ("radiance_image_count", "convergence_image_count", "radiance_view_count", "convergence_view_count"):
        add_gate(gates, f"{name}_{field}", final_history.get(field) == 1, final_history.get(field), 1)
    add_gate(
        gates,
        f"{name}_exact_history_bytes",
        final_history.get("live_byte_size") == expected_bytes,
        final_history.get("live_byte_size"),
        expected_bytes,
    )
    add_gate(
        gates,
        f"{name}_history_plateau",
        history.get("capture_minimum_live_history_count") == 1
        and history.get("capture_maximum_live_history_count") == 1,
        [history.get("capture_minimum_live_history_count"), history.get("capture_maximum_live_history_count")],
        [1, 1],
    )
    add_gate(
        gates,
        f"{name}_no_measured_history_allocations",
        history.get("capture_creation_count") == 0,
        history.get("capture_creation_count"),
        0,
    )
    for field in ("creation_count", "reuse_count", "invalidation_count", "retirement_count"):
        baseline_value = baseline_history.get(field)
        final_value = final_history.get(field)
        add_gate(
            gates,
            f"{name}_{field}_monotonic",
            isinstance(baseline_value, int) and isinstance(final_value, int) and final_value >= baseline_value,
            [baseline_value, final_value],
            "final >= measurement baseline",
        )
    add_gate(
        gates,
        f"{name}_variant_ready",
        variant.get("ready") is True and variant.get("pending") is False,
        variant,
        "ready",
    )

    descriptor_min = descriptors.get("capture_minimum_live_count")
    descriptor_max = descriptors.get("capture_maximum_live_count")
    descriptor_spread = (
        descriptor_max - descriptor_min
        if isinstance(descriptor_min, int) and isinstance(descriptor_max, int)
        else None
    )
    add_gate(
        gates,
        f"{name}_descriptor_live_bound",
        descriptor_spread is not None and 0 <= descriptor_spread <= 8,
        descriptor_spread,
        "0..8",
    )

    live_pipelines = final_pipelines.get("live_pipeline_count")
    live_sbt = final_pipelines.get("live_shader_binding_table_count")
    add_gate(
        gates,
        f"{name}_sbt_matches_live_pipelines",
        isinstance(live_pipelines, int) and isinstance(live_sbt, int) and live_sbt == live_pipelines * 3,
        {"pipelines": live_pipelines, "sbt": live_sbt},
        "sbt == pipelines * 3",
    )
    add_gate(
        gates,
        f"{name}_pipeline_plateau",
        ray_pipelines.get("capture_minimum_live_pipeline_count")
        == ray_pipelines.get("capture_maximum_live_pipeline_count")
        and ray_pipelines.get("capture_minimum_live_shader_binding_table_count")
        == ray_pipelines.get("capture_maximum_live_shader_binding_table_count"),
        ray_pipelines,
        "stable live pipeline and SBT counts",
    )
    add_gate(
        gates,
        f"{name}_no_measured_pipeline_allocations",
        ray_pipelines.get("capture_pipeline_creation_count") == 0
        and ray_pipelines.get("capture_shader_binding_table_creation_count") == 0,
        [
            ray_pipelines.get("capture_pipeline_creation_count"),
            ray_pipelines.get("capture_shader_binding_table_creation_count"),
        ],
        [0, 0],
    )
    if technique == "RayQuery":
        add_gate(
            gates,
            f"{name}_query_only_backend",
            metrics.get("query_only") is True,
            metrics.get("query_only"),
            True,
        )
        add_gate(
            gates,
            f"{name}_no_rtx_pipeline_or_sbt",
            live_pipelines == 0 and live_sbt == 0,
            [live_pipelines, live_sbt],
            [0, 0],
        )
    return gates


def validate(rtx_path: Path, query_only_path: Path) -> dict[str, Any]:
    gates = validate_lane("rtx", load_metrics(rtx_path), "RayTracing")
    gates.extend(validate_lane("query_only", load_metrics(query_only_path), "RayQuery"))
    return {
        "schema": 1,
        "type": "raytracer_m15_validation",
        "pass": all(gate["passed"] for gate in gates),
        "gate_count": len(gates),
        "gates": gates,
        "captures": {"rtx": str(rtx_path.resolve()), "query_only": str(query_only_path.resolve())},
    }


def synthetic_metrics(technique: str) -> dict[str, Any]:
    is_rtx = technique == "RayTracing"
    pipeline_count = 2 if is_rtx else 0
    history = {
        "live_camera_count": 1,
        "live_history_count": 1,
        "live_ray_tracing_history_count": 1 if is_rtx else 0,
        "live_ray_query_history_count": 0 if is_rtx else 1,
        "valid_history_count": 1,
        "radiance_image_count": 1,
        "convergence_image_count": 1,
        "radiance_view_count": 1,
        "convergence_view_count": 1,
        "live_byte_size": 64 * 32 * 32,
        "creation_count": 1,
        "reuse_count": 4,
        "invalidation_count": 0,
        "retirement_count": 0,
    }
    return {
        "schema": 2,
        "type": "evoengine_ray_capture",
        "width": 64,
        "height": 32,
        "query_only": not is_rtx,
        "ray_shader_variant": {"ready": True, "pending": False},
        "ray_camera_history": {
            "ownership": "camera-owned-single-slot",
            "maximum_histories_per_camera": 1,
            "bytes_per_pixel_per_history": 32,
            "measurement_baseline": {
                "creation_count": 1,
                "reuse_count": 2,
                "invalidation_count": 0,
                "retirement_count": 0,
            },
            "final": history,
            "capture_minimum_live_history_count": 1,
            "capture_maximum_live_history_count": 1,
            "capture_creation_count": 0,
        },
        "resource_lifetime": {
            "descriptor_sets": {"capture_minimum_live_count": 20, "capture_maximum_live_count": 21},
            "ray_tracing_pipelines": {
                "final": {
                    "live_pipeline_count": pipeline_count,
                    "live_shader_binding_table_count": pipeline_count * 3,
                },
                "capture_minimum_live_pipeline_count": pipeline_count,
                "capture_maximum_live_pipeline_count": pipeline_count,
                "capture_minimum_live_shader_binding_table_count": pipeline_count * 3,
                "capture_maximum_live_shader_binding_table_count": pipeline_count * 3,
                "capture_pipeline_creation_count": 0,
                "capture_shader_binding_table_creation_count": 0,
            },
        },
    }


def self_test() -> None:
    for technique in ("RayTracing", "RayQuery"):
        gates = validate_lane(technique.lower(), synthetic_metrics(technique), technique)
        if not all(gate["passed"] for gate in gates):
            raise AssertionError([gate for gate in gates if not gate["passed"]])
    broken = synthetic_metrics("RayTracing")
    broken["ray_camera_history"]["final"]["live_history_count"] = 3
    if all(gate["passed"] for gate in validate_lane("broken", broken, "RayTracing")):
        raise AssertionError("M15 validator accepted an over-bound camera history count")
    broken = synthetic_metrics("RayTracing")
    broken["ray_camera_history"]["final"]["creation_count"] = 0
    if all(gate["passed"] for gate in validate_lane("broken", broken, "RayTracing")):
        raise AssertionError("M15 validator accepted decreasing cumulative history counters")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rtx-metrics", type=Path)
    parser.add_argument("--query-only-metrics", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        print("M15 validator self-test passed.")
        return 0
    if not args.rtx_metrics or not args.query_only_metrics or not args.output:
        parser.error("--rtx-metrics, --query-only-metrics, and --output are required")
    report = validate(args.rtx_metrics, args.query_only_metrics)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    failed = [gate for gate in report["gates"] if not gate["passed"]]
    print(json.dumps({"pass": report["pass"], "gate_count": report["gate_count"], "failed": failed}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
