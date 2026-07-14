#!/usr/bin/env python3
"""Validate the two-lane M16 frame-path slice and Debug validation smoke."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any

from compare_reference_render import compare_render_images, read_image


ROOT = Path(__file__).resolve().parents[1]
M11_REPORT = ROOT / "out" / "raytracer-m11" / "m11-report.json"
M11_REPORT_SHA256 = "090e1c7eb68638c5f50c90fe5a46ede020feb4a32ab6fb8180a33b628bf1c59e"
M11_SUITE_SHA256 = "4444b107cdcbddcbf3f2f0f55f13b180f7487b11672e60af24fb2e2496ce3536"
M11_ASSET_SHA256 = "4521bdaae5816a96d6c55f1f7a426a3166f5e580ad1178c1815de14f40c745aa"
M11_CAMERA_SHA256 = "96467f4bbe0b1978640cb7655b42f13dcf2b018fb526b9b6db30dd921415d597"
M11_CAMERA_DEPENDENCIES_SHA256 = "3339391a2a53f2a933a9dc1fb4aa813b6761fa93d2f264804369cb05bf98a8d7"
M12_BASELINE_RECORDS = {
    "rtx": ROOT / "out" / "raytracer-m12" / "measure" / "measure" /
           "measure--measure--cross-renderer--bistro--overview--evo--rtx--specialized--cold--r01--1280x720--512spp.record.json",
    "query-only": ROOT / "out" / "raytracer-m12" / "measure" / "measure" /
                  "measure--measure--cross-renderer--bistro--overview--evo--query-only--specialized--cold--r01--1280x720--512spp.record.json",
}
M12_BASELINE_RECORD_SHA256 = {
    "rtx": "da6dc59e5bdc908b7134053c312c7b175afcb49a0c827ccb0c57e060c493eca2",
    "query-only": "fdab8b5f7e6eab67b7f0fc0f06bf0100ac1b4fa9fd43147bde63432d956978c4",
}
M12_PATH_MEDIANS = {"rtx": 63.85888, "query-only": 67.269312}
RTX_PATH_MEDIAN_CEILING = 67.051824
M12_QUERY_WALL_MS_PER_FRAME = 81.64293228346455
QUERY_WALL_MS_PER_FRAME_CEILING = 85.72507889763778
ERROR_PATTERNS = (
    re.compile(r"\bVUID-"),
    re.compile(r"\bvalidation error\b", re.IGNORECASE),
    re.compile(r"\bVK_ERROR_DEVICE_LOST\b", re.IGNORECASE),
    re.compile(r"\bdevice[- ]lost\b", re.IGNORECASE),
    re.compile(r"\bfatal\b", re.IGNORECASE),
    re.compile(r"\bunhandled exception\b", re.IGNORECASE),
    re.compile(r"\baccess violation\b", re.IGNORECASE),
    re.compile(r"\bassert(?:ion)? failed\b", re.IGNORECASE),
    re.compile(r"\bEVOENGINE_(?:APP_TEST_RESULT failed|FATAL|ERROR)\b", re.IGNORECASE),
)


def add_gate(gates: list[dict[str, Any]], name: str, passed: bool, actual: Any, expected: Any) -> None:
    gates.append({"name": name, "passed": bool(passed), "actual": actual, "expected": expected})


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object: {path}")
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def optional_sha256(path: Path) -> str | None:
    return sha256(path) if path.is_file() else None


def number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def timing(metrics: dict[str, Any], section: str, name: str) -> dict[str, Any]:
    rows = metrics.get(section)
    if not isinstance(rows, list):
        return {}
    for row in rows:
        if isinstance(row, dict) and row.get("name") == name:
            return row
    return {}


def log_failures(path: Path) -> list[str]:
    if not path.is_file():
        return ["missing log"]
    return [
        line
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
        if any(pattern.search(line) for pattern in ERROR_PATTERNS)
    ]


def capture_json_from_log(path: Path) -> dict[str, Any] | None:
    prefix = "RAY_CAPTURE_JSON "
    for line in reversed(path.read_text(encoding="utf-8", errors="replace").splitlines()):
        if line.startswith(prefix):
            value = json.loads(line[len(prefix):])
            return value if isinstance(value, dict) else None
    return None


def gpu_identity(metrics: dict[str, Any]) -> dict[str, Any]:
    gpu = metrics.get("gpu") if isinstance(metrics.get("gpu"), dict) else {}
    return {key: gpu.get(key) for key in ("vendor_id", "device_id", "driver_version")}


def validate_lane(lane: str, metrics: dict[str, Any]) -> list[dict[str, Any]]:
    gates: list[dict[str, Any]] = []
    is_rtx = lane == "rtx"
    path = timing(metrics, "gpu_sections", "Path Trace (RTX)" if is_rtx else "Path Trace (RQ)")
    queue = timing(metrics, "cpu_sections", "Queue Submit")
    frame_path = metrics.get("ray_camera_frame_path") if isinstance(metrics.get("ray_camera_frame_path"), dict) else {}
    capture = frame_path.get("capture") if isinstance(frame_path.get("capture"), dict) else {}
    final_frame_path = frame_path.get("final") if isinstance(frame_path.get("final"), dict) else {}
    final_cache = (final_frame_path.get("render_graph_plan_cache")
                   if isinstance(final_frame_path.get("render_graph_plan_cache"), dict) else {})
    synchronization = (metrics.get("synchronization_waits")
                       if isinstance(metrics.get("synchronization_waits"), dict) else {})
    frame_sync = metrics.get("frame_synchronization") if isinstance(metrics.get("frame_synchronization"), dict) else {}
    measured_frames = metrics.get("measured_frames")
    wall_seconds = number(metrics.get("accumulation_wall_seconds"))
    path_total_ms = number(path.get("total_ms"))
    valid_wall = (isinstance(measured_frames, int) and measured_frames > 0 and wall_seconds is not None
                  and wall_seconds > 0.0 and path_total_ms is not None and path_total_ms > 0.0
                  and wall_seconds * 1000.0 >= path_total_ms)
    overhead = ((wall_seconds * 1000.0 - path_total_ms) / measured_frames) if valid_wall else None
    wall_ms_per_frame = wall_seconds * 1000.0 / measured_frames if valid_wall else None

    add_gate(gates, f"{lane}_capture_identity",
             [metrics.get("schema"), metrics.get("type"), metrics.get("renderer"), metrics.get("demo_profile"),
              metrics.get("ray_debug_view"), metrics.get("output_format")] ==
             [2, "evoengine_ray_capture", "EvoEngine", "bistro", "Beauty", "radiance_hdr_linear"],
             [metrics.get("schema"), metrics.get("type"), metrics.get("renderer"), metrics.get("demo_profile"),
              metrics.get("ray_debug_view"), metrics.get("output_format")], "M16 Bistro linear Beauty capture")
    add_gate(gates, f"{lane}_dimensions", [metrics.get("width"), metrics.get("height")] == [1280, 720],
             [metrics.get("width"), metrics.get("height")], [1280, 720])
    add_gate(gates, f"{lane}_frame_profile",
             [metrics.get("requested_frames"), metrics.get("timing_warmup_frames"), measured_frames] == [128, 1, 127],
             [metrics.get("requested_frames"), metrics.get("timing_warmup_frames"), measured_frames], [128, 1, 127])
    add_gate(gates, f"{lane}_sample_profile",
             [metrics.get("samples_per_frame"), metrics.get("measured_spp"), metrics.get("effective_spp")] == [4, 508, 512],
             [metrics.get("samples_per_frame"), metrics.get("measured_spp"), metrics.get("effective_spp")],
             [4, 508, 512])
    add_gate(gates, f"{lane}_deterministic_ser_off",
             metrics.get("deterministic") is True and metrics.get("ser_enabled") is False and
             str(metrics.get("ser_mode_requested", "")).lower() == "disabled" and
             metrics.get("auto_spp_enabled") is False and
             metrics.get("temporal_motion_capture") is False,
             [metrics.get("deterministic"), metrics.get("ser_enabled"), metrics.get("ser_mode_requested"),
              metrics.get("auto_spp_enabled"), metrics.get("temporal_motion_capture")],
             [True, False, "disabled", False, False])
    variant = metrics.get("ray_shader_variant") if isinstance(metrics.get("ray_shader_variant"), dict) else {}
    expected_key = "rtx:0x2001" if is_rtx else "rq:0x2001"
    add_gate(gates, f"{lane}_specialized_variant",
             variant.get("active_mask") == 8193 and variant.get("requested_mask") == 8193 and
             variant.get("active_key") == expected_key and variant.get("requested_key") == expected_key and
             variant.get("selection_mode") == "auto" and variant.get("ready") is True and
             variant.get("pending") is False and variant.get("fallback_active") is False,
             variant, f"ready {expected_key}")
    capabilities = metrics.get("capabilities") if isinstance(metrics.get("capabilities"), dict) else {}
    backend_actual = [metrics.get("render_mode"), metrics.get("active_ray_backend"), metrics.get("query_only"),
                      capabilities.get("acceleration_structure"), capabilities.get("ray_query"),
                      capabilities.get("ray_tracing_pipeline"), metrics.get("ray_pipeline_max_recursion_depth")]
    backend_expected = (["RayTracing", "ray-tracing-pipeline", False, True, True, True, 1] if is_rtx else
                        ["RayQuery", "ray-query-compute", True, True, True, False, None])
    add_gate(gates, f"{lane}_backend", backend_actual == backend_expected, backend_actual, backend_expected)
    add_gate(gates, f"{lane}_gpu_samples", metrics.get("gpu_timestamps_available") is True and
             path.get("sample_count") == 127, [metrics.get("gpu_timestamps_available"), path.get("sample_count")],
             [True, 127])
    path_median = number(path.get("median_ms"))
    if is_rtx:
        add_gate(gates, "rtx_path_gpu_ceiling",
                 path_median is not None and path_median <= RTX_PATH_MEDIAN_CEILING, path.get("median_ms"),
                 f"<= {RTX_PATH_MEDIAN_CEILING}")
    else:
        add_gate(gates, "query-only_wall_frame_ceiling",
                 wall_ms_per_frame is not None and wall_ms_per_frame <= QUERY_WALL_MS_PER_FRAME_CEILING,
                 {"wall_ms_per_frame": wall_ms_per_frame, "path_median_ms_diagnostic": path_median},
                 f"wall <= {QUERY_WALL_MS_PER_FRAME_CEILING} ms/frame; path median is diagnostic under sustained load")
    queue_median = number(queue.get("median_ms"))
    add_gate(gates, f"{lane}_queue_samples", queue.get("sample_count") == 127, queue.get("sample_count"), 127)
    add_gate(gates, f"{lane}_queue_submit", queue_median is not None and queue_median <= 0.10,
             queue.get("median_ms"), "<= 0.10 ms")
    add_gate(gates, f"{lane}_normalized_overhead", overhead is not None and overhead <= 12.5, overhead,
             "0..12.5 ms/frame")
    add_gate(gates, f"{lane}_wait_policy",
             frame_sync.get("policy") == "wait-on-frame-slot-reuse" and
             frame_sync.get("max_frames_in_flight") == 2 and
             frame_sync.get("pending_submissions_after_capture_flush") == 0,
             frame_sync, "two slots, wait on reuse, drained capture")
    for reason in ("just_submitted_frame", "redundant"):
        reason_stats = synchronization.get(reason) if isinstance(synchronization.get(reason), dict) else {}
        add_gate(gates, f"{lane}_zero_{reason}", reason_stats.get("count") == 0, reason_stats, {"count": 0})
    add_gate(gates, f"{lane}_plan_cache_hot",
             capture.get("plan_compilations") == 0 and capture.get("plan_cache_misses") == 0 and
             int(capture.get("plan_cache_hits", 0)) >= 127,
             capture, "0 compile/miss and >=127 hits")
    add_gate(gates, f"{lane}_output_descriptors_hot",
             capture.get("output_descriptor_creations") == 0 and
             int(capture.get("output_descriptor_reuses", 0)) >= 127,
             capture, "0 creates and >=127 reuses")
    add_gate(gates, f"{lane}_bounded_cache_and_retention",
             isinstance(final_cache.get("entry_count"), int) and 0 < final_cache["entry_count"] <= 16 and
             final_cache.get("capacity") == 16 and
             final_frame_path.get("retained_frame_slot_count") == 2,
             {"cache": final_cache, "retained_frame_slot_count": final_frame_path.get("retained_frame_slot_count")},
             "1..16 plans and two retained frame slots")
    add_gate(gates, f"{lane}_two_output_descriptors",
             final_frame_path.get("live_output_descriptor_count") == 2 and
             int(final_frame_path.get("peak_live_output_descriptor_count", 0)) >= 2,
             final_frame_path, "2 live and peak >=2")
    history = metrics.get("ray_camera_history") if isinstance(metrics.get("ray_camera_history"), dict) else {}
    history_final = history.get("final") if isinstance(history.get("final"), dict) else {}
    add_gate(gates, f"{lane}_single_history",
             history.get("ownership") == "camera-owned-single-slot" and
             history.get("maximum_histories_per_camera") == 1 and history_final.get("live_history_count") == 1 and
             history.get("capture_creation_count") == 0,
             history, "one camera history, no measured allocation")
    memory = metrics.get("gpu_memory") if isinstance(metrics.get("gpu_memory"), dict) else {}
    startup_memory = memory.get("startup_ready") if isinstance(memory.get("startup_ready"), dict) else {}
    final_memory = memory.get("final") if isinstance(memory.get("final"), dict) else {}
    startup_allocations = startup_memory.get("allocation_count")
    final_allocations = final_memory.get("allocation_count")
    add_gate(gates, f"{lane}_stable_vma_allocations",
             isinstance(startup_allocations, int) and startup_allocations > 0 and
             startup_allocations == final_allocations,
             [startup_allocations, final_allocations], "equal positive counts")
    return gates


def validate_record(lane: str, record: dict[str, Any], provenance: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, Any], Path]:
    gates: list[dict[str, Any]] = []
    expected_technique = "rtx" if lane == "rtx" else "query-only"
    expected_spec = {"renderer": "evo", "technique": expected_technique, "profile": "measure",
                     "purpose": "measure", "suite": "cross-renderer", "scene": "bistro", "demo": "bistro",
                     "camera": "overview", "variant": "specialized", "cache_state": "cold", "width": 1280,
                     "height": 720, "spp": 512, "samples_per_frame": 4, "motion": False}
    spec = record.get("spec") if isinstance(record.get("spec"), dict) else {}
    add_gate(gates, f"{lane}_record_complete",
             [record.get("schema"), record.get("type"), record.get("status")] == [1, "raytracer_m6_run", "complete"],
             [record.get("schema"), record.get("type"), record.get("status")], [1, "raytracer_m6_run", "complete"])
    add_gate(gates, f"{lane}_record_spec", all(spec.get(key) == value for key, value in expected_spec.items()),
             spec, expected_spec)
    metrics_path = Path(str(record.get("metrics_path", "")))
    output_path = Path(str(record.get("output", "")))
    log_path = Path(str(record.get("log", "")))
    add_gate(gates, f"{lane}_artifact_integrity",
             optional_sha256(metrics_path) == record.get("metrics_sha256") and
             optional_sha256(output_path) == record.get("output_sha256") and
             optional_sha256(log_path) == record.get("log_sha256"),
             {"metrics": optional_sha256(metrics_path), "output": optional_sha256(output_path),
              "log": optional_sha256(log_path)}, "recorded SHA-256 values")
    failures = log_failures(log_path)
    add_gate(gates, f"{lane}_clean_process_log", not failures, failures[:5], [])
    cache = record.get("cache") if isinstance(record.get("cache"), dict) else {}
    environment = record.get("environment") if isinstance(record.get("environment"), dict) else {}
    add_gate(gates, f"{lane}_isolated_cold_cache",
             cache.get("frontend_cache") == "cold" and bool(cache.get("application_pipeline_cache")) and
             bool(environment.get("EVOENGINE_SHADER_CACHE_DIR")),
             {"cache": cache, "environment": environment}, "cold isolated shader and pipeline caches")

    build = provenance.get("builds", {}).get("evoengine", {})
    binary = provenance.get("binaries", {}).get("evoengine", {})
    repository = provenance.get("repositories", {}).get("evoengine", {})
    camera = provenance.get("camera_assets", {}).get("overview", {})
    add_gate(gates, f"{lane}_provenance_identity",
             provenance.get("schema") == 1 and provenance.get("type") == "raytracer_m6_provenance" and
             build.get("configuration") == "RelWithDebInfo" and bool(binary.get("combined_sha256")) and
             bool(repository.get("head")) and bool(repository.get("working_diff_sha256")) and
             provenance.get("suite", {}).get("sha256") == M11_SUITE_SHA256 and
             provenance.get("assets", {}).get("evoengine", {}).get("combined_sha256") == M11_ASSET_SHA256 and
             camera.get("output_sha256") == M11_CAMERA_SHA256 and
             camera.get("dependencies", {}).get("combined_sha256") == M11_CAMERA_DEPENDENCIES_SHA256 and
             record.get("evidence_fingerprint") == provenance.get("evidence_fingerprint"),
             {"build": build, "binary_sha256": binary.get("combined_sha256"), "repository": repository,
              "suite": provenance.get("suite"), "asset_sha256": provenance.get("assets", {}).get("evoengine", {}).get("combined_sha256"),
              "camera_sha256": camera.get("output_sha256"), "camera_dependencies_sha256": camera.get("dependencies", {}).get("combined_sha256")},
             "M11-matched RelWithDebInfo provenance")
    metrics = load_json(metrics_path) if metrics_path.is_file() else {}
    gates.extend(validate_lane(lane, metrics))
    return gates, metrics, output_path


def validate_debug(metrics: dict[str, Any], exit_code: int, executable: Path, failures: list[str],
                   image_dimensions: list[int] | None, artifacts_bound: bool, log_matches_metrics: bool,
                   vma_leak_bytes: int | None, image_finite: bool, image_nonblank: bool) -> list[dict[str, Any]]:
    gates: list[dict[str, Any]] = []
    frame_sync = metrics.get("frame_synchronization") if isinstance(metrics.get("frame_synchronization"), dict) else {}
    waits = metrics.get("synchronization_waits") if isinstance(metrics.get("synchronization_waits"), dict) else {}
    add_gate(gates, "debug_process_and_executable",
             exit_code == 0 and executable.resolve() ==
             (ROOT / "out/build/vs2026-x64/EvoEngine_App/Debug/EvoEngineEditor.exe").resolve(),
             {"exit_code": exit_code, "executable": str(executable)}, "exit 0 from Debug EvoEngineEditor")
    add_gate(gates, "debug_artifacts_bound", artifacts_bound, artifacts_bound,
             "metrics, log, exit code, and image share one evidence directory")
    add_gate(gates, "debug_log_matches_metrics", log_matches_metrics, log_matches_metrics,
             "RAY_CAPTURE_JSON exactly matches metrics.json")
    add_gate(gates, "debug_clean_shutdown_log", not failures, failures[:5], [])
    add_gate(gates, "debug_no_vma_leaks", vma_leak_bytes in (None, 0), vma_leak_bytes,
             "vma-leaks.log absent or empty")
    add_gate(gates, "debug_capture_identity",
             [metrics.get("schema"), metrics.get("type"), metrics.get("renderer"), metrics.get("demo_profile"),
              metrics.get("render_mode"), metrics.get("ray_debug_view"), metrics.get("output_format")] ==
             [2, "evoengine_ray_capture", "EvoEngine", "bistro", "RayTracing", "Beauty", "radiance_hdr_linear"],
             [metrics.get("schema"), metrics.get("type"), metrics.get("renderer"), metrics.get("demo_profile"),
              metrics.get("render_mode"), metrics.get("ray_debug_view"), metrics.get("output_format")],
             "Debug Bistro RTX linear Beauty capture")
    add_gate(gates, "debug_profile",
             [metrics.get("width"), metrics.get("height"), metrics.get("requested_frames"),
              metrics.get("timing_warmup_frames"), metrics.get("measured_frames"), metrics.get("samples_per_frame"),
              metrics.get("measured_spp"), metrics.get("effective_spp"), image_dimensions] ==
             [640, 360, 16, 1, 15, 4, 60, 64, [640, 360]],
             [metrics.get("width"), metrics.get("height"), metrics.get("requested_frames"),
              metrics.get("timing_warmup_frames"), metrics.get("measured_frames"), metrics.get("samples_per_frame"),
              metrics.get("measured_spp"), metrics.get("effective_spp"), image_dimensions],
             [640, 360, 16, 1, 15, 4, 60, 64, [640, 360]])
    add_gate(gates, "debug_finite_nonblank_image", image_finite and image_nonblank,
             {"finite": image_finite, "nonblank": image_nonblank}, {"finite": True, "nonblank": True})
    add_gate(gates, "debug_validation_layers", frame_sync.get("validation_layers_enabled") is True,
             frame_sync.get("validation_layers_enabled"), True)
    add_gate(gates, "debug_capture_drained", frame_sync.get("pending_submissions_after_capture_flush") == 0,
             frame_sync, "pending=0")
    add_gate(gates, "debug_no_forbidden_waits",
             waits.get("just_submitted_frame", {}).get("count") == 0 and
             waits.get("redundant", {}).get("count") == 0,
             waits, "zero just-submitted/redundant")
    return gates


def synthetic_metrics(lane: str) -> dict[str, Any]:
    path_name = "Path Trace (RTX)" if lane == "rtx" else "Path Trace (RQ)"
    is_rtx = lane == "rtx"
    return {
        "schema": 2, "type": "evoengine_ray_capture", "renderer": "EvoEngine", "demo_profile": "bistro",
        "ray_debug_view": "Beauty", "output_format": "radiance_hdr_linear", "width": 1280, "height": 720,
        "requested_frames": 128, "timing_warmup_frames": 1, "measured_frames": 127, "samples_per_frame": 4,
        "measured_spp": 508, "effective_spp": 512, "deterministic": True, "ser_enabled": False,
        "ser_mode_requested": "disabled", "auto_spp_enabled": False, "temporal_motion_capture": False,
        "render_mode": "RayTracing" if is_rtx else "RayQuery", "active_ray_backend": "ray-tracing-pipeline" if is_rtx else "ray-query-compute",
        "query_only": not is_rtx, "ray_pipeline_max_recursion_depth": 1 if is_rtx else None,
        "capabilities": {"acceleration_structure": True, "ray_query": True, "ray_tracing_pipeline": is_rtx},
        "accumulation_wall_seconds": 8.8, "gpu_timestamps_available": True,
        "ray_shader_variant": {"active_mask": 8193, "requested_mask": 8193, "active_key": "rtx:0x2001" if is_rtx else "rq:0x2001",
                               "requested_key": "rtx:0x2001" if is_rtx else "rq:0x2001", "selection_mode": "auto",
                               "ready": True, "pending": False, "fallback_active": False},
        "gpu_sections": [{"name": path_name, "sample_count": 127, "median_ms": 58.0, "total_ms": 7366.0}],
        "cpu_sections": [{"name": "Queue Submit", "sample_count": 127, "median_ms": 0.09}],
        "synchronization_waits": {"just_submitted_frame": {"count": 0}, "redundant": {"count": 0}},
        "frame_synchronization": {"policy": "wait-on-frame-slot-reuse", "max_frames_in_flight": 2,
                                  "pending_submissions_after_capture_flush": 0, "validation_layers_enabled": True},
        "ray_camera_frame_path": {
            "capture": {"plan_compilations": 0, "plan_cache_misses": 0, "plan_cache_hits": 127,
                        "output_descriptor_creations": 0, "output_descriptor_reuses": 127},
            "final": {"render_graph_plan_cache": {"entry_count": 1, "capacity": 16},
                      "live_output_descriptor_count": 2, "peak_live_output_descriptor_count": 2,
                      "retained_frame_slot_count": 2}},
        "ray_camera_history": {"ownership": "camera-owned-single-slot", "maximum_histories_per_camera": 1,
                               "capture_creation_count": 0, "final": {"live_history_count": 1}},
        "gpu_memory": {"startup_ready": {"allocation_count": 100}, "final": {"allocation_count": 100}},
    }


def self_test() -> None:
    for lane in ("rtx", "query-only"):
        metrics = synthetic_metrics(lane)
        if failed := [gate for gate in validate_lane(lane, metrics) if not gate["passed"]]:
            raise AssertionError(failed)
        broken = json.loads(json.dumps(metrics))
        del broken["accumulation_wall_seconds"]
        if all(gate["passed"] for gate in validate_lane(lane, broken)):
            raise AssertionError("M16 validator accepted missing wall timing")
        broken = json.loads(json.dumps(metrics))
        broken["gpu_memory"] = {}
        if all(gate["passed"] for gate in validate_lane(lane, broken)):
            raise AssertionError("M16 validator accepted missing VMA telemetry")
        broken = json.loads(json.dumps(metrics))
        broken["ray_camera_frame_path"]["final"]["render_graph_plan_cache"]["capacity"] = 8
        if all(gate["passed"] for gate in validate_lane(lane, broken)):
            raise AssertionError("M16 validator accepted the wrong plan-cache capacity")
        if lane == "query-only":
            broken = json.loads(json.dumps(metrics))
            broken["accumulation_wall_seconds"] = 12.0
            if all(gate["passed"] for gate in validate_lane(lane, broken)):
                raise AssertionError("M16 validator accepted regressed query-only wall throughput")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rtx-record", type=Path)
    parser.add_argument("--query-only-record", type=Path)
    parser.add_argument("--rtx-provenance", type=Path)
    parser.add_argument("--query-only-provenance", type=Path)
    parser.add_argument("--debug-metrics", type=Path)
    parser.add_argument("--debug-log", type=Path)
    parser.add_argument("--debug-executable", type=Path)
    parser.add_argument("--debug-exit-code-file", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        print("M16 validator self-test passed.")
        return 0
    required = (args.rtx_record, args.query_only_record, args.rtx_provenance, args.query_only_provenance,
                args.debug_metrics, args.debug_log, args.debug_executable, args.debug_exit_code_file, args.output)
    if any(value is None for value in required):
        parser.error("all record, provenance, Debug, and output arguments are required")
    baseline_hash = sha256(M11_REPORT)
    if baseline_hash != M11_REPORT_SHA256:
        raise RuntimeError(f"M11 report hash mismatch: {baseline_hash}")
    baseline = load_json(M11_REPORT)
    expected_gpu = baseline.get("approved_m6", {}).get("gpu", {})
    m12_baselines: dict[str, dict[str, Any]] = {}
    for lane, record_path in M12_BASELINE_RECORDS.items():
        record_hash = sha256(record_path)
        if record_hash != M12_BASELINE_RECORD_SHA256[lane]:
            raise RuntimeError(f"M12 {lane} record hash mismatch: {record_hash}")
        record = load_json(record_path)
        metrics_path = Path(str(record.get("metrics_path", "")))
        metrics_hash = optional_sha256(metrics_path)
        if metrics_hash != record.get("metrics_sha256"):
            raise RuntimeError(f"M12 {lane} metrics hash mismatch: {metrics_hash}")
        metrics = load_json(metrics_path)
        path_name = "Path Trace (RTX)" if lane == "rtx" else "Path Trace (RQ)"
        path_median = number(timing(metrics, "gpu_sections", path_name).get("median_ms"))
        if path_median != M12_PATH_MEDIANS[lane]:
            raise RuntimeError(f"M12 {lane} path median mismatch: {path_median}")
        wall_seconds = number(metrics.get("accumulation_wall_seconds"))
        measured_frames = metrics.get("measured_frames")
        wall_ms_per_frame = (wall_seconds * 1000.0 / measured_frames
                             if wall_seconds is not None and isinstance(measured_frames, int) and measured_frames > 0
                             else None)
        if lane == "query-only" and (wall_ms_per_frame is None or
                                     not math.isclose(wall_ms_per_frame, M12_QUERY_WALL_MS_PER_FRAME,
                                                      rel_tol=0.0, abs_tol=1e-12)):
            raise RuntimeError(f"M12 query-only wall time mismatch: {wall_ms_per_frame}")
        m12_baselines[lane] = {
            "record": str(record_path.resolve()),
            "record_sha256": record_hash,
            "metrics": str(metrics_path.resolve()),
            "metrics_sha256": metrics_hash,
            "path_median_ms": path_median,
            "path_ceiling_ms": RTX_PATH_MEDIAN_CEILING if lane == "rtx" else None,
            "wall_ms_per_frame": wall_ms_per_frame,
            "wall_ceiling_ms_per_frame": QUERY_WALL_MS_PER_FRAME_CEILING if lane == "query-only" else None,
        }
    rtx_record = load_json(args.rtx_record)
    query_record = load_json(args.query_only_record)
    rtx_provenance = load_json(args.rtx_provenance)
    query_provenance = load_json(args.query_only_provenance)
    gates, rtx_metrics, rtx_image = validate_record("rtx", rtx_record, rtx_provenance)
    query_gates, query_metrics, query_image = validate_record("query-only", query_record, query_provenance)
    gates.extend(query_gates)
    add_gate(gates, "matched_gpu_driver", gpu_identity(rtx_metrics) == expected_gpu and
             gpu_identity(query_metrics) == expected_gpu,
             {"rtx": gpu_identity(rtx_metrics), "query-only": gpu_identity(query_metrics)}, expected_gpu)
    rtx_binary = rtx_provenance.get("binaries", {}).get("evoengine", {}).get("combined_sha256")
    query_binary = query_provenance.get("binaries", {}).get("evoengine", {}).get("combined_sha256")
    rtx_repo = rtx_provenance.get("repositories", {}).get("evoengine", {})
    query_repo = query_provenance.get("repositories", {}).get("evoengine", {})
    add_gate(gates, "matched_source_and_binary",
             bool(rtx_binary) and rtx_binary == query_binary and rtx_repo.get("head") == query_repo.get("head") and
             rtx_repo.get("working_diff_sha256") == query_repo.get("working_diff_sha256"),
             {"rtx_binary": rtx_binary, "query_binary": query_binary, "rtx_repository": rtx_repo,
              "query_repository": query_repo}, "identical binary closure and source state")

    debug_metrics = load_json(args.debug_metrics)
    debug_image = Path(str(debug_metrics.get("output_path", "")))
    debug_dimensions = None
    debug_image_finite = False
    debug_image_nonblank = False
    if debug_image.is_file():
        pixels = read_image(debug_image)
        debug_dimensions = [pixels.width, pixels.height]
        values = getattr(pixels, "rgb", ())
        debug_image_finite = bool(values) and all(math.isfinite(value) for value in values)
        debug_image_nonblank = any(value != 0.0 for value in values)
    debug_root = args.debug_metrics.resolve().parent
    debug_artifacts_bound = all(path.resolve().parent == debug_root for path in
                                (args.debug_log, args.debug_exit_code_file, debug_image))
    debug_exit_code = int(args.debug_exit_code_file.read_text(encoding="utf-8").strip())
    debug_log_metrics = capture_json_from_log(args.debug_log)
    vma_leak_log = debug_root / "vma-leaks.log"
    vma_leak_bytes = vma_leak_log.stat().st_size if vma_leak_log.is_file() else None
    debug_failures = log_failures(args.debug_log)
    gates.extend(validate_debug(debug_metrics, debug_exit_code, args.debug_executable, debug_failures,
                                debug_dimensions, debug_artifacts_bound, debug_log_metrics == debug_metrics,
                                vma_leak_bytes, debug_image_finite, debug_image_nonblank))
    add_gate(gates, "debug_gpu_driver", gpu_identity(debug_metrics) == expected_gpu,
             gpu_identity(debug_metrics), expected_gpu)
    comparison = compare_render_images(read_image(rtx_image), read_image(query_image), True)
    add_gate(gates, "rtx_query_only_relative_l2", float(comparison["relative_l2_error"]) <= 0.010,
             comparison["relative_l2_error"], "<= 0.010")
    report = {
        "schema": 1,
        "type": "raytracer_m16_validation",
        "pass": all(gate["passed"] for gate in gates),
        "gate_count": len(gates),
        "m11_report": str(M11_REPORT.resolve()),
        "m11_report_sha256": baseline_hash,
        "m12_performance_baselines": m12_baselines,
        "records": {"rtx": {"path": str(args.rtx_record.resolve()), "sha256": sha256(args.rtx_record)},
                    "query-only": {"path": str(args.query_only_record.resolve()),
                                   "sha256": sha256(args.query_only_record)}},
        "provenance": {"rtx": {"path": str(args.rtx_provenance.resolve()),
                                 "sha256": sha256(args.rtx_provenance)},
                       "query-only": {"path": str(args.query_only_provenance.resolve()),
                                      "sha256": sha256(args.query_only_provenance)}},
        "debug": {"metrics": str(args.debug_metrics.resolve()), "metrics_sha256": sha256(args.debug_metrics),
                  "log": str(args.debug_log.resolve()), "log_sha256": sha256(args.debug_log),
                  "executable": str(args.debug_executable.resolve()),
                  "executable_sha256": sha256(args.debug_executable),
                  "exit_code_file": str(args.debug_exit_code_file.resolve()),
                  "exit_code_file_sha256": sha256(args.debug_exit_code_file), "exit_code": debug_exit_code,
                  "vma_leak_log": str(vma_leak_log.resolve()) if vma_leak_log.is_file() else None,
                  "vma_leak_bytes": vma_leak_bytes,
                  "image": str(debug_image.resolve()), "image_sha256": sha256(debug_image)},
        "image_comparison": comparison,
        "gates": gates,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    failed = [gate for gate in gates if not gate["passed"]]
    print(json.dumps({"pass": report["pass"], "gate_count": len(gates), "failed": failed}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
