#!/usr/bin/env python3
"""Capture and validate M16b camera-owned post-processing resources."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from compare_reference_render import read_png


ROOT = Path(__file__).resolve().parents[1]
SIZE = (640, 360)
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
FORBIDDEN_WAITS = (
    "Required TAA History Resize Fence Wait",
    "Required SMAA Resize Fence Wait",
    "Required Post-Processing Resize Fence Wait",
    "Required Post-Processing Pipeline Rebuild Fence Wait",
)


@dataclass(frozen=True)
class Lane:
    name: str
    configuration: str
    editor: Path
    validation_layers: bool


def add_gate(gates: list[dict[str, Any]], name: str, passed: bool, actual: Any, expected: Any) -> None:
    gates.append({"name": name, "passed": bool(passed), "actual": actual, "expected": expected})


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object: {path}")
    return value


def capture_json_from_log(path: Path) -> dict[str, Any] | None:
    prefix = "RAY_CAPTURE_JSON "
    if not path.is_file():
        return None
    for line in reversed(path.read_text(encoding="utf-8", errors="replace").splitlines()):
        if line.startswith(prefix):
            value = json.loads(line[len(prefix):])
            return value if isinstance(value, dict) else None
    return None


def log_failures(path: Path) -> list[str]:
    if not path.is_file():
        return ["missing log"]
    return [
        line
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
        if any(pattern.search(line) for pattern in ERROR_PATTERNS)
    ]


def lane_paths(root: Path, lane: Lane) -> dict[str, Path]:
    directory = root / lane.name
    return {
        "directory": directory,
        "image": directory / "capture.png",
        "metrics": directory / "metrics.json",
        "log": directory / "run.log",
        "exit_code": directory / "exit-code.txt",
        "provenance": directory / "provenance.json",
        "imgui": directory / "imgui.ini",
        "shader_cache": directory / "ShaderBinaries",
        "pipeline_cache": directory / "PipelineCache",
        "vma": directory / "vma-leaks.log",
    }


def capture_command(lane: Lane, paths: dict[str, Path]) -> list[str]:
    return [
        str(lane.editor.resolve()),
        "--demo", "rendering-regression",
        "--editor",
        "--capture-demo-preview", str(paths["image"].resolve()),
        "--preview-metrics-json", str(paths["metrics"].resolve()),
        "--preview-render-mode", "rasterization",
        "--preview-width", str(SIZE[0]),
        "--preview-height", str(SIZE[1]),
        "--preview-warmup-frames", "8",
        "--preview-timing-warmup-frames", "1",
        "--preview-aa", "taa",
        "--preview-aa-preset", "best-quality",
        "--preview-deterministic",
        "--preview-post-processing-stress",
    ]


def lane_environment(lane: Lane, paths: dict[str, Path]) -> dict[str, str]:
    environment = os.environ.copy()
    environment["EVOENGINE_SHADER_CACHE_DIR"] = str(paths["shader_cache"].resolve())
    environment["EVOENGINE_PIPELINE_CACHE_DIR"] = str(paths["pipeline_cache"].resolve())
    environment["EVOENGINE_IMGUI_INI_PATH"] = str(paths["imgui"].resolve())
    if lane.validation_layers:
        environment["EVOENGINE_VMA_LEAK_LOG"] = str(paths["vma"].resolve())
    else:
        environment.pop("EVOENGINE_VMA_LEAK_LOG", None)
    return environment


def write_launch_ledger(root: Path, launches: list[dict[str, Any]]) -> None:
    (root / "launch-ledger.json").write_text(
        json.dumps({"schema": 1, "renderer_launch_count": len(launches), "launches": launches}, indent=2) + "\n",
        encoding="utf-8",
    )


def run_lane(lane: Lane, root: Path, timeout_seconds: int, launches: list[dict[str, Any]]) -> None:
    paths = lane_paths(root, lane)
    paths["directory"].mkdir(parents=True)
    command = capture_command(lane, paths)
    provenance = {
        "lane": lane.name,
        "configuration": lane.configuration,
        "executable": str(lane.editor.resolve()),
        "executable_sha256": sha256(lane.editor),
        "command": command,
    }
    paths["provenance"].write_text(json.dumps(provenance, indent=2) + "\n", encoding="utf-8")
    return_code = -1
    try:
        with paths["log"].open("w", encoding="utf-8") as log:
            result = subprocess.run(
                command,
                cwd=ROOT,
                env=lane_environment(lane, paths),
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_seconds,
                check=False,
            )
        return_code = result.returncode
    except subprocess.TimeoutExpired as error:
        with paths["log"].open("a", encoding="utf-8") as log:
            log.write(f"\nM16b validator timeout: {error}\n")
        return_code = -2
    paths["exit_code"].write_text(f"{return_code}\n", encoding="utf-8")
    launches.append({"lane": lane.name, "configuration": lane.configuration, "exit_code": return_code})
    write_launch_ledger(root, launches)


def runtime(stress: dict[str, Any], snapshot: str, camera: str) -> dict[str, Any]:
    value = stress.get(snapshot, {})
    camera_value = value.get(camera, {}) if isinstance(value, dict) else {}
    runtime_value = camera_value.get("runtime", {}) if isinstance(camera_value, dict) else {}
    return runtime_value if isinstance(runtime_value, dict) else {}


def camera_snapshot(stress: dict[str, Any], snapshot: str, camera: str) -> dict[str, Any]:
    value = stress.get(snapshot, {})
    camera_value = value.get(camera, {}) if isinstance(value, dict) else {}
    return camera_value if isinstance(camera_value, dict) else {}


def disjoint_nonzero(first: list[Any], second: list[Any]) -> bool:
    return bool(first) and bool(second) and all(isinstance(value, int) and value != 0 for value in first + second) and not (
        set(first) & set(second)
    )


def scratch(runtime_value: dict[str, Any]) -> list[Any]:
    return [runtime_value.get("source_texture"), runtime_value.get("result_texture"), runtime_value.get("swap_texture")]


def validate_stress(stress: dict[str, Any]) -> list[dict[str, Any]]:
    gates: list[dict[str, Any]] = []
    base_a = runtime(stress, "taa_baseline", "a")
    base_b = runtime(stress, "taa_baseline", "b")
    only_a = runtime(stress, "after_a_only", "a")
    only_b = runtime(stress, "after_a_only", "b")
    version_a = runtime(stress, "after_version_a", "a")
    version_b_pending = runtime(stress, "after_version_a", "b")
    version_b = runtime(stress, "after_version_b", "b")
    resized_a = runtime(stress, "after_resize_a", "a")
    resized_b = runtime(stress, "after_resize_a", "b")
    query_b = runtime(stress, "after_technique_b_ray_query", "b")
    raster_b = runtime(stress, "after_technique_b_rasterization", "b")
    technique_a = runtime(stress, "after_technique_b_rasterization", "a")

    add_gate(gates, "stress_reported_pass",
             stress.get("pass") is True and stress.get("failures") == [],
             {"pass": stress.get("pass"), "failures": stress.get("failures")},
             {"pass": True, "failures": []})
    add_gate(gates, "different_camera_sizes",
             base_a.get("scratch_size") == [640, 360] and base_b.get("scratch_size") == [320, 180],
             [base_a.get("scratch_size"), base_b.get("scratch_size")], [[640, 360], [320, 180]])
    add_gate(gates, "shared_stack_distinct_cameras",
             isinstance(base_a.get("stack_handle"), int) and base_a.get("stack_handle", 0) != 0 and
             base_a.get("stack_handle") == base_b.get("stack_handle") and
             camera_snapshot(stress, "taa_baseline", "a").get("camera_handle") !=
             camera_snapshot(stress, "taa_baseline", "b").get("camera_handle"),
             [base_a.get("stack_handle"), base_b.get("stack_handle")], "same nonzero stack; distinct cameras")
    add_gate(gates, "same_frame_independent_temporal_state",
             base_a.get("taa_history_valid") is True and base_b.get("taa_history_valid") is True and
             base_a.get("auto_exposure_process_count", 0) > 0 and
             base_b.get("auto_exposure_process_count", 0) > 0 and
             base_a.get("luminance_reset_pending") is False and
             base_b.get("luminance_reset_pending") is False and
             base_a.get("taa_last_processed_frame") == base_b.get("taa_last_processed_frame"),
             [base_a.get("taa_last_processed_frame"), base_b.get("taa_last_processed_frame")], "same frame")
    add_gate(gates, "scratch_nonalias", disjoint_nonzero(scratch(base_a), scratch(base_b)),
             [scratch(base_a), scratch(base_b)], "disjoint nonzero")
    add_gate(gates, "taa_nonalias",
             disjoint_nonzero(base_a.get("taa_color_textures", []) + base_a.get("taa_depth_textures", []),
                              base_b.get("taa_color_textures", []) + base_b.get("taa_depth_textures", [])),
             [base_a.get("taa_color_textures"), base_b.get("taa_color_textures")], "disjoint nonzero")
    add_gate(gates, "exposure_nonalias",
             disjoint_nonzero([base_a.get("histogram_buffer"), base_a.get("luminance_buffer")],
                              [base_b.get("histogram_buffer"), base_b.get("luminance_buffer")]),
             [base_a.get("histogram_buffer"), base_b.get("histogram_buffer")], "disjoint nonzero")
    add_gate(gates, "descriptor_nonalias",
             disjoint_nonzero(base_a.get("descriptor_sets", []), base_b.get("descriptor_sets", [])),
             [len(base_a.get("descriptor_sets", [])), len(base_b.get("descriptor_sets", []))], "disjoint nonzero")
    add_gate(gates, "a_only_independence",
             only_b == base_b and only_a.get("taa_last_processed_frame", 0) > base_a.get("taa_last_processed_frame", 0) and
             only_a.get("auto_exposure_process_count", 0) > base_a.get("auto_exposure_process_count", 0),
             {"a": only_a, "b_unchanged": only_b == base_b}, "A advances; B unchanged")
    add_gate(gates, "single_version_increment",
             stress.get("stack_version_after_mutation") == stress.get("stack_version_before_mutation", -2) + 1,
             [stress.get("stack_version_before_mutation"), stress.get("stack_version_after_mutation")], "+1")
    add_gate(gates, "lazy_version_observation",
             version_a.get("stack_version") == stress.get("stack_version_after_mutation") and
             version_b_pending.get("stack_version") == stress.get("stack_version_before_mutation"),
             [version_a.get("stack_version"), version_b_pending.get("stack_version")], "new, old")
    add_gate(gates, "version_reset_preserves_scratch_a",
             version_a.get("version_reset_count") == only_a.get("version_reset_count", -1) + 1 and
             version_a.get("auto_exposure_reset_count") == only_a.get("auto_exposure_reset_count", -1) + 1 and
             version_a.get("temporal_reset_count") == only_a.get("temporal_reset_count", -1) + 1 and
             version_a.get("scratch_generation") == only_a.get("scratch_generation") and scratch(version_a) == scratch(only_a),
             version_a, "one temporal reset; same compatible scratch")
    add_gate(gates, "version_reset_preserves_scratch_b",
             version_b.get("version_reset_count") == only_b.get("version_reset_count", -1) + 1 and
             version_b.get("auto_exposure_reset_count") == only_b.get("auto_exposure_reset_count", -1) + 1 and
             version_b.get("temporal_reset_count") == only_b.get("temporal_reset_count", -1) + 1 and
             version_b.get("scratch_generation") == only_b.get("scratch_generation") and scratch(version_b) == scratch(only_b),
             version_b, "one temporal reset; same compatible scratch")
    add_gate(gates, "smaa_nonalias",
             disjoint_nonzero([version_a.get("smaa_edges_texture"), version_a.get("smaa_blend_texture")],
                              [version_b.get("smaa_edges_texture"), version_b.get("smaa_blend_texture")]),
             [version_a.get("smaa_edges_texture"), version_b.get("smaa_edges_texture")], "disjoint nonzero")
    add_gate(gates, "resolution_invalidation_isolated",
             resized_a.get("scratch_size") == [480, 270] and
             resized_a.get("scratch_generation") == version_a.get("scratch_generation", -1) + 1 and
             resized_a.get("resolution_reset_count") == version_a.get("resolution_reset_count", -1) + 1 and
             resized_a.get("version_reset_count") == version_a.get("version_reset_count") and
             resized_a.get("technique_reset_count") == version_a.get("technique_reset_count") and
             resized_a.get("temporal_reset_count") == version_a.get("temporal_reset_count", -1) + 1 and
             resized_a.get("auto_exposure_reset_count") == version_a.get("auto_exposure_reset_count", -1) + 1 and
             scratch(resized_a) != scratch(version_a) and resized_b == version_b,
             resized_a, "A resolution only; B unchanged")
    add_gate(gates, "technique_invalidation_isolated",
             query_b.get("render_technique") == 2 and raster_b.get("render_technique") == 0 and
             query_b.get("technique_reset_count") == resized_b.get("technique_reset_count", -1) + 1 and
             raster_b.get("technique_reset_count") == query_b.get("technique_reset_count", -1) + 1 and
             query_b.get("scratch_generation") == resized_b.get("scratch_generation") == raster_b.get("scratch_generation") and
             query_b.get("resolution_reset_count") == resized_b.get("resolution_reset_count") == raster_b.get("resolution_reset_count") and
             query_b.get("version_reset_count") == resized_b.get("version_reset_count") == raster_b.get("version_reset_count") and
             query_b.get("temporal_reset_count") == resized_b.get("temporal_reset_count", -1) + 1 and
             raster_b.get("temporal_reset_count") == query_b.get("temporal_reset_count", -1) + 1 and
             query_b.get("auto_exposure_reset_count") == resized_b.get("auto_exposure_reset_count", -1) + 1 and
             raster_b.get("auto_exposure_reset_count") == query_b.get("auto_exposure_reset_count", -1) + 1 and
             technique_a == resized_a,
             {"query": query_b, "raster": raster_b}, "two technique-only resets; A unchanged")
    churn = stress.get("churn", {}) if isinstance(stress.get("churn"), dict) else {}
    baseline_memory = churn.get("baseline_memory", {}) if isinstance(churn.get("baseline_memory"), dict) else {}
    final_memory = churn.get("final_memory", {}) if isinstance(churn.get("final_memory"), dict) else {}
    baseline_descriptors = churn.get("baseline_descriptor_sets", {}) if isinstance(churn.get("baseline_descriptor_sets"), dict) else {}
    final_descriptors = churn.get("final_descriptor_sets", {}) if isinstance(churn.get("final_descriptor_sets"), dict) else {}
    first_active_memory = churn.get("first_active_memory", {}) if isinstance(churn.get("first_active_memory"), dict) else {}
    cycles = churn.get("cycles", []) if isinstance(churn.get("cycles"), list) else []
    cycles_bounded = len(cycles) == 4 and all(
        isinstance(cycle, dict) and isinstance(cycle.get("memory"), dict) and
        isinstance(cycle.get("descriptor_sets"), dict) and
        cycle["memory"].get("allocation_count", float("inf")) <= first_active_memory.get("allocation_count", -1) and
        cycle["memory"].get("allocation_bytes", float("inf")) <= first_active_memory.get("allocation_bytes", -1) and
        cycle["descriptor_sets"].get("live_count", float("inf")) <= churn.get("first_active_descriptor_count", -1)
        for cycle in cycles
    )
    add_gate(gates, "bounded_churn",
             churn.get("iterations") == 4 and cycles_bounded and
             baseline_memory.get("allocation_count") == final_memory.get("allocation_count") and
             baseline_memory.get("allocation_bytes") == final_memory.get("allocation_bytes") and
             baseline_descriptors.get("live_count") == final_descriptors.get("live_count"),
             churn, "four cycles return VMA and descriptors to baseline")
    return gates


def inspect_lane(lane: Lane, root: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    paths = lane_paths(root, lane)
    gates: list[dict[str, Any]] = []
    try:
        exit_code = int(paths["exit_code"].read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        exit_code = None
    add_gate(gates, f"{lane.name}_exit_code", exit_code == 0, exit_code, 0)
    provenance = load_json(paths["provenance"]) if paths["provenance"].is_file() else {}
    expected_editor = str(lane.editor.resolve())
    add_gate(gates, f"{lane.name}_executable",
             provenance.get("executable") == expected_editor and lane.editor.is_file() and
             provenance.get("executable_sha256") == sha256(lane.editor),
             provenance, {"executable": expected_editor, "configuration": lane.configuration})
    metrics = load_json(paths["metrics"]) if paths["metrics"].is_file() else {}
    embedded = capture_json_from_log(paths["log"])
    add_gate(gates, f"{lane.name}_log_metrics_identity", embedded == metrics and bool(metrics),
             embedded == metrics, True)
    failures = log_failures(paths["log"])
    add_gate(gates, f"{lane.name}_clean_log", not failures, failures, [])
    log_text = paths["log"].read_text(encoding="utf-8", errors="replace") if paths["log"].is_file() else ""
    found_waits = [label for label in FORBIDDEN_WAITS if label in log_text]
    add_gate(gates, f"{lane.name}_no_removed_waits", not found_waits, found_waits, [])
    identity = [metrics.get(key) for key in ("schema", "type", "renderer", "demo_profile", "render_mode", "output_format")]
    add_gate(gates, f"{lane.name}_capture_identity",
             identity == [2, "evoengine_ray_capture", "EvoEngine", "rendering-regression", "Rasterization", "png_display"],
             identity, [2, "evoengine_ray_capture", "EvoEngine", "rendering-regression", "Rasterization", "png_display"])
    add_gate(gates, f"{lane.name}_metrics_dimensions", [metrics.get("width"), metrics.get("height")] == list(SIZE),
             [metrics.get("width"), metrics.get("height")], list(SIZE))
    output_path = Path(str(metrics.get("output_path", ""))).resolve() if metrics.get("output_path") else None
    add_gate(gates, f"{lane.name}_output_path", output_path == paths["image"].resolve(),
             str(output_path) if output_path else None, str(paths["image"].resolve()))
    try:
        image = read_png(paths["image"])
        rgb = [image.rgba[index] for index in range(len(image.rgba)) if index % 4 != 3]
        image_actual = [image.width, image.height, max(rgb) - min(rgb)]
        image_pass = [image.width, image.height] == list(SIZE) and bool(rgb) and image_actual[2] >= 32
    except (OSError, ValueError) as error:
        image_actual = str(error)
        image_pass = False
    add_gate(gates, f"{lane.name}_nonblank_png", image_pass, image_actual, [SIZE[0], SIZE[1], "RGB range >= 32"])
    frame_sync = metrics.get("frame_synchronization", {}) if isinstance(metrics.get("frame_synchronization"), dict) else {}
    add_gate(gates, f"{lane.name}_validation_and_drain",
             frame_sync.get("validation_layers_enabled") is lane.validation_layers and
             frame_sync.get("pending_submissions_after_capture_flush") == 0,
             frame_sync, {"validation_layers_enabled": lane.validation_layers, "pending_submissions_after_capture_flush": 0})
    waits = metrics.get("synchronization_waits", {}) if isinstance(metrics.get("synchronization_waits"), dict) else {}
    add_gate(gates, f"{lane.name}_zero_avoidable_waits",
             all(isinstance(waits.get(key), dict) and waits[key].get("count") == 0 for key in ("just_submitted_frame", "redundant")),
             waits, "zero just-submitted and redundant waits")
    stress = metrics.get("post_processing_stress", {}) if isinstance(metrics.get("post_processing_stress"), dict) else {}
    gates.extend({**gate, "name": f"{lane.name}_{gate['name']}"} for gate in validate_stress(stress))
    leak_bytes = paths["vma"].stat().st_size if paths["vma"].is_file() else None
    add_gate(gates, f"{lane.name}_vma_leak_log",
             not lane.validation_layers or leak_bytes in (None, 0), leak_bytes, "absent or empty")
    return gates, {"metrics": metrics, "provenance": provenance, "vma_leak_bytes": leak_bytes}


def validate(root: Path, lanes: list[Lane]) -> dict[str, Any]:
    gates: list[dict[str, Any]] = []
    evidence: dict[str, Any] = {}
    for lane in lanes:
        lane_gates, lane_evidence = inspect_lane(lane, root)
        gates.extend(lane_gates)
        evidence[lane.name] = lane_evidence
    ledger = load_json(root / "launch-ledger.json") if (root / "launch-ledger.json").is_file() else {}
    add_gate(gates, "exact_two_precommit_launches", ledger.get("renderer_launch_count") == 2,
             ledger.get("renderer_launch_count"), 2)
    return {
        "schema": 1,
        "type": "raytracer_m16b_validation",
        "pass": all(gate["passed"] for gate in gates),
        "gate_count": len(gates),
        "capture_accounting": {"relwithdebinfo": 1, "debug": 1, "precommit_executed": 2},
        "lanes": evidence,
        "gates": gates,
    }


def synthetic_runtime(offset: int, size: list[int]) -> dict[str, Any]:
    return {
        "scratch_size": size, "stack_handle": 9, "stack_version": 1, "render_technique": 0,
        "scratch_generation": 1, "source_texture": offset + 1, "result_texture": offset + 2,
        "swap_texture": offset + 3, "taa_color_textures": [offset + 4, offset + 5],
        "taa_depth_textures": [offset + 6, offset + 7], "smaa_edges_texture": 0, "smaa_blend_texture": 0,
        "histogram_buffer": offset + 8, "luminance_buffer": offset + 9, "taa_frame_index": 8,
        "taa_last_processed_frame": 40, "taa_history_valid": True, "auto_exposure_time_initialized": False,
        "luminance_reset_pending": False, "auto_exposure_process_count": 8, "auto_exposure_reset_count": 1,
        "temporal_reset_count": 1, "version_reset_count": 0, "resolution_reset_count": 0,
        "technique_reset_count": 0, "descriptor_sets": [offset + 10, offset + 11],
    }


def synthetic_stress() -> dict[str, Any]:
    a = synthetic_runtime(100, [640, 360])
    b = synthetic_runtime(200, [320, 180])
    result: dict[str, Any] = {"pass": True, "failures": [],
                              "stack_version_before_mutation": 1, "stack_version_after_mutation": 2}
    result["taa_baseline"] = {"a": {"camera_handle": 1, "runtime": copy.deepcopy(a)},
                              "b": {"camera_handle": 2, "runtime": copy.deepcopy(b)}}
    only_a = copy.deepcopy(a)
    only_a["taa_last_processed_frame"] += 1
    only_a["auto_exposure_process_count"] += 1
    result["after_a_only"] = {"a": {"runtime": only_a}, "b": {"runtime": copy.deepcopy(b)}}
    version_a = copy.deepcopy(only_a)
    version_a.update({"stack_version": 2, "version_reset_count": 1, "auto_exposure_reset_count": 2,
                      "temporal_reset_count": 2,
                      "smaa_edges_texture": 150, "smaa_blend_texture": 151})
    result["after_version_a"] = {"a": {"runtime": version_a}, "b": {"runtime": copy.deepcopy(b)}}
    version_b = copy.deepcopy(b)
    version_b.update({"stack_version": 2, "version_reset_count": 1, "auto_exposure_reset_count": 2,
                      "temporal_reset_count": 2,
                      "smaa_edges_texture": 250, "smaa_blend_texture": 251})
    result["after_version_b"] = {"a": {"runtime": version_a}, "b": {"runtime": version_b}}
    resized_a = copy.deepcopy(version_a)
    resized_a.update({"scratch_size": [480, 270], "scratch_generation": 2, "resolution_reset_count": 1,
                      "source_texture": 301, "result_texture": 302, "swap_texture": 303,
                      "temporal_reset_count": 3, "auto_exposure_reset_count": 3})
    result["after_resize_a"] = {"a": {"runtime": resized_a}, "b": {"runtime": copy.deepcopy(version_b)}}
    query_b = copy.deepcopy(version_b)
    query_b.update({"render_technique": 2, "technique_reset_count": 1,
                    "temporal_reset_count": 3, "auto_exposure_reset_count": 3})
    result["after_technique_b_ray_query"] = {"a": {"runtime": resized_a}, "b": {"runtime": query_b}}
    raster_b = copy.deepcopy(query_b)
    raster_b.update({"render_technique": 0, "technique_reset_count": 2,
                     "temporal_reset_count": 4, "auto_exposure_reset_count": 4})
    result["after_technique_b_rasterization"] = {"a": {"runtime": resized_a}, "b": {"runtime": raster_b}}
    result["churn"] = {
        "iterations": 4,
        "baseline_memory": {"allocation_count": 10, "allocation_bytes": 1000},
        "final_memory": {"allocation_count": 10, "allocation_bytes": 1000},
        "baseline_descriptor_sets": {"live_count": 20},
        "first_active_memory": {"allocation_count": 15, "allocation_bytes": 1500},
        "first_active_descriptor_count": 25,
        "cycles": [{"iteration": index,
                    "memory": {"allocation_count": 15, "allocation_bytes": 1500},
                    "descriptor_sets": {"live_count": 25}} for index in range(4)],
        "final_descriptor_sets": {"live_count": 20},
    }
    return result


def self_test() -> None:
    stress = synthetic_stress()
    failed = [gate for gate in validate_stress(stress) if not gate["passed"]]
    if failed:
        raise AssertionError(failed)
    for mutate in (
        lambda value: value["after_a_only"]["b"]["runtime"].update({"taa_frame_index": 99}),
        lambda value: value["after_version_a"]["a"]["runtime"].update({"source_texture": 999}),
        lambda value: value["after_resize_a"]["a"]["runtime"].update({"temporal_reset_count": 2}),
        lambda value: value["after_technique_b_ray_query"]["b"]["runtime"].update({"render_technique": 0}),
        lambda value: value["churn"]["cycles"][2]["memory"].update({"allocation_count": 16}),
        lambda value: value["churn"]["final_memory"].update({"allocation_count": 11}),
    ):
        broken = copy.deepcopy(stress)
        mutate(broken)
        if all(gate["passed"] for gate in validate_stress(broken)):
            raise AssertionError("M16b validator accepted mutated stress evidence")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rel-editor", type=Path,
                        default=Path("out/build/vs2026-x64/EvoEngine_App/RelWithDebInfo/EvoEngineEditor.exe"))
    parser.add_argument("--debug-editor", type=Path,
                        default=Path("out/build/vs2026-x64/EvoEngine_App/Debug/EvoEngineEditor.exe"))
    parser.add_argument("--output-dir", type=Path, default=Path("out/raytracer-m16b"))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--capture", action="store_true")
    mode.add_argument("--analyze-existing", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--timeout-seconds", type=int, default=900)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        print("M16b validator self-test passed.")
        return 0
    lanes = [
        Lane("relwithdebinfo", "RelWithDebInfo", args.rel_editor, False),
        Lane("debug", "Debug", args.debug_editor, True),
    ]
    if args.dry_run:
        for lane in lanes:
            print(json.dumps(capture_command(lane, lane_paths(args.output_dir, lane))))
        return 0
    if not args.capture and not args.analyze_existing:
        parser.error("choose --capture or --analyze-existing")
    if args.capture:
        missing = [str(lane.editor) for lane in lanes if not lane.editor.is_file()]
        if missing:
            parser.error(f"missing editor executable(s): {missing}")
        if args.output_dir.exists() and any(args.output_dir.iterdir()):
            parser.error(f"capture output must be fresh: {args.output_dir}")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        launches: list[dict[str, Any]] = []
        for lane in lanes:
            run_lane(lane, args.output_dir, args.timeout_seconds, launches)
    report = validate(args.output_dir, lanes)
    (args.output_dir / "validation.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    failed = [gate for gate in report["gates"] if not gate["passed"]]
    print(json.dumps({"pass": report["pass"], "gate_count": report["gate_count"], "failed": failed}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
