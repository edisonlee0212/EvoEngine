#!/usr/bin/env python3
"""Run the M6 manifest-driven ray-tracer benchmark and emit JSON/CSV evidence."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import shutil
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import run_raytracer_baseline as legacy
from compare_reference_render import compare_hdr_images, read_hdr


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SUITE = ROOT / "Scripts" / "raytracer_m6_suite.json"
M6_REFERENCE_PATCH = ROOT / "Scripts" / "reference_patches" / "vk_gltf_renderer_m6.patch"
PINNED_EVOENGINE_BASE = "83c2f8f4407161425f23c520081f4bd3705e6a51"
EXPECTED_REFERENCE_DIFF_SHA256 = "2af8ca7653977986b9994c1503ea3b3a843a3774ab402f20c506194fde8f4331"

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
    re.compile(r"\b(?:shader|pipeline).{0,48}\bfailed\b", re.IGNORECASE),
)


@dataclass(frozen=True)
class RunSpec:
    purpose: str
    profile: str
    suite: str
    scene: str
    camera: str
    renderer: str
    technique: str
    variant: str
    cache_state: str
    repetition: int
    width: int
    height: int
    spp: int
    samples_per_frame: int
    demo: str
    motion: bool = False

    @property
    def frames(self) -> int:
        if self.spp % self.samples_per_frame:
            raise ValueError(f"SPP must be divisible by samples per frame: {self.spp} / {self.samples_per_frame}")
        return self.spp // self.samples_per_frame

    @property
    def run_id(self) -> str:
        fields = (
            self.purpose,
            self.profile,
            self.suite,
            self.scene,
            self.camera,
            self.renderer,
            self.technique,
            self.variant,
            self.cache_state,
            f"r{self.repetition:02d}",
            f"{self.width}x{self.height}",
            f"{self.spp}spp",
        )
        return "--".join(fields).replace("_", "-")

    @property
    def cache_pair_id(self) -> str:
        fields = (
            self.profile,
            self.suite,
            self.scene,
            self.camera,
            self.renderer,
            self.technique,
            self.variant,
            f"r{self.repetition:02d}",
            f"{self.spp}spp",
        )
        return "--".join(fields).replace("_", "-")


def atomic_write_text(path: Path, contents: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(contents, encoding="utf-8", newline="\n")
    temporary.replace(path)


def write_json(path: Path, value: object) -> None:
    atomic_write_text(path, json.dumps(value, indent=2, sort_keys=True) + "\n")


def json_sha256(value: object) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def directory_manifest(root: Path) -> dict[str, object]:
    files = []
    combined = hashlib.sha256()
    if root.is_dir():
        for path in sorted((path for path in root.rglob("*") if path.is_file()), key=lambda item: item.as_posix()):
            relative = path.relative_to(root).as_posix()
            digest = legacy.sha256(path)
            files.append({"path": relative, "bytes": path.stat().st_size, "sha256": digest})
            combined.update(relative.encode("utf-8"))
            combined.update(b"\0")
            combined.update(bytes.fromhex(digest))
    return {"root": str(root.resolve()), "combined_sha256": combined.hexdigest(), "files": files}


def percentile(values: Iterable[float], fraction: float) -> float:
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return 0.0
    rank = min(max(fraction, 0.0), 1.0) * (len(ordered) - 1)
    lower = math.floor(rank)
    upper = math.ceil(rank)
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def load_suite(path: Path) -> dict[str, object]:
    suite = json.loads(path.read_text(encoding="utf-8"))
    if suite.get("schema") != 1:
        raise ValueError("M6 suite schema must be 1")
    compact = suite.get("validation_mode") == "compact"
    required_repetitions = 1 if compact else 5
    repetitions = int(suite.get("repetitions", 0))
    if repetitions < required_repetitions or (compact and repetitions != 1):
        raise ValueError(f"M6 requires at least {required_repetitions} repetition(s)")
    samples_per_frame = int(suite.get("samples_per_frame", 0))
    if samples_per_frame <= 0 or (compact and samples_per_frame != 4):
        raise ValueError("samples_per_frame must be positive")
    cameras = suite.get("cross_renderer", {}).get("cameras", [])
    required_camera_count = 1 if compact else 3
    if len(cameras) != required_camera_count or len({camera.get("id") for camera in cameras}) != required_camera_count:
        raise ValueError(f"The cross-renderer suite must contain {required_camera_count} uniquely named camera(s)")
    for camera in cameras:
        for field in ("reference_position", "reference_look_at", "evo_position", "evo_look_at"):
            value = camera.get(field)
            if not isinstance(value, list) or len(value) != 3 or not all(math.isfinite(float(item)) for item in value):
                raise ValueError(f"Camera {camera.get('id')} has invalid {field}")
    for profile in suite.get("profiles", {}).values():
        if int(profile.get("width", 0)) <= 0 or int(profile.get("height", 0)) <= 0:
            raise ValueError("Profile dimensions must be positive")
        all_spp = [*profile.get("candidate_spp", []), profile.get("truth_spp", 0)]
        if any(int(spp) <= 0 or int(spp) % samples_per_frame for spp in all_spp):
            raise ValueError("Every profile SPP must be positive and divisible by samples_per_frame")
        candidates = [int(spp) for spp in profile.get("candidate_spp", [])]
        if candidates != sorted(set(candidates)):
            raise ValueError("Candidate SPP checkpoints must be sorted and unique")
    if not compact and len(suite["profiles"]["measure"]["candidate_spp"]) < 2:
        raise ValueError("Time-to-quality measurement requires more than one SPP checkpoint")
    capture_renderers = [str(renderer) for renderer in suite.get("capture_renderers", ("reference", "evo"))]
    if not capture_renderers or len(set(capture_renderers)) != len(capture_renderers) or any(
        renderer not in ("reference", "evo") for renderer in capture_renderers
    ):
        raise ValueError("capture_renderers must contain unique reference/evo values")
    frozen_reference = suite.get("frozen_reference")
    frozen_records = frozen_reference.get("records", []) if isinstance(frozen_reference, dict) else []
    frozen_camera = frozen_reference.get("camera") if isinstance(frozen_reference, dict) else None
    if compact and (
        suite["profiles"]["measure"]["candidate_spp"] != [512]
        or int(suite["profiles"]["measure"]["width"]) != 1280
        or int(suite["profiles"]["measure"]["height"]) != 720
        or variant_ids(suite) != ["specialized"]
        or capture_renderers != ["evo"]
        or suite.get("cache_states") != ["cold"]
        or suite.get("evo_only")
        or suite.get("cross_renderer", {}).get("scene") != "bistro"
        or suite.get("cross_renderer", {}).get("techniques") != ["rtx", "rq", "query-only"]
        or cameras[0].get("id") != "overview"
        or not isinstance(frozen_reference, dict)
        or not isinstance(frozen_reference.get("root"), str)
        or not isinstance(frozen_reference.get("provenance_sha256"), str)
        or not isinstance(frozen_camera, dict)
        or any(
            frozen_camera.get(field) != cameras[0].get(field)
            for field in ("id", "reference_position", "reference_look_at", "evo_position", "evo_look_at")
        )
        or len(frozen_records) != 2
        or {record.get("technique") for record in frozen_records if isinstance(record, dict)} != {"rtx", "rq"}
        or any(
            not isinstance(record, dict)
            or not isinstance(record.get("path"), str)
            or not isinstance(record.get("sha256"), str)
            for record in frozen_records
        )
    ):
        raise ValueError("Compact M6 must use three fresh 512-SPP specialized cold Evo captures and two frozen references")
    quality = suite.get("quality", {})
    if quality.get("cache_pair_output_policy") != "diagnostic":
        raise ValueError("M6 quality.cache_pair_output_policy must be diagnostic")
    if float(quality.get("cache_pair_investigation_relative_l2_error", 0.0)) <= 0.0:
        raise ValueError("M6 quality.cache_pair_investigation_relative_l2_error must be positive")
    for scene in suite.get("evo_only", []):
        candidates = [int(spp) for spp in scene.get("candidate_spp", [])]
        if candidates and (
            candidates != sorted(set(candidates))
            or any(spp <= 0 or spp % samples_per_frame for spp in candidates)
        ):
            raise ValueError(f"Evo-only scene {scene.get('id')} has invalid candidate SPP checkpoints")
    return suite


def profile_values(suite: dict[str, object], profile_name: str) -> tuple[int, int, list[int], int, int]:
    profile = suite["profiles"][profile_name]
    return (
        int(profile["width"]),
        int(profile["height"]),
        [int(value) for value in profile["candidate_spp"]],
        int(profile["truth_spp"]),
        int(suite["samples_per_frame"]),
    )


def variant_ids(suite: dict[str, object]) -> list[str]:
    return [str(variant["id"]) for variant in suite["variants"]]


def expand_measure_specs(suite: dict[str, object], repetitions: int | None = None) -> list[RunSpec]:
    width, height, candidate_spp, _, samples_per_frame = profile_values(suite, "measure")
    repetition_count = repetitions or int(suite["repetitions"])
    specs: list[RunSpec] = []
    variants = variant_ids(suite)
    cache_states = [str(value) for value in suite["cache_states"]]
    cross = suite["cross_renderer"]
    renderers = [str(renderer) for renderer in suite.get("capture_renderers", ("reference", "evo"))]
    for camera in cross["cameras"]:
        camera_id = str(camera["id"])
        for renderer in renderers:
            for technique in cross["techniques"]:
                technique = str(technique)
                if renderer == "reference" and technique == "query-only":
                    continue
                for variant in variants:
                    for spp in candidate_spp:
                        for repetition in range(1, repetition_count + 1):
                            for cache_state in cache_states:
                                specs.append(
                                    RunSpec(
                                        "measure",
                                        "measure",
                                        "cross-renderer",
                                        str(cross["scene"]),
                                        camera_id,
                                        renderer,
                                        technique,
                                        variant,
                                        cache_state,
                                        repetition,
                                        width,
                                        height,
                                        spp,
                                        samples_per_frame,
                                        "bistro",
                                    )
                                )
    for scene in suite["evo_only"]:
        scene_candidate_spp = [int(value) for value in scene.get("candidate_spp", candidate_spp)]
        for technique in scene["techniques"]:
            for variant in variants:
                for spp in scene_candidate_spp:
                    for repetition in range(1, repetition_count + 1):
                        for cache_state in cache_states:
                            specs.append(
                                RunSpec(
                                    "measure",
                                    "measure",
                                    "evo-only",
                                    str(scene["id"]),
                                    "default",
                                    "evo",
                                    str(technique),
                                    variant,
                                    cache_state,
                                    repetition,
                                    width,
                                    height,
                                    spp,
                                    samples_per_frame,
                                    str(scene["demo"]),
                                    bool(scene["motion"]),
                                )
                            )
    return specs


def expand_canonical_specs(suite: dict[str, object]) -> list[RunSpec]:
    width, height, candidate_spp, _, samples_per_frame = profile_values(suite, "canonical")
    spp = candidate_spp[-1]
    camera = str(suite["quality"]["pilot_camera"])
    return [
        RunSpec(
            "canonical",
            "canonical",
            "cross-renderer",
            str(suite["cross_renderer"]["scene"]),
            camera,
            renderer,
            technique,
            "specialized",
            "cold",
            0,
            width,
            height,
            spp,
            samples_per_frame,
            "bistro",
        )
        for renderer in ("reference", "evo")
        for technique in ("rtx", "rq")
    ]


def expand_truth_specs(suite: dict[str, object], profile_name: str, low_truth: bool = False) -> list[RunSpec]:
    width, height, _, truth_spp, samples_per_frame = profile_values(suite, profile_name)
    if low_truth:
        truth_spp //= 2
    purpose = f"{profile_name}-truth" + ("-low" if low_truth else "")
    specs: list[RunSpec] = []
    cross = suite["cross_renderer"]
    cameras = cross["cameras"]
    if profile_name == "pilot":
        cameras = [camera for camera in cameras if camera["id"] == suite["quality"]["pilot_camera"]]
    for camera in cameras:
        for renderer in ("reference", "evo"):
            for technique in ("rtx", "rq"):
                specs.append(
                    RunSpec(
                        purpose,
                        profile_name,
                        "cross-renderer",
                        str(cross["scene"]),
                        str(camera["id"]),
                        renderer,
                        technique,
                        "specialized",
                        "cold",
                        0,
                        width,
                        height,
                        truth_spp,
                        samples_per_frame,
                        "bistro",
                    )
                )
    for scene in suite["evo_only"]:
        if not scene["quality"] or profile_name == "pilot":
            continue
        for technique in ("rtx", "rq"):
            specs.append(
                RunSpec(
                    purpose,
                    profile_name,
                    "evo-only",
                    str(scene["id"]),
                    "default",
                    "evo",
                    technique,
                    "specialized",
                    "cold",
                    0,
                    width,
                    height,
                    truth_spp,
                    samples_per_frame,
                    str(scene["demo"]),
                    False,
                )
            )
    return specs


def expand_pilot_specs(suite: dict[str, object]) -> list[RunSpec]:
    width, height, candidate_spp, _, samples_per_frame = profile_values(suite, "pilot")
    camera_id = str(suite["quality"]["pilot_camera"])
    specs: list[RunSpec] = []
    for renderer in ("reference", "evo"):
        for technique in ("rtx", "rq"):
            for spp in candidate_spp:
                for cache_state in ("cold", "warm"):
                    specs.append(
                        RunSpec(
                            "pilot",
                            "pilot",
                            "cross-renderer",
                            "bistro",
                            camera_id,
                            renderer,
                            technique,
                            "specialized",
                            cache_state,
                            0,
                            width,
                            height,
                            spp,
                            samples_per_frame,
                            "bistro",
                        )
                    )
    return specs


def vector_subtract(left: list[float], right: list[float]) -> list[float]:
    return [float(left[index]) - float(right[index]) for index in range(3)]


def vector_cross(left: list[float], right: list[float]) -> list[float]:
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def vector_normalize(value: list[float]) -> list[float]:
    length = math.sqrt(sum(component * component for component in value))
    if length <= 1.0e-12:
        raise ValueError("Camera direction is degenerate")
    return [component / length for component in value]


def matrix_to_quaternion(matrix: list[list[float]]) -> list[float]:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2][1] - matrix[1][2]) / scale
        y = (matrix[0][2] - matrix[2][0]) / scale
        z = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / scale
        x = 0.25 * scale
        y = (matrix[0][1] + matrix[1][0]) / scale
        z = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / scale
        x = (matrix[0][1] + matrix[1][0]) / scale
        y = 0.25 * scale
        z = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / scale
        x = (matrix[0][2] + matrix[2][0]) / scale
        y = (matrix[1][2] + matrix[2][1]) / scale
        z = 0.25 * scale
    quaternion = [x, y, z, w]
    length = math.sqrt(sum(component * component for component in quaternion))
    return [component / length for component in quaternion]


def look_at_quaternion(position: list[float], target: list[float]) -> list[float]:
    front = vector_normalize(vector_subtract(target, position))
    right = vector_normalize(vector_cross(front, [0.0, 1.0, 0.0]))
    up = vector_cross(right, front)
    return matrix_to_quaternion(
        [
            [right[0], up[0], -front[0]],
            [right[1], up[1], -front[1]],
            [right[2], up[2], -front[2]],
        ]
    )


def camera_by_id(suite: dict[str, object], camera_id: str) -> dict[str, object]:
    for camera in suite["cross_renderer"]["cameras"]:
        if camera["id"] == camera_id:
            return camera
    raise KeyError(f"Unknown camera: {camera_id}")


def variant_by_id(suite: dict[str, object], variant_id: str) -> dict[str, object]:
    for variant in suite["variants"]:
        if variant["id"] == variant_id:
            return variant
    raise KeyError(f"Unknown variant: {variant_id}")


def generate_reference_camera_asset(
    source_root: Path, camera: dict[str, object], output_root: Path
) -> dict[str, object]:
    source = source_root / "bistro.gltf"
    if legacy.sha256(source) != legacy.PINNED_BISTRO_GLTF_SHA256:
        raise RuntimeError("Pinned Bistro source glTF does not match the recorded input")
    document = json.loads(source.read_text(encoding="utf-8-sig"))
    camera_nodes = [node for node in document["nodes"] if node.get("camera") == 0]
    if len(camera_nodes) != 1:
        raise ValueError("Pinned Bistro must contain exactly one node for camera 0")
    position = [float(value) for value in camera["reference_position"]]
    look_at = [float(value) for value in camera["reference_look_at"]]
    camera_nodes[0]["translation"] = position
    camera_nodes[0]["rotation"] = look_at_quaternion(position, look_at)
    light = document["extensions"]["KHR_lights_punctual"]["lights"][0]
    if light.get("intensity") != 6830:
        raise ValueError("Pinned Bistro directional light intensity changed")
    light["intensity"] = 10
    output_root.mkdir(parents=True, exist_ok=True)
    output = output_root / f"bistro-m6-{camera['id']}.gltf"
    contents = json.dumps(document, indent=2, ensure_ascii=True).encode("utf-8")
    if not output.is_file() or output.read_bytes() != contents:
        output.write_bytes(contents)
    return {
        "camera": camera["id"],
        "source": str(source.resolve()),
        "source_sha256": legacy.sha256(source),
        "output": str(output.resolve()),
        "output_sha256": legacy.sha256(output),
        "position": position,
        "look_at": look_at,
        "rotation_xyzw": camera_nodes[0]["rotation"],
        "dependencies": legacy.gltf_dependency_manifest(output),
    }


def comparison_limit_exceedances(
    comparison: dict[str, object], limits: dict[str, float]
) -> dict[str, float]:
    return {field: float(comparison[field]) for field, limit in limits.items() if float(comparison[field]) > limit}


def filter_specs(specs: list[RunSpec], args: argparse.Namespace) -> list[RunSpec]:
    def selected(requested: str, actual: str) -> bool:
        return requested == "all" or requested == actual

    return [
        spec
        for spec in specs
        if selected(args.suite, spec.suite)
        and selected(args.camera, spec.camera)
        and selected(args.renderer, spec.renderer)
        and selected(args.technique, spec.technique)
        and selected(args.variant, spec.variant)
        and selected(args.cache_state, spec.cache_state)
    ]


def spec_paths(output_root: Path, spec: RunSpec) -> dict[str, Path]:
    directory = output_root / spec.profile / spec.purpose
    cache_root = output_root / "cache-pairs" / spec.cache_pair_id
    return {
        "output": directory / f"{spec.run_id}.hdr",
        "metrics": directory / f"{spec.run_id}.metrics.json",
        "log": directory / f"{spec.run_id}.log",
        "record": directory / f"{spec.run_id}.record.json",
        "imgui": directory / f"{spec.run_id}.imgui.ini",
        "cache_root": cache_root,
        "cache_prime": cache_root.with_name(cache_root.name + ".prime.json"),
    }


def find_record(records: list[dict[str, object]], record_type: str) -> dict[str, object] | None:
    return next((record for record in reversed(records) if record.get("type") == record_type), None)


def validate_run_log(path: Path, renderer: str) -> None:
    text = path.read_text(encoding="utf-8", errors="replace")
    failures = [line for line in text.splitlines() if any(pattern.search(line) for pattern in ERROR_PATTERNS)]
    if failures:
        raise RuntimeError(f"{path.name}: fatal/validation log diagnostics: {failures[:5]}")
    if renderer != "reference":
        return
    missing_png = set(re.findall(r"File not found: (.+?\.png)\s*$", text, flags=re.IGNORECASE | re.MULTILINE))
    fallback_png = set(
        re.findall(
            r'Image "(.+?\.png)" not found on disk; using ".+?\.dds" instead\.',
            text,
            flags=re.IGNORECASE,
        )
    )
    unresolved = sorted(missing_png - fallback_png)
    if unresolved:
        raise RuntimeError(f"{path.name}: reference PNG inputs lack DDS fallback: {unresolved[:5]}")


def vector_argument(value: list[float]) -> str:
    return ",".join(f"{float(component):.17g}" for component in value)


def evo_command(
    spec: RunSpec,
    suite: dict[str, object],
    editor: Path,
    paths: dict[str, Path],
) -> tuple[list[str], dict[str, str], dict[str, object]]:
    cache_root = paths["cache_root"]
    cache_root.mkdir(parents=True, exist_ok=True)
    shader_cache = cache_root / "ShaderBinaries"
    pipeline_cache = cache_root / "PipelineCache"
    environment = {
        "EVOENGINE_SHADER_CACHE_DIR": str(shader_cache.resolve()),
        "EVOENGINE_PIPELINE_CACHE_DIR": str(pipeline_cache.resolve()),
        "EVOENGINE_IMGUI_INI_PATH": str(paths["imgui"].resolve()),
    }
    render_mode = "raytracing" if spec.technique == "rtx" else "rayquery"
    command = [
        str(editor),
        "--demo",
        spec.demo,
        "--editor",
        "--capture-demo-preview",
        str(paths["output"]),
        "--preview-metrics-json",
        str(paths["metrics"]),
        "--preview-render-mode",
        render_mode,
        "--preview-warmup-frames",
        str(spec.frames),
        "--preview-timing-warmup-frames",
        "1",
        "--preview-sample-size",
        str(spec.samples_per_frame),
        "--preview-auto-spp",
        "disabled",
        "--preview-firefly-clamp",
        "enabled",
        "--preview-firefly-clamp-threshold",
        "10",
        "--preview-ser",
        "disabled",
        "--preview-ray-shader-variant",
        str(variant_by_id(suite, spec.variant)["evo"]),
        "--preview-width",
        str(spec.width),
        "--preview-height",
        str(spec.height),
        "--preview-deterministic",
    ]
    if spec.suite == "cross-renderer":
        camera = camera_by_id(suite, spec.camera)
        command.extend(("--preview-camera-position", vector_argument(camera["evo_position"])))
        command.extend(("--preview-camera-look-at", vector_argument(camera["evo_look_at"])))
    if spec.motion:
        command.extend(("--preview-aa", "taa", "--preview-aa-motion-sequence", "enabled"))
    if spec.technique == "query-only":
        command.append("--disable-ray-tracing-pipeline")
    return command, environment, {
        "frontend_cache": spec.cache_state,
        "application_pipeline_cache": str(pipeline_cache.resolve()),
        "opaque_driver_cache": "uncontrolled",
        "shader_cache": str(shader_cache.resolve()),
    }


def reference_command(
    spec: RunSpec,
    suite: dict[str, object],
    reference_exe: Path,
    reference_root: Path,
    reference_environment: Path,
    camera_assets: dict[str, dict[str, object]],
    paths: dict[str, Path],
) -> tuple[list[str], dict[str, str], dict[str, object]]:
    cache_root = paths["cache_root"]
    cache_root.mkdir(parents=True, exist_ok=True)
    pipeline_cache = cache_root / "pipeline_cache.bin"
    if spec.suite != "cross-renderer":
        raise ValueError("The reference renderer is only valid for the cross-renderer suite")
    camera_asset = Path(camera_assets[spec.camera]["output"])
    variant = variant_by_id(suite, spec.variant)
    command = [
        str(reference_exe),
        "--headless",
        "--size",
        str(spec.width),
        str(spec.height),
        "--scenefile",
        str(camera_asset),
        "--frames",
        str(spec.frames),
        "--maxFrames",
        str(spec.frames),
        "--ptSamples",
        str(spec.samples_per_frame),
        "--ptAdaptiveSampling",
        "0",
        "--ptTechnique",
        "1" if spec.technique == "rtx" else "0",
        "--ptSER",
        "0",
        "--ptMaxDepth",
        "5",
        "--ptFireflyClamp",
        "10",
        "--ptTexGradScale",
        "1",
        "--ptAperture",
        "0",
        "--renderSystem",
        "0",
        "--envSystem",
        "1",
        "--hdrfile",
        str(reference_environment),
        "--hdrEnvIntensity",
        "0",
        "--useSolidBackground",
        "--solidBackgroundColor",
        "0",
        "0",
        "0",
        "--gltfCamera",
        "0",
        "--wireframe",
        "0",
        "--optimalShader",
        str(variant["reference_optimal"]),
        "--pipelineCachePath",
        str(pipeline_cache.resolve()),
        "--output",
        str(paths["output"]),
    ]
    return command, {}, {
        "frontend_cache": "unsupported" if spec.variant == "specialized" else "embedded_spirv",
        "application_pipeline_cache": spec.cache_state,
        "opaque_driver_cache": "uncontrolled",
        "pipeline_cache": str(pipeline_cache.resolve()),
        "working_directory": str(reference_root.resolve()),
    }


def close_vector(actual: object, expected: list[float], tolerance: float = 2.0e-4) -> bool:
    return (
        isinstance(actual, list)
        and len(actual) == len(expected)
        and all(abs(float(actual[index]) - float(expected[index])) <= tolerance for index in range(len(expected)))
    )


def validate_evo_record(
    spec: RunSpec, suite: dict[str, object], summary: dict[str, object], output_path: Path, metrics_path: Path
) -> None:
    expected_mode = "RayTracing" if spec.technique == "rtx" else "RayQuery"
    expected_measured_frames = max(spec.frames - 1, 0)
    expected = {
        "schema": 2,
        "type": "evoengine_ray_capture",
        "width": spec.width,
        "height": spec.height,
        "requested_frames": spec.frames,
        "timing_warmup_frames": 1,
        "measured_frames": expected_measured_frames,
        "samples_per_frame": spec.samples_per_frame,
        "measured_spp": expected_measured_frames * spec.samples_per_frame,
        "render_mode": expected_mode,
        "ser_enabled": False,
        "query_only": spec.technique == "query-only",
        "deterministic": True,
        "output_format": "radiance_hdr_linear",
        "demo_profile": spec.demo,
    }
    for field, value in expected.items():
        if summary.get(field) != value:
            raise RuntimeError(f"{spec.run_id}: EvoEngine {field} must be {value!r}; got {summary.get(field)!r}")
    if not spec.motion:
        for field, value in (("camera_frames", spec.frames), ("effective_spp", spec.spp)):
            if summary.get(field) != value:
                raise RuntimeError(f"{spec.run_id}: EvoEngine {field} must be {value!r}; got {summary.get(field)!r}")
    if Path(str(summary.get("output_path", ""))).resolve() != output_path.resolve():
        raise RuntimeError(f"{spec.run_id}: EvoEngine summary output path does not match the run output")
    capabilities = summary.get("capabilities") or {}
    if not capabilities.get("acceleration_structure") or not capabilities.get("ray_query"):
        raise RuntimeError(f"{spec.run_id}: EvoEngine lacks required acceleration-structure/RayQuery capability")
    if spec.technique == "rtx" and not capabilities.get("ray_tracing_pipeline"):
        raise RuntimeError(f"{spec.run_id}: RTX run lacks ray-tracing-pipeline capability")
    if spec.technique == "query-only" and capabilities.get("ray_tracing_pipeline"):
        raise RuntimeError(f"{spec.run_id}: query-only run still exposes the ray-tracing pipeline")
    if spec.suite == "cross-renderer":
        camera = camera_by_id(suite, spec.camera)
        if not close_vector(summary.get("camera_position"), camera["evo_position"]):
            raise RuntimeError(f"{spec.run_id}: EvoEngine camera position differs from the manifest")
        if not close_vector(summary.get("camera_look_at_override"), camera["evo_look_at"]):
            raise RuntimeError(f"{spec.run_id}: EvoEngine camera look-at differs from the manifest")
    path_name = "Path Trace (RTX)" if spec.technique == "rtx" else "Path Trace (RQ)"
    sections = {section.get("name"): section for section in summary.get("gpu_sections", [])}
    if path_name not in sections or sections[path_name].get("sample_count") != expected_measured_frames:
        raise RuntimeError(f"{spec.run_id}: missing one measured GPU sample per non-warmup frame")
    cpu_sections = {section.get("name"): section for section in summary.get("cpu_sections", [])}
    if "Queue Submit" not in cpu_sections or cpu_sections["Queue Submit"].get("sample_count") != expected_measured_frames:
        raise RuntimeError(f"{spec.run_id}: missing Queue Submit CPU samples")
    if not summary.get("gpu_memory", {}).get("peak") or not summary.get("gpu_memory", {}).get("final"):
        raise RuntimeError(f"{spec.run_id}: missing GPU memory telemetry")
    variant = summary.get("ray_shader_variant") or {}
    if not variant.get("ready") or variant.get("pending") or variant.get("failed"):
        raise RuntimeError(f"{spec.run_id}: selected shader variant is not ready")
    if str(variant.get("selection_mode")) != ("full" if spec.variant == "full" else "auto"):
        raise RuntimeError(f"{spec.run_id}: shader variant mode mismatch")
    if variant.get("requested_key") != variant.get("active_key") or variant.get("requested_mask") != variant.get("active_mask"):
        raise RuntimeError(f"{spec.run_id}: requested ray shader variant is not the active variant")
    if spec.variant == "full" and (variant.get("active_mask") != 32767 or not variant.get("fallback_active")):
        raise RuntimeError(f"{spec.run_id}: full shader run did not use the permanent all-feature fallback")
    if spec.variant == "specialized" and variant.get("active_mask") != 32767 and variant.get("fallback_active"):
        raise RuntimeError(f"{spec.run_id}: specialized shader unexpectedly retained the fallback")
    if spec.variant == "specialized":
        expected_sources = {"compiled"} if spec.cache_state == "cold" else {"disk"}
        if variant.get("cache_source") not in expected_sources:
            raise RuntimeError(
                f"{spec.run_id}: specialized {spec.cache_state} cache source was {variant.get('cache_source')!r}"
            )
    shader_cache = variant.get("shader_cache") or {}
    if spec.cache_state == "cold" and (
        int(shader_cache.get("compilations", 0)) <= 0 or int(shader_cache.get("disk_misses", 0)) <= 0
    ):
        raise RuntimeError(f"{spec.run_id}: cold EvoEngine run did not compile from an empty frontend cache")
    if spec.cache_state == "warm" and (
        int(shader_cache.get("disk_hits", 0)) <= 0 or int(shader_cache.get("compilations", 0)) != 0
    ):
        raise RuntimeError(f"{spec.run_id}: warm EvoEngine run did not consume its populated frontend cache")
    if not metrics_path.is_file() or json.loads(metrics_path.read_text(encoding="utf-8")) != summary:
        raise RuntimeError(f"{spec.run_id}: metrics file and stdout summary differ")


def validate_reference_record(spec: RunSpec, summary: dict[str, object], memory: dict[str, object] | None) -> None:
    expected_measured_frames = max(spec.frames - 1, 0)
    expected = {
        "schema": 2,
        "type": "headless_summary",
        "resolution_w": spec.width,
        "resolution_h": spec.height,
        "frames": spec.frames,
        "maxFrames": spec.frames,
        "ptSamples": spec.samples_per_frame,
        "effective_spp": spec.spp,
        "measured_effective_spp": expected_measured_frames * spec.samples_per_frame,
        "warmup_frames": 1,
        "measured_frames": expected_measured_frames,
        "render_technique": "raytracing" if spec.technique == "rtx" else "rayquery",
        "ser_enabled": False,
        "ser_requested": 0,
        "optimal_shader": spec.variant == "specialized",
        "compiled_optimal": spec.variant == "specialized",
    }
    for field, value in expected.items():
        if summary.get(field) != value:
            raise RuntimeError(f"{spec.run_id}: reference {field} must be {value!r}; got {summary.get(field)!r}")
    if spec.spp >= 64 and (not summary.get("gpu_sample_count") or "gpu_average_ms" not in summary):
        raise RuntimeError(f"{spec.run_id}: reference GPU distribution is incomplete")
    if spec.spp >= 64 and ("cpu_median_ms" not in summary or "cpu_p95_ms" not in summary):
        raise RuntimeError(f"{spec.run_id}: reference CPU distribution is incomplete")
    if memory is None or not memory.get("memory"):
        raise RuntimeError(f"{spec.run_id}: reference memory snapshot is missing")
    reported_cache = Path(str(summary.get("pipeline_cache_path", "")))
    if reported_cache.name != "pipeline_cache.bin" or reported_cache.parent.name != spec.cache_pair_id:
        raise RuntimeError(f"{spec.run_id}: reference pipeline-cache path was not reported")
    if spec.cache_state == "cold" and summary.get("pipeline_cache_initial_bytes") != 0:
        raise RuntimeError(f"{spec.run_id}: cold reference run loaded an existing pipeline cache")
    if not summary.get("pipeline_feedback_valid"):
        raise RuntimeError(f"{spec.run_id}: reference pipeline creation feedback is invalid")
    if spec.cache_state == "cold" and summary.get("pipeline_cache_hit"):
        raise RuntimeError(f"{spec.run_id}: cold reference run reported an application pipeline-cache hit")
    if spec.cache_state == "warm" and int(summary.get("pipeline_cache_initial_bytes", 0)) <= 0:
        raise RuntimeError(f"{spec.run_id}: warm reference run did not load its cold pipeline cache")
    if spec.cache_state == "warm" and not summary.get("pipeline_cache_hit"):
        raise RuntimeError(f"{spec.run_id}: warm reference run missed its application pipeline cache")
    if spec.variant == "full" and summary.get("frontend_source") != "embedded_spirv":
        raise RuntimeError(f"{spec.run_id}: full reference run did not use embedded SPIR-V")
    if spec.variant == "specialized" and summary.get("frontend_source") != "slang_compile":
        raise RuntimeError(f"{spec.run_id}: optimal reference run did not compile the specialized shader")
    if summary.get("active_features") != summary.get("scene_features") or not summary.get("scene_features"):
        raise RuntimeError(f"{spec.run_id}: reference scene feature reporting is incomplete")
    expected_feature_mode = "all" if spec.variant == "full" else "scene_specialized"
    expected_compiled_features = "all" if spec.variant == "full" else summary.get("scene_features")
    if (
        summary.get("compiled_feature_mode") != expected_feature_mode
        or summary.get("compiled_features") != expected_compiled_features
    ):
        raise RuntimeError(f"{spec.run_id}: reference compiled feature coverage is inaccurate")
    if int(summary.get("blas_count", 0)) <= 0 or int(summary.get("tlas_instance_count", 0)) <= 0:
        raise RuntimeError(f"{spec.run_id}: reference acceleration-structure counts are missing")
    blas_memory = next((row for row in memory["memory"] if row.get("category") == "BLAS"), None)
    original_blas_bytes = int(summary.get("blas_original_bytes", 0))
    compacted_blas_bytes = int(summary.get("blas_compacted_bytes", 0))
    live_blas_bytes = int(summary.get("blas_live_bytes", 0))
    peak_blas_bytes = int(summary.get("blas_peak_bytes", 0))
    if (
        not blas_memory
        or original_blas_bytes <= 0
        or compacted_blas_bytes <= 0
        or live_blas_bytes != original_blas_bytes + compacted_blas_bytes
        or peak_blas_bytes != live_blas_bytes
        or int(blas_memory.get("device_used", 0)) != live_blas_bytes
        or int(blas_memory.get("device_allocated", 0)) != peak_blas_bytes
    ):
        raise RuntimeError(f"{spec.run_id}: reference live/compacted BLAS memory telemetry is invalid")


def prepare_cache_pair(
    spec: RunSpec,
    suite: dict[str, object],
    paths: dict[str, Path],
    evidence_fingerprint: str,
    dry_run: bool,
) -> dict[str, object]:
    cache_root = paths["cache_root"]
    prime_path = paths["cache_prime"]
    if dry_run:
        return {"state": spec.cache_state, "prime": str(prime_path), "before": None}
    if spec.cache_state == "cold":
        if cache_root.exists():
            shutil.rmtree(cache_root)
        prime_path.unlink(missing_ok=True)
        cache_root.mkdir(parents=True, exist_ok=True)
        return {"state": "cold", "prime": str(prime_path), "before": directory_manifest(cache_root)}
    if not prime_path.is_file():
        raise RuntimeError(f"{spec.run_id}: warm cache requires its completed cold prime: {prime_path}")
    prime = json.loads(prime_path.read_text(encoding="utf-8"))
    cold_values = asdict(spec)
    cold_values["cache_state"] = "cold"
    cold_spec = RunSpec(**cold_values)
    cold_record_path = paths["record"].with_name(f"{cold_spec.run_id}.record.json")
    if not cold_record_path.is_file():
        raise RuntimeError(f"{spec.run_id}: warm cache requires a durable validated cold record")
    cold_record = json.loads(cold_record_path.read_text(encoding="utf-8"))
    verify_record_integrity(cold_record, cold_spec, suite, evidence_fingerprint)
    before = directory_manifest(cache_root)
    if (
        prime.get("evidence_fingerprint") != evidence_fingerprint
        or prime.get("artifact") != before
        or prime.get("run_id") != cold_spec.run_id
        or prime.get("output_sha256") != cold_record.get("output_sha256")
    ):
        raise RuntimeError(f"{spec.run_id}: warm cache input no longer matches its cold prime")
    return {
        "state": "warm",
        "prime": str(prime_path),
        "before": before,
        "cold_output_sha256": prime.get("output_sha256"),
        "cold_output": cold_record.get("output"),
    }


def finish_cache_pair(
    spec: RunSpec,
    suite: dict[str, object],
    paths: dict[str, Path],
    evidence_fingerprint: str,
    cache_evidence: dict[str, object],
    output_sha256: str,
) -> dict[str, object]:
    after = directory_manifest(paths["cache_root"])
    if not after["files"]:
        raise RuntimeError(f"{spec.run_id}: renderer produced no cache artifact")
    cache_evidence["after"] = after
    if spec.cache_state == "cold":
        write_json(
            paths["cache_prime"],
            {
                "schema": 1,
                "type": "raytracer_m6_cache_prime",
                "run_id": spec.run_id,
                "evidence_fingerprint": evidence_fingerprint,
                "artifact": after,
                "output": str(paths["output"]),
                "output_sha256": output_sha256,
            },
        )
    else:
        quality = suite["quality"]
        limits = {"relative_l2_error": float(quality["cache_pair_investigation_relative_l2_error"])}
        exact_match = cache_evidence.get("cold_output_sha256") == output_sha256
        cache_evidence["output_comparison_policy"] = quality["cache_pair_output_policy"]
        cache_evidence["output_exact_match"] = exact_match
        cache_evidence["output_comparison_investigation_limits"] = limits
        cache_evidence["output_comparison_investigate"] = False
        if not exact_match:
            cold_output = Path(str(cache_evidence.get("cold_output", "")))
            comparison = compare_hdr_images(read_hdr(cold_output), read_hdr(paths["output"]))
            cache_evidence["output_comparison"] = comparison
            exceedances = comparison_limit_exceedances(comparison, limits)
            cache_evidence["output_comparison_investigate"] = bool(exceedances)
            if exceedances:
                cache_evidence["output_comparison_exceedances"] = exceedances
    return cache_evidence


def verify_record_integrity(
    record: dict[str, object], spec: RunSpec, suite: dict[str, object], evidence_fingerprint: str
) -> None:
    if record.get("run_id") != spec.run_id or record.get("spec") != asdict(spec):
        raise RuntimeError(f"{spec.run_id}: resumed record does not match the requested run")
    if record.get("status") != "complete" or record.get("evidence_fingerprint") != evidence_fingerprint:
        raise RuntimeError(f"{spec.run_id}: resumed record belongs to different evidence provenance")
    for path_field, hash_field in (("output", "output_sha256"), ("log", "log_sha256"), ("metrics_path", "metrics_sha256")):
        value = record.get(path_field)
        expected_hash = record.get(hash_field)
        if value is None:
            if expected_hash is not None:
                raise RuntimeError(f"{spec.run_id}: {hash_field} exists without {path_field}")
            continue
        path = Path(str(value))
        if not path.is_file() or legacy.sha256(path) != expected_hash:
            raise RuntimeError(f"{spec.run_id}: resumed {path_field} is missing or changed")
    output_path = Path(str(record["output"]))
    metrics_path = Path(str(record["metrics_path"])) if record.get("metrics_path") else None
    validate_run_log(Path(str(record["log"])), spec.renderer)
    legacy.validate_output_image(
        output_path, legacy.Profile(spec.width, spec.height, spec.frames, spec.samples_per_frame)
    )
    if spec.renderer == "evo":
        if metrics_path is None or record.get("summary") != json.loads(metrics_path.read_text(encoding="utf-8")):
            raise RuntimeError(f"{spec.run_id}: resumed summary differs from its hashed metrics file")
        validate_evo_record(spec, suite, record["summary"], output_path, metrics_path)
        cache_evidence = record.get("cache_evidence") or {}
        after = cache_evidence.get("after")
        if (
            cache_evidence.get("state") != spec.cache_state
            or not isinstance(after, dict)
            or directory_manifest(Path(str(after.get("root", "")))) != after
        ):
            raise RuntimeError(f"{spec.run_id}: resumed shader-cache artifact differs from its manifest")
        prime_path = Path(str(cache_evidence.get("prime", "")))
        prime = json.loads(prime_path.read_text(encoding="utf-8")) if prime_path.is_file() else {}
        prime_run_id = spec.run_id
        prime_artifact = after
        prime_output_sha256 = record.get("output_sha256")
        if spec.cache_state == "warm":
            cold_values = asdict(spec)
            cold_values["cache_state"] = "cold"
            prime_run_id = RunSpec(**cold_values).run_id
            prime_artifact = cache_evidence.get("before")
            prime_output_sha256 = cache_evidence.get("cold_output_sha256")
        if (
            prime.get("evidence_fingerprint") != evidence_fingerprint
            or prime.get("run_id") != prime_run_id
            or prime.get("artifact") != prime_artifact
            or prime.get("output_sha256") != prime_output_sha256
        ):
            raise RuntimeError(f"{spec.run_id}: resumed shader-cache prime is missing or changed")
        if spec.cache_state == "cold":
            before = cache_evidence.get("before") or {}
            if before.get("files") != [] or before.get("combined_sha256") != hashlib.sha256().hexdigest():
                raise RuntimeError(f"{spec.run_id}: resumed cold run did not start from an empty cache")
        if spec.demo == "bistro":
            project_before = record.get("evo_project_state_before")
            project_reset = record.get("evo_project_state_reset")
            if project_before != project_reset or project_reset != legacy.evo_project_state():
                raise RuntimeError(f"{spec.run_id}: resumed Bistro project is not in its recorded reset state")
            for field in ("evo_project_preparation", "evo_project_reset"):
                preparation = record.get(field) or {}
                if not preparation:
                    if field == "evo_project_preparation":
                        continue
                    raise RuntimeError(f"{spec.run_id}: resumed Bistro reset evidence is missing")
                log = Path(str(preparation.get("log", "")))
                if not log.is_file() or legacy.sha256(log) != preparation.get("log_sha256"):
                    raise RuntimeError(f"{spec.run_id}: resumed Bistro preparation evidence changed")
    else:
        validate_reference_record(spec, record["summary"], record.get("memory"))
    process_wall_seconds = float(record.get("process_wall_seconds", 0.0))
    if not math.isfinite(process_wall_seconds) or process_wall_seconds <= 0.0:
        raise RuntimeError(f"{spec.run_id}: resumed process wall time is invalid")


def execute_spec(
    spec: RunSpec,
    suite: dict[str, object],
    output_root: Path,
    editor: Path,
    reference_root: Path,
    reference_exe: Path,
    reference_environment: Path,
    camera_assets: dict[str, dict[str, object]],
    evidence_fingerprint: str,
    prepare_evo_bistro: bool,
    resume: bool,
    dry_run: bool,
) -> dict[str, object]:
    paths = spec_paths(output_root, spec)
    if resume and paths["record"].is_file() and paths["output"].is_file():
        record = json.loads(paths["record"].read_text(encoding="utf-8"))
        if record.get("run_id") == spec.run_id and record.get("status") == "complete":
            verify_record_integrity(record, spec, suite, evidence_fingerprint)
            print(f"Resume: {spec.run_id}", flush=True)
            return record
    paths["output"].parent.mkdir(parents=True, exist_ok=True)
    if not dry_run:
        paths["output"].unlink(missing_ok=True)
        paths["metrics"].unlink(missing_ok=True)
        paths["record"].unlink(missing_ok=True)
        paths["imgui"].unlink(missing_ok=True)
    preparation = None
    reset_preparation = None
    project_before = None
    project_after = None
    project_reset = None
    if spec.renderer == "evo" and spec.demo == "bistro":
        if prepare_evo_bistro:
            label = hashlib.sha256(spec.run_id.encode("utf-8")).hexdigest()[:12]
            preparation = legacy.prepare_evo_bistro(output_root, label, dry_run)
        if not dry_run:
            project_before = legacy.evo_project_state()
    cache_evidence = prepare_cache_pair(spec, suite, paths, evidence_fingerprint, dry_run)
    if spec.renderer == "evo":
        command, environment, cache = evo_command(spec, suite, editor, paths)
        cwd = ROOT
    else:
        command, environment, cache = reference_command(
            spec,
            suite,
            reference_exe,
            reference_root,
            reference_environment,
            camera_assets,
            paths,
        )
        cwd = reference_root
    started = time.perf_counter()
    _, records = legacy.run_command(command, cwd, paths["log"], dry_run, environment)
    process_wall_seconds = time.perf_counter() - started
    if project_before is not None:
        project_after = legacy.evo_project_state()
        reset_preparation = legacy.prepare_evo_bistro(
            output_root, f"{hashlib.sha256(spec.run_id.encode('utf-8')).hexdigest()[:12]}-post", dry_run
        )
        project_reset = legacy.evo_project_state()
        if project_reset != project_before:
            raise RuntimeError(f"{spec.run_id}: Bistro project reset did not restore the prepared state")
    record: dict[str, object] = {
        "schema": 1,
        "type": "raytracer_m6_run",
        "status": "planned" if dry_run else "complete",
        "run_id": spec.run_id,
        "spec": asdict(spec),
        "command": command,
        "environment": environment,
        "cache": cache,
        "cache_evidence": cache_evidence,
        "evidence_fingerprint": evidence_fingerprint,
        "evo_project_preparation": preparation,
        "evo_project_state_before": project_before,
        "evo_project_state_after": project_after,
        "evo_project_reset": reset_preparation,
        "evo_project_state_reset": project_reset,
        "process_wall_seconds": process_wall_seconds,
        "log": str(paths["log"]),
        "output": str(paths["output"]),
        "metrics_path": str(paths["metrics"]) if spec.renderer == "evo" else None,
    }
    if dry_run:
        return record
    validate_run_log(paths["log"], spec.renderer)
    profile = legacy.Profile(spec.width, spec.height, spec.frames, spec.samples_per_frame)
    legacy.validate_output_image(paths["output"], profile)
    if spec.renderer == "evo":
        summary = find_record(records, "evoengine_ray_capture")
        if summary is None:
            raise RuntimeError(f"{spec.run_id}: missing EvoEngine summary")
        validate_evo_record(spec, suite, summary, paths["output"], paths["metrics"])
        memory = summary.get("gpu_memory")
    else:
        summary = find_record(records, "headless_summary")
        if summary is None:
            raise RuntimeError(f"{spec.run_id}: missing reference summary")
        memory = find_record(records, "sequence_memory")
        validate_reference_record(spec, summary, memory)
    output_sha256 = legacy.sha256(paths["output"])
    record["cache_evidence"] = finish_cache_pair(
        spec, suite, paths, evidence_fingerprint, cache_evidence, output_sha256
    )
    record.update(
        {
            "summary": summary,
            "memory": memory,
            "output_sha256": output_sha256,
            "log_sha256": legacy.sha256(paths["log"]),
            "metrics_sha256": legacy.sha256(paths["metrics"]) if paths["metrics"].is_file() else None,
        }
    )
    write_json(paths["record"], record)
    return record


def run_specs(
    specs: list[RunSpec],
    suite: dict[str, object],
    args: argparse.Namespace,
    camera_assets: dict[str, dict[str, object]],
    evidence_fingerprint: str,
) -> list[dict[str, object]]:
    output_root = args.output_dir.resolve()
    editor = args.editor.resolve()
    reference_root = args.reference_root.resolve()
    reference_exe = (
        args.reference_exe.resolve()
        if args.reference_exe
        else reference_root / "_bin" / "RelWithDebInfo" / "vk_gltf_renderer.exe"
    )
    reference_environment = reference_root / "resources" / "std_env.hdr"
    pair_groups: dict[str, list[RunSpec]] = {}
    for spec in specs:
        pair_groups.setdefault(spec.cache_pair_id, []).append(spec)
    ordered_groups = sorted(
        pair_groups.values(),
        key=lambda group: hashlib.sha256(
            f"{suite['order_seed']}:{group[0].cache_pair_id}".encode("utf-8")
        ).hexdigest(),
    )
    ordered_specs = []
    for group in ordered_groups:
        ordered_specs.extend(sorted(group, key=lambda spec: (spec.cache_state != "cold", spec.run_id)))
    records = []
    for index, spec in enumerate(ordered_specs, start=1):
        print(f"M6 run {index}/{len(specs)}: {spec.run_id}", flush=True)
        records.append(
            execute_spec(
                spec,
                suite,
                output_root,
                editor,
                reference_root,
                reference_exe,
                reference_environment,
                camera_assets,
                evidence_fingerprint,
                not args.skip_evo_prepare,
                args.resume,
                args.dry_run,
            )
        )
    return records


def load_records(output_root: Path, purpose_prefix: str | None = None) -> list[dict[str, object]]:
    records = []
    for path in sorted(output_root.glob("*/*/*.record.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        purpose = str(record.get("spec", {}).get("purpose", ""))
        if record.get("status") == "complete" and (purpose_prefix is None or purpose.startswith(purpose_prefix)):
            records.append(record)
    return records


def truth_record_for(candidate: dict[str, object], records: list[dict[str, object]]) -> dict[str, object] | None:
    spec = candidate["spec"]
    truth_technique = "rq" if spec["technique"] == "query-only" else spec["technique"]
    purpose = f"{spec['profile']}-truth"
    for record in records:
        truth = record["spec"]
        if (
            truth["purpose"] == purpose
            and truth["suite"] == spec["suite"]
            and truth["scene"] == spec["scene"]
            and truth["camera"] == spec["camera"]
            and truth["renderer"] == spec["renderer"]
            and truth["technique"] == truth_technique
        ):
            return record
    return None


def timing_section(summary: dict[str, object], field: str, name: str) -> dict[str, object]:
    return next((section for section in summary.get(field, []) if section.get("name") == name), {})


def device_local_memory(memory: dict[str, object], phase: str, field: str) -> int:
    snapshot = memory.get(phase, {}) if memory else {}
    return sum(
        int(heap.get(field, 0)) for heap in snapshot.get("heaps", []) if heap.get("category") == "device_local"
    )


def require_records(
    output_root: Path,
    expected_specs: list[RunSpec],
    suite: dict[str, object],
    evidence_fingerprint: str,
    label: str,
) -> list[dict[str, object]]:
    expected = {spec.run_id: spec for spec in expected_specs}
    purposes = {spec.purpose for spec in expected_specs}
    candidates = [record for record in load_records(output_root) if record.get("spec", {}).get("purpose") in purposes]
    actual: dict[str, dict[str, object]] = {}
    for record in candidates:
        run_id = str(record.get("run_id", ""))
        if run_id in actual:
            raise RuntimeError(f"{label}: duplicate run record {run_id}")
        actual[run_id] = record
    missing = sorted(set(expected) - set(actual))
    extra = sorted(set(actual) - set(expected))
    if missing or extra:
        raise RuntimeError(f"{label}: incomplete evidence; missing={missing[:5]}, extra={extra[:5]}")
    for run_id, spec in expected.items():
        verify_record_integrity(actual[run_id], spec, suite, evidence_fingerprint)
    return [actual[spec.run_id] for spec in expected_specs]


def gpu_identity(record: dict[str, object]) -> tuple[int, int, int]:
    summary = record["summary"]
    if record["spec"]["renderer"] == "evo":
        gpu = summary.get("gpu") or {}
        return int(gpu.get("vendor_id", -1)), int(gpu.get("device_id", -1)), int(gpu.get("driver_version", -1))
    return (
        int(summary.get("gpu_vendor_id", -1)),
        int(summary.get("gpu_device_id", -1)),
        int(summary.get("gpu_driver_version", -1)),
    )


def require_matched_gpu(records: list[dict[str, object]], label: str) -> tuple[int, int, int]:
    identities = {gpu_identity(record) for record in records}
    if len(identities) != 1 or next(iter(identities))[0] < 0:
        raise RuntimeError(f"{label}: records were not produced by one matched GPU and driver: {sorted(identities)}")
    return next(iter(identities))


def record_set_digest(records: list[dict[str, object]]) -> str:
    return json_sha256(
        [
            {
                "run_id": record["run_id"],
                "output_sha256": record["output_sha256"],
                "log_sha256": record["log_sha256"],
                "metrics_sha256": record["metrics_sha256"],
            }
            for record in sorted(records, key=lambda item: str(item["run_id"]))
        ]
    )


def timing_sample_count(sections: object, name: str) -> int:
    if not isinstance(sections, list):
        return 0
    return sum(
        int(section.get("sample_count", 0))
        for section in sections
        if isinstance(section, dict) and section.get("name") == name
    )


def scored_row(record: dict[str, object], truth: dict[str, object] | None, threshold: float) -> dict[str, object]:
    spec = record["spec"]
    summary = record["summary"]
    cache_evidence = record.get("cache_evidence") or {}
    cache_pair_output_exact = None
    cache_pair_relative_l2_error = None
    cache_pair_investigate = None
    if spec["cache_state"] == "warm":
        cache_pair_output_exact = bool(cache_evidence.get("output_exact_match", False))
        cache_pair_comparison = cache_evidence.get("output_comparison") or {}
        cache_pair_relative_l2_error = (
            0.0 if cache_pair_output_exact else float(cache_pair_comparison["relative_l2_error"])
        )
        cache_pair_investigate = bool(cache_evidence.get("output_comparison_investigate", False))
    comparison = None
    time_to_quality = None
    if truth is not None and not spec["motion"]:
        comparison = compare_hdr_images(read_hdr(Path(truth["output"])), read_hdr(Path(record["output"])))
        if float(comparison["relative_l2_error"]) <= threshold:
            if spec["renderer"] == "evo":
                wall_seconds = float(summary["accumulation_wall_seconds"])
            else:
                wall_seconds = float(summary["wall_ms"]) / 1000.0
            measured_spp = int(summary.get("measured_spp", summary.get("measured_effective_spp", 0)))
            time_to_quality = wall_seconds * int(spec["spp"]) / max(measured_spp, 1)
    if spec["renderer"] == "evo":
        path_name = "Path Trace (RTX)" if spec["technique"] == "rtx" else "Path Trace (RQ)"
        gpu = timing_section(summary, "gpu_sections", path_name)
        cpu = timing_section(summary, "cpu_sections", "Queue Submit")
        memory = summary.get("gpu_memory", {})
        variant = summary.get("ray_shader_variant") or {}
        fallback_build_ms = float(variant.get("fallback_build_ms", 0.0))
        selected_build_ms = float(variant.get("build_ms", 0.0))
        cache_build_ms = selected_build_ms if variant.get("fallback_active") else fallback_build_ms + selected_build_ms
        cache_ready_ms = float(variant.get("request_to_ready_ms", 0.0))
        gpu_median_ms = float(gpu.get("median_ms", 0.0))
        gpu_p95_ms = float(gpu.get("p95_ms", 0.0))
        gpu_representative_ms = gpu_median_ms
        gpu_sample_source = "capture_frame_distribution"
        gpu_sample_count = int(gpu.get("sample_count", 0))
        cpu_median_ms = float(cpu.get("median_ms", 0.0))
        cpu_p95_ms = float(cpu.get("p95_ms", 0.0))
        cpu_sample_count = int(cpu.get("sample_count", 0))
        cpu_metric = "queue_submit"
        peak_vram = device_local_memory(memory, "peak", "allocation_bytes")
        final_vram = device_local_memory(memory, "final", "allocation_bytes")
        startup_sections = summary.get("startup_gpu_sections", [])
        capture_sections = summary.get("gpu_sections", [])
        startup_as_gpu_ms = sum(
            float(section.get("total_ms", 0.0))
            for section in startup_sections
            if str(section.get("name", "")).startswith(("BLAS ", "TLAS "))
        )
        capture_as_gpu_ms = sum(
            float(section.get("total_ms", 0.0))
            for section in capture_sections
            if str(section.get("name", "")).startswith(("BLAS ", "TLAS "))
        )
        requested_feature_mask = int(variant.get("requested_mask", 0))
        active_feature_mask = int(variant.get("active_mask", 0))
        requested_variant_key = str(variant.get("requested_key", ""))
        active_variant_key = str(variant.get("active_key", ""))
        shader_cache = variant.get("shader_cache") or {}
        frontend_cache_source = str(variant.get("cache_source", ""))
        if spec["variant"] == "full":
            frontend_cache_source = "compiled" if int(shader_cache.get("compilations", 0)) > 0 else "disk"
        compiled_optimal = spec["variant"] == "specialized"
        active_features = None
        scene_features = None
        compiled_features = None
        compiled_feature_mode = None
        pipeline_feedback_valid = None
        pipeline_cache_hit = None
        reference_blas_count = None
        reference_tlas_instance_count = None
        reference_blas_original_bytes = None
        reference_blas_compacted_bytes = None
        reference_blas_live_bytes = None
        reference_blas_peak_bytes = None
        startup_blas_build_count = timing_sample_count(startup_sections, "BLAS Build")
        startup_blas_update_count = timing_sample_count(startup_sections, "BLAS Update")
        startup_tlas_build_count = timing_sample_count(startup_sections, "TLAS Build")
        startup_tlas_update_count = timing_sample_count(startup_sections, "TLAS Update")
        capture_blas_build_count = timing_sample_count(capture_sections, "BLAS Build")
        capture_blas_update_count = timing_sample_count(capture_sections, "BLAS Update")
        capture_tlas_build_count = timing_sample_count(capture_sections, "TLAS Build")
        capture_tlas_update_count = timing_sample_count(capture_sections, "TLAS Update")
    else:
        gpu_median_ms = float(summary.get("gpu_median_ms", 0.0))
        gpu_p95_ms = float(summary.get("gpu_p95_ms", 0.0))
        gpu_representative_ms = float(summary.get("gpu_average_ms", 0.0))
        gpu_sample_source = "nvutils_delayed_window_average"
        gpu_sample_count = int(summary.get("gpu_sample_count", 0))
        cpu_median_ms = float(summary.get("cpu_median_ms", 0.0))
        cpu_p95_ms = float(summary.get("cpu_p95_ms", 0.0))
        cpu_sample_count = int(summary.get("cpu_sample_count", 0))
        cpu_metric = "host_render_record"
        cache_build_ms = float(summary.get("pipeline_build_ms", 0.0))
        cache_ready_ms = float(summary.get("frontend_compile_ms", 0.0)) + cache_build_ms
        memory_rows = record.get("memory", {}).get("memory", [])
        peak_vram = sum(int(row.get("device_allocated", 0)) for row in memory_rows)
        final_vram = sum(int(row.get("device_used", 0)) for row in memory_rows)
        startup_as_gpu_ms = None
        capture_as_gpu_ms = None
        requested_feature_mask = None
        active_feature_mask = None
        requested_variant_key = None
        active_variant_key = None
        frontend_cache_source = str(summary.get("frontend_source", ""))
        compiled_optimal = bool(summary.get("compiled_optimal", False))
        active_features = str(summary.get("active_features", ""))
        scene_features = str(summary.get("scene_features", ""))
        compiled_features = str(summary.get("compiled_features", ""))
        compiled_feature_mode = str(summary.get("compiled_feature_mode", ""))
        pipeline_feedback_valid = bool(summary.get("pipeline_feedback_valid", False))
        pipeline_cache_hit = bool(summary.get("pipeline_cache_hit", False))
        reference_blas_count = int(summary.get("blas_count", 0))
        reference_tlas_instance_count = int(summary.get("tlas_instance_count", 0))
        reference_blas_original_bytes = int(summary.get("blas_original_bytes", 0))
        reference_blas_compacted_bytes = int(summary.get("blas_compacted_bytes", 0))
        reference_blas_live_bytes = int(summary.get("blas_live_bytes", 0))
        reference_blas_peak_bytes = int(summary.get("blas_peak_bytes", 0))
        startup_blas_build_count = None
        startup_blas_update_count = None
        startup_tlas_build_count = None
        startup_tlas_update_count = None
        capture_blas_build_count = None
        capture_blas_update_count = None
        capture_tlas_build_count = None
        capture_tlas_update_count = None
    throughput = float(
        summary.get("wall_throughput_msamples_per_second", summary.get("throughput_MSps", 0.0))
    )
    relative_l2 = None if comparison is None else float(comparison["relative_l2_error"])
    return {
        "run_id": record["run_id"],
        **spec,
        "relative_l2_error": relative_l2,
        "quality_threshold": threshold if comparison is not None else None,
        "quality_pass": None if comparison is None else relative_l2 <= threshold,
        "time_to_quality_seconds": time_to_quality,
        "gpu_median_ms": gpu_median_ms,
        "gpu_p95_ms": gpu_p95_ms,
        "gpu_representative_ms": gpu_representative_ms,
        "gpu_sample_source": gpu_sample_source,
        "gpu_sample_count": gpu_sample_count,
        "cpu_metric": cpu_metric,
        "cpu_median_ms": cpu_median_ms,
        "cpu_p95_ms": cpu_p95_ms,
        "cpu_representative_ms": cpu_median_ms,
        "cpu_sample_count": cpu_sample_count,
        "wall_throughput_msamples_per_second": throughput,
        "cache_build_ms": cache_build_ms,
        "cache_request_to_ready_ms": cache_ready_ms,
        "frontend_cache_source": frontend_cache_source,
        "cache_pair_output_exact": cache_pair_output_exact,
        "cache_pair_relative_l2_error": cache_pair_relative_l2_error,
        "cache_pair_investigate": cache_pair_investigate,
        "requested_feature_mask": requested_feature_mask,
        "active_feature_mask": active_feature_mask,
        "requested_variant_key": requested_variant_key,
        "active_variant_key": active_variant_key,
        "active_features": active_features,
        "scene_features": scene_features,
        "compiled_features": compiled_features,
        "compiled_feature_mode": compiled_feature_mode,
        "compiled_optimal": compiled_optimal,
        "pipeline_feedback_valid": pipeline_feedback_valid,
        "pipeline_cache_hit": pipeline_cache_hit,
        "startup_as_gpu_ms": startup_as_gpu_ms,
        "capture_as_gpu_ms": capture_as_gpu_ms,
        "reference_blas_count": reference_blas_count,
        "reference_tlas_instance_count": reference_tlas_instance_count,
        "reference_blas_original_bytes": reference_blas_original_bytes,
        "reference_blas_compacted_bytes": reference_blas_compacted_bytes,
        "reference_blas_live_bytes": reference_blas_live_bytes,
        "reference_blas_peak_bytes": reference_blas_peak_bytes,
        "startup_blas_build_count": startup_blas_build_count,
        "startup_blas_update_count": startup_blas_update_count,
        "startup_tlas_build_count": startup_tlas_build_count,
        "startup_tlas_update_count": startup_tlas_update_count,
        "capture_blas_build_count": capture_blas_build_count,
        "capture_blas_update_count": capture_blas_update_count,
        "capture_tlas_build_count": capture_tlas_build_count,
        "capture_tlas_update_count": capture_tlas_update_count,
        "peak_device_local_allocation_bytes": peak_vram,
        "final_device_local_allocation_bytes": final_vram,
        "memory_peak_scope": "capture_window_sampled" if spec["renderer"] == "evo" else "tracker_lifetime",
        "process_wall_seconds": record["process_wall_seconds"],
        "output": record["output"],
    }


def calibrate_quality(
    suite: dict[str, object],
    suite_path: Path,
    output_root: Path,
    expected_specs: list[RunSpec],
    evidence_fingerprint: str,
) -> dict[str, object]:
    records = require_records(output_root, expected_specs, suite, evidence_fingerprint, "M6 pilot")
    pilot_gpu = require_matched_gpu(records, "M6 pilot")
    pilot_records = [record for record in records if record["spec"]["purpose"] == "pilot"]
    if not pilot_records:
        raise RuntimeError("Pilot records are required before quality calibration")
    highest_spp = max(int(record["spec"]["spp"]) for record in pilot_records)
    endpoint_errors = []
    pilot_rows = []
    convergence_rows = []
    for record in pilot_records:
        truth = truth_record_for(record, records)
        if truth is None:
            raise RuntimeError(f"Missing pilot truth for {record['run_id']}")
        comparison = compare_hdr_images(read_hdr(Path(truth["output"])), read_hdr(Path(record["output"])))
        error = float(comparison["relative_l2_error"])
        pilot_rows.append(
            {
                "run_id": record["run_id"],
                "renderer": record["spec"]["renderer"],
                "technique": record["spec"]["technique"],
                "spp": record["spec"]["spp"],
                "relative_l2_error": error,
            }
        )
        if int(record["spec"]["spp"]) == highest_spp:
            endpoint_errors.append(error)
    if not endpoint_errors:
        raise RuntimeError("Pilot did not produce an endpoint error")
    threshold = max(endpoint_errors) * float(suite["quality"]["threshold_margin"])
    for high_truth in [record for record in records if record["spec"]["purpose"] == "pilot-truth"]:
        high_spec = high_truth["spec"]
        low_truth = next(
            (
                record
                for record in records
                if record["spec"]["purpose"] == "pilot-truth-low"
                and record["spec"]["renderer"] == high_spec["renderer"]
                and record["spec"]["technique"] == high_spec["technique"]
                and record["spec"]["camera"] == high_spec["camera"]
            ),
            None,
        )
        if low_truth is None:
            raise RuntimeError(f"Missing low-SPP truth for {high_truth['run_id']}")
        comparison = compare_hdr_images(read_hdr(Path(high_truth["output"])), read_hdr(Path(low_truth["output"])))
        convergence_rows.append(
            {
                "renderer": high_spec["renderer"],
                "technique": high_spec["technique"],
                "low_spp": low_truth["spec"]["spp"],
                "high_spp": high_spec["spp"],
                "relative_l2_error": float(comparison["relative_l2_error"]),
            }
        )
    convergence_limit = threshold * float(suite["quality"]["truth_convergence_fraction"])
    calibration = {
        "schema": 1,
        "type": "raytracer_m6_quality_calibration",
        "locked": True,
        "evidence_fingerprint": evidence_fingerprint,
        "source_records_sha256": record_set_digest(records),
        "gpu": {"vendor_id": pilot_gpu[0], "device_id": pilot_gpu[1], "driver_version": pilot_gpu[2]},
        "suite_sha256": legacy.sha256(suite_path),
        "metric": suite["quality"]["metric"],
        "pilot_camera": suite["quality"]["pilot_camera"],
        "selected_candidate_spp": highest_spp,
        "relative_l2_threshold": threshold,
        "threshold_margin": suite["quality"]["threshold_margin"],
        "truth_convergence_limit": convergence_limit,
        "truth_convergence_pass": all(row["relative_l2_error"] <= convergence_limit for row in convergence_rows),
        "pilot": pilot_rows,
        "truth_convergence": convergence_rows,
    }
    if not calibration["truth_convergence_pass"]:
        raise RuntimeError("Pilot high-SPP truth did not meet the frozen convergence rule")
    write_json(output_root / "quality-calibration.json", calibration)
    return calibration


def validate_measure_truth_convergence(
    suite: dict[str, object],
    output_root: Path,
    expected_specs: list[RunSpec],
    evidence_fingerprint: str,
) -> dict[str, object]:
    calibration_path = output_root / "quality-calibration.json"
    if not calibration_path.is_file():
        raise RuntimeError("Pilot calibration is required before measure-truth convergence")
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    if calibration.get("evidence_fingerprint") != evidence_fingerprint:
        raise RuntimeError("Quality calibration belongs to different evidence provenance")
    records = require_records(output_root, expected_specs, suite, evidence_fingerprint, "M6 measure truth")
    truth_gpu = require_matched_gpu(records, "M6 measure truth")
    calibration_gpu = calibration.get("gpu") or {}
    if truth_gpu != (
        int(calibration_gpu.get("vendor_id", -1)),
        int(calibration_gpu.get("device_id", -1)),
        int(calibration_gpu.get("driver_version", -1)),
    ):
        raise RuntimeError("Measure truths used a different GPU/driver than the frozen pilot")
    rows = []
    for high in [record for record in records if record["spec"]["purpose"] == "measure-truth"]:
        high_spec = high["spec"]
        low = next(
            (
                record
                for record in records
                if record["spec"]["purpose"] == "measure-truth-low"
                and all(
                    record["spec"][field] == high_spec[field]
                    for field in ("suite", "scene", "camera", "renderer", "technique")
                )
            ),
            None,
        )
        if low is None:
            raise RuntimeError(f"Missing low truth for {high['run_id']}")
        comparison = compare_hdr_images(read_hdr(Path(high["output"])), read_hdr(Path(low["output"])))
        rows.append(
            {
                "renderer": high_spec["renderer"],
                "technique": high_spec["technique"],
                "scene": high_spec["scene"],
                "camera": high_spec["camera"],
                "low_spp": low["spec"]["spp"],
                "high_spp": high_spec["spp"],
                "relative_l2_error": float(comparison["relative_l2_error"]),
            }
        )
    limit = float(calibration["relative_l2_threshold"]) * float(suite["quality"]["truth_convergence_fraction"])
    result = {
        "schema": 1,
        "type": "raytracer_m6_truth_convergence",
        "evidence_fingerprint": evidence_fingerprint,
        "calibration_sha256": legacy.sha256(calibration_path),
        "relative_l2_threshold": float(calibration["relative_l2_threshold"]),
        "source_records_sha256": record_set_digest(records),
        "relative_l2_limit": limit,
        "pass": all(row["relative_l2_error"] <= limit for row in rows),
        "rows": rows,
    }
    if not result["pass"]:
        raise RuntimeError("One or more measure truths did not meet the frozen convergence rule")
    write_json(output_root / "truth-convergence.json", result)
    return result


def collapse_repetitions(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    fields = ("suite", "scene", "camera", "renderer", "technique", "variant", "cache_state", "repetition")
    groups: dict[tuple[object, ...], list[dict[str, object]]] = {}
    for row in rows:
        groups.setdefault(tuple(row[field] for field in fields), []).append(row)
    repetitions = []
    for key, group in sorted(groups.items()):
        ordered = sorted(group, key=lambda row: int(row["spp"]))
        endpoint = dict(ordered[-1])
        passing = next((row for row in ordered if row["quality_pass"] is True), None)
        has_quality = any(row["quality_pass"] is not None for row in ordered)
        endpoint["endpoint_spp"] = endpoint.pop("spp")
        endpoint["selected_quality_spp"] = None if passing is None else passing["spp"]
        endpoint["time_to_quality_seconds"] = None if passing is None else passing["time_to_quality_seconds"]
        endpoint["quality_pass"] = None if not has_quality else passing is not None
        endpoint["checkpoint_run_ids"] = ";".join(str(row["run_id"]) for row in ordered)
        repetitions.append(endpoint)
    return repetitions


def aggregate_rows(rows: list[dict[str, object]], required_repetitions: int) -> list[dict[str, object]]:
    groups: dict[tuple[object, ...], list[dict[str, object]]] = {}
    fields = ("suite", "scene", "camera", "renderer", "technique", "variant", "cache_state")
    for row in rows:
        groups.setdefault(tuple(row[field] for field in fields), []).append(row)
    aggregates = []
    metric_fields = (
        "time_to_quality_seconds",
        "gpu_representative_ms",
        "cpu_representative_ms",
        "wall_throughput_msamples_per_second",
        "cache_build_ms",
        "cache_request_to_ready_ms",
        "startup_as_gpu_ms",
        "capture_as_gpu_ms",
        "peak_device_local_allocation_bytes",
        "final_device_local_allocation_bytes",
        "process_wall_seconds",
    )
    for key, group in sorted(groups.items()):
        if len(group) != required_repetitions:
            raise RuntimeError(f"Aggregate {key} has {len(group)} repetitions; expected {required_repetitions}")
        aggregate = {field: key[index] for index, field in enumerate(fields)}
        aggregate["repetitions"] = len(group)
        quality_values = [row["quality_pass"] for row in group if row["quality_pass"] is not None]
        aggregate["quality_pass"] = all(quality_values) if quality_values else None
        selected = [int(row["selected_quality_spp"]) for row in group if row["selected_quality_spp"] is not None]
        aggregate["selected_quality_spp_median"] = statistics.median(selected) if selected else None
        for field in metric_fields:
            values = [float(row[field]) for row in group if row[field] is not None and math.isfinite(float(row[field]))]
            aggregate[f"{field}_median"] = statistics.median(values) if values else None
            aggregate[f"{field}_p95"] = percentile(values, 0.95) if values else None
        throughput_values = [float(row["wall_throughput_msamples_per_second"]) for row in group]
        mean_throughput = statistics.mean(throughput_values)
        coefficient_of_variation = (
            statistics.pstdev(throughput_values) / mean_throughput if mean_throughput > 0.0 else None
        )
        aggregate["throughput_coefficient_of_variation"] = coefficient_of_variation
        aggregate["variance_adjusted_throughput"] = (
            statistics.median(throughput_values) / (1.0 + coefficient_of_variation * coefficient_of_variation)
            if coefficient_of_variation is not None
            else None
        )
        aggregate["cpu_metric"] = group[0]["cpu_metric"]
        aggregate["gpu_sample_source"] = group[0]["gpu_sample_source"]
        aggregate["memory_peak_scope"] = group[0]["memory_peak_scope"]
        aggregates.append(aggregate)
    return aggregates


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({field for row in rows for field in row})
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def canonical_comparisons(records: list[dict[str, object]]) -> list[dict[str, object]]:
    by_key = {(record["spec"]["renderer"], record["spec"]["technique"]): record for record in records}
    pairs = (
        ("reference-vs-evo-rtx", ("reference", "rtx"), ("evo", "rtx")),
        ("reference-vs-evo-rq", ("reference", "rq"), ("evo", "rq")),
        ("evo-rtx-vs-rq", ("evo", "rtx"), ("evo", "rq")),
        ("reference-rtx-vs-rq", ("reference", "rtx"), ("reference", "rq")),
    )
    results = []
    for name, left_key, right_key in pairs:
        left = by_key[left_key]
        right = by_key[right_key]
        results.append(
            {
                "name": name,
                "left": left["output"],
                "right": right["output"],
                "metrics": compare_hdr_images(read_hdr(Path(left["output"])), read_hdr(Path(right["output"]))),
            }
        )
    return results


def compact_comparisons(records: list[dict[str, object]]) -> list[dict[str, object]]:
    by_key = {
        (record["spec"]["renderer"], record["spec"]["technique"], record["spec"]["cache_state"]): record
        for record in records
    }
    pairs = (
        ("reference-vs-evo-rtx", ("reference", "rtx"), ("evo", "rtx")),
        ("reference-vs-evo-rq", ("reference", "rq"), ("evo", "rq")),
        ("evo-rtx-vs-rq", ("evo", "rtx"), ("evo", "rq")),
        ("evo-rq-vs-query-only", ("evo", "rq"), ("evo", "query-only")),
        ("reference-rtx-vs-rq", ("reference", "rtx"), ("reference", "rq")),
    )
    results = []
    for name, left_key, right_key in pairs:
        left = by_key[(*left_key, "cold")]
        right = by_key[(*right_key, "cold")]
        results.append(
            {
                "name": name,
                "cache_state": "cold",
                "uses_frozen_reference": "reference" in (left_key[0], right_key[0]),
                "left": left["output"],
                "right": right["output"],
                "metrics": compare_hdr_images(read_hdr(Path(left["output"])), read_hdr(Path(right["output"]))),
            }
        )
    return results


def analyze_compact(
    suite: dict[str, object],
    suite_path: Path,
    output_root: Path,
    expected_specs: list[RunSpec],
    evidence_fingerprint: str,
) -> dict[str, object]:
    if len(expected_specs) != 3:
        raise RuntimeError(f"Compact M6 analysis requires exactly 3 fresh captures, got {len(expected_specs)}")
    records = require_records(output_root, expected_specs, suite, evidence_fingerprint, "M6 fresh Evo slice")
    gpu = require_matched_gpu(records, "M6 fresh Evo slice")
    frozen_reference, reference_records = load_frozen_reference_records(suite)
    reference_gpu = require_matched_gpu(reference_records, "M6 frozen reference")
    rows = [
        {
            **scored_row(record, None, 0.0),
            "measurement_session": "fresh_m6",
            "timing_interpretation": "current",
        }
        for record in records
    ]
    reference_rows = [
        {
            **scored_row(record, None, 0.0),
            "measurement_session": "frozen_reference",
            "timing_interpretation": "historical_context_only",
        }
        for record in reference_records
    ]
    cache_pair_diagnostics = {
        "policy": "not_scheduled",
        "pair_count": 0,
        "reason": "The approved four-capture delivery uses three isolated cold Evo runs plus one post-commit image.",
    }
    comparisons = compact_comparisons([*reference_records, *records])
    frozen_reference = {
        **frozen_reference,
        "same_gpu_device_as_fresh_evo": reference_gpu[:2] == gpu[:2],
        "same_driver_as_fresh_evo": reference_gpu == gpu,
        "binding_speedup_claims_allowed": False,
    }
    report = {
        "schema": 1,
        "type": "raytracer_m6_compact_report",
        "evidence_fingerprint": evidence_fingerprint,
        "gpu": {"vendor_id": gpu[0], "device_id": gpu[1], "driver_version": gpu[2]},
        "suite_sha256": legacy.sha256(suite_path),
        "descriptive_baseline": True,
        "statistical_claims": False,
        "run_count": len(rows),
        "frozen_reference_record_count": len(reference_rows),
        "observed_new_capture_count": len(rows),
        "planned_post_commit_delivery_capture_count": 1,
        "planned_total_new_capture_count_including_delivery": len(rows) + 1,
        "cache_pair_diagnostics": cache_pair_diagnostics,
        "frozen_reference": frozen_reference,
        "runs": rows,
        "historical_reference_runs": reference_rows,
        "image_comparisons": comparisons,
    }
    write_json(output_root / "report.json", report)
    write_json(output_root / "image-comparisons.json", comparisons)
    write_csv(output_root / "runs.csv", rows)
    write_csv(output_root / "historical-reference-runs.csv", reference_rows)
    (output_root / "aggregates.json").unlink(missing_ok=True)
    (output_root / "aggregates.csv").unlink(missing_ok=True)
    return report


def analyze(
    suite: dict[str, object],
    suite_path: Path,
    output_root: Path,
    expected_pilot_specs: list[RunSpec],
    expected_measure_specs: list[RunSpec],
    expected_truth_specs: list[RunSpec],
    expected_canonical_specs: list[RunSpec],
    evidence_fingerprint: str,
) -> dict[str, object]:
    calibration_path = output_root / "quality-calibration.json"
    if not calibration_path.is_file():
        raise RuntimeError("Quality calibration must be completed before scored analysis")
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    if (
        not calibration.get("locked")
        or calibration.get("suite_sha256") != legacy.sha256(suite_path)
        or calibration.get("evidence_fingerprint") != evidence_fingerprint
    ):
        raise RuntimeError("Quality calibration does not match the current suite")
    convergence_path = output_root / "truth-convergence.json"
    if not convergence_path.is_file():
        raise RuntimeError("Measure-truth convergence must pass before analysis")
    convergence = json.loads(convergence_path.read_text(encoding="utf-8"))
    if not convergence.get("pass") or convergence.get("evidence_fingerprint") != evidence_fingerprint:
        raise RuntimeError("Measure-truth convergence evidence is invalid")
    threshold = float(calibration["relative_l2_threshold"])
    expected_convergence_limit = threshold * float(suite["quality"]["truth_convergence_fraction"])
    if (
        convergence.get("calibration_sha256") != legacy.sha256(calibration_path)
        or float(convergence.get("relative_l2_threshold", -1.0)) != threshold
        or float(convergence.get("relative_l2_limit", -1.0)) != expected_convergence_limit
    ):
        raise RuntimeError("Measure-truth convergence belongs to a different quality calibration")
    pilot_records = require_records(output_root, expected_pilot_specs, suite, evidence_fingerprint, "M6 pilot")
    measure_records = require_records(output_root, expected_measure_specs, suite, evidence_fingerprint, "M6 measure")
    truth_records = require_records(output_root, expected_truth_specs, suite, evidence_fingerprint, "M6 measure truth")
    canonical_records = require_records(output_root, expected_canonical_specs, suite, evidence_fingerprint, "M6 canonical")
    if calibration.get("source_records_sha256") != record_set_digest(pilot_records):
        raise RuntimeError("Quality calibration is not bound to the current pilot records")
    if convergence.get("source_records_sha256") != record_set_digest(truth_records):
        raise RuntimeError("Measure-truth convergence is not bound to the current truth records")
    all_records = [*pilot_records, *measure_records, *truth_records, *canonical_records]
    gpu = require_matched_gpu(all_records, "M6 final report")
    calibration_gpu = calibration.get("gpu") or {}
    if gpu != (
        int(calibration_gpu.get("vendor_id", -1)),
        int(calibration_gpu.get("device_id", -1)),
        int(calibration_gpu.get("driver_version", -1)),
    ):
        raise RuntimeError("Final M6 records used a different GPU/driver than the frozen pilot")
    rows = []
    for record in measure_records:
        truth = truth_record_for(record, truth_records)
        if truth is None and not record["spec"]["motion"]:
            raise RuntimeError(f"Missing scored truth for {record['run_id']}")
        rows.append(scored_row(record, truth, threshold))
    repetitions = collapse_repetitions(rows)
    aggregates = aggregate_rows(repetitions, int(suite["repetitions"]))
    canonical = canonical_comparisons(canonical_records)
    warm_rows = [row for row in rows if row["cache_state"] == "warm"]
    non_exact_cache_rows = [row for row in warm_rows if row["cache_pair_output_exact"] is False]
    investigated_cache_rows = [row for row in warm_rows if row["cache_pair_investigate"] is True]
    cache_pair_diagnostics = {
        "policy": suite["quality"]["cache_pair_output_policy"],
        "investigation_relative_l2_error": suite["quality"]["cache_pair_investigation_relative_l2_error"],
        "pair_count": len(warm_rows),
        "exact_pair_count": len(warm_rows) - len(non_exact_cache_rows),
        "non_exact_pair_count": len(non_exact_cache_rows),
        "investigate_pair_count": len(investigated_cache_rows),
        "maximum_relative_l2_error": max(
            (float(row["cache_pair_relative_l2_error"]) for row in warm_rows), default=0.0
        ),
        "investigate_run_ids": [row["run_id"] for row in investigated_cache_rows],
    }
    report = {
        "schema": 1,
        "type": "raytracer_m6_report",
        "evidence_fingerprint": evidence_fingerprint,
        "gpu": {"vendor_id": gpu[0], "device_id": gpu[1], "driver_version": gpu[2]},
        "suite_sha256": legacy.sha256(suite_path),
        "calibration_sha256": legacy.sha256(calibration_path),
        "truth_convergence_sha256": legacy.sha256(convergence_path),
        "pilot_records_sha256": record_set_digest(pilot_records),
        "truth_records_sha256": record_set_digest(truth_records),
        "quality_metric": suite["quality"]["metric"],
        "quality_threshold": threshold,
        "run_count": len(rows),
        "repetition_count": len(repetitions),
        "aggregate_count": len(aggregates),
        "quality_failures": [row["checkpoint_run_ids"] for row in repetitions if row["quality_pass"] is False],
        "cache_pair_diagnostics": cache_pair_diagnostics,
        "runs": rows,
        "repetitions": repetitions,
        "aggregates": aggregates,
        "canonical_comparisons": canonical,
    }
    write_json(output_root / "report.json", report)
    write_json(output_root / "aggregates.json", aggregates)
    write_json(output_root / "canonical-comparisons.json", canonical)
    write_csv(output_root / "runs.csv", rows)
    write_csv(output_root / "repetitions.csv", repetitions)
    write_csv(output_root / "aggregates.csv", aggregates)
    return report


def reference_patch_hashes() -> list[dict[str, object]]:
    return [
        {"path": str(path.resolve()), "sha256": legacy.sha256(path)}
        for path in (legacy.REFERENCE_PATCH, M6_REFERENCE_PATCH)
    ]


def load_frozen_reference_records(
    suite: dict[str, object],
) -> tuple[dict[str, object], list[dict[str, object]]]:
    config = suite["frozen_reference"]
    configured_root = Path(str(config["root"]))
    root = (configured_root if configured_root.is_absolute() else ROOT / configured_root).resolve()
    provenance_path = root / "provenance.json"
    if not provenance_path.is_file() or legacy.sha256(provenance_path) != config["provenance_sha256"]:
        raise RuntimeError("Frozen reference provenance is missing or changed")
    provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    expected_pins = {
        "evoengine_base": PINNED_EVOENGINE_BASE,
        "vk_gltf_renderer": legacy.PINNED_REFERENCE,
        "nvpro_core2": legacy.PINNED_NVPRO_CORE2,
        "niagara_bistro": legacy.PINNED_BISTRO_SOURCE,
    }
    if (
        provenance.get("schema") != 1
        or provenance.get("type") != "raytracer_m6_provenance"
        or provenance.get("pins") != expected_pins
        or provenance.get("comparison_sha256") != legacy.sha256(ROOT / "Scripts" / "compare_reference_render.py")
        or provenance.get("repositories", {}).get("vk_gltf_renderer", {}).get("working_diff_sha256")
        != EXPECTED_REFERENCE_DIFF_SHA256
    ):
        raise RuntimeError("Frozen reference provenance does not match the pinned M6 setup")
    archived_patch_hashes = [entry.get("sha256") for entry in provenance.get("reference_patches", [])]
    if archived_patch_hashes != [entry["sha256"] for entry in reference_patch_hashes()]:
        raise RuntimeError("Frozen reference patch provenance changed")
    frozen_camera = config["camera"]
    current_camera = camera_by_id(suite, str(frozen_camera["id"]))
    if any(
        frozen_camera[field] != current_camera[field]
        for field in ("reference_position", "reference_look_at", "evo_position", "evo_look_at")
    ):
        raise RuntimeError("Frozen reference camera mapping differs from the current M6 camera")
    archived_camera = provenance.get("camera_assets", {}).get(str(frozen_camera["id"]), {})
    if (
        archived_camera.get("position") != frozen_camera["reference_position"]
        or archived_camera.get("look_at") != frozen_camera["reference_look_at"]
    ):
        raise RuntimeError("Frozen reference camera mapping differs from the archived camera asset")

    records = []
    record_metadata = []
    provenance_fingerprint = provenance.get("evidence_fingerprint")
    for entry in config["records"]:
        relative = Path(str(entry["path"]))
        record_path = (root / relative).resolve()
        try:
            record_path.relative_to(root)
        except ValueError as error:
            raise RuntimeError(f"Frozen reference record escapes its root: {relative}") from error
        if not record_path.is_file() or legacy.sha256(record_path) != entry["sha256"]:
            raise RuntimeError(f"Frozen reference record is missing or changed: {relative}")
        record = json.loads(record_path.read_text(encoding="utf-8"))
        if record.get("status") != "complete" or record.get("evidence_fingerprint") != provenance_fingerprint:
            raise RuntimeError(f"Frozen reference record has invalid provenance: {relative}")
        spec = RunSpec(**record["spec"])
        if (
            spec.purpose != "measure"
            or spec.profile != "measure"
            or spec.suite != "cross-renderer"
            or spec.scene != "bistro"
            or spec.camera != "overview"
            or spec.renderer != "reference"
            or spec.technique != entry["technique"]
            or spec.variant != "specialized"
            or spec.cache_state != "cold"
            or spec.width != 1280
            or spec.height != 720
            or spec.spp != 512
            or spec.samples_per_frame != 4
            or spec.motion
        ):
            raise RuntimeError(f"Frozen reference record has the wrong benchmark specification: {relative}")
        validate_reference_record(spec, record["summary"], record.get("memory"))
        stem = record_path.name.removesuffix(".record.json")
        output_path = record_path.with_name(stem + ".hdr")
        log_path = record_path.with_name(stem + ".log")
        if (
            not output_path.is_file()
            or legacy.sha256(output_path) != record.get("output_sha256")
            or not log_path.is_file()
            or legacy.sha256(log_path) != record.get("log_sha256")
            or record.get("metrics_path") is not None
            or record.get("metrics_sha256") is not None
        ):
            raise RuntimeError(f"Frozen reference artifacts are missing or changed: {relative}")
        image = read_hdr(output_path)
        if image.width != 1280 or image.height != 720 or image.digest != record["output_sha256"]:
            raise RuntimeError(f"Frozen reference HDR is invalid: {relative}")
        validate_run_log(log_path, "reference")
        record = dict(record)
        record["output"] = str(output_path)
        record["log"] = str(log_path)
        records.append(record)
        record_metadata.append(
            {
                "technique": spec.technique,
                "run_id": spec.run_id,
                "record": str(record_path),
                "record_sha256": entry["sha256"],
                "output": str(output_path),
                "output_sha256": record["output_sha256"],
                "log": str(log_path),
                "log_sha256": record["log_sha256"],
            }
        )
    if len(records) != 2 or {record["spec"]["technique"] for record in records} != {"rtx", "rq"}:
        raise RuntimeError("Frozen reference evidence must contain exactly one RTX and one RayQuery record")
    gpu = require_matched_gpu(records, "Frozen M6 reference")
    metadata = {
        "mode": "frozen_historical",
        "same_session": False,
        "performance_use": "historical_context_only",
        "root": str(root),
        "provenance": str(provenance_path),
        "provenance_sha256": config["provenance_sha256"],
        "evidence_fingerprint": provenance_fingerprint,
        "record_set_digest": record_set_digest(records),
        "gpu": {"vendor_id": gpu[0], "device_id": gpu[1], "driver_version": gpu[2]},
        "records": record_metadata,
    }
    return metadata, records


def prepare_environment(
    suite: dict[str, object], suite_path: Path, args: argparse.Namespace
) -> tuple[dict[str, dict[str, object]], dict[str, object]]:
    output_root = args.output_dir.resolve()
    editor = args.editor.resolve()
    reference_root = args.reference_root.resolve()
    reference_exe = (
        args.reference_exe.resolve()
        if args.reference_exe
        else reference_root / "_bin" / "RelWithDebInfo" / "vk_gltf_renderer.exe"
    )
    nvpro_root = args.nvpro_root.resolve()
    evo_build_dir = args.evo_build_dir.resolve()
    reference_build_dir = (
        args.reference_build_dir.resolve() if args.reference_build_dir else reference_root / "build-m0"
    )
    fresh_reference = "reference" in suite.get("capture_renderers", ("reference", "evo"))
    frozen_reference = None
    if "frozen_reference" in suite:
        frozen_reference, _ = load_frozen_reference_records(suite)
    required = (suite_path, legacy.REFERENCE_PATCH, M6_REFERENCE_PATCH)
    for path in required:
        if not path.is_file():
            raise FileNotFoundError(path)
    if not args.dry_run:
        required_binaries = [editor]
        if fresh_reference:
            required_binaries.extend((reference_exe, reference_root / "resources" / "std_env.hdr"))
        for path in required_binaries:
            if not path.is_file():
                raise FileNotFoundError(path)
    legacy.git(ROOT, "merge-base", "--is-ancestor", PINNED_EVOENGINE_BASE, "HEAD")
    reference_state = None
    nvpro_state = None
    reference_options = None
    if fresh_reference:
        reference_state = legacy.repo_state(reference_root)
        nvpro_state = legacy.repo_state(nvpro_root)
        if reference_state["head"] != legacy.PINNED_REFERENCE:
            raise RuntimeError("Reference worktree is not at the pinned revision")
        if nvpro_state["head"] != legacy.PINNED_NVPRO_CORE2 or nvpro_state["dirty"]:
            raise RuntimeError("nvpro_core2 must be clean at the pinned revision")
        if reference_state["working_diff_sha256"] != EXPECTED_REFERENCE_DIFF_SHA256:
            raise RuntimeError("Reference tracked changes do not match the combined M0+M6 instrumentation")
        reference_options = legacy.cmake_cache_values(
            reference_build_dir / "CMakeCache.txt", ("USE_DLSS", "USE_OPTIX_DENOISER", "NvproCore2_ROOT")
        )
        if any(reference_options.get(name) != "OFF" for name in ("USE_DLSS", "USE_OPTIX_DENOISER")):
            raise RuntimeError("Reference build must disable DLSS and OptiX denoising")
        configured_nvpro = Path(reference_options.get("NvproCore2_ROOT", "")) / "nvpro_core2"
        if configured_nvpro.resolve() != nvpro_root:
            raise RuntimeError(f"Reference build uses unexpected nvpro_core2 root: {configured_nvpro}")
        unexpected_reference_files = [
            line
            for line in reference_state["status"]
            if line.startswith("?? ") and not line[3:].replace("\\", "/").startswith(("build-m0/", "_bin/"))
        ]
        if unexpected_reference_files:
            raise RuntimeError(f"Reference worktree contains unexpected untracked files: {unexpected_reference_files}")
    if not args.dry_run:
        legacy.verify_evo_install_matches_build(editor, evo_build_dir)
    if not args.skip_evo_prepare:
        legacy.prepare_evo_bistro(output_root, "m6", args.dry_run)
    camera_assets = {
        str(camera["id"]): generate_reference_camera_asset(legacy.BISTRO_SOURCE_ROOT, camera, legacy.BISTRO_SOURCE_ROOT)
        for camera in suite["cross_renderer"]["cameras"]
    }
    bistro_state = legacy.repo_state(legacy.BISTRO_SOURCE_ROOT)
    if bistro_state["head"] != legacy.PINNED_BISTRO_SOURCE:
        raise RuntimeError("Niagara Bistro source is not at the pinned revision")
    allowed_bistro_files = {
        "bistro-directional-intensity-10.gltf",
        *(f"bistro-m6-{camera['id']}.gltf" for camera in suite["cross_renderer"]["cameras"]),
    }
    unexpected_bistro_files = [
        line for line in bistro_state["status"] if line.startswith("?? ") and line[3:].replace("\\", "/") not in allowed_bistro_files
    ]
    if unexpected_bistro_files or any(not line.startswith("?? ") for line in bistro_state["status"]):
        raise RuntimeError(f"Pinned Bistro source contains unexpected changes: {bistro_state['status']}")
    source_dependencies = legacy.gltf_dependency_manifest(legacy.BISTRO_SOURCE_ROOT / "bistro.gltf")
    source_external = {
        entry["path"]: entry["sha256"]
        for entry in source_dependencies["files"]
        if entry["path"] != "bistro.gltf"
    }
    for asset in camera_assets.values():
        generated_name = Path(asset["output"]).name
        external = {
            entry["path"]: entry["sha256"]
            for entry in asset["dependencies"]["files"]
            if entry["path"] != generated_name
        }
        if external != source_external:
            raise RuntimeError(f"Generated reference camera {asset['camera']} changed the Bistro dependency closure")
    evo_asset = (
        legacy.EVO_BISTRO_PROJECT_ROOT / "Assets" / "Models" / "Bistro" / "bistro.gltf"
    ).resolve()
    evo_assets = legacy.gltf_dependency_manifest(evo_asset) if evo_asset.is_file() else None
    if not args.dry_run and (
        evo_assets is None or evo_assets["combined_sha256"] != legacy.EXPECTED_EVO_BISTRO_CLOSURE
    ):
        raise RuntimeError("EvoEngine Bistro dependency closure does not match the pinned baseline")
    provenance = {
        "schema": 1,
        "type": "raytracer_m6_provenance",
        "pins": {
            "evoengine_base": PINNED_EVOENGINE_BASE,
            "vk_gltf_renderer": legacy.PINNED_REFERENCE,
            "nvpro_core2": legacy.PINNED_NVPRO_CORE2,
            "niagara_bistro": legacy.PINNED_BISTRO_SOURCE,
        },
        "suite": {"path": str(suite_path), "sha256": legacy.sha256(suite_path)},
        "runner": {"path": str(Path(__file__).resolve()), "sha256": legacy.sha256(Path(__file__).resolve())},
        "comparison_sha256": legacy.sha256(ROOT / "Scripts" / "compare_reference_render.py"),
        "reference_patches": reference_patch_hashes(),
        "frozen_reference": frozen_reference,
        "repositories": {
            "evoengine": legacy.repo_state(ROOT),
            "vk_gltf_renderer": reference_state,
            "nvpro_core2": nvpro_state,
            "niagara_bistro": bistro_state,
        },
        "builds": {
            "evoengine": legacy.cmake_build_metadata(evo_build_dir, "RelWithDebInfo"),
            "vk_gltf_renderer": (
                legacy.cmake_build_metadata(reference_build_dir, "RelWithDebInfo") if fresh_reference else None
            ),
            "reference_options": reference_options,
        },
        "binaries": {
            "evoengine": legacy.runtime_binary_manifest(editor) if editor.is_file() else None,
            "reference": (
                legacy.runtime_binary_manifest(reference_exe) if fresh_reference and reference_exe.is_file() else None
            ),
        },
        "camera_assets": camera_assets,
        "assets": {"evoengine": evo_assets, "reference_source": source_dependencies},
        "evo_project_prepared": not args.skip_evo_prepare,
    }
    provenance["evidence_fingerprint"] = json_sha256(provenance)
    provenance_path = output_root / "provenance.json"
    if provenance_path.is_file():
        existing = json.loads(provenance_path.read_text(encoding="utf-8"))
        if existing.get("evidence_fingerprint") != provenance["evidence_fingerprint"]:
            raise RuntimeError("Existing M6 evidence directory belongs to different source/build provenance")
    else:
        write_json(provenance_path, provenance)
    return camera_assets, provenance


def run_self_test() -> int:
    suite = load_suite(DEFAULT_SUITE)
    measure = expand_measure_specs(suite)
    if len(measure) != 3:
        raise AssertionError(len(measure))
    if len({spec.run_id for spec in measure}) != len(measure):
        raise AssertionError("Run IDs are not unique")
    expected_keys = {("evo", technique, "cold") for technique in ("rtx", "rq", "query-only")}
    actual_keys = {(spec.renderer, spec.technique, spec.cache_state) for spec in measure}
    if actual_keys != expected_keys or any(
        spec.camera != "overview" or spec.variant != "specialized" or spec.spp != 512 or spec.repetition != 1
        for spec in measure
    ):
        raise AssertionError("Compact M6 matrix changed")
    if percentile([8.0, 2.0, 6.0, 4.0], 0.5) != 5.0 or percentile([1.0, 2.0, 3.0], 0.95) != 2.9:
        raise AssertionError("Percentile interpolation changed")
    exceedances = comparison_limit_exceedances(
        {"relative_l2_error": 0.005, "rms_error": 0.0011, "mean_abs_error": 0.00002},
        {"relative_l2_error": 0.004},
    )
    if exceedances != {"relative_l2_error": 0.005}:
        raise AssertionError("Cache-pair investigation limits changed")
    identity = look_at_quaternion([0.0, 0.0, 0.0], [0.0, 0.0, -1.0])
    if any(abs(identity[index] - expected) > 1.0e-12 for index, expected in enumerate((0.0, 0.0, 0.0, 1.0))):
        raise AssertionError(identity)
    synthetic = []
    for spp, quality in ((16, False), (32, True), (64, True)):
        synthetic.append(
            {
                "run_id": str(spp),
                "suite": "cross-renderer",
                "scene": "bistro",
                "camera": "overview",
                "renderer": "evo",
                "technique": "rtx",
                "variant": "full",
                "cache_state": "cold",
                "repetition": 1,
                "spp": spp,
                "quality_pass": quality,
                "time_to_quality_seconds": float(spp),
            }
        )
    collapsed = collapse_repetitions(synthetic)
    if len(collapsed) != 1 or collapsed[0]["selected_quality_spp"] != 32:
        raise AssertionError("Time-to-quality did not select the first passing checkpoint")
    print("M6 runner self-test passed")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--phase",
        choices=("plan", "pilot", "references", "measure", "canonical", "analyze", "all", "self-test"),
        default="plan",
    )
    parser.add_argument("--suite-manifest", type=Path, default=DEFAULT_SUITE)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "out" / "raytracer-m6")
    parser.add_argument(
        "--editor", type=Path, default=ROOT / "out" / "install" / "vs2026-x64" / "bin" / "EvoEngineEditor.exe"
    )
    parser.add_argument(
        "--reference-root", type=Path, default=ROOT / "out" / "reference" / "vk_gltf_renderer"
    )
    parser.add_argument("--reference-exe", type=Path)
    parser.add_argument("--reference-build-dir", type=Path)
    parser.add_argument("--nvpro-root", type=Path, default=ROOT.parent / "nvpro_core2")
    parser.add_argument("--evo-build-dir", type=Path, default=ROOT / "out" / "build" / "vs2026-x64")
    parser.add_argument("--suite", choices=("all", "cross-renderer", "evo-only"), default="all")
    parser.add_argument("--camera", choices=("all", "overview", "rooftop", "alley", "default"), default="all")
    parser.add_argument("--renderer", choices=("all", "reference", "evo"), default="all")
    parser.add_argument("--technique", choices=("all", "rtx", "rq", "query-only"), default="all")
    parser.add_argument("--variant", choices=("all", "full", "specialized"), default="all")
    parser.add_argument("--cache-state", choices=("all", "cold", "warm"), default="all")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--skip-evo-prepare", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def plan_specs(suite: dict[str, object], args: argparse.Namespace) -> dict[str, list[RunSpec]]:
    if suite.get("validation_mode") == "compact":
        measure = filter_specs(expand_measure_specs(suite), args)
        if len(measure) > 3:
            raise RuntimeError(f"Compact M6 plan exceeds the 3-capture benchmark cap: {len(measure)}")
        return {
            "pilot_truth_low": [],
            "pilot_truth": [],
            "pilot": [],
            "measure_truth_low": [],
            "measure_truth": [],
            "measure": measure,
            "canonical": [],
        }
    return {
        "pilot_truth_low": filter_specs(expand_truth_specs(suite, "pilot", True), args),
        "pilot_truth": filter_specs(expand_truth_specs(suite, "pilot"), args),
        "pilot": filter_specs(expand_pilot_specs(suite), args),
        "measure_truth_low": filter_specs(expand_truth_specs(suite, "measure", True), args),
        "measure_truth": filter_specs(expand_truth_specs(suite, "measure"), args),
        "measure": filter_specs(expand_measure_specs(suite), args),
        "canonical": filter_specs(expand_canonical_specs(suite), args),
    }


def write_plan(
    suite: dict[str, object], suite_path: Path, output_root: Path, groups: dict[str, list[RunSpec]], dry_run: bool
) -> dict[str, object]:
    plan = {
        "schema": 1,
        "type": "raytracer_m6_plan",
        "suite": str(suite_path),
        "suite_sha256": legacy.sha256(suite_path),
        "dry_run": dry_run,
        "counts": {name: len(specs) for name, specs in groups.items()},
        "total_runs": sum(len(specs) for specs in groups.values()),
        "groups": {
            name: [{"run_id": spec.run_id, **asdict(spec)} for spec in specs] for name, specs in groups.items()
        },
    }
    if suite.get("validation_mode") == "compact":
        plan.update(
            {
                "fresh_benchmark_capture_count": plan["total_runs"],
                "frozen_reference_record_count": len(suite["frozen_reference"]["records"]),
                "post_commit_delivery_capture_count": 1,
                "total_new_capture_count_including_delivery": plan["total_runs"] + 1,
            }
        )
    write_json(output_root / ("plan.dry-run.json" if dry_run else "plan.json"), plan)
    return plan


def full_selection(args: argparse.Namespace) -> bool:
    return (
        all(
            getattr(args, field) == "all"
            for field in ("suite", "camera", "renderer", "technique", "variant", "cache_state")
        )
        and not args.skip_evo_prepare
    )


def main() -> int:
    args = parse_args()
    if args.phase == "self-test":
        return run_self_test()
    suite_path = args.suite_manifest.resolve()
    suite = load_suite(suite_path)
    output_root = args.output_dir.resolve()
    groups = plan_specs(suite, args)
    plan = write_plan(suite, suite_path, output_root, groups, args.dry_run)
    plan_name = "plan.dry-run.json" if args.dry_run else "plan.json"
    print(f"M6 plan: {plan['total_runs']} runs -> {output_root / plan_name}", flush=True)
    if args.phase == "plan":
        return 0

    if args.phase in ("all", "analyze") and not full_selection(args):
        raise RuntimeError("Full M6 analysis does not permit matrix filters or --skip-evo-prepare")
    camera_assets, provenance = prepare_environment(suite, suite_path, args)
    fingerprint = str(provenance["evidence_fingerprint"])
    if suite.get("validation_mode") == "compact":
        if args.phase in ("measure", "all"):
            run_specs(groups["measure"], suite, args, camera_assets, fingerprint)
        if args.phase in ("analyze", "all") and not args.dry_run:
            report = analyze_compact(suite, suite_path, output_root, groups["measure"], fingerprint)
            print(f"M6 compact report: {report['run_count']} runs -> {output_root / 'report.json'}", flush=True)
        return 0
    if args.phase in ("pilot", "all"):
        run_specs(groups["pilot_truth_low"], suite, args, camera_assets, fingerprint)
        run_specs(groups["pilot_truth"], suite, args, camera_assets, fingerprint)
        run_specs(groups["pilot"], suite, args, camera_assets, fingerprint)
        if not args.dry_run and full_selection(args):
            calibration = calibrate_quality(
                suite,
                suite_path,
                output_root,
                [*groups["pilot_truth_low"], *groups["pilot_truth"], *groups["pilot"]],
                fingerprint,
            )
            print(
                f"Frozen relative-L2 threshold: {calibration['relative_l2_threshold']:.8g}",
                flush=True,
            )
    if args.phase in ("references", "all"):
        run_specs(groups["measure_truth_low"], suite, args, camera_assets, fingerprint)
        run_specs(groups["measure_truth"], suite, args, camera_assets, fingerprint)
        if not args.dry_run and full_selection(args):
            validate_measure_truth_convergence(
                suite,
                output_root,
                [*groups["measure_truth_low"], *groups["measure_truth"]],
                fingerprint,
            )
    if args.phase in ("measure", "all"):
        run_specs(groups["measure"], suite, args, camera_assets, fingerprint)
    if args.phase in ("canonical", "all"):
        run_specs(groups["canonical"], suite, args, camera_assets, fingerprint)
    if args.phase in ("analyze", "all") and not args.dry_run:
        report = analyze(
            suite,
            suite_path,
            output_root,
            [*groups["pilot_truth_low"], *groups["pilot_truth"], *groups["pilot"]],
            groups["measure"],
            [*groups["measure_truth_low"], *groups["measure_truth"]],
            groups["canonical"],
            fingerprint,
        )
        print(
            f"M6 report: {report['run_count']} runs, {report['aggregate_count']} aggregates -> "
            f"{output_root / 'report.json'}",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
