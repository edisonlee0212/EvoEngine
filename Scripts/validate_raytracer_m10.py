#!/usr/bin/env python3
"""Validate two fresh M10 ray-transport atlases with retained query-only provenance."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
from collections import Counter
from pathlib import Path

import validate_ray_debug_views as m7

PRIMARY_LANES = ("rtx", "rq")
M10_CAMERA_POSITION = (48.0, 1.5, 5.6)
M10_CAMERA_LOOK_AT = (48.0, 1.15, -1.4)
REPO_ROOT = Path(__file__).resolve().parents[1]
ARTIFACT_HASH_MODE = "sha256-byte-exact-v1"
SHADER_SOURCE_HASH_MODE = "sha256-lf-normalized-v1"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--editor", type=Path)
    parser.add_argument("--matrix-dir", type=Path, default=Path("out/m10-validation/atlas"))
    parser.add_argument("--out", type=Path)
    parser.add_argument(
        "--expectations",
        type=Path,
        default=Path(__file__).with_name("raytracer_m10_expectations.json"),
    )
    parser.add_argument(
        "--retained-query-only-dir",
        type=Path,
        help="Reuse an earlier rq/rq-only pair while replacing only the RTX and RayQuery captures.",
    )
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--timeout-seconds", type=int, default=900)
    return parser.parse_args()


def capture_command(editor: Path, directory: Path, name: str) -> list[str]:
    command = m7.capture_command(editor, directory, name)
    command[command.index("--preview-camera-position") + 1] = ",".join(
        str(value) for value in M10_CAMERA_POSITION
    )
    command[command.index("--preview-camera-look-at") + 1] = ",".join(
        str(value) for value in M10_CAMERA_LOOK_AT
    )
    command.append("--preview-m10-ray-transport")
    return command


def capture_lanes(args: argparse.Namespace) -> tuple[str, ...]:
    if not args.retained_query_only_dir:
        raise ValueError("M10 replacement validation requires --retained-query-only-dir.")
    return PRIMARY_LANES


def validate_directories(args: argparse.Namespace) -> None:
    if not args.retained_query_only_dir:
        raise ValueError("M10 replacement validation requires --retained-query-only-dir.")
    if args.matrix_dir.resolve() == args.retained_query_only_dir.resolve():
        raise ValueError("Replacement output must not overwrite the retained query-only directory.")


def retained_proof_preflight(args: argparse.Namespace) -> None:
    expectations = json.loads(args.expectations.read_text(encoding="utf-8"))
    proof = expectations.get("retained_query_only_proof", {})
    artifacts = proof.get("artifacts", {})
    sources = proof.get("shader_sources", {})
    if proof.get("artifact_hash_mode") != ARTIFACT_HASH_MODE:
        raise ValueError("Retained artifact hash mode is missing or unsupported.")
    if proof.get("shader_source_hash_mode") != SHADER_SOURCE_HASH_MODE:
        raise ValueError("Retained shader-source hash mode is missing or unsupported.")
    if len(artifacts) != 8 or len(sources) != 10:
        raise ValueError("Retained query-only provenance is incomplete.")
    for filename, expected_hash in artifacts.items():
        path = args.retained_query_only_dir / filename
        if not path.is_file() or sha256(path) != expected_hash:
            raise ValueError(f"Retained query-only artifact changed or is missing: {path}")
    for filename, expected_hash in sources.items():
        path = REPO_ROOT / filename
        if not path.is_file() or sha256_lf_normalized(path) != expected_hash:
            raise ValueError(f"A retained-proof ray shader changed or is missing: {path}")


def run_captures(args: argparse.Namespace) -> None:
    if not args.editor or not args.editor.is_file():
        raise ValueError("--capture and --dry-run require an existing --editor executable")
    validate_directories(args)
    retained_proof_preflight(args)
    for name in capture_lanes(args):
        for extension in ("hdr", "json", "log"):
            path = args.matrix_dir / f"{name}.{extension}"
            if path.exists():
                raise FileExistsError(f"Refusing to overwrite M10 capture evidence: {path}")
    args.matrix_dir.mkdir(parents=True, exist_ok=True)
    for name in capture_lanes(args):
        command = capture_command(args.editor, args.matrix_dir, name)
        print(json.dumps(command))
        if args.dry_run:
            continue
        environment = os.environ.copy()
        environment["EVOENGINE_SHADER_CACHE_DIR"] = str(
            (args.matrix_dir / "shader-cache" / name).resolve()
        )
        environment["EVOENGINE_IMGUI_INI_PATH"] = str(
            (args.matrix_dir / f"{name}.imgui.ini").resolve()
        )
        log_path = args.matrix_dir / f"{name}.log"
        with log_path.open("w", encoding="utf-8") as log:
            result = subprocess.run(
                command,
                cwd=Path(__file__).resolve().parents[1],
                env=environment,
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=args.timeout_seconds,
                check=False,
            )
        log_text = log_path.read_text(encoding="utf-8", errors="replace")
        fatal_matches = [pattern.pattern for pattern in m7.FATAL_LOG_PATTERNS if pattern.search(log_text)]
        if result.returncode or fatal_matches:
            tail = "\n".join(log_text.splitlines()[-40:])
            raise RuntimeError(
                f"{name} capture failed with exit code {result.returncode}; "
                f"fatal patterns={fatal_matches}:\n{tail}"
            )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def lf_normalized(data: bytes) -> bytes:
    return data.replace(b"\r\n", b"\n").replace(b"\r", b"\n")


def sha256_lf_normalized(path: Path) -> str:
    return hashlib.sha256(lf_normalized(path.read_bytes())).hexdigest()


def roi_statistics(values: list[float]) -> dict[str, object]:
    mean_rgb = m7.mean_rgb(values)
    luminances = [
        0.2126 * values[index] + 0.7152 * values[index + 1] + 0.0722 * values[index + 2]
        for index in range(0, len(values), 3)
    ]
    mean_luminance = sum(luminances) / max(len(luminances), 1)
    variance = sum((value - mean_luminance) ** 2 for value in luminances) / max(
        len(luminances), 1
    )
    return {
        "mean_rgb": mean_rgb,
        "mean_luminance": mean_luminance,
        "contrast": math.sqrt(variance) / max(mean_luminance, 1.0e-8),
    }


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    position = min(max(fraction, 0.0), 1.0) * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def matched_frequency_response(
    luminances: list[float],
    width: int,
    origin_x: int,
    origin_y: int,
    frequency_x: float,
    frequency_y: float,
    winsor_low: float,
    winsor_high: float,
) -> float | None:
    low = percentile(luminances, winsor_low)
    high = percentile(luminances, winsor_high)
    centered = [min(max(value, low), high) for value in luminances]
    median = percentile(centered, 0.5)
    centered = [value - median for value in centered]
    magnitude = sum(abs(value) for value in centered)
    if magnitude <= 1.0e-12:
        return None

    amplitudes: list[float] = []
    for y_sign in (-1.0, 1.0):
        real = 0.0
        imaginary = 0.0
        for index, value in enumerate(centered):
            x = origin_x + index % width
            y = origin_y + index // width
            phase = 2.0 * math.pi * (frequency_x * x + y_sign * frequency_y * y)
            real += value * math.cos(phase)
            imaginary -= value * math.sin(phase)
        amplitudes.append(math.hypot(real, imaginary) / magnitude)
    return math.sqrt(sum(value * value for value in amplitudes) / len(amplitudes))


def checker_frequency_result(image: m7.HdrImage, config: dict[str, object]) -> dict[str, object]:
    patch_size = int(config["patch_size"])
    minimum_patch_count = int(config["minimum_patch_count"])
    minimum_valid_fraction = float(config["minimum_valid_fraction"])
    checker_resolution = int(config["checker_resolution"])
    winsor_low = float(config["winsor_low"])
    winsor_high = float(config["winsor_high"])
    material_values = m7.view_values(image, m7.VIEW_NAMES.index("Material ID"))
    material_ids = [
        tuple(material_values[index : index + 3])
        for index in range(0, len(material_values), 3)
    ]
    modes = Counter(value for value in material_ids if max(value) > 1.0e-6).most_common(2)
    result: dict[str, object] = {"valid": False, "reasons": []}
    reasons = result["reasons"]
    if len(modes) != 2:
        reasons.append("fewer than two non-black Material ID modes")
        return result

    panel_id, panel_count = modes[0]
    volume_id, volume_count = modes[1]

    def bounds(material_id: tuple[float, ...]) -> tuple[int, int, int, int]:
        coordinates = [
            (index % 256, index // 256)
            for index, value in enumerate(material_ids)
            if value == material_id
        ]
        return (
            min(x for x, _ in coordinates),
            min(y for _, y in coordinates),
            max(x for x, _ in coordinates) + 1,
            max(y for _, y in coordinates) + 1,
        )

    panel_bounds = bounds(panel_id)
    volume_bounds = bounds(volume_id)
    panel_width = panel_bounds[2] - panel_bounds[0]
    panel_height = panel_bounds[3] - panel_bounds[1]
    volume_width = volume_bounds[2] - volume_bounds[0]
    volume_height = volume_bounds[3] - volume_bounds[1]
    if panel_count <= volume_count:
        reasons.append("checker panel is not the dominant non-black Material ID")
    if not (
        panel_bounds[0] < volume_bounds[0] < volume_bounds[2] < panel_bounds[2]
        and panel_bounds[1] < volume_bounds[1] < volume_bounds[3] < panel_bounds[3]
    ):
        reasons.append("volume Material ID bounds are not strictly inside the checker panel")
    if min(panel_width, panel_height, volume_width, volume_height) < patch_size:
        reasons.append("fixture Material ID bounds are smaller than the analysis patch")
    if reasons:
        return result

    cycles = checker_resolution / 2.0

    def alias_frequency(frequency: float) -> float:
        fraction = frequency % 1.0
        return min(fraction, 1.0 - fraction)

    frequency_x = alias_frequency(cycles / panel_width)
    frequency_y = alias_frequency(cycles / panel_height)
    beauty_values = m7.view_values(image, m7.VIEW_NAMES.index("Beauty"))
    beauty_luminances = [
        0.2126 * beauty_values[index]
        + 0.7152 * beauty_values[index + 1]
        + 0.0722 * beauty_values[index + 2]
        for index in range(0, len(beauty_values), 3)
    ]

    def patch_responses(
        material_id: tuple[float, ...], material_bounds: tuple[int, int, int, int]
    ) -> tuple[int, list[float]]:
        eligible_count = 0
        responses: list[float] = []
        for top in range(material_bounds[1], material_bounds[3] - patch_size + 1):
            for left in range(material_bounds[0], material_bounds[2] - patch_size + 1):
                indices = [
                    y * 256 + x
                    for y in range(top, top + patch_size)
                    for x in range(left, left + patch_size)
                ]
                if any(material_ids[index] != material_id for index in indices):
                    continue
                eligible_count += 1
                response = matched_frequency_response(
                    [beauty_luminances[index] for index in indices],
                    patch_size,
                    left,
                    top,
                    frequency_x,
                    frequency_y,
                    winsor_low,
                    winsor_high,
                )
                if response is not None:
                    responses.append(response)
        return eligible_count, responses

    panel_patch_count, panel_responses = patch_responses(panel_id, panel_bounds)
    volume_patch_count, volume_responses = patch_responses(volume_id, volume_bounds)
    for name, eligible_count, responses in (
        ("clear", panel_patch_count, panel_responses),
        ("volume", volume_patch_count, volume_responses),
    ):
        if eligible_count < minimum_patch_count:
            reasons.append(f"{name} Material ID has too few eligible patches")
        if len(responses) < math.ceil(eligible_count * minimum_valid_fraction):
            reasons.append(f"{name} Material ID has too few nonconstant patches")

    clear_response = percentile(panel_responses, 0.5)
    volume_response = percentile(volume_responses, 0.5)
    result.update(
        {
            "valid": not reasons,
            "panel_material_id": list(panel_id),
            "volume_material_id": list(volume_id),
            "panel_pixel_count": panel_count,
            "volume_pixel_count": volume_count,
            "panel_bounds": list(panel_bounds),
            "volume_bounds": list(volume_bounds),
            "clear_patch_count": panel_patch_count,
            "volume_patch_count": volume_patch_count,
            "clear_valid_patch_count": len(panel_responses),
            "volume_valid_patch_count": len(volume_responses),
            "frequency_cycles_per_pixel": [frequency_x, frequency_y],
            "clear_response": clear_response,
            "volume_response": volume_response,
            "clear_to_volume_ratio": clear_response / max(volume_response, 1.0e-12),
        }
    )
    return result


def volume_cone_lod_result(config: dict[str, object]) -> dict[str, float]:
    ray_spread = 2.0 * float(config["inverse_projection_y"]) / float(config["resolution_y"])
    entry_width = float(config["entry_distance"]) * ray_spread
    scatter_width = entry_width + float(config["scatter_distance"]) * ray_spread
    next_hit_distance = float(config["next_hit_distance"])
    incidence = float(config["incidence_cosine"])
    texel_density = float(config["texel_density"])
    texture_extent = float(config["texture_extent"])
    gradient_before = (
        (entry_width + next_hit_distance * ray_spread) / incidence * texel_density
    )
    gradient_after = (
        (scatter_width + next_hit_distance * ray_spread) / incidence * texel_density
    )
    lod_before = math.log2(max(texture_extent * gradient_before, 1.0e-12))
    lod_after = math.log2(max(texture_extent * gradient_after, 1.0e-12))
    return {
        "ray_spread": ray_spread,
        "gradient_before_scatter": gradient_before,
        "gradient_after_scatter": gradient_after,
        "lod_before_scatter": lod_before,
        "lod_after_scatter": lod_after,
        "lod_increase": lod_after - lod_before,
    }


def fixture_results(
    images: dict[str, m7.HdrImage], expectations: dict[str, object]
) -> tuple[dict[str, object], dict[str, bool]]:
    results: dict[str, object] = {}
    gates: dict[str, bool] = {}
    rois = expectations.get("m10_rois", [])
    for roi in rois if isinstance(rois, list) else []:
        if not isinstance(roi, dict):
            continue
        name = str(roi["name"])
        view = m7.VIEW_NAMES.index(str(roi["view"]))
        rect = [int(value) for value in roi["rect"]]
        lane_stats = {
            lane: roi_statistics(m7.view_values(images[lane], view, rect))
            for lane in ("rtx", "rq")
        }
        results[name] = lane_stats
        for lane, stats in lane_stats.items():
            luminance = float(stats["mean_luminance"])
            contrast = float(stats["contrast"])
            if "min_luminance" in roi:
                gates[f"m10_roi_{name}_{lane}_min_luminance"] = luminance >= float(
                    roi["min_luminance"]
                )
            if "max_luminance" in roi:
                gates[f"m10_roi_{name}_{lane}_max_luminance"] = luminance <= float(
                    roi["max_luminance"]
                )
            if "min_contrast" in roi:
                gates[f"m10_roi_{name}_{lane}_min_contrast"] = contrast >= float(
                    roi["min_contrast"]
                )
            if "max_contrast" in roi:
                gates[f"m10_roi_{name}_{lane}_max_contrast"] = contrast <= float(
                    roi["max_contrast"]
                )
        if "cross_technique_luminance_max" in roi:
            gates[f"m10_roi_{name}_cross_technique_luminance"] = abs(
                float(lane_stats["rtx"]["mean_luminance"])
                - float(lane_stats["rq"]["mean_luminance"])
            ) <= float(roi["cross_technique_luminance_max"])

    relations = expectations.get("m10_relations", [])
    for relation in relations if isinstance(relations, list) else []:
        if not isinstance(relation, dict):
            continue
        name = str(relation["name"])
        lhs = str(relation["lhs"])
        rhs = str(relation["rhs"])
        kind = str(relation["kind"])
        relation_results: dict[str, float] = {}
        for lane in ("rtx", "rq"):
            if kind == "luminance_difference":
                value = float(results[lhs][lane]["mean_luminance"]) - float(
                    results[rhs][lane]["mean_luminance"]
                )
            elif kind == "luminance_ratio":
                value = float(results[lhs][lane]["mean_luminance"]) / max(
                    float(results[rhs][lane]["mean_luminance"]), 1.0e-8
                )
            elif kind == "contrast_ratio":
                value = float(results[lhs][lane]["contrast"]) / max(
                    float(results[rhs][lane]["contrast"]), 1.0e-8
                )
            else:
                raise ValueError(f"Unknown M10 relation kind: {kind}")
            relation_results[lane] = value
            if "minimum" in relation:
                gates[f"m10_relation_{name}_{lane}_minimum"] = value >= float(
                    relation["minimum"]
                )
            if "maximum" in relation:
                gates[f"m10_relation_{name}_{lane}_maximum"] = value <= float(
                    relation["maximum"]
                )
        results[f"relation:{name}"] = relation_results

    frequency_config = expectations.get("m10_checker_frequency", {})
    frequency_results = {
        lane: checker_frequency_result(image, frequency_config)
        for lane, image in images.items()
    }
    results["checker_frequency_suppression"] = frequency_results
    for lane, frequency in frequency_results.items():
        gates[f"m10_checker_frequency_{lane}_valid"] = bool(frequency["valid"])
        gates[f"m10_checker_frequency_{lane}_clear_response"] = (
            float(frequency.get("clear_response", 0.0))
            >= float(frequency_config["minimum_clear_response"])
        )
        gates[f"m10_checker_frequency_{lane}_suppression"] = (
            float(frequency.get("clear_to_volume_ratio", 0.0))
            >= float(frequency_config["minimum_clear_to_volume_ratio"])
        )
    return results, gates


def artifact_record(directory: Path, name: str) -> dict[str, object]:
    return {
        extension: {
            "path": str((directory / f"{name}.{extension}").resolve()),
            "sha256": sha256(directory / f"{name}.{extension}"),
        }
        for extension in ("hdr", "json", "log")
    }


def metrics_checks(
    name: str,
    metrics: dict[str, object],
    camera_position: tuple[float, float, float],
    camera_look_at: tuple[float, float, float],
) -> dict[str, bool]:
    checks = m7.metrics_checks(name, metrics)
    position = metrics.get("camera_position_override")
    look_at = metrics.get("camera_look_at_override")
    checks["camera_override"] = (
        isinstance(position, list)
        and isinstance(look_at, list)
        and len(position) == 3
        and len(look_at) == 3
        and all(
            math.isclose(float(value), expected, abs_tol=1.0e-6)
            for value, expected in zip(position, camera_position)
        )
        and all(
            math.isclose(float(value), expected, abs_tol=1.0e-6)
            for value, expected in zip(look_at, camera_look_at)
        )
    )
    return checks


def validate(args: argparse.Namespace) -> dict[str, object]:
    validate_directories(args)
    expectations = json.loads(args.expectations.read_text(encoding="utf-8"))
    images = {
        name: m7.read_hdr(args.matrix_dir / f"{name}.hdr") for name in PRIMARY_LANES
    }
    metrics = {
        name: json.loads((args.matrix_dir / f"{name}.json").read_text(encoding="utf-8"))
        for name in PRIMARY_LANES
    }
    proof_directory = args.retained_query_only_dir
    proof_images = {
        name: m7.read_hdr(proof_directory / f"{name}.hdr") for name in ("rq", "rq-only")
    }
    proof_metrics = {
        name: json.loads((proof_directory / f"{name}.json").read_text(encoding="utf-8"))
        for name in ("rq", "rq-only")
    }
    replacement_evidence = {
        name: metrics_checks(name, metric, M10_CAMERA_POSITION, M10_CAMERA_LOOK_AT)
        for name, metric in metrics.items()
    }
    proof_camera_position = m7.CAMERA_POSITION
    proof_camera_look_at = m7.CAMERA_LOOK_AT
    proof_evidence = {
        name: metrics_checks(name, metric, proof_camera_position, proof_camera_look_at)
        for name, metric in proof_metrics.items()
    }
    query_only_comparison = m7.compare_hdr_images(proof_images["rq"], proof_images["rq-only"])
    attributes = {
        m7.VIEW_NAMES[view]: m7.comparison(
            m7.view_values(images["rtx"], view), m7.view_values(images["rq"], view)
        )
        for view in m7.ATTRIBUTE_VIEWS
    }
    transport_limits = expectations.get("m10_transport_limits", m7.RTX_RQ_TRANSPORT_LIMITS)
    transport = {
        m7.VIEW_NAMES[view]: m7.transport_comparison(
            m7.view_values(images["rtx"], view),
            m7.view_values(images["rq"], view),
            transport_limits[m7.VIEW_NAMES[view]],
        )
        for view in m7.TRANSPORT_VIEWS
    }
    conservation = {name: m7.conservation(image) for name, image in images.items()}
    bounded = {
        name: all(
            math.isfinite(value) and -1.0e-6 <= value <= 1.0001
            for view in m7.BOUNDED_VIEWS
            for value in m7.view_values(image, view)
        )
        for name, image in images.items()
    }
    fixture, fixture_gates = fixture_results(images, expectations)
    volume_lod_config = expectations.get("m10_volume_cone_lod", {})
    volume_lod = volume_cone_lod_result(volume_lod_config)
    classifications = expectations.get("classifications", [])
    classifications_valid = (
        isinstance(classifications, list)
        and bool(classifications)
        and all(
            isinstance(item, dict)
            and item.get("category") in m7.ALLOWED_CLASSIFICATIONS
            and item.get("subject")
            for item in classifications
        )
    )

    retained_expectations = expectations.get("retained_query_only_proof", {})
    retained_artifact_results: dict[str, object] = {}
    retained_source_results: dict[str, object] = {}
    retained_artifacts_match = True
    retained_sources_match = True
    retained_artifact_hash_mode = retained_expectations.get("artifact_hash_mode")
    retained_source_hash_mode = retained_expectations.get("shader_source_hash_mode")
    if args.retained_query_only_dir:
        expected_artifacts = retained_expectations.get("artifacts", {})
        expected_sources = retained_expectations.get("shader_sources", {})
        retained_artifacts_match = isinstance(expected_artifacts, dict) and len(expected_artifacts) == 8
        retained_sources_match = isinstance(expected_sources, dict) and len(expected_sources) == 10
        for filename, expected_hash in expected_artifacts.items():
            path = proof_directory / filename
            actual_hash = sha256(path)
            retained_artifact_results[filename] = {
                "path": str(path.resolve()),
                "expected_sha256": expected_hash,
                "actual_sha256": actual_hash,
                "matches": actual_hash == expected_hash,
            }
            retained_artifacts_match = retained_artifacts_match and actual_hash == expected_hash
        for filename, expected_hash in expected_sources.items():
            actual_hash = sha256_lf_normalized(REPO_ROOT / filename)
            retained_source_results[filename] = {
                "expected_sha256": expected_hash,
                "actual_sha256": actual_hash,
                "matches": actual_hash == expected_hash,
            }
            retained_sources_match = retained_sources_match and actual_hash == expected_hash

    m7_contract = {
        "attribute_mae": m7.RTX_RQ_ATTRIBUTE_MAE_LIMIT,
        "attribute_rms": m7.RTX_RQ_ATTRIBUTE_RMS_LIMIT,
        "transport": m7.RTX_RQ_TRANSPORT_LIMITS,
        "conservation_relative_l2": m7.CONSERVATION_RELATIVE_L2_LIMIT,
    }
    m7_contract_sha256 = hashlib.sha256(
        json.dumps(m7_contract, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    roi_schema = expectations.get("m10_rois", [])
    relation_schema = expectations.get("m10_relations", [])
    frequency_schema = expectations.get("m10_checker_frequency", {})
    volume_lod_schema = expectations.get("m10_volume_cone_lod", {})
    fixture_schema_valid = (
        isinstance(roi_schema, list)
        and len(roi_schema) == 5
        and all(
            isinstance(roi, dict)
            and any(
                key in roi
                for key in (
                    "min_luminance",
                    "max_luminance",
                    "min_contrast",
                    "max_contrast",
                    "cross_technique_luminance_max",
                )
            )
            for roi in roi_schema
        )
        and isinstance(relation_schema, list)
        and len(relation_schema) == 1
        and all(
            isinstance(relation, dict) and ("minimum" in relation or "maximum" in relation)
            for relation in relation_schema
        )
        and isinstance(frequency_schema, dict)
        and all(
            key in frequency_schema
            for key in (
                "checker_resolution",
                "patch_size",
                "minimum_patch_count",
                "minimum_valid_fraction",
                "winsor_low",
                "winsor_high",
                "minimum_clear_response",
                "minimum_clear_to_volume_ratio",
            )
        )
        and isinstance(volume_lod_schema, dict)
        and all(
            key in volume_lod_schema
            for key in (
                "inverse_projection_y",
                "resolution_y",
                "entry_distance",
                "scatter_distance",
                "next_hit_distance",
                "incidence_cosine",
                "texel_density",
                "texture_extent",
                "minimum_lod_increase",
            )
        )
    )
    retained_variant_contract = all(
        metric.get("m10_ray_transport") is True
        and metric.get("ray_shader_variant", {}).get("active_key") == "rq:0x6ebf:debug"
        and metric.get("ray_shader_variant", {}).get("active_mask") == 28351
        for metric in proof_metrics.values()
    )
    fresh_variant_contract = (
        metrics["rq"].get("ray_shader_variant", {}).get("active_key") == "rq:0x6ebf:debug"
        and metrics["rq"].get("ray_shader_variant", {}).get("active_mask") == 28351
    )

    gates = {
        **{f"evidence_{name}": all(checks.values()) for name, checks in replacement_evidence.items()},
        **{f"retained_evidence_{name}": all(checks.values()) for name, checks in proof_evidence.items()},
        "retained_query_only_artifacts": retained_artifacts_match,
        "retained_query_only_shader_sources": retained_sources_match,
        "retained_query_only_artifact_hash_mode": retained_artifact_hash_mode == ARTIFACT_HASH_MODE,
        "retained_query_only_shader_source_hash_mode": retained_source_hash_mode == SHADER_SOURCE_HASH_MODE,
        "retained_query_only_variant_contract": retained_variant_contract,
        "fresh_query_variant_contract": fresh_variant_contract,
        "historical_query_independence_exact": bool(query_only_comparison.get("exact_match")),
        "fresh_fixture_version": all(
            metric.get("m10_fixture_version") == "isolated-v2" for metric in metrics.values()
        ),
        "m7_limits_unchanged": m7_contract_sha256 == expectations.get("m7_contract_sha256"),
        **{
            f"attribute_{name}": float(result["mean_abs_error"]) <= m7.RTX_RQ_ATTRIBUTE_MAE_LIMIT
            and float(result["rms_error"]) <= m7.RTX_RQ_ATTRIBUTE_RMS_LIMIT
            for name, result in attributes.items()
        },
        **{
            f"transport_{name}": m7.transport_passes(result, transport_limits[name])
            for name, result in transport.items()
        },
        **{
            f"conservation_{name}": float(result["relative_l2"]) <= m7.CONSERVATION_RELATIVE_L2_LIMIT
            for name, result in conservation.items()
        },
        **{f"bounded_{name}": passed for name, passed in bounded.items()},
        **{
            f"m10_fixture_{name}": metric.get("m10_ray_transport") is True
            for name, metric in metrics.items()
        },
        **fixture_gates,
        "m10_volume_cone_lod_numerical": float(volume_lod["lod_increase"])
        >= float(volume_lod_config["minimum_lod_increase"]),
        "classifications": classifications_valid,
        "m10_fixture_schema": fixture_schema_valid,
        "m10_fixture_calibrated": expectations.get("calibrated") is True and bool(fixture_gates),
        "approved_replacement_capture_count": len(PRIMARY_LANES) == 2,
        "reference_not_executed": True,
    }
    for name, image in images.items():
        m7.write_display_png(image, args.matrix_dir / f"{name}.png")
    return {
        "schema": 4,
        "milestone": "M10",
        "matrix_dir": str(args.matrix_dir.resolve()),
        "atlas": {
            "grid": [5, 4],
            "size": list(m7.SIZE),
            "storage_origin": "top-left",
            "view_origin": "bottom-left",
            "rows_top_to_bottom": [
                list(m7.VIEW_NAMES[row * 5 : (row + 1) * 5]) for row in range(3, -1, -1)
            ],
        },
        "limits": {
            "attribute_mae": m7.RTX_RQ_ATTRIBUTE_MAE_LIMIT,
            "attribute_rms": m7.RTX_RQ_ATTRIBUTE_RMS_LIMIT,
            "transport": transport_limits,
            "conservation_relative_l2": m7.CONSERVATION_RELATIVE_L2_LIMIT,
            "m10_checker_frequency": frequency_schema,
            "m10_volume_cone_lod": volume_lod_schema,
            "expectations_sha256": sha256(args.expectations),
        },
        "capture_plan": {
            "rejected_fixture_capture_count": 3,
            "replacement_capture_count": len(PRIMARY_LANES),
            "focused_capture_count": 5,
            "planned_delivery_capture_count": 1,
            "total_milestone_capture_count": 6,
            "reference_executed": False,
            "replacement_lanes": list(PRIMARY_LANES),
            "retained_query_only_directory": str(proof_directory.resolve()),
        },
        "capture_evidence": {
            "replacement": replacement_evidence,
            "retained_query_only": proof_evidence,
        },
        "retained_query_only_proof": {
            "comparison": query_only_comparison,
            "artifacts": retained_artifact_results,
            "shader_sources": retained_source_results,
            "shader_sources_match": retained_sources_match,
            "artifact_hash_mode": retained_artifact_hash_mode,
            "shader_source_hash_mode": retained_source_hash_mode,
            "scope": "historical extension-independence proof only; no retained transport or fixture result is reused",
        },
        "rtx_vs_rq_attributes": attributes,
        "rtx_vs_rq_transport": transport,
        "conservation": conservation,
        "bounded_views": bounded,
        "classifications": classifications,
        "deterministic_replay": {
            "cpu_contract": "known PCG vectors and global-sample/domain mapping are covered by focused tests",
            "gpu_process_proxy": "retained ordinary RayQuery and forced query-only processes",
            "exact": bool(query_only_comparison.get("exact_match")),
        },
        "numerical_contracts": {
            "environment_pdf": "exact solid-angle normalization, constant energy, and high-contrast variance",
            "environment_rotation": "inverse/forward Y rotation and sample/evaluation PDF parity",
            "distant_shadow": "shared traversal range independent of camera far",
            "volume_lod": volume_lod,
        },
        "m10_fixture": fixture,
        "artifacts": {
            "replacement": {name: artifact_record(args.matrix_dir, name) for name in PRIMARY_LANES},
            "retained_query_only": {
                name: artifact_record(proof_directory, name) for name in ("rq", "rq-only")
            },
        },
        "gates": gates,
        "passed": all(gates.values()),
    }


def self_test() -> None:
    m7.self_test()
    command = capture_command(Path("EvoEngineEditor.exe"), Path("atlas"), "rtx")
    assert command.count("--preview-m10-ray-transport") == 1
    assert command[command.index("--preview-camera-position") + 1] == "48.0,1.5,5.6"
    assert command[command.index("--preview-camera-look-at") + 1] == "48.0,1.15,-1.4"
    assert len(m7.LANES) == 3
    assert len(PRIMARY_LANES) == 2
    assert lf_normalized(b"a\r\nb\rc\n") == b"a\nb\nc\n"
    assert capture_lanes(argparse.Namespace(retained_query_only_dir=Path("retained"))) == PRIMARY_LANES
    try:
        capture_lanes(argparse.Namespace(retained_query_only_dir=None))
        raise AssertionError("M10 replacement mode accepted a missing retained directory")
    except ValueError:
        pass
    stats = roi_statistics([0.0, 0.0, 0.0, 2.0, 2.0, 2.0])
    assert abs(float(stats["mean_luminance"]) - 1.0) < 1.0e-6
    assert abs(float(stats["contrast"]) - 1.0) < 1.0e-6
    assert percentile([0.0, 2.0], 0.25) == 0.5
    checker = [
        math.cos(2.0 * math.pi * (0.25 * x + 0.25 * y))
        + math.cos(2.0 * math.pi * (0.25 * x - 0.25 * y))
        for y in range(12)
        for x in range(12)
    ]
    smooth = [float(x + y) for y in range(12) for x in range(12)]
    checker_response = matched_frequency_response(checker, 12, 0, 0, 0.25, 0.25, 0.05, 0.95)
    smooth_response = matched_frequency_response(smooth, 12, 0, 0, 0.25, 0.25, 0.05, 0.95)
    assert checker_response is not None and smooth_response is not None
    assert checker_response >= 0.2 and checker_response >= 2.0 * smooth_response
    expectations = json.loads(
        Path(__file__).with_name("raytracer_m10_expectations.json").read_text(encoding="utf-8")
    )
    volume_lod = volume_cone_lod_result(expectations["m10_volume_cone_lod"])
    assert volume_lod["lod_increase"] >= expectations["m10_volume_cone_lod"]["minimum_lod_increase"]
    print("M10 validator self-test passed")


def main() -> int:
    args = parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.capture or args.dry_run:
        run_captures(args)
    if args.dry_run:
        return 0
    summary = validate(args)
    output = args.out or args.matrix_dir / "validation.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
