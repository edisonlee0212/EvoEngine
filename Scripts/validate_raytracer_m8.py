#!/usr/bin/env python3
"""Capture and validate M8 glTF material semantics across raster, RTX, and RayQuery."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
from pathlib import Path

import validate_ray_debug_views as m7
from compare_reference_render import PngImage, read_png


COMPACT_RAY_LANES = ("rtx", "rq-only")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--editor", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("out/m8-validation"))
    parser.add_argument(
        "--expectations",
        type=Path,
        default=Path(__file__).with_name("raytracer_m8_expectations.json"),
    )
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--timeout-seconds", type=int, default=900)
    parser.add_argument(
        "--m7-validation",
        type=Path,
        default=Path("out/m7-validation/atlas/validation.json"),
        help="Accepted M7 report that proves ordinary RayQuery/query-only identity.",
    )
    return parser.parse_args()


def raster_command(editor: Path, output_dir: Path) -> list[str]:
    return [
        str(editor.resolve()),
        "--demo",
        "rendering-regression",
        "--editor",
        "--capture-demo-preview",
        str((output_dir / "raster.png").resolve()),
        "--preview-metrics-json",
        str((output_dir / "raster.json").resolve()),
        "--preview-render-mode",
        "rasterization",
        "--preview-camera-position",
        ",".join(str(value) for value in m7.CAMERA_POSITION),
        "--preview-camera-look-at",
        ",".join(str(value) for value in m7.CAMERA_LOOK_AT),
        "--preview-warmup-frames",
        "8",
        "--preview-width",
        str(m7.SIZE[0]),
        "--preview-height",
        str(m7.SIZE[1]),
        "--preview-deterministic",
    ]


def run_command(command: list[str], log_path: Path, environment: dict[str, str], timeout_seconds: int) -> None:
    with log_path.open("w", encoding="utf-8") as log:
        result = subprocess.run(
            command,
            cwd=Path(__file__).resolve().parents[1],
            env=environment,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout_seconds,
            check=False,
        )
    log_text = log_path.read_text(encoding="utf-8", errors="replace")
    fatal_matches = [pattern.pattern for pattern in m7.FATAL_LOG_PATTERNS if pattern.search(log_text)]
    if result.returncode or fatal_matches:
        tail = "\n".join(log_text.splitlines()[-40:])
        raise RuntimeError(
            f"capture failed with exit code {result.returncode}; fatal patterns={fatal_matches}:\n{tail}"
        )


def capture(args: argparse.Namespace) -> None:
    if not args.editor or not args.editor.is_file():
        raise ValueError("--capture and --dry-run require an existing --editor executable")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    command = raster_command(args.editor, args.output_dir)
    print(json.dumps(command))
    if not args.dry_run:
        environment = os.environ.copy()
        environment["EVOENGINE_SHADER_CACHE_DIR"] = str((args.output_dir / "shader-cache" / "raster").resolve())
        environment["EVOENGINE_IMGUI_INI_PATH"] = str((args.output_dir / "raster.imgui.ini").resolve())
        run_command(command, args.output_dir / "raster.log", environment, args.timeout_seconds)
    atlas_dir = args.output_dir / "atlas"
    atlas_dir.mkdir(parents=True, exist_ok=True)
    for name in COMPACT_RAY_LANES:
        command = m7.capture_command(args.editor, atlas_dir, name)
        print(json.dumps(command))
        if args.dry_run:
            continue
        environment = os.environ.copy()
        environment["EVOENGINE_SHADER_CACHE_DIR"] = str((atlas_dir / "shader-cache" / name).resolve())
        environment["EVOENGINE_IMGUI_INI_PATH"] = str((atlas_dir / f"{name}.imgui.ini").resolve())
        run_command(command, atlas_dir / f"{name}.log", environment, args.timeout_seconds)


def png_roi_mean(image: PngImage, rect: list[int]) -> list[float]:
    x0, y0, x1, y1 = rect
    if not (0 <= x0 < x1 <= image.width and 0 <= y0 < y1 <= image.height):
        raise ValueError(f"PNG ROI {rect} is outside {image.width}x{image.height}")
    totals = [0.0, 0.0, 0.0]
    count = 0
    for y in range(y0, y1):
        for x in range(x0, x1):
            offset = (y * image.width + x) * 4
            for channel in range(3):
                totals[channel] += image.rgba[offset + channel] / 255.0
            count += 1
    return [value / count for value in totals]


def luminance(rgb: list[float]) -> float:
    return 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]


def relation_gates(values: dict[str, list[float]], relations: list[dict[str, object]]) -> dict[str, bool]:
    gates: dict[str, bool] = {}
    for relation in relations:
        name = str(relation["name"])
        lhs = values[str(relation["lhs"])]
        rhs = values[str(relation["rhs"])]
        distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(lhs, rhs)))
        passed = float(relation.get("rgb_distance_min", 0.0)) <= distance <= float(
            relation.get("rgb_distance_max", math.inf)
        )
        if "luminance_ratio_max" in relation:
            passed = passed and luminance(lhs) / max(luminance(rhs), 1.0e-8) <= float(
                relation["luminance_ratio_max"]
            )
        if "luminance_ratio_min" in relation:
            passed = passed and luminance(lhs) / max(luminance(rhs), 1.0e-8) >= float(
                relation["luminance_ratio_min"]
            )
        if "green_minus_red_min" in relation:
            passed = passed and lhs[1] - lhs[0] >= float(relation["green_minus_red_min"])
        if "distance_ratio_min" in relation:
            reference_lhs = values[str(relation["reference_lhs"])]
            reference_rhs = values[str(relation["reference_rhs"])]
            reference_distance = math.sqrt(
                sum((a - b) ** 2 for a, b in zip(reference_lhs, reference_rhs))
            )
            passed = passed and distance / max(reference_distance, 1.0e-8) >= float(
                relation["distance_ratio_min"]
            )
        gates[f"relation_{name}"] = passed
    return gates


def vectors_close(lhs: object, rhs: tuple[float, ...], tolerance: float = 1.0e-5) -> bool:
    return isinstance(lhs, list) and len(lhs) == len(rhs) and all(
        abs(float(actual) - expected) <= tolerance for actual, expected in zip(lhs, rhs)
    )


def validate_compact_atlas(args: argparse.Namespace, expectations: dict[str, object]) -> dict[str, object]:
    images = {name: m7.read_hdr(args.output_dir / "atlas" / f"{name}.hdr") for name in COMPACT_RAY_LANES}
    metrics = {
        name: json.loads((args.output_dir / "atlas" / f"{name}.json").read_text(encoding="utf-8"))
        for name in COMPACT_RAY_LANES
    }
    m7_expectations = json.loads(
        Path(__file__).with_name("raytracer_m7_expectations.json").read_text(encoding="utf-8")
    )
    transport_limits = dict(m7.RTX_RQ_TRANSPORT_LIMITS)
    transport_limits.update(expectations.get("transport_overrides", {}))
    evidence = {name: m7.metrics_checks(name, metrics[name]) for name in COMPACT_RAY_LANES}
    attributes = {
        m7.VIEW_NAMES[view]: m7.comparison(
            m7.view_values(images["rtx"], view), m7.view_values(images["rq-only"], view)
        )
        for view in m7.ATTRIBUTE_VIEWS
    }
    transport = {
        m7.VIEW_NAMES[view]: m7.transport_comparison(
            m7.view_values(images["rtx"], view),
            m7.view_values(images["rq-only"], view),
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
    raw_rois, raw_roi_gates = m7.roi_results(
        {"rtx": images["rtx"], "rq": images["rq-only"]}, m7_expectations
    )
    rois = {
        name: {"rtx": values["rtx"], "rq-only": values["rq"]}
        for name, values in raw_rois.items()
    }
    roi_gates = {
        (name[:-3] + "_rq-only" if name.endswith("_rq") else name.replace("_rq_", "_rq-only_")): passed
        for name, passed in raw_roi_gates.items()
    }

    inherited_expectation = dict(expectations["inherited_query_only_evidence"])
    inherited_bytes = args.m7_validation.read_bytes() if args.m7_validation.is_file() else b""
    inherited = json.loads(inherited_bytes.decode("utf-8")) if inherited_bytes else {}
    inherited_digest = hashlib.sha256(inherited_bytes).hexdigest() if inherited_bytes else ""
    inherited_copy = args.output_dir / "atlas" / "inherited-m7-validation.json"
    if inherited_bytes:
        inherited_copy.write_bytes(inherited_bytes)
    inherited_checks = {
        "path_exists": bool(inherited_bytes),
        "sha256": inherited_digest == str(inherited_expectation["sha256"]),
        "m7_passed": inherited.get("passed") is True,
        "ordinary_rq_query_only_exact": inherited.get("gates", {}).get("ray_query_independent_exact") is True,
        "same_image_hash": inherited.get("rq_vs_query_only", {}).get("reference_sha256")
        == inherited.get("rq_vs_query_only", {}).get("candidate_sha256"),
    }
    classifications = m7_expectations.get("classifications", [])
    gates = {
        **{f"evidence_{name}": all(checks.values()) for name, checks in evidence.items()},
        "inherited_rq_query_only_identity": all(inherited_checks.values()),
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
        **roi_gates,
        "classifications": isinstance(classifications, list)
        and bool(classifications)
        and all(
            isinstance(item, dict)
            and item.get("category") in m7.ALLOWED_CLASSIFICATIONS
            and item.get("subject")
            for item in classifications
        ),
    }
    atlas_dir = args.output_dir / "atlas"
    for name, image in images.items():
        m7.write_display_png(image, atlas_dir / f"{name}.png")
    return {
        "schema": 1,
        "matrix_dir": str(atlas_dir.resolve()),
        "fresh_lanes": list(COMPACT_RAY_LANES),
        "ordinary_ray_query": "Inherited from the accepted M7 exact RayQuery/query-only comparison.",
        "transport_limits": transport_limits,
        "capture_evidence": evidence,
        "inherited_query_only_evidence": {
            "path": str(args.m7_validation.resolve()),
            "preserved_copy": str(inherited_copy.resolve()),
            "expected": inherited_expectation,
            "actual_sha256": inherited_digest,
            "checks": inherited_checks,
        },
        "rtx_vs_query_only_attributes": attributes,
        "rtx_vs_query_only_transport": transport,
        "conservation": conservation,
        "bounded_views": bounded,
        "rois": rois,
        "classifications": classifications,
        "gates": gates,
        "passed": all(gates.values()),
    }


def validate(args: argparse.Namespace) -> dict[str, object]:
    expectations = json.loads(args.expectations.read_text(encoding="utf-8"))
    raster = read_png(args.output_dir / "raster.png")
    raster_metrics = json.loads((args.output_dir / "raster.json").read_text(encoding="utf-8"))
    atlas_dir = args.output_dir / "atlas"
    atlas = validate_compact_atlas(args, expectations)
    (atlas_dir / "validation.json").write_text(json.dumps(atlas, indent=2) + "\n", encoding="utf-8")

    raster_values = {
        str(roi["name"]): png_roi_mean(raster, list(roi["rect"]))
        for roi in expectations.get("raster_rois", [])
    }
    raster_gates = relation_gates(raster_values, expectations.get("raster_relations", []))
    raster_rgb_range = max(max(raster.rgba[channel::4]) for channel in range(3)) - min(
        min(raster.rgba[channel::4]) for channel in range(3)
    )
    raster_gates.update(
        {
            "dimensions": (raster.width, raster.height) == m7.SIZE,
            "nonblank": raster_rgb_range >= 32,
            "metrics_schema": raster_metrics.get("schema") == 2
            and raster_metrics.get("type") == "evoengine_ray_capture"
            and raster_metrics.get("renderer") == "EvoEngine",
            "metrics_profile": raster_metrics.get("demo_profile") == "rendering-regression",
            "metrics_render_mode": raster_metrics.get("render_mode") == "Rasterization",
            "metrics_output_format": raster_metrics.get("output_format") == "png_display",
            "metrics_dimensions": [raster_metrics.get("width"), raster_metrics.get("height")] == list(m7.SIZE),
            "metrics_frames": raster_metrics.get("requested_frames") == 8
            and raster_metrics.get("rendered_frames") == 8
            and raster_metrics.get("timing_warmup_frames") == 0,
            "metrics_deterministic": raster_metrics.get("deterministic") is True,
            "metrics_camera_position": vectors_close(
                raster_metrics.get("camera_position_override"), m7.CAMERA_POSITION
            ),
            "metrics_camera_look_at": vectors_close(
                raster_metrics.get("camera_look_at_override"), m7.CAMERA_LOOK_AT
            ),
        }
    )

    ray_images = {name: m7.read_hdr(atlas_dir / f"{name}.hdr") for name in COMPACT_RAY_LANES}
    view_indices = {name: index for index, name in enumerate(m7.VIEW_NAMES)}
    ray_values: dict[str, dict[str, list[float]]] = {}
    ray_gates: dict[str, bool] = {}
    for lane, image in ray_images.items():
        lane_values: dict[str, list[float]] = {}
        for roi in expectations.get("ray_rois", []):
            name = str(roi["name"])
            values = m7.view_values(image, view_indices[str(roi["view"])], list(roi["rect"]))
            mean = m7.mean_rgb(values)
            lane_values[name] = mean
            minimum = list(roi.get("min_rgb", [-math.inf] * 3))
            maximum = list(roi.get("max_rgb", [math.inf] * 3))
            ray_gates[f"roi_{name}_{lane}"] = all(
                float(minimum[channel]) <= mean[channel] <= float(maximum[channel]) for channel in range(3)
            )
        ray_values[lane] = lane_values
        for name, passed in relation_gates(lane_values, expectations.get("ray_relations", [])).items():
            ray_gates[f"{name}_{lane}"] = passed

    gates = {
        "m7_compact_regression": bool(atlas["passed"]),
        **{f"raster_{name}": passed for name, passed in raster_gates.items()},
        **ray_gates,
    }
    supersedes = list(expectations.get("supersedes", []))
    classifications = list(expectations.get("classifications", []))
    gates["m7_material_defect_classification_resolved"] = any(
        isinstance(item, dict)
        and item.get("source") == "raytracer_m7_expectations.json"
        and item.get("subject") == "native specular-glossiness, fractional specular F90, and coated emission"
        and item.get("resolved_in") == "M8"
        for item in supersedes
    )
    gates["m8_material_classifications_current"] = all(
        isinstance(item, dict)
        and item.get("category") in m7.ALLOWED_CLASSIFICATIONS
        and item.get("category") != "evoengine_defect"
        and item.get("subject")
        and item.get("resolved_in") == "M8"
        for item in classifications
    ) and bool(classifications)
    return {
        "schema": 1,
        "reference_revision": expectations["reference_revision"],
        "focused_capture_count": 3,
        "planned_delivery_capture_count": 4,
        "raster": {
            "path": str((args.output_dir / "raster.png").resolve()),
            "sha256": raster.digest,
            "size": [raster.width, raster.height],
            "rois": raster_values,
            "metrics": raster_metrics,
        },
        "ray_atlas": atlas,
        "ray_rois": ray_values,
        "supersedes": supersedes,
        "classifications": classifications,
        "gates": gates,
        "passed": all(gates.values()),
    }


def self_test() -> None:
    m7.self_test()
    values = {
        "green": [0.1, 0.8, 0.2],
        "red": [0.8, 0.1, 0.2],
        "near_green": [0.1, 0.7, 0.2],
        "near_red": [0.7, 0.1, 0.2],
    }
    gates = relation_gates(
        values,
        [
            {"name": "different", "lhs": "green", "rhs": "red", "rgb_distance_min": 0.5},
            {"name": "green", "lhs": "green", "rhs": "red", "green_minus_red_min": 0.5},
            {
                "name": "bounded_luminance_ratio",
                "lhs": "near_green",
                "rhs": "green",
                "luminance_ratio_min": 0.8,
                "luminance_ratio_max": 0.95,
            },
            {
                "name": "ratio",
                "lhs": "green",
                "rhs": "red",
                "reference_lhs": "near_green",
                "reference_rhs": "near_red",
                "distance_ratio_min": 1.1,
            },
        ],
    )
    if not all(gates.values()):
        raise AssertionError(gates)
    failed_ratio = relation_gates(
        values,
        [
            {
                "name": "ratio",
                "lhs": "green",
                "rhs": "red",
                "reference_lhs": "near_green",
                "reference_rhs": "near_red",
                "distance_ratio_min": 2.0,
            }
        ],
    )
    if failed_ratio["relation_ratio"]:
        raise AssertionError("distance-ratio rejection self-test unexpectedly passed")
    black_values = {**values, "black": [0.0, 0.0, 0.0]}
    if relation_gates(
        black_values,
        [{"name": "nonzero", "lhs": "black", "rhs": "green", "luminance_ratio_min": 0.5}],
    )["relation_nonzero"]:
        raise AssertionError("black lower-bound rejection self-test unexpectedly passed")


def main() -> int:
    args = parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.capture or args.dry_run:
        capture(args)
    if args.dry_run:
        return 0
    summary = validate(args)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / "validation.json"
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
