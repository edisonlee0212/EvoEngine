#!/usr/bin/env python3
"""Capture and validate the shared RTX/RayQuery M7 diagnostic atlas."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
from array import array
from pathlib import Path

from compare_reference_render import HdrImage, compare_hdr_images, read_hdr, write_png_rgba8

SIZE = (1280, 720)
FRAMES = 16
SAMPLES_PER_FRAME = 4
CAMERA_POSITION = (0.0, 1.5, 5.6)
CAMERA_LOOK_AT = (0.0, 1.15, -1.4)
VIEW_NAMES = (
    "Beauty",
    "Material ID",
    "Base Color",
    "Geometric Normal",
    "Shading Normal",
    "Roughness",
    "Metallic",
    "Specular F0",
    "Alpha/Coverage",
    "Transmission",
    "Iridescence",
    "Emission",
    "Direct Punctual",
    "Direct Environment",
    "Direct Emissive",
    "Indirect Radiance",
    "Path Depth",
    "BSDF PDF",
    "Light PDF",
    "Emissive PDF",
)
LANES = {
    "rtx": ("raytracing", False),
    "rq": ("rayquery", False),
    "rq-only": ("rayquery", True),
}
ATTRIBUTE_VIEWS = range(1, 11)
BOUNDED_VIEWS = (*ATTRIBUTE_VIEWS, 16, 17, 18, 19)
TRANSPORT_VIEWS = (0, 11, 12, 13, 14, 15)
RTX_RQ_ATTRIBUTE_MAE_LIMIT = 0.002
RTX_RQ_ATTRIBUTE_RMS_LIMIT = 0.015
RTX_RQ_TRANSPORT_LIMITS = {
    "Beauty": {
        "mean_abs_error": 0.0045,
        "p999_abs_error": 0.2,
        "max_abs_error": 128.0,
        "tail_abs_error": 1.0,
        "tail_count": 20,
        "classification": "bounded_stochastic_noise",
    },
    "Emission": {
        "mean_abs_error": 0.00015,
        "p999_abs_error": 0.05,
        "max_abs_error": 1.0,
        "tail_abs_error": 0.5,
        "tail_count": 5,
        "classification": "bounded_stochastic_noise",
    },
    "Direct Punctual": {
        "mean_abs_error": 0.0035,
        "p999_abs_error": 0.08,
        "max_abs_error": 128.0,
        "tail_abs_error": 1.0,
        "tail_count": 12,
        "classification": "bounded_stochastic_noise",
    },
    "Direct Environment": {
        "mean_abs_error": 0.00003,
        "p999_abs_error": 0.01,
        "max_abs_error": 0.05,
        "classification": "conformant",
    },
    "Direct Emissive": {
        "mean_abs_error": 0.000002,
        "p999_abs_error": 0.00025,
        "max_abs_error": 0.012,
        "classification": "conformant",
    },
    "Indirect Radiance": {
        "mean_abs_error": 0.0008,
        "p999_abs_error": 0.12,
        "max_abs_error": 10.0,
        "tail_abs_error": 1.0,
        "tail_count": 8,
        "classification": "bounded_stochastic_noise",
    },
}
CONSERVATION_RELATIVE_L2_LIMIT = 0.035
ALLOWED_CLASSIFICATIONS = {
    "evoengine_defect",
    "khronos_reference_deviation",
    "bounded_stochastic_noise",
    "conformant",
}
FATAL_LOG_PATTERNS = (
    re.compile(r"\bVUID-"),
    re.compile(r"\bvalidation error\b", re.IGNORECASE),
    re.compile(r"\bVK_ERROR_DEVICE_LOST\b", re.IGNORECASE),
    re.compile(r"\bdevice[- ]lost\b", re.IGNORECASE),
    re.compile(r"\bunhandled exception\b", re.IGNORECASE),
    re.compile(r"\baccess violation\b", re.IGNORECASE),
    re.compile(r"\bEVOENGINE_(?:APP_TEST_RESULT failed|FATAL|ERROR)\b", re.IGNORECASE),
    re.compile(r"\b(?:shader|pipeline).{0,48}\bfailed\b", re.IGNORECASE),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--editor", type=Path)
    parser.add_argument("--matrix-dir", type=Path, default=Path("out/m7-validation/atlas"))
    parser.add_argument("--out", type=Path)
    parser.add_argument(
        "--expectations",
        type=Path,
        default=Path(__file__).with_name("raytracer_m7_expectations.json"),
    )
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--timeout-seconds", type=int, default=900)
    return parser.parse_args()


def atlas_rect(width: int, height: int, view: int) -> tuple[int, int, int, int]:
    tile_x, tile_y = view % 5, 3 - view // 5
    cell_x0, cell_x1 = tile_x * width // 5, (tile_x + 1) * width // 5
    cell_y0, cell_y1 = tile_y * height // 4, (tile_y + 1) * height // 4
    cell_width, cell_height = cell_x1 - cell_x0, cell_y1 - cell_y0
    viewport_width, viewport_height = cell_width, cell_height
    if cell_width * 9 <= cell_height * 16:
        viewport_height = max(cell_width * 9 // 16, 1)
    else:
        viewport_width = max(cell_height * 16 // 9, 1)
    return (
        cell_x0 + (cell_width - viewport_width) // 2,
        cell_y0 + (cell_height - viewport_height) // 2,
        viewport_width,
        viewport_height,
    )


def view_values(image: HdrImage, view: int, roi: list[int] | None = None) -> list[float]:
    x0, y0, width, height = atlas_rect(image.width, image.height, view)
    if roi:
        sx, sy = width / 256.0, height / 144.0
        rx0, ry0, rx1, ry1 = roi
        x0 += round(rx0 * sx)
        y0 += round(ry0 * sy)
        width = max(round((rx1 - rx0) * sx), 1)
        height = max(round((ry1 - ry0) * sy), 1)
    values: list[float] = []
    for y in range(y0, y0 + height):
        start = (y * image.width + x0) * 3
        values.extend(image.rgb[start : start + width * 3])
    return values


def comparison(lhs: list[float], rhs: list[float]) -> dict[str, float | int]:
    if len(lhs) != len(rhs):
        raise ValueError("Diagnostic view payload lengths differ")
    differences = [abs(a - b) for a, b in zip(lhs, rhs)]
    squared_error = sum(value * value for value in differences)
    absolute_error = sum(differences)
    reference_squared = sum(value * value for value in lhs)
    count = max(len(lhs), 1)
    sorted_differences = sorted(differences)
    return {
        "value_count": len(lhs),
        "max_abs_error": max(differences, default=0.0),
        "mean_abs_error": absolute_error / count,
        "rms_error": math.sqrt(squared_error / count),
        "relative_l2": math.sqrt(squared_error / max(reference_squared, 1.0e-12)),
        "p999_abs_error": sorted_differences[round(0.999 * (len(sorted_differences) - 1))]
        if sorted_differences
        else 0.0,
    }


def transport_comparison(lhs: list[float], rhs: list[float], limits: dict[str, float | int | str]) -> dict[str, object]:
    result: dict[str, object] = comparison(lhs, rhs)
    if "tail_abs_error" in limits:
        threshold = float(limits["tail_abs_error"])
        result["tail_abs_error"] = threshold
        result["tail_count"] = sum(abs(a - b) > threshold for a, b in zip(lhs, rhs))
    result["classification"] = str(limits["classification"])
    return result


def transport_passes(result: dict[str, object], limits: dict[str, float | int | str]) -> bool:
    passed = (
        float(result["mean_abs_error"]) <= float(limits["mean_abs_error"])
        and float(result["p999_abs_error"]) <= float(limits["p999_abs_error"])
        and float(result["max_abs_error"]) <= float(limits["max_abs_error"])
    )
    if "tail_count" in limits:
        passed = passed and int(result["tail_count"]) <= int(limits["tail_count"])
    return passed


def mean_rgb(values: list[float]) -> list[float]:
    pixels = max(len(values) // 3, 1)
    return [sum(values[channel::3]) / pixels for channel in range(3)]


def conservation(image: HdrImage) -> dict[str, float | int]:
    views = {view: view_values(image, view) for view in TRANSPORT_VIEWS}
    summed = [
        sum(values[index] for view, values in views.items() if view != 0)
        for index in range(len(views[0]))
    ]
    return comparison(views[0], summed)


def metrics_checks(name: str, metrics: dict[str, object]) -> dict[str, bool]:
    mode, query_only = LANES[name]
    capabilities = metrics.get("capabilities")
    variant = metrics.get("ray_shader_variant")
    if not isinstance(capabilities, dict):
        capabilities = {}
    if not isinstance(variant, dict):
        variant = {}
    position = metrics.get("camera_position_override")
    look_at = metrics.get("camera_look_at_override")
    camera_override = (
        isinstance(position, list)
        and isinstance(look_at, list)
        and len(position) == 3
        and len(look_at) == 3
        and all(
            math.isclose(float(value), expected, abs_tol=1.0e-6)
            for value, expected in zip(position, CAMERA_POSITION)
        )
        and all(
            math.isclose(float(value), expected, abs_tol=1.0e-6)
            for value, expected in zip(look_at, CAMERA_LOOK_AT)
        )
    )
    return {
        "capture_schema": metrics.get("type") == "evoengine_ray_capture",
        "profile": metrics.get("demo_profile") == "rendering-regression",
        "linear_hdr": metrics.get("output_format") == "radiance_hdr_linear",
        "dimensions": (metrics.get("width"), metrics.get("height")) == SIZE,
        "render_mode": metrics.get("render_mode") == ("RayTracing" if mode == "raytracing" else "RayQuery"),
        "debug_view": metrics.get("ray_debug_view") == "Validation Atlas",
        "camera_override": camera_override,
        "effective_spp": metrics.get("effective_spp") == FRAMES * SAMPLES_PER_FRAME,
        "deterministic": metrics.get("deterministic") is True,
        "manual_spp": metrics.get("auto_spp_enabled") is False,
        "firefly_clamp_disabled": metrics.get("firefly_clamp_enabled") is False,
        "emissive_nee_enabled": metrics.get("emissive_triangle_nee_enabled") is True,
        "ser_requested_disabled": metrics.get("ser_mode_requested") == "Disabled",
        "ser_disabled": metrics.get("ser_enabled") is False,
        "acceleration_structure": capabilities.get("acceleration_structure") is True,
        "ray_query": capabilities.get("ray_query") is True,
        "ray_pipeline_capability": capabilities.get("ray_tracing_pipeline") is (not query_only),
        "variant_ready": variant.get("ready") is True and variant.get("pending") is False,
        "variant_exact": variant.get("requested_key") == variant.get("active_key")
        and variant.get("requested_mask") == variant.get("active_mask"),
    }


def capture_command(editor: Path, directory: Path, name: str) -> list[str]:
    mode, query_only = LANES[name]
    command = [
        str(editor.resolve()),
        "--demo",
        "rendering-regression",
        "--editor",
        "--capture-demo-preview",
        str((directory / f"{name}.hdr").resolve()),
        "--preview-metrics-json",
        str((directory / f"{name}.json").resolve()),
        "--preview-render-mode",
        mode,
        "--preview-ray-debug",
        "validation-atlas",
        "--preview-ray-shader-variant",
        "auto",
        "--preview-camera-position",
        ",".join(str(value) for value in CAMERA_POSITION),
        "--preview-camera-look-at",
        ",".join(str(value) for value in CAMERA_LOOK_AT),
        "--preview-warmup-frames",
        str(FRAMES),
        "--preview-sample-size",
        str(SAMPLES_PER_FRAME),
        "--preview-auto-spp",
        "disabled",
        "--preview-firefly-clamp",
        "disabled",
        "--preview-emissive-nee",
        "enabled",
        "--preview-ser",
        "disabled",
        "--preview-width",
        str(SIZE[0]),
        "--preview-height",
        str(SIZE[1]),
        "--preview-deterministic",
    ]
    if query_only:
        command.append("--disable-ray-tracing-pipeline")
    return command


def run_captures(args: argparse.Namespace) -> None:
    if not args.editor or not args.editor.is_file():
        raise ValueError("--capture and --dry-run require an existing --editor executable")
    args.matrix_dir.mkdir(parents=True, exist_ok=True)
    for name in LANES:
        command = capture_command(args.editor, args.matrix_dir, name)
        print(json.dumps(command))
        if args.dry_run:
            continue
        environment = os.environ.copy()
        environment["EVOENGINE_SHADER_CACHE_DIR"] = str((args.matrix_dir / "shader-cache" / name).resolve())
        environment["EVOENGINE_IMGUI_INI_PATH"] = str((args.matrix_dir / f"{name}.imgui.ini").resolve())
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
        fatal_matches = [pattern.pattern for pattern in FATAL_LOG_PATTERNS if pattern.search(log_text)]
        if result.returncode or fatal_matches:
            tail = "\n".join(log_text.splitlines()[-40:])
            raise RuntimeError(
                f"{name} capture failed with exit code {result.returncode}; fatal patterns={fatal_matches}:\n{tail}"
            )


def write_display_png(image: HdrImage, path: Path) -> None:
    rgba = bytearray()
    for index in range(0, len(image.rgb), 3):
        for channel in image.rgb[index : index + 3]:
            mapped = max(channel, 0.0) / (1.0 + max(channel, 0.0))
            rgba.append(round(pow(mapped, 1.0 / 2.2) * 255.0))
        rgba.append(255)
    write_png_rgba8(path, image.width, image.height, bytes(rgba))


def roi_results(images: dict[str, HdrImage], expectations: dict[str, object]) -> tuple[dict[str, object], dict[str, bool]]:
    results: dict[str, object] = {}
    gates: dict[str, bool] = {}
    rois = expectations.get("rois", [])
    for roi in rois if isinstance(rois, list) else []:
        if not isinstance(roi, dict):
            continue
        name = str(roi["name"])
        view = VIEW_NAMES.index(str(roi["view"]))
        rect = [int(value) for value in roi["rect"]]
        lane_means = {lane: mean_rgb(view_values(images[lane], view, rect)) for lane in ("rtx", "rq")}
        results[name] = lane_means
        minimum = roi.get("min_rgb", [0.0, 0.0, 0.0])
        maximum = roi.get("max_rgb", [float("inf")] * 3)
        for lane, values in lane_means.items():
            gates[f"roi_{name}_{lane}_range"] = all(
                float(minimum[index]) <= value <= float(maximum[index]) for index, value in enumerate(values)
            )
        gates[f"roi_{name}_cross_technique"] = max(
            abs(a - b) for a, b in zip(lane_means["rtx"], lane_means["rq"])
        ) <= float(roi.get("cross_technique_max", 0.02))
    relations = expectations.get("relations", [])
    for relation in relations if isinstance(relations, list) else []:
        if not isinstance(relation, dict):
            continue
        name = str(relation["name"])
        lhs, rhs = str(relation["lhs"]), str(relation["rhs"])
        minimum = float(relation.get("rgb_distance_min", 0.0))
        maximum = float(relation.get("rgb_distance_max", float("inf")))
        for lane in ("rtx", "rq"):
            distance = math.sqrt(
                sum((a - b) ** 2 for a, b in zip(results[lhs][lane], results[rhs][lane]))
            )
            gates[f"relation_{name}_{lane}"] = minimum <= distance <= maximum
    return results, gates


def validate(args: argparse.Namespace) -> dict[str, object]:
    images = {name: read_hdr(args.matrix_dir / f"{name}.hdr") for name in LANES}
    metrics = {
        name: json.loads((args.matrix_dir / f"{name}.json").read_text(encoding="utf-8")) for name in LANES
    }
    expectations = json.loads(args.expectations.read_text(encoding="utf-8"))
    evidence = {name: metrics_checks(name, metrics[name]) for name in LANES}
    rq_query_only = compare_hdr_images(images["rq"], images["rq-only"])
    attributes = {
        VIEW_NAMES[view]: comparison(view_values(images["rtx"], view), view_values(images["rq"], view))
        for view in ATTRIBUTE_VIEWS
    }
    transport = {
        VIEW_NAMES[view]: transport_comparison(
            view_values(images["rtx"], view),
            view_values(images["rq"], view),
            RTX_RQ_TRANSPORT_LIMITS[VIEW_NAMES[view]],
        )
        for view in TRANSPORT_VIEWS
    }
    conservation_results = {name: conservation(image) for name, image in images.items()}
    bounded = {
        name: all(
            math.isfinite(value) and -1.0e-6 <= value <= 1.0001
            for view in BOUNDED_VIEWS
            for value in view_values(image, view)
        )
        for name, image in images.items()
    }
    roi_summary, roi_gates = roi_results(images, expectations)
    classifications = expectations.get("classifications", [])
    classifications_valid = isinstance(classifications, list) and bool(classifications) and all(
        isinstance(item, dict) and item.get("category") in ALLOWED_CLASSIFICATIONS and item.get("subject")
        for item in classifications
    )
    gates = {
        **{f"evidence_{name}": all(checks.values()) for name, checks in evidence.items()},
        "ray_query_independent_exact": bool(rq_query_only.get("exact_match")),
        **{
            f"attribute_{name}": float(result["mean_abs_error"]) <= RTX_RQ_ATTRIBUTE_MAE_LIMIT
            and float(result["rms_error"]) <= RTX_RQ_ATTRIBUTE_RMS_LIMIT
            for name, result in attributes.items()
        },
        **{
            f"transport_{name}": transport_passes(result, RTX_RQ_TRANSPORT_LIMITS[name])
            for name, result in transport.items()
        },
        **{
            f"conservation_{name}": float(result["relative_l2"]) <= CONSERVATION_RELATIVE_L2_LIMIT
            for name, result in conservation_results.items()
        },
        **{f"bounded_{name}": passed for name, passed in bounded.items()},
        **roi_gates,
        "classifications": classifications_valid,
    }
    for name, image in images.items():
        write_display_png(image, args.matrix_dir / f"{name}.png")
    return {
        "schema": 1,
        "matrix_dir": str(args.matrix_dir.resolve()),
        "atlas": {
            "grid": [5, 4],
            "size": list(SIZE),
            "storage_origin": "top-left",
            "view_origin": "bottom-left",
            "rows_top_to_bottom": [list(VIEW_NAMES[row * 5 : (row + 1) * 5]) for row in range(3, -1, -1)],
        },
        "limits": {
            "attribute_mae": RTX_RQ_ATTRIBUTE_MAE_LIMIT,
            "attribute_rms": RTX_RQ_ATTRIBUTE_RMS_LIMIT,
            "transport": RTX_RQ_TRANSPORT_LIMITS,
            "conservation_relative_l2": CONSERVATION_RELATIVE_L2_LIMIT,
        },
        "capture_evidence": evidence,
        "rq_vs_query_only": rq_query_only,
        "rtx_vs_rq_attributes": attributes,
        "rtx_vs_rq_transport": transport,
        "conservation": conservation_results,
        "bounded_views": bounded,
        "rois": roi_summary,
        "classifications": classifications,
        "gates": gates,
        "passed": all(gates.values()),
    }


def self_test() -> None:
    width, height = 80, 36
    rgb = array("f", [0.0] * (width * height * 3))
    image = HdrImage(Path("synthetic.hdr"), width, height, rgb, "synthetic")
    for view in range(20):
        x0, y0, tile_width, tile_height = atlas_rect(width, height, view)
        assert tile_width == 16 and tile_height == 9
        for y in range(y0, y0 + tile_height):
            for x in range(x0, x0 + tile_width):
                base = (y * width + x) * 3
                rgb[base : base + 3] = array("f", [float(view)] * 3)
        assert all(value == float(view) for value in view_values(image, view))
    assert atlas_rect(width, height, 0)[1] == 27
    assert atlas_rect(width, height, 15)[1] == 0
    assert comparison([1.0, 2.0], [1.0, 2.0])["relative_l2"] == 0.0
    print("ray debug validator self-test passed")


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
