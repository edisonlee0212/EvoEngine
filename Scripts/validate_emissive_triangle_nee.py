#!/usr/bin/env python3
"""Validate the local rendering-regression emissive-triangle NEE capture matrix."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from compare_reference_render import HdrImage, compare_hdr_images, read_hdr

BASE_SIZE = (1280, 720)
BASE_RECEIVER_ROIS = ((500, 310, 780, 390), (475, 465, 805, 495))
M0_MAE_LIMIT = 0.001003
M0_RMS_LIMIT = 0.002483
ENERGY_LUMINANCE_LIMIT = 0.03
ENERGY_CHANNEL_LIMIT = 0.05
VARIANCE_RMS_RATIO_LIMIT = 0.75


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    return parser.parse_args()


def load_matrix(root: Path) -> tuple[dict[str, HdrImage], dict[str, dict[str, object]]]:
    names = (
        "rtx-on-2048",
        "rtx-off-2048",
        "rq-on-2048",
        "rq-off-2048",
        "rtx-on-64",
        "rtx-off-64",
        "rq-on-64",
        "rq-off-64",
        "rtx-on-2048-clamp10",
        "rq-on-2048-clamp10",
        "rq-only-on-64",
    )
    images = {name: read_hdr(root / f"{name}.hdr") for name in names}
    metrics = {
        name: json.loads((root / f"{name}.json").read_text(encoding="utf-8")) for name in names
    }
    return images, metrics


def validate_capture_evidence(
    root: Path, name: str, image: HdrImage, metrics: dict[str, object]
) -> dict[str, bool]:
    technique = name.split("-", 1)[0]
    expected_mode = {"rtx": "RayTracing", "rq": "RayQuery"}[technique]
    expected_spp = 2048 if "-2048" in name else 64
    expected_nee = "-on-" in name
    expected_clamp = name.endswith("-clamp10")
    query_only = name.startswith("rq-only-")
    capabilities = metrics.get("capabilities")
    if not isinstance(capabilities, dict):
        capabilities = {}
    rendered_frames = metrics.get("rendered_frames")
    samples_per_frame = metrics.get("samples_per_frame")
    accumulated_spp = (
        rendered_frames * samples_per_frame
        if isinstance(rendered_frames, int) and isinstance(samples_per_frame, int)
        else None
    )

    def vec3_matches(value: object, expected: tuple[float, float, float]) -> bool:
        return (
            isinstance(value, list)
            and len(value) == 3
            and all(
                isinstance(component, (int, float))
                and math.isclose(float(component), target, rel_tol=0.0, abs_tol=1.0e-6)
                for component, target in zip(value, expected)
            )
        )

    output_path = metrics.get("output_path")
    checks = {
        "capture_schema": metrics.get("type") == "evoengine_ray_capture",
        "demo_profile": metrics.get("demo_profile") == "rendering-regression",
        "linear_hdr": metrics.get("output_format") == "radiance_hdr_linear",
        "output_path": isinstance(output_path, str)
        and Path(output_path).resolve() == (root / f"{name}.hdr").resolve(),
        "camera_position_override": vec3_matches(
            metrics.get("camera_position_override"), (0.0, 4.8, 5.6)
        ),
        "camera_look_at_override": vec3_matches(
            metrics.get("camera_look_at_override"), (0.0, 4.4, -2.4)
        ),
        "render_mode": metrics.get("render_mode") == expected_mode,
        "width": metrics.get("width") == image.width,
        "height": metrics.get("height") == image.height,
        "effective_spp": metrics.get("effective_spp") == expected_spp,
        "accumulated_spp": accumulated_spp == expected_spp,
        "emissive_nee_toggle": metrics.get("emissive_triangle_nee_enabled") is expected_nee,
        "firefly_clamp_toggle": metrics.get("firefly_clamp_enabled") is expected_clamp,
        "auto_spp_disabled": metrics.get("auto_spp_enabled") is False,
        "deterministic": metrics.get("deterministic") is True,
        "acceleration_structure": capabilities.get("acceleration_structure") is True,
        "ray_query": capabilities.get("ray_query") is True,
        "ray_tracing_pipeline": capabilities.get("ray_tracing_pipeline") is (not query_only),
        "ser_requested_disabled": metrics.get("ser_mode_requested") == "Disabled",
        "ser_disabled": metrics.get("ser_enabled") is False,
    }
    if expected_clamp:
        checks["firefly_clamp_threshold"] = math.isclose(
            float(metrics.get("firefly_clamp_threshold", math.nan)), 10.0
        )
    if query_only:
        checks["query_only_ser_unsupported"] = metrics.get("ser_supported") is False
        checks["query_only_ser_capability"] = capabilities.get("shader_execution_reordering") is False
    return checks


def receiver_rois(image: HdrImage) -> list[tuple[int, int, int, int]]:
    scale_x = image.width / BASE_SIZE[0]
    scale_y = image.height / BASE_SIZE[1]
    return [
        (
            round(x0 * scale_x),
            round(y0 * scale_y),
            round(x1 * scale_x),
            round(y1 * scale_y),
        )
        for x0, y0, x1, y1 in BASE_RECEIVER_ROIS
    ]


def roi_values(image: HdrImage, rois: list[tuple[int, int, int, int]]) -> list[float]:
    values: list[float] = []
    for x0, y0, x1, y1 in rois:
        for y in range(y0, y1):
            for x in range(x0, x1):
                index = (y * image.width + x) * 3
                values.extend(image.rgb[index : index + 3])
    return values


def mean_rgb(values: list[float]) -> list[float]:
    pixel_count = len(values) // 3
    return [sum(values[channel::3]) / pixel_count for channel in range(3)]


def rms(reference: list[float], candidate: list[float]) -> float:
    return math.sqrt(sum((lhs - rhs) ** 2 for lhs, rhs in zip(reference, candidate)) / len(reference))


def symmetric_delta(lhs: float, rhs: float) -> float:
    return 2.0 * abs(lhs - rhs) / max(abs(lhs) + abs(rhs), 1.0e-9)


def main() -> int:
    args = parse_args()
    images, metrics = load_matrix(args.matrix_dir)
    sizes = {(image.width, image.height) for image in images.values()}
    if len(sizes) != 1:
        raise ValueError(f"Capture matrix dimensions differ: {sorted(sizes)}")
    rois = receiver_rois(images["rtx-on-2048"])

    parity = compare_hdr_images(images["rtx-on-2048-clamp10"], images["rq-on-2048-clamp10"])
    query_only = compare_hdr_images(images["rq-on-64"], images["rq-only-on-64"])
    gates: dict[str, bool] = {
        "rtx_rq_mae": float(parity["mean_abs_error"]) <= M0_MAE_LIMIT,
        "rtx_rq_rms": float(parity["rms_error"]) <= M0_RMS_LIMIT,
        "ray_query_independent_exact": bool(query_only["exact_match"]),
    }
    evidence = {
        name: validate_capture_evidence(args.matrix_dir, name, images[name], metrics[name]) for name in images
    }
    gates.update({f"evidence_{name}": all(checks.values()) for name, checks in evidence.items()})
    techniques: dict[str, object] = {}
    for technique in ("rtx", "rq"):
        high_on = roi_values(images[f"{technique}-on-2048"], rois)
        high_off = roi_values(images[f"{technique}-off-2048"], rois)
        low_on = roi_values(images[f"{technique}-on-64"], rois)
        low_off = roi_values(images[f"{technique}-off-64"], rois)
        high_on_mean = mean_rgb(high_on)
        high_off_mean = mean_rgb(high_off)
        luminance_weights = (0.2126, 0.7152, 0.0722)
        high_on_luminance = sum(value * weight for value, weight in zip(high_on_mean, luminance_weights))
        high_off_luminance = sum(value * weight for value, weight in zip(high_off_mean, luminance_weights))
        channel_deltas = [symmetric_delta(lhs, rhs) for lhs, rhs in zip(high_on_mean, high_off_mean)]
        luminance_delta = symmetric_delta(high_on_luminance, high_off_luminance)
        low_on_rms = rms(high_on, low_on)
        low_off_rms = rms(high_on, low_off)
        variance_ratio = low_on_rms / low_off_rms if low_off_rms > 0.0 else math.inf
        gates[f"{technique}_energy_luminance"] = luminance_delta <= ENERGY_LUMINANCE_LIMIT
        gates[f"{technique}_energy_channels"] = max(channel_deltas) <= ENERGY_CHANNEL_LIMIT
        gates[f"{technique}_variance"] = variance_ratio <= VARIANCE_RMS_RATIO_LIMIT
        techniques[technique] = {
            "receiver_mean_rgb_nee": high_on_mean,
            "receiver_mean_rgb_hit_only": high_off_mean,
            "receiver_luminance_symmetric_delta": luminance_delta,
            "receiver_channel_symmetric_delta": channel_deltas,
            "low_spp_nee_rms": low_on_rms,
            "low_spp_hit_only_rms": low_off_rms,
            "low_spp_rms_ratio": variance_ratio,
        }

    summary = {
        "schema": 1,
        "matrix_dir": str(args.matrix_dir.resolve()),
        "size": list(next(iter(sizes))),
        "receiver_rois": [list(roi) for roi in rois],
        "limits": {
            "rtx_rq_mae": M0_MAE_LIMIT,
            "rtx_rq_rms": M0_RMS_LIMIT,
            "energy_luminance_symmetric_delta": ENERGY_LUMINANCE_LIMIT,
            "energy_channel_symmetric_delta": ENERGY_CHANNEL_LIMIT,
            "low_spp_rms_ratio": VARIANCE_RMS_RATIO_LIMIT,
        },
        "rtx_vs_ray_query": parity,
        "ray_query_independence": query_only,
        "capture_evidence": evidence,
        "techniques": techniques,
        "gates": gates,
        "passed": all(gates.values()),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
