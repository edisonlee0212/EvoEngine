#!/usr/bin/env python3
"""Validate the local camera-ray scene-feature shader-variant capture matrix."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from compare_reference_render import HdrImage, compare_hdr_images, read_hdr

EXPECTED_SIZE = (640, 360)
EXPECTED_SPP = 512
ALL_FEATURES = (1 << 15) - 1
VARIANT_MAX_ABS_ERROR = 1.0e-2
VARIANT_MEAN_ABS_ERROR = 1.0e-6
VARIANT_RMS_ERROR = 5.0e-5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    return parser.parse_args()


CAPTURES = (
    "bistro-rtx-full",
    "bistro-rtx-auto-cold",
    "bistro-rtx-auto-warm",
    "bistro-rq-full",
    "bistro-rq-auto-cold",
    "bistro-rq-auto-warm",
    "bistro-rq-only-auto",
    "regression-rtx-full",
    "regression-rtx-auto",
    "regression-rq-full",
    "regression-rq-auto",
)


def load_matrix(root: Path) -> tuple[dict[str, HdrImage], dict[str, dict[str, object]]]:
    return (
        {name: read_hdr(root / f"{name}.hdr") for name in CAPTURES},
        {
            name: json.loads((root / f"{name}.json").read_text(encoding="utf-8"))
            for name in CAPTURES
        },
    )


def evidence(name: str, image: HdrImage, metrics: dict[str, object]) -> dict[str, bool]:
    variant = metrics.get("ray_shader_variant")
    if not isinstance(variant, dict):
        variant = {}
    capabilities = metrics.get("capabilities")
    if not isinstance(capabilities, dict):
        capabilities = {}
    expected_mode = "RayQuery" if "-rq" in name else "RayTracing"
    expected_profile = "bistro" if name.startswith("bistro-") else "rendering-regression"
    expected_selection = "full" if name.endswith("-full") else "auto"
    requested_mask = variant.get("requested_mask")
    active_mask = variant.get("active_mask")
    query_only = name == "bistro-rq-only-auto"
    checks = {
        "capture_schema": metrics.get("type") == "evoengine_ray_capture",
        "profile": metrics.get("demo_profile") == expected_profile,
        "linear_hdr": metrics.get("output_format") == "radiance_hdr_linear",
        "dimensions": (image.width, image.height) == EXPECTED_SIZE,
        "metrics_dimensions": (metrics.get("width"), metrics.get("height")) == EXPECTED_SIZE,
        "mode": metrics.get("render_mode") == expected_mode,
        "spp": metrics.get("effective_spp") == EXPECTED_SPP,
        "deterministic": metrics.get("deterministic") is True,
        "auto_spp_disabled": metrics.get("auto_spp_enabled") is False,
        "clamp_enabled": metrics.get("firefly_clamp_enabled") is True,
        "clamp_threshold": metrics.get("firefly_clamp_threshold") == 10.0,
        "ser_requested_disabled": metrics.get("ser_mode_requested") == "Disabled",
        "ser_disabled": metrics.get("ser_enabled") is False,
        "variant_technique": variant.get("technique") == expected_mode,
        "variant_selection": variant.get("selection_mode") == expected_selection,
        "variant_ready": variant.get("ready") is True and variant.get("pending") is False,
        "variant_not_failed": variant.get("failed") is False,
        "variant_exact": isinstance(requested_mask, int)
        and requested_mask == active_mask
        and variant.get("requested_key") == variant.get("active_key"),
        "variant_covers_scene": isinstance(requested_mask, int)
        and isinstance(active_mask, int)
        and (active_mask & requested_mask) == requested_mask,
        "acceleration_structure": capabilities.get("acceleration_structure") is True,
        "ray_query": capabilities.get("ray_query") is True,
        "ray_pipeline_capability": capabilities.get("ray_tracing_pipeline") is (not query_only),
    }
    if expected_selection == "full":
        checks["full_mask"] = requested_mask == ALL_FEATURES
        checks["full_fallback"] = variant.get("fallback_active") is True
    else:
        checks["specialized_active"] = variant.get("fallback_active") is False
    if name.endswith("auto-cold"):
        checks["cold_compiled"] = variant.get("cache_source") == "compiled"
    if name.endswith("auto-warm"):
        checks["warm_cache_hit"] = variant.get("cache_source") in {"disk", "memory"}
    return checks


def exact(images: dict[str, HdrImage], first: str, second: str) -> dict[str, object]:
    return compare_hdr_images(images[first], images[second])


def equivalent(comparison: dict[str, object]) -> bool:
    return (
        bool(comparison["same_dimensions"])
        and float(comparison["max_abs_error"]) <= VARIANT_MAX_ABS_ERROR
        and float(comparison["mean_abs_error"]) <= VARIANT_MEAN_ABS_ERROR
        and float(comparison["rms_error"]) <= VARIANT_RMS_ERROR
    )


def main() -> int:
    args = parse_args()
    images, metrics = load_matrix(args.matrix_dir)
    capture_evidence = {
        name: evidence(name, images[name], metrics[name]) for name in CAPTURES
    }
    comparisons = {
        "bistro_rtx_full_auto": exact(images, "bistro-rtx-full", "bistro-rtx-auto-cold"),
        "bistro_rtx_cold_warm": exact(
            images, "bistro-rtx-auto-cold", "bistro-rtx-auto-warm"
        ),
        "bistro_rq_full_auto": exact(images, "bistro-rq-full", "bistro-rq-auto-cold"),
        "bistro_rq_cold_warm": exact(
            images, "bistro-rq-auto-cold", "bistro-rq-auto-warm"
        ),
        "bistro_rq_query_only": exact(
            images, "bistro-rq-auto-warm", "bistro-rq-only-auto"
        ),
        "regression_rtx_full_auto": exact(
            images, "regression-rtx-full", "regression-rtx-auto"
        ),
        "regression_rq_full_auto": exact(
            images, "regression-rq-full", "regression-rq-auto"
        ),
    }
    full_auto_comparisons = {
        "bistro_rtx_full_auto",
        "bistro_rq_full_auto",
        "regression_rtx_full_auto",
        "regression_rq_full_auto",
    }
    gates = {
        **{f"evidence_{name}": all(checks.values()) for name, checks in capture_evidence.items()},
        **{
            ("equivalent_" if name in full_auto_comparisons else "exact_") + name:
                equivalent(comparison)
                if name in full_auto_comparisons
                else bool(comparison["exact_match"])
            for name, comparison in comparisons.items()
        },
    }
    summary = {
        "schema": 1,
        "matrix_dir": str(args.matrix_dir.resolve()),
        "expected_size": list(EXPECTED_SIZE),
        "expected_spp": EXPECTED_SPP,
        "full_auto_tolerance": {
            "max_abs_error": VARIANT_MAX_ABS_ERROR,
            "mean_abs_error": VARIANT_MEAN_ABS_ERROR,
            "rms_error": VARIANT_RMS_ERROR,
        },
        "capture_evidence": capture_evidence,
        "comparisons": comparisons,
        "gates": gates,
        "passed": all(gates.values()),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
