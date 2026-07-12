#!/usr/bin/env python3
"""Capture and validate M9 glTF texture, UV, sampler, and tangent fidelity."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import validate_ray_debug_views as m7
import validate_raytracer_m8 as m8
from compare_reference_render import PngImage, read_png


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--editor", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("out/m9-validation"))
    parser.add_argument(
        "--expectations",
        type=Path,
        default=Path(__file__).with_name("raytracer_m9_expectations.json"),
    )
    parser.add_argument(
        "--m8-expectations",
        type=Path,
        default=Path(__file__).with_name("raytracer_m8_expectations.json"),
    )
    parser.add_argument(
        "--m7-validation",
        type=Path,
        default=Path("out/m7-validation/atlas/validation.json"),
    )
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--timeout-seconds", type=int, default=900)
    return parser.parse_args()


def inherited_args(args: argparse.Namespace) -> argparse.Namespace:
    result = argparse.Namespace(**vars(args))
    result.expectations = args.m8_expectations
    return result


def roi_range(values: list[float], expectation: dict[str, object]) -> bool:
    minimum = list(expectation.get("min_rgb", [-math.inf] * 3))
    maximum = list(expectation.get("max_rgb", [math.inf] * 3))
    passed = all(float(minimum[channel]) <= values[channel] <= float(maximum[channel]) for channel in range(3))
    if "channel_spread_max" in expectation:
        passed = passed and max(values) - min(values) <= float(expectation["channel_spread_max"])
    return passed


def rgb_stddev(samples: list[float], mean: list[float]) -> list[float]:
    pixel_count = max(len(samples) // 3, 1)
    return [
        math.sqrt(sum((value - mean[channel]) ** 2 for value in samples[channel::3]) / pixel_count)
        for channel in range(3)
    ]


def false_gates(report: dict[str, object]) -> set[str]:
    gates = report.get("gates", {})
    if not isinstance(gates, dict):
        return {"<missing-gates>"}
    return {str(name) for name, passed in gates.items() if passed is not True}


def png_roi_samples(image: PngImage, rect: list[int]) -> list[float]:
    x0, y0, x1, y1 = rect
    if not (0 <= x0 < x1 <= image.width and 0 <= y0 < y1 <= image.height):
        raise ValueError(f"PNG ROI {rect} is outside {image.width}x{image.height}")
    samples: list[float] = []
    for y in range(y0, y1):
        for x in range(x0, x1):
            offset = (y * image.width + x) * 4
            samples.extend(image.rgba[offset + channel] / 255.0 for channel in range(3))
    return samples


def validate(args: argparse.Namespace) -> dict[str, object]:
    expectations = json.loads(args.expectations.read_text(encoding="utf-8"))
    inherited = m8.validate(inherited_args(args))
    raster = read_png(args.output_dir / "raster.png")
    raster_samples = {
        str(roi["name"]): png_roi_samples(raster, list(roi["rect"]))
        for roi in expectations.get("raster_rois", [])
    }
    raster_values = {name: m7.mean_rgb(samples) for name, samples in raster_samples.items()}
    raster_stddev = {name: rgb_stddev(samples, raster_values[name]) for name, samples in raster_samples.items()}
    raster_gates = {
        f"roi_{roi['name']}": roi_range(raster_values[str(roi["name"])], roi)
        for roi in expectations.get("raster_rois", [])
    }
    raster_gates.update(
        {
            f"roi_{roi['name']}_stddev": max(raster_stddev[str(roi["name"])]) <= float(roi["stddev_max"])
            for roi in expectations.get("raster_rois", [])
            if "stddev_max" in roi
        }
    )
    raster_gates.update(m8.relation_gates(raster_values, expectations.get("raster_relations", [])))

    atlas_dir = args.output_dir / "atlas"
    images = {lane: m7.read_hdr(atlas_dir / f"{lane}.hdr") for lane in m8.COMPACT_RAY_LANES}
    ray_values: dict[str, dict[str, list[float]]] = {}
    ray_stddev: dict[str, dict[str, list[float]]] = {}
    ray_gates: dict[str, bool] = {}
    for lane, image in images.items():
        lane_values: dict[str, list[float]] = {}
        lane_stddev: dict[str, list[float]] = {}
        for roi in expectations.get("ray_rois", []):
            name = str(roi["name"])
            view = m7.VIEW_NAMES.index(str(roi["view"]))
            samples = m7.view_values(image, view, list(roi["rect"]))
            lane_values[name] = m7.mean_rgb(samples)
            lane_stddev[name] = rgb_stddev(samples, lane_values[name])
            ray_gates[f"roi_{name}_{lane}"] = roi_range(lane_values[name], roi)
            if "stddev_max" in roi:
                ray_gates[f"roi_{name}_{lane}_stddev"] = max(lane_stddev[name]) <= float(roi["stddev_max"])
        ray_values[lane] = lane_values
        ray_stddev[lane] = lane_stddev
        for name, passed in m8.relation_gates(lane_values, expectations.get("ray_relations", [])).items():
            ray_gates[f"{name}_{lane}"] = passed

    for roi in expectations.get("ray_rois", []):
        name = str(roi["name"])
        maximum = float(roi.get("cross_technique_max", 0.02))
        ray_gates[f"roi_{name}_cross_technique"] = max(
            abs(a - b) for a, b in zip(ray_values["rtx"][name], ray_values["rq-only"][name])
        ) <= maximum

    classifications = list(expectations.get("classifications", []))
    allowed_m8_failures = {str(name) for name in expectations.get("allowed_inherited_m8_gate_failures", [])}
    allowed_atlas_failures = {
        str(name) for name in expectations.get("allowed_inherited_atlas_gate_failures", [])
    }
    inherited_atlas = inherited.get("ray_atlas", {})
    gates = {
        "m8_regression": false_gates(inherited) <= allowed_m8_failures
        and isinstance(inherited_atlas, dict)
        and false_gates(inherited_atlas) <= allowed_atlas_failures,
        "three_precommit_launches": inherited.get("focused_capture_count") == 3,
        "approved_five_launch_exception": expectations.get("approved_total_launch_count") == 5
        and expectations.get("rejected_launch_count") == 1,
        **{f"raster_{name}": passed for name, passed in raster_gates.items()},
        **ray_gates,
        "m9_classifications_current": bool(classifications)
        and all(
            isinstance(item, dict)
            and item.get("category") in m7.ALLOWED_CLASSIFICATIONS
            and item.get("category") != "evoengine_defect"
            and item.get("resolved_in") == "M9"
            for item in classifications
        ),
        "m7_sampler_tangent_defect_resolved": any(
            isinstance(item, dict)
            and item.get("source") == "raytracer_m7_expectations.json"
            and item.get("resolved_in") == "M9"
            for item in expectations.get("supersedes", [])
        ),
    }
    return {
        "schema": 1,
        "reference_revision": expectations["reference_revision"],
        "focused_capture_count": 3,
        "planned_delivery_capture_count": expectations.get("approved_total_launch_count", 4),
        "rejected_launch_count": expectations.get("rejected_launch_count", 0),
        "resource_cases": expectations.get("resource_cases", []),
        "unit_test_filter": expectations.get("unit_test_filter", ""),
        "inherited_m8": inherited,
        "raster_rois": raster_values,
        "raster_roi_stddev": raster_stddev,
        "ray_rois": ray_values,
        "ray_roi_stddev": ray_stddev,
        "supersedes": expectations.get("supersedes", []),
        "classifications": classifications,
        "gates": gates,
        "passed": all(gates.values()),
    }


def self_test() -> None:
    m8.self_test()
    if not roi_range([0.5, 0.51, 0.49], {"min_rgb": [0.4] * 3, "max_rgb": [0.6] * 3,
                                          "channel_spread_max": 0.03}):
        raise AssertionError("neutral linear-mip ROI unexpectedly failed")
    if roi_range([0.214, 0.214, 0.214], {"min_rgb": [0.4] * 3, "max_rgb": [0.6] * 3}):
        raise AssertionError("gamma-space mip rejection unexpectedly passed")
    if max(rgb_stddev([0.49, 0.5, 0.51, 0.51, 0.5, 0.49], [0.5, 0.5, 0.5])) > 0.011:
        raise AssertionError("uniform mip standard-deviation check unexpectedly failed")
    if max(rgb_stddev([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], [0.5, 0.5, 0.5])) < 0.49:
        raise AssertionError("aliased checker standard-deviation rejection unexpectedly passed")
    if false_gates({"gates": {"kept": True, "superseded": False}}) != {"superseded"}:
        raise AssertionError("inherited gate filtering did not isolate the superseded failure")


def main() -> int:
    args = parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.capture or args.dry_run:
        m8.capture(inherited_args(args))
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
