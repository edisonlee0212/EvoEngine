#!/usr/bin/env python3
"""Capture the deterministic M0 DDGI small-emitter quality and performance baseline."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

from compare_reference_render import HdrImage, compare_hdr_images, read_image


SEED = 0x6D2B79F5
FIXTURES = (
    "emissive-large",
    "emissive-small-equal-radiance",
    "emissive-small-equal-power",
)
REGIONS = {
    "receiver_wall": (0.365, 0.231, 0.635, 0.491),
    "receiver_floor": (0.339, 0.741, 0.651, 0.815),
}
FATAL_PATTERNS = (
    re.compile(r"Vulkan\s+\[Validation\]-\[Error\]", re.IGNORECASE),
    re.compile(r"\bVK_ERROR_DEVICE_LOST\b", re.IGNORECASE),
    re.compile(r"\bdevice\s+lost\b", re.IGNORECASE),
    re.compile(r"\bunhandled\s+exception\b", re.IGNORECASE),
    re.compile(r"\bfatal(?:\s+error)?\b", re.IGNORECASE),
    re.compile(r"\bassertion\s+failed\b", re.IGNORECASE),
)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="RelWithDebInfo", help="Installed build configuration label.")
    parser.add_argument("--width", type=int, default=1920, help="Capture width; must be 1920.")
    parser.add_argument("--height", type=int, default=1080, help="Capture height; must be 1080.")
    parser.add_argument("--measure-frames", type=int, default=120, help="Measured DDGI frames; must be 120.")
    parser.add_argument("--output-dir", type=Path, default=root / "out/ddgi-small-emitter/m0")
    parser.add_argument("--timeout", type=float, default=900.0, help="Per-launch timeout in seconds.")
    parser.add_argument("--editor", type=Path, help="Installed EvoEngineEditor executable override.")
    parser.add_argument("--uniform-rays", type=int, default=192, help="Uniform/fixed rays per probe.")
    parser.add_argument("--guided-rays", type=int, default=64, help="Additive guided irradiance rays per probe.")
    parser.add_argument("--guided-emitters", type=int, default=4, help="Maximum emissive guide records (1-8).")
    parser.add_argument("--self-test", action="store_true", help="Validate baseline math and region analysis only.")
    return parser.parse_args()


def require_hdr(path: Path, width: int, height: int) -> HdrImage:
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"Missing or empty DDGI small-emitter capture: {path}")
    image = read_image(path)
    if not isinstance(image, HdrImage):
        raise RuntimeError(f"DDGI small-emitter capture is not linear HDR: {path}")
    if (image.width, image.height) != (width, height):
        raise RuntimeError(
            f"DDGI small-emitter capture has {image.width}x{image.height}; expected {width}x{height}: {path}"
        )
    if not all(math.isfinite(value) for value in image.rgb):
        raise RuntimeError(f"DDGI small-emitter capture contains non-finite values: {path}")
    return image


def region_pixel_bounds(image: HdrImage, bounds: tuple[float, float, float, float]) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = bounds
    return (
        max(0, min(image.width, math.floor(x0 * image.width))),
        max(0, min(image.height, math.floor(y0 * image.height))),
        max(0, min(image.width, math.ceil(x1 * image.width))),
        max(0, min(image.height, math.ceil(y1 * image.height))),
    )


def summarize_region(
    image: HdrImage,
    bounds: tuple[float, float, float, float],
    subtract: HdrImage | None = None,
) -> dict[str, object]:
    if subtract and (image.width, image.height) != (subtract.width, subtract.height):
        raise ValueError("Region subtraction requires equal image dimensions.")
    x0, y0, x1, y1 = region_pixel_bounds(image, bounds)
    if x1 <= x0 or y1 <= y0:
        raise ValueError(f"Empty normalized region: {bounds}")
    luminance_sum = 0.0
    luminance_square_sum = 0.0
    peak_luminance = 0.0
    nonzero_count = 0
    sample_count = 0
    for y in range(y0, y1):
        for x in range(x0, x1):
            index = (y * image.width + x) * 3
            color = [max(float(image.rgb[index + channel]), 0.0) for channel in range(3)]
            if subtract:
                color = [
                    max(color[channel] - max(float(subtract.rgb[index + channel]), 0.0), 0.0)
                    for channel in range(3)
                ]
            luminance = color[0] * 0.2126 + color[1] * 0.7152 + color[2] * 0.0722
            luminance_sum += luminance
            luminance_square_sum += luminance * luminance
            peak_luminance = max(peak_luminance, luminance)
            nonzero_count += luminance > 1.0e-8
            sample_count += 1
    return {
        "normalized_bounds": list(bounds),
        "pixel_bounds": [x0, y0, x1, y1],
        "sample_count": sample_count,
        "mean_luminance": luminance_sum / sample_count,
        "rms_luminance": math.sqrt(luminance_square_sum / sample_count),
        "peak_luminance": peak_luminance,
        "nonzero_fraction": nonzero_count / sample_count,
    }


def summarize_image(image: HdrImage, subtract: HdrImage | None = None) -> dict[str, object]:
    return {name: summarize_region(image, bounds, subtract) for name, bounds in REGIONS.items()}


def contribution_mean(regions: dict[str, object]) -> float:
    return sum(float(region["mean_luminance"]) for region in regions.values()) / len(regions)


def receiver_relative_l2(candidate: HdrImage, reference: HdrImage, normalize_energy: bool) -> float:
    if (candidate.width, candidate.height) != (reference.width, reference.height):
        raise ValueError("Receiver comparison requires equal image dimensions.")
    samples: list[tuple[float, float]] = []
    for bounds in REGIONS.values():
        x0, y0, x1, y1 = region_pixel_bounds(candidate, bounds)
        for y in range(y0, y1):
            for x in range(x0, x1):
                index = (y * candidate.width + x) * 3
                candidate_luminance = sum(
                    max(float(candidate.rgb[index + channel]), 0.0) * weight
                    for channel, weight in enumerate((0.2126, 0.7152, 0.0722))
                )
                reference_luminance = sum(
                    max(float(reference.rgb[index + channel]), 0.0) * weight
                    for channel, weight in enumerate((0.2126, 0.7152, 0.0722))
                )
                samples.append((candidate_luminance, reference_luminance))
    scale = 1.0
    if normalize_energy:
        scale = sum(reference_value for _, reference_value in samples) / max(
            sum(candidate_value for candidate_value, _ in samples), 1.0e-12
        )
    squared_error = sum((candidate_value * scale - reference_value) ** 2 for candidate_value, reference_value in samples)
    squared_reference = sum(reference_value**2 for _, reference_value in samples)
    return math.sqrt(squared_error / max(squared_reference, 1.0e-24))


def relative_difference(lhs: float, rhs: float) -> float:
    return abs(lhs - rhs) / max(abs(rhs), 1.0e-12)


def json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


def validate_emissive_sampling_report(report: dict[str, object], require_stats: bool) -> None:
    sampling = report.get("emissive_sampling")
    if not isinstance(sampling, dict):
        raise RuntimeError("DDGI report is missing emissive_sampling diagnostics.")
    if bool(sampling.get("stats_available")) != require_stats:
        raise RuntimeError(f"Unexpected emissive sampling stats availability: {sampling}")
    if not require_stats:
        return
    required_positive = ("inventory_triangles", "eligible_instances", "estimated_emitted_power", "nee_attempts")
    if any(float(sampling.get(name, 0)) <= 0 for name in required_positive):
        raise RuntimeError(f"Emissive sampling diagnostics lack a positive inventory or attempts: {sampling}")
    if (
        int(sampling.get("excluded_emissive_instances", -1)) != 0
        or int(sampling.get("unrepresentable_probabilities", -1)) != 0
    ):
        raise RuntimeError(f"Baseline fixture has an unhealthy emissive inventory: {sampling}")
    attempts = int(sampling.get("nee_attempts", 0))
    outcomes = sum(
        int(sampling.get(name, 0))
        for name in (
            "zero_pdf_rejects",
            "emitter_backface_rejects",
            "alpha_mask_rejects",
            "invalid_sample_rejects",
            "receiver_backface_rejects",
            "shadowed_samples",
            "zero_radiance_samples",
            "nonzero_contributions",
        )
    )
    if outcomes != attempts or int(sampling.get("nonzero_contributions", 0)) <= 0:
        raise RuntimeError(f"Emissive sampling outcomes do not partition attempts: {sampling}")


def run_command(command: list[str], cwd: Path, environment: dict[str, str], timeout: float, log_path: Path) -> None:
    started = time.perf_counter()
    completed = subprocess.run(
        command,
        cwd=cwd,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=timeout,
        check=False,
    )
    elapsed = time.perf_counter() - started
    log_path.write_text(completed.stdout, encoding="utf-8")
    print(f"[{elapsed:.1f}s] {' '.join(command)}")
    if completed.returncode != 0:
        raise RuntimeError(f"Installed editor exited with code {completed.returncode}; log={log_path}")
    fatal_line = next(
        (line for line in completed.stdout.splitlines() if any(pattern.search(line) for pattern in FATAL_PATTERNS)),
        None,
    )
    if fatal_line:
        raise RuntimeError(f"Fatal runtime output: {fatal_line}; log={log_path}")


def common_command(editor: Path, image_path: Path, fixture: str, width: int, height: int) -> list[str]:
    return [
        str(editor),
        "--demo",
        "rendering-regression",
        "--editor",
        "--capture-demo-preview",
        str(image_path),
        "--preview-width",
        str(width),
        "--preview-height",
        str(height),
        "--preview-deterministic",
        "--preview-ddgi-fixture",
        fixture,
        "--preview-ddgi-seed",
        str(SEED),
    ]


def capture_ddgi(
    editor: Path,
    output_dir: Path,
    environment: dict[str, str],
    fixture: str,
    phase: str,
    width: int,
    height: int,
    measure_frames: int,
    timeout: float,
    guided_rays: int,
    guided_emitters: int,
    uniform_rays: int,
) -> tuple[HdrImage, dict[str, object]]:
    image_path = output_dir / f"{fixture}-{phase}.hdr"
    report_path = output_dir / f"{fixture}-{phase}.json"
    log_path = output_dir / f"{fixture}-{phase}.log"
    command = common_command(editor, image_path, fixture, width, height)
    command.extend(
        [
            "--preview-render-mode",
            "rasterization",
            "--preview-warmup-frames",
            "8",
            "--preview-ddgi-report",
            str(report_path),
            "--preview-ddgi-measure-frames",
            str(measure_frames),
            "--preview-ddgi-guided-rays",
            str(guided_rays),
            "--preview-ddgi-uniform-rays",
            str(uniform_rays),
            "--preview-ddgi-guided-emitters",
            str(guided_emitters),
        ]
    )
    if phase == "disabled":
        command.append("--preview-ddgi-disabled")
    run_command(command, editor.parent, environment, timeout, log_path)
    image = require_hdr(image_path, width, height)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("schema_version") != 7 or report.get("fixture_definition", {}).get("version") != 6:
        raise RuntimeError(f"Unexpected DDGI small-emitter report contract: {report_path}")
    validate_emissive_sampling_report(report, phase != "disabled")
    return image, report


def capture_reference(
    editor: Path,
    output_dir: Path,
    environment: dict[str, str],
    fixture: str,
    width: int,
    height: int,
    timeout: float,
) -> HdrImage:
    image_path = output_dir / f"{fixture}-reference.hdr"
    log_path = output_dir / f"{fixture}-reference.log"
    command = common_command(editor, image_path, fixture, width, height)
    command.extend(
        [
            "--preview-render-mode",
            "raytracing",
            "--preview-warmup-frames",
            "64",
            "--preview-sample-size",
            "4",
            "--preview-ray-debug",
            "beauty",
            "--preview-ser",
            "disabled",
            "--preview-auto-spp",
            "disabled",
            "--preview-ddgi-reference",
        ]
    )
    run_command(command, editor.parent, environment, timeout, log_path)
    return require_hdr(image_path, width, height)


def self_test() -> None:
    small_area = 2.0 * (0.16 * 0.025 + 0.16 * 0.16 + 0.025 * 0.16)
    large_area = 2.0 * (1.35 * 0.025 + 1.35 * 1.1 + 0.025 * 1.1)
    ratio = large_area / small_area
    if not math.isclose(ratio, 46.01934523809524, rel_tol=1.0e-12):
        raise AssertionError(f"Unexpected equal-power fixture ratio: {ratio}")
    if relative_difference(2.0, 2.0) != 0.0 or not math.isclose(relative_difference(1.0, 2.0), 0.5):
        raise AssertionError("Relative-difference contract failed.")
    if json_safe({"finite": 1.0, "infinite": math.inf}) != {"finite": 1.0, "infinite": None}:
        raise AssertionError("JSON-safe metric conversion failed.")
    candidate = HdrImage(Path("candidate.hdr"), 1, 1, (1.0, 1.0, 1.0), "candidate", "test")
    reference = HdrImage(Path("reference.hdr"), 1, 1, (2.0, 2.0, 2.0), "reference", "test")
    if not math.isclose(receiver_relative_l2(candidate, reference, False), 0.5):
        raise AssertionError("Receiver relative-L2 contract failed.")
    if receiver_relative_l2(candidate, reference, True) != 0.0:
        raise AssertionError("Energy-normalized receiver relative-L2 contract failed.")
    validate_emissive_sampling_report(
        {
            "emissive_sampling": {
                "stats_available": True,
                "inventory_triangles": 1,
                "eligible_instances": 1,
                "excluded_emissive_instances": 0,
                "unrepresentable_probabilities": 0,
                "estimated_emitted_power": 1.0,
                "nee_attempts": 4,
                "receiver_backface_rejects": 1,
                "shadowed_samples": 1,
                "zero_radiance_samples": 1,
                "nonzero_contributions": 1,
            }
        },
        True,
    )
    print("DDGI small-emitter baseline self-test passed.")


def main() -> int:
    args = parse_args()
    try:
        if args.self_test:
            self_test()
            return 0
        if (args.width, args.height) != (1920, 1080):
            raise ValueError("The M0 DDGI small-emitter baseline requires exactly 1920x1080.")
        if args.measure_frames != 120:
            raise ValueError("The M0 DDGI small-emitter baseline requires exactly 120 measured DDGI frames.")
        if args.timeout <= 0:
            raise ValueError("--timeout must be positive.")
        if args.guided_rays < 0 or args.guided_rays > 4096:
            raise ValueError("--guided-rays must be between 0 and 4096.")
        if args.uniform_rays < 1 or args.uniform_rays > 4096:
            raise ValueError("--uniform-rays must be between 1 and 4096.")
        if args.guided_emitters < 1 or args.guided_emitters > 8:
            raise ValueError("--guided-emitters must be between 1 and 8.")

        root = repo_root()
        editor = (args.editor or root / "out/install/vs2026-x64/bin/EvoEngineEditor.exe").resolve()
        if not editor.is_file():
            raise FileNotFoundError(f"Installed editor not found: {editor}")
        output_dir = args.output_dir if args.output_dir.is_absolute() else root / args.output_dir
        output_dir = output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        shader_cache = output_dir / "shader-cache"
        shader_cache.mkdir(parents=True, exist_ok=True)
        environment = os.environ.copy()
        environment["EVOENGINE_SHADER_CACHE_DIR"] = str(shader_cache)
        environment["EVOENGINE_IMGUI_INI_PATH"] = str(output_dir / "imgui.ini")

        captures: dict[str, dict[str, object]] = {}
        images: dict[str, dict[str, HdrImage]] = {}
        for fixture in FIXTURES:
            enabled, enabled_report = capture_ddgi(
                editor, output_dir, environment, fixture, "enabled", args.width, args.height,
                args.measure_frames, args.timeout, args.guided_rays, args.guided_emitters, args.uniform_rays
            )
            disabled, disabled_report = capture_ddgi(
                editor, output_dir, environment, fixture, "disabled", args.width, args.height,
                args.measure_frames, args.timeout, args.guided_rays, args.guided_emitters, args.uniform_rays
            )
            reference = capture_reference(
                editor, output_dir, environment, fixture, args.width, args.height, args.timeout
            )
            images[fixture] = {"enabled": enabled, "disabled": disabled, "reference": reference}
            captures[fixture] = {
                "enabled": {
                    "file": enabled.path.name,
                    "sha256": enabled.digest,
                    "regions": summarize_image(enabled),
                },
                "disabled": {
                    "file": disabled.path.name,
                    "sha256": disabled.digest,
                    "regions": summarize_image(disabled),
                },
                "ddgi_contribution": {
                    "regions": summarize_image(enabled, disabled),
                },
                "reference": {
                    "file": reference.path.name,
                    "sha256": reference.digest,
                    "regions": summarize_image(reference),
                },
                "runtime_report": enabled_report,
            }

        repeat, repeat_report = capture_ddgi(
            editor, output_dir, environment, "emissive-small-equal-power", "repeat", args.width, args.height,
            args.measure_frames, args.timeout, args.guided_rays, args.guided_emitters, args.uniform_rays
        )
        equal_power = images["emissive-small-equal-power"]
        repeatability = compare_hdr_images(equal_power["enabled"], repeat)
        repeatability["runtime_report"] = repeat_report

        ddgi_energy = {
            fixture: contribution_mean(captures[fixture]["ddgi_contribution"]["regions"])
            for fixture in FIXTURES
        }
        reference_energy = {
            fixture: contribution_mean(captures[fixture]["reference"]["regions"])
            for fixture in FIXTURES
        }
        comparisons = {
            "equal_power_small_vs_large_ddgi_relative_error": relative_difference(
                ddgi_energy["emissive-small-equal-power"], ddgi_energy["emissive-large"]
            ),
            "equal_power_small_vs_large_reference_relative_error": relative_difference(
                reference_energy["emissive-small-equal-power"], reference_energy["emissive-large"]
            ),
            "equal_power_small_ddgi_vs_reference_relative_error": relative_difference(
                ddgi_energy["emissive-small-equal-power"], reference_energy["emissive-small-equal-power"]
            ),
            "large_ddgi_vs_reference_relative_error": relative_difference(
                ddgi_energy["emissive-large"], reference_energy["emissive-large"]
            ),
            "equal_radiance_small_to_large_ddgi_ratio":
                ddgi_energy["emissive-small-equal-radiance"] / max(ddgi_energy["emissive-large"], 1.0e-12),
            "equal_radiance_small_to_large_reference_ratio":
                reference_energy["emissive-small-equal-radiance"] / max(reference_energy["emissive-large"], 1.0e-12),
            "large_ddgi_vs_reference_receiver_relative_l2": receiver_relative_l2(
                images["emissive-large"]["enabled"], images["emissive-large"]["reference"], False
            ),
            "equal_power_small_ddgi_vs_reference_receiver_relative_l2": receiver_relative_l2(
                equal_power["enabled"], equal_power["reference"], False
            ),
            "large_energy_normalized_receiver_relative_l2": receiver_relative_l2(
                images["emissive-large"]["enabled"], images["emissive-large"]["reference"], True
            ),
            "equal_power_small_energy_normalized_receiver_relative_l2": receiver_relative_l2(
                equal_power["enabled"], equal_power["reference"], True
            ),
        }
        evidence = {
            "schema_version": 1,
            "contract": {
                "resolution": [args.width, args.height],
                "measure_frames": args.measure_frames,
                "reference_frames": 64,
                "reference_samples_per_frame": 4,
                "deterministic_seed": SEED,
                "output_encoding": "linear-rgbe-hdr",
                "performance_authority": "NVIDIA GeForce RTX 5070",
                "small_to_large_box_surface_area_ratio": 46.01934523809524,
                "uniform_rays_per_probe": args.uniform_rays,
                "guided_rays_per_probe": args.guided_rays,
                "guided_emitter_limit": args.guided_emitters,
            },
            "captures": captures,
            "repeatability": repeatability,
            "comparisons": comparisons,
            "proposed_thresholds": {
                "status": "approved",
                "maximum_equal_power_ddgi_relative_error": 0.15,
                "maximum_ddgi_to_path_trace_energy_relative_error": 0.30,
                "maximum_repeat_relative_l2_error": 1.0e-6,
                "maximum_energy_normalized_receiver_relative_l2": 0.30,
                "maximum_static_response_frames": 48,
                "maximum_ddgi_gpu_time_regression_fraction": 0.05,
            },
        }
        evidence_path = output_dir / "evidence.json"
        evidence_path.write_text(json.dumps(json_safe(evidence), indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(json.dumps({"evidence": str(evidence_path), "comparisons": comparisons}, indent=2))
        return 0
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
        print(f"DDGI small-emitter baseline failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
