#!/usr/bin/env python3
"""Validate split environment lighting controls in one installed-editor launch."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys

from compare_reference_render import PngImage, compare_render_images, read_image


RASTER_CONTROL_NAMES = (
    "default",
    "indirect-off",
    "all-off",
    "sky-off",
    "background-only",
    "background-lit",
    "sky-double",
    "indirect-double",
    "sky-off-indirect-double",
    "sky-double-indirect-off",
    "both-double",
)
RAY_CONTROL_NAMES = (
    "ray-neutral-reference",
    "ray-sky-double",
    "ray-background-lit",
    "ray-indirect-off",
    "ray-indirect-double",
    "ray-sky-off",
    "ray-all-off",
)
SYNTHETIC_CAPTURE_NAMES = RASTER_CONTROL_NAMES + RAY_CONTROL_NAMES
SPONZA_M15_CAPTURE_NAMES = (
    "sponza-diffuse",
    "sponza-unoccluded",
    "sponza-gtao-beauty",
    "sponza-gtao-diffuse",
    "sponza-gtao-diffuse-probe-double",
    "sponza-gtao-visibility",
    "sponza-gtao-occluded",
    "sponza-ssao-diffuse",
    "sponza-ssao-occluded",
    "sponza-gtao-occluded-indirect-double",
    "sponza-gtao-occluded-ddgi-disabled",
    "sponza-gtao-occluded-ddgi-outside",
    "sponza-gtao-left-room",
    "sponza-gtao-right-room",
)
CAPTURE_NAMES = SYNTHETIC_CAPTURE_NAMES + (
    "sponza-default",
) + SPONZA_M15_CAPTURE_NAMES + (
    "sponza-repeatability-anchor",
    "sponza-ray-reference",
)
SPONZA_QUALITY_ERROR_MAX = 0.07743769200665579
REGIONS = {
    "background": (0.05, 0.05, 0.20, 0.20),
    "dielectric": (0.30, 0.44, 0.36, 0.53),
    "metal": (0.64, 0.44, 0.70, 0.53),
    "ibl_diffuse": (0.72, 0.19, 0.76, 0.25),
    "direct": (0.43, 0.12, 0.47, 0.20),
    "emission": (0.53, 0.12, 0.57, 0.20),
    "sponza_exact_metal": (0.435, 0.375, 0.465, 0.425),
    "sponza_rough_metal": (0.535, 0.375, 0.565, 0.425),
    "sponza_smooth_dielectric": (0.435, 0.565, 0.465, 0.615),
    "sponza_rough_dielectric": (0.535, 0.565, 0.565, 0.615),
    "sponza_room": (0.20, 0.20, 0.80, 0.82),
}
FATAL_PATTERNS = (
    re.compile(r"Vulkan\s+\[Validation\]-\[Error\]", re.IGNORECASE),
    re.compile(r"\bVK_ERROR_DEVICE_LOST\b", re.IGNORECASE),
    re.compile(r"\bdevice\s+lost\b", re.IGNORECASE),
    re.compile(r"\bunhandled\s+exception\b", re.IGNORECASE),
    re.compile(r"\bfatal(?:\s+error)?\b", re.IGNORECASE),
    re.compile(r"\bassertion\s+failed\b", re.IGNORECASE),
    re.compile(r"EVOENGINE_ENVIRONMENT_LIGHTING_ERROR", re.IGNORECASE),
)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="RelWithDebInfo", help="Installed build configuration label.")
    parser.add_argument("--width", type=int, default=1920, help="Render width; must be 1920.")
    parser.add_argument("--height", type=int, default=1080, help="Render height; must be 1080.")
    parser.add_argument("--output-dir", type=Path, default=root / "out/environment-lighting-validation-m15")
    parser.add_argument("--timeout", type=float, default=900.0, help="Maximum launch duration in seconds.")
    parser.add_argument("--editor", type=Path, help="Installed EvoEngineEditor executable override.")
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def region_luminance(image: PngImage, region: tuple[float, float, float, float]) -> float:
    begin_x = int(region[0] * image.width)
    begin_y = int(region[1] * image.height)
    end_x = int(region[2] * image.width + 0.999999)
    end_y = int(region[3] * image.height + 0.999999)
    total = 0.0
    count = 0
    for y in range(begin_y, end_y):
        for x in range(begin_x, end_x):
            base = (y * image.width + x) * 4
            total += (
                image.rgba[base] * 0.2126
                + image.rgba[base + 1] * 0.7152
                + image.rgba[base + 2] * 0.0722
            ) / 255.0
            count += 1
    if count == 0:
        raise RuntimeError("Environment lighting ROI contains no pixels.")
    return total / count


def inverse_filmic(mapped: float) -> float:
    low = 0.0
    high = 64.0
    for _ in range(32):
        color = (low + high) * 0.5
        temporary = max(0.0, color - 0.004)
        filmic = temporary * (6.2 * temporary + 0.5) / (temporary * (6.2 * temporary + 1.7) + 0.06)
        if filmic < mapped:
            low = color
        else:
            high = color
    return (low + high) * 0.5


def region_linear_luminance(image: PngImage, region: tuple[float, float, float, float]) -> float:
    begin_x = int(region[0] * image.width)
    begin_y = int(region[1] * image.height)
    end_x = int(region[2] * image.width + 0.999999)
    end_y = int(region[3] * image.height + 0.999999)
    total = 0.0
    count = 0
    for y in range(begin_y, end_y):
        for x in range(begin_x, end_x):
            base = (y * image.width + x) * 4
            total += (
                inverse_filmic(image.rgba[base] / 255.0) * 0.2126
                + inverse_filmic(image.rgba[base + 1] / 255.0) * 0.7152
                + inverse_filmic(image.rgba[base + 2] / 255.0) * 0.0722
            )
            count += 1
    if count == 0:
        raise RuntimeError("Environment lighting linear ROI contains no pixels.")
    return total / count


def validate_png(
    path: Path, width: int, height: int
) -> tuple[dict[str, object], dict[str, float], dict[str, float], PngImage]:
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"Missing or empty environment lighting capture: {path}")
    image = read_image(path)
    if not isinstance(image, PngImage):
        raise RuntimeError(f"Environment lighting capture is not a PNG: {path}")
    if (image.width, image.height) != (width, height):
        raise RuntimeError(
            f"Environment lighting capture has {image.width}x{image.height}; expected {width}x{height}: {path}"
        )
    metrics = {name: region_luminance(image, region) for name, region in REGIONS.items()}
    linear_metrics = {name: region_linear_luminance(image, region) for name, region in REGIONS.items()}
    return (
        {
            "file": path.name,
            "width": image.width,
            "height": image.height,
            "bytes": path.stat().st_size,
            "sha256": sha256(path),
        },
        metrics,
        linear_metrics,
        image,
    )


def normalized_rms(lhs: PngImage, rhs: PngImage) -> float:
    if (lhs.width, lhs.height) != (rhs.width, rhs.height):
        return math.inf
    squared_error = 0.0
    squared_signal = 0.0
    for index in range(0, len(lhs.rgba), 4):
        for channel in range(3):
            a = lhs.rgba[index + channel] / 255.0
            b = rhs.rgba[index + channel] / 255.0
            squared_error += (a - b) ** 2
            squared_signal += a**2
    return math.sqrt(squared_error / squared_signal) if squared_signal else math.sqrt(squared_error)


def scalar_channel_error(image: PngImage) -> float:
    return max(
        max(abs(image.rgba[index] - image.rgba[index + 1]), abs(image.rgba[index] - image.rgba[index + 2]))
        / 255.0
        for index in range(0, len(image.rgba), 4)
    )


def maximum_channel_amplification(unoccluded: PngImage, occluded: PngImage) -> float:
    return max(
        (occluded.rgba[index + channel] - unoccluded.rgba[index + channel]) / 255.0
        for index in range(0, len(unoccluded.rgba), 4)
        for channel in range(3)
    )


def sphere_center_rim_luminance(
    image: PngImage, region: tuple[float, float, float, float]
) -> tuple[float, float]:
    begin_x = int(region[0] * image.width)
    begin_y = int(region[1] * image.height)
    end_x = int(region[2] * image.width + 0.999999)
    end_y = int(region[3] * image.height + 0.999999)
    center_total = 0.0
    center_count = 0
    rim_total = 0.0
    rim_count = 0
    center_x = (begin_x + end_x - 1) * 0.5
    center_y = (begin_y + end_y - 1) * 0.5
    radius_x = max((end_x - begin_x) * 0.5, 1.0)
    radius_y = max((end_y - begin_y) * 0.5, 1.0)
    for y in range(begin_y, end_y):
        for x in range(begin_x, end_x):
            radius = math.sqrt(((x - center_x) / radius_x) ** 2 + ((y - center_y) / radius_y) ** 2)
            if radius > 0.95:
                continue
            base = (y * image.width + x) * 4
            value = (
                image.rgba[base] * 0.2126
                + image.rgba[base + 1] * 0.7152
                + image.rgba[base + 2] * 0.0722
            ) / 255.0
            if radius <= 0.45:
                center_total += value
                center_count += 1
            elif radius >= 0.65:
                rim_total += value
                rim_count += 1
    if center_count == 0 or rim_count == 0:
        raise RuntimeError("Sponza sphere center/rim ROI contains no pixels.")
    return center_total / center_count, rim_total / rim_count


def validate_m15_image_matrix(
    images: dict[str, PngImage], metrics: dict[str, dict[str, float]]
) -> dict[str, object]:
    unoccluded = images["sponza-unoccluded"]
    occluded = images["sponza-gtao-occluded"]
    deltas = {
        "gtao_specular_nrmse": normalized_rms(unoccluded, occluded),
        "probe_intensity_diffuse_nrmse": normalized_rms(
            images["sponza-gtao-diffuse"], images["sponza-gtao-diffuse-probe-double"]
        ),
        "ssao_specular_nrmse": normalized_rms(unoccluded, images["sponza-ssao-occluded"]),
        "indirect_intensity_specular_nrmse": normalized_rms(
            occluded, images["sponza-gtao-occluded-indirect-double"]
        ),
        "ddgi_disabled_specular_nrmse": normalized_rms(
            occluded, images["sponza-gtao-occluded-ddgi-disabled"]
        ),
        "ddgi_outside_specular_nrmse": normalized_rms(
            occluded, images["sponza-gtao-occluded-ddgi-outside"]
        ),
        "left_right_room_nrmse": normalized_rms(
            images["sponza-gtao-left-room"], images["sponza-gtao-right-room"]
        ),
        "scalar_channel_error": scalar_channel_error(images["sponza-gtao-visibility"]),
        "maximum_amplification": maximum_channel_amplification(unoccluded, occluded),
    }
    sphere_retention = {}
    for region_name in ("sponza_rough_metal", "sponza_rough_dielectric"):
        unoccluded_center, unoccluded_rim = sphere_center_rim_luminance(unoccluded, REGIONS[region_name])
        occluded_center, occluded_rim = sphere_center_rim_luminance(occluded, REGIONS[region_name])
        sphere_retention[region_name] = {
            "center": occluded_center / max(unoccluded_center, 1.0e-6),
            "rim": occluded_rim / max(unoccluded_rim, 1.0e-6),
        }
    checks = {
        "exact_metal_is_lit": metrics["sponza-gtao-beauty"]["sponza_exact_metal"] > 0.02,
        "gtao_visibility_is_scalar": deltas["scalar_channel_error"] == 0.0,
        "gtao_visibility_is_nontrivial": deltas["gtao_specular_nrmse"] > 0.000001,
        "visibility_does_not_amplify": deltas["maximum_amplification"] <= 1.0 / 255.0,
        "rough_metal_is_suppressed": metrics["sponza-gtao-occluded"]["sponza_rough_metal"]
        < metrics["sponza-unoccluded"]["sponza_rough_metal"] * 0.99,
        "rough_dielectric_is_suppressed": metrics["sponza-gtao-occluded"]["sponza_rough_dielectric"]
        < metrics["sponza-unoccluded"]["sponza_rough_dielectric"] * 0.99,
        "smooth_metal_is_retained": metrics["sponza-gtao-occluded"]["sponza_exact_metal"]
        > metrics["sponza-unoccluded"]["sponza_exact_metal"] * 0.8,
        "smooth_dielectric_is_retained": metrics["sponza-gtao-occluded"]["sponza_smooth_dielectric"]
        > metrics["sponza-unoccluded"]["sponza_smooth_dielectric"] * 0.8,
        "rough_grazing_response_is_retained": all(
            value["rim"] + 0.03 >= value["center"] for value in sphere_retention.values()
        ),
        "probe_intensity_is_diffuse_invariant": deltas["probe_intensity_diffuse_nrmse"] < 1.0e-5,
        "ssao_is_diffuse_only": deltas["ssao_specular_nrmse"] < 1.0e-5,
        "indirect_intensity_does_not_scale_specular": deltas["indirect_intensity_specular_nrmse"] < 1.0e-5,
        "ddgi_disabled_does_not_change_specular": deltas["ddgi_disabled_specular_nrmse"] < 1.0e-5,
        "ddgi_outside_does_not_change_specular": deltas["ddgi_outside_specular_nrmse"] < 1.0e-5,
        "adjacent_rooms_are_nonblack_and_distinct": metrics["sponza-gtao-left-room"]["sponza_room"] > 0.0001
        and metrics["sponza-gtao-right-room"]["sponza_room"] > 0.0001
        and deltas["left_right_room_nrmse"] > 0.0001,
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise RuntimeError("Independent M15 Sponza checks failed: " + ", ".join(failed))
    return {"deltas": deltas, "sphere_retention": sphere_retention, "checks": checks}


def validate_image_matrix(
    metrics: dict[str, dict[str, float]], linear_metrics: dict[str, dict[str, float]]
) -> None:
    default = metrics["default"]
    indirect_off = metrics["indirect-off"]
    all_off = metrics["all-off"]
    sky_off = metrics["sky-off"]
    background = metrics["background-only"]
    background_lit = metrics["background-lit"]
    sky_double = metrics["sky-double"]
    indirect_double = metrics["indirect-double"]
    sky_double_indirect_off = metrics["sky-double-indirect-off"]
    both_double = metrics["both-double"]
    ray_neutral = metrics["ray-neutral-reference"]
    ray_sky_double = metrics["ray-sky-double"]
    ray_background_lit = metrics["ray-background-lit"]
    ray_indirect_off = metrics["ray-indirect-off"]
    ray_indirect_double = metrics["ray-indirect-double"]
    ray_sky_off = metrics["ray-sky-off"]
    ray_all_off = metrics["ray-all-off"]
    sponza = metrics["sponza-default"]
    near_black = lambda value: value < 0.02
    equal = lambda lhs, rhs: abs(lhs - rhs) <= max(0.02, max(lhs, rhs) * 0.12)
    ray_equal = lambda lhs, rhs: abs(lhs - rhs) <= max(0.04, max(lhs, rhs) * 0.20)
    scales_twice = lambda off, once, twice: once - off > 0.002 and abs((twice - off) - 2.0 * (once - off)) <= max(
        0.01, 0.5 * (once - off)
    )
    raster_names = RASTER_CONTROL_NAMES
    checks = {
        "background_is_camera_only": background["background"] > all_off["background"] + 0.01
        and equal(background["background"], background_lit["background"])
        and equal(background["dielectric"], all_off["dielectric"])
        and equal(background["metal"], all_off["metal"])
        and equal(background_lit["dielectric"], both_double["dielectric"])
        and equal(background_lit["metal"], both_double["metal"])
        and equal(background_lit["ibl_diffuse"], both_double["ibl_diffuse"])
        and ray_background_lit["background"] > ray_neutral["background"] + 0.01
        and ray_equal(ray_background_lit["dielectric"], ray_neutral["dielectric"])
        and ray_equal(ray_background_lit["metal"], ray_neutral["metal"])
        and ray_equal(ray_background_lit["ibl_diffuse"], ray_neutral["ibl_diffuse"])
        and all(
            near_black(metrics[name]["background"])
            for name in SYNTHETIC_CAPTURE_NAMES
            if name not in ("background-only", "background-lit", "ray-background-lit")
        ),
        "sky_scale_owns_global_specular": indirect_off["metal"] > all_off["metal"] + 0.01
        and sky_double_indirect_off["metal"] > indirect_off["metal"] + 0.01,
        "sky_scale_owns_diffuse_source": default["dielectric"] > sky_off["dielectric"] + 0.01
        and default["ibl_diffuse"] > sky_off["ibl_diffuse"] + 0.01,
        "indirect_owns_diffuse": default["dielectric"] > indirect_off["dielectric"] + 0.01
        and default["ibl_diffuse"] > indirect_off["ibl_diffuse"] + 0.01
        and indirect_double["ibl_diffuse"] > default["ibl_diffuse"] + 0.01
        and sky_double["ibl_diffuse"] > sky_double_indirect_off["ibl_diffuse"] + 0.01
        and both_double["ibl_diffuse"] > sky_double["ibl_diffuse"] + 0.01,
        "sky_scale_is_applied_once": scales_twice(
            linear_metrics["all-off"]["metal"],
            linear_metrics["indirect-off"]["metal"],
            linear_metrics["sky-double-indirect-off"]["metal"],
        ),
        "sky_scale_is_applied_once_to_diffuse": scales_twice(
            linear_metrics["sky-off"]["ibl_diffuse"],
            linear_metrics["default"]["ibl_diffuse"],
            linear_metrics["sky-double"]["ibl_diffuse"],
        ),
        "indirect_intensity_is_applied_once": scales_twice(
            linear_metrics["indirect-off"]["ibl_diffuse"],
            linear_metrics["default"]["ibl_diffuse"],
            linear_metrics["indirect-double"]["ibl_diffuse"],
        ),
        "ray_sky_scale_is_applied_once": scales_twice(
            linear_metrics["ray-sky-off"]["metal"],
            linear_metrics["ray-neutral-reference"]["metal"],
            linear_metrics["ray-sky-double"]["metal"],
        ),
        "ray_sky_scale_is_applied_once_to_diffuse": scales_twice(
            linear_metrics["ray-sky-off"]["ibl_diffuse"],
            linear_metrics["ray-neutral-reference"]["ibl_diffuse"],
            linear_metrics["ray-sky-double"]["ibl_diffuse"],
        ),
        "ray_indirect_intensity_is_applied_once": scales_twice(
            linear_metrics["ray-indirect-off"]["ibl_diffuse"],
            linear_metrics["ray-neutral-reference"]["ibl_diffuse"],
            linear_metrics["ray-indirect-double"]["ibl_diffuse"],
        ),
        "metal_is_indirect_invariant": equal(indirect_off["metal"], default["metal"])
        and equal(default["metal"], indirect_double["metal"])
        and equal(sky_double_indirect_off["metal"], sky_double["metal"])
        and equal(sky_double["metal"], both_double["metal"]),
        "direct_is_invariant": all_off["direct"] > 0.02
        and equal(all_off["direct"], sky_off["direct"])
        and equal(all_off["direct"], indirect_off["direct"])
        and equal(all_off["direct"], metrics["sky-off-indirect-double"]["direct"])
        and equal(all_off["direct"], sky_double_indirect_off["direct"]),
        "emission_is_invariant": default["emission"] > 0.02
        and all(equal(default["emission"], metrics[name]["emission"]) for name in raster_names[1:]),
        "ray_ownership": ray_all_off["direct"] > 0.01
        and ray_neutral["emission"] > 0.01
        and ray_neutral["dielectric"] > ray_indirect_off["dielectric"] + 0.01
        and ray_equal(ray_neutral["metal"], ray_indirect_off["metal"])
        and ray_equal(ray_neutral["metal"], ray_indirect_double["metal"])
        and ray_neutral["metal"] > ray_sky_off["metal"] + 0.01
        and ray_equal(ray_all_off["direct"], ray_indirect_off["direct"])
        and ray_equal(ray_all_off["direct"], ray_indirect_double["direct"])
        and ray_equal(ray_all_off["direct"], ray_sky_double["direct"])
        and ray_equal(ray_all_off["direct"], ray_sky_off["direct"])
        and ray_equal(ray_neutral["emission"], ray_indirect_off["emission"])
        and ray_equal(ray_neutral["emission"], ray_indirect_double["emission"])
        and ray_equal(ray_neutral["emission"], ray_sky_double["emission"])
        and ray_equal(ray_neutral["emission"], ray_sky_off["emission"])
        and ray_equal(ray_neutral["emission"], ray_all_off["emission"]),
        "sponza_exact_metal_is_lit": sponza["sponza_exact_metal"] > 0.02,
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise RuntimeError("Independent environment lighting image checks failed: " + ", ".join(failed))


def main() -> int:
    args = parse_args()
    try:
        if (args.width, args.height) != (1920, 1080):
            raise ValueError("The M15 environment lighting gate requires exactly 1920x1080.")
        if args.timeout <= 0:
            raise ValueError("--timeout must be positive.")

        root = repo_root()
        editor = (args.editor or root / "out/install/vs2026-x64/bin/EvoEngineEditor.exe").resolve()
        if not editor.is_file():
            raise FileNotFoundError(f"Installed editor not found: {editor}")
        output_dir = args.output_dir if args.output_dir.is_absolute() else root / args.output_dir
        output_dir = output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        for name in (*[f"{name}.png" for name in CAPTURE_NAMES], "report.json", "run.log", "evidence.json"):
            (output_dir / name).unlink(missing_ok=True)

        shader_cache = output_dir / "shader-cache"
        shader_cache.mkdir(parents=True, exist_ok=True)
        imgui_path = output_dir / "imgui.ini"
        imgui_path.unlink(missing_ok=True)
        environment = os.environ.copy()
        environment["EVOENGINE_ENVIRONMENT_LIGHTING_EVIDENCE"] = str(output_dir)
        environment["EVOENGINE_SHADER_CACHE_DIR"] = str(shader_cache)
        environment["EVOENGINE_IMGUI_INI_PATH"] = str(imgui_path)

        command = [
            str(editor),
            "--demo",
            "rendering-regression",
            "--editor",
            "--preview-width",
            str(args.width),
            "--preview-height",
            str(args.height),
        ]
        print(f"Environment lighting validation ({args.config}): {' '.join(command)}")
        try:
            completed = subprocess.run(
                command,
                cwd=editor.parent,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=args.timeout,
                check=False,
            )
            output = completed.stdout
        except subprocess.TimeoutExpired as error:
            output = error.stdout or ""
            if isinstance(output, bytes):
                output = output.decode("utf-8", errors="replace")
            (output_dir / "run.log").write_text(output, encoding="utf-8")
            raise TimeoutError(f"Installed editor exceeded the {args.timeout:g}-second timeout.") from error

        log_path = output_dir / "run.log"
        log_path.write_text(output, encoding="utf-8")
        print(output, end="")
        if completed.returncode != 0:
            raise RuntimeError(f"Installed editor exited with code {completed.returncode}; log={log_path}")
        fatal_line = next(
            (line for line in output.splitlines() if any(pattern.search(line) for pattern in FATAL_PATTERNS)), None
        )
        if fatal_line:
            raise RuntimeError(f"Fatal runtime output: {fatal_line}")
        for marker in (
            "EVOENGINE_VULKAN_VALIDATION enabled",
            "EVOENGINE_VULKAN_SYNCHRONIZATION_VALIDATION enabled",
            "EVOENGINE_ENVIRONMENT_LIGHTING_REPORT",
            "passed=true",
            "EVOENGINE_ENVIRONMENT_LIGHTING_SHUTDOWN_COMPLETE",
        ):
            if marker not in output:
                raise RuntimeError(f"Missing runtime marker {marker!r}; log={log_path}")
        reference_marker = (
            "EVOENGINE_DDGI_REFERENCE fixture=sponza render_mode=RayTracing resolution=1920x1080 "
            "frames=64 spp_per_frame=4 total_spp=256 output=.png"
        )
        reference_markers = [line.strip() for line in output.splitlines() if line.startswith("EVOENGINE_DDGI_REFERENCE")]
        if reference_markers != [reference_marker]:
            raise RuntimeError(f"Environment lighting reference marker list is wrong: {reference_markers!r}")

        report_path = output_dir / "report.json"
        if not report_path.is_file():
            raise RuntimeError(f"Environment lighting report was not written: {report_path}")
        report = json.loads(report_path.read_text(encoding="utf-8"))
        contract = report.get("contract", {})
        if report.get("schema_version") != 3 or report.get("passed") is not True:
            raise RuntimeError("Environment lighting report did not pass its schema contract.")
        expected_contract = {
            "resolution": [1920, 1080],
            "render_modes": ["Rasterization", "RayTracing"],
            "graphics_validation": True,
            "launch_count": 1,
            "raster_settle_frames": 4,
            "ray_settle_frames": 48,
            "screen_space_reflections": False,
            "ray_traced_reflections": False,
            "metallic_values": [0.0, 1.0],
            "physical_reference": "ray-neutral-reference",
            "specular_visibility_model": "1-roughness^2*mix(0.04*tanh((1-min(material_ao,gtao))/0.04),1-min(material_ao,gtao),smoothstep(0.8,1,NdotV))",
            "sponza_local_probe_count": 5,
            "canonical_sponza": {
                "fixture_id": "sponza",
                "deterministic_seed": 1831565813,
                "spheres_enabled": False,
                "title_enabled": False,
                "raster": {
                    "render_mode": "Rasterization",
                    "ddgi_enabled": True,
                    "samples_per_frame": 1,
                    "warmup_frames": 64,
                    "max_convergence_frames": 1024,
                    "preparation_frames": 8,
                    "measure_frames": 120,
                    "image": "sponza-repeatability-anchor.png",
                },
                "ray": {
                    "render_mode": "RayTracing",
                    "ddgi_enabled": False,
                    "frames": 64,
                    "samples_per_frame": 4,
                    "total_spp": 256,
                    "debug_view": "Beauty",
                    "ser": "disabled",
                    "firefly_clamp": False,
                    "emissive_nee": True,
                    "auto_spp": False,
                    "image": "sponza-ray-reference.png",
                },
            },
        }
        if contract != expected_contract:
            raise RuntimeError("Environment lighting report has the wrong fixed validation contract.")
        checks = report.get("checks")
        if not isinstance(checks, dict) or not checks or any(value is not True for value in checks.values()):
            failed = sorted(name for name, value in (checks or {}).items() if value is not True)
            raise RuntimeError("Environment lighting runtime checks failed: " + ", ".join(failed))
        captures = report.get("captures")
        if not isinstance(captures, list) or [capture.get("name") for capture in captures] != list(CAPTURE_NAMES):
            raise RuntimeError("Environment lighting report does not contain the ordered M15 capture matrix.")
        expected_controls = {
            "default": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "indirect-off": {"background": 0.0, "sky": 1.0, "indirect": 0.0},
            "all-off": {"background": 0.0, "sky": 0.0, "indirect": 0.0},
            "sky-off": {"background": 0.0, "sky": 0.0, "indirect": 1.0},
            "background-only": {"background": 1.0, "sky": 0.0, "indirect": 0.0},
            "background-lit": {"background": 1.0, "sky": 2.0, "indirect": 2.0},
            "sky-double": {"background": 0.0, "sky": 2.0, "indirect": 1.0},
            "indirect-double": {"background": 0.0, "sky": 1.0, "indirect": 2.0},
            "sky-off-indirect-double": {"background": 0.0, "sky": 0.0, "indirect": 2.0},
            "sky-double-indirect-off": {"background": 0.0, "sky": 2.0, "indirect": 0.0},
            "both-double": {"background": 0.0, "sky": 2.0, "indirect": 2.0},
            "ray-neutral-reference": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "ray-sky-double": {"background": 0.0, "sky": 2.0, "indirect": 1.0},
            "ray-background-lit": {"background": 1.0, "sky": 1.0, "indirect": 1.0},
            "ray-indirect-off": {"background": 0.0, "sky": 1.0, "indirect": 0.0},
            "ray-indirect-double": {"background": 0.0, "sky": 1.0, "indirect": 2.0},
            "ray-sky-off": {"background": 0.0, "sky": 0.0, "indirect": 1.0},
            "ray-all-off": {"background": 0.0, "sky": 0.0, "indirect": 0.0},
            "sponza-default": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-diffuse": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-unoccluded": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-gtao-beauty": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-gtao-diffuse": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-gtao-diffuse-probe-double": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-gtao-visibility": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-gtao-occluded": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-ssao-diffuse": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-ssao-occluded": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-gtao-occluded-indirect-double": {"background": 0.0, "sky": 1.0, "indirect": 2.0},
            "sponza-gtao-occluded-ddgi-disabled": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-gtao-occluded-ddgi-outside": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-gtao-left-room": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-gtao-right-room": {"background": 0.0, "sky": 1.0, "indirect": 1.0},
            "sponza-repeatability-anchor": {"background": 1.0, "sky": 1.0, "indirect": 1.0},
            "sponza-ray-reference": {"background": 1.0, "sky": 1.0, "indirect": 1.0},
        }
        expected_modes = dict.fromkeys(
            (*RASTER_CONTROL_NAMES, "sponza-default", *SPONZA_M15_CAPTURE_NAMES, "sponza-repeatability-anchor"),
            "Rasterization",
        )
        expected_modes.update(dict.fromkeys((*RAY_CONTROL_NAMES, "sponza-ray-reference"), "RayTracing"))
        expected_settle = dict.fromkeys(
            (*RASTER_CONTROL_NAMES, "sponza-default", *SPONZA_M15_CAPTURE_NAMES), 4
        )
        expected_settle.update(dict.fromkeys(RAY_CONTROL_NAMES, 48))
        expected_settle["sponza-repeatability-anchor"] = 0
        expected_settle["sponza-ray-reference"] = 64
        expected_m15_debug = {
            "sponza-diffuse": (1, "disabled"),
            "sponza-unoccluded": (2, "disabled"),
            "sponza-gtao-beauty": (0, "gtao"),
            "sponza-gtao-diffuse": (1, "gtao"),
            "sponza-gtao-diffuse-probe-double": (1, "gtao"),
            "sponza-gtao-visibility": (3, "gtao"),
            "sponza-gtao-occluded": (4, "gtao"),
            "sponza-ssao-diffuse": (1, "ssao"),
            "sponza-ssao-occluded": (4, "ssao"),
            "sponza-gtao-occluded-indirect-double": (4, "gtao"),
            "sponza-gtao-occluded-ddgi-disabled": (4, "gtao"),
            "sponza-gtao-occluded-ddgi-outside": (4, "gtao"),
            "sponza-gtao-left-room": (4, "gtao"),
            "sponza-gtao-right-room": (4, "gtao"),
        }
        report_captures = {capture["name"]: capture for capture in captures}
        for name, capture in report_captures.items():
            if capture.get("controls") != expected_controls[name]:
                raise RuntimeError(f"Environment lighting controls are wrong for {name!r}.")
            if capture.get("render_mode") != expected_modes[name] or capture.get("settle_frames") != expected_settle[name]:
                raise RuntimeError(f"Environment lighting render contract is wrong for {name!r}.")
            if capture.get("image") != f"{name}.png" or capture.get("finite") is not True:
                raise RuntimeError(f"Environment lighting artifact metadata is wrong for {name!r}.")
            report_regions = capture.get("regions")
            if not isinstance(report_regions, dict) or set(report_regions) != set(REGIONS):
                raise RuntimeError(f"Environment lighting report regions are wrong for {name!r}.")
        for name, (debug_view, ambient_occlusion) in expected_m15_debug.items():
            capture = report_captures[name]
            if (
                capture.get("indirect_debug_view") != debug_view
                or capture.get("ambient_occlusion") != ambient_occlusion
                or capture.get("local_probe_count") != 5
                or capture.get("valid_local_probe_count") != 5
                or not math.isclose(
                    capture.get("local_probe_intensity", math.nan),
                    2.0 if name == "sponza-gtao-diffuse-probe-double" else 1.0,
                    abs_tol=1.0e-6,
                )
                or capture.get("ddgi_enabled") is not (name != "sponza-gtao-occluded-ddgi-disabled")
            ):
                raise RuntimeError(f"Environment lighting M15 metadata is wrong for {name!r}: {capture!r}.")

        artifacts: list[dict[str, object]] = []
        image_metrics: dict[str, dict[str, float]] = {}
        linear_image_metrics: dict[str, dict[str, float]] = {}
        images: dict[str, PngImage] = {}
        for name in CAPTURE_NAMES:
            artifact, metrics, linear_metrics, image = validate_png(
                output_dir / f"{name}.png", args.width, args.height
            )
            artifacts.append(artifact)
            image_metrics[name] = metrics
            linear_image_metrics[name] = linear_metrics
            images[name] = image
            for region_name, region in REGIONS.items():
                begin_x = int(region[0] * args.width)
                begin_y = int(region[1] * args.height)
                end_x = int(region[2] * args.width + 0.999999)
                end_y = int(region[3] * args.height + 0.999999)
                report_region = report_captures[name]["regions"][region_name]
                if report_region.get("sample_count") != (end_x - begin_x) * (end_y - begin_y):
                    raise RuntimeError(f"Environment lighting sample count is wrong for {name!r}/{region_name!r}.")
                reported_luminance = report_region.get("average_luminance")
                if (
                    not isinstance(reported_luminance, (int, float))
                    or not math.isfinite(reported_luminance)
                    or abs(reported_luminance - metrics[region_name]) > 0.01
                ):
                    raise RuntimeError(f"Environment lighting report/PNG ROI mismatch for {name!r}/{region_name!r}.")
                reported_linear_luminance = report_region.get("linear_luminance")
                if (
                    not isinstance(reported_linear_luminance, (int, float))
                    or not math.isfinite(reported_linear_luminance)
                    or abs(reported_linear_luminance - linear_metrics[region_name])
                    > max(0.05, linear_metrics[region_name] * 0.2)
                ):
                    raise RuntimeError(
                        f"Environment lighting report/PNG linear ROI mismatch for {name!r}/{region_name!r}."
                    )
        validate_image_matrix(image_metrics, linear_image_metrics)
        m15_image_evidence = validate_m15_image_matrix(images, image_metrics)
        canonical_images = {name: images[name] for name in ("sponza-repeatability-anchor", "sponza-ray-reference")}
        if any(region_luminance(image, (0.0, 0.0, 1.0, 1.0)) <= 0.02 for image in canonical_images.values()):
            raise RuntimeError("Canonical Sponza raster or ray artifact is black.")
        sponza_quality = compare_render_images(
            canonical_images["sponza-ray-reference"],
            canonical_images["sponza-repeatability-anchor"],
            ignore_alpha=True,
        )
        if sponza_quality["normalized_rms_error"] > SPONZA_QUALITY_ERROR_MAX:
            raise RuntimeError(
                "Canonical Sponza raster anchor exceeds the frozen ray-reference quality limit: "
                f"{sponza_quality['normalized_rms_error']} > {SPONZA_QUALITY_ERROR_MAX}"
            )
        rough_specular = report.get("rough_specular", {})
        expected_rough_keys = {
            "gtao_specular_nrmse",
            "gtao_diffuse_nrmse",
            "ssao_diffuse_nrmse",
            "ssao_specular_nrmse",
            "probe_intensity_diffuse_nrmse",
            "indirect_intensity_specular_nrmse",
            "ddgi_disabled_specular_nrmse",
            "ddgi_outside_specular_nrmse",
            "scalar_channel_error",
            "maximum_amplification",
            "gtao_role",
            "ddgi_role",
        }
        if (
            set(rough_specular) != expected_rough_keys
            or rough_specular.get("gtao_role") != "scalar_visibility_only"
            or rough_specular.get("ddgi_role") != "none"
            or any(
                not isinstance(rough_specular.get(name), (int, float))
                or not math.isfinite(rough_specular[name])
                for name in expected_rough_keys - {"gtao_role", "ddgi_role"}
            )
        ):
            raise RuntimeError("Environment lighting rough-specular report is malformed.")
        invalidation = report.get("ddgi_invalidation", {})
        indirect_edit = invalidation.get("indirect_edit", {})
        sky_edit = invalidation.get("sky_edit", {})
        if (
            set(indirect_edit) != {"update_reasons", "recorded_probe_updates", "recorded_ray_samples"}
            or not isinstance(indirect_edit["update_reasons"], int)
            or indirect_edit["update_reasons"] & 35
            or not isinstance(indirect_edit["recorded_probe_updates"], int)
            or indirect_edit["recorded_probe_updates"] != 0
            or not isinstance(indirect_edit["recorded_ray_samples"], int)
            or indirect_edit["recorded_ray_samples"] != 0
            or set(sky_edit) != {"update_reasons", "recorded_probe_updates", "recorded_ray_samples"}
            or not isinstance(sky_edit["update_reasons"], int)
            or not sky_edit["update_reasons"] & 32
            or not isinstance(sky_edit["recorded_probe_updates"], int)
            or sky_edit["recorded_probe_updates"] <= 0
            or not isinstance(sky_edit["recorded_ray_samples"], int)
            or sky_edit["recorded_ray_samples"] <= 0
            or not isinstance(invalidation.get("canonical_sponza_convergence_frames"), int)
            or invalidation["canonical_sponza_convergence_frames"] <= 0
            or invalidation["canonical_sponza_convergence_frames"] > 1024
            or not isinstance(invalidation.get("m15_sponza_convergence_frames"), int)
            or invalidation["m15_sponza_convergence_frames"] <= 0
            or invalidation["m15_sponza_convergence_frames"] > 1024
        ):
            raise RuntimeError("Environment lighting DDGI invalidation evidence is malformed.")
        artifacts.extend(
            (
                {"file": report_path.name, "bytes": report_path.stat().st_size, "sha256": sha256(report_path)},
                {"file": log_path.name, "bytes": log_path.stat().st_size, "sha256": sha256(log_path)},
            )
        )
        artifact_by_file = {artifact["file"]: artifact for artifact in artifacts}
        evidence = {
            "schema_version": 3,
            "configuration": args.config,
            "launch_count": 1,
            "command": command,
            "editor": str(editor),
            "editor_sha256": sha256(editor),
            "checks": checks,
            "ddgi_invalidation": invalidation,
            "rough_specular": rough_specular,
            "m15_image_evidence": m15_image_evidence,
            "canonical_sponza": {
                "quality_metric": "normalized_rms_error",
                "quality_error": sponza_quality["normalized_rms_error"],
                "quality_error_max": SPONZA_QUALITY_ERROR_MAX,
                "repeatability_anchor": artifact_by_file["sponza-repeatability-anchor.png"],
                "quality_reference": artifact_by_file["sponza-ray-reference.png"],
                "reference_log": artifact_by_file[log_path.name],
            },
            "image_roi_luminance": image_metrics,
            "image_roi_linear_luminance": linear_image_metrics,
            "artifacts": artifacts,
            "passed": True,
        }
        evidence_path = output_dir / "evidence.json"
        evidence_path.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
        print(f"Environment lighting validation passed in one launch; evidence={evidence_path}")
        return 0
    except Exception as error:
        print(f"Environment lighting validation failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
