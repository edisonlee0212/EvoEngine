#!/usr/bin/env python3
"""Validate spatial reflection probes in one installed-editor launch."""

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

from compare_reference_render import PngImage, read_image


SPECULAR_VISIBILITY_MODEL = (
    "1-roughness^2*mix(0.04*tanh((1-min(material_ao,gtao))/0.04),"
    "1-min(material_ao,gtao),smoothstep(0.8,1,NdotV))"
)
CAPTURE_NAMES = (
    "baseline",
    "sky-off-local",
    "box-unprojected",
    "owner-order-reversed",
    "camera-moved",
    "missing-asset",
    "removed",
    "m15-unoccluded",
    "m15-visibility-off",
    "m15-occluded-off",
    "m15-visibility-material",
    "m15-occluded-material",
    "m15-visibility-gtao",
    "m15-occluded-gtao",
    "m15-visibility-material-gtao",
    "m15-occluded-material-gtao",
    "m15-visibility-unavailable",
    "m15-occluded-unavailable",
    "m15-occluded-ssao",
    "m15-occluded-indirect-double",
    "m15-boundary-left",
    "m15-boundary-center",
    "m15-boundary-right",
    "m15-camera-moved-occluded",
    "debug-bounds-off",
    "debug-bounds-on",
)
REGIONS = {
    "left": (0.29, 0.38, 0.41, 0.60),
    "center": (0.46, 0.42, 0.53, 0.58),
    "boundary": (0.53, 0.43, 0.58, 0.57),
    "right": (0.59, 0.38, 0.71, 0.60),
    "smooth": (0.40, 0.24, 0.48, 0.39),
    "rough": (0.52, 0.24, 0.60, 0.39),
    "fallback": (0.46, 0.61, 0.54, 0.76),
}
FATAL_PATTERNS = (
    re.compile(r"Vulkan\s+\[Validation\]-\[Error\]", re.IGNORECASE),
    re.compile(r"\bVK_ERROR_DEVICE_LOST\b", re.IGNORECASE),
    re.compile(r"\bdevice\s+lost\b", re.IGNORECASE),
    re.compile(r"\bunhandled\s+exception\b", re.IGNORECASE),
    re.compile(r"\bfatal(?:\s+error)?\b", re.IGNORECASE),
    re.compile(r"\bassertion\s+failed\b", re.IGNORECASE),
    re.compile(r"EVOENGINE_REFLECTION_PROBE_ERROR", re.IGNORECASE),
)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="RelWithDebInfo", help="Installed build configuration label.")
    parser.add_argument("--width", type=int, default=1920, help="Render width; must be 1920.")
    parser.add_argument("--height", type=int, default=1080, help="Render height; must be 1080.")
    parser.add_argument("--output-dir", type=Path, default=root / "out/reflection-probe-validation-m15")
    parser.add_argument("--timeout", type=float, default=1200.0, help="Maximum launch duration in seconds.")
    parser.add_argument("--editor", type=Path, help="Installed EvoEngineEditor executable override.")
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_png(path: Path, width: int, height: int) -> tuple[dict[str, object], PngImage]:
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"Missing or empty reflection probe capture: {path}")
    image = read_image(path)
    if not isinstance(image, PngImage):
        raise RuntimeError(f"Reflection probe capture is not a PNG: {path}")
    if (image.width, image.height) != (width, height):
        raise RuntimeError(
            f"Reflection probe capture has {image.width}x{image.height}; expected {width}x{height}: {path}"
        )
    return (
        {
            "file": path.name,
            "width": image.width,
            "height": image.height,
            "bytes": path.stat().st_size,
            "sha256": sha256(path),
        },
        image,
    )


def region_average(image: PngImage, region: tuple[float, float, float, float]) -> tuple[float, float, float]:
    begin_x = int(region[0] * image.width)
    begin_y = int(region[1] * image.height)
    end_x = math.ceil(region[2] * image.width)
    end_y = math.ceil(region[3] * image.height)
    total = [0.0, 0.0, 0.0]
    count = 0
    for y in range(begin_y, end_y):
        for x in range(begin_x, end_x):
            base = (y * image.width + x) * 4
            for channel in range(3):
                total[channel] += image.rgba[base + channel] / 255.0
            count += 1
    if count == 0:
        raise RuntimeError("Reflection probe validation ROI contains no pixels.")
    return tuple(value / count for value in total)


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


def percentile(samples: list[float], fraction: float) -> float:
    ordered = sorted(samples)
    rank = min(max(fraction, 0.0), 1.0) * (len(ordered) - 1)
    lower = math.floor(rank)
    upper = math.ceil(rank)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (rank - lower)


def relative_mad(samples: list[float]) -> float:
    median = percentile(samples, 0.5)
    absolute_deviations = [abs(value - median) for value in samples]
    return 1.4826 * percentile(absolute_deviations, 0.5) / max(abs(median), 0.01)


def dominant(color: tuple[float, float, float], channel: int) -> bool:
    selected = color[channel]
    return selected > 0.005 and all(selected > value * 1.2 for index, value in enumerate(color) if index != channel)


def validate_images(images: dict[str, PngImage]) -> dict[str, object]:
    baseline = images["baseline"]
    metrics = {
        name: {region: region_average(image, bounds) for region, bounds in REGIONS.items()}
        for name, image in images.items()
    }
    deltas = {
        "box_projection_nrmse": normalized_rms(baseline, images["box-unprojected"]),
        "owner_reorder_nrmse": normalized_rms(baseline, images["owner-order-reversed"]),
        "missing_removal_nrmse": normalized_rms(images["missing-asset"], images["removed"]),
        "disabled_bypass_nrmse": normalized_rms(images["m15-unoccluded"], images["m15-occluded-off"]),
        "material_ao_nrmse": normalized_rms(images["m15-unoccluded"], images["m15-occluded-material"]),
        "gtao_nrmse": normalized_rms(images["m15-unoccluded"], images["m15-occluded-gtao"]),
        "combined_nrmse": normalized_rms(images["m15-unoccluded"], images["m15-occluded-material-gtao"]),
        "unavailable_bypass_nrmse": normalized_rms(
            images["m15-unoccluded"], images["m15-occluded-unavailable"]
        ),
        "unavailable_visibility_nrmse": normalized_rms(
            images["m15-visibility-off"], images["m15-visibility-unavailable"]
        ),
        "ssao_specular_nrmse": normalized_rms(images["m15-unoccluded"], images["m15-occluded-ssao"]),
        "indirect_intensity_nrmse": normalized_rms(
            images["m15-occluded-material"], images["m15-occluded-indirect-double"]
        ),
        "debug_bounds_nrmse": normalized_rms(images["debug-bounds-off"], images["debug-bounds-on"]),
    }
    debug_off = images["debug-bounds-off"]
    debug_on = images["debug-bounds-on"]
    debug_bounds_changed_fraction = sum(
        max(abs(debug_off.rgba[index + channel] - debug_on.rgba[index + channel]) for channel in range(3)) > 8
        for index in range(0, len(debug_off.rgba), 4)
    ) / (debug_off.width * debug_off.height)
    scalar_channel_error = max(
        max(
            abs(image.rgba[index] - image.rgba[index + 1]),
            abs(image.rgba[index] - image.rgba[index + 2]),
        )
        / 255.0
        for name in (
            "m15-visibility-off",
            "m15-visibility-material",
            "m15-visibility-gtao",
            "m15-visibility-material-gtao",
            "m15-visibility-unavailable",
        )
        for image in (images[name],)
        for index in range(0, len(image.rgba), 4)
    )
    maximum_amplification = max(
        (images["m15-occluded-material-gtao"].rgba[index + channel]
         - images["m15-unoccluded"].rgba[index + channel])
        / 255.0
        for index in range(0, len(images["m15-unoccluded"].rgba), 4)
        for channel in range(3)
    )
    deltas["scalar_channel_error"] = scalar_channel_error
    deltas["maximum_amplification"] = maximum_amplification
    fallback = metrics["missing-asset"]["fallback"]
    luminance = lambda color: 0.2126 * color[0] + 0.7152 * color[1] + 0.0722 * color[2]
    boundary_luminances = [
        luminance(metrics[name]["boundary"])
        for name in ("m15-boundary-left", "m15-boundary-center", "m15-boundary-right")
    ]
    checks = {
        "left_is_red": dominant(metrics["baseline"]["left"], 0),
        "center_is_green": dominant(metrics["baseline"]["center"], 1),
        "camera_move_keeps_green": dominant(metrics["camera-moved"]["center"], 1),
        "right_is_blue": dominant(metrics["baseline"]["right"], 2),
        "local_probes_are_sky_scale_invariant": all(
            abs(metrics["baseline"][region][channel] - metrics["sky-off-local"][region][channel])
            <= max(
                0.02,
                max(metrics["baseline"][region][channel], metrics["sky-off-local"][region][channel]) * 0.12,
            )
            for region, channel in (("left", 0), ("center", 1), ("right", 2))
        ),
        "roughness_changes_lod": sum(metrics["baseline"]["smooth"])
        > sum(metrics["baseline"]["rough"]) * 1.15,
        "box_projection_changes_image": deltas["box_projection_nrmse"] > 0.00005,
        "owner_reorder_is_stable": deltas["owner_reorder_nrmse"] < 1.0e-5,
        "missing_and_removed_match": deltas["missing_removal_nrmse"] < 1.0e-5,
        "fallback_is_neutral": sum(fallback) / 3.0 > 0.005
        and max(fallback) - min(fallback) < max(0.04, sum(fallback) / 3.0 * 0.25),
        "visibility_is_scalar": scalar_channel_error == 0.0,
        "visibility_does_not_amplify": maximum_amplification <= 1.0 / 255.0,
        "disabled_bypasses_visibility": deltas["disabled_bypass_nrmse"] < 1.0e-5,
        "unavailable_bypasses_visibility": deltas["unavailable_bypass_nrmse"] < 1.0e-5
        and deltas["unavailable_visibility_nrmse"] < 1.0e-5,
        "ssao_is_diffuse_only": deltas["ssao_specular_nrmse"] < 1.0e-5,
        "material_ao_suppresses_rough_specular": deltas["material_ao_nrmse"] > 0.00005
        and luminance(metrics["m15-occluded-material"]["rough"])
        < luminance(metrics["m15-unoccluded"]["rough"]) * 0.97,
        "smooth_specular_is_retained": abs(
            luminance(metrics["m15-occluded-material"]["smooth"])
            - luminance(metrics["m15-unoccluded"]["smooth"])
        )
        <= 0.02,
        "gtao_contributes_visibility": deltas["gtao_nrmse"] > 0.000001,
        "combined_visibility_is_bounded": deltas["combined_nrmse"] + 1.0e-5
        >= max(deltas["material_ao_nrmse"], deltas["gtao_nrmse"]),
        "indirect_intensity_does_not_scale_specular": deltas["indirect_intensity_nrmse"] < 1.0e-5,
        "boundary_is_finite_nonblack": min(boundary_luminances) > 0.0001
        and max(boundary_luminances) < min(boundary_luminances) * 2.0 + 0.01,
        "moved_camera_keeps_specular": luminance(metrics["m15-camera-moved-occluded"]["center"]) > 0.0001,
        "transformed_debug_bounds_are_visible": deltas["debug_bounds_nrmse"] > 0.0001
        and 0.01 < debug_bounds_changed_fraction < 0.1,
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise RuntimeError("Reflection probe PNG checks failed: " + ", ".join(failed))
    return {
        "regions": metrics,
        "deltas": deltas,
        "debug_bounds_changed_fraction": debug_bounds_changed_fraction,
        "checks": checks,
    }


def main() -> int:
    args = parse_args()
    try:
        if (args.width, args.height) != (1920, 1080):
            raise ValueError("The M15 reflection probe gate requires exactly 1920x1080.")
        if args.timeout <= 0:
            raise ValueError("--timeout must be positive.")

        root = repo_root()
        editor = (args.editor or root / "out/install/vs2026-x64/bin/EvoEngineEditor.exe").resolve()
        if not editor.is_file():
            raise FileNotFoundError(f"Installed editor not found: {editor}")
        output_dir = args.output_dir if args.output_dir.is_absolute() else root / args.output_dir
        output_dir = output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        for name in (
            *[f"{name}.png" for name in CAPTURE_NAMES],
            "explicit-payload-enabled.png",
            "explicit-payload-disabled.png",
            "report.json",
            "run.log",
            "evidence.json",
        ):
            (output_dir / name).unlink(missing_ok=True)

        shader_cache = output_dir / "shader-cache"
        shader_cache.mkdir(parents=True, exist_ok=True)
        imgui_path = output_dir / "imgui.ini"
        imgui_path.unlink(missing_ok=True)
        environment = os.environ.copy()
        environment["EVOENGINE_REFLECTION_PROBE_EVIDENCE"] = str(output_dir)
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
        print(f"Reflection probe validation ({args.config}): {' '.join(command)}")
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
            "EVOENGINE_REFLECTION_PROBE_REPORT",
            "passed=true",
            "EVOENGINE_REFLECTION_PROBE_SHUTDOWN_COMPLETE",
        ):
            if marker not in output:
                raise RuntimeError(f"Missing runtime marker {marker!r}; log={log_path}")

        report_path = output_dir / "report.json"
        if not report_path.is_file():
            raise RuntimeError(f"Reflection probe report was not written: {report_path}")
        report = json.loads(report_path.read_text(encoding="utf-8"))
        expected_contract = {
            "resolution": [1920, 1080],
            "render_mode": "Rasterization",
            "graphics_validation": True,
            "launch_count": 1,
            "probe_resolution": 256,
            "probe_mips": 9,
            "probe_format": "RGBA16F",
            "canonical_texels": 524286,
            "canonical_payload_bytes": 4194288,
            "packed_payload_bytes": 2097144,
            "max_enabled_probes": 32,
            "bake_near": 0.1,
            "bake_far": 1000.0,
            "screen_space_reflections": False,
            "ray_traced_reflections": False,
            "gpu_timestamps": True,
            "no_auto_bake_observation_frames": 4,
            "timing_warmup_frames": 8,
            "timing_measure_frames": 120,
            "batch_explicit_rebake_policy": "serial",
            "specular_visibility_model": SPECULAR_VISIBILITY_MODEL,
        }
        if report.get("schema_version") != 3 or report.get("passed") is not True:
            raise RuntimeError("Reflection probe report did not pass its schema contract.")
        if report.get("contract") != expected_contract:
            raise RuntimeError("Reflection probe report has the wrong fixed validation contract.")
        checks = report.get("checks")
        if not isinstance(checks, dict) or not checks or any(value is not True for value in checks.values()):
            failed = sorted(name for name, value in (checks or {}).items() if value is not True)
            raise RuntimeError("Reflection probe runtime checks failed: " + ", ".join(failed))
        captures = report.get("captures")
        if not isinstance(captures, list) or [capture.get("name") for capture in captures] != list(CAPTURE_NAMES):
            raise RuntimeError("Reflection probe report does not contain the canonical capture sequence.")
        expected_counts = dict.fromkeys(CAPTURE_NAMES, 5)
        expected_counts["removed"] = 4
        expected_counts["debug-bounds-off"] = 4
        expected_counts["debug-bounds-on"] = 4
        if any(
            capture.get("finite") is not True or capture.get("probe_count") != expected_counts[capture["name"]]
            for capture in captures
        ):
            raise RuntimeError("Reflection probe capture metadata is invalid.")
        expected_m15_controls = {
            "m15-unoccluded": (2, "disabled", 1.0, 0.0),
            "m15-visibility-off": (3, "disabled", 1.0, 0.0),
            "m15-occluded-off": (4, "disabled", 1.0, 0.0),
            "m15-visibility-material": (3, "disabled", 0.2, 0.0),
            "m15-occluded-material": (4, "disabled", 0.2, 0.0),
            "m15-visibility-gtao": (3, "gtao", 1.0, 0.0),
            "m15-occluded-gtao": (4, "gtao", 1.0, 0.0),
            "m15-visibility-material-gtao": (3, "gtao", 0.2, 0.0),
            "m15-occluded-material-gtao": (4, "gtao", 0.2, 0.0),
            "m15-visibility-unavailable": (3, "unavailable", 1.0, 0.0),
            "m15-occluded-unavailable": (4, "unavailable", 1.0, 0.0),
            "m15-occluded-ssao": (4, "ssao", 1.0, 0.0),
            "m15-occluded-indirect-double": (4, "disabled", 0.2, 2.0),
            "m15-boundary-left": (4, "disabled", 0.2, 0.0),
            "m15-boundary-center": (4, "disabled", 0.2, 0.0),
            "m15-boundary-right": (4, "disabled", 0.2, 0.0),
            "m15-camera-moved-occluded": (4, "disabled", 0.2, 0.0),
        }
        captures_by_name = {capture["name"]: capture for capture in captures}
        for name, expected in expected_m15_controls.items():
            capture = captures_by_name[name]
            if (
                capture.get("indirect_debug_view") != expected[0]
                or capture.get("ambient_occlusion") != expected[1]
                or not math.isclose(capture.get("material_occlusion", math.nan), expected[2], abs_tol=1.0e-6)
                or not math.isclose(
                    capture.get("indirect_lighting_intensity", math.nan), expected[3], abs_tol=1.0e-6
                )
            ):
                raise RuntimeError(f"Reflection probe capture {name!r} has invalid M15 controls: {capture!r}.")
        bake = report.get("bake", {})
        if (
            bake.get("nonrecursive_equal") is not True
            or bake.get("nonblack") is not True
            or bake.get("persisted_payload_bytes") != 4194288
        ):
            raise RuntimeError("Reflection probe canonical non-recursive bake evidence is invalid.")
        explicit_rebake = report.get("explicit_rebake", {})
        no_auto = explicit_rebake.get("no_auto_rebake", {})
        single = explicit_rebake.get("single", {})
        imported = explicit_rebake.get("imported_probe", {})
        batch = explicit_rebake.get("batch", {})
        if (
            explicit_rebake.get("asset_entry_count") != 2
            or explicit_rebake.get("unique_entries") is not True
            or explicit_rebake.get("unique_assets") is not True
            or explicit_rebake.get("initial_controls") != {"sky": 0.75, "indirect": 0.5}
            or explicit_rebake.get("final_controls") != {"sky": 1.0, "indirect": 0.0}
        ):
            raise RuntimeError("Reflection probe explicit-bake evidence is invalid.")
        for key in ("payload_hashes_before", "payload_hashes_after"):
            values = no_auto.get(key)
            if (
                not isinstance(values, list)
                or len(values) != 2
                or any(not isinstance(value, int) or value == 0 for value in values)
            ):
                raise RuntimeError(f"Reflection probe no-auto-rebake field {key!r} is invalid.")
        if (
            no_auto.get("frames") != 4
            or no_auto.get("payload_active") is not True
            or no_auto.get("payload_hashes_unchanged") is not True
            or no_auto.get("last_valid_payload_rendered") is not True
            or not isinstance(no_auto.get("render_nrmse"), (int, float))
            or not math.isfinite(no_auto["render_nrmse"])
            or no_auto["render_nrmse"] <= 0.0001
            or no_auto.get("enabled_image") != "explicit-payload-enabled.png"
            or no_auto.get("disabled_image") != "explicit-payload-disabled.png"
            or no_auto["payload_hashes_before"] != no_auto["payload_hashes_after"]
        ):
            raise RuntimeError("Reflection probes baked automatically or lost their explicit payload.")
        if (
            single.get("ready") is not True
            or not isinstance(single.get("payload_hash_before"), int)
            or not isinstance(single.get("payload_hash_after"), int)
            or single["payload_hash_before"] == 0
            or single["payload_hash_after"] == 0
            or single["payload_hash_before"] == single["payload_hash_after"]
        ):
            raise RuntimeError("Reflection probe single explicit rebake evidence is invalid.")
        if (
            imported.get("status") != "Imported"
            or imported.get("unchanged") is not True
            or not isinstance(imported.get("payload_hash_before"), int)
            or imported.get("payload_hash_before") == 0
            or imported.get("payload_hash_before") != imported.get("payload_hash_after")
        ):
            raise RuntimeError("Imported reflection probe explicit-bake evidence is invalid.")
        pending_counts = batch.get("pending_counts")
        payload_hashes = batch.get("payload_hashes")
        batch_timing = batch.get("timing", {})
        if (
            batch.get("queued_count") != 2
            or not isinstance(pending_counts, list)
            or not pending_counts
            or any(not isinstance(value, int) or value < 0 or value > 2 for value in pending_counts)
            or batch.get("max_pending") != 2
            or batch.get("both_observed_pending") is not True
            or batch.get("final_ready") is not True
            or not isinstance(payload_hashes, list)
            or len(payload_hashes) != 2
            or payload_hashes[0] == 0
            or payload_hashes[0] != payload_hashes[1]
        ):
            raise RuntimeError("Reflection probe batch-rebake evidence is invalid.")
        for key in ("wall_ms", "gpu_total_ms", "face_capture_ms", "ggx_prefilter_ms", "cpu_total_ms"):
            value = batch_timing.get(key)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0.0:
                raise RuntimeError(f"Reflection probe batch timing field {key!r} is invalid.")
        for key in ("gpu_total_samples", "cpu_total_samples"):
            if batch_timing.get(key) != 1:
                raise RuntimeError(f"Reflection probe batch timing field {key!r} does not describe one batch.")
        for key in ("face_capture_samples", "ggx_prefilter_samples"):
            value = batch_timing.get(key)
            if not isinstance(value, int) or value < 0 or value > 2:
                raise RuntimeError(f"Reflection probe batch timing field {key!r} is invalid.")
        gpu_timing = report.get("gpu_timing", {})
        timing_samples = gpu_timing.get("samples_ms")
        if (
            gpu_timing.get("scope") != "Deferred Lighting"
            or gpu_timing.get("warmup_frames") != 8
            or gpu_timing.get("measure_frames") != 120
            or gpu_timing.get("sample_count") != 120
            or not isinstance(timing_samples, list)
            or len(timing_samples) != 120
            or any(
                not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0
                for value in timing_samples
            )
        ):
            raise RuntimeError("Reflection probe Deferred Lighting timing samples are invalid.")
        timing_median = gpu_timing.get("median_ms")
        timing_p95 = gpu_timing.get("p95_ms")
        if (
            not isinstance(timing_median, (int, float))
            or not math.isfinite(timing_median)
            or timing_median < 0
            or not isinstance(timing_p95, (int, float))
            or not math.isfinite(timing_p95)
            or timing_p95 < timing_median
            or not math.isclose(timing_median, percentile(timing_samples, 0.5), rel_tol=1.0e-9, abs_tol=1.0e-12)
            or not math.isclose(timing_p95, percentile(timing_samples, 0.95), rel_tol=1.0e-9, abs_tol=1.0e-12)
        ):
            raise RuntimeError("Reflection probe Deferred Lighting timing summary is invalid.")
        timing_relative_mad = relative_mad(timing_samples)
        if (
            not math.isclose(gpu_timing.get("m14_baseline_median_ms", math.nan), 0.087536, abs_tol=1.0e-12)
            or not math.isclose(gpu_timing.get("m14_baseline_p95_ms", math.nan), 0.0970064, abs_tol=1.0e-12)
            or not math.isclose(gpu_timing.get("median_limit_ms", math.nan), 0.137536, abs_tol=1.0e-12)
            or not math.isclose(gpu_timing.get("p95_limit_ms", math.nan), 0.1970064, abs_tol=1.0e-12)
            or not math.isclose(gpu_timing.get("relative_mad_limit", math.nan), 0.25, abs_tol=1.0e-12)
            or timing_median > gpu_timing["median_limit_ms"]
            or timing_p95 > gpu_timing["p95_limit_ms"]
            or timing_relative_mad > gpu_timing["relative_mad_limit"]
        ):
            raise RuntimeError("Reflection probe Deferred Lighting timing exceeded the frozen M15 limits.")
        abi = report.get("abi")
        expected_abi = {
            "new_descriptors": 0,
            "lighting_sampler_count": 37,
            "reflection_probe_info_bytes": 128,
            "render_info_bytes": 6032,
            "camera_info_bytes": 704,
        }
        if abi != expected_abi:
            raise RuntimeError("Reflection probe ABI evidence is invalid.")
        rough_specular = report.get("rough_specular", {})
        numeric_rough_fields = (
            "material_ao_nrmse",
            "gtao_nrmse",
            "combined_nrmse",
            "disabled_bypass_nrmse",
            "unavailable_bypass_nrmse",
            "unavailable_visibility_nrmse",
            "ssao_specular_nrmse",
            "indirect_intensity_nrmse",
            "scalar_channel_error",
            "maximum_amplification",
            "boundary_luminance_min",
            "boundary_luminance_max",
        )
        if any(
            not isinstance(rough_specular.get(name), (int, float))
            or not math.isfinite(rough_specular[name])
            or rough_specular[name] < 0
            for name in numeric_rough_fields
        ):
            raise RuntimeError("Reflection probe rough-specular metrics are malformed.")
        if (
            rough_specular["disabled_bypass_nrmse"] >= 1.0e-6
            or rough_specular["unavailable_bypass_nrmse"] >= 1.0e-6
            or rough_specular["unavailable_visibility_nrmse"] >= 1.0e-6
            or rough_specular["ssao_specular_nrmse"] >= 1.0e-6
            or rough_specular["indirect_intensity_nrmse"] >= 1.0e-6
            or rough_specular["scalar_channel_error"] >= 1.0e-6
            or rough_specular["material_ao_nrmse"] <= 0.0001
            or rough_specular["gtao_nrmse"] <= 0.000001
            or rough_specular["combined_nrmse"] + 1.0e-6
            < max(rough_specular["material_ao_nrmse"], rough_specular["gtao_nrmse"])
            or rough_specular["maximum_amplification"] >= 1.0e-5
            or rough_specular["boundary_luminance_min"] <= 0.0001
            or rough_specular["boundary_luminance_max"]
            >= rough_specular["boundary_luminance_min"] * 2.0 + 0.01
        ):
            raise RuntimeError("Reflection probe rough-specular evidence exceeded its M15 limits.")
        packed = report.get("packed_runtime", {})
        expected_quality_cases = [
            "bright_glossy",
            "roughness_extremes",
            "dark_saturated_gradients",
            "small_intense_emitter",
        ]
        if (
            packed.get("adopted") is not False
            or packed.get("active_format") != "RGBA16F"
            or packed.get("quality_cases") != expected_quality_cases
            or packed.get("gpu_time_improvement_measured") is not False
            or not math.isclose(
                packed.get("minimum_gpu_time_improvement", math.nan), 0.05, rel_tol=0.0, abs_tol=1.0e-8
            )
            or packed.get("memory_saving", 0.0) < 0.49
        ):
            raise RuntimeError("Reflection probe packed-format no-adopt evidence is invalid.")
        expected_memory = {
            "canonical_gpu_image_logical_bytes": 4194288,
            "canonical_steady_cpu_payload_bytes": 4194288,
            "canonical_serialized_payload_bytes": 4194288,
            "packed_gpu_image_logical_bytes": 2097144,
            "packed_steady_cpu_payload_bytes": 0,
            "packed_serialized_payload_bytes": 0,
        }
        memory = report.get("memory")
        if memory != expected_memory:
            raise RuntimeError("Reflection probe memory accounting evidence is invalid.")

        artifacts: list[dict[str, object]] = []
        images: dict[str, PngImage] = {}
        for name in CAPTURE_NAMES:
            artifact, image = validate_png(output_dir / f"{name}.png", args.width, args.height)
            artifacts.append(artifact)
            images[name] = image
        image_evidence = validate_images(images)
        explicit_enabled_artifact, explicit_enabled = validate_png(
            output_dir / "explicit-payload-enabled.png", args.width, args.height
        )
        explicit_disabled_artifact, explicit_disabled = validate_png(
            output_dir / "explicit-payload-disabled.png", args.width, args.height
        )
        explicit_render_nrmse = normalized_rms(explicit_enabled, explicit_disabled)
        if not math.isfinite(explicit_render_nrmse) or explicit_render_nrmse <= 0.00005:
            raise RuntimeError("Explicit reflection probe payload did not produce an independent rendered image delta.")
        artifacts.extend((explicit_enabled_artifact, explicit_disabled_artifact))
        image_evidence["explicit_payload_render_nrmse"] = explicit_render_nrmse
        artifacts.extend(
            (
                {"file": report_path.name, "bytes": report_path.stat().st_size, "sha256": sha256(report_path)},
                {"file": log_path.name, "bytes": log_path.stat().st_size, "sha256": sha256(log_path)},
            )
        )
        gpu_timing_evidence = {**gpu_timing, "relative_mad": timing_relative_mad}
        evidence = {
            "schema_version": 3,
            "configuration": args.config,
            "launch_count": 1,
            "command": command,
            "editor": str(editor),
            "editor_sha256": sha256(editor),
            "checks": checks,
            "bake": bake,
            "explicit_rebake": explicit_rebake,
            "gpu_timing": gpu_timing_evidence,
            "rough_specular": rough_specular,
            "abi": abi,
            "packed_runtime": packed,
            "memory": memory,
            "image_evidence": image_evidence,
            "artifacts": artifacts,
            "passed": True,
        }
        evidence_path = output_dir / "evidence.json"
        evidence_path.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
        print(f"Reflection probe validation passed in one launch; evidence={evidence_path}")
        return 0
    except Exception as error:
        print(f"Reflection probe validation failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
