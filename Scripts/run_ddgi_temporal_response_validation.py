#!/usr/bin/env python3
"""Validate confidence-aware DDGI irradiance response with frame-indexed HDR captures."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path

from compare_reference_render import compare_hdr_images
from run_ddgi_small_emitter_baseline import (
    REGIONS,
    SEED,
    common_command,
    contribution_mean,
    json_safe,
    require_hdr,
    run_command,
    summarize_image,
)


RESPONSE_FRAMES = (1, 2, 4, 8, 16, 32)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=root / "out/ddgi-small-emitter/m4")
    parser.add_argument("--editor", type=Path, help="Installed EvoEngineEditor executable override.")
    parser.add_argument("--timeout", type=float, default=900.0, help="Per-launch timeout in seconds.")
    parser.add_argument("--reuse-settled", action="store_true", help="Reuse existing settled/repeat captures.")
    parser.add_argument("--uniform-rays", type=int, default=192, help="Uniform/fixed rays per probe.")
    parser.add_argument("--emissive-rays", "--guided-rays", dest="emissive_rays", type=int, default=64,
                        help="Exact emissive-triangle rays per probe.")
    parser.add_argument("--continuous-updates", action="store_true", help="Do not pause probe updates after convergence.")
    return parser.parse_args()


def capture(editor: Path, output_dir: Path, environment: dict[str, str], fixture: str, suffix: str,
            timeout: float, response_frames: int = 0, emissive_rays: int = 0,
            uniform_rays: int | None = None, continuous_updates: bool = False):
    image_path = output_dir / f"{fixture}-{suffix}.hdr"
    report_path = output_dir / f"{fixture}-{suffix}.json"
    command = common_command(editor, image_path, fixture, 1920, 1080)
    command.extend((
        "--preview-render-mode", "rasterization",
        "--preview-warmup-frames", "64",
        "--preview-ddgi-report", str(report_path),
        "--preview-ddgi-measure-frames", "120",
        "--preview-ddgi-emissive-rays", str(emissive_rays),
    ))
    if uniform_rays is not None:
        command.extend(("--preview-ddgi-uniform-rays", str(uniform_rays)))
    if continuous_updates:
        command.append("--preview-ddgi-continuous-updates")
    if response_frames:
        command.extend(("--preview-ddgi-response-frames", str(response_frames)))
    run_command(command, editor.parent, environment, timeout, output_dir / f"{fixture}-{suffix}.log")
    image = require_hdr(image_path, 1920, 1080)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("schema_version") != 7 or report.get("capture", {}).get("response_frames") != response_frames:
        raise RuntimeError(f"Unexpected M4 response report contract: {report_path}")
    return image, report


def receiver_contribution(image, dark) -> float:
    return contribution_mean(summarize_image(image, dark))


def load_capture(output_dir: Path, fixture: str, suffix: str):
    image_path = output_dir / f"{fixture}-{suffix}.hdr"
    report_path = output_dir / f"{fixture}-{suffix}.json"
    image = require_hdr(image_path, 1920, 1080)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("schema_version") != 7:
        raise RuntimeError(f"Unexpected reused M4 report contract: {report_path}")
    return image, report


def legacy_bright_response(frames: int) -> float:
    response = 0.0
    for frame in range(frames):
        blend = 0.25 if frame == 0 else (1.0 - 0.97) * 0.25
        response += (1.0 - response) * blend
    return response


def first_frame_at_least(curve: dict[int, float], threshold: float) -> int | None:
    return next((frame for frame in sorted(curve) if curve[frame] >= threshold), None)


def main() -> int:
    try:
        args = parse_args()
        if args.timeout <= 0:
            raise ValueError("--timeout must be positive.")
        if args.uniform_rays < 1 or args.uniform_rays > 4096 or args.emissive_rays < 0 or args.emissive_rays > 4096:
            raise ValueError("Uniform rays must be 1-4096 and emissive rays 0-4096.")
        root = repo_root()
        editor = (args.editor or root / "out/install/vs2026-x64/bin/EvoEngineEditor.exe").resolve()
        if not editor.is_file():
            raise FileNotFoundError(f"Installed editor not found: {editor}")
        output_dir = (args.output_dir if args.output_dir.is_absolute() else root / args.output_dir).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        environment = os.environ.copy()
        environment["EVOENGINE_SHADER_CACHE_DIR"] = str(output_dir / "shader-cache")
        environment["EVOENGINE_IMGUI_INI_PATH"] = str(output_dir / "imgui.ini")

        settled_capture = load_capture if args.reuse_settled else (
            lambda directory, fixture, suffix: capture(
                editor, directory, environment, fixture, suffix, args.timeout, 0,
                args.emissive_rays, args.uniform_rays, args.continuous_updates
            )
        )
        dark, dark_report = settled_capture(output_dir, "emissive-empty", "settled")
        settled_on, settled_on_report = settled_capture(output_dir, "emissive-enable", "settled")
        settled_on_repeat, settled_on_repeat_report = settled_capture(output_dir, "emissive-enable", "settled-repeat")
        settled_hdr, settled_hdr_report = settled_capture(output_dir, "emissive-enable-hdr", "settled")
        stable_scenes = {}
        stable_scene_reports = {}
        for fixture in ("emissive-large", "furnace", "analytic-light"):
            stable, stable_scene_reports[f"{fixture}-settled"] = settled_capture(output_dir, fixture, "settled")
            repeat, stable_scene_reports[f"{fixture}-repeat"] = settled_capture(output_dir, fixture, "repeat")
            stable_scenes[fixture] = compare_hdr_images(stable, repeat)
        settled_energy = receiver_contribution(settled_on, dark)
        settled_hdr_energy = receiver_contribution(settled_hdr, dark)
        if settled_energy <= 0.0 or settled_hdr_energy <= settled_energy:
            raise RuntimeError("M4 settled response fixtures did not produce ordered positive receiver energy.")

        reports = {}
        on_curve = {}
        off_curve = {}
        hdr_curve = {}
        hdr_peak_ratio = {}
        for frame in RESPONSE_FRAMES:
            enabled, reports[f"enable-{frame}"] = capture(
                editor, output_dir, environment, "emissive-enable", f"response-{frame}", args.timeout, frame,
                args.emissive_rays, args.uniform_rays, args.continuous_updates
            )
            disabled, reports[f"disable-{frame}"] = capture(
                editor, output_dir, environment, "emissive-disable", f"response-{frame}", args.timeout, frame,
                args.emissive_rays, args.uniform_rays, args.continuous_updates
            )
            hdr, reports[f"hdr-{frame}"] = capture(
                editor, output_dir, environment, "emissive-enable-hdr", f"response-{frame}", args.timeout, frame,
                args.emissive_rays, args.uniform_rays, args.continuous_updates
            )
            on_curve[frame] = receiver_contribution(enabled, dark) / settled_energy
            off_curve[frame] = receiver_contribution(disabled, dark) / settled_energy
            hdr_curve[frame] = receiver_contribution(hdr, dark) / settled_hdr_energy
            hdr_peak = max(float(region["peak_luminance"]) for region in summarize_image(hdr, dark).values())
            settled_hdr_peak = max(
                float(region["peak_luminance"]) for region in summarize_image(settled_hdr, dark).values()
            )
            hdr_peak_ratio[frame] = hdr_peak / max(settled_hdr_peak, 1.0e-12)

        legacy_curve = {frame: legacy_bright_response(frame) for frame in RESPONSE_FRAMES}
        repeatability = compare_hdr_images(settled_on, settled_on_repeat)
        on_latency = first_frame_at_least(on_curve, 0.9)
        off_latency = first_frame_at_least({frame: 1.0 - value for frame, value in off_curve.items()}, 0.9)
        checks = {
            "legacy_90_percent_exceeds_256_frames": legacy_bright_response(256) < 0.9,
            "confidence_response_reaches_90_percent_by_16_frames": on_latency is not None and on_latency <= 16,
            "darkening_reaches_90_percent_by_8_frames": off_latency is not None and off_latency <= 8,
            "steady_state_repeat_is_exact": repeatability["relative_l2_error"] <= 1.0e-6,
            "hdr_response_is_finite_and_bounded": all(
                math.isfinite(value) and 0.0 <= value <= 1.5 for value in (*hdr_curve.values(), *hdr_peak_ratio.values())
            ),
            "response_is_materially_faster_than_legacy": on_curve[16] >= legacy_curve[16] + 0.5,
            "large_area_is_stable": stable_scenes["emissive-large"]["relative_l2_error"] <= 1.0e-6,
            "environment_is_stable": stable_scenes["furnace"]["relative_l2_error"] <= 1.0e-6,
            "analytic_light_is_stable": stable_scenes["analytic-light"]["relative_l2_error"] <= 1.0e-6,
        }
        if not all(checks.values()):
            raise RuntimeError("M4 checks failed: " + ", ".join(name for name, passed in checks.items() if not passed))

        evidence = {
            "schema_version": 1,
            "contract": {
                "resolution": [1920, 1080],
                "warmup_frames": 64,
                "response_frames": list(RESPONSE_FRAMES),
                "regions": REGIONS,
                "seed": SEED,
                "legacy_hysteresis": 0.97,
                "legacy_bright_delta_scale": 0.25,
                "emissive_rays_per_probe": args.emissive_rays,
                "uniform_rays_per_probe": args.uniform_rays,
                "pause_updates_after_convergence": not args.continuous_updates,
            },
            "checks": checks,
            "legacy_projected_curve": legacy_curve,
            "confidence_response_curve": on_curve,
            "darkening_remaining_curve": off_curve,
            "hdr_response_curve": hdr_curve,
            "hdr_peak_ratio": hdr_peak_ratio,
            "on_90_percent_latency_frames": on_latency,
            "off_90_percent_latency_frames": off_latency,
            "repeatability": repeatability,
            "stable_scene_repeatability": stable_scenes,
            "settled_reports": {
                "dark": dark_report,
                "on": settled_on_report,
                "on_repeat": settled_on_repeat_report,
                "hdr": settled_hdr_report,
            },
            "response_reports": reports,
            "stable_scene_reports": stable_scene_reports,
            "passed": True,
        }
        evidence_path = output_dir / "evidence.json"
        evidence_path.write_text(json.dumps(json_safe(evidence), indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(f"DDGI M4 temporal response validation passed; evidence={evidence_path}")
        return 0
    except Exception as error:
        print(f"DDGI M4 temporal response validation failed: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
