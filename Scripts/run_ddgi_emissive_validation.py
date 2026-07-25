#!/usr/bin/env python3
"""Validate DDGI emissive-mesh sampling with one installed-editor A/B launch."""

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

from compare_reference_render import PngImage, compare_images, read_image


FATAL_PATTERNS = (
    re.compile(r"Vulkan\s+\[Validation\]-\[Error\]", re.IGNORECASE),
    re.compile(r"\bVK_ERROR_DEVICE_LOST\b", re.IGNORECASE),
    re.compile(r"\bdevice\s+lost\b", re.IGNORECASE),
    re.compile(r"\bunhandled\s+exception\b", re.IGNORECASE),
    re.compile(r"\bfatal(?:\s+error)?\b", re.IGNORECASE),
    re.compile(r"\bassertion\s+failed\b", re.IGNORECASE),
    re.compile(r"EVOENGINE_DDGI_EMISSIVE_ERROR", re.IGNORECASE),
)
PHASES = (("enabled", True), ("disabled", False), ("enabled-repeat", True))
REPEATABILITY_LIMIT = 0.002


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="RelWithDebInfo", help="Installed build configuration label.")
    parser.add_argument("--width", type=int, default=1920, help="Render width; must be 1920.")
    parser.add_argument("--height", type=int, default=1080, help="Render height; must be 1080.")
    parser.add_argument("--measure-frames", type=int, default=120, help="Per-phase measured frames; fixed at 120.")
    parser.add_argument("--output-dir", type=Path, default=root / "out/ddgi-validation-m8-emissive")
    parser.add_argument("--timeout", type=float, default=900.0, help="Maximum launch duration in seconds.")
    parser.add_argument("--editor", type=Path, help="Installed EvoEngineEditor executable override.")
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def percentile(samples: list[float], value: float) -> float:
    ordered = sorted(samples)
    rank = min(max(value, 0.0), 1.0) * (len(ordered) - 1)
    lower = math.floor(rank)
    upper = math.ceil(rank)
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def validate_png(path: Path, width: int, height: int) -> tuple[PngImage, dict[str, object]]:
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"Missing or empty DDGI emissive capture: {path}")
    image = read_image(path)
    if not isinstance(image, PngImage):
        raise RuntimeError(f"DDGI emissive capture is not a PNG: {path}")
    if (image.width, image.height) != (width, height):
        raise RuntimeError(f"DDGI emissive capture has {image.width}x{image.height}; expected {width}x{height}: {path}")
    if not any(image.rgba[channel] for channel in range(0, len(image.rgba), 4)) and not any(
        image.rgba[channel] for channel in range(1, len(image.rgba), 4)
    ) and not any(image.rgba[channel] for channel in range(2, len(image.rgba), 4)):
        raise RuntimeError(f"DDGI emissive capture is black: {path}")
    return image, {
        "file": path.name,
        "width": image.width,
        "height": image.height,
        "bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def validate_phase(phase: object, name: str, enabled: bool, measure_frames: int) -> None:
    if not isinstance(phase, dict) or phase.get("name") != name:
        raise RuntimeError(f"DDGI emissive report is missing phase {name!r}.")
    if phase.get("requested_enabled") is not enabled or phase.get("effective_enabled") is not enabled:
        raise RuntimeError(f"DDGI emissive phase {name!r} did not report its requested effective state.")
    if not isinstance(phase.get("convergence_frames"), int) or not 0 < phase["convergence_frames"] <= 1024:
        raise RuntimeError(f"DDGI emissive phase {name!r} has invalid convergence evidence.")
    if not isinstance(phase.get("emissive_triangle_count"), int) or phase["emissive_triangle_count"] <= 0:
        raise RuntimeError(f"DDGI emissive phase {name!r} has an empty shared triangle inventory.")
    if phase.get("enabled_volume_count") != (1 if enabled else 0):
        raise RuntimeError(f"DDGI emissive phase {name!r} has the wrong enabled-volume count.")
    variability = phase.get("variability")
    variability_samples = phase.get("variability_samples")
    if not isinstance(variability, (int, float)) or not math.isfinite(variability) or variability < 0:
        raise RuntimeError(f"DDGI emissive phase {name!r} has invalid variability.")
    if not isinstance(variability_samples, int) or variability_samples <= 0:
        raise RuntimeError(f"DDGI emissive phase {name!r} has invalid variability sample accounting.")
    candidate_rays = phase.get("candidate_ray_count")
    if not isinstance(candidate_rays, int) or (candidate_rays <= 0 if enabled else candidate_rays != 0):
        raise RuntimeError(f"DDGI emissive phase {name!r} has invalid candidate-ray accounting.")
    if not isinstance(phase.get("luminance_sum"), (int, float)) or not math.isfinite(phase["luminance_sum"]) or phase[
        "luminance_sum"
    ] <= 0:
        raise RuntimeError(f"DDGI emissive phase {name!r} has invalid luminance.")
    timing = phase.get("probe_trace")
    if not isinstance(timing, dict) or timing.get("sample_count") != measure_frames:
        raise RuntimeError(f"DDGI emissive phase {name!r} has the wrong DDGI Probe Trace sample count.")
    samples = timing.get("samples_ms")
    if not isinstance(samples, list) or len(samples) != measure_frames:
        raise RuntimeError(f"DDGI emissive phase {name!r} is missing raw DDGI Probe Trace samples.")
    if any(not isinstance(sample, (int, float)) or not math.isfinite(sample) or sample < 0 for sample in samples):
        raise RuntimeError(f"DDGI emissive phase {name!r} contains invalid timing samples.")
    if len(set(samples)) < 2:
        raise RuntimeError(f"DDGI emissive phase {name!r} has a constant timing series.")
    median = timing.get("median_ms")
    p95 = timing.get("p95_ms")
    if not isinstance(median, (int, float)) or not math.isfinite(median) or median < 0:
        raise RuntimeError(f"DDGI emissive phase {name!r} has an invalid timing median.")
    if not isinstance(p95, (int, float)) or not math.isfinite(p95) or p95 < median:
        raise RuntimeError(f"DDGI emissive phase {name!r} has an invalid timing p95.")
    for label, reported, recomputed in (
        ("median", median, percentile(samples, 0.5)),
        ("p95", p95, percentile(samples, 0.95)),
    ):
        if not math.isclose(reported, recomputed, rel_tol=1e-9, abs_tol=1e-12):
            raise RuntimeError(f"DDGI emissive phase {name!r} has an inconsistent timing {label}.")


def main() -> int:
    args = parse_args()
    try:
        if (args.width, args.height) != (1920, 1080):
            raise ValueError("The DDGI emissive closeout gate requires exactly 1920x1080.")
        if args.measure_frames != 120:
            raise ValueError("The DDGI emissive closeout gate requires exactly 120 measured frames per phase.")
        if args.timeout <= 0:
            raise ValueError("--timeout must be positive.")

        root = repo_root()
        editor = (args.editor or root / "out/install/vs2026-x64/bin/EvoEngineEditor.exe").resolve()
        if not editor.is_file():
            raise FileNotFoundError(f"Installed editor not found: {editor}")
        output_dir = args.output_dir if args.output_dir.is_absolute() else root / args.output_dir
        output_dir = output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        for name in ("enabled.png", "disabled.png", "enabled-repeat.png", "report.json", "run.log", "evidence.json"):
            (output_dir / name).unlink(missing_ok=True)

        shader_cache = output_dir / "shader-cache"
        shader_cache.mkdir(parents=True, exist_ok=True)
        imgui_path = output_dir / "imgui.ini"
        imgui_path.unlink(missing_ok=True)
        environment = os.environ.copy()
        environment["EVOENGINE_DDGI_EMISSIVE_EVIDENCE"] = str(output_dir)
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
        print(f"DDGI emissive validation ({args.config}): {' '.join(command)}")
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
            "EVOENGINE_DDGI_EMISSIVE_REPORT",
            "passed=true",
            "EVOENGINE_DDGI_EMISSIVE_SHUTDOWN_COMPLETE",
        ):
            if marker not in output:
                raise RuntimeError(f"Missing runtime marker {marker!r}; log={log_path}")

        report_path = output_dir / "report.json"
        if not report_path.is_file():
            raise RuntimeError(f"DDGI emissive report was not written: {report_path}")
        report = json.loads(report_path.read_text(encoding="utf-8"))
        contract = report.get("contract", {})
        if report.get("schema_version") != 1 or report.get("passed") is not True:
            raise RuntimeError("DDGI emissive report did not pass its schema contract.")
        if contract.get("resolution") != [1920, 1080] or contract.get("launch_count") != 1:
            raise RuntimeError("DDGI emissive report has the wrong resolution or launch count.")
        if contract.get("measure_frames") != args.measure_frames or contract.get("deterministic_seed") != 0x6D2B79F5:
            raise RuntimeError("DDGI emissive report has the wrong measurement or seed contract.")
        if contract.get("vulkan_rt_pipeline") is not True or contract.get("graphics_validation") is not True:
            raise RuntimeError("DDGI emissive report lacks the required RT-pipeline validation contract.")
        checks = report.get("checks")
        if not isinstance(checks, dict) or not checks or any(value is not True for value in checks.values()):
            failed = sorted(name for name, value in (checks or {}).items() if value is not True)
            raise RuntimeError("DDGI emissive checks failed: " + ", ".join(failed))
        phases = report.get("phases")
        if not isinstance(phases, list) or len(phases) != len(PHASES):
            raise RuntimeError("DDGI emissive report does not contain exactly three phases.")
        for phase, (name, enabled) in zip(phases, PHASES, strict=True):
            validate_phase(phase, name, enabled, args.measure_frames)
        if not phases[0]["emissive_triangle_count"] == phases[1]["emissive_triangle_count"] == phases[2][
            "emissive_triangle_count"
        ]:
            raise RuntimeError("DDGI emissive inventory changed across the A/B phases.")
        if phases[0]["luminance_sum"] <= phases[1]["luminance_sum"]:
            raise RuntimeError("Enabled DDGI emissive sampling did not increase scene luminance.")

        images: dict[str, PngImage] = {}
        artifacts: list[dict[str, object]] = []
        for name, _ in PHASES:
            image, artifact = validate_png(output_dir / f"{name}.png", args.width, args.height)
            images[name] = image
            artifacts.append(artifact)
        repeatability = compare_images(images["enabled"], images["enabled-repeat"], ignore_alpha=True)
        separation = compare_images(images["enabled"], images["disabled"], ignore_alpha=True)
        if repeatability.get("normalized_rms_error", math.inf) > REPEATABILITY_LIMIT:
            raise RuntimeError("Enabled DDGI emissive repeat exceeded the 0.002 normalized RMS limit.")
        if separation.get("normalized_rms_error", 0.0) <= REPEATABILITY_LIMIT:
            raise RuntimeError("Enabled and disabled DDGI emissive captures are not measurably separated.")

        artifacts.extend(
            [
                {"file": report_path.name, "bytes": report_path.stat().st_size, "sha256": sha256(report_path)},
                {"file": log_path.name, "bytes": log_path.stat().st_size, "sha256": sha256(log_path)},
            ]
        )
        evidence = {
            "schema_version": 1,
            "configuration": args.config,
            "launch_count": 1,
            "command": command,
            "editor": str(editor),
            "checks": checks,
            "metrics": {
                "enabled_repeat": repeatability,
                "enabled_disabled": separation,
                "timing_delta_ms": {
                    "median": phases[0]["probe_trace"]["median_ms"] - phases[1]["probe_trace"]["median_ms"],
                    "p95": phases[0]["probe_trace"]["p95_ms"] - phases[1]["probe_trace"]["p95_ms"],
                },
                "variability_delta": phases[0]["variability"] - phases[1]["variability"],
                "hard_performance_and_variance_gate": "four frozen M2 holdouts",
            },
            "artifacts": artifacts,
            "passed": True,
        }
        evidence_path = output_dir / "evidence.json"
        evidence_path.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
        print(f"DDGI emissive validation passed in one launch; evidence={evidence_path}")
        return 0
    except Exception as error:
        print(f"DDGI emissive validation failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
