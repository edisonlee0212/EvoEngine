#!/usr/bin/env python3
"""Validate the installed DDGI Cornell application in one 1920x1080 launch."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

from compare_reference_render import PngImage, read_image


FATAL_PATTERNS = (
    re.compile(r"Vulkan\s+\[Validation\]-\[Error\]", re.IGNORECASE),
    re.compile(r"\bVK_ERROR_DEVICE_LOST\b", re.IGNORECASE),
    re.compile(r"\bdevice\s+lost\b", re.IGNORECASE),
    re.compile(r"\bunhandled\s+exception\b", re.IGNORECASE),
    re.compile(r"\bfatal(?:\s+error)?\b", re.IGNORECASE),
    re.compile(r"\bassertion\s+failed\b", re.IGNORECASE),
    re.compile(r"DDGI_APP_RESULT failed", re.IGNORECASE),
)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="RelWithDebInfo", help="Installed build configuration label.")
    parser.add_argument("--width", type=int, default=1920, help="Render width; must be 1920.")
    parser.add_argument("--height", type=int, default=1080, help="Render height; must be 1080.")
    parser.add_argument("--warmup-frames", type=int, default=360, help="Warmup frame count; must be 360.")
    parser.add_argument("--output-dir", type=Path, default=root / "out/ddgi-validation-m9-ddgiapp")
    parser.add_argument("--timeout", type=float, default=900.0, help="Maximum launch duration in seconds.")
    parser.add_argument("--app", type=Path, help="Installed DDGIApp executable override.")
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    args = parse_args()
    try:
        if (args.width, args.height) != (1920, 1080):
            raise ValueError("The DDGIApp closeout gate requires exactly 1920x1080.")
        if args.warmup_frames != 360:
            raise ValueError("The DDGIApp closeout gate requires exactly 360 warmup frames.")
        if args.timeout <= 0:
            raise ValueError("--timeout must be positive.")

        root = repo_root()
        app = (args.app or root / "out/install/vs2026-x64/bin/DDGIApp.exe").resolve()
        if not app.is_file():
            raise FileNotFoundError(f"Installed DDGIApp not found: {app}")
        output_dir = args.output_dir if args.output_dir.is_absolute() else root / args.output_dir
        output_dir = output_dir.resolve()
        if output_dir.exists() and any(output_dir.iterdir()):
            raise FileExistsError(f"DDGIApp validation output directory is not empty: {output_dir}")
        output_dir.mkdir(parents=True, exist_ok=True)
        screenshot_path = output_dir / "ddgi-app.png"
        log_path = output_dir / "run.log"
        evidence_path = output_dir / "evidence.json"
        isolated_imgui_path = output_dir / "imgui.ini"
        installed_imgui_path = app.parent / "imgui.ini"
        installed_imgui_before = installed_imgui_path.read_bytes() if installed_imgui_path.is_file() else None
        shader_cache = output_dir / "shader-cache"
        shader_cache.mkdir(parents=True, exist_ok=True)
        environment = os.environ.copy()
        environment["EVOENGINE_SHADER_CACHE_DIR"] = str(shader_cache)
        environment["EVOENGINE_IMGUI_INI_PATH"] = str(isolated_imgui_path)

        command = [
            str(app),
            "--player",
            "--width",
            str(args.width),
            "--height",
            str(args.height),
            "--screenshot-warmup-frames",
            str(args.warmup_frames),
            "--max-load-frames",
            "30000",
            "--screenshot",
            str(screenshot_path),
        ]
        print(f"DDGIApp validation ({args.config}): {' '.join(command)}")
        try:
            completed = subprocess.run(
                command,
                cwd=app.parent,
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
            log_path.write_text(output, encoding="utf-8")
            raise TimeoutError(f"Installed DDGIApp exceeded the {args.timeout:g}-second timeout.") from error

        log_path.write_text(output, encoding="utf-8")
        print(output, end="")
        installed_imgui_after = installed_imgui_path.read_bytes() if installed_imgui_path.is_file() else None
        if installed_imgui_after != installed_imgui_before:
            raise RuntimeError(f"Installed DDGIApp ImGui layout changed during validation: {installed_imgui_path}")
        if completed.returncode != 0:
            raise RuntimeError(f"Installed DDGIApp exited with code {completed.returncode}; log={log_path}")
        fatal_line = next(
            (line for line in output.splitlines() if any(pattern.search(line) for pattern in FATAL_PATTERNS)), None
        )
        if fatal_line:
            raise RuntimeError(f"Fatal runtime output: {fatal_line}")
        for marker in (
            "EVOENGINE_VULKAN_VALIDATION enabled",
            "EVOENGINE_VULKAN_SYNCHRONIZATION_VALIDATION enabled",
            f"DDGI_APP_DDGI_READY resolution={args.width}x{args.height}",
            "DDGI_APP_SCREENSHOT_CAPTURED",
            "DDGI_APP_SHUTDOWN_COMPLETE",
            "DDGI_APP_RESULT passed",
        ):
            if marker not in output:
                raise RuntimeError(f"Missing runtime marker {marker!r}; log={log_path}")

        if not screenshot_path.is_file() or screenshot_path.stat().st_size == 0:
            raise RuntimeError(f"DDGIApp did not create a screenshot: {screenshot_path}")
        image = read_image(screenshot_path)
        if not isinstance(image, PngImage):
            raise RuntimeError(f"DDGIApp screenshot is not a PNG: {screenshot_path}")
        if (image.width, image.height) != (args.width, args.height):
            raise RuntimeError(
                f"DDGIApp screenshot has {image.width}x{image.height}; expected {args.width}x{args.height}."
            )
        non_black_pixels = 0
        luminance_sum = 0.0
        for offset in range(0, len(image.rgba), 4):
            red, green, blue = image.rgba[offset : offset + 3]
            non_black_pixels += red != 0 or green != 0 or blue != 0
            luminance_sum += red * 0.2126 + green * 0.7152 + blue * 0.0722
        if non_black_pixels == 0 or luminance_sum <= 0:
            raise RuntimeError(f"DDGIApp screenshot is black: {screenshot_path}")

        evidence = {
            "schema_version": 1,
            "configuration": args.config,
            "launch_count": 1,
            "command": command,
            "application": str(app),
            "application_sha256": sha256(app),
            "resolution": [image.width, image.height],
            "warmup_frames": args.warmup_frames,
            "metrics": {
                "non_black_pixels": non_black_pixels,
                "mean_luminance_8bit": luminance_sum / (image.width * image.height),
                "benchmark_gate": "four_frozen_m2_holdouts",
            },
            "artifacts": [
                {
                    "file": screenshot_path.name,
                    "bytes": screenshot_path.stat().st_size,
                    "sha256": sha256(screenshot_path),
                },
                {"file": log_path.name, "bytes": log_path.stat().st_size, "sha256": sha256(log_path)},
            ],
            "installed_imgui_layout_unchanged": True,
            "passed": True,
        }
        evidence_path.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
        print(f"DDGIApp validation passed in one launch; evidence={evidence_path}")
        return 0
    except Exception as error:
        print(f"DDGIApp validation failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
