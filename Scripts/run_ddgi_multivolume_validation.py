#!/usr/bin/env python3
"""Validate DDGI multi-volume lifecycle behavior in one installed-editor launch."""

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
    re.compile(r"EVOENGINE_DDGI_MULTI_VOLUME_ERROR", re.IGNORECASE),
)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="RelWithDebInfo", help="Installed build configuration label.")
    parser.add_argument("--width", type=int, default=1920, help="Render width; must be 1920.")
    parser.add_argument("--height", type=int, default=1080, help="Render height; must be 1080.")
    parser.add_argument("--output-dir", type=Path, default=root / "out/ddgi-validation-m7-multivolume")
    parser.add_argument("--timeout", type=float, default=900.0, help="Maximum launch duration in seconds.")
    parser.add_argument("--editor", type=Path, help="Installed EvoEngineEditor executable override.")
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_png(path: Path, width: int, height: int) -> dict[str, object]:
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"Missing or empty DDGI multi-volume capture: {path}")
    image = read_image(path)
    if not isinstance(image, PngImage):
        raise RuntimeError(f"DDGI multi-volume capture is not a PNG: {path}")
    if (image.width, image.height) != (width, height):
        raise RuntimeError(
            f"DDGI multi-volume capture has {image.width}x{image.height}; expected {width}x{height}: {path}"
        )
    if not any(image.rgba[channel] for channel in range(0, len(image.rgba), 4)) and not any(
        image.rgba[channel] for channel in range(1, len(image.rgba), 4)
    ) and not any(image.rgba[channel] for channel in range(2, len(image.rgba), 4)):
        raise RuntimeError(f"DDGI multi-volume capture is black: {path}")
    return {
        "file": path.name,
        "width": image.width,
        "height": image.height,
        "bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def main() -> int:
    args = parse_args()
    try:
        if (args.width, args.height) != (1920, 1080):
            raise ValueError("The DDGI multi-volume closeout gate requires exactly 1920x1080.")
        if args.timeout <= 0:
            raise ValueError("--timeout must be positive.")

        root = repo_root()
        editor = (args.editor or root / "out/install/vs2026-x64/bin/EvoEngineEditor.exe").resolve()
        if not editor.is_file():
            raise FileNotFoundError(f"Installed editor not found: {editor}")
        output_dir = args.output_dir if args.output_dir.is_absolute() else root / args.output_dir
        output_dir = output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        for name in ("initial.png", "scrolled.png", "removed.png", "report.json", "run.log", "evidence.json"):
            (output_dir / name).unlink(missing_ok=True)

        shader_cache = output_dir / "shader-cache"
        shader_cache.mkdir(parents=True, exist_ok=True)
        imgui_path = output_dir / "imgui.ini"
        imgui_path.unlink(missing_ok=True)
        environment = os.environ.copy()
        environment["EVOENGINE_DDGI_MULTI_VOLUME_EVIDENCE"] = str(output_dir)
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
        print(f"DDGI multi-volume validation ({args.config}): {' '.join(command)}")
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
            "EVOENGINE_DDGI_MULTI_VOLUME_REPORT",
            "passed=true",
            "EVOENGINE_DDGI_MULTI_VOLUME_SHUTDOWN_COMPLETE",
        ):
            if marker not in output:
                raise RuntimeError(f"Missing runtime marker {marker!r}; log={log_path}")

        report_path = output_dir / "report.json"
        if not report_path.is_file():
            raise RuntimeError(f"DDGI multi-volume report was not written: {report_path}")
        report = json.loads(report_path.read_text(encoding="utf-8"))
        contract = report.get("contract", {})
        if report.get("schema_version") != 1 or report.get("passed") is not True:
            raise RuntimeError("DDGI multi-volume report did not pass its schema contract.")
        if contract.get("resolution") != [1920, 1080] or contract.get("launch_count") != 1:
            raise RuntimeError("DDGI multi-volume report has the wrong resolution or launch count.")
        if contract.get("vulkan_rt_pipeline") is not True or contract.get("graphics_validation") is not True:
            raise RuntimeError("DDGI multi-volume report lacks the required RT-pipeline validation contract.")
        checks = report.get("checks")
        if not isinstance(checks, dict) or not checks or any(value is not True for value in checks.values()):
            failed = sorted(name for name, value in (checks or {}).items() if value is not True)
            raise RuntimeError("DDGI multi-volume checks failed: " + ", ".join(failed))

        runtime = report.get("runtime", {})
        initial = runtime.get("initial")
        scrolled = runtime.get("scrolled")
        removed = runtime.get("removed")
        if not isinstance(initial, list) or not isinstance(scrolled, list) or not isinstance(removed, list):
            raise RuntimeError("DDGI multi-volume report is missing runtime arrays.")
        if (len(initial), len(scrolled), len(removed)) != (8, 8, 7):
            raise RuntimeError("DDGI multi-volume runtime counts are not 8/8/7.")
        if tuple(sum(int(volume["probe_count"]) for volume in phase) for phase in (initial, scrolled, removed)) != (
            750,
            750,
            407,
        ):
            raise RuntimeError("DDGI multi-volume runtime probe totals are not 750/750/407.")
        created_order = report.get("created_order")
        expected_order = report.get("expected_runtime_order")
        if not isinstance(created_order, list) or len(created_order) != 8 or not isinstance(expected_order, list):
            raise RuntimeError("DDGI multi-volume report is missing its authored/runtime ordering evidence.")
        if [volume["stable_entity_id"] for volume in initial] != expected_order:
            raise RuntimeError("Initial DDGI runtime order does not match the declared policy order.")
        if [volume["stable_entity_id"] for volume in scrolled] != expected_order:
            raise RuntimeError("Scrolling changed DDGI runtime order.")
        if [volume["stable_entity_id"] for volume in removed] != expected_order[1:]:
            raise RuntimeError("DDGI removal did not retain the expected survivor order.")
        for phase in (initial, scrolled, removed):
            if any(
                volume.get("history_valid") is not True
                or volume.get("contributes_lighting") is not True
                or volume.get("resources_ready") is not True
                for volume in phase
            ):
                raise RuntimeError("A reported DDGI runtime is not ready, history-valid, and contributing.")
        initial_resources = {volume["stable_entity_id"]: volume["resource_ids"] for volume in initial}
        flattened_resources = [resource for resources in initial_resources.values() for resource in resources]
        if any(not isinstance(resource, int) or resource == 0 for resource in flattened_resources) or len(
            set(flattened_resources)
        ) != len(flattened_resources):
            raise RuntimeError("Initial DDGI runtime resources are null or shared between volumes.")
        if any(initial_resources[volume["stable_entity_id"]] != volume["resource_ids"] for volume in scrolled):
            raise RuntimeError("Scrolling rebuilt a DDGI survivor resource set.")
        if any(initial_resources[volume["stable_entity_id"]] != volume["resource_ids"] for volume in removed):
            raise RuntimeError("Removal rebuilt a surviving DDGI resource set.")
        scrolling_id = created_order[1]
        moved = next((volume for volume in scrolled if volume["stable_entity_id"] == scrolling_id), None)
        if moved is None or moved.get("last_scroll_delta") != [1, 0, 0] or moved.get("scroll_offset") != [1, 0, 0]:
            raise RuntimeError("The scrolling volume did not report a persistent one-cell offset.")
        if any(
            volume.get("last_scroll_delta") != [0, 0, 0]
            for volume in scrolled
            if volume["stable_entity_id"] != scrolling_id
        ):
            raise RuntimeError("A non-scrolling volume reported a scroll delta.")
        if created_order[3] in {volume["stable_entity_id"] for volume in removed}:
            raise RuntimeError("The removed dense DDGI volume remains in runtime state.")

        captures = [validate_png(output_dir / name, args.width, args.height) for name in (
            "initial.png",
            "scrolled.png",
            "removed.png",
        )]
        artifacts = captures + [
            {
                "file": report_path.name,
                "bytes": report_path.stat().st_size,
                "sha256": sha256(report_path),
            },
            {"file": log_path.name, "bytes": log_path.stat().st_size, "sha256": sha256(log_path)},
        ]
        evidence = {
            "schema_version": 1,
            "configuration": args.config,
            "launch_count": 1,
            "command": command,
            "editor": str(editor),
            "checks": checks,
            "artifacts": artifacts,
            "passed": True,
        }
        evidence_path = output_dir / "evidence.json"
        evidence_path.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
        print(f"DDGI multi-volume validation passed in one launch; evidence={evidence_path}")
        return 0
    except Exception as error:
        print(f"DDGI multi-volume validation failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
