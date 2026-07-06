#!/usr/bin/env python3
"""Run the local CSM milestone closeout gate."""

from __future__ import annotations

import argparse
import os
import re
import shlex
import subprocess
import sys
from datetime import datetime
from pathlib import Path

ERROR_PATTERNS = (
    re.compile(r"\bfatal\b", re.IGNORECASE),
    re.compile(r"\bunhandled exception\b", re.IGNORECASE),
    re.compile(r"\baccess violation\b", re.IGNORECASE),
    re.compile(r"\bassert(?:ion)? failed\b", re.IGNORECASE),
    re.compile(r"\bvalidation error\b", re.IGNORECASE),
    re.compile(r"\bdevice[- ]lost\b", re.IGNORECASE),
    re.compile(r"\bVK_ERROR_DEVICE_LOST\b", re.IGNORECASE),
    re.compile(r"\bEVOENGINE_(?:APP_TEST_RESULT failed|FATAL|ERROR)\b", re.IGNORECASE),
)

SHADOW_DEBUG_CAPTURE_MODES = ("cascade-index", "light-uv", "light-depth", "atlas-uv", "texel-density")


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def executable_name(name: str) -> str:
    return f"{name}.exe" if os.name == "nt" else name


def default_editor_path(root: Path) -> Path:
    return root / "out" / "install" / "vs2026-x64" / "bin" / executable_name("EvoEngineEditor")


def default_build_dir(root: Path) -> Path:
    return root / "out" / "build" / "vs2026-x64"


def format_command(command: list[str]) -> str:
    return subprocess.list2cmdline(command) if os.name == "nt" else shlex.join(command)


def run_step(name: str, command: list[str], cwd: Path) -> None:
    print(f"\n==> {name}", flush=True)
    print(format_command(command), flush=True)
    completed = subprocess.run(command, cwd=cwd)
    if completed.returncode != 0:
        raise SystemExit(completed.returncode)


def run_logged_step(name: str, command: list[str], cwd: Path, log_path: Path, timeout: int | None = None) -> str:
    print(f"\n==> {name}", flush=True)
    print(format_command(command), flush=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    completed = subprocess.run(
        command,
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout,
    )
    output = completed.stdout or ""
    log_path.write_text(f"$ {format_command(command)}\n\n{output}", encoding="utf-8")
    if output:
        print(output, end="" if output.endswith("\n") else "\n")
    if completed.returncode != 0:
        raise SystemExit(completed.returncode)
    assert_log_is_clean(log_path, output)
    return output


def assert_log_is_clean(log_path: Path, output: str) -> None:
    for line in output.splitlines():
        if any(pattern.search(line) for pattern in ERROR_PATTERNS):
            raise SystemExit(f"Error marker found in {log_path}: {line}")


def ensure_bistro_resources(root: Path, editor: Path, skip_generate: bool) -> None:
    bistro_project = root / "Resources" / ".generated" / "EvoEngine-DemoProjects" / "Bistro" / "Bistro.eveproj"
    if bistro_project.exists():
        print(f"Bistro project: {bistro_project}", flush=True)
        return
    if skip_generate:
        raise SystemExit(f"Missing Bistro project: {bistro_project}")
    run_step(
        "Generate Bistro demo resources",
        [
            sys.executable,
            str(root / "Scripts" / "generate_bistro_demo.py"),
            "--editor",
            str(editor),
        ],
        root,
    )


def smoke_demo(editor: Path, demo_id: str, seconds: int, log_path: Path, extra_args: list[str] | None = None) -> None:
    command = [str(editor), "--demo", demo_id, "--editor"]
    if extra_args:
        command.extend(extra_args)
    print(f"\n==> Smoke {demo_id} for {seconds} seconds", flush=True)
    print(format_command(command), flush=True)
    process = subprocess.Popen(
        command,
        cwd=editor.parent,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    timed_out = False
    try:
        output, _ = process.communicate(timeout=seconds)
    except subprocess.TimeoutExpired:
        timed_out = True
        process.terminate()
        try:
            output, _ = process.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=10)

    output = output or ""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(f"$ {format_command(command)}\n\n{output}", encoding="utf-8")
    if output:
        print(output, end="" if output.endswith("\n") else "\n")
    assert_log_is_clean(log_path, output)
    if not timed_out:
        raise SystemExit(
            f"{demo_id} exited after less than {seconds} seconds with code {process.returncode}; see {log_path}"
        )
    print(f"{demo_id} stayed up for {seconds} seconds.", flush=True)


def capture_preview(
    editor: Path,
    demo_id: str,
    render_mode: str,
    output_path: Path,
    width: int,
    height: int,
    warmup_frames: int,
    log_path: Path,
    extra_args: list[str] | None = None,
) -> None:
    command = [
        str(editor),
        "--demo",
        demo_id,
        "--editor",
        "--capture-demo-preview",
        str(output_path),
        "--preview-render-mode",
        render_mode,
        "--preview-width",
        str(width),
        "--preview-height",
        str(height),
        "--preview-warmup-frames",
        str(warmup_frames),
        "--preview-deterministic",
    ]
    if extra_args:
        command.extend(extra_args)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    run_logged_step(f"Capture {demo_id} {render_mode}", command, editor.parent, log_path, timeout=900)
    if not output_path.exists() or output_path.stat().st_size == 0:
        raise SystemExit(f"Preview was not written: {output_path}")


def write_report(
    report_path: Path,
    milestone: str,
    editor: Path,
    smoke_seconds: int,
    shadow_split_lambda: float,
    shadow_cascade_transition_width: float | None,
    shadow_distance_fade: float | None,
    shadow_map_resolution: str,
    preview_paths: list[Path],
    log_paths: list[Path],
) -> None:
    preview_lines = "\n".join(f"- `{path}`" for path in preview_paths)
    log_lines = "\n".join(f"- `{path}`" for path in log_paths)
    shadow_resolution_command_suffix = ""
    if not shadow_map_resolution.startswith("default"):
        shadow_resolution_command_suffix = f" --shadow-map-resolution {shadow_map_resolution}"
    report_path.write_text(
        f"""# {milestone} CSM Validation Report

Generated: {datetime.now().isoformat(timespec="seconds")}

Editor:
`{editor}`

Smoke launch commands:
- `{editor} --demo bistro --editor{shadow_resolution_command_suffix}`
- `{editor} --demo rendering --editor{shadow_resolution_command_suffix}`

Smoke duration:
{smoke_seconds} seconds per demo.

Shadow fit policy:
Legacy Stable

Shadow split policy:
Practical Log/Uniform, lambda={shadow_split_lambda:.3f}

Shadow sampling:
PCF, radius_texels=100 x light_size

Shadow map resolution:
{shadow_map_resolution}

Shadow fade override:
{f"cascade_transition_width={shadow_cascade_transition_width:.3f}" if shadow_cascade_transition_width is not None else "default"}{f" distance_fade={shadow_distance_fade:.3f}" if shadow_distance_fade is not None else ""}

Preview captures:
{preview_lines}

Logs:
{log_lines}

Visual check checklist:
- Bistro cascade seams: road curb, street edge, building facade transitions.
- Bistro near/far sharpness: curb stones, plant pot, motorbike front, distant building shadows.
- Bistro grazing surfaces: curb edges and shallow-angle road surfaces.
- Rendering scene coverage: near primitives, far surfaces, camera rotation, and camera translation.
- Alpha-tested foliage: Bistro trees and plant leaves where visible in the captured view.
""",
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--milestone", default="M1", help="Milestone label used for output folders.")
    parser.add_argument("--config", default="RelWithDebInfo", help="CMake build configuration.")
    parser.add_argument("--build-dir", type=Path, default=default_build_dir(root), help="CMake build directory.")
    parser.add_argument("--editor", type=Path, default=default_editor_path(root), help="Installed EvoEngineEditor path.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=root / "out" / "csm-validation",
        help="Directory where validation logs and captures are written.",
    )
    parser.add_argument("--width", type=int, default=1920, help="Preview capture width.")
    parser.add_argument("--height", type=int, default=1080, help="Preview capture height.")
    parser.add_argument("--warmup-frames", type=int, default=1800, help="Raster preview warmup frames.")
    parser.add_argument("--shadow-debug-warmup-frames", type=int, default=120, help="Shadow diagnostic warmup frames.")
    parser.add_argument("--shadow-debug-cascade", type=int, default=0, help="Selected cascade for shadow diagnostics.")
    parser.add_argument("--shadow-debug-light", type=int, default=0, help="Selected directional light for diagnostics.")
    parser.add_argument("--shadow-split-lambda", type=float, default=0.5, help="Practical split lambda.")
    parser.add_argument(
        "--shadow-map-resolution",
        choices=("low", "medium", "high", "very-high"),
        help="Optional startup shadow map resolution quality for editor smoke and capture commands.",
    )
    parser.add_argument(
        "--shadow-cascade-transition-width",
        type=float,
        help="Optional cascade transition width in positive linear view-depth units.",
    )
    parser.add_argument(
        "--shadow-distance-fade",
        type=float,
        help="Optional final max-shadow-distance fade width in positive linear view-depth units.",
    )
    parser.add_argument("--smoke-seconds", type=int, default=30, help="Smoke-test duration per demo.")
    parser.add_argument("--skip-format", action="store_true", help="Skip C++ format check.")
    parser.add_argument("--skip-build", action="store_true", help="Skip EvoEngineEditor build.")
    parser.add_argument("--skip-install", action="store_true", help="Skip app install step.")
    parser.add_argument("--skip-smoke", action="store_true", help="Skip 30-second demo smoke tests.")
    parser.add_argument("--skip-capture", action="store_true", help="Skip preview captures.")
    parser.add_argument("--capture-shadow-diagnostics", action="store_true", help="Capture CSM diagnostic previews.")
    parser.add_argument("--skip-bistro-generate", action="store_true", help="Do not auto-generate missing Bistro assets.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = repo_root()
    milestone_dir = args.output_dir.resolve() / args.milestone.lower()
    log_dir = milestone_dir / "logs"
    preview_dir = milestone_dir / "previews"
    editor = args.editor.resolve()
    build_dir = args.build_dir.resolve()
    if args.width <= 0 or args.height <= 0:
        raise SystemExit("--width and --height must be positive.")
    if args.warmup_frames < 0:
        raise SystemExit("--warmup-frames must be non-negative.")
    if args.shadow_debug_warmup_frames < 0:
        raise SystemExit("--shadow-debug-warmup-frames must be non-negative.")
    args.shadow_debug_cascade = max(0, min(args.shadow_debug_cascade, 3))
    args.shadow_debug_light = max(0, args.shadow_debug_light)
    args.shadow_split_lambda = max(0.0, min(args.shadow_split_lambda, 1.0))
    if args.shadow_cascade_transition_width is not None:
        args.shadow_cascade_transition_width = max(0.0, args.shadow_cascade_transition_width)
    if args.shadow_distance_fade is not None:
        args.shadow_distance_fade = max(0.0, args.shadow_distance_fade)
    if args.smoke_seconds <= 0:
        raise SystemExit("--smoke-seconds must be positive.")
    shadow_map_resolution_args: list[str] = []
    shadow_map_resolution_report = "default (High, 4096 x 4096)"
    if args.shadow_map_resolution:
        shadow_map_resolution_args = ["--shadow-map-resolution", args.shadow_map_resolution]
        shadow_map_resolution_report = args.shadow_map_resolution

    if not args.skip_format:
        run_step(
            "C++ format check",
            [
                sys.executable,
                str(root / "Scripts" / "format_cpp.py"),
                "--check",
                "--root",
                "EvoEngine_SDK",
                "--root",
                "EvoEngine_App",
                "--root",
                "EvoEngine_Tests",
            ],
            root,
        )
    if not args.skip_build:
        run_step(
            "Build EvoEngineEditor",
            ["cmake", "--build", str(build_dir), "--config", args.config, "--target", "EvoEngineEditor"],
            root,
        )
    if not args.skip_install:
        run_step(
            "Install apps",
            [
                sys.executable,
                str(root / "Scripts" / "install_apps.py"),
                "--config",
                args.config,
                "--no-open",
                "--incremental",
            ],
            root,
        )
    if not editor.exists():
        raise SystemExit(f"Missing EvoEngineEditor: {editor}")
    ensure_bistro_resources(root, editor, args.skip_bistro_generate)

    log_paths: list[Path] = []
    if not args.skip_smoke:
        for demo_id in ("bistro", "rendering"):
            log_path = log_dir / f"{demo_id}-smoke.log"
            smoke_demo(editor, demo_id, args.smoke_seconds, log_path, shadow_map_resolution_args)
            log_paths.append(log_path)

    preview_paths: list[Path] = []
    if not args.skip_capture:
        preview_override_args = [
            *shadow_map_resolution_args,
            "--preview-shadow-split-lambda",
            str(args.shadow_split_lambda),
        ]
        if args.shadow_cascade_transition_width is not None:
            preview_override_args.extend(
                ["--preview-shadow-cascade-transition-width", str(args.shadow_cascade_transition_width)]
            )
        if args.shadow_distance_fade is not None:
            preview_override_args.extend(["--preview-shadow-distance-fade", str(args.shadow_distance_fade)])
        for demo_id in ("bistro", "rendering"):
            preview_path = preview_dir / f"{demo_id}-rasterization-{args.width}x{args.height}.png"
            log_path = log_dir / f"{demo_id}-rasterization-capture.log"
            capture_preview(
                editor,
                demo_id,
                "rasterization",
                preview_path,
                args.width,
                args.height,
                args.warmup_frames,
                log_path,
                preview_override_args,
            )
            preview_paths.append(preview_path)
            log_paths.append(log_path)
        if args.capture_shadow_diagnostics:
            for demo_id in ("bistro", "rendering"):
                for debug_mode in SHADOW_DEBUG_CAPTURE_MODES:
                    preview_path = (
                        preview_dir
                        / f"{demo_id}-shadow-{debug_mode}-c{args.shadow_debug_cascade}-{args.width}x{args.height}.png"
                    )
                    log_path = log_dir / f"{demo_id}-shadow-{debug_mode}-capture.log"
                    capture_preview(
                        editor,
                        demo_id,
                        "rasterization",
                        preview_path,
                        args.width,
                        args.height,
                        args.shadow_debug_warmup_frames,
                        log_path,
                        [
                            *preview_override_args,
                            "--preview-shadow-debug",
                            debug_mode,
                            "--preview-shadow-debug-cascade",
                            str(args.shadow_debug_cascade),
                            "--preview-shadow-debug-light",
                            str(args.shadow_debug_light),
                        ],
                    )
                    preview_paths.append(preview_path)
                    log_paths.append(log_path)

    report_path = milestone_dir / "validation-report.md"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    write_report(
        report_path,
        args.milestone,
        editor,
        args.smoke_seconds,
        args.shadow_split_lambda,
        args.shadow_cascade_transition_width,
        args.shadow_distance_fade,
        shadow_map_resolution_report,
        preview_paths,
        log_paths,
    )
    print(f"\nValidation report: {report_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
