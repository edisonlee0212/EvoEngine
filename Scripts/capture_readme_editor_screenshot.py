#!/usr/bin/env python3
"""Capture the legacy DemoApp editor layout used by the README gallery."""

from __future__ import annotations

import argparse
import subprocess
import textwrap
import threading
from pathlib import Path


DEFAULT_BUILD_DIR = Path("out/build/vs2026-x64")
DEFAULT_ARTIFACT = Path("out/readme-screenshots/RenderingDemo.png")
README_IMAGE = Path("Resources/GitHub/RenderingDemo.png")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true", help="overwrite Resources/GitHub/RenderingDemo.png")
    parser.add_argument("--build", dest="build", action="store_true", default=True, help="build DemoApp first")
    parser.add_argument("--no-build", dest="build", action="store_false", help="skip building DemoApp")
    parser.add_argument("--preset", default="vs2026-x64", help="CMake preset used to enable the DemoApp target")
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument("--config", default="RelWithDebInfo")
    parser.add_argument("--output", type=Path, help="capture output path")
    parser.add_argument("--demo-setup", default="Rendering", choices=("Rendering", "CornellBox", "ThinWall"))
    parser.add_argument("--inspect-render-layer", action="store_true", help="open the RenderLayer inspection window")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--warmup-frames", type=int, default=30)
    return parser.parse_args()


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def run_build(root: Path, build_dir: Path, config: str, preset: str) -> None:
    configure_command = ["cmake", "--preset", preset, "-DEvoEngine_App-DemoApp=ON"]
    subprocess.run(configure_command, cwd=root, check=True)
    command = ["cmake", "--build", str(root / build_dir), "--config", config, "--target", "DemoApp"]
    subprocess.run(command, cwd=root, check=True)


def demo_app_path(root: Path, build_dir: Path, config: str) -> Path:
    path = root / build_dir / "EvoEngine_App" / config / "DemoApp.exe"
    if not path.exists():
        raise FileNotFoundError(f"DemoApp executable was not found: {path}")
    return path


def yaml_path(path: Path) -> str:
    return path.resolve().as_posix()


def write_run_config(
    path: Path,
    output: Path,
    demo_setup: str,
    inspect_render_layer: bool,
    width: int,
    height: int,
    warmup_frames: int,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        textwrap.dedent(
            f"""\
            mode: editor_screenshot
            demo_setup: {demo_setup}
            screenshot_width: {width}
            screenshot_height: {height}
            inspect_render_layer: {str(inspect_render_layer).lower()}
            warmup_frames: {warmup_frames}
            max_load_frames: 30000
            exit_on_complete: true
            screenshot_file: "{yaml_path(output)}"
            """
        ),
        encoding="utf-8",
    )


def read_process_output(process: subprocess.Popen[str], lines: list[str]) -> threading.Thread:
    def reader() -> None:
        if process.stdout is None:
            return
        for line in process.stdout:
            lines.append(line.rstrip())
            print(line, end="")

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    return thread


def main() -> int:
    args = parse_args()
    root = repo_root()
    output = root / (README_IMAGE if args.apply else args.output or DEFAULT_ARTIFACT)
    build_dir = args.build_dir if args.build_dir.is_absolute() else root / args.build_dir

    if args.build:
        run_build(root, build_dir, args.config, args.preset)

    app_path = demo_app_path(root, build_dir, args.config)
    run_dir = root / "out/readme-screenshots/run"
    run_config = run_dir / "DemoApp.editor-screenshot.yaml"
    output.unlink(missing_ok=True)
    write_run_config(
        run_config,
        output,
        args.demo_setup,
        args.inspect_render_layer,
        args.width,
        args.height,
        args.warmup_frames,
    )

    process = subprocess.Popen(
        [str(app_path), "--run-config", str(run_config)],
        cwd=app_path.parent,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    output_lines: list[str] = []
    read_process_output(process, output_lines)
    try:
        exit_code = process.wait(timeout=args.timeout)
        if exit_code != 0:
            raise RuntimeError(f"DemoApp exited with code {exit_code}")
        if not output.exists() or output.stat().st_size == 0:
            raise RuntimeError(f"DemoApp did not create {output}")
    except Exception:
        process.terminate()
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            process.kill()
        raise

    print(f"Captured README editor screenshot: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
