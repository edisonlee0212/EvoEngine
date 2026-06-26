#!/usr/bin/env python3
"""Generate static launcher preview PNGs for EvoEngine demo profiles."""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from pathlib import Path

DEMO_PROFILE_IDS = (
    "rendering",
    "ddgi",
    "ecosyslab",
    "digital-agriculture",
    "lsystem",
    "procedural-galaxy",
)
DEFAULT_WARMUP_FRAMES = 8
PROFILE_WARMUP_FRAMES = {
    "ddgi": 360,
}


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_editor_path(root: Path) -> Path:
    return root / "out" / "install" / "vs2026-x64" / "bin" / (
        "EvoEngineEditor.exe" if os.name == "nt" else "EvoEngineEditor"
    )


def format_command(command: list[str]) -> str:
    return subprocess.list2cmdline(command) if os.name == "nt" else shlex.join(command)


def parse_profiles(value: str) -> list[str]:
    profiles = [profile.strip() for profile in value.split(",") if profile.strip()]
    invalid_profiles = [profile for profile in profiles if profile not in DEMO_PROFILE_IDS]
    if invalid_profiles:
        raise argparse.ArgumentTypeError("Unknown demo profile: " + ", ".join(invalid_profiles))
    return profiles


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--editor", type=Path, default=default_editor_path(root), help="Path to EvoEngineEditor.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=root / "Resources" / "Launcher" / "DemoPreviews",
        help="Directory where preview PNGs are written.",
    )
    parser.add_argument(
        "--profiles",
        type=parse_profiles,
        default=list(DEMO_PROFILE_IDS),
        help="Comma-separated demo profile IDs to generate. Defaults to all profiles.",
    )
    parser.add_argument("--width", type=int, default=1280, help="Preview image width.")
    parser.add_argument("--height", type=int, default=720, help="Preview image height.")
    parser.add_argument(
        "--warmup-frames",
        type=int,
        help="Rendered frames before capture. Defaults to 8, or 360 for DDGI.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    editor = args.editor.resolve()
    if not editor.exists():
        print(f"Missing EvoEngineEditor: {editor}", file=sys.stderr)
        return 1
    if args.width <= 0 or args.height <= 0:
        print("--width and --height must be positive.", file=sys.stderr)
        return 1
    if args.warmup_frames is not None and args.warmup_frames < 0:
        print("--warmup-frames must be non-negative.", file=sys.stderr)
        return 1

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    for profile_id in args.profiles:
        warmup_frames = (
            args.warmup_frames
            if args.warmup_frames is not None
            else PROFILE_WARMUP_FRAMES.get(profile_id, DEFAULT_WARMUP_FRAMES)
        )
        output_path = output_dir / f"{profile_id}.png"
        command = [
            str(editor),
            "--demo",
            profile_id,
            "--editor",
            "--capture-demo-preview",
            str(output_path),
            "--preview-width",
            str(args.width),
            "--preview-height",
            str(args.height),
            "--preview-warmup-frames",
            str(warmup_frames),
        ]
        print(format_command(command), flush=True)
        completed = subprocess.run(command, cwd=editor.parent)
        if completed.returncode != 0:
            return completed.returncode
        if not output_path.exists() or output_path.stat().st_size == 0:
            print(f"Preview was not written: {output_path}", file=sys.stderr)
            return 1
        print(f"Wrote {output_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
