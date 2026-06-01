#!/usr/bin/env python3
"""Generate the EvoEngine Visual Studio solution with CMake presets."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def format_command(command: list[str]) -> str:
    if os.name == "nt":
        return subprocess.list2cmdline(command)
    return shlex.join(command)


def run_step(name: str, command: list[str]) -> None:
    print(f"\n==> {name}", flush=True)
    print(format_command(command), flush=True)
    completed = subprocess.run(command, cwd=repo_root())
    if completed.returncode != 0:
        raise SystemExit(completed.returncode)


def load_cmake_presets(root: Path) -> dict[str, Any]:
    presets_path = root / "CMakePresets.json"
    if not presets_path.exists():
        raise SystemExit(f"Missing CMakePresets.json: {presets_path}")
    return json.loads(presets_path.read_text(encoding="utf-8"))


def expand_preset_path(value: str, root: Path) -> Path:
    expanded = value.replace("${sourceDir}", str(root))
    path = Path(expanded)
    if not path.is_absolute():
        path = root / path
    return path.resolve()


def configure_preset(presets: dict[str, Any], name: str) -> dict[str, Any]:
    for preset in presets.get("configurePresets", []):
        if preset.get("name") == name:
            return preset
    raise SystemExit(f"Configure preset not found: {name}")


def build_dir_for_preset(root: Path, preset_name: str) -> Path:
    preset = configure_preset(load_cmake_presets(root), preset_name)
    binary_dir = preset.get("binaryDir")
    if not binary_dir:
        raise SystemExit(f"Configure preset '{preset_name}' does not define binaryDir.")
    return expand_preset_path(binary_dir, root)


def clean_build_dir(root: Path, build_dir: Path) -> None:
    if not build_dir.exists():
        return

    allowed_root = (root / "out" / "build").resolve()
    try:
        build_dir.resolve().relative_to(allowed_root)
    except ValueError:
        raise SystemExit(
            f"Refusing to clean build directory outside {allowed_root}: {build_dir}"
        )

    print(f"Cleaning {build_dir}")
    shutil.rmtree(build_dir)


def find_solution_file(build_dir: Path) -> Path | None:
    solutions = sorted(build_dir.glob("*.sln"))
    if solutions:
        return solutions[0]
    modern_solutions = sorted(build_dir.glob("*.slnx"))
    if modern_solutions:
        return modern_solutions[0]
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the EvoEngine Visual Studio solution without building binaries."
    )
    parser.add_argument(
        "--preset",
        default="vs2026-x64",
        help="CMake configure preset to use. Defaults to vs2026-x64.",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Delete the preset build directory before configuring.",
    )
    parser.add_argument(
        "--cmake-arg",
        action="append",
        default=[],
        help="Extra argument passed to CMake configure. May be repeated.",
    )
    parser.add_argument("--verbose", action="store_true", help="Pass verbose logging to CMake.")
    return parser.parse_args()


def main() -> int:
    if os.name != "nt":
        print(
            "Scripts/build_project.py targets Windows Visual Studio presets. "
            "Use direct CMake commands on non-Windows platforms.",
            file=sys.stderr,
        )
        return 1

    args = parse_args()
    root = repo_root()
    build_dir = build_dir_for_preset(root, args.preset)

    if args.clean:
        clean_build_dir(root, build_dir)

    command = ["cmake", "--preset", args.preset, "-DBUILD_TESTING=ON"]
    if args.verbose:
        command.append("--log-level=VERBOSE")
    command.extend(args.cmake_arg)
    run_step("Configure Visual Studio project", command)

    solution_path = find_solution_file(build_dir)
    if solution_path is None:
        print(f"\nVisual Studio solution: {build_dir / 'EvoEngine.sln'}")
        print("Warning: expected solution file was not found.", file=sys.stderr)
        return 1
    print(f"\nVisual Studio solution: {solution_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
