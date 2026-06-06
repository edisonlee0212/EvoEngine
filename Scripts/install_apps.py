#!/usr/bin/env python3
"""Build and install EvoEngine app runtimes with CMake presets."""

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


def build_preset_exists(presets: dict[str, Any], name: str) -> bool:
    return any(preset.get("name") == name for preset in presets.get("buildPresets", []))


def build_dir_for_preset(root: Path, presets: dict[str, Any], preset_name: str) -> Path:
    preset = configure_preset(presets, preset_name)
    binary_dir = preset.get("binaryDir")
    if not binary_dir:
        raise SystemExit(f"Configure preset '{preset_name}' does not define binaryDir.")
    return expand_preset_path(binary_dir, root)


def install_dir_for_preset(root: Path, presets: dict[str, Any], preset_name: str) -> Path:
    preset = configure_preset(presets, preset_name)
    install_dir = preset.get("installDir")
    if not install_dir:
        cache_variables = preset.get("cacheVariables", {})
        install_dir = cache_variables.get("CMAKE_INSTALL_PREFIX")
    if not install_dir:
        raise SystemExit(f"Configure preset '{preset_name}' does not define installDir.")
    return expand_preset_path(install_dir, root)


def clean_install_dir(root: Path, install_dir: Path) -> None:
    if not install_dir.exists():
        return

    allowed_root = (root / "out" / "install").resolve()
    try:
        install_dir.resolve().relative_to(allowed_root)
    except ValueError:
        raise SystemExit(
            f"Refusing to clean install directory outside {allowed_root}: {install_dir}"
        )

    print(f"Cleaning {install_dir}")
    shutil.rmtree(install_dir)


def open_folder(path: Path) -> None:
    if os.name != "nt":
        return
    os.startfile(path)  # type: ignore[attr-defined]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build and install EvoEngine app runtimes through CMake presets."
    )
    parser.add_argument(
        "--preset",
        default="vs2026-x64",
        help="CMake configure preset to use. Defaults to vs2026-x64.",
    )
    parser.add_argument(
        "--config",
        default="RelWithDebInfo",
        choices=["Debug", "RelWithDebInfo"],
        help="Configuration to install. Defaults to RelWithDebInfo.",
    )
    parser.add_argument(
        "--no-open",
        action="store_true",
        help="Do not open the install output folder after a successful install.",
    )
    parser.add_argument(
        "--no-clean-install",
        action="store_true",
        help="Do not clean the install directory before building the install target.",
    )
    parser.add_argument(
        "--incremental",
        action="store_true",
        help=(
            "Reuse the existing build and install directories. Skips configure when "
            "the preset build cache already exists."
        ),
    )
    parser.add_argument(
        "--cmake-arg",
        action="append",
        default=[],
        help="Extra argument passed to CMake configure. May be repeated.",
    )
    parser.add_argument("--verbose", action="store_true", help="Pass verbose output to CMake.")
    return parser.parse_args()


def main() -> int:
    if os.name != "nt":
        print(
            "Scripts/install_apps.py targets Windows Visual Studio presets. "
            "Use direct CMake commands on non-Windows platforms.",
            file=sys.stderr,
        )
        return 1

    args = parse_args()
    root = repo_root()
    presets = load_cmake_presets(root)
    build_dir = build_dir_for_preset(root, presets, args.preset)
    install_dir = install_dir_for_preset(root, presets, args.preset)
    build_preset = f"install-{args.preset}-{args.config}"

    if not build_preset_exists(presets, build_preset):
        raise SystemExit(f"Build preset not found: {build_preset}")

    if args.incremental and not args.cmake_arg and (build_dir / "CMakeCache.txt").exists():
        print(f"Skipping configure; reusing {build_dir / 'CMakeCache.txt'}", flush=True)
    else:
        configure_command = ["cmake", "--preset", args.preset, "-DBUILD_TESTING=OFF"]
        if args.verbose:
            configure_command.append("--log-level=VERBOSE")
        configure_command.extend(args.cmake_arg)
        run_step("Configure Visual Studio project", configure_command)

    if not args.incremental and not args.no_clean_install:
        clean_install_dir(root, install_dir)

    build_command = ["cmake", "--build", "--preset", build_preset]
    if args.verbose:
        build_command.append("--verbose")
    run_step(f"Build and install {args.config}", build_command)

    print(f"\nInstall output: {install_dir}")
    print(f"App binaries: {install_dir / 'bin'}")
    print(f"Python runtime: {install_dir / 'python'}")
    if not args.no_open:
        open_folder(install_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
