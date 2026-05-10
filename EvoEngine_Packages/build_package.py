#!/usr/bin/env python3
"""Build one EvoEngine runtime package target.

Examples:
  python EvoEngine_Packages/build_package.py BillboardClouds
  python EvoEngine_Packages/build_package.py Universe --config Debug
  python EvoEngine_Packages/build_package.py --list
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path


PACKAGES_DIR = Path(__file__).resolve().parent
REPO_ROOT = PACKAGES_DIR.parent
PRESETS_FILE = REPO_ROOT / "CMakePresets.json"


def run_command(command: list[str], dry_run: bool) -> None:
    print(" ".join(command), flush=True)
    if dry_run:
        return
    subprocess.run(command, cwd=REPO_ROOT, check=True)


def read_presets() -> dict:
    if not PRESETS_FILE.exists():
        raise FileNotFoundError(f"Cannot find {PRESETS_FILE}")
    with PRESETS_FILE.open("r", encoding="utf-8") as file:
        return json.load(file)


def discover_packages() -> dict[str, dict[str, str]]:
    packages: dict[str, dict[str, str]] = {}
    for package_dir in sorted(PACKAGES_DIR.iterdir()):
        if not package_dir.is_dir() or package_dir.name.startswith("."):
            continue
        cmake_file = package_dir / "CMakeLists.txt"
        if not cmake_file.exists():
            continue

        cmake_text = cmake_file.read_text(encoding="utf-8", errors="ignore")
        match = re.search(r"add_library\s*\(\s*([A-Za-z0-9_]+Package)\b", cmake_text)
        target = match.group(1) if match else f"{package_dir.name}Package"
        package_name = target.removesuffix("Package")
        entry = {
            "name": package_name,
            "target": target,
            "path": str(package_dir.relative_to(REPO_ROOT)),
            "enable_variable": f"EVOENGINE_ENABLE_{package_name}_PACKAGE",
        }
        packages[package_name.lower()] = entry
        packages[target.lower()] = entry
    return packages


def select_configure_preset(presets: dict, requested: str | None) -> dict:
    configure_presets = presets.get("configurePresets", [])
    if requested:
        for preset in configure_presets:
            if preset.get("name") == requested:
                return preset
        raise ValueError(f"Unknown configure preset: {requested}")

    if sys.platform.startswith("win"):
        for preset in configure_presets:
            if preset.get("name") == "vs2026-x64":
                return preset

    if not configure_presets:
        raise ValueError("No configure presets found in CMakePresets.json")
    return configure_presets[0]


def expand_binary_dir(binary_dir: str, preset_name: str) -> Path:
    replacements = {
        "${sourceDir}": str(REPO_ROOT),
        "${sourceParentDir}": str(REPO_ROOT.parent),
        "${sourceDirName}": REPO_ROOT.name,
        "${presetName}": preset_name,
    }
    expanded = binary_dir
    for key, value in replacements.items():
        expanded = expanded.replace(key, value)
    return Path(expanded)


def list_packages(packages: dict[str, dict[str, str]]) -> None:
    unique_packages = {entry["name"]: entry for entry in packages.values()}
    for name in sorted(unique_packages):
        entry = unique_packages[name]
        print(f"{entry['name']:<20} target={entry['target']:<24} path={entry['path']}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Configure and build one EvoEngine runtime package.")
    parser.add_argument("package", nargs="?", help="Package name or target, for example BillboardClouds or BillboardCloudsPackage.")
    parser.add_argument("--list", action="store_true", help="List discovered runtime packages.")
    parser.add_argument("--preset", help="CMake configure preset. Defaults to vs2026-x64 on Windows, otherwise the first preset.")
    parser.add_argument("--config", default="RelWithDebInfo", help="Build configuration. Default: RelWithDebInfo.")
    parser.add_argument("--skip-configure", action="store_true", help="Skip the cmake configure step.")
    parser.add_argument("--install", action="store_true", help="Run cmake --install after building. This invokes the project install step.")
    parser.add_argument("--clean-first", action="store_true", help="Ask CMake to clean the package target before building it.")
    parser.add_argument("-j", "--parallel", type=int, help="Parallel build level passed to cmake --build.")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without executing them.")
    parser.add_argument(
        "--configure-arg",
        action="append",
        default=[],
        help="Extra argument forwarded to cmake configure. May be specified multiple times.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    packages = discover_packages()

    if args.list:
        list_packages(packages)
        return 0

    if not args.package:
        print("error: package is required unless --list is used", file=sys.stderr)
        return 2

    package_key = args.package.removesuffix("Package").lower()
    package = packages.get(package_key) or packages.get(args.package.lower())
    if not package:
        print(f"error: unknown package: {args.package}", file=sys.stderr)
        print("Available packages:", file=sys.stderr)
        list_packages(packages)
        return 2

    presets = read_presets()
    configure_preset = select_configure_preset(presets, args.preset)
    preset_name = configure_preset["name"]
    binary_dir = expand_binary_dir(configure_preset.get("binaryDir", f"${{sourceDir}}/out/build/{preset_name}"), preset_name)

    if not args.skip_configure:
        configure_command = [
            "cmake",
            "--preset",
            preset_name,
            f"-D{package['enable_variable']}=ON",
            *args.configure_arg,
        ]
        run_command(configure_command, args.dry_run)

    build_command = [
        "cmake",
        "--build",
        str(binary_dir),
        "--config",
        args.config,
        "--target",
        package["target"],
    ]
    if args.clean_first:
        build_command.append("--clean-first")
    if args.parallel:
        build_command.extend(["--parallel", str(args.parallel)])
    run_command(build_command, args.dry_run)

    if args.install:
        install_command = ["cmake", "--install", str(binary_dir), "--config", args.config]
        run_command(install_command, args.dry_run)

    package_output_dir = binary_dir / "EvoEngine_App" / args.config / "Packages"
    print(f"Package target: {package['target']}")
    print(f"Build output:    {package_output_dir}")
    if args.install:
        print("Install output:  see the CMAKE_INSTALL_PREFIX configured by the selected preset")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
