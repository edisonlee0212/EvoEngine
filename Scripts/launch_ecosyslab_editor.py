#!/usr/bin/env python3
"""Launch EvoEngineEditor with EcoSysLabProject/test.eveproj and a visible OS console.

The project file must declare EcoSysLab under startup_runtime_packages so the
editor loads EcoSysLabPackage.dll before opening the start scene. Without that
entry, package-backed components (DynamicStrandsDemo, DynamicTreeStrands, …)
deserialize as UnknownPrivateComponent.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

REQUIRED_PACKAGE = "EcoSysLab"


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def resolve_editor(root: Path, explicit: Path | None) -> Path:
    if explicit is not None:
        path = explicit if explicit.is_absolute() else root / explicit
        if not path.is_file():
            raise SystemExit(f"Editor executable not found: {path}")
        return path.resolve()

    candidates = [
        root / "out" / "install" / "vs2026-x64" / "bin" / "EvoEngineEditor.exe",
        root / "out" / "build" / "vs2026-x64" / "EvoEngine_App" / "RelWithDebInfo" / "EvoEngineEditor.exe",
        root / "out" / "build" / "vs2026-x64" / "EvoEngine_App" / "Release" / "EvoEngineEditor.exe",
        root / "out" / "build" / "vs2026-x64" / "EvoEngine_App" / "Debug" / "EvoEngineEditor.exe",
        root / "out" / "build" / "cli-Release" / "EvoEngine_App" / "EvoEngineEditor.exe",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise SystemExit(
        "EvoEngineEditor.exe not found. Build/install apps first, or pass --editor <path>.\n"
        "Tried:\n  - " + "\n  - ".join(str(path) for path in candidates)
    )


def read_startup_runtime_packages(project: Path) -> list[str]:
    """Parse startup_runtime_packages from a .eveproj without requiring PyYAML."""
    lines = project.read_text(encoding="utf-8").splitlines()
    for index, line in enumerate(lines):
        match = re.match(r"^startup_runtime_packages:\s*(.*)$", line)
        if not match:
            continue

        inline = match.group(1).strip()
        if inline.startswith("[") and inline.endswith("]"):
            inner = inline[1:-1].strip()
            if not inner:
                return []
            return [item.strip().strip("'\"") for item in inner.split(",") if item.strip()]
        if inline:
            return [inline.strip("'\"")]

        packages: list[str] = []
        for continuation in lines[index + 1 :]:
            if not continuation.strip():
                continue
            if re.match(r"^\S", continuation):
                break
            item = re.match(r"^\s*-\s+(.+?)\s*$", continuation)
            if item:
                packages.append(item.group(1).strip().strip("'\""))
        return packages
    return []


def require_ecosyslab_package(project: Path) -> None:
    packages = read_startup_runtime_packages(project)
    if REQUIRED_PACKAGE in packages:
        return
    raise SystemExit(
        f"{project} must list '{REQUIRED_PACKAGE}' under startup_runtime_packages "
        "so EvoEngineEditor loads the EcoSysLab package before the start scene.\n"
        f"Found startup_runtime_packages: {packages or '(missing)'}\n"
        "Example:\n"
        "  application_name: EcoSysLab\n"
        "  preferred_editor: EvoEngineEditor\n"
        "  startup_runtime_packages:\n"
        f"    - {REQUIRED_PACKAGE}\n"
        "  start_scene_handle: <handle>"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Launch EvoEngineEditor on Resources/EcoSysLabProject/test.eveproj with a console "
            "window. Requires the project to declare EcoSysLab in startup_runtime_packages."
        )
    )
    parser.add_argument(
        "--editor",
        type=Path,
        default=None,
        help="Path to EvoEngineEditor.exe. Defaults to the installed vs2026-x64 binary when present.",
    )
    parser.add_argument(
        "--project",
        type=Path,
        default=None,
        help="Path to an .eveproj. Defaults to Resources/EcoSysLabProject/test.eveproj.",
    )
    parser.add_argument(
        "--no-console",
        action="store_true",
        help="Do not pass --console (editor stays a pure GUI process).",
    )
    parser.add_argument(
        "--rebuild",
        nargs="?",
        const="incremental",
        default=None,
        choices=["incremental", "clean"],
        metavar="MODE",
        help=(
            "Rebuild/install apps via Scripts/install_apps.py before launching. "
            "Optional MODE: incremental (default) reuses the build cache and only "
            "rebuilds out-of-date targets; clean reconfigures and cleans the install "
            "directory before building."
        ),
    )
    parser.add_argument(
        "--preset",
        default="vs2026-x64",
        help="CMake configure preset used with --rebuild. Defaults to vs2026-x64.",
    )
    parser.add_argument(
        "--config",
        default="RelWithDebInfo",
        choices=["Debug", "RelWithDebInfo", "Release"],
        help="Build configuration used with --rebuild. Defaults to RelWithDebInfo.",
    )
    parser.add_argument(
        "editor_args",
        nargs=argparse.REMAINDER,
        help="Extra arguments forwarded to EvoEngineEditor after --. Example: -- --console",
    )
    return parser.parse_args()


def rebuild_apps(root: Path, preset: str, config: str, *, clean: bool) -> None:
    command = [
        sys.executable,
        str(root / "Scripts" / "install_apps.py"),
        "--preset",
        preset,
        "--config",
        config,
        "--no-open",
    ]
    if not clean:
        command.append("--incremental")
    print(
        subprocess.list2cmdline(command) if os.name == "nt" else " ".join(command),
        flush=True,
    )
    completed = subprocess.run(command, cwd=str(root))
    if completed.returncode != 0:
        raise SystemExit(completed.returncode)


def main() -> int:
    args = parse_args()
    root = repo_root()
    if args.rebuild is not None:
        rebuild_apps(root, args.preset, args.config, clean=args.rebuild == "clean")

    editor = resolve_editor(root, args.editor)
    project = args.project
    if project is None:
        project = root / "Resources" / "EcoSysLabProject" / "test.eveproj"
    elif not project.is_absolute():
        project = root / project
    project = project.resolve()
    if not project.is_file():
        raise SystemExit(f"Project not found: {project}")
    require_ecosyslab_package(project)

    command = [str(editor), "--project", str(project)]
    if not args.no_console:
        command.append("--console")
    forwarded = list(args.editor_args)
    if forwarded and forwarded[0] == "--":
        forwarded = forwarded[1:]
    command.extend(forwarded)

    print(subprocess.list2cmdline(command) if os.name == "nt" else " ".join(command), flush=True)
    return subprocess.call(command, cwd=str(editor.parent))


if __name__ == "__main__":
    raise SystemExit(main())
