#!/usr/bin/env python3
"""Launch EvoEngineEditor with EcoSysLabProject/test.eveproj and a visible OS console."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Launch EvoEngineEditor on Resources/EcoSysLabProject/test.eveproj with a console window."
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
        "editor_args",
        nargs=argparse.REMAINDER,
        help="Extra arguments forwarded to EvoEngineEditor after --. Example: -- --demo ecosyslab",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = repo_root()
    editor = resolve_editor(root, args.editor)
    project = args.project
    if project is None:
        project = root / "Resources" / "EcoSysLabProject" / "test.eveproj"
    elif not project.is_absolute():
        project = root / project
    project = project.resolve()
    if not project.is_file():
        raise SystemExit(f"Project not found: {project}")

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
