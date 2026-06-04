#!/usr/bin/env python3
"""Format EvoEngine C++ source files with clang-format.

By default this script formats .cpp and .hpp files under the SDK, services,
runtime packages, and app folders. It intentionally does not walk Extern or
other third-party dependency folders.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import os
import shutil
import subprocess
import sys
from pathlib import Path


DEFAULT_ROOTS = (
    "EvoEngine_SDK",
    "EvoEngine_Services",
    "EvoEngine_Packages",
    "EvoEngine_App",
    "EvoEngine_Tests",
)

DEFAULT_EXTENSIONS = (".cpp", ".hpp")

EXCLUDED_DIRECTORY_NAMES = {
    ".git",
    ".vs",
    "build",
    "out",
    "Extern",
    "3rdParty",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run clang-format on EvoEngine .cpp and .hpp files."
    )
    parser.add_argument(
        "--clang-format",
        default="clang-format",
        help="clang-format executable to use. Defaults to clang-format on PATH.",
    )
    parser.add_argument(
        "--root",
        action="append",
        dest="roots",
        help=(
            "Source root to scan. Can be passed multiple times. "
            f"Defaults to: {', '.join(DEFAULT_ROOTS)}."
        ),
    )
    parser.add_argument(
        "--extension",
        action="append",
        dest="extensions",
        help=(
            "File extension to format. Can be passed multiple times. "
            f"Defaults to: {', '.join(DEFAULT_EXTENSIONS)}."
        ),
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Check whether files are formatted without modifying them.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the number of files that would be formatted without running clang-format.",
    )
    parser.add_argument(
        "--jobs",
        type=int,
        default=0,
        help="Number of parallel clang-format processes. Defaults to the Python executor default.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print each file as it is processed.",
    )
    return parser.parse_args()


def normalize_extensions(extensions: list[str] | None) -> set[str]:
    selected = extensions if extensions else list(DEFAULT_EXTENSIONS)
    return {extension if extension.startswith(".") else f".{extension}" for extension in selected}


def has_excluded_part(path: Path) -> bool:
    return any(part in EXCLUDED_DIRECTORY_NAMES for part in path.parts)


def discover_files(repo_root: Path, roots: list[str] | None, extensions: set[str]) -> list[Path]:
    selected_roots = roots if roots else list(DEFAULT_ROOTS)
    files: list[Path] = []
    for root_name in selected_roots:
        source_root = repo_root / root_name
        if not source_root.exists():
            continue
        if source_root.is_file():
            if source_root.suffix in extensions and not has_excluded_part(source_root.relative_to(repo_root)):
                files.append(source_root)
            continue
        for path in source_root.rglob("*"):
            relative_path = path.relative_to(repo_root)
            if has_excluded_part(relative_path):
                continue
            if path.is_file() and path.suffix in extensions:
                files.append(path)
    return sorted(set(files))


def find_clang_format(executable: str) -> str | None:
    explicit_path = Path(executable)
    if explicit_path.exists():
        return str(explicit_path.resolve())

    path_match = shutil.which(executable)
    if path_match:
        return path_match

    candidate_paths = [
        Path(os.environ.get("ProgramFiles", "")) / "LLVM" / "bin" / "clang-format.exe",
        Path(os.environ.get("ProgramFiles(x86)", "")) / "LLVM" / "bin" / "clang-format.exe",
    ]

    vswhere = (
        Path(os.environ.get("ProgramFiles(x86)", ""))
        / "Microsoft Visual Studio"
        / "Installer"
        / "vswhere.exe"
    )
    if vswhere.exists():
        result = subprocess.run(
            [
                str(vswhere),
                "-latest",
                "-products",
                "*",
                "-property",
                "installationPath",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        visual_studio_path = result.stdout.strip()
        if result.returncode == 0 and visual_studio_path:
            visual_studio_root = Path(visual_studio_path)
            candidate_paths.extend(
                [
                    visual_studio_root / "VC" / "Tools" / "Llvm" / "x64" / "bin" / "clang-format.exe",
                    visual_studio_root / "VC" / "Tools" / "Llvm" / "bin" / "clang-format.exe",
                ]
            )

    for candidate_path in candidate_paths:
        if candidate_path.exists():
            return str(candidate_path.resolve())
    return None


def normalize_lf(data: bytes) -> bytes:
    return data.replace(b"\r\n", b"\n").replace(b"\r", b"\n")


def run_format(clang_format: str, path: Path, repo_root: Path, check: bool, verbose: bool) -> tuple[Path, bool, str]:
    relative_path = path.relative_to(repo_root)
    if verbose:
        print(relative_path)

    if check:
        original = path.read_bytes()
        result = subprocess.run(
            [clang_format, str(path)],
            cwd=repo_root,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if result.returncode != 0:
            return path, False, result.stderr.decode(errors="replace").strip()
        if normalize_lf(result.stdout) != original:
            return path, False, "needs formatting"
        return path, True, ""

    result = subprocess.run(
        [clang_format, "-i", str(path)],
        cwd=repo_root,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        return path, False, result.stderr.decode(errors="replace").strip()
    formatted = path.read_bytes()
    normalized = normalize_lf(formatted)
    if normalized != formatted:
        path.write_bytes(normalized)
    return path, True, ""


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    extensions = normalize_extensions(args.extensions)
    files = discover_files(repo_root, args.roots, extensions)

    if args.dry_run:
        print(f"Matched {len(files)} file(s).")
        if args.verbose:
            for path in files:
                print(path.relative_to(repo_root))
        return 0

    clang_format = find_clang_format(args.clang_format)
    if clang_format is None:
        print(
            f"Could not find '{args.clang_format}'. Install clang-format or pass --clang-format <path>.",
            file=sys.stderr,
        )
        return 1

    if not files:
        print("No matching files found.")
        return 0

    max_workers = args.jobs if args.jobs > 0 else None
    failures: list[tuple[Path, str]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = [
            executor.submit(run_format, clang_format, path, repo_root, args.check, args.verbose)
            for path in files
        ]
        for future in concurrent.futures.as_completed(futures):
            path, ok, message = future.result()
            if not ok:
                failures.append((path, message))

    if failures:
        action = "failed formatting check" if args.check else "failed to format"
        print(f"{len(failures)} file(s) {action}:", file=sys.stderr)
        for path, message in sorted(failures):
            print(f"  {path.relative_to(repo_root)}: {message}", file=sys.stderr)
        return 1

    action = "already formatted" if args.check else "formatted"
    print(f"{len(files)} file(s) {action}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
