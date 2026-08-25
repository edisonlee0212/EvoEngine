#!/usr/bin/env python3
"""Reject legacy shader files, paths, and syntax in tracked first-party sources."""

from __future__ import annotations

from pathlib import Path
import re
import subprocess


LEGACY_EXTENSIONS = {
    ".glsl", ".vert", ".tesc", ".tese", ".geom", ".frag", ".comp", ".task",
    ".mesh", ".rgen", ".rmiss", ".rahit", ".rchit", ".rint", ".rcall",
}
SHADER_EXTENSIONS = {".slang", ".slangh"}
SOURCE_EXTENSIONS = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx", ".inl"}
INCLUDE_RE = re.compile(r'^\s*#\s*include\s*[<"]([^">]+)[">]', re.MULTILINE)
LEGACY_LOAD_RE = re.compile(
    r"Shaders[/\\][^\"'\r\n]*\.(?:glsl|vert|tesc|tese|geom|frag|comp|task|mesh|rgen|rmiss|rahit|rchit|rint|rcall)\b",
    re.IGNORECASE,
)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def run_git(root: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(root), *args],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="replace",
    ).stdout


def first_party_repositories(root: Path) -> list[tuple[str, Path]]:
    repositories = [(".", root)]
    output = subprocess.run(
        ["git", "config", "--file", str(root / ".gitmodules"), "--get-regexp", "path"],
        check=False,
        stdout=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="replace",
    ).stdout
    for line in output.splitlines():
        relative = line.split(maxsplit=1)[1].replace("\\", "/")
        if relative != "Extern" and not relative.startswith("Extern/"):
            repositories.append((relative, root / relative))
    return repositories


def strip_comments(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", source)


def uses_compatibility_syntax(source: str) -> bool:
    return any(
        token in source
        for token in ("#extension GL_", "layout(", "layout (", "precision highp", "readonly buffer", "writeonly buffer")
    )


def main() -> int:
    root = repo_root()
    failures: list[str] = []
    for repository, path in first_party_repositories(root):
        for tracked in run_git(path, "ls-files").splitlines():
            normalized = tracked.replace("\\", "/")
            if repository == "." and normalized.startswith("Extern/"):
                continue
            tracked_path = Path(tracked)
            extension = tracked_path.suffix.lower()
            if extension in LEGACY_EXTENSIONS:
                failures.append(f"Legacy shader source: {repository}:{normalized}")
                continue
            if extension not in SOURCE_EXTENSIONS | SHADER_EXTENSIONS:
                continue
            source = (path / tracked_path).read_text(encoding="utf-8", errors="replace")
            if extension in SOURCE_EXTENSIONS:
                for match in LEGACY_LOAD_RE.finditer(source):
                    failures.append(f"Legacy runtime shader path: {repository}:{normalized}: {match.group(0)}")
                continue
            uncommented = strip_comments(source)
            if uses_compatibility_syntax(uncommented):
                failures.append(f"Compatibility syntax: {repository}:{normalized}")
            if INCLUDE_RE.search(uncommented):
                failures.append(f"Textual include: {repository}:{normalized}")

    if failures:
        print("First-party native-Slang policy violations:")
        for failure in sorted(failures):
            print(f"  {failure}")
        return 1
    print("First-party Slang-only shader policy passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
