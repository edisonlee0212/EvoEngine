"""Reject editor dependencies in the configuration-resolved runtime target graph."""

from __future__ import annotations

import argparse
from functools import cache
from pathlib import Path
import re
import sys


SOURCE_SUFFIXES = {".h", ".hpp", ".hxx", ".inl", ".c", ".cc", ".cpp", ".cxx"}
INCLUDE = re.compile(r'^\s*#\s*include\s*[<"]([^">]+)[">]', re.MULTILINE)
INCLUDE_DIRECTIVE = re.compile(r'^\s*#\s*include\b[^\n]*', re.MULTILINE)
GUI = re.compile(r'\b(?:ImGui|ImGuizmo|ImNodes|ImVec[24]|ImTextureID)\b')
GUARD = re.compile(r'^\s*#\s*(?:if|ifdef|ifndef|elif)\b[^\n]*\bEVOENGINE_WITH_EDITOR\b', re.MULTILINE)


def sections(path: Path) -> dict[str, list[str]]:
    result: dict[str, list[str]] = {}
    current: list[str] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("@"):
            current = result.setdefault(line[1:], [])
        elif line:
            current.append(line)
    return result


def uncomment(source: str) -> str:
    # Preserve string literals, including quoted include paths.
    return re.sub(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*.*?\*/',
                  lambda m: m[0] if m[0][0] in "\"'" else "\n" * m[0].count("\n"),
                  source, flags=re.DOTALL)


def check(graph: Path) -> list[str]:
    index = sections(graph / "index.txt")
    root = Path(index["root"][0]).resolve()
    targets = {data["name"][0]: data for path in (graph / "targets").glob("*.txt")
               if (data := sections(path))}
    aliases = dict(line.split("=", 1) for line in index.get("aliases", []))
    editor_roots = [Path(p).resolve() for p in index.get("editor_roots", [])]
    errors: set[str] = set()

    @cache
    def forbidden(path: Path) -> bool:
        return any(path.is_relative_to(p) for p in editor_roots) or (
            path.is_relative_to(root) and "editor" in {p.lower() for p in path.relative_to(root).parts})

    editor_headers = {p.name.lower() for base in editor_roots if base.is_dir()
                      for p in base.rglob("*") if p.suffix in SOURCE_SUFFIXES}
    for runtime in index["runtime"]:
        visited: set[Path] = set()
        closure: dict[str, dict[str, list[str]]] = {}
        pending = [runtime]
        while pending:
            name = pending.pop()
            name = aliases.get(name, name)
            if name in closure:
                continue
            if name not in targets:
                if forbidden(Path(name).resolve()) or re.search(r'(?:EditorSDK|EditorPackage|imgui|imguizmo)', name, re.I):
                    errors.add(f"{runtime}: forbidden library {name}")
                continue
            target = targets[name]
            closure[name] = target
            if forbidden(Path(target["source_dir"][0]).resolve()):
                errors.add(f"{runtime}: links editor target {name}")
            pending.extend(target.get("LINK_LIBRARIES", []) + target.get("INTERFACE_LINK_LIBRARIES", []))

        includes: list[Path] = []
        sources: set[Path] = set()
        for name, target in closure.items():
            base = Path(target["source_dir"][0])
            for value in target.get("INCLUDE_DIRECTORIES", []) + target.get("INTERFACE_INCLUDE_DIRECTORIES", []):
                path = (base / value).resolve()
                if forbidden(path):
                    errors.add(f"{runtime}: editor include directory from {name}: {path}")
                includes.append(path)
            for value in target.get("SOURCES", []) + target.get("INTERFACE_SOURCES", []) + target.get("PRECOMPILE_HEADERS", []) + target.get("INTERFACE_PRECOMPILE_HEADERS", []):
                if value.startswith("<"):
                    continue
                path = (base / value.strip('"')).resolve()
                if forbidden(path):
                    errors.add(f"{runtime}: editor source/PCH from {name}: {path}")
                if path.is_relative_to(root) and not path.is_relative_to(root / "Extern"):
                    sources.add(path)
        includes = list(dict.fromkeys(includes))
        while sources:
            path = sources.pop()
            if path in visited or path.suffix not in SOURCE_SUFFIXES or not path.is_file():
                continue
            visited.add(path)
            source = uncomment(path.read_text(encoding="utf-8", errors="replace"))
            if GUI.search(source) or GUARD.search(source):
                errors.add(f"{runtime}: editor GUI code or per-class editor guard in {path}")
            if any(not INCLUDE.match(line) for line in INCLUDE_DIRECTIVE.findall(source)):
                errors.add(f"{runtime}: use a literal include path so dependencies can be checked: {path}")
            for include in INCLUDE.findall(source):
                resolved = next((candidate for base in [path.parent, *includes]
                                 if (candidate := base / include).is_file()), None)
                if resolved:
                    resolved = resolved.resolve()
                if ((resolved is not None and forbidden(resolved)) or
                        (resolved is None and Path(include).name.lower() in editor_headers)):
                    errors.add(f"{runtime}: {path} includes editor header {include}")
                elif resolved and resolved.is_relative_to(root) and not resolved.is_relative_to(root / "Extern"):
                    sources.add(resolved)
    return sorted(errors)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--graph", required=True, type=Path)
    errors = check(parser.parse_args().graph)
    if errors:
        print("Runtime/editor boundary violations:\n" + "\n".join(errors), file=sys.stderr)
        return 1
    print("Runtime/editor dependency boundary passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
