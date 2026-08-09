#!/usr/bin/env python3
"""Generate or validate the current EvoEngine SDK native-Slang policy."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re


SHADER_EXTENSIONS = {".slang", ".slangh"}
INCLUDE_RE = re.compile(r'^\s*#\s*include\s*[<"]([^">]+)[">]', re.MULTILINE)
IMPORT_RE = re.compile(r"^\s*(?:__exported\s+)?import\s+([A-Za-z_][A-Za-z0-9_.]*)\s*;", re.MULTILINE)
MACRO_RE = re.compile(r"^\s*#\s*(define|if|ifdef|ifndef|elif|else|endif)\b", re.MULTILINE)
STRICT_NATIVE_MARKER = "@evoengine-dialect native"


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--policy",
        type=Path,
        default=root / "EvoEngine_Tests" / "ShaderPolicy" / "sdk-shader-policy.json",
    )
    parser.add_argument("--write", action="store_true", help="Write the policy from the current shader tree.")
    return parser.parse_args()


def strip_comments(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", source)


def uses_compatibility_syntax(source: str) -> bool:
    return any(
        token in source
        for token in ("#extension GL_", "layout(", "layout (", "precision highp", "readonly buffer", "writeonly buffer")
    )


def module_name(relative: Path) -> str:
    parts = [re.sub(r"[^A-Za-z0-9_]", "_", part) for part in relative.with_suffix("").parts]
    if parts[0] == "Modules":
        return ".".join(parts[1:])
    return ".".join(("EvoEngine", "EntryPoints", *parts))


def collect_sources(root: Path) -> list[dict[str, object]]:
    shader_root = root / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders"
    records: list[dict[str, object]] = []
    for path in sorted(candidate for candidate in shader_root.rglob("*") if candidate.suffix in SHADER_EXTENSIONS):
        relative = path.relative_to(shader_root)
        source = path.read_text(encoding="utf-8", errors="replace")
        uncommented = strip_comments(source)
        records.append(
            {
                "path": relative.as_posix(),
                "module": module_name(relative),
                "strict_native": STRICT_NATIVE_MARKER in source,
                "compatibility": uses_compatibility_syntax(uncommented),
                "includes": INCLUDE_RE.findall(uncommented),
                "imports": IMPORT_RE.findall(uncommented),
                "macro_controls": len(MACRO_RE.findall(uncommented)),
            }
        )
    return records


def make_policy(root: Path) -> dict[str, object]:
    records = collect_sources(root)
    return {
        "schema": 2,
        "scope": "EvoEngine_SDK current native-Slang shader policy",
        "module_prefix": "EvoEngine",
        "counts": {
            "files": len(records),
            "imports": sum(len(record["imports"]) for record in records),
            "macro_control_files": sum(bool(record["macro_controls"]) for record in records),
        },
        "macro_adapters": [
            {"path": record["path"], "macro_controls": record["macro_controls"]}
            for record in records
            if record["macro_controls"]
        ],
        "module_names": [{"path": record["path"], "module": record["module"]} for record in records],
    }


def validate_transfer(root: Path, failures: list[str]) -> None:
    baseline_path = (
        root / "EvoEngine_Tests" / "ShaderPolicy" / "Baselines" / "sdk-shader-baseline.json"
    )
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    transfer = baseline["ecosyslab_glsl_ownership_transfer"]
    source = root / transfer["source"]
    destination = root / transfer["destination"]
    for record in transfer["files"]:
        source_path = source / record["name"]
        destination_path = destination / record["name"]
        if source_path.exists():
            failures.append(f"SDK still owns relocated GLSL header: {source_path}")
        if not destination_path.is_file():
            failures.append(f"EcoSysLab is missing relocated GLSL header: {destination_path}")
            continue
        if hashlib.sha256(destination_path.read_bytes()).hexdigest() != record["sha256"]:
            failures.append(f"Relocated GLSL header hash changed: {destination_path}")


def validate_policy(root: Path, policy: dict[str, object]) -> list[str]:
    failures: list[str] = []
    records = collect_sources(root)
    by_path = {record["path"]: record for record in records}
    modules = {record["module"]: record["path"] for record in records}
    if len(modules) != len(records):
        failures.append("SDK module-name mapping contains a collision.")

    expected_modules = {record["path"]: record["module"] for record in policy["module_names"]}
    actual_modules = {record["path"]: record["module"] for record in records}
    if expected_modules != actual_modules:
        failures.append("SDK module-name inventory changed; regenerate and review the policy.")

    expected_adapters = {
        record["path"]: record["macro_controls"] for record in policy["macro_adapters"]
    }
    actual_adapters = {
        record["path"]: record["macro_controls"] for record in records if record["macro_controls"]
    }
    if expected_adapters != actual_adapters:
        failures.append("SDK macro-adapter allowlist changed; regenerate and review the policy.")

    for record in records:
        path = record["path"]
        if not record["strict_native"]:
            failures.append(f"SDK source lacks the native dialect marker: {path}")
        if record["compatibility"]:
            failures.append(f"SDK source uses compatibility syntax: {path}")
        if record["includes"]:
            failures.append(f"SDK source uses textual includes: {path}")
        for imported in record["imports"]:
            if not imported.startswith(policy["module_prefix"] + "."):
                failures.append(f"SDK import is outside the module namespace: {path}: {imported}")
            elif imported not in modules:
                failures.append(f"SDK import does not resolve uniquely: {path}: {imported}")

    shader_root = root / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders"
    glsl_files = sorted(shader_root.rglob("*.glsl"))
    if glsl_files:
        failures.append(f"SDK contains {len(glsl_files)} .glsl files.")
    validate_transfer(root, failures)
    return failures


def main() -> int:
    args = parse_args()
    root = repo_root()
    policy_path = args.policy.resolve()
    if args.write:
        policy_path.write_text(
            json.dumps(make_policy(root), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
            newline="",
        )
        print(f"SDK shader policy: {policy_path}")
    if not policy_path.is_file():
        raise RuntimeError(f"Missing SDK shader policy: {policy_path}")
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    failures = validate_policy(root, policy)
    if failures:
        for failure in failures:
            print(f"ERROR: {failure}")
        return 1
    print(json.dumps(make_policy(root)["counts"], sort_keys=True))
    print("SDK shader policy passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
