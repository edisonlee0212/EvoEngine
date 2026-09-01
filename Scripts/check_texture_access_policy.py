#!/usr/bin/env python3
"""Validate the reviewed first-party sampled-texture access inventory."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re

from check_first_party_shader_policy import first_party_repositories, repo_root, run_git, strip_comments


CATEGORIES = {
    "PersistentSampledAsset": "Asset-backed sampled data with identity beyond one pass or frame.",
    "TransientPassResource": "Render target, history, attachment, or other pass-owned sampled input.",
    "BoundedPassArray": "Explicitly bounded pass-local collection such as shadows, DDGI atlases, or lookup tables.",
}
SHADER_EXTENSIONS = {".slang", ".slangh"}
HOST_EXTENSIONS = {".c", ".cc", ".cpp", ".cxx", ".h", ".hpp", ".inl"}
RESOURCE_RE = re.compile(
    r"^\s*(?:\[\[[^\n]*\]\]\s*)?(?:public\s+)?"
    r"(?P<type>(?:Sampler|Texture)(?:1D|2D|3D|Cube|1DArray|2DArray|CubeArray))"
    r"(?:\s*<[^;\n]+>)?\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)"
    r"(?:\s*\[[^;\n]*\])?\s*;",
    re.MULTILINE,
)
BINDING_CALL_RE = re.compile(r"(?P<call>[A-Za-z_][A-Za-z0-9_]*(?:->|\.)PushDescriptorBinding\s*\(.*?\)\s*;)", re.DOTALL)
SAMPLED_DESCRIPTOR_RE = re.compile(r"VK_DESCRIPTOR_TYPE_(?:COMBINED_IMAGE_SAMPLER|SAMPLED_IMAGE)")
OWNER_RE = re.compile(r"(?P<owner>[A-Za-z_][A-Za-z0-9_]*)\s*(?:->|\.)PushDescriptorBinding")


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--policy",
        type=Path,
        default=root / "EvoEngine_Tests" / "ShaderPolicy" / "texture-access-policy.json",
    )
    parser.add_argument("--write", action="store_true", help="Refresh records while preserving reviewed classifications.")
    parser.add_argument(
        "--bootstrap",
        action="store_true",
        help="Assign the initial reviewed classifications; valid only together with --write.",
    )
    return parser.parse_args()


def source_files(root: Path) -> list[tuple[str, Path]]:
    files: list[tuple[str, Path]] = []
    for repository, path in first_party_repositories(root):
        for tracked in run_git(path, "ls-files", "--cached", "--others", "--exclude-standard").splitlines():
            normalized = tracked.replace("\\", "/")
            if repository == "." and normalized.startswith("Extern/"):
                continue
            source = path / tracked
            if not source.is_file() or source.suffix.lower() not in SHADER_EXTENSIONS | HOST_EXTENSIONS:
                continue
            qualified = normalized if repository == "." else f"{repository}/{normalized}"
            files.append((qualified, source))
    return sorted(files)


def normalize_binding(expression: str) -> str:
    return " ".join(expression.split())


def first_argument(call: str) -> str:
    body = call[call.find("(") + 1 : call.rfind(")")]
    depth = 0
    for index, character in enumerate(body):
        if character in "(<[{":
            depth += 1
        elif character in ")>]}":
            depth -= 1
        elif character == "," and depth == 0:
            return normalize_binding(body[:index])
    return normalize_binding(body)


def collect_records(root: Path) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    occurrences: dict[str, int] = {}
    def record_id(key: str) -> str:
        ordinal = occurrences.get(key, 0)
        occurrences[key] = ordinal + 1
        return f"{key}:{ordinal}"

    for relative, path in source_files(root):
        source = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
        if path.suffix.lower() in SHADER_EXTENSIONS:
            for match in RESOURCE_RE.finditer(source):
                key = f"shader:{relative}:{match.group('type')}:{match.group('name')}"
                records.append(
                    {
                        "id": record_id(key),
                        "kind": "shader-resource",
                        "path": relative,
                        "resource_type": match.group("type"),
                        "symbol": match.group("name"),
                    }
                )
            continue
        for match in BINDING_CALL_RE.finditer(source):
            call = match.group("call")
            descriptor = SAMPLED_DESCRIPTOR_RE.search(call)
            if not descriptor:
                continue
            owner = OWNER_RE.search(call)
            owner_name = owner.group("owner") if owner else "unknown"
            binding = first_argument(call)
            key = f"host:{relative}:{owner_name}:{binding}:{descriptor.group(0)}"
            records.append(
                {
                    "id": record_id(key),
                    "kind": "host-binding",
                    "path": relative,
                    "owner": owner_name,
                    "binding": binding,
                    "descriptor_type": descriptor.group(0),
                }
            )
    return records


def bootstrap_classification(record: dict[str, object]) -> str:
    path = str(record["path"])
    symbol = str(record.get("symbol", ""))
    owner = str(record.get("owner", ""))
    persistent_markers = (
        "GltfRasterMaterial.slang",
        "LightingFixedSet3.slang",
        "LightingFixedSet4.slang",
        "Textures.slang",
        "EnvironmentalMapIrradianceConvolution.slang",
        "EnvironmentalMapPrefilter.slang",
        "EquirectangularMapToCubemap.slang",
    )
    persistent_owners = {
        "raster_material_layout_",
        "raster_lighting_texture_layout_",
    }
    if any(marker in path for marker in persistent_markers) or owner in persistent_owners:
        return "PersistentSampledAsset"
    if path.endswith("EvoEngine_SDK/src/RenderLayer.cpp") and owner == "layout" and str(record.get("binding")) in {
        "9",
        "10",
    }:
        return "PersistentSampledAsset"
    bounded_markers = ("DDGI", "SMAA", "Shadow", "VolumetricCloud")
    if any(marker.lower() in f"{path}/{symbol}/{owner}".lower() for marker in bounded_markers):
        return "BoundedPassArray"
    return "TransientPassResource"


def make_policy(root: Path, previous: dict[str, object] | None, bootstrap: bool) -> dict[str, object]:
    classifications = {
        str(record["id"]): str(record["classification"])
        for record in (previous or {}).get("records", [])
        if "classification" in record
    }
    records = collect_records(root)
    for record in records:
        record["classification"] = classifications.get(
            str(record["id"]), bootstrap_classification(record) if bootstrap else "UNCLASSIFIED"
        )
    return {
        "schema": 1,
        "scope": "Tracked first-party sampled-image declarations and host descriptor bindings",
        "categories": CATEGORIES,
        "records": records,
    }


def validate(policy: dict[str, object], actual: dict[str, object]) -> list[str]:
    failures: list[str] = []
    if policy.get("schema") != 1:
        failures.append("Unsupported texture-access policy schema.")
    if policy.get("categories") != CATEGORIES:
        failures.append("Texture ownership categories changed; review and regenerate the policy.")
    expected_records = policy.get("records", [])
    actual_records = actual.get("records", [])
    if expected_records != actual_records:
        failures.append("Sampled-texture inventory changed; run --write, classify every new record, and review the diff.")
    for record in expected_records:
        classification = record.get("classification")
        if classification not in CATEGORIES:
            failures.append(f"Unclassified sampled-texture record: {record.get('id', '<missing id>')}")
    return failures


def main() -> int:
    args = parse_args()
    if args.bootstrap and not args.write:
        raise RuntimeError("--bootstrap requires --write")
    policy_path = args.policy.resolve()
    previous = json.loads(policy_path.read_text(encoding="utf-8")) if policy_path.is_file() else None
    if args.write:
        policy = make_policy(repo_root(), previous, args.bootstrap)
        policy_path.write_text(json.dumps(policy, indent=2) + "\n", encoding="utf-8", newline="")
        print(f"Texture access policy: {policy_path}")
    if not policy_path.is_file():
        raise RuntimeError(f"Missing texture access policy: {policy_path}")
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    actual = make_policy(repo_root(), policy, False)
    failures = validate(policy, actual)
    if failures:
        for failure in failures:
            print(f"ERROR: {failure}")
        return 1
    counts = {category: 0 for category in CATEGORIES}
    for record in policy["records"]:
        counts[record["classification"]] += 1
    print(json.dumps({"records": len(policy["records"]), "classifications": counts}, sort_keys=True))
    print("Texture access policy passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
