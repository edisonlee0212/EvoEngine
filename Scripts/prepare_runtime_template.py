#!/usr/bin/env python3
"""Prepare an immutable EvoEngine native runtime template from a CMake layout."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any

SCHEMA = 1
OWNER_MARKER = ".evoengine-runtime-template.json"


class TemplateError(ValueError):
    pass


@dataclass(frozen=True)
class CopyEntry:
    source: Path
    destination: PurePosixPath


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise TemplateError(f"Cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise TemplateError(f"Expected a JSON object in {path}.")
    return value


def _relative_path(value: Any, label: str, *, allow_root: bool = False) -> PurePosixPath:
    if not isinstance(value, str):
        raise TemplateError(f"{label} must be a relative path string.")
    normalized = value.replace("\\", "/")
    if normalized in ("", "."):
        if allow_root:
            return PurePosixPath(".")
        raise TemplateError(f"{label} must not be empty.")
    if PurePosixPath(normalized).is_absolute() or PureWindowsPath(value).is_absolute():
        raise TemplateError(f"{label} must be relative: {value}")
    parts = normalized.split("/")
    if any(part in ("", ".", "..") for part in parts) or any(":" in part for part in parts):
        raise TemplateError(f"{label} contains an invalid or traversing component: {value}")
    return PurePosixPath(*parts)


def _absolute_source(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise TemplateError(f"{label} must be an absolute path.")
    path = Path(value)
    if not path.is_absolute():
        raise TemplateError(f"{label} must be absolute: {value}")
    return path


def _collect_entries(layout: dict[str, Any]) -> list[CopyEntry]:
    entries: list[CopyEntry] = []
    files = layout.get("files", [])
    resources = layout.get("resource_directories", [])
    if not isinstance(files, list) or not isinstance(resources, list):
        raise TemplateError("files and resource_directories must be arrays.")

    for index, item in enumerate(files):
        if not isinstance(item, dict):
            raise TemplateError(f"files[{index}] must be an object.")
        source_value = item.get("source")
        if source_value == "":
            continue
        source = _absolute_source(source_value, f"files[{index}].source")
        destination_value = item.get("destination", source.name)
        destination = _relative_path(destination_value, f"files[{index}].destination")
        required = item.get("required", True)
        if not isinstance(required, bool):
            raise TemplateError(f"files[{index}].required must be boolean.")
        if not source.exists():
            if required:
                raise TemplateError(f"Missing required payload file: {source}")
            continue
        if not source.is_file() or source.is_symlink():
            raise TemplateError(f"Payload source must be a regular file: {source}")
        entries.append(CopyEntry(source, destination))

    for index, item in enumerate(resources):
        if not isinstance(item, dict):
            raise TemplateError(f"resource_directories[{index}] must be an object.")
        source = _absolute_source(item.get("source"), f"resource_directories[{index}].source")
        destination = _relative_path(
            item.get("destination"), f"resource_directories[{index}].destination", allow_root=True
        )
        excludes = item.get("exclude", [])
        if not isinstance(excludes, list) or not all(isinstance(value, str) and value for value in excludes):
            raise TemplateError(f"resource_directories[{index}].exclude must contain names.")
        excluded_names = {value.casefold() for value in excludes}
        if not source.is_dir() or source.is_symlink():
            raise TemplateError(f"Missing or invalid resource directory: {source}")
        for current, directory_names, file_names in os.walk(source, followlinks=False):
            current_path = Path(current)
            kept_directories: list[str] = []
            for name in sorted(directory_names, key=str.casefold):
                child = current_path / name
                if name.casefold() in excluded_names:
                    continue
                if child.is_symlink():
                    raise TemplateError(f"Resource directory contains a symlink: {child}")
                kept_directories.append(name)
            directory_names[:] = kept_directories
            for name in sorted(file_names, key=str.casefold):
                if name.casefold() in excluded_names:
                    continue
                child = current_path / name
                if child.is_symlink() or not child.is_file():
                    raise TemplateError(f"Resource payload must be a regular file: {child}")
                relative = PurePosixPath(child.relative_to(source).as_posix())
                entries.append(CopyEntry(child, destination / relative))

    destinations: dict[str, PurePosixPath] = {}
    for entry in entries:
        key = entry.destination.as_posix().casefold()
        if entry.destination.parts[0].casefold() in {"template.json", OWNER_MARKER.casefold()}:
            raise TemplateError(f"Payload destination is reserved for template metadata: {entry.destination}")
        if key in destinations:
            raise TemplateError(f"Payload destination collision: {entry.destination}")
        destinations[key] = entry.destination
    for key, destination in destinations.items():
        parent = destination.parent
        while parent != PurePosixPath("."):
            if parent.as_posix().casefold() in destinations:
                raise TemplateError(f"Payload file/directory collision: {destination}")
            parent = parent.parent
    return sorted(entries, key=lambda entry: entry.destination.as_posix().casefold())


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_layout(layout: dict[str, Any]) -> dict[str, Any]:
    if layout.get("schema_version") != SCHEMA:
        raise TemplateError(f"Unsupported layout schema: {layout.get('schema_version')!r}")
    identity_source: dict[str, Any] = {}
    identity_file = layout.get("identity_file")
    if identity_file is not None:
        identity_path = _absolute_source(identity_file, "identity_file")
        identity_source = _load_json(identity_path)
        if identity_source.get("schema_version") != SCHEMA:
            raise TemplateError(f"Unsupported build identity schema: {identity_source.get('schema_version')!r}")
    identity: dict[str, Any] = {}
    for key in ("platform", "architecture", "configuration", "sdk_source_id", "compiler_id", "compiler_version"):
        layout_value = layout.get(key)
        source_value = identity_source.get(key)
        if layout_value is not None and source_value is not None and layout_value != source_value:
            raise TemplateError(f"Layout and build identity disagree on {key}.")
        value = source_value if source_value is not None else layout_value
        if not isinstance(value, str) or not value:
            raise TemplateError(f"Layout field {key} must be a non-empty string.")
        identity[key] = value
    if identity["platform"] != "Windows" or identity["architecture"] != "x64":
        raise TemplateError("Only Windows x64 runtime layouts are supported.")
    with_editor = identity_source.get("with_editor", layout.get("with_editor", False))
    if with_editor is not False:
        raise TemplateError("Runtime build identity must set with_editor to false.")
    packages = identity_source.get("packages", layout.get("package_sources", {}))
    if not isinstance(packages, dict) or not all(
        isinstance(name, str) and name and isinstance(source_id, str) and source_id
        for name, source_id in packages.items()
    ):
        raise TemplateError("Build identity packages must map package names to source IDs.")
    build_options = identity_source.get("build_options", {})
    if not isinstance(build_options, dict):
        raise TemplateError("Build identity build_options must be an object.")
    package_options = identity_source.get("package_options", {})
    if not isinstance(package_options, dict):
        raise TemplateError("Build identity package_options must be an object.")
    identity["schema_version"] = SCHEMA
    identity["with_editor"] = False
    identity["packages"] = packages
    identity["build_options"] = build_options
    identity["package_options"] = package_options
    identity["host_source_id"] = identity_source.get("host_source_id")
    return identity


def _normalize_packages(
    packages: Any, inventory_paths: set[str], identity_packages: dict[str, str]
) -> list[dict[str, Any]]:
    if not isinstance(packages, list):
        raise TemplateError("packages must be an array.")
    normalized: list[dict[str, Any]] = []
    names: set[str] = set()
    for index, item in enumerate(packages):
        if not isinstance(item, dict):
            raise TemplateError(f"packages[{index}] must be an object.")
        name = item.get("name")
        source_id = (item.get("source_id") or identity_packages.get(name)) if isinstance(name, str) else None
        dependencies = item.get("dependencies", [])
        if not isinstance(name, str) or not name or name.casefold() in names:
            raise TemplateError(f"packages[{index}].name must be unique and non-empty.")
        if not isinstance(source_id, str) or not source_id:
            raise TemplateError(f"packages[{index}].source_id must be non-empty.")
        identity_source_id = identity_packages.get(name)
        if identity_source_id is None:
            raise TemplateError(f"Package {name} is absent from the build identity package map.")
        if source_id != identity_source_id:
            raise TemplateError(f"Package {name} source ID disagrees with the build identity.")
        if not isinstance(dependencies, list) or not all(isinstance(value, str) and value for value in dependencies):
            raise TemplateError(f"packages[{index}].dependencies must contain names.")
        names.add(name.casefold())
        package = {"name": name, "source_id": source_id, "dependencies": dependencies}
        for field in ("library", "manifest"):
            value = _relative_path(item.get(field), f"packages[{index}].{field}").as_posix()
            if value.casefold() not in inventory_paths:
                raise TemplateError(f"Package {name} references missing {field}: {value}")
            package[field] = value
        pdb = item.get("pdb")
        if pdb is not None:
            pdb = _relative_path(pdb, f"packages[{index}].pdb").as_posix()
            if pdb.casefold() not in inventory_paths:
                raise TemplateError(f"Package {name} references missing pdb: {pdb}")
        package["pdb"] = pdb
        roots = item.get("resources", [])
        if not isinstance(roots, list):
            raise TemplateError(f"packages[{index}].resources must be an array.")
        package_roots: list[str] = []
        for root_index, value in enumerate(roots):
            root = _relative_path(
                value, f"packages[{index}].resources[{root_index}]", allow_root=True
            ).as_posix()
            prefix = "" if root == "." else f"{root.casefold()}/"
            if root != "." and not any(path == root.casefold() or path.startswith(prefix) for path in inventory_paths):
                raise TemplateError(f"Package {name} references missing resource root: {root}")
            package_roots.append(root)
        package["resources"] = package_roots
        normalized.append(package)
    identity_names = {name.casefold() for name in identity_packages}
    if names != identity_names:
        missing = sorted(identity_names - names)
        unexpected = sorted(names - identity_names)
        raise TemplateError(f"Package layout does not match build identity; missing={missing}, unexpected={unexpected}")
    for package in normalized:
        unknown = [dependency for dependency in package["dependencies"] if dependency.casefold() not in names]
        if unknown:
            raise TemplateError(f"Package {package['name']} has unknown dependencies: {unknown}")
    return normalized


def _output_is_owned(output: Path) -> bool:
    marker = output / OWNER_MARKER
    if not marker.is_file():
        return False
    try:
        value = json.loads(marker.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    return (
        isinstance(value, dict)
        and value.get("schema_version") == SCHEMA
        and value.get("kind") == "EvoEngineRuntimeTemplate"
    )


def _rename_with_retry(source: Path, destination: Path) -> None:
    attempts = 6 if os.name == "nt" else 1
    for attempt in range(attempts):
        try:
            source.rename(destination)
            return
        except OSError as error:
            if attempt + 1 == attempts or getattr(error, "winerror", None) not in (5, 32, 33):
                raise
            time.sleep(0.05 * (attempt + 1))


def _remove_owned_backup(path: Path) -> None:
    attempts = 6 if os.name == "nt" else 1
    for attempt in range(attempts):
        try:
            shutil.rmtree(path)
            return
        except OSError as error:
            if attempt + 1 == attempts or getattr(error, "winerror", None) not in (5, 32, 33):
                raise
            time.sleep(0.05 * (attempt + 1))


def _resolved_sibling(parent: Path, name: str) -> Path:
    path = (parent / name).resolve(strict=False)
    if path.parent != parent:
        raise TemplateError(f"Template replacement path escapes its parent: {path}")
    return path


def build_template(layout_path: Path, output: Path) -> dict[str, Any]:
    layout = _load_json(layout_path)
    identity = _validate_layout(layout)
    entries = _collect_entries(layout)
    if output.is_symlink():
        raise TemplateError(f"Template output must not be a symlink: {output}")
    output = output.resolve(strict=False)
    if output.exists() and not output.is_dir():
        raise TemplateError(f"Template output is not a directory: {output}")
    if output.exists() and any(output.iterdir()) and not _output_is_owned(output):
        raise TemplateError(f"Refusing to replace unknown non-empty directory: {output}")

    parent = output.parent.resolve(strict=False)
    parent.mkdir(parents=True, exist_ok=True)
    staging = _resolved_sibling(parent, f".{output.name}.tmp-{os.getpid()}-{uuid.uuid4().hex}")
    backup = _resolved_sibling(parent, f".{output.name}.old-{os.getpid()}-{uuid.uuid4().hex}")
    staging.mkdir()
    try:
        inventory: list[dict[str, Any]] = []
        for entry in entries:
            destination = staging.joinpath(*entry.destination.parts)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(entry.source, destination)
            inventory.append(
                {
                    "path": entry.destination.as_posix(),
                    "sha256": _sha256(destination),
                    "size": destination.stat().st_size,
                }
            )
        inventory_paths = {item["path"].casefold() for item in inventory}
        packages = _normalize_packages(layout.get("packages", []), inventory_paths, identity["packages"])
        host = layout.get("host")
        if host is not None and not isinstance(host, dict):
            raise TemplateError("host must be null or an object.")
        content = {
            "schema_version": SCHEMA,
            "identity": identity,
            "packages": packages,
            "host": host,
            "files": inventory,
        }
        canonical = json.dumps(content, sort_keys=True, separators=(",", ":")).encode("utf-8")
        content["template_id"] = hashlib.sha256(canonical).hexdigest()
        (staging / "template.json").write_text(json.dumps(content, indent=2) + "\n", encoding="utf-8")
        marker = {
            "schema_version": SCHEMA,
            "kind": "EvoEngineRuntimeTemplate",
            "template_id": content["template_id"],
        }
        (staging / OWNER_MARKER).write_text(json.dumps(marker, indent=2) + "\n", encoding="utf-8")
        previous_output = output.exists()
        if previous_output:
            _rename_with_retry(output, backup)
        try:
            _rename_with_retry(staging, output)
        except BaseException:
            if previous_output and backup.exists() and not output.exists():
                _rename_with_retry(backup, output)
            raise
        if previous_output:
            _remove_owned_backup(backup)
        return content
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", type=Path, required=True, help="CMake-generated runtime layout JSON.")
    parser.add_argument("--output", type=Path, required=True, help="Builder-owned template output directory.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        template = build_template(args.layout.resolve(), args.output.absolute())
    except (TemplateError, OSError) as error:
        print(f"Runtime template error: {error}", file=sys.stderr)
        return 1
    print(f"Prepared runtime template {template['template_id']} at {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
