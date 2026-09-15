"""Write native build provenance without changing files whose contents are unchanged."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess


def file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def tree_records(root: Path, paths: list[str], editor: bool | None = None) -> list[tuple[str, str]]:
    files: set[Path] = set()
    for name in paths:
        path = root / name
        if path.is_file():
            files.add(path)
        elif path.is_dir():
            files.update(p for p in path.rglob("*") if p.is_file() and ".git" not in p.parts)
        else:
            raise ValueError(f"Missing native source input: {path}")
    return [(p.relative_to(root).as_posix(), file_hash(p)) for p in sorted(files)
            if editor is None or ("Editor" in p.relative_to(root).parts) == editor]


def git(root: Path, *arguments: str) -> bytes:
    return subprocess.run(
        ["git", "-C", str(root), *arguments], check=True, capture_output=True
    ).stdout


def dependency_records(root: Path) -> list[tuple[str, str]]:
    if not (root / ".git").exists() or not shutil.which("git"):
        return tree_records(root, ["Extern"])
    records = []
    paths = git(root, "ls-files", "-z", "--", "Extern").split(b"\0")
    paths += git(root, "ls-files", "--others", "--exclude-standard", "-z", "--", "Extern").split(b"\0")
    for encoded in sorted(set(paths)):
        if encoded:
            name = encoded.decode("utf-8")
            if (root / name).is_file():
                records.append((name, file_hash(root / name)))
    status = git(root, "submodule", "status", "--recursive", "--", "Extern").decode("utf-8")
    for line in status.splitlines():
        match = re.match(r"^[ +\-U]([0-9a-f]{40,64}) (.+?)(?: \(.*\))?$", line)
        if not match:
            raise ValueError(f"Unrecognized native dependency status: {line}")
        revision, name = match.groups()
        records.append((name + "/revision", revision))
        if line.startswith("-"):
            continue
        module = root / name
        diff = git(module, "diff", "--no-ext-diff", "--no-textconv", "--binary", "HEAD", "--", ".")
        records.append((name + "/working-tree", hashlib.sha256(diff).hexdigest()))
        for encoded in git(module, "ls-files", "--others", "--exclude-standard", "-z").split(b"\0"):
            if encoded:
                path = module / encoded.decode("utf-8")
                if path.is_file():
                    records.append((path.relative_to(root).as_posix(), file_hash(path)))
    return sorted(records)


def digest(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def identity(spec: dict) -> dict:
    root = Path(spec["source_root"]).resolve()
    native = {key: spec[key] for key in (
        "compiler_id", "compiler_version", "configuration", "platform", "architecture", "build_options"
    )}
    native["sdk_source_id"] = digest({
        "sources": tree_records(root, spec["sdk_roots"], editor=False),
        "dependencies": dependency_records(root),
        "toolchain": native.copy(),
    })
    native["schema_version"] = 1
    native["with_editor"] = spec["with_editor"]
    native["package_options"] = spec.get("package_options", {}).copy()
    native["host_source_id"] = digest(tree_records(root, spec.get("host_roots", [])))
    packages = {package["name"]: package for package in spec["packages"]}
    package_ids = {}
    resolving = set()

    def package_id(name: str) -> str:
        if name in package_ids:
            return package_ids[name]
        if name not in packages or name in resolving:
            raise ValueError(f"Missing or cyclic native package dependency: {name}")
        resolving.add(name)
        package = packages[name]
        package_ids[name] = digest({
            "sources": tree_records(root, package["roots"], editor=False),
            "dependencies": {dependency: package_id(dependency)
                             for dependency in sorted(package.get("dependencies", []))},
        })
        resolving.remove(name)
        return package_ids[name]

    native["packages"] = {name: package_id(name) for name in sorted(packages)}
    if spec["with_editor"]:
        native["editor_source_id"] = digest({
            "runtime": native["sdk_source_id"],
            "sources": tree_records(root, spec["sdk_roots"], editor=True),
        })
        editor_ids = {}
        resolving.clear()

        def editor_package_id(name: str) -> str:
            if name in editor_ids:
                return editor_ids[name]
            if name not in packages or name in resolving:
                raise ValueError(f"Missing or cyclic editor package dependency: {name}")
            resolving.add(name)
            package = packages[name]
            dependencies = package.get("editor_dependencies", [])
            if any(dependency not in package.get("dependencies", []) for dependency in dependencies):
                raise ValueError(f"Editor dependencies must also be runtime dependencies: {name}")
            editor_ids[name] = digest({
                "runtime": package_ids[name],
                "editor_sdk": native["editor_source_id"],
                "sources": tree_records(root, package["roots"], editor=True),
                "dependencies": {dependency: editor_package_id(dependency) for dependency in sorted(dependencies)},
            })
            resolving.remove(name)
            return editor_ids[name]

        native["editor_packages"] = {name: editor_package_id(name) for name in sorted(packages)}
    return native


def write_if_changed(path: str, content: str) -> None:
    destination = Path(path)
    if destination.exists() and destination.read_text(encoding="utf-8") == content:
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(content, encoding="utf-8", newline="\n")
    temporary.replace(destination)


def generate(spec: dict, verify: bool = False) -> dict:
    current = identity(spec)
    if verify:
        recorded = json.loads(Path(spec["metadata_json"]).read_text(encoding="utf-8"))
        if current != recorded:
            raise ValueError("Native source inputs changed during the build; rebuild before preparing a runtime template.")
        return current
    definitions = {
        "EVOENGINE_SDK_SOURCE_ID": current["sdk_source_id"],
        "EVOENGINE_NATIVE_COMPILER_ID": current["compiler_id"],
        "EVOENGINE_NATIVE_COMPILER_VERSION": current["compiler_version"],
        "EVOENGINE_NATIVE_BUILD_CONFIGURATION": current["configuration"],
        "EVOENGINE_NATIVE_PLATFORM": current["platform"],
        "EVOENGINE_NATIVE_ARCHITECTURE": current["architecture"],
    }
    header = "#pragma once\n" + "".join(f"#define {key} {json.dumps(value)}\n" for key, value in definitions.items())
    write_if_changed(spec["sdk_header"], header)
    cmake = "".join(f"set({key} {json.dumps(value)})\n" for key, value in definitions.items())
    cmake += f"set(EVOENGINE_NATIVE_WITH_EDITOR {'true' if current['with_editor'] else 'false'})\n"
    if current["with_editor"]:
        editor_id = current["editor_source_id"]
        write_if_changed(spec.get("editor_header", str(Path(spec["sdk_header"]).with_name("EvoEngineEditorBuildIdentity.hpp"))),
                         f'#pragma once\n#define EVOENGINE_EDITOR_SOURCE_ID "{editor_id}"\n')
        cmake += f'set(EVOENGINE_EDITOR_SOURCE_ID "{editor_id}")\n'
    for package in spec["packages"]:
        source_id = current["packages"][package["name"]]
        write_if_changed(package["header"], f'#pragma once\n#define EVOENGINE_PACKAGE_SOURCE_ID "{source_id}"\n')
        cmake += f'set(EVOENGINE_PACKAGE_SOURCE_ID_{package["name"]} "{source_id}")\n'
        if current["with_editor"]:
            editor_id = current["editor_packages"][package["name"]]
            write_if_changed(package.get("editor_header", str(Path(package["header"]).with_name("EvoEngineEditorPackageBuildIdentity.hpp"))),
                             f'#pragma once\n#define EVOENGINE_EDITOR_PACKAGE_SOURCE_ID "{editor_id}"\n')
            cmake += f'set(EVOENGINE_EDITOR_PACKAGE_SOURCE_ID_{package["name"]} "{editor_id}")\n'
    write_if_changed(spec["metadata_json"], json.dumps(current, indent=2, sort_keys=True) + "\n")
    write_if_changed(spec["metadata_cmake"], cmake)
    return current


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()
    generate(json.loads(args.spec.read_text(encoding="utf-8")), args.verify)


if __name__ == "__main__":
    main()
