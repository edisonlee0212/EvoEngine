#!/usr/bin/env python3
"""Build and install EvoEngine app runtimes with CMake presets."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

DEFAULT_DISABLED_DEMO_APP_ARGS = [
    "-DEvoEngine_App-DemoApp=OFF",
    "-DEvoEngine_App-DDGIApp=OFF",
    "-DEvoEngine_App-EcoSysLabApp=OFF",
    "-DEvoEngine_App-DigitalAgricultureApp=OFF",
    "-DEvoEngine_App-LSystemApp=OFF",
]
INSTALL_CONFIG_MARKER = ".evoengine-install-config"
RUNTIME_TEMPLATE_MARKER = ".evoengine-runtime-template.json"
RUNTIME_TEMPLATE_SCHEMA = 1


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def format_command(command: list[str]) -> str:
    if os.name == "nt":
        return subprocess.list2cmdline(command)
    return shlex.join(command)


def run_step(name: str, command: list[str]) -> None:
    print(f"\n==> {name}", flush=True)
    print(format_command(command), flush=True)
    completed = subprocess.run(command, cwd=repo_root())
    if completed.returncode != 0:
        raise SystemExit(completed.returncode)


def load_cmake_presets(root: Path) -> dict[str, Any]:
    presets_path = root / "CMakePresets.json"
    if not presets_path.exists():
        raise SystemExit(f"Missing CMakePresets.json: {presets_path}")
    return json.loads(presets_path.read_text(encoding="utf-8"))


def expand_preset_path(value: str, root: Path) -> Path:
    expanded = value.replace("${sourceDir}", str(root))
    path = Path(expanded)
    if not path.is_absolute():
        path = root / path
    return path.resolve()


def configure_preset(presets: dict[str, Any], name: str) -> dict[str, Any]:
    for preset in presets.get("configurePresets", []):
        if preset.get("name") == name:
            return preset
    raise SystemExit(f"Configure preset not found: {name}")


def build_preset_exists(presets: dict[str, Any], name: str) -> bool:
    return any(preset.get("name") == name for preset in presets.get("buildPresets", []))


def build_dir_for_preset(root: Path, presets: dict[str, Any], preset_name: str) -> Path:
    preset = configure_preset(presets, preset_name)
    binary_dir = preset.get("binaryDir")
    if not binary_dir:
        raise SystemExit(f"Configure preset '{preset_name}' does not define binaryDir.")
    return expand_preset_path(binary_dir, root)


def install_dir_for_preset(root: Path, presets: dict[str, Any], preset_name: str) -> Path:
    preset = configure_preset(presets, preset_name)
    install_dir = preset.get("installDir")
    if not install_dir:
        cache_variables = preset.get("cacheVariables", {})
        install_dir = cache_variables.get("CMAKE_INSTALL_PREFIX")
    if not install_dir:
        raise SystemExit(f"Configure preset '{preset_name}' does not define installDir.")
    return expand_preset_path(install_dir, root)


def clean_install_dir(root: Path, install_dir: Path) -> None:
    if not install_dir.exists():
        return

    allowed_root = (root / "out" / "install").resolve()
    try:
        install_dir.resolve().relative_to(allowed_root)
    except ValueError:
        raise SystemExit(
            f"Refusing to clean install directory outside {allowed_root}: {install_dir}"
        )

    print(f"Cleaning {install_dir}")
    preserved_templates = install_dir / "bin" / "RuntimeTemplates"
    for child in install_dir.iterdir():
        if child == preserved_templates.parent:
            for bin_child in child.iterdir():
                if bin_child != preserved_templates:
                    if bin_child.is_dir():
                        shutil.rmtree(bin_child)
                    else:
                        bin_child.unlink()
        elif child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()


def installed_config(install_dir: Path) -> str | None:
    try:
        return (install_dir / INSTALL_CONFIG_MARKER).read_text(encoding="utf-8").strip()
    except OSError:
        return None


def load_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise SystemExit(f"Cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise SystemExit(f"Expected a JSON object in {path}.")
    return value


def build_identity(document: dict[str, Any]) -> dict[str, Any]:
    identity = document.get("identity", document)
    if not isinstance(identity, dict):
        raise SystemExit("Build identity must be a JSON object.")
    return identity


def validate_runtime_identity(editor_build: dict[str, Any], template: dict[str, Any]) -> None:
    editor = build_identity(editor_build)
    runtime = build_identity(template)
    keys = ("configuration", "sdk_source_id", "compiler_id", "compiler_version", "platform", "architecture")
    missing = [key for key in keys if not isinstance(editor.get(key), str) or not isinstance(runtime.get(key), str)]
    if missing:
        raise SystemExit(f"Missing build identity fields: {', '.join(missing)}")
    mismatches = [key for key in keys if editor[key] != runtime[key]]
    if mismatches:
        details = ", ".join(f"{key}: editor={editor[key]!r}, runtime={runtime[key]!r}" for key in mismatches)
        raise SystemExit(f"Editor/runtime build identity mismatch: {details}")
    if editor.get("with_editor") is not True or runtime.get("with_editor") is not False:
        raise SystemExit("Expected editor with_editor=true and runtime with_editor=false.")
    if editor.get("packages") != runtime.get("packages"):
        raise SystemExit("Editor/runtime package source identities differ; rebuild the matching package set.")


def cmake_build_option_args(editor_build: dict[str, Any]) -> list[str]:
    identity = build_identity(editor_build)
    options: dict[str, Any] = {}
    for field in ("build_options", "package_options"):
        values = identity.get(field, {})
        if not isinstance(values, dict):
            raise SystemExit(f"Editor {field} must be a JSON object.")
        for name, value in values.items():
            if name in options and options[name] != value:
                raise SystemExit(f"Editor build option {name} has conflicting values.")
            options[name] = value
    arguments: list[str] = []
    for name in sorted(options):
        value = options[name]
        if (
            not isinstance(name, str)
            or not name
            or not all(character.isalnum() or character in "_.-" for character in name)
        ):
            raise SystemExit(f"Invalid editor build option name: {name!r}")
        if isinstance(value, bool):
            value = "ON" if value else "OFF"
        elif not isinstance(value, (str, int, float)):
            raise SystemExit(f"Invalid value for editor build option {name}: {value!r}")
        arguments.append(f"-D{name}={value}")
    return arguments


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_runtime_template(path: Path, expected_id: str | None = None) -> dict[str, Any]:
    template = load_json_object(path / "template.json")
    template_id = template.get("template_id")
    inventory = template.get("files")
    if template.get("schema_version") != RUNTIME_TEMPLATE_SCHEMA or not isinstance(template_id, str) or not template_id:
        raise SystemExit(f"Invalid runtime template metadata: {path / 'template.json'}")
    hashed_content = dict(template)
    del hashed_content["template_id"]
    canonical = json.dumps(hashed_content, sort_keys=True, separators=(",", ":")).encode("utf-8")
    if hashlib.sha256(canonical).hexdigest() != template_id:
        raise SystemExit(f"Runtime template content hash mismatch: {path / 'template.json'}")
    if expected_id is not None and template_id != expected_id:
        raise SystemExit(f"Runtime template ID mismatch at {path}: expected {expected_id}, found {template_id}")
    if not isinstance(inventory, list):
        raise SystemExit(f"Invalid runtime template inventory: {path / 'template.json'}")
    expected_files = {"template.json", RUNTIME_TEMPLATE_MARKER}
    for index, item in enumerate(inventory):
        if not isinstance(item, dict) or not isinstance(item.get("path"), str):
            raise SystemExit(f"Invalid runtime template inventory entry {index}.")
        relative = Path(item["path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise SystemExit(f"Invalid runtime template inventory path: {item['path']}")
        payload = path / relative
        if not payload.is_file() or payload.stat().st_size != item.get("size") or sha256_file(payload) != item.get("sha256"):
            raise SystemExit(f"Runtime template payload verification failed: {payload}")
        expected_files.add(relative.as_posix())
    actual_files = {item.relative_to(path).as_posix() for item in path.rglob("*") if item.is_file()}
    if actual_files != expected_files:
        raise SystemExit(f"Runtime template contains undeclared or missing files: {path}")
    marker = load_json_object(path / RUNTIME_TEMPLATE_MARKER)
    if marker.get("kind") != "EvoEngineRuntimeTemplate" or marker.get("template_id") != template_id:
        raise SystemExit(f"Invalid runtime template ownership marker: {path}")
    return template


def publish_runtime_template(template_dir: Path, install_dir: Path) -> Path:
    template = verify_runtime_template(template_dir)
    identity = build_identity(template)
    template_id = template["template_id"]
    destination_root = (
        install_dir
        / "bin"
        / "RuntimeTemplates"
        / identity["platform"]
        / identity["architecture"]
        / identity["configuration"]
    )
    destination = destination_root / template_id
    destination_root.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        verify_runtime_template(destination, template_id)
    else:
        staging = destination_root / f".{template_id}.tmp-{os.getpid()}"
        if staging.exists():
            raise SystemExit(f"Refusing to replace unknown template staging path: {staging}")
        shutil.copytree(template_dir, staging)
        staging.rename(destination)

    pointer_path = destination_root / "current.json"
    if pointer_path.exists():
        pointer = load_json_object(pointer_path)
        if (
            pointer.get("schema_version") != RUNTIME_TEMPLATE_SCHEMA
            or pointer.get("kind") != "EvoEngineRuntimeTemplatePointer"
        ):
            raise SystemExit(f"Refusing to overwrite unknown template pointer: {pointer_path}")
    pointer = {
        "schema_version": RUNTIME_TEMPLATE_SCHEMA,
        "kind": "EvoEngineRuntimeTemplatePointer",
        "template_id": template_id,
    }
    temporary_pointer = pointer_path.with_name(f".{pointer_path.name}.tmp-{os.getpid()}")
    temporary_pointer.write_text(json.dumps(pointer, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary_pointer, pointer_path)
    return destination


def open_folder(path: Path) -> None:
    if os.name != "nt":
        return
    os.startfile(path)  # type: ignore[attr-defined]


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build and install EvoEngine app runtimes through CMake presets."
    )
    parser.add_argument(
        "--preset",
        default="vs2026-x64",
        help="CMake configure preset to use. Defaults to vs2026-x64.",
    )
    parser.add_argument(
        "--config",
        default="RelWithDebInfo",
        choices=["Debug", "RelWithDebInfo", "Release"],
        help="Configuration to install. Defaults to RelWithDebInfo.",
    )
    parser.add_argument(
        "--runtime-build-dir",
        type=Path,
        help="Use an already configured runtime build directory instead of the <preset>-runtime preset.",
    )
    parser.add_argument(
        "--no-open",
        action="store_true",
        help="Do not open the install output folder after a successful install.",
    )
    parser.add_argument(
        "--no-clean-install",
        action="store_true",
        help="Do not clean the install directory before building the install target.",
    )
    parser.add_argument(
        "--incremental",
        action="store_true",
        help=(
            "Reuse the existing build and install directories. Skips configure when "
            "the preset build cache already exists."
        ),
    )
    parser.add_argument(
        "--cmake-arg",
        action="append",
        default=[],
        help="Extra argument passed to CMake configure. May be repeated.",
    )
    parser.add_argument(
        "--jobs",
        type=positive_int,
        default=os.cpu_count() or 1,
        help="Maximum parallel build processes. Defaults to all logical CPUs.",
    )
    parser.add_argument("--verbose", action="store_true", help="Pass verbose output to CMake.")
    return parser.parse_args()


def main() -> int:
    if os.name != "nt":
        print(
            "Scripts/install_apps.py targets Windows Visual Studio presets. "
            "Use direct CMake commands on non-Windows platforms.",
            file=sys.stderr,
        )
        return 1

    args = parse_args()
    root = repo_root()
    presets = load_cmake_presets(root)
    build_dir = build_dir_for_preset(root, presets, args.preset)
    install_dir = install_dir_for_preset(root, presets, args.preset)
    build_preset = f"install-{args.preset}-{args.config}"

    if not build_preset_exists(presets, build_preset):
        raise SystemExit(f"Build preset not found: {build_preset}")

    if args.incremental and not args.cmake_arg and (build_dir / "CMakeCache.txt").exists():
        print(f"Skipping configure; reusing {build_dir / 'CMakeCache.txt'}", flush=True)
    else:
        configure_command = [
            "cmake",
            "--preset",
            args.preset,
            "-DBUILD_TESTING=OFF",
            *DEFAULT_DISABLED_DEMO_APP_ARGS,
        ]
        if args.verbose:
            configure_command.append("--log-level=VERBOSE")
        configure_command.extend(args.cmake_arg)
        run_step("Configure Visual Studio project", configure_command)

    if not args.no_clean_install and (
        not args.incremental or installed_config(install_dir) != args.config
    ):
        clean_install_dir(root, install_dir)

    build_command = [
        "cmake",
        "--build",
        "--preset",
        build_preset,
        "--parallel",
        str(args.jobs),
    ]
    if args.verbose:
        build_command.append("--verbose")
    build_command.extend(
        [
            "--",
            "/p:UseMultiToolTask=true",
            "/p:EnforceProcessCountAcrossBuilds=true",
            f"/p:MultiProcMaxCount={args.jobs}",
        ]
    )
    run_step(f"Build and install {args.config}", build_command)
    (install_dir / INSTALL_CONFIG_MARKER).write_text(
        f"{args.config}\n", encoding="utf-8"
    )

    editor_build = load_json_object(install_dir / "bin" / "evoengine-build.json")
    runtime_preset = f"{args.preset}-runtime"
    if args.runtime_build_dir is None:
        runtime_build_dir = build_dir_for_preset(root, presets, runtime_preset)
        runtime_configure_command = [
            "cmake",
            "--preset",
            runtime_preset,
            "-DBUILD_TESTING=OFF",
            *DEFAULT_DISABLED_DEMO_APP_ARGS,
            *cmake_build_option_args(editor_build),
        ]
        if args.verbose:
            runtime_configure_command.append("--log-level=VERBOSE")
        run_step("Configure native runtime template", runtime_configure_command)
    else:
        runtime_build_dir = args.runtime_build_dir
        if not runtime_build_dir.is_absolute():
            runtime_build_dir = root / runtime_build_dir
        runtime_build_dir = runtime_build_dir.resolve()
        if not (runtime_build_dir / "CMakeCache.txt").is_file():
            raise SystemExit(f"Runtime build directory is not configured: {runtime_build_dir}")

    runtime_build_command = [
        "cmake",
        "--build",
        str(runtime_build_dir),
        "--config",
        args.config,
        "--target",
        "EvoEngineRuntimePayload",
        "--parallel",
        str(args.jobs),
    ]
    if args.verbose:
        runtime_build_command.append("--verbose")
    runtime_build_command.extend(
        [
            "--",
            "/p:UseMultiToolTask=true",
            "/p:EnforceProcessCountAcrossBuilds=true",
            f"/p:MultiProcMaxCount={args.jobs}",
        ]
    )
    run_step(f"Prepare native runtime template {args.config}", runtime_build_command)

    template_dir = runtime_build_dir / "RuntimePayload" / args.config / "template"
    template = verify_runtime_template(template_dir)
    validate_runtime_identity(editor_build, template)
    published_template = publish_runtime_template(template_dir, install_dir)

    print(f"\nInstall output: {install_dir}")
    print(f"App binaries: {install_dir / 'bin'}")
    print(f"Python runtime: {install_dir / 'python'}")
    print(f"Native runtime template: {published_template}")
    if not args.no_open:
        open_folder(install_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
