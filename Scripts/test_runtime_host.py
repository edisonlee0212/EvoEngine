#!/usr/bin/env python3
"""Exercise an EvoEngine runtime template as a relocatable, read-only distribution."""

from __future__ import annotations

import argparse
import ctypes
from ctypes import wintypes
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path, PureWindowsPath
from typing import Any

WORK_MARKER = ".evoengine-runtime-host-test.json"
WORK_KIND = "EvoEngineRuntimeHostTest"
SCENE_HANDLE = 0xE703_0000_0000_0001
CAMERA_ENTITY_HANDLE = 0xE703_0000_0000_0002
DORMANT_ASSET_HANDLE = 0xE703_0000_0000_0003
SOURCE_START_SCENE_HANDLE = 0xE703_0000_0000_0004


class RuntimeHostTestError(RuntimeError):
    pass


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeHostTestError(f"Cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise RuntimeHostTestError(f"Expected a JSON object in {path}.")
    return value


def _is_within(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def _prepare_work_dir(work_dir: Path, template: Path) -> Path:
    work_dir = work_dir.absolute()
    if work_dir.is_symlink() or (hasattr(work_dir, "is_junction") and work_dir.is_junction()):
        raise RuntimeHostTestError(f"Work directory must not be a symlink or junction: {work_dir}")
    work_dir = work_dir.resolve(strict=False)
    template = template.resolve(strict=True)
    if work_dir == Path(work_dir.anchor) or work_dir == Path.home().resolve():
        raise RuntimeHostTestError(f"Refusing unsafe work directory: {work_dir}")
    if _is_within(template, work_dir):
        raise RuntimeHostTestError("The runtime template must be outside the disposable work directory.")
    if work_dir.exists() and not work_dir.is_dir():
        raise RuntimeHostTestError(f"Work directory must be a regular directory: {work_dir}")
    if work_dir.exists() and any(work_dir.iterdir()):
        marker = _load_json(work_dir / WORK_MARKER)
        if marker.get("schema_version") != 1 or marker.get("kind") != WORK_KIND:
            raise RuntimeHostTestError(f"Refusing to replace an unowned work directory: {work_dir}")
        if Path(marker.get("owned_root", "")).resolve(strict=False) != work_dir:
            raise RuntimeHostTestError(f"Work directory ownership marker does not match its path: {work_dir}")
        shutil.rmtree(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    (work_dir / WORK_MARKER).write_text(
        json.dumps({"schema_version": 1, "kind": WORK_KIND, "owned_root": str(work_dir)}, indent=2) + "\n",
        encoding="utf-8",
    )
    return work_dir


def _template_host(template: Path) -> tuple[dict[str, Any], Path]:
    metadata = _load_json(template / "template.json")
    if metadata.get("schema_version") != 1 or not isinstance(metadata.get("identity"), dict):
        raise RuntimeHostTestError("Runtime template metadata is invalid.")
    host = metadata.get("host")
    if not isinstance(host, dict) or not isinstance(host.get("executable"), str):
        raise RuntimeHostTestError("Runtime template does not contain a native host executable.")
    executable_name = host["executable"]
    relative = PureWindowsPath(executable_name)
    if relative.is_absolute() or not relative.parts or any(part in ("", ".", "..") for part in relative.parts):
        raise RuntimeHostTestError(f"Runtime host path is invalid: {executable_name}")
    executable = (template / Path(*relative.parts)).resolve(strict=True)
    if not _is_within(executable, template.resolve(strict=True)) or not executable.is_file():
        raise RuntimeHostTestError(f"Runtime host is outside the template or missing: {executable_name}")
    return metadata, executable.relative_to(template.resolve(strict=True))


def _scene_yaml() -> str:
    return f"""global_reflection_probe_fallback:
  asset_handle_: 0
  type_name_: ""
environmental_lighting:
  asset_handle_: 0
  type_name_: ""
main_camera:
  entity_handle_: {CAMERA_ENTITY_HANDLE}
  private_component_type_name_: Camera
entity_metadata_list:
  - n: Main Camera
    h: {CAMERA_ENTITY_HANDLE}
    e: true
    s: false
    r: {CAMERA_ENTITY_HANDLE}
    pc:
      - tn: Camera
        e: true
        render_mode: Rasterization
        background_source: ClearColor
        clear_color: [0, 0, 0, 1]
        near_distance: 0.1
        far_distance: 100
        fov: 60
        skybox:
          asset_handle_: 0
          type_name_: ""
        background_environment:
          asset_handle_: 0
          type_name_: ""
        post_processing_stack_ref:
          asset_handle_: 0
          type_name_: ""
systems_: []
data_component_storage_list:
  - entity_size: 130
    chunk_capacity: 126
    entity_alive_count: 1
    data_component_types:
      - type_name: Transform
        type_size: 64
        type_offset: 0
      - type_name: GlobalTransform
        type_size: 64
        type_offset: 64
      - type_name: TransformUpdateFlag
        type_size: 2
        type_offset: 128
    chunk_array:
      - h: {CAMERA_ENTITY_HANDLE}
        dc:
          - d: !!binary "AACAPwAAAAAAAAAAAAAAAAAAAAAAAIA/AAAAAAAAAAAAAAAAAAAAAAAAgD8AAAAAAAAAAAAAoEAAACBBAACAPw=="
          - d: !!binary "AACAPwAAAAAAAAAAAAAAAAAAAAAAAIA/AAAAAAAAAAAAAAAAAAAAAAAAgD8AAAAAAAAAAAAAAAAAAAAAAACAPw=="
          - d: !!binary "AAE="
"""


def _write_asset(assets: Path, filename: str, handle: int, type_name: str, contents: str) -> Path:
    path = assets / filename
    path.write_text(contents, encoding="utf-8")
    metadata = path.with_name(path.name + ".evefilemeta")
    metadata.write_text(
        f"asset_extension_: {path.suffix}\n"
        f"asset_file_name_: {path.stem}\n"
        f"asset_type_name_: {type_name}\n"
        f"asset_handle_: {handle}\n",
        encoding="utf-8",
    )
    return metadata


def _write_distribution_configuration(distribution: Path, template: dict[str, Any]) -> Path:
    project = distribution / "Project"
    assets = project / "Assets"
    assets.mkdir(parents=True)
    (project / "Runtime.eveproj").write_text(f"start_scene_handle: {SCENE_HANDLE}\n", encoding="utf-8")
    scene_metadata = _write_asset(assets, "Startup.evescene", SCENE_HANDLE, "Scene", _scene_yaml())
    _write_asset(assets, "Dormant.unknown", DORMANT_ASSET_HANDLE, "UnavailableAssetProvider", "opaque payload\n")

    identity = template["identity"]
    keys = ("sdk_source_id", "compiler_id", "compiler_version", "configuration", "platform", "architecture")
    if any(not isinstance(identity.get(key), str) or not identity[key] for key in keys):
        raise RuntimeHostTestError("Runtime template identity is incomplete.")
    if identity.get("with_editor") is not False:
        raise RuntimeHostTestError("Runtime template identity must set with_editor to false.")
    lines = ["schema_version: 1", "identity:"]
    lines.extend(f"  {key}: {json.dumps(identity[key])}" for key in keys)
    lines.extend(
        [
            f"  with_editor: {str(identity.get('with_editor')).lower()}",
            'application_name: "EvoEngine Runtime Host Test"',
            'project: "Project/Runtime.eveproj"',
            "window:",
            "  width: 320",
            "  height: 240",
            "graphics:",
            "  use_mesh_shader: false",
            "  use_ray_tracing: false",
            "packages: []",
        ]
    )
    (distribution / "runtime.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return scene_metadata


def _runtime_environment(decoy: Path) -> dict[str, str]:
    environment = os.environ.copy()
    system_root = Path(os.environ.get("SystemRoot", r"C:\Windows"))
    environment["PATH"] = os.pathsep.join(
        str(path) for path in (system_root / "System32", system_root, system_root / "System32" / "Wbem")
    )
    environment["EVOENGINE_SHADER_CACHE_DIR"] = str(decoy / "shader-cache")
    environment["EVOENGINE_PIPELINE_CACHE_DIR"] = str(decoy / "pipeline-cache")
    environment["EVOENGINE_VMA_LEAK_LOG"] = str(decoy / "vma-leaks.txt")
    environment["TEMP"] = str(decoy / "temp")
    environment["TMP"] = str(decoy / "tmp")
    return environment


def _run_host(distribution: Path, executable_relative: Path, unrelated_cwd: Path, decoy: Path) -> subprocess.CompletedProcess[str]:
    executable = distribution / executable_relative
    return subprocess.run(
        [str(executable), "--no-error-dialog", "--frames", "3", "--capture", "UserData/smoke.png"],
        cwd=unrelated_cwd,
        env=_runtime_environment(decoy),
        capture_output=True,
        text=True,
        timeout=300,
        check=False,
    )


def _runtime_log(distribution: Path) -> str:
    path = distribution / "Logs" / "runtime.log"
    return path.read_text(encoding="utf-8", errors="replace") if path.is_file() else ""


def _assert_success(result: subprocess.CompletedProcess[str], distribution: Path) -> None:
    log = _runtime_log(distribution)
    if result.returncode != 0:
        raise RuntimeHostTestError(
            f"Runtime host exited with {result.returncode}.\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}\nlog:\n{log}"
        )
    for marker in (f"RUNTIME_STARTED scene={SCENE_HANDLE} packages=0", "RUNTIME_SMOKE_COMPLETE frames=3 camera=320x240"):
        if marker not in log:
            raise RuntimeHostTestError(f"Runtime log is missing success marker {marker!r}:\n{log}")
    capture = distribution / "UserData" / "smoke.png"
    if not capture.is_file() or capture.stat().st_size <= 8 or capture.read_bytes()[:8] != b"\x89PNG\r\n\x1a\n":
        raise RuntimeHostTestError(f"Runtime capture is missing or invalid: {capture}")


def _assert_isolated(*directories: Path) -> None:
    for directory in directories:
        if any(directory.iterdir()):
            raise RuntimeHostTestError(f"Runtime wrote outside its distribution: {directory}")


def _assert_error_dialog(distribution: Path, executable_relative: Path, unrelated_cwd: Path, decoy: Path) -> None:
    user32 = ctypes.WinDLL("user32", use_last_error=True)
    callback_type = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    user32.EnumWindows.argtypes = [callback_type, wintypes.LPARAM]
    user32.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
    user32.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
    user32.PostMessageW.argtypes = [wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM]
    process = subprocess.Popen([str(distribution / executable_relative)], cwd=unrelated_cwd,
                               env=_runtime_environment(decoy), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    found = False

    @callback_type
    def visit(window: int, _: int) -> bool:
        nonlocal found
        owner = wintypes.DWORD()
        user32.GetWindowThreadProcessId(window, ctypes.byref(owner))
        if owner.value == process.pid:
            title = ctypes.create_unicode_buffer(256)
            user32.GetWindowTextW(window, title, len(title))
            if title.value == "EvoEngine Runtime":
                found = True
                user32.PostMessageW(window, 0x0010, 0, 0)  # WM_CLOSE dismisses the native error dialog.
        return True

    try:
        deadline = time.monotonic() + 60
        while process.poll() is None and time.monotonic() < deadline:
            user32.EnumWindows(visit, 0)
            time.sleep(0.1)
        if not found or process.wait(timeout=5) == 0:
            raise RuntimeHostTestError("Fatal startup did not display the Windows error dialog and exit with failure.")
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_inventory(project_file: Path) -> list[dict[str, object]]:
    root = project_file.parent
    paths = [project_file, *(path for path in (root / "Assets").rglob("*") if path.is_file())]
    return [
        {
            "path": path.relative_to(root).as_posix(),
            "size": path.stat().st_size,
            "sha256": _sha256(path),
        }
        for path in sorted(paths)
    ]


def _package_payload_paths(metadata: dict[str, Any]) -> list[str]:
    paths: list[str] = []
    for package in metadata.get("packages", []):
        if not isinstance(package, dict):
            raise RuntimeHostTestError("Runtime template package metadata is invalid.")
        for key in ("library", "manifest", "pdb"):
            value = package.get(key)
            if isinstance(value, str) and value:
                paths.append(value)
        resources = package.get("resources", [])
        if not isinstance(resources, list) or any(not isinstance(value, str) for value in resources):
            raise RuntimeHostTestError("Runtime template package resource metadata is invalid.")
        paths.extend(resources)
    return paths


def _export_distribution(
    exporter: Path, template: Path, metadata: dict[str, Any], work_dir: Path
) -> tuple[Path, Path, Path]:
    source_root = work_dir / "distinct source project"
    selected_metadata = _write_distribution_configuration(source_root, metadata)
    source_project = source_root / "Project"
    _write_asset(
        source_project / "Assets",
        "SourceDefault.evescene",
        SOURCE_START_SCENE_HANDLE,
        "Scene",
        _scene_yaml(),
    )
    source_project_file = source_project / "Runtime.eveproj"
    source_project_file.write_text(
        f"start_scene_handle: {SOURCE_START_SCENE_HANDLE}\nEditorLayer:\n  source_only: true\n",
        encoding="utf-8",
    )
    source_snapshot = _snapshot_files(source_project)
    identity = dict(metadata["identity"])
    identity["with_editor"] = True
    application_name = "Runtime Smoke 应用"
    request = {
        "schema_version": 1,
        "application_name": application_name,
        "startup_scene_handle": SCENE_HANDLE,
        "project": str(source_project_file.resolve()),
        "editor_identity": identity,
        "loaded_packages": [],
        "source_inventory": _source_inventory(source_project_file),
        "runtime_config": {
            "window": {
                "mode": "windowed",
                "width": 320,
                "height": 240,
                "allow_resize": True,
                "allow_resolution_change": True,
            },
            "graphics": {"use_mesh_shader": False, "use_ray_tracing": False},
        },
    }
    request_path = work_dir / "native export request.json"
    request_path.write_text(json.dumps(request, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    distribution = work_dir / "distribution"
    distribution.mkdir()
    result = subprocess.run(
        [str(exporter), "--request", str(request_path), "--template", str(template), "--output", str(distribution)],
        cwd=work_dir,
        capture_output=True,
        text=True,
        timeout=300,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeHostTestError(
            f"Native exporter exited with {result.returncode}.\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )
    _assert_project_unchanged(source_project, source_snapshot)
    exported_project = distribution / "Project"
    if _snapshot_files(exported_project / "Assets") != _snapshot_files(source_project / "Assets"):
        raise RuntimeHostTestError("Native exporter did not copy every source asset byte-for-byte.")
    runtime_configuration = _load_json(distribution / "runtime.yaml")
    runtime_project = runtime_configuration.get("project")
    if not isinstance(runtime_project, str):
        raise RuntimeHostTestError("Exported runtime configuration does not name its project.")
    runtime_project_path = distribution / Path(*PureWindowsPath(runtime_project).parts)
    exported_project_file = _load_json(runtime_project_path)
    if (
        exported_project_file.get("start_scene_handle") != SCENE_HANDLE
        or exported_project_file.get("startup_runtime_packages") != []
        or runtime_configuration.get("packages") != []
    ):
        raise RuntimeHostTestError(
            "Exported project did not use the independently selected startup scene and package list."
        )
    for relative in _package_payload_paths(metadata):
        path = distribution / Path(*PureWindowsPath(relative).parts)
        if path.exists():
            raise RuntimeHostTestError(f"Unselected optional package payload was exported: {relative}")
    icon_files = [
        entry["path"]
        for entry in metadata.get("files", [])
        if isinstance(entry, dict)
        and isinstance(entry.get("path"), str)
        and PureWindowsPath(entry["path"]).parts[:2] == ("DefaultResources", "Icons")
    ]
    if not icon_files or any(not (distribution / Path(*PureWindowsPath(path).parts)).is_file() for path in icon_files):
        raise RuntimeHostTestError("Exported distribution did not retain the standard SDK icons.")
    executable_relative = Path(f"{application_name}.exe")
    if not (distribution / executable_relative).is_file():
        raise RuntimeHostTestError("Native exporter did not rename the runtime host executable.")
    original_host = metadata.get("host", {})
    pdb = original_host.get("pdb") if isinstance(original_host, dict) else None
    if pdb is not None:
        if not isinstance(pdb, str) or not pdb:
            raise RuntimeHostTestError("Runtime template host PDB path is invalid.")
        pdb_relative = Path(*PureWindowsPath(pdb).parts)
        if (
            not (distribution / pdb_relative).is_file()
            or _sha256(distribution / pdb_relative) != _sha256(template / pdb_relative)
        ):
            raise RuntimeHostTestError("Native exporter did not retain the runtime host PDB unchanged.")
    return distribution, executable_relative, distribution / selected_metadata.relative_to(source_root)


def _snapshot_files(directory: Path) -> dict[str, bytes]:
    return {
        path.relative_to(directory).as_posix(): path.read_bytes()
        for path in sorted(directory.rglob("*"))
        if path.is_file()
    }


def _assert_project_unchanged(project: Path, expected: dict[str, bytes]) -> None:
    if _snapshot_files(project) != expected:
        raise RuntimeHostTestError(f"Runtime modified authored project content: {project}")


def run(template: Path, work_dir: Path, exporter: Path | None = None) -> None:
    template = template.resolve(strict=True)
    metadata, template_executable_relative = _template_host(template)
    work_dir = _prepare_work_dir(work_dir, template)
    unrelated_cwd = work_dir / "unrelated-cwd"
    decoy = work_dir / "environment-cache-decoy"
    unrelated_cwd.mkdir()
    decoy.mkdir()

    if exporter is None:
        distribution = work_dir / "distribution"
        shutil.copytree(template, distribution)
        scene_metadata = _write_distribution_configuration(distribution, metadata)
        executable_relative = template_executable_relative
    else:
        exporter = exporter.resolve(strict=True)
        if not exporter.is_file():
            raise RuntimeHostTestError(f"Runtime exporter is not a file: {exporter}")
        distribution, executable_relative, scene_metadata = _export_distribution(
            exporter, template, metadata, work_dir
        )
    project_snapshot = _snapshot_files(distribution / "Project")
    _assert_success(_run_host(distribution, executable_relative, unrelated_cwd, decoy), distribution)
    if "Compiling Shaders" not in _runtime_log(distribution):
        raise RuntimeHostTestError("Cold startup did not log shader compilation.")
    _assert_project_unchanged(distribution / "Project", project_snapshot)
    _assert_isolated(unrelated_cwd, decoy)

    relocated_parent = work_dir / "relocated runtime Ω"
    relocated_parent.mkdir()
    relocated = relocated_parent / "distribution"
    distribution.rename(relocated)
    (relocated / "Logs" / "runtime.log").unlink()
    (relocated / "UserData" / "smoke.png").unlink()
    configuration_path = relocated / "runtime.yaml"
    configuration_text = configuration_path.read_text(encoding="utf-8")
    if configuration_text.lstrip().startswith("{"):
        configuration = json.loads(configuration_text)
        configuration["show_console"] = True
        configuration_path.write_text(json.dumps(configuration), encoding="utf-8")
    else:
        configuration_path.write_text(configuration_text + "\nshow_console: true\n", encoding="utf-8")
    _assert_success(_run_host(relocated, executable_relative, unrelated_cwd, decoy), relocated)
    if "Runtime console enabled." not in _runtime_log(relocated):
        raise RuntimeHostTestError("Console-enabled runtime did not preserve file logging.")
    if distribution.exists():
        raise RuntimeHostTestError(f"Original distribution still exists after relocation: {distribution}")
    _assert_project_unchanged(relocated / "Project", project_snapshot)
    _assert_isolated(unrelated_cwd, decoy)

    scene = relocated / "Project" / "Assets" / "Startup.evescene"
    scene_digest = scene.read_bytes()
    scene_metadata = relocated / scene_metadata.relative_to(distribution)
    scene_metadata.unlink()
    result = _run_host(relocated, executable_relative, unrelated_cwd, decoy)
    log = _runtime_log(relocated)
    if result.returncode == 0 or "RUNTIME_FATAL:" not in log or "Missing asset metadata" not in log:
        raise RuntimeHostTestError(
            f"Runtime accepted a startup scene without metadata.\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}\nlog:\n{log}"
        )
    if scene_metadata.exists() or scene.read_bytes() != scene_digest:
        raise RuntimeHostTestError("Strict runtime repaired or modified authoring content after a failed launch.")
    _assert_error_dialog(relocated, executable_relative, unrelated_cwd, decoy)
    _assert_isolated(unrelated_cwd, decoy)
    print(f"Runtime host integration passed in {relocated}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", required=True, type=Path, help="Prepared runtime template directory.")
    parser.add_argument("--work-dir", required=True, type=Path, help="Script-owned integration sandbox.")
    parser.add_argument(
        "--exporter",
        type=Path,
        help="Optional native exporter; when set, test its real output before host relocation and failures.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        run(args.template, args.work_dir, args.exporter)
    except (OSError, RuntimeHostTestError, subprocess.SubprocessError) as error:
        print(f"Runtime host integration failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
