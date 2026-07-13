#!/usr/bin/env python3
"""Run reproducible EvoEngine/vk_gltf_renderer Bistro ray-tracing baselines."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import unquote, urlparse

from compare_reference_render import compare_render_images, read_image


ROOT = Path(__file__).resolve().parents[1]
PINNED_EVOENGINE_BASE = "025511b21ec566f1420f0cf66fba82527e2a662c"
PINNED_REFERENCE = "f72d2f3711116261a76e7b8b0f4724e167703a55"
PINNED_NVPRO_CORE2 = "907fba3c5b7a9597e7e63a5388079b964bd6ddb4"
PINNED_BISTRO_SOURCE = "a096b939aaa5857150904a38763ebd75b19e3e45"
PINNED_BISTRO_GLTF_SHA256 = "96138eb738a85631802f03e8755157bc7661d53a2ea60f2f960fb04957ef2529"
PINNED_BISTRO_REFERENCE_GLTF_SHA256 = "4ac68acd18d97e22f934c3329a55f51475974fed8c85b80be401c7a54a0201c1"
EXPECTED_EVO_BISTRO_CLOSURE = "4521bdaae5816a96d6c55f1f7a426a3166f5e580ad1178c1815de14f40c745aa"
EXPECTED_REFERENCE_BISTRO_CLOSURE = "f45ad755356aa7859759a75dd36e5ac9e91d4a21931997f862663e72ff185770"
REFERENCE_PATCH = ROOT / "Scripts" / "reference_patches" / "vk_gltf_renderer_m0.patch"
EVO_BISTRO_PROJECT_ROOT = ROOT / "Resources" / ".generated" / "EvoEngine-DemoProjects" / "Bistro"
BISTRO_SOURCE_ROOT = ROOT / "Resources" / ".generated" / "niagara_bistro"


@dataclass(frozen=True)
class Profile:
    width: int
    height: int
    frames: int
    samples_per_frame: int = 4

    @property
    def effective_spp(self) -> int:
        return self.frames * self.samples_per_frame


PROFILES = {
    "fast": Profile(1280, 720, 16),
    "canonical": Profile(2560, 1440, 512),
}
TECHNIQUES = {
    "rtx": ("raytracing", "RayTracing", "1", "Path Trace (RTX)"),
    "rq": ("rayquery", "RayQuery", "0", "Path Trace (RQ)"),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git(repo: Path, *arguments: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *arguments], capture_output=True, text=True, encoding="utf-8", errors="replace"
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip())
    return result.stdout.strip()


def repo_state(repo: Path) -> dict[str, object]:
    status = git(repo, "status", "--porcelain")
    diff = subprocess.run(
        ["git", "-C", str(repo), "diff", "--binary", "HEAD"], capture_output=True, check=True
    ).stdout
    return {
        "path": str(repo.resolve()),
        "head": git(repo, "rev-parse", "HEAD"),
        "dirty": bool(status),
        "status": status.splitlines(),
        "working_diff_sha256": hashlib.sha256(diff).hexdigest(),
    }


def cmake_cache_values(cache_path: Path, names: tuple[str, ...]) -> dict[str, str]:
    values: dict[str, str] = {}
    if not cache_path.is_file():
        return values
    for line in cache_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line or line.startswith(("#", "//")) or "=" not in line:
            continue
        key_and_type, value = line.split("=", 1)
        key = key_and_type.split(":", 1)[0]
        if key in names:
            values[key] = value
    return values


def cmake_build_metadata(build_dir: Path, configuration: str) -> dict[str, object]:
    cache_path = build_dir / "CMakeCache.txt"
    compiler_files = sorted((build_dir / "CMakeFiles").glob("*/CMakeCXXCompiler.cmake"))
    compiler_path = compiler_files[-1] if compiler_files else None
    compiler_values: dict[str, str] = {}
    if compiler_path:
        compiler_source = compiler_path.read_text(encoding="utf-8", errors="replace")
        for name in ("CMAKE_CXX_COMPILER", "CMAKE_CXX_COMPILER_VERSION", "CMAKE_CXX_COMPILER_ID"):
            match = re.search(rf'^set\({name} "([^"]*)"\)', compiler_source, flags=re.MULTILINE)
            if match:
                compiler_values[name] = match.group(1)
    return {
        "directory": str(build_dir),
        "configuration": configuration,
        "cache_path": str(cache_path),
        "cache_sha256": sha256(cache_path) if cache_path.is_file() else None,
        "cache": cmake_cache_values(cache_path, ("CMAKE_GENERATOR", "CMAKE_CONFIGURATION_TYPES")),
        "compiler_file": None if compiler_path is None else str(compiler_path),
        "compiler_file_sha256": None if compiler_path is None else sha256(compiler_path),
        "compiler": compiler_values,
    }


def runtime_binary_manifest(executable: Path) -> dict[str, object]:
    dlls = sorted(
        (path for path in executable.parent.rglob("*") if path.is_file() and path.suffix.lower() == ".dll"),
        key=lambda path: path.relative_to(executable.parent).as_posix().lower(),
    )
    files = [executable, *dlls]
    entries = []
    combined = hashlib.sha256()
    for path in files:
        digest = sha256(path)
        relative_path = path.relative_to(executable.parent).as_posix()
        entries.append(
            {"path": str(path.resolve()), "relative_path": relative_path, "bytes": path.stat().st_size, "sha256": digest}
        )
        combined.update(relative_path.encode("utf-8"))
        combined.update(b"\0")
        combined.update(bytes.fromhex(digest))
    return {"directory": str(executable.parent.resolve()), "combined_sha256": combined.hexdigest(), "files": entries}


def verify_evo_install_matches_build(editor: Path, build_dir: Path) -> list[dict[str, object]]:
    build_runtime_dir = build_dir / "EvoEngine_App" / "RelWithDebInfo"
    pairs = (
        (editor, build_runtime_dir / editor.name),
        (editor.parent / "EvoEngine_SDK.dll", build_dir / "EvoEngine_SDK" / "RelWithDebInfo" / "EvoEngine_SDK.dll"),
    )
    results = []
    for installed, built in pairs:
        if not installed.is_file() or not built.is_file():
            raise FileNotFoundError(f"Missing installed/build artifact pair: {installed} / {built}")
        installed_sha256 = sha256(installed)
        built_sha256 = sha256(built)
        if installed_sha256 != built_sha256:
            raise RuntimeError(f"Installed artifact does not match the RelWithDebInfo build: {installed} != {built}")
        results.append(
            {
                "installed": str(installed.resolve()),
                "built": str(built.resolve()),
                "sha256": installed_sha256,
            }
        )
    return results


def gltf_dependency_manifest(gltf_path: Path) -> dict[str, object]:
    document = json.loads(gltf_path.read_text(encoding="utf-8"))
    relative_paths = {gltf_path.name}
    optional_fallbacks: list[str] = []

    def add_uri(uri: str) -> str:
        parsed = urlparse(uri)
        if parsed.scheme or parsed.netloc:
            raise ValueError(f"External glTF URI is not reproducible: {uri}")
        path = unquote(parsed.path)
        relative_paths.add(path)
        return path

    for buffer in document.get("buffers", []):
        uri = buffer.get("uri")
        if uri and not uri.startswith("data:"):
            add_uri(uri)

    images = document.get("images", [])
    for texture in document.get("textures", []):
        dds_source = texture.get("extensions", {}).get("MSFT_texture_dds", {}).get("source")
        source = dds_source if dds_source is not None else texture.get("source")
        if source is None:
            continue
        uri = images[source].get("uri")
        if uri and not uri.startswith("data:"):
            add_uri(uri)
        fallback_source = texture.get("source")
        if dds_source is not None and fallback_source is not None and fallback_source != dds_source:
            fallback_uri = images[fallback_source].get("uri")
            if fallback_uri and not (gltf_path.parent / fallback_uri).is_file():
                optional_fallbacks.append(fallback_uri)
    files = []
    combined = hashlib.sha256()
    for relative_path in sorted(relative_paths):
        path = (gltf_path if relative_path == gltf_path.name else gltf_path.parent / Path(relative_path)).resolve()
        try:
            path.relative_to(gltf_path.parent.resolve())
        except ValueError as error:
            raise ValueError(f"glTF dependency escapes the asset directory: {relative_path}") from error
        if not path.is_file():
            raise FileNotFoundError(f"Missing glTF dependency: {path}")
        digest = sha256(path)
        files.append({"path": relative_path, "bytes": path.stat().st_size, "sha256": digest})
        combined.update(relative_path.encode("utf-8"))
        combined.update(b"\0")
        combined.update(bytes.fromhex(digest))
    return {
        "gltf": str(gltf_path.resolve()),
        "combined_sha256": combined.hexdigest(),
        "texture_source_policy": "MSFT_texture_dds when present; core source otherwise",
        "missing_optional_core_fallbacks": sorted(optional_fallbacks),
        "files": files,
    }


def generate_bistro_reference_asset(source_root: Path, output: Path) -> dict[str, object]:
    source = (source_root / "bistro.gltf").resolve()
    output = output.resolve()
    if output.parent != source_root.resolve():
        raise ValueError("The generated reference glTF must remain beside the pinned Bistro dependency tree")
    if sha256(source) != PINNED_BISTRO_GLTF_SHA256:
        raise RuntimeError("Pinned Bistro source glTF hash does not match the M0 input")
    document = json.loads(source.read_text(encoding="utf-8-sig"))
    try:
        light = document["extensions"]["KHR_lights_punctual"]["lights"][0]
        source_intensity = light["intensity"]
    except (KeyError, IndexError, TypeError) as error:
        raise ValueError("Pinned Bistro source does not contain the expected directional light") from error
    if source_intensity != 6830:
        raise ValueError(f"Pinned Bistro directional-light intensity must be 6830; got {source_intensity}")
    light["intensity"] = 10
    contents = json.dumps(document, indent=2, ensure_ascii=True).encode("utf-8")
    if not output.is_file() or output.read_bytes() != contents:
        output.write_bytes(contents)
    if sha256(output) != PINNED_BISTRO_REFERENCE_GLTF_SHA256:
        raise RuntimeError("Generated Bistro reference glTF hash does not match the M0 input")
    return {
        "source": str(source),
        "source_sha256": sha256(source),
        "output": str(output),
        "output_sha256": sha256(output),
        "source_directional_light_intensity": source_intensity,
        "reference_directional_light_intensity": 10,
        "serialization": "json.dumps(indent=2, ensure_ascii=True), no trailing newline",
    }


def bistro_asset_manifest(evo_gltf: Path, reference_gltf: Path) -> dict[str, object]:
    evo_document = json.loads(evo_gltf.read_text(encoding="utf-8"))
    reference_document = json.loads(reference_gltf.read_text(encoding="utf-8"))
    try:
        evo_light = evo_document["extensions"]["KHR_lights_punctual"]["lights"][0]
        reference_light = reference_document["extensions"]["KHR_lights_punctual"]["lights"][0]
        evo_intensity = evo_light["intensity"]
        reference_intensity = reference_light["intensity"]
    except (KeyError, IndexError, TypeError) as error:
        raise ValueError("Bistro assets must contain the expected directional light") from error
    evo_light["intensity"] = reference_intensity
    if evo_document != reference_document:
        raise ValueError("EvoEngine and reference Bistro JSON differ beyond directional-light intensity")
    if reference_intensity != 10:
        raise ValueError(f"Reference Bistro directional-light intensity must be 10; got {reference_intensity}")
    return {
        "semantic_delta": {
            "json_pointer": "/extensions/KHR_lights_punctual/lights/0/intensity",
            "evo_source": evo_intensity,
            "reference": reference_intensity,
            "evo_runtime_override": reference_intensity,
        },
        "evoengine": gltf_dependency_manifest(evo_gltf),
        "reference": gltf_dependency_manifest(reference_gltf),
    }


def command_text(command: list[str]) -> str:
    return subprocess.list2cmdline(command) if sys.platform == "win32" else shlex.join(command)


def run_command(
    command: list[str],
    cwd: Path,
    log_path: Path,
    dry_run: bool,
    environment: dict[str, str] | None = None,
) -> tuple[int, list[dict[str, object]]]:
    print(command_text(command), flush=True)
    if environment:
        print("Environment overrides: " + json.dumps(environment, sort_keys=True), flush=True)
    if dry_run:
        return 0, []
    log_path.parent.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, object]] = []
    with log_path.open("w", encoding="utf-8", newline="\n") as log:
        process_environment = os.environ.copy()
        process_environment.update(environment or {})
        process = subprocess.Popen(
            command,
            cwd=cwd,
            env=process_environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="", flush=True)
            log.write(line)
            for prefix in ("RAY_CAPTURE_JSON ", "BENCHMARK_JSON "):
                if line.startswith(prefix):
                    try:
                        records.append(json.loads(line[len(prefix) :]))
                    except json.JSONDecodeError:
                        pass
        return_code = process.wait()
    if return_code != 0:
        raise RuntimeError(f"Command failed with exit code {return_code}: {command_text(command)}")
    return return_code, records


def prepare_evo_runtime_environment(profile_dir: Path, technique: str, dry_run: bool) -> dict[str, object]:
    runtime_root = (profile_dir / f"evo-{technique}-runtime").resolve()
    shader_cache = runtime_root / "ShaderBinaries"
    pipeline_cache = runtime_root / "PipelineCache"
    imgui_ini = runtime_root / "imgui.ini"
    if not dry_run:
        if runtime_root.exists():
            shutil.rmtree(runtime_root)
        runtime_root.mkdir(parents=True, exist_ok=True)
    environment = {
        "EVOENGINE_SHADER_CACHE_DIR": str(shader_cache),
        "EVOENGINE_PIPELINE_CACHE_DIR": str(pipeline_cache),
        "EVOENGINE_IMGUI_INI_PATH": str(imgui_ini),
    }
    return {
        "root": str(runtime_root),
        "shader_cache": str(shader_cache),
        "pipeline_cache": str(pipeline_cache),
        "imgui_ini": str(imgui_ini),
        "environment": environment,
    }


def prepare_evo_bistro(output_root: Path, label: str, dry_run: bool) -> dict[str, object]:
    project_path = EVO_BISTRO_PROJECT_ROOT / "Bistro.eveproj"
    if not dry_run:
        project_path.unlink(missing_ok=True)
    command = [
        sys.executable,
        str(ROOT / "Scripts" / "prepare_demos.py"),
        "--demo",
        "bistro",
        "--no-download",
        "--no-previews",
        "--reset-scene",
        "--bistro-asset-mode",
        "none",
    ]
    log_path = output_root / f"evo-bistro-prepare-{label}.log"
    run_command(command, ROOT, log_path, dry_run)
    scene_files = sorted((EVO_BISTRO_PROJECT_ROOT / "Assets").glob("New Scene*.evescene"))
    if not dry_run and scene_files:
        raise RuntimeError(f"Bistro reset left generated scene state behind: {scene_files}")
    return {
        "label": label,
        "reset_project": str(project_path),
        "command": command,
        "log": str(log_path),
        "log_sha256": None if dry_run or not log_path.is_file() else sha256(log_path),
    }


def evo_project_state() -> dict[str, object]:
    project_path = EVO_BISTRO_PROJECT_ROOT / "Bistro.eveproj"
    scene_files = sorted((EVO_BISTRO_PROJECT_ROOT / "Assets").glob("New Scene*.evescene"))
    return {
        "project": None
        if not project_path.is_file()
        else {"path": str(project_path), "bytes": project_path.stat().st_size, "sha256": sha256(project_path)},
        "generated_scenes": [
            {"path": str(path), "bytes": path.stat().st_size, "sha256": sha256(path)} for path in scene_files
        ],
    }


def find_record(records: list[dict[str, object]], record_type: str) -> dict[str, object] | None:
    for record in reversed(records):
        if record.get("type") == record_type:
            return record
    return None


def remove_stale_output(path: Path) -> None:
    path.unlink(missing_ok=True)


def validate_output_image(path: Path, profile: Profile) -> None:
    if not path.is_file():
        raise RuntimeError(f"Renderer did not create output image: {path}")
    image = read_image(path)
    if (image.width, image.height) != (profile.width, profile.height):
        raise RuntimeError(
            f"Output dimensions must be {profile.width}x{profile.height}; got {image.width}x{image.height}: {path}"
        )
    if hasattr(image, "rgb"):
        maximum = 0.0
        for value in image.rgb:
            if not math.isfinite(value):
                raise RuntimeError(f"Output contains non-finite radiance: {path}")
            maximum = max(maximum, value)
        if maximum <= 0.0:
            raise RuntimeError(f"Output contains no positive radiance: {path}")


def validate_reference_summary(
    summary: dict[str, object] | None,
    profile: Profile,
    expected_render_technique: str,
    expected_gpu_timer: str,
) -> dict[str, object]:
    if summary is None:
        raise RuntimeError("Reference output did not contain a BENCHMARK_JSON headless summary")
    expected = {
        "resolution_w": profile.width,
        "resolution_h": profile.height,
        "effective_spp": profile.effective_spp,
        "frames": profile.frames,
        "maxFrames": profile.frames,
        "ptSamples": profile.samples_per_frame,
        "gpu_timer_name": expected_gpu_timer,
        "bounce_depth": 5,
        "firefly_clamp": 10.0,
        "texture_gradient_scale": 1.0,
        "render_technique": expected_render_technique,
    }
    for name, value in expected.items():
        if summary.get(name) != value:
            raise RuntimeError(f"Reference summary {name} must be {value!r}; got {summary.get(name)!r}")
    if not summary.get("gpu_sample_count"):
        raise RuntimeError("Reference summary did not contain GPU timestamp samples")
    if not summary.get("gpu_device_name") or not summary.get("gpu_device_id"):
        raise RuntimeError("Reference summary did not contain physical-device identity")
    return summary


def validate_evo_summary(
    summary: dict[str, object] | None,
    metrics_path: Path,
    profile: Profile,
    expected_render_mode: str,
    expected_gpu_timer: str,
) -> dict[str, object]:
    if summary is None:
        raise RuntimeError("EvoEngine output did not contain a RAY_CAPTURE_JSON summary")
    expected = {
        "width": profile.width,
        "height": profile.height,
        "requested_frames": profile.frames,
        "camera_frames": profile.frames,
        "effective_spp": profile.effective_spp,
        "samples_per_frame": profile.samples_per_frame,
        "render_mode": expected_render_mode,
        "bounce_depth": 5,
        "firefly_clamp_enabled": True,
        "firefly_clamp_threshold": 10.0,
        "auto_spp_enabled": False,
        "deterministic": True,
        "output_format": "radiance_hdr_linear",
    }
    for name, value in expected.items():
        if summary.get(name) != value:
            raise RuntimeError(f"EvoEngine summary {name} must be {value!r}; got {summary.get(name)!r}")
    if not summary.get("gpu_timestamps_available"):
        raise RuntimeError("EvoEngine GPU timestamps are unavailable on this device")
    path_sections = {
        section.get("name"): section
        for section in summary.get("gpu_sections", [])
        if str(section.get("name", "")).startswith("Path Trace (")
    }
    if expected_gpu_timer not in path_sections:
        raise RuntimeError(f"EvoEngine summary is missing GPU timer {expected_gpu_timer!r}")
    if path_sections[expected_gpu_timer].get("sample_count") != profile.frames:
        raise RuntimeError(
            f"EvoEngine GPU timer {expected_gpu_timer!r} must contain one sample per capture frame"
        )
    unexpected_path_timers = set(path_sections) - {expected_gpu_timer}
    if unexpected_path_timers:
        raise RuntimeError(f"EvoEngine capture rendered additional ray cameras: {sorted(unexpected_path_timers)}")
    if not metrics_path.is_file():
        raise RuntimeError(f"EvoEngine did not create metrics JSON: {metrics_path}")
    file_summary = json.loads(metrics_path.read_text(encoding="utf-8"))
    if file_summary != summary:
        raise RuntimeError("EvoEngine metrics file and RAY_CAPTURE_JSON output differ")
    return summary


def compare_outputs(reference: Path, candidate: Path, output: Path) -> dict[str, object]:
    summary = compare_render_images(read_image(reference), read_image(candidate), ignore_alpha=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def selected_profiles(value: str) -> list[str]:
    return list(PROFILES) if value == "all" else [value]


def selected_techniques(value: str) -> list[str]:
    return list(TECHNIQUES) if value == "all" else [value]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=(*PROFILES, "all"), default="fast")
    parser.add_argument("--technique", choices=(*TECHNIQUES, "all"), default="all")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "out" / "raytracer-baseline")
    parser.add_argument(
        "--editor", type=Path, default=ROOT / "out" / "install" / "vs2026-x64" / "bin" / "EvoEngineEditor.exe"
    )
    parser.add_argument(
        "--reference-root", type=Path, default=ROOT / "out" / "reference" / "vk_gltf_renderer"
    )
    parser.add_argument("--nvpro-root", type=Path, default=ROOT.parent / "nvpro_core2")
    parser.add_argument("--reference-exe", type=Path)
    parser.add_argument("--reference-build-dir", type=Path)
    parser.add_argument("--evo-build-dir", type=Path, default=ROOT / "out" / "build" / "vs2026-x64")
    parser.add_argument(
        "--asset",
        type=Path,
        default=BISTRO_SOURCE_ROOT / "bistro-directional-intensity-10.gltf",
    )
    parser.add_argument("--bistro-source-root", type=Path, default=BISTRO_SOURCE_ROOT)
    parser.add_argument(
        "--evo-asset",
        type=Path,
        default=ROOT
        / "Resources"
        / ".generated"
        / "EvoEngine-DemoProjects"
        / "Bistro"
        / "Assets"
        / "Models"
        / "Bistro"
        / "bistro.gltf",
    )
    parser.add_argument("--skip-evo", action="store_true")
    parser.add_argument("--skip-reference", action="store_true")
    parser.add_argument(
        "--skip-evo-prepare",
        action="store_true",
        help="Use the current generated Bistro project instead of resetting it before each EvoEngine run.",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_root = args.output_dir.resolve()
    manifest_path = output_root / ("manifest.dry-run.json" if args.dry_run else "manifest.json")
    manifest_path.unlink(missing_ok=True)
    editor = args.editor.resolve()
    evo_build_dir = args.evo_build_dir.resolve()
    reference_root = args.reference_root.resolve()
    nvpro_root = args.nvpro_root.resolve()
    reference_build_dir = (
        args.reference_build_dir.resolve() if args.reference_build_dir else reference_root / "build-m0"
    )
    reference_exe = (
        args.reference_exe.resolve()
        if args.reference_exe
        else reference_root / "_bin" / "RelWithDebInfo" / "vk_gltf_renderer.exe"
    )
    asset = args.asset.resolve()
    evo_asset = args.evo_asset.resolve()
    bistro_source_root = args.bistro_source_root.resolve()
    reference_environment = reference_root / "resources" / "std_env.hdr"
    if args.skip_evo and args.skip_reference:
        raise ValueError("At least one renderer must be selected")

    bistro_state = repo_state(bistro_source_root)
    if bistro_state["head"] != PINNED_BISTRO_SOURCE:
        raise RuntimeError(f"Bistro source HEAD must be {PINNED_BISTRO_SOURCE}; got {bistro_state['head']}")
    try:
        generated_asset_relative = asset.relative_to(bistro_source_root).as_posix()
    except ValueError as error:
        raise ValueError("The reference asset must be generated inside the pinned Bistro source checkout") from error
    unexpected_bistro_status = [
        line for line in bistro_state["status"] if line != f"?? {generated_asset_relative}"
    ]
    if unexpected_bistro_status:
        raise RuntimeError(f"Pinned Bistro source contains unexpected changes: {unexpected_bistro_status}")
    reference_asset_generation = generate_bistro_reference_asset(bistro_source_root, asset)
    bistro_state = repo_state(bistro_source_root)

    preparation_records: list[dict[str, object]] = []
    if not args.skip_evo and not args.skip_evo_prepare:
        preparation_records.append(prepare_evo_bistro(output_root, "initial", args.dry_run))
    required_files = []
    if not args.skip_evo:
        required_files.append(evo_asset)
    if not args.skip_reference:
        required_files.extend((asset, REFERENCE_PATCH, reference_environment))
    for required in required_files:
        if not required.is_file():
            raise FileNotFoundError(required)
    if not args.skip_evo and not args.dry_run and not editor.is_file():
        raise FileNotFoundError(editor)
    if not args.skip_reference and not args.dry_run and not reference_exe.is_file():
        raise FileNotFoundError(reference_exe)

    evo_build_verification = None
    if not args.skip_evo and not args.dry_run:
        evo_build_verification = verify_evo_install_matches_build(editor, evo_build_dir)

    evo_state = repo_state(ROOT) if not args.skip_evo else None
    reference_state = repo_state(reference_root) if not args.skip_reference else None
    nvpro_state = repo_state(nvpro_root) if not args.skip_reference else None
    if not args.skip_evo:
        git(ROOT, "merge-base", "--is-ancestor", PINNED_EVOENGINE_BASE, "HEAD")
    if not args.skip_reference:
        assert reference_state is not None
        assert nvpro_state is not None
        if reference_state["head"] != PINNED_REFERENCE:
            raise RuntimeError(f"Reference HEAD must be {PINNED_REFERENCE}; got {reference_state['head']}")
        if nvpro_state["head"] != PINNED_NVPRO_CORE2:
            raise RuntimeError(f"nvpro_core2 HEAD must be {PINNED_NVPRO_CORE2}; got {nvpro_state['head']}")
        if reference_state["working_diff_sha256"] != sha256(REFERENCE_PATCH):
            raise RuntimeError("Reference tracked changes must exactly match the recorded M0 patch")

    reference_options = cmake_cache_values(
        reference_build_dir / "CMakeCache.txt", ("USE_DLSS", "USE_OPTIX_DENOISER", "NvproCore2_ROOT")
    )
    if not args.skip_reference:
        for name in ("USE_DLSS", "USE_OPTIX_DENOISER"):
            if reference_options.get(name) != "OFF":
                raise RuntimeError("Reference build must disable DLSS and the OptiX denoiser")
        configured_nvpro = Path(reference_options.get("NvproCore2_ROOT", "")) / "nvpro_core2"
        if configured_nvpro.resolve() != nvpro_root:
            raise RuntimeError(f"Reference build uses unexpected nvpro_core2 root: {configured_nvpro}")
        unexpected_untracked = [
            line
            for line in reference_state["status"]
            if line.startswith("?? ") and not line[3:].replace("\\", "/").startswith(("build-m0/", "_bin/"))
        ]
        if unexpected_untracked:
            raise RuntimeError(f"Reference worktree contains unexpected untracked files: {unexpected_untracked}")
        if nvpro_state["dirty"]:
            raise RuntimeError("Pinned nvpro_core2 worktree must be clean")

    common_manifest = {
        "schema": 1,
        "type": "raytracer_baseline_manifest",
        "dry_run": args.dry_run,
        "pins": {
            "evoengine_base": PINNED_EVOENGINE_BASE,
            "vk_gltf_renderer": PINNED_REFERENCE,
            "nvpro_core2": PINNED_NVPRO_CORE2,
            "niagara_bistro": PINNED_BISTRO_SOURCE,
            "reference_patch_sha256": sha256(REFERENCE_PATCH),
        },
        "tools": {
            "runner_sha256": sha256(Path(__file__).resolve()),
            "comparison_sha256": sha256(ROOT / "Scripts" / "compare_reference_render.py"),
        },
        "repositories": {
            "evoengine": evo_state,
            "vk_gltf_renderer": reference_state,
            "nvpro_core2": nvpro_state,
            "niagara_bistro": bistro_state,
        },
        "builds": {
            "evoengine": None
            if args.skip_evo
            else {
                **cmake_build_metadata(evo_build_dir, "RelWithDebInfo"),
                "installed_artifact_verification": evo_build_verification,
            },
            "vk_gltf_renderer": None
            if args.skip_reference
            else cmake_build_metadata(reference_build_dir, "RelWithDebInfo"),
        },
        "reference_build": {
            "directory": str(reference_build_dir),
            "options": reference_options,
        },
        "evo_project_prepared": not args.skip_evo and not args.skip_evo_prepare,
        "evo_project_state": None if args.skip_evo else evo_project_state(),
        "preparations": preparation_records,
        "reference_asset_generation": reference_asset_generation,
        "reference_environment": None
        if args.skip_reference
        else {
            "path": str(reference_environment),
            "bytes": reference_environment.stat().st_size,
            "sha256": sha256(reference_environment),
            "intensity": 0,
        },
        "binaries": {
            "evoengine": None if not editor.is_file() else runtime_binary_manifest(editor),
            "reference": None
            if not reference_exe.is_file()
            else runtime_binary_manifest(reference_exe),
        },
        "encoding": "Radiance RGBE linear; absolute and symmetric-relative HDR metrics",
        "runs": [],
        "comparisons": [],
    }

    assets = (
        bistro_asset_manifest(evo_asset, asset)
        if not args.skip_evo and not args.skip_reference
        else {
            "evoengine": gltf_dependency_manifest(evo_asset) if not args.skip_evo else None,
            "reference": gltf_dependency_manifest(asset) if not args.skip_reference else None,
        }
    )
    if assets.get("evoengine") and assets["evoengine"]["combined_sha256"] != EXPECTED_EVO_BISTRO_CLOSURE:
        raise RuntimeError("EvoEngine Bistro dependency closure does not match the pinned M0 input")
    if assets.get("reference") and assets["reference"]["combined_sha256"] != EXPECTED_REFERENCE_BISTRO_CLOSURE:
        raise RuntimeError("Reference Bistro dependency closure does not match the pinned M0 input")
    common_manifest["assets"] = assets

    evo_capture_count = 0
    for profile_name in selected_profiles(args.profile):
        profile = PROFILES[profile_name]
        profile_dir = output_root / profile_name
        outputs: dict[tuple[str, str], Path] = {}
        summaries: dict[tuple[str, str], dict[str, object]] = {}
        for technique_name in selected_techniques(args.technique):
            evo_mode, evo_telemetry_mode, reference_technique, expected_gpu_timer = TECHNIQUES[technique_name]
            if not args.skip_reference:
                reference_output = profile_dir / f"vk-{technique_name}-{profile.effective_spp}spp.hdr"
                reference_log = profile_dir / f"vk-{technique_name}.log"
                if not args.dry_run:
                    remove_stale_output(reference_output)
                reference_command = [
                    str(reference_exe),
                    "--headless",
                    "--size",
                    str(profile.width),
                    str(profile.height),
                    "--scenefile",
                    str(asset),
                    "--frames",
                    str(profile.frames),
                    "--maxFrames",
                    str(profile.frames),
                    "--ptSamples",
                    str(profile.samples_per_frame),
                    "--ptAdaptiveSampling",
                    "0",
                    "--ptTechnique",
                    reference_technique,
                    "--ptMaxDepth",
                    "5",
                    "--ptFireflyClamp",
                    "10",
                    "--ptTexGradScale",
                    "1",
                    "--ptAperture",
                    "0",
                    "--renderSystem",
                    "0",
                    "--envSystem",
                    "1",
                    "--hdrfile",
                    str(reference_environment),
                    "--hdrEnvIntensity",
                    "0",
                    "--useSolidBackground",
                    "--solidBackgroundColor",
                    "0",
                    "0",
                    "0",
                    "--gltfCamera",
                    "0",
                    "--wireframe",
                    "0",
                    "--optimalShader",
                    "0",
                    "--output",
                    str(reference_output),
                ]
                _, records = run_command(reference_command, reference_root, reference_log, args.dry_run)
                summary = find_record(records, "headless_summary")
                if not args.dry_run:
                    validate_output_image(reference_output, profile)
                    summary = validate_reference_summary(summary, profile, evo_mode, expected_gpu_timer)
                common_manifest["runs"].append(
                    {
                        "profile": profile_name,
                        "renderer": "vk_gltf_renderer",
                        "technique": technique_name,
                        "command": reference_command,
                        "log": str(reference_log),
                        "log_sha256": None
                        if args.dry_run or not reference_log.is_file()
                        else sha256(reference_log),
                        "output": str(reference_output),
                        "output_sha256": None
                        if args.dry_run or not reference_output.is_file()
                        else sha256(reference_output),
                        "metrics": summary,
                    }
                )
                outputs[("reference", technique_name)] = reference_output
                if summary is not None:
                    summaries[("reference", technique_name)] = summary

            if not args.skip_evo:
                if not args.skip_evo_prepare and evo_capture_count != 0:
                    preparation_records.append(
                        prepare_evo_bistro(output_root, f"{profile_name}-{technique_name}", args.dry_run)
                    )
                evo_output = profile_dir / f"evo-{technique_name}-{profile.effective_spp}spp.hdr"
                evo_metrics = profile_dir / f"evo-{technique_name}.json"
                evo_log = profile_dir / f"evo-{technique_name}.log"
                runtime_state = prepare_evo_runtime_environment(profile_dir, technique_name, args.dry_run)
                if not args.dry_run:
                    remove_stale_output(evo_output)
                    remove_stale_output(evo_metrics)
                evo_command = [
                    str(editor),
                    "--demo",
                    "bistro",
                    "--editor",
                    "--capture-demo-preview",
                    str(evo_output),
                    "--preview-metrics-json",
                    str(evo_metrics),
                    "--preview-render-mode",
                    evo_mode,
                    "--preview-warmup-frames",
                    str(profile.frames),
                    "--preview-sample-size",
                    str(profile.samples_per_frame),
                    "--preview-auto-spp",
                    "disabled",
                    "--preview-firefly-clamp",
                    "enabled",
                    "--preview-firefly-clamp-threshold",
                    "10",
                    "--preview-ser",
                    "automatic",
                    "--preview-width",
                    str(profile.width),
                    "--preview-height",
                    str(profile.height),
                    "--preview-deterministic",
                ]
                _, records = run_command(
                    evo_command,
                    ROOT,
                    evo_log,
                    args.dry_run,
                    runtime_state["environment"],
                )
                summary = find_record(records, "evoengine_ray_capture")
                if not args.dry_run:
                    validate_output_image(evo_output, profile)
                    summary = validate_evo_summary(summary, evo_metrics, profile, evo_telemetry_mode, expected_gpu_timer)
                    if not Path(runtime_state["shader_cache"]).is_dir():
                        raise RuntimeError("EvoEngine did not use the isolated shader cache")
                    if not Path(runtime_state["pipeline_cache"]).is_dir():
                        raise RuntimeError("EvoEngine did not use the isolated pipeline cache")
                common_manifest["runs"].append(
                    {
                        "profile": profile_name,
                        "renderer": "EvoEngine",
                        "technique": technique_name,
                        "command": evo_command,
                        "runtime_state": runtime_state,
                        "log": str(evo_log),
                        "log_sha256": None if args.dry_run or not evo_log.is_file() else sha256(evo_log),
                        "metrics_path": str(evo_metrics),
                        "metrics_sha256": None
                        if args.dry_run or not evo_metrics.is_file()
                        else sha256(evo_metrics),
                        "output": str(evo_output),
                        "output_sha256": None if args.dry_run or not evo_output.is_file() else sha256(evo_output),
                        "metrics": summary,
                    }
                )
                outputs[("evo", technique_name)] = evo_output
                if summary is not None:
                    summaries[("evo", technique_name)] = summary
                evo_capture_count += 1

            if not args.dry_run:
                reference_summary = summaries.get(("reference", technique_name))
                evo_summary = summaries.get(("evo", technique_name))
                if reference_summary and evo_summary:
                    evo_gpu = evo_summary.get("gpu", {})
                    reference_device = (
                        reference_summary.get("gpu_vendor_id"),
                        reference_summary.get("gpu_device_id"),
                    )
                    evo_device = (evo_gpu.get("vendor_id"), evo_gpu.get("device_id"))
                    if reference_device != evo_device:
                        raise RuntimeError(
                            f"Reference and EvoEngine selected different physical devices: "
                            f"{reference_device} != {evo_device}"
                        )
                    if reference_summary.get("ser_enabled") != evo_summary.get("ser_enabled"):
                        raise RuntimeError("Reference and EvoEngine did not use the same effective SER state")

        if not args.dry_run:
            for technique_name in selected_techniques(args.technique):
                reference_output = outputs.get(("reference", technique_name))
                evo_output = outputs.get(("evo", technique_name))
                if reference_output and evo_output:
                    comparison_path = profile_dir / f"reference-vs-evo-{technique_name}.json"
                    common_manifest["comparisons"].append(
                        {
                            "profile": profile_name,
                            "name": f"reference-vs-evo-{technique_name}",
                            "metrics": compare_outputs(reference_output, evo_output, comparison_path),
                            "path": str(comparison_path),
                        }
                    )
            evo_rtx = outputs.get(("evo", "rtx"))
            evo_rq = outputs.get(("evo", "rq"))
            if evo_rtx and evo_rq:
                comparison_path = profile_dir / "evo-rtx-vs-rq.json"
                common_manifest["comparisons"].append(
                    {
                        "profile": profile_name,
                        "name": "evo-rtx-vs-rq",
                        "metrics": compare_outputs(evo_rtx, evo_rq, comparison_path),
                        "path": str(comparison_path),
                    }
                )
            reference_rtx = outputs.get(("reference", "rtx"))
            reference_rq = outputs.get(("reference", "rq"))
            if reference_rtx and reference_rq:
                comparison_path = profile_dir / "vk-rtx-vs-rq.json"
                common_manifest["comparisons"].append(
                    {
                        "profile": profile_name,
                        "name": "vk-rtx-vs-rq",
                        "metrics": compare_outputs(reference_rtx, reference_rq, comparison_path),
                        "path": str(comparison_path),
                    }
                )

    output_root.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(common_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Manifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
