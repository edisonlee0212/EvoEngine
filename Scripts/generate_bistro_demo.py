#!/usr/bin/env python3
"""Generate the local ignored EvoEngine Bistro glTF demo resources."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path


SOURCE_REPO_URL = "https://github.com/zeux/niagara_bistro.git"
PREFAB_ASSET_HANDLE = 12011778184302674731
MODELS_FOLDER_HANDLE = 12634270581618749311
BISTRO_FOLDER_HANDLE = 12634270581618749312
IMAGE_FALLBACK_EXTENSIONS = (".dds", ".png", ".tga", ".jpg", ".jpeg")


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def write_text_if_changed(path: Path, text: str) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.read_text(encoding="utf-8") == text:
        return False
    path.write_text(text, encoding="utf-8", newline="\n")
    return True


def run_git(arguments: list[str], cwd: Path | None = None) -> None:
    command = ["git", *arguments]
    print("Running " + subprocess.list2cmdline(command) if os.name == "nt" else "Running " + " ".join(command))
    subprocess.run(command, cwd=cwd, check=True)


def update_source_cache(source_root: Path, repo_url: str, force_download: bool, no_download: bool) -> Path:
    if source_root.is_dir() and (source_root / ".git").is_dir():
        if force_download:
            run_git(["fetch", "--depth", "1", "origin"], source_root)
            run_git(["checkout", "--detach", "FETCH_HEAD"], source_root)
        else:
            print(f"Using existing Bistro source cache: {source_root}")
        return source_root

    if source_root.exists() and not (source_root / ".git").is_dir():
        print(f"Using supplied Bistro source folder: {source_root}")
        return source_root

    if no_download:
        raise RuntimeError(f"Missing Bistro source cache and --no-download was supplied: {source_root}")

    source_root.parent.mkdir(parents=True, exist_ok=True)
    run_git(["clone", "--depth", "1", repo_url, str(source_root)])
    return source_root


def verify_source(source_root: Path, gltf_name: str) -> dict[str, int | str]:
    gltf_path = source_root / gltf_name
    required_paths = [
        gltf_path,
        source_root / "bistro.bin",
        source_root / "LICENSE",
        source_root / "objects",
        source_root / "textures",
    ]
    missing = [str(path.relative_to(source_root)) for path in required_paths if not path.exists()]
    if missing:
        raise RuntimeError("Bistro source is missing " + ", ".join(missing))

    with gltf_path.open("r", encoding="utf-8") as stream:
        gltf = json.load(stream)
    extensions = set(gltf.get("extensionsUsed", []))
    if "MSFT_texture_dds" not in extensions:
        raise RuntimeError(f"{gltf_name} does not declare MSFT_texture_dds.")
    if "KHR_lights_punctual" not in extensions:
        raise RuntimeError(f"{gltf_name} does not declare KHR_lights_punctual.")

    images = gltf.get("images", [])
    textures = gltf.get("textures", [])
    materials = gltf.get("materials", [])
    dds_texture_count = 0
    missing_images: list[str] = []
    alternate_image_count = 0
    for texture in textures:
        extension = texture.get("extensions", {}).get("MSFT_texture_dds")
        if isinstance(extension, dict) and "source" in extension:
            dds_texture_count += 1
    for image in images:
        uri = image.get("uri")
        if isinstance(uri, str) and not (source_root / uri).exists():
            image_path = source_root / uri
            if any(image_path.with_suffix(extension).exists() for extension in IMAGE_FALLBACK_EXTENSIONS):
                alternate_image_count += 1
            else:
                missing_images.append(uri)
    if dds_texture_count == 0:
        raise RuntimeError(f"{gltf_name} does not contain MSFT_texture_dds texture sources.")
    if missing_images:
        raise RuntimeError("Bistro source is missing image URIs: " + ", ".join(missing_images[:8]))

    return {
        "gltf": gltf_name,
        "images": len(images),
        "textures": len(textures),
        "materials": len(materials),
        "dds_textures": dds_texture_count,
        "alternate_images": alternate_image_count,
    }


def reset_scene_files(demo_root: Path) -> None:
    assets_root = demo_root / "Assets"
    for scene_path in assets_root.glob("New Scene*.evescene"):
        scene_path.unlink(missing_ok=True)
        Path(str(scene_path) + ".evefilemeta").unlink(missing_ok=True)


def copy_path(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if source.is_dir():
        shutil.copytree(source, destination, dirs_exist_ok=True)
    else:
        shutil.copy2(source, destination)


def link_or_copy(source: Path, destination: Path, copy_fallback: bool) -> str:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_symlink() and Path(os.readlink(destination)) == source:
        return "symlink"
    if destination.exists() and not destination.is_symlink():
        if source.is_dir():
            copy_path(source, destination)
        else:
            if destination.stat().st_size == source.stat().st_size:
                return "existing"
            copy_path(source, destination)
        return "copy"
    if destination.is_symlink() or destination.exists():
        if destination.is_dir() and not destination.is_symlink():
            shutil.rmtree(destination)
        else:
            destination.unlink()

    try:
        destination.symlink_to(source, target_is_directory=source.is_dir())
        return "symlink"
    except OSError:
        if not copy_fallback:
            raise
        copy_path(source, destination)
        return "copy"


def prepare_one_asset(source: Path, destination: Path, asset_mode: str, copy_fallback: bool) -> str:
    if asset_mode == "copy":
        copy_path(source, destination)
        return "copy"
    return link_or_copy(source, destination, copy_fallback)


def prepare_asset_path(source_root: Path, asset_root: Path, asset_mode: str, copy_fallback: bool) -> dict[str, str]:
    modes: dict[str, str] = {}
    if asset_mode == "none":
        return modes
    for name in ("bistro.gltf", "bistro.bin", "LICENSE", "README.md"):
        source = source_root / name
        if source.exists():
            destination = asset_root / name
            modes[name] = prepare_one_asset(source, destination, asset_mode, copy_fallback)
    for name in ("objects", "textures"):
        source = source_root / name
        destination = asset_root / name
        modes[name] = prepare_one_asset(source, destination, asset_mode, copy_fallback)
    return modes


def write_demo_files(
    resource_root: Path,
    source_root: Path,
    stats: dict[str, int | str],
    asset_mode: str,
    copy_fallback: bool,
    reset_project: bool,
) -> tuple[Path, Path, dict[str, str]]:
    demo_root = resource_root / ".generated" / "EvoEngine-DemoProjects" / "Bistro"
    asset_root = demo_root / "Assets" / "Models" / "Bistro"
    project_path = demo_root / "Bistro.eveproj"

    project_text = """application_name: Bistro
preferred_editor: EvoEngineEditor
startup_runtime_packages:
  []
EditorLayer:
  velocity: 15
  enable_gizmos: false
  default_scene_camera_rotation: [-0.0436194, 0, 0, 0.999048]
  default_scene_camera_position: [3, 25, 150]
  scene_camera_position: [3, 25, 150]
  scene_camera_rotation: [-0.0436194, 0, 0, 0.999048]
  scene_camera_settings:
    near_distance: 0.1
    far_distance: 500
    fov: 60
"""
    if reset_project or not project_path.exists():
        write_text_if_changed(project_path, project_text)

    models_folder_meta = f"""handle_: {MODELS_FOLDER_HANDLE}
type_name: Models
"""
    write_text_if_changed(demo_root / "Assets" / "Models.evefoldermeta", models_folder_meta)

    bistro_folder_meta = f"""handle_: {BISTRO_FOLDER_HANDLE}
type_name: Bistro
"""
    write_text_if_changed(demo_root / "Assets" / "Models" / "Bistro.evefoldermeta", bistro_folder_meta)

    asset_meta = f"""asset_extension_: .gltf
asset_file_name_: bistro
asset_type_name_: Prefab
asset_handle_: {PREFAB_ASSET_HANDLE}
"""
    write_text_if_changed(asset_root / "bistro.gltf.evefilemeta", asset_meta)

    readme_text = f"""# Bistro Demo

This generated demo uses the Amazon Lumberyard Bistro glTF scene from zeux/niagara_bistro.

- Source: {SOURCE_REPO_URL}
- Source cache: {source_root}
- Local glTF: Assets/Models/Bistro/bistro.gltf
- Images: {stats["images"]}
- Textures: {stats["textures"]}
- Materials: {stats["materials"]}
- MSFT_texture_dds textures: {stats["dds_textures"]}
- Image URI alternate files: {stats["alternate_images"]}

Run `python Scripts/generate_bistro_demo.py` from the EvoEngine repository root to recreate the local ignored project.
Run the editor with `--demo bistro` after generation.
"""
    write_text_if_changed(demo_root / "README.md", readme_text)

    modes = prepare_asset_path(source_root, asset_root, asset_mode, copy_fallback)
    return demo_root, project_path, modes


def capture_preview(editor: Path, preview_path: Path, warmup_frames: int) -> None:
    command = [
        str(editor),
        "--demo",
        "bistro",
        "--capture-demo-preview",
        str(preview_path),
        "--preview-render-mode",
        "raytracing",
        "--preview-warmup-frames",
        str(warmup_frames),
        "--preview-sample-size",
        "4",
        "--preview-auto-spp",
        "disabled",
        "--preview-firefly-clamp",
        "enabled",
        "--preview-firefly-clamp-threshold",
        "10",
        "--preview-deterministic",
    ]
    print("Running " + subprocess.list2cmdline(command) if os.name == "nt" else "Running " + " ".join(command))
    subprocess.run(command, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--resource-root", type=Path, default=repo_root() / "Resources")
    parser.add_argument("--source-root", type=Path, help="Existing zeux/niagara_bistro checkout to use.")
    parser.add_argument("--repo-url", default=SOURCE_REPO_URL)
    parser.add_argument("--gltf", default="bistro.gltf")
    parser.add_argument("--force-download", action="store_true")
    parser.add_argument("--no-download", action="store_true")
    parser.add_argument("--reset-scene", action="store_true")
    parser.add_argument("--asset-mode", choices=("symlink", "copy", "none"), default="symlink")
    parser.add_argument("--no-copy-fallback", action="store_true")
    parser.add_argument("--editor", type=Path, help="Optional EvoEngineEditor executable used to create the scene.")
    parser.add_argument(
        "--preview",
        type=Path,
        default=repo_root() / "Resources" / "Launcher" / "DemoPreviews" / "bistro.png",
    )
    parser.add_argument("--preview-warmup-frames", type=int, default=256)
    args = parser.parse_args()

    resource_root = args.resource_root.resolve()
    source_root = args.source_root.resolve() if args.source_root else resource_root / ".generated" / "niagara_bistro"
    source_root = update_source_cache(source_root, args.repo_url, args.force_download, args.no_download)
    stats = verify_source(source_root, args.gltf)
    demo_root, project_path, modes = write_demo_files(
        resource_root,
        source_root,
        stats,
        args.asset_mode,
        not args.no_copy_fallback,
        args.reset_scene,
    )
    if args.reset_scene:
        reset_scene_files(demo_root)
    if args.editor:
        capture_preview(args.editor.resolve(), args.preview.resolve(), max(0, args.preview_warmup_frames))

    print(
        "Verified Bistro source: "
        f"{stats['images']} images, {stats['textures']} textures, {stats['materials']} materials, "
        f"{stats['dds_textures']} MSFT_texture_dds textures, {stats['alternate_images']} image URI alternates"
    )
    if modes:
        print("Prepared asset links/files: " + ", ".join(f"{name}={mode}" for name, mode in sorted(modes.items())))
    print(f"Bistro demo resources are ready under {demo_root}")
    print(f"Project: {project_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"generate_bistro_demo.py: {error}", file=sys.stderr)
        raise SystemExit(1)
