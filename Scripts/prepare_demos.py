#!/usr/bin/env python3
"""Validate and prepare local EvoEngine demo resources."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
import urllib.request
import zipfile
from pathlib import Path


DEMO_ORDER = ("rendering", "3dgs", "bicycle", "bistro", "rendering-regression")

BISTRO_SOURCE_REPO_URL = "https://github.com/zeux/niagara_bistro.git"
BISTRO_PREFAB_ASSET_HANDLE = 12011778184302674731
BISTRO_MODELS_FOLDER_HANDLE = 12634270581618749311
BISTRO_FOLDER_HANDLE = 12634270581618749312
BISTRO_IMAGE_FALLBACK_EXTENSIONS = (".dds", ".png", ".tga", ".jpg", ".jpeg")

SPATIAL_DRAGON_ASSET_URL = (
    "https://raw.githubusercontent.com/sekiguchiaimi/spatialdragon-3dgs/main/data/spatial_dragon.ply"
)
SPATIAL_DRAGON_ASSET_SIZE = 1_571_111
SPATIAL_DRAGON_ASSET_SHA256 = "40D7FDEBEB6A9A5755074F4F02A759EEE19BF15F46520A8D79B5F42BDE42921D"
SPATIAL_DRAGON_ASSET_HANDLE = 9739484957885691067
SPATIAL_DRAGON_FOLDER_HANDLE = 17421499951906451750

BICYCLE_ASSET_URL = "https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/datasets/pretrained/models.zip"
BICYCLE_PLY_MEMBERS = (
    "bicycle/bicycle/point_cloud/iteration_30000/point_cloud.ply",
    "bicycle/point_cloud/iteration_30000/point_cloud.ply",
)
BICYCLE_ASSET_HANDLE = 14453709846752502031
BICYCLE_FOLDER_HANDLE = 13041787869273319741

README_IMAGE = Path("Resources/GitHub/RenderingDemo.png")
DEFAULT_BUILD_DIR = Path("out/build/vs2026-x64")
SPONZA_LIGHTING_FILES = (
    "SponzaEnvironment.eveenvironmentalmap",
    "SponzaGlobal.evereflectionprobe",
    "SponzaLeftGallery.evereflectionprobe",
    "SponzaRightGallery.evereflectionprobe",
    "SponzaCentralFront.evereflectionprobe",
    "SponzaCentralMiddle.evereflectionprobe",
    "SponzaCentralRear.evereflectionprobe",
)
SPONZA_PROBE_BASE64_PAYLOAD_SIZE = 5_592_384


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def executable_name(name: str) -> str:
    return f"{name}.exe" if os.name == "nt" else name


def default_editor_path(root: Path) -> Path:
    return root / "out" / "install" / "vs2026-x64" / "bin" / executable_name("EvoEngineEditor")


def format_command(command: list[str]) -> str:
    return subprocess.list2cmdline(command) if os.name == "nt" else shlex.join(command)


def run(command: list[str], cwd: Path) -> None:
    print(format_command(command), flush=True)
    subprocess.run(command, cwd=cwd, check=True)


def write_text_if_changed(path: Path, text: str) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.read_text(encoding="utf-8") == text:
        return False
    path.write_text(text, encoding="utf-8", newline="\n")
    return True


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def validate_spatial_dragon(path: Path) -> bool:
    return path.is_file() and path.stat().st_size == SPATIAL_DRAGON_ASSET_SIZE and sha256(path) == SPATIAL_DRAGON_ASSET_SHA256


def validate_ply(path: Path) -> bool:
    if not path.is_file() or path.stat().st_size == 0:
        return False
    header = path.read_bytes()[:4096].decode("ascii", errors="ignore")
    return header.startswith("ply") and "element vertex" in header and "end_header" in header


def validate_nonempty_file(path: Path, label: str, missing: list[str]) -> None:
    if not path.is_file() or path.stat().st_size == 0:
        missing.append(label)


def validate_file(path: Path, label: str, missing: list[str]) -> None:
    if not path.is_file():
        missing.append(label)


def validate_directory(path: Path, label: str, missing: list[str]) -> None:
    if not path.is_dir():
        missing.append(label)


def validate_sponza_lighting_assets(root: Path, missing: list[str]) -> None:
    for index, name in enumerate(SPONZA_LIGHTING_FILES):
        path = root / name
        if not path.is_file() or path.stat().st_size <= (0 if index == 0 else SPONZA_PROBE_BASE64_PAYLOAD_SIZE):
            missing.append(name)


def preview_path(resource_root: Path, demo_id: str) -> Path:
    return resource_root / "Launcher" / "DemoPreviews" / f"{demo_id}.png"


def maybe_capture_launcher_preview(args: argparse.Namespace, demo_id: str, output_path: Path, warmup_frames: int) -> None:
    if not args.previews:
        return
    if output_path.is_file() and output_path.stat().st_size > 0 and not args.override:
        print(f"{demo_id}: preview is already present: {output_path}", flush=True)
        return
    editor = args.editor.resolve()
    if not editor.exists():
        raise RuntimeError(f"{demo_id}: missing editor needed to capture preview: {editor}")
    command = [
        str(editor),
        "--demo",
        demo_id,
        "--editor",
        "--capture-demo-preview",
        str(output_path.resolve()),
        "--preview-width",
        str(args.preview_width),
        "--preview-height",
        str(args.preview_height),
        "--preview-warmup-frames",
        str(warmup_frames),
    ]
    if demo_id == "bistro":
        command.extend(
            [
                "--preview-render-mode",
                "raytracing",
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
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.unlink(missing_ok=True)
    run(command, editor.parent)
    if not output_path.is_file() or output_path.stat().st_size == 0:
        raise RuntimeError(f"{demo_id}: preview was not written: {output_path}")


def reset_scene_files(demo_root: Path) -> None:
    assets_root = demo_root / "Assets"
    for scene_path in assets_root.glob("New Scene*.evescene"):
        scene_path.unlink(missing_ok=True)
        Path(str(scene_path) + ".evefilemeta").unlink(missing_ok=True)


def validate_rendering(args: argparse.Namespace) -> list[str]:
    resource_root = args.resource_root
    missing: list[str] = []
    validate_file(resource_root / "EvoEngine-DemoProjects" / "Rendering" / "Rendering.eveproj", "Rendering.eveproj", missing)
    validate_sponza_lighting_assets(
        resource_root / "EvoEngine-DemoProjects" / "Rendering" / "Assets" / "Lighting" / "Sponza", missing
    )
    if args.rendering_readme:
        validate_nonempty_file(repo_root() / README_IMAGE, "Resources/GitHub/RenderingDemo.png", missing)
    if args.previews:
        validate_nonempty_file(preview_path(resource_root, "rendering"), "Launcher/DemoPreviews/rendering.png", missing)
    return missing


def capture_rendering_readme_image(args: argparse.Namespace) -> None:
    root = repo_root()
    output = root / README_IMAGE
    if output.is_file() and output.stat().st_size > 0 and not args.override:
        print(f"rendering: README image is already present: {output}", flush=True)
        return
    if args.resource_root.resolve() != (root / "Resources").resolve():
        raise RuntimeError("rendering: README image capture only supports the repository Resources folder.")
    command = [
        sys.executable,
        str(root / "Scripts" / "capture_readme_editor_screenshot.py"),
        "--apply",
        "--build-dir",
        str(args.build_dir),
        "--config",
        args.config,
        "--demo-setup",
        "Rendering",
        "--width",
        str(args.readme_width),
        "--height",
        str(args.readme_height),
        "--warmup-frames",
        str(args.readme_warmup_frames),
        "--timeout",
        str(args.readme_timeout),
    ]
    if not args.build_rendering_readme:
        command.append("--no-build")
    run(command, root)


def prepare_rendering(args: argparse.Namespace) -> None:
    if args.rendering_readme:
        capture_rendering_readme_image(args)
    if args.previews:
        maybe_capture_launcher_preview(args, "rendering", preview_path(args.resource_root, "rendering"), args.preview_warmup_frames)
    print("rendering: resources are ready.", flush=True)


def rendering_regression_project_path(resource_root: Path) -> Path:
    return resource_root / ".generated" / "EvoEngine-DemoProjects" / "RenderingRegression" / "RenderingRegression.eveproj"


def validate_rendering_regression(args: argparse.Namespace) -> list[str]:
    missing: list[str] = []
    validate_file(rendering_regression_project_path(args.resource_root), "RenderingRegression.eveproj", missing)
    validate_sponza_lighting_assets(
        args.resource_root
        / ".generated"
        / "EvoEngine-DemoProjects"
        / "RenderingRegression"
        / "Assets"
        / "Lighting"
        / "Sponza",
        missing,
    )
    if args.previews:
        validate_nonempty_file(
            preview_path(args.resource_root, "rendering-regression"),
            "Launcher/DemoPreviews/rendering-regression.png",
            missing,
        )
    return missing


def capture_rendering_regression(args: argparse.Namespace, output_path: Path) -> None:
    editor = args.editor.resolve()
    if not editor.exists():
        raise RuntimeError(f"rendering-regression: missing editor needed to prepare demo: {editor}")
    command = [
        str(editor),
        "--demo",
        "rendering-regression",
        "--editor",
        "--capture-demo-preview",
        str(output_path.resolve()),
        "--preview-render-mode",
        "rasterization",
        "--preview-width",
        str(args.preview_width),
        "--preview-height",
        str(args.preview_height),
        "--preview-warmup-frames",
        str(args.rendering_regression_preview_warmup_frames),
    ]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.unlink(missing_ok=True)
    run(command, editor.parent)
    if not output_path.is_file() or output_path.stat().st_size == 0:
        raise RuntimeError(f"rendering-regression: preview was not written: {output_path}")


def prepare_rendering_regression(args: argparse.Namespace) -> None:
    if args.previews:
        capture_rendering_regression(args, preview_path(args.resource_root, "rendering-regression"))
    elif args.override or not rendering_regression_project_path(args.resource_root).is_file():
        capture_rendering_regression(args, repo_root() / "out" / "prepare-demos" / "rendering-regression.png")
    print("rendering-regression: resources are ready.", flush=True)


def spatial_dragon_demo_root(resource_root: Path) -> Path:
    return resource_root / "EvoEngine-DemoProjects" / "3DGS"


def validate_3dgs(args: argparse.Namespace) -> list[str]:
    demo_root = spatial_dragon_demo_root(args.resource_root)
    asset_path = demo_root / "Assets" / "GaussianSplats" / "spatial_dragon.ply"
    missing: list[str] = []
    validate_file(demo_root / "3DGS.eveproj", "3DGS.eveproj", missing)
    if not validate_spatial_dragon(asset_path):
        missing.append("spatial_dragon.ply")
    validate_file(Path(str(asset_path) + ".evefilemeta"), "spatial_dragon.ply.evefilemeta", missing)
    if args.previews:
        validate_nonempty_file(preview_path(args.resource_root, "3dgs"), "Launcher/DemoPreviews/3dgs.png", missing)
    return missing


def download_spatial_dragon(url: str, path: Path, force: bool, no_download: bool) -> None:
    if not force and validate_spatial_dragon(path):
        print(f"3dgs: verified existing asset: {path}", flush=True)
        return
    if no_download:
        raise RuntimeError(f"3dgs: missing or invalid asset and --no-download was supplied: {path}")

    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_suffix(path.suffix + ".download")
    temp_path.unlink(missing_ok=True)
    print(f"Downloading {url}", flush=True)
    with urllib.request.urlopen(url) as response, temp_path.open("wb") as output:
        output.write(response.read())
    if temp_path.stat().st_size != SPATIAL_DRAGON_ASSET_SIZE:
        temp_path.unlink(missing_ok=True)
        raise RuntimeError(f"3dgs: downloaded asset size mismatch: expected {SPATIAL_DRAGON_ASSET_SIZE} bytes")
    if sha256(temp_path) != SPATIAL_DRAGON_ASSET_SHA256:
        temp_path.unlink(missing_ok=True)
        raise RuntimeError(f"3dgs: downloaded asset SHA256 mismatch: expected {SPATIAL_DRAGON_ASSET_SHA256}")
    temp_path.replace(path)
    print(f"3dgs: wrote verified asset: {path}", flush=True)


def write_3dgs_files(resource_root: Path, overwrite_project: bool) -> tuple[Path, Path]:
    demo_root = spatial_dragon_demo_root(resource_root)
    asset_path = demo_root / "Assets" / "GaussianSplats" / "spatial_dragon.ply"
    project_path = demo_root / "3DGS.eveproj"
    project_text = """application_name: 3D Gaussian Splatting
preferred_editor: EvoEngineEditor
startup_runtime_packages:
  []
"""
    if overwrite_project or not project_path.exists():
        write_text_if_changed(project_path, project_text)
    folder_meta = f"""handle_: {SPATIAL_DRAGON_FOLDER_HANDLE}
type_name: GaussianSplats
"""
    write_text_if_changed(demo_root / "Assets" / "GaussianSplats.evefoldermeta", folder_meta)
    asset_meta = f"""asset_extension_: .ply
asset_file_name_: spatial_dragon
asset_type_name_: GaussianSplat
asset_handle_: {SPATIAL_DRAGON_ASSET_HANDLE}
"""
    write_text_if_changed(Path(str(asset_path) + ".evefilemeta"), asset_meta)
    readme_text = f"""# 3D Gaussian Splatting Demo

This generated demo uses Aimi Sekiguchi's Spatial Dragon 3DGS asset.

- Source: https://github.com/sekiguchiaimi/spatialdragon-3dgs
- Asset: data/spatial_dragon.ply
- License: CC0 1.0 public domain
- Size: {SPATIAL_DRAGON_ASSET_SIZE} bytes
- SHA256: {SPATIAL_DRAGON_ASSET_SHA256}

Run `python Scripts/prepare_demos.py --demo 3dgs` from the EvoEngine repository root to recreate the asset metadata.
Run the editor with `--demo 3dgs` to create or refresh the persistent scene.
"""
    write_text_if_changed(demo_root / "README.md", readme_text)
    return demo_root, asset_path


def prepare_3dgs(args: argparse.Namespace) -> None:
    demo_root, asset_path = write_3dgs_files(args.resource_root, args.override)
    if args.reset_scene:
        reset_scene_files(demo_root)
    download_spatial_dragon(args.spatial_dragon_asset_url, asset_path, args.override, args.no_download)
    maybe_capture_launcher_preview(args, "3dgs", preview_path(args.resource_root, "3dgs"), args.preview_warmup_frames)
    print(f"3dgs: resources are ready under {demo_root}", flush=True)


def bicycle_demo_root(resource_root: Path) -> Path:
    return resource_root / "EvoEngine-DemoProjects" / "Bicycle"


def validate_bicycle(args: argparse.Namespace) -> list[str]:
    demo_root = bicycle_demo_root(args.resource_root)
    asset_path = demo_root / "Assets" / "GaussianSplats" / "bicycle.ply"
    missing: list[str] = []
    validate_file(demo_root / "Bicycle.eveproj", "Bicycle.eveproj", missing)
    if not validate_ply(asset_path):
        missing.append("bicycle.ply")
    validate_file(Path(str(asset_path) + ".evefilemeta"), "bicycle.ply.evefilemeta", missing)
    if args.previews:
        validate_nonempty_file(preview_path(args.resource_root, "bicycle"), "Launcher/DemoPreviews/bicycle.png", missing)
    return missing


def download_archive(url: str, archive_path: Path, force: bool, no_download: bool) -> None:
    if archive_path.is_file() and not force:
        print(f"bicycle: using existing archive: {archive_path}", flush=True)
        return
    if no_download:
        raise RuntimeError(f"bicycle: missing archive and --no-download was supplied: {archive_path}")

    archive_path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = archive_path.with_suffix(archive_path.suffix + ".download")
    temp_path.unlink(missing_ok=True)
    print(f"Downloading {url}", flush=True)
    with urllib.request.urlopen(url) as response, temp_path.open("wb") as output:
        shutil.copyfileobj(response, output, 1024 * 1024)
    temp_path.replace(archive_path)
    print(f"bicycle: wrote archive: {archive_path}", flush=True)


def find_zip_member(archive: zipfile.ZipFile, candidates: tuple[str, ...]) -> str:
    members = {name.replace("\\", "/"): name for name in archive.namelist()}
    for candidate in candidates:
        if candidate in members:
            return members[candidate]
    raise RuntimeError("bicycle: archive does not contain any expected member: " + ", ".join(candidates))


def extract_member(archive: zipfile.ZipFile, member: str, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temp_path = destination.with_suffix(destination.suffix + ".extract")
    temp_path.unlink(missing_ok=True)
    with archive.open(member) as source, temp_path.open("wb") as output:
        shutil.copyfileobj(source, output, 1024 * 1024)
    temp_path.replace(destination)


def extract_bicycle_files(archive_path: Path, asset_path: Path) -> None:
    with zipfile.ZipFile(archive_path) as archive:
        ply_member = find_zip_member(archive, BICYCLE_PLY_MEMBERS)
        extract_member(archive, ply_member, asset_path)
    if not validate_ply(asset_path):
        raise RuntimeError(f"bicycle: extracted PLY is missing a valid PLY header: {asset_path}")
    print(f"bicycle: extracted PLY: {asset_path}", flush=True)


def remove_legacy_bicycle_camera_files(demo_root: Path) -> None:
    assets_root = demo_root / "Assets"
    shutil.rmtree(assets_root / "Cameras", ignore_errors=True)
    (assets_root / "Cameras.evefoldermeta").unlink(missing_ok=True)


def write_bicycle_files(resource_root: Path, overwrite_project: bool) -> tuple[Path, Path, Path]:
    demo_root = bicycle_demo_root(resource_root)
    asset_path = demo_root / "Assets" / "GaussianSplats" / "bicycle.ply"
    archive_path = demo_root / "models.zip"
    project_path = demo_root / "Bicycle.eveproj"
    remove_legacy_bicycle_camera_files(demo_root)

    project_text = """application_name: Bicycle
preferred_editor: EvoEngineEditor
startup_runtime_packages:
  []
"""
    if overwrite_project or not project_path.exists():
        write_text_if_changed(project_path, project_text)
    splats_folder_meta = f"""handle_: {BICYCLE_FOLDER_HANDLE}
type_name: GaussianSplats
"""
    write_text_if_changed(demo_root / "Assets" / "GaussianSplats.evefoldermeta", splats_folder_meta)
    asset_meta = f"""asset_extension_: .ply
asset_file_name_: bicycle
asset_type_name_: GaussianSplat
asset_handle_: {BICYCLE_ASSET_HANDLE}
"""
    write_text_if_changed(Path(str(asset_path) + ".evefilemeta"), asset_meta)
    readme_text = f"""# Bicycle Demo

This generated demo uses the pretrained INRIA 3D Gaussian Splatting Bicycle scene.

- Source: {BICYCLE_ASSET_URL}
- PLY member: {BICYCLE_PLY_MEMBERS[0]}
- Local PLY: Assets/GaussianSplats/bicycle.ply

Run `python Scripts/prepare_demos.py --demo bicycle` from the EvoEngine repository root to recreate the extracted files
and asset metadata. The archive is removed after extraction by default; pass `--keep-archive` to keep `models.zip`.
Run the editor with `--demo bicycle` to create or refresh the persistent scene.
"""
    write_text_if_changed(demo_root / "README.md", readme_text)
    return demo_root, asset_path, archive_path


def prepare_bicycle(args: argparse.Namespace) -> None:
    demo_root, asset_path, archive_path = write_bicycle_files(args.resource_root, args.override)
    if args.reset_scene:
        reset_scene_files(demo_root)
    if not args.override and validate_ply(asset_path):
        print(f"bicycle: verified existing files under {demo_root}", flush=True)
    else:
        download_archive(args.bicycle_asset_url, archive_path, args.override, args.no_download)
        extract_bicycle_files(archive_path, asset_path)
    if not args.keep_archive:
        archive_path.unlink(missing_ok=True)
    maybe_capture_launcher_preview(args, "bicycle", preview_path(args.resource_root, "bicycle"), args.preview_warmup_frames)
    print(f"bicycle: resources are ready under {demo_root}", flush=True)


def bistro_demo_root(resource_root: Path) -> Path:
    return resource_root / ".generated" / "EvoEngine-DemoProjects" / "Bistro"


def validate_bistro(args: argparse.Namespace) -> list[str]:
    demo_root = bistro_demo_root(args.resource_root)
    asset_root = demo_root / "Assets" / "Models" / "Bistro"
    missing: list[str] = []
    validate_file(demo_root / "Bistro.eveproj", "Bistro.eveproj", missing)
    validate_file(asset_root / "bistro.gltf", "bistro.gltf", missing)
    validate_file(asset_root / "bistro.bin", "bistro.bin", missing)
    validate_directory(asset_root / "textures", "textures", missing)
    validate_directory(asset_root / "objects", "objects", missing)
    validate_file(asset_root / "bistro.gltf.evefilemeta", "bistro.gltf.evefilemeta", missing)
    if args.previews:
        validate_nonempty_file(preview_path(args.resource_root, "bistro"), "Launcher/DemoPreviews/bistro.png", missing)
    return missing


def run_git(arguments: list[str], cwd: Path | None = None) -> None:
    command = ["git", *arguments]
    print("Running " + format_command(command), flush=True)
    subprocess.run(command, cwd=cwd, check=True)


def update_bistro_source_cache(source_root: Path, repo_url: str, force_download: bool, no_download: bool) -> Path:
    if source_root.is_dir() and (source_root / ".git").is_dir():
        if force_download:
            run_git(["fetch", "--depth", "1", "origin"], source_root)
            run_git(["checkout", "--detach", "FETCH_HEAD"], source_root)
        else:
            print(f"bistro: using existing source cache: {source_root}", flush=True)
        return source_root

    if source_root.exists() and not (source_root / ".git").is_dir():
        print(f"bistro: using supplied source folder: {source_root}", flush=True)
        return source_root

    if no_download:
        raise RuntimeError(f"bistro: missing source cache and --no-download was supplied: {source_root}")

    source_root.parent.mkdir(parents=True, exist_ok=True)
    run_git(["clone", "--depth", "1", repo_url, str(source_root)])
    return source_root


def verify_bistro_source(source_root: Path, gltf_name: str) -> dict[str, int | str]:
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
        raise RuntimeError("bistro: source is missing " + ", ".join(missing))

    with gltf_path.open("r", encoding="utf-8-sig") as stream:
        gltf = json.load(stream)
    extensions = set(gltf.get("extensionsUsed", []))
    if "MSFT_texture_dds" not in extensions:
        raise RuntimeError(f"bistro: {gltf_name} does not declare MSFT_texture_dds.")
    if "KHR_lights_punctual" not in extensions:
        raise RuntimeError(f"bistro: {gltf_name} does not declare KHR_lights_punctual.")

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
            if any(image_path.with_suffix(extension).exists() for extension in BISTRO_IMAGE_FALLBACK_EXTENSIONS):
                alternate_image_count += 1
            else:
                missing_images.append(uri)
    if dds_texture_count == 0:
        raise RuntimeError(f"bistro: {gltf_name} does not contain MSFT_texture_dds texture sources.")
    if missing_images:
        raise RuntimeError("bistro: source is missing image URIs: " + ", ".join(missing_images[:8]))

    return {
        "gltf": gltf_name,
        "images": len(images),
        "textures": len(textures),
        "materials": len(materials),
        "dds_textures": dds_texture_count,
        "alternate_images": alternate_image_count,
    }


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
        elif destination.stat().st_size != source.stat().st_size:
            copy_path(source, destination)
        else:
            return "existing"
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


def prepare_one_bistro_asset(source: Path, destination: Path, asset_mode: str, copy_fallback: bool) -> str:
    if asset_mode == "copy":
        copy_path(source, destination)
        return "copy"
    return link_or_copy(source, destination, copy_fallback)


def prepare_bistro_asset_path(source_root: Path, asset_root: Path, asset_mode: str, copy_fallback: bool) -> dict[str, str]:
    modes: dict[str, str] = {}
    if asset_mode == "none":
        return modes
    for name in ("bistro.gltf", "bistro.bin", "LICENSE", "README.md"):
        source = source_root / name
        if source.exists():
            modes[name] = prepare_one_bistro_asset(source, asset_root / name, asset_mode, copy_fallback)
    for name in ("objects", "textures"):
        modes[name] = prepare_one_bistro_asset(source_root / name, asset_root / name, asset_mode, copy_fallback)
    return modes


def write_bistro_files(
    resource_root: Path,
    source_root: Path,
    stats: dict[str, int | str],
    asset_mode: str,
    copy_fallback: bool,
    overwrite_project: bool,
) -> tuple[Path, Path, dict[str, str]]:
    demo_root = bistro_demo_root(resource_root)
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
    if overwrite_project or not project_path.exists():
        write_text_if_changed(project_path, project_text)
    models_folder_meta = f"""handle_: {BISTRO_MODELS_FOLDER_HANDLE}
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
asset_handle_: {BISTRO_PREFAB_ASSET_HANDLE}
"""
    write_text_if_changed(asset_root / "bistro.gltf.evefilemeta", asset_meta)
    readme_text = f"""# Bistro Demo

This generated demo uses the Amazon Lumberyard Bistro glTF scene from zeux/niagara_bistro.

- Source: {BISTRO_SOURCE_REPO_URL}
- Source cache: {source_root}
- Local glTF: Assets/Models/Bistro/bistro.gltf
- Images: {stats["images"]}
- Textures: {stats["textures"]}
- Materials: {stats["materials"]}
- MSFT_texture_dds textures: {stats["dds_textures"]}
- Image URI alternate files: {stats["alternate_images"]}

Run `python Scripts/prepare_demos.py --demo bistro` from the EvoEngine repository root to recreate the local ignored
project. Run the editor with `--demo bistro` after generation.
"""
    write_text_if_changed(demo_root / "README.md", readme_text)
    modes = prepare_bistro_asset_path(source_root, asset_root, asset_mode, copy_fallback)
    return demo_root, project_path, modes


def prepare_bistro(args: argparse.Namespace) -> None:
    source_root = args.bistro_source_root.resolve() if args.bistro_source_root else args.resource_root / ".generated" / "niagara_bistro"
    source_root = update_bistro_source_cache(source_root, args.bistro_repo_url, args.override, args.no_download)
    stats = verify_bistro_source(source_root, args.bistro_gltf)
    demo_root, project_path, modes = write_bistro_files(
        args.resource_root,
        source_root,
        stats,
        args.bistro_asset_mode,
        not args.no_copy_fallback,
        args.override,
    )
    if args.reset_scene:
        reset_scene_files(demo_root)
    if modes:
        print("bistro: prepared asset links/files: " + ", ".join(f"{k}={v}" for k, v in sorted(modes.items())), flush=True)
    maybe_capture_launcher_preview(args, "bistro", preview_path(args.resource_root, "bistro"), args.bistro_preview_warmup_frames)
    print(
        "bistro: verified source: "
        f"{stats['images']} images, {stats['textures']} textures, {stats['materials']} materials, "
        f"{stats['dds_textures']} MSFT_texture_dds textures, {stats['alternate_images']} image URI alternates",
        flush=True,
    )
    print(f"bistro: resources are ready under {demo_root}", flush=True)
    print(f"bistro: project: {project_path}", flush=True)


DEMO_VALIDATORS = {
    "rendering": validate_rendering,
    "rendering-regression": validate_rendering_regression,
    "3dgs": validate_3dgs,
    "bicycle": validate_bicycle,
    "bistro": validate_bistro,
}

DEMO_PREPARERS = {
    "rendering": prepare_rendering,
    "rendering-regression": prepare_rendering_regression,
    "3dgs": prepare_3dgs,
    "bicycle": prepare_bicycle,
    "bistro": prepare_bistro,
}


def parse_demo_values(values: list[str] | None) -> list[str]:
    if not values:
        return list(DEMO_ORDER)
    demos: list[str] = []
    for value in values:
        for demo in value.split(","):
            demo = demo.strip().lower()
            if not demo:
                continue
            if demo == "all":
                demos.extend(DEMO_ORDER)
                continue
            if demo not in DEMO_ORDER:
                raise argparse.ArgumentTypeError(f"Unknown demo '{demo}'. Expected one of: {', '.join(DEMO_ORDER)}")
            demos.append(demo)
    ordered: list[str] = []
    for demo in demos:
        if demo not in ordered:
            ordered.append(demo)
    return ordered


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--demo",
        action="append",
        help="Demo id to process. Repeat or pass comma-separated ids. Defaults to all generated demos.",
    )
    parser.add_argument("--validate", action="store_true", help="Run validation. Defaults on when no action is supplied.")
    parser.add_argument("--prepare", action="store_true", help="Prepare missing files. Defaults on when no action is supplied.")
    parser.add_argument("--override", action="store_true", help="Force preparation without running validation.")
    parser.add_argument("--resource-root", type=Path, default=root / "Resources")
    parser.add_argument("--editor", type=Path, default=default_editor_path(root), help="EvoEngineEditor used for previews.")
    parser.add_argument("--no-download", action="store_true", help="Fail instead of downloading missing external assets.")
    parser.add_argument("--reset-scene", action="store_true", help="Remove generated New Scene files for selected demos.")

    parser.add_argument("--previews", dest="previews", action="store_true", default=True)
    parser.add_argument("--no-previews", dest="previews", action="store_false", help="Skip launcher preview validation/capture.")
    parser.add_argument("--preview-width", type=int, default=1280)
    parser.add_argument("--preview-height", type=int, default=720)
    parser.add_argument("--preview-warmup-frames", type=int, default=24)
    parser.add_argument("--bistro-preview-warmup-frames", type=int, default=256)
    parser.add_argument("--rendering-regression-preview-warmup-frames", type=int, default=1800)

    parser.add_argument("--config", default="RelWithDebInfo")
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument("--build-rendering-readme", dest="build_rendering_readme", action="store_true", default=True)
    parser.add_argument("--no-build-rendering-readme", dest="build_rendering_readme", action="store_false")
    parser.add_argument("--rendering-readme", dest="rendering_readme", action="store_true", default=True)
    parser.add_argument("--no-rendering-readme", dest="rendering_readme", action="store_false")
    parser.add_argument("--readme-width", type=int, default=1920)
    parser.add_argument("--readme-height", type=int, default=1080)
    parser.add_argument("--readme-warmup-frames", type=int, default=512)
    parser.add_argument("--readme-timeout", type=float, default=900.0)

    parser.add_argument("--spatial-dragon-asset-url", default=SPATIAL_DRAGON_ASSET_URL)
    parser.add_argument("--bicycle-asset-url", default=BICYCLE_ASSET_URL)
    parser.add_argument("--keep-archive", action="store_true")

    parser.add_argument("--bistro-source-root", type=Path)
    parser.add_argument("--bistro-repo-url", default=BISTRO_SOURCE_REPO_URL)
    parser.add_argument("--bistro-gltf", default="bistro.gltf")
    parser.add_argument("--bistro-asset-mode", choices=("symlink", "copy", "none"), default="symlink")
    parser.add_argument("--no-copy-fallback", action="store_true")
    args = parser.parse_args()

    try:
        args.demos = parse_demo_values(args.demo)
    except argparse.ArgumentTypeError as error:
        parser.error(str(error))
    if args.override:
        args.validate = False
        args.prepare = True
    elif not args.validate and not args.prepare:
        args.validate = True
        args.prepare = True
    args.resource_root = args.resource_root.resolve()
    args.build_dir = args.build_dir.resolve() if args.build_dir.is_absolute() else (root / args.build_dir).resolve()
    if args.preview_width <= 0 or args.preview_height <= 0:
        raise SystemExit("--preview-width and --preview-height must be positive.")
    if (
        args.preview_warmup_frames < 0
        or args.bistro_preview_warmup_frames < 0
        or args.rendering_regression_preview_warmup_frames < 0
        or args.readme_warmup_frames < 0
    ):
        raise SystemExit("Warmup frame counts must be non-negative.")
    if args.readme_width <= 0 or args.readme_height <= 0:
        raise SystemExit("--readme-width and --readme-height must be positive.")
    return args


def print_validation(demo: str, issues: list[str], phase: str) -> None:
    if issues:
        print(f"{demo}: {phase} validation missing/invalid: " + ", ".join(issues), flush=True)
    else:
        print(f"{demo}: {phase} validation passed.", flush=True)


def process_demo(args: argparse.Namespace, demo: str) -> bool:
    print(f"\n==> {demo}", flush=True)
    validator = DEMO_VALIDATORS[demo]
    preparer = DEMO_PREPARERS[demo]
    if args.override:
        preparer(args)
        return True

    initial_issues = validator(args)
    if args.validate:
        print_validation(demo, initial_issues, "initial")

    needs_prepare = args.prepare and (args.override or bool(initial_issues))
    if needs_prepare:
        preparer(args)
    elif args.prepare:
        print(f"{demo}: preparation skipped because validation already passes.", flush=True)

    if args.validate and args.prepare:
        final_issues = validator(args)
        print_validation(demo, final_issues, "final")
        return not final_issues
    if args.validate:
        return not initial_issues
    return True


def main() -> int:
    args = parse_args()
    ok = True
    for demo in args.demos:
        try:
            ok = process_demo(args, demo) and ok
        except Exception as error:
            print(f"{demo}: {error}", file=sys.stderr, flush=True)
            ok = False
            if len(args.demos) == 1:
                break
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
