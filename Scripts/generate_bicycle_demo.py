#!/usr/bin/env python3
"""Generate the local EvoEngine Bicycle 3D Gaussian Splatting demo resources."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import urllib.request
import zipfile
from pathlib import Path


ASSET_URL = "https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/datasets/pretrained/models.zip"
BICYCLE_PLY_MEMBERS = (
    "bicycle/bicycle/point_cloud/iteration_30000/point_cloud.ply",
    "bicycle/point_cloud/iteration_30000/point_cloud.ply",
)
BICYCLE_CAMERA_MEMBERS = ("bicycle/bicycle/cameras.json", "bicycle/cameras.json")
ASSET_HANDLE = 14453709846752502031
GAUSSIAN_SPLATS_FOLDER_HANDLE = 13041787869273319741
CAMERAS_FOLDER_HANDLE = 15379860129740290489
CAMERAS_ASSET_HANDLE = 6692924907754216903


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def write_text_if_changed(path: Path, text: str) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.read_text(encoding="utf-8") == text:
        return False
    path.write_text(text, encoding="utf-8", newline="\n")
    return True


def validate_ply(path: Path) -> bool:
    if not path.is_file() or path.stat().st_size == 0:
        return False
    header = path.read_bytes()[:4096].decode("ascii", errors="ignore")
    return header.startswith("ply") and "element vertex" in header and "end_header" in header


def validate_cameras(path: Path) -> bool:
    if not path.is_file() or path.stat().st_size == 0:
        return False
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return False
    return isinstance(data, (list, dict))


def verify_outputs(asset_path: Path, cameras_path: Path) -> bool:
    return validate_ply(asset_path) and validate_cameras(cameras_path)


def download_archive(url: str, archive_path: Path, force: bool, no_download: bool) -> None:
    if archive_path.is_file() and not force:
        print(f"Using existing archive: {archive_path}")
        return
    if no_download:
        raise RuntimeError(f"Missing archive and --no-download was supplied: {archive_path}")

    archive_path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = archive_path.with_suffix(archive_path.suffix + ".download")
    temp_path.unlink(missing_ok=True)
    print(f"Downloading {url}")
    with urllib.request.urlopen(url) as response, temp_path.open("wb") as output:
        shutil.copyfileobj(response, output, 1024 * 1024)
    temp_path.replace(archive_path)
    print(f"Wrote archive: {archive_path}")


def find_zip_member(archive: zipfile.ZipFile, candidates: tuple[str, ...]) -> str:
    members = {name.replace("\\", "/"): name for name in archive.namelist()}
    for candidate in candidates:
        if candidate in members:
            return members[candidate]
    raise RuntimeError("Archive does not contain any expected Bicycle member: " + ", ".join(candidates))


def extract_member(archive: zipfile.ZipFile, member: str, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temp_path = destination.with_suffix(destination.suffix + ".extract")
    temp_path.unlink(missing_ok=True)
    with archive.open(member) as source, temp_path.open("wb") as output:
        shutil.copyfileobj(source, output, 1024 * 1024)
    temp_path.replace(destination)


def extract_bicycle_files(archive_path: Path, asset_path: Path, cameras_path: Path) -> None:
    with zipfile.ZipFile(archive_path) as archive:
        ply_member = find_zip_member(archive, BICYCLE_PLY_MEMBERS)
        camera_member = find_zip_member(archive, BICYCLE_CAMERA_MEMBERS)
        extract_member(archive, ply_member, asset_path)
        extract_member(archive, camera_member, cameras_path)
    if not validate_ply(asset_path):
        raise RuntimeError(f"Extracted Bicycle PLY is missing a valid PLY header: {asset_path}")
    if not validate_cameras(cameras_path):
        raise RuntimeError(f"Extracted Bicycle cameras file is not valid JSON: {cameras_path}")
    print(f"Extracted Bicycle PLY: {asset_path}")
    print(f"Extracted Bicycle cameras: {cameras_path}")


def reset_scene_files(demo_root: Path) -> None:
    assets_root = demo_root / "Assets"
    for scene_path in assets_root.glob("New Scene*.evescene"):
        scene_path.unlink(missing_ok=True)
        Path(str(scene_path) + ".evefilemeta").unlink(missing_ok=True)


def write_demo_files(resource_root: Path, reset_project: bool) -> tuple[Path, Path, Path, Path]:
    demo_root = resource_root / "EvoEngine-DemoProjects" / "Bicycle"
    asset_path = demo_root / "Assets" / "GaussianSplats" / "bicycle.ply"
    cameras_path = demo_root / "Assets" / "Cameras" / "cameras.json"
    archive_path = demo_root / "models.zip"
    project_path = demo_root / "Bicycle.eveproj"

    project_text = """application_name: Bicycle
preferred_editor: EvoEngineEditor
startup_runtime_packages:
  []
"""
    if reset_project or not project_path.exists():
        write_text_if_changed(project_path, project_text)

    splats_folder_meta = f"""handle_: {GAUSSIAN_SPLATS_FOLDER_HANDLE}
type_name: GaussianSplats
"""
    write_text_if_changed(demo_root / "Assets" / "GaussianSplats.evefoldermeta", splats_folder_meta)

    asset_meta = f"""asset_extension_: .ply
asset_file_name_: bicycle
asset_type_name_: GaussianSplat
asset_handle_: {ASSET_HANDLE}
"""
    write_text_if_changed(Path(str(asset_path) + ".evefilemeta"), asset_meta)

    cameras_folder_meta = f"""handle_: {CAMERAS_FOLDER_HANDLE}
type_name: Cameras
"""
    write_text_if_changed(demo_root / "Assets" / "Cameras.evefoldermeta", cameras_folder_meta)

    cameras_meta = f"""asset_extension_: .json
asset_file_name_: cameras
asset_type_name_: Json
asset_handle_: {CAMERAS_ASSET_HANDLE}
"""
    write_text_if_changed(Path(str(cameras_path) + ".evefilemeta"), cameras_meta)

    readme_text = f"""# Bicycle Demo

This generated demo uses the pretrained INRIA 3D Gaussian Splatting Bicycle scene.

- Source: https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/datasets/pretrained/models.zip
- PLY member: {BICYCLE_PLY_MEMBERS[0]}
- Camera member: {BICYCLE_CAMERA_MEMBERS[0]}
- Local PLY: Assets/GaussianSplats/bicycle.ply
- Local cameras: Assets/Cameras/cameras.json

Run `python Scripts/generate_bicycle_demo.py` from the EvoEngine repository root to recreate the extracted files and
asset metadata. The archive is removed after extraction by default; pass `--keep-archive` to keep `models.zip`.
Run the editor with `--demo bicycle` to create or refresh the persistent scene.
"""
    write_text_if_changed(demo_root / "README.md", readme_text)
    return demo_root, asset_path, cameras_path, archive_path


def capture_preview(editor: Path, preview_path: Path, warmup_frames: int) -> None:
    command = [
        str(editor),
        "--demo",
        "bicycle",
        "--capture-demo-preview",
        str(preview_path),
        "--preview-warmup-frames",
        str(warmup_frames),
    ]
    print("Running " + " ".join(command))
    subprocess.run(command, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--resource-root", type=Path, default=repo_root() / "Resources")
    parser.add_argument("--asset-url", default=ASSET_URL)
    parser.add_argument("--force-download", action="store_true")
    parser.add_argument("--no-download", action="store_true")
    parser.add_argument("--reset-scene", action="store_true")
    parser.add_argument("--keep-archive", action="store_true")
    parser.add_argument("--editor", type=Path, help="Optional EvoEngineEditor executable used to create the scene.")
    parser.add_argument(
        "--preview",
        type=Path,
        default=repo_root() / "Resources" / "Launcher" / "DemoPreviews" / "bicycle.png",
    )
    parser.add_argument("--preview-warmup-frames", type=int, default=24)
    args = parser.parse_args()

    resource_root = args.resource_root.resolve()
    demo_root, asset_path, cameras_path, archive_path = write_demo_files(resource_root, args.reset_scene)
    if args.reset_scene:
        reset_scene_files(demo_root)

    if not args.force_download and verify_outputs(asset_path, cameras_path):
        print(f"Verified existing Bicycle files under {demo_root}")
    else:
        download_archive(args.asset_url, archive_path, args.force_download, args.no_download)
        extract_bicycle_files(archive_path, asset_path, cameras_path)
    if not args.keep_archive:
        archive_path.unlink(missing_ok=True)
    if args.editor:
        capture_preview(args.editor.resolve(), args.preview.resolve(), max(0, args.preview_warmup_frames))
    print(f"Bicycle demo resources are ready under {demo_root}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"generate_bicycle_demo.py: {error}", file=sys.stderr)
        raise SystemExit(1)
