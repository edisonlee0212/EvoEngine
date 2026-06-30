#!/usr/bin/env python3
"""Generate the local EvoEngine 3D Gaussian Splatting demo resources."""

from __future__ import annotations

import argparse
import hashlib
import subprocess
import sys
import urllib.request
from pathlib import Path


ASSET_URL = "https://raw.githubusercontent.com/sekiguchiaimi/spatialdragon-3dgs/main/data/spatial_dragon.ply"
ASSET_SIZE = 1_571_111
ASSET_SHA256 = "40D7FDEBEB6A9A5755074F4F02A759EEE19BF15F46520A8D79B5F42BDE42921D"
ASSET_HANDLE = 9739484957885691067
GAUSSIAN_SPLATS_FOLDER_HANDLE = 17421499951906451750


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def verify_asset(path: Path) -> bool:
    return path.is_file() and path.stat().st_size == ASSET_SIZE and sha256(path) == ASSET_SHA256


def write_text_if_changed(path: Path, text: str) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.read_text(encoding="utf-8") == text:
        return False
    path.write_text(text, encoding="utf-8", newline="\n")
    return True


def download_asset(url: str, path: Path, force: bool, no_download: bool) -> None:
    if not force and verify_asset(path):
        print(f"Verified existing asset: {path}")
        return
    if no_download:
        raise RuntimeError(f"Missing or invalid asset and --no-download was supplied: {path}")

    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_suffix(path.suffix + ".download")
    print(f"Downloading {url}")
    with urllib.request.urlopen(url) as response, temp_path.open("wb") as output:
        output.write(response.read())
    if temp_path.stat().st_size != ASSET_SIZE:
        temp_path.unlink(missing_ok=True)
        raise RuntimeError(f"Downloaded asset size mismatch: expected {ASSET_SIZE} bytes")
    if sha256(temp_path) != ASSET_SHA256:
        temp_path.unlink(missing_ok=True)
        raise RuntimeError(f"Downloaded asset SHA256 mismatch: expected {ASSET_SHA256}")
    temp_path.replace(path)
    print(f"Wrote verified asset: {path}")


def reset_scene_files(demo_root: Path) -> None:
    assets_root = demo_root / "Assets"
    for scene_path in assets_root.glob("New Scene*.evescene"):
        scene_path.unlink(missing_ok=True)
        Path(str(scene_path) + ".evefilemeta").unlink(missing_ok=True)


def write_demo_files(resource_root: Path, reset_project: bool) -> tuple[Path, Path]:
    demo_root = resource_root / "EvoEngine-DemoProjects" / "3DGS"
    asset_path = demo_root / "Assets" / "GaussianSplats" / "spatial_dragon.ply"
    project_path = demo_root / "3DGS.eveproj"

    project_text = """application_name: 3D Gaussian Splatting
preferred_editor: EvoEngineEditor
startup_runtime_packages:
  []
"""
    if reset_project or not project_path.exists():
        write_text_if_changed(project_path, project_text)

    folder_meta = f"""handle_: {GAUSSIAN_SPLATS_FOLDER_HANDLE}
type_name: GaussianSplats
"""
    write_text_if_changed(demo_root / "Assets" / "GaussianSplats.evefoldermeta", folder_meta)

    asset_meta = f"""asset_extension_: .ply
asset_file_name_: spatial_dragon
asset_type_name_: GaussianSplat
asset_handle_: {ASSET_HANDLE}
"""
    write_text_if_changed(Path(str(asset_path) + ".evefilemeta"), asset_meta)

    readme_text = f"""# 3D Gaussian Splatting Demo

This generated demo uses Aimi Sekiguchi's Spatial Dragon 3DGS asset.

- Source: https://github.com/sekiguchiaimi/spatialdragon-3dgs
- Asset: data/spatial_dragon.ply
- License: CC0 1.0 public domain
- Size: {ASSET_SIZE} bytes
- SHA256: {ASSET_SHA256}

Run `python Scripts/generate_3dgs_demo.py` from the EvoEngine repository root to recreate the asset metadata.
Run the editor with `--demo 3dgs` to create or refresh the persistent scene.
"""
    write_text_if_changed(demo_root / "README.md", readme_text)
    return demo_root, asset_path


def capture_preview(editor: Path, preview_path: Path, warmup_frames: int) -> None:
    command = [
        str(editor),
        "--demo",
        "3dgs",
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
    parser.add_argument("--editor", type=Path, help="Optional EvoEngineEditor executable used to create the scene.")
    parser.add_argument(
        "--preview",
        type=Path,
        default=repo_root() / "Resources" / "Launcher" / "DemoPreviews" / "3dgs.png",
    )
    parser.add_argument("--preview-warmup-frames", type=int, default=24)
    args = parser.parse_args()

    resource_root = args.resource_root.resolve()
    demo_root, asset_path = write_demo_files(resource_root, args.reset_scene)
    if args.reset_scene:
        reset_scene_files(demo_root)
    download_asset(args.asset_url, asset_path, args.force_download, args.no_download)
    if args.editor:
        capture_preview(args.editor.resolve(), args.preview.resolve(), max(0, args.preview_warmup_frames))
    print(f"3DGS demo resources are ready under {demo_root}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"generate_3dgs_demo.py: {error}", file=sys.stderr)
        raise SystemExit(1)
