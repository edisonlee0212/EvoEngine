#!/usr/bin/env python3
"""Normalize plant-free manual scene templates with PBR soil context."""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path


REFERENCE_SCENES = {
    "10x10": {
        "target": "ManualAssets/Scenes/Sorghum_10x10.evescene",
        "project": "test_lsystem_sorghum_10x10_overlap.eveproj",
        "marker_count": 200,
    },
    "4x10": {
        "target": "ManualAssets/Scenes/Sorghum_4x10_PARBAR.evescene",
        "project": "test_lsystem_sorghum.eveproj",
        "marker_count": 40,
    },
}


def repo_root_from_script() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "CMakeLists.txt").exists() and (parent / "Resources" / "DigitalAgricultureProject").exists():
            return parent
    return Path(__file__).resolve().parents[1]


def configure_engine_imports(repo_root: Path, build_dir: Path, config: str) -> None:
    paths = [
        build_dir / "PythonBinding" / config,
        build_dir / "EvoEngine_App" / config,
        build_dir / "EvoEngine_App" / config / "Packages",
        build_dir / "EvoEngine_SDK" / config,
        build_dir / "EvoEngine_Services" / "CudaModule" / config,
    ]
    sys.path.insert(0, str(paths[0]))
    for path in paths:
        if path.exists() and hasattr(os, "add_dll_directory"):
            os.add_dll_directory(str(path))
    sys.path.insert(0, str(repo_root / "PythonBinding"))


def collect_default_scene_side_effects(project: Path) -> set[Path]:
    assets = project.resolve().parent / "Assets"
    return {path.resolve() for path in assets.glob("New Scene*.evescene*")}


def cleanup_new_default_scene_side_effects(project: Path, before: set[Path]) -> None:
    assets = (project.resolve().parent / "Assets").resolve()
    for path in sorted(collect_default_scene_side_effects(project) - before):
        resolved = path.resolve()
        try:
            resolved.relative_to(assets)
        except ValueError:
            continue
        resolved.unlink(missing_ok=True)


def update_project_start_scene(project: Path, metadata: Path) -> None:
    handle_match = re.search(r"^asset_handle_:\s*(\d+)\s*$", metadata.read_text(encoding="utf-8"), re.MULTILINE)
    if not handle_match:
        raise RuntimeError(f"Missing asset_handle_ in {metadata}")
    contents = project.read_text(encoding="utf-8")
    updated, count = re.subn(
        r"^start_scene_handle:\s*\d+\s*$",
        f"start_scene_handle: {handle_match.group(1)}",
        contents,
        count=1,
        flags=re.MULTILINE,
    )
    if count != 1:
        raise RuntimeError(f"Missing start_scene_handle in {project}")
    project.write_text(updated, encoding="utf-8")


def create_reference_scene(evo: object, args: argparse.Namespace, name: str) -> None:
    spec = REFERENCE_SCENES[name]
    project = args.project_root / spec["project"]
    target = Path(spec["target"])
    staged = target.with_name(f"_Staging_{target.name}")
    (args.project_root / "Assets" / target.parent).mkdir(parents=True, exist_ok=True)
    project_bytes = project.read_bytes()
    side_effects_before = collect_default_scene_side_effects(project)

    try:
        ok = evo.RunLSystemSorghumProject(project.resolve(), args.runtime_package_dir.resolve(), target)
        if not ok:
            raise RuntimeError(f"{name}: failed to start manual scene {target}")
        if not evo.WaitForProjectIdle(args.max_wait_frames):
            raise RuntimeError(f"{name}: project did not become idle after loading manual scene")
        marker_count = int(evo.ConvertSorghumLsPlantsToPlantingMarkers())
        if marker_count != spec["marker_count"]:
            raise RuntimeError(f"{name}: expected {spec['marker_count']} planting markers, got {marker_count}")
        if evo.GetSorghumLsPlantSceneMetadata(False):
            raise RuntimeError(f"{name}: manual template still contains SorghumLS plants")
        if not evo.EnsureIlluminationSoilContext():
            raise RuntimeError(f"{name}: failed to ensure PBR soil context")
        if not evo.ValidateIlluminationContext():
            raise RuntimeError(f"{name}: reference scene failed illumination context validation")
        if not evo.SaveActiveSceneAsProjectAsset(staged):
            raise RuntimeError(f"{name}: failed to save manual scene {staged}")
    finally:
        try:
            evo.Terminate()
        finally:
            project.write_bytes(project_bytes)
            cleanup_new_default_scene_side_effects(project, side_effects_before)
    assets = args.project_root / "Assets"
    staged_path = assets / staged
    target_path = assets / target
    os.replace(staged_path, target_path)
    staged_metadata = Path(f"{staged_path}.evefilemeta")
    target_metadata = Path(f"{target_path}.evefilemeta")
    metadata = staged_metadata.read_text(encoding="utf-8").replace(staged.stem, target.stem)
    promoted_metadata = staged_metadata.with_name(f"{staged_metadata.name}.promoted")
    promoted_metadata.write_text(metadata, encoding="utf-8")
    target_metadata.unlink(missing_ok=True)
    os.replace(promoted_metadata, target_metadata)
    staged_metadata.unlink()
    update_project_start_scene(project, target_metadata)
    print(f"{name}: normalized {target} with {spec['marker_count']} plant-free planting markers", flush=True)


def build_parser() -> argparse.ArgumentParser:
    repo_root = repo_root_from_script()
    project_root = repo_root / "Resources" / "DigitalAgricultureProject"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument("--build-dir", type=Path, default=repo_root / "out" / "build" / "vs2026-x64")
    parser.add_argument("--config", default="RelWithDebInfo")
    parser.add_argument("--project-root", type=Path, default=project_root)
    parser.add_argument("--runtime-package-dir", type=Path, default=repo_root / "out" / "build" / "vs2026-x64" / "EvoEngine_App" / "RelWithDebInfo" / "Packages")
    parser.add_argument("--scenes", choices=tuple(REFERENCE_SCENES) + ("all",), default="all")
    parser.add_argument("--max-wait-frames", type=int, default=30000)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    args.repo_root = args.repo_root.resolve()
    args.build_dir = args.build_dir.resolve()
    args.project_root = args.project_root.resolve()
    args.runtime_package_dir = args.runtime_package_dir.resolve()
    if args.scenes == "all":
        for name in REFERENCE_SCENES:
            subprocess.run(
                [
                    sys.executable,
                    str(Path(__file__).resolve()),
                    "--repo-root",
                    str(args.repo_root),
                    "--build-dir",
                    str(args.build_dir),
                    "--config",
                    args.config,
                    "--project-root",
                    str(args.project_root),
                    "--runtime-package-dir",
                    str(args.runtime_package_dir),
                    "--scenes",
                    name,
                    "--max-wait-frames",
                    str(args.max_wait_frames),
                ],
                check=True,
            )
        return
    configure_engine_imports(args.repo_root, args.build_dir, args.config)
    import PyDigitalAgriculture as evo

    for name in (args.scenes,):
        try:
            create_reference_scene(evo, args, name)
        finally:
            try:
                evo.Terminate()
            except Exception:
                pass


if __name__ == "__main__":
    main()
