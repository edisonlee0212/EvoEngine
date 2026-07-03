#!/usr/bin/env python3
"""Apply the baked sorghum leaf/stem atlas textures to generated scene materials.

This intentionally edits scene asset YAML only. It does not require C++ changes:
Leaf Mesh and Stem Mesh renderers keep their existing material handles, and the
matching Material local assets are updated to point at regular project Texture2D
assets registered by their .evefilemeta files.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable


TEXTURE_FILES = {
    "albedo_texture_": "sorghum_leaf_stem_atlas_albedo.png.evefilemeta",
    "normal_texture_": "sorghum_leaf_stem_atlas_normal.png.evefilemeta",
    "metallic_texture_": "sorghum_leaf_stem_atlas_metallic.png.evefilemeta",
    "roughness_texture_": "sorghum_leaf_stem_atlas_roughness.png.evefilemeta",
    "ao_texture_": "sorghum_leaf_stem_atlas_ao.png.evefilemeta",
}


def load_texture_handles(atlas_dir: Path) -> dict[str, int]:
    handles: dict[str, int] = {}
    for slot, filename in TEXTURE_FILES.items():
        meta_path = atlas_dir / filename
        if not meta_path.exists():
            raise FileNotFoundError(f"Missing texture metadata: {meta_path}")
        handle = None
        asset_type = None
        for line in meta_path.read_text(encoding="utf-8").splitlines():
            if line.startswith("asset_handle_:"):
                handle = int(line.split(":", 1)[1].strip())
            elif line.startswith("asset_type_name_:"):
                asset_type = line.split(":", 1)[1].strip()
        if handle is None:
            raise ValueError(f"No asset_handle_ in {meta_path}")
        if asset_type != "Texture2D":
            raise ValueError(f"{meta_path} is {asset_type!r}, expected Texture2D")
        handles[slot] = handle
    return handles


def collect_renderer_materials(scene_lines: Iterable[str], target_entity_names: set[str]) -> set[int]:
    material_handles: set[int] = set()
    current_entity = ""
    in_mesh_renderer = False
    in_material_ref = False

    for line in scene_lines:
        stripped = line.strip()
        if stripped == "LocalAssets:":
            break
        if stripped.startswith("- n: "):
            current_entity = stripped[5:]
            in_mesh_renderer = False
            in_material_ref = False
        elif stripped == "- tn: MeshRenderer":
            in_mesh_renderer = True
            in_material_ref = False
        elif in_mesh_renderer and stripped == "material:":
            in_material_ref = True
        elif in_mesh_renderer and in_material_ref and stripped.startswith("asset_handle_:"):
            if current_entity in target_entity_names:
                material_handles.add(int(stripped.split(":", 1)[1].strip()))
            in_mesh_renderer = False
            in_material_ref = False

    return material_handles


def patch_scene_lines(lines: list[str], target_materials: set[int], texture_handles: dict[str, int]) -> tuple[list[str], int]:
    patched: list[str] = []
    in_material_asset = False
    waiting_for_material_handle = False
    patch_current_material = False
    pending_texture_slot = ""
    patched_materials: set[int] = set()

    for line in lines:
        stripped = line.strip()

        if stripped.startswith("- type_name: "):
            in_material_asset = stripped == "- type_name: Material"
            waiting_for_material_handle = in_material_asset
            patch_current_material = False
            pending_texture_slot = ""
            patched.append(line)
            continue

        if in_material_asset and waiting_for_material_handle and stripped.startswith("handle:"):
            handle = int(stripped.split(":", 1)[1].strip())
            patch_current_material = handle in target_materials
            if patch_current_material:
                patched_materials.add(handle)
            waiting_for_material_handle = False
            patched.append(line)
            continue

        texture_slot = stripped[:-1] if stripped.endswith(":") else ""
        if patch_current_material and texture_slot in TEXTURE_FILES:
            pending_texture_slot = texture_slot
            patched.append(line)
            continue

        if patch_current_material and pending_texture_slot and stripped.startswith("asset_handle_:"):
            indent = line[: len(line) - len(line.lstrip())]
            patched.append(f"{indent}asset_handle_: {texture_handles[pending_texture_slot]}\n")
            continue

        if patch_current_material and pending_texture_slot and stripped.startswith("type_name_:"):
            indent = line[: len(line) - len(line.lstrip())]
            patched.append(f'{indent}type_name_: Texture2D\n')
            pending_texture_slot = ""
            continue

        if patch_current_material and stripped.startswith("albedo_color:"):
            indent = line[: len(line) - len(line.lstrip())]
            patched.append(f"{indent}albedo_color: [1, 1, 1]\n")
            continue

        if patch_current_material and stripped.startswith("metallic:"):
            indent = line[: len(line) - len(line.lstrip())]
            patched.append(f"{indent}metallic: 0\n")
            continue

        patched.append(line)

    missing = target_materials - patched_materials
    if missing:
        sample = ", ".join(str(handle) for handle in sorted(missing)[:8])
        raise ValueError(f"Missing {len(missing)} target Material local assets, sample: {sample}")

    return patched, len(patched_materials)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--atlas-dir", type=Path, required=True)
    parser.add_argument(
        "--target-entity",
        action="append",
        default=[],
        help="Entity name whose MeshRenderer material should receive the atlas. Repeatable.",
    )
    args = parser.parse_args()

    target_entities = set(args.target_entity or ["Leaf Mesh", "Stem Mesh"])
    scene_path = args.scene
    lines = scene_path.read_text(encoding="utf-8", errors="replace").splitlines(keepends=True)

    target_materials = collect_renderer_materials(lines, target_entities)
    if not target_materials:
        raise ValueError(f"No MeshRenderer materials found for entities: {sorted(target_entities)}")

    texture_handles = load_texture_handles(args.atlas_dir)
    patched, count = patch_scene_lines(lines, target_materials, texture_handles)
    scene_path.write_text("".join(patched), encoding="utf-8", newline="")

    print(f"Patched {count} scene-local Material assets for {', '.join(sorted(target_entities))}.")
    for slot, handle in texture_handles.items():
        print(f"{slot}: {handle}")


if __name__ == "__main__":
    main()
