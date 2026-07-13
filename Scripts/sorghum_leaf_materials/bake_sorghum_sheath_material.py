#!/usr/bin/env python3
"""Bake a seamless culm/sheath PBR set from all nine lower atlas regions."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image

from bake_sorghum_leaf_atlas import ensure_evefilemeta


MAPS = ("albedo", "normal", "roughness", "metallic", "ao")


def bake_map(source: Path, destination: Path, output_size: int) -> None:
    atlas = Image.open(source).convert("RGBA")
    tile_width = atlas.width // 3
    tile_height = atlas.height // 3
    yy, xx = np.mgrid[0:output_size, 0:output_size]
    u = xx / output_size
    v = yy / output_size
    weighted = np.zeros((output_size, output_size, 4), dtype=np.float64)
    total_weight = np.zeros((output_size, output_size, 1), dtype=np.float64)
    for index in range(9):
        column, row = index % 3, index // 3
        tile = atlas.crop((
            column * tile_width,
            row * tile_height + tile_height // 2,
            (column + 1) * tile_width,
            (row + 1) * tile_height,
        )).resize((output_size, output_size), Image.Resampling.LANCZOS)
        phase = index * 2.399963229728653
        weight = 1.0 + 0.55 * np.sin(2.0 * np.pi * u + phase) * np.cos(2.0 * np.pi * v + phase * 0.73)
        weight = weight[..., None]
        weighted += np.asarray(tile, dtype=np.float64) * weight
        total_weight += weight
    result = Image.fromarray(np.clip(weighted / total_weight, 0, 255).astype(np.uint8), "RGBA")
    if destination.stem.endswith("albedo"):
        result.putalpha(255)
    destination.parent.mkdir(parents=True, exist_ok=True)
    result.save(destination, optimize=True)
    ensure_evefilemeta(destination, "Texture2D")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--atlas-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--size", type=int, default=1024)
    args = parser.parse_args()
    for map_name in MAPS:
        bake_map(
            args.atlas_dir / f"sorghum_lsystem_leaf_variants_{map_name}.png",
            args.out_dir / f"sorghum_sheath_{map_name}.png",
            args.size,
        )


if __name__ == "__main__":
    main()
