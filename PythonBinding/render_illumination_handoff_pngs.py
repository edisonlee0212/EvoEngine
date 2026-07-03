#!/usr/bin/env python3
"""Render labeled 10x10 green-red PNG grids from the 10x10 illumination handoff."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap


PLANT_VALUE = "total_simulated_light_interception_proxy"


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[1]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def slug(value: object) -> str:
    text = str(value).strip().lower()
    for old, new in ((" ", "_"), (".", "p"), ("/", "_"), ("\\", "_"), (":", "_")):
        text = text.replace(old, new)
    return "".join(char for char in text if char.isalnum() or char in "_-")


def green_red_cmap() -> LinearSegmentedColormap:
    return LinearSegmentedColormap.from_list("red_yellow_green", ("#c9302c", "#f2df5a", "#238b45"))


def text_color(cmap: LinearSegmentedColormap, value: float) -> str:
    red, green, blue, _alpha = cmap(max(0.0, min(1.0, value)))
    luminance = 0.2126 * red + 0.7152 * green + 0.0722 * blue
    return "black" if luminance > 0.55 else "white"


def plant_retention_grids(rows: list[dict[str, str]]) -> dict[tuple[str, float], np.ndarray]:
    by_group: dict[tuple[str, float, int, int], dict[str, float]] = defaultdict(dict)
    for row in rows:
        key = (
            row["cultivar"],
            float(row["spacing_m"]),
            int(row["grid_row"]),
            int(row["grid_column"]),
        )
        by_group[key][row["scenario"]] = float(row[PLANT_VALUE])

    grids: dict[tuple[str, float], np.ndarray] = {}
    for cultivar in sorted({row["cultivar"] for row in rows}):
        spacings = sorted({float(row["spacing_m"]) for row in rows if row["cultivar"] == cultivar}, reverse=True)
        for spacing in spacings:
            grid = np.full((10, 10), np.nan, dtype=float)
            for grid_row in range(10):
                for grid_column in range(10):
                    pair = by_group[(cultivar, spacing, grid_row, grid_column)]
                    alone = pair.get("plant_alone", 0.0)
                    full = pair.get("full_context", math.nan)
                    grid[grid_row, grid_column] = full / alone if alone > 0.0 else math.nan
            grids[(cultivar, spacing)] = grid
    return grids


def render_plant_grid(
    output_path: Path,
    cultivar: str,
    spacing: float,
    values: np.ndarray,
    cmap: LinearSegmentedColormap,
) -> None:
    clipped = np.clip(values, 0.0, 1.0)
    fig, ax = plt.subplots(figsize=(7.8, 8.0), dpi=160)
    image = ax.imshow(clipped, cmap=cmap, vmin=0.0, vmax=1.0, origin="upper")

    ax.set_xticks(range(10))
    ax.set_yticks(range(10))
    ax.set_xlabel("Plant column")
    ax.set_ylabel("Plant row")
    ax.set_title(
        f"{cultivar} 10x10 Inter-Shadow Grid\n"
        f"Spacing {spacing:.1f} m; cell value = full-context light as percent of plant-alone light",
        fontsize=12,
        pad=14,
    )
    ax.set_xticks(np.arange(-0.5, 10, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, 10, 1), minor=True)
    ax.grid(which="minor", color="#262626", linewidth=1.1)
    ax.tick_params(which="minor", bottom=False, left=False)

    for row in range(10):
        for column in range(10):
            value = values[row, column]
            if math.isfinite(value):
                ax.text(
                    column,
                    row,
                    f"{value * 100:.0f}%",
                    ha="center",
                    va="center",
                    fontsize=7.5,
                    color=text_color(cmap, clipped[row, column]),
                )

    colorbar = fig.colorbar(image, ax=ax, fraction=0.046, pad=0.04)
    colorbar.set_label("Percent of plant-alone light")
    colorbar.set_ticks([0.0, 0.25, 0.5, 0.75, 1.0])
    colorbar.set_ticklabels(["0%", "25%", "50%", "75%", "100%"])
    fig.text(
        0.5,
        0.025,
        "Green = more light retained; red = stronger neighbor shading. Color is clipped at 100%.",
        ha="center",
        fontsize=9,
    )
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path)
    plt.close(fig)


def write_manifest(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["figure_type", "file", "label", "rows", "columns", "metric"])
        writer.writeheader()
        writer.writerows(rows)


def render_all(args: argparse.Namespace) -> tuple[int, Path]:
    handoff_dir = args.handoff_dir.resolve()
    output_dir = args.output_dir.resolve()
    plant_output_dir = output_dir / "10x10_green_red_grids"
    cmap = green_red_cmap()
    manifest: list[dict[str, object]] = []

    plant_rows = read_csv(handoff_dir / "10x10_inter_shadow_plants_long.csv")
    plant_grids = plant_retention_grids(plant_rows)
    for cultivar, spacing in sorted(plant_grids, key=lambda item: (item[0], -item[1])):
        output_path = plant_output_dir / f"10x10_{slug(cultivar)}_spacing_{slug(f'{spacing:.1f}m')}.png"
        render_plant_grid(output_path, cultivar, spacing, plant_grids[(cultivar, spacing)], cmap)
        manifest.append(
            {
                "figure_type": "10x10_inter_shadow_grid",
                "file": str(output_path.relative_to(output_dir)),
                "label": f"{cultivar} spacing {spacing:.1f} m",
                "rows": 10,
                "columns": 10,
                "metric": "full-context light as percent of plant-alone light",
            }
        )

    write_manifest(output_dir / "visualization_manifest.csv", manifest)
    return len(plant_grids), output_dir


def build_parser() -> argparse.ArgumentParser:
    repo_root = repo_root_from_script()
    handoff_dir = repo_root / "out" / "handoff" / "illumination_csv_handoff_2026-06-30"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--handoff-dir", default=handoff_dir, type=Path)
    parser.add_argument("--output-dir", default=handoff_dir / "labeled_pngs", type=Path)
    return parser


def main() -> None:
    plant_count, output_dir = render_all(build_parser().parse_args())
    print(f"output_dir={output_dir}")
    print(f"10x10_pngs={plant_count}")


if __name__ == "__main__":
    main()
