#!/usr/bin/env python3
"""Render labeled PARBAR heatmap PNGs from the date-height handoff CSVs."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap


DATE_ORDER = ("2021-07-01", "2021-07-14", "2021-08-18", "2021-08-30", "2021-09-02")
PANEL_ORDER = (
    ("Pawaga", "top", "Pawaga top"),
    ("Pawaga", "middle", "Pawaga middle"),
    ("Pawaga", "bottom", "Pawaga bottom"),
    ("BTX", "top", "BTX top"),
    ("BTX", "middle", "BTX middle"),
    ("BTX", "bottom", "BTX bottom"),
)
VALUE_COLUMN = "illumination_total_simulated"


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[1]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def write_manifest(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["figure_type", "file", "label", "rows", "columns", "metric"])
        writer.writeheader()
        writer.writerows(rows)


def green_red_cmap() -> LinearSegmentedColormap:
    return LinearSegmentedColormap.from_list("red_yellow_green", ("#c9302c", "#f2df5a", "#238b45"))


def rows_by_date(rows: list[dict[str, str]]) -> dict[str, list[dict[str, str]]]:
    by_date: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        by_date[row["date"]].append(row)
    return dict(by_date)


def parbar_matrix(rows: list[dict[str, str]]) -> np.ndarray:
    matrix = np.full((len(PANEL_ORDER), 100), np.nan, dtype=float)
    by_panel: dict[tuple[str, str], dict[int, float]] = defaultdict(dict)
    for row in rows:
        key = (row["cultivar"], row["sensor_bar_level"])
        by_panel[key][int(row["probe_number"])] = float(row[VALUE_COLUMN])

    for panel_index, (cultivar, level, _label) in enumerate(PANEL_ORDER):
        probes = by_panel[(cultivar, level)]
        if len(probes) != 100:
            raise ValueError(f"expected 100 probes for {cultivar} {level}, found {len(probes)}")
        for probe_number in range(1, 101):
            matrix[panel_index, probe_number - 1] = probes[probe_number]
    return matrix


def render_heatmap(output_path: Path, date: str, matrix: np.ndarray, cmap: LinearSegmentedColormap, scale_min: float, scale_max: float) -> None:
    fig, ax = plt.subplots(figsize=(16.5, 5.2), dpi=160)
    image = ax.imshow(matrix, cmap=cmap, vmin=scale_min, vmax=scale_max, aspect="auto", origin="upper")

    ax.set_yticks(range(len(PANEL_ORDER)))
    ax.set_yticklabels([label for _cultivar, _level, label in PANEL_ORDER])
    ax.set_xticks([0, 9, 19, 29, 39, 49, 59, 69, 79, 89, 99])
    ax.set_xticklabels(["1", "10", "20", "30", "40", "50", "60", "70", "80", "90", "100"])
    ax.set_xlabel("Probe number across each sensor bar")
    ax.set_ylabel("Sensor bar")
    ax.set_title(
        f"Date-Height PARBAR Light Probe Heatmap - {date}\n"
        "Middle bars are at 2/3 average represented clump height",
        fontsize=12,
        pad=14,
    )
    ax.set_xticks(np.arange(-0.5, 100, 10), minor=True)
    ax.set_yticks(np.arange(-0.5, len(PANEL_ORDER), 1), minor=True)
    ax.grid(which="minor", color="#262626", linewidth=0.75)
    ax.tick_params(which="minor", bottom=False, left=False)

    colorbar = fig.colorbar(image, ax=ax, fraction=0.022, pad=0.02)
    colorbar.set_label("Simulated light at probe (relative)")
    fig.text(
        0.5,
        0.025,
        "Green = higher simulated probe light; red = lower. Color scale is shared across all date-height PARBAR images.",
        ha="center",
        fontsize=9,
    )
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path)
    plt.close(fig)


def render_all(args: argparse.Namespace) -> tuple[int, Path]:
    handoff_dir = args.handoff_dir.resolve()
    output_dir = args.output_dir.resolve()
    parbar_output_dir = output_dir / "4x10_parbar_probe_heatmaps"
    rows = read_csv(handoff_dir / "all_parbar_sensors_long.csv")
    values = [float(row[VALUE_COLUMN]) for row in rows]
    if not values:
        raise ValueError("all_parbar_sensors_long.csv has no rows")

    cmap = green_red_cmap()
    manifest: list[dict[str, object]] = []
    grouped = rows_by_date(rows)
    for date in [date for date in DATE_ORDER if date in grouped]:
        output_path = parbar_output_dir / f"parbar_date_{date}.png"
        render_heatmap(output_path, date, parbar_matrix(grouped[date]), cmap, min(values), max(values))
        manifest.append(
            {
                "figure_type": "4x10_parbar_probe_heatmap",
                "file": str(output_path.relative_to(output_dir)),
                "label": f"Date {date}",
                "rows": 6,
                "columns": 100,
                "metric": "simulated light at PARBAR probe",
            }
        )
    write_manifest(output_dir / "visualization_manifest.csv", manifest)
    return len(manifest), output_dir


def build_parser() -> argparse.ArgumentParser:
    repo_root = repo_root_from_script()
    handoff_dir = repo_root / "out" / "handoff" / "date_height_parbar_illumination"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--handoff-dir", default=handoff_dir, type=Path)
    parser.add_argument("--output-dir", default=handoff_dir / "labeled_pngs", type=Path)
    return parser


def main() -> None:
    count, output_dir = render_all(build_parser().parse_args())
    print(f"output_dir={output_dir}")
    print(f"parbar_pngs={count}")


if __name__ == "__main__":
    main()
