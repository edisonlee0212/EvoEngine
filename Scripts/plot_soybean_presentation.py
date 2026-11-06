#!/usr/bin/env python3
"""Render slide-ready figures from existing analysis files; never run the engine."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager, pyplot as plt
import numpy as np
import pandas as pd

METRICS = {
    "leaf_mean_irradiance": ("Leaf mean irradiance", "Relative irradiance"),
    "leaf_incident_per_plant": ("Leaf incident light per plant", "Relative light × m² / plant"),
    "whole_incident_per_plant": ("Total incident light per plant", "Relative light × m² / plant"),
    "leaf_incident_per_ground_area": ("Leaf incident light per ground area", "Relative light / ground area"),
}
TYPES = ("broad", "narrow")


def spacing_grids(frame, metric):
    """Reject missing/pooled cases instead of silently averaging or interpolating."""
    fixed = ["rows", "columns", "roi", "samples", "bounces", "repeats"]
    if any(frame[c].nunique(dropna=False) != 1 for c in fixed):
        raise ValueError("Expected one field/sampling configuration and repeat count")
    if set(frame.asset_name) != set(TYPES):
        raise ValueError("Expected broad and narrow plants")
    rows = np.sort(frame.row_spacing_inches.unique())
    columns = np.sort(frame.column_spacing_inches.unique())
    grids = {}
    for name in TYPES:
        selected = frame[frame.asset_name == name]
        grid = selected.pivot(index="row_spacing_inches", columns="column_spacing_inches", values=metric)
        grid = grid.reindex(index=rows, columns=columns)
        if not np.isfinite(grid.to_numpy()).all():
            raise ValueError(f"Missing or non-finite {name} spacing cells")
        grids[name] = grid
    return grids


def configure_font(font_file=None):
    # Register installed font files explicitly: older Matplotlib caches may omit them.
    if font_file:
        for font in Path(font_file).parent.glob("LibertinusSerif-*.ttf"):
            font_manager.fontManager.addfont(str(font))
        font_manager.fontManager.addfont(str(font_file))
    resolved = font_manager.findfont(font_manager.FontProperties(family="Libertinus Serif"), fallback_to_default=False)
    plt.rcParams.update({"font.family": "Libertinus Serif", "font.size": 14,
                         "text.color": "#233D34", "axes.labelcolor": "#233D34",
                         "xtick.color": "#233D34", "ytick.color": "#233D34",
                         "figure.facecolor": "#F8F8F3", "axes.facecolor": "#F8F8F3"})
    return resolved


def heatmap(frame, metric, destination):
    grids = spacing_grids(frame, metric)
    low = min(g.to_numpy().min() for g in grids.values())
    high = max(g.to_numpy().max() for g in grids.values())
    fig, axes = plt.subplots(1, 2, figsize=(11.52, 4.8))
    fig.subplots_adjust(left=.067, right=.865, bottom=.29, top=.9, wspace=.22)
    best = {}
    for ax, (name, grid) in zip(axes, grids.items()):
        data = grid.to_numpy()
        image = ax.imshow(data, origin="lower", aspect="auto", vmin=low, vmax=high, cmap="viridis")
        ax.set_xticks(range(len(grid.columns)), [f"{v:g}" for v in grid.columns])
        ax.set_yticks(range(len(grid.index)), [f"{v:g}" for v in grid.index])
        ax.tick_params(labelsize=12)
        ax.set_xlabel("Seeding spacing (in)", labelpad=6)
        ax.set_ylabel("Row spacing (in)", labelpad=5)
        ax.set_title(f"{name.capitalize()} leaves", fontsize=18, fontweight="bold", pad=10)
        r, c = np.unravel_index(np.argmax(data), data.shape)
        ax.scatter(c, r, marker="*", s=180, c="white", edgecolors="#172D25", linewidths=1.1)
        best[name] = {"row_spacing_inches": float(grid.index[r]), "seeding_spacing_inches": float(grid.columns[c]),
                      "value": float(data[r, c]), "tied_max_cells": int(np.sum(data == data[r, c]))}
        ax.text(.5, -.27, f"★ Best tested: row {grid.index[r]:g} in, seeding {grid.columns[c]:g} in\n"
                f"Mean = {data[r, c]:.4f}", transform=ax.transAxes, ha="center", va="top", fontsize=12)
    colorbar = fig.colorbar(image, cax=fig.add_axes([.895, .29, .019, .61]))
    colorbar.set_label(METRICS[metric][1], fontsize=13, labelpad=10)
    colorbar.ax.tick_params(labelsize=12)
    fig.savefig(destination, dpi=200)
    plt.close(fig)
    return {"metric": metric, "title": METRICS[metric][0], "image": destination.name,
            "shared_color_limits": [float(low), float(high)], "best_tested": best,
            "row_spacing_inches": grids["broad"].index.tolist(),
            "seeding_spacing_inches": grids["broad"].columns.tolist()}


def isolated_comparison(facts, destination):
    keys = list(METRICS)[:3] + ["leaf_area_per_plant_m2"]
    titles = ["Leaf mean irradiance", "Leaf incident light / plant",
              "Total incident light / plant", "Leaf mesh area / plant (m²)"]
    fig, axes = plt.subplots(2, 2, figsize=(8.72, 5.2))
    fig.subplots_adjust(left=.07, right=.99, top=.92, bottom=.17, wspace=.28, hspace=1.2)
    for ax, key, title in zip(axes.flat, keys, titles):
        values = facts["metrics"][key]
        means = [values[f"{name}_mean"] for name in TYPES]
        ax.bar([0, 1], means, width=.6, color=["#28664F", "#AD6B37"])
        ax.set_xticks([0, 1], ["Broad", "Narrow"], fontsize=12)
        ax.set_ylim(0, max(means) * 1.28)
        ax.set_title(title, fontsize=14, fontweight="bold", pad=9)
        ax.tick_params(axis="y", labelsize=11)
        ax.spines[["top", "right"]].set_visible(False)
        for x, value in enumerate(means):
            ax.text(x, value + max(means) * .04, f"{value:.3f}", ha="center", fontsize=12)
        lo, hi = values["cluster_bootstrap_95_interval"]
        ax.text(.5, -.38, f"Broad − narrow: {values['broad_minus_narrow_mean']:.3f}\n"
                f"95% CI: [{lo:.3f}, {hi:.3f}]", transform=ax.transAxes, ha="center", va="top", fontsize=11.5)
    fig.savefig(destination, dpi=200)
    plt.close(fig)


def render(study, font_file=None):
    study = Path(study)
    source = study / "analysis/03_field_search/ranked_combinations.csv"
    isolated = study / "analysis/01_single_plant/facts.json"
    destination = study / "analysis/presentation_figures"
    destination.mkdir(parents=True, exist_ok=True)
    font = configure_font(font_file)
    frame = pd.read_csv(source)
    figures = [heatmap(frame, metric, destination / f"{metric}.png") for metric in METRICS]
    isolated_comparison(json.loads(isolated.read_text(encoding="utf-8")), destination / "isolated_comparison.png")
    manifest = {"generator": "Scripts/plot_soybean_presentation.py", "font": font,
                "sources_sha256": {str(p.relative_to(study)): hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in [source, isolated]},
                "aggregation": "Existing cell means over five field subsets; no interpolation or additional pooling.",
                "units": "Relative linear-RGB luminance scores, not W, PAR, absorption, or net interception fractions.",
                "fixed_settings": frame[["rows", "columns", "roi", "samples", "bounces", "repeats"]].iloc[0].to_dict(),
                "field_figures": figures, "isolated_figure": "isolated_comparison.png"}
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", type=Path, required=True)
    parser.add_argument("--font-file", type=Path, default=Path("C:/Windows/Fonts/LibertinusSerif-Regular.ttf"))
    args = parser.parse_args()
    result = render(args.study, args.font_file)
    print(f"Rendered {len(result['field_figures']) + 1} figures; no engine or measurement run.")
