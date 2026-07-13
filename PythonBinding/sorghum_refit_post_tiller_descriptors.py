#!/usr/bin/env python3
"""Refit descriptor length ranges after true tillers change whole-plant height."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import yaml


LENGTH_FIELDS = (
    "internode_length",
    "leaf_blade_length",
    "leaf_sheath_length",
    "leaf_neck_length",
)


def population_std(values: list[float]) -> float:
    mean = sum(values) / len(values)
    return math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    args = parser.parse_args()

    with args.manifest.open(newline="", encoding="utf-8-sig") as stream:
        manifest = list(csv.DictReader(stream))
    with args.summary.open(newline="", encoding="utf-8-sig") as stream:
        summary = list(csv.DictReader(stream))

    grouped: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    for row in manifest:
        grouped[(row["date"], row["cultivar"])].append(row)

    for row in summary:
        key = (row["date"], row["cultivar"])
        heights = [float(item["final_height_m"]) for item in grouped[key]]
        observed_mean = sum(heights) / len(heights)
        observed_std = population_std(heights)
        target_mean = float(row["target_height_m"])
        target_std = float(row["target_height_std_m"])
        mean_ratio = target_mean / max(observed_mean, 1.0e-6)
        std_ratio = target_std / max(observed_std, 1.0e-6)

        descriptor_path = args.assets / row["descriptor_asset_path"]
        descriptor = yaml.safe_load(descriptor_path.read_text(encoding="utf-8"))
        for field in LENGTH_FIELDS:
            distribution = descriptor[field]
            for bound in ("min_value", "max_value"):
                distribution["mean"][bound] *= mean_ratio
                distribution["deviation"][bound] *= std_ratio
        descriptor_path.write_text(
            yaml.safe_dump(descriptor, sort_keys=False, width=120), encoding="utf-8"
        )
        row["length_mean_scale"] = str(float(row["length_mean_scale"]) * mean_ratio)
        row["length_deviation_scale"] = str(float(row["length_deviation_scale"]) * std_ratio)
        for field in (
            "validation_leaf_mean",
            "validation_leaf_std",
            "validation_height_mean_m",
            "validation_height_std_m",
            "validation_tiller_leaf_ratio_mean",
            "validation_tiller_leaf_ratio_min",
            "validation_tiller_leaf_ratio_max",
            "validation_tiller_height_ratio_mean",
            "validation_tiller_height_ratio_min",
            "validation_tiller_height_ratio_max",
            "success",
            "failed_metrics",
        ):
            row[field] = ""
        row["validation_basis"] = "invalidated_by_post_tiller_refit; see post_tiller_scene_validation.csv"
        print(
            f"{key[0]} {key[1]}: mean x{mean_ratio:.5f}, std x{std_ratio:.5f}"
        )

    with args.summary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=summary[0].keys())
        writer.writeheader()
        writer.writerows(summary)


if __name__ == "__main__":
    main()
