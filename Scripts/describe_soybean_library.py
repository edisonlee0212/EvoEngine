#!/usr/bin/env python3
"""Measure exported OBJ geometry without interpreting alpha-masked mesh area as biological area."""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


def describe_obj(path):
    vertices, groups = [], {}
    name = "unnamed"
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            if line.startswith("v "):
                vertices.append([float(value) for value in line.split()[1:4]])
            elif line.startswith("o "):
                name = line[2:].strip()
            elif line.startswith("f "):
                indices = [int(value.split("/")[0]) - 1 for value in line.split()[1:]]
                groups.setdefault(name, []).extend((indices[0], indices[index], indices[index + 1])
                                                  for index in range(1, len(indices) - 1))
    vertices = np.asarray(vertices)
    result = {"obj_vertex_count": len(vertices), "obj_height_m": float(np.ptp(vertices[:, 2])),
              "obj_width_x_m": float(np.ptp(vertices[:, 0])), "obj_width_y_m": float(np.ptp(vertices[:, 1]))}
    for name, indices in groups.items():
        triangles = vertices[np.asarray(indices)]
        vectors = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        twice_area = np.linalg.norm(vectors, axis=1)
        result[f"{name}_triangle_count"] = len(indices)
        result[f"{name}_mesh_area_m2"] = float(twice_area.sum() / 2)
        if name == "leaf":
            nonzero = twice_area > 0
            inclination = np.degrees(np.arccos(np.clip(np.abs(vectors[nonzero, 2]) / twice_area[nonzero], 0, 1)))
            result["leaf_area_weighted_inclination_deg"] = float(np.average(inclination, weights=twice_area[nonzero]))
    return result


def describe_library(root, output):
    records = []
    for kind in ["Broad", "Narrow"]:
        folder = Path(root) / kind
        manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
        for entry in manifest["models"]:
            records.append({"asset_name": kind.lower(), "generation_seed": entry["seed"], "source_obj": entry["obj"],
                            **(entry.get("geometry") or {}), **describe_obj(folder / entry["obj"])})
            if len(records) % 128 == 0:
                print(f"Described {len(records)} plants", flush=True)
    frame = pd.DataFrame(records)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    frame.to_csv(output / "plant_geometry.csv", index=False)
    geometry_columns = [column for column in frame.select_dtypes("number") if column not in {"generation_seed", "seed"}]
    summary = frame.groupby("asset_name")[geometry_columns].agg(["mean", "std", "min", "max"])
    summary.columns = [f"{metric}_{statistic}" for metric, statistic in summary.columns]
    summary.reset_index().to_csv(output / "geometry_summary.csv", index=False)
    paired = frame[frame.asset_name == "broad"].merge(frame[frame.asset_name == "narrow"], on="generation_seed", suffixes=("_broad", "_narrow"), validate="one_to_one")
    for metric in ["leaf_count", "height_m", "leaf_mesh_area_m2", "leaf_triangle_count", "leaf_area_weighted_inclination_deg", "shoot_mesh_area_m2", "petiole_mesh_area_m2"]:
        paired[f"{metric}_difference"] = paired[f"{metric}_broad"] - paired[f"{metric}_narrow"]
    paired.to_csv(output / "paired_geometry.csv", index=False)
    return frame


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    frame = describe_library(args.library, args.output)
    print(frame.head(4).to_string(index=False))
    print(frame.groupby("asset_name").size().to_string())
