#!/usr/bin/env python3
"""Generate persisted calibrated 4x10 and 10x10 sorghum field scenes."""

from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import statistics
import subprocess
import sys
import tempfile
import uuid
from pathlib import Path

from sorghum_asset_layout import (
    DATE_ORDER,
    GENERATED_DESCRIPTOR_ROOT,
    GENERATED_REPORT_ROOT,
    GENERATED_SCENE_ROOT,
    MANUAL_4X10_SCENE,
    MANUAL_10X10_SCENE,
    descriptor_path,
    four_by_ten_scene,
)


CULTIVARS = ("BTX", "Pawaga")
LEAF_THICKNESS_M_BY_DATE = {
    "2021-07-01": 0.00030,
    "2021-07-14": 0.00035,
    "2021-08-18": 0.00045,
    "2021-08-30": 0.00050,
    "2021-09-02": 0.00045,
}
TEN_BY_TEN_DESCRIPTOR_DATE = "2021-08-30"
TEN_BY_TEN_TARGET_SCENE = (GENERATED_SCENE_ROOT / "Sorghum_10x10_Mature.evescene").as_posix()
TEN_BY_TEN_SOURCE_SCENE = MANUAL_10X10_SCENE.as_posix()
FOUR_BY_TEN_REFERENCE_SCENE = MANUAL_4X10_SCENE.as_posix()
WORKER_METADATA_COLUMNS = [
    "date",
    "plant_name",
    "source_cluster_offset_x_m",
    "source_cluster_offset_z_m",
    "source_cluster_offset_radius_m",
    "cluster_offset_x_m",
    "cluster_offset_z_m",
    "cluster_offset_radius_m",
    "plant_height_m",
    "leaf_count",
    "main_culm_leaf_count",
    "tiller_leaf_count",
    "primary_tiller_count",
    "tiller_leaf_ratio_mean",
    "tiller_leaf_ratio_min",
    "tiller_leaf_ratio_max",
    "tiller_height_ratio_mean",
    "tiller_height_ratio_min",
    "tiller_height_ratio_max",
    "middle_parbar_top_elevation_m",
]
POST_TILLER_VALIDATION_COLUMNS = [
    "date",
    "cultivar",
    "sample_count",
    "target_leaf_mean",
    "observed_main_leaf_mean",
    "observed_main_leaf_std",
    "target_height_mean_m",
    "target_height_std_m",
    "observed_height_mean_m",
    "observed_height_std_m",
    "height_mean_relative_error",
    "primary_tiller_min",
    "primary_tiller_max",
    "tiller_leaf_ratio_mean",
    "tiller_leaf_ratio_min",
    "tiller_leaf_ratio_max",
    "tiller_height_ratio_mean",
    "tiller_height_ratio_min",
    "tiller_height_ratio_max",
]


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


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_post_tiller_validation(manifest_path: Path, summary_path: Path, output_path: Path) -> None:
    manifest = read_csv(manifest_path)
    targets = {(row["date"], row["cultivar"]): row for row in read_csv(summary_path)}
    grouped: dict[tuple[str, str], list[dict[str, str]]] = {}
    for row in manifest:
        grouped.setdefault((row["date"], row["cultivar"]), []).append(row)
    validation: list[dict[str, object]] = []
    for key, group in sorted(grouped.items()):
        target = targets[key]
        heights = [float(row["final_height_m"]) for row in group]
        leaves = [float(row["main_culm_leaf_count"]) for row in group]
        observed_height = statistics.fmean(heights)
        target_height = float(target["target_height_m"])
        leaf_ratios = [float(row["tiller_leaf_ratio_mean"]) for row in group]
        height_ratios = [float(row["tiller_height_ratio_mean"]) for row in group]
        validation.append(
            {
                "date": key[0],
                "cultivar": key[1],
                "sample_count": len(group),
                "target_leaf_mean": float(target["leaf_modules_mean"]),
                "observed_main_leaf_mean": statistics.fmean(leaves),
                "observed_main_leaf_std": statistics.pstdev(leaves),
                "target_height_mean_m": target_height,
                "target_height_std_m": float(target["target_height_std_m"]),
                "observed_height_mean_m": observed_height,
                "observed_height_std_m": statistics.pstdev(heights),
                "height_mean_relative_error": abs(observed_height - target_height) / target_height,
                "primary_tiller_min": min(int(float(row["primary_tiller_count"])) for row in group),
                "primary_tiller_max": max(int(float(row["primary_tiller_count"])) for row in group),
                "tiller_leaf_ratio_mean": statistics.fmean(leaf_ratios),
                "tiller_leaf_ratio_min": min(float(row["tiller_leaf_ratio_min"]) for row in group),
                "tiller_leaf_ratio_max": max(float(row["tiller_leaf_ratio_max"]) for row in group),
                "tiller_height_ratio_mean": statistics.fmean(height_ratios),
                "tiller_height_ratio_min": min(float(row["tiller_height_ratio_min"]) for row in group),
                "tiller_height_ratio_max": max(float(row["tiller_height_ratio_max"]) for row in group),
            }
        )
    write_csv(output_path, POST_TILLER_VALIDATION_COLUMNS, validation)  # type: ignore[arg-type]


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


def asset_path(path: Path) -> str:
    return path.as_posix()


def calibrated_descriptor(root: Path, date: str, cultivar: str) -> Path:
    return descriptor_path(root, date, cultivar)


def target_4x10_scene(date: str) -> Path:
    return four_by_ten_scene(date)


def rows_for_dates(rows: list[dict[str, str]], dates: list[str]) -> list[dict[str, str]]:
    selected = [row for row in rows if row["date"] in dates]
    missing = [date for date in dates if not any(row["date"] == date for row in selected)]
    if missing:
        raise ValueError(f"manifest is missing date(s): {', '.join(missing)}")
    return selected


def source_scene_for_date(args: argparse.Namespace) -> str:
    return args.reference_4x10_scene.as_posix()


def leaf_thickness_for_date(rows: list[dict[str, str]], date: str) -> float:
    return LEAF_THICKNESS_M_BY_DATE[date]


def validate_descriptors(args: argparse.Namespace, dates: list[str]) -> None:
    assets_root = args.project_root / "Assets"
    missing = [
        calibrated_descriptor(args.calibrated_descriptor_root, date, cultivar)
        for date in dates
        for cultivar in CULTIVARS
        if not (assets_root / calibrated_descriptor(args.calibrated_descriptor_root, date, cultivar)).exists()
    ]
    if missing:
        raise FileNotFoundError("missing calibrated descriptor(s): " + ", ".join(path.as_posix() for path in missing))


def start_project(evo: object, project: Path, runtime_package_dir: Path, scene_asset_path: str, max_wait_frames: int) -> None:
    ok = evo.RunLSystemSorghumProject(project.resolve(), runtime_package_dir.resolve(), Path(scene_asset_path))
    if not ok:
        raise RuntimeError(f"failed to start source scene: {scene_asset_path}")
    if not evo.WaitForProjectIdle(max_wait_frames):
        raise RuntimeError(f"project did not become idle after loading {scene_asset_path}")


def save_active_scene(evo: object, target: Path, max_wait_frames: int) -> None:
    if not evo.EnsureIlluminationSoilContext():
        raise RuntimeError(f"failed to ensure PBR soil context for {target}")
    if not evo.ValidateIlluminationContext():
        raise RuntimeError(f"illumination context validation failed for {target}")
    if not evo.WaitForProjectIdle(max_wait_frames):
        raise RuntimeError(f"project did not become idle before saving {target}")
    if not evo.SaveActiveSceneAsProjectAsset(target):
        raise RuntimeError(f"failed to save scene {target}")


def staging_scene_target(target: Path) -> Path:
    return target.with_name(f"_Staging_{target.stem}_{uuid.uuid4().hex}{target.suffix}")


def scene_files(assets_root: Path, scene: Path) -> tuple[Path, Path]:
    return assets_root / scene, assets_root / f"{scene.as_posix()}.evefilemeta"


def discard_staged_scene(project_root: Path, staged: Path) -> None:
    assets_root = project_root / "Assets"
    for path in scene_files(assets_root, staged):
        resolved = path.resolve()
        try:
            resolved.relative_to(assets_root.resolve())
        except ValueError as exc:
            raise RuntimeError(f"refusing to remove staged output outside project assets: {resolved}") from exc
        if path.exists():
            path.unlink()


def promote_staged_scene(project_root: Path, staged: Path, target: Path) -> None:
    assets_root = project_root / "Assets"
    staged_files = scene_files(assets_root, staged)
    target_files = scene_files(assets_root, target)
    if not staged_files[0].exists():
        raise RuntimeError(f"staged scene was not created: {staged}")
    previous = {path: path.read_bytes() if path.exists() else None for path in target_files}
    try:
        os.replace(staged_files[0], target_files[0])
        if staged_files[1].exists():
            metadata = staged_files[1].read_text(encoding="utf-8").replace(staged.stem, target.stem)
            staged_files[1].write_text(metadata, encoding="utf-8")
            os.replace(staged_files[1], target_files[1])
        elif target_files[1].exists():
            target_files[1].unlink()
    except Exception:
        for path, content in previous.items():
            path.unlink(missing_ok=True)
            if content is not None:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(content)
        raise
    finally:
        discard_staged_scene(project_root, staged)


def scene_metadata_rows(
    date: str, source_records: list[object], generated_records: list[object]
) -> list[dict[str, object]]:
    source = {record.name: record for record in source_records}
    generated = {record.name: record for record in generated_records}
    if len(source) != 40 or len(generated) != 40 or source.keys() != generated.keys():
        raise RuntimeError(f"{date}: expected matching 40-rooted-plant source and generated metadata")

    rows: list[dict[str, object]] = []
    for name, record in generated.items():
        if not record.has_geometry:
            raise RuntimeError(f"{date}: generated plant has no measurable geometry: {name}")
        base_name = name.split("_cluster_", 1)[0]
        source_position = source[name].local_position
        source_anchor = source[base_name].local_position
        position = record.local_position
        anchor = generated[base_name].local_position
        source_x = float(source_position.x - source_anchor.x)
        source_z = float(source_position.z - source_anchor.z)
        offset_x = float(position.x - anchor.x)
        offset_z = float(position.z - anchor.z)
        tiller_axes = [axis for axis in record.axes if int(axis.axis_id) != 0]
        if not tiller_axes:
            raise RuntimeError(f"{date}: generated plant has no primary tiller axes: {name}")
        leaf_ratios = [float(axis.leaf_ratio_to_main) for axis in tiller_axes]
        height_ratios = [float(axis.height_ratio_to_main) for axis in tiller_axes]
        rows.append(
            {
                "date": date,
                "plant_name": name,
                "source_cluster_offset_x_m": source_x,
                "source_cluster_offset_z_m": source_z,
                "source_cluster_offset_radius_m": math.hypot(source_x, source_z),
                "cluster_offset_x_m": offset_x,
                "cluster_offset_z_m": offset_z,
                "cluster_offset_radius_m": math.hypot(offset_x, offset_z),
                "plant_height_m": float(record.plant_height_m),
                "leaf_count": int(record.leaf_count),
                "main_culm_leaf_count": int(record.main_culm_leaf_count),
                "tiller_leaf_count": int(record.tiller_leaf_count),
                "primary_tiller_count": int(record.primary_tiller_count),
                "tiller_leaf_ratio_mean": statistics.fmean(leaf_ratios),
                "tiller_leaf_ratio_min": min(leaf_ratios),
                "tiller_leaf_ratio_max": max(leaf_ratios),
                "tiller_height_ratio_mean": statistics.fmean(height_ratios),
                "tiller_height_ratio_min": min(height_ratios),
                "tiller_height_ratio_max": max(height_ratios),
                "middle_parbar_top_elevation_m": float(record.middle_parbar_top_elevation_m),
            }
        )
    return rows


def generate_4x10_scene(
    evo: object, args: argparse.Namespace, rows: list[dict[str, str]], date: str
) -> tuple[Path, list[dict[str, object]]]:
    project = args.project_root / "test_lsystem_sorghum.eveproj"
    project_bytes = project.read_bytes()
    side_effects_before = collect_default_scene_side_effects(project)
    target = target_4x10_scene(date)
    staged_target = staging_scene_target(target)
    source_scene = source_scene_for_date(args)
    source_path = args.project_root / "Assets" / source_scene
    source_bytes = source_path.read_bytes()
    metadata_rows: list[dict[str, object]] = []
    try:
        try:
            start_project(evo, project, args.runtime_package_dir, source_scene, args.max_wait_frames)
            instantiated = int(evo.InstantiateSorghumLsPlantsFromPlantingMarkers())
            if instantiated != 40:
                raise RuntimeError(f"{date}: expected 40 instantiated planting markers, got {instantiated}")
            source_records = [
                record
                for record in evo.GetSorghumLsPlantSceneMetadata(False)
                if "_cluster_" not in record.name
            ]
            if len(source_records) != 40:
                raise RuntimeError(f"{date}: expected 40 rooted source plants, got {len(source_records)}")
            btx_descriptor = calibrated_descriptor(args.calibrated_descriptor_root, date, "BTX")
            pawaga_descriptor = calibrated_descriptor(args.calibrated_descriptor_root, date, "Pawaga")
            assigned = int(evo.SetSorghumLsCultivarDescriptors(btx_descriptor, pawaga_descriptor, False, -1))
            if assigned != 40:
                raise RuntimeError(f"{date}: expected 40 descriptor-assigned plants, got {assigned}")
            thickness_count = int(evo.SetSorghumLsLeafThickness(leaf_thickness_for_date(rows, date), False))
            if thickness_count != 40:
                raise RuntimeError(f"{date}: expected 40 leaf-thickness updates, got {thickness_count}")
            width_count = int(evo.SetSorghumLsLeafWidthScale(1.0, False))
            if width_count != 40:
                raise RuntimeError(f"{date}: expected 40 leaf-width updates, got {width_count}")
            grown = int(evo.GrowSorghumLsPlantsToAdulthood(args.geometry_seed))
            if grown != 40:
                raise RuntimeError(f"{date}: expected 40 grown rooted plants, got {grown}")
            if not evo.WaitForProjectIdle(args.max_wait_frames):
                raise RuntimeError(f"{date}: project did not become idle after growth")
            moved = int(evo.MoveParbarMiddlePanelsToPlantHeightFraction(args.middle_panel_height_fraction))
            if moved != 2:
                raise RuntimeError(f"{date}: expected 2 moved middle PARBAR panels, got {moved}")
            evo.LoopFrames(args.after_panel_move_frames)
            if not evo.WaitForProjectIdle(args.max_wait_frames):
                raise RuntimeError(f"{date}: project did not become idle before measuring generated plants")
            generated_records = list(evo.GetSorghumLsPlantSceneMetadata(True))
            metadata_rows = scene_metadata_rows(date, source_records, generated_records)
            save_active_scene(evo, staged_target, args.max_wait_frames)
        finally:
            project.write_bytes(project_bytes)
            cleanup_new_default_scene_side_effects(project, side_effects_before)
            if source_path.read_bytes() != source_bytes:
                source_path.write_bytes(source_bytes)
                raise RuntimeError(
                    f"reference scene was modified during generation and has been restored: {source_scene}"
                )
    except Exception:
        discard_staged_scene(args.project_root, staged_target)
        raise
    promote_staged_scene(args.project_root, staged_target, target)
    print(f"4x10 {date}: {target.as_posix()}")
    return target, metadata_rows


def generate_10x10_scene(evo: object, args: argparse.Namespace) -> Path:
    project = args.project_root / "test_lsystem_sorghum_10x10_overlap.eveproj"
    project_bytes = project.read_bytes()
    side_effects_before = collect_default_scene_side_effects(project)
    target = Path(TEN_BY_TEN_TARGET_SCENE)
    staged_target = staging_scene_target(target)
    try:
        try:
            start_project(evo, project, args.runtime_package_dir, TEN_BY_TEN_SOURCE_SCENE, args.max_wait_frames)
            instantiated = int(evo.InstantiateSorghumLsPlantsFromPlantingMarkers())
            if instantiated != 200:
                raise RuntimeError(f"10x10: expected 200 instantiated planting markers, got {instantiated}")
            btx_descriptor = calibrated_descriptor(
                args.calibrated_descriptor_root, args.ten_by_ten_descriptor_date, "BTX"
            )
            pawaga_descriptor = calibrated_descriptor(
                args.calibrated_descriptor_root, args.ten_by_ten_descriptor_date, "Pawaga"
            )
            assigned = int(evo.SetSorghumLsCultivarDescriptors(btx_descriptor, pawaga_descriptor, False, -1))
            if assigned != 200:
                raise RuntimeError(f"10x10: expected 200 descriptor-assigned plants, got {assigned}")
            thickness_count = int(evo.SetSorghumLsLeafThickness(
                leaf_thickness_for_date([], args.ten_by_ten_descriptor_date), False
            ))
            if thickness_count != 200:
                raise RuntimeError(f"10x10: expected 200 leaf-thickness updates, got {thickness_count}")
            grown = int(evo.GrowSorghumLsPlantsToAdulthood(args.ten_by_ten_geometry_seed))
            if grown != 200:
                raise RuntimeError(f"10x10: expected 200 grown plants, got {grown}")
            save_active_scene(evo, staged_target, args.max_wait_frames)
        finally:
            project.write_bytes(project_bytes)
            cleanup_new_default_scene_side_effects(project, side_effects_before)
    except Exception:
        discard_staged_scene(args.project_root, staged_target)
        raise
    promote_staged_scene(args.project_root, staged_target, target)
    print(f"10x10 {args.ten_by_ten_descriptor_date}: {target.as_posix()}")
    return target


def source_value(row: dict[str, str], source_column: str, effective_column: str) -> float:
    return float(row.get(source_column) or row[effective_column])


def descriptor_length_scales(args: argparse.Namespace, dates: list[str]) -> dict[tuple[str, str], float]:
    path = args.project_root / "Assets" / GENERATED_REPORT_ROOT / "calibration_summary.csv"
    scales = {
        (row["date"], row["cultivar"]): float(row["length_mean_scale"])
        for row in read_csv(path)
        if row["date"] in dates
    }
    missing = [(date, cultivar) for date in dates for cultivar in CULTIVARS if (date, cultivar) not in scales]
    if missing:
        raise ValueError(f"calibration summary is missing descriptor scale(s): {missing}")
    return scales


def update_manifest(
    path: Path,
    rows: list[dict[str, str]],
    dates: list[str],
    descriptor_root: Path,
    generated_metadata: list[dict[str, str]],
    descriptor_scales: dict[tuple[str, str], float],
) -> None:
    fieldnames = list(rows[0])
    metadata_columns = [
        "source_leaf_thickness_m",
        "leaf_thickness_scale",
        "source_leaf_width_scale",
        "leaf_width_scale",
        "source_cluster_offset_x_m",
        "source_cluster_offset_z_m",
        "source_cluster_offset_radius_m",
        "cluster_radius_scale",
        "cluster_outward_lean_degrees",
        "source_pre_fit_height_m",
        "source_final_height_m",
        "source_height_error_m",
        "source_optimized_descriptor_scale",
        "source_per_plant_scale",
        "source_leaf_count",
        "source_clump_mean_height_m",
        "source_middle_parbar_top_elevation_m",
        "main_culm_leaf_count",
        "tiller_leaf_count",
        "primary_tiller_count",
        "tiller_leaf_ratio_mean",
        "tiller_leaf_ratio_min",
        "tiller_leaf_ratio_max",
        "tiller_height_ratio_mean",
        "tiller_height_ratio_min",
        "tiller_height_ratio_max",
    ]
    fieldnames.extend(column for column in metadata_columns if column not in fieldnames)
    measured = {(row["date"], row["plant_name"]): row for row in generated_metadata}
    rows[:] = [
        row
        for row in rows
        if row["date"] not in dates or int(float(row["cluster_index"])) == 0
    ]
    expected_metadata_count = 40 * len(dates)
    if len(measured) != expected_metadata_count:
        raise ValueError(f"expected {expected_metadata_count} generated metadata rows, got {len(measured)}")
    clump_heights: dict[tuple[str, str, str], list[float]] = {}
    for row in rows:
        source_leaf_thickness = source_value(row, "source_leaf_thickness_m", "leaf_thickness_m")
        row["source_leaf_thickness_m"] = source_leaf_thickness
        row["source_leaf_width_scale"] = float(row.get("source_leaf_width_scale") or 1.0)
        row["source_cluster_offset_x_m"] = source_value(
            row, "source_cluster_offset_x_m", "cluster_offset_x_m"
        )
        row["source_cluster_offset_z_m"] = source_value(
            row, "source_cluster_offset_z_m", "cluster_offset_z_m"
        )
        row["source_cluster_offset_radius_m"] = source_value(
            row, "source_cluster_offset_radius_m", "cluster_offset_radius_m"
        )
        for source_column, effective_column in (
            ("source_pre_fit_height_m", "pre_fit_height_m"),
            ("source_final_height_m", "final_height_m"),
            ("source_height_error_m", "height_error_m"),
            ("source_optimized_descriptor_scale", "optimized_descriptor_scale"),
            ("source_per_plant_scale", "per_plant_scale"),
            ("source_leaf_count", "leaf_count"),
            ("source_clump_mean_height_m", "clump_mean_height_m"),
            ("source_middle_parbar_top_elevation_m", "middle_parbar_top_elevation_m"),
        ):
            if not row.get(source_column):
                row[source_column] = row[effective_column]
        if row["date"] not in dates:
            row["leaf_thickness_scale"] = row.get("leaf_thickness_scale") or 1.0
            row["leaf_width_scale"] = row.get("leaf_width_scale") or row["source_leaf_width_scale"]
            row["cluster_radius_scale"] = row.get("cluster_radius_scale") or 1.0
            row["cluster_outward_lean_degrees"] = row.get("cluster_outward_lean_degrees") or 0.0
            continue

        metadata = measured[(row["date"], row["plant_name"])]
        row["leaf_thickness_m"] = LEAF_THICKNESS_M_BY_DATE[row["date"]]
        row["leaf_thickness_scale"] = row["leaf_thickness_m"] / source_leaf_thickness
        row["leaf_width_scale"] = 1.0
        for column in (
            "source_cluster_offset_x_m",
            "source_cluster_offset_z_m",
            "source_cluster_offset_radius_m",
            "cluster_offset_x_m",
            "cluster_offset_z_m",
            "cluster_offset_radius_m",
        ):
            row[column] = metadata[column]
        row["cluster_radius_scale"] = 1.0
        row["cluster_outward_lean_degrees"] = 0.0
        height = float(metadata["plant_height_m"])
        row["pre_fit_height_m"] = height
        row["final_height_m"] = height
        row["height_error_m"] = height - float(row["target_height_m"])
        row["optimized_descriptor_scale"] = descriptor_scales[(row["date"], row["cultivar"])]
        row["per_plant_scale"] = 1.0
        row["leaf_count"] = int(float(metadata["leaf_count"]))
        row["main_culm_leaf_count"] = int(float(metadata["main_culm_leaf_count"]))
        row["tiller_leaf_count"] = int(float(metadata["tiller_leaf_count"]))
        row["primary_tiller_count"] = int(float(metadata["primary_tiller_count"]))
        for column in (
            "tiller_leaf_ratio_mean",
            "tiller_leaf_ratio_min",
            "tiller_leaf_ratio_max",
            "tiller_height_ratio_mean",
            "tiller_height_ratio_min",
            "tiller_height_ratio_max",
        ):
            row[column] = float(metadata[column])
        row["middle_parbar_top_elevation_m"] = float(metadata["middle_parbar_top_elevation_m"])
        clump_heights.setdefault((row["date"], row["cultivar"], row["base_plant_name"]), []).append(height)
        row["scene_asset_path"] = asset_path(target_4x10_scene(row["date"]))
        row["descriptor_asset_path"] = asset_path(calibrated_descriptor(descriptor_root, row["date"], row["cultivar"]))
    for row in rows:
        key = (row["date"], row["cultivar"], row["base_plant_name"])
        if key in clump_heights:
            row["clump_mean_height_m"] = statistics.fmean(clump_heights[key])
    write_csv(path, fieldnames, rows)


def safe_remove_path(path: Path, assets_root: Path, dry_run: bool) -> None:
    resolved = path.resolve()
    try:
        resolved.relative_to(assets_root.resolve())
    except ValueError as exc:
        raise RuntimeError(f"refusing to remove outside project assets: {resolved}") from exc
    if not path.exists():
        return
    print(f"remove: {path}")
    if dry_run:
        return
    if path.is_dir():
        shutil.rmtree(path)
    else:
        path.unlink()


def remove_obsolete_assets(args: argparse.Namespace, rows: list[dict[str, str]], dates: list[str]) -> None:
    assets_root = args.project_root / "Assets"
    old_scenes = {
        Path(row["scene_asset_path"])
        for row in rows
        if row["date"] in dates and row["scene_asset_path"].endswith("_HeightFit.evescene")
    }
    old_scenes.update(
        path.relative_to(assets_root)
        for path in (assets_root / "Generated").glob("Sorghum_LSystem_4x10_PARBAR_*_HeightFit.evescene")
        if any(date in path.name for date in dates)
    )
    old_scenes = sorted(old_scenes)
    for relative in old_scenes:
        safe_remove_path(assets_root / relative, assets_root, args.dry_run_delete)
        safe_remove_path(assets_root / f"{relative.as_posix()}.evefilemeta", assets_root, args.dry_run_delete)
    descriptor_root = assets_root / "Generated" / "Descriptors"
    for date in dates:
        safe_remove_path(descriptor_root / date, assets_root, args.dry_run_delete)
        safe_remove_path(descriptor_root / f"{date}.evefoldermeta", assets_root, args.dry_run_delete)
    if set(dates) == set(DATE_ORDER):
        safe_remove_path(descriptor_root, assets_root, args.dry_run_delete)
        safe_remove_path(assets_root / "Generated" / "Descriptors.evefoldermeta", assets_root, args.dry_run_delete)


def generated_scenes_exist(project_root: Path, scenes: list[Path]) -> bool:
    assets_root = project_root / "Assets"
    return all((assets_root / scene).exists() for scene in scenes)


def parse_dates(value: str) -> list[str]:
    dates = [item.strip() for item in value.split(",") if item.strip()]
    unknown = [date for date in dates if date not in DATE_ORDER]
    if unknown:
        raise argparse.ArgumentTypeError(f"unsupported date(s): {', '.join(unknown)}")
    return dates or list(DATE_ORDER)


def build_parser() -> argparse.ArgumentParser:
    repo_root = repo_root_from_script()
    project_root = repo_root / "Resources" / "DigitalAgricultureProject"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument("--build-dir", type=Path, default=repo_root / "out" / "build" / "vs2026-x64")
    parser.add_argument("--config", default="RelWithDebInfo")
    parser.add_argument("--project-root", type=Path, default=project_root)
    parser.add_argument("--runtime-package-dir", type=Path, default=repo_root / "out" / "build" / "vs2026-x64" / "EvoEngine_App" / "RelWithDebInfo" / "Packages")
    parser.add_argument("--manifest", type=Path, default=project_root / "Assets" / GENERATED_REPORT_ROOT / "field_manifest.csv")
    parser.add_argument("--calibrated-descriptor-root", type=Path, default=GENERATED_DESCRIPTOR_ROOT)
    parser.add_argument("--reference-4x10-scene", type=Path, default=Path(FOUR_BY_TEN_REFERENCE_SCENE))
    parser.add_argument("--dates", type=parse_dates, default=list(DATE_ORDER))
    parser.add_argument("--geometry-seed", type=int, default=2_000_000)
    parser.add_argument("--ten-by-ten-geometry-seed", type=int, default=1_000_000)
    parser.add_argument("--ten-by-ten-descriptor-date", default=TEN_BY_TEN_DESCRIPTOR_DATE)
    parser.add_argument("--middle-panel-height-fraction", type=float, default=2.0 / 3.0)
    parser.add_argument("--after-panel-move-frames", type=int, default=2)
    parser.add_argument("--max-wait-frames", type=int, default=30000)
    parser.add_argument("--keep-obsolete", action="store_true")
    parser.add_argument("--skip-10x10", action="store_true")
    parser.add_argument(
        "--publish-mobile-review",
        action="store_true",
        help="render and publish the five-date 4x10 mobile review package after generation",
    )
    parser.add_argument("--dry-run-delete", action="store_true")
    parser.add_argument("--worker-kind", choices=("", "4x10", "10x10"), default="", help=argparse.SUPPRESS)
    parser.add_argument("--worker-date", default="", help=argparse.SUPPRESS)
    parser.add_argument("--worker-metadata-path", type=Path, help=argparse.SUPPRESS)
    return parser


def worker_command(args: argparse.Namespace, kind: str, date: str = "", metadata_path: Path | None = None) -> list[str]:
    command = [
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
        "--manifest",
        str(args.manifest),
        "--calibrated-descriptor-root",
        str(args.calibrated_descriptor_root),
        "--reference-4x10-scene",
        str(args.reference_4x10_scene),
        "--dates",
        date or ",".join(args.dates),
        "--geometry-seed",
        str(args.geometry_seed),
        "--ten-by-ten-geometry-seed",
        str(args.ten_by_ten_geometry_seed),
        "--ten-by-ten-descriptor-date",
        args.ten_by_ten_descriptor_date,
        "--middle-panel-height-fraction",
        str(args.middle_panel_height_fraction),
        "--after-panel-move-frames",
        str(args.after_panel_move_frames),
        "--max-wait-frames",
        str(args.max_wait_frames),
        "--keep-obsolete",
        "--worker-kind",
        kind,
    ]
    if date:
        command.extend(["--worker-date", date])
    if metadata_path:
        command.extend(["--worker-metadata-path", str(metadata_path)])
    return command


def run_worker_processes(args: argparse.Namespace, dates: list[str]) -> tuple[list[Path], list[dict[str, str]]]:
    generated: list[Path] = []
    metadata_rows: list[dict[str, str]] = []
    with tempfile.TemporaryDirectory(prefix="sorghum_4x10_metadata_") as temporary_dir:
        for date in dates:
            metadata_path = Path(temporary_dir) / f"{date}.csv"
            result = subprocess.run(
                worker_command(args, "4x10", date, metadata_path), cwd=args.repo_root, check=False
            )
            scene_path = args.project_root / "Assets" / target_4x10_scene(date)
            if not metadata_path.exists() or not scene_path.exists():
                raise RuntimeError(f"4x10 worker failed for {date} with exit code {result.returncode}")
            metadata_rows.extend(read_csv(metadata_path))
            generated.append(target_4x10_scene(date))
    if not args.skip_10x10:
        result = subprocess.run(worker_command(args, "10x10"), cwd=args.repo_root, check=False)
        scene_path = args.project_root / "Assets" / TEN_BY_TEN_TARGET_SCENE
        if not scene_path.exists():
            raise RuntimeError(f"10x10 worker failed with exit code {result.returncode}")
        if result.returncode != 0:
            print(f"10x10 worker exited during engine shutdown ({result.returncode}); saved scene verified")
        generated.append(Path(TEN_BY_TEN_TARGET_SCENE))
    return generated, metadata_rows


def publish_mobile_review(args: argparse.Namespace) -> None:
    command = [
        sys.executable,
        str(args.repo_root / "PythonBinding" / "sorghum_render_4x10_mobile_review.py"),
        "--repo-root",
        str(args.repo_root),
        "--build-dir",
        str(args.build_dir),
        "--config",
        args.config,
        "--project",
        str(args.project_root / "test_lsystem_sorghum.eveproj"),
        "--runtime-package-dir",
        str(args.runtime_package_dir),
        "--source-root",
        str(args.project_root / "Assets" / GENERATED_SCENE_ROOT),
        "--manifest",
        str(args.manifest),
        "--geometry-seed",
        str(args.geometry_seed),
        "--publish-drive",
    ]
    result = subprocess.run(command, cwd=args.repo_root, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"mobile review publication failed with exit code {result.returncode}")


def main() -> None:
    args = build_parser().parse_args()
    args.repo_root = args.repo_root.resolve()
    args.build_dir = args.build_dir.resolve()
    args.project_root = args.project_root.resolve()
    args.runtime_package_dir = args.runtime_package_dir.resolve()
    args.manifest = args.manifest.resolve()
    if args.reference_4x10_scene.is_absolute() or ".." in args.reference_4x10_scene.parts:
        raise ValueError("--reference-4x10-scene must be relative to the project Assets folder")
    dates = list(args.dates)
    if args.ten_by_ten_descriptor_date not in DATE_ORDER:
        raise ValueError(f"unsupported --ten-by-ten-descriptor-date: {args.ten_by_ten_descriptor_date}")
    if args.publish_mobile_review and dates != list(DATE_ORDER):
        raise ValueError("--publish-mobile-review requires all five 4x10 dates")

    manifest_rows = read_csv(args.manifest)
    selected_rows = rows_for_dates(manifest_rows, dates)
    validate_descriptors(args, sorted(set(dates + [args.ten_by_ten_descriptor_date])))

    if args.worker_kind:
        configure_engine_imports(args.repo_root, args.build_dir, args.config)
        import PyDigitalAgriculture as evo

        if args.worker_kind == "4x10":
            worker_dates = [args.worker_date or dates[0]]
            _, metadata_rows = generate_4x10_scene(evo, args, selected_rows, worker_dates[0])
            if not args.worker_metadata_path:
                raise ValueError("4x10 worker requires --worker-metadata-path")
            write_csv(args.worker_metadata_path, WORKER_METADATA_COLUMNS, metadata_rows)
        else:
            generate_10x10_scene(evo, args)
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)

    generated, generated_metadata = run_worker_processes(args, dates)
    if not generated_scenes_exist(args.project_root, generated):
        raise RuntimeError("not all generated scenes exist; refusing manifest update and obsolete asset removal")
    update_manifest(
        args.manifest,
        manifest_rows,
        dates,
        args.calibrated_descriptor_root,
        generated_metadata,
        descriptor_length_scales(args, dates),
    )
    report_root = args.project_root / "Assets" / GENERATED_REPORT_ROOT
    write_post_tiller_validation(
        args.manifest,
        report_root / "calibration_summary.csv",
        report_root / "post_tiller_scene_validation.csv",
    )
    if not args.keep_obsolete:
        remove_obsolete_assets(args, selected_rows, dates)
    print(f"generated_scene_count={len(generated)}")
    if args.publish_mobile_review:
        publish_mobile_review(args)


if __name__ == "__main__":
    main()
