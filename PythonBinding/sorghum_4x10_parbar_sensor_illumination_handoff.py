#!/usr/bin/env python3
"""Generate date-height PARBAR illumination and morphology handoff CSVs."""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import subprocess
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

from sorghum_asset_layout import DATE_ORDER, GENERATED_DESCRIPTOR_ROOT, GENERATED_REPORT_ROOT, MANUAL_4X10_SCENE, descriptor_path


CULTIVARS = ("BTX", "Pawaga")
BAR_LEVELS = ("top", "middle", "bottom")
REFERENCE_4X10_SCENE = MANUAL_4X10_SCENE.as_posix()

SENSOR_NUMERIC_COLUMNS = [
    "illumination_total_simulated",
    "probe_position_x_m",
    "probe_position_y_m",
    "probe_position_z_m",
    "probe_normal_x",
    "probe_normal_y",
    "probe_normal_z",
    "represented_clump_count",
    "average_represented_root_elevation_m",
    "average_represented_plant_height_m",
    "height_fraction_of_average_height",
    "sensor_top_elevation_m",
]

SENSOR_COLUMNS = [
    "date",
    "cultivar",
    "source_scene",
    "reference_scene",
    "calibrated_btx_descriptor",
    "calibrated_pawaga_descriptor",
    "sensor_bar_level",
    "probe_number",
    "height_rule",
    "replicate_count",
] + [f"{column}_{suffix}" for column in SENSOR_NUMERIC_COLUMNS for suffix in ("mean", "std")] + [
    "ray_samples",
    "ray_bounces",
    "ray_seed_base",
    "ray_seed_last",
    "geometry_seed_base",
    "geometry_seed_last",
    "geometry_seed_stride",
    "push_normal_distance_m",
]

PLANT_COLUMNS = [
    "date",
    "cultivar",
    "original_position_id",
    "plant_id",
    "cluster_member_number",
    "plants_at_original_position",
    "plant_height_m",
    "source_final_height_m",
    "target_height_m",
    "height_error_m",
    "source_height_error_m",
    "leaf_count",
    "source_leaf_count",
    "target_leaf_modules_mean",
    "target_leaf_modules_stdev",
    "clump_mean_height_m",
    "source_clump_mean_height_m",
    "source_leaf_thickness_m",
    "leaf_thickness_scale",
    "leaf_thickness_m",
    "source_leaf_width_scale",
    "leaf_width_scale",
    "source_cluster_offset_x_m",
    "source_cluster_offset_z_m",
    "source_cluster_offset_radius_m",
    "cluster_offset_x_m",
    "cluster_offset_z_m",
    "cluster_offset_radius_m",
    "cluster_radius_scale",
    "cluster_outward_lean_degrees",
    "middle_parbar_top_elevation_m",
    "source_middle_parbar_top_elevation_m",
    "source_scene",
]

CLUMP_COLUMNS = [
    "date",
    "cultivar",
    "original_position_id",
    "plants_at_original_position",
    "extra_clustered_plants",
    "anchor_plant_id",
    "anchor_plant_height_m",
    "mean_plant_height_m",
    "min_plant_height_m",
    "max_plant_height_m",
    "mean_leaf_count",
    "min_leaf_count",
    "max_leaf_count",
    "member_plant_ids",
    "target_height_m",
    "target_leaf_modules_mean",
    "target_leaf_modules_stdev",
    "middle_parbar_top_elevation_m",
    "source_scene",
]


@dataclass
class RunningStats:
    count: int = 0
    mean: float = 0.0
    m2: float = 0.0

    def add(self, value: float) -> None:
        self.count += 1
        delta = value - self.mean
        self.mean += delta / self.count
        self.m2 += delta * (value - self.mean)

    @property
    def std(self) -> float:
        return math.sqrt(self.m2 / (self.count - 1)) if self.count > 1 else 0.0


@dataclass
class SummaryAccumulator:
    base: dict[str, object]
    stats: dict[str, RunningStats] = field(default_factory=dict)

    def add(self, values: dict[str, float]) -> None:
        for column, value in values.items():
            self.stats.setdefault(column, RunningStats()).add(float(value))

    def row(self, args: argparse.Namespace) -> dict[str, object]:
        output = dict(self.base)
        output["replicate_count"] = next(iter(self.stats.values())).count if self.stats else 0
        for column in SENSOR_NUMERIC_COLUMNS:
            stats = self.stats.get(column, RunningStats())
            output[f"{column}_mean"] = stats.mean
            output[f"{column}_std"] = stats.std
        output.update(
            {
                "ray_samples": args.samples,
                "ray_bounces": args.bounces,
                "ray_seed_base": args.seed,
                "ray_seed_last": args.seed + max(0, args.replicates - 1),
                "geometry_seed_base": args.geometry_seed,
                "geometry_seed_last": args.geometry_seed + max(0, args.replicates - 1) * args.geometry_seed_stride,
                "geometry_seed_stride": args.geometry_seed_stride,
                "push_normal_distance_m": args.push_normal_distance,
            }
        )
        return output


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


def parse_dates(value: str) -> list[str]:
    dates = [item.strip() for item in value.split(",") if item.strip()]
    if not dates:
        raise argparse.ArgumentTypeError("at least one date is required")
    unknown = [date for date in dates if date not in DATE_ORDER]
    if unknown:
        raise argparse.ArgumentTypeError(f"unsupported date(s): {', '.join(unknown)}")
    return dates


def natural_key(value: str) -> tuple[str, int, str]:
    digits = "".join(ch if ch.isdigit() else " " for ch in value).split()
    return value.split("_LSystem_")[0], int(digits[-1]) if digits else -1, value


def unique_output_dir(path: Path) -> Path:
    if not path.exists():
        return path
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return path.with_name(f"{path.name}_{timestamp}")


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, columns: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def tail_text(path: Path, line_count: int = 80) -> str:
    if not path.exists():
        return ""
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(lines[-line_count:])


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


def vec3(value: object) -> tuple[float, float, float]:
    return float(value.x), float(value.y), float(value.z)


def float_value(row: dict[str, str], column: str) -> float:
    return float(row[column])


def int_value(row: dict[str, str], column: str) -> int:
    return int(float(row[column]))


def load_height_manifest(source_root: Path, dates: list[str]) -> list[dict[str, str]]:
    path = source_root / "field_manifest.csv"
    if not path.exists():
        raise FileNotFoundError(
            f"height-fit manifest not found: {path}\n"
            "Install the Sorghum L-System resource bundle so "
            "Resources/DigitalAgricultureProject/Assets/GeneratedAssets/Reports/field_manifest.csv exists, "
            "or pass --source-root."
        )
    rows = [row for row in read_csv(path) if row["date"] in dates]
    missing = [date for date in dates if not any(row["date"] == date for row in rows)]
    if missing:
        raise ValueError(f"height-fit manifest is missing date(s): {', '.join(missing)}")
    return rows


def source_scene_for_date(manifest_rows: list[dict[str, str]], date: str) -> str:
    scenes = sorted({row["scene_asset_path"] for row in manifest_rows if row["date"] == date})
    if len(scenes) != 1:
        raise ValueError(f"{date} has {len(scenes)} source scenes in height-fit manifest")
    return scenes[0]


def leaf_thickness_for_date(manifest_rows: list[dict[str, str]], date: str) -> float:
    values = {float(row["leaf_thickness_m"]) for row in manifest_rows if row["date"] == date}
    if len(values) != 1:
        raise ValueError(f"{date} has {len(values)} leaf thickness values in height-fit manifest")
    return values.pop()


def leaf_width_scale_for_date(manifest_rows: list[dict[str, str]], date: str) -> float:
    values = {float(row["leaf_width_scale"]) for row in manifest_rows if row["date"] == date}
    if len(values) != 1:
        raise ValueError(f"{date} has {len(values)} leaf width scale values in height-fit manifest")
    return values.pop()


def calibrated_descriptor(args: argparse.Namespace, date: str, cultivar: str) -> str:
    return descriptor_path(args.calibrated_descriptor_root, date, cultivar).as_posix()


def plant_rows_from_manifest(rows: list[dict[str, str]]) -> list[dict[str, object]]:
    output: list[dict[str, object]] = []
    for row in rows:
        output.append(
            {
                "date": row["date"],
                "cultivar": row["cultivar"],
                "original_position_id": row["base_plant_name"],
                "plant_id": row["plant_name"],
                "cluster_member_number": int_value(row, "cluster_index") + 1,
                "plants_at_original_position": int_value(row, "cluster_size"),
                "plant_height_m": float_value(row, "final_height_m"),
                "source_final_height_m": float_value(row, "source_final_height_m"),
                "target_height_m": float_value(row, "target_height_m"),
                "height_error_m": float_value(row, "height_error_m"),
                "source_height_error_m": float_value(row, "source_height_error_m"),
                "leaf_count": int_value(row, "leaf_count"),
                "source_leaf_count": int_value(row, "source_leaf_count"),
                "target_leaf_modules_mean": float_value(row, "leaf_modules_mean"),
                "target_leaf_modules_stdev": float_value(row, "leaf_modules_deviation"),
                "clump_mean_height_m": float_value(row, "clump_mean_height_m"),
                "source_clump_mean_height_m": float_value(row, "source_clump_mean_height_m"),
                "source_leaf_thickness_m": float_value(row, "source_leaf_thickness_m"),
                "leaf_thickness_scale": float_value(row, "leaf_thickness_scale"),
                "leaf_thickness_m": float_value(row, "leaf_thickness_m"),
                "source_leaf_width_scale": float_value(row, "source_leaf_width_scale"),
                "leaf_width_scale": float_value(row, "leaf_width_scale"),
                "source_cluster_offset_x_m": float_value(row, "source_cluster_offset_x_m"),
                "source_cluster_offset_z_m": float_value(row, "source_cluster_offset_z_m"),
                "source_cluster_offset_radius_m": float_value(row, "source_cluster_offset_radius_m"),
                "cluster_offset_x_m": float_value(row, "cluster_offset_x_m"),
                "cluster_offset_z_m": float_value(row, "cluster_offset_z_m"),
                "cluster_offset_radius_m": float_value(row, "cluster_offset_radius_m"),
                "cluster_radius_scale": float_value(row, "cluster_radius_scale"),
                "cluster_outward_lean_degrees": float_value(row, "cluster_outward_lean_degrees"),
                "middle_parbar_top_elevation_m": float_value(row, "middle_parbar_top_elevation_m"),
                "source_middle_parbar_top_elevation_m": float_value(
                    row, "source_middle_parbar_top_elevation_m"
                ),
                "source_scene": row["scene_asset_path"],
            }
        )
    return sorted(output, key=lambda row: (row["date"], row["cultivar"], natural_key(str(row["original_position_id"])), int(row["cluster_member_number"])))


def clump_rows_from_plants(plant_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    groups: dict[tuple[str, str, str], list[dict[str, object]]] = defaultdict(list)
    for row in plant_rows:
        groups[(str(row["date"]), str(row["cultivar"]), str(row["original_position_id"]))].append(row)

    output: list[dict[str, object]] = []
    for (date, cultivar, original_position_id), members in groups.items():
        members = sorted(members, key=lambda row: int(row["cluster_member_number"]))
        heights = [float(row["plant_height_m"]) for row in members]
        leaves = [int(row["leaf_count"]) for row in members]
        anchor = next((row for row in members if int(row["cluster_member_number"]) == 1), members[0])
        output.append(
            {
                "date": date,
                "cultivar": cultivar,
                "original_position_id": original_position_id,
                "plants_at_original_position": len(members),
                "extra_clustered_plants": max(0, len(members) - 1),
                "anchor_plant_id": anchor["plant_id"],
                "anchor_plant_height_m": anchor["plant_height_m"],
                "mean_plant_height_m": statistics.fmean(heights),
                "min_plant_height_m": min(heights),
                "max_plant_height_m": max(heights),
                "mean_leaf_count": statistics.fmean(leaves),
                "min_leaf_count": min(leaves),
                "max_leaf_count": max(leaves),
                "member_plant_ids": ";".join(str(row["plant_id"]) for row in members),
                "target_height_m": members[0]["target_height_m"],
                "target_leaf_modules_mean": members[0]["target_leaf_modules_mean"],
                "target_leaf_modules_stdev": members[0]["target_leaf_modules_stdev"],
                "middle_parbar_top_elevation_m": members[0]["middle_parbar_top_elevation_m"],
                "source_scene": members[0]["source_scene"],
            }
        )
    return sorted(output, key=lambda row: (row["date"], row["cultivar"], natural_key(str(row["original_position_id"]))))


def start_project(evo: object, project: Path, runtime_package_dir: Path, scene_asset_path: str, max_wait_frames: int) -> None:
    ok = evo.RunLSystemSorghumProject(project.resolve(), runtime_package_dir.resolve(), Path(scene_asset_path))
    if not ok:
        raise RuntimeError(f"failed to start project scene: {scene_asset_path}")
    if not evo.WaitForProjectIdle(max_wait_frames):
        raise RuntimeError(f"project did not become idle after loading {scene_asset_path}")
    if not evo.EnsureIlluminationSoilContext():
        raise RuntimeError(f"failed to ensure PBR soil context in scene: {scene_asset_path}")
    if not evo.ValidateIlluminationContext():
        raise RuntimeError(f"scene failed illumination context validation: {scene_asset_path}")


def sensor_values(record: object) -> dict[str, float]:
    position = vec3(record.position)
    normal = vec3(record.normal)
    return {
        "illumination_total_simulated": float(record.scalar),
        "probe_position_x_m": position[0],
        "probe_position_y_m": position[1],
        "probe_position_z_m": position[2],
        "probe_normal_x": normal[0],
        "probe_normal_y": normal[1],
        "probe_normal_z": normal[2],
        "represented_clump_count": float(record.represented_plant_count),
        "average_represented_root_elevation_m": float(record.average_represented_root_elevation_m),
        "average_represented_plant_height_m": float(record.average_represented_plant_height_m),
        "height_fraction_of_average_height": float(record.height_fraction_of_average_height),
        "sensor_top_elevation_m": float(record.sensor_top_elevation_m),
    }


def sensor_base_row(args: argparse.Namespace, date: str, scene_asset_path: str, record: object) -> dict[str, object]:
    return {
        "date": date,
        "cultivar": record.cultivar,
        "source_scene": scene_asset_path,
        "reference_scene": args.reference_scene,
        "calibrated_btx_descriptor": calibrated_descriptor(args, date, "BTX"),
        "calibrated_pawaga_descriptor": calibrated_descriptor(args, date, "Pawaga"),
        "sensor_bar_level": record.sensor_bar_level,
        "probe_number": int(record.column) + 1,
        "height_rule": record.height_rule,
    }


def sensor_rows_for_date(
    evo: object,
    args: argparse.Namespace,
    date: str,
    scene_asset_path: str,
    leaf_thickness_m: float,
    leaf_width_scale: float,
) -> list[dict[str, object]]:
    project_bytes = args.project.read_bytes()
    side_effects_before = collect_default_scene_side_effects(args.project)
    sensors = None
    try:
        start_project(evo, args.project, args.runtime_package_dir, scene_asset_path, args.max_wait_frames)
        btx_descriptor = calibrated_descriptor(args, date, "BTX")
        pawaga_descriptor = calibrated_descriptor(args, date, "Pawaga")
        assigned = int(evo.SetSorghumLsCultivarDescriptors(Path(btx_descriptor), Path(pawaga_descriptor), False, -1))
        if assigned != 163:
            raise RuntimeError(f"{date}: expected 163 descriptor-assigned L-System plants, got {assigned}")

        thickness_count = int(evo.SetSorghumLsLeafThickness(leaf_thickness_m, False))
        if thickness_count != 163:
            raise RuntimeError(f"{date}: expected 163 thickness-updated L-System plants, got {thickness_count}")
        width_count = int(evo.SetSorghumLsLeafWidthScale(leaf_width_scale, False))
        if width_count != 163:
            raise RuntimeError(f"{date}: expected 163 width-updated L-System plants, got {width_count}")

        base_regenerated = int(evo.GrowSorghumLsPlantsToAdulthood(args.geometry_seed))
        if base_regenerated != 163:
            raise RuntimeError(f"{date}: expected 163 base regenerated L-System plants, got {base_regenerated}")
        evo.LoopFrames(args.after_panel_move_frames)
        if not evo.WaitForProjectIdle(args.max_wait_frames):
            raise RuntimeError(f"project did not become idle after base regeneration for {date}")

        moved = int(evo.MoveParbarMiddlePanelsToPlantHeightFraction(args.middle_panel_height_fraction))
        if moved != 2:
            raise RuntimeError(f"{date}: expected 2 moved middle PARBAR panels, got {moved}")
        evo.LoopFrames(args.after_panel_move_frames)
        if not evo.WaitForProjectIdle(args.max_wait_frames):
            raise RuntimeError(f"project did not become idle after moving middle panels for {date}")

        sensors = evo.CreateParbarTopFaceSensorGroup(args.probes_per_panel)
        expected = len(CULTIVARS) * len(BAR_LEVELS) * args.probes_per_panel
        accumulators: dict[tuple[str, str, int], SummaryAccumulator] = {}
        for replicate in range(args.replicates):
            geometry_seed = args.geometry_seed + replicate * args.geometry_seed_stride
            ray_seed = args.seed + replicate
            regenerated = int(evo.GrowSorghumLsPlantsToAdulthood(geometry_seed))
            if regenerated != 163:
                raise RuntimeError(f"{date}: expected 163 regenerated L-System plants, got {regenerated}")
            evo.LoopFrames(1)
            if not evo.WaitForProjectIdle(args.max_wait_frames):
                raise RuntimeError(f"project did not become idle after replicate {replicate} regeneration for {date}")
            evo.EstimatePARSensors(sensors, args.samples, args.bounces, args.push_normal_distance, ray_seed)
            records = evo.GetParbarTopFaceSensorResults(sensors, args.probes_per_panel)
            if len(records) != expected:
                raise RuntimeError(f"{date}: expected {expected} PARBAR sensor records, got {len(records)}")
            for record in records:
                key = (record.cultivar, record.sensor_bar_level, int(record.column))
                if key not in accumulators:
                    accumulators[key] = SummaryAccumulator(sensor_base_row(args, date, scene_asset_path, record))
                accumulators[key].add(sensor_values(record))
        rows = [accumulator.row(args) for accumulator in accumulators.values()]
    finally:
        try:
            if sensors is not None and hasattr(evo, "DeleteRuntimeAsset"):
                evo.DeleteRuntimeAsset(sensors)
        except Exception:
            pass
        try:
            evo.Terminate()
        finally:
            args.project.write_bytes(project_bytes)
            cleanup_new_default_scene_side_effects(args.project, side_effects_before)
    return sorted(rows, key=lambda row: (row["date"], row["cultivar"], BAR_LEVELS.index(str(row["sensor_bar_level"])), int(row["probe_number"])))


def worker_command(args: argparse.Namespace, date: str, sensor_csv: Path) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "--repo-root",
        str(args.repo_root),
        "--build-dir",
        str(args.build_dir),
        "--config",
        args.config,
        "--runtime-package-dir",
        str(args.runtime_package_dir),
        "--project",
        str(args.project),
        "--source-root",
        str(args.source_root),
        "--calibrated-descriptor-root",
        str(args.calibrated_descriptor_root),
        "--reference-scene",
        args.reference_scene,
        "--output-dir",
        str(args.output_dir),
        "--dates",
        date,
        "--probes-per-panel",
        str(args.probes_per_panel),
        "--replicates",
        str(args.replicates),
        "--samples",
        str(args.samples),
        "--bounces",
        str(args.bounces),
        "--seed",
        str(args.seed),
        "--geometry-seed",
        str(args.geometry_seed),
        "--geometry-seed-stride",
        str(args.geometry_seed_stride),
        "--push-normal-distance",
        str(args.push_normal_distance),
        "--middle-panel-height-fraction",
        str(args.middle_panel_height_fraction),
        "--max-wait-frames",
        str(args.max_wait_frames),
        "--after-panel-move-frames",
        str(args.after_panel_move_frames),
        "--worker-sensor-csv",
        str(sensor_csv),
    ]


def sensor_rows_with_workers(args: argparse.Namespace) -> list[dict[str, object]]:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    worker_dir = args.output_dir.parent / f".{args.output_dir.name}_workers_{stamp}"
    worker_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    for date in args.dates:
        sensor_csv = worker_dir / f"{date}_sensors.csv"
        log_path = worker_dir / f"{date}_worker.log"
        with log_path.open("w", encoding="utf-8", errors="replace") as log:
            result = subprocess.run(worker_command(args, date, sensor_csv), cwd=args.repo_root, stdout=log, stderr=subprocess.STDOUT, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"{date} worker failed with exit code {result.returncode}. Log: {log_path}\n{tail_text(log_path)}")
        date_rows = read_csv(sensor_csv)
        if len(date_rows) != len(CULTIVARS) * len(BAR_LEVELS) * args.probes_per_panel:
            raise RuntimeError(f"{date} worker wrote {len(date_rows)} sensor rows")
        print(f"{date}: sensor_rows={len(date_rows)} log={log_path}")
        rows.extend(date_rows)
    return sorted(rows, key=lambda row: (row["date"], row["cultivar"], BAR_LEVELS.index(str(row["sensor_bar_level"])), int(row["probe_number"])))


def write_split_outputs(output_dir: Path, sensor_rows: list[dict[str, object]], plant_rows: list[dict[str, object]], clump_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    index_rows: list[dict[str, object]] = []

    def write_named(relative_path: Path, columns: list[str], rows: list[dict[str, object]], description: str) -> None:
        write_csv(output_dir / relative_path, columns, rows)
        index_rows.append({"file": relative_path.as_posix(), "row_count": len(rows), "description": description})

    write_named(Path("all_parbar_sensors_summary.csv"), SENSOR_COLUMNS, sensor_rows, "All replicated PARBAR sensor probe summary rows.")
    write_named(Path("all_individual_plants_long.csv"), PLANT_COLUMNS, plant_rows, "All fitted individual simulated plant layout/target rows.")
    write_named(Path("all_clumps_long.csv"), CLUMP_COLUMNS, clump_rows, "All original-position clump layout/target summary rows.")

    for date in DATE_ORDER:
        for cultivar in CULTIVARS:
            sensors = [row for row in sensor_rows if row["date"] == date and row["cultivar"] == cultivar]
            plants = [row for row in plant_rows if row["date"] == date and row["cultivar"] == cultivar]
            clumps = [row for row in clump_rows if row["date"] == date and row["cultivar"] == cultivar]
            if sensors:
                write_named(Path("sensors") / f"{date}_{cultivar}_parbar_sensors_summary.csv", SENSOR_COLUMNS, sensors, f"{date} {cultivar} replicated PARBAR sensor probe summaries.")
            if plants:
                write_named(Path("plants") / f"{date}_{cultivar}_individual_plants.csv", PLANT_COLUMNS, plants, f"{date} {cultivar} fitted individual simulated plant layout rows.")
            if clumps:
                write_named(Path("clumps") / f"{date}_{cultivar}_clump_summary.csv", CLUMP_COLUMNS, clumps, f"{date} {cultivar} original-position clump summaries.")
    return sorted(index_rows, key=lambda row: str(row["file"]))


def validate_outputs(sensor_rows: list[dict[str, object]], plant_rows: list[dict[str, object]], clump_rows: list[dict[str, object]], args: argparse.Namespace) -> None:
    expected_sensor_rows = len(args.dates) * len(CULTIVARS) * len(BAR_LEVELS) * args.probes_per_panel
    if len(sensor_rows) != expected_sensor_rows:
        raise ValueError(f"expected {expected_sensor_rows} sensor rows, got {len(sensor_rows)}")
    for row in sensor_rows:
        value = float(row["illumination_total_simulated_mean"])
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"invalid illumination mean value: {value}")
        if row["sensor_bar_level"] not in BAR_LEVELS:
            raise ValueError(f"unexpected sensor bar level: {row['sensor_bar_level']}")

    expected_plant_counts = {"BTX": 81, "Pawaga": 82}
    for date in args.dates:
        for cultivar, count in expected_plant_counts.items():
            plants = [row for row in plant_rows if row["date"] == date and row["cultivar"] == cultivar]
            clumps = [row for row in clump_rows if row["date"] == date and row["cultivar"] == cultivar]
            if len(plants) != count:
                raise ValueError(f"{date} {cultivar}: expected {count} plant rows, got {len(plants)}")
            if len(clumps) != 20:
                raise ValueError(f"{date} {cultivar}: expected 20 clump rows, got {len(clumps)}")


def write_readme(path: Path, args: argparse.Namespace, sensor_rows: list[dict[str, object]], plant_rows: list[dict[str, object]], clump_rows: list[dict[str, object]]) -> None:
    text = f"""# Date-Height PARBAR Illumination Handoff

This folder contains replicated simulated PARBAR sensor illumination summaries plus plant and clump layout metadata for five date-specific L-System sorghum scenes.

## Files

- `sensors/`: one CSV per date and cultivar with replicated PARBAR probe summary rows.
- `plants/`: one CSV per date and cultivar with one row per simulated plant from the source layout manifest.
- `clumps/`: one CSV per date and cultivar with one row per original field position.
- `all_parbar_sensors_summary.csv`, `all_individual_plants_long.csv`, and `all_clumps_long.csv`: combined versions of the split CSVs.
- `handoff_file_index.csv`: file list and row counts.

Plant rows include both authored source and effective leaf-thickness, leaf-width-scale, and cluster-offset values,
plus the cluster radius scale and outward lean used by the generated scene. Current height, leaf-count, clump-height,
and middle-PARBAR columns are measured from the deterministic base geometry generated with the configured geometry
seed; corresponding `source_*` columns retain the prior fitted manifest values.

## Illumination Values

`illumination_total_simulated_mean` and `_std` summarize EvoEngine's scalar estimate for point probes on the top face of simulated PARBAR sensor bars. The values are simulated relative light estimates, not calibrated physical PAR units.

Before tracing rays, each scene is repaired so `Ground Mesh` uses the PBR soil material through a `MeshRenderer`. Each date scene also swaps BTX and Pawaga plants onto the calibrated date/cultivar descriptors, then regenerates plant geometry from deterministic replicate seeds.

Middle PARBAR panels are placed once per date scene at two-thirds of the represented crop height and remain fixed across replicate geometry draws.

## Run Settings

- Source project: `{args.project}`
- Reference scene: `{args.reference_scene}`
- Source generated assets: `{args.source_root}`
- Calibrated descriptor root: `{args.calibrated_descriptor_root}`
- Dates: {', '.join(args.dates)}
- Replicates per summary row: {args.replicates}
- Probes per sensor bar: {args.probes_per_panel}
- Ray samples: {args.samples}
- Ray bounces: {args.bounces}
- Ray seed range: {args.seed} to {args.seed + max(0, args.replicates - 1)}
- Geometry seed range: {args.geometry_seed} to {args.geometry_seed + max(0, args.replicates - 1) * args.geometry_seed_stride}
- Push normal distance: {args.push_normal_distance} m

## Row Counts

- Sensor summary rows: {len(sensor_rows)}
- Individual plant rows: {len(plant_rows)}
- Clump summary rows: {len(clump_rows)}
"""
    path.write_text(text, encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    repo_root = repo_root_from_script()
    source_project_root = repo_root / "Resources" / "DigitalAgricultureProject"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument("--build-dir", type=Path, default=repo_root / "out" / "build" / "vs2026-x64")
    parser.add_argument("--config", default="RelWithDebInfo")
    parser.add_argument("--runtime-package-dir", type=Path, default=repo_root / "out" / "build" / "vs2026-x64" / "EvoEngine_App" / "RelWithDebInfo" / "Packages")
    parser.add_argument("--project", type=Path, default=source_project_root / "test_lsystem_sorghum.eveproj")
    parser.add_argument("--source-root", type=Path, default=source_project_root / "Assets" / GENERATED_REPORT_ROOT)
    parser.add_argument("--calibrated-descriptor-root", type=Path, default=GENERATED_DESCRIPTOR_ROOT)
    parser.add_argument("--reference-scene", default=REFERENCE_4X10_SCENE)
    parser.add_argument("--output-dir", type=Path, default=repo_root / "out" / "handoff" / "sorghum_4x10_parbar_sensor_illumination_handoff")
    parser.add_argument("--dates", type=parse_dates, default=list(DATE_ORDER))
    parser.add_argument("--probes-per-panel", type=int, default=100)
    parser.add_argument("--replicates", type=int, default=10000)
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--bounces", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--geometry-seed", type=int, default=2_000_000)
    parser.add_argument("--geometry-seed-stride", type=int, default=1000)
    parser.add_argument("--push-normal-distance", type=float, default=0.001)
    parser.add_argument("--middle-panel-height-fraction", type=float, default=2.0 / 3.0)
    parser.add_argument("--max-wait-frames", type=int, default=30000)
    parser.add_argument("--after-panel-move-frames", type=int, default=2)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--worker-sensor-csv", type=Path, default=None, help=argparse.SUPPRESS)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    args.repo_root = args.repo_root.resolve()
    args.build_dir = args.build_dir.resolve()
    args.runtime_package_dir = args.runtime_package_dir.resolve()
    args.project = args.project.resolve()
    args.source_root = args.source_root.resolve()
    args.output_dir = args.output_dir.resolve() if args.worker_sensor_csv else unique_output_dir(args.output_dir.resolve())
    if args.probes_per_panel <= 0:
        raise ValueError("--probes-per-panel must be positive")
    if args.replicates <= 0:
        raise ValueError("--replicates must be positive")
    if args.geometry_seed_stride <= 0:
        raise ValueError("--geometry-seed-stride must be positive")
    if args.smoke:
        args.dates = args.dates[:1]
        args.probes_per_panel = min(args.probes_per_panel, 2)
        args.replicates = min(args.replicates, 2)
        args.samples = min(args.samples, 1)

    manifest_rows = load_height_manifest(args.source_root, args.dates)
    plant_rows = plant_rows_from_manifest(manifest_rows)
    clump_rows = clump_rows_from_plants(plant_rows)

    if args.worker_sensor_csv:
        if len(args.dates) != 1:
            raise ValueError("--worker-sensor-csv requires exactly one date")
        configure_engine_imports(args.repo_root, args.build_dir, args.config)
        import PyDigitalAgriculture as evo

        try:
            sensor_rows = sensor_rows_for_date(
                evo,
                args,
                args.dates[0],
                source_scene_for_date(manifest_rows, args.dates[0]),
                leaf_thickness_for_date(manifest_rows, args.dates[0]),
                leaf_width_scale_for_date(manifest_rows, args.dates[0]),
            )
        finally:
            try:
                evo.Terminate()
            except Exception:
                pass
        validate_outputs(sensor_rows, plant_rows, clump_rows, args)
        write_csv(args.worker_sensor_csv, SENSOR_COLUMNS, sensor_rows)
        print(f"worker_sensor_csv={args.worker_sensor_csv}")
        print(f"sensor_rows={len(sensor_rows)}")
        return

    if len(args.dates) > 1:
        sensor_rows = sensor_rows_with_workers(args)
    else:
        configure_engine_imports(args.repo_root, args.build_dir, args.config)
        import PyDigitalAgriculture as evo

        try:
            sensor_rows = sensor_rows_for_date(
                evo,
                args,
                args.dates[0],
                source_scene_for_date(manifest_rows, args.dates[0]),
                leaf_thickness_for_date(manifest_rows, args.dates[0]),
                leaf_width_scale_for_date(manifest_rows, args.dates[0]),
            )
        finally:
            try:
                evo.Terminate()
            except Exception:
                pass

    validate_outputs(sensor_rows, plant_rows, clump_rows, args)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    index_rows = write_split_outputs(args.output_dir, sensor_rows, plant_rows, clump_rows)
    write_csv(args.output_dir / "handoff_file_index.csv", ["file", "row_count", "description"], index_rows)
    write_readme(args.output_dir / "README.md", args, sensor_rows, plant_rows, clump_rows)

    print(f"output_dir={args.output_dir}")
    print(f"sensor_summary_rows={len(sensor_rows)}")
    print(f"plant_rows={len(plant_rows)}")
    print(f"clump_rows={len(clump_rows)}")


if __name__ == "__main__":
    main()
