#!/usr/bin/env python3
"""Calibrate one Sorghum L-system descriptor per cultivar/date target."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import random
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from sorghum_asset_layout import (
    GENERATED_DESCRIPTOR_ROOT,
    GENERATED_REPORT_ROOT,
    MANUAL_4X10_SCENE,
    descriptor_path,
    growth_stage,
)
from statistics import fmean
from sorghum_migrate_fidelity_v4_descriptors import STAGES as FIDELITY_STAGE_VALUES, migrate as migrate_fidelity_v4


BASE_SCENE = MANUAL_4X10_SCENE.as_posix()
DEFAULT_OUTPUT_ASSET_ROOT = GENERATED_DESCRIPTOR_ROOT
LEAF_WIDTH_SCALE_BY_DATE = {
    date: 1.0
    for date in ("2021-07-01", "2021-07-14", "2021-08-18", "2021-08-30", "2021-09-02")
}
MAIN_CULM_DIAMETER_M_BY_DATE = {
    "2021-07-01": 0.0120,
    "2021-07-14": 0.0135,
    "2021-08-18": 0.0150,
    "2021-08-30": 0.0165,
    "2021-09-02": 0.0165,
}
TARGET_COLUMNS = (
    "date",
    "cultivar",
    "leaf_modules_mean",
    "leaf_modules_deviation",
    "target_height_m",
    "target_height_std_m",
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
    "length_mean_scale",
    "length_deviation_scale",
    "descriptor_asset_path",
    "success",
    "failed_metrics",
)


@dataclass(frozen=True)
class Target:
    date: str
    cultivar: str
    leaf_mean: float
    leaf_std: float
    height_mean_m: float
    height_std_m: float
    initial_length_mean_scale: float
    row_count: int


@dataclass
class Knobs:
    leaf_mean: float
    leaf_std: float
    length_mean_scale: float
    length_deviation_scale: float
    tiller_leaf_count_ratio: float = 0.90
    tiller_height_ratio: float = 0.90


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


def validate_asset_relative_path(path: Path, option_name: str) -> None:
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"{option_name} must be relative to the project Assets folder")


def project_assets_root(project_path: Path) -> Path:
    return project_path.resolve().parent / "Assets"


def output_sidecar_path(args: argparse.Namespace) -> Path:
    return project_assets_root(args.project) / f"{args.output_asset_root.as_posix()}.evefoldermeta"


def is_output_path(args: argparse.Namespace, path: Path) -> bool:
    resolved = path.resolve()
    output_root = (project_assets_root(args.project) / args.output_asset_root).resolve()
    try:
        resolved.relative_to(output_root)
        return True
    except ValueError:
        return resolved == output_sidecar_path(args).resolve()


def collect_engine_side_effect_paths(args: argparse.Namespace) -> set[Path]:
    assets_root = project_assets_root(args.project)
    if not assets_root.exists():
        return set()
    paths = set(assets_root.rglob("*.evefilemeta"))
    paths.update(assets_root.rglob("*.evefoldermeta"))
    new_scene = assets_root / "New Scene.evescene"
    if new_scene.exists():
        paths.add(new_scene)
    return paths


def cleanup_new_engine_side_effects(args: argparse.Namespace, before: set[Path]) -> None:
    for path in sorted(collect_engine_side_effect_paths(args) - before, key=lambda value: len(value.parts), reverse=True):
        if is_output_path(args, path):
            continue
        if path.is_file():
            path.unlink()


def parse_date(value: str) -> str:
    value = value.strip()
    for fmt in ("%Y-%m-%d", "%m/%d/%y", "%m/%d/%Y"):
        try:
            return datetime.strptime(value, fmt).date().isoformat()
        except ValueError:
            pass
    raise ValueError(f"unsupported date format: {value}")


def population_std(values: list[float]) -> float:
    if not values:
        return 0.0
    mean = fmean(values)
    return math.sqrt(fmean([(value - mean) ** 2 for value in values]))


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def finite_or(value: float, fallback: float) -> float:
    return value if math.isfinite(value) else fallback


def read_targets(path: Path) -> list[Target]:
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    if not rows:
        raise ValueError(f"target manifest has no rows: {path}")

    groups: dict[tuple[str, str], list[dict[str, str]]] = {}
    for row in rows:
        groups.setdefault((parse_date(row["date"]), row["cultivar"].strip()), []).append(row)

    targets: list[Target] = []
    for (date, cultivar), group in sorted(groups.items()):
        leaf_means = [float(row["leaf_modules_mean"]) for row in group]
        leaf_stds = [float(row["leaf_modules_deviation"]) for row in group]
        target_heights = [float(row["target_height_m"]) for row in group]
        pre_fit_heights = [
            float(row.get("source_pre_fit_height_m") or row["pre_fit_height_m"])
            for row in group
        ]
        length_scales = [float(row["optimized_descriptor_scale"]) for row in group]
        targets.append(
            Target(
                date=date,
                cultivar=cultivar,
                leaf_mean=fmean(leaf_means),
                leaf_std=fmean(leaf_stds),
                height_mean_m=fmean(target_heights),
                height_std_m=population_std(pre_fit_heights),
                initial_length_mean_scale=fmean(length_scales),
                row_count=len(group),
            )
        )
    return targets


def selected_targets(args: argparse.Namespace, targets: list[Target]) -> list[Target]:
    dates = {parse_date(date) for date in args.dates.split(",") if date.strip()} if args.dates else set()
    cultivars = {cultivar.strip().lower() for cultivar in args.cultivars.split(",") if cultivar.strip()}
    if args.worker_date:
        dates = {parse_date(args.worker_date)}
    if args.worker_cultivar:
        cultivars = {args.worker_cultivar.strip().lower()}
    selected = [
        target
        for target in targets
        if (not dates or target.date in dates) and (not cultivars or target.cultivar.lower() in cultivars)
    ]
    if not selected:
        raise ValueError("no descriptor targets selected")
    return selected


def target_base_descriptor(args: argparse.Namespace, target: Target) -> Path:
    return args.btx_descriptor if target.cultivar.lower().startswith("btx") else args.pawaga_descriptor


def descriptor_output_path(args: argparse.Namespace, target: Target) -> Path:
    return descriptor_path(args.output_asset_root, target.date, target.cultivar)


def stable_seed(base_seed: int, date: str, cultivar: str) -> int:
    digest = hashlib.sha256(f"{base_seed}|{date}|{cultivar}".encode("utf-8")).digest()
    return int.from_bytes(digest[:4], "little") & 0x7fffffff


def seed_base(rng: random.Random) -> int:
    return rng.randrange(0, 2_000_000_000)


def summarize_records(records: list[object], expected_count: int) -> dict[str, float | int]:
    heights = [float(record.height_m) for record in records]
    leaves = [float(record.main_culm_leaf_count) for record in records]
    live_leaves = [float(record.live_leaf_count) for record in records]
    leaf_areas = [float(record.leaf_area) for record in records]
    stem_areas = [float(record.stem_area) for record in records]
    geometry_count = sum(1 for record in records if bool(record.has_geometry))
    tiller_axes = [axis for record in records for axis in record.axes if int(axis.axis_id) != 0]
    tiller_leaf_ratios = [float(axis.leaf_ratio_to_main) for axis in tiller_axes]
    tiller_height_ratios = [float(axis.height_ratio_to_main) for axis in tiller_axes]
    tiller_counts = [int(record.primary_tiller_count) for record in records]
    return {
        "sample_count": len(records),
        "expected_sample_count": expected_count,
        "geometry_count": geometry_count,
        "leaf_mean": fmean(leaves) if leaves else 0.0,
        "leaf_std": population_std(leaves),
        "live_leaf_mean": fmean(live_leaves) if live_leaves else 0.0,
        "live_leaf_std": population_std(live_leaves),
        "height_mean_m": fmean(heights) if heights else 0.0,
        "height_std_m": population_std(heights),
        "leaf_area_mean_m2": fmean(leaf_areas) if leaf_areas else 0.0,
        "stem_area_mean_m2": fmean(stem_areas) if stem_areas else 0.0,
        "tiller_count_mean": fmean(tiller_counts) if tiller_counts else 0.0,
        "tiller_count_min": min(tiller_counts, default=0),
        "tiller_count_max": max(tiller_counts, default=0),
        "tiller_leaf_ratio_mean": fmean(tiller_leaf_ratios) if tiller_leaf_ratios else 0.0,
        "tiller_leaf_ratio_min": min(tiller_leaf_ratios, default=0.0),
        "tiller_leaf_ratio_max": max(tiller_leaf_ratios, default=0.0),
        "tiller_height_ratio_mean": fmean(tiller_height_ratios) if tiller_height_ratios else 0.0,
        "tiller_height_ratio_min": min(tiller_height_ratios, default=0.0),
        "tiller_height_ratio_max": max(tiller_height_ratios, default=0.0),
    }


def metric_errors(target: Target, metrics: dict[str, float | int]) -> dict[str, float]:
    leaf_mean = float(metrics["leaf_mean"])
    leaf_std = float(metrics["leaf_std"])
    height_mean = float(metrics["height_mean_m"])
    height_std = float(metrics["height_std_m"])
    return {
        "leaf_mean": abs(leaf_mean - target.leaf_mean) / max(abs(target.leaf_mean), 1.0),
        "leaf_std": abs(leaf_std - target.leaf_std) / max(abs(target.leaf_std), 0.25),
        "height_mean_m": abs(height_mean - target.height_mean_m) / max(abs(target.height_mean_m), 0.01),
        "height_std_m": abs(height_std - target.height_std_m) / max(abs(target.height_std_m), 0.01),
    }


def failed_metrics(
    target: Target,
    metrics: dict[str, float | int],
    mean_tolerance: float,
    std_tolerance: float,
) -> list[str]:
    errors = metric_errors(target, metrics)
    failures: list[str] = []
    if int(metrics["geometry_count"]) != int(metrics["expected_sample_count"]):
        failures.append("geometry_count")
    if errors["leaf_mean"] > mean_tolerance:
        failures.append("leaf_mean")
    if errors["height_mean_m"] > mean_tolerance:
        failures.append("height_mean_m")
    if errors["leaf_std"] > std_tolerance:
        failures.append("leaf_std")
    if errors["height_std_m"] > std_tolerance:
        failures.append("height_std_m")
    if not 0.85 <= float(metrics["tiller_leaf_ratio_mean"]) <= 0.95:
        failures.append("tiller_leaf_ratio_mean")
    if not 0.85 <= float(metrics["tiller_height_ratio_mean"]) <= 0.95:
        failures.append("tiller_height_ratio_mean")
    if float(metrics["tiller_leaf_ratio_min"]) < 0.75 or float(metrics["tiller_leaf_ratio_max"]) > 1.05:
        failures.append("tiller_leaf_ratio_range")
    if float(metrics["tiller_height_ratio_min"]) < 0.75 or float(metrics["tiller_height_ratio_max"]) > 1.05:
        failures.append("tiller_height_ratio_range")
    if int(metrics["tiller_count_min"]) < 3 or int(metrics["tiller_count_max"]) > 5:
        failures.append("tiller_count_range")
    return failures


def loss_for(target: Target, metrics: dict[str, float | int], mean_tolerance: float, std_tolerance: float) -> float:
    errors = metric_errors(target, metrics)
    geometry_penalty = 0.0
    if int(metrics["geometry_count"]) != int(metrics["expected_sample_count"]):
        geometry_penalty = 100.0
    return geometry_penalty + (
        errors["leaf_mean"] / max(mean_tolerance, 1e-6)
    ) ** 2 + (errors["height_mean_m"] / max(mean_tolerance, 1e-6)) ** 2 + (
        errors["leaf_std"] / max(std_tolerance, 1e-6)
    ) ** 2 + (
        errors["height_std_m"] / max(std_tolerance, 1e-6)
    ) ** 2 + ((float(metrics["tiller_leaf_ratio_mean"]) - 0.90) / 0.05) ** 2 + (
        (float(metrics["tiller_height_ratio_mean"]) - 0.90) / 0.05
    ) ** 2


def update_knobs(target: Target, metrics: dict[str, float | int], knobs: Knobs) -> Knobs:
    leaf_mean = float(metrics["leaf_mean"])
    leaf_std = float(metrics["leaf_std"])
    height_mean = float(metrics["height_mean_m"])
    height_std = float(metrics["height_std_m"])
    tiller_leaf_ratio = float(metrics["tiller_leaf_ratio_mean"])
    tiller_height_ratio = float(metrics["tiller_height_ratio_mean"])

    leaf_delta = clamp(target.leaf_mean - leaf_mean, -1.0, 1.0)
    next_leaf_mean = knobs.leaf_mean + 0.65 * leaf_delta
    if target.leaf_std <= 1e-6:
        next_leaf_std = knobs.leaf_std * 0.75
    elif leaf_std <= 1e-6:
        next_leaf_std = knobs.leaf_std + 0.25
    else:
        next_leaf_std = knobs.leaf_std * clamp(target.leaf_std / leaf_std, 0.70, 1.40)

    if height_mean <= 1e-6:
        next_length_mean = knobs.length_mean_scale * 1.15
    else:
        next_length_mean = knobs.length_mean_scale * clamp(target.height_mean_m / height_mean, 0.85, 1.15)

    if target.height_std_m <= 1e-6:
        next_length_std = knobs.length_deviation_scale * 0.75
    elif height_std <= 1e-6:
        next_length_std = knobs.length_deviation_scale + 0.10
    else:
        next_length_std = knobs.length_deviation_scale * clamp(target.height_std_m / height_std, 0.75, 1.35)

    return Knobs(
        leaf_mean=clamp(finite_or(next_leaf_mean, target.leaf_mean), 1.0, 64.0),
        leaf_std=clamp(finite_or(next_leaf_std, target.leaf_std), 0.0, 8.0),
        length_mean_scale=clamp(finite_or(next_length_mean, 1.0), 0.20, 3.0),
        length_deviation_scale=clamp(finite_or(next_length_std, 1.0), 0.0, 4.0),
        tiller_leaf_count_ratio=clamp(
            knobs.tiller_leaf_count_ratio * (0.90 / tiller_leaf_ratio) if tiller_leaf_ratio > 1e-6
            else knobs.tiller_leaf_count_ratio,
            0.5,
            1.1,
        ),
        tiller_height_ratio=clamp(
            knobs.tiller_height_ratio * (0.90 / tiller_height_ratio) if tiller_height_ratio > 1e-6
            else knobs.tiller_height_ratio,
            0.5,
            1.1,
        ),
    )


def sample_descriptor(evo: object, args: argparse.Namespace, target: Target, knobs: Knobs, count: int, seed: int) -> dict[str, float | int]:
    records = evo.SampleSorghumLsDescriptorPhenotypes(
        target_base_descriptor(args, target),
        knobs.leaf_mean,
        knobs.leaf_std,
        knobs.length_mean_scale,
        knobs.length_deviation_scale,
        count,
        seed,
        LEAF_WIDTH_SCALE_BY_DATE.get(target.date, 1.0),
        MAIN_CULM_DIAMETER_M_BY_DATE[target.date],
        knobs.tiller_leaf_count_ratio,
        knobs.tiller_height_ratio,
    )
    return summarize_records(records, count)


def initial_knobs(target: Target) -> Knobs:
    return Knobs(
        leaf_mean=target.leaf_mean,
        leaf_std=target.leaf_std,
        length_mean_scale=clamp(target.initial_length_mean_scale, 0.20, 3.0),
        length_deviation_scale=1.0,
        tiller_leaf_count_ratio=0.90,
        tiller_height_ratio=0.90,
    )


def start_project(args: argparse.Namespace) -> object:
    configure_engine_imports(args.repo_root.resolve(), args.build_dir.resolve(), args.config)
    import PyDigitalAgriculture as evo  # type: ignore

    ok = evo.RunLSystemSorghumProject(
        args.project.resolve(),
        args.runtime_package_dir.resolve(),
        args.base_scene,
        False,
    )
    if not ok:
        raise RuntimeError(f"failed to start project scene: {args.base_scene}")
    if not evo.WaitForProjectIdle(args.max_wait_frames):
        raise RuntimeError("project did not become idle before descriptor calibration")
    return evo


def calibrate_target(args: argparse.Namespace, target: Target) -> dict[str, object]:
    validate_asset_relative_path(target_base_descriptor(args, target), "base descriptor")
    output_descriptor_path = descriptor_output_path(args, target)
    validate_asset_relative_path(output_descriptor_path, "--output-asset-root")

    project_bytes = args.project.read_bytes()
    before_side_effects = collect_engine_side_effect_paths(args)
    evo = None
    try:
        evo = start_project(args)
        rng = random.Random(stable_seed(args.seed, target.date, target.cultivar))
        knobs = initial_knobs(target)
        best_knobs = Knobs(**asdict(knobs))
        best_metrics: dict[str, float | int] | None = None
        best_loss = math.inf
        consecutive_passes = 0
        iterations: list[dict[str, object]] = []
        current_seed = seed_base(rng)

        for iteration in range(min(6, max(1, args.max_iterations))):
            metrics = sample_descriptor(evo, args, target, knobs, args.iteration_sample_count, current_seed)
            current_loss = loss_for(target, metrics, args.mean_tolerance, args.std_tolerance)
            failures = failed_metrics(target, metrics, args.mean_tolerance, args.std_tolerance)
            passed = not failures
            iterations.append(
                {
                    "iteration": iteration,
                    "seed_base": current_seed,
                    "knobs": asdict(knobs),
                    "metrics": metrics,
                    "loss": current_loss,
                    "passed": passed,
                    "failed_metrics": failures,
                }
            )
            if current_loss < best_loss:
                best_loss = current_loss
                best_knobs = Knobs(**asdict(knobs))
                best_metrics = dict(metrics)
            consecutive_passes = consecutive_passes + 1 if passed else 0
            if consecutive_passes >= 2:
                break
            knobs = update_knobs(target, metrics, knobs)

        validation_rounds: list[dict[str, object]] = []
        validation_seed = seed_base(rng)
        for validation_round in range(6):
            validation_metrics = sample_descriptor(
                evo,
                args,
                target,
                best_knobs,
                args.validation_sample_count,
                validation_seed,
            )
            validation_failures = failed_metrics(
                target, validation_metrics, args.mean_tolerance, args.std_tolerance
            )
            validation_rounds.append(
                {
                    "round": validation_round,
                    "seed_base": validation_seed,
                    "knobs": asdict(best_knobs),
                    "metrics": validation_metrics,
                    "failed_metrics": validation_failures,
                }
            )
            if not validation_failures:
                break
            best_knobs = update_knobs(target, validation_metrics, best_knobs)
        success = not validation_failures
        saved = evo.SaveCalibratedSorghumLsDescriptor(
            target_base_descriptor(args, target),
            output_descriptor_path,
            best_knobs.leaf_mean,
            best_knobs.leaf_std,
            best_knobs.length_mean_scale,
            best_knobs.length_deviation_scale,
            LEAF_WIDTH_SCALE_BY_DATE.get(target.date, 1.0),
            MAIN_CULM_DIAMETER_M_BY_DATE[target.date],
            best_knobs.tiller_leaf_count_ratio,
            best_knobs.tiller_height_ratio,
        )
        if not saved:
            raise RuntimeError(f"failed to save descriptor: {output_descriptor_path}")
        migrate_fidelity_v4(
            project_assets_root(args.project) / output_descriptor_path,
            FIDELITY_STAGE_VALUES[growth_stage(target.date)],
        )
        return {
            "schema_version": 1,
            "date": target.date,
            "cultivar": target.cultivar,
            "target": asdict(target),
            "base_descriptor_path": str(target_base_descriptor(args, target).as_posix()),
            "descriptor_asset_path": str(output_descriptor_path.as_posix()),
            "best_knobs": asdict(best_knobs),
            "best_iteration_metrics": best_metrics,
            "iteration_count": len(iterations),
            "iterations": iterations,
            "validation": {
                "seed_base": validation_seed,
                "sample_count": args.validation_sample_count,
                "metrics": validation_metrics,
                "failed_metrics": validation_failures,
                "rounds": validation_rounds,
                "success": success,
            },
            "success": success,
        }
    finally:
        if not args.worker_date:
            if evo is not None:
                evo.Terminate()
            args.project.write_bytes(project_bytes)
            cleanup_new_engine_side_effects(args, before_side_effects)


def report_to_row(report: dict[str, object]) -> dict[str, object]:
    target = report["target"]
    validation = report["validation"]
    metrics = validation["metrics"]
    knobs = report["best_knobs"]
    return {
        "date": report["date"],
        "cultivar": report["cultivar"],
        "leaf_modules_mean": target["leaf_mean"],
        "leaf_modules_deviation": target["leaf_std"],
        "target_height_m": target["height_mean_m"],
        "target_height_std_m": target["height_std_m"],
        "validation_leaf_mean": metrics["leaf_mean"],
        "validation_leaf_std": metrics["leaf_std"],
        "validation_height_mean_m": metrics["height_mean_m"],
        "validation_height_std_m": metrics["height_std_m"],
        "validation_tiller_leaf_ratio_mean": metrics["tiller_leaf_ratio_mean"],
        "validation_tiller_leaf_ratio_min": metrics["tiller_leaf_ratio_min"],
        "validation_tiller_leaf_ratio_max": metrics["tiller_leaf_ratio_max"],
        "validation_tiller_height_ratio_mean": metrics["tiller_height_ratio_mean"],
        "validation_tiller_height_ratio_min": metrics["tiller_height_ratio_min"],
        "validation_tiller_height_ratio_max": metrics["tiller_height_ratio_max"],
        "length_mean_scale": knobs["length_mean_scale"],
        "length_deviation_scale": knobs["length_deviation_scale"],
        "descriptor_asset_path": report["descriptor_asset_path"],
        "success": report["success"],
        "failed_metrics": ";".join(validation["failed_metrics"]),
    }


def write_csv(path: Path, reports: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=TARGET_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(report_to_row(report) for report in reports)


def write_json(path: Path, reports: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"schema_version": 1, "descriptors": reports}, indent=2), encoding="utf-8")


def write_readme(path: Path, args: argparse.Namespace, reports: list[dict[str, object]]) -> None:
    dates = sorted({str(report["date"]) for report in reports})
    failures = [report for report in reports if not bool(report["success"])]
    text = f"""# Sorghum Calibration Reports

This folder is produced by `PythonBinding/sorghum_lsystem_calibrate_date_cultivar_descriptors.py`.

- Target manifest: `{args.target_manifest}`
- Base BTX descriptor: `{args.btx_descriptor}`
- Base Pawaga descriptor: `{args.pawaga_descriptor}`
- Output descriptor root: `{args.output_asset_root}`
- Output descriptors: one `.sorghumls` per selected cultivar/date, stored under stable growth-stage names
- Dates: {', '.join(dates)}
- Iteration batch size: {args.iteration_sample_count}
- Validation batch size: {args.validation_sample_count}
- Acceptance: means within {args.mean_tolerance:.1%}, stds within {args.std_tolerance:.1%}
- Failed descriptors: {len(failures)}

The script changes descriptor knobs: total phytomer count mean/deviation, a shared length mean scale,
a shared length deviation scale for internode, leaf blade, leaf sheath, and leaf neck distributions,
the date-specific absolute leaf widths and main-culm diameters, and v4 tiller leaf/height ratios.
It does not generate scene assets or tune target GDD.
"""
    path.write_text(text, encoding="utf-8")


def run_target_subprocesses(
    args: argparse.Namespace,
    targets: list[Target],
    output_abs_root: Path,
) -> tuple[list[dict[str, object]], list[str]]:
    reports: list[dict[str, object]] = []
    failures: list[str] = []
    script = Path(__file__).resolve()
    with tempfile.TemporaryDirectory(prefix="_descriptor_reports_", dir=str(output_abs_root)) as report_dir:
        report_dir_path = Path(report_dir)
        def run_target(target: Target) -> tuple[dict[str, object] | None, str | None]:
            worker_report = report_dir_path / f"{growth_stage(target.date)}_{target.cultivar}.json"
            command = [
                sys.executable,
                str(script),
                "--repo-root",
                str(args.repo_root.resolve()),
                "--build-dir",
                str(args.build_dir.resolve()),
                "--config",
                args.config,
                "--runtime-package-dir",
                str(args.runtime_package_dir.resolve()),
                "--project",
                str(args.project.resolve()),
                "--base-scene",
                str(args.base_scene),
                "--target-manifest",
                str(args.target_manifest.resolve()),
                "--output-asset-root",
                str(args.output_asset_root),
                "--report-output-root",
                str(args.report_output_root),
                "--btx-descriptor",
                str(args.btx_descriptor),
                "--pawaga-descriptor",
                str(args.pawaga_descriptor),
                "--iteration-sample-count",
                str(args.iteration_sample_count),
                "--validation-sample-count",
                str(args.validation_sample_count),
                "--max-iterations",
                str(args.max_iterations),
                "--mean-tolerance",
                str(args.mean_tolerance),
                "--std-tolerance",
                str(args.std_tolerance),
                "--seed",
                str(args.seed),
                "--max-wait-frames",
                str(args.max_wait_frames),
                "--worker-date",
                target.date,
                "--worker-cultivar",
                target.cultivar,
                "--worker-report",
                str(worker_report),
            ]
            result = subprocess.run(command, cwd=args.repo_root.resolve(), check=False)
            if worker_report.exists():
                report = json.loads(worker_report.read_text(encoding="utf-8"))
                if not bool(report.get("success")):
                    return report, f"{target.date} {target.cultivar}: {report['validation']['failed_metrics']}"
                return report, None
            return None, f"{target.date} {target.cultivar}: worker exited {result.returncode} without report"

        with ThreadPoolExecutor(max_workers=max(1, args.worker_count)) as executor:
            for report, failure in executor.map(run_target, targets):
                if report is not None:
                    reports.append(report)
                if failure is not None:
                    failures.append(failure)
    return reports, failures


def build_parser() -> argparse.ArgumentParser:
    repo_root = repo_root_from_script()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument("--build-dir", type=Path, default=repo_root / "out" / "build" / "vs2026-x64")
    parser.add_argument("--config", default="RelWithDebInfo")
    parser.add_argument(
        "--runtime-package-dir",
        type=Path,
        default=repo_root / "out" / "build" / "vs2026-x64" / "EvoEngine_App" / "RelWithDebInfo" / "Packages",
    )
    parser.add_argument(
        "--project",
        type=Path,
        default=repo_root / "Resources" / "DigitalAgricultureProject" / "test_lsystem_sorghum.eveproj",
    )
    parser.add_argument("--base-scene", type=Path, default=Path(BASE_SCENE))
    parser.add_argument(
        "--target-manifest",
        type=Path,
        default=repo_root / "Resources" / "DigitalAgricultureProject" / "Assets" / GENERATED_REPORT_ROOT / "field_manifest.csv",
    )
    parser.add_argument("--output-asset-root", type=Path, default=DEFAULT_OUTPUT_ASSET_ROOT)
    parser.add_argument("--btx-descriptor", type=Path, default=Path("ManualAssets/Descriptors/BTX.sorghumls"))
    parser.add_argument("--pawaga-descriptor", type=Path, default=Path("ManualAssets/Descriptors/Pawaga.sorghumls"))
    parser.add_argument("--report-output-root", type=Path, default=GENERATED_REPORT_ROOT)
    parser.add_argument("--dates", default="")
    parser.add_argument("--cultivars", default="BTX,Pawaga")
    parser.add_argument("--iteration-sample-count", type=int, default=1000)
    parser.add_argument("--validation-sample-count", type=int, default=10000)
    parser.add_argument("--max-iterations", type=int, default=30)
    parser.add_argument("--mean-tolerance", type=float, default=0.01)
    parser.add_argument("--std-tolerance", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=2601)
    parser.add_argument("--max-wait-frames", type=int, default=30000)
    parser.add_argument("--single-process", action="store_true")
    parser.add_argument("--worker-count", type=int, default=1)
    parser.add_argument("--worker-date", default="", help=argparse.SUPPRESS)
    parser.add_argument("--worker-cultivar", default="", help=argparse.SUPPRESS)
    parser.add_argument("--worker-report", type=Path, default=None, help=argparse.SUPPRESS)
    return parser


def run(args: argparse.Namespace) -> tuple[Path, list[str]]:
    validate_asset_relative_path(args.output_asset_root, "--output-asset-root")
    validate_asset_relative_path(args.report_output_root, "--report-output-root")
    targets = selected_targets(args, read_targets(args.target_manifest.resolve()))
    output_abs_root = project_assets_root(args.project) / args.output_asset_root
    output_abs_root.mkdir(parents=True, exist_ok=True)

    if not args.worker_date:
        assets_root = project_assets_root(args.project)
        for target in targets:
            descriptor = assets_root / descriptor_output_path(args, target)
            descriptor.unlink(missing_ok=True)

    if args.worker_date:
        report = calibrate_target(args, targets[0])
        if args.worker_report:
            args.worker_report.parent.mkdir(parents=True, exist_ok=True)
            args.worker_report.write_text(json.dumps(report, indent=2), encoding="utf-8")
        return output_abs_root, [] if bool(report["success"]) else [f"{report['date']} {report['cultivar']}"]

    if len(targets) > 1 and not args.single_process:
        reports, failures = run_target_subprocesses(args, targets, output_abs_root)
    else:
        reports = [calibrate_target(args, target) for target in targets]
        failures = [
            f"{report['date']} {report['cultivar']}: {report['validation']['failed_metrics']}"
            for report in reports
            if not bool(report["success"])
        ]

    reports.sort(key=lambda report: (str(report["date"]), str(report["cultivar"])))
    report_root = project_assets_root(args.project) / args.report_output_root
    report_root.mkdir(parents=True, exist_ok=True)
    write_csv(report_root / "calibration_summary.csv", reports)
    write_json(report_root / "calibration_summary.json", reports)
    write_readme(report_root / "README.md", args, reports)
    return output_abs_root, failures


def main() -> None:
    args = build_parser().parse_args()
    output, failures = run(args)
    print(f"output={output}")
    if args.worker_date:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1 if failures else 0)
    if failures:
        print("failed metrics:")
        for failure in failures:
            print(f"  {failure}")
        sys.exit(1)


if __name__ == "__main__":
    main()
