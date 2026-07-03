#!/usr/bin/env python3
"""Generate self-contained 10x10 inter-shadow illumination CSV handoff files."""

from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import sys
from pathlib import Path


INTER_SHADOW_SCENES = {
    "Pawaga": "2026-07-01_Sorghum/2026-06-30_Sorghum_LSystem_10x10_Pawaga.evescene",
    "BTX": "2026-07-01_Sorghum/2026-06-30_Sorghum_LSystem_10x10_BTX.evescene",
}

INTER_SHADOW_COLUMNS = [
    "experiment_id",
    "cultivar",
    "source_scene",
    "spacing_m",
    "grid_row",
    "grid_column",
    "scenario",
    "total_simulated_light_interception_proxy",
    "simulated_light_interception_proxy_red",
    "simulated_light_interception_proxy_green",
    "simulated_light_interception_proxy_blue",
    "green_tissue_area_m2",
    "leaf_area_m2",
    "stem_area_m2",
    "green_triangle_count",
    "leaf_triangle_count",
    "stem_triangle_count",
    "plant_height_m",
    "plant_position_x_m",
    "plant_position_y_m",
    "plant_position_z_m",
    "ray_samples",
    "ray_bounces",
    "ray_seed",
    "push_normal_distance_m",
    "measurement_surface",
]


def repo_root_from_script() -> Path:
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


def parse_float_list(value: str) -> list[float]:
    return [float(item.strip()) for item in value.split(",") if item.strip()]


def vec3_tuple(value: object) -> tuple[float, float, float]:
    return float(value.x), float(value.y), float(value.z)


def write_csv(path: Path, columns: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def tail_text(path: Path, line_count: int = 80) -> str:
    if not path.exists():
        return ""
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(lines[-line_count:])


def finite_nonnegative(rows: list[dict[str, object]], column: str) -> bool:
    for row in rows:
        value = float(row[column])
        if not math.isfinite(value) or value < 0.0:
            return False
    return True


def validate_outputs(rows: list[dict[str, object]], args: argparse.Namespace, cultivar_count: int = len(INTER_SHADOW_SCENES)) -> None:
    expected = cultivar_count * len(args.spacing_values) * 100 * 2
    if not args.smoke and len(rows) != expected:
        raise ValueError(f"10x10 row count {len(rows)} did not match expected {expected}")
    if not finite_nonnegative(rows, "total_simulated_light_interception_proxy"):
        raise ValueError("10x10 handoff has invalid light interception values")


def start_project(evo: object, project: Path, runtime_package_dir: Path, max_wait_frames: int, scene: str) -> None:
    ok = evo.RunLSystemSorghumProject(project.resolve(), runtime_package_dir.resolve(), Path(scene))
    if not ok:
        raise RuntimeError(f"failed to start project scene: {scene}")
    if not evo.WaitForProjectIdle(max_wait_frames):
        raise RuntimeError("project did not become idle before timeout")


def inter_shadow_rows(
    evo: object, args: argparse.Namespace, cultivar_filter: str | None = None
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    rows: list[dict[str, object]] = []
    manifest: list[dict[str, object]] = []
    scenes = {cultivar_filter: INTER_SHADOW_SCENES[cultivar_filter]} if cultivar_filter else INTER_SHADOW_SCENES
    for cultivar, scene in scenes.items():
        start_project(evo, args.project, args.runtime_package_dir, args.max_wait_frames, scene)
        plant_count = int(evo.GrowSorghumLsPlantsToAdulthood())

        for spacing in args.spacing_values:
            moved_plants = int(evo.SetSorghumLsGridSpacing(spacing, spacing))
            records = evo.EstimateSorghumLsGridIllumination(
                args.samples,
                args.bounces,
                args.max_triangles_per_plant,
                args.push_normal_distance,
                args.seed,
            )
            for record in records:
                context_rgb = vec3_tuple(record.total_flux)
                alone_rgb = vec3_tuple(record.isolated_total_flux)
                base = {
                    "experiment_id": "10x10_inter_shadow",
                    "cultivar": record.cultivar or cultivar,
                    "source_scene": scene,
                    "spacing_m": spacing,
                    "grid_row": int(record.row),
                    "grid_column": int(record.column),
                    "green_tissue_area_m2": float(record.area),
                    "leaf_area_m2": float(record.leaf_area),
                    "stem_area_m2": float(record.stem_area),
                    "green_triangle_count": int(record.triangle_count),
                    "leaf_triangle_count": int(record.leaf_triangle_count),
                    "stem_triangle_count": int(record.stem_triangle_count),
                    "plant_height_m": float(record.plant_height_m),
                    "plant_position_x_m": float(record.position.x),
                    "plant_position_y_m": float(record.position.y),
                    "plant_position_z_m": float(record.position.z),
                    "ray_samples": args.samples,
                    "ray_bounces": args.bounces,
                    "ray_seed": args.seed,
                    "push_normal_distance_m": args.push_normal_distance,
                    "measurement_surface": "leaves_both_sides_and_stems_exterior",
                }
                rows.append(
                    base
                    | {
                        "scenario": "full_context",
                        "total_simulated_light_interception_proxy": float(record.scalar),
                        "simulated_light_interception_proxy_red": context_rgb[0],
                        "simulated_light_interception_proxy_green": context_rgb[1],
                        "simulated_light_interception_proxy_blue": context_rgb[2],
                    }
                )
                rows.append(
                    base
                    | {
                        "scenario": "plant_alone",
                        "total_simulated_light_interception_proxy": float(record.isolated_scalar),
                        "simulated_light_interception_proxy_red": alone_rgb[0],
                        "simulated_light_interception_proxy_green": alone_rgb[1],
                        "simulated_light_interception_proxy_blue": alone_rgb[2],
                    }
                )
            manifest.append(
                {
                    "experiment_id": "10x10_inter_shadow",
                    "cultivar": cultivar,
                    "source_scene": scene,
                    "spacing_m": spacing,
                    "plant_count": plant_count,
                    "moved_plants": moved_plants,
                    "rows": len(records) * 2,
                }
            )
        evo.Terminate()
    return rows, manifest


def worker_command(args: argparse.Namespace, cultivar: str, rows_csv: Path, manifest_csv: Path) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "--repo-root",
        str(args.repo_root),
        "--build-dir",
        str(args.build_dir),
        "--project",
        str(args.project),
        "--config",
        args.config,
        "--runtime-package-dir",
        str(args.runtime_package_dir),
        "--output-dir",
        str(args.output_dir),
        "--spacing-values",
        ",".join(str(value) for value in args.spacing_values),
        "--samples",
        str(args.samples),
        "--bounces",
        str(args.bounces),
        "--push-normal-distance",
        str(args.push_normal_distance),
        "--seed",
        str(args.seed),
        "--max-triangles-per-plant",
        str(args.max_triangles_per_plant),
        "--max-wait-frames",
        str(args.max_wait_frames),
        "--worker-cultivar",
        cultivar,
        "--worker-rows-csv",
        str(rows_csv),
        "--worker-manifest-csv",
        str(manifest_csv),
    ]


def inter_shadow_rows_with_workers(args: argparse.Namespace) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    worker_dir = args.output_dir.parent / f".{args.output_dir.name}_workers"
    worker_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    manifest: list[dict[str, object]] = []
    for cultivar in INTER_SHADOW_SCENES:
        rows_csv = worker_dir / f"{cultivar}_rows.csv"
        manifest_csv = worker_dir / f"{cultivar}_manifest.csv"
        log_path = worker_dir / f"{cultivar}_worker.log"
        with log_path.open("w", encoding="utf-8", errors="replace") as log:
            result = subprocess.run(
                worker_command(args, cultivar, rows_csv, manifest_csv),
                cwd=args.repo_root,
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
            )
        if result.returncode != 0:
            raise RuntimeError(f"{cultivar} worker failed with exit code {result.returncode}. Log: {log_path}\n{tail_text(log_path)}")
        rows.extend(read_csv(rows_csv))
        manifest.extend(read_csv(manifest_csv))
        print(f"{cultivar}: rows={len(read_csv(rows_csv))} log={log_path}")
    return rows, manifest


def write_readme(path: Path, row_count: int, args: argparse.Namespace) -> None:
    text = f"""# 10x10 Inter-Shadow Illumination CSV Handoff

Generated from EvoEngine L-System sorghum 10x10 inter-shadow scenes.

## Files

- `10x10_inter_shadow_plants_long.csv`: plant-level 10x10 inter-shadow results. Expected full run rows: 4000.
- `10x10_inter_shadow_manifest.csv`: source scene and spacing rows generated for the 10x10 file.
- `handoff_manifest.csv`: row counts for delivered CSV files.

## Values

The light values are simulation proxies, not calibrated physical PAR units. They come from EvoEngine's CUDA/OptiX Monte Carlo illumination estimator using skydome lighting. Treat them as internally comparable simulated light estimates unless calibrated later against field PAR measurements.

`total_simulated_light_interception_proxy` is an area-integrated plant total for green tissue. Leaves are measured on both sides; stems/internodes are measured on their exterior surface only. Each plant has two scenarios: `full_context`, where neighboring plants are present, and `plant_alone`, where only that plant remains visible to the ray tracer.

## Run Settings

- Source project: `{args.project}`
- Ray samples per estimate: {args.samples}
- Ray bounces: {args.bounces}
- Ray seed: {args.seed}
- Push normal distance: {args.push_normal_distance} m
- 10x10 spacings: {', '.join(str(value) for value in args.spacing_values)} m

## Delivered Row Counts

- 10x10 rows: {row_count}
"""
    path.write_text(text, encoding="utf-8")


def write_handoff_manifest(path: Path, rows: list[dict[str, object]], manifest: list[dict[str, object]]) -> None:
    write_csv(
        path,
        ["file", "row_count", "description"],
        [
            {
                "file": "10x10_inter_shadow_plants_long.csv",
                "row_count": len(rows),
                "description": "Plant-level 10x10 full-context and plant-alone light interception proxy rows.",
            },
            {
                "file": "10x10_inter_shadow_manifest.csv",
                "row_count": len(manifest),
                "description": "10x10 source scene and spacing rows.",
            },
        ],
    )


def build_parser() -> argparse.ArgumentParser:
    repo_root = repo_root_from_script()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", default=repo_root, type=Path)
    parser.add_argument("--build-dir", default=repo_root / "out" / "build" / "vs2026-x64", type=Path)
    parser.add_argument(
        "--project",
        default=repo_root / "Resources" / "DigitalAgricultureProject" / "test_lsystem_sorghum_10x10_overlap.eveproj",
        type=Path,
    )
    parser.add_argument("--config", default="RelWithDebInfo")
    parser.add_argument(
        "--runtime-package-dir",
        default=repo_root / "out" / "build" / "vs2026-x64" / "EvoEngine_App" / "RelWithDebInfo" / "Packages",
        type=Path,
    )
    parser.add_argument(
        "--output-dir",
        default=repo_root / "out" / "handoff" / "illumination_csv_handoff_2026-06-30",
        type=Path,
    )
    parser.add_argument("--spacing-values", default="1.0,0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1", type=parse_float_list)
    parser.add_argument("--samples", default=64, type=int)
    parser.add_argument("--bounces", default=4, type=int)
    parser.add_argument("--push-normal-distance", default=0.001, type=float)
    parser.add_argument("--seed", default=0, type=int)
    parser.add_argument("--max-triangles-per-plant", default=0, type=int)
    parser.add_argument("--max-wait-frames", default=30000, type=int)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--worker-cultivar", choices=tuple(INTER_SHADOW_SCENES), default=None, help=argparse.SUPPRESS)
    parser.add_argument("--worker-rows-csv", type=Path, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--worker-manifest-csv", type=Path, default=None, help=argparse.SUPPRESS)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    args.repo_root = args.repo_root.resolve()
    args.build_dir = args.build_dir.resolve()
    args.project = args.project.resolve()
    args.runtime_package_dir = args.runtime_package_dir.resolve()
    args.output_dir = args.output_dir.resolve()
    if args.smoke:
        args.spacing_values = args.spacing_values[:1]
        args.samples = min(args.samples, 1)
        args.max_triangles_per_plant = args.max_triangles_per_plant or 64

    if args.worker_cultivar:
        configure_engine_imports(args.repo_root, args.build_dir, args.config)
        import PyDigitalAgriculture as evo

        try:
            rows, manifest = inter_shadow_rows(evo, args, args.worker_cultivar)
        finally:
            try:
                evo.Terminate()
            except Exception:
                pass
        validate_outputs(rows, args, cultivar_count=1)
        if not args.worker_rows_csv or not args.worker_manifest_csv:
            raise ValueError("worker CSV output paths are required")
        write_csv(args.worker_rows_csv, INTER_SHADOW_COLUMNS, rows)
        write_csv(args.worker_manifest_csv, list(manifest[0]), manifest)
        print(f"worker_rows={len(rows)}")
        return

    rows, manifest = inter_shadow_rows_with_workers(args)

    validate_outputs(rows, args)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "10x10_inter_shadow_plants_long.csv", INTER_SHADOW_COLUMNS, rows)
    write_csv(args.output_dir / "10x10_inter_shadow_manifest.csv", list(manifest[0]), manifest)
    write_handoff_manifest(args.output_dir / "handoff_manifest.csv", rows, manifest)
    write_readme(args.output_dir / "README.md", len(rows), args)

    print(f"output_dir={args.output_dir}")
    print(f"10x10_rows={len(rows)}")


if __name__ == "__main__":
    main()
