#!/usr/bin/env python3
"""Generate self-contained 10x10 inter-shadow illumination CSV handoff files."""

from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

from sorghum_asset_layout import GENERATED_SCENE_ROOT


INTER_SHADOW_CULTIVARS = ("Pawaga", "BTX")
REFERENCE_10X10_SCENE = (GENERATED_SCENE_ROOT / "Sorghum_10x10_Mature.evescene").as_posix()

NUMERIC_COLUMNS = [
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
]

INTER_SHADOW_COLUMNS = [
    "experiment_id",
    "cultivar",
    "source_scene",
    "spacing_m",
    "grid_row",
    "grid_column",
    "plant_id",
    "scenario",
    "replicate_count",
] + [f"{column}_{suffix}" for column in NUMERIC_COLUMNS for suffix in ("mean", "std")] + [
    "ray_samples",
    "ray_bounces",
    "ray_seed_base",
    "ray_seed_last",
    "geometry_seed_base",
    "geometry_seed_last",
    "geometry_seed_stride",
    "push_normal_distance_m",
    "measurement_surface",
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
        for column in NUMERIC_COLUMNS:
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
                "measurement_surface": "leaves_both_sides_and_stems_exterior_soil_mesh_renderer_context",
            }
        )
        return output


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


def finite_nonnegative(rows: list[dict[str, object]], column: str) -> bool:
    for row in rows:
        value = float(row[column])
        if not math.isfinite(value) or value < 0.0:
            return False
    return True


def validate_outputs(rows: list[dict[str, object]], args: argparse.Namespace, cultivar_count: int = len(INTER_SHADOW_CULTIVARS)) -> None:
    expected = cultivar_count * len(args.spacing_values) * 100 * 2
    if not args.smoke and len(rows) != expected:
        raise ValueError(f"10x10 row count {len(rows)} did not match expected {expected}")
    if not finite_nonnegative(rows, "total_simulated_light_interception_proxy_mean"):
        raise ValueError("10x10 handoff has invalid light interception means")


def start_project(evo: object, project: Path, runtime_package_dir: Path, max_wait_frames: int, scene: str) -> None:
    ok = evo.RunLSystemSorghumProject(project.resolve(), runtime_package_dir.resolve(), Path(scene))
    if not ok:
        raise RuntimeError(f"failed to start project scene: {scene}")
    if not evo.WaitForProjectIdle(max_wait_frames):
        raise RuntimeError("project did not become idle before timeout")
    if not evo.EnsureIlluminationSoilContext():
        raise RuntimeError(f"failed to ensure PBR soil context in scene: {scene}")
    if not evo.ValidateIlluminationContext():
        raise RuntimeError(f"scene failed illumination context validation: {scene}")


def record_values(record: object, scenario: str) -> dict[str, float]:
    context_rgb = vec3_tuple(record.total_flux)
    alone_rgb = vec3_tuple(record.isolated_total_flux)
    rgb = context_rgb if scenario == "full_context" else alone_rgb
    scalar = float(record.scalar if scenario == "full_context" else record.isolated_scalar)
    return {
        "total_simulated_light_interception_proxy": scalar,
        "simulated_light_interception_proxy_red": rgb[0],
        "simulated_light_interception_proxy_green": rgb[1],
        "simulated_light_interception_proxy_blue": rgb[2],
        "green_tissue_area_m2": float(record.area),
        "leaf_area_m2": float(record.leaf_area),
        "stem_area_m2": float(record.stem_area),
        "green_triangle_count": float(record.triangle_count),
        "leaf_triangle_count": float(record.leaf_triangle_count),
        "stem_triangle_count": float(record.stem_triangle_count),
        "plant_height_m": float(record.plant_height_m),
        "plant_position_x_m": float(record.position.x),
        "plant_position_y_m": float(record.position.y),
        "plant_position_z_m": float(record.position.z),
    }


def accumulator_key(cultivar: str, spacing: float, record: object, scenario: str) -> tuple[object, ...]:
    return cultivar, spacing, int(record.row), int(record.column), str(record.name), scenario


def base_row(cultivar: str, scene: str, spacing: float, record: object, scenario: str) -> dict[str, object]:
    return {
        "experiment_id": "10x10_inter_shadow",
        "cultivar": record.cultivar or cultivar,
        "source_scene": scene,
        "spacing_m": spacing,
        "grid_row": int(record.row),
        "grid_column": int(record.column),
        "plant_id": str(record.name),
        "scenario": scenario,
    }


def inter_shadow_rows(
    evo: object, args: argparse.Namespace, cultivar_filter: str | None = None
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    rows: list[dict[str, object]] = []
    manifest: list[dict[str, object]] = []
    cultivars = (cultivar_filter,) if cultivar_filter else INTER_SHADOW_CULTIVARS
    for cultivar in cultivars:
        project_bytes = args.project.read_bytes()
        side_effects_before = collect_default_scene_side_effects(args.project)
        try:
            start_project(evo, args.project, args.runtime_package_dir, args.max_wait_frames, args.reference_scene)

            for spacing in args.spacing_values:
                accumulators: dict[tuple[object, ...], SummaryAccumulator] = {}
                plant_count = 0
                moved_plants = 0
                for replicate in range(args.replicates):
                    geometry_seed = args.geometry_seed + replicate * args.geometry_seed_stride
                    ray_seed = args.seed + replicate
                    plant_count = int(evo.GrowSorghumLsPlantsToAdulthood(geometry_seed, cultivar))
                    moved_plants = int(evo.SetSorghumLsGridSpacing(spacing, spacing, cultivar))
                    records = evo.EstimateSorghumLsGridIllumination(
                        args.samples,
                        args.bounces,
                        args.max_triangles_per_plant,
                        args.push_normal_distance,
                        ray_seed,
                        cultivar,
                    )
                    for record in records:
                        for scenario in ("full_context", "plant_alone"):
                            key = accumulator_key(cultivar, spacing, record, scenario)
                            if key not in accumulators:
                                accumulators[key] = SummaryAccumulator(base_row(cultivar, args.reference_scene, spacing, record, scenario))
                            accumulators[key].add(record_values(record, scenario))
                rows.extend(accumulator.row(args) for accumulator in accumulators.values())
                manifest.append(
                    {
                        "experiment_id": "10x10_inter_shadow",
                        "cultivar": cultivar,
                        "source_scene": args.reference_scene,
                        "spacing_m": spacing,
                        "plant_count": plant_count,
                        "moved_plants": moved_plants,
                        "summary_rows": len(accumulators),
                        "replicate_count": args.replicates,
                        "soil_context": "PBR soil material on Ground Mesh MeshRenderer",
                    }
                )
        finally:
            try:
                evo.Terminate()
            finally:
                args.project.write_bytes(project_bytes)
                cleanup_new_default_scene_side_effects(args.project, side_effects_before)
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
        "--reference-scene",
        args.reference_scene,
        "--spacing-values",
        ",".join(str(value) for value in args.spacing_values),
        "--replicates",
        str(args.replicates),
        "--samples",
        str(args.samples),
        "--bounces",
        str(args.bounces),
        "--push-normal-distance",
        str(args.push_normal_distance),
        "--seed",
        str(args.seed),
        "--geometry-seed",
        str(args.geometry_seed),
        "--geometry-seed-stride",
        str(args.geometry_seed_stride),
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
    for cultivar in INTER_SHADOW_CULTIVARS:
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

Generated from EvoEngine L-System sorghum 10x10 inter-shadow reference scene `{args.reference_scene}`.

## Files

- `10x10_inter_shadow_plants_summary.csv`: plant-level 10x10 inter-shadow summary rows. Expected full run rows: 4000.
- `10x10_inter_shadow_manifest.csv`: source scene, spacing, and replicate settings.
- `handoff_manifest.csv`: row counts for delivered CSV files.

## Values

The light values are simulation proxies, not calibrated physical PAR units. They come from EvoEngine's CUDA/OptiX Monte Carlo illumination estimator using skydome lighting.

Each plant has two scenarios: `full_context`, where same-cultivar neighboring plants and soil are present, and `plant_alone`, where only that plant and non-plant context such as soil remain visible to the ray tracer.

The scene is repaired and validated before estimation so `Ground Mesh` uses the PBR soil material through a `MeshRenderer`.

## Run Settings

- Source project: `{args.project}`
- Reference scene: `{args.reference_scene}`
- Replicates per summary row: {args.replicates}
- Ray samples per estimate: {args.samples}
- Ray bounces: {args.bounces}
- Ray seed range: {args.seed} to {args.seed + max(0, args.replicates - 1)}
- Geometry seed range: {args.geometry_seed} to {args.geometry_seed + max(0, args.replicates - 1) * args.geometry_seed_stride}
- Push normal distance: {args.push_normal_distance} m
- 10x10 spacings: {', '.join(str(value) for value in args.spacing_values)} m

## Delivered Row Counts

- 10x10 summary rows: {row_count}
"""
    path.write_text(text, encoding="utf-8")


def write_handoff_manifest(path: Path, rows: list[dict[str, object]], manifest: list[dict[str, object]]) -> None:
    write_csv(
        path,
        ["file", "row_count", "description"],
        [
            {
                "file": "10x10_inter_shadow_plants_summary.csv",
                "row_count": len(rows),
                "description": "Plant-level 10x10 full-context and plant-alone mean/std light interception summaries.",
            },
            {
                "file": "10x10_inter_shadow_manifest.csv",
                "row_count": len(manifest),
                "description": "10x10 source scene, spacing, and replicate summary rows.",
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
        default=repo_root / "out" / "handoff" / "sorghum_10x10_grid_illumination_handoff_2026-07-09",
        type=Path,
    )
    parser.add_argument("--reference-scene", default=REFERENCE_10X10_SCENE)
    parser.add_argument("--spacing-values", default="1.0,0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1", type=parse_float_list)
    parser.add_argument("--replicates", default=10000, type=int)
    parser.add_argument("--samples", default=64, type=int)
    parser.add_argument("--bounces", default=4, type=int)
    parser.add_argument("--push-normal-distance", default=0.001, type=float)
    parser.add_argument("--seed", default=0, type=int)
    parser.add_argument("--geometry-seed", default=1_000_000, type=int)
    parser.add_argument("--geometry-seed-stride", default=1000, type=int)
    parser.add_argument("--max-triangles-per-plant", default=0, type=int)
    parser.add_argument("--max-wait-frames", default=30000, type=int)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--worker-cultivar", choices=INTER_SHADOW_CULTIVARS, default=None, help=argparse.SUPPRESS)
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
    if args.replicates <= 0:
        raise ValueError("--replicates must be positive")
    if args.geometry_seed_stride <= 0:
        raise ValueError("--geometry-seed-stride must be positive")
    if args.smoke:
        args.spacing_values = args.spacing_values[:1]
        args.replicates = min(args.replicates, 2)
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
    write_csv(args.output_dir / "10x10_inter_shadow_plants_summary.csv", INTER_SHADOW_COLUMNS, rows)
    write_csv(args.output_dir / "10x10_inter_shadow_manifest.csv", list(manifest[0]), manifest)
    write_handoff_manifest(args.output_dir / "handoff_manifest.csv", rows, manifest)
    write_readme(args.output_dir / "README.md", len(rows), args)

    print(f"output_dir={args.output_dir}")
    print(f"10x10_summary_rows={len(rows)}")


if __name__ == "__main__":
    main()
