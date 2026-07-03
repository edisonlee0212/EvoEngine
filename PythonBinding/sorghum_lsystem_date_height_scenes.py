#!/usr/bin/env python3
"""Generate date-targeted 4x10 L-System PARBAR scenes from empirical height targets.

By default this creates an isolated generated project under out/ so batch outputs do not
pollute the hand-edited Resources/*/Assets trees. Use --write-project-assets only for an
intentional promotion into a live project.
"""

from __future__ import annotations

import argparse
import csv
import os
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path


BASE_SCENE = "2026-07-01_Sorghum/2026-06-04_Sorghum_LSystem.evescene"
GENERATED_PROJECT_ROOT = Path("out") / "generated_projects" / "sorghum_lsystem_date_height_scenes"
LEAF_TARGETS = {
    "2021-07-01": {"mean": 13.06666667, "stdev": 1.437590577, "rounded": 13},
    "2021-07-14": {"mean": 15.33333333, "stdev": 1.632993162, "rounded": 15},
    "2021-08-18": {"mean": 17.4, "stdev": 0.828078671, "rounded": 17},
    "2021-08-30": {"mean": 18.0, "stdev": 1.0, "rounded": 18},
    "2021-09-02": {"mean": 18.2, "stdev": 1.082325539, "rounded": 18},
}
HEIGHT_TARGETS_M = {
    "2021-07-01": {"BTX": 0.5440000295639038, "Pawaga": 0.5479999780654907},
    "2021-07-14": {"BTX": 0.9649999737739563, "Pawaga": 0.9539999961853027},
    "2021-08-18": {"BTX": 1.8350000381469727, "Pawaga": 1.8799999952316284},
    "2021-08-30": {"BTX": 2.1570000648498535, "Pawaga": 2.492000102996826},
    "2021-09-02": {"BTX": 2.002000093460083, "Pawaga": 2.2950000762939453},
}
LEAF_THICKNESS_BY_DATE_M = {
    "2021-07-01": 0.00015,
    "2021-07-14": 0.00020,
    "2021-08-18": 0.00030,
    "2021-08-30": 0.00040,
    "2021-09-02": 0.00045,
}
DATE_ORDER = tuple(LEAF_TARGETS)
MANIFEST_COLUMNS = [
    "date",
    "cultivar",
    "plant_name",
    "base_plant_name",
    "cluster_index",
    "cluster_size",
    "cluster_offset_x_m",
    "cluster_offset_z_m",
    "cluster_offset_radius_m",
    "clump_mean_height_m",
    "target_height_m",
    "pre_fit_height_m",
    "final_height_m",
    "height_error_m",
    "optimized_descriptor_scale",
    "per_plant_scale",
    "leaf_modules_mean",
    "leaf_modules_deviation",
    "leaf_thickness_m",
    "leaf_count",
    "descriptor_asset_path",
    "scene_asset_path",
    "middle_parbar_top_elevation_m",
]


def repo_root_from_script() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "CMakeLists.txt").exists() and (parent / "Resources" / "DigitalAgricultureProject").exists():
            return parent
    return Path(__file__).resolve().parents[1]


def unique_run_name() -> str:
    return f"DateHeightFit_{datetime.now().strftime('%Y%m%d_%H%M%S')}"


def project_assets_root(project_path: Path) -> Path:
    return project_path.resolve().parent / "Assets"


def validate_asset_relative_path(path: Path, option_name: str) -> None:
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"{option_name} must be a child path relative to the project Assets folder")


def write_generated_project(project_path: Path) -> None:
    project_path.parent.mkdir(parents=True, exist_ok=True)
    project_path.write_text(
        "application_name: DigitalAgriculture\n"
        "preferred_editor: EvoEngineEditor\n"
        "startup_runtime_packages:\n"
        "  - DigitalAgriculture\n"
        "  - LSystem\n"
        "start_scene_handle: 0\n",
        encoding="utf-8",
    )


def resolve_base_assets_root(repo_root: Path, project_path: Path, base_scene: Path) -> Path:
    validate_asset_relative_path(base_scene, "--base-scene")
    candidates = [
        project_assets_root(project_path),
        repo_root / "Resources" / "LSystemProject" / "Assets",
        repo_root / "Resources" / "DigitalAgricultureProject" / "Assets",
    ]
    seen: set[Path] = set()
    for candidate in candidates:
        candidate = candidate.resolve()
        if candidate in seen:
            continue
        seen.add(candidate)
        if (candidate / base_scene).exists():
            return candidate
    searched = "\n  ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f"base scene not found: {base_scene}\nSearched:\n  {searched}")


def copy_base_scene_assets(source_assets: Path, target_assets: Path, base_scene: Path) -> None:
    def ignore_generated_date_fits(_: str, names: list[str]) -> set[str]:
        return {name for name in names if name.startswith("DateHeightFit")}

    def copy_if_exists(relative_path: Path) -> None:
        source = source_assets / relative_path
        target = target_assets / relative_path
        if not source.exists():
            return
        if source.is_dir():
            shutil.copytree(source, target, dirs_exist_ok=True)
            return
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    target_assets.mkdir(parents=True, exist_ok=True)
    source_scene = source_assets / base_scene
    if base_scene.parent == Path("."):
        target_scene = target_assets / base_scene
        target_scene.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_scene, target_scene)
        return

    source_dir = source_assets / base_scene.parent
    target_dir = target_assets / base_scene.parent
    shutil.copytree(source_dir, target_dir, dirs_exist_ok=True, ignore=ignore_generated_date_fits)
    folder_meta = source_assets / f"{base_scene.parts[0]}.evefoldermeta"
    if folder_meta.exists():
        shutil.copy2(folder_meta, target_assets / folder_meta.name)
    if base_scene.parts[0] == "2026-07-01_Sorghum":
        copy_if_exists(Path("2026-06-04_Sorghum") / "bigSoil.soil")
        copy_if_exists(Path("2026-06-04_Sorghum") / "bigSoil.soil.evefilemeta")
        copy_if_exists(Path("2026-06-04_Sorghum") / "soil_PBRv2")
        copy_if_exists(Path("2026-06-04_Sorghum.evefoldermeta"))


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


def parse_date(value: str) -> str:
    value = value.strip()
    for fmt in ("%Y-%m-%d", "%m/%d/%y", "%m/%d/%Y"):
        try:
            return datetime.strptime(value, fmt).date().isoformat()
        except ValueError:
            pass
    raise ValueError(f"unsupported date format: {value}")


def find_column(columns: list[str], *tokens: str) -> str:
    lowered = {column: column.lower() for column in columns}
    for column, text in lowered.items():
        if all(token.lower() in text for token in tokens):
            return column
    raise ValueError(f"missing CSV column containing tokens: {tokens}")


def read_height_targets(path: Path | None) -> dict[str, dict[str, float]]:
    if path is None:
        return {date: dict(targets) for date, targets in HEIGHT_TARGETS_M.items()}
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise ValueError(f"height CSV has no header: {path}")
        date_column = find_column(reader.fieldnames, "date")
        btx_column = find_column(reader.fieldnames, "btx", "height")
        pawaga_column = find_column(reader.fieldnames, "pawaga", "height")
        targets: dict[str, dict[str, float]] = {}
        for row in reader:
            date = parse_date(row[date_column])
            targets[date] = {
                "BTX": float(row[btx_column]),
                "Pawaga": float(row[pawaga_column]),
            }
    missing = [date for date in DATE_ORDER if date not in targets]
    if missing:
        raise ValueError(f"height CSV is missing required target dates: {', '.join(missing)}")
    return targets


def record_to_row(record: object) -> dict[str, object]:
    final_height = float(record.final_height_m)
    target_height = float(record.target_height_m)
    return {
        "date": record.date,
        "cultivar": record.cultivar,
        "plant_name": record.plant_name,
        "base_plant_name": record.base_plant_name,
        "cluster_index": int(record.cluster_index),
        "cluster_size": int(record.cluster_size),
        "cluster_offset_x_m": float(record.cluster_offset_x_m),
        "cluster_offset_z_m": float(record.cluster_offset_z_m),
        "cluster_offset_radius_m": float(record.cluster_offset_radius_m),
        "clump_mean_height_m": float(record.clump_mean_height_m),
        "target_height_m": target_height,
        "pre_fit_height_m": float(record.pre_fit_height_m),
        "final_height_m": final_height,
        "height_error_m": final_height - target_height,
        "optimized_descriptor_scale": float(record.optimized_descriptor_scale),
        "per_plant_scale": float(record.per_plant_scale),
        "leaf_modules_mean": float(record.leaf_modules_mean),
        "leaf_modules_deviation": float(record.leaf_modules_deviation),
        "leaf_thickness_m": float(record.leaf_thickness_m),
        "leaf_count": int(record.leaf_count),
        "descriptor_asset_path": record.descriptor_asset_path,
        "scene_asset_path": record.scene_asset_path,
        "middle_parbar_top_elevation_m": float(record.middle_parbar_top_elevation_m),
    }


def assert_height_tolerance(rows: list[dict[str, object]], tolerance_m: float) -> None:
    failures = [
        row
        for row in rows
        if abs(float(row["height_error_m"])) > tolerance_m
    ]
    if failures:
        worst = max(failures, key=lambda row: abs(float(row["height_error_m"])))
        raise RuntimeError(
            f"{len(failures)} plants missed height tolerance; worst is "
            f"{worst['plant_name']} on {worst['date']} at {float(worst['height_error_m']):.6f} m"
        )


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=MANIFEST_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_readme(path: Path, args: argparse.Namespace, rows: list[dict[str, object]]) -> None:
    dates = sorted({row["date"] for row in rows})
    max_error = max((abs(float(row["height_error_m"])) for row in rows), default=0.0)
    height_source = str(args.height_csv) if args.height_csv else "embedded accepted target heights"
    text = f"""# L-System Date Height-Fit PARBAR Scenes

Generated L-System-only 4x10 PARBAR scenes from empirical target heights.

- Base scene: `{args.base_scene}`
- Height source: `{height_source}`
- Generated dates: {', '.join(dates)}
- Plant rows: {len(rows)}
- Max absolute final height error: {max_error:.6f} m
- Optimizer samples per date/cultivar: {args.optimizer_sample_count}
- Exact-fit tolerance: {args.tolerance_m} m
- Leaf thickness schedule: {', '.join(f'{date}={LEAF_THICKNESS_BY_DATE_M[date]:.5f} m' for date in dates)}
- Cluster size per original plant: {args.cluster_min_count}-{args.cluster_max_count} real L-System plants
- Cluster radius: {args.cluster_radius_m:.3f} m

Each generated scene preserves the base camera, soil, textures, lighting, plant positions, and fixed PARBAR bars.
Each original plant position is retained as a clump anchor, with additional `_cluster_*` L-System plant entities placed nearby.
The plants reference generated per-plant `.sorghumls` descriptors. No `.sg` assets are used.
Middle PARBAR bars are moved after final plant fitting to two-thirds of the equal-weighted average clump height.
"""
    path.write_text(text, encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    repo_root = repo_root_from_script()
    run_name = unique_run_name()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument("--run-name", default=run_name)
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=repo_root / "out" / "build" / "vs2026-x64",
    )
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
        "--generated-project-root",
        type=Path,
        default=None,
        help="Directory for the generated scratch project used by default.",
    )
    parser.add_argument(
        "--write-project-assets",
        action="store_true",
        help="Opt in to writing generated scenes/descriptors into --project's live Assets folder.",
    )
    parser.add_argument(
        "--height-csv",
        type=Path,
        default=None,
        help="Optional empirical target-height CSV. Defaults to the accepted embedded target heights.",
    )
    parser.add_argument(
        "--output-asset-root",
        type=Path,
        default=None,
        help=(
            "Generated output folder relative to the active Assets folder. Defaults to Generated for scratch "
            "projects, or 2026-06-04_Sorghum_LSystem/<run-name> when --write-project-assets is used."
        ),
    )
    parser.add_argument("--dates", default=",".join(DATE_ORDER))
    parser.add_argument("--optimizer-sample-count", type=int, default=240)
    parser.add_argument("--tolerance-m", type=float, default=0.005)
    parser.add_argument("--max-fit-iterations", type=int, default=6)
    parser.add_argument("--cluster-min-count", type=int, default=3)
    parser.add_argument("--cluster-max-count", type=int, default=5)
    parser.add_argument("--cluster-radius-m", type=float, default=0.05)
    parser.add_argument("--max-wait-frames", type=int, default=30000)
    parser.add_argument("--single-process", action="store_true")
    parser.add_argument("--worker-date", default="", help=argparse.SUPPRESS)
    parser.add_argument("--worker-manifest", type=Path, default=None, help=argparse.SUPPRESS)
    return parser


def selected_dates(args: argparse.Namespace) -> list[str]:
    dates = [parse_date(date) for date in args.dates.split(",") if date.strip()]
    if args.worker_date:
        dates = [parse_date(args.worker_date)]
    for date in dates:
        if date not in LEAF_TARGETS:
            raise ValueError(f"date has no leaf target: {date}")
    return dates


def fit_date(args: argparse.Namespace, date: str, output_asset_root: Path, height_targets: dict[str, dict[str, float]]) -> list[dict[str, object]]:
    repo_root = args.repo_root.resolve()
    configure_engine_imports(repo_root, args.build_dir.resolve(), args.config)
    import PyDigitalAgriculture as evo  # type: ignore

    try:
        ok = evo.RunLSystemSorghumProject(
            args.project.resolve(),
            args.runtime_package_dir.resolve(),
            args.base_scene,
        )
        if not ok:
            raise RuntimeError(f"failed to start project scene: {args.base_scene}")
        if not evo.WaitForProjectIdle(args.max_wait_frames):
            raise RuntimeError("project did not become idle before date fitting")

        leaf = LEAF_TARGETS[date]
        records = evo.FitSorghumLsDateHeightScene(
            date,
            height_targets[date],
            float(leaf["mean"]),
            float(leaf["stdev"]),
            output_asset_root / "Descriptors" / date,
            output_asset_root / f"Sorghum_LSystem_4x10_PARBAR_{date}_HeightFit.evescene",
            args.optimizer_sample_count,
            args.tolerance_m,
            args.max_fit_iterations,
            LEAF_THICKNESS_BY_DATE_M[date],
            args.cluster_min_count,
            args.cluster_max_count,
            args.cluster_radius_m,
        )
        min_records = 40 * max(1, args.cluster_min_count)
        max_records = 40 * max(max(1, args.cluster_min_count), args.cluster_max_count)
        if not min_records <= len(records) <= max_records:
            raise RuntimeError(
                f"{date} produced {len(records)} fitted records, expected {min_records}-{max_records}"
            )
        rows = [record_to_row(record) for record in records]
        assert_height_tolerance(rows, args.tolerance_m)
        return rows
    finally:
        evo.Terminate()


def run_date_subprocesses(
    args: argparse.Namespace,
    dates: list[str],
    output_asset_root: Path,
    output_abs_root: Path,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    script = Path(__file__).resolve()
    with tempfile.TemporaryDirectory(prefix="_date_manifests_", dir=str(output_abs_root)) as manifest_dir:
        worker_manifest_dir = Path(manifest_dir)
        for date in dates:
            worker_manifest = worker_manifest_dir / f"{date}.csv"
            height_csv_args = ["--height-csv", str(args.height_csv)] if args.height_csv else []
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
                *height_csv_args,
                "--output-asset-root",
                str(output_asset_root),
                "--run-name",
                args.run_name,
                "--worker-date",
                date,
                "--worker-manifest",
                str(worker_manifest),
                "--optimizer-sample-count",
                str(args.optimizer_sample_count),
                "--tolerance-m",
                str(args.tolerance_m),
                "--max-fit-iterations",
                str(args.max_fit_iterations),
                "--cluster-min-count",
                str(args.cluster_min_count),
                "--cluster-max-count",
                str(args.cluster_max_count),
                "--cluster-radius-m",
                str(args.cluster_radius_m),
                "--max-wait-frames",
                str(args.max_wait_frames),
            ]
            subprocess.run(command, cwd=args.repo_root.resolve(), check=True)
            with worker_manifest.open(newline="") as stream:
                rows.extend(csv.DictReader(stream))
    return rows


def default_output_asset_root(args: argparse.Namespace) -> Path:
    if args.write_project_assets:
        return Path("2026-07-01_Sorghum") / args.run_name
    return Path("Generated")


def prepare_project_for_output(args: argparse.Namespace, output_asset_root: Path) -> tuple[Path, Path]:
    validate_asset_relative_path(output_asset_root, "--output-asset-root")
    project_path = args.project.resolve()
    if args.worker_date or args.write_project_assets:
        return project_path, project_assets_root(project_path)

    generated_root = (
        args.generated_project_root
        or args.repo_root.resolve() / GENERATED_PROJECT_ROOT / args.run_name
    ).resolve()
    generated_project = generated_root / project_path.name
    generated_assets = generated_root / "Assets"
    source_assets = resolve_base_assets_root(args.repo_root.resolve(), project_path, args.base_scene)
    write_generated_project(generated_project)
    copy_base_scene_assets(source_assets, generated_assets, args.base_scene)
    return generated_project, generated_assets


def run(args: argparse.Namespace) -> Path:
    output_asset_root = args.output_asset_root or default_output_asset_root(args)
    args.project, assets_root = prepare_project_for_output(args, output_asset_root)
    output_abs_root = assets_root / output_asset_root
    if output_abs_root.exists() and not args.worker_date:
        raise FileExistsError(f"output asset root already exists: {output_abs_root}")

    height_targets = read_height_targets(args.height_csv)
    dates = selected_dates(args)
    if args.worker_date:
        manifest_rows = fit_date(args, dates[0], output_asset_root, height_targets)
        if args.worker_manifest:
            write_csv(args.worker_manifest, manifest_rows)
        return output_abs_root

    output_abs_root.mkdir(parents=True, exist_ok=True)
    if len(dates) > 1 and not args.single_process:
        manifest_rows = run_date_subprocesses(args, dates, output_asset_root, output_abs_root)
    else:
        manifest_rows = []
        for date in dates:
            manifest_rows.extend(fit_date(args, date, output_asset_root, height_targets))

    write_csv(output_abs_root / "height_fit_manifest.csv", manifest_rows)
    write_readme(output_abs_root / "README.md", args, manifest_rows)
    return output_abs_root


def main() -> None:
    args = build_parser().parse_args()
    output = run(args)
    if not args.worker_date:
        print(f"project={args.project}")
        print(f"output={output}")


if __name__ == "__main__":
    main()
