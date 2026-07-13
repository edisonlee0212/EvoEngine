"""Export generated date-height-fit L-System PARBAR scenes to Blender."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_APP = ROOT / "out" / "build" / "vs2026-x64" / "EvoEngine_App" / "RelWithDebInfo" / "DigitalAgricultureApp.exe"
DEFAULT_PROJECT = ROOT / "Resources" / "DigitalAgricultureProject" / "test_lsystem_sorghum.eveproj"
DEFAULT_SOURCE_ROOT = DEFAULT_PROJECT.parent / "Assets" / "GeneratedAssets" / "Reports"
DEFAULT_OUTPUT_DIR = ROOT / "out" / "exports" / "lsystem_date_height_fit_blender"
PREPARE_SCRIPT = ROOT / "Scripts" / "blender" / "prepare_lsystem_cycles_scene.py"
GROUND_SCRIPT = ROOT / "Scripts" / "blender" / "setup_ground_displacement_render.py"
DATE_ORDER = ("2021-07-01", "2021-07-14", "2021-08-18", "2021-08-30", "2021-09-02")


@dataclass(frozen=True)
class DateScene:
    date: str
    scene_asset_path: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--app", type=Path, default=DEFAULT_APP)
    parser.add_argument("--blender", default="blender")
    parser.add_argument(
        "--project",
        type=Path,
        help="Project to export from. Defaults to the generated project adjacent to --source-root when present.",
    )
    parser.add_argument(
        "--source-root",
        type=Path,
        help="Generated report folder. Defaults to Resources/DigitalAgricultureProject/Assets/GeneratedAssets/Reports.",
    )
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--dates", default=",".join(DATE_ORDER))
    parser.add_argument("--samples", type=int, default=512)
    parser.add_argument("--resolution-x", type=int, default=3000)
    parser.add_argument("--resolution-y", type=int, default=2000)
    parser.add_argument("--displacement-strength", type=float, default=0.5)
    parser.add_argument("--material-displacement-strength", type=float, default=0.08)
    parser.add_argument("--skip-ground-pass", action="store_true")
    parser.add_argument("--skip-render", action="store_true")
    return parser.parse_args()


def run_logged(command: list[str], log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(command, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    log_path.write_text(result.stdout, encoding="utf-8")
    if result.returncode:
        raise SystemExit(f"Command failed with exit code {result.returncode}. See {log_path}")


def selected_dates(raw: str) -> list[str]:
    dates = [date.strip() for date in raw.split(",") if date.strip()]
    return dates or list(DATE_ORDER)


def default_source_root() -> Path:
    if not (DEFAULT_SOURCE_ROOT / "field_manifest.csv").exists():
        raise FileNotFoundError(
            f"Field manifest not found: {DEFAULT_SOURCE_ROOT / 'field_manifest.csv'}\n"
            "Install the Sorghum L-System resource bundle under Resources/DigitalAgricultureProject "
            "or pass --source-root."
        )
    return DEFAULT_SOURCE_ROOT


def scene_manifest_path(args: argparse.Namespace) -> Path:
    return (args.manifest or args.source_root / "field_manifest.csv").resolve()


def read_date_scenes(path: Path, dates: list[str]) -> list[DateScene]:
    if not path.exists():
        raise FileNotFoundError(f"Height-fit manifest not found: {path}")
    by_date: dict[str, str] = {}
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = {"date", "scene_asset_path"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path} is missing columns: {', '.join(sorted(missing))}")
        for row in reader:
            date = (row.get("date") or "").strip()
            scene = (row.get("scene_asset_path") or "").strip().replace("\\", "/")
            if not date or not scene:
                continue
            existing = by_date.setdefault(date, scene)
            if existing != scene:
                raise ValueError(f"Manifest has multiple scene paths for {date}: {existing} and {scene}")
    missing_dates = [date for date in dates if date not in by_date]
    if missing_dates:
        raise ValueError(f"Manifest is missing requested dates: {', '.join(missing_dates)}")
    return [DateScene(date, by_date[date]) for date in dates]


def assets_root(project: Path) -> Path:
    return project.resolve().parent / "Assets"


def verify_scene_assets(project: Path, scenes: list[DateScene]) -> None:
    root = assets_root(project)
    missing = [scene.scene_asset_path for scene in scenes if not (root / scene.scene_asset_path).exists()]
    if missing:
        raise FileNotFoundError("Missing scene assets:\n" + "\n".join(missing))


def write_manifest(rows: list[dict[str, str]], output_dir: Path) -> None:
    if not rows:
        return
    csv_path = output_dir / "date_height_fit_blender_exports.csv"
    json_path = output_dir / "date_height_fit_blender_exports.json"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    json_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")


def export_one(args: argparse.Namespace, scene: DateScene) -> dict[str, str]:
    date_dir = args.output_dir / scene.date
    stem = f"sorghum_lsystem_4x10_parbar_{scene.date}_height_fit"
    gltf_path = date_dir / f"{stem}.gltf"
    export_manifest = date_dir / f"{stem}_manifest.json"
    blend_path = date_dir / f"{stem}_cycles.blend"
    final_blend_path = date_dir / f"{stem}_cycles_ground_displacement.blend"
    render_path = date_dir / f"{stem}_labeled.png"

    run_logged(
        [
            str(args.app),
            "--export-lsystem-blender-scene",
            "--project",
            str(args.project),
            "--rt-scene",
            scene.scene_asset_path,
            "--blender-preserve-lsystem-state",
            "--blender-output",
            str(gltf_path),
        ],
        date_dir / "evoengine_export.log",
    )

    prepare_command = [
        args.blender,
        "--background",
        "--python",
        str(PREPARE_SCRIPT),
        "--",
        "--gltf",
        str(gltf_path),
        "--manifest",
        str(export_manifest),
        "--output-blend",
        str(blend_path),
        "--samples",
        str(args.samples),
        "--resolution-x",
        str(args.resolution_x),
        "--resolution-y",
        str(args.resolution_y),
    ]
    if args.skip_ground_pass and not args.skip_render:
        prepare_command.extend(["--render-output", str(render_path)])
    run_logged(prepare_command, date_dir / "blender_prepare.log")

    if args.skip_ground_pass:
        final_blend_path = blend_path
        ground_log = ""
    else:
        ground_command = [
            args.blender,
            "--background",
            "--python",
            str(GROUND_SCRIPT),
            "--",
            "--blend",
            str(blend_path),
            "--output-blend",
            str(final_blend_path),
            "--render-output",
            str(render_path),
            "--samples",
            str(args.samples),
            "--resolution-x",
            str(args.resolution_x),
            "--resolution-y",
            str(args.resolution_y),
            "--displacement-strength",
            str(args.displacement_strength),
            "--material-displacement-strength",
            str(args.material_displacement_strength),
            "--label",
            f"{scene.date} height-fit PARBAR",
        ]
        if args.skip_render:
            ground_command.append("--skip-render")
        ground_log = str(date_dir / "blender_ground.log")
        run_logged(ground_command, date_dir / "blender_ground.log")

    expected = [gltf_path, export_manifest, blend_path, final_blend_path]
    if not args.skip_render:
        expected.append(render_path)
    missing = [str(path) for path in expected if not path.exists()]
    if missing:
        raise FileNotFoundError("Expected export outputs were not created:\n" + "\n".join(missing))

    return {
        "date": scene.date,
        "scene_asset_path": scene.scene_asset_path,
        "gltf_path": str(gltf_path),
        "evoengine_manifest_path": str(export_manifest),
        "blend_path": str(final_blend_path),
        "preview_png_path": "" if args.skip_render else str(render_path),
        "evoengine_log": str(date_dir / "evoengine_export.log"),
        "blender_prepare_log": str(date_dir / "blender_prepare.log"),
        "blender_ground_log": ground_log,
    }


def main() -> None:
    args = parse_args()
    args.app = args.app.resolve()
    args.source_root = (args.source_root or default_source_root()).resolve()
    if args.project:
        args.project = args.project.resolve()
    else:
        generated_project = args.source_root.parents[1] / DEFAULT_PROJECT.name
        args.project = generated_project if generated_project.exists() else DEFAULT_PROJECT.resolve()
    args.output_dir = args.output_dir.resolve()
    dates = selected_dates(args.dates)
    scenes = read_date_scenes(scene_manifest_path(args), dates)
    verify_scene_assets(args.project, scenes)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows = [export_one(args, scene) for scene in scenes]
    write_manifest(rows, args.output_dir)
    for row in rows:
        print(f"{row['date']}: {row['blend_path']}")
        if row["preview_png_path"]:
            print(f"{row['date']}: {row['preview_png_path']}")


if __name__ == "__main__":
    main()
