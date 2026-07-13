#!/usr/bin/env python3
"""Append five mature 10x10 fidelity views to the 4x10 review package."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import sorghum_render_4x10_mobile_review as review


VIEWS = ("field_overview", "btx_leaf", "pawaga_leaf", "btx_basal", "pawaga_basal")
LABELS = {
    "field_overview": "10x10 field overview",
    "btx_leaf": "BTX canopy and leaf surface",
    "pawaga_leaf": "Pawaga canopy and leaf surface",
    "btx_basal": "BTX crown, culms, and tillers",
    "pawaga_basal": "Pawaga crown, culms, and tillers",
}


def worker_command(arguments: list[str]) -> list[str]:
    worker_arguments = (arg for arg in arguments if arg != "--publish-drive")
    return [sys.executable, str(Path(__file__).resolve()), *worker_arguments, "--worker"]


def append_pdf(package: Path, paths: dict[str, Path], dependencies: tuple[object, ...]) -> None:
    _Image, _ImageDraw, _ImageFont, _ImageStat, pdf_deps = dependencies
    PdfReader, ImageReader, canvas_module = pdf_deps
    source = package / "Sorghum_4x10_Scene_Review.pdf"
    with tempfile.TemporaryDirectory(prefix="sorghum_10x10_pdf_") as temporary:
        appendix = Path(temporary) / "appendix.pdf"
        canvas = canvas_module.Canvas(str(appendix), pagesize=(720.0, 960.0), pageCompression=1)
        pages = (
            ("August 30 10x10 mature field", ("field_overview",)),
            ("10x10 cultivar leaf details", ("btx_leaf", "pawaga_leaf")),
            ("10x10 basal tiller details", ("btx_basal", "pawaga_basal")),
        )
        for title, view_ids in pages:
            canvas.setFont("Helvetica-Bold", 22)
            canvas.drawString(36, 915, title)
            height = 780.0 / len(view_ids)
            for index, view_id in enumerate(view_ids):
                review.draw_pdf_image(canvas, ImageReader, paths[view_id], 36, 70 + (len(view_ids) - 1 - index) * height, 648, height - 18)
            canvas.showPage()
        canvas.save()

        from pypdf import PdfWriter

        writer = PdfWriter()
        source_reader = PdfReader(str(source))
        if len(source_reader.pages) < 16:
            raise RuntimeError("source fidelity PDF must contain at least 16 core pages")
        for page in source_reader.pages[:16]:
            writer.add_page(page)
        writer.append(str(appendix))
        merged = Path(temporary) / "merged.pdf"
        with merged.open("wb") as output:
            writer.write(output)
        merged.replace(source)
        if len(PdfReader(str(source)).pages) != 19:
            raise RuntimeError("combined fidelity PDF must contain 19 pages")


def main() -> None:
    repo = review.repo_root_from_script()
    project_root = repo / "Resources" / "DigitalAgricultureProject"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo)
    parser.add_argument("--build-dir", type=Path, default=repo / "out" / "build" / "vs2026-x64")
    parser.add_argument("--config", default="RelWithDebInfo")
    parser.add_argument("--project", type=Path, default=project_root / "test_lsystem_sorghum_10x10_overlap.eveproj")
    parser.add_argument("--runtime-package-dir", type=Path, default=repo / "out" / "build" / "vs2026-x64" / "EvoEngine_App" / "RelWithDebInfo" / "Packages")
    parser.add_argument("--scene", type=Path, default=Path("GeneratedAssets/Scenes/Sorghum_10x10_Mature.evescene"))
    parser.add_argument("--output-dir", type=Path, default=repo / "out" / "realism_review" / "sorghum_4x10" / review.DRIVE_FOLDER_NAME)
    parser.add_argument("--width", type=int, default=3840)
    parser.add_argument("--height", type=int, default=2160)
    parser.add_argument("--samples", type=int, default=128)
    parser.add_argument("--bounces", type=int, default=4)
    parser.add_argument("--seed", type=int, default=1_000_000)
    parser.add_argument("--max-wait-frames", type=int, default=30000)
    parser.add_argument("--warmup-frames", type=int, default=2)
    parser.add_argument("--publish-drive", action="store_true")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    source_scene = args.scene
    source_path = project_root / "Assets" / source_scene
    proxy_scene = source_scene.with_name(f"{source_scene.stem}_RenderProxy.evescene")
    proxy_path = project_root / "Assets" / proxy_scene
    proxy_meta = proxy_path.with_name(f"{proxy_path.name}.evefilemeta")
    if not args.worker:
        completed = subprocess.run(worker_command(sys.argv[1:]))
        proxy_path.unlink(missing_ok=True)
        proxy_meta.unlink(missing_ok=True)
        if completed.returncode:
            raise RuntimeError(f"10x10 render worker exited with code {completed.returncode}")
        if args.publish_drive:
            review.publish_to_drive(args.output_dir, review.DEFAULT_DRIVE_OUTPUT_DIR)
        print("10x10_views=5")
        print(f"output_dir={args.output_dir}")
        return
    proxy_text = source_path.read_text(encoding="utf-8")
    proxy_text = re.sub(r"(?m)^\s+leaf_vertical_subdivision_length:.*$",
                        "        leaf_vertical_subdivision_length: 0.015", proxy_text)
    proxy_text = re.sub(r"(?m)^\s+leaf_horizontal_subdivision_step:.*$",
                        "        leaf_horizontal_subdivision_step: 4", proxy_text)
    proxy_path.write_text(proxy_text, encoding="utf-8", newline="\n")
    proxy_meta.unlink(missing_ok=True)
    proxy_meta.write_text(
        "asset_extension_: .evescene\n"
        f"asset_file_name_: {proxy_path.stem}\n"
        "asset_type_name_: Scene\n"
        "asset_handle_: 16769943170723918641\n",
        encoding="utf-8",
        newline="\n",
    )
    args.scene = proxy_scene
    dependencies = review.require_artifact_dependencies()
    Image, ImageDraw, ImageFont, ImageStat, _pdf = dependencies
    review.configure_engine_imports(args.repo_root, args.build_dir, args.config)
    import PyDigitalAgriculture as evo

    if not evo.RunLSystemSorghumProject(args.project.resolve(), args.runtime_package_dir.resolve(), args.scene):
        raise RuntimeError("failed to load 10x10 scene")
    if not evo.WaitForProjectIdle(args.max_wait_frames):
        raise RuntimeError("10x10 project did not become idle")
    grown = int(evo.GrowSorghumLsPlantsToAdulthood(args.seed))
    if grown != 200:
        raise RuntimeError(f"expected 200 grown plants, got {grown}")
    if not evo.WaitForProjectIdle(args.max_wait_frames):
        raise RuntimeError("10x10 geometry did not become idle")
    records = list(evo.GetSorghumLsPlantSceneMetadata(True))
    if len(records) != 200 or any(not record.has_geometry for record in records):
        raise RuntimeError("10x10 metadata must contain 200 measurable plants")

    bounds = review.geometry_bounds(records)
    aspect = args.width / (args.height - review.LABEL_BAND_HEIGHT)
    representatives = {}
    for cultivar in ("BTX", "Pawaga"):
        selected = [record for record in records if record.cultivar == cultivar]
        front_z = max(float(record.global_position.z) for record in selected)
        front = [record for record in selected if float(record.global_position.z) >= front_z - 0.01]
        center_x = sum(float(record.global_position.x) for record in front) / len(front)
        representatives[cultivar] = min(front, key=lambda record: abs(float(record.global_position.x) - center_x))
    cameras = {
        "field_overview": review.fit_camera(bounds, (1.0, 0.60, 1.0), (0.0, 1.0, 0.0), 50.0, aspect, 1.08),
    }
    for cultivar, record in representatives.items():
        root = review.metadata_position(record)
        height = max(float(record.plant_height_m), 0.1)
        cameras[f"{cultivar.lower()}_leaf"] = review.focus_camera(
            (root[0], root[1] + 0.62 * height, root[2]), (-0.3, 0.12, 1.0), max(1.0, height), 38.0
        )
        cameras[f"{cultivar.lower()}_basal"] = review.focus_camera(
            (root[0], root[1] + 0.16 * height, root[2]), (0.25, 0.12, 1.0), max(0.8, 0.75 * height), 42.0
        )

    if not evo.ConfigureRayTracerSkydome(
        review.make_vec3(evo, review.SUN_ANGLES_DEGREES),
        review.SUN_ANGULAR_DIAMETER_RADIANS,
        1.0,
        review.make_vec3(evo, (1.0, 1.0, 1.0)),
        1.0,
        0.1,
        2.2,
    ):
        raise RuntimeError("failed to configure 10x10 skydome")

    paths: dict[str, Path] = {}
    view_records: list[dict[str, object]] = []
    for index, view_id in enumerate(VIEWS, start=1):
        review.apply_camera(evo, cameras[view_id])
        evo.LoopFrames(args.warmup_frames)
        raw = args.output_dir / "raw" / f"10x10_{view_id}.png"
        still = args.output_dir / "stills" / f"{45 + index:02d}_10x10_{view_id}.jpg"
        raw.parent.mkdir(parents=True, exist_ok=True)
        if not evo.CaptureCurrentSceneRayTraced(
            args.width, args.height - review.LABEL_BAND_HEIGHT, raw, args.samples, args.bounces, 2.2
        ):
            raise RuntimeError(f"failed to capture {view_id}")
        metrics = review.validate_image(raw, args.width, args.height - review.LABEL_BAND_HEIGHT, Image, ImageStat)
        review.label_image(
            raw,
            still,
            f"GrowthStage04 | 2021-08-30 | {LABELS[view_id]}",
            f"200 plants | seed {args.seed} | RT {args.samples} spp / {args.bounces} bounces",
            Image,
            ImageDraw,
            ImageFont,
            94,
        )
        paths[view_id] = still
        view_records.append({
            "id": view_id,
            "file": still.relative_to(args.output_dir).as_posix(),
            "raw_file": raw.relative_to(args.output_dir).as_posix(),
            "sha256": review.sha256(still),
            "raw_sha256": review.sha256(raw),
            "camera": cameras[view_id],
            **metrics,
        })

    append_pdf(args.output_dir, paths, dependencies)
    manifest_path = args.output_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["ten_by_ten"] = {
        "date": "2021-08-30",
        "scene": source_scene.as_posix(),
        "plant_count": 200,
        "geometry_seed": args.seed,
        "views": view_records,
    }
    manifest["files"] = review.package_hashes(args.output_dir)
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    os._exit(0)


if __name__ == "__main__":
    main()
