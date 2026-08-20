#!/usr/bin/env python3
"""Render the Aug. 11 6x10 endpoint through Vulkan RT and compare it to CUDA/OptiX.

The preserved CUDA/OptiX manifest is the source of truth for all 18 world-space
camera transforms.  This script reuses those transforms verbatim, captures the
same saved endpoint scene through EvoEngine's Vulkan RayTracing camera mode,
applies the original presentation grade, and builds paired review sheets.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any


EXPERIMENT_ID = "Sorghum2026_6x10_2026-08-11"
PRESENTATION_GRADE = {
    "brightness": 0.94,
    "contrast": 0.95,
    "saturation": 0.90,
    "red_gain": 1.015,
    "green_gain": 1.0,
    "blue_gain": 0.965,
}

VULKAN_LEGACY_RADIOMETRIC_SCALE = 6.0

OUTDOOR_LIGHTING_PROFILE = {
    "sun_euler_degrees": [55.0, 30.0, 0.0],
    "sun_angular_diameter_radians": 0.012,
    "cuda_source_sun_intensity": 0.92,
    "sun_intensity": 0.92 * VULKAN_LEGACY_RADIOMETRIC_SCALE,
    "sun_color": [1.0, 0.975, 0.94],
    "sky_light_intensity": 0.92,
    "cuda_source_ambient_light_intensity": 0.14,
    "ambient_light_intensity": 0.14 * VULKAN_LEGACY_RADIOMETRIC_SCALE,
    "vulkan_legacy_radiometric_scale": VULKAN_LEGACY_RADIOMETRIC_SCALE,
    "background_color_linear": [0.794, 0.794, 0.7],
    "gamma": 2.2,
}


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument(
        "--install-root",
        type=Path,
        default=repo_root / "out" / "install" / "vrt",
    )
    parser.add_argument(
        "--project",
        type=Path,
        default=repo_root / "Resources" / "DigitalAgricultureProject" / "test_lsystem_sorghum.eveproj",
    )
    parser.add_argument(
        "--cuda-reference-root",
        type=Path,
        default=(
            repo_root
            / "out"
            / "preserved_legacy_outputs"
            / "realism_review"
            / EXPERIMENT_ID
        ),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=repo_root / "out" / "renderer_comparison" / EXPERIMENT_ID,
    )
    parser.add_argument("--render-mode", choices=("RayTracing", "RayQuery"), default="RayTracing")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--samples-per-frame", type=int, default=8)
    parser.add_argument("--accumulation-frames", type=int, default=16)
    parser.add_argument("--bounces", type=int, default=3)
    parser.add_argument("--view", action="append", help="Render only this manifest view name (repeatable).")
    parser.add_argument("--view-limit", type=int, help="Render only the first N selected views.")
    parser.add_argument("--skip-capture", action="store_true", help="Only rebuild review artifacts from existing stills.")
    parser.add_argument("--no-grade", action="store_true", help="Keep Vulkan captures ungraded.")
    return parser.parse_args()


def require_pillow() -> tuple[Any, Any, Any, Any, Any, Any]:
    try:
        from PIL import Image, ImageChops, ImageDraw, ImageEnhance, ImageOps, ImageStat
    except ImportError as exc:
        raise RuntimeError("Pillow is required: py -3 -m pip install --user Pillow") from exc
    return Image, ImageChops, ImageDraw, ImageEnhance, ImageOps, ImageStat


def load_manifest(reference_root: Path) -> dict[str, Any]:
    manifest_path = reference_root / "stills_render_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"CUDA reference manifest is missing: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("view_count") != len(manifest.get("views", [])):
        raise RuntimeError("CUDA reference manifest view count is inconsistent.")
    return manifest


def selected_views(manifest: dict[str, Any], names: list[str] | None, limit: int | None) -> list[dict[str, Any]]:
    views = manifest["views"]
    if names:
        by_name = {view["view"]: view for view in views}
        unknown = sorted(set(names) - set(by_name))
        if unknown:
            raise RuntimeError(f"Unknown view name(s): {', '.join(unknown)}")
        views = [by_name[name] for name in names]
    if limit is not None:
        if limit <= 0:
            raise RuntimeError("--view-limit must be positive.")
        views = views[:limit]
    return views


def reference_still(reference_root: Path, view: dict[str, Any]) -> Path:
    filename = Path(view["output"]).name
    path = reference_root / "stills" / filename
    if not path.is_file():
        raise FileNotFoundError(f"CUDA reference still is missing: {path}")
    return path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def validate_cuda_references(reference_root: Path, views: list[dict[str, Any]]) -> None:
    for view in views:
        path = reference_still(reference_root, view)
        expected = view.get("sha256")
        if expected and sha256(path) != expected.upper():
            raise RuntimeError(f"CUDA reference was modified: {path}")


def configure_module_search(install_root: Path) -> Path:
    python_dir = install_root / "python"
    bin_dir = install_root / "bin"
    package_dir = bin_dir / "Packages"
    module_candidates = list(python_dir.glob("PyEvoEngine*.pyd"))
    if not module_candidates:
        raise FileNotFoundError(f"PyEvoEngine module is missing under {python_dir}")
    for directory in (python_dir, bin_dir, package_dir):
        if directory.is_dir():
            os.add_dll_directory(str(directory))
    sys.path.insert(0, str(python_dir))
    os.chdir(bin_dir)
    return bin_dir


def evo_vec3(evo: Any, values: list[float]) -> Any:
    vector = evo.Vec3()
    vector.x, vector.y, vector.z = (float(value) for value in values)
    return vector


def capture_vulkan(args: argparse.Namespace, views: list[dict[str, Any]]) -> None:
    configure_module_search(args.install_root.resolve())
    import PyEvoEngine as evo

    raw_dir = args.output_root / "vulkan_ray_tracing" / "raw"
    raw_dir.mkdir(parents=True, exist_ok=True)
    if not evo.RunWindowless(str(args.project.resolve())):
        raise RuntimeError(f"EvoEngine failed to start project: {args.project}")
    try:
        if not evo.WaitForCurrentSceneReady(3000):
            raise RuntimeError("The endpoint scene did not become ready for deterministic capture.")
        if not evo.ConfigureCurrentSceneOutdoorLightingForCapture(
            evo_vec3(evo, OUTDOOR_LIGHTING_PROFILE["sun_euler_degrees"]),
            OUTDOOR_LIGHTING_PROFILE["sun_angular_diameter_radians"],
            OUTDOOR_LIGHTING_PROFILE["sun_intensity"],
            evo_vec3(evo, OUTDOOR_LIGHTING_PROFILE["sun_color"]),
            OUTDOOR_LIGHTING_PROFILE["sky_light_intensity"],
            OUTDOOR_LIGHTING_PROFILE["ambient_light_intensity"],
            evo_vec3(evo, OUTDOOR_LIGHTING_PROFILE["background_color_linear"]),
            OUTDOOR_LIGHTING_PROFILE["gamma"],
        ):
            raise RuntimeError("Failed to apply the renderer-neutral outdoor lighting profile.")
        for index, view in enumerate(views, start=1):
            camera = view["camera"]
            output_name = f"{index:02d}_{view['view']}.png"
            output_path = raw_dir / output_name
            if not evo.SetMainCameraLookAt(
                evo_vec3(evo, camera["position"]),
                evo_vec3(evo, camera["target"]),
                evo_vec3(evo, camera["up"]),
                float(camera["fov_degrees"]),
            ):
                raise RuntimeError(f"Failed to set camera for {view['view']}")
            if not evo.ConfigureCurrentSceneCameraForCapture(
                args.render_mode, args.samples_per_frame, args.bounces
            ):
                raise RuntimeError(f"Vulkan {args.render_mode} is unavailable for {view['view']}")
            if not evo.CaptureCurrentScene(
                args.width,
                args.height,
                str(output_path.resolve()),
                args.accumulation_frames,
                True,
            ):
                raise RuntimeError(f"Capture failed for {view['view']}")
            print(f"[{index:02d}/{len(views):02d}] captured {output_path}", flush=True)
    finally:
        evo.Terminate()


def grade_image(source: Path, destination: Path, no_grade: bool) -> None:
    Image, _, _, ImageEnhance, _, _ = require_pillow()
    image = Image.open(source).convert("RGB")
    if not no_grade:
        image = ImageEnhance.Brightness(image).enhance(PRESENTATION_GRADE["brightness"])
        image = ImageEnhance.Contrast(image).enhance(PRESENTATION_GRADE["contrast"])
        image = ImageEnhance.Color(image).enhance(PRESENTATION_GRADE["saturation"])
        red, green, blue = image.split()
        red = red.point(lambda value: min(255, round(value * PRESENTATION_GRADE["red_gain"])))
        green = green.point(lambda value: min(255, round(value * PRESENTATION_GRADE["green_gain"])))
        blue = blue.point(lambda value: min(255, round(value * PRESENTATION_GRADE["blue_gain"])))
        image = Image.merge("RGB", (red, green, blue))
    destination.parent.mkdir(parents=True, exist_ok=True)
    image.save(destination, compress_level=6)


def image_metrics(path: Path) -> dict[str, Any]:
    Image, _, _, _, _, ImageStat = require_pillow()
    image = Image.open(path).convert("RGB")
    stat = ImageStat.Stat(image)
    return {
        "size": list(image.size),
        "rgb_mean": [round(value, 5) for value in stat.mean],
        "rgb_stddev": [round(value, 5) for value in stat.stddev],
    }


def pair_metrics(cuda_path: Path, vulkan_path: Path) -> dict[str, Any]:
    Image, ImageChops, _, _, _, ImageStat = require_pillow()
    cuda_image = Image.open(cuda_path).convert("RGB")
    vulkan_image = Image.open(vulkan_path).convert("RGB")
    if cuda_image.size != vulkan_image.size:
        vulkan_image = vulkan_image.resize(cuda_image.size, Image.Resampling.LANCZOS)
    difference = ImageChops.difference(cuda_image, vulkan_image)
    return {
        "mean_absolute_rgb_difference": [round(value, 5) for value in ImageStat.Stat(difference).mean],
    }


def labeled_pair(cuda_path: Path, vulkan_path: Path, label: str, destination: Path) -> None:
    Image, _, ImageDraw, _, _, _ = require_pillow()
    cuda_image = Image.open(cuda_path).convert("RGB")
    vulkan_image = Image.open(vulkan_path).convert("RGB")
    if vulkan_image.size != cuda_image.size:
        vulkan_image = vulkan_image.resize(cuda_image.size, Image.Resampling.LANCZOS)
    header_height = 64
    canvas = Image.new("RGB", (cuda_image.width * 2, cuda_image.height + header_height), (22, 24, 27))
    canvas.paste(cuda_image, (0, header_height))
    canvas.paste(vulkan_image, (cuda_image.width, header_height))
    draw = ImageDraw.Draw(canvas)
    draw.text((12, 8), label, fill=(238, 238, 238))
    draw.text((12, 36), "CUDA / OptiX reference", fill=(130, 205, 255))
    draw.text((cuda_image.width + 12, 36), "Vulkan ray tracing", fill=(255, 196, 105))
    destination.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(destination, compress_level=6)


def contact_sheet(
    image_records: list[tuple[Path, Path, str]],
    destination: Path,
    title: str,
    columns: int = 3,
) -> None:
    Image, _, ImageDraw, _, ImageOps, _ = require_pillow()
    cell_width, renderer_height = 400, 225
    cell_height = 48 + renderer_height
    rows = (len(image_records) + columns - 1) // columns
    title_height = 48
    canvas = Image.new("RGB", (columns * cell_width * 2, title_height + rows * cell_height), (18, 20, 23))
    draw = ImageDraw.Draw(canvas)
    draw.text((14, 15), title, fill=(245, 245, 245))
    for index, (cuda_path, vulkan_path, label) in enumerate(image_records):
        row, column = divmod(index, columns)
        x = column * cell_width * 2
        y = title_height + row * cell_height
        cuda_image = ImageOps.fit(Image.open(cuda_path).convert("RGB"), (cell_width, renderer_height))
        vulkan_image = ImageOps.fit(Image.open(vulkan_path).convert("RGB"), (cell_width, renderer_height))
        canvas.paste(cuda_image, (x, y + 48))
        canvas.paste(vulkan_image, (x + cell_width, y + 48))
        draw.text((x + 8, y + 7), label, fill=(230, 230, 230))
        draw.text((x + 8, y + 27), "CUDA / OptiX", fill=(130, 205, 255))
        draw.text((x + cell_width + 8, y + 27), "Vulkan RT", fill=(255, 196, 105))
    destination.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(destination, compress_level=6)


def build_review(args: argparse.Namespace, manifest: dict[str, Any], views: list[dict[str, Any]]) -> Path:
    raw_dir = args.output_root / "vulkan_ray_tracing" / "raw"
    graded_dir = args.output_root / "vulkan_ray_tracing" / "graded"
    paired_dir = args.output_root / "paired"
    records: list[tuple[Path, Path, str]] = []
    report_views: list[dict[str, Any]] = []
    for index, view in enumerate(views, start=1):
        cuda_path = reference_still(args.cuda_reference_root, view)
        raw_path = raw_dir / f"{index:02d}_{view['view']}.png"
        if not raw_path.is_file():
            raise FileNotFoundError(f"Vulkan still is missing: {raw_path}")
        vulkan_path = graded_dir / raw_path.name
        grade_image(raw_path, vulkan_path, args.no_grade)
        pair_path = paired_dir / raw_path.name
        labeled_pair(cuda_path, vulkan_path, view["label"], pair_path)
        records.append((cuda_path, vulkan_path, view["label"]))
        report_views.append(
            {
                "view": view["view"],
                "label": view["label"],
                "camera": view["camera"],
                "cuda_reference": str(cuda_path.resolve()),
                "cuda_sha256": sha256(cuda_path),
                "vulkan_ray_tracing": str(vulkan_path.resolve()),
                "vulkan_sha256": sha256(vulkan_path),
                "cuda_metrics": image_metrics(cuda_path),
                "vulkan_metrics": image_metrics(vulkan_path),
                **pair_metrics(cuda_path, vulkan_path),
            }
        )

    sheet_path = args.output_root / "contact_sheets" / "cuda_vs_vulkan_18_view.png"
    contact_sheet(records, sheet_path, f"{EXPERIMENT_ID} | fixed-camera CUDA / OptiX vs Vulkan ray tracing")
    report = {
        "schema_version": 1,
        "experiment_id": EXPERIMENT_ID,
        "scene_asset": manifest.get("scene_asset"),
        "camera_source": str((args.cuda_reference_root / "stills_render_manifest.json").resolve()),
        "comparison_policy": "identical saved endpoint scene and manifest camera transforms",
        "cuda_reference_renderer": manifest.get("render", {}).get("renderer"),
        "vulkan_renderer": args.render_mode,
        "vulkan_width": args.width,
        "vulkan_height": args.height,
        "vulkan_samples_per_frame": args.samples_per_frame,
        "vulkan_accumulation_frames": args.accumulation_frames,
        "vulkan_total_spp": args.samples_per_frame * args.accumulation_frames,
        "vulkan_bounces": args.bounces,
        "outdoor_lighting_profile": OUTDOOR_LIGHTING_PROFILE,
        "presentation_grade": None if args.no_grade else PRESENTATION_GRADE,
        "view_count": len(report_views),
        "views": report_views,
        "contact_sheet": str(sheet_path.resolve()),
    }
    report_path = args.output_root / "comparison_manifest.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return sheet_path


def main() -> int:
    args = parse_args()
    args.repo_root = args.repo_root.resolve()
    args.install_root = args.install_root.resolve()
    args.project = args.project.resolve()
    args.cuda_reference_root = args.cuda_reference_root.resolve()
    args.output_root = args.output_root.resolve()
    require_pillow()
    manifest = load_manifest(args.cuda_reference_root)
    views = selected_views(manifest, args.view, args.view_limit)
    validate_cuda_references(args.cuda_reference_root, views)
    if not args.skip_capture:
        capture_vulkan(args, views)
    sheet = build_review(args, manifest, views)
    print(f"Comparison complete: {sheet}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise
