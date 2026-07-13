#!/usr/bin/env python3
"""Render and optionally publish the five-scene 4x10 mobile review package."""

from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import math
import os
import shutil
import statistics
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

from sorghum_asset_layout import (
    DATE_ORDER,
    GENERATED_REPORT_ROOT,
    GENERATED_SCENE_ROOT,
    MANUAL_4X10_SCENE,
    four_by_ten_scene,
    growth_stage,
)


VIEW_ORDER = (
    "scene_oblique",
    "scene_top",
    "scene_row",
    "btx_plant",
    "btx_basal",
    "btx_leaf",
    "pawaga_plant",
    "pawaga_basal",
    "pawaga_leaf",
)
VIEW_LABELS = {
    "scene_oblique": "RT oblique overview",
    "scene_top": "RT top-down canopy",
    "scene_row": "RT row-level overview",
    "btx_plant": "BTX representative plant",
    "btx_basal": "BTX culm, sheath, and tillers",
    "btx_leaf": "BTX mid-canopy leaf surface",
    "pawaga_plant": "Pawaga representative plant",
    "pawaga_basal": "Pawaga culm, sheath, and tillers",
    "pawaga_leaf": "Pawaga mid-canopy leaf surface",
}
REFERENCE_SCENE = MANUAL_4X10_SCENE
DRIVE_FOLDER_NAME = "Iteration_01_RT_PhysicalSun"
DEFAULT_DRIVE_OUTPUT_DIR = Path(r"G:\My Drive\Sorghum\4x10_Scene_Review_Latest") / DRIVE_FOLDER_NAME
LABEL_BAND_HEIGHT = 116
SUN_ANGLES_DEGREES = (55.0, 30.0, 0.0)
SUN_ANGULAR_DIAMETER_RADIANS = 0.00918043


@dataclass(frozen=True)
class Bounds:
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]

    @property
    def center(self) -> tuple[float, float, float]:
        return tuple((low + high) * 0.5 for low, high in zip(self.minimum, self.maximum))

    def corners(self) -> Iterable[tuple[float, float, float]]:
        return itertools.product(*zip(self.minimum, self.maximum))


@dataclass(frozen=True)
class ProtectedAssetSnapshot:
    files: dict[Path, bytes]
    side_effect_root: Path
    side_effects: set[Path]


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[1]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def parse_dates(value: str) -> list[str]:
    dates = [item.strip() for item in value.split(",") if item.strip()]
    unknown = [date for date in dates if date not in DATE_ORDER]
    if unknown:
        raise argparse.ArgumentTypeError(f"unsupported date(s): {', '.join(unknown)}")
    return dates or list(DATE_ORDER)


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


def require_artifact_dependencies() -> tuple[object, object, object, object, object]:
    try:
        from PIL import Image, ImageDraw, ImageFont, ImageStat
        from pypdf import PdfReader
        from reportlab.lib.utils import ImageReader
        from reportlab.pdfgen import canvas
    except ImportError as error:
        raise RuntimeError(
            "mobile review dependencies are missing; run: "
            "python -m pip install -r PythonBinding/requirements-mobile-review.txt"
        ) from error
    return Image, ImageDraw, ImageFont, ImageStat, (PdfReader, ImageReader, canvas)


def vec_add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(x + y for x, y in zip(a, b))


def vec_sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(x - y for x, y in zip(a, b))


def vec_scale(value: tuple[float, float, float], scale: float) -> tuple[float, float, float]:
    return tuple(component * scale for component in value)


def dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def normalize(value: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(dot(value, value))
    if length <= 1e-9:
        raise ValueError("cannot normalize a zero-length vector")
    return vec_scale(value, 1.0 / length)


def fit_camera(
    bounds: Bounds,
    camera_offset_direction: tuple[float, float, float],
    requested_up: tuple[float, float, float],
    fov_degrees: float,
    aspect_ratio: float,
    padding: float = 1.12,
) -> dict[str, object]:
    center = bounds.center
    front = vec_scale(normalize(camera_offset_direction), -1.0)
    right = normalize(cross(front, normalize(requested_up)))
    up = normalize(cross(right, front))
    tan_vertical = math.tan(math.radians(fov_degrees) * 0.5)
    tan_horizontal = tan_vertical * aspect_ratio
    distance = 0.1
    for corner in bounds.corners():
        relative = vec_sub(corner, center)
        along_front = dot(relative, front)
        distance = max(
            distance,
            abs(dot(relative, right)) / tan_horizontal - along_front,
            abs(dot(relative, up)) / tan_vertical - along_front,
        )
    distance *= padding
    position = vec_sub(center, vec_scale(front, distance))
    return {
        "position": position,
        "target": center,
        "up": up,
        "fov_degrees": fov_degrees,
    }


def metadata_position(record: object) -> tuple[float, float, float]:
    value = record.global_position
    return float(value.x), float(value.y), float(value.z)


def layouts_match(
    actual: dict[str, Iterable[float]], expected: dict[str, Iterable[float]], tolerance: float = 1e-4
) -> bool:
    return actual.keys() == expected.keys() and all(
        max(abs(a - b) for a, b in zip(actual[name], expected[name])) <= tolerance for name in actual
    )


def record_vec3(record: object, name: str) -> tuple[float, float, float]:
    value = getattr(record, name)
    return float(value.x), float(value.y), float(value.z)


def geometry_bounds(records: list[object]) -> Bounds:
    if not records:
        raise ValueError("cannot frame an empty plant set")
    minimums = [record_vec3(record, "geometry_min_position") for record in records]
    maximums = [record_vec3(record, "geometry_max_position") for record in records]
    return Bounds(
        tuple(min(value[index] for value in minimums) for index in range(3)),
        tuple(max(value[index] for value in maximums) for index in range(3)),
    )


def union_bounds(bounds: Iterable[Bounds]) -> Bounds:
    values = list(bounds)
    if not values:
        raise ValueError("cannot combine an empty bounds set")
    return Bounds(
        tuple(min(value.minimum[index] for value in values) for index in range(3)),
        tuple(max(value.maximum[index] for value in values) for index in range(3)),
    )


def bounds_to_json(bounds: Bounds) -> dict[str, list[float]]:
    return {"minimum": list(bounds.minimum), "maximum": list(bounds.maximum)}


def bounds_from_json(value: dict[str, list[float]]) -> Bounds:
    return Bounds(tuple(value["minimum"]), tuple(value["maximum"]))


def representative_plant(records: list[object], cultivar: str) -> object:
    selected = [record for record in records if record.cultivar == cultivar]
    center_x = statistics.fmean(float(record.global_position.x) for record in selected)
    center_z = statistics.fmean(float(record.global_position.z) for record in selected)
    median_height = statistics.median(float(record.plant_height_m) for record in selected)
    return min(
        selected,
        key=lambda record: (
            math.hypot(float(record.global_position.x) - center_x, float(record.global_position.z) - center_z)
            + abs(float(record.plant_height_m) - median_height) / max(median_height, 0.01),
            record.name,
        ),
    )


def record_bounds(record: object) -> Bounds:
    return Bounds(record_vec3(record, "geometry_min_position"), record_vec3(record, "geometry_max_position"))


def focus_camera(
    target: tuple[float, float, float],
    direction: tuple[float, float, float],
    distance: float,
    fov_degrees: float,
) -> dict[str, object]:
    return {
        "position": vec_add(target, vec_scale(normalize(direction), distance)),
        "target": target,
        "up": (0.0, 1.0, 0.0),
        "fov_degrees": fov_degrees,
    }


def plant_detail_cameras(record: object, aspect: float) -> dict[str, dict[str, object]]:
    bounds = record_bounds(record)
    root = metadata_position(record)
    height = max(float(record.plant_height_m), 0.1)
    plant = fit_camera(bounds, (0.2, 0.22, 1.0), (0.0, 1.0, 0.0), 46.0, aspect, 1.10)
    basal = focus_camera((root[0], root[1] + 0.16 * height, root[2]), (0.25, 0.12, 1.0), max(0.32, 0.34 * height), 38.0)
    leaf = focus_camera((root[0], root[1] + 0.62 * height, root[2]), (-0.3, 0.12, 1.0), max(0.28, 0.28 * height), 34.0)
    return {"plant": plant, "basal": basal, "leaf": leaf}


def make_vec3(evo: object, value: tuple[float, float, float]) -> object:
    result = evo.Vec3()
    result.x, result.y, result.z = value
    return result


def apply_camera(evo: object, camera: dict[str, object]) -> None:
    if not evo.SetMainCameraLookAt(
        make_vec3(evo, camera["position"]),
        make_vec3(evo, camera["target"]),
        make_vec3(evo, camera["up"]),
        camera["fov_degrees"],
    ):
        raise RuntimeError("failed to apply review camera")


def date_settings(rows: list[dict[str, str]], date: str) -> tuple[float, float, float]:
    selected = [row for row in rows if row["date"] == date]
    if not selected:
        raise ValueError(f"manifest has no rows for {date}")
    widths = {float(row["leaf_width_scale"]) for row in selected}
    thicknesses = {float(row["leaf_thickness_m"]) for row in selected}
    if len(widths) != 1 or len(thicknesses) != 1:
        raise ValueError(f"{date} has inconsistent leaf width or thickness settings")
    return widths.pop(), thicknesses.pop(), max(float(row["final_height_m"]) for row in selected)


def validate_image(path: Path, width: int, height: int, Image: object, ImageStat: object) -> dict[str, float]:
    with Image.open(path) as image:
        image.load()
        if image.size != (width, height):
            raise RuntimeError(f"unexpected image dimensions for {path}: {image.size}")
        grayscale = image.convert("L")
        stats = ImageStat.Stat(grayscale)
        mean = float(stats.mean[0])
        deviation = float(stats.stddev[0])
        extrema = grayscale.getextrema()
    if deviation < 3.0 or extrema[1] - extrema[0] < 15:
        raise RuntimeError(f"image appears blank or flat: {path}")
    return {"luminance_mean": mean, "luminance_stddev": deviation}


def fonts(ImageFont: object) -> tuple[object, object, object]:
    regular = Path(r"C:\Windows\Fonts\segoeui.ttf")
    bold = Path(r"C:\Windows\Fonts\segoeuib.ttf")
    if regular.exists() and bold.exists():
        return ImageFont.truetype(str(bold), 43), ImageFont.truetype(str(regular), 25), ImageFont.truetype(str(bold), 27)
    return ImageFont.load_default(), ImageFont.load_default(), ImageFont.load_default()


def label_image(
    raw_path: Path,
    output_path: Path,
    title: str,
    subtitle: str,
    Image: object,
    ImageDraw: object,
    ImageFont: object,
    quality: int,
) -> None:
    title_font, subtitle_font, _ = fonts(ImageFont)
    with Image.open(raw_path).convert("RGB") as source:
        image = Image.new("RGB", (source.width, source.height + LABEL_BAND_HEIGHT), (17, 24, 28))
        image.paste(source, (0, 0))
    draw = ImageDraw.Draw(image)
    top = image.height - LABEL_BAND_HEIGHT
    draw.rectangle((0, top, image.width, image.height), fill=(17, 24, 28))
    draw.text((42, top + 13), title, font=title_font, fill=(255, 255, 255, 255))
    draw.text((44, top + 70), subtitle, font=subtitle_font, fill=(210, 222, 225, 255))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path, "JPEG", quality=quality, optimize=True)


def capture_scene(
    evo: object,
    args: argparse.Namespace,
    date: str,
    scene_relative_path: Path,
    settings: tuple[float, float, float],
    shared_bounds: Bounds | None,
    expected_layout: dict[str, tuple[float, float, float]] | None,
    package_root: Path,
    dependencies: tuple[object, object, object, object, object],
    capture_views: bool = True,
) -> tuple[dict[str, object], Bounds, dict[str, tuple[float, float, float]], dict[str, Path]]:
    Image, ImageDraw, ImageFont, ImageStat, _pdf_dependencies = dependencies
    scene_path = args.source_root / scene_relative_path.name
    width_scale, thickness_m, manifest_max_height = settings
    scene_record: dict[str, object] = {
        "date": date,
        "scene": scene_relative_path.as_posix(),
        "scene_sha256": sha256(scene_path),
        "leaf_width_scale": width_scale,
        "leaf_thickness_m": thickness_m,
        "geometry_seed": args.geometry_seed,
        "views": [],
    }
    raw_dir = package_root / "raw"
    still_dir = package_root / "stills"
    paths: dict[str, Path] = {}
    if not evo.RunLSystemSorghumProject(
        args.project.resolve(), args.runtime_package_dir.resolve(), scene_relative_path
    ):
        raise RuntimeError(f"failed to load {scene_relative_path}")
    try:
        if not evo.WaitForProjectIdle(args.max_wait_frames):
            raise RuntimeError(f"project did not become idle for {date}")
        if not evo.EnsureIlluminationSoilContext() or not evo.ValidateIlluminationContext():
            raise RuntimeError(f"PBR soil validation failed for {date}")
        grown = int(evo.GrowSorghumLsPlantsToAdulthood(args.geometry_seed))
        if grown != 40:
            raise RuntimeError(f"{date}: expected 40 grown plants, got {grown}")
        if not evo.WaitForProjectIdle(args.max_wait_frames):
            raise RuntimeError(f"geometry did not become idle for {date}")
        records = list(evo.GetSorghumLsPlantSceneMetadata(True))
        if len(records) != 40 or any(not record.has_geometry for record in records):
            raise RuntimeError(f"{date}: expected 40 measurable plants")
        cultivars = {record.cultivar for record in records}
        if cultivars != {"BTX", "Pawaga"}:
            raise RuntimeError(f"{date}: unexpected cultivars: {sorted(cultivars)}")
        if any(abs(float(record.leaf_width_scale) - width_scale) > 1e-5 for record in records):
            raise RuntimeError(f"{date}: loaded scene leaf width does not match the calibration manifest")
        if any(abs(float(record.leaf_thickness_m) - thickness_m) > 1e-7 for record in records):
            raise RuntimeError(f"{date}: loaded scene leaf thickness does not match the calibration manifest")

        layout = {record.name: metadata_position(record) for record in records}
        if expected_layout is not None and not layouts_match(layout, expected_layout):
            raise RuntimeError(f"{date}: world-space plant layout differs from the first date")
        current_bounds = geometry_bounds(records)
        scene_record["geometry_bounds"] = bounds_to_json(current_bounds)
        if shared_bounds is None:
            shared_bounds = current_bounds

        actual_heights = [float(record.plant_height_m) for record in records]
        scene_record["plant_count"] = len(records)
        scene_record["mean_plant_height_m"] = statistics.fmean(actual_heights)
        scene_record["maximum_plant_height_m"] = max(actual_heights)
        scene_record["manifest_maximum_plant_height_m"] = manifest_max_height
        if not capture_views:
            return scene_record, current_bounds, layout, paths

        capture_height = args.height - LABEL_BAND_HEIGHT
        aspect = args.width / capture_height
        representatives = {cultivar: representative_plant(records, cultivar) for cultivar in ("BTX", "Pawaga")}
        scene_record["representative_plants"] = {
            cultivar: {"name": record.name, "height_m": float(record.plant_height_m)}
            for cultivar, record in representatives.items()
        }
        cameras = {
            "scene_oblique": fit_camera(shared_bounds, (1.0, 0.65, 1.0), (0.0, 1.0, 0.0), 50.0, aspect, 1.10),
            "scene_top": fit_camera(shared_bounds, (0.0, 1.0, 0.0), (0.0, 0.0, -1.0), 50.0, aspect, 1.06),
            "scene_row": fit_camera(shared_bounds, (0.0, 0.18, 1.0), (0.0, 1.0, 0.0), 48.0, aspect, 1.08),
        }
        for cultivar, record in representatives.items():
            for detail, camera in plant_detail_cameras(record, aspect).items():
                cameras[f"{cultivar.lower()}_{detail}"] = camera

        sun_angles = make_vec3(evo, SUN_ANGLES_DEGREES)
        sun_color = make_vec3(evo, (1.0, 1.0, 1.0))
        if not evo.ConfigureRayTracerSkydome(
            sun_angles,
            SUN_ANGULAR_DIAMETER_RADIANS,
            1.0,
            sun_color,
            1.0,
            0.1,
            2.2,
        ):
            raise RuntimeError(f"{date}: failed to configure the RT skydome")

        for view_index, view in enumerate(VIEW_ORDER, start=1):
            camera = cameras[view]
            apply_camera(evo, camera)
            evo.LoopFrames(args.warmup_frames)
            stage = growth_stage(date)
            raw_path = raw_dir / f"{stage}_{view}.png"
            if not evo.CaptureCurrentSceneRayTraced(
                args.width, capture_height, raw_path, args.samples, args.bounces, 2.2
            ):
                raise RuntimeError(f"capture failed: {date} {view}")
            metrics = validate_image(raw_path, args.width, capture_height, Image, ImageStat)
            output_path = still_dir / f"{view_index:02d}_{stage}_{view}.jpg"
            framing = "fixed comparison framing" if view.startswith("scene_") else "representative-plant framing"
            subtitle = (
                f"width {width_scale:.3f}x | thickness {thickness_m * 1000.0:.3f} mm | "
                f"seed {args.geometry_seed} | RT {args.samples} spp / {args.bounces} bounces | {framing}"
            )
            label_image(
                raw_path,
                output_path,
                f"{stage} | {date} | {VIEW_LABELS[view]}",
                subtitle,
                Image,
                ImageDraw,
                ImageFont,
                args.jpeg_quality,
            )
            validate_image(output_path, args.width, args.height, Image, ImageStat)
            paths[view] = output_path
            scene_record["views"].append(
                {
                    "id": view,
                    "file": output_path.relative_to(package_root).as_posix(),
                    "raw_file": raw_path.relative_to(package_root).as_posix(),
                    "sha256": sha256(output_path),
                    "raw_sha256": sha256(raw_path),
                    "lighting": "rt_physical_sun",
                    "framing": framing,
                    "camera": camera,
                    **metrics,
                }
            )
    finally:
        pass
    return scene_record, shared_bounds, layout, paths


def make_contact_sheet(
    output_path: Path,
    paths_by_date: dict[str, dict[str, Path]],
    dependencies: tuple[object, object, object, object, object],
) -> None:
    Image, ImageDraw, ImageFont, _ImageStat, _pdf_dependencies = dependencies
    canvas = Image.new("RGB", (1440, 1920), (243, 245, 244))
    draw = ImageDraw.Draw(canvas)
    title_font, _subtitle_font, label_font = fonts(ImageFont)
    draw.text((56, 48), "4x10 Sorghum RT Realism Review", font=title_font, fill=(24, 31, 33))
    draw.text((58, 107), "Physical-sun oblique views - fixed framing across stages", font=label_font, fill=(67, 79, 82))
    slots = ((55, 165), (745, 165), (55, 700), (745, 700), (375, 1235))
    for date, (x, y) in zip(paths_by_date, slots):
        with Image.open(paths_by_date[date]["scene_oblique"]).convert("RGB") as image:
            image.thumbnail((640, 430))
            canvas.paste(image, (x, y + 52))
        draw.text((x, y), date, font=label_font, fill=(24, 31, 33))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path, "JPEG", quality=92, optimize=True)


def make_stage_contact_sheet(
    output_path: Path,
    date: str,
    paths: dict[str, Path],
    dependencies: tuple[object, object, object, object, object],
) -> None:
    Image, ImageDraw, ImageFont, _ImageStat, _pdf_dependencies = dependencies
    canvas = Image.new("RGB", (1920, 1920), (243, 245, 244))
    draw = ImageDraw.Draw(canvas)
    title_font, _subtitle_font, label_font = fonts(ImageFont)
    draw.text((55, 40), f"{growth_stage(date)} | {date}", font=title_font, fill=(24, 31, 33))
    slots = [(45 + column * 625, 125 + row * 585) for row in range(3) for column in range(3)]
    for view, (x, y) in zip(VIEW_ORDER, slots):
        draw.text((x, y), VIEW_LABELS[view], font=label_font, fill=(24, 31, 33))
        with Image.open(paths[view]).convert("RGB") as image:
            image.thumbnail((590, 500))
            canvas.paste(image, (x, y + 42))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path, "JPEG", quality=92, optimize=True)


def draw_pdf_image(pdf: object, ImageReader: object, image_path: Path, x: float, y: float, width: float, height: float) -> None:
    from PIL import Image

    with Image.open(image_path) as image:
        ratio = min(width / image.width, height / image.height)
        draw_width = image.width * ratio
        draw_height = image.height * ratio
    pdf.drawImage(
        ImageReader(str(image_path)),
        x + (width - draw_width) * 0.5,
        y + (height - draw_height) * 0.5,
        draw_width,
        draw_height,
        preserveAspectRatio=True,
    )


def make_pdf(
    output_path: Path,
    contact_sheet: Path,
    paths_by_date: dict[str, dict[str, Path]],
    settings_by_date: dict[str, tuple[float, float, float]],
    geometry_seed: int,
    dependencies: tuple[object, object, object, object, object],
) -> None:
    _Image, _ImageDraw, _ImageFont, _ImageStat, pdf_dependencies = dependencies
    _PdfReader, ImageReader, canvas_module = pdf_dependencies
    page_width, page_height = 720.0, 960.0
    pdf = canvas_module.Canvas(str(output_path), pagesize=(page_width, page_height), pageCompression=1)
    pdf.setTitle("4x10 Sorghum Scene Review")
    page_number = 0

    def begin_page(title: str, subtitle: str = "") -> None:
        nonlocal page_number
        page_number += 1
        pdf.setFillColorRGB(0.09, 0.12, 0.13)
        pdf.setFont("Helvetica-Bold", 22)
        pdf.drawString(36, 922, title)
        if subtitle:
            pdf.setFillColorRGB(0.32, 0.39, 0.4)
            pdf.setFont("Helvetica", 11)
            pdf.drawString(36, 901, subtitle)

    def finish_page() -> None:
        pdf.setFillColorRGB(0.4, 0.45, 0.46)
        pdf.setFont("Helvetica", 9)
        pdf.drawRightString(684, 18, f"Page {page_number}")
        pdf.showPage()

    date_label = "date" if len(paths_by_date) == 1 else "dates"
    begin_page(
        "4x10 Sorghum Scene Review",
        f"{len(paths_by_date)} calibrated {date_label}; deterministic geometry seed {geometry_seed}",
    )
    draw_pdf_image(pdf, ImageReader, contact_sheet, 36, 48, 648, 830)
    finish_page()

    for date, paths in paths_by_date.items():
        width_scale, thickness_m, _maximum_height = settings_by_date[date]
        metadata = f"Leaf width {width_scale:.3f}x | thickness {thickness_m * 1000.0:.3f} mm"
        stage = growth_stage(date)
        begin_page(f"{stage} | {date} - Scene", metadata + " | OptiX physical sun")
        draw_pdf_image(pdf, ImageReader, paths["scene_oblique"], 36, 610, 648, 265)
        draw_pdf_image(pdf, ImageReader, paths["scene_top"], 36, 335, 648, 245)
        draw_pdf_image(pdf, ImageReader, paths["scene_row"], 36, 60, 648, 245)
        finish_page()

        begin_page(f"{stage} | {date} - BTX", metadata + " | representative median-height plant")
        draw_pdf_image(pdf, ImageReader, paths["btx_plant"], 36, 610, 648, 265)
        draw_pdf_image(pdf, ImageReader, paths["btx_basal"], 36, 335, 648, 245)
        draw_pdf_image(pdf, ImageReader, paths["btx_leaf"], 36, 60, 648, 245)
        finish_page()

        begin_page(f"{stage} | {date} - Pawaga", metadata + " | representative median-height plant")
        draw_pdf_image(pdf, ImageReader, paths["pawaga_plant"], 36, 610, 648, 265)
        draw_pdf_image(pdf, ImageReader, paths["pawaga_basal"], 36, 335, 648, 245)
        draw_pdf_image(pdf, ImageReader, paths["pawaga_leaf"], 36, 60, 648, 245)
        finish_page()
    pdf.save()


def write_readme(path: Path) -> None:
    path.write_text(
        "4x10 Sorghum RT Realism Review - Iteration 01\n"
        "================================================\n\n"
        "Open Sorghum_4x10_Scene_Review.pdf for the mobile review book.\n"
        "Lossless OptiX captures are under raw/ and labeled review images are under stills/.\n"
        "Per-stage 3x3 summaries are under contact_sheets/.\n"
        "All views use a 0.526-degree physical sun with fixed RT settings.\n"
        "Scene files are never saved or modified by this renderer.\n",
        encoding="utf-8",
    )


def package_hashes(root: Path) -> list[dict[str, object]]:
    return [
        {"file": path.relative_to(root).as_posix(), "bytes": path.stat().st_size, "sha256": sha256(path)}
        for path in sorted(root.rglob("*"))
        if path.is_file() and path.name != "manifest.json"
    ]


def validate_package(root: Path, dates: list[str], dependencies: tuple[object, object, object, object, object]) -> None:
    _Image, _ImageDraw, _ImageFont, _ImageStat, pdf_dependencies = dependencies
    PdfReader, _ImageReader, _canvas = pdf_dependencies
    stills = list((root / "stills").glob("*.jpg"))
    raw = list((root / "raw").glob("*.png"))
    if len(stills) != len(dates) * len(VIEW_ORDER) or len(raw) != len(stills):
        raise RuntimeError(f"unexpected render count: {len(stills)} stills and {len(raw)} raw captures")
    pdf_path = root / "Sorghum_4x10_Scene_Review.pdf"
    expected_pages = 1 + len(dates) * 3
    if len(PdfReader(str(pdf_path)).pages) != expected_pages:
        raise RuntimeError(f"review PDF does not contain {expected_pages} pages")
    if len(list((root / "contact_sheets").glob("*.jpg"))) != len(dates):
        raise RuntimeError("review package does not contain one contact sheet per stage")
    for required in (root / "00_all_dates_overview.jpg", root / "manifest.json", root / "README.txt"):
        if not required.exists() or required.stat().st_size == 0:
            raise RuntimeError(f"missing package file: {required}")


def published_files(package_root: Path) -> list[Path]:
    names = [
        "Sorghum_4x10_Scene_Review.pdf",
        "00_all_dates_overview.jpg",
        "manifest.json",
        "README.txt",
        "raw",
        "stills",
        "contact_sheets",
    ]
    missing = [name for name in names if not (package_root / name).exists()]
    if missing:
        raise FileNotFoundError("mobile review package is incomplete: " + ", ".join(missing))
    return [
        path
        for name in names
        for path in ([package_root / name] if (package_root / name).is_file() else (package_root / name).rglob("*"))
        if path.is_file()
    ]


def verify_published_files(package_root: Path, target_root: Path) -> None:
    for source in published_files(package_root):
        target = target_root / source.relative_to(package_root)
        if not target.exists() or sha256(source) != sha256(target):
            raise RuntimeError(f"Drive verification failed: {target}")


def publish_to_drive(package_root: Path, drive_output_dir: Path) -> None:
    package_root = package_root.resolve()
    drive_output_dir = drive_output_dir.resolve()
    if drive_output_dir != DEFAULT_DRIVE_OUTPUT_DIR.resolve():
        raise ValueError(f"refusing to replace unexpected Drive folder: {drive_output_dir}")
    if (
        package_root == drive_output_dir
        or package_root.is_relative_to(drive_output_dir)
        or drive_output_dir.is_relative_to(package_root)
    ):
        raise ValueError("local package and Drive target must not overlap")

    parent = drive_output_dir.parent
    parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{DRIVE_FOLDER_NAME}.staging-", dir=parent))
    backup: Path | None = None
    try:
        for source in published_files(package_root):
            target = staging / source.relative_to(package_root)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        verify_published_files(package_root, staging)

        if drive_output_dir.exists():
            if not drive_output_dir.is_dir():
                raise ValueError(f"Drive target is not a directory: {drive_output_dir}")
            backup = Path(tempfile.mkdtemp(prefix=f".{DRIVE_FOLDER_NAME}.backup-", dir=parent))
            backup.rmdir()
            drive_output_dir.replace(backup)
        staging.replace(drive_output_dir)
        verify_published_files(package_root, drive_output_dir)
        version_stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        version_dir = drive_output_dir.parent.parent / f"4x10_Scene_Review_{version_stamp}" / DRIVE_FOLDER_NAME
        version_dir.mkdir(parents=True)
        for source in published_files(package_root):
            target = version_dir / source.relative_to(package_root)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        verify_published_files(package_root, version_dir)
        print(f"drive_versioned_dir={version_dir}")
        if backup is not None:
            shutil.rmtree(backup)
            backup = None
    except Exception:
        if backup is not None and backup.exists():
            if drive_output_dir.exists():
                shutil.rmtree(drive_output_dir)
            backup.replace(drive_output_dir)
        raise
    finally:
        if staging.exists():
            shutil.rmtree(staging)


def replace_local_package(staging_root: Path, output_dir: Path) -> None:
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    backup: Path | None = None
    try:
        if output_dir.exists():
            backup = Path(tempfile.mkdtemp(prefix=f".{output_dir.name}.backup-", dir=output_dir.parent))
            backup.rmdir()
            output_dir.replace(backup)
        staging_root.replace(output_dir)
        if backup is not None:
            shutil.rmtree(backup)
            backup = None
    except Exception:
        if backup is not None and backup.exists():
            if output_dir.exists():
                shutil.rmtree(output_dir)
            backup.replace(output_dir)
        raise


def protected_files(args: argparse.Namespace, dates: list[str]) -> ProtectedAssetSnapshot:
    assets_root = (args.project.parent / "Assets").resolve()
    paths = [args.project, assets_root / REFERENCE_SCENE]
    paths.extend(args.project.parent / "Assets" / four_by_ten_scene(date) for date in dates)
    side_effects = {path.resolve() for path in assets_root.glob("New Scene*.evescene*")}
    paths.extend(side_effects)
    return ProtectedAssetSnapshot(
        {path.resolve(): path.resolve().read_bytes() for path in paths}, assets_root, side_effects
    )


def restore_and_fail_on_asset_changes(snapshot: ProtectedAssetSnapshot) -> None:
    changed = [
        path
        for path, content in snapshot.files.items()
        if not path.exists() or path.read_bytes() != content
    ]
    unexpected = {
        path.resolve()
        for path in snapshot.side_effect_root.glob("New Scene*.evescene*")
        if path.resolve() not in snapshot.side_effects
    }
    for path in unexpected:
        path.unlink(missing_ok=True)
    for path in changed:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(snapshot.files[path])
    unexpected_changes = [path for path in changed if path.suffix != ".eveproj"]
    if unexpected_changes:
        affected = sorted(map(str, unexpected_changes))
        raise RuntimeError("renderer changed project assets; originals were restored: " + ", ".join(affected))


def worker_command(
    args: argparse.Namespace,
    date: str,
    output_dir: Path,
    context_path: Path | None,
    probe: bool = False,
) -> list[str]:
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--repo-root",
        str(args.repo_root),
        "--build-dir",
        str(args.build_dir),
        "--config",
        args.config,
        "--project",
        str(args.project),
        "--runtime-package-dir",
        str(args.runtime_package_dir),
        "--source-root",
        str(args.source_root),
        "--manifest",
        str(args.manifest),
        "--dates",
        ",".join(args.dates),
        "--geometry-seed",
        str(args.geometry_seed),
        "--width",
        str(args.width),
        "--height",
        str(args.height),
        "--jpeg-quality",
        str(args.jpeg_quality),
        "--samples",
        str(args.samples),
        "--bounces",
        str(args.bounces),
        "--warmup-frames",
        str(args.warmup_frames),
        "--max-wait-frames",
        str(args.max_wait_frames),
        "--worker-date",
        date,
        "--worker-output-dir",
        str(output_dir),
    ]
    if context_path is not None:
        command.extend(["--worker-context", str(context_path)])
    if probe:
        command.append("--worker-probe")
    return command


def run_worker(
    args: argparse.Namespace,
    date: str,
    output_dir: Path,
    context_path: Path | None,
    probe: bool = False,
) -> dict[str, object]:
    result = subprocess.run(
        worker_command(args, date, output_dir, context_path, probe), cwd=args.repo_root, check=False
    )
    result_path = output_dir / "worker_result.json"
    if not result_path.exists():
        raise RuntimeError(f"mobile review worker failed for {date} with exit code {result.returncode}")
    return json.loads(result_path.read_text(encoding="utf-8"))


def worker_main(
    args: argparse.Namespace,
    rows: list[dict[str, str]],
    dependencies: tuple[object, object, object, object, object],
) -> None:
    date = args.worker_date
    if date not in args.dates:
        raise ValueError(f"worker date is not selected: {date}")
    output_dir = args.worker_output_dir.resolve()
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True)

    shared_bounds = None
    expected_layout = None
    if args.worker_context:
        context = json.loads(args.worker_context.read_text(encoding="utf-8"))
        shared_bounds = bounds_from_json(context["shared_bounds"])
        expected_layout = {name: tuple(position) for name, position in context["layout"].items()}

    configure_engine_imports(args.repo_root, args.build_dir, args.config)
    import PyDigitalAgriculture as evo

    snapshot = protected_files(args, [date])
    scene = four_by_ten_scene(date)
    try:
        record, shared_bounds, layout, _paths = capture_scene(
            evo,
            args,
            date,
            scene,
            date_settings(rows, date),
            shared_bounds,
            expected_layout,
            output_dir,
            dependencies,
            not args.worker_probe,
        )
    finally:
        restore_and_fail_on_asset_changes(snapshot)
    result = {
        "scene": record,
        "shared_bounds": bounds_to_json(shared_bounds),
        "layout": {name: list(position) for name, position in layout.items()},
    }
    (output_dir / "worker_result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


def build_parser() -> argparse.ArgumentParser:
    repo_root = repo_root_from_script()
    project_root = repo_root / "Resources" / "DigitalAgricultureProject"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument("--build-dir", type=Path, default=repo_root / "out" / "build" / "vs2026-x64")
    parser.add_argument("--config", default="RelWithDebInfo")
    parser.add_argument("--project", type=Path, default=project_root / "test_lsystem_sorghum.eveproj")
    parser.add_argument("--runtime-package-dir", type=Path, default=repo_root / "out" / "build" / "vs2026-x64" / "EvoEngine_App" / "RelWithDebInfo" / "Packages")
    parser.add_argument("--source-root", type=Path, default=project_root / "Assets" / GENERATED_SCENE_ROOT)
    parser.add_argument("--manifest", type=Path, default=project_root / "Assets" / GENERATED_REPORT_ROOT / "field_manifest.csv")
    parser.add_argument("--dates", type=parse_dates, default=list(DATE_ORDER))
    parser.add_argument("--geometry-seed", type=int, default=2_000_000)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repo_root / "out" / "realism_review" / "sorghum_4x10" / DRIVE_FOLDER_NAME,
    )
    parser.add_argument("--drive-output-dir", type=Path, default=DEFAULT_DRIVE_OUTPUT_DIR)
    parser.add_argument("--publish-drive", action="store_true")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--jpeg-quality", type=int, default=92)
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--bounces", type=int, default=4)
    parser.add_argument("--warmup-frames", type=int, default=4)
    parser.add_argument("--max-wait-frames", type=int, default=30000)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--worker-date", default="", help=argparse.SUPPRESS)
    parser.add_argument("--worker-output-dir", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--worker-context", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--worker-probe", action="store_true", help=argparse.SUPPRESS)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    args.repo_root = args.repo_root.resolve()
    args.build_dir = args.build_dir.resolve()
    args.project = args.project.resolve()
    args.runtime_package_dir = args.runtime_package_dir.resolve()
    args.source_root = args.source_root.resolve()
    args.manifest = args.manifest.resolve()
    args.output_dir = args.output_dir.resolve()
    args.drive_output_dir = args.drive_output_dir.resolve()
    args.dates = args.dates[:1] if args.smoke else list(args.dates)
    if (
        args.width <= 0
        or args.height <= LABEL_BAND_HEIGHT
        or not 1 <= args.jpeg_quality <= 100
        or args.samples <= 0
        or args.bounces < 0
    ):
        raise ValueError("invalid image dimensions or JPEG quality")

    dependencies = require_artifact_dependencies()
    rows = read_csv(args.manifest)
    settings_by_date = {date: date_settings(rows, date) for date in args.dates}
    scenes = {date: four_by_ten_scene(date) for date in args.dates}
    for date, scene in scenes.items():
        if not (args.project.parent / "Assets" / scene).exists():
            raise FileNotFoundError(f"missing scene for {date}: {scene}")
    if args.worker_date:
        if not args.worker_output_dir:
            raise ValueError("review worker requires --worker-output-dir")
        worker_main(args, rows, dependencies)
        print(f"worker_date={args.worker_date}")
        return

    snapshot = protected_files(args, args.dates)
    args.output_dir.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="sorghum_mobile_review_", dir=args.output_dir.parent) as temporary:
        temporary_root = Path(temporary)
        package_root = temporary_root / "package"
        package_root.mkdir()
        records_by_date: dict[str, dict[str, object]] = {}
        paths_by_date: dict[str, dict[str, Path]] = {}
        try:
            probe_bounds: list[Bounds] = []
            expected_layout: dict[str, list[float]] | None = None
            for date in args.dates:
                probe_output = temporary_root / "probes" / date
                result = run_worker(args, date, probe_output, None, True)
                probe_bounds.append(bounds_from_json(result["shared_bounds"]))
                if expected_layout is None:
                    expected_layout = result["layout"]
                elif not layouts_match(result["layout"], expected_layout):
                    raise RuntimeError(f"{date}: world-space plant layout differs from the first date")

            fixed_bounds = union_bounds(probe_bounds)
            context_path = temporary_root / "worker_context.json"
            context_path.write_text(
                json.dumps(
                    {"shared_bounds": bounds_to_json(fixed_bounds), "layout": expected_layout}, indent=2
                ),
                encoding="utf-8",
            )

            for date in args.dates:
                worker_output = temporary_root / "workers" / date
                result = run_worker(args, date, worker_output, context_path)
                record = result["scene"]
                for folder in ("raw", "stills"):
                    target = package_root / folder
                    target.mkdir(exist_ok=True)
                    for source in (worker_output / folder).iterdir():
                        shutil.copy2(source, target / source.name)
                records_by_date[date] = record
                paths_by_date[date] = {
                    view["id"]: package_root / view["file"] for view in record["views"]
                }

            records = [records_by_date[date] for date in args.dates]
            paths_by_date = {date: paths_by_date[date] for date in args.dates}

            contact_sheet = package_root / "00_all_dates_overview.jpg"
            make_contact_sheet(contact_sheet, paths_by_date, dependencies)
            for date, paths in paths_by_date.items():
                make_stage_contact_sheet(
                    package_root / "contact_sheets" / f"{growth_stage(date)}.jpg",
                    date,
                    paths,
                    dependencies,
                )
            pdf_path = package_root / "Sorghum_4x10_Scene_Review.pdf"
            make_pdf(
                pdf_path, contact_sheet, paths_by_date, settings_by_date, args.geometry_seed, dependencies
            )
            write_readme(package_root / "README.txt")
            manifest = {
                "package_version": 1,
                "generated_utc": datetime.now(timezone.utc).isoformat(),
                "geometry_seed": args.geometry_seed,
                "resolution": [args.width, args.height],
                "fixed_geometry_bounds": bounds_to_json(fixed_bounds),
                "ray_tracing": {
                    "samples_per_pixel": args.samples,
                    "bounces": args.bounces,
                    "gamma": 2.2,
                },
                "skydome_lighting": {
                    "sun_angles_degrees": list(SUN_ANGLES_DEGREES),
                    "sun_angular_diameter_radians": SUN_ANGULAR_DIAMETER_RADIANS,
                    "sun_intensity": 1.0,
                    "sun_color": [1.0, 1.0, 1.0],
                    "skylight_intensity": 1.0,
                    "ambient_light_intensity": 0.1,
                },
                "scenes": records,
            }
            manifest["files"] = package_hashes(package_root)
            (package_root / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
            validate_package(package_root, args.dates, dependencies)
            replace_local_package(package_root, args.output_dir)
        finally:
            restore_and_fail_on_asset_changes(snapshot)

    if args.publish_drive:
        publish_to_drive(args.output_dir, args.drive_output_dir)
    print(f"output_dir={args.output_dir}")
    print(f"scene_count={len(args.dates)}")
    print(f"still_count={len(args.dates) * len(VIEW_ORDER)}")
    print(f"raw_count={len(args.dates) * len(VIEW_ORDER)}")
    print(f"pdf_pages={1 + len(args.dates) * 3}")
    if args.publish_drive:
        print(f"drive_output_dir={args.drive_output_dir}")


if __name__ == "__main__":
    main()
