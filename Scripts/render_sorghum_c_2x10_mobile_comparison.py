#!/usr/bin/env python3
"""Render matched Vulkan RT and CUDA/OptiX views of the August 11 C-only 2x10 field."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import statistics
import sys
from typing import Any, Iterable


EXPERIMENT_ID = "Sorghum2026_GenotypeC_2x10_2026-08-11"
VIEW_GROUPS = {
    "overview": (
        "field_southeast",
        "field_northwest",
        "field_top_down",
        "field_near_top_down",
        "field_along_rows",
        "field_across_rows",
    ),
    "in_field": (
        "aisle_positive_x",
        "aisle_negative_x",
        "row_0_positive_x",
        "row_0_negative_x",
        "row_1_positive_x",
        "row_1_negative_x",
    ),
    "morphology": (
        "plant_front",
        "plant_back",
        "basal_tillers",
        "basal_side",
        "leaf_canopy",
        "panicle",
    ),
}
VIEW_LABELS = {
    "field_southeast": "Whole field | southeast oblique",
    "field_northwest": "Whole field | northwest oblique",
    "field_top_down": "Whole field | true top-down",
    "field_near_top_down": "Whole field | near top-down",
    "field_along_rows": "Whole field | along rows",
    "field_across_rows": "Whole field | across rows",
    "aisle_positive_x": "Inter-row aisle | walking +X",
    "aisle_negative_x": "Inter-row aisle | walking -X",
    "row_0_positive_x": "Row 0 | within-row +X",
    "row_0_negative_x": "Row 0 | within-row -X",
    "row_1_positive_x": "Row 1 | within-row +X",
    "row_1_negative_x": "Row 1 | within-row -X",
    "plant_front": "Genotype C | representative plant front",
    "plant_back": "Genotype C | representative plant back",
    "basal_tillers": "Genotype C | basal culm and tillers",
    "basal_side": "Genotype C | tiller side profile",
    "leaf_canopy": "Genotype C | leaf surface and canopy",
    "panicle": "Genotype C | panicle morphology",
}
PRESENTATION_GRADE = {
    "brightness": 0.94,
    "contrast": 0.95,
    "saturation": 0.90,
    "red_gain": 1.015,
    "green_gain": 1.0,
    "blue_gain": 0.965,
}
VULKAN_RADIOMETRIC_SCALE = 6.0
LIGHTING = {
    "sun_euler_degrees": (55.0, 30.0, 0.0),
    "sun_angular_diameter_radians": 0.012,
    "cuda_sun_intensity": 0.92,
    "vulkan_sun_intensity": 0.92 * VULKAN_RADIOMETRIC_SCALE,
    "sun_color": (1.0, 0.975, 0.94),
    "sky_light_intensity": 0.92,
    "cuda_ambient_light_intensity": 0.14,
    "vulkan_ambient_light_intensity": 0.14 * VULKAN_RADIOMETRIC_SCALE,
    "background_color_linear": (0.794, 0.794, 0.7),
    "gamma": 2.2,
}
DLL_DIRECTORY_HANDLES: list[Any] = []


@dataclass(frozen=True)
class Bounds:
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]

    @property
    def center(self) -> tuple[float, float, float]:
        return tuple((low + high) * 0.5 for low, high in zip(self.minimum, self.maximum))

    def corners(self) -> Iterable[tuple[float, float, float]]:
        return itertools.product(*zip(self.minimum, self.maximum))


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    project_root = root / "Resources" / "DigitalAgricultureProject"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install-root", type=Path, default=root / "out" / "install" / "vs2026-x64-cuda")
    parser.add_argument(
        "--project", type=Path, default=project_root / "test_lsystem_sorghum_genotype_c_aug11_2x10.eveproj"
    )
    parser.add_argument(
        "--scene-manifest",
        type=Path,
        default=project_root / "Data" / "Experiments" / EXPERIMENT_ID / "scene_manifest.csv",
    )
    parser.add_argument("--output-root", type=Path, default=root / "out" / "realism_review" / EXPERIMENT_ID)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--vulkan-samples-per-frame", type=int, default=8)
    parser.add_argument("--vulkan-accumulation-frames", type=int, default=8)
    parser.add_argument("--cuda-samples", type=int, default=64)
    parser.add_argument("--bounces", type=int, default=4)
    parser.add_argument("--cuda-denoiser-strength", type=float, default=0.0)
    parser.add_argument("--view", action="append", choices=tuple(VIEW_LABELS))
    parser.add_argument("--skip-capture", action="store_true")
    parser.add_argument("--smoke", action="store_true")
    return parser.parse_args()


def require_pillow() -> tuple[Any, Any, Any, Any, Any, Any]:
    from PIL import Image, ImageChops, ImageDraw, ImageEnhance, ImageFont, ImageStat

    return Image, ImageChops, ImageDraw, ImageEnhance, ImageFont, ImageStat


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


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
    offset_direction: tuple[float, float, float],
    requested_up: tuple[float, float, float],
    fov_degrees: float,
    aspect_ratio: float,
    padding: float,
) -> dict[str, object]:
    center = bounds.center
    front = vec_scale(normalize(offset_direction), -1.0)
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
    position = vec_sub(center, vec_scale(front, distance * padding))
    return {"position": position, "target": center, "up": up, "fov_degrees": fov_degrees}


def focus_camera(
    target: tuple[float, float, float], direction: tuple[float, float, float], distance: float, fov_degrees: float
) -> dict[str, object]:
    return {
        "position": vec_add(target, vec_scale(normalize(direction), distance)),
        "target": target,
        "up": (0.0, 1.0, 0.0),
        "fov_degrees": fov_degrees,
    }


def load_layout(path: Path) -> list[dict[str, object]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 20 or {row["genotype_id"] for row in rows} != {"GenotypeC"}:
        raise RuntimeError("the scene manifest is not the expected 20-plant Genotype C experiment")
    return [
        {
            "name": row["plant_name"],
            "row": int(row["row"]),
            "column": int(row["column"]),
            "root": (float(row["x_m"]), float(row["y_m"]), float(row["z_m"])),
            "height": float(row["plant_height_m"]),
        }
        for row in rows
    ]


def derive_cameras(layout: list[dict[str, object]], aspect: float) -> tuple[Bounds, str, dict[str, dict[str, object]]]:
    roots = [row["root"] for row in layout]
    heights = [float(row["height"]) for row in layout]
    median_height = statistics.median(heights)
    horizontal_pad = max(0.85, median_height * 0.38)
    depth_pad = max(0.90, median_height * 0.42)
    field = Bounds(
        (
            min(root[0] for root in roots) - horizontal_pad,
            min(root[1] for root in roots) - 0.24,
            min(root[2] for root in roots) - depth_pad,
        ),
        (
            max(root[0] for root in roots) + horizontal_pad,
            max(root[1] + height for root, height in zip(roots, heights)) + 0.15,
            max(root[2] for root in roots) + depth_pad,
        ),
    )
    cameras = {
        "field_southeast": fit_camera(field, (1.0, 0.38, 1.0), (0.0, 1.0, 0.0), 42.0, aspect, 1.08),
        "field_northwest": fit_camera(field, (-1.0, 0.38, -1.0), (0.0, 1.0, 0.0), 42.0, aspect, 1.08),
        "field_top_down": fit_camera(field, (0.001, 1.0, 0.001), (0.0, 0.0, -1.0), 45.0, aspect, 1.08),
        "field_near_top_down": fit_camera(field, (0.22, 1.0, 0.18), (0.0, 0.0, -1.0), 44.0, aspect, 1.08),
        "field_along_rows": fit_camera(field, (1.0, 0.20, 0.05), (0.0, 1.0, 0.0), 48.0, aspect, 1.06),
        "field_across_rows": fit_camera(field, (0.05, 0.22, 1.0), (0.0, 1.0, 0.0), 48.0, aspect, 1.06),
    }

    x_min = min(root[0] for root in roots)
    x_max = max(root[0] for root in roots)
    x_center = (x_min + x_max) * 0.5
    row_z = {row: statistics.fmean(item["root"][2] for item in layout if item["row"] == row) for row in (0, 1)}
    root_y = statistics.fmean(root[1] for root in roots)
    eye_y = root_y + 1.65
    aisle_z = statistics.fmean(row_z.values())
    for prefix, z_value in (("aisle", aisle_z), ("row_0", row_z[0]), ("row_1", row_z[1])):
        cameras[f"{prefix}_positive_x"] = {
            "position": (x_center, eye_y, z_value),
            "target": (x_max + 1.0, eye_y - 0.30, z_value),
            "up": (0.0, 1.0, 0.0),
            "fov_degrees": 58.0,
        }
        cameras[f"{prefix}_negative_x"] = {
            "position": (x_center, eye_y, z_value),
            "target": (x_min - 1.0, eye_y - 0.30, z_value),
            "up": (0.0, 1.0, 0.0),
            "fov_degrees": 58.0,
        }

    center_x = statistics.fmean(root[0] for root in roots)
    span_x = x_max - x_min
    representative = min(
        layout,
        key=lambda row: (
            abs(abs(row["root"][0] - center_x) - 0.28 * span_x) / max(span_x, 0.01)
            + 0.35 * abs(float(row["height"]) - median_height) / max(median_height, 0.01),
            row["name"],
        ),
    )
    root = representative["root"]
    height = max(float(representative["height"]), 0.1)
    plant_bounds = Bounds(
        (root[0] - max(0.55, 0.34 * height), root[1] - 0.05, root[2] - max(0.65, 0.42 * height)),
        (root[0] + max(0.55, 0.34 * height), root[1] + height, root[2] + max(0.65, 0.42 * height)),
    )
    cameras.update(
        {
            "plant_front": fit_camera(plant_bounds, (0.22, 0.20, 1.0), (0.0, 1.0, 0.0), 46.0, aspect, 1.10),
            "plant_back": fit_camera(plant_bounds, (-0.22, 0.20, -1.0), (0.0, 1.0, 0.0), 46.0, aspect, 1.10),
            "basal_tillers": focus_camera(
                (root[0], root[1] + 0.16 * height, root[2]), (0.25, 0.12, 1.0), max(0.72, 0.42 * height), 38.0
            ),
            "basal_side": focus_camera(
                (root[0], root[1] + 0.18 * height, root[2]), (1.0, 0.12, 0.18), max(0.72, 0.42 * height), 38.0
            ),
            "leaf_canopy": focus_camera(
                (root[0], root[1] + 0.62 * height, root[2]), (-0.30, 0.12, 1.0), max(0.68, 0.36 * height), 34.0
            ),
            "panicle": focus_camera(
                (root[0], root[1] + 0.90 * height, root[2]), (0.25, 0.10, -1.0), max(0.78, 0.40 * height), 32.0
            ),
        }
    )
    return field, str(representative["name"]), cameras


def configure_module_search(install_root: Path) -> None:
    for directory in (install_root / "python", install_root / "bin", install_root / "bin" / "Packages"):
        if directory.is_dir():
            if hasattr(os, "add_dll_directory"):
                DLL_DIRECTORY_HANDLES.append(os.add_dll_directory(str(directory)))
    cuda_path = Path(os.environ.get("CUDA_PATH", "")) / "bin"
    if cuda_path.is_dir() and hasattr(os, "add_dll_directory"):
        DLL_DIRECTORY_HANDLES.append(os.add_dll_directory(str(cuda_path)))
    sys.path.insert(0, str(install_root / "python"))
    os.chdir(install_root / "bin")


def evo_vec3(evo: Any, values: tuple[float, float, float]) -> Any:
    result = evo.Vec3()
    result.x, result.y, result.z = values
    return result


def apply_camera(evo: Any, camera: dict[str, object]) -> None:
    if not evo.SetMainCameraLookAt(
        evo_vec3(evo, camera["position"]),
        evo_vec3(evo, camera["target"]),
        evo_vec3(evo, camera["up"]),
        float(camera["fov_degrees"]),
    ):
        raise RuntimeError("failed to apply camera")


def grade_image(source: Path, destination: Path) -> None:
    Image, _, _, ImageEnhance, _, _ = require_pillow()
    image = Image.open(source).convert("RGB")
    image = ImageEnhance.Brightness(image).enhance(PRESENTATION_GRADE["brightness"])
    image = ImageEnhance.Contrast(image).enhance(PRESENTATION_GRADE["contrast"])
    image = ImageEnhance.Color(image).enhance(PRESENTATION_GRADE["saturation"])
    red, green, blue = image.split()
    red = red.point(lambda value: min(255, round(value * PRESENTATION_GRADE["red_gain"])))
    green = green.point(lambda value: min(255, round(value * PRESENTATION_GRADE["green_gain"])))
    blue = blue.point(lambda value: min(255, round(value * PRESENTATION_GRADE["blue_gain"])))
    destination.parent.mkdir(parents=True, exist_ok=True)
    Image.merge("RGB", (red, green, blue)).save(destination, compress_level=6)


def image_metrics(path: Path) -> dict[str, object]:
    Image, _, _, _, _, ImageStat = require_pillow()
    image = Image.open(path).convert("RGB")
    stats = ImageStat.Stat(image)
    return {
        "size": list(image.size),
        "rgb_mean": [round(value, 5) for value in stats.mean],
        "rgb_stddev": [round(value, 5) for value in stats.stddev],
    }


def pair_metrics(cuda_path: Path, vulkan_path: Path) -> dict[str, object]:
    Image, ImageChops, _, _, _, ImageStat = require_pillow()
    cuda = Image.open(cuda_path).convert("RGB")
    vulkan = Image.open(vulkan_path).convert("RGB")
    difference = ImageChops.difference(cuda, vulkan)
    return {"mean_absolute_rgb_difference": [round(value, 5) for value in ImageStat.Stat(difference).mean]}


def make_mobile_sheet(records: list[dict[str, object]], group: str, destination: Path) -> None:
    Image, _, ImageDraw, _, ImageFont, _ = require_pillow()
    canvas = Image.new("RGB", (1080, 1920), (17, 21, 24))
    draw = ImageDraw.Draw(canvas)
    regular = Path(r"C:\Windows\Fonts\segoeui.ttf")
    bold = Path(r"C:\Windows\Fonts\segoeuib.ttf")
    title_font = ImageFont.truetype(str(bold), 34) if bold.exists() else ImageFont.load_default()
    label_font = ImageFont.truetype(str(regular), 22) if regular.exists() else ImageFont.load_default()
    renderer_font = ImageFont.truetype(str(bold), 17) if bold.exists() else ImageFont.load_default()
    draw.text((30, 18), f"Genotype C 2x10 | {group.replace('_', ' ').title()}", font=title_font, fill=(245, 248, 247))
    draw.text((30, 66), "CUDA / OptiX and Vulkan ray tracing | matched cameras", font=label_font, fill=(190, 205, 210))
    for index, record in enumerate(records):
        y = 120 + index * 300
        draw.text((50, y), str(record["label"]), font=label_font, fill=(232, 237, 235))
        cuda = Image.open(record["cuda_presentation"]).convert("RGB").resize((480, 270), Image.Resampling.LANCZOS)
        vulkan = Image.open(record["vulkan_presentation"]).convert("RGB").resize((480, 270), Image.Resampling.LANCZOS)
        canvas.paste(cuda, (50, y + 28))
        canvas.paste(vulkan, (550, y + 28))
        draw.rectangle((50, y + 28, 205, y + 55), fill=(19, 33, 42))
        draw.rectangle((550, y + 28, 705, y + 55), fill=(44, 33, 18))
        draw.text((60, y + 31), "CUDA / OptiX", font=renderer_font, fill=(130, 205, 255))
        draw.text((560, y + 31), "Vulkan RT", font=renderer_font, fill=(255, 196, 105))
    destination.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(destination, compress_level=6)


def capture(args: argparse.Namespace, selected: list[str], cameras: dict[str, dict[str, object]]) -> None:
    configure_module_search(args.install_root)
    import PyEvoEngine as evo

    project_bytes = args.project.read_bytes()
    evo.PushRayTracerLayer()
    if not evo.RunWindowless(str(args.project)):
        raise RuntimeError("EvoEngine failed to start the C-only project")
    try:
        if not evo.WaitForCurrentSceneReady(30000):
            raise RuntimeError("the C-only scene did not become ready")
        if not evo.ConfigureCurrentSceneOutdoorLightingForCapture(
            evo_vec3(evo, LIGHTING["sun_euler_degrees"]),
            LIGHTING["sun_angular_diameter_radians"],
            LIGHTING["vulkan_sun_intensity"],
            evo_vec3(evo, LIGHTING["sun_color"]),
            LIGHTING["sky_light_intensity"],
            LIGHTING["vulkan_ambient_light_intensity"],
            evo_vec3(evo, LIGHTING["background_color_linear"]),
            LIGHTING["gamma"],
        ):
            raise RuntimeError("failed to configure Vulkan outdoor lighting")
        if not evo.ConfigureCurrentSceneCudaOutdoorLightingForCapture(
            evo_vec3(evo, LIGHTING["sun_euler_degrees"]),
            LIGHTING["sun_angular_diameter_radians"],
            LIGHTING["cuda_sun_intensity"],
            evo_vec3(evo, LIGHTING["sun_color"]),
            LIGHTING["sky_light_intensity"],
            LIGHTING["cuda_ambient_light_intensity"],
            LIGHTING["gamma"],
        ):
            raise RuntimeError("failed to configure CUDA/OptiX outdoor lighting")
        raw_vulkan = args.output_root / "full_resolution" / "vulkan_rt" / "raw"
        raw_cuda = args.output_root / "full_resolution" / "cuda_optix" / "raw"
        for view in selected:
            index = list(VIEW_LABELS).index(view) + 1
            apply_camera(evo, cameras[view])
            if not evo.ConfigureCurrentSceneCameraForCapture(
                "RayTracing", args.vulkan_samples_per_frame, args.bounces
            ):
                raise RuntimeError(f"Vulkan RT is unavailable for {view}")
            if not evo.CaptureCurrentScene(
                args.width,
                args.height,
                str((raw_vulkan / f"{index:02d}_{view}.png").resolve()),
                args.vulkan_accumulation_frames,
                True,
            ):
                raise RuntimeError(f"Vulkan capture failed for {view}")
            if not evo.CaptureCurrentSceneCuda(
                args.width,
                args.height,
                str((raw_cuda / f"{index:02d}_{view}.png").resolve()),
                args.cuda_samples,
                args.bounces,
                LIGHTING["gamma"],
                args.cuda_denoiser_strength,
            ):
                raise RuntimeError(f"CUDA/OptiX capture failed for {view}")
            print(f"[{selected.index(view) + 1:02d}/{len(selected):02d}] {view}", flush=True)
    finally:
        evo.Terminate()
        args.project.write_bytes(project_bytes)


def build_package(
    args: argparse.Namespace,
    selected: list[str],
    field_bounds: Bounds,
    representative: str,
    cameras: dict[str, dict[str, object]],
) -> None:
    records: list[dict[str, object]] = []
    raw_vulkan = args.output_root / "full_resolution" / "vulkan_rt" / "raw"
    raw_cuda = args.output_root / "full_resolution" / "cuda_optix" / "raw"
    presentation_vulkan = args.output_root / "full_resolution" / "vulkan_rt" / "presentation"
    presentation_cuda = args.output_root / "full_resolution" / "cuda_optix" / "presentation"
    for view in selected:
        index = list(VIEW_LABELS).index(view) + 1
        vulkan_raw = raw_vulkan / f"{index:02d}_{view}.png"
        cuda_raw = raw_cuda / f"{index:02d}_{view}.png"
        if not vulkan_raw.is_file() or not cuda_raw.is_file():
            raise FileNotFoundError(f"matched captures are incomplete for {view}")
        vulkan_final = presentation_vulkan / vulkan_raw.name
        cuda_final = presentation_cuda / cuda_raw.name
        grade_image(vulkan_raw, vulkan_final)
        grade_image(cuda_raw, cuda_final)
        metrics = {
            "view": view,
            "label": VIEW_LABELS[view],
            "camera": cameras[view],
            "vulkan_raw": str(vulkan_raw.resolve()),
            "cuda_raw": str(cuda_raw.resolve()),
            "vulkan_presentation": str(vulkan_final.resolve()),
            "cuda_presentation": str(cuda_final.resolve()),
            "vulkan_sha256": sha256(vulkan_raw),
            "cuda_sha256": sha256(cuda_raw),
            "vulkan_metrics": image_metrics(vulkan_raw),
            "cuda_metrics": image_metrics(cuda_raw),
            **pair_metrics(cuda_final, vulkan_final),
        }
        if min(metrics["vulkan_metrics"]["rgb_stddev"]) < 2 or min(metrics["cuda_metrics"]["rgb_stddev"]) < 2:
            raise RuntimeError(f"{view} appears blank or flat")
        records.append(metrics)

    sheets = {}
    for group, group_views in VIEW_GROUPS.items():
        group_records = [record for record in records if record["view"] in group_views]
        if len(group_records) == 6:
            destination = args.output_root / "mobile" / f"{group}_1080x1920.png"
            make_mobile_sheet(group_records, group, destination)
            sheets[group] = str(destination.resolve())
    manifest = {
        "schema_version": 1,
        "experiment_id": EXPERIMENT_ID,
        "project": str(args.project.resolve()),
        "camera_policy": "field framing and walking locations derive from the persisted 20-plant layout; morphology framing derives from a representative measured plant envelope",
        "field_bounds": {"minimum": field_bounds.minimum, "maximum": field_bounds.maximum},
        "representative_plant": representative,
        "resolution": [args.width, args.height],
        "vulkan": {
            "renderer": "Vulkan RayTracing",
            "samples_per_frame": args.vulkan_samples_per_frame,
            "accumulation_frames": args.vulkan_accumulation_frames,
            "total_spp": args.vulkan_samples_per_frame * args.vulkan_accumulation_frames,
        },
        "cuda": {
            "renderer": "CUDA / OptiX",
            "samples": args.cuda_samples,
            "denoiser_strength": args.cuda_denoiser_strength,
        },
        "bounces": args.bounces,
        "lighting": LIGHTING,
        "presentation_grade": PRESENTATION_GRADE,
        "view_count": len(records),
        "views": records,
        "mobile_sheets": sheets,
    }
    args.output_root.mkdir(parents=True, exist_ok=True)
    (args.output_root / "render_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    require_pillow()
    args.install_root = args.install_root.resolve()
    args.project = args.project.resolve()
    args.scene_manifest = args.scene_manifest.resolve()
    args.output_root = args.output_root.resolve()
    if min(args.width, args.height, args.vulkan_samples_per_frame, args.vulkan_accumulation_frames, args.cuda_samples) <= 0:
        raise ValueError("capture dimensions and sample counts must be positive")
    layout = load_layout(args.scene_manifest)
    field_bounds, representative, cameras = derive_cameras(layout, args.width / args.height)
    selected = list(args.view or VIEW_LABELS)
    if args.smoke:
        selected = selected[:1]
    if not args.skip_capture:
        capture(args, selected, cameras)
    build_package(args, selected, field_bounds, representative, cameras)
    print(args.output_root / "render_manifest.json")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise
