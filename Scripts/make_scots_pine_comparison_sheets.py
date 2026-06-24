#!/usr/bin/env python3
"""Build phone-friendly Scots pine synthetic-vs-real comparison sheets."""

from __future__ import annotations

import argparse
import random
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def collect_synthetic_groups(root: Path) -> dict[str, list[Path]]:
    groups: dict[str, list[Path]] = {}
    for path in sorted(root.rglob("*_scene_main_camera_composited.*")):
        if path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        name = path.name
        if "_000" not in name:
            continue
        key = name.split("_000", 1)[0]
        groups.setdefault(key, []).append(path)
    return groups


def image_paths(directory: Path) -> list[Path]:
    return sorted(path for path in directory.iterdir() if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS)


def fit_image(image: Image.Image, size: tuple[int, int]) -> Image.Image:
    image = image.convert("RGB")
    image.thumbnail(size, Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", size, (245, 245, 242))
    x = (size[0] - image.width) // 2
    y = (size[1] - image.height) // 2
    canvas.paste(image, (x, y))
    return canvas


def draw_tile(
    sheet: Image.Image,
    draw: ImageDraw.ImageDraw,
    path: Path | None,
    label: str,
    origin: tuple[int, int],
    tile_size: tuple[int, int],
    font: ImageFont.ImageFont,
) -> None:
    x, y = origin
    if path is not None:
        with Image.open(path) as image:
            fitted = fit_image(image, tile_size)
        sheet.paste(fitted, origin)
    else:
        draw.rectangle((x, y, x + tile_size[0], y + tile_size[1]), fill=(235, 235, 232))
    draw.rectangle((x, y, x + tile_size[0], y + 24), fill=(20, 20, 20))
    draw.text((x + 8, y + 5), label, fill=(255, 255, 255), font=font)


def make_sheet(
    synthetic_paths: list[Path],
    real_paths: list[Path],
    title: str,
    out_path: Path,
    tile_size: tuple[int, int],
) -> None:
    margin = 18
    gap = 10
    center_gap = 26
    title_h = 44
    columns = 4
    rows = 3
    width = margin * 2 + tile_size[0] * columns + gap * 2 + center_gap
    height = margin * 2 + title_h + tile_size[1] * rows + gap * (rows - 1)
    sheet = Image.new("RGB", (width, height), (250, 250, 247))
    draw = ImageDraw.Draw(sheet)
    try:
        title_font = ImageFont.truetype("arial.ttf", 24)
        label_font = ImageFont.truetype("arial.ttf", 15)
    except OSError:
        title_font = ImageFont.load_default()
        label_font = ImageFont.load_default()

    draw.text((margin, margin), title, fill=(15, 15, 15), font=title_font)
    draw.text((margin, margin + 27), "Synthetic", fill=(70, 70, 70), font=label_font)
    real_x = margin + tile_size[0] * 2 + gap + center_gap
    draw.text((real_x, margin + 27), "Real random reference", fill=(70, 70, 70), font=label_font)

    tile_y0 = margin + title_h
    positions: list[tuple[int, int]] = []
    for row in range(rows):
        for col in range(columns):
            extra = center_gap - gap if col >= 2 else 0
            x = margin + col * (tile_size[0] + gap) + extra
            y = tile_y0 + row * (tile_size[1] + gap)
            positions.append((x, y))

    synthetic_slots = [0, 1, 4, 5, 8]
    real_slots = [2, 3, 6, 7, 10]
    for index, slot in enumerate(synthetic_slots):
        path = synthetic_paths[index] if index < len(synthetic_paths) else None
        draw_tile(sheet, draw, path, f"S{index + 1}", positions[slot], tile_size, label_font)
    for index, slot in enumerate(real_slots):
        path = real_paths[index] if index < len(real_paths) else None
        draw_tile(sheet, draw, path, f"R{index + 1}", positions[slot], tile_size, label_font)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out_path, quality=92)


def make_pair_sheet(
    synthetic_path: Path,
    real_path: Path,
    title: str,
    out_path: Path,
    tile_size: tuple[int, int],
) -> None:
    margin = 20
    gap = 24
    title_h = 48
    width = margin * 2 + tile_size[0] * 2 + gap
    height = margin * 2 + title_h + tile_size[1]
    sheet = Image.new("RGB", (width, height), (250, 250, 247))
    draw = ImageDraw.Draw(sheet)
    try:
        title_font = ImageFont.truetype("arial.ttf", 26)
        label_font = ImageFont.truetype("arial.ttf", 18)
    except OSError:
        title_font = ImageFont.load_default()
        label_font = ImageFont.load_default()

    draw.text((margin, margin), title, fill=(15, 15, 15), font=title_font)
    y = margin + title_h
    left = (margin, y)
    right = (margin + tile_size[0] + gap, y)
    draw_tile(sheet, draw, synthetic_path, "Synthetic", left, tile_size, label_font)
    draw_tile(sheet, draw, real_path, "Real random reference", right, tile_size, label_font)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out_path, quality=92)


def format_thickness_label(key: str) -> str:
    if key.startswith("candidate_"):
        parts = key.split("_")
        if len(parts) >= 2 and parts[1].isdigit():
            return f"candidate {int(parts[1])}"
    raw = key.removeprefix("thickness_").removesuffix("mm")
    if raw.isdigit():
        return f"{int(raw) / 100.0:.2f} mm"
    return raw.replace("_", " ")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--synthetic-root", type=Path, required=True)
    parser.add_argument("--real-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--tile-width", type=int, default=330)
    parser.add_argument("--tile-height", type=int, default=245)
    parser.add_argument("--pair", action="store_true", help="Create one large synthetic-vs-real pair per candidate.")
    parser.add_argument(
        "--pair-each-synthetic",
        action="store_true",
        help="Create one large synthetic-vs-real pair for each synthetic image in every candidate group.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rng = random.Random(args.seed)
    real_candidates = image_paths(args.real_dir)
    if len(real_candidates) < 5:
        raise SystemExit(f"Need at least 5 real images in {args.real_dir}")

    groups = collect_synthetic_groups(args.synthetic_root)
    if not groups:
        raise SystemExit(f"No composited synthetic images found under {args.synthetic_root}")

    for key, synthetic_paths in sorted(groups.items()):
        if len(synthetic_paths) < 1:
            continue
        thickness_label = format_thickness_label(key)
        if args.pair_each_synthetic:
            for index, synthetic_path in enumerate(synthetic_paths):
                out_path = args.out_dir / f"{key}_sample_{index + 1:02d}_pair_comparison.jpg"
                make_pair_sheet(
                    synthetic_path,
                    rng.choice(real_candidates),
                    f"Comparison {index + 1}: {thickness_label}",
                    out_path,
                    (args.tile_width, args.tile_height),
                )
                print(out_path)
            continue
        if args.pair:
            out_path = args.out_dir / f"{key}_pair_comparison.jpg"
            make_pair_sheet(
                rng.choice(synthetic_paths),
                rng.choice(real_candidates),
                f"Needle minimum strand thickness: {thickness_label}",
                out_path,
                (args.tile_width, args.tile_height),
            )
        else:
            real_paths = rng.sample(real_candidates, 5)
            out_path = args.out_dir / f"{key}_comparison.jpg"
            make_sheet(
                synthetic_paths[:5],
                real_paths,
                f"Needle minimum strand thickness: {thickness_label}",
                out_path,
                (args.tile_width, args.tile_height),
            )
        print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
