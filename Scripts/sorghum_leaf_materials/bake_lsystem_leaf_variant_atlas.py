#!/usr/bin/env python3
"""Bake multiple LSystem sorghum leaf RGBA captures into one variant atlas."""

from __future__ import annotations

import argparse
import colorsys
import math
import random
import shutil
import struct
import zlib
from pathlib import Path

from bake_sorghum_leaf_atlas import ensure_evefilemeta, read_png_rgba, write_png_rgb


DEFAULT_INPUT_DIR = Path("out/generated_assets/SorghumLeafMaterials/ImageTestLeafVariants/source")
DEFAULT_SHEATH_SOURCE = Path(
    "Resources/DigitalAgricultureProject/Assets/GeneratedAssets/Materials/SorghumLeaves/LeafAtlas/atlas/"
    "sorghum_lsystem_leaf_variants_albedo.png"
)
IMAGE_EXTENSIONS = {".png"}


def repo_root_from_script() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "CMakeLists.txt").exists() and (parent / "Resources").exists():
            return parent
    return Path(__file__).resolve().parents[2]


def is_live_project_asset_path(path: Path) -> bool:
    try:
        relative = path.resolve().relative_to((repo_root_from_script() / "Resources").resolve())
    except ValueError:
        return False
    return len(relative.parts) >= 2 and relative.parts[1] == "Assets"


def write_png_rgba(path: Path, width: int, height: int, rgba: bytearray) -> None:
    def chunk(name: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + name
            + payload
            + struct.pack(">I", zlib.crc32(name + payload) & 0xFFFFFFFF)
        )

    scanlines = bytearray()
    stride = width * 4
    for y in range(height):
        scanlines.append(0)
        scanlines.extend(rgba[y * stride : (y + 1) * stride])

    payload = b"\x89PNG\r\n\x1a\n"
    payload += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
    payload += chunk(b"IDAT", zlib.compress(bytes(scanlines), 6))
    payload += chunk(b"IEND", b"")
    path.write_bytes(payload)


def put_rgba(image: bytearray, width: int, x: int, y: int, color: tuple[int, int, int, int]) -> None:
    idx = (y * width + x) * 4
    image[idx : idx + 4] = bytes(color)


def put_rgb(image: bytearray, width: int, x: int, y: int, color: tuple[int, int, int]) -> None:
    idx = (y * width + x) * 3
    image[idx : idx + 3] = bytes(color)


def clamp_byte(value: float) -> int:
    return max(0, min(255, round(value)))


def luminance(color: tuple[int, int, int, int]) -> float:
    return (0.2126 * color[0] + 0.7152 * color[1] + 0.0722 * color[2]) / 255.0


def sample_rgba_bilinear(width: int, height: int, rgba: bytearray, x: float, y: float) -> tuple[int, int, int, int]:
    x = max(0.0, min(width - 1.0, x))
    y = max(0.0, min(height - 1.0, y))
    x0 = int(math.floor(x))
    y0 = int(math.floor(y))
    x1 = min(width - 1, x0 + 1)
    y1 = min(height - 1, y0 + 1)
    tx = x - x0
    ty = y - y0

    def px(xx: int, yy: int) -> tuple[int, int, int, int]:
        idx = (yy * width + xx) * 4
        return rgba[idx], rgba[idx + 1], rgba[idx + 2], rgba[idx + 3]

    c00 = px(x0, y0)
    c10 = px(x1, y0)
    c01 = px(x0, y1)
    c11 = px(x1, y1)
    out = []
    for channel in range(4):
        top = c00[channel] * (1.0 - tx) + c10[channel] * tx
        bottom = c01[channel] * (1.0 - tx) + c11[channel] * tx
        out.append(round(top * (1.0 - ty) + bottom * ty))
    return out[0], out[1], out[2], out[3]


def dilate_rgb_into_transparent(
    width: int,
    height: int,
    rgba: bytearray,
    iterations: int = 18,
    alpha_threshold: int = 8,
) -> bytearray:
    result = bytearray(rgba)
    filled = bytearray(1 if rgba[i * 4 + 3] > alpha_threshold else 0 for i in range(width * height))
    offsets = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, -1), (-1, 1), (1, 1))

    for _ in range(iterations):
        next_filled = bytearray(filled)
        changed = False
        for y in range(height):
            for x in range(width):
                pixel = y * width + x
                if filled[pixel]:
                    continue
                r = g = b = count = 0
                for dx, dy in offsets:
                    xx = x + dx
                    yy = y + dy
                    if xx < 0 or yy < 0 or xx >= width or yy >= height:
                        continue
                    neighbor = yy * width + xx
                    if not filled[neighbor]:
                        continue
                    idx = neighbor * 4
                    r += result[idx]
                    g += result[idx + 1]
                    b += result[idx + 2]
                    count += 1
                if count == 0:
                    continue
                idx = pixel * 4
                result[idx] = r // count
                result[idx + 1] = g // count
                result[idx + 2] = b // count
                next_filled[pixel] = 1
                changed = True
        filled = next_filled
        if not changed:
            break

    return result


def discover_inputs(input_dir: Path) -> list[Path]:
    if not input_dir.exists():
        raise FileNotFoundError(input_dir)
    files = sorted(p for p in input_dir.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS)
    if len(files) != 9:
        raise ValueError(f"Expected exactly 9 RGBA leaf inputs in {input_dir}, found {len(files)}")
    return files


def make_color_jitter(rng: random.Random) -> tuple[float, float, float]:
    return rng.uniform(-0.025, 0.025), rng.uniform(0.88, 1.08), rng.uniform(0.86, 1.12)


def apply_color_jitter(color: tuple[int, int, int, int], jitter: tuple[float, float, float]) -> tuple[int, int, int, int]:
    h, s, v = colorsys.rgb_to_hsv(color[0] / 255.0, color[1] / 255.0, color[2] / 255.0)
    hue_offset, saturation_scale, value_scale = jitter
    h = (h + hue_offset) % 1.0
    s = max(0.0, min(1.0, s * saturation_scale))
    v = max(0.0, min(1.0, v * value_scale))
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    return round(r * 255.0), round(g * 255.0), round(b * 255.0), 255


def load_sheath_source(path: Path | None) -> tuple[int, int, bytearray] | None:
    if path is None or not path.exists():
        return None
    return read_png_rgba(path)


def procedural_sheath_color(u: float, v: float, jitter: tuple[float, float, float]) -> tuple[int, int, int, int]:
    fiber = 0.5 + 0.5 * math.sin(u * math.tau * 18.0 + 0.4 * math.sin(v * math.tau * 5.0))
    band = 0.5 + 0.5 * math.sin(v * math.tau * 6.0)
    r = round(76 + 18 * fiber + 9 * band)
    g = round(112 + 24 * fiber + 10 * band)
    b = round(49 + 11 * fiber + 6 * band)
    return apply_color_jitter((r, g, b, 255), jitter)


def height_to_normal(
    height_values: list[float],
    tile_size: int,
    x: int,
    y: int,
    y_min: int,
    y_max: int,
    strength: float,
) -> tuple[int, int, int]:
    x0 = max(0, x - 1)
    x1 = min(tile_size - 1, x + 1)
    y0 = max(y_min, y - 1)
    y1 = min(y_max, y + 1)
    dx = height_values[y * tile_size + x1] - height_values[y * tile_size + x0]
    dy = height_values[y1 * tile_size + x] - height_values[y0 * tile_size + x]
    nx = -dx * strength
    ny = -dy * strength
    nz = 1.0
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    return (
        clamp_byte(127.5 + 127.5 * nx / length),
        clamp_byte(127.5 + 127.5 * ny / length),
        clamp_byte(127.5 + 127.5 * nz / length),
    )


def bake_variant_atlas(
    inputs: list[Path],
    out_dir: Path,
    sheath_source_path: Path | None,
    tile_size: int,
    columns: int,
    rows: int,
) -> dict[str, Path]:
    if columns * rows < len(inputs):
        raise ValueError("Atlas grid is too small for the input count")
    if tile_size < 256 or tile_size % 2 != 0:
        raise ValueError("--tile-size must be an even integer >= 256")

    out_dir.mkdir(parents=True, exist_ok=True)
    source_dir = out_dir / "source"
    atlas_dir = out_dir / "atlas"
    source_dir.mkdir(parents=True, exist_ok=True)
    atlas_dir.mkdir(parents=True, exist_ok=True)

    atlas_w = columns * tile_size
    atlas_h = rows * tile_size
    half = tile_size // 2
    albedo = bytearray(atlas_w * atlas_h * 4)
    normal = bytearray(atlas_w * atlas_h * 3)
    roughness = bytearray(atlas_w * atlas_h * 3)
    metallic = bytearray(atlas_w * atlas_h * 3)
    ao = bytearray(atlas_w * atlas_h * 3)
    height = bytearray(atlas_w * atlas_h * 3)
    preview = bytearray(atlas_w * atlas_h * 3)

    sheath_source = load_sheath_source(sheath_source_path)

    for tile_index, input_path in enumerate(inputs):
        src_w, src_h, src_rgba = read_png_rgba(input_path)
        src_rgba = dilate_rgb_into_transparent(src_w, src_h, src_rgba)
        tile_x = tile_index % columns
        tile_y = tile_index // columns
        x0 = tile_x * tile_size
        y0 = tile_y * tile_size
        rng = random.Random(0x51F00D00 + tile_index)
        sheath_jitter = make_color_jitter(rng)
        blade_vein_frequency = 42.0 + rng.uniform(-3.0, 3.0)
        sheath_fiber_frequency = 17.0 + rng.uniform(-1.5, 1.5)
        source_copy = source_dir / f"leaf_variant_{tile_index + 1:02d}{input_path.suffix.lower()}"
        if input_path.resolve() != source_copy.resolve():
            shutil.copy2(input_path, source_copy)
        ensure_evefilemeta(source_copy, "Texture2D")

        tile_albedo = bytearray(tile_size * tile_size * 4)
        tile_height = [0.0] * (tile_size * tile_size)
        tile_roughness = bytearray(tile_size * tile_size)
        tile_ao = bytearray(tile_size * tile_size)

        for y in range(tile_size):
            local_v = y / max(1, tile_size - 1)
            if y < half:
                region_v = local_v * 2.0
                src_y = region_v * (src_h - 1)
            else:
                region_v = (local_v - 0.5) * 2.0
            for x in range(tile_size):
                local_u = x / max(1, tile_size - 1)
                pixel = y * tile_size + x
                if y < half:
                    color = sample_rgba_bilinear(src_w, src_h, src_rgba, local_u * (src_w - 1), src_y)
                    midrib = math.exp(-((local_u - 0.5) / 0.035) ** 2)
                    vein = 0.5 + 0.5 * math.sin(local_u * math.tau * blade_vein_frequency + region_v * 7.5)
                    fine_vein = 0.5 + 0.5 * math.sin((local_u * 93.0 + region_v * 5.0 + tile_index * 0.19) * math.tau)
                    alpha = color[3] / 255.0
                    visible = alpha**0.6
                    leaf_luma = luminance(color)
                    tile_height[pixel] = visible * (0.28 + 0.32 * midrib + 0.055 * vein + 0.025 * fine_vein)
                    rough = 156 + 20 * (1.0 - midrib) + 12 * vein + 8 * (1.0 - leaf_luma)
                    occ = 251 - 13 * midrib - 5 * (1.0 - vein) - 20 * (1.0 - alpha)
                    if color[3] <= 8:
                        rough = 190
                        occ = 255
                else:
                    sheath_v = region_v
                    if sheath_source:
                        sh_w, sh_h, sh_rgba = sheath_source
                        color = sample_rgba_bilinear(
                            sh_w,
                            sh_h,
                            sh_rgba,
                            local_u * (sh_w - 1),
                            (0.5 + sheath_v * 0.5) * (sh_h - 1),
                        )
                        color = apply_color_jitter(color, sheath_jitter)
                    else:
                        color = procedural_sheath_color(local_u, sheath_v, sheath_jitter)
                    fiber = 0.5 + 0.5 * math.sin(local_u * math.tau * sheath_fiber_frequency + 0.35 * math.sin(sheath_v * math.tau * 5.0))
                    fold = 0.5 + 0.5 * math.sin(local_u * math.tau * 6.0 + sheath_v * math.tau * 1.75)
                    band = 0.5 + 0.5 * math.sin(sheath_v * math.tau * 6.0)
                    tile_height[pixel] = 0.32 + 0.11 * fiber + 0.075 * fold + 0.025 * band
                    rough = 181 + 17 * (1.0 - fiber) + 8 * band
                    occ = 241 - 12 * (1.0 - fold) - 4 * band

                tile_offset = pixel * 4
                tile_albedo[tile_offset : tile_offset + 4] = bytes(color)
                tile_roughness[pixel] = clamp_byte(rough)
                tile_ao[pixel] = clamp_byte(occ)

        for y in range(tile_size):
            normal_y_min = 0 if y < half else half
            normal_y_max = half - 1 if y < half else tile_size - 1
            for x in range(tile_size):
                pixel = y * tile_size + x
                dst_x = x0 + x
                dst_y = y0 + y
                alpha = tile_albedo[pixel * 4 + 3]
                if y < half and alpha <= 8:
                    n = (128, 128, 255)
                else:
                    n = height_to_normal(tile_height, tile_size, x, y, normal_y_min, normal_y_max, 18.0)
                height_byte = clamp_byte(tile_height[pixel] * 255.0)
                color = tuple(tile_albedo[pixel * 4 : pixel * 4 + 4])
                rough = tile_roughness[pixel]
                occ = tile_ao[pixel]

                put_rgba(albedo, atlas_w, dst_x, dst_y, color)
                put_rgb(normal, atlas_w, dst_x, dst_y, n)
                put_rgb(roughness, atlas_w, dst_x, dst_y, (rough, rough, rough))
                put_rgb(metallic, atlas_w, dst_x, dst_y, (0, 0, 0))
                put_rgb(ao, atlas_w, dst_x, dst_y, (occ, occ, occ))
                put_rgb(height, atlas_w, dst_x, dst_y, (height_byte, height_byte, height_byte))
                put_rgb(preview, atlas_w, dst_x, dst_y, color[:3])

    outputs = {
        "albedo": atlas_dir / "sorghum_lsystem_leaf_variants_albedo.png",
        "normal": atlas_dir / "sorghum_lsystem_leaf_variants_normal.png",
        "roughness": atlas_dir / "sorghum_lsystem_leaf_variants_roughness.png",
        "metallic": atlas_dir / "sorghum_lsystem_leaf_variants_metallic.png",
        "ao": atlas_dir / "sorghum_lsystem_leaf_variants_ao.png",
        "height": atlas_dir / "sorghum_lsystem_leaf_variants_height.png",
        "preview": atlas_dir / "sorghum_lsystem_leaf_variants_preview.png",
    }
    write_png_rgba(outputs["albedo"], atlas_w, atlas_h, albedo)
    write_png_rgb(outputs["normal"], atlas_w, atlas_h, normal)
    write_png_rgb(outputs["roughness"], atlas_w, atlas_h, roughness)
    write_png_rgb(outputs["metallic"], atlas_w, atlas_h, metallic)
    write_png_rgb(outputs["ao"], atlas_w, atlas_h, ao)
    write_png_rgb(outputs["height"], atlas_w, atlas_h, height)
    write_png_rgb(outputs["preview"], atlas_w, atlas_h, preview)

    for path in outputs.values():
        ensure_evefilemeta(path, "Texture2D")

    readme = atlas_dir / "README.md"
    readme.write_text(
        "# LSystem Sorghum Leaf Variant Atlas\n\n"
        "This atlas packs nine RGBA sorghum leaf variants for the LSystem sorghum renderer.\n\n"
        "- Grid: 3 columns x 3 rows.\n"
        "- Each tile: top half stores neck + blade with source alpha; bottom half stores generated sheath albedo.\n"
        "- The albedo atlas is RGBA so the standard material alpha discard clips leaf edges directly.\n"
        "- Transparent RGB is edge-dilated before atlas sampling to reduce alpha-cut mip halos.\n"
        "- Normal, roughness, AO, and height maps are procedural starter PBR maps derived from leaf midrib, vein, sheath fiber, and alpha-edge height cues.\n"
        "- LSystem leaf UVs choose a tile deterministically per leaf and preserve the historical 0..0.5 sheath / 0.5..1 blade split inside each tile.\n",
        encoding="utf-8",
    )
    ensure_evefilemeta(readme, "Binary")
    outputs["readme"] = readme
    return outputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument(
        "--allow-project-assets",
        action="store_true",
        help="Allow --out-dir under Resources/<Project>/Assets. Prefer out/generated_assets for scratch bakes.",
    )
    parser.add_argument("--sheath-source", type=Path, default=DEFAULT_SHEATH_SOURCE)
    parser.add_argument("--tile-size", type=int, default=1024)
    parser.add_argument("--columns", type=int, default=3)
    parser.add_argument("--rows", type=int, default=3)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if is_live_project_asset_path(args.out_dir) and not args.allow_project_assets:
        raise SystemExit(
            f"Refusing to write generated atlas files into live project assets: {args.out_dir}\n"
            "Use an out/generated_assets path, or pass --allow-project-assets for an intentional promotion."
        )
    input_dir = args.input_dir if args.input_dir.is_absolute() else repo_root_from_script() / args.input_dir
    inputs = discover_inputs(input_dir)
    outputs = bake_variant_atlas(inputs, args.out_dir, args.sheath_source, args.tile_size, args.columns, args.rows)
    print(f"inputs: {len(inputs)} from {input_dir}")
    for key, path in outputs.items():
        print(f"{key}: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
