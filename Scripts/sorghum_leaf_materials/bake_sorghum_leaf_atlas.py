#!/usr/bin/env python3
"""Bake Gemini sorghum leaf captures into EvoEngine's existing sorghum atlas UV layout.

The current DigitalAgriculture sorghum mesh expects one material atlas:
  - UV V 0.0..0.5: stem/internode texture
  - UV V 0.5..1.0: leaf texture
  - UV U 0.0..1.0: across leaf width or around stem circumference

This script warps a horizontal leaf photo/mask into that layout without requiring
any runtime engine UV changes.
"""

from __future__ import annotations

import argparse
import collections
import hashlib
import math
import shutil
import struct
import zlib
from pathlib import Path


def _paeth(a: int, b: int, c: int) -> int:
    p = a + b - c
    pa = abs(p - a)
    pb = abs(p - b)
    pc = abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c


def read_png_rgba(path: Path) -> tuple[int, int, bytearray]:
    data = path.read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"{path} is not a PNG file")

    pos = 8
    width = height = color_type = bit_depth = None
    compressed = bytearray()
    while pos < len(data):
        length = struct.unpack(">I", data[pos : pos + 4])[0]
        chunk_type = data[pos + 4 : pos + 8]
        chunk_data = data[pos + 8 : pos + 8 + length]
        pos += 12 + length
        if chunk_type == b"IHDR":
            width, height, bit_depth, color_type, compression, filter_method, interlace = struct.unpack(
                ">IIBBBBB", chunk_data
            )
            if bit_depth != 8 or compression != 0 or filter_method != 0 or interlace != 0:
                raise ValueError("Only non-interlaced 8-bit PNG files are supported")
            if color_type not in (0, 2, 4, 6):
                raise ValueError(f"Unsupported PNG color type: {color_type}")
        elif chunk_type == b"IDAT":
            compressed.extend(chunk_data)
        elif chunk_type == b"IEND":
            break

    if width is None or height is None or color_type is None:
        raise ValueError(f"{path} is missing an IHDR chunk")

    channels = {0: 1, 2: 3, 4: 2, 6: 4}[color_type]
    stride = width * channels
    raw = zlib.decompress(bytes(compressed))
    rgba = bytearray(width * height * 4)
    previous = bytearray(stride)
    source_pos = 0

    for y in range(height):
        filter_type = raw[source_pos]
        source_pos += 1
        row = bytearray(raw[source_pos : source_pos + stride])
        source_pos += stride

        for x in range(stride):
            left = row[x - channels] if x >= channels else 0
            up = previous[x]
            up_left = previous[x - channels] if x >= channels else 0
            if filter_type == 1:
                row[x] = (row[x] + left) & 0xFF
            elif filter_type == 2:
                row[x] = (row[x] + up) & 0xFF
            elif filter_type == 3:
                row[x] = (row[x] + ((left + up) // 2)) & 0xFF
            elif filter_type == 4:
                row[x] = (row[x] + _paeth(left, up, up_left)) & 0xFF
            elif filter_type != 0:
                raise ValueError(f"Unsupported PNG filter type: {filter_type}")

        for x in range(width):
            src = x * channels
            dst = (y * width + x) * 4
            if color_type == 0:
                v = row[src]
                rgba[dst : dst + 4] = bytes((v, v, v, 255))
            elif color_type == 2:
                rgba[dst : dst + 4] = bytes((row[src], row[src + 1], row[src + 2], 255))
            elif color_type == 4:
                v = row[src]
                rgba[dst : dst + 4] = bytes((v, v, v, row[src + 1]))
            else:
                rgba[dst : dst + 4] = row[src : src + 4]
        previous = row
    return width, height, rgba


def write_png_rgb(path: Path, width: int, height: int, rgb: bytearray) -> None:
    def chunk(name: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + name
            + payload
            + struct.pack(">I", zlib.crc32(name + payload) & 0xFFFFFFFF)
        )

    scanlines = bytearray()
    stride = width * 3
    for y in range(height):
        scanlines.append(0)
        scanlines.extend(rgb[y * stride : (y + 1) * stride])

    payload = b"\x89PNG\r\n\x1a\n"
    payload += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    payload += chunk(b"IDAT", zlib.compress(bytes(scanlines), 6))
    payload += chunk(b"IEND", b"")
    path.write_bytes(payload)


def largest_component(mask: bytearray, width: int, height: int) -> bytearray:
    visited = bytearray(width * height)
    best: list[int] = []
    neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1))
    for start in range(width * height):
        if not mask[start] or visited[start]:
            continue
        visited[start] = 1
        queue = collections.deque([start])
        component: list[int] = []
        while queue:
            idx = queue.popleft()
            component.append(idx)
            x = idx % width
            y = idx // width
            for dx, dy in neighbors:
                nx = x + dx
                ny = y + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                ni = ny * width + nx
                if mask[ni] and not visited[ni]:
                    visited[ni] = 1
                    queue.append(ni)
        if len(component) > len(best):
            best = component

    out = bytearray(width * height)
    for idx in best:
        out[idx] = 1
    return out


def clamp_int(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def sample_rgb_bilinear(width: int, height: int, rgba: bytearray, x: float, y: float) -> tuple[int, int, int]:
    x = max(0.0, min(width - 1.0, x))
    y = max(0.0, min(height - 1.0, y))
    x0 = int(math.floor(x))
    y0 = int(math.floor(y))
    x1 = min(width - 1, x0 + 1)
    y1 = min(height - 1, y0 + 1)
    tx = x - x0
    ty = y - y0

    def px(xx: int, yy: int) -> tuple[int, int, int]:
        idx = (yy * width + xx) * 4
        return rgba[idx], rgba[idx + 1], rgba[idx + 2]

    c00 = px(x0, y0)
    c10 = px(x1, y0)
    c01 = px(x0, y1)
    c11 = px(x1, y1)
    out = []
    for channel in range(3):
        top = c00[channel] * (1.0 - tx) + c10[channel] * tx
        bottom = c01[channel] * (1.0 - tx) + c11[channel] * tx
        out.append(round(top * (1.0 - ty) + bottom * ty))
    return out[0], out[1], out[2]


def build_mask_columns(mask_path: Path, threshold: int) -> tuple[int, int, bytearray, dict[int, tuple[int, int, float]]]:
    width, height, rgba = read_png_rgba(mask_path)
    mask = bytearray(width * height)
    for i in range(width * height):
        r, g, b, a = rgba[i * 4 : i * 4 + 4]
        if a > 0 and (int(r) + int(g) + int(b)) / 3 >= threshold:
            mask[i] = 1

    mask = largest_component(mask, width, height)
    xs = [i % width for i, value in enumerate(mask) if value]
    if not xs:
        raise ValueError("Mask contains no foreground pixels")

    columns: dict[int, tuple[int, int, float]] = {}
    for x in range(min(xs), max(xs) + 1):
        ys = [y for y in range(height) if mask[y * width + x]]
        if ys:
            top = min(ys)
            bottom = max(ys)
            columns[x] = (top, bottom, (top + bottom) * 0.5)
    return width, height, mask, columns


def make_column_lookup(width: int, columns: dict[int, tuple[int, int, float]]) -> list[tuple[int, int, float]]:
    valid_x = sorted(columns)
    if not valid_x:
        raise ValueError("No mask columns were detected")

    lookup: list[tuple[int, int, float]] = [columns[valid_x[0]]] * width
    cursor = 0
    for x in range(width):
        while cursor + 1 < len(valid_x) and abs(valid_x[cursor + 1] - x) <= abs(valid_x[cursor] - x):
            cursor += 1
        lookup[x] = columns[valid_x[cursor]]
    return lookup


def put_rgb(image: bytearray, size: int, x: int, y: int, color: tuple[int, int, int]) -> None:
    idx = (y * size + x) * 3
    image[idx : idx + 3] = bytes((clamp_int(color[0], 0, 255), clamp_int(color[1], 0, 255), clamp_int(color[2], 0, 255)))


def load_optional_map(path: Path | None, expected_size: tuple[int, int], label: str) -> tuple[int, int, bytearray] | None:
    if path is None or not path.exists():
        return None
    width, height, rgba = read_png_rgba(path)
    if (width, height) != expected_size:
        raise ValueError(f"{label} dimensions {(width, height)} do not match albedo/mask dimensions {expected_size}")
    return width, height, rgba


def sample_optional_map(
    source: tuple[int, int, bytearray] | None,
    fallback: tuple[int, int, int],
    x: float,
    y: float,
) -> tuple[int, int, int]:
    if source is None:
        return fallback
    width, height, rgba = source
    return sample_rgb_bilinear(width, height, rgba, x, y)


def grayscale(color: tuple[int, int, int]) -> tuple[int, int, int]:
    value = round(color[0] * 0.299 + color[1] * 0.587 + color[2] * 0.114)
    return value, value, value


def stable_asset_handle(path: Path) -> int:
    digest = hashlib.sha256(str(path.as_posix()).lower().encode("utf-8")).digest()
    value = int.from_bytes(digest[:8], "little")
    return value if value != 0 else 1


def ensure_evefilemeta(path: Path, asset_type_name: str) -> None:
    meta_path = path.with_name(path.name + ".evefilemeta")
    if meta_path.exists():
        return
    meta_path.write_text(
        f"asset_extension_: {path.suffix}\n"
        f"asset_file_name_: {path.stem}\n"
        f"asset_type_name_: {asset_type_name}\n"
        f"asset_handle_: {stable_asset_handle(path)}\n",
        encoding="utf-8",
    )


def copy_if_different(source: Path, destination: Path) -> None:
    if source.resolve() == destination.resolve():
        return
    shutil.copy2(source, destination)


def stem_color(u: float, v: float) -> tuple[int, int, int]:
    fiber = 0.5 + 0.5 * math.sin(u * math.tau * 18.0 + 0.4 * math.sin(v * math.tau * 5.0))
    band = 0.5 + 0.5 * math.sin(v * math.tau * 6.0)
    noise = 0.5 + 0.5 * math.sin((u * 91.7 + v * 37.3) * math.tau)
    r = 76 + 14 * fiber + 8 * band + 4 * noise
    g = 112 + 22 * fiber + 9 * band + 4 * noise
    b = 49 + 10 * fiber + 6 * band
    return round(r), round(g), round(b)


def bake_atlas(
    albedo_path: Path,
    mask_path: Path,
    out_dir: Path,
    size: int,
    threshold: int,
    pbr_dir: Path | None = None,
) -> dict[str, Path]:
    src_w, src_h, src_rgba = read_png_rgba(albedo_path)
    mask_w, mask_h, _mask, columns = build_mask_columns(mask_path, threshold)
    if (src_w, src_h) != (mask_w, mask_h):
        raise ValueError("Albedo and mask dimensions must match")
    expected_size = (src_w, src_h)

    pbr_albedo = load_optional_map(pbr_dir / "albedo.png" if pbr_dir else None, expected_size, "Chord albedo")
    pbr_normal = load_optional_map(pbr_dir / "normal.png" if pbr_dir else None, expected_size, "Chord normal")
    pbr_roughness = load_optional_map(pbr_dir / "roughness.png" if pbr_dir else None, expected_size, "Chord roughness")
    pbr_metallic = load_optional_map(pbr_dir / "metallic.png" if pbr_dir else None, expected_size, "Chord metallic")
    leaf_albedo_source = pbr_albedo if pbr_albedo is not None else (src_w, src_h, src_rgba)

    tip_x = min(columns)
    neck_x = max(columns)
    column_lookup = make_column_lookup(src_w, columns)
    half = size // 2
    u_values = [x / max(1, size - 1) for x in range(size)]

    albedo = bytearray(size * size * 3)
    normal = bytearray(size * size * 3)
    roughness = bytearray(size * size * 3)
    metallic = bytearray(size * size * 3)
    ao = bytearray(size * size * 3)
    opacity = bytearray(size * size * 3)
    preview = bytearray(size * size * 3)

    for y in range(size):
        visual_v = 1.0 - y / max(1, size - 1)
        if y < half:
            s = max(0.0, min(1.0, (visual_v - 0.5) * 2.0))
            src_x_target = neck_x + s * (tip_x - neck_x)
            source_column_index = clamp_int(round(src_x_target), 0, src_w - 1)
            top_y, bottom_y, center_y = column_lookup[source_column_index]
        else:
            stem_v = max(0.0, min(1.0, visual_v * 2.0))
        for x in range(size):
            u = u_values[x]
            if y < half:
                # Leaf region: file-top half corresponds to engine UV V 0.5..1.0.
                lateral = (u - 0.5) * 2.0
                src_y_target = center_y + lateral * ((bottom_y - top_y) * 0.5)
                color = sample_optional_map(leaf_albedo_source, (0, 0, 0), src_x_target, src_y_target)
                midrib = math.exp(-((u - 0.5) / 0.035) ** 2)
                vein = 0.5 + 0.5 * math.sin(u * math.tau * 58.0 + s * 8.0)
                normal_x = round(128 + (u - 0.5) * 22.0 + (vein - 0.5) * 10.0)
                normal_y = round(128 - midrib * 16.0)
                rough = round(160 + 18 * (1.0 - midrib) + 8 * vein)
                occ = round(246 - 18 * midrib)
                alpha = 255
                n = sample_optional_map(pbr_normal, (normal_x, normal_y, 255), src_x_target, src_y_target)
                r = grayscale(sample_optional_map(pbr_roughness, (rough, rough, rough), src_x_target, src_y_target))
                m = grayscale(sample_optional_map(pbr_metallic, (0, 0, 0), src_x_target, src_y_target))
                a = (255, 255, 255) if pbr_dir else (occ, occ, occ)
            else:
                # Stem/internode region: file-bottom half corresponds to engine UV V 0.0..0.5.
                color = stem_color(u, stem_v)
                fiber = 0.5 + 0.5 * math.sin(u * math.tau * 18.0)
                n = (round(128 + (fiber - 0.5) * 18.0), 128, 255)
                rough = 185
                r = (rough, rough, rough)
                m = (0, 0, 0)
                a = (235, 235, 235)
                alpha = 255

            put_rgb(albedo, size, x, y, color)
            put_rgb(normal, size, x, y, n)
            put_rgb(roughness, size, x, y, r)
            put_rgb(metallic, size, x, y, m)
            put_rgb(ao, size, x, y, a)
            put_rgb(opacity, size, x, y, (alpha, alpha, alpha))
            put_rgb(preview, size, x, y, color)

    # Draw UV split and midrib/circumference guide lines on preview only.
    for x in range(size):
        put_rgb(preview, size, x, half, (255, 220, 0))
    for y in range(0, half):
        put_rgb(preview, size, size // 2, y, (255, 40, 40))

    outputs = {
        "albedo": out_dir / "sorghum_leaf_stem_atlas_albedo.png",
        "normal": out_dir / "sorghum_leaf_stem_atlas_normal.png",
        "roughness": out_dir / "sorghum_leaf_stem_atlas_roughness.png",
        "metallic": out_dir / "sorghum_leaf_stem_atlas_metallic.png",
        "ao": out_dir / "sorghum_leaf_stem_atlas_ao.png",
        "opacity": out_dir / "sorghum_leaf_stem_atlas_opacity.png",
        "preview": out_dir / "sorghum_leaf_stem_atlas_preview.png",
    }
    write_png_rgb(outputs["albedo"], size, size, albedo)
    write_png_rgb(outputs["normal"], size, size, normal)
    write_png_rgb(outputs["roughness"], size, size, roughness)
    write_png_rgb(outputs["metallic"], size, size, metallic)
    write_png_rgb(outputs["ao"], size, size, ao)
    write_png_rgb(outputs["opacity"], size, size, opacity)
    write_png_rgb(outputs["preview"], size, size, preview)
    for key in ("albedo", "normal", "roughness", "metallic", "ao", "opacity", "preview"):
        ensure_evefilemeta(outputs[key], "Texture2D")

    readme = out_dir / "README.md"
    readme.write_text(
        "# GeminiLeaf01 Sorghum Atlas\n\n"
        "This folder contains square PBR texture atlases baked for the existing DigitalAgriculture sorghum UVs.\n\n"
        "- Top half of each atlas: leaf region, used by UV V 0.5..1.0.\n"
        "- Bottom half of each atlas: stem/internode region, used by UV V 0.0..0.5.\n"
        "- U 0.0..1.0 is across the leaf blade for leaves and around the stem for internodes.\n"
        "- The leaf albedo was warped from the Gemini horizontal leaf image or Chord basecolor using the Gemini mask.\n"
        "- If a Chord PBR directory is supplied, leaf normal, roughness, and metallic maps are warped from Chord outputs.\n"
        "- Chord does not output AO; AO is neutral white in the leaf region when Chord maps are used.\n"
        "- The bottom stem/internode atlas region remains procedural unless separate stem maps are supplied later.\n",
        encoding="utf-8",
    )
    ensure_evefilemeta(readme, "Binary")
    outputs["readme"] = readme
    return outputs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--albedo", type=Path, required=True)
    parser.add_argument("--mask", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--pbr-dir", type=Path, default=None, help="Optional Chord PBR map folder with albedo/normal/roughness/metallic PNGs.")
    parser.add_argument("--size", type=int, default=2048)
    parser.add_argument("--threshold", type=int, default=127)
    args = parser.parse_args()

    if args.size < 256 or args.size % 2 != 0:
        raise ValueError("--size must be an even integer >= 256")

    source_dir = args.out_dir / "source"
    atlas_dir = args.out_dir / "atlas"
    source_dir.mkdir(parents=True, exist_ok=True)
    atlas_dir.mkdir(parents=True, exist_ok=True)

    copy_if_different(args.albedo, source_dir / "gemini_sorghum_leaf_albedo.png")
    copy_if_different(args.mask, source_dir / "gemini_sorghum_leaf_mask.png")
    outputs = bake_atlas(args.albedo, args.mask, atlas_dir, args.size, args.threshold, args.pbr_dir)

    print(f"source: {source_dir}")
    for key, path in outputs.items():
        print(f"{key}: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
