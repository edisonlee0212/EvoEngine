#!/usr/bin/env python3
"""Capture and stitch rendering-demo probe comparison views."""

from __future__ import annotations

import argparse
import binascii
import os
import shlex
import struct
import subprocess
import sys
import zlib
from pathlib import Path

VIEWS = (
    ("left-gallery", (-2.35, 0.9, 0.25), (-2.35, -0.25, -2.55)),
    ("hallway", (0.0, 1.05, 2.4), (0.0, -0.1, -3.4)),
    ("right-gallery", (2.2, 0.9, 0.25), (2.2, -0.25, -2.55)),
)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_editor_path(root: Path) -> Path:
    executable = "EvoEngineEditor.exe" if os.name == "nt" else "EvoEngineEditor"
    return root / "out" / "install" / "vs2026-x64" / "bin" / executable


def format_command(command: list[str]) -> str:
    return subprocess.list2cmdline(command) if os.name == "nt" else shlex.join(command)


def add_vec3(command: list[str], flag: str, value: tuple[float, float, float]) -> None:
    command.append(flag)
    command.append(",".join(f"{component:.6g}" for component in value))


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--editor", type=Path, default=default_editor_path(root), help="Path to EvoEngineEditor.")
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "out" / "reflection-probe-comparison" / "rendering-demo-galleries.png",
        help="Combined output PNG.",
    )
    parser.add_argument(
        "--views-dir",
        type=Path,
        help="Directory for individual view captures. Defaults beside --output.",
    )
    parser.add_argument("--width", type=int, default=1280, help="Width of each captured view.")
    parser.add_argument("--height", type=int, default=720, help="Height of each captured view.")
    parser.add_argument("--warmup-frames", type=int, default=96, help="Frames to render before each capture.")
    parser.add_argument(
        "--render-mode",
        default="rasterization",
        choices=("rasterization", "raytracing", "rayquery"),
        help="Preview camera render mode.",
    )
    return parser.parse_args()


def capture_view(args: argparse.Namespace, editor: Path, view_name: str, position: tuple[float, float, float],
                 look_at: tuple[float, float, float], output_path: Path) -> None:
    command = [
        str(editor),
        "--demo",
        "rendering",
        "--editor",
        "--capture-demo-preview",
        str(output_path),
        "--preview-width",
        str(args.width),
        "--preview-height",
        str(args.height),
        "--preview-warmup-frames",
        str(args.warmup_frames),
        "--preview-render-mode",
        args.render_mode,
    ]
    add_vec3(command, "--preview-camera-position", position)
    add_vec3(command, "--preview-camera-look-at", look_at)
    print(format_command(command), flush=True)
    completed = subprocess.run(command, cwd=editor.parent)
    if completed.returncode != 0:
        raise RuntimeError(f"{view_name}: capture failed with exit code {completed.returncode}")
    if not output_path.is_file() or output_path.stat().st_size == 0:
        raise RuntimeError(f"{view_name}: capture was not written: {output_path}")


def paeth_predictor(left: int, up: int, up_left: int) -> int:
    prediction = left + up - up_left
    left_distance = abs(prediction - left)
    up_distance = abs(prediction - up)
    up_left_distance = abs(prediction - up_left)
    if left_distance <= up_distance and left_distance <= up_left_distance:
        return left
    if up_distance <= up_left_distance:
        return up
    return up_left


def png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    return (
        struct.pack(">I", len(data))
        + chunk_type
        + data
        + struct.pack(">I", binascii.crc32(chunk_type + data) & 0xFFFFFFFF)
    )


def read_png_rgb(path: Path) -> tuple[int, int, list[bytes]]:
    data = path.read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise RuntimeError(f"{path} is not a PNG file.")
    offset = 8
    width = height = bit_depth = color_type = None
    idat_chunks: list[bytes] = []
    while offset < len(data):
        length = struct.unpack(">I", data[offset : offset + 4])[0]
        offset += 4
        chunk_type = data[offset : offset + 4]
        offset += 4
        chunk_data = data[offset : offset + length]
        offset += length + 4
        if chunk_type == b"IHDR":
            width, height, bit_depth, color_type, _, _, _ = struct.unpack(">IIBBBBB", chunk_data)
        elif chunk_type == b"IDAT":
            idat_chunks.append(chunk_data)
        elif chunk_type == b"IEND":
            break
    if width is None or height is None or bit_depth != 8 or color_type not in (0, 2, 6):
        raise RuntimeError(f"{path} uses an unsupported PNG format.")

    source_channels = {0: 1, 2: 3, 6: 4}[color_type]
    row_stride = width * source_channels
    raw = zlib.decompress(b"".join(idat_chunks))
    rows: list[bytes] = []
    previous = bytearray(row_stride)
    offset = 0
    for _ in range(height):
        filter_type = raw[offset]
        offset += 1
        row = bytearray(raw[offset : offset + row_stride])
        offset += row_stride
        for index, value in enumerate(row):
            left = row[index - source_channels] if index >= source_channels else 0
            up = previous[index]
            up_left = previous[index - source_channels] if index >= source_channels else 0
            if filter_type == 1:
                value += left
            elif filter_type == 2:
                value += up
            elif filter_type == 3:
                value += (left + up) // 2
            elif filter_type == 4:
                value += paeth_predictor(left, up, up_left)
            elif filter_type != 0:
                raise RuntimeError(f"{path} uses unsupported PNG filter {filter_type}.")
            row[index] = value & 0xFF
        if source_channels == 1:
            rgb = bytearray()
            for gray in row:
                rgb.extend((gray, gray, gray))
            rows.append(bytes(rgb))
        elif source_channels == 3:
            rows.append(bytes(row))
        else:
            rgb = bytearray()
            for index in range(0, len(row), 4):
                rgb.extend(row[index : index + 3])
            rows.append(bytes(rgb))
        previous = row
    return width, height, rows


def write_png_rgb(path: Path, width: int, height: int, rows: list[bytes]) -> None:
    raw = bytearray()
    for row in rows:
        raw.append(0)
        raw.extend(row)
    payload = (
        b"\x89PNG\r\n\x1a\n"
        + png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + png_chunk(b"IDAT", zlib.compress(bytes(raw), 9))
        + png_chunk(b"IEND", b"")
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.unlink(missing_ok=True)
    path.write_bytes(payload)


def stitch_views(view_paths: list[tuple[str, Path]], output_path: Path) -> None:
    images = [read_png_rgb(path) for _, path in view_paths]
    output_width = sum(width for width, _, _ in images)
    output_height = max(height for _, height, _ in images)
    rows: list[bytes] = []
    for row_index in range(output_height):
        row = bytearray()
        for width, height, image_rows in images:
            row.extend(image_rows[row_index] if row_index < height else b"\x00" * width * 3)
        rows.append(bytes(row))
    write_png_rgb(output_path, output_width, output_height, rows)


def main() -> int:
    args = parse_args()
    if args.width <= 0 or args.height <= 0:
        print("--width and --height must be positive.", file=sys.stderr)
        return 1
    if args.warmup_frames < 0:
        print("--warmup-frames must be non-negative.", file=sys.stderr)
        return 1

    editor = args.editor.resolve()
    if not editor.exists():
        print(f"Missing EvoEngineEditor: {editor}", file=sys.stderr)
        return 1
    output_path = args.output.resolve()
    views_dir = (args.views_dir or output_path.parent / f"{output_path.stem}_views").resolve()
    views_dir.mkdir(parents=True, exist_ok=True)

    view_paths: list[tuple[str, Path]] = []
    try:
        for view_name, position, look_at in VIEWS:
            view_path = views_dir / f"{view_name}.png"
            view_path.unlink(missing_ok=True)
            capture_view(args, editor, view_name, position, look_at, view_path)
            view_paths.append((view_name, view_path))
        stitch_views(view_paths, output_path)
    except RuntimeError as error:
        print(error, file=sys.stderr)
        return 1
    print(f"Wrote {output_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
