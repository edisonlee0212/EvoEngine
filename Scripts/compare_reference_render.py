#!/usr/bin/env python3
"""Compare two PNG render outputs and report exact-match plus error metrics."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
import tempfile
import zlib
from dataclasses import dataclass
from pathlib import Path

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


@dataclass(frozen=True)
class PngImage:
    path: Path
    width: int
    height: int
    rgba: bytes
    digest: str


def paeth_predictor(left: int, up: int, upper_left: int) -> int:
    estimate = left + up - upper_left
    distance_left = abs(estimate - left)
    distance_up = abs(estimate - up)
    distance_upper_left = abs(estimate - upper_left)
    if distance_left <= distance_up and distance_left <= distance_upper_left:
        return left
    if distance_up <= distance_upper_left:
        return up
    return upper_left


def unfilter_scanline(filter_type: int, raw: bytes, previous: bytes, bytes_per_pixel: int) -> bytes:
    output = bytearray(raw)
    for index, value in enumerate(raw):
        left = output[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
        up = previous[index] if previous else 0
        upper_left = previous[index - bytes_per_pixel] if previous and index >= bytes_per_pixel else 0
        if filter_type == 0:
            predictor = 0
        elif filter_type == 1:
            predictor = left
        elif filter_type == 2:
            predictor = up
        elif filter_type == 3:
            predictor = (left + up) // 2
        elif filter_type == 4:
            predictor = paeth_predictor(left, up, upper_left)
        else:
            raise ValueError(f"Unsupported PNG filter type: {filter_type}")
        output[index] = (value + predictor) & 0xFF
    return bytes(output)


def rgba_from_scanline(scanline: bytes, color_type: int) -> bytes:
    rgba = bytearray()
    if color_type == 0:
        for gray in scanline:
            rgba.extend((gray, gray, gray, 255))
    elif color_type == 2:
        for index in range(0, len(scanline), 3):
            rgba.extend((scanline[index], scanline[index + 1], scanline[index + 2], 255))
    elif color_type == 4:
        for index in range(0, len(scanline), 2):
            gray = scanline[index]
            rgba.extend((gray, gray, gray, scanline[index + 1]))
    elif color_type == 6:
        rgba.extend(scanline)
    else:
        raise ValueError(f"Unsupported PNG color type: {color_type}")
    return bytes(rgba)


def read_png(path: Path) -> PngImage:
    data = path.read_bytes()
    if not data.startswith(PNG_SIGNATURE):
        raise ValueError(f"Not a PNG file: {path}")

    offset = len(PNG_SIGNATURE)
    width = 0
    height = 0
    bit_depth = 0
    color_type = 0
    idat = bytearray()
    while offset < len(data):
        if offset + 8 > len(data):
            raise ValueError(f"Truncated PNG chunk header: {path}")
        chunk_length = struct.unpack(">I", data[offset : offset + 4])[0]
        chunk_type = data[offset + 4 : offset + 8]
        chunk_start = offset + 8
        chunk_end = chunk_start + chunk_length
        if chunk_end + 4 > len(data):
            raise ValueError(f"Truncated PNG chunk body: {path}")
        chunk = data[chunk_start:chunk_end]
        offset = chunk_end + 4

        if chunk_type == b"IHDR":
            width, height, bit_depth, color_type, compression, filter_method, interlace = struct.unpack(
                ">IIBBBBB", chunk
            )
            if compression != 0 or filter_method != 0 or interlace != 0:
                raise ValueError(f"Unsupported PNG compression/filter/interlace mode: {path}")
        elif chunk_type == b"IDAT":
            idat.extend(chunk)
        elif chunk_type == b"IEND":
            break

    if width <= 0 or height <= 0:
        raise ValueError(f"Missing or invalid PNG IHDR: {path}")
    if bit_depth != 8:
        raise ValueError(f"Only 8-bit PNGs are supported: {path} has bit depth {bit_depth}")

    channels_by_color_type = {0: 1, 2: 3, 4: 2, 6: 4}
    if color_type not in channels_by_color_type:
        raise ValueError(f"Unsupported PNG color type: {color_type} in {path}")

    channels = channels_by_color_type[color_type]
    bytes_per_pixel = channels
    stride = width * channels
    decompressed = zlib.decompress(bytes(idat))
    expected_size = height * (1 + stride)
    if len(decompressed) != expected_size:
        raise ValueError(
            f"Unexpected PNG data length for {path}: got {len(decompressed)}, expected {expected_size}"
        )

    previous = bytes(stride)
    rgba = bytearray()
    offset = 0
    for _ in range(height):
        filter_type = decompressed[offset]
        offset += 1
        scanline = unfilter_scanline(filter_type, decompressed[offset : offset + stride], previous, bytes_per_pixel)
        offset += stride
        rgba.extend(rgba_from_scanline(scanline, color_type))
        previous = scanline

    return PngImage(path=path, width=width, height=height, rgba=bytes(rgba), digest=hashlib.sha256(data).hexdigest())


def compare_images(reference: PngImage, candidate: PngImage, ignore_alpha: bool) -> dict[str, object]:
    summary: dict[str, object] = {
        "reference": str(reference.path),
        "candidate": str(candidate.path),
        "reference_sha256": reference.digest,
        "candidate_sha256": candidate.digest,
        "reference_size": [reference.width, reference.height],
        "candidate_size": [candidate.width, candidate.height],
        "ignore_alpha": ignore_alpha,
    }
    if (reference.width, reference.height) != (candidate.width, candidate.height):
        summary.update(
            {
                "exact_match": False,
                "same_dimensions": False,
                "reason": "image dimensions differ",
            }
        )
        return summary

    compared_channels = (0, 1, 2) if ignore_alpha else (0, 1, 2, 3)
    channel_count = len(compared_channels)
    pixel_count = reference.width * reference.height
    channel_diff_sum = [0] * channel_count
    channel_diff_square_sum = [0] * channel_count
    channel_max = [0] * channel_count
    byte_mismatch_count = 0
    pixel_mismatch_count = 0

    for pixel in range(pixel_count):
        pixel_mismatch = False
        base = pixel * 4
        for out_index, channel in enumerate(compared_channels):
            diff = abs(reference.rgba[base + channel] - candidate.rgba[base + channel])
            channel_diff_sum[out_index] += diff
            channel_diff_square_sum[out_index] += diff * diff
            channel_max[out_index] = max(channel_max[out_index], diff)
            if diff:
                byte_mismatch_count += 1
                pixel_mismatch = True
        if pixel_mismatch:
            pixel_mismatch_count += 1

    compared_values = pixel_count * channel_count
    total_diff_sum = sum(channel_diff_sum)
    total_diff_square_sum = sum(channel_diff_square_sum)
    summary.update(
        {
            "exact_match": byte_mismatch_count == 0,
            "same_dimensions": True,
            "pixels": pixel_count,
            "channels_compared": ["r", "g", "b"] if ignore_alpha else ["r", "g", "b", "a"],
            "byte_mismatch_count": byte_mismatch_count,
            "pixel_mismatch_count": pixel_mismatch_count,
            "max_abs_error": max(channel_max),
            "max_abs_error_per_channel": channel_max,
            "mean_abs_error": total_diff_sum / compared_values,
            "mean_abs_error_per_channel": [value / pixel_count for value in channel_diff_sum],
            "rms_error": math.sqrt(total_diff_square_sum / compared_values),
            "rms_error_per_channel": [math.sqrt(value / pixel_count) for value in channel_diff_square_sum],
            "normalized_mean_abs_error": total_diff_sum / (compared_values * 255.0),
            "normalized_rms_error": math.sqrt(total_diff_square_sum / compared_values) / 255.0,
        }
    )
    return summary


def write_png_rgba8(path: Path, width: int, height: int, rgba: bytes) -> None:
    if len(rgba) != width * height * 4:
        raise ValueError("RGBA payload size does not match width and height")

    def chunk(chunk_type: bytes, payload: bytes) -> bytes:
        checksum = zlib.crc32(chunk_type)
        checksum = zlib.crc32(payload, checksum)
        return struct.pack(">I", len(payload)) + chunk_type + payload + struct.pack(">I", checksum & 0xFFFFFFFF)

    scanlines = bytearray()
    for y in range(height):
        scanlines.append(0)
        start = y * width * 4
        scanlines.extend(rgba[start : start + width * 4])

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    path.write_bytes(PNG_SIGNATURE + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(bytes(scanlines))) + chunk(b"IEND", b""))


def run_self_test() -> int:
    rgba_a = bytes(
        (
            10,
            20,
            30,
            255,
            40,
            50,
            60,
            255,
            70,
            80,
            90,
            255,
            100,
            110,
            120,
            255,
        )
    )
    rgba_b = bytearray(rgba_a)
    rgba_b[4] += 3
    rgba_b[9] += 2
    with tempfile.TemporaryDirectory() as temp_dir:
        temp = Path(temp_dir)
        first = temp / "first.png"
        second = temp / "second.png"
        third = temp / "third.png"
        write_png_rgba8(first, 2, 2, rgba_a)
        write_png_rgba8(second, 2, 2, rgba_a)
        write_png_rgba8(third, 2, 2, bytes(rgba_b))
        exact = compare_images(read_png(first), read_png(second), ignore_alpha=False)
        changed = compare_images(read_png(first), read_png(third), ignore_alpha=False)
    if exact["exact_match"] is not True:
        print("Self-test failed: identical images did not match", file=sys.stderr)
        return 1
    if changed["exact_match"] is not False or changed["max_abs_error"] != 3:
        print("Self-test failed: changed image metrics were incorrect", file=sys.stderr)
        return 1
    print("Self-test passed")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", nargs="?", type=Path, help="Reference PNG output.")
    parser.add_argument("candidate", nargs="?", type=Path, help="Candidate PNG output.")
    parser.add_argument("--out", type=Path, help="Optional JSON summary output path.")
    parser.add_argument("--ignore-alpha", action="store_true", help="Compare RGB channels only.")
    parser.add_argument("--require-exact", action="store_true", help="Return a non-zero exit code unless images match.")
    parser.add_argument("--self-test", action="store_true", help="Run the built-in PNG reader/diff self-test.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test()
    if args.reference is None or args.candidate is None:
        print("reference and candidate PNG paths are required unless --self-test is used", file=sys.stderr)
        return 1

    summary = compare_images(read_png(args.reference.resolve()), read_png(args.candidate.resolve()), args.ignore_alpha)
    output = json.dumps(summary, indent=2, sort_keys=True)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(output + "\n", encoding="utf-8")
    print(output)
    if args.require_exact and not summary["exact_match"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
