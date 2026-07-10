#!/usr/bin/env python3
"""Compare PNG or linear Radiance HDR render outputs and report error metrics."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
import tempfile
import zlib
from array import array
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


@dataclass(frozen=True)
class HdrImage:
    path: Path
    width: int
    height: int
    rgb: array
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


def decode_rgbe(rgbe: bytes, output: array) -> None:
    for index in range(0, len(rgbe), 4):
        red, green, blue, exponent = rgbe[index : index + 4]
        if exponent == 0:
            output.extend((0.0, 0.0, 0.0))
            continue
        scale = math.ldexp(1.0, exponent - (128 + 8))
        output.extend((red * scale, green * scale, blue * scale))


def decode_hdr_scanline(data: bytes, offset: int, width: int) -> tuple[bytes, int]:
    if offset + 4 > len(data):
        raise ValueError("Truncated Radiance HDR scanline")
    header = data[offset : offset + 4]
    if width < 8 or width > 0x7FFF or header[0:2] != b"\x02\x02" or header[2] & 0x80:
        byte_count = width * 4
        if offset + byte_count > len(data):
            raise ValueError("Truncated uncompressed Radiance HDR scanline")
        return data[offset : offset + byte_count], offset + byte_count
    encoded_width = (header[2] << 8) | header[3]
    if encoded_width != width:
        raise ValueError(f"Radiance HDR scanline width mismatch: {encoded_width} != {width}")
    offset += 4
    channels = [bytearray() for _ in range(4)]
    for channel in channels:
        while len(channel) < width:
            if offset >= len(data):
                raise ValueError("Truncated Radiance HDR RLE packet")
            count = data[offset]
            offset += 1
            if count > 128:
                run_length = count - 128
                if run_length == 0 or offset >= len(data) or len(channel) + run_length > width:
                    raise ValueError("Invalid Radiance HDR RLE run")
                channel.extend((data[offset],) * run_length)
                offset += 1
            else:
                if count == 0 or offset + count > len(data) or len(channel) + count > width:
                    raise ValueError("Invalid Radiance HDR RLE literal")
                channel.extend(data[offset : offset + count])
                offset += count
    scanline = bytearray(width * 4)
    for pixel in range(width):
        for channel_index in range(4):
            scanline[pixel * 4 + channel_index] = channels[channel_index][pixel]
    return bytes(scanline), offset


def read_hdr(path: Path) -> HdrImage:
    data = path.read_bytes()
    if not data.startswith((b"#?RADIANCE\n", b"#?RGBE\n")):
        raise ValueError(f"Not a Radiance HDR file: {path}")
    header_end = data.find(b"\n\n")
    if header_end < 0:
        raise ValueError(f"Missing Radiance HDR header terminator: {path}")
    header = data[:header_end].decode("ascii", errors="strict")
    if "FORMAT=32-bit_rle_rgbe" not in header:
        raise ValueError(f"Unsupported Radiance HDR encoding: {path}")
    resolution_start = header_end + 2
    resolution_end = data.find(b"\n", resolution_start)
    if resolution_end < 0:
        raise ValueError(f"Missing Radiance HDR resolution: {path}")
    resolution = data[resolution_start:resolution_end].decode("ascii", errors="strict").split()
    if len(resolution) != 4 or resolution[0] not in ("-Y", "+Y") or resolution[2] not in ("+X", "-X"):
        raise ValueError(f"Unsupported Radiance HDR orientation: {path}")
    height = int(resolution[1])
    width = int(resolution[3])
    if width <= 0 or height <= 0:
        raise ValueError(f"Invalid Radiance HDR dimensions: {path}")

    offset = resolution_end + 1
    scanlines: list[bytes] = []
    for _ in range(height):
        scanline, offset = decode_hdr_scanline(data, offset, width)
        scanlines.append(scanline)
    if resolution[0] == "+Y":
        scanlines.reverse()
    if resolution[2] == "-X":
        scanlines = [b"".join(scanline[index : index + 4] for index in range(len(scanline) - 4, -1, -4)) for scanline in scanlines]
    rgb = array("f")
    for scanline in scanlines:
        decode_rgbe(scanline, rgb)
    return HdrImage(path=path, width=width, height=height, rgb=rgb, digest=hashlib.sha256(data).hexdigest())


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


def compare_hdr_images(reference: HdrImage, candidate: HdrImage) -> dict[str, object]:
    summary: dict[str, object] = {
        "reference": str(reference.path),
        "candidate": str(candidate.path),
        "reference_sha256": reference.digest,
        "candidate_sha256": candidate.digest,
        "reference_size": [reference.width, reference.height],
        "candidate_size": [candidate.width, candidate.height],
        "encoding": "radiance_rgbe_linear",
    }
    if (reference.width, reference.height) != (candidate.width, candidate.height):
        summary.update({"exact_match": False, "same_dimensions": False, "reason": "image dimensions differ"})
        return summary

    value_count = len(reference.rgb)
    if value_count != len(candidate.rgb):
        raise ValueError("Radiance HDR payload lengths differ despite equal dimensions")
    channel_diff_sum = [0.0, 0.0, 0.0]
    channel_diff_square_sum = [0.0, 0.0, 0.0]
    channel_max = [0.0, 0.0, 0.0]
    relative_sum = 0.0
    relative_square_sum = 0.0
    mismatch_count = 0
    for index, (reference_value, candidate_value) in enumerate(zip(reference.rgb, candidate.rgb)):
        channel = index % 3
        difference = abs(reference_value - candidate_value)
        channel_diff_sum[channel] += difference
        channel_diff_square_sum[channel] += difference * difference
        channel_max[channel] = max(channel_max[channel], difference)
        denominator = abs(reference_value) + abs(candidate_value) + 1.0e-6
        relative = 2.0 * difference / denominator
        relative_sum += relative
        relative_square_sum += relative * relative
        mismatch_count += difference != 0.0
    pixels = reference.width * reference.height
    total_diff_sum = sum(channel_diff_sum)
    total_diff_square_sum = sum(channel_diff_square_sum)
    summary.update(
        {
            "exact_match": mismatch_count == 0,
            "same_dimensions": True,
            "pixels": pixels,
            "channels_compared": ["r", "g", "b"],
            "value_mismatch_count": mismatch_count,
            "max_abs_error": max(channel_max),
            "max_abs_error_per_channel": channel_max,
            "mean_abs_error": total_diff_sum / value_count,
            "mean_abs_error_per_channel": [value / pixels for value in channel_diff_sum],
            "rms_error": math.sqrt(total_diff_square_sum / value_count),
            "rms_error_per_channel": [math.sqrt(value / pixels) for value in channel_diff_square_sum],
            "mean_symmetric_relative_error": relative_sum / value_count,
            "rms_symmetric_relative_error": math.sqrt(relative_square_sum / value_count),
        }
    )
    return summary


def read_image(path: Path) -> PngImage | HdrImage:
    extension = path.suffix.lower()
    if extension == ".png":
        return read_png(path)
    if extension == ".hdr":
        return read_hdr(path)
    raise ValueError(f"Unsupported image extension: {path}")


def compare_render_images(reference: PngImage | HdrImage, candidate: PngImage | HdrImage, ignore_alpha: bool) -> dict[str, object]:
    if isinstance(reference, PngImage) and isinstance(candidate, PngImage):
        return compare_images(reference, candidate, ignore_alpha)
    if isinstance(reference, HdrImage) and isinstance(candidate, HdrImage):
        return compare_hdr_images(reference, candidate)
    raise ValueError("Reference and candidate must use the same image format")


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


def write_hdr_rgbe(path: Path, width: int, height: int, rgbe: bytes, rle: bool) -> None:
    if len(rgbe) != width * height * 4:
        raise ValueError("RGBE payload size does not match width and height")
    output = bytearray(b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n")
    output.extend(f"-Y {height} +X {width}\n".encode("ascii"))
    for row in range(height):
        scanline = rgbe[row * width * 4 : (row + 1) * width * 4]
        if not rle:
            output.extend(scanline)
            continue
        output.extend((2, 2, width >> 8, width & 0xFF))
        for channel_index in range(4):
            channel = bytes(scanline[channel_index::4])
            offset = 0
            while offset < width:
                chunk = channel[offset : offset + 128]
                if len(chunk) <= 127 and len(set(chunk)) == 1:
                    output.extend((128 + len(chunk), chunk[0]))
                else:
                    output.append(len(chunk))
                    output.extend(chunk)
                offset += len(chunk)
    path.write_bytes(output)


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
        uncompressed_hdr = temp / "uncompressed.hdr"
        rle_hdr = temp / "rle.hdr"
        changed_hdr = temp / "changed.hdr"
        uncompressed_rgbe = bytes((128, 64, 32, 129, 32, 64, 128, 130))
        rle_rgbe = bytes((128, 64, 32, 129)) * 8
        changed_rgbe = bytearray(rle_rgbe)
        changed_rgbe[0] = 96
        write_hdr_rgbe(uncompressed_hdr, 2, 1, uncompressed_rgbe, rle=False)
        write_hdr_rgbe(rle_hdr, 8, 1, rle_rgbe, rle=True)
        write_hdr_rgbe(changed_hdr, 8, 1, bytes(changed_rgbe), rle=True)
        uncompressed = read_hdr(uncompressed_hdr)
        hdr_exact = compare_hdr_images(read_hdr(rle_hdr), read_hdr(rle_hdr))
        hdr_changed = compare_hdr_images(read_hdr(rle_hdr), read_hdr(changed_hdr))
    if exact["exact_match"] is not True:
        print("Self-test failed: identical images did not match", file=sys.stderr)
        return 1
    if changed["exact_match"] is not False or changed["max_abs_error"] != 3:
        print("Self-test failed: changed image metrics were incorrect", file=sys.stderr)
        return 1
    if uncompressed.width != 2 or list(uncompressed.rgb) != [1.0, 0.5, 0.25, 0.5, 1.0, 2.0]:
        print("Self-test failed: uncompressed HDR decoding was incorrect", file=sys.stderr)
        return 1
    if hdr_exact["exact_match"] is not True or hdr_changed["exact_match"] is not False:
        print("Self-test failed: HDR comparison metrics were incorrect", file=sys.stderr)
        return 1
    print("Self-test passed")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", nargs="?", type=Path, help="Reference PNG or Radiance HDR output.")
    parser.add_argument("candidate", nargs="?", type=Path, help="Candidate PNG or Radiance HDR output.")
    parser.add_argument("--out", type=Path, help="Optional JSON summary output path.")
    parser.add_argument("--ignore-alpha", action="store_true", help="Compare RGB channels only.")
    parser.add_argument("--require-exact", action="store_true", help="Return a non-zero exit code unless images match.")
    parser.add_argument("--self-test", action="store_true", help="Run the built-in PNG/HDR reader and diff self-test.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test()
    if args.reference is None or args.candidate is None:
        print("reference and candidate image paths are required unless --self-test is used", file=sys.stderr)
        return 1

    summary = compare_render_images(read_image(args.reference.resolve()), read_image(args.candidate.resolve()), args.ignore_alpha)
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
