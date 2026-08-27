#!/usr/bin/env python3
"""Generate the deterministic CC0 HDR environment used by asset previews."""

from __future__ import annotations

import math
from pathlib import Path


WIDTH = 256
HEIGHT = 128


def angular_distance(azimuth: float, elevation: float, target_azimuth: float, target_elevation: float) -> float:
    delta = abs(azimuth - target_azimuth)
    delta = min(delta, 2.0 * math.pi - delta)
    return math.hypot(delta * math.cos((elevation + target_elevation) * 0.5), elevation - target_elevation)


def softbox(azimuth: float, elevation: float, target_azimuth: float, target_elevation: float,
            radius: float, intensity: float) -> float:
    distance = angular_distance(azimuth, elevation, target_azimuth, target_elevation)
    return intensity * math.exp(-0.5 * (distance / radius) ** 4)


def to_rgbe(red: float, green: float, blue: float) -> bytes:
    maximum = max(red, green, blue)
    if maximum < 1.0e-32:
        return b"\0\0\0\0"
    mantissa, exponent = math.frexp(maximum)
    scale = mantissa * 256.0 / maximum
    return bytes((min(255, int(red * scale)), min(255, int(green * scale)),
                  min(255, int(blue * scale)), exponent + 128))


def main() -> None:
    output = (Path(__file__).resolve().parents[1] /
              "EvoEngine_SDK/Internals/DefaultResources/Textures/MaterialPreview/neutral_studio.hdr")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as stream:
        stream.write(b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\nEXPOSURE=1.000000\n\n")
        stream.write(f"-Y {HEIGHT} +X {WIDTH}\n".encode("ascii"))
        for y in range(HEIGHT):
            elevation = math.pi * (0.5 - (y + 0.5) / HEIGHT)
            for x in range(WIDTH):
                azimuth = 2.0 * math.pi * ((x + 0.5) / WIDTH - 0.5)
                horizon = math.exp(-0.5 * (elevation / math.radians(24.0)) ** 2)
                floor = max(0.0, -math.sin(elevation))
                base = 0.12 + 0.07 * horizon - 0.035 * floor
                key = softbox(azimuth, elevation, math.radians(-42.0), math.radians(34.0), math.radians(16.0), 5.5)
                fill = softbox(azimuth, elevation, math.radians(68.0), math.radians(18.0), math.radians(24.0), 1.6)
                rim = softbox(azimuth, elevation, math.radians(158.0), math.radians(28.0), math.radians(13.0), 3.2)
                stream.write(to_rgbe(0.35 * base + 1.03 * key + 0.90 * fill + 0.82 * rim,
                                     0.70 * base + 1.00 * key + 0.95 * fill + 0.90 * rim,
                                     1.20 * base + 0.92 * key + 1.05 * fill + 1.05 * rim))


if __name__ == "__main__":
    main()
