#!/usr/bin/env python3
"""Create an alternating Scots pine synthetic/real review folder."""

from __future__ import annotations

import argparse
import json
import random
import shutil
import struct
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png"}
DEFAULT_REAL_MANIFEST = Path(
    r"D:\AlexD\02_skellies\03_ConiferProject\plant_color_profile_research"
    r"\runs\sam3_full_1080_color_profile_20260613_allpots\inputs_manifest.json"
)
DEFAULT_RENDER_RESOLUTION = "1372x1040"
RUN_ROOT_PREFIX = "[scots-pine-preview] run root:"


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_resolution(value: str) -> tuple[int, int]:
    normalized = value.lower().strip()
    if "x" not in normalized:
        raise argparse.ArgumentTypeError("Resolution must be WxH.")
    width_text, height_text = normalized.split("x", 1)
    width = int(width_text)
    height = int(height_text)
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError("Resolution must be positive.")
    return width, height


def png_size(path: Path) -> tuple[int, int] | None:
    with path.open("rb") as file:
        header = file.read(24)
    if len(header) < 24 or header[:8] != b"\x89PNG\r\n\x1a\n":
        return None
    return struct.unpack(">II", header[16:24])


def jpeg_size(path: Path) -> tuple[int, int] | None:
    with path.open("rb") as file:
        if file.read(2) != b"\xff\xd8":
            return None
        while True:
            marker_prefix = file.read(1)
            if not marker_prefix:
                return None
            if marker_prefix != b"\xff":
                continue
            marker = file.read(1)
            while marker == b"\xff":
                marker = file.read(1)
            if not marker or marker in {b"\xd8", b"\xd9"}:
                continue
            size_bytes = file.read(2)
            if len(size_bytes) != 2:
                return None
            segment_size = int.from_bytes(size_bytes, "big")
            if segment_size < 2:
                return None
            if marker[0] in {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}:
                data = file.read(5)
                if len(data) != 5:
                    return None
                return int.from_bytes(data[3:5], "big"), int.from_bytes(data[1:3], "big")
            file.seek(segment_size - 2, 1)


def image_size(path: Path) -> tuple[int, int]:
    size = png_size(path) if path.suffix.lower() == ".png" else jpeg_size(path)
    if size is None:
        raise ValueError(f"Unsupported or unreadable image file: {path}")
    return size


def load_real_paths(manifest_path: Path) -> list[Path]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    real_paths: list[Path] = []
    pots = manifest.get("pots", {})
    for pot_name in sorted(pots):
        for item in pots[pot_name]:
            path = Path(item.get("path", ""))
            if path.suffix.lower() in IMAGE_EXTENSIONS and path.exists():
                real_paths.append(path)
    if len(real_paths) < 4:
        raise SystemExit(f"Need at least 4 existing real images in {manifest_path}; found {len(real_paths)}.")
    return real_paths


def run_preview(command: list[str]) -> tuple[int, str, Path | None]:
    print("[scots-pine-iteration] cmd:", " ".join(command), flush=True)
    output_lines: list[str] = []
    preview_root: Path | None = None
    process = subprocess.Popen(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
    )
    assert process.stdout is not None
    for line in process.stdout:
        print(line, end="", flush=True)
        output_lines.append(line)
        if line.startswith(RUN_ROOT_PREFIX):
            preview_root = Path(line.partition(RUN_ROOT_PREFIX)[2].strip())
    return process.wait(), "".join(output_lines), preview_root


def synthetic_paths(preview_root: Path) -> list[Path]:
    paths = sorted(
        path
        for path in preview_root.rglob("*_scene_main_camera_composited.*")
        if path.suffix.lower() in IMAGE_EXTENSIONS
    )
    if len(paths) < 4:
        raise SystemExit(f"Need at least 4 synthetic composited images under {preview_root}; found {len(paths)}.")
    return paths[:4]


def review_name(slot: int, kind: str, source: Path) -> str:
    return f"{slot:02d}_{kind}_{source.stem}{source.suffix.lower()}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-root", type=Path, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--worker", type=Path, default=None)
    parser.add_argument("--real-manifest", type=Path, default=DEFAULT_REAL_MANIFEST)
    parser.add_argument("--base-seed", type=int, default=None)
    parser.add_argument("--render-resolution", default=DEFAULT_RENDER_RESOLUTION, type=parse_resolution)
    parser.add_argument("--plant-blur-radius-px", type=float, default=1.25)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    width, height = args.render_resolution
    seed = args.seed if args.seed is not None else int(time.time())
    base_seed = args.base_seed if args.base_seed is not None else seed
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    root = repo_root()
    run_root = args.out_root.resolve() if args.out_root else root / "tmp" / "scots-pine-iteration-review" / f"review_{timestamp}"
    synthetic_root = run_root / "synthetic"
    review_dir = run_root / "review"
    review_dir.mkdir(parents=True, exist_ok=False)

    command = [
        str(Path(sys.executable).resolve()),
        str((root / "Scripts" / "scots_pine_synth_preview.py").resolve()),
        "--scene-count",
        "4",
        "--image-only",
        "--render-resolution",
        f"{width}x{height}",
        "--render-mode",
        "rasterization",
        "--non-strict-parity",
        "--vary-scene-pine-seed",
        "--out-root",
        str(synthetic_root),
        "--output-name",
        "iteration",
        "--base-seed",
        str(base_seed),
        "--plant-blur-radius-px",
        str(args.plant_blur_radius_px),
    ]
    if args.worker:
        command.extend(["--worker", str(args.worker.resolve())])

    exit_code, preview_stdout, preview_root = run_preview(command)
    run_root.mkdir(parents=True, exist_ok=True)
    stdout_path = run_root / "preview_stdout.txt"
    stdout_path.write_text(preview_stdout, encoding="utf-8")
    if exit_code != 0:
        raise SystemExit(exit_code)
    if preview_root is None:
        raise SystemExit("Preview run root was not reported by scots_pine_synth_preview.py.")
    preview_root = preview_root.resolve()

    rng = random.Random(seed)
    synthetic = synthetic_paths(preview_root)
    real = rng.sample(load_real_paths(args.real_manifest.resolve()), 4)
    items: list[dict[str, object]] = []
    for index in range(4):
        for kind, source in (("synthetic", synthetic[index]), ("real", real[index])):
            slot = index * 2 + (1 if kind == "synthetic" else 2)
            actual_size = image_size(source)
            if actual_size != (width, height):
                raise SystemExit(f"{source} is {actual_size[0]}x{actual_size[1]}, expected {width}x{height}.")
            output = review_dir / review_name(slot, kind, source)
            shutil.copy2(source, output)
            items.append(
                {
                    "slot": slot,
                    "kind": kind,
                    "source": str(source.resolve()),
                    "output": str(output.resolve()),
                    "size": [width, height],
                }
            )

    manifest = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "seed": seed,
        "base_seed": base_seed,
        "render_resolution": [width, height],
        "preview_command": command,
        "preview_root": str(preview_root),
        "preview_stdout": str(stdout_path.resolve()),
        "real_manifest": str(args.real_manifest.resolve()),
        "review_dir": str(review_dir.resolve()),
        "items": items,
    }
    (run_root / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"[scots-pine-iteration] review: {review_dir.resolve()}")
    print(f"[scots-pine-iteration] manifest: {(run_root / 'manifest.json').resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
