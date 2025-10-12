#!/usr/bin/env python3
"""
Make a video from a folder of images, ordered by filename.

Usage:
  python images_to_video.py --input /path/to/images --output out.mp4 --fps 30 --codec mp4v

Notes:
- Requires: pip install opencv-python
- Supported extensions: .jpg .jpeg .png .bmp .tif .tiff
- Frame size is taken from the first image; others will be resized to match.
"""

import argparse
import cv2
import os
from pathlib import Path
from typing import List

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}

def natural_key(s: str):
    """Sort key that handles numbers in filenames naturally (e.g., img2 < img10)."""
    import re
    return [int(text) if text.isdigit() else text.lower() for text in re.split(r"(\d+)", s)]

def find_images(folder: Path) -> List[Path]:
    files = [p for p in folder.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTS]
    files.sort(key=lambda p: natural_key(p.name))
    return files

def main():
    ap = argparse.ArgumentParser(description="Convert images in a folder to a video.")
    ap.add_argument("--input", "-i", required=True, type=Path, help="Input folder containing images")
    ap.add_argument("--output", "-o", required=True, type=Path, help="Output video file, e.g., out.mp4")
    ap.add_argument("--fps", type=float, default=30.0, help="Frames per second (default: 30)")
    ap.add_argument("--codec", type=str, default="mp4v",
                    help="FourCC codec (e.g., mp4v, avc1, XVID). Default: mp4v")
    ap.add_argument("--pattern", type=str, default="",
                    help="Optional substring filter; only images whose names contain this will be used")
    ap.add_argument("--loop-first", type=int, default=1,
                    help="Repeat first frame N times (hold) before playback (default: 1)")
    ap.add_argument("--loop-last", type=int, default=1,
                    help="Repeat last frame N times (hold) after playback (default: 1)")
    ap.add_argument("--resize", type=str, default="",
                    help='Force output size WxH (e.g., "1920x1080"). If omitted, size of first image is used.')
    args = ap.parse_args()

    if not args.input.exists() or not args.input.is_dir():
        raise SystemExit(f"Input folder not found or not a directory: {args.input}")

    images = find_images(args.input)
    if args.pattern:
        images = [p for p in images if args.pattern in p.name]

    if not images:
        raise SystemExit("No images found (check folder, extensions, or --pattern).")

    # Read first frame to determine size
    first = cv2.imread(str(images[0]))
    if first is None:
        raise SystemExit(f"Failed to read first image: {images[0]}")
    if args.resize:
        try:
            w_str, h_str = args.resize.lower().split("x")
            out_w, out_h = int(w_str), int(h_str)
        except Exception:
            raise SystemExit('Invalid --resize format. Use "WIDTHxHEIGHT", e.g., "1920x1080".')
        frame_size = (out_w, out_h)
        first_frame = cv2.resize(first, frame_size, interpolation=cv2.INTER_AREA)
    else:
        h, w = first.shape[:2]
        frame_size = (w, h)
        first_frame = first

    # Prepare writer
    fourcc = cv2.VideoWriter_fourcc(*args.codec)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(args.output), fourcc, args.fps, frame_size)
    if not writer.isOpened():
        raise SystemExit("Failed to open video writer. Try a different --codec or output extension.")

    def write_frame(img_path: Path):
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"Warning: could not read image: {img_path}")
            return
        if (img.shape[1], img.shape[0]) != frame_size:
            img = cv2.resize(img, frame_size, interpolation=cv2.INTER_AREA)
        writer.write(img)

    # Optional hold on first frame
    for _ in range(max(1, args.loop_first)):
        writer.write(first_frame)

    # Remaining frames
    for p in images[1:]:
        write_frame(p)

    # Optional hold on last frame
    last_img = cv2.imread(str(images[-1]))
    if last_img is not None:
        if (last_img.shape[1], last_img.shape[0]) != frame_size:
            last_img = cv2.resize(last_img, frame_size, interpolation=cv2.INTER_AREA)
        for _ in range(max(1, args.loop_last)):
            writer.write(last_img)

    writer.release()
    print(f"Done. Wrote {len(images)} frames to {args.output} at {args.fps} FPS, size={frame_size}, codec={args.codec}")

if __name__ == "__main__":
    main()


# pip install opencv-python
# python images_to_video.py --input /path/to/images --output out.mp4 --fps 30