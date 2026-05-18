#!/usr/bin/env python3
"""Headless Scots pine growth frame generator for queue-driven jobs.

This script keeps a CLI shape compatible with the existing queue runner while
switching generation to Scots pine (L-system) instead of EcoSysLab tree growth.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Tuple

INT32_MAX = 2_147_483_647
IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


@dataclass
class ScotsPineConfig:
    output_root: Path
    output_name: str
    descriptor_path: str
    post_repot_descriptor_path: str
    project_path: Path
    use_gpu: bool
    seed: int
    frame_count: int
    delta_time: float
    repot_switch_gdd: float
    render_resolution: Tuple[int, int]
    export_mesh: bool


def parse_resolution(value: str) -> Tuple[int, int]:
    parts = value.lower().split("x")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(f"Resolution must be WxH, got: {value}")
    try:
        width = int(parts[0])
        height = int(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Resolution must be integers, got: {value}") from exc
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError(f"Resolution values must be positive, got: {value}")
    return width, height


def normalize_seed_for_binding(seed: int) -> int:
    return int(seed) % (INT32_MAX + 1)


def resolve_app_executable(evoengine_directory: Path) -> Path:
    preferred = evoengine_directory / "out" / "build" / "x64-Release" / "EvoEngine_App" / "ScotsPineDataGeneratorApp.exe"
    if preferred.exists():
        return preferred

    build_root = evoengine_directory / "out" / "build"
    if build_root.exists():
        matches = sorted(build_root.glob("**/ScotsPineDataGeneratorApp.exe"))
        if matches:
            return matches[0]

    raise FileNotFoundError(
        "ScotsPineDataGeneratorApp.exe not found. Build command:\n"
        "cmake --build 04_EvoEngine/out/build/x64-Release --config Release --target ScotsPineDataGeneratorApp"
    )


def count_generated_frames(output_root: Path, output_name: str) -> int:
    count = 0
    for path in output_root.glob(f"{output_name}_*"):
        if path.is_file() and path.suffix.lower() in IMAGE_EXTS:
            count += 1
    return count


def run_generation(config: ScotsPineConfig, app_executable: Path) -> Dict[str, Any]:
    output_root = config.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    # Map legacy tree-style (delta_time, max_iteration) controls onto Scots pine
    # target_gdd while ensuring we do not stay in near-invisible seedling stages.
    max_target_gdd = max(3000.0, config.delta_time * float(config.frame_count) * 4.0)

    command: List[str] = [
        str(app_executable),
        "--output-root",
        str(output_root),
        "--output-name",
        config.output_name,
        "--seed",
        str(config.seed),
        "--descriptor-path",
        config.descriptor_path,
        "--project-path",
        str(config.project_path.resolve()),
        "--frame-count",
        str(config.frame_count),
        "--max-target-gdd",
        str(max_target_gdd),
        "--repot-switch-gdd",
        str(config.repot_switch_gdd),
        "--render-resolution",
        f"{config.render_resolution[0]}x{config.render_resolution[1]}",
    ]

    if config.post_repot_descriptor_path:
        command.extend(
            [
                "--post-repot-descriptor-path",
                config.post_repot_descriptor_path,
            ]
        )

    command.append("--gpu" if config.use_gpu else "--cpu")
    if config.export_mesh:
        command.append("--export-mesh")

    completed = subprocess.run(
        command,
        cwd=str(app_executable.parent),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )

    if completed.returncode != 0:
        print(completed.stdout)
        raise RuntimeError(f"ScotsPineDataGeneratorApp failed with code {completed.returncode}")

    frame_count = count_generated_frames(output_root, config.output_name)
    if frame_count < 2:
        raise RuntimeError(
            f"Scots pine generation produced insufficient frames ({frame_count}) in {output_root}"
        )

    return {
        "output_root": str(output_root),
        "output_name": config.output_name,
        "descriptor_path": config.descriptor_path,
        "post_repot_descriptor_path": config.post_repot_descriptor_path,
        "project_path": str(config.project_path.resolve()),
        "seed": config.seed,
        "frame_count": config.frame_count,
        "delta_time": config.delta_time,
        "repot_switch_gdd": config.repot_switch_gdd,
        "max_target_gdd": max_target_gdd,
        "render_resolution": [config.render_resolution[0], config.render_resolution[1]],
        "use_gpu": config.use_gpu,
        "export_mesh": config.export_mesh,
        "generated_frame_count": frame_count,
        "app_executable": str(app_executable),
    }


def build_parser(default_project_path: Path) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate Scots pine growth frames with EvoEngine app backend.")
    parser.add_argument("--output-root", type=Path, required=True, help="Output directory root.")
    parser.add_argument("--output-name", type=str, default="ScotsPine_Sample", help="Output file prefix.")

    # Compatibility alias: queue runner still uses --tree-descriptor-path.
    parser.add_argument(
        "--tree-descriptor-path",
        type=str,
        default="./New ScotsPineDescriptor.spine",
        help="Scots pine descriptor path (relative to project assets or absolute).",
    )

    parser.add_argument(
        "--post-repot-descriptor-path",
        type=str,
        default="",
        help="Optional post-repot descriptor path used after switch GDD.",
    )

    parser.add_argument("--project-path", type=Path, default=default_project_path, help="Path to the .eveproj file.")
    parser.add_argument("--seed", type=int, default=0, help="Random seed.")

    # Compatibility mapping: queue runner uses --max-iteration as sequence length.
    parser.add_argument("--max-iteration", type=int, default=10, help="Number of growth frames.")
    parser.add_argument("--delta-time", type=float, default=30.0, help="GDD increment per frame.")
    parser.add_argument(
        "--repot-switch-gdd",
        type=float,
        default=6000.0,
        help="GDD threshold where post-repot descriptor becomes active.",
    )

    parser.add_argument("--render-resolution", type=parse_resolution, default=(1024, 1024), help="Render WxH.")

    # Accepted for CLI compatibility; currently unused by ScotsPineDataGeneratorApp.
    parser.add_argument("--output-resolution", type=parse_resolution, default=(1024, 1024))
    parser.add_argument("--camera-fov", type=float, default=90.0)
    parser.add_argument("--ambient-light", type=float, default=0.3)
    parser.add_argument("--directional-light", type=float, default=7.0)
    parser.add_argument("--export-rendering", dest="export_rendering", action="store_true", default=True)
    parser.add_argument("--no-export-rendering", dest="export_rendering", action="store_false")
    parser.add_argument("--export-ray-traced-rendering", action="store_true")

    parser.add_argument("--export-mesh", action="store_true", help="Export per-frame OBJ files.")
    parser.add_argument("--use-gpu", dest="use_gpu", action="store_true", default=False)
    parser.add_argument("--no-gpu", dest="use_gpu", action="store_false")

    parser.add_argument("--manifest-out", type=Path, default=None, help="Optional JSON manifest path.")
    return parser


def main() -> int:
    script_path = Path(__file__).resolve()
    script_folder = script_path.parent
    evoengine_directory = script_folder.parent

    default_project_path = evoengine_directory / "Resources" / "DigitalAgricultureProject" / "test.eveproj"
    parser = build_parser(default_project_path)
    args = parser.parse_args()

    if args.max_iteration <= 0:
        raise SystemExit("--max-iteration must be > 0")

    normalized_seed = normalize_seed_for_binding(args.seed)
    if normalized_seed != args.seed:
        print(f"[scotspine] normalized seed {args.seed} -> {normalized_seed}")

    config = ScotsPineConfig(
        output_root=args.output_root,
        output_name=args.output_name,
        descriptor_path=args.tree_descriptor_path,
        post_repot_descriptor_path=args.post_repot_descriptor_path,
        project_path=args.project_path,
        use_gpu=bool(args.use_gpu),
        seed=normalized_seed,
        frame_count=int(args.max_iteration),
        delta_time=float(args.delta_time),
        repot_switch_gdd=max(0.0, float(args.repot_switch_gdd)),
        render_resolution=args.render_resolution,
        export_mesh=bool(args.export_mesh),
    )

    app_executable = resolve_app_executable(evoengine_directory)

    print(f"[scotspine] app={app_executable}")
    print(f"[scotspine] output_root={config.output_root}")
    print(
        "[scotspine] "
        f"seed={config.seed} frames={config.frame_count} delta_time={config.delta_time} "
        f"resolution={config.render_resolution[0]}x{config.render_resolution[1]}"
    )

    manifest = run_generation(config, app_executable)
    if args.manifest_out:
        args.manifest_out.parent.mkdir(parents=True, exist_ok=True)
        args.manifest_out.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    print("[scotspine] Finished")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
