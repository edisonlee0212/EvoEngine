#!/usr/bin/env python3
"""Build the Helios bridge and export a reproducible batch of soybean OBJ files."""

from __future__ import annotations

import argparse
import json
import hashlib
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_SOURCE = REPO_ROOT / "Tools" / "HeliosSoybeanExporter"
DEFAULT_BUILD_DIR = REPO_ROOT / "out" / "helios-soybean-exporter" / "build"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "out" / "helios-soybeans"


def positive_int(text: str) -> int:
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return value


def non_negative_float(text: str) -> float:
    value = float(text)
    if value < 0:
        raise argparse.ArgumentTypeError("must be non-negative")
    return value


def positive_float(text: str) -> float:
    value = float(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return value


def parser() -> argparse.ArgumentParser:
    candidate = REPO_ROOT.parent / "Helios"
    default_helios = os.environ.get("HELIOS_ROOT") or (str(candidate) if candidate.exists() else None)
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--helios-root", type=Path, default=default_helios)
    result.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    result.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    result.add_argument("--exporter", type=Path, help="Use an already-built exporter")
    result.add_argument("--skip-build", action="store_true", help="Reuse the exporter in --build-dir")
    result.add_argument("--count", type=positive_int, default=10)
    result.add_argument("--seed-start", type=int, default=1)
    result.add_argument("--age-days", type=non_negative_float, default=45.0)
    result.add_argument("--max-nodes", type=positive_int)
    result.add_argument("--internode-length-m", type=positive_float)
    result.add_argument("--leaf-size-range-m", type=positive_float, nargs=2, metavar=("MIN", "MAX"))
    result.add_argument(
        "--leaf-aspect-ratio-range",
        type=positive_float,
        nargs=2,
        metavar=("MIN", "MAX"),
        help="Leaflet width/length ratio range",
    )
    result.add_argument("--petiole-length-range-m", type=positive_float, nargs=2, metavar=("MIN", "MAX"))
    result.add_argument("--leaf-subdivisions", type=positive_int)
    result.add_argument(
        "--keep-mtl-opacity-map",
        action="store_true",
        help="Keep Helios map_d entries even when the diffuse PNG already contains alpha",
    )
    return result


def subprocess_environment() -> dict[str, str]:
    environment = dict(os.environ)
    if os.name == "nt" and "PATH" in environment:
        environment["Path"] = environment.pop("PATH")
    return environment


def build_exporter(helios_root: Path, build_dir: Path) -> Path:
    if not (helios_root / "core" / "CMake_project.cmake").is_file():
        raise RuntimeError(f"Not a Helios source tree: {helios_root}")
    configure = [
        "cmake",
        "-S",
        str(TOOL_SOURCE),
        "-B",
        str(build_dir),
        f"-DHELIOS_ROOT={helios_root.resolve()}",
        "-DCMAKE_BUILD_TYPE=Release",
        "-DENABLE_OPENMP=ON",
    ]
    environment = subprocess_environment()
    subprocess.run(configure, check=True, env=environment)
    build = ["cmake", "--build", str(build_dir), "--config", "Release", "--parallel"]
    if os.name == "nt":
        build.append("1")
    subprocess.run(build, check=True, env=environment)
    return exporter_in(build_dir)


def exporter_in(build_dir: Path) -> Path:
    suffix = ".exe" if os.name == "nt" else ""
    return build_dir / f"helios_soybean_exporter{suffix}"


def helios_commit(exporter: Path) -> str:
    result = subprocess.run(
        [str(exporter), "--version"],
        cwd=exporter.parent,
        check=True,
        capture_output=True,
        text=True,
        env=subprocess_environment(),
    )
    return result.stdout.strip()


def validate_args(args: argparse.Namespace) -> None:
    if args.seed_start < 0 or args.seed_start + args.count - 1 > 0xFFFFFFFF:
        raise RuntimeError("The requested seed range must fit in an unsigned 32-bit integer")
    for name in ("leaf_size_range_m", "leaf_aspect_ratio_range", "petiole_length_range_m"):
        value = getattr(args, name)
        if value and value[0] > value[1]:
            raise RuntimeError(f"--{name.replace('_', '-')} MIN must not exceed MAX")


def parameter_overrides(args: argparse.Namespace) -> dict[str, object]:
    values = {
        "max_nodes": args.max_nodes,
        "internode_length_m": args.internode_length_m,
        "leaf_size_range_m": args.leaf_size_range_m,
        "leaf_aspect_ratio_range": args.leaf_aspect_ratio_range,
        "petiole_length_range_m": args.petiole_length_range_m,
        "leaf_subdivisions": args.leaf_subdivisions,
    }
    return {name: value for name, value in values.items() if value is not None}


def exporter_arguments(args: argparse.Namespace, exporter: Path, output: Path, seed: int) -> list[str]:
    command = [
        str(exporter),
        "--output",
        str(output),
        "--seed",
        str(seed),
        "--age-days",
        str(args.age_days),
    ]
    scalar_options = (
        ("--max-nodes", args.max_nodes),
        ("--internode-length-m", args.internode_length_m),
        ("--leaf-subdivisions", args.leaf_subdivisions),
    )
    for name, value in scalar_options:
        if value is not None:
            command.extend((name, str(value)))
    for minimum_name, maximum_name, value in (
        ("--leaf-size-min-m", "--leaf-size-max-m", args.leaf_size_range_m),
        ("--leaf-aspect-ratio-min", "--leaf-aspect-ratio-max", args.leaf_aspect_ratio_range),
        ("--petiole-length-min-m", "--petiole-length-max-m", args.petiole_length_range_m),
    ):
        if value:
            command.extend((minimum_name, str(value[0]), maximum_name, str(value[1])))
    return command


def remove_duplicate_opacity_map(material_path: Path) -> None:
    """Avoid replacing a diffuse PNG's alpha with its red channel during OBJ import."""
    lines = material_path.read_text(encoding="utf-8").splitlines()
    diffuse_maps = {line.split(maxsplit=1)[1] for line in lines if line.startswith("map_Kd ")}
    filtered = [
        line
        for line in lines
        if not (line.startswith("map_d ") and line.split(maxsplit=1)[1] in diffuse_maps)
    ]
    material_path.write_text("\n".join(filtered) + "\n", encoding="utf-8")


def main() -> int:
    started_at = datetime.now(timezone.utc).isoformat()
    started = time.perf_counter()
    args = parser().parse_args()
    try:
        validate_args(args)
        build_dir = args.build_dir.resolve()
        if args.exporter:
            exporter = args.exporter.resolve()
        elif args.skip_build:
            exporter = exporter_in(build_dir)
        else:
            if args.helios_root is None:
                raise RuntimeError("Pass --helios-root or set HELIOS_ROOT")
            exporter = build_exporter(Path(args.helios_root).resolve(), build_dir)
        if not exporter.is_file():
            raise RuntimeError(f"Exporter executable not found: {exporter}")

        output_dir = args.output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        width = max(4, len(str(args.count - 1)))
        models = []
        for index in range(args.count):
            model_started = time.perf_counter()
            seed = args.seed_start + index
            output = output_dir / f"soybean_{index:0{width}d}.obj"
            command = exporter_arguments(args, exporter, output, seed)
            subprocess.run(command, cwd=exporter.parent, check=True, env=subprocess_environment())
            material = output.with_suffix(".mtl")
            if not output.is_file() or not material.is_file():
                raise RuntimeError(f"Incomplete export for seed {seed}")
            if not args.keep_mtl_opacity_map:
                remove_duplicate_opacity_map(material)
            models.append(
                {
                    "index": index,
                    "seed": seed,
                    "obj": output.name,
                    "mtl": material.name,
                    "obj_bytes": output.stat().st_size,
                    "obj_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
                    "generation_seconds": time.perf_counter() - model_started,
                    "geometry": json.loads(output.with_suffix(".geometry.json").read_text(encoding="utf-8"))
                    if output.with_suffix(".geometry.json").is_file() else None,
                }
            )

        manifest = {
            "schema_version": 1,
            "generator": "Helios plantarchitecture/soybean",
            "helios_commit": subprocess.run(
                ["git", "-C", str(args.helios_root), "rev-parse", "HEAD"],
                check=True, capture_output=True, text=True, env=subprocess_environment(),
            ).stdout.strip() if args.helios_root else "unverified",
            "exporter_reported_version": helios_commit(exporter),
            "exporter_sha256": hashlib.sha256(exporter.read_bytes()).hexdigest(),
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "started_at_utc": started_at,
            "elapsed_seconds": time.perf_counter() - started,
            "age_days": args.age_days,
            "parameter_overrides": parameter_overrides(args),
            "duplicate_mtl_opacity_map_removed": not args.keep_mtl_opacity_map,
            "models": models,
        }
        manifest_path = output_dir / "manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        print(f"Exported {len(models)} soybean models to {output_dir}")
        print(f"Manifest: {manifest_path}")
        return 0
    except (OSError, RuntimeError, subprocess.CalledProcessError, ValueError) as error:
        print(f"export_helios_soybeans.py: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
