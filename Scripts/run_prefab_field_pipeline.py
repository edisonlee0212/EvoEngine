#!/usr/bin/env python3
"""Create, save, render, and optionally measure a model field through PyDigitalAgriculture."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module-dir", type=Path, required=True)
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--base-scene", type=Path, required=True, help="Scene path relative to Assets")
    parser.add_argument("--asset-folder", type=Path, required=True, help="Model folder relative to Assets")
    parser.add_argument("--rows", type=int, default=4)
    parser.add_argument("--columns", type=int, default=10)
    parser.add_argument("--roi", type=float, default=1.0, help="Central measured fraction per grid dimension, in (0, 1]")
    parser.add_argument("--column-spacing", type=float, default=0.3)
    parser.add_argument("--row-spacing", type=float, default=0.75)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--model-rotation", type=float, nargs=3, default=(-90.0, 0.0, 0.0))
    parser.add_argument("--model-scale", type=float, default=1.0)
    parser.add_argument("--random-yaw", type=float, default=10.0)
    parser.add_argument("--camera-distance-scale", type=float, default=1.35)
    parser.add_argument("--recursive", action="store_true")
    parser.add_argument("--output-scene", type=Path, help="Optional output scene path relative to Assets")
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument("--cuda-screenshot", type=Path)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--illumination-json", type=Path)
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--bounces", type=int, default=4)
    parser.add_argument(
        "--delete-entity",
        action="append",
        default=["Pawaga_Field", "BTX_Field", "PARBAR_Pawaga", "PARBAR_BTX", "New Entity"],
        help="Exact entity name to remove from the base scene; may be repeated",
    )
    return parser.parse_args()


def vec3(module, values):
    result = module.Vec3()
    result.x, result.y, result.z = values
    return result


def report(message: str) -> None:
    print(f"[prefab-field] {message}", flush=True)


def validate_args(args: argparse.Namespace) -> None:
    if args.rows <= 0 or args.columns <= 0:
        raise ValueError("rows and columns must be positive")
    if not math.isfinite(args.roi) or not 0 < args.roi <= 1:
        raise ValueError("roi must be finite and in (0, 1]")
    if args.column_spacing < 0 or args.row_spacing < 0 or args.model_scale <= 0:
        raise ValueError("spacing must be non-negative and model scale must be positive")
    if args.camera_distance_scale <= 0:
        raise ValueError("camera distance scale must be positive")
    if args.output_scene and args.output_scene.suffix != ".evescene":
        raise ValueError("--output-scene must end in .evescene")


def main() -> int:
    args = parse_args()
    validate_args(args)
    project = args.project.resolve()
    screenshot = args.screenshot.resolve() if args.screenshot else None
    cuda_screenshot = args.cuda_screenshot.resolve() if args.cuda_screenshot else None
    illumination_json = args.illumination_json.resolve() if args.illumination_json else None
    module_dir = args.module_dir.resolve()
    os.chdir(module_dir)
    sys.path.insert(0, str(module_dir))
    import PyDigitalAgriculture as evo

    # Layers must be registered before project initialization so their component types are available.
    evo.PushRenderLayer()
    if illumination_json or cuda_screenshot:
        evo.PushRayTracerLayer()
    evo.RunWithScene(str(project), str(args.base_scene))
    try:
        # Remove crop and sensor roots before the first rendered frame so their GPU resources are never prepared.
        report("removing source crop and sensor entities")
        for name in args.delete_entity:
            evo.DeleteCurrentSceneEntitiesByName(name)

        report("generating model instances")
        field = evo.CreatePrefabField(
            str(args.asset_folder),
            args.rows,
            args.columns,
            args.column_spacing,
            args.row_spacing,
            args.seed,
            *args.model_rotation,
            args.model_scale,
            args.random_yaw,
            args.recursive,
            args.roi,
        )
        expected_count = args.rows * args.columns
        if evo.GetPrefabFieldModelCount(field) != expected_count:
            raise RuntimeError("field generation did not produce the requested number of models")
        report("waiting for imported model resources")
        if not evo.WaitForCurrentSceneReady(3000):
            raise RuntimeError("imported field assets did not finish loading")

        # Frame the generated field with the project's calibrated outdoor lighting profile.
        if screenshot or cuda_screenshot:
            field_width = max((args.columns - 1) * args.column_spacing, 1.0)
            field_depth = max((args.rows - 1) * args.row_spacing, 1.0)
            extent = max(field_width, field_depth)
            target = (0.0, 0.45, 0.0)
            base_position = (extent * 0.75 + 1.0, extent * 0.65 + 1.2, extent + 1.5)
            camera_position = tuple(
                target[index] + (base_position[index] - target[index]) * args.camera_distance_scale
                for index in range(3)
            )
            evo.ConfigureCurrentSceneOutdoorLightingForCapture(
                vec3(evo, (55.0, 30.0, 0.0)),
                0.012,
                0.92 * 6.0,
                vec3(evo, (1.0, 0.975, 0.94)),
                0.92,
                0.14 * 6.0,
                vec3(evo, (0.794, 0.794, 0.7)),
                2.2,
            )
            evo.SetMainCameraLookAt(
                vec3(evo, camera_position),
                vec3(evo, target),
                vec3(evo, (0.0, 1.0, 0.0)),
                42.0,
            )
            if not evo.ConfigureCurrentSceneCameraForCapture("Rasterization", 1, 2):
                raise RuntimeError("Rasterization capture is unavailable")
            if cuda_screenshot and not evo.ConfigureCurrentSceneCudaOutdoorLightingForCapture(
                vec3(evo, (55.0, 30.0, 0.0)),
                0.012,
                0.92,
                vec3(evo, (1.0, 0.975, 0.94)),
                0.92,
                0.14,
                2.2,
            ):
                raise RuntimeError("CUDA outdoor lighting setup is unavailable")

        # Saving clones the active scene, so the loaded base scene remains unchanged.
        if args.output_scene and not evo.SaveCurrentSceneAs(str(args.output_scene)):
            raise RuntimeError(f"failed to save scene: {args.output_scene}")
        if args.output_scene:
            report(f"saved scene to {args.output_scene}")

        if screenshot:
            report("capturing field preview")
            if not evo.CaptureCurrentScene(args.width, args.height, str(screenshot), 30, False):
                raise RuntimeError("field screenshot failed")

        if cuda_screenshot:
            report("capturing CUDA field preview")
            if not evo.CaptureCurrentSceneCuda(
                args.width, args.height, str(cuda_screenshot), args.samples, args.bounces, 2.2, 0.0
            ):
                raise RuntimeError("CUDA field screenshot failed")

        if illumination_json:
            report("estimating per-model illumination")
            values = evo.EstimatePrefabFieldIllumination(
                field, args.samples, args.bounces, args.seed, 0.0001
            )
            region = evo.GetPrefabFieldMeasurementRegion(field)
            if len(values) != region.rows * region.columns:
                raise RuntimeError("illumination did not return the requested ROI plant count")
            payload = [
                {
                    "model_index": value.model_index,
                    "asset_index": value.asset_index,
                    "row": value.row,
                    "column": value.column,
                    "total_area": value.total_area,
                    "total_flux": [value.total_flux.x, value.total_flux.y, value.total_flux.z],
                    "average_flux": [value.average_flux.x, value.average_flux.y, value.average_flux.z],
                    "organs": {
                        name: {
                            "total_area": organ.total_area,
                            "total_flux": [organ.total_flux.x, organ.total_flux.y, organ.total_flux.z],
                            "average_flux": [organ.average_flux.x, organ.average_flux.y, organ.average_flux.z],
                        }
                        for name, organ in value.organs.items()
                    },
                }
                for value in values
            ]
            illumination_json.parent.mkdir(parents=True, exist_ok=True)
            illumination_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
            report(f"saved illumination results to {illumination_json}")
    finally:
        evo.Terminate()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
