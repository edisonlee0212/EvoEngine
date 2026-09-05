"""Capture RT-disabled Sponza SDFGI voxel or SDF/occlusion slices at 2560x1440.

Use a disposable copy of Rendering demo resources, not the authored project.
The script does not compare against or update any DDGI baseline.
"""

import argparse
import json
import os
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module-dir", required=True, type=Path)
    parser.add_argument("--resources", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--cascade", type=int, default=0)
    parser.add_argument("--slice", type=int, default=64)
    parser.add_argument("--view", choices=("voxels", "preprocess"), default="voxels")
    args = parser.parse_args()
    module_dir, resources, output = (path.resolve() for path in (args.module_dir, args.resources, args.output))
    if not (resources / "EvoEngine-DemoProjects/Rendering/Assets/Models/Sponza_FBX/Sponza.fbx").is_file():
        raise ValueError("The disposable resource folder must contain the Rendering demo's Sponza assets")
    if (resources / "EvoEngine-DemoProjects/Rendering/Rendering.eveproj").exists():
        raise ValueError("Use a fresh disposable Rendering/Assets copy without a generated Rendering.eveproj")
    os.chdir(module_dir)
    sys.path.insert(0, str(module_dir))
    import PyEvoEngine as engine

    try:
        if not engine.RunDemoWindowless("Rendering", resources, False, False):
            raise RuntimeError("Could not initialize the Rendering demo")
        report = engine.SdfgiCapabilityReport()
        print(json.dumps({"capabilities": report}), flush=True)
        if not report["supported"] or any(report[name] for name in
                ("ray_tracing_enabled", "ray_query_enabled", "blas_enabled", "tlas_enabled")):
            raise RuntimeError("SDFGI capture requires supported hardware with all RT facilities disabled")
        if not engine.ConfigureCurrentSceneCameraForCapture("Rasterization", 1, 1):
            raise RuntimeError("Could not configure the main raster camera")
        engine.ResizeCurrentSceneCameraForCapture(2560, 1440)
        engine.SetCurrentSceneGiProvider(engine.IndirectGiProvider.AutomaticSdfgi)
        for frame in range(300):
            if not engine.Loop():
                raise RuntimeError("Rendering demo ended before voxelization")
            state = engine.GetCurrentSceneGiStatus()
            if engine.IsCurrentSceneReadyForCapture() and state["voxelization_recorded"]:
                break
        else:
            raise RuntimeError(f"Voxelization did not become ready: {state}")
        if state["static_contributor_count"] == 0:
            raise RuntimeError(f"The Rendering demo has no static contributors: {state}")
        preprocess = args.view == "preprocess"
        request = engine.RequestCurrentSceneSdfgiPreprocessDebug if preprocess else engine.RequestCurrentSceneSdfgiVoxelDebug
        capture = engine.CaptureCurrentSceneSdfgiPreprocessDebug if preprocess else engine.CaptureCurrentSceneSdfgiVoxelDebug
        request(args.cascade, args.slice)
        if not engine.Loop():
            raise RuntimeError("Rendering demo ended before diagnostic capture")
        state = engine.GetCurrentSceneGiStatus()
        prefix = "preprocess" if preprocess else "voxel"
        if state[f"{prefix}_failure"] or not state[f"{prefix}_debug_recorded"]:
            raise RuntimeError(f"SDFGI diagnostic failed: {state}")
        output.parent.mkdir(parents=True, exist_ok=True)
        capture(output)
        state = engine.GetCurrentSceneGiStatus()
        if preprocess and (not state["preprocess_status_available"] or state["preprocess_failure_flags"]):
            raise RuntimeError(f"SDFGI preprocessing failed: {state}")
        print(json.dumps({"status": state, "view": args.view, "resolution": [2560, 1440], "cascade": args.cascade,
                          "slice": args.slice, "readiness_frames": frame + 1, "output": str(output)}), flush=True)
    finally:
        engine.Terminate()


if __name__ == "__main__":
    main()
