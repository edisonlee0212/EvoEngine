"""Capture RT-disabled Sponza SDFGI diagnostics or beauty at 2560x1440.

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
    parser.add_argument("--view", choices=("voxels", "preprocess", "lighting", "transport", "beauty"), default="voxels")
    parser.add_argument("--probe", type=int, default=2456)
    parser.add_argument("--occlusion-off-output", type=Path, help="Optional second beauty capture after disabling occlusion")
    parser.add_argument("--traverse", action="store_true", help="After beauty, exercise signed scrolls and a teleport/return")
    parser.add_argument("--edit-check", action="store_true", help="Check material/light edits and provider lifecycle after beauty")
    args = parser.parse_args()
    if args.occlusion_off_output and args.view != "beauty":
        parser.error("--occlusion-off-output requires --view beauty")
    if args.traverse and (args.view != "beauty" or args.occlusion_off_output):
        parser.error("--traverse requires --view beauty without an occlusion-off comparison")
    if args.edit_check and (args.view != "beauty" or args.traverse or args.occlusion_off_output):
        parser.error("--edit-check requires --view beauty without other comparison modes")
    module_dir, resources, output = (path.resolve() for path in (args.module_dir, args.resources, args.output))
    occlusion_off_output = args.occlusion_off_output.resolve() if args.occlusion_off_output else None
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
        settings = engine.SdfgiSettings()
        engine.SetCurrentSceneSdfgiSettings(settings)
        engine.SetGpuTimingCaptureEnabled(args.view in ("transport", "beauty"))
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
        if args.view in ("transport", "beauty"):
            for warmup in range(120):
                if not engine.Loop():
                    raise RuntimeError("Rendering demo ended during probe warmup")
                state = engine.GetCurrentSceneGiStatus()
                if state["transport_recorded"] and state["transport_pass"] >= 90:
                    break
            else:
                raise RuntimeError(f"Probe transport did not reach three history cycles: {state}")
        if args.view == "beauty":
            if not state["published"] or not engine.CaptureCurrentScene(2560, 1440, output, 1):
                raise RuntimeError(f"SDFGI beauty capture failed: {state}")
            field_status = engine.ReadCurrentSceneSdfgiFieldStatus()
            if not field_status["ready"] or field_status["failure_flags"]:
                raise RuntimeError(f"SDFGI GPU publication failed: {field_status}")
            print(json.dumps({"status": engine.GetCurrentSceneGiStatus(), "capture_status": field_status, "view": args.view,
                              "use_occlusion": settings.use_occlusion,
                              "resolution": [2560, 1440], "output": str(output)}), flush=True)
            if occlusion_off_output:
                settings.use_occlusion = False
                engine.SetCurrentSceneSdfgiSettings(settings)
                for warmup in range(180):
                    if not engine.Loop():
                        raise RuntimeError("Rendering demo ended during occlusion-toggle reconvergence")
                    state = engine.GetCurrentSceneGiStatus()
                    if state["published"] and state["transport_pass"] >= 90:
                        break
                else:
                    raise RuntimeError(f"Occlusion-toggle field did not reconverge: {state}")
                if not engine.CaptureCurrentScene(2560, 1440, occlusion_off_output, 1):
                    raise RuntimeError("Occlusion-off beauty capture failed")
                field_status = engine.ReadCurrentSceneSdfgiFieldStatus()
                if not field_status["ready"] or field_status["failure_flags"]:
                    raise RuntimeError(f"Occlusion-off publication failed: {field_status}")
                print(json.dumps({"status": engine.GetCurrentSceneGiStatus(), "capture_status": field_status,
                                  "view": args.view, "use_occlusion": False, "resolution": [2560, 1440],
                                  "output": str(occlusion_off_output)}), flush=True)
            if args.edit_check:
                baseline = engine.GetCurrentSceneGiStatus()
                materials = engine.ScaleCurrentSceneStaticMaterialsForCapture(0.75, 2.0)
                if not materials or not engine.Loop():
                    raise RuntimeError("No static material edit was exercised")
                material_state = engine.GetCurrentSceneGiStatus()
                if (material_state["geometry_update_count"] != baseline["geometry_update_count"] or
                        material_state["payload_update_count"] <= baseline["payload_update_count"]):
                    raise RuntimeError(f"Material edit did not take the payload-only path: {material_state}")
                lights = engine.ScaleCurrentSceneDirectionalLightsForCapture(0.5)
                if not lights:
                    raise RuntimeError("No directional light edit was exercised")
                for _ in range(settings.history_size * 3):
                    if not engine.Loop():
                        raise RuntimeError("Demo ended during edit reconvergence")
                edited = engine.GetCurrentSceneGiStatus()
                if (edited["geometry_update_count"] != material_state["geometry_update_count"] or
                        edited["payload_update_count"] != material_state["payload_update_count"] or not edited["published"]):
                    raise RuntimeError(f"Unrelated work or light edit caused representation rebuilding: {edited}")
                path = output.with_stem(output.stem + "-edited")
                if not engine.CaptureCurrentScene(2560, 1440, path, 1):
                    raise RuntimeError("Edited scene capture failed")
                gpu = engine.ReadCurrentSceneSdfgiFieldStatus()
                if not gpu["ready"] or gpu["failure_flags"]:
                    raise RuntimeError(f"Edited publication failed: {gpu}")
                print(json.dumps({"view": "edited", "output": str(path), "capture_status": gpu,
                                  "materials_edited": materials, "lights_edited": lights,
                                  "before": baseline, "after_material": material_state, "status": edited}), flush=True)
                for history in (5, 10, settings.history_size):
                    engine.SetCurrentSceneGiProvider(engine.IndirectGiProvider.Environment)
                    engine.Loop()
                    if engine.GetCurrentSceneGiStatus()["sdfgi_state_active"]:
                        raise RuntimeError("Disabled SDFGI retained active state")
                    settings.history_size = history
                    engine.SetCurrentSceneSdfgiSettings(settings)
                    engine.SetCurrentSceneGiProvider(engine.IndirectGiProvider.AutomaticSdfgi)
                    for _ in range(4):
                        engine.Loop()
                    gpu = engine.ReadCurrentSceneSdfgiFieldStatus()
                    state = engine.GetCurrentSceneGiStatus()
                    if not state["published"] or not gpu["ready"] or gpu["failure_flags"]:
                        raise RuntimeError(f"Provider/layout recovery failed: {state}, {gpu}")
                    print(json.dumps({"lifecycle_history": history, "capture_status": gpu,
                                      "status": state}), flush=True)
            if args.traverse:
                origin = engine.GetCurrentSceneCameraPositionForCapture()
                movement = []

                def step(offset):
                    engine.SetCurrentSceneCameraPositionForCapture(*(a + b for a, b in zip(origin, offset)))
                    before = engine.GetCurrentSceneGiStatus()["transport_pass"]
                    if not engine.Loop():
                        raise RuntimeError("Rendering demo ended during movement")
                    state = engine.GetCurrentSceneGiStatus()
                    if not state["published"] or state["transport_pass"] != before + 1:
                        raise RuntimeError(f"Movement lost coherent publication: {state}")
                    movement.append({"offset": offset, "generation": state["transport_pass"],
                                     "maintenance_count": state["maintenance_count"],
                                     "cascades": state["cascades"], "pending_regions": state["pending_regions"]})

                for axis in range(3):
                    for distance in (1.2, 2.4, 0.0, -1.2, -2.4, 0.0):
                        offset = [0.0, 0.0, 0.0]
                        offset[axis] = distance
                        step(offset)
                for offset in ([8.0, -4.0, 8.0], [24.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                               [1000.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [0.0, 0.0, 0.0]):
                    step(offset)
                for label, frames in (("return-immediate", 0), ("return-one-cycle", settings.history_size),
                                      ("return-three-cycles", settings.history_size * 2)):
                    for _ in range(frames):
                        step([0.0, 0.0, 0.0])
                    path = output.with_stem(output.stem + "-" + label)
                    if not engine.CaptureCurrentScene(2560, 1440, path, 1):
                        raise RuntimeError(f"Could not capture {label}")
                    gpu = engine.ReadCurrentSceneSdfgiFieldStatus()
                    if not gpu["ready"] or gpu["failure_flags"]:
                        raise RuntimeError(f"Movement GPU publication failed: {gpu}")
                    print(json.dumps({"view": label, "output": str(path), "capture_status": gpu,
                                      "resolution": [2560, 1440], "status": engine.GetCurrentSceneGiStatus()}), flush=True)
                if not any(c["full_redraw"] for event in movement for c in event["cascades"]):
                    raise RuntimeError("Traversal did not exercise a full redraw")
                if not all(any(any(c["dirty_regions"]) for c in event["cascades"])
                           for event in (movement[1], movement[4])):
                    raise RuntimeError("Traversal did not exercise both scroll directions")
                print(json.dumps({"movement": movement, "use_occlusion": settings.use_occlusion}), flush=True)
            return
        preprocess = args.view == "preprocess"
        request = engine.RequestCurrentSceneSdfgiPreprocessDebug if preprocess else engine.RequestCurrentSceneSdfgiVoxelDebug
        capture = engine.CaptureCurrentSceneSdfgiPreprocessDebug if preprocess else engine.CaptureCurrentSceneSdfgiVoxelDebug
        if args.view == "lighting":
            request, capture = engine.RequestCurrentSceneSdfgiLightDebug, engine.CaptureCurrentSceneSdfgiLightDebug
        if args.view == "transport":
            request, capture = engine.RequestCurrentSceneSdfgiProbeDebug, engine.CaptureCurrentSceneSdfgiProbeDebug
        request(args.cascade, args.probe if args.view == "transport" else args.slice)
        if not engine.Loop():
            raise RuntimeError("Rendering demo ended before diagnostic capture")
        state = engine.GetCurrentSceneGiStatus()
        prefix = {"preprocess": "preprocess", "voxels": "voxel", "lighting": "light", "transport": "probe"}[args.view]
        failure = "transport_failure" if args.view == "transport" else f"{prefix}_failure"
        if state[failure] or not state[f"{prefix}_debug_recorded"]:
            raise RuntimeError(f"SDFGI diagnostic failed: {state}")
        output.parent.mkdir(parents=True, exist_ok=True)
        capture_status = capture(output)
        state = engine.GetCurrentSceneGiStatus()
        if preprocess and (not state["preprocess_status_available"] or state["preprocess_failure_flags"]):
            raise RuntimeError(f"SDFGI preprocessing failed: {state}")
        print(json.dumps({"status": state, "capture_status": capture_status, "probe": args.probe,
                          "view": args.view, "resolution": [2560, 1440], "cascade": args.cascade,
                          "slice": args.slice, "readiness_frames": frame + 1, "output": str(output)}), flush=True)
    finally:
        engine.Terminate()


if __name__ == "__main__":
    main()
