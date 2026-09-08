"""Record matched-coverage GI comparisons using an installed, disposable Rendering demo."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time


def summary(values):
    values = sorted(values)
    return {"count": len(values), "median": statistics.median(values),
            "p95": values[math.ceil(len(values) * 0.95) - 1], "max": values[-1]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module-dir", type=Path, required=True)
    parser.add_argument("--resources", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--provider", choices=("Hddagi", "Sdfgi", "Ddgi"), required=True)
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--full-resolution", action="store_true")
    parser.add_argument("--workloads", nargs="+", default=["stationary", "slow", "fast", "geometry", "material", "light"],
                        choices=("stationary", "slow", "fast", "geometry", "material", "light"))
    args = parser.parse_args()
    if args.frames < 300 or args.repeats < 3:
        parser.error("Closeout requires at least 300 frames and three repeats")
    module, resources, output = [p.resolve() for p in (args.module_dir, args.resources, args.output)]
    if (resources / "EvoEngine-DemoProjects/Rendering/Rendering.eveproj").exists():
        parser.error("Use a fresh disposable copy of Rendering/Assets")
    output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[1]
    revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
    dirty = subprocess.check_output(["git", "status", "--short"], cwd=repo, text=True)
    os.chdir(module)
    sys.path.insert(0, str(module))
    import PyEvoEngine as engine

    def loop():
        if not engine.Loop():
            raise RuntimeError("Demo ended unexpectedly")

    def capture(name):
        if not engine.CaptureCurrentScene(2560, 1440, output / name, 1):
            raise RuntimeError(f"Camera capture failed: {name}")

    def check(state):
        expected = {"Hddagi": "Automatic HDDAGI", "Sdfgi": "Automatic SDFGI", "Ddgi": "Automatic DDGI (RT)"}[args.provider]
        if state["effective_provider"] != expected:
            raise RuntimeError(f"Provider fallback: {state['effective_provider']}")
        if args.provider == "Hddagi" and (state["hddagi_transport_failure_flags"] or state["hddagi_failure_flags"]):
            raise RuntimeError("HDDAGI GPU failure")

    try:
        if not engine.RunDemoWindowless("Rendering", resources, False, args.provider == "Ddgi"):
            raise RuntimeError("Could not initialize Rendering")
        engine.ConfigureCurrentSceneCameraForCapture("Rasterization", 1, 1)
        engine.ResizeCurrentSceneCameraForCapture(2560, 1440)
        settings = engine.GetCurrentSceneGiSettings()
        settings.provider = getattr(engine.IndirectGiProvider, "Automatic" + args.provider)
        settings.hddagi.half_resolution = not args.full_resolution
        engine.SetCurrentSceneGiSettings(settings)
        engine.SetGpuTimingCaptureEnabled(True)
        for _ in range(600):
            loop()
            if engine.IsCurrentSceneReadyForCapture():
                break
        else:
            raise RuntimeError("Scene did not load")
        for _ in range(120):
            loop()
        state = engine.GetCurrentSceneGiStatus()
        check(state)
        anchor = engine.GetCurrentSceneCameraPositionForCapture()
        capabilities = engine.SdfgiCapabilityReport()
        if args.provider != "Ddgi" and any(capabilities[k] for k in
                                           ("ray_tracing_enabled", "ray_query_enabled", "blas_enabled", "tlas_enabled")):
            raise RuntimeError("No-RT comparison has RT enabled")
        capture("stationary.png")
        settings_record = {}
        for name, obj in (("probes", settings.probes), ("hddagi", settings.hddagi), ("sdfgi", settings.sdfgi),
                          ("ddgi", settings.ddgi.runtime)):
            settings_record[name] = {key: (value if isinstance(value, (bool, int, float, str)) else str(value))
                                     for key in dir(obj) if not key.startswith("_") and
                                     not callable(value := getattr(obj, key))}
        runs = []
        manifest = {"engine_revision": revision, "working_tree": dirty,
                    "godot_hddagi_revision": "da1410fa3516d08cc31b6e86bd6673b9ce776316",
                    "godot_sdfgi_revision": "34d06658a85845111a50db9e485ec4a0701d4298",
                    "module_directory": str(module), "capabilities": capabilities,
                    "binary_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                                      [*module.glob("PyEvoEngine*.pyd"), module / "EvoEngine_SDK.dll"]},
                    "settings": settings_record, "provider": args.provider, "viewport": [2560, 1440],
                    "warmup_frames": 120, "initial_camera_position": anchor,
                    "resource_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo / "Resources/EvoEngine-DemoProjects", text=True).strip(),
                    "resource_working_tree": subprocess.check_output(["git", "status", "--short"], cwd=repo / "Resources/EvoEngine-DemoProjects", text=True),
                    "initial_status": state,
                    "measurement": "Loop wall time excludes status polling; GPU spans/stages use frame IDs. VMA totals include scene and host-visible GPU allocations, exclude driver allocations. Record validation state separately from the build configuration and startup log.",
                    "runs": runs}
        for workload in args.workloads:
            for repeat in range(args.repeats):
                engine.SetCurrentSceneCameraPositionForCapture(*anchor)
                for _ in range(120):
                    loop()
                before = engine.GetCurrentSceneGiStatus()["application_frame"]
                samples, gpu, edit_count = [], {}, 0
                geometry_offset, material_scale, light_scale = 0.0, 1.0, 1.0
                for index in range(args.frames):
                    if workload in ("slow", "fast"):
                        phase = 2 * math.pi * (index + 1) / args.frames * (1 if workload == "slow" else 10)
                        engine.SetCurrentSceneCameraPositionForCapture(anchor[0] + math.sin(phase), anchor[1],
                                                                      anchor[2] + 4 * math.sin(phase))
                    elif workload in ("geometry", "material", "light") and index % 30 == 0:
                        low = (index // 30) % 2 == 0
                        if workload == "geometry":
                            target = 0.25 if low else 0.0
                            edit_count += engine.OffsetCurrentSceneStaticMeshesForCapture(target - geometry_offset, 0, 0)
                            geometry_offset = target
                        elif workload == "material":
                            target = 0.5 if low else 1.0
                            edit_count += engine.ScaleCurrentSceneStaticMaterialsForCapture(target / material_scale, 1)
                            material_scale = target
                        else:
                            target = 0.5 if low else 1.0
                            edit_count += engine.ScaleCurrentSceneDirectionalLightsForCapture(target / light_scale)
                            light_scale = target
                    start = time.perf_counter()
                    loop()
                    wall_ms = (time.perf_counter() - start) * 1000
                    state = engine.GetCurrentSceneGiStatus()
                    check(state)
                    samples.append({"frame": state["application_frame"], "loop_ms": wall_ms,
                                    "vma_bytes": state["vma_allocation_bytes"], "vma_block_bytes": state["vma_block_bytes"],
                                    "hddagi_bytes": state["hddagi_allocation_bytes"],
                                    "hddagi_retiring_bytes": state["hddagi_retiring_bytes"],
                                    "hddagi_updated_regions": state["hddagi_last_updated_regions"]})
                    gpu.update({f["application_frame"]: f for f in state["gpu_timings"]})
                last = state["application_frame"]
                for _ in range(4):
                    loop()
                gpu.update({f["application_frame"]: f for f in engine.GetCurrentSceneGiStatus()["gpu_timings"]})
                selected = [gpu[i] for i in range(before + 1, last + 1) if i in gpu]
                if len(selected) != args.frames:
                    raise RuntimeError(f"Missing GPU frame measurements: {len(selected)} / {args.frames}")
                stages = sorted({key for f in selected for key in f["stages_ms"]})
                if args.provider == "Hddagi" and not {"HddagiIntegrate", "HddagiDirectLight", "HddagiCameraSurface", "HddagiCameraGather"}.issubset(stages):
                    raise RuntimeError("HDDAGI stage instrumentation is incomplete")
                run = {"workload": workload, "repeat": repeat, "frames": args.frames, "edited_objects": edit_count,
                       "loop_ms": summary([s["loop_ms"] for s in samples]),
                       "gpu_span_ms": summary([f["span_ms"] for f in selected]),
                       "stages_ms": {key: summary([f["stages_ms"].get(key, 0) for f in selected]) for key in stages},
                       "vma_peak_bytes": max(s["vma_bytes"] for s in samples),
                       "vma_live_bytes": samples[-1]["vma_bytes"]}
                (output / f"{workload}-{repeat}.json").write_text(json.dumps({"summary": run, "frames": samples,
                                                                            "gpu": selected}, indent=2))
                runs.append(run)
                (output / "manifest.json").write_text(json.dumps(manifest, indent=2))
                print(json.dumps(run), flush=True)
                if geometry_offset:
                    engine.OffsetCurrentSceneStaticMeshesForCapture(-geometry_offset, 0, 0)
                if material_scale != 1:
                    engine.ScaleCurrentSceneStaticMaterialsForCapture(1 / material_scale, 1)
                if light_scale != 1:
                    engine.ScaleCurrentSceneDirectionalLightsForCapture(1 / light_scale)
        engine.SetCurrentSceneCameraPositionForCapture(*anchor)
        for _ in range(120):
            loop()
        capture("recovered.png")
        convergence = []
        for event in ("motion", "geometry", "material", "light"):
            if event == "motion":
                engine.SetCurrentSceneCameraPositionForCapture(anchor[0] + 1, anchor[1], anchor[2] + 2)
            elif event == "geometry":
                engine.OffsetCurrentSceneStaticMeshesForCapture(0.25, 0, 0)
            elif event == "material":
                engine.ScaleCurrentSceneStaticMaterialsForCapture(0.5, 1)
            else:
                engine.ScaleCurrentSceneDirectionalLightsForCapture(0.5)
            start = engine.GetCurrentSceneGiStatus()["application_frame"]
            for target in (1, 12, 120):
                while engine.GetCurrentSceneGiStatus()["application_frame"] - start < target - 1:
                    loop()
                name = f"convergence-{event}-{target}.png"
                capture(name)
                state = engine.GetCurrentSceneGiStatus()
                check(state)
                convergence.append({"event": event, "requested_frame": target,
                                    "actual_frame": state["application_frame"] - start,
                                    "capture": name, "sha256": hashlib.sha256((output / name).read_bytes()).hexdigest()})
            if event == "motion":
                engine.SetCurrentSceneCameraPositionForCapture(*anchor)
            elif event == "geometry":
                engine.OffsetCurrentSceneStaticMeshesForCapture(-0.25, 0, 0)
            elif event == "material":
                engine.ScaleCurrentSceneStaticMaterialsForCapture(2, 1)
            else:
                engine.ScaleCurrentSceneDirectionalLightsForCapture(2)
            for _ in range(120):
                loop()
        manifest["convergence"] = convergence
        manifest["final_status"] = engine.GetCurrentSceneGiStatus()
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2))
    finally:
        engine.Terminate()


if __name__ == "__main__":
    main()
