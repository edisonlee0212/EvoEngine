"""Capture stationary, RT-disabled HDDAGI transport from a disposable Rendering demo."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module-dir", required=True, type=Path)
    parser.add_argument("--resources", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--beauty", action="store_true")
    args = parser.parse_args()
    module_dir, resources, output = (p.resolve() for p in (args.module_dir, args.resources, args.output))
    if args.frames < 32:
        parser.error("Use at least 32 stationary frames")
    if (resources / "EvoEngine-DemoProjects/Rendering/Rendering.eveproj").exists():
        parser.error("Use a fresh disposable Rendering/Assets copy")
    output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[1]
    revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
    dirty = subprocess.check_output(["git", "status", "--short"], cwd=repo, text=True)
    os.chdir(module_dir)
    sys.path.insert(0, str(module_dir))
    import PyEvoEngine as engine

    try:
        if not engine.RunDemoWindowless("Rendering", resources, False, False):
            raise RuntimeError("Could not initialize Rendering")
        capabilities = engine.SdfgiCapabilityReport()
        if any(capabilities[name] for name in ("ray_tracing_enabled", "ray_query_enabled", "blas_enabled", "tlas_enabled")):
            raise RuntimeError("All RT facilities must be disabled")
        engine.ConfigureCurrentSceneCameraForCapture("Rasterization", 1, 1)
        engine.ResizeCurrentSceneCameraForCapture(2560, 1440)
        engine.SetCurrentSceneGiProvider(engine.IndirectGiProvider.AutomaticHddagi)
        settings = engine.GetCurrentSceneGiSettings()
        engine.SetGpuTimingCaptureEnabled(True)
        for _ in range(300):
            if not engine.Loop():
                raise RuntimeError("Demo ended before transport initialization")
            state = engine.GetCurrentSceneGiStatus()
            if engine.IsCurrentSceneReadyForCapture() and state["hddagi_transport_ready"]:
                break
        else:
            raise RuntimeError(f"Transport did not become ready: {state}")
        for _ in range(args.frames):
            if not engine.Loop():
                raise RuntimeError("Demo ended during warmup")
        if args.beauty and not engine.CaptureCurrentScene(2560, 1440, output / "beauty.png", 1):
            raise RuntimeError("HDDAGI camera capture failed")
        state = engine.GetCurrentSceneGiStatus()
        if not state["hddagi_transport_ready"] or state["hddagi_transport_failure_flags"]:
            raise RuntimeError(f"Invalid transport: {state}")
        if args.beauty and state["effective_provider"] != "Automatic HDDAGI":
            raise RuntimeError(f"Camera capture used fallback: {state}")
        captures = {}
        if args.beauty:
            captures["beauty"] = {"sha256": hashlib.sha256((output / "beauty.png").read_bytes()).hexdigest()}
        for name in ("Light", "StaticLight", "Diffuse", "FilteredDiffuse", "Specular", "Occlusion0", "Occlusion1",
                     "History", "HistorySum", "ProcessFrame", "Proximity"):
            path = output / f"{name}.png"
            z_slice = (settings.probes.probe_count_x - 1) * 4 if name in ("Light", "StaticLight", "Occlusion0", "Occlusion1") else 0
            engine.CaptureCurrentSceneHddagiDebug(path, name, 0, z_slice)
            captures[name] = {"layer": 0, "z_slice": z_slice, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        after = engine.GetCurrentSceneGiStatus()
        if after["hddagi_update_count"] != state["hddagi_update_count"]:
            raise RuntimeError("Export changed scene updates")
        manifest = {"engine_revision": revision, "working_tree": dirty,
                    "godot_revision": "da1410fa3516d08cc31b6e86bd6673b9ce776316",
                    "module_directory": str(module_dir), "capabilities": capabilities,
                    "warmup_frames": args.frames, "camera_resolution": [2560, 1440],
                    "probe_settings": {name: getattr(settings.probes, name) for name in
                                       ("probe_count_x", "probe_count_y", "cascade_count", "base_probe_distance")},
                    "hddagi_settings": {name: getattr(settings.hddagi, name) for name in
                                        ("history_size", "light_update_frames", "filter_probes",
                                         "filter_ambient", "filter_reflections", "read_sky_light", "static_entities_only",
                                         "bounce_feedback", "energy", "normal_bias", "probe_bias", "reflection_bias", "occlusion_bias")},
                    "binary_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                                      [*module_dir.glob("PyEvoEngine*.pyd"), module_dir / "EvoEngine_SDK.dll"] if p.is_file()},
                    "history_size": settings.hddagi.history_size,
                    "light_update_frames": settings.hddagi.light_update_frames,
                    "bounce_feedback": settings.hddagi.bounce_feedback,
                    "status": after, "captures": captures,
                    "interpretation": "Physical circular storage, layer 0. Radiance uses Reinhard and gamma 2.2; occlusion is packed RGBA parity. Optional beauty is a separate camera capture."}
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        print(json.dumps(manifest), flush=True)
    finally:
        engine.Terminate()


if __name__ == "__main__":
    main()
