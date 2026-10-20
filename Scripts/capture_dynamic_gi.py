"""Capture fixed-camera Sponza GI with independently controlled animation sources.

Use the Python version matching the installed binding. Copy Rendering/Assets into
<resources>/EvoEngine-DemoProjects/Rendering/Assets in a fresh disposable directory;
do not use an authored project. The output directory must be new.

Example: python Scripts/capture_dynamic_gi.py --module-dir out/install/vs2026-x64/python
--resources <resources> --output <new-output> --provider Hddagi --motion combined
--occlusion --no-ao --frames 128

--no-ao disables screen-space AO independently of GI occlusion. To check settling,
use --freeze-at-frame 64 --frames 256. Outputs are numbered PNGs and records.json
with provider status, failure flags and history-reset diagnostics. CAPTURE_COMPLETE
marks completed image readbacks and JSON. Sparse-support generation/revision logging
is enabled through EVOENGINE_GI_SUPPORT_DIAGNOSTICS.
"""

import argparse
import json
import os
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module-dir", type=Path, required=True)
    parser.add_argument("--resources", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--provider", choices=("Ddgi", "Sdfgi", "Hddagi"), required=True)
    parser.add_argument("--motion", choices=("static", "light", "sphere", "skin", "combined"), required=True)
    parser.add_argument("--occlusion", action="store_true")
    parser.add_argument("--no-ao", action="store_true")
    parser.add_argument("--no-ssr", action="store_true")
    parser.add_argument("--beauty", action="store_true")
    parser.add_argument("--light-update-frames", type=int)
    parser.add_argument("--omit-sphere", action="store_true")
    parser.add_argument("--sdfgi-rays", type=int)
    parser.add_argument("--freeze-at-frame", type=int)
    parser.add_argument("--frames", type=int, default=64)
    parser.add_argument("--warmup", type=int, default=128)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    args = parser.parse_args()
    if args.freeze_at_frame is not None and args.freeze_at_frame < 0:
        parser.error("--freeze-at-frame must be nonnegative")
    module, resources, output = (p.resolve() for p in (args.module_dir, args.resources, args.output))
    if (resources / "EvoEngine-DemoProjects/Rendering/Rendering.eveproj").exists():
        parser.error("Use a fresh disposable Rendering/Assets copy")
    output.mkdir(parents=True, exist_ok=False)
    os.environ["EVOENGINE_GI_SUPPORT_DIAGNOSTICS"] = "1"
    os.chdir(module)
    sys.path.insert(0, str(module))
    import PyEvoEngine as engine

    def loop():
        if not engine.Loop():
            raise RuntimeError("Application ended during GI capture")

    try:
        if not engine.RunDemoWindowless("Rendering", resources, False, True):
            raise RuntimeError("Sponza initialization failed")
        assert engine.ConfigureCurrentSceneCameraForCapture("Rasterization", 1, 1)
        engine.ResizeCurrentSceneCameraForCapture(args.width, args.height)
        engine.ConfigureIndirectLightingDebugForCapture(0 if args.beauty else 1, False)
        if args.no_ao or args.no_ssr:
            engine.ConfigureCurrentScenePostProcessingForCapture(not args.no_ao, not args.no_ssr)
        for _ in range(600):
            loop()
            if engine.IsCurrentSceneReadyForCapture():
                break
        else:
            raise RuntimeError("Sponza assets did not become ready")
        combined = args.motion == "combined"
        sphere = engine.ConfigureSponzaMotionCapture(args.motion == "light" or combined,
                                                    args.motion == "sphere" or combined,
                                                    args.motion == "skin" or combined)
        if args.omit_sphere:
            engine.DeleteEntity(sphere)
        engine.SetSponzaMotionCaptureTime(0)
        settings = engine.GetCurrentSceneGiSettings()
        settings.provider = getattr(engine.IndirectGiProvider, "Automatic" + args.provider)
        settings.use_occlusion = args.occlusion
        if args.light_update_frames is not None:
            settings.sdfgi.light_update_frames = args.light_update_frames
            settings.hddagi.light_update_frames = args.light_update_frames
        if args.sdfgi_rays is not None:
            settings.sdfgi.ray_count = args.sdfgi_rays
        engine.SetCurrentSceneGiSettings(settings)
        for _ in range(args.warmup):
            loop()
        expected = {"Ddgi": "Automatic DDGI (RT)", "Sdfgi": "Automatic SDFGI",
                    "Hddagi": "Automatic HDDAGI"}[args.provider]
        records = []
        for frame in range(args.frames):
            # Sample the same physical timeline regardless of GPU/readback speed.
            seconds = min(frame, args.freeze_at_frame) / 30 if args.freeze_at_frame is not None else frame / 30
            engine.SetSponzaMotionCaptureTime(seconds)
            image = output / f"{frame:04}.png"
            if not engine.CaptureCurrentScene(args.width, args.height, image, 1):
                raise RuntimeError(f"Failed capture {frame}")
            state = engine.GetCurrentSceneGiStatus()
            if state["effective_provider"] != expected:
                raise RuntimeError(f"Provider fell back: {state}")
            records.append({"frame": frame, "seconds": seconds, "state": state,
                            "ddgi": engine.GetCurrentSceneDdgiHistoryStatus()})
        (output / "records.json").write_text(json.dumps({
            "provider": args.provider, "motion": args.motion, "occlusion": args.occlusion,
            "freeze_at_frame": args.freeze_at_frame,
            "width": args.width, "height": args.height, "warmup": args.warmup,
            "screen_space_ao": not args.no_ao,
            "screen_space_reflections": not args.no_ssr, "beauty": args.beauty,
            "light_update_frames": {"sdfgi": settings.sdfgi.light_update_frames,
                                   "hddagi": settings.hddagi.light_update_frames},
            "sphere_present": not args.omit_sphere,
            "sdfgi_rays": settings.sdfgi.ray_count,
            "capabilities": engine.SdfgiCapabilityReport(),
            "probe_settings": {key: getattr(settings.probes, key) for key in (
                "probe_count_x", "probe_count_y", "cascade_count", "base_probe_distance")},
            "history": {"ddgi": settings.ddgi.runtime.history_count, "sdfgi": settings.sdfgi.history_size,
                        "hddagi": settings.hddagi.history_size},
            "camera": engine.GetCurrentSceneCameraPositionForCapture(), "records": records,
        }, indent=2))
        print(f"CAPTURE_COMPLETE {output}", flush=True)
    finally:
        engine.Terminate()


if __name__ == "__main__":
    main()
