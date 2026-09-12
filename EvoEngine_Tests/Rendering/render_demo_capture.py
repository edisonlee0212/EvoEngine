import argparse
import json
import os
import shutil
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--module-dir", required=True)
    parser.add_argument("--source-resources-root", required=True)
    parser.add_argument("--test-resources-root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--scene", choices=("sponza", "bistro"), default="sponza")
    parser.add_argument("--view", choices=("default", "doorway"), default="default")
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--warmup-frames", type=int, default=60)
    parser.add_argument(
        "--accumulation-frames",
        type=int,
        default=0,
        help="Require this many accumulated camera frames, including recovery from history resets.",
    )
    parser.add_argument(
        "--render-mode",
        choices=("Rasterization", "RayTracing", "RayQuery"),
        default="Rasterization",
    )
    parser.add_argument("--samples-per-frame", type=int, default=4)
    parser.add_argument("--bounces", type=int, default=4)
    parser.add_argument("--meshlet", choices=("enabled", "disabled"))
    parser.add_argument("--indirect", choices=("enabled", "disabled"))
    parser.add_argument("--ray-features", choices=("enabled", "disabled"), default="enabled")
    parser.add_argument("--gi-provider", choices=("ddgi", "sdfgi", "environment"))
    parser.add_argument("--gi-occlusion", choices=("enabled", "disabled"))
    parser.add_argument("--texture-lifecycle-stress", action="store_true")
    parser.add_argument("--secondary-camera-output")
    parser.add_argument("--expect-stable-texture-registrations", action="store_true")
    parser.add_argument("--sdfgi-debug-views", action="store_true")
    return parser.parse_args()


def copy_rendering_assets(source_resources_root: Path, test_resources_root: Path) -> None:
    source_assets = source_resources_root / "EvoEngine-DemoProjects" / "Rendering" / "Assets"
    target_assets = test_resources_root / "EvoEngine-DemoProjects" / "Rendering" / "Assets"
    fixture_scene = Path(__file__).resolve().parent / "Fixtures" / "Rendering" / "Assets" / "New Scene.evescene"
    if not source_assets.exists():
        raise FileNotFoundError(f"Rendering demo assets not found: {source_assets}")
    if not fixture_scene.is_file():
        raise FileNotFoundError(f"Rendering test scene not found: {fixture_scene}")
    if target_assets.exists():
        shutil.rmtree(target_assets)
    target_assets.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source_assets, target_assets)
    shutil.copy2(fixture_scene, target_assets / fixture_scene.name)


def main() -> int:
    args = parse_args()
    module_dir = Path(args.module_dir).resolve()
    source_resources_root = Path(args.source_resources_root).resolve()
    test_resources_root = Path(args.test_resources_root).resolve()
    output = Path(args.output).resolve()
    if args.warmup_frames < 0 or args.accumulation_frames < 0:
        raise ValueError("Capture frame counts must be non-negative")
    if args.samples_per_frame <= 0 or args.bounces < 0:
        raise ValueError("Capture samples per frame must be positive and bounces must be non-negative")
    if (args.meshlet is None) != (args.indirect is None):
        raise ValueError("--meshlet and --indirect must be provided together")
    gi_provider = args.gi_provider or ("ddgi" if args.ray_features == "enabled" else "environment")
    if gi_provider == "ddgi" and args.ray_features == "disabled":
        raise ValueError("DDGI capture requires ray features")

    if args.scene == "sponza" and args.view != "default":
        raise ValueError("The doorway view requires --scene bistro")
    if test_resources_root == source_resources_root or source_resources_root.is_relative_to(test_resources_root):
        raise ValueError("Test resources must be isolated from source resources")
    if args.scene == "bistro":
        relative_project = Path(".generated/EvoEngine-DemoProjects/Bistro")
        source_project = source_resources_root / relative_project
        target_project = (test_resources_root / relative_project).resolve()
        if not target_project.is_relative_to(test_resources_root):
            raise ValueError("Bistro test project must remain inside test resources")
        if not (source_project / "Assets/Models/Bistro/bistro.gltf").is_file():
            raise FileNotFoundError("Prepare Bistro assets with Scripts/prepare_demos.py before capturing")
        if target_project.exists():
            shutil.rmtree(target_project)
        shutil.copytree(source_project / "Assets/Models", target_project / "Assets/Models")
        (target_project / "Bistro.eveproj").write_text(
            "application_name: Bistro\npreferred_editor: EvoEngineEditor\nstartup_runtime_packages: []\n"
        )
    else:
        copy_rendering_assets(source_resources_root, test_resources_root)

    os.chdir(module_dir)
    sys.path.insert(0, str(module_dir))

    import PyEvoEngine as evoengine

    try:
        if not evoengine.RunDemoWindowless(
            "Bistro" if args.scene == "bistro" else "Rendering",
            test_resources_root, args.scene != "bistro", args.ray_features == "enabled"
        ):
            raise RuntimeError("RunDemoWindowless failed")
        if not evoengine.SharedTextureDescriptorArraysEnabled():
            raise RuntimeError("Shared sampled-texture descriptor arrays are not active")
        if args.ray_features == "disabled" and (
            evoengine.RayTracingEnabled()
            or evoengine.RayQueryEnabled()
            or evoengine.RayAccelerationStructureEnabled()
        ):
            raise RuntimeError("Raster-only capture unexpectedly enabled ray tracing, ray query, or acceleration structures")
        print(
            "EVOENGINE_TEXTURE_ARRAYS_ACTIVE "
            f"ray_tracing={evoengine.RayTracingEnabled()} ray_query={evoengine.RayQueryEnabled()}"
        )
        if args.texture_lifecycle_stress:
            if not evoengine.ExerciseTextureLifecycleForCapture():
                raise RuntimeError("Texture lifecycle stress failed")
            print("EVOENGINE_TEXTURE_LIFECYCLE_STRESS passed")
        for _ in range(30000):
            if not evoengine.Loop():
                raise RuntimeError("Rendering demo ended during scene loading")
            if evoengine.IsCurrentSceneReadyForCapture():
                break
        else:
            raise RuntimeError("Rendering demo did not finish scene loading")
        provider = {
            "ddgi": evoengine.IndirectGiProvider.AutomaticDdgi,
            "sdfgi": evoengine.IndirectGiProvider.AutomaticSdfgi,
            "environment": evoengine.IndirectGiProvider.Environment,
        }[gi_provider]
        evoengine.SetCurrentSceneGiProvider(provider)
        if args.gi_occlusion is not None:
            settings = evoengine.GetCurrentSceneGiSettings()
            settings.use_occlusion = args.gi_occlusion == "enabled"
            evoengine.SetCurrentSceneGiSettings(settings)
        if gi_provider == "sdfgi":
            debug = evoengine.GetCurrentSceneSdfgiDebug()
            debug.seed = 0
        for _ in range(2):
            if not evoengine.Loop():
                raise RuntimeError("Rendering demo ended during frame-slot warmup")
        if evoengine.IsCurrentSceneDdgiEnabled() != (gi_provider == "ddgi"):
            raise RuntimeError("Rendering demo did not activate the requested GI provider")
        if args.meshlet is not None:
            if not evoengine.ConfigureRasterPathForCapture(
                args.meshlet == "enabled", args.indirect == "enabled"
            ):
                raise RuntimeError(
                    f"Requested raster path is unavailable: meshlet={args.meshlet}, indirect={args.indirect}"
                )
            print(f"EVOENGINE_RASTER_PATH_CAPTURE meshlet={args.meshlet} indirect={args.indirect}")
        if not evoengine.ConfigureCurrentSceneCameraForCapture(
            args.render_mode, args.samples_per_frame, args.bounces
        ):
            raise RuntimeError(f"Requested render mode is unavailable: {args.render_mode}")
        if args.secondary_camera_output and not evoengine.ConfigureSecondarySceneCameraForCapture(
            args.width, args.height
        ):
            raise RuntimeError("Failed to configure secondary capture camera")
        if args.scene == "bistro":
            evoengine.ConfigureBistroCaptureView(args.view)
        capture_frames = args.accumulation_frames or args.warmup_frames
        if gi_provider == "sdfgi":
            output.parent.mkdir(parents=True, exist_ok=True)
            output.with_suffix(".before.yaml").write_text(evoengine.GetCurrentSceneSdfgiSnapshot())
        if not evoengine.CaptureCurrentScene(
            args.width,
            args.height,
            output,
            capture_frames,
            args.accumulation_frames > 0,
            args.expect_stable_texture_registrations,
        ):
            raise RuntimeError("CaptureCurrentScene failed")
        gi_status = evoengine.GetCurrentSceneGiStatus()
        gi_status["capture_scene"] = args.scene
        gi_status["capture_view"] = args.view
        gi_status["capture_camera_position"] = evoengine.GetCurrentSceneCameraPositionForCapture()
        output.with_suffix(".json").write_text(json.dumps(gi_status, indent=2))
        if gi_provider == "ddgi" and args.gi_occlusion == "enabled" and (
            not gi_status["ddgi_voxel_occlusion_active"] or gi_status["ddgi_relocation_enabled"]
        ):
            raise RuntimeError("DDGI voxel occlusion is inactive or relocation is still enabled")
        if gi_provider == "sdfgi":
            output.with_suffix(".after.yaml").write_text(evoengine.GetCurrentSceneSdfgiSnapshot())
        print(
            f"EVOENGINE_GI_CAPTURE requested={gi_status['requested_provider']} "
            f"effective={gi_status['effective_provider']} transport_pass={gi_status['transport_pass']}"
        )
        if gi_provider == "sdfgi" and (
            gi_status["effective_provider"] != "Automatic SDFGI" or not gi_status["transport_pass"]
        ):
            raise RuntimeError("SDFGI capture did not update its lighting field")
        if gi_provider == "sdfgi" and args.sdfgi_debug_views:
            debug.enabled = True
            debug.frozen = True
            evoengine.SelectMainCameraForSdfgiDebug()
            for view in ("Diffuse", "Specular", "Fallback"):
                debug.view = getattr(evoengine.SdfgiDebugView, view)
                for _ in range(4):
                    if not evoengine.Loop():
                        raise RuntimeError("Rendering demo ended during SDFGI diagnostic capture")
                evoengine.CaptureCurrentSceneSdfgiDebug(output.with_name(f"{output.stem}-{view}.png"))
            debug.view = evoengine.SdfgiDebugView.Beauty
            debug.frozen = False
            debug.enabled = False
        if args.secondary_camera_output and not evoengine.CaptureSecondarySceneCamera(
            Path(args.secondary_camera_output).resolve()
        ):
            raise RuntimeError("CaptureSecondarySceneCamera failed")
    finally:
        evoengine.Terminate()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
