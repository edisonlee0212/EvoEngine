import argparse
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

    copy_rendering_assets(source_resources_root, test_resources_root)

    os.chdir(module_dir)
    sys.path.insert(0, str(module_dir))

    import PyEvoEngine as evoengine

    try:
        if not evoengine.RunDemoWindowless("Rendering", test_resources_root, True):
            raise RuntimeError("RunDemoWindowless failed")
        if not evoengine.IsCurrentSceneDdgiEnabled():
            raise RuntimeError("Rendering demo capture requires DDGI to be enabled")
        for _ in range(2):
            if not evoengine.Loop():
                raise RuntimeError("Rendering demo ended during frame-slot warmup")
        if not evoengine.ConfigureCurrentSceneCameraForCapture(
            args.render_mode, args.samples_per_frame, args.bounces
        ):
            raise RuntimeError(f"Requested render mode is unavailable: {args.render_mode}")
        capture_frames = args.accumulation_frames or args.warmup_frames
        if not evoengine.CaptureCurrentScene(
            args.width,
            args.height,
            output,
            capture_frames,
            args.accumulation_frames > 0,
        ):
            raise RuntimeError("CaptureCurrentScene failed")
    finally:
        evoengine.Terminate()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
