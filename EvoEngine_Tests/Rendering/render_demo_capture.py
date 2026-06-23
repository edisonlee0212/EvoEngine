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
    return parser.parse_args()


def copy_rendering_assets(source_resources_root: Path, test_resources_root: Path) -> None:
    source_assets = source_resources_root / "EvoEngine-DemoProjects" / "Rendering" / "Assets"
    target_assets = test_resources_root / "EvoEngine-DemoProjects" / "Rendering" / "Assets"
    if not source_assets.exists():
        raise FileNotFoundError(f"Rendering demo assets not found: {source_assets}")
    if target_assets.exists():
        shutil.rmtree(target_assets)
    target_assets.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source_assets, target_assets)


def main() -> int:
    args = parse_args()
    module_dir = Path(args.module_dir).resolve()
    source_resources_root = Path(args.source_resources_root).resolve()
    test_resources_root = Path(args.test_resources_root).resolve()
    output = Path(args.output).resolve()

    copy_rendering_assets(source_resources_root, test_resources_root)

    os.chdir(module_dir)
    sys.path.insert(0, str(module_dir))

    import PyEvoEngine as evoengine

    try:
        if not evoengine.RunDemoWindowless("Rendering", test_resources_root, True):
            raise RuntimeError("RunDemoWindowless failed")
        if not evoengine.IsCurrentSceneDdgiEnabled():
            raise RuntimeError("Rendering demo capture requires DDGI to be enabled")
        if not evoengine.CaptureCurrentScene(args.width, args.height, output, args.warmup_frames):
            raise RuntimeError("CaptureCurrentScene failed")
    finally:
        evoengine.Terminate()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
