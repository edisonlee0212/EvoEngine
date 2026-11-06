"""Convert one GoPro RAW sky and render a saved soybean field with that incoming light.

Run with Python 3.12, rawpy/numpy/Pillow on PYTHONPATH, and the installed PyDigitalAgriculture.
This is a visual preview, not a calibrated radiometric experiment.
"""
import argparse
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import rawpy


def prepare_sky(args):
    # Reuse the existing projection/RGBE writer; only RAW decompression needs the GoPro SDK.
    spec = importlib.util.spec_from_file_location("sky_converter", args.converter)
    converter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(converter)
    dng = args.output / (args.input.stem + ".DNG")
    subprocess.run([str(args.decoder), str(args.input), str(dng)], check=True)
    with rawpy.imread(str(dng)) as raw:
        rgb = raw.postprocess(output_bps=16, no_auto_bright=True, use_camera_wb=True, gamma=(1, 1))
        raw_settings = {"white_level": raw.white_level, "black_levels": raw.black_level_per_channel,
                        "camera_white_balance": raw.camera_whitebalance}
    rgb = rgb.astype(np.float32) / 65535.0
    latlong, valid = converter.convert(rgb, 4096, 2048, *args.circle, 0, 0, 180, "equidistant", tuple(args.ground_color))
    hdr = args.output / (args.input.stem + "_latlong.hdr")
    converter.save_radiance_hdr(hdr, latlong)
    converter.save_preview(args.output / "sky_preview.jpg", latlong)
    return hdr, {"input": str(args.input), "input_sha256": hashlib.file_digest(args.input.open("rb"), "sha256").hexdigest(),
                 "decoder": str(args.decoder), "converter": str(args.converter), "raw": raw_settings,
                 "output_bps": 16, "auto_bright": False, "gamma": [1, 1], "dimensions": [4096, 2048],
                 "circle": args.circle, "projection": "equidistant", "fov_degrees": 180,
                 "projection_calibrated": False, "lower_hemisphere_rgb": args.ground_color,
                 "lower_hemisphere_measured": False,
                 "brightest_pixel_estimate": converter.find_sun(latlong, valid),
                 "absolute_radiometric_calibration": False}


def capture(args, hdr):
    # Scene lighting and visible sky share the unfiltered HDR; do not add a second sun.
    root = Path(__file__).resolve().parents[1]
    module = root / "out/install/vs2026-x64/python"
    os.chdir(module)
    sys.path.insert(0, str(module))
    import PyDigitalAgriculture as evo
    from search_prefab_field_illumination import DEFAULT_DELETE_ENTITIES, engine_vec3
    evo.PushRenderLayer()
    evo.PushRayTracerLayer()
    evo.RunWithScene(str(root / "Resources/DigitalAgricultureProject/DigitalAgricultureProject.eveproj"),
                     "GeneratedAssets/Scenes/Soybean_Narrow_4x10_Field.evescene")
    try:
        for name in DEFAULT_DELETE_ENTITIES:
            evo.DeleteCurrentSceneEntitiesByName(name)
        for name in ["bigSoil *", "Ground Mesh"]:
            if not evo.GetCurrentSceneEntityCountByName(name):
                raise RuntimeError(f"Missing soil context: {name}")
        field = evo.CreatePrefabField(
            "GeneratedAssets/Soybean/Experiments/20261016_robust_v1/Training/Narrow",
            4, 10, .127, .762, 101, -90., 0., 0., 1., 0., False, 1.)
        if evo.GetPrefabFieldModelCount(field) != 40 or not evo.WaitForCurrentSceneReady(3000):
            raise RuntimeError("Field setup failed")
        if not evo.ConfigureCurrentSceneCudaEnvironmentMapForCapture(str(hdr), args.intensity, args.rotation, 2.2):
            raise RuntimeError("OptiX did not import the HDR environment")
        position, target = [2.25, 1.05, 3.], [0., .45, 0.]
        if not evo.SetMainCameraLookAt(engine_vec3(evo, position), engine_vec3(evo, target),
                                       engine_vec3(evo, [0., 1., 0.]), 48.):
            raise RuntimeError("Camera setup failed")
        image = args.output / "soybean_G1706994_optix.png"
        if not evo.CaptureCurrentSceneCuda(1600, 1000, str(image), args.samples, 2, 2.2, 0.):
            raise RuntimeError("OptiX capture failed")
        scene = "GeneratedAssets/Scenes/Soybean_G1706994_HDR_Preview.evescene"
        if not evo.SaveCurrentSceneAs(scene):
            raise RuntimeError("Scene save failed")
        return {"renderer": "EvoEngine OptiX", "python_executable": sys.executable,
                "python_module": str(next(module.glob("PyDigitalAgriculture*.pyd"))),
                "hdr": str(hdr), "intensity_multiplier": args.intensity, "rotation_degrees": args.rotation,
                "separate_sun": False, "ambient_fallback": 0, "rows": 4, "columns": 10,
                "row_spacing_m": .762, "seeding_spacing_m": .127, "leaf_type": "narrow", "seed": 101,
                "camera_position": position, "camera_target": target, "fov": 48., "image": str(image),
                "samples_per_pixel": args.samples, "bounces": 2, "display_gamma": 2.2, "denoiser": 0,
                "saved_scene": scene, "illumination_experiment_rerun": False}
    finally:
        evo.Terminate()


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("D:/UIUCGoProCamera/103GOPRO/103GOPRO/G1706994.GPR"))
    parser.add_argument("--converter", type=Path, default=Path("D:/UIUCGoProCamera/convert_fisheye_sky_to_latlong.py"))
    parser.add_argument("--decoder", type=Path, default=root / "out/gopro-sky/tools/bin/GprToDng.exe")
    parser.add_argument("--output", type=Path, default=root / "out/gopro-sky/G1706994")
    parser.add_argument("--circle", nargs=3, type=float, default=[2783.5, 2435.5, 2435.5], metavar=("CX", "CY", "RADIUS"))
    parser.add_argument("--ground-color", nargs=3, type=float, default=[.02, .025, .02])
    parser.add_argument("--intensity", type=float, default=2.)
    parser.add_argument("--rotation", type=float, default=0.)
    parser.add_argument("--samples", type=int, default=512)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--capture-only", action="store_true")
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = args.output / "settings.json"
    if args.capture_only:
        settings = json.loads(manifest.read_text())
        hdr = args.output / (args.input.stem + "_latlong.hdr")
    else:
        hdr, settings = prepare_sky(args)
    if not args.prepare_only:
        settings["capture"] = capture(args, hdr)
    manifest.write_text(json.dumps(settings, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(settings, indent=2), flush=True)


if __name__ == "__main__":
    main()
