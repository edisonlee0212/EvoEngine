"""Render wider field views from recorded cases, without rerunning illumination estimation."""
import argparse
import json
import os
import sys
from pathlib import Path
from search_prefab_field_illumination import DEFAULT_DELETE_ENTITIES, build_cases, configure_lighting, engine_vec3, resolve_lighting


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", type=Path, required=True)
    parser.add_argument("--run-ids", nargs="+", type=int)
    parser.add_argument("--sky-view", action="store_true", help="Also capture a lower view of the same field")
    args = parser.parse_args()
    study = args.study.resolve()
    root = Path(__file__).resolve().parents[1]
    config = json.loads((study / "configs/03_field_search.json").read_text(encoding="utf-8"))
    cases = build_cases(config)
    folder = study / "figures/field"
    folder.mkdir(parents=True, exist_ok=True)
    module = root / "out/install/vs2026-x64/python"
    os.chdir(module)
    sys.path.insert(0, str(module))
    import PyDigitalAgriculture as evo
    evo.PushRenderLayer()
    evo.PushRayTracerLayer()
    evo.RunWithScene(config["project"], config["base_scene"])
    metadata = []
    try:
        for name in DEFAULT_DELETE_ENTITIES:
            evo.DeleteCurrentSceneEntitiesByName(name)
        for name in config["required_entities"]:
            if not evo.GetCurrentSceneEntityCountByName(name):
                raise RuntimeError(f"Missing context entity: {name}")
        light = resolve_lighting(config, study / "configs")
        configure_lighting(evo, light)
        field = None
        for run_id in args.run_ids or config["capture_run_ids"]:
            case = cases[run_id]
            arguments = (case["asset_folder"], case["rows"], case["columns"], case["column_spacing"], case["row_spacing"],
                         case["seed"], *case["model_rotation"], case["model_scale"], case["random_yaw"], False, case["roi"])
            if field is None:
                field = evo.CreatePrefabField(*arguments)
            else:
                evo.ConfigurePrefabField(field, *arguments)
            if evo.GetPrefabFieldModelCount(field) != case["rows"] * case["columns"] or not evo.WaitForCurrentSceneReady(3000):
                raise RuntimeError("Field setup failed")
            # Include canopy overhang, not just the distances between the outermost stem centres.
            extent = max((case["columns"] - 1) * case["column_spacing"], (case["rows"] - 1) * case["row_spacing"]) + .9
            target = [0, .4, 0]
            position = [extent * .9, extent * .8 + .6, extent * 1.25]
            if not evo.SetMainCameraLookAt(engine_vec3(evo, position), engine_vec3(evo, target), engine_vec3(evo, [0, 1, 0]), 42):
                raise RuntimeError("Camera setup failed")
            image = folder / f"run_{run_id:04d}.png"
            if not evo.CaptureCurrentSceneCuda(1600, 1000, str(image), 256, config["ray_tracing"]["bounces"], light["gamma"], 0):
                raise RuntimeError("OptiX capture failed")
            metadata.append({"run_id": run_id, "case": case, "position": position, "target": target,
                             "image": image.name, "renderer": "EvoEngine OptiX", "image_samples": 256,
                             "resolved_lighting": light,
                             "illumination_estimation_rerun": False})
            if args.sky_view:
                sky_position = [extent * .9, 1.05, extent * 1.25]
                if not evo.SetMainCameraLookAt(engine_vec3(evo, sky_position), engine_vec3(evo, target),
                                               engine_vec3(evo, [0, 1, 0]), 42):
                    raise RuntimeError("Sky-view camera setup failed")
                sky_image = folder / f"run_{run_id:04d}_sky.png"
                if not evo.CaptureCurrentSceneCuda(1600, 1000, str(sky_image), 256,
                                                   config["ray_tracing"]["bounces"], light["gamma"], 0):
                    raise RuntimeError("Sky-view capture failed")
                metadata[-1].update(sky_view_image=sky_image.name, sky_view_position=sky_position)
            print(f"Captured field {run_id} with canopy margin", flush=True)
    finally:
        evo.Terminate()
    suffix = "_" + "_".join(map(str, args.run_ids)) if args.run_ids else ""
    (folder / f"capture_manifest{suffix}.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
