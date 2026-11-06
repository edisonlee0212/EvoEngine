#!/usr/bin/env python3
"""Grid-search PrefabField parameters and save OptiX illumination measurements as CSV."""

from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import math
import os
import secrets
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


DEFAULT_DELETE_ENTITIES = [
    "Pawaga_Field",
    "BTX_Field",
    "PARBAR_Pawaga",
    "PARBAR_BTX",
    "New Entity",
    "Prefab Field",
]
INCH_TO_METERS = 0.0254


def select_run_ids(config, count):
    """Keep original run IDs when executing an explicitly selected continuation or reference check."""
    selected = config.get("run_ids", list(range(count)))
    if not isinstance(selected, list) or not selected or any(type(value) is not int or not 0 <= value < count for value in selected):
        raise ValueError("run_ids must be a nonempty list of integer indices within the full search")
    if len(set(selected)) != len(selected):
        raise ValueError("run_ids must not contain duplicates")
    return sorted(selected)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module-dir", type=Path, required=True, help="Folder containing PyDigitalAgriculture.pyd")
    parser.add_argument("--config", type=Path, required=True, help="JSON search configuration")
    parser.add_argument("--output-dir", type=Path, help="Override the config output_directory")
    parser.add_argument("--validate-only", action="store_true", help="Validate and count runs without starting EvoEngine")
    parser.add_argument(
        "--prepare-model-library-only",
        action="store_true",
        help="Generate the configured Helios model libraries and exit without running the field search",
    )
    return parser.parse_args()


def as_list(value, name: str) -> list:
    values = value if isinstance(value, list) else [value]
    if not values:
        raise ValueError(f"search.{name} must not be empty")
    return values


def expand_values(value, name: str) -> list:
    if not isinstance(value, dict):
        return as_list(value, name)
    if set(value) != {"start", "stop", "step"}:
        raise ValueError(f"search.{name} range needs exactly start, stop, and step")
    start, stop, step = (float(value[key]) for key in ("start", "stop", "step"))
    if step <= 0 or stop < start:
        raise ValueError(f"search.{name} range needs start <= stop and step > 0")
    count = round((stop - start) / step)
    if not math.isclose(start + count * step, stop, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError(f"search.{name} step must land exactly on stop")
    return [round(start + index * step, 10) for index in range(count + 1)]


def spacing_values(search: dict, axis: str, default_meters: float) -> list[float]:
    meter_key = f"{axis}_spacing"
    inch_key = f"{axis}_spacing_inches"
    if meter_key in search and inch_key in search:
        raise ValueError(f"search cannot contain both {meter_key} and {inch_key}")
    if inch_key in search:
        return [value * INCH_TO_METERS for value in expand_values(search[inch_key], inch_key)]
    return expand_values(search.get(meter_key, default_meters), meter_key)


def load_config(path: Path) -> dict:
    config = json.loads(path.read_text(encoding="utf-8"))
    for key in ("project", "base_scene", "assets", "search"):
        if key not in config:
            raise ValueError(f"missing config key: {key}")
    if not config["assets"]:
        raise ValueError("assets must not be empty")
    for asset in config["assets"]:
        if not asset.get("name") or not asset.get("folder"):
            raise ValueError("each asset needs non-empty name and folder values")
    return config


def resolve_path(base: Path, value: str | Path) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def validate_helios_generation(config: dict) -> None:
    generation = config.get("helios_generation")
    if not generation:
        return
    count = int(generation.get("count_per_type", 0))
    if count < 40:
        raise ValueError("helios_generation.count_per_type must be at least 40")
    seed_start = generation.get("seed_start", "random")
    if seed_start != "random" and (int(seed_start) < 0 or int(seed_start) + count - 1 > 0xFFFFFFFF):
        raise ValueError("helios_generation seed range must fit in an unsigned 32-bit integer")
    for asset in config["assets"]:
        ratio = asset.get("leaf_aspect_ratio_range")
        if not isinstance(ratio, list) or len(ratio) != 2 or ratio[0] <= 0 or ratio[0] > ratio[1]:
            raise ValueError(f"asset {asset['name']} needs a positive leaf_aspect_ratio_range [min, max]")


def inspect_model_libraries(config: dict, project: Path, required_models: int) -> dict:
    supported_extensions = {".eveprefab", ".obj", ".gltf", ".glb", ".blend", ".ply", ".fbx", ".dae", ".x3d"}
    assets_root = project.parent / "Assets"
    libraries = {}
    for asset in config["assets"]:
        folder = (assets_root / asset["folder"]).resolve()
        models = sorted(path for path in folder.iterdir() if path.is_file() and path.suffix.lower() in supported_extensions)
        if len(models) < required_models:
            raise RuntimeError(
                f"model library {asset['name']} has {len(models)} models, but the largest field needs {required_models}: {folder}"
            )
        seed_start = None
        entries_by_name = {}
        manifest_path = folder / "manifest.json"
        if manifest_path.is_file():
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            entries = manifest.get("models", [])
            entries_by_name = {entry["obj"]: entry for entry in entries}
            if entries:
                seed_start = entries[0].get("seed")
        libraries[asset["name"]] = {
            "folder": str(folder), "count": len(models), "seed_start": seed_start,
            "models": [{"obj": path.name, "seed": entries_by_name.get(path.name, {}).get("seed")}
                       for path in models],
        }
    return {"prepared_this_run": False, "assets": libraries}


def prepare_helios_libraries(config: dict, config_dir: Path, project: Path, required_models: int) -> dict:
    generation = config.get("helios_generation")
    if not generation:
        raise ValueError("helios_generation is required to prepare the model libraries")

    count = int(generation["count_per_type"])
    requested_seed = generation.get("seed_start", "random")
    seed_start = secrets.randbelow(0x100000000 - count) if requested_seed == "random" else int(requested_seed)
    build_dir = resolve_path(config_dir, generation.get("build_dir", "../out/helios-soybean-exporter/build"))
    exporter = generation.get("exporter")
    helios_root = generation.get("helios_root")
    if not exporter and not helios_root:
        raise ValueError("helios_generation needs exporter or helios_root")

    common = [
        sys.executable,
        str(Path(__file__).with_name("export_helios_soybeans.py")),
        "--build-dir",
        str(build_dir),
        "--count",
        str(count),
        "--seed-start",
        str(seed_start),
        "--age-days",
        str(generation.get("age_days", 45.0)),
    ]
    if exporter:
        common.extend(("--exporter", str(resolve_path(config_dir, exporter))))
    else:
        common.extend(("--helios-root", str(resolve_path(config_dir, helios_root))))
    for option, key in (
        ("--max-nodes", "max_nodes"),
        ("--internode-length-m", "internode_length_m"),
        ("--leaf-subdivisions", "leaf_subdivisions"),
    ):
        if key in generation:
            common.extend((option, str(generation[key])))
    for option, key in (
        ("--leaf-size-range-m", "leaf_size_range_m"),
        ("--petiole-length-range-m", "petiole_length_range_m"),
    ):
        if key in generation:
            common.extend((option, *(str(value) for value in generation[key])))

    assets_root = project.parent / "Assets"
    for asset_index, asset in enumerate(config["assets"]):
        command = [
            *common,
            "--output-dir",
            str((assets_root / asset["folder"]).resolve()),
            "--leaf-aspect-ratio-range",
            *(str(value) for value in asset["leaf_aspect_ratio_range"]),
        ]
        if asset_index > 0 and not exporter:
            command.append("--skip-build")
        subprocess.run(command, check=True)

    libraries = inspect_model_libraries(config, project, required_models)
    libraries["prepared_this_run"] = True
    return libraries


def build_cases(config: dict) -> list[dict]:
    search = config["search"]
    dimensions = {
        "rows": as_list(search.get("rows", 4), "rows"),
        "columns": as_list(search.get("columns", 10), "columns"),
        "column_spacing": spacing_values(search, "column", 0.3),
        "row_spacing": spacing_values(search, "row", 0.75),
        "seed": as_list(search.get("seed", 1), "seed"),
        "model_scale": as_list(search.get("model_scale", 1.0), "model_scale"),
        "random_yaw": as_list(search.get("random_yaw", 0.0), "random_yaw"),
        "roi": as_list(search.get("roi", 1.0), "roi"),
        "model_rotation": as_list(search.get("model_rotation", [[-90.0, 0.0, 0.0]]), "model_rotation"),
    }
    if "spacing_pairs_inches" in search:
        dimensions.pop("row_spacing")
        dimensions.pop("column_spacing")
        dimensions["spacing_pair_inches"] = as_list(search["spacing_pairs_inches"], "spacing_pairs_inches")
    if "samples" in search:
        dimensions["samples"] = as_list(search["samples"], "samples")
    cases = []
    names = list(dimensions)
    for asset, values in itertools.product(config["assets"], itertools.product(*(dimensions[name] for name in names))):
        case = {name: value for name, value in zip(names, values)}
        if "spacing_pair_inches" in case:
            row, column = case.pop("spacing_pair_inches")
            case.update(row_spacing=row * INCH_TO_METERS, column_spacing=column * INCH_TO_METERS)
        if "samples" in case and (not isinstance(case["samples"], int) or case["samples"] <= 0):
            raise ValueError("search.samples must contain positive integers")
        case["asset_name"] = asset["name"]
        case["asset_folder"] = asset["folder"]
        case["column_spacing_inches"] = round(case["column_spacing"] / INCH_TO_METERS, 10)
        case["row_spacing_inches"] = round(case["row_spacing"] / INCH_TO_METERS, 10)
        if len(case["model_rotation"]) != 3:
            raise ValueError("each model_rotation entry must contain exactly three values")
        if case["rows"] <= 0 or case["columns"] <= 0:
            raise ValueError("rows and columns must be positive")
        if not math.isfinite(case["roi"]) or not 0 < case["roi"] <= 1:
            raise ValueError("search.roi must be finite and in (0, 1]")
        if case["column_spacing"] < 0 or case["row_spacing"] < 0 or case["model_scale"] <= 0:
            raise ValueError("spacing must be non-negative and model_scale must be positive")
        cases.append(case)
    return cases


def vec3_columns(prefix: str, value) -> dict[str, float]:
    return {f"{prefix}_r": value.x, f"{prefix}_g": value.y, f"{prefix}_b": value.z}


def engine_vec3(evo, values):
    result = evo.Vec3()
    result.x, result.y, result.z = values
    return result


def resolve_lighting(config, config_dir):
    """Keep legacy skydome settings unless a linear HDR source is explicitly selected."""
    lighting = {
        "type": "skydome", "sun_euler_degrees": [90.0, 0.0, 0.0],
        "sun_angular_diameter_radians": 0.00918043, "sun_intensity": 1.0,
        "sun_color": [1.0, 1.0, 1.0], "skylight_intensity": 1.0,
        "ambient_light_intensity": 0.1, "gamma": 2.2,
    }
    lighting.update(config.get("lighting", {}))
    if lighting["type"] not in {"skydome", "hdr"}:
        raise ValueError("lighting.type must be skydome or hdr")
    if len(lighting["sun_euler_degrees"]) != 3 or len(lighting["sun_color"]) != 3:
        raise ValueError("lighting sun_euler_degrees and sun_color must each contain three values")
    for key in ("sun_angular_diameter_radians", "sun_intensity", "skylight_intensity", "ambient_light_intensity", "gamma"):
        if not math.isfinite(lighting[key]) or lighting[key] < 0 or (key == "gamma" and lighting[key] == 0):
            raise ValueError(f"invalid lighting.{key}")
    if lighting["type"] == "hdr":
        hdr = resolve_path(config_dir, lighting["hdr_path"])
        if hdr.suffix.lower() != ".hdr" or not hdr.is_file():
            raise ValueError(f"missing HDR environment: {hdr}")
        if not math.isfinite(lighting.get("rotation_degrees", 0.0)):
            raise ValueError("lighting.rotation_degrees must be finite")
        with hdr.open("rb") as stream:
            hdr_sha256 = hashlib.file_digest(stream, "sha256").hexdigest()
        lighting.update(hdr_path=str(hdr), hdr_sha256=hdr_sha256,
                        rotation_degrees=lighting.get("rotation_degrees", 0.0),
                        sun_intensity=0.0, ambient_light_intensity=0.0)
    return lighting


def configure_lighting(evo, lighting):
    if lighting["type"] == "hdr":
        success = evo.ConfigureCurrentSceneCudaEnvironmentMapForCapture(
            lighting["hdr_path"], lighting["skylight_intensity"], lighting["rotation_degrees"], lighting["gamma"])
    else:
        success = evo.ConfigureCurrentSceneCudaOutdoorLightingForCapture(
            engine_vec3(evo, lighting["sun_euler_degrees"]), lighting["sun_angular_diameter_radians"],
            lighting["sun_intensity"], engine_vec3(evo, lighting["sun_color"]),
            lighting["skylight_intensity"], lighting["ambient_light_intensity"], lighting["gamma"])
    if not success:
        raise RuntimeError(f"failed to configure the OptiX {lighting['type']} environment")


def case_columns(run_id: int, case: dict, ray: dict, lighting: dict, library: dict) -> dict:
    rotation = case["model_rotation"]
    asset_library = library["assets"][case["asset_name"]]
    return {
        "run_id": run_id,
        "asset_name": case["asset_name"],
        "asset_folder": case["asset_folder"],
        "rows": case["rows"],
        "columns": case["columns"],
        "roi": case["roi"],
        "column_spacing_inches": case["column_spacing_inches"],
        "row_spacing_inches": case["row_spacing_inches"],
        "column_spacing": case["column_spacing"],
        "row_spacing": case["row_spacing"],
        "seed": case["seed"],
        "model_library_seed_start": asset_library["seed_start"],
        "model_library_size": asset_library["count"],
        "model_rotation_x": rotation[0],
        "model_rotation_y": rotation[1],
        "model_rotation_z": rotation[2],
        "model_scale": case["model_scale"],
        "random_yaw": case["random_yaw"],
        "samples": ray["samples"],
        "bounces": ray["bounces"],
        "illumination_seed": case["seed"] + ray["seed_offset"],
        "push_normal_distance": ray["push_normal_distance"],
        "sun_euler_x": lighting["sun_euler_degrees"][0],
        "sun_euler_y": lighting["sun_euler_degrees"][1],
        "sun_euler_z": lighting["sun_euler_degrees"][2],
        "sun_angular_diameter_radians": lighting["sun_angular_diameter_radians"],
        "sun_intensity": lighting["sun_intensity"],
        "sun_color_r": lighting["sun_color"][0],
        "sun_color_g": lighting["sun_color"][1],
        "sun_color_b": lighting["sun_color"][2],
        "skylight_intensity": lighting["skylight_intensity"],
        "ambient_light_intensity": lighting["ambient_light_intensity"],
        "gamma": lighting["gamma"],
        "environment_lighting_type": lighting["type"],
        "hdr_path": lighting.get("hdr_path", ""),
        "hdr_sha256": lighting.get("hdr_sha256", ""),
        "environment_rotation_degrees": lighting.get("rotation_degrees", 0.0),
    }


def main() -> int:
    started_at = datetime.now(timezone.utc).isoformat()
    started = time.perf_counter()
    args = parse_args()
    config_path = args.config.resolve()
    config = load_config(config_path)
    validate_helios_generation(config)
    cases = build_cases(config)
    run_ids = select_run_ids(config, len(cases))
    required_models = max(case["rows"] * case["columns"] for case in cases)
    config_dir = config_path.parent
    project = resolve_path(config_dir, config["project"])
    ray = {"samples": 64, "bounces": 4, "seed_offset": 0, "push_normal_distance": 0.0001}
    ray.update(config.get("ray_tracing", {}))
    if ray["samples"] <= 0 or ray["bounces"] < 0 or ray["push_normal_distance"] < 0:
        raise ValueError("invalid ray_tracing samples, bounces, or push_normal_distance")
    print(f"[field-search] validated {len(cases)} parameter combinations; executing {len(run_ids)}", flush=True)

    lighting = resolve_lighting(config, config_dir)
    if args.validate_only:
        if args.prepare_model_library_only:
            generation = config.get("helios_generation", {})
            print(
                f"[field-search] would generate {generation['count_per_type']} matched models per asset type",
                flush=True,
            )
        else:
            libraries = inspect_model_libraries(config, project, required_models)
            counts = ", ".join(f"{name}={values['count']}" for name, values in libraries["assets"].items())
            print(f"[field-search] reusing model libraries: {counts}", flush=True)
        return 0

    if args.prepare_model_library_only:
        libraries = prepare_helios_libraries(config, config_dir, project, required_models)
        counts = ", ".join(f"{name}={values['count']}" for name, values in libraries["assets"].items())
        print(f"[field-search] prepared model libraries: {counts}", flush=True)
        return 0

    output_dir = args.output_dir or Path(config.get("output_directory", "field-illumination-results"))
    output_dir = output_dir.resolve() if output_dir.is_absolute() else (config_dir / output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if any((output_dir / name).exists() for name in ("field_summary.csv", "plant_measurements.csv", "completion.json")):
        raise ValueError(f"Experiment output already contains measurements; choose a new output directory: {output_dir}")
    library = inspect_model_libraries(config, project, required_models)
    (output_dir / "search_manifest.json").write_text(
        json.dumps({
            "config": config,
            "executed_run_ids": run_ids,
            "started_at_utc": started_at,
            "resolved_model_library": library,
            "resolved_lighting": lighting,
            "measurement_roi": {
                "fraction": "per grid dimension, default 1",
                "selection": "central cells; nearest integer (ties up), at least 1 per dimension",
                "indices": "original full-field row, column, and model_index",
                "ground_area": "ROI cell count * row_spacing * column_spacing; excludes buffer cells",
            },
            "organ_reporting": {
                "schema_version": 2,
                "mesh_name_mapping": {"shoot": "stem", "petiolule": "petiole"},
                "unknown_mesh_names": "retained",
                "average_flux": "per geometric area; leaves average front/back samples",
                "total_flux": "sum of incident irradiance over sampled faces times geometric area; relative RGB, not absorbed PAR",
            },
        }, indent=2) + "\n", encoding="utf-8"
    )

    module_dir = args.module_dir.resolve()
    os.chdir(module_dir)
    sys.path.insert(0, str(module_dir))
    import PyDigitalAgriculture as evo

    # Register OptiX before loading the scene so all estimator component types are available.
    evo.PushRenderLayer()
    evo.PushRayTracerLayer()
    evo.RunWithScene(str(project), config["base_scene"])
    summary_path = output_dir / "field_summary.csv"
    plants_path = output_dir / "plant_measurements.csv"
    organ_summary_path = output_dir / "field_organ_summary.csv"
    organs_path = output_dir / "plant_organ_measurements.csv"
    field = None
    last_configuration = None
    try:
        for name in config.get("delete_entities", DEFAULT_DELETE_ENTITIES):
            evo.DeleteCurrentSceneEntitiesByName(name)

        # Soil is scene context rather than part of the generated field; fail before a long search if it is absent.
        for name in config.get("required_entities", []):
            if evo.GetCurrentSceneEntityCountByName(name) == 0:
                raise RuntimeError(f"required scene entity is missing: {name}")

        configure_lighting(evo, lighting)

        summary_file = summary_path.open("w", newline="", encoding="utf-8")
        plants_file = plants_path.open("w", newline="", encoding="utf-8")
        organ_summary_file = organ_summary_path.open("w", newline="", encoding="utf-8")
        organs_file = organs_path.open("w", newline="", encoding="utf-8")
        timing_file = (output_dir / "case_timings.csv").open("w", newline="", encoding="utf-8")
        with summary_file, plants_file, organ_summary_file, organs_file, timing_file:
            timing_writer = csv.DictWriter(timing_file, fieldnames=["run_id", "started_at_utc", "completed_at_utc", "elapsed_seconds"])
            timing_writer.writeheader()
            summary_writer = None
            plants_writer = None
            organ_summary_writer = None
            organs_writer = None
            for run_id in run_ids:
                case = cases[run_id]
                case_started_at = datetime.now(timezone.utc).isoformat()
                case_started = time.perf_counter()
                case_ray = {**ray, "samples": case.get("samples", ray["samples"])}
                rotation = case["model_rotation"]
                configure_args = (
                    case["asset_folder"], case["rows"], case["columns"], case["column_spacing"],
                    case["row_spacing"], case["seed"], rotation[0], rotation[1], rotation[2],
                    case["model_scale"], case["random_yaw"], bool(config.get("recursive", False)), case["roi"],
                )
                if field is None:
                    field = evo.CreatePrefabField(*configure_args)
                elif configure_args != last_configuration:
                    evo.ConfigurePrefabField(field, *configure_args)
                last_configuration = configure_args
                expected = case["rows"] * case["columns"]
                if evo.GetPrefabFieldModelCount(field) != expected:
                    raise RuntimeError(f"run {run_id}: field generation did not produce {expected} models")
                region = evo.GetPrefabFieldMeasurementRegion(field)
                measured_count = region.rows * region.columns
                if not evo.WaitForCurrentSceneReady(int(config.get("maximum_ready_frames", 3000))):
                    raise RuntimeError(f"run {run_id}: imported field assets did not finish loading")

                print(f"[field-search] measuring run {run_id + 1}/{len(cases)}: {case['asset_name']}", flush=True)
                sampling_started = time.perf_counter()
                summary = evo.EstimatePrefabFieldIlluminationSummary(
                    field, case_ray["samples"], case_ray["bounces"], case["seed"] + case_ray["seed_offset"],
                    case_ray["push_normal_distance"],
                )
                if len(summary.models) != measured_count:
                    raise RuntimeError(f"run {run_id}: illumination returned {len(summary.models)} of {measured_count} ROI plants")
                common = case_columns(run_id, case, case_ray, lighting, library)
                common.update({
                    "started_at_utc": case_started_at,
                    "setup_seconds": sampling_started - case_started,
                    "sampling_seconds": time.perf_counter() - sampling_started,
                    "field_plant_count": expected,
                    "roi_row_start": region.row_start,
                    "roi_column_start": region.column_start,
                    "roi_rows": region.rows,
                    "roi_columns": region.columns,
                    "measurement_ground_area": measured_count * case["row_spacing"] * case["column_spacing"],
                })
                summary_row = {
                    **common,
                    "plant_count": len(summary.models),
                    "total_area": summary.total_area,
                    **vec3_columns("total_flux", summary.total_flux),
                    **vec3_columns("area_weighted_average_flux", summary.area_weighted_average_flux),
                    **vec3_columns("mean_model_average_flux", summary.mean_model_average_flux),
                }
                if summary_writer is None:
                    summary_writer = csv.DictWriter(summary_file, fieldnames=list(summary_row))
                    summary_writer.writeheader()
                summary_writer.writerow(summary_row)

                # Organ scores use the same samples as the whole plant, with every mesh still casting shadows.
                for name, organ in summary.organs.items():
                    row = {
                        **common,
                        "organ": name,
                        "total_area": organ.total_area,
                        **vec3_columns("total_flux", organ.total_flux),
                        **vec3_columns("average_flux", organ.average_flux),
                    }
                    if organ_summary_writer is None:
                        organ_summary_writer = csv.DictWriter(organ_summary_file, fieldnames=list(row))
                        organ_summary_writer.writeheader()
                    organ_summary_writer.writerow(row)

                for model in summary.models:
                    source = library["assets"][case["asset_name"]]["models"][model.asset_index]
                    plant_row = {
                        **common,
                        "model_index": model.model_index,
                        "asset_index": model.asset_index,
                        "source_obj": source["obj"],
                        "generation_seed": source["seed"],
                        "row": model.row,
                        "column": model.column,
                        "total_area": model.total_area,
                        **vec3_columns("total_flux", model.total_flux),
                        **vec3_columns("average_flux", model.average_flux),
                    }
                    if plants_writer is None:
                        plants_writer = csv.DictWriter(plants_file, fieldnames=list(plant_row))
                        plants_writer.writeheader()
                    plants_writer.writerow(plant_row)
                    for name, organ in model.organs.items():
                        row = {
                            **common,
                            "model_index": model.model_index,
                            "asset_index": model.asset_index,
                            "source_obj": source["obj"],
                            "generation_seed": source["seed"],
                            "row": model.row,
                            "column": model.column,
                            "organ": name,
                            "total_area": organ.total_area,
                            **vec3_columns("total_flux", organ.total_flux),
                            **vec3_columns("average_flux", organ.average_flux),
                        }
                        if organs_writer is None:
                            organs_writer = csv.DictWriter(organs_file, fieldnames=list(row))
                            organs_writer.writeheader()
                        organs_writer.writerow(row)
                summary_file.flush()
                plants_file.flush()
                organ_summary_file.flush()
                organs_file.flush()

                # Capture selected, reproducible cases without changing their measurement lighting.
                if run_id in config.get("capture_run_ids", []):
                    capture_dir = output_dir / "captures"
                    capture_dir.mkdir(exist_ok=True)
                    extent = max((case["columns"] - 1) * case["column_spacing"],
                                 (case["rows"] - 1) * case["row_spacing"], 0.7)
                    target = config.get("capture_target", [0.0, 0.4, 0.0])
                    position = config.get("capture_position", [extent * 0.8, extent * 0.7 + 0.6, extent * 1.1])
                    if not evo.SetMainCameraLookAt(engine_vec3(evo, position), engine_vec3(evo, target),
                                                  engine_vec3(evo, [0, 1, 0]), 42.0):
                        raise RuntimeError("failed to set capture camera")
                    if not evo.CaptureCurrentSceneCuda(1600, 1000, str(capture_dir / f"run_{run_id:04d}.png"),
                                                       256, ray["bounces"], lighting["gamma"], 0.0):
                        raise RuntimeError("failed to capture OptiX image")
                    scene_path = config.get("save_scene_prefix")
                    if scene_path and not evo.SaveCurrentSceneAs(f"{scene_path}_{run_id:04d}.evescene"):
                        raise RuntimeError("failed to save representative scene")
                timing_writer.writerow({"run_id": run_id, "started_at_utc": case_started_at,
                                        "completed_at_utc": datetime.now(timezone.utc).isoformat(),
                                        "elapsed_seconds": time.perf_counter() - case_started})
                timing_file.flush()
    finally:
        evo.Terminate()

    (output_dir / "completion.json").write_text(json.dumps({
        "started_at_utc": started_at, "completed_at_utc": datetime.now(timezone.utc).isoformat(),
        "elapsed_seconds": time.perf_counter() - started, "completed_cases": len(run_ids),
        "include_in_study_analysis": config.get("include_in_study_analysis", True),
    }, indent=2) + "\n", encoding="utf-8")

    print(f"[field-search] wrote {summary_path}", flush=True)
    print(f"[field-search] wrote {plants_path}", flush=True)
    print(f"[field-search] wrote {organ_summary_path}", flush=True)
    print(f"[field-search] wrote {organs_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
