"""Measure a scene through PyDigitalAgriculture, or redraw cached voxel data without loading the engine."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = "GeneratedAssets/Scenes/Sorghum_4x10_GrowthStage03.evescene"


def scene_layout(path):
    """Read entity metadata/transforms without decoding the scene's large embedded mesh assets."""
    import yaml

    sections = []
    keep = True
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.startswith("LocalAssets:"):
                keep = False
            if line.startswith("data_component_storage_list:"):
                keep = True
            if keep:
                sections.append(line)
    scene = yaml.load("".join(sections), Loader=yaml.CSafeLoader)
    transforms = {}
    for storage in scene["data_component_storage_list"]:
        index = next(i for i, value in enumerate(storage["data_component_types"])
                     if value["type_name"] == "GlobalTransform")
        for entry in storage["chunk_array"]:
            transforms[entry["h"]] = np.frombuffer(entry["dc"][index]["d"], dtype="<f4").reshape(4, 4).T
    plants = [entity for entity in scene["entity_metadata_list"]
              if any(c["tn"] == "SorghumLS" for c in entity.get("pc", []))]
    return np.array([transforms[entity["h"]][:3, 3] for entity in plants], dtype=np.float32).reshape(-1, 3)


def file_hash(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def validate_result(result):
    shape = tuple(result["resolution"])
    if len(shape) != 3 or any(n < 2 for n in shape):
        raise ValueError("Isosurfaces require at least two voxel centers along each axis")
    for key, expected in (("energy", shape + (3,)), ("intensity", shape),
                          ("centers_local", shape + (3,)), ("sample_count", shape)):
        array = result[key]
        if array.shape != expected or not np.all(np.isfinite(array)):
            raise ValueError(f"Invalid {key} shape or non-finite values")
    if not np.all(result["sample_count"] == result["samples_per_voxel"]):
        raise ValueError("Incomplete voxel sampling")
    if not np.allclose(result["intensity"], np.linalg.norm(result["energy"], axis=-1)):
        raise ValueError("Intensity is inconsistent with the mean RGB energy")
    matrix = np.asarray(result["local_to_world"])
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("Invalid local-to-world transform")


def make_grid(result, sigma=0):
    import pyvista as pv
    from scipy.ndimage import gaussian_filter

    validate_result(result)
    if not np.isfinite(sigma) or sigma < 0:
        raise ValueError("Smoothing sigma must be finite and non-negative")
    # Cell averages are interpreted as samples at cell centers, not at volume-boundary vertices.
    spacing = np.asarray(result["volume_size"]) / np.asarray(result["resolution"])
    grid = pv.ImageData(dimensions=result["resolution"], spacing=spacing,
                        origin=-np.asarray(result["volume_size"]) / 2 + spacing / 2)
    values = result["intensity"]
    if sigma > 0:
        values = gaussian_filter(values, sigma=sigma, mode="nearest")
    # NumPy result is xyz/C-order; VTK's first (x) dimension varies fastest.
    grid.point_data["intensity"] = values.ravel(order="F")
    grid.point_data["raw_intensity"] = result["intensity"].ravel(order="F")
    return grid.cast_to_structured_grid().transform(result["local_to_world"], inplace=False)


def save_result(result, output, provenance):
    output.mkdir(parents=True, exist_ok=False)
    arrays = {key: value for key, value in result.items() if isinstance(value, np.ndarray)}
    metadata = {key: value for key, value in result.items() if key not in arrays}
    metadata.update(provenance)
    np.savez_compressed(output / "voxels.npz", **arrays)
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


def load_result(folder):
    result = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
    with np.load(folder / "voxels.npz", allow_pickle=False) as archive:
        result.update({key: archive[key].copy() for key in archive.files})
    validate_result(result)
    return result


def plot_result(result, output, levels=None, sigma=0, interactive=False):
    import pyvista as pv

    grid = make_grid(result, sigma)
    values = grid["intensity"]
    if levels is None:
        levels = np.unique(np.quantile(values, [0.25, 0.5, 0.75])).tolist()
    if not levels or not np.isfinite(levels).all():
        raise ValueError("Supply at least one finite isovalue")
    if any(not values.min() < level < values.max() for level in levels):
        raise ValueError(f"Isovalues must lie strictly within ({values.min()}, {values.max()})")
    output.mkdir(parents=True, exist_ok=True)
    grid.save(output / "voxel_centers.vts")
    plotter = pv.Plotter(shape=(1, 2), off_screen=not interactive, window_size=(1600, 850))
    plotter.set_background("#f1f4f7", all_renderers=True)
    colors = ["#2767a0", "#efb040", "#c84540"]
    surfaces = []
    plotter.subplot(0, 0)
    plotter.add_text("Sorghum 4 x 10 | illumination isosurfaces", font_size=15, color="#182b40")
    for index, level in enumerate(levels):
        surface = grid.contour([level], scalars="intensity")
        surface.save(output / f"isosurface_{index:02d}.vtp")
        surfaces.append({"level": float(level), "triangles": surface.n_cells})
        color = colors[index % len(colors)]
        if surface.n_cells:
            plotter.add_mesh(surface, color=color, opacity=0.45)
        plotter.add_text(f"I = {level:.4g}", position=(18, 770 - 27 * index), font_size=12, color=color)
    plotter.subplot(0, 1)
    plotter.add_text("Raw samples | horizontal sections", font_size=15, color="#182b40")
    center = np.asarray(result["volume_size"]) * np.array([0, -0.25, 0])
    matrix = result["local_to_world"]
    normal = np.linalg.inv(matrix[:3, :3]).T @ np.array([0, 1, 0])
    for fraction in [-0.25, 0, 0.25]:
        center[1] = result["volume_size"][1] * fraction
        origin = (matrix @ np.r_[center, 1])[:3]
        section = grid.slice(normal=normal, origin=origin)
        plotter.add_mesh(section, scalars="raw_intensity", cmap="viridis",
                         clim=(result["intensity"].min(), result["intensity"].max()),
                         scalar_bar_args={"title": "|mean RGB energy| (engine units)", "color": "#182b40"})
    roots = np.asarray(result.get("plant_roots_world", []))
    for column in [0, 1]:
        plotter.subplot(0, column)
        plotter.add_mesh(grid.outline(), color="#6b7888")
        if roots.size:
            plotter.add_points(roots, color="#23342a", point_size=7, render_points_as_spheres=True)
        plotter.add_axes(color="#182b40", ylabel="Y (up)", viewport=(0, 0.07, 0.18, 0.25))
        plotter.view_vector((1.2, 0.9, 1.4), viewup=(0, 1, 0))
        plotter.camera.zoom(0.8)
    plotter.subplot(0, 0)
    size_label = " x ".join(f"{v:.2f}" for v in result["volume_size"])
    plotter.add_text(f"Volume: {size_label} world units | sigma = {sigma:g} voxels\n"
                    "Dots: plant roots | center-sample interpolation | not calibrated PAR",
                     position="lower_left", font_size=10, color="#182b40")
    try:
        plotter.show(screenshot=str(output / "isosurfaces.png"), auto_close=True)
    finally:
        plotter.close()
    (output / "plot_settings.json").write_text(json.dumps({"smooth_sigma_voxels": sigma,
        "surfaces": surfaces, "raw_range": [float(result["intensity"].min()), float(result["intensity"].max())]},
        indent=2) + "\n", encoding="utf-8")


def measure(args):
    project = args.project.resolve()
    scene = project.parent / "Assets" / args.scene
    scene_sha = file_hash(scene)
    roots = scene_layout(scene)
    if args.center is None or args.size is None:
        if not len(roots):
            raise ValueError("No SorghumLS roots found; specify --center and --size explicitly")
        low, high = roots.min(axis=0).astype(float), roots.max(axis=0).astype(float)
        low -= [0.75, -0.05, 0.75]
        high += [0.75, 3.05, 0.75]
        center = args.center or ((low + high) / 2).tolist()
        size = args.size or (high - low).tolist()
    else:
        center, size = args.center, args.size
    module_dir = args.module_dir.resolve()
    os.chdir(module_dir)
    sys.path.insert(0, str(module_dir))
    import PyDigitalAgriculture as evo

    start = time.perf_counter()
    evo.PushRenderLayer()
    evo.PushRayTracerLayer()
    evo.RunWithScene(str(project), args.scene)
    try:
        if not evo.WaitForCurrentSceneReady(3000):
            raise RuntimeError("Scene did not finish loading")
        def vec3(values):
            value = evo.Vec3()
            value.x, value.y, value.z = values
            return value
        if not evo.ConfigureCurrentSceneCudaOutdoorLightingForCapture(
                vec3((55, 30, 0)), 0.00918043, 1.0, vec3((1, 1, 1)), 1.0, 0.1, 1.0):
            raise RuntimeError("Skydome initialization failed")
        sensor = evo.CreateVoxelIlluminationEstimator(center=center)
        if args.verify_api:
            verify_api(evo, sensor)
        ready = time.perf_counter()
        print(f"Measuring {args.resolution}, center={center}, size={size}, plants={len(roots)}", flush=True)
        result = evo.EstimateVoxelIllumination(sensor, size, args.resolution, args.samples_per_voxel,
                                               args.rays_per_point, args.bounces, args.seed)
        elapsed = time.perf_counter() - ready
        validate_result(result)
        # Exercise ownership while the engine is alive, then again after teardown.
        evo.DeleteEntity(sensor)
        validate_result(result)
    finally:
        evo.Terminate()
    validate_result(result)
    if file_hash(scene) != scene_sha:
        raise RuntimeError("The source scene changed during measurement")
    save_result(result, args.output, {"scene": str(scene), "scene_sha256": scene_sha,
        "project": str(project), "plant_roots_world": roots.tolist(), "plant_count": len(roots),
        "lighting_setup": "Explicit Skydome; sun Euler degrees (55,30,0); no HDR",
        "startup_seconds": ready - start, "estimate_seconds": elapsed, "api_checks_enabled": args.verify_api,
        "module_sha256": file_hash(next(module_dir.glob("PyDigitalAgriculture*.pyd")))})
    return load_result(args.output)


def verify_api(evo, sensor):
    for settings in ({"resolution": (0, 2, 2)}, {"resolution": (1000000, 1000000, 1000000)},
                     {"samples_per_voxel": 0}, {"volume_size": (float("nan"), 1, 1)}):
        kwargs = dict(volume_size=(1, 1, 1), resolution=(2, 3, 4), samples_per_voxel=2, rays_per_point=8)
        kwargs.update(settings)
        try:
            evo.EstimateVoxelIllumination(sensor, **kwargs)
        except ValueError:
            pass
        else:
            raise AssertionError(f"Invalid settings accepted: {settings}")
    first = evo.EstimateVoxelIllumination(sensor, (1, 1, 1), (2, 3, 4), 2, 8, 1, 42)
    original = first["energy"].copy()
    repeated = evo.EstimateVoxelIllumination(sensor, (1, 1, 1), (2, 3, 4), 2, 8, 1, 42)
    np.testing.assert_array_equal(original, repeated["energy"])
    evo.EstimateVoxelIllumination(sensor, (2, 2, 2), (3, 2, 2), 2, 8, 1, 43)
    np.testing.assert_array_equal(first["energy"], original)
    assert first["energy"].flags.owndata and not np.shares_memory(first["energy"], repeated["energy"])
    rotated = evo.CreateVoxelIlluminationEstimator(center=(1, 2, 3), rotation_degrees=(0, 0, 90))
    data = evo.EstimateVoxelIllumination(rotated, (1, 1, 1), (2, 2, 2), 1, 8, 1, 0)
    np.testing.assert_allclose(data["local_to_world"][:3, 3], (1, 2, 3), atol=1e-6)
    np.testing.assert_allclose(data["sensor_normal_world"], (-1, 0, 0), atol=1e-6)
    evo.DeleteEntity(rotated)
    validate_result(data)
    print("API checks passed: validation, deterministic repeats, snapshots, transform and sensor normal", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, help="Redraw a saved result; do not import/start the engine")
    parser.add_argument("--project", type=Path, default=ROOT / "Resources/DigitalAgricultureProject/test_lsystem_sorghum_genotype_c_aug11_2x10.eveproj")
    parser.add_argument("--scene", default=DEFAULT_SCENE)
    parser.add_argument("--module-dir", type=Path, default=ROOT / "out/install/vs2026-x64/python")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--center", type=float, nargs=3)
    parser.add_argument("--size", type=float, nargs=3)
    parser.add_argument("--resolution", type=int, nargs=3, default=[40, 24, 40])
    parser.add_argument("--samples-per-voxel", type=int, default=8)
    parser.add_argument("--rays-per-point", type=int, default=128)
    parser.add_argument("--bounces", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--levels", type=float, nargs="+")
    parser.add_argument("--smooth-sigma", type=float, default=0)
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--verify-api", action="store_true", help="Run small actual-OptiX API checks before measurement")
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.input:
        result = load_result(args.input.resolve())
        plot_dir = args.output
    else:
        if args.output.exists():
            parser.error("Measurement output already exists; use a new directory or --input to redraw")
        result = measure(args)
        plot_dir = args.output / "plots"
    plot_result(result, plot_dir, args.levels, args.smooth_sigma, args.interactive)
    print(f"Saved visualization: {plot_dir / 'isosurfaces.png'}", flush=True)


if __name__ == "__main__":
    main()
