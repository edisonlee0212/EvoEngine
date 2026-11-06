# Voxel illumination from Python

Use Python 3.12 (the interpreter used to build `PyDigitalAgriculture`) and install
`Scripts/requirements-voxel.txt` into a virtual environment. Build/install
`PyDigitalAgriculture` with CUDA support before running the measurement.

```python
import PyDigitalAgriculture as evo

evo.PushRenderLayer()
evo.PushRayTracerLayer()
evo.RunWithScene(project_path, "GeneratedAssets/Scenes/Sorghum_4x10_GrowthStage03.evescene")
try:
    if not evo.WaitForCurrentSceneReady(3000):
        raise RuntimeError("Scene not ready")
    # Configure lighting explicitly, e.g. ConfigureCurrentSceneCudaOutdoorLightingForCapture.
    sensor = evo.CreateVoxelIlluminationEstimator(
        name="Light field", center=(2.58, 1.55, -7.51), rotation_degrees=(0, 0, 0))
    data = evo.EstimateVoxelIllumination(
        sensor, volume_size=(8.37, 3.03, 5.90), resolution=(40, 24, 40),
        samples_per_voxel=8, rays_per_point=128, bounces=4, seed=0)
finally:
    evo.Terminate()
# data remains valid after another estimate, entity deletion, or engine termination.
```

This runs the engine in the Python process; it does not attach to an existing editor.
Call estimation again on the same entity to change grid/sampling settings without
reloading the scene. Wait for scene resources after any geometry edits. Estimation
synchronizes transforms and the OptiX scene before invoking the existing component.
Calls are synchronous and intended for the engine's owning Python thread.

## Result contract

- `energy`: owned float32 array `(nx, ny, nz, 3)`, mean RGB energy per voxel.
- `intensity`: owned float32 `(nx, ny, nz)`, Euclidean norm of mean RGB energy.
- `centers_local`: owned float32 `(nx, ny, nz, 3)`.
- `sample_count`: owned uint32 `(nx, ny, nz)`.
- `local_to_world`: owned float32 `(4, 4)`, multiplying column position vectors.
- Other entries record resolution, volume size, sensor normal, sampling settings,
  and effective environment parameters at measurement time.

Arrays use XYZ axes in C order (Z fastest). Samples face local +Y transformed into
world space; this is a directional-sensor irradiance-like spatial field, not a full
angular light field. Values are uncalibrated engine units, **not PPFD**.
Non-positive/invalid settings and more than 1,000,000 virtual points raise errors.

## Reproducible measurement and visualization

From the repository root, with the matching Python environment:

```powershell
python Scripts/voxel_illumination.py --output out/voxel-growthstage03/run01
python Scripts/voxel_illumination.py --input out/voxel-growthstage03/run01 --output out/voxel-growthstage03/run01/smoothed --smooth-sigma 0.7
```

The first command loads the unchanged Sorghum 4x10 GrowthStage03 scene and uses
explicit Skydome lighting: sun Euler angles `(55,30,0)` degrees, solar angular
diameter `0.00918043` radians, white sun intensity `1`, skylight `1`, ambient `0.1`,
gamma `1`. It does not use the black background saved in this old scene or an HDR.
The source scene SHA256 and effective lighting are recorded.

Automatic volume bounds use the 40 saved SorghumLS root positions, a 0.75-unit
horizontal margin and a vertical range from 0.05 above the lowest root to 3.05
above the highest root. These are sampling bounds, not an exact canopy mesh AABB.
Override `--center X Y Z --size X Y Z` for another volume. Default settings are
40x24x40 voxels, 8 points/voxel, 128 rays/point, 4 bounces, seed 0.

Outputs: `voxels.npz`, `metadata.json`, `plots/isosurfaces.png`, a `.vts` grid,
individual `.vtp` surfaces and `plot_settings.json`. Measurement refuses to overwrite
an existing output directory. Raw data survive a visualization failure and can be
redrawn with `--input`. Plot-only mode never imports the engine.

Use `--levels 0.2 0.4 0.6` for absolute isovalues (must lie inside the measured range),
or omit it for the 25th/50th/75th percentiles. `--interactive` opens an interactive
viewer. `--smooth-sigma` is in voxel-index units and defaults to zero; smoothing is
optional postprocessing, not additional measurement accuracy. Raw slices are always
unsmoothed. The plotted dots are plant roots, not reconstructed plant geometry.

Cell averages are approximated as samples at voxel centers before contouring.
There is no extrapolation to the outer half-voxel boundary, no zero padding and no
automatic surface closing. VTK ordering is converted explicitly (X fastest).

```powershell
python -m unittest discover -s Scripts/tests -p test_voxel_illumination.py -v
```
