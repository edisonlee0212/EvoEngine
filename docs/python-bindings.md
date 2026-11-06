# Python Bindings

[Back to README](../README.md)

`PythonBinding` builds pybind11 modules for automation workflows.

`PyEvoEngine` exposes the general engine API. `PyDigitalAgriculture` also exposes that API and adds sorghum, prefab field, and illumination bindings. Field creation, field measurement, and voxel illumination functions and their result classes are registered only in `PyDigitalAgriculture`. Build it with the DigitalAgriculture and DatasetGeneration runtime packages enabled.

The older `sorghum_field_illumination_estimation.py` workflow still needs its historical coordinate-field and PAR-sensor functions (`InstantiateSorghumField`, `SetPARSensors`, and related result APIs) ported to the current package interfaces before it can run unchanged.

Example scripts live in `PythonBinding`. `PyEcoSysLab` remains disabled while its Python bindings are migrated to runtime packages.

Build and install output is documented in [building.md](building.md). Local render tests that exercise Python capture workflows are documented in [testing.md](testing.md).

## OptiX clouds from Python

Both CUDA-enabled modules expose the same cloud controls as **Ray Tracer Layer**. After starting a
project with a ray tracer layer, read the settings, edit the copy, and apply it to the running layer.
`RunLSystemSorghumProject` starts this layer automatically. For `RunWindowless`, call `PushRayTracerLayer()` first.

```python
import PyDigitalAgriculture as evo

# With the project running:
clouds = evo.GetRayTracerCloudSettings()
clouds.ApplyPreset(evo.CloudFormation.BroadBank)
clouds.enabled = True
clouds.affect_illumination_probes = True
clouds.primary_step_count = 192
clouds.fine_step_scale = 0.15
evo.SetRayTracerCloudSettings(clouds)
```

The next triangle, voxel or PAR measurement uses the applied settings. To compare the same probe
under clear sky, set `clouds.enabled = False`, call the setter again and repeat with the same seed.
`affect_illumination_probes` independently controls the measurement and defaults to false.
For a Python camera capture with the layer's clouds, pass `use_layer_clouds=True` to
`CaptureCurrentSceneCuda`. This option defaults to false. The cloud buffer resolution and
temporal reprojection settings apply to the camera. Probes sample the cloud along their own rays.

`BroadBank` and `Scattered` apply the same five formation fields as the editor. Apply a preset before
individual overrides. The snapshot exposes all 31 cloud settings, including `coverage`, `density`,
`extinction_scale`, `bottom_altitude`, `top_altitude`, and `wind_direction` (an X/Z pair).
The setter uses the engine's range limits and normalizes the wind direction. Call the getter again
to read the effective values. These settings belong to the running layer and should be reapplied in a new run.

The [sorghum field PAR data note](sorghum-field-par-data.md) maps the 2021 ceptometer CSVs to plot positions and lists the measurement dates and calibration caveats.

The [Position-Y PARbar workflow](tau-diurnal-current-engine.md) adds L-system field generation and bar sensor APIs
under `PyDigitalAgriculture`, using the current CUDA measurement implementation and the collaborator's scene layout.

L-system `.sorghumls` descriptors can set `randomize_initial_leaf_azimuth: false`
and `initial_leaf_azimuth_degrees: 0` to fix the initial main-axis phyllotaxis phase.
Random initialization remains the default. This preserves the random draws used
by other traits; leaf roll and tiller jitter still follow their original distributions.
`PyDigitalAgriculture.GetSorghumLsInitialLeafAzimuths()` returns the actual initial
phase in degrees keyed by plant name, for verifying an experiment after growth.

For bounded uniform initialization, keep `randomize_initial_leaf_azimuth: true`,
set `initial_leaf_azimuth_degrees` to the arc start, and set
`initial_leaf_azimuth_range_degrees` to its total width (default 360).
The sampled angle is `(start + original_uniform_draw * width) mod 360`.
For example, start 315 and width 90 represent a circular interval centered
on 0 degrees with limits -45 to +45. The start and width are clamped to
0-360 degrees when loaded; random mode with width 0 also produces a fixed angle.

The matched-scene [soybean HDR rerun](soybean-hdr-rerun.md) documents selecting a Scene HDR through
`ConfigureCurrentSceneCudaEnvironmentMapForCapture` while reusing the original plant library and experimental cases.
