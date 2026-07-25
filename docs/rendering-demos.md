# Rendering Demos

[Back to rendering overview](rendering.md)

This page documents rendering-oriented demo profiles and the assets they generate or capture.

## Demo Preparation

Generated demo resources and missing launcher previews are prepared through one script:

```bat
python Scripts\prepare_demos.py --editor out\install\vs2026-x64\bin\EvoEngineEditor.exe
```

By default it walks Rendering, 3DGS, Bicycle, Bistro, and Rendering Regression in order, validates each demo, prepares
missing files, then validates again. Use `--demo bistro` to select one demo, `--validate` to validate only, `--prepare`
to prepare missing files only, and `--override` to force preparation without validation. Pass `--no-previews` when only
project/assets are needed.

Launcher thumbnails live under `Resources/Launcher/DemoPreviews`. The README gallery currently references the tracked
RT-Bistro and 3DGS-Bicycle launcher previews directly.

## Rendering

The `rendering` profile loads the Sponza-based Rendering demo. It is used for README/editor screenshots, DDGI smoke
coverage, material-preview coverage, directional/point light checks, and general render-path regression checks.

Key scene defaults:

- tracked Sponza sky, global reflection probe, and five asset-owned hallway/gallery reflection probes;
- imported Sponza punctual lights disabled;
- asset-owned DDGI volume enabled through the scene's `EnvironmentalLighting` asset;
- top-down directional light brightness `5.0`;
- yellow point light and non-shadow-casting visualizer for light-response checks;
- main camera configured for ray tracing by the README/smoke editor setup;
- scene camera remains rasterization for editor inspection.

## Rendering Regression

The `rendering-regression` launcher profile is the cross-technique regression scene. It is generated under:

```text
Resources/.generated/EvoEngine-DemoProjects/RenderingRegression
```

It combines:

- opaque, metallic, rough, transmissive, and emissive material probes;
- imported Sponza/Capoeira material texture slots;
- directional, point, and spot light probes;
- enabled skinned Capoeira probe;
- high-emission firefly clamp probe;
- low/high contrast Auto SPP convergence probes;
- transformed UV2/UV3 material and normal-map probes;
- repeat/clamp and nearest/linear glTF sampler probes;
- a minified sRGB checker whose expected linear mip value is `0.5`;
- a shared-index mirrored chart that requires MikkTSpace tangent splitting.

The scene exists for cross-technique regression coverage. Rasterization, RayTracing, and RayQuery are not expected to be
pixel-equal because their integrators and post-processing histories differ.

Its launcher thumbnail is generated from the actual profile:

```bat
python Scripts\prepare_demos.py --demo rendering-regression --override
```

## RT-Bistro

The `bistro` launcher profile opens a local ignored glTF validation project under:

```text
Resources/.generated/EvoEngine-DemoProjects/Bistro
```

Generate or refresh it with:

```bat
python Scripts\prepare_demos.py --demo bistro
```

The script fetches or reuses `https://github.com/zeux/niagara_bistro`, verifies `bistro.gltf`, prepares project assets
under `Assets/Models/Bistro`, records selected source counts, and falls back from symlinks to copies on Windows when
needed. Bistro is kept under `Resources/.generated` so the large source checkout and generated project remain local
branch evidence instead of Resources submodule content.

The profile matches `vk_gltf_renderer` Bistro screenshot framing by using glTF camera 0, applying the corresponding
scene-root offset, and preserving a shared EvoEngine demo camera pose. It is both a scene/import validation target and the
long-form path-tracing parity scene. Its optional DDGI setup authors a temporary `EnvironmentalLighting` asset directly
instead of creating a scene-local `DdgiVolume` component.

## 3DGS And 3DGS-Bicycle

The `3dgs` launcher profile opens a script-generated 3D Gaussian Splatting project under:

```text
Resources/EvoEngine-DemoProjects/3DGS
```

Generate it with:

```bat
python Scripts\prepare_demos.py --demo 3dgs
```

The `bicycle` launcher profile is a second 3D Gaussian Splatting project under:

```text
Resources/EvoEngine-DemoProjects/Bicycle
```

Generate it with:

```bat
python Scripts\prepare_demos.py --demo bicycle
```

The Bicycle script downloads INRIA's pretrained `models.zip`, extracts only
`bicycle/bicycle/point_cloud/iteration_30000/point_cloud.ply`, and writes it to
`Assets/GaussianSplats/bicycle.ply`. It removes the archive after extraction by default; pass `--keep-archive` only when
the local archive is intentionally needed.

Gaussian splats do not yet participate in TLAS traversal, mesh occlusion, ray-traced reflections, shadows, DDGI, or ray
queries. The Bicycle PLY is large and should remain local unless the Resources submodule workflow explicitly includes it.
