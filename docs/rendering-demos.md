# Rendering Demos

[Back to rendering overview](rendering.md)

Demo profiles provide ready-made scenes for learning renderer features and checking changes. Launch a profile from the
EvoEngine launcher or directly from the installed editor:

```powershell
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering
```

## Profiles

| Profile | Purpose | Resource model |
| --- | --- | --- |
| `rendering` | Sponza scene for raster/ray cameras, DDGI, reflection probes, lights, shadows, and material inspection. | Tracked demo project. |
| `ddgi` | Cornell-box application focused on DDGI placement, update, and gather behavior. | Generated application scene. |
| `rendering-regression` | Cross-technique material, light, skinning, temporal, and path-tracing fixtures. | Generated local project. |
| `bistro` | Large glTF scene used for import and path-tracing validation. | Generated local project using an external Bistro checkout. |
| `sss` | Male head sculpt with a shared skin diffusion profile and studio key/fill/rim lights. | Generated local project using bundled licensed assets. |
| `3dgs` | Spatial Dragon Gaussian-splat scene. | Script-generated project. |
| `bicycle` | INRIA Bicycle Gaussian-splat scene. | Script-generated project with a downloaded point cloud. |
| `ecosyslab` | EcoSysLab package scene with an animated Acacia tree and split Scene/Plant Visual editor layout. | Tracked package demo project. |
| `procedural-galaxy` | One generated 500,000-star cluster with overview/follow navigation. | Universe package and generated local project. |
| `digital-agriculture` | DigitalAgriculture starter with camera, light and soil. | Tracked package demo; crops require authoring. |
| `lsystem` | LSystem package starter. | Tracked starter; does not initially demonstrate a generated plant. |

Rasterization, ray tracing, and ray query share scene and material intent but are not expected to be pixel-identical.
Their integrators, sampling, capability paths, and temporal histories differ.

## Preparing Generated Resources

Use the preparation script for Rendering, Rendering Regression, Bistro, 3DGS, and Bicycle resources:

```powershell
python Scripts\prepare_demos.py --editor out\install\vs2026-x64\bin\EvoEngineEditor.exe
```

Pass `--demo <id>` to select one supported profile, `--validate` to perform read-only validation, `--prepare` to create
missing resources, and `--no-previews` to skip launcher preview generation.

Large or downloaded projects live under `Resources/.generated` or another ignored local path. They are validation input,
not tracked repository content. The script verifies required sources and falls back from symbolic links to copies on
Windows when necessary.

Launcher thumbnails live under `Resources/Launcher/DemoPreviews`. A preview should represent the profile's ordinary
camera output rather than a synthetic substitute.

The preparation script handles only its listed five profiles; `--demo sss` and the package starters are editor profiles,
not preparation-script choices. SSS copies its bundled licensed head/profile assets into an ignored project; saved profile
edits survive restarts. See [SSS authoring](subsurface-scattering.md#human-skin-demo) and [asset attribution](../Resources/SubsurfaceScattering/README.md).

For automated authoring/captures, use a separate resource copy and `--resource-root <absolute Resources directory>`;
generated demo setup can recreate project files. Run package profiles from the installed `bin` directory because package
shader/resource paths can be working-directory-relative. Use absolute output paths. This differs from exported players,
which resolve content relative to their executable. See [runtime demo validation](runtime-builds.md#demo-validation).

## Rendering Profile

The Rendering profile loads Sponza with:

- a tracked indirect environment source and scene-global reflection probe;
- five local box-projected reflection probes covering the hallway and galleries;
- shared camera-following GI cascades, with Automatic SDFGI selected by default and Automatic DDGI available with RT;
- disabled imported Sponza punctual lights plus enabled EvoEngine-authored directional and point lights;
- the Capoeira character enabled;
- a raster editor Scene camera; the demo does not override the main camera's render mode.

The profile is useful for comparing camera techniques, inspecting probe placement, exercising raster material previews,
and capturing README or launcher images.

## Rendering Regression And Bistro

Rendering Regression is the compact deterministic fixture. It contains representative opaque, masked, transmissive,
emissive, textured, skinned, sampler, UV-transform, tangent, convergence, and bright outlier cases. Use it for focused
cross-technique capture and shader debugging.

Bistro is the large-scene integration fixture. It stresses glTF import, material and texture counts, acceleration
structures, path tracing, shadows, and optional DDGI at a scale closer to a production scene.
Its editor Scene camera is set to rasterization, while the main camera keeps its default or saved render mode.

Validation commands for these profiles are collected in [Rendering validation](rendering-validation.md).
