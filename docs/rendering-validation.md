# Rendering Validation

This page describes the current local validation flow for rasterization, ray tracing, ray query, and cascaded directional
shadows. It intentionally records commands and acceptance checks, not development history.

## Format, Build, and Test

Run the formatter check before building:

```powershell
python Scripts\format_cpp.py --check --root EvoEngine_SDK --root EvoEngine_App --root EvoEngine_Tests --root PythonBinding
```

Build the affected targets with the normal Windows preset:

```powershell
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target EvoEngine_Tests/EvoEngine_Tests --parallel 4
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target EvoEngine_App/EvoEngineEditor --parallel 4
```

Run the focused CPU and source-contract coverage:

```powershell
out\build\vs2026-x64\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter="CameraRenderTechnique.*:GltfRayTracingMaterial.*:RayCameraShaderVariantCache.*:RayCameraHistory.*:DirectionalShadowCascadeFit.*:RenderGraph.DirectionalShadowCasterPathsUseProductionRenderers:RenderGraph.RenderPassDrawCountersRouteRasterAccountingByPass"
```

On a Vulkan device that supports the required features, also run the shader and numerical probes:

```powershell
out\build\vs2026-x64\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter="GpuService.CameraRayTransportShadersCompile:GpuService.GltfRayTracingNumericalProbeMatchesAnalyticValues:GpuService.DirectionalShadowComparisonSamplerFiltersDepthStep"
```

Install the applications after the focused tests pass:

```powershell
python Scripts\install_apps.py --config RelWithDebInfo --no-open --incremental
```

The executable used for manual validation is:

```text
out\install\vs2026-x64\bin\EvoEngineEditor.exe
```

## Ray-Camera Captures

The `rendering-regression` profile contains representative material, emissive, punctual-light, skinned, temporal, and
high-contrast probes. Capture both ray-camera techniques at the same resolution and sample count:

```powershell
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\render-validation\raytracing.hdr --preview-render-mode raytracing --preview-warmup-frames 64 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\render-validation\rayquery.hdr --preview-render-mode rayquery --preview-warmup-frames 64 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
```

Use `.png` when validating the presented image and `.hdr` when comparing linear radiance. The capture path waits for the
requested ray-camera shader variant before accumulating the requested warmup frames. Unsupported techniques fall back
according to the runtime capability policy and report that choice in the log.

An individual shared debug view can be captured with `--preview-ray-debug <name>`. Useful names include `material-id`,
`base-color`, `geometric-normal`, `shading-normal`, `roughness`, `metallic`, `emission`, `direct-punctual`,
`direct-environment`, `direct-emissive`, `indirect-radiance`, `path-depth`, `bsdf-pdf`, `light-pdf`, and `emissive-pdf`.
RayTracing and RayQuery captures both use the ordinary path tracer. Use the same resolution, warmup frame count, sample
size, deterministic settings, scene state, and shader cache when comparing the two backends. Optional ray outputs can be
requested with `--preview-ray-outputs` using `albedo`, `normal`, `ray-count`, `path-length`, `time`, or `debug`; validate
the emitted sidecar and each requested image for dimensions, finiteness, and expected data type. `all` enables these six
generic outputs.

## Image Comparison

Verify the image reader before using it for an acceptance comparison:

```powershell
python Scripts\compare_reference_render.py --self-test
```

Compare two PNG or HDR captures with:

```powershell
python Scripts\compare_reference_render.py reference.hdr candidate.hdr --out out\render-validation\comparison.json
```

Add `--require-exact` only when byte-equivalent decoded pixels are the intended contract. Most stochastic ray-camera
comparisons should instead review the reported error metrics and the images themselves.

## DDGI Reference Images And Runtime Validation

The frozen DDGI baseline release, manifest history, and baseline validator are no longer part of the checked-in
validation surface. Stable ray-camera reference captures that are still useful for manual comparison live under
`EvoEngine_Tests/Rendering/DDGI/References/`.

Use `Scripts\compare_reference_render.py` for ad-hoc reference comparisons when a fresh capture needs a numeric image
delta. The retained images are reference material only; they do not imply an authoritative manifest, committed evidence
bundle, or automatic pass/fail policy.

DDGI runtime coverage comes from focused installed-editor validation scripts. The multi-volume process validates all
eight DDGI runtime slots, deterministic overlap selection, isolated scrolling, removal, and persistent-resource identity
at 1920x1080:

```powershell
python Scripts\run_ddgi_multivolume_validation.py --config RelWithDebInfo --width 1920 --height 1080
```

The emissive A/B process runs deterministic `enabled`, `disabled`, and `enabled-repeat` phases, records 120
`DDGI Probe Trace` samples per phase, and checks effective controls, shared-inventory stability, non-fixed candidate-ray
accounting, source invalidation, image separation, enabled-repeat stability, and internally consistent timing/variability
summaries:

```powershell
python Scripts\run_ddgi_emissive_validation.py --config RelWithDebInfo --width 1920 --height 1080 --measure-frames 120 --output-dir out\ddgi-validation-m8-emissive
```

Sponza runs from the generated rendering-regression project populated from the raw Rendering assets, so validation never
writes the persisted Rendering demo project. Every fixture deletes all non-camera scene entities, reconstructs canonical
geometry, and overwrites inherited environment and camera state, so each process is independent of persisted demo-project
state.

The environment-lighting process captures `background-only`,
`diffuse-only`, `specular-only`, `both`, and `all-off` at 1920x1080 around exact-metal and dielectric regions. The same
process proves that a specular-only edit records no DDGI invalidation or probe work and that a diffuse edit records a
scene-input refresh with probe work. SSR is disabled and ray-traced reflections are absent:

```powershell
python Scripts\run_environment_lighting_validation.py --config RelWithDebInfo --output-dir out\environment-lighting-validation-m11
```

The spatial-reflection-probe launch performs two canonical 256x256 RGBA16F bakes and
captures `baseline`, `box-unprojected`, `owner-order-reversed`, `camera-moved`, `missing-asset`, and `removed` at
1920x1080. The gate checks adjacent colored receiver regions, a nested priority tie, a lower-priority boundary blend,
sphere selection, rotated box parallax, roughness-driven mip selection, exact-metal energy, collection-order independence,
surface-position selection while the camera remains outside every influence volume, and identical global fallback for an
assigned-but-empty versus removed probe. The two bakes differ only in placed local-probe inputs and must have identical
payload hashes, proving that capture is non-recursive:

```powershell
python Scripts\run_reflection_probe_validation.py --config RelWithDebInfo --output-dir out\reflection-probe-validation-m12
```

The report separately records logical GPU image bytes, steady CPU payload bytes, and serialized payload bytes. Packed
memory reports only a candidate logical image size because that representation is neither retained nor serialized. It
evaluates packed B10G11R11 reconstruction and exact Vulkan image-usage capability, but records an explicit no-adopt
result when the packed path is not adopted.

The current reflection specialty launch and environment specialty launch cover local-probe and environment-lighting
regressions. The reflection report includes bake, selection, fallback, packed-format, and 120-frame timing evidence, then
adds isolated
unoccluded-specular, scalar-visibility, and occluded-specular captures for material AO, GTAO, their bounded combination,
DDGI visibility, disabled/unavailable AO, SSAO, missing DDGI coverage, roughness, grazing response, probe boundaries,
camera motion, and `I` invariance. The environment specialty preserves the complete M14 `S/I` raster/ray matrix and adds
the persistent Sponza probe set, hallway and adjacent-gallery views, GTAO/SSAO separation, local-probe-intensity diffuse
invariance, DDGI disabled/outside specular invariance, and rough-probe DDGI visibility occlusion:

```powershell
python Scripts\run_reflection_probe_validation.py --config RelWithDebInfo --output-dir out\reflection-probe-validation-m15
python Scripts\run_environment_lighting_validation.py --config RelWithDebInfo --output-dir out\environment-lighting-validation-m15
```

Probe-update variant comparisons use separate output directories and never use `--resume` across variants. M4 spends one
Sponza launch with `EVOENGINE_DDGI_PROBE_UPDATE_VARIANT=parallel-direct` and one with
`EVOENGINE_DDGI_PROBE_UPDATE_VARIANT=parallel-shared`. Each candidate and Bistro log must contain
`EVOENGINE_VULKAN_SYNCHRONIZATION_VALIDATION enabled`, the exact requested/selected variant marker, and the executed-path
marker. These explicit log checks prevent evidence from being assigned to the wrong implementation.

The installed-runtime digest covers immutable installed binaries, libraries, shaders, and resources. It excludes
`bin/ShaderBinaries`, `bin/PipelineCache`, and `bin/imgui.ini`, which are runtime-generated mutable cache/UI state; a
successful Bistro smoke therefore cannot invalidate the immutable runtime closure.

All 17 deterministic fixtures remain available for explicit diagnostics, including furnace HDR, alpha-tested geometry,
UV0-UV3, emitter size, sidedness, moving-rigid, direct-hit, and empty-inventory cases. Omitted matrix semantics close
through focused shader, host, and numerical tests. Larger ad-hoc captures must opt in with `--deep-suite`.

The suite retains the engine's `0.2` variability threshold for neutral Cornell, Sponza, furnace, and empty-emitter scenes.
Sparse high-contrast alpha, scrolling, and emissive scenes use an explicit `0.5` threshold with the same 16-sample minimum;
both values are validation-script contracts rather than runtime defaults. `--resume` reuses captures only after the
script-specific source, runtime, resource, fixture, hardware, report, image, and log checks pass.

## Acceptance Checklist

- The editor exits normally without Vulkan validation errors, device loss, fatal logs, or shader compilation failures.
- RTX and RayQuery show the same scene, camera, material interpretation, lighting controls, and selected debug view.
- Saved HDR values are finite and preserve expected highlights; presented PNG output has no clipping or row inversion.
- Material boundaries, normal maps, alpha masking, transmission, emissive lighting, and punctual shadows are intact.
- Camera or scene changes reset accumulation, while an unchanged view continues accumulating without flicker.
- CSM captures satisfy the separate checks in [csm_validation.md](csm_validation.md).

Always record the exact installed executable, command line, GPU, and inspected output paths with the milestone evidence.
