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

## Acceptance Checklist

- The editor exits normally without Vulkan validation errors, device loss, fatal logs, or shader compilation failures.
- RTX and RayQuery show the same scene, camera, material interpretation, lighting controls, and selected debug view.
- Saved HDR values are finite and preserve expected highlights; presented PNG output has no clipping or row inversion.
- Material boundaries, normal maps, alpha masking, transmission, emissive lighting, and punctual shadows are intact.
- Camera or scene changes reset accumulation, while an unchanged view continues accumulating without flicker.
- CSM captures satisfy the separate checks in [csm_validation.md](csm_validation.md).

Always record the exact installed executable, command line, GPU, and inspected output paths with the PR validation notes.
