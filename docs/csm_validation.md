# Cascaded Shadow Map Validation

This page describes the current CSM contract and its local validation flow.

## Runtime Contract

- Directional shadows use four cascades selected from positive linear view depth.
- `Stable Sphere` is the default fit. It encloses each frustum slice with a quantized sphere and snaps the light-space
  projection to shadow texels for camera-motion stability.
- `Tight Light-Space AABB` tightly fits each slice without stabilization and is available for comparison.
- The fit mode is a global, non-serialized RenderLayer inspection setting.
- The default shadow-map quality is `High`, which allocates 4096 by 4096 directional, point, and spot shadow maps.
- Directional shadows use 16 Vogel-disk PCF samples by default. Point and spot shadow sample counts remain independent.
- Cascade split lambda, transition width, distance fade, bias, normal offset, and PCF controls retain their runtime
  defaults unless a test explicitly overrides them.

## Format, Build, and Test

```powershell
python Scripts\format_cpp.py --check --root EvoEngine_SDK --root EvoEngine_App --root EvoEngine_Tests
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target EvoEngine_Tests/EvoEngine_Tests --parallel 4
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target EvoEngine_App/EvoEngineEditor --parallel 4
out\build\vs2026-x64\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter="DirectionalShadowCascadeFit.*:CameraRenderTechnique.DirectionalShadow*:CameraRenderTechnique.ZeroToOneDepthHelpersUseProjectionTranslation:RenderGraph.DirectionalShadowCasterPathsUseProductionRenderers:GpuService.DirectionalShadowComparisonSamplerFiltersDepthStep"
python Scripts\install_apps.py --config RelWithDebInfo --no-open --incremental
```

The GPU comparison probe requires a Vulkan device with the test shader prerequisites. If it is unavailable, report that
separately rather than treating the CPU cascade-fit tests as equivalent coverage.

## Fit-Mode Comparison

Use the installed editor and keep every capture option except the fit mode identical:

```powershell
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo bistro --editor --capture-demo-preview out\csm-validation\stable-sphere.png --preview-render-mode rasterization --preview-shadow-fit stable-sphere --preview-warmup-frames 32 --preview-width 1280 --preview-height 720 --preview-deterministic
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo bistro --editor --capture-demo-preview out\csm-validation\tight-aabb.png --preview-render-mode rasterization --preview-shadow-fit tight-aabb --preview-warmup-frames 32 --preview-width 1280 --preview-height 720 --preview-deterministic
```

Optional capture overrides are:

- `--shadow-map-resolution low|medium|high|very-high`
- `--preview-shadow-pcf-samples 1..64`
- `--preview-shadow-split-lambda 0..1`
- `--preview-shadow-cascade-transition-width <non-negative width>`
- `--preview-shadow-distance-fade <non-negative width>`

These are capture controls only; do not use them to redefine runtime defaults.

## Diagnostics

Capture one diagnostic at a time with `--preview-shadow-debug`:

- `cascade-index` shows the selected cascade and transition regions.
- `light-uv` shows the selected cascade's local shadow-map coordinates.
- `light-depth` shows the receiver depth used for comparison.
- `atlas-uv` shows packed directional-light atlas coordinates.
- `texel-density` exposes changes in projected shadow texel density.

Use `--preview-shadow-debug-cascade 0..3` to select a cascade and `--preview-shadow-debug-light <index>` to select a
directional light where the diagnostic supports it.

## Acceptance Checklist

- All four cascade regions receive shadows in both fit modes; no far cascade silently samples the wrong atlas tile.
- Cascade transitions blend without a visible hard band, double-darkening, or a missing-shadow gap.
- Stable Sphere does not shimmer under small camera translations or rotations.
- Tight AABB tracks the frustum slice closely and remains finite for degenerate bounds or empty visible geometry.
- Off-camera casters along the light direction remain represented when they can shadow the visible slice.
- Directional bias and PCF stay in shadow-texel units across resolution and fit-mode changes.
- Regular, mesh-shader, instanced, skinned, strand, and external directional casters continue through their production
  render paths where those renderers are present and supported.
- The editor exits without Vulkan validation errors, shader failures, device loss, or fatal logs.

Record the exact installed executable, command lines, GPU, and visually inspected image paths with the PR validation
notes.
