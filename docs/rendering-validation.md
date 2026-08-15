# Rendering Validation

[Back to rendering overview](rendering.md)

This contributor runbook covers local rendering checks. GitHub Actions compile and format the repository, while visual
and GPU validation remains local because it requires a Vulkan-capable device and produces artifacts for inspection.

## Format, Build, And Test

Use LLVM clang-format 19.1.7, matching CI, then build the tests and editor. Pass its executable explicitly when another
version is first on `PATH`:

```powershell
python Scripts\format_cpp.py --clang-format C:\path\to\clang-format.exe --check --root EvoEngine_SDK --root EvoEngine_App --root EvoEngine_Tests --root PythonBinding
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target EvoEngine_Tests --parallel 4
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target EvoEngineEditor --parallel 4
```

Run the focused suite appropriate to the changed subsystem. A broad rendering contract pass is:

```powershell
out\build\vs2026-x64\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter="CameraRenderTechnique.*:GltfRasterMaterial.*:GltfRayTracingMaterial.*:EnvironmentalLightingContract.*:DdgiVolume.*:DirectionalShadowCascadeFit.*"
```

GPU shader and numerical tests require a device with their Vulkan prerequisites. Report an unavailable prerequisite as
missing coverage rather than treating a CPU/source test as equivalent.

Install applications before process or visual testing:

```powershell
cmake --install out\build\vs2026-x64 --config RelWithDebInfo
```

Use `out\install\vs2026-x64\bin\EvoEngineEditor.exe` for installed-runtime checks.

## Camera Captures

Use the same scene, resolution, sample count, deterministic settings, and shader cache when comparing camera techniques:

```powershell
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\render-validation\raytracing.hdr --preview-render-mode raytracing --preview-warmup-frames 64 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\render-validation\rayquery.hdr --preview-render-mode rayquery --preview-warmup-frames 64 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
```

Use PNG for presented output and HDR for linear-radiance comparison. Confirm the log reports the requested technique;
unsupported ray modes may otherwise produce a valid raster fallback image.

Ray debug views use `--preview-ray-debug <name>`. Optional outputs use `--preview-ray-outputs` with `albedo`, `normal`,
`ray-count`, `path-length`, `time`, `debug`, or `all`. Validate every emitted image and sidecar for expected dimensions,
type, and finite values.

### Linear Swept Sphere Strands

```powershell
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\render-validation\strand-raytracing.hdr --preview-strand-fixture --preview-render-mode raytracing --preview-warmup-frames 64 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\render-validation\strand-rayquery.hdr --preview-strand-fixture --preview-render-mode rayquery --preview-warmup-frames 64 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
```

On a capable NVIDIA device, require the startup log to report linear swept-sphere support and inspect framing,
silhouette, taper, materials, transforms, and shadows in both images. On an unsupported device, both cameras must render
without the strands and without Vulkan feature, descriptor, or acceleration-structure errors.

## DDGI And Reflection Probes

The installed-editor validation scripts write their captures, logs, and reports below `out/`:

| Area | Command |
| --- | --- |
| DDGI application smoke | `python Scripts\run_ddgi_app_validation.py --config RelWithDebInfo` |
| Multi-volume selection and lifecycle | `python Scripts\run_ddgi_multivolume_validation.py --config RelWithDebInfo --width 1920 --height 1080` |
| Emissive sampling | `python Scripts\run_ddgi_emissive_validation.py --config RelWithDebInfo --width 1920 --height 1080` |
| Localized convergence | `python Scripts\run_ddgi_localized_convergence_validation.py --config RelWithDebInfo` |
| Environment controls | `python Scripts\run_environment_lighting_validation.py --config RelWithDebInfo --output-dir out\environment-lighting-validation` |
| Reflection probes | `python Scripts\run_reflection_probe_validation.py --config RelWithDebInfo --output-dir out\reflection-probe-validation` |

Inspect DDGI reports for readiness, convergence, update reasons, finite atlas metadata, correct overlap selection, and
expected response to light, material, geometry, and environment changes. Reflection-probe checks should cover explicit
bake publication, priority and boundary selection, global fallback, box/sphere projection, dynamic updates, and the
absence of recursive local-probe capture.

## Cascaded Directional Shadows

Directional shadows use four cascades. Stable Sphere is the default fit and should remain stable under small camera
motion. Tight Light-Space AABB is an unsnapped comparison mode.

Capture both modes with otherwise identical settings:

```powershell
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo bistro --editor --capture-demo-preview out\render-validation\csm-stable.png --preview-render-mode rasterization --preview-shadow-fit stable-sphere --preview-warmup-frames 32 --preview-width 1280 --preview-height 720 --preview-deterministic
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo bistro --editor --capture-demo-preview out\render-validation\csm-tight.png --preview-render-mode rasterization --preview-shadow-fit tight-aabb --preview-warmup-frames 32 --preview-width 1280 --preview-height 720 --preview-deterministic
```

Use `--preview-shadow-debug` with `cascade-index`, `light-uv`, `light-depth`, `atlas-uv`, or `texel-density` to isolate
cascade selection and sampling. Optional capture overrides can change resolution, PCF samples, split lambda, transition
width, and distance fade without changing runtime defaults.

Verify all four cascades receive shadows, transitions have no gaps or double-darkening, stable fitting does not shimmer,
off-camera casters remain represented, and bias/filtering stay consistent across resolution and fit changes.

## Image Comparison

```powershell
python Scripts\compare_reference_render.py --self-test
python Scripts\compare_reference_render.py reference.hdr candidate.hdr --out out\render-validation\comparison.json
```

Use exact comparison only for deterministic byte-equivalent contracts. Stochastic camera captures require metric review
plus visual inspection.

## Acceptance Checklist

- The requested renderer and optional features are active rather than silently falling back.
- The process exits normally without Vulkan validation errors, device loss, fatal logs, or shader failures.
- Output dimensions and formats are correct; HDR values are finite and preserve expected highlights.
- Materials, normal maps, alpha masking, transmission, emission, direct lights, and shadows remain intact.
- Camera and scene changes invalidate the appropriate temporal history; unchanged views continue accumulating.
- Rasterization, ray tracing, and ray query preserve the same scene and authored material meaning.
- Validation notes record the installed executable, command line, GPU, and inspected artifact paths.
