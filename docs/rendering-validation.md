# Rendering Validation

[Back to rendering overview](rendering.md)

This contributor runbook covers local rendering checks. GitHub Actions compile and format the repository, while visual
and GPU validation remains local because it requires a Vulkan-capable device and produces artifacts for inspection.

## Format, Build, And Test

Install the exact clang-format release recorded in `.clang-format-version`, then build the tests and editor. The format
script selects the pinned Python package and rejects a mismatched executable:

```powershell
python -m pip install clang-format==22.1.8
python Scripts\format_cpp.py --check
python Scripts\build_project.py
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target EvoEngine_Tests --parallel 4
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target EvoEngineEditor --parallel 4
```

Run the focused suite appropriate to the changed subsystem. A broad rendering contract pass is:

```powershell
out\build\vs2026-x64\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter="CameraRenderTechnique.*:GltfRasterMaterial.*:GltfRayTracingMaterial.*:EnvironmentalLightingContract.*:DdgiVolume.*:DirectionalShadowCascadeFit.*"
```

GPU shader and numerical tests require a device with their Vulkan prerequisites. Report an unavailable prerequisite as
missing coverage rather than treating a CPU/source test as equivalent.

Inspect validation output as well as test exit status: a Vulkan callback error does not automatically fail a GoogleTest
assertion. The geometry compaction readback test requires the vertex buffer to have `TRANSFER_SRC` usage, in addition to
its upload, storage and vertex usages; check that `VUID-vkCmdCopyBuffer-srcBuffer-00118` is absent.

Vulkan validation is disabled by default, including Debug and RelWithDebInfo builds. Enable it explicitly when running
graphics API correctness checks:

```powershell
cmake -S . -B out\build\vs2026-x64 -DEVOENGINE_ENABLE_GRAPHICS_VALIDATION=ON
```

The option only changes Debug and RelWithDebInfo configurations; Release builds remain validation-free.

Install applications before process or visual testing:

```powershell
python Scripts\install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --jobs 8
```

Use `out\install\vs2026-x64\bin\EvoEngineEditor.exe` for installed-runtime checks.

## Raster Performance Baseline

The capture script `EvoEngine_Tests/Rendering/render_demo_capture.py` accepts `--scene bistro` with
`--view default` (exterior) or `--view doorway` (looking into the bar). Prepare Bistro assets first.
`RenderingDemo.BistroDefaultGoldenImage` and `RenderingDemo.BistroDoorwayGoldenImage` capture 1280x720 raster
views with DDGI occlusion after 600 warmup frames; they skip when Bistro is unavailable. Each has its own baseline.
The script also accepts Sponza (the default scene), GI settings, and all three camera render modes.

Capture fixtures wait for scene setup before selecting GI. `ConfigureIndirectLightingDebugForCapture(view, pause_ddgi_updates)`
selects beauty (0), diffuse (1), unoccluded specular (2), specular visibility (3), occluded specular (4), or DDGI blend loss (5).

`RenderingDemo.SdfgiGoldenImage` selects Automatic SDFGI explicitly with RT pipeline, ray query and acceleration
structures disabled. It captures the same 2560x1440 scene after 1800 warmup frames, checks that SDFGI is effective and
has updated its field, and compares against its own `RenderingDemo.SdfgiGoldenImage.2560x1440.png` baseline.
The DDGI texture-lifecycle and SDFGI golden tests retain PSNR >= 30 dB and SSIM >= 0.95. After inspecting an intentional
baseline change, regenerate these targets with the following command, then rerun without acceptance:

```powershell
python Scripts/test.py --render-only --accept-render-baseline --ctest-arg=-R --ctest-arg="RenderingDemo.(TextureLifecycleStressThenCanonicalRasterGolden|SdfgiGoldenImage)$"
```

For SDFGI alone, use `^RenderingDemo.SdfgiGoldenImage$`. The accepted image uses fixed-grid probes with occlusion.
The pillar/ground seam also occurs with HDDAGI and remains unresolved; baseline acceptance does not fix it.
SDFGI captures pin seed zero and save `.before.yaml` / `.after.yaml` snapshots and `.json` status.
`--sdfgi-debug-views` exports frozen diffuse, specular and coverage views after beauty.
Compare sampling/update state across failures; equal seeds do not establish repeatability.

The raster gate compares all meshlet/indirect combinations against one 2560x1440 golden (PSNR >= 29 dB, SSIM >= 0.94):

| Cell | Meshlet | Indirect |
| --- | --- | --- |
| 1 | enabled | enabled |
| 2 | enabled | disabled |
| 3 | disabled | enabled |
| 4 | disabled | disabled |

```powershell
python Scripts\test.py --render-only --ctest-arg=-R --ctest-arg=RenderingDemo.RasterPathMatrixGoldenImage
```

Meshlet cells require mesh shaders. Refresh from the meshlet-enabled, indirect-enabled cell:

```powershell
python Scripts\test.py --render-only --accept-render-baseline --ctest-arg=-R --ctest-arg=RenderingDemo.RasterPathMatrixGoldenImage
```

Only that cell writes the baseline. Retain all four PNGs when diagnosing failures.

Use an installed RelWithDebInfo editor, a fixed camera, and the same shader cache for every comparison. The raster
profile report records CPU frame median/p95 and every named GPU timestamp median/p95 after scene and shader readiness:

```powershell
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo bistro --editor --capture-demo-preview out\raster-visibility\bistro.png --preview-render-mode rasterization --preview-raster-profile-report out\raster-visibility\bistro.json --preview-warmup-frames 240 --preview-width 1920 --preview-height 1080 --preview-deterministic
```

Raster profile schema 4 captures the first requested frame window, plus a final interpretation of at most 240 frames.
It enables CPU/GPU profiling automatically and records hierarchy statistics, worker work, GPU span/summed work, pass
frequency, draw counts, DDGI activity and `render_path`. It does not wait for DDGI steady state. Use
`--preview-gpu-timestamps disabled` for instrumentation A/B checks. See [profiler timing semantics](profiler.md).

Run the command at least three times and retain the median run. Record the executable path, commit, GPU and driver from
the JSON, camera overrides, resolution, enabled renderer settings, image hash, CPU frame median/p95, and GPU timestamp
median/p95. The `Deferred Geometry`, `Depth Pyramid`, `Directional Shadow`, `Point Shadow`, `Spot Shadow`, and
`Transparent Geometry` scopes isolate the raster work changed by visibility optimizations. Use the editor Rendering
Diagnostics panel to record direct/indirect draw calls, indirect commands, and primitive counts for the same view.

For image correctness, pair the Bistro measurement with the rendering-regression and strand fixtures described below.
Compare captures against a known-good image from the same fixed render path.

Depth-hierarchy numerical tests check odd edges and maximum-depth reduction; visibility tests cover conservative
camera/shadow culling and the persistent command arenas. Run those focused contracts when changing these paths.

## Camera Captures

Use the same scene, resolution, sample count, deterministic settings, and shader cache when comparing camera techniques:

```powershell
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\render-validation\raytracing.hdr --preview-render-mode raytracing --preview-warmup-frames 64 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\render-validation\rayquery.hdr --preview-render-mode rayquery --preview-warmup-frames 64 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
```

Use PNG for presented output and HDR for linear-radiance comparison. Confirm the log reports the requested technique;
unsupported ray modes may otherwise produce a valid raster fallback image.

Use disposable demo resources for capture/authoring runs. Package profiles can require the installed `bin` as working
directory; pass absolute resource/output paths. See [demo preparation](rendering-demos.md#preparing-generated-resources).

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
| Automatic GI cascades and provider edits | `python Scripts/run_automatic_gi_validation.py --provider ddgi --resources Resources/.generated/demos` (installed RT-enabled 1440p Sponza); use `--provider sdfgi` for RT-disabled SDFGI |
| Emissive sampling | `python Scripts\run_ddgi_emissive_validation.py --config RelWithDebInfo --width 1920 --height 1080` |
| Environment controls | `python Scripts\run_environment_lighting_validation.py --config RelWithDebInfo --output-dir out\environment-lighting-validation` |
| Reflection probes | `python Scripts\run_reflection_probe_validation.py --config RelWithDebInfo --output-dir out\reflection-probe-validation` |

DDGI report schema 9 records periodic rolling-history windows instead of convergence/variability or hysteresis boosts.
Existing image baselines need fresh temporal-policy baselines. Python `CaptureCurrentScene` rejects empty or nonfinite
float output before PNG conversion.

Inspect DDGI reports for readiness, completed history windows, update reasons, finite atlas metadata, correct overlap selection, and
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
cascade selection and sampling. Directional, point, and spot shadows use eight PCF samples fixed in shader code.
Optional capture overrides can change resolution, split lambda, transition width, and distance fade without changing
runtime defaults.

Verify all four cascades receive shadows, transitions have no gaps or double-darkening, stable fitting does not shimmer,
off-camera casters remain represented, and bias/filtering stay consistent across resolution and fit changes.

## Render-Instance Uploads

`BufferUploadBatch` is the reusable SDK upload-plan API. Added payloads borrow their CPU memory: callers must keep every
source range alive and unchanged until `SubmitImmediate` or `Record` returns. Uploads default to `RequireCapacity`;
`GrowIfNeeded` is explicit and is valid only when no recorded or submitted work references the destination. Batches
reject overlapping destination ranges and provide uniform, storage, vertex, index, and indirect consumer usages plus a
raw Vulkan stage/access override. Cross-queue consumers remain responsible for semaphore and ownership synchronization.

`RenderInstanceStorage` records its batch through a per-frame-slot `BufferUploadArena`. Arena blocks are append-only
while the slot submission is pending, stay persistently mapped, flush written ranges for non-coherent memory, retain
overflow blocks, and reset only after the existing frame fence resolves. Copies and barriers are recorded before later
main-queue consumers; there is no render-instance immediate submission or upload fence. Unchanged prepared payloads are
omitted without skipping camera or shadow visibility classification, and meshlet statistics are reset with a recorded
buffer fill.

For upload changes, repeat matched Bistro captures and inspect immediate-submit counts, upload CPU time, frame fences,
and complete CPU/GPU frame costs. A lower upload time alone does not establish a faster frame.

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
