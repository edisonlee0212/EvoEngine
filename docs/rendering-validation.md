# Rendering Validation

[Back to rendering overview](rendering.md)

This contributor runbook covers local rendering checks. GitHub Actions compile and format the repository, while visual
and GPU validation remains local because it requires a Vulkan-capable device and produces artifacts for inspection.

## Format, Build, And Test

Use LLVM clang-format 19.1.7, matching CI, then build the tests and editor. Pass its executable explicitly when another
version is first on `PATH`:

```powershell
python Scripts\format_cpp.py --clang-format C:\path\to\clang-format.exe --check --root EvoEngine_SDK --root EvoEngine_App --root EvoEngine_Tests --root PythonBinding
cmake --build out\build\vs2026-x64\EvoEngine_Tests --config RelWithDebInfo --target EvoEngine_Tests --parallel 4
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target EvoEngineEditor --parallel 4
```

Run the focused suite appropriate to the changed subsystem. A broad rendering contract pass is:

```powershell
out\build\vs2026-x64\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter="CameraRenderTechnique.*:GltfRasterMaterial.*:GltfRayTracingMaterial.*:EnvironmentalLightingContract.*:DdgiVolume.*:DirectionalShadowCascadeFit.*"
```

GPU shader and numerical tests require a device with their Vulkan prerequisites. Report an unavailable prerequisite as
missing coverage rather than treating a CPU/source test as equivalent.

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

Use an installed RelWithDebInfo editor, a fixed camera, and the same shader cache for every comparison. The raster
profile report records CPU frame median/p95 and every named GPU timestamp median/p95 after scene and shader readiness:

```powershell
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo bistro --editor --capture-demo-preview out\raster-visibility\bistro.png --preview-render-mode rasterization --preview-raster-profile-report out\raster-visibility\bistro.json --preview-warmup-frames 240 --preview-width 1920 --preview-height 1080 --preview-deterministic
```

Raster profile schema 4 uses a fixed-window frame-budget model. Requesting
`--preview-raster-profile-report` automatically enables CPU and GPU profiler capture only for the first requested
`--preview-warmup-frames` window; interactive profiler capture remains disabled by default. The report labels this
policy as `first-requested-frames` and includes both the full window and a final, at-most-240-frame interpretation.
It records the stable Application Loop hierarchy with inclusive/self average, median, p95, maximum, observation counts,
missing-frame zeros, main-thread wall time, and separate worker CPU work. GPU data is grouped by render-pass taxonomy
with span, summed contributing work, pass duty cycle, and frequency-weighted time. Raster resolution and DDGI
active/update/trace/convergence sample counts make transient convergence activity explicit; the capture does not wait
for DDGI steady state.
For instrumentation-overhead A/B runs, append `--preview-gpu-timestamps disabled`; profile reports otherwise enable GPU
timestamps automatically. The report records the effective `capture.gpu_timestamps` state.
The report intentionally contains only stable measurements: capture metadata, the CPU profiler hierarchy, GPU history
and timestamps, render-pass draw accounting, DDGI activity, and a fixed `render_path` description. It does not expose
implementation counters or temporary timing probes. Camera and shadow visibility use the optimized spatially indexed,
parallel compact-command path; this path is not configurable from the capture CLI.

Camera indexed commands and mesh-task commands use two persistent arenas. Every camera segment starts at
`minStorageBufferOffsetAlignment`; deferred draws add its byte offset, so shader command indices remain local.
Directional-cascade, point-face, and spot-view
commands share one persistent shadow arena for the active indexed or mesh-task path. This removes per-view buffer
allocation and synchronous upload waits while preserving stable camera/view order and dynamic reflection-probe cameras.
Run the command at least three times and retain the median run. Record the executable path, commit, GPU and driver from
the JSON, camera overrides, resolution, enabled renderer settings, image hash, CPU frame median/p95, and GPU timestamp
median/p95. The `Deferred Geometry`, `Depth Pyramid`, `Directional Shadow`, `Point Shadow`, `Spot Shadow`, and
`Transparent Geometry` scopes isolate the raster work changed by visibility optimizations. Use the editor Rendering
Diagnostics panel to record direct/indirect draw calls, indirect commands, and primitive counts for the same view.

For image correctness, pair the Bistro measurement with the rendering-regression and strand fixtures described below.
Compare captures against a known-good image from the same fixed render path.

The depth hierarchy is a separate R32F image derived from the regular camera depth after deferred geometry and motion
coverage. Mip 0 is an exact copy
of the camera's D32 depth and every coarser mip stores the maximum depth over its complete source footprint, including
odd image edges. Under the conventional depth convention (clear 1, nearer values smaller), that maximum reduction is
required to prove that every covered sample is in front of an occlusion candidate. Per-mip compute barriers make each
level available to the next dispatch. AO and DDGI continue to sample exact camera depth rather than the coarse hierarchy.
Reflection-probe face cameras do not generate the hierarchy. The former depth prepass and same-frame object Hi-Z
occlusion pass have been removed.

Camera, shadow, meshlet-frustum, and eligible rigid normal-cone culling are permanent parts of the optimized raster
path. Invalid bounds and unsupported deformation or transform cases remain conservatively visible.

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

The matched Bistro validation uses three installed-editor runs at 1920x1080 with 240 measured frames. The original
baseline recorded about 11 immediate submissions per frame, a 6.07-6.73 ms upload median, and a 16.16-17.19 ms CPU
median. Immediate batching reduced that to about one submission per frame, a 0.39-0.60 ms upload median, and a
12.73-13.20 ms CPU median. Frame-stream recording then left only one non-render-instance immediate submission in each
entire 240-frame capture; final upload medians were 0.43-0.44 ms, CPU medians 12.92-13.09 ms, and GPU medians
13.09-13.27 ms. The recycled-frame fence absorbs the remaining GPU wait, so further CPU upload work is lower priority
than the current GPU workload.

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
