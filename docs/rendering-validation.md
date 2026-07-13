# Rendering Validation

[Back to rendering overview](rendering.md)

Rendering validation is local-only because it requires a Vulkan-capable GPU and often produces image artifacts for manual
inspection. Hosted CI should stay focused on format/build checks unless a task explicitly changes that policy.

## Baseline Checks

For rendering documentation or lightweight render behavior changes, run the smallest checks that exercise the changed
surface:

```bat
python Scripts\format_cpp.py --check --root EvoEngine_SDK --root EvoEngine_App --root EvoEngine_Tests
python Scripts\compare_reference_render.py --self-test
python Scripts\run_raytracer_baseline.py --profile fast --technique all --dry-run
git diff --check
```

Validate and prepare generated demo resources with:

```bat
python Scripts\prepare_demos.py --editor out\install\vs2026-x64\bin\EvoEngineEditor.exe
```

Pass `--demo <id>` for a single demo, `--validate` for validation only, `--prepare` for missing-file preparation only,
`--override` to force preparation without validation, or `--no-previews` when preview images are outside the current task.

If app behavior changed, build or install the relevant executable before manual validation:

```bat
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target DemoApp
python Scripts\install_apps.py --config RelWithDebInfo --no-open --incremental
```

Installed binaries are written to:

```text
out\install\vs2026-x64\bin
```

## README Screenshot Capture

Use the unified demo preparation script to refresh the README editor image:

```bat
python Scripts\prepare_demos.py --demo rendering --override --no-previews
```

The script writes `Resources/GitHub/RenderingDemo.png` for the Rendering demo. Inspect the image after capture.

Useful DDGI inspection variants:

```bat
python Scripts\capture_readme_editor_screenshot.py --output out\visual-inspection\sponza-rendering-ddgi.png --demo-setup Rendering --width 1920 --height 1080 --warmup-frames 512
python Scripts\capture_readme_editor_screenshot.py --output out\visual-inspection\sponza-ddgi-atlas-preview.png --demo-setup Rendering --ddgi-atlas-preview --width 1920 --height 1080 --warmup-frames 512
python Scripts\capture_readme_editor_screenshot.py --output out\visual-inspection\cornell-box-ddgi.png --demo-setup CornellBox --width 1920 --height 1080 --warmup-frames 512
python Scripts\capture_readme_editor_screenshot.py --output out\visual-inspection\thin-wall-ddgi.png --demo-setup ThinWall --width 1920 --height 1080 --warmup-frames 512
```

## Demo Preview Capture

Preview captures should be run from an installed app tree so shader and resource installation are validated with the same
executable a reviewer can launch.

Rendering regression examples:

```bat
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\rendering-regression-rasterization.png --preview-render-mode rasterization --preview-warmup-frames 1800 --preview-width 1280 --preview-height 720
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\rendering-regression-raytracing.png --preview-render-mode raytracing --preview-warmup-frames 512 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\rendering-regression-rayquery.png --preview-render-mode rayquery --preview-warmup-frames 512 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
```

`--preview-render-mode` accepts `rasterization`, `raytracing`, and `rayquery`. RayQuery captures require a device with
RayQuery support. `--preview-sample-size` controls manual samples per rendered frame for ray techniques.

`--disable-ray-tracing-pipeline` is a validation mask applied before Vulkan device creation. It omits RT-pipeline support
without disabling acceleration structures or RayQuery, allowing an RT-capable development GPU to exercise the genuine
query-only initialization path. Capture JSON records resolved `acceleration_structure`, `ray_tracing_pipeline`,
`ray_query`, and `shader_execution_reordering` capabilities.

The output extension selects the capture contract. PNG stores the display result. Radiance HDR (`.hdr`) stores linear RGB
and is restricted to `raytracing` and `rayquery`; ambient occlusion, bloom, screen-space reflections, anti-aliasing, and
tone mapping are disabled for that capture so renderer comparisons do not include a presentation path. Pass
`--preview-metrics-json <path>` with `--capture-demo-preview` to write the same structured record printed after the
`RAY_CAPTURE_JSON` prefix. It includes effective SPP, wall throughput, GPU/driver identity, startup GPU timestamps, and
capture-only GPU timestamps. `startup_gpu_sections` covers initialization before the fixed capture window;
`gpu_sections` preserves the profile's exact frame/sample budget. The expected section names are `Path Trace (RTX)`,
`Path Trace (RQ)`, `TLAS Build`, `TLAS Update`, `BLAS Build` when geometry is rebuilt, and `BLAS Update` for persistent
animated geometry refits. Timestamp availability is reported explicitly because some Vulkan devices do not expose
graphics-and-compute timestamps.

Other useful preview flags:

- `--preview-firefly-clamp enabled|disabled`
- `--preview-firefly-clamp-threshold <value>`
- `--preview-auto-spp enabled|disabled`
- `--preview-auto-spp-min-samples <n>`
- `--preview-auto-spp-max-samples <n>`
- `--preview-auto-spp-threshold <value>`
- `--preview-ser disabled|automatic|enabled`
- `--preview-ao ssao|gtao|disabled`
- `--preview-aa disabled|taa|smaa`
- `--preview-aa-preset best-quality|high-quality|performance|low|medium|high|ultra`
- `--preview-aa-tgsm enabled|disabled`
- `--preview-aa-fp16 enabled|disabled`
- `--preview-aa-motion-sequence enabled|disabled` for the `rendering-regression` profile
- `--preview-debug none|taa-motion|taa-depth-confidence|taa-history-confidence|taa-no-history|smaa-edges|smaa-weights`

Auto SPP convergence mode is optional in both ray techniques; reproducible baselines disable it and use an exact frame and
sample budget. NaN/Inf radiance rejection happens before accumulation in both ray techniques. The M0 comparison keeps the
configurable firefly luminance clamp enabled at luminance `10.0` in both renderers.

Exercise the query-only device path with an installed editor:

```bat
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo bistro --editor --capture-demo-preview out\m2-rq-only\evo-rq-64spp.hdr --preview-metrics-json out\m2-rq-only\evo-rq.json --preview-render-mode rayquery --preview-warmup-frames 16 --preview-sample-size 4 --preview-auto-spp disabled --preview-firefly-clamp enabled --preview-firefly-clamp-threshold 10 --preview-ser automatic --preview-width 1280 --preview-height 720 --preview-deterministic --disable-ray-tracing-pipeline
```

The result must report acceleration structures and RayQuery enabled, the RT pipeline and SER disabled, only a
`Path Trace (RQ)` GPU section, finite nonblank HDR pixels, and no validation, device-loss, or fatal errors.

TAA presets, controls, and debug modes require `--preview-aa taa`; SMAA presets and debug modes require
`--preview-aa smaa`. Incompatible combinations are rejected. The TAA debug modes capture motion vectors,
depth confidence, accumulated history confidence, or pixels where history was rejected. Best Quality and High Quality use
FP32 by default. Performance or an explicit FP16 override uses FP16 only when the selected Vulkan device advertises
`shaderFloat16`; otherwise the FP32 fallback is selected. Preview post-processing overrides require
`--capture-demo-preview`.

Best Quality comparison example:

```bat
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering --editor --capture-demo-preview out\taa-best.png --preview-width 1280 --preview-height 720 --preview-warmup-frames 64 --preview-aa taa --preview-aa-preset best-quality
```

The deterministic temporal-motion scene is captured with:

```bat
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\taa-motion-best.png --preview-width 1280 --preview-height 720 --preview-warmup-frames 120 --preview-deterministic --preview-aa taa --preview-aa-preset best-quality --preview-aa-motion-sequence enabled
```

Ray-technique captures with `--preview-aa-motion-sequence enabled` use the requested value as an exact rendered-frame
budget because every animated transform correctly resets progressive accumulation. Their reported effective SPP therefore
describes the final reset frame, while GPU section sample counts describe the full motion sequence.

The full Rendering demo AA capture matrix is:

```bat
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering --editor --capture-demo-preview out\aa-none.png --preview-width 1280 --preview-height 720 --preview-warmup-frames 120 --preview-deterministic --preview-aa disabled
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering --editor --capture-demo-preview out\aa-taa-best.png --preview-width 1280 --preview-height 720 --preview-warmup-frames 120 --preview-deterministic --preview-aa taa --preview-aa-preset best-quality
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering --editor --capture-demo-preview out\aa-smaa-low.png --preview-width 1280 --preview-height 720 --preview-warmup-frames 120 --preview-deterministic --preview-aa smaa --preview-aa-preset low
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering --editor --capture-demo-preview out\aa-smaa-medium.png --preview-width 1280 --preview-height 720 --preview-warmup-frames 120 --preview-deterministic --preview-aa smaa --preview-aa-preset medium
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering --editor --capture-demo-preview out\aa-smaa-high.png --preview-width 1280 --preview-height 720 --preview-warmup-frames 120 --preview-deterministic --preview-aa smaa --preview-aa-preset high
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering --editor --capture-demo-preview out\aa-smaa-ultra.png --preview-width 1280 --preview-height 720 --preview-warmup-frames 120 --preview-deterministic --preview-aa smaa --preview-aa-preset ultra
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering --editor --capture-demo-preview out\aa-smaa-edges.png --preview-width 1280 --preview-height 720 --preview-warmup-frames 120 --preview-deterministic --preview-aa smaa --preview-aa-preset ultra --preview-debug smaa-edges
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering --editor --capture-demo-preview out\aa-smaa-weights.png --preview-width 1280 --preview-height 720 --preview-warmup-frames 120 --preview-deterministic --preview-aa smaa --preview-aa-preset ultra --preview-debug smaa-weights
```

## DemoApp Smoke Run

A temporary run config can exercise the Rendering demo smoke path:

```yaml
mode: smoke_test
demo_setup: Rendering
application_mode: Editor
warmup_frames: 30
frames_after_play: 30
max_load_frames: 30000
max_play_frames: 1000
exit_on_complete: true
```

Run it with:

```bat
out\build\vs2026-x64\EvoEngine_App\RelWithDebInfo\DemoApp.exe --run-config out\documentation-rendering-smoke.yaml
```

The Rendering smoke path validates the canonical DDGI volume, the top-down directional light, the yellow point light, DDGI
update reasons, disabled-light behavior, relocation/classification toggles, and a real Sponza hallway lighting readback.

## Bistro Reference Parity

Bistro path-tracing parity compares EvoEngine against `vk_gltf_renderer` with a pinned, locally patched validation build.
The normalized glTF uses `KHR_lights_punctual` directional Sun intensity `10`, matching the EvoEngine scene rather than the
raw downloaded asset's `6830` value.

| Input | Revision |
| --- | --- |
| EvoEngine baseline | `025511b21ec566f1420f0cf66fba82527e2a662c` |
| `vk_gltf_renderer` | `f72d2f3711116261a76e7b8b0f4724e167703a55` |
| `nvpro_core2` | `907fba3c5b7a9597e7e63a5388079b964bd6ddb4` |
| `zeux/niagara_bistro` static source | `a096b939aaa5857150904a38763ebd75b19e3e45` |

Create the detached reference worktree, apply the tracked headless HDR/metrics patch, and build an optimized reference with
both optional reconstruction backends disabled:

```bat
git -C C:\Users\lllll\Documents\GitHub\vk_gltf_renderer worktree add --detach C:\Users\lllll\Documents\GitHub\EvoEngine\out\reference\vk_gltf_renderer f72d2f3711116261a76e7b8b0f4724e167703a55
git -C out\reference\vk_gltf_renderer apply C:\Users\lllll\Documents\GitHub\EvoEngine\Scripts\reference_patches\vk_gltf_renderer_m0.patch
cmake -S out\reference\vk_gltf_renderer -B out\reference\vk_gltf_renderer\build-m0 -A x64 -DNvproCore2_ROOT=C:\Users\lllll\Documents\GitHub -DUSE_DLSS=OFF -DUSE_OPTIX_DENOISER=OFF
cmake --build out\reference\vk_gltf_renderer\build-m0 --config RelWithDebInfo --target vk_gltf_renderer
git -C Resources\.generated\niagara_bistro checkout --detach a096b939aaa5857150904a38763ebd75b19e3e45
```

The reference patch is validation instrumentation, not an engine dependency. The runner verifies the three revisions and
the exact patch hash. It does not use a denoiser or upscaler. It also does not use the reference Physical Sky: both
renderers use a neutral black environment. The reference command expresses that as `--envSystem 1`, a zero-intensity
`std_env.hdr`, and solid background color `0 0 0`; the EvoEngine Bistro capture uses zero color and ambient environment.
Both executables use the optimized `RelWithDebInfo` configuration. Before rendering, the runner verifies that the installed
EvoEngine editor and SDK DLL exactly match the primary build-tree outputs. The manifest hashes each executable and every DLL
below its runtime directory, plus the CMake cache, generator, compiler description, and physical-device/driver telemetry; a
run fails if the two renderers select different GPU vendor/device IDs.

The fixed profiles are:

| Profile | Resolution | Frames x samples/frame | Effective SPP |
| --- | ---: | ---: | ---: |
| Fast | 1280x720 | 16 x 4 | 64 |
| Canonical | 2560x1440 | 512 x 4 | 2048 |

The M0 baseline on the NVIDIA GeForce RTX 5070 (driver `2496774144`) is:

| Profile | Renderer/technique | Path trace GPU average | AS GPU average/count |
| --- | --- | ---: | ---: |
| Fast | `vk_gltf_renderer` RTX | 27.089 ms | reference-managed |
| Fast | EvoEngine RTX | 41.238 ms | TLAS 0.175 ms x 16 |
| Fast | `vk_gltf_renderer` RayQuery | 43.305 ms | reference-managed |
| Fast | EvoEngine RayQuery | 55.930 ms | TLAS 0.172 ms x 16 |
| Canonical | `vk_gltf_renderer` RTX | 105.252 ms | reference-managed |
| Canonical | EvoEngine RTX | 158.787 ms | TLAS 0.176 ms x 512 |
| Canonical | `vk_gltf_renderer` RayQuery | 167.926 ms | reference-managed |
| Canonical | EvoEngine RayQuery | 279.394 ms | TLAS 0.175 ms x 512 |

The canonical linear-HDR reference-versus-EvoEngine MAE/RMS is `0.004413/0.018645` for RTX and
`0.004410/0.018658` for RayQuery. EvoEngine RTX-versus-RayQuery is `0.001003/0.002483`. These values are an M0
tracking baseline, not a final material-parity gate.

The separate 1280x720 rendering-regression motion probe renders 16 reset frames at four samples per frame. It measures two
animated mesh parts: 32 `BLAS Build` samples at `0.311 ms` average (`9.97 ms` total), TLAS at `0.127 ms` per frame,
path tracing at `3.784 ms`, and `0.677 s` wall time. M1a targets no static steady-state TLAS work after at most two
frame-slot initialization builds. M1b targets no steady-state full BLAS builds, an equivalent update/refit GPU total below
`9.97 ms`, and wall time below `0.677 s` on this probe.

The M1a validation uses the same GPU/driver and installed optimized editor. The static Bistro startup records exactly two
frame-slot builds: RTX averages `0.182352 ms` (`0.364704 ms` total) versus M0's `2.800 ms` 16-frame total, while RayQuery
averages `0.181600 ms` (`0.363200 ms` total) versus M0's `2.752 ms` total. The following 16-frame capture windows report no
`TLAS Build` or `TLAS Update` samples, and their RTX and RayQuery linear-HDR hashes exactly match M0. The motion probe
reports 16 in-place `TLAS Update` samples at `0.023036 ms` average (`0.368576 ms` total), no TLAS rebuilds, unchanged 32
animated `BLAS Build` samples, and `0.691 s` wall time. The final static evidence is under
`out\raytracer-m1a-final-fast`; the transform evidence is under `out\m1a-final-motion`. Both log sets are free of
validation, device-loss, and fatal-error messages. Unit/source-contract coverage separately checks the Vulkan update
classifier, mesh-empty inactive dummy instance, per-particle world-transform blocks, and submitted-versus-discarded frame
tickets.

The M1b motion validation replaces all 32 capture-window animated `BLAS Build` samples with in-place `BLAS Update`
samples. RTX averages `0.095087 ms` (`3.043 ms` total) with `0.313 s` wall time, and RayQuery averages `0.095416 ms`
(`3.053 ms` total) with `0.303 s` wall time. Both remain below the M0 `9.97 ms` GPU and `0.677 s` wall targets, record
16 TLAS updates for the changing animated bounds, and report no capture-window BLAS or TLAS builds. Each technique's HDR
is bit-exact to its pre-M1b deterministic motion capture. Static Bistro RTX and RayQuery remain bit-exact to M1a with no
capture-window acceleration-structure work. Evidence is under `out\m1b-final-motion` and
`out\raytracer-m1b-final-fast`.

The M2 query-only smoke capture reports acceleration structures and RayQuery enabled with the RT pipeline and SER disabled.
Its 1280x720, 64-SPP HDR is finite and nonblank, records only `Path Trace (RQ)`, and has no validation, fallback,
device-loss, or fatal log hits. The normal fast RTX and RayQuery HDR hashes remain bit-exact to M1b. The canonical
RTX-versus-RayQuery MAE/RMS is `0.001001/0.002475`, within the M0 `0.001003/0.002483` gate. Evidence is under
`out\m2-rq-only`, `out\raytracer-m2-fast`, and `out\raytracer-m2-canonical-final`.

### M3a Base-Material Parity

The `rendering-regression` scene contains focused M3a probes that Bistro does not: independent UV0/UV1 coordinates,
rotated/nonuniform `KHR_texture_transform`, vertex RGBA, a UV1 normal map under nonuniform instance scale, native
specular-glossiness colored F0, OPAQUE alpha zero, vertex-driven MASK coverage, and vertex-driven BLEND opacity. Capture
the scene in all three techniques with the same dimensions and deterministic settings. Mirrored single-sided,
double-sided normal-map, and mixed positive/negative instanced probes additionally cover winding and tangent-frame
handedness. Inspect the named `M3a` probes and compare the linear RTX/RayQuery outputs in addition to checking the display
PNGs.

```bat
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\m3a-raster.png --preview-render-mode rasterization --preview-warmup-frames 1800 --preview-width 1280 --preview-height 720 --preview-deterministic
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\m3a-rtx.hdr --preview-render-mode raytracing --preview-warmup-frames 512 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\m3a-rayquery.hdr --preview-render-mode rayquery --preview-warmup-frames 512 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
```

The finalized M3a probe captures at 1280x720 and 2048 SPP produced an RT-pipeline-versus-RayQuery linear HDR MAE of
`0.0001972075` and RMS of `0.001589219`. The 2560x1440 Bistro baselines below record the broader reference gap without
using it as a substitute for the focused probes:

| Profile | Comparison | Linear MAE | Linear RMS |
| --- | --- | ---: | ---: |
| fast, 64 SPP | reference RTX vs EvoEngine RTX | 0.0174610 | 0.0454867 |
| fast, 64 SPP | reference RayQuery vs EvoEngine RayQuery | 0.0174641 | 0.0452488 |
| fast, 64 SPP | EvoEngine RTX vs RayQuery | 0.00201219 | 0.0109933 |
| canonical, 2048 SPP | reference RTX vs EvoEngine RTX | 0.0127995 | 0.0310318 |
| canonical, 2048 SPP | reference RayQuery vs EvoEngine RayQuery | 0.0128016 | 0.0310532 |
| canonical, 2048 SPP | EvoEngine RTX vs RayQuery | 0.000708048 | 0.00215062 |

The corresponding manifests are `out/raytracer-m3a-final-fast/manifest.json` and
`out/raytracer-m3a-final-canonical/manifest.json`; focused-probe evidence is under `out/m3a-validation/final`.

Bistro remains the canonical performance and broad imported-material regression scene, but its current asset closure has
no `TEXCOORD_1`, `COLOR_0`, or `KHR_texture_transform` primitives, so a valid Bistro image alone is not an M3a material
parity gate.

### M3b Advanced-Material Parity

The `rendering-regression` scene adds named M3b controls for colored-F0 iridescence, anisotropy at 0 and 90 degrees
counter-clockwise, dispersion off/on over colored emissive strips, experimental retroreflection at factors 0, 0.5, and 1,
explicit specular factor 0, a factor-0.5 HDR specular-color/F90 control, and unlit base color with a deliberately
conflicting emissive factor. Temporary advanced data textures exercise iridescence R/thickness G, anisotropy RG/strength
B, and retroreflection R through UV1 plus `KHR_texture_transform`. The paired controls make extension behavior visible
without downloading external models. Bistro remains the broad imported-material and post-commit 2048-SPP screenshot gate,
not proof that these extensions work.

Use fresh isolated shader caches and capture the focused scene in RTX, RayQuery, and forced query-only modes. The fast
profile is 1280x720 at 64 SPP; the final focused profile is 1280x720 at 2048 SPP. Compare RTX and RayQuery in linear HDR,
inspect a display PNG, and keep the JSON timing and logs with the images:

```bat
set EVOENGINE_SHADER_CACHE_DIR=out\m3b-rtx-runtime\ShaderBinaries
set EVOENGINE_IMGUI_INI_PATH=out\m3b-rtx-runtime\imgui.ini
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\m3b-rtx.hdr --preview-metrics-json out\m3b-rtx.json --preview-render-mode raytracing --preview-warmup-frames 512 --preview-sample-size 4 --preview-auto-spp disabled --preview-firefly-clamp enabled --preview-firefly-clamp-threshold 10 --preview-width 1280 --preview-height 720 --preview-deterministic
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\m3b-display.png --preview-metrics-json out\m3b-display.json --preview-render-mode raytracing --preview-warmup-frames 512 --preview-sample-size 4 --preview-auto-spp disabled --preview-firefly-clamp enabled --preview-firefly-clamp-threshold 10 --preview-width 1280 --preview-height 720 --preview-deterministic
set EVOENGINE_SHADER_CACHE_DIR=out\m3b-rayquery-runtime\ShaderBinaries
set EVOENGINE_IMGUI_INI_PATH=out\m3b-rayquery-runtime\imgui.ini
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\m3b-rayquery.hdr --preview-metrics-json out\m3b-rayquery.json --preview-render-mode rayquery --preview-warmup-frames 512 --preview-sample-size 4 --preview-auto-spp disabled --preview-firefly-clamp enabled --preview-firefly-clamp-threshold 10 --preview-width 1280 --preview-height 720 --preview-deterministic
set EVOENGINE_SHADER_CACHE_DIR=out\m3b-rayquery-only-runtime\ShaderBinaries
set EVOENGINE_IMGUI_INI_PATH=out\m3b-rayquery-only-runtime\imgui.ini
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\m3b-rayquery-only.hdr --preview-metrics-json out\m3b-rayquery-only.json --preview-render-mode rayquery --preview-warmup-frames 16 --preview-sample-size 4 --preview-auto-spp disabled --preview-firefly-clamp enabled --preview-firefly-clamp-threshold 10 --preview-width 1280 --preview-height 720 --preview-deterministic --disable-ray-tracing-pipeline
python Scripts\compare_reference_render.py out\m3b-rtx.hdr out\m3b-rayquery.hdr --out out\m3b-rtx-vs-rayquery.json
```

Acceptance checks are effect-specific as well as whole-frame: the two anisotropy highlights rotate 90 degrees in the
specified direction; dispersion changes only the enabled transmissive volume; retro factor 0.5 remains between the 0 and
1 controls without inverse-probability brightening; specular factor 0 removes the dielectric highlight; and the unlit probe
shows its green base rather than the conflicting red emission. Both ray techniques must remain finite/nonblank, forced
RayQuery must report the RT pipeline disabled, and logs must contain no shader, validation, device-loss, unexpected
render-technique fallback, or fatal errors. The forced query-only run may report the expected SER capability fallback
because disabling the RT pipeline also disables SER.

Final evidence is under `out/m3b-validation/final-approved`. The 1280x720, 2048-SPP focused captures produced
RT-pipeline-versus-RayQuery linear HDR MAE/RMS `0.0002642074/0.0018277064`; both outputs were finite and nonblank. The
64-SPP normal and forced query-only RayQuery HDRs were bit-exact, and the forced run reported acceleration structures and
RayQuery enabled with the RT pipeline disabled. Retro ROI mean luminance for factors 0/0.5/1 was
`1.402282/1.274665/0.313722`, so the midpoint remained between both endpoints without inverse-probability brightening. All
ROI coordinates and means are stored in `out/m3b-validation/final-approved/retro-roi.json`. All focused logs were clean
apart from the expected forced-run SER capability fallback. The broad Bistro matrices are under
`out/m3b-validation/bistro-fast-final` and `out/m3b-validation/bistro-canonical-final`; canonical EvoEngine
RT-pipeline-versus-RayQuery MAE/RMS was `0.0008044209/0.0022774957`, within the M0 gate. The focused material filter passed
all 93 tests across layout, conversion, raster, ray-material, and serialization-migration suites.

### M4 Static Emissive-Triangle NEE

The `rendering-regression` scene adds isolated constant and high-frequency 32x32 sRGB-textured static emitters above
diffuse floor/back-wall receivers. The textured emitter uses UV1 plus a texture transform. The camera override
`--preview-camera-position 0,4.8,5.6 --preview-camera-look-at 0,4.4,-2.4` frames only this upper probe. Capture RTX and
RayQuery with `--preview-emissive-nee enabled` and `disabled`; disabling NEE preserves hit-only emission. Use 2048 SPP
for energy/parity, 64 SPP for variance, firefly clamp disabled for the energy/variance pairs, and clamp 10 for the
existing M0 cross-technique MAE/RMS gate. A forced query-only 64-SPP capture must match normal RayQuery exactly.

The matrix filenames and local validator are fixed so the gate is repeatable:

```bat
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\m4-validation\matrix\rtx-on-2048.hdr --preview-metrics-json out\m4-validation\matrix\rtx-on-2048.json --preview-render-mode raytracing --preview-warmup-frames 512 --preview-sample-size 4 --preview-auto-spp disabled --preview-firefly-clamp disabled --preview-emissive-nee enabled --preview-camera-position 0,4.8,5.6 --preview-camera-look-at 0,4.4,-2.4 --preview-width 1280 --preview-height 720 --preview-deterministic
python Scripts\validate_emissive_triangle_nee.py --matrix-dir out\m4-validation\matrix --out out\m4-validation\matrix\validation.json
```

The validator reads every adjacent metrics JSON and rejects a mismatched output path, demo profile, camera override,
render mode, SPP, clamp/NEE/SER state, dimensions, non-deterministic capture, missing ray capability, or invalid
forced-query-only capability set before accepting image comparisons. The final fresh-cache matrix reports clamp-10
RTX-versus-RayQuery MAE/RMS `0.0000439001/0.0018905856`, below the M0 limits `0.001003/0.002483`, and
normal-versus-forced-query-only RayQuery is bit-exact. With clamp disabled, 2048-SPP NEE versus hit-only receiver
luminance differs by `0.0567%` RTX and `0.0560%` RayQuery; every RGB channel differs by less than `0.06%`. At 64 SPP,
NEE receiver RMS is `0.6817x` RTX and `0.6814x` RayQuery relative to hit-only, exceeding the required 25% variance
reduction. Evidence is under `out/m4-validation/matrix-final`.

The final 2560x1440, 2048-SPP Bistro run reports RTX-versus-RayQuery linear-HDR MAE/RMS
`0.0007596530/0.0022384434`, within the M0 gate. Its optimized path-trace averages are `243.599 ms` RTX and `342.975 ms`
RayQuery on the recorded RTX 5070 configuration. Evidence is under `out/m4-validation/bistro-canonical-final`.

### M5 Camera-Ray Shader Variants

Camera-ray shaders keep the full 288-byte glTF material ABI while compiling behavior-only variants from the feature set
actually referenced by the current scene. `MAT_EXT_*` remains layout-only; deterministic `EE_GLTF_USE_*` defines select
transmission, volume/scatter, clearcoat, iridescence, anisotropy, sheen, dispersion, diffuse transmission,
retroreflection, unlit, specular, IOR, specular-glossiness, and texture-transform behavior. Volume scatter promotes
volume, and volume promotes transmission. RTX specializes raygen plus any-hit while sharing miss/closest-hit modules;
RayQuery builds only its independent compute pipeline. A permanent all-feature pipeline remains available for startup,
failure fallback, and `--preview-ray-shader-variant full` comparisons.

Variant compilation runs on the render executor and publishes only during normal scene preparation. A published pipeline
resets matching camera accumulation exactly once. Automated captures wait for the exact requested variant before counting
SPP and record requested/active masks and keys, cache origin, pending/failure state, fallback frames, activations, resets,
and shader-cache counters under `ray_shader_variant` in the metrics JSON.

Use isolated cache directories for cold runs, reuse the same directory for the matching warm run, and capture the fixed
640x360, 512-SPP matrix named by `Scripts/validate_ray_shader_variants.py`. Every capture uses deterministic mode,
4 samples per frame for 128 frames, Auto-SPP disabled, firefly clamp 10, and SER disabled. The matrix compares full versus
automatic variants for Bistro and `rendering-regression`, cold versus warm automatic variants, and normal versus forced
query-only RayQuery:

```bat
set EVOENGINE_SHADER_CACHE_DIR=out\m5-validation\matrix\rtx-cache
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo bistro --editor --capture-demo-preview out\m5-validation\matrix\bistro-rtx-auto-cold.hdr --preview-metrics-json out\m5-validation\matrix\bistro-rtx-auto-cold.json --preview-render-mode raytracing --preview-ray-shader-variant auto --preview-warmup-frames 128 --preview-sample-size 4 --preview-auto-spp disabled --preview-firefly-clamp enabled --preview-firefly-clamp-threshold 10 --preview-ser disabled --preview-width 640 --preview-height 360 --preview-deterministic
python Scripts\validate_ray_shader_variants.py --matrix-dir out\m5-validation\matrix --out out\m5-validation\matrix\validation.json
```

Cold/warm and normal/query-only captures must be bit-exact because they execute the same specialized SPIR-V. Full versus
automatic variants use a tight linear-HDR equivalence gate (`max <= 1e-2`, `mean <= 1e-6`, `RMS <= 5e-5`) because removing
inactive material branches can change floating-point instruction scheduling even when the rendered behavior is equivalent.

The shader disk cache preprocesses before keying, so transitive include content participates alongside stage, schema,
Vulkan 1.3/SPIR-V 1.4 targets, and compile options. Entries carry validated metadata and payload checksums, publish via a
same-directory temporary file plus rename, treat truncated/corrupt data as a miss, and coalesce identical in-process
requests. `EVOENGINE_SHADER_CACHE_DIR` still selects the cache root; legacy decimal YAML entries are ignored.

### M6 Four-Capture Evo Benchmark

M6 adds `Scripts/reference_patches/vk_gltf_renderer_m6.patch` on top of the M0 reference instrumentation. That patch
recorded benchmark-only SER state, isolated Vulkan pipeline-cache evidence, GPU and host-render timing, active shader
features, pipeline/frontend build latency, acceleration-structure counts, and categorized memory without enabling
denoising, upscaling, or Physical Sky. The pinned reference completed and excluded one warmup submission before its
measurement window. Its BLAS telemetry distinguishes original, compacted, simultaneous-live, and peak allocation bytes.

The approved M6 plan does not rebuild or rerun the reference. Two earlier specialized cold captures are copied read-only
to `out/raytracer-m6-reference-baseline`: RTX repetition 4 and RayQuery repetition 3, both at the fixed Bistro overview
camera, 1280x720, and 512 requested SPP. `Scripts/raytracer_m6_suite.json` pins their provenance, record, HDR, and log
hashes plus the complete reference/Evo camera mapping. The runner rejects a missing or changed archive or camera, stale
absolute paths inside the moved records are never trusted, and the archived records remain on their original evidence
fingerprint. Reapplying the M6 patch and rebuilding the reference is an archival-reproduction step only:

```bat
git -C out\reference\vk_gltf_renderer apply C:\Users\lllll\Documents\GitHub\EvoEngine\Scripts\reference_patches\vk_gltf_renderer_m6.patch
cmake --build out\reference\vk_gltf_renderer\build-m0 --config RelWithDebInfo --target vk_gltf_renderer
```

There are exactly four new captures. The automated benchmark contains three fresh EvoEngine captures: RTX, RayQuery, and
forced query-only RayQuery, each using the fixed Bistro overview camera at 1280x720, 512 requested SPP, specialized/auto
shaders, SER disabled, and isolated cold runtime/cache state. After the M6 commit, the normal 2560x1440 2048-SPP Bistro PNG
is the fourth capture and remains a delivery artifact outside the automated benchmark plan. No fresh reference, warm-cache,
pilot, truth, EvoEngine-only, canonical, material, or motion/AS benchmark capture is hidden in the plan.

The compact report is descriptive, not statistically repeated. Fresh EvoEngine rows contain wall throughput,
capture-frame GPU median/p95, CPU queue-submit timing, shader build latency and source, active features,
acceleration-structure work, and categorized memory. Separate historical-reference rows preserve the archived timing,
cache, AS, and memory context. Image comparisons cover frozen-reference RTX versus fresh EvoEngine RTX,
frozen-reference RayQuery versus fresh EvoEngine RayQuery, fresh EvoEngine RTX versus RayQuery, normal versus forced
query-only RayQuery, and frozen-reference RTX versus RayQuery. Do not infer time-to-quality, cross-run p95,
variance-adjusted targets, same-session speedups, or binding acceptance targets from the archived reference timing.

The four-capture delivery plan has no warm pairs. Every fresh EvoEngine process still receives an isolated shader cache and
ImGui state, must report the expected cold compile source and artifact, and passes strict project-reset, log, and artifact
integrity gates. The stopped 877-record expanded run remains exploratory cache-pair evidence under
`out/raytracer-m6-expanded-partial-877`; it is not rewritten or represented as the final M6 plan.

```bat
python Scripts\run_raytracer_m6.py --phase self-test
python Scripts\run_raytracer_m6.py --phase plan
python Scripts\run_raytracer_m6.py --phase all
```

Compact mode may run `measure` and `analyze` separately. `--resume` applies only to complete fresh EvoEngine records whose
image, log, metrics, specification, cache artifact, project-reset evidence, and current provenance still validate; it never
imports an archived reference as a fresh measurement. Each Evo Bistro process records its post-run generated project
state, resets it, and requires the reset to be byte-identical to the prepared pre-run state. The runner verifies the
frozen-reference archive, pinned assets, optimized EvoEngine build metadata, installed binary/DLL closure, generated camera
assets, and per-run telemetry before it writes `runs.csv`, `historical-reference-runs.csv`, `image-comparisons.json`, and
`report.json` under `out/raytracer-m6`.
The exact runner, suite, and comparator used for the three captures are preserved under
`out/raytracer-m6/harness-snapshot` with hashes matching `provenance.json`. Two post-capture runner corrections strengthen
resume validation for expanded warm-cache and optional `--skip-evo-prepare` records; neither branch is reachable from the
frozen cold compact plan. The captured root remains tied to its preserved harness snapshot instead of being relabeled or
rerun as fresh evidence.

EvoEngine's reported memory peak is explicitly limited to startup-ready and capture-window samples; it excludes pre-ready
transient allocation peaks and therefore cannot by itself define M12's transient startup-memory target. Pinned-reference
timing is a historical archived observation, not a concurrent measurement. Reports retain its original revision,
instrumentation, hardware/driver, timing source, and sample count, and exclude it from matched speedup ratios or target
acceptance.

#### M6 Baseline and Approved Targets

The fresh one-run M6 slice on the NVIDIA GPU/driver recorded in `report.json` produced the following current EvoEngine
baseline. These are descriptive observations, not estimates of run-to-run variance:

| Lane | Accumulation wall | Wall throughput | Path GPU median | Queue-submit median | Startup BLAS GPU |
| --- | ---: | ---: | ---: | ---: | ---: |
| RTX, SER off | 10.4163 s | 44.946 Msample/s | 66.085 ms | 0.1322 ms | 399.757 ms / 2,909 builds |
| RayQuery | 9.7394 s | 48.070 Msample/s | 62.986 ms | 0.1187 ms | 400.764 ms / 2,909 builds |
| forced query-only | 9.7549 s | 47.994 Msample/s | 61.933 ms | 0.1297 ms | 401.202 ms / 2,909 builds |

All three lanes recorded two startup TLAS builds totaling about `0.366-0.368 ms`, zero capture-window BLAS/TLAS work,
and final device-local VMA allocation of about `6.086e9` bytes (`5.668 GiB`). RayQuery and forced query-only were
bit-exact. RTX versus RayQuery relative L2 was `0.008947`. The following targets were explicitly approved before M7:

- M11, with SER disabled: RTX must reach at least `49.5 Msample/s`, at most `60.0 ms` GPU median, and at most `9.50 s`
  accumulation wall time. RayQuery must reach at least `53.0 Msample/s`, at most `57.5 ms` GPU median, and at most
  `8.90 s` wall time. Both queue-submit medians must remain at or below `0.15 ms`; forced query-only must remain bit-exact
  to RayQuery and reach at least `45.0 Msample/s`; RTX-versus-RayQuery relative L2 must remain at or below `0.010`.
- M12: preserve exactly zero capture-window AS work; keep raw startup BLAS-build GPU total at or below `425 ms` while
  adding compaction; honor the fixed 512 MiB scheduling hint except for one oversized singleton; keep aggregate compacted
  bytes for eligible static BLASes at or below 75% of those same BLASes' aggregate uncompacted bytes; and reduce final
  device-local allocation on this exact slice to at most `6.00e9` bytes. M6 did not observe pre-ready transient peaks or
  EvoEngine scratch/compaction categories, so it cannot define binding transient-peak, scratch-peak, pass-count, or
  compaction-inclusive startup-wall targets; M12 must add that telemetry before evaluating them.
- M13: the fresh M6 binding gate is exact zero static capture-window BLAS/TLAS builds and updates. The interrupted expanded
  run's unchanged 64-SPP `motion-as` probe is exploratory only: its complete specialized/cold records observed 30 BLAS
  updates, 15 TLAS updates, zero builds, `2.853-3.066 ms` BLAS total, `0.326-0.562 ms` TLAS total, and wall ranges of
  `0.405-0.420 s` RTX, `0.346-0.354 s` RayQuery, and `0.342-0.347 s` query-only. Before M13 changes, refresh that probe;
  proposed regression ceilings are zero builds, the expected 30/15 updates, `3.20 ms` BLAS total, `0.60 ms` TLAS total,
  `0.45 s` RTX wall, and `0.38 s` RayQuery/query-only wall. Morph and large-instance paths require their own fresh baselines.
- M16: each lane's queue-submit median must be at most `0.10 ms`; normalized non-path wall overhead,
  `(accumulation_wall_ms - measured_path_gpu_total_ms) / 127`, must be at most `12.5 ms/frame`; new wait-reason telemetry
  must report zero redundant or just-submitted-frame waits; and path GPU median must not regress by more than 5% from the
  approved post-M11 slice. The existing generic fence-wait duration follows GPU completion and is not an independent
  acceptance target.

#### M7 Ray Diagnostic Views

Ray cameras expose the same `RayDebugView` implementation through the RTX pipeline and RayQuery integrator. `Beauty` is
the default. The editor Camera inspector provides `Ray Debug View`, and automated captures use
`--preview-ray-debug <view>` together with an explicit ray `--preview-render-mode`. Capture metrics record the normalized
view name as `ray_debug_view`. Changing the view resets accumulation.

Automatic shader variants reserve a non-glTF cache-key bit for diagnostics. Steady-state Beauty-only RTX and RayQuery
techniques, including all-material specializations, compile every diagnostic field and branch out of the shared
integrator; the permanent all-material safety fallback retains diagnostics while an exact specialization builds.
Diagnostic demand is tracked independently for RTX and RayQuery. If any camera of a technique selects a diagnostic view,
that technique uses its diagnostic variant until all its cameras return to Beauty. Public material feature masks remain
glTF-only, while the variant key appends `:debug` when instrumentation is active.

Diagnostic HDRs are raw linear RGB rather than the pinned reference's display-oriented sRGB visualization. Normals are
encoded as `0.5 * N + 0.5`. Roughness stores perceptual roughness and squared anisotropic alpha-x/alpha-y. Alpha stores
per-accepted-candidate raw alpha, transport opacity, and candidate acceptance probability. Accumulated blend pixels are
coverage-weighted because rejected candidates continue traversal; they do not display unconditional raw material values.
M10 owns that documented sampling distinction. Transmission stores specular, diffuse, and combined transmission.
Iridescence stores factor, thickness divided by 1200 nm, and IOR divided by 3. `Specular F0` is the effective
scalar-weighted colored dielectric F0. PDF channels use zero for invalid, `0.75` for Dirac, and `0.05-0.50` for
log-encoded finite positive values; the BSDF view contains path-sampled, punctual/environment-NEE-evaluated, and
emissive-NEE-evaluated PDFs.

`Validation Atlas` packs Beauty plus the 19 diagnostic outputs into a 5x4 image. Each cell letterboxes the same 16:9
camera and seeds the estimator from tile-local coordinates, so Beauty, primary emission, the three direct-light terms,
and indirect radiance can be checked for additive conservation without launching one process per view. Saved HDR/derived
PNG rows are bottom-up in enum order: Beauty through Shading Normal are on the bottom row and Indirect Radiance through
Emissive PDF are on the top row. `validation.json` records the explicit top-to-bottom mapping.

The final binding matrix contains three fresh 1280x720, 64-SPP captures: RTX, RayQuery, and forced query-only. One earlier
three-capture diagnostic matrix was discarded after it exposed a saved-HDR row-mapping error in the validator and an
empty-UV1 tangent fixture; both were corrected before repeating the same matrix. The first post-commit Beauty capture was
also rejected: although its image was bit-identical to M6, keeping diagnostic state live increased RTX GPU median from
`264.458 ms` to `329.702 ms` (`24.67%`). M7 therefore compiles diagnostics out of Beauty, uses one focused forced-query-only
capture to prove the guarded diagnostic path remains equivalent, and replaces the post-commit Beauty delivery capture.
The resulting nine total renderer launches remain within ten. The pinned reference is not rerun because the generated
EvoEngine regression scene is not a matched portable input.

Run the focused gate with:

```bat
python Scripts\validate_ray_debug_views.py --self-test
python Scripts\validate_ray_debug_views.py --dry-run --editor out\install\vs2026-x64\bin\EvoEngineEditor.exe
python Scripts\validate_ray_debug_views.py --capture --editor out\install\vs2026-x64\bin\EvoEngineEditor.exe
```

The accepted report is `out/m7-validation/atlas/validation.json`. RayQuery and forced query-only are bit-exact with
SHA-256 `07f4fceae79b5cbb682a7c558c03a68c4f7bc1aeb2646c88ec389afee242576f`. Every focused ROI and relation passes;
the worst attribute MAE/RMS is `0.00002665/0.00061523`, and contribution-conservation relative L2 is at most `0.000610`.
After the Beauty compile-out change, `out/m7-validation/perf-fix-query-only/rq-only.hdr` reproduced that SHA-256
bit-for-bit with RT-pipeline capability disabled and exact active/requested key `rq:0x6eb3:debug`; its log was clean.
Transport is gated per term rather than pooled. Beauty, emission, direct-punctual, and indirect differences are classified
as bounded 64-SPP stochastic tails with explicit mean, 99.9th-percentile, maximum, and high-energy-count limits; direct
environment and direct emissive pass tighter conformant bounds. The observed single-channel maxima of `112` in Beauty
and direct punctual are therefore visible and bounded rather than diluted by other views. These single-capture limits are
guardrails, not variance estimates; the established clamped 512-SPP Bistro relative-L2 gate remains separate.

The validator also checks finite channel ranges, named material/sampler/tangent ROIs, capture provenance, and the
classifications committed in `Scripts/raytracer_m7_expectations.json`. Known semantic gaps remain explicit M8/M9 inputs
rather than being hidden by the visualization; alpha-blend traversal-edge and coverage-weighting behavior remains
classified as bounded stochastic noise pending M10.

### M8 Material-Semantic Closure

M8 keeps the M7 5x4 atlas contract and adds focused `rendering-regression` probes rather than changing the diagnostic
enum or invalidating M7 evidence. The scene now pairs clearcoat normal scale 0/1, uncoated/coated emission, and matched
fractional-specular controls with and without iridescence. Existing probes continue to cover native colored-F0
specular-glossiness, transmission, scalar specular 0/0.5, and unlit base color. Import-only forbidden combinations stay in
unit tests and never receive accidental render goldens.

The focused gate uses exactly three renderer launches: one deterministic 1280x720 raster PNG plus matched 1280x720,
64-SPP RTX and forced query-only atlases. It reruns M7 attribute, transport, conservation, provenance, and ROI checks for
the two fresh ray lanes, while hash-verifying the accepted M7 report that proved ordinary RayQuery and forced query-only
were bit-exact. M8 then checks raster and ray clearcoat-normal/coated-emission relationships plus the raster F90/unlit
path. The normal post-commit 2560x1440, 2048-SPP Bistro image is the fourth and final launch. The pinned reference is not
rerun because neither its revision nor the matched reference input changed.

Adding two opaque coated-emission probes changes the 64-SPP emissive-selection distribution, so M8 records a separate
deterministic single-capture, bounded-stochastic guardrail for the fresh RTX/query-only Direct Emissive atlas
(`mean <= 4e-6`, `p99.9 <= 5.5e-4`, `max <= 0.025`) instead of relabeling the older M7 scene envelope. The accepted M7
report and its tighter ordinary RayQuery/query-only result remain hash-pinned and unchanged; every other fresh transport
limit retains the M7 value.

The material unit gate includes the 288-byte host/shader ABI, import diagnostics and deterministic recovery, material
serialization, raster and shared-ray source contracts, plus one GPU numerical dispatch covering colored F0, scalar F90,
clearcoat normal scaling, coated-emission bounds, and finite energy. Run the focused validation with:

```bat
out\build\vs2026-x64-tests\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter="GltfMaterialConversion.*:GltfMaterialLayout.*:GltfRasterMaterial.*:GltfRayTracingMaterial.*:GpuService.GltfRayTracingNumericalProbeMatchesAnalyticValues:SerializationRegistry.MaterialRoundTripKeepsTransparentExtensionFields"
python Scripts\validate_raytracer_m8.py --self-test
python Scripts\validate_raytracer_m8.py --capture --editor out\install\vs2026-x64\bin\EvoEngineEditor.exe
```

The retained pre-commit evidence under `out/m8-validation/precommit-final` passes all 48 M8 gates and all 88 compact-atlas
gates. Raster grazing F90 separation is `22.99x` its matched center separation; clearcoat-normal and coated-emission rim
luminance ratios are `0.891` and `0.930`. RTX/query-only coated-emission ratios agree within `0.0001`, the matched
iridescence controls retain bit-exact F0, and their Beauty values separate by `0.0461`/`0.0474`. The raster PNG SHA-256 is
`45c4136d3993c4a8c599b6e1345ff9d76067c37793ce686bbdf81a9a7b2600a3`; RTX and forced-query-only HDR SHA-256 values are
`4045071c6f107b8ea119c6af3d4ef3b3793db913283e5a0759944675bd02976b` and
`9fb034a277104270fa9276de8d376eabc65efa8a1ff9f0b011fb7d9461cbb928`.

### M9 glTF Resource And Texture Fidelity

M9 keeps M8's exact three successful pre-commit launch structure: one deterministic 1280x720 raster image and matched
64-SPP RTX and forced query-only validation atlases. It reruns every inherited M7/M8 provenance, material, transport,
conservation, and classification gate and hash-verifies the accepted M7 ordinary-RayQuery/query-only identity. It does
not launch ordinary RayQuery or the pinned reference. One rejected raster wrote its image and metrics but crashed during
shutdown after the ImGui Vulkan backend was gone; the explicitly approved replacement makes the post-commit 2560x1440,
2048-SPP Bistro delivery image launch five. The rejected evidence remains under
`out/m9-validation/precommit-shutdown-crash`.

The rendering-regression scene routes the transformed base-color and normal textures through `TEXCOORD_3`, routes a
minified checker through `TEXCOORD_2`, uses persistent texture sampler settings, and replaces the duplicated seam fixture
with a shared-index mirrored chart that MikkTSpace must split. The checker is authored as alternating encoded sRGB black
and white: correct linear filtering converges near `0.5`; encoded-space averaging followed by sRGB decoding would produce
about `0.214`. Vertex color modulates the rendered probe, so the validator uses channel-specific bounds around a tight
interior ROI plus spatial-deviation and RTX/query-only agreement gates. M9 permits only the two inherited M7 seam-right
range failures superseded by the new Mikk fixture; every unrelated M7/M8 gate remains binding.

Focused unit tests cover external-image `.gltf`, data-URI `.gltf`, binary `.glb`, sampler enum mapping, all four UV sets,
invalid-set binding rejection, Mikk chart splitting, authored-tangent preservation, and texture/mesh serialization. Run:

```bat
out\build\vs2026-x64-tests\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter="GltfMaterialConversion.*:GltfMaterialLayout.*:Texture2D.*:GpuService.Texture2D*:GpuService.TextureStorage*:SerializationRegistry.BuiltInAnimationAndPostProcessingTypesInstallSerializationHandlers:GltfRasterMaterial.*:GltfRayTracingMaterial.*:CameraRenderTechnique.*"
python Scripts\validate_raytracer_m9.py --self-test
python Scripts\validate_raytracer_m9.py --capture --editor out\build\vs2026-x64-tests\EvoEngine_App\RelWithDebInfo\EvoEngineEditor.exe --output-dir out\m9-validation\precommit-final
```

Khronos sampler definitions are authoritative: `NEAREST_MIPMAP_NEAREST`/`LINEAR_MIPMAP_NEAREST` select nearest mip
selection, while `NEAREST_MIPMAP_LINEAR`/`LINEAR_MIPMAP_LINEAR` interpolate between mip levels. These explicit mappings
also match the pinned reference. When either filter or the texture's sampler is omitted, EvoEngine retains its conforming
repeat/trilinear implementation choice rather than inventing a serialized glTF value.

### M10 Sampling And Environment Transport

M10 uses one PCG implementation with domain-separated streams derived from the pixel/global-sample seed and a monotonic
path-segment index. Camera jitter, surface alpha, punctual/environment light selection, direct and continuation BSDF,
emissive NEE, surface shadows, volume free flight/phase/NEE, and roulette each own a documented domain. Every local stream
advances through `EE_PCG_RANDOM`; sequenced two- and three-value helpers avoid relying on expression evaluation order.
This keeps variable alpha-candidate counts from shifting later transport decisions and keeps ordinary RayQuery and forced
query-only deterministic across independent processes.

Texture2D environment maps use a two-level marginal/conditional CDF built from exact lat-long texel solid angles. The two
CDF draws are remapped within the selected probability intervals, so azimuth is continuous and elevation is uniform in
solid angle inside the texel. PDF evaluation remains piecewise constant and uses the selected texel. Zero-energy maps omit
the CDF and use the uniform-sphere fallback. Direct cubemap and EvoEngine's existing baked-sky construction also use that
valid `1 / (4 pi)` fallback; M10 does not add or port the reference renderer's Physical Sky.

`EnvironmentalMap` retains the raw constructed cubemap separately from its irradiance/reflection probes. Ray-traced
environment lighting evaluates that scene source, while the camera clear color or skybox remains a primary-background
choice. Environment rotation is radians around world `+Y`: sampling rotates local directions by `+rotation`, while
radiance and PDF evaluation rotate world directions by `-rotation`. Raster skybox, irradiance, and prefiltered lookups use
the same convention. Cubemap assets serialize every RGBA32F face and mip as binary floats in Vulkan face order
`+X, -X, +Y, -Y, +Z, -Z`, with each face's mip chain contiguous. Generated maps persist a rebuild recipe for a
file-backed Texture2D/Cubemap source or the complete baked-sky parameters, so temporary runtime cubemaps and PDF textures
are restored after asset reload without recursively embedding those generated assets. Serialization rejects allocated
cubemaps whose GPU storage has never received valid texels instead of reading or persisting uninitialized device memory.
This milestone's
rotation scope is the active glTF raster, RTX, RayQuery, and forced query-only paths; the roadmap-excluded `CameraLegacy`
and point-cloud ray shaders remain unchanged.

Directional and environment shadow rays use the shared finite traversal maximum rather than camera far distance. Camera
far remains the primary/miss depth sentinel. A true volume scatter advances ray-cone width by travel distance times the
camera spread angle before the next surface chooses texture gradients; the diagnostic atlas uses its effective per-tile
viewport height for that spread. Absorption-only segments retain the existing single-segment surface update.

The first deterministic 1280x720, 64-SPP RTX, ordinary-RayQuery, and forced-query-only atlases under
`out/m10-validation/precommit` are retained only as rejected fixture-calibration evidence. They proved byte-exact
ordinary-RayQuery/query-only extension independence, but the camera still viewed the crowded M42 rig instead of the
translated M10 probes, so none of their RTX, transport, or ROI results is accepted. The approved acceptance-blocker
exception adds exactly two replacement launches: RTX and ordinary RayQuery viewing the isolated `isolated-v2` fixture at
world `X=48`. The validator recomputes the historical query-only comparison from its original directory, pins all source
artifacts plus the original executable byte-for-byte and pins affected ray-shader hashes after canonical LF newline
normalization, so checkout line-ending conversion cannot invalidate unchanged source. It fails if the fresh RayQuery variant no longer
matches the retained key/mask. It never labels the retained pair as fresh corrected-fixture evidence.

The corrected fixture includes a rotated high-contrast environment, enlarged directional receivers with a blocker beyond
camera far, and a checker around a transmissive scattering volume. Fresh RTX/RayQuery attributes and conservation retain
the M7 limits. M10's separately named high-contrast transport limits were pre-locked from the rejected pilot before the
replacement images; they do not modify M7. PDF normalization, constant-environment energy, high-contrast variance,
rotation round trips, PCG vectors/domains, distant range, and volume-cone math remain focused tests rather than extra
renderer launches. Together with the three rejected launches and normal post-commit delivery capture, M10 uses six
launches total. The pinned reference is not rerun.

The replacement atlases are revalidated from their saved HDRs without another renderer launch. Material ID locates the
actual receiver interiors: the beyond-far blocker produces zero direct-punctual luminance while the control measures
`0.686155` in both RTX and RayQuery. For the checker, the validator does not use coefficient of variation: three sparse
volume pixels dominate that statistic. Instead it finds the dominant checker-panel and volume Material IDs, erodes their
antialiased boundaries with every eligible `12x12` patch, and measures the two diagonal fundamentals of the known `64x64`
XOR texture after 5/95-percent winsorization. The median clear-to-volume frequency response must be at least `2x`; the
accepted saved images measure `5.57x` RTX and `6.28x` RayQuery. This image gate is deliberately named checker-frequency
suppression: scattering or opacity can also suppress the pattern, so it is not presented as causal LOD evidence. The
focused numerical contract separately proves that the shader's scatter-distance cone advance increases the next
surface's texture gradient and selected LOD, while shader-source ordering and GPU compilation keep that math connected to
both traversal techniques. Use:

The accepted schema-4 saved-HDR report is `out/m10-validation/replacement/validation-corrected.json`; it embeds the M10
frequency/LOD threshold blocks and the complete expectations-file SHA-256. The earlier `validation.json` remains rejected
evidence of the stale receiver rectangles and invalid raw-contrast metric.

```bat
out\build\vs2026-x64-tests\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter="GltfRayTracingMaterial.*:CameraRenderTechnique.*:GpuService.Cubemap*:GpuService.M10CameraRayTransportShadersCompile:SerializationRegistry.BuiltInAnimationAndPostProcessingTypesInstallSerializationHandlers:VolumetricCloudSettings.SceneEnvironment*"
python Scripts\validate_raytracer_m10.py --self-test
python Scripts\validate_raytracer_m10.py --matrix-dir out\m10-validation\replacement --retained-query-only-dir out\m10-validation\precommit --out out\m10-validation\replacement\validation-corrected.json
```

### M11 Portable Path Dispatch

M11 keeps SER disabled for binding acceptance and changes only vendor-neutral camera-path structure. Generic
`RayTracingPipeline` users retain recursion depth 8 by default because the legacy camera, point-cloud, and DDGI diagnostic
pipelines can trace from a hit shader. The iterative modern camera pipeline requests depth 1, rejects zero or values above
the selected device's `maxRayRecursionDepth`, and records both requested depth and device limit in capture JSON.

The active RTX and RayQuery cameras use a 48-byte compact payload containing intersection identity plus transparent-shadow
state. Closest-hit and committed RayQuery handling return distance, instance, primitive, and barycentrics; the shared
raygen/compute integrator reconstructs geometry and material once. Primary background and secondary environment misses
are also evaluated in that shared integrator, leaving the RTX miss shader as a payload-state transition. CameraLegacy
keeps the original full payload. RTX shadow rays no longer copy/restore the full path payload: ignored alpha/transmission
intersections continue traversal, the shadow miss preserves accumulated transmission, and a committed opaque hit remains
occluded while `TerminateOnFirstHit` and `SkipClosestHitShader` avoid a redundant closest-hit invocation.

Shared camera geometry reconstruction normalizes each finite nonzero triangle edge before taking the face cross product,
so valid geometric normals are independent of object-space scale. Only zero or non-finite geometry falls back to the
interpolated vertex normal. The geometric normal is oriented toward the incoming ray; the shading normal is kept on the
same hemisphere and replaced by the geometric normal when reflection would enter the surface. Bounce and shadow origins
retain the geometric-normal `safeOffsetRay` policy, including the vertex-normal shadow-terminator position.

`Scripts/raytracer_m11_suite.json` pins the approved M6 report SHA-256, device/driver, per-lane baseline, active material
mask/key, and absolute M11 targets. `Scripts/validate_raytracer_m11.py` reuses the compact M6 capture harness but emits an
M11 plan and acceptance report. Before any capture it hashes the archived M6 report and derives the pinned lane metrics
from that file, preventing the committed delta inputs from drifting independently. It requires exactly three isolated
cold processes: RTX, ordinary RayQuery, and forced
query-only, all at 1280x720 with 512 effective SPP, 508 measured SPP, the specialized `0x2001` material variant, and SER
off. Capture JSON names the active ray backend; forced query-only must report the RayQuery compute backend while the RTX
pipeline capability is disabled. The pinned reference is historical input only and is not launched. One separate
post-commit 2560x1440 2048-SPP
delivery capture brings the milestone total to four launches. Run the file-only checks and the single three-launch slice
with:

```bat
python Scripts\validate_raytracer_m11.py --phase self-test
python Scripts\validate_raytracer_m11.py --phase plan --output-dir out\raytracer-m11
python Scripts\validate_raytracer_m11.py --phase all --output-dir out\raytracer-m11
```

The acceptance report gates RTX at `>=49.5 Msample/s`, `<=60.0 ms` GPU median, and `<=9.50 s` accumulation wall time;
RayQuery at `>=53.0 Msample/s`, `<=57.5 ms`, and `<=8.90 s`; forced query-only at `>=45.0 Msample/s`; and every lane at
`<=0.15 ms` queue-submit median. Ordinary and forced RayQuery must be bit-exact, RTX/RayQuery relative L2 must be
`<=0.010`, and the fresh device/driver must match the approved M6 baseline. The report includes signed percentage deltas
from M6 without presenting the archived reference timing as a contemporaneous speedup.

### M12 Static BLAS Construction and Compaction

Static mesh and bind-pose BLASes are queued against committed `GeometryStorage` ranges instead of uploading private
vertex, index, and transform buffers per mesh. The shared vertex buffer is addressed from its base because packed triangle
indices are global; each build range supplies the committed triangle byte offset. Updateable skinned-renderer BLASes keep
their private inputs and persistent update scratch.

The asynchronous builder uses a binary 512 MiB (`536870912` byte) hint independently for aggregate original-AS
destinations and shared scratch planning. Normal passes cannot exceed either bound; an individually oversized BLAS is an
isolated forward-progress pass. Scratch slices are aligned and reused in barrier-separated waves. Each pass completes its
build and compact-size query, submits compact copies, retires the original handles only after copy completion, and then
continues to the next pass. A static BLAS is published only after compaction, so TLAS construction never observes an
original handle that will be retired. Geometry uploads defer buffer replacement while this chain is active.
On Windows, the immediate-submit queue used by the builder has lower Vulkan queue priority than the main render and
same-family present queues; task priority also keeps its CPU work behind interactive jobs. Queue priorities remain driver
scheduling hints rather than a preemption guarantee.

External glTF images inside the active project reuse the project-managed `Texture2D` image when the requested glTF color
space is view-compatible. BC7 and RGBA8 images are created mutable with both UNORM and sRGB view formats; each glTF
binding retains its own image view and sampler while sharing image allocation and lifetime. Pre-decoded float sRGB
fallbacks preserve their linear-sampling marker. Unsupported formats, unavailable views, and failed managed loads retain
the independent import path and continue to the glTF core-image fallback.

Capture JSON reports `blas_builder`, including pass records, shared/private input ownership, scratch and transient peaks,
eligible uncompacted/compacted bytes, compaction ratio, and compaction-inclusive wall time. `BLAS Build` and
`BLAS Compact` remain separate startup GPU sections. `Scripts/validate_raytracer_m12.py` reuses the three cold M11 harness
lanes without launching the reference. Automated capture enables GPU timestamps before default resources create any BLAS,
so startup timing samples and builder pass history share one domain. Builder pass/timing and `cumulative_*` telemetry are
process-lifetime totals; eligible counts/bytes and `final_compacted_storage_bytes` describe only live BLAS storage. The
absolute M6-derived gates require the exact approved vendor, device, and driver. Validation observes three pre-commit
captures and records the fourth post-commit delivery as planned until it is run. It requires zero capture-window AS work,
raw startup `BLAS Build` total at most `425 ms`, compacted eligible-static bytes at most 75% of the matching uncompacted set,
and final device-local allocation at most `6.00e9` bytes. Scratch peak, transient peak, pass count, and builder wall time are
recorded but have no M12 threshold.

```bat
python Scripts\validate_raytracer_m12.py --phase self-test
python Scripts\validate_raytracer_m12.py --phase plan --output-dir out\raytracer-m12
python Scripts\validate_raytracer_m12.py --phase all --output-dir out\raytracer-m12
```

The original three-lane report is immutable acceptance evidence even if a later repair resolves one of its failures. For
the approved texture-residency repair, `Scripts/validate_raytracer_m12_repair.py` creates a separate overlay and permits
exactly one supplementary cold specialized RTX capture. The repair keeps the original RayQuery/query-only records,
requires the new RTX image to remain bit-exact, and verifies the common import path through unchanged 390-asset scan and
338-texture counts plus a project-load dispatch reduction from 390 to 52. It reports RayQuery/query-only post-fix memory
compliance as an approved shared-resource inference, never as a fresh measurement. This exception raises M12 accounting to
four pre-commit captures plus the normal post-commit delivery; it does not launch the reference.

```bat
python Scripts\validate_raytracer_m12_repair.py --phase self-test
python Scripts\validate_raytracer_m12_repair.py --phase plan
python Scripts\validate_raytracer_m12_repair.py --phase measure
python Scripts\validate_raytracer_m12_repair.py --phase analyze
```

### M14 Pipeline and Variant Caching

All engine compute, graphics, and ray-tracing pipeline creation uses one `VkPipelineCache` owned by `Platform`. The raw
driver payload is wrapped in a schema-1 file containing vendor, device, driver, API, and pipeline-cache UUID identity plus
a checksum. Both the wrapper and Vulkan's version-one payload header are validated before driver use. Load and save are
limited to 256 MiB, matching the pinned reference's safety ceiling, and publication uses a same-directory temporary file
with replace semantics. `EVOENGINE_PIPELINE_CACHE_DIR` selects the cache directory; otherwise it is placed beside an
explicit `EVOENGINE_SHADER_CACHE_DIR`, or under the git-ignored `./PipelineCache`.

Pipeline creation records wall latency and optional Vulkan creation feedback. RTX creation first requests
`VK_KHR_deferred_host_operations`, joins the driver work with bounded host concurrency, treats
`VK_OPERATION_NOT_DEFERRED_KHR` as successful completion, and makes at most one synchronous retry after a deferred-path
failure. Devices without creation feedback or deferred execution remain supported. Capture JSON exposes aggregate
`pipeline_cache` state and the active ray variant's `pipeline_creation` result.

The camera-ray cache retains the permanent all-feature fallback and at most eight successful nonfallback variants for
each technique. It admits only one pending build per technique, bounds failed retry records to eight, and evicts by a
deterministic last-use serial. An evicted pipeline and its RTX SBT remain retained by the current frame's submission token
until that frame fence completes or the commands are discarded; eviction never waits for the queue to idle.

M14 uses exactly three pre-commit renderer launches and no reference process: isolated RTX cold, isolated RayQuery cold,
and forced query-only warm using the RayQuery cache. All are 1280x720, 64 SPP, deterministic Bistro captures. The warm
lane must load both persistent caches, report valid aggregate feedback with more application pipeline-cache hits than the
isolated cold RayQuery lane, and remain bit-exact to ordinary RayQuery. Corrupt/incompatible files, concurrent request
coalescing, failed retry records, submission retirement, and
1,000-mask churn are focused tests rather than extra launches. Run:

```bat
python Scripts\validate_raytracer_m14.py --self-test
out\build\vs2026-x64-tests\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter="VulkanPipelineCache.*:RayCameraShaderVariantCache.*:ShaderCache.*"
python Scripts\validate_raytracer_m14.py --output-dir out\raytracer-m14
```

The full-pipeline `VK_PIPELINE_CREATION_FEEDBACK_APPLICATION_PIPELINE_CACHE_HIT_BIT` is not a portable per-pipeline
requirement: [Vulkan describes it](https://docs.vulkan.org/refpages/latest/refpages/source/VkPipelineCreationFeedbackFlagBits.html)
as a signal that the implementation avoided the large majority of creation work. If a
completed three-launch slice was rejected only because the active query-only pipeline left that bit unset, preserve the
original ledger and use `--analyze-existing`. The no-launch overlay requires the warm disk payload, valid aggregate
feedback, a hit count above the cold RayQuery lane, zero frontend recompilations, exact RayQuery/query-only HDR identity,
and every other original gate; it does not rewrite the failed ledger or launch another renderer process.

```bat
python Scripts\validate_raytracer_m14.py --analyze-existing --output-dir out\raytracer-m14
```

### M15 Ray-Camera History Lifetime

Each `Camera` owns one progressive-history slot containing one RGBA32F radiance image and one RGBA32F convergence image,
for 32 bytes per output pixel. Switching between the RT-pipeline and RayQuery techniques always resets that shared slot's
accumulation before the new technique renders. Camera resize retires the allocation; scene/history invalidation and camera
deletion reset or retire only the affected camera state. `RenderLayer` keeps weak active-camera metadata, prunes histories
for cameras no longer in the current render set, retains submitted image views through the frame transient-resource store,
and clears registered histories only after draining GPU resource work during shutdown.

Capture JSON exposes `ray_camera_history` plus descriptor-set and ray-tracing pipeline/SBT counters under
`resource_lifetime`. M15 uses two 1280x720, 64-SPP pre-commit lanes: ordinary RTX and forced query-only. The validator
requires one correctly typed live camera history at the exact 32-byte-per-pixel bound, stable live history and
pipeline/SBT counts after the timing warmup, bounded descriptor-set residency, no capture-window history or pipeline/SBT
creation, and zero RT-pipeline/SBT resources in forced query-only mode. Camera churn, technique switching, resize,
submitted-view retention, and shutdown ordering are no-renderer tests. No reference process is launched. With the
rejected wrapper attempt documented below, the post-commit 2560x1440, 2048-SPP Bistro delivery is the fourth and final
M15 renderer launch.

The accepted M15 run preserved one rejected initial wrapper attempt under
`out/raytracer-m15/rtx-attempt1-no-capture`: invoking the Windows GUI executable directly from PowerShell returned before
capture and allowed the project-reset cleanup to race it, leaving an empty log and no image or metrics. The established
subprocess harness then completed the RTX and forced query-only lanes. Count the rejected process conservatively, making
the post-commit delivery the fourth and final launch within the milestone ceiling; do not add an ordinary RayQuery or
reference run.

```bat
python Scripts\validate_raytracer_m15.py --self-test
out\build\vs2026-x64-tests\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter="RayCameraHistory.*:CameraRenderTechnique.*:RayCameraShaderVariantCache.*"
python Scripts\validate_raytracer_m15.py --rtx-metrics out\raytracer-m15\rtx\metrics.json --query-only-metrics out\raytracer-m15\query-only\metrics.json --output out\raytracer-m15\validation.json
```

Run both RT-pipeline and RayQuery techniques with:

```bat
python Scripts\run_raytracer_baseline.py --profile fast --technique all
python Scripts\run_raytracer_baseline.py --profile canonical --technique all
```

The runner writes linear HDR images, per-process logs/JSON, runtime binary/DLL closure hashes, complete selected glTF/DDS
input closure hashes, repository states, commands, and comparison results under `out\raytracer-baseline`. It produces
reference-versus-EvoEngine comparisons for each technique plus an EvoEngine RT-pipeline-versus-RayQuery comparison. HDR
reports contain absolute MAE/RMS and symmetric-relative MAE/RMS; the old display-PNG thresholds are not reused for linear
radiance.
By default it regenerates the ignored Bistro project template and removes saved `New Scene` state before each EvoEngine run;
`--skip-evo-prepare` exists only for deliberate debugging of a locally edited generated scene.
The reset reuses the existing source cache without fetching or checking out a newer upstream revision. The runner requires
the static Bistro pin above, deterministically derives `bistro-directional-intensity-10.gltf` from its `bistro.gltf`, and
rejects source or derived glTF and selected glTF/DDS closure hashes that differ from M0. Every Evo process also receives a
fresh output-local shader-cache and Vulkan pipeline-cache directory plus an ImGui ini path, so root working-directory
state cannot reuse stale binaries or leak into a capture. Dry runs write `manifest.dry-run.json` and do not replace
measured evidence.

Bistro raster parity remains a separate display-space check. It explicitly enables the configured DDGI volume and uses a
newly initialized post-processing stack: GTAO, SMAA Ultra, and tone mapping enabled with bloom and SSR disabled. The
imported Bistro sun uses light size `0.01`, and the default directional shadow resource is 8192. Ray-tracing and ray-query
parity captures keep DDGI disabled. A matched raster capture uses:

```bat
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo bistro --editor --capture-demo-preview out\bistro-rasterization-2560x1440.png --preview-render-mode rasterization --preview-warmup-frames 256 --preview-width 2560 --preview-height 1440 --preview-deterministic
```

## Visual Checks

Before closing rendering milestones, inspect the generated images and record the exact executable and command used.

Minimum visual checks:

- Rendering demo editor screenshot is nonblank and has the expected editor layout.
- RT-Bistro and 3DGS-Bicycle gallery images remain valid.
- Rasterization and ray captures have the requested dimensions and are not blank.
- Bistro HDR captures contain finite linear RGB and have complete run metadata; Radiance HDR has no alpha channel.
- Logs do not contain crash, hang, validation, device-lost, or unresolved missing-file errors. The reference loader's
  expected missing core-PNG messages are acceptable only when each is followed by its selected, hashed
  `MSFT_texture_dds` fallback.
