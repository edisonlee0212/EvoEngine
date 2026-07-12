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

#### M6 Baseline and Approval-Pending Targets

The fresh one-run M6 slice on the NVIDIA GPU/driver recorded in `report.json` produced the following current EvoEngine
baseline. These are descriptive observations, not estimates of run-to-run variance:

| Lane | Accumulation wall | Wall throughput | Path GPU median | Queue-submit median | Startup BLAS GPU |
| --- | ---: | ---: | ---: | ---: | ---: |
| RTX, SER off | 10.4163 s | 44.946 Msample/s | 66.085 ms | 0.1322 ms | 399.757 ms / 2,909 builds |
| RayQuery | 9.7394 s | 48.070 Msample/s | 62.986 ms | 0.1187 ms | 400.764 ms / 2,909 builds |
| forced query-only | 9.7549 s | 47.994 Msample/s | 61.933 ms | 0.1297 ms | 401.202 ms / 2,909 builds |

All three lanes recorded two startup TLAS builds totaling about `0.366-0.368 ms`, zero capture-window BLAS/TLAS work,
and final device-local VMA allocation of about `6.086e9` bytes (`5.668 GiB`). RayQuery and forced query-only were
bit-exact. RTX versus RayQuery relative L2 was `0.008947`. The following targets remain proposals until explicitly
approved:

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
fresh output-local shader-cache directory and ImGui ini path, so root working-directory state cannot reuse stale SPIR-V or
leak into a capture. Dry runs write `manifest.dry-run.json` and do not replace measured evidence.

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
