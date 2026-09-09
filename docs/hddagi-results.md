# HDDAGI implementation results

Historical H6 baseline: the later GI refinement removes half-resolution HDDAGI. Half/full comparisons here describe the recorded H6 revision; use that revision to reproduce both modes. The current benchmark script always uses full resolution and has no `--full-resolution` argument.

H0–H6 implement the pinned core Godot HDDAGI as an opt-in third provider. SDFGI remains the default. This report records correctness checks and observed behavior; it does not establish parity with a Godot reference render or a quality improvement over the other providers.

## Configuration and provenance

- Godot HDDAGI: `da1410fa3516d08cc31b6e86bd6673b9ce776316`; existing SDFGI: `34d06658a85845111a50db9e485ec4a0701d4298`.
- EvoEngine comparison binaries: H5 `db9e2ad257203a32ead519d6adf8b5f9c40a3d92` plus H6 timing/memory instrumentation in this change. No transport shader changed after H5. Manifests retain dirty-tree provenance, resource revision and binary hashes.
- NVIDIA GeForce RTX 5070, driver 595.71 (Vulkan encoded version 2496774144), Windows, RelWithDebInfo. Benchmarks use validation OFF; GPU tests use core/synchronization validation ON. Results are from one device.
- Installed runtime: `C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/python`. `PyEvoEngine.cp315-win_amd64.pyd` SHA-256: `557566f1b53e78f73257cafb48a4c2865d3853de851a457a744b8cbee58ed119`; SDK DLL: `56801b8b912e813edd64eee946b64a0a8e13962a8cd72ae64f2459e52b6999ae`.
- Rendering/Sponza scene, 2560×1440 viewport, shared 33×17×33 probes, four cascades, 0.8 base probe interval, 100% Y scale. Default HDDAGI uses 1280×720 GI; full HDDAGI and SDFGI use 2560×1440 GI. HDDAGI uses eight voxels per probe interval, history 12, light cadence 4, reflection filtering off. Full settings are retained in each manifest; equal coverage does not imply equal sampling quality.
- HDDAGI and SDFGI run with RT, ray queries, BLAS and TLAS disabled. DDGI is a separate RT-enabled comparison. Authored dynamic reflection-probe updates and normal postprocessing remain active for all providers.

## Measurement method

Each provider configuration runs six workloads, three repetitions of 300 measured frames each, with 120 warmup frames before every repetition. Slow motion makes one sinusoidal cycle with X amplitude 1 and Z amplitude 4; fast motion makes ten. Geometry translates all 524 static mesh owners by 0.25 along X and back every 30 frames. Material edits alternate 156 unique static materials between half and original color; light edits alternate directional brightness between half and original. These are broad edit stress cases, not one-object edits.

GPU samples are matched to application frame IDs and drained after each interval. The GPU span covers the instrumented frame interval; individual stages can nest and must not be summed. Upload stage scopes cover their recorded barrier segment, not an independent transfer-inclusive total. Loop wall time excludes diagnostic status polling and edit API calls, so it is not observer-inclusive end-to-end application time. Timestamp collection and memory polling stay enabled; these are comparative development measurements, not shipping performance claims. The original raw manifests incorrectly say validation remained enabled: the verified production CMake cache has `EVOENGINE_ENABLE_GRAPHICS_VALIDATION=OFF`, while the test cache has it ON. This report corrects that attribution; raw measurements are unchanged.

VMA allocated and reserved block bytes are sampled every measured frame. They include the entire scene, host-visible GPU allocations and retained resources; they exclude driver-owned allocations and are not HDDAGI-only memory. Raw records retain stage median/p95/max, frame samples, live/peak allocations and block bytes.

After each matrix, motion, geometry, material and lighting changes are captured at exactly 1, 12 and 120 application frames. The 120-frame image is an observation endpoint, not converged ground truth.

## Timings and memory

The matrix contains 72 runs and 21,600 measured frames. Values below are **GPU span median / p95 in milliseconds**, each aggregated as the median of three per-run statistics. Full per-stage and Loop wall-time median/p95/max data are in [the machine-readable results](hddagi-results.json).

| Workload | HDDAGI half | HDDAGI full | SDFGI | DDGI RT |
|---|---:|---:|---:|---:|
| Stationary | 19.15 / 19.45 | 19.83 / 20.05 | 19.66 / 19.87 | 23.54 / 23.78 |
| Slow motion | 20.99 / 25.38 | 22.06 / 26.14 | 21.26 / 25.95 | 23.59 / 28.43 |
| Fast motion | 22.36 / 26.59 | 23.70 / 27.43 | 23.49 / 28.89 | 25.68 / 30.21 |
| Geometry edits | 23.23 / 26.56 | 24.20 / 27.32 | 22.97 / 23.91 | 27.56 / 28.91 |
| Material edits | 23.33 / 26.29 | 24.00 / 27.17 | 22.78 / 23.69 | 27.22 / 28.48 |
| Light edits | 22.25 / 22.98 | 22.28 / 23.89 | 23.05 / 23.55 | 27.46 / 28.50 |

Full-resolution HDDAGI is approximately tied with SDFGI while stationary (0.17 ms higher here), and has higher geometry/material-edit costs. Default half-resolution HDDAGI is cheaper in the motion cases, with a different reconstruction resolution. DDGI is slower in this matrix, but also has different sampling and RT costs. Three sequential repetitions on one device do not establish a general ranking; clocks, thermal state and temporal scene state were not locked. No universal speedup is claimed.

| Configuration | Peak VMA allocated GiB | Peak reserved GiB | Worst measured GPU span ms |
|---|---:|---:|---:|
| HDDAGI half | 6.813 | 7.029 | 37.80 |
| HDDAGI full | 6.859 | 7.029 | 38.61 |
| SDFGI | 6.597 | 6.792 | 54.34 |
| DDGI RT | 7.014 | 7.299 | 30.78 |

These peaks cover the measured intervals, not startup or arbitrary future scene edits. HDDAGI uses more whole-application allocated memory than SDFGI in this scene. Worst frames are isolated observed spikes, not p95 values.

Stationary HDDAGI half-resolution stage medians are 0.317 ms for receiver surface evaluation, 0.308 ms for camera gather, 0.453 ms for direct lighting, 0.286 ms for integration and 0.196 ms for probe filtering. Full-resolution camera gather rises to 0.894 ms while surface evaluation remains 0.315 ms. The separate post-GTAO material evaluation therefore has a measured cost; no shared renderer cache was introduced. SDFGI probe process/store are 1.385/0.144 ms; DDGI trace/update are 2.599/1.711 ms. These stage boundaries and algorithms differ and are not interchangeable work units.

Raw frame records, 48 edit-response PNGs and stationary/recovered captures remain in the local ignored `tasks/h6-hddagi-timed`, `tasks/h6-hddagi-full`, `tasks/h6-sdfgi` and `tasks/h6-ddgi` directories. The compact fixture is in `tasks/h6-compact2`. The report and aggregate JSON are tracked; these larger local artifacts are not included in the commit.

## Correctness and regression evidence

- H5: 80 focused tests passed, including RT-disabled transport, odd/tiny/multiple cameras, resolution/filter switching, receiver classification, static/dynamic/immediate reflection captures and provider/layout replacement during pending capture faces.
- H6 broad run: 813 of 821 tests passed initially. The SDK shader-inventory failure was fixed by compiling its actual mode variants, and its rerun passed: 143 entrypoints, 160 variants, zero compilation failures. Seven unrelated failures remain listed below; the full suite is not green.
- SPIR-V validation: 159 of 160 SDK variants passed, including all 22 HDDAGI variants. The unchanged legacy fragment `Graphics/Fragment/PostProcessing/SSRCombine.slang` fails because it declares two push-constant blocks. The active compute SSRCombine path passes. This legacy shader was not changed.
- Existing image baselines were not accepted or modified. All four DDGI raster-path variants passed their existing 29 dB / 0.94 thresholds: PSNR 98.1938, 40.3366, 31.7486, 31.2206 dB; SSIM 1, 0.997459, 0.988376, 0.985752. SDFGI passed its 30 dB / 0.95 thresholds with 56.6283 dB / 0.999498.
- The compact GPU fixture covers a one-voxel-thick wall, corner, masked checker, emissive panel, curved metallic surfaces at roughness 0/0.5/1 and a moving receiver-only sphere. Receiver movement leaves voxel-update count unchanged; emission removal triggers an update. Images remain finite and transport ready. Half/full GI and reflection filtering are exercised. Its three images were visually inspected.
- The separate installed RT-disabled 1440p capture completed with core and synchronization validation active, no disabled checks and no findings. `tasks/h6-installed-validation.log` records the layer's explicit startup confirmation; its manifest records ready transport, zero failure flags and binary hashes. Validation was enabled only for that process using `VK_INSTANCE_LAYERS=VK_LAYER_KHRONOS_validation`, `VK_LAYER_PATH=C:/VulkanSDK/1.4.341.1/Bin` and `VK_LAYER_SETTINGS_PATH` pointing to a file with `khronos_validation.validate_core=true`, `validate_sync=true`, `debug_action=VK_DBG_LAYER_ACTION_LOG_MSG`, `log_filename=stdout`, and `report_flags=error,warn,perf,info` (all keys have the `khronos_validation.` prefix). The production cache remains OFF. The capture command was `python Scripts/capture_hddagi_transport.py --module-dir out/install/vs2026-x64/python --resources tasks/h6-validation-resources --output tasks/h6-installed-validation --frames 120 --beauty`.

The seven remaining failures concern unchanged paths relative to pre-HDDAGI `3e7e3fc143498eae63f01c37b9256953adfa5c13`:

| Test | Finding |
|---|---|
| BistroDemoScript.DemoSceneAlignsRootToReferenceCamera | Stale source-string expectation for fallback intensities |
| GltfRayTracingMaterial.BistroParityCaptureDisablesUnrelatedStateAndLogsCounts | Stale DDGI/fallback source strings |
| ShaderCache.EcoSysLabNativeModulesCompileAndReflectGpuAbi | Expected push sizes 64/36/40 versus reflected 68/40/44 |
| ShaderCache.ProductionEcoSysLabNativeSlangInventoryCompiles | Expected 123 stages versus actual 134 |
| PlatformFrameScheduling.ProtectsMutableResourcesAcrossFrameSlots | Source-string expectation does not account for line wrapping |
| RenderingDocumentation.StaysWithinReadingBudget | Existing rendering guides exceed their word budgets |
| SerializationRegistry.FirstPartyPostProcessingAssetsEnableSsrProductionDefaults | External project asset YAML conversion failure |

These assertions/assets were not rewritten to make the run pass. Unrelated dirty resource submodules were preserved. Logs, XML and the unchanged-path audit are under `tasks/h6-regression.*`, `tasks/h6-inventory.*`, `tasks/h6-shader-inventory/validation.json`, `tasks/h6-goldens.*` and `tasks/h6-unrelated-regressions.json`.

## Visual findings and limits

The compact fixture shows the masked surface and receiver movement correctly; emissive color disappears after its removal. Sharp curved reflections retain visible voxel blockiness. Broad geometry/material invalidation can visibly reset indirect illumination before it rebuilds; the HDDAGI geometry frame-1 capture is substantially darker than its frame-120 endpoint, unlike the corresponding SDFGI pair. Faster hierarchy updates do not remove probe visibility, reconstruction or temporal artifacts. Sponza's bright yellow indirect illumination and bands on its reflective spheres also appear with SDFGI; local reflection probes and SSR remain in the composition, so those images cannot isolate HDDAGI reflection quality. DDGI's stationary image has substantially less yellow indirect illumination. Neither brightness nor the visible differences establish accuracy or an improvement.

There is no Godot-versus-EvoEngine image equivalence test, ground-truth irradiance comparison, multi-vendor run or exhaustive arbitrary-ray proof. Contributor eligibility remains SDFGI's supported rigid-mesh subset; this adds no deforming/skinned contribution algorithm. Camera-visibility scheduling remains disabled. Conservative cache/history invalidation favors correctness over edit convergence. No speculative screen-probe or advanced-denoising extension is included.

## Reproduction

Build the test target with `cmake --build out/build/vs2026-x64-tests --config RelWithDebInfo --target EvoEngine_Tests --parallel 8`. Run the test executable from `out/build/vs2026-x64-tests/EvoEngine_Tests/RelWithDebInfo` with `--gtest_filter=-RenderingDemo.*` for the broad run. Run `--gtest_filter=RenderingDemo.RasterPathMatrixGoldenImage:RenderingDemo.SdfgiGoldenImage` with `EVOENGINE_ACCEPT_RENDER_BASELINE=0` for existing goldens. Set `EVOENGINE_HDDAGI_CAPTURE_DIRECTORY` to an absolute disposable output path and run `--gtest_filter=HddagiCamera.CompactVisualFixturesAndReceiverOnlyMovementWithoutRt` for the compact fixture.

Install all enabled applications:

```powershell
python Scripts/install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --jobs 8
```

The final invocation succeeded (`tasks/h6-install-final.log`); the test target and editor were built before their captures. User visual feedback was requested during H5 and remains optional; the behavior checks and image inspection recorded here were completed during implementation.

For each comparison, copy `Resources/EvoEngine-DemoProjects/Rendering/Assets` into a fresh disposable `<resources>/EvoEngine-DemoProjects/Rendering/Assets` folder, then run sequentially:

```powershell
python Scripts/benchmark_hddagi.py --module-dir out/install/vs2026-x64/python --resources <resources> --output <output> --provider Hddagi
```

Repeat with a fresh resource/output directory for `--provider Hddagi --full-resolution`, `--provider Sdfgi`, and `--provider Ddgi`. Do not run competing GPU workloads. The script refuses a previously generated project and requires at least 300 frames and three repeats. It records provider/failure checks, settings, frame timings, memory, PNGs and SHA-256 provenance. Benchmark captures use the installed Python runtime; the separately built/installed editor is `C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/bin/EvoEngineEditor.exe`.
