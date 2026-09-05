# Automatic SDFGI

Implementation baseline: `codex/universe-performance`, `f977f012f413`, 2026-09-05.
Capability preflight, the opt-in provider shell, and initial GPU resource plumbing are implemented. SDFGI rendering is
not yet available: Automatic SDFGI allocates and clears storage when it has an eligible anchor, but reports Environment
fallback and publishes no lighting. Shader entry points are explicitly ABI-only until their algorithm milestones.

## Reference

The local reference repository is `C:\Users\lllll\Documents\GitHub\godot`, beside EvoEngine under the GitHub directory.
The upstream repository is [godotengine/godot](https://github.com/godotengine/godot), pinned to
`34d06658a85845111a50db9e485ec4a0701d4298`. When in doubt, always inspect this pinned source and its callers/shader variants
before changing behavior or adding a workaround. The adjacent checkout is not a build dependency.

Godot-derived portions retain the [Godot MIT notice](licenses/Godot-MIT.txt). The pinned `COPYRIGHT.txt` assigns the core
SDFGI source files to the default Expat/MIT entry; the specifically listed effects-shader exceptions are outside this port.
Future copied source must name the originating file/commit and retain the corresponding notice. Host integration remains
subject to the EvoEngine license.
App installation includes the notice at `bin/licenses/Godot-MIT.txt`.

| EvoEngine source / planned destination | Pinned Godot source |
|---|---|
| `EvoEngine_SDK/src/SdfgiCapabilities.cpp` | `servers/rendering/renderer_rd/environment/gi.cpp::SDFGI::create`, `gi.h`, and SDFGI shader resource/workgroup declarations |
| `SdfgiSettings.hpp/.cpp`, `SdfgiRuntime.hpp/.cpp` (ownership shell) | Environment defaults; `render_forward_clustered.cpp::sdfgi_update`; later `gi.h::SDFGI` and `gi.cpp` cascade/update logic |
| `SdfgiResources.hpp/.cpp`, `SdfgiTypes.hpp`, `Shaders/Modules/EvoEngine/SdfgiTypes.slang` | `gi.h::SDFGIShader`, `gi.h::SDFGI::Cascade`, `gi.cpp::SDFGI::create`, and shader ABI records |
| `Shaders/Compute/SdfgiPreprocess.slang` (ABI only) | `shaders/environment/sdfgi_preprocess.glsl` |
| `Shaders/Compute/SdfgiDirectLight.slang` (ABI only) | `shaders/environment/sdfgi_direct_light.glsl` |
| `Shaders/Compute/SdfgiIntegrate.slang` (ABI only) | `shaders/environment/sdfgi_integrate.glsl` |
| `Shaders/Graphics/Vertex/SDFGI/SdfgiVoxelize.slang`, `Shaders/Graphics/Fragment/SDFGI/SdfgiVoxelize.slang` (ABI only) | ForwardClustered `_render_sdfgi` and `scene_forward_clustered.glsl::MODE_RENDER_SDF` |
| `Shaders/Compute/SdfgiGatherAbi.slang` (temporary layout check only) | `gi.h::SDFGIData` and the accepted six-set deferred adapter |
| Planned `Shaders/Compute/SdfgiDebug.slang`, `Shaders/Graphics/Vertex/SDFGI/SdfgiDebugProbes.slang`, `Shaders/Graphics/Fragment/SDFGI/SdfgiDebugProbes.slang` | `sdfgi_debug.glsl`, `sdfgi_debug_probes.glsl` |
| Planned `Shaders/Modules/EvoEngine/Sdfgi.slang` | `shaders/scene_forward_gi_inc.glsl::sdfgi_process` |

## Capability report

`QuerySdfgiCapabilities(cascade_count = 4, history_size = 30)` checks the reference's image/view pairs, dimensions, array
layers, transfer/storage/sampled/filter/atomic support, per-image allocation limits, feature bits, buffer ranges, descriptor
limits, push constants, and workgroups. It performs no field allocation. `Supported()` requires every check to pass;
`ToString()` names failed checks. Invalid configuration or an uninitialized platform is unsupported. Actual allocation
success and the full host pipeline's combined descriptor counts are checked when resources/pipelines are created.

From an initialized Python runtime, `PyEvoEngine.SdfgiCapabilityReport()` returns a dictionary with all checks, summary,
device/driver, reference commit, and effective ray-tracing/ray-query/BLAS/TLAS flags. Initialize with
`graphics_settings.use_ray_tracing = false`, or `RunDemoWindowless(..., enable_ray_features=False)` for the Python demo
runtime. All four effective flags must be false for SDFGI validation. These flags describe renderer permission to use the
facilities, not a claim that the physical GPU lacks them or that no optional Vulkan feature was enabled on device creation.

Run the focused `SdfgiCapabilities.*` GoogleTest filter for configuration, unsupported limits/formats, packed-view usage,
and one live preflight on the current GPU with RT disabled. No render image is needed for this milestone.

M0 verification (2026-09-05): SDK, Python binding, and `EvoEngine_Tests` built in `vs2026-x64-tests`, RelWithDebInfo.
All five focused tests passed on NVIDIA GeForce RTX 5070 (Vulkan driver value `2496774144`) with Vulkan and synchronization
validation enabled; the effective ray-tracing, ray-query, BLAS, and TLAS flags were all zero. No validation errors were
reported. This verifies preflight only, not field allocation or rendering; no image test or full test suite was run.

No DDGI atlas, update state, trigger, convergence, or gather behavior is an SDFGI input. Image validation later uses only
the Sponza Rendering demo at 2560 x 1440 with RT explicitly disabled and its own incremental baselines. Manual review is
required at the first stationary result, scrolling/camera integration, and final acceptance. All GPU maintenance uses the
normal main queue, with synchronization present as soon as each resource use exists.

## Provider shell and settings

Inspect the scene's Environmental Lighting asset and choose **Indirect GI provider > Automatic SDFGI**. Its tab exposes
the reference controls; no GI entity, volume, or pack is needed. Requested and effective providers are shown separately.
The shell always reports Environment until the later transport/gather milestones publish a complete field.

Defaults are four 128-cell cascades, minimum cell size 0.2, 75% vertical scale, occlusion off, 16 rays per probe,
30-frame history, four-frame dynamic-light cadence, bounce feedback 0.5, sky read on, energy 1.0, and both biases 1.1.
The reference ray/history/cadence choices are retained. Malformed numeric settings are diagnosed before field allocation.
Cascades, minimum cell size, vertical scale, occlusion, and history length recreate the field, matching Godot's
`RenderForwardClustered::sdfgi_update` reset condition.

One non-serialized runtime belongs to each active scene, never to its lighting asset or cameras. An enabled explicit scene
camera overrides selection; otherwise play/pause/step uses the enabled main camera, editing uses the canonical editor
Scene camera, and headless use falls back to the scene main camera. The override stores a scene-local camera entity
handle, not bounds. Invalid overrides fall through and report why; an absent anchor retains the previous position.
Preview, reflection, and injected utility cameras are not implicit anchors. Scene replacement, provider changes, scene
purge/clone, or incompatible layout settings discard the old CPU field state.
GPU owners remain retained by every submitting frame slot until its fence is recycled, including replaced fields.

`RenderAll` executes a separate scene frame graph once before ordinary cameras. The SDFGI maintenance hook uses Frame
scope and the normal Graphics queue; cameras do not call it. External frame passes follow `SceneGiComplete`, with the
old `DDGIVolumesComplete` name retained as a compatibility marker, not a DDGI execution path. SDFGI itself uses no DDGI
inputs or algorithms. Inactive authored volumes are not resolved, preflighted, or updated. DDGI pipelines now initialize
lazily when that provider is active; already-created immutable pipeline objects may remain cached. Imported DDGI GPU
inputs are retained by the submitting frame slot before a provider switch can release its runtime.

Legacy assets without a provider field retain Authored DDGI selection and their existing enable flag; newly constructed
lighting assets use the same convention (DDGI defaults disabled, hence effective Environment). Automatic SDFGI is never
selected implicitly. Selecting Authored DDGI in the UI or Python also enables its existing runtime flag. Inactive provider
settings/packs remain serialized. Unchanged legacy settings do not acquire an SDFGI settings block. The existing shared
lighting layout still receives existing missing-texture/one-record buffer fillers for inactive, unread bindings, not a
DDGI atlas or field. No additional provider-specific device requirement is imposed by those fillers.

Python exposes `IndirectGiProvider`, `SdfgiSettings`, `SdfgiVerticalScale`, `SetCurrentSceneGiProvider`,
`SetCurrentSceneSdfgiSettings`, and `GetCurrentSceneGiStatus` for the active scene. Setters change in-memory asset intent;
they do not save assets automatically. The status includes publication, maintenance count, anchor source/identity,
missing-anchor/override fallback, and the current fallback reason.

M1 verification (2026-09-05): SDK, Python binding, tests, and `EvoEngineEditor` built in `vs2026-x64-tests`,
RelWithDebInfo. All 15 focused provider/settings/anchor/frame-boundary, existing asset/DDGI-contract, and documentation
checks passed. The live frame-boundary test used the current RTX 5070 with all four effective RT flags false and Vulkan
and synchronization validation enabled; no validation errors were reported. No editor UI or image test was performed;
the settings UI is build-verified, and visual/manual acceptance remains at the agreed working-render checkpoints.

## GPU storage and initialization

The enabled field owns persistent cascade data and one shared scratch set, independent of ordinary camera count.
Defaults allocate 46 images (including a black sky filler), persistent solid-cell/dispatch/status buffers, and separate
per-frame cascade/gather/voxel metadata and static/dynamic light inputs. Unused cascade descriptor entries point to valid
compatible existing views and remain outside `max_cascades`; no partially-bound descriptors are required.

Packed light and probe images use `R32_UINT` storage and `E5B9G9R9_UFLOAT_PACK32` sampled views. Packed occlusion uses
`R16_UINT` storage and `R4G4B4A4_UNORM_PACK16` sampled views. Each pair shares one image and graph identity, with explicit
format lists, mutable/extended image usage, and per-view storage-only or sampled-only usage. All views use `GENERAL`
during maintenance. The reference signed SH formats, 128-cell grid, 17-probe axes, 25% solid-cell capacity, and
2312-by-136 atlas with `2C` layers are unchanged.

Preprocessing/direct-light/integration constants retain 48/48/112-byte layouts. Cascade records retain 48-byte stride.
The light record appends `host_photometry` at byte 112 for a 128-byte stride; no area-light field is repurposed.
Gather retains the 496-byte reference block and appends anchor origin/generation for 512 bytes. Status has separate
readiness/failure/generation/capacity fields. The six-set gather ABI layout is created only for the opted-in SDFGI field;
ordinary and excluded camera pipelines still have their existing five-set layouts and never bind the SDFGI set.

`SdfgiInitialize` imports explicit unmanaged images/buffers into the scene graph, clears every used layer, and leaves
readiness zero. Main-queue barriers cover prior shader/transfer/indirect users, clears, and subsequent consumers even
across separate graph/frame executions. Initialization alone never publishes a lighting result. There is no SDFGI
immediate submission, new queue, or device-idle wait. Allocation/layout/pipeline failure is diagnosed and leaves no field;
it is not retried every frame. Changing provider or incompatible settings creates a fresh initialization attempt.

`GetCurrentSceneGiStatus()` reports allocation and initialization-recorded state plus `active_allocation_bytes`, split
into field, scratch, upload, and diagnostic categories using actual VMA allocation sizes. These are active-owner totals,
not driver-pool usage or a peak that includes temporarily retiring old fields. Diagnostic allocation is currently zero;
payload staging is not yet owned by this initialization-only path. The ABI-only shaders are compiled/layout-checked but never dispatched by
normal rendering; focused GPU tests use explicitly compiled resource-check variants.

M2 verification (2026-09-05): SDK, Python binding, and tests built in `vs2026-x64-tests`, RelWithDebInfo.
All eight `SdfgiResources.*:SdfgiRuntime.*` checks passed (2.98 seconds), including a forced partial-allocation failure,
two main-queue clear/read submissions on one field, packed RGB9E5/RGBA4 reads, all SH-history layers, ABI sentinels,
six-set descriptor limits, and create/destroy repetition. The RTX 5070 run had all effective RT facilities disabled,
with Vulkan and synchronization validation enabled and no reported errors. All 21 exported shader variants passed
`spirv-val --target-env vulkan1.3`; disassembly confirms 128-byte lights, 48-byte cascades, and gather offsets 496/508.
Actual active-owner allocations: field 294,988,944 bytes, scratch 94,085,120 bytes, input 1,181,696 bytes, diagnostic zero.
No image, editor UI, or full-suite run was performed; this is storage/ABI evidence, not demonstrated lighting parity.
