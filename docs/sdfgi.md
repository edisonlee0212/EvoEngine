# Automatic SDFGI

Implementation baseline: `codex/universe-performance`, `f977f012f413`, 2026-09-05.
Capability preflight, provider ownership, GPU storage, placement/scene inputs, static voxelization, SDF/occlusion
preprocessing, voxel lighting, probe transport/storage, and deferred gather are implemented. M9 adds automatic scrolling
and retained history; focused validation passes and the user has accepted movement review. Eligible opaque/masked raster cameras share
one complete field and use Environment fallback otherwise. M0-M8a are complete, committed as separate milestones, and the
user accepted the occlusion-on stationary result. Occlusion stays on by default and Godot's sharp-reflection path remains
enabled. M10-M12 have not started.

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
| `SdfgiScene.hpp/.cpp` | `gi.cpp::SDFGI::{create,update,get_pending_region_data,update_cascades,pre_process_gi}`, ForwardClustered `_render_sdfgi`/`_fill_render_list`; dedicated EvoEngine scene/material/light adapter |
| `SdfgiResources.hpp/.cpp`, `SdfgiTypes.hpp`, `Shaders/Modules/EvoEngine/SdfgiTypes.slang` | `gi.h::SDFGIShader`, `gi.h::SDFGI::Cascade`, `gi.cpp::SDFGI::create`, and shader ABI records |
| `SdfgiPreprocess.hpp/.cpp`, `Shaders/Compute/SdfgiPreprocess.slang` | `gi.cpp::SDFGI::render_region`, its uniform sets, and `shaders/environment/sdfgi_preprocess.glsl`, including scroll variants |
| `SdfgiLight.hpp/.cpp`, `Shaders/Compute/SdfgiDirectLight.slang` | `gi.cpp::SDFGI::{render_static_lights,pre_process_gi,update_light}`, `LightStorage::light_get_aabb`, and `shaders/environment/sdfgi_direct_light.glsl` |
| `SdfgiProbe.hpp/.cpp`, `Shaders/Compute/SdfgiIntegrate.slang` | `gi.cpp::SDFGI::{render_region,update_probes,store_probes}` and `shaders/environment/sdfgi_integrate.glsl`, including scroll variants; host cubemap and diagnostic readback adapters |
| `SdfgiVoxelizer.hpp/.cpp`, `Shaders/Modules/EvoEngine/SdfgiVoxel.slang`, `Shaders/Graphics/Vertex/SDFGI/SdfgiVoxelize.slang`, `Shaders/Graphics/Fragment/SDFGI/SdfgiVoxelize.slang` | ForwardClustered `_render_sdfgi` and `scene_forward_clustered.glsl::MODE_RENDER_SDF`; host-only diagnostic plane readback |
| `Shaders/Compute/SdfgiGatherAbi.slang` (temporary layout check only) | `gi.h::SDFGIData` and the accepted six-set deferred adapter |
| Planned `Shaders/Compute/SdfgiDebug.slang`, `Shaders/Graphics/Vertex/SDFGI/SdfgiDebugProbes.slang`, `Shaders/Graphics/Fragment/SDFGI/SdfgiDebugProbes.slang` | `sdfgi_debug.glsl`, `sdfgi_debug_probes.glsl` |
| `SdfgiGather.hpp/.cpp`, `Shaders/Modules/EvoEngine/Sdfgi.slang` | `gi.cpp::SDFGI::pre_process_gi`, `shaders/environment/gi.glsl::{sdfvoxel_gi_process,sdfgi_process}` and its caller/uniform sets; shared-anchor/publication adapter |
| `Shaders/Modules/EvoEngine/SdfgiLighting.slang`, `Shaders/Compute/SdfgiPublish.slang` | Accepted EvoEngine scene-linear material/AO/reflection composition and GPU publication guard; no DDGI gather or state |

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
Environment is reported until a complete transport/gather generation has been recorded for publication. The GPU readiness,
failure, and generation checks remain authoritative before any camera samples the field.

Defaults are four 128-cell cascades, minimum cell size 0.2, 75% vertical scale, occlusion on, 16 rays per probe,
30-frame history, four-frame dynamic-light cadence, bounce feedback 0.5, sky read on, energy 1.0, and both biases 1.1.
The reference ray/history/cadence choices are retained. Malformed numeric settings are diagnosed before field allocation.
Occlusion on is a user-requested default difference from Godot (off), not a change to its visibility algorithm.
**Environmental Lighting > Automatic SDFGI > Use Occlusion** controls this serialized setting. Explicit saved false
values stay false; only new/default settings turn it on. It reweights neighboring probes by approximate visibility to
reduce leaks, including bounce feedback; it can suppress bright or dark hidden probes and may produce dark patches.
It is not a second ambient-occlusion multiplier and does not disable SDF transport/shadow tracing when off. Both states
still generate the reference occlusion volume. The checkbox tooltip notes field recreation and reconvergence on change.
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
per-frame cascade/gather metadata and static/dynamic light inputs. Voxel metadata is immutable per pending region/axis.
Unused cascade descriptor entries point to valid compatible existing views and remain outside `max_cascades`; the field
arrays need no partially-bound descriptors. Voxel materials use the host's existing partially-bound texture layout.

Packed light and probe images use `R32_UINT` storage and `E5B9G9R9_UFLOAT_PACK32` sampled views. Packed occlusion uses
`R16_UINT` storage and `R4G4B4A4_UNORM_PACK16` sampled views. Each pair shares one image and graph identity, with explicit
format lists, mutable/extended image usage, and per-view storage-only or sampled-only usage. All views use `GENERAL`
during maintenance. The reference signed SH formats, 128-cell grid, 17-probe axes, 25% solid-cell capacity, and
2312-by-136 atlas with `2C` layers are unchanged.

Preprocessing/direct-light/integration constants retain 48/48/112-byte layouts. Cascade records retain 48-byte stride.
The light record appends `host_photometry` at byte 112 for a 128-byte stride; no area-light field is repurposed.
Gather retains the 496-byte reference block and appends anchor origin/generation for 512 bytes. Status has separate
readiness/failure/generation/capacity fields. The six-set gather layout is created only for the opted-in SDFGI field;
eligible ordinary raster cameras use it after publication. No-SDFGI and excluded camera pipelines retain their existing
five-set layouts and never bind the SDFGI set.

`SdfgiInitialize` imports explicit unmanaged images/buffers into the scene graph, clears every used layer, and leaves
readiness zero. Main-queue barriers cover prior shader/transfer/indirect users, clears, and subsequent consumers even
across separate graph/frame executions. Initialization alone never publishes a lighting result. There is no SDFGI
immediate submission, new queue, or device-idle wait. Allocation/layout/pipeline failure is diagnosed and leaves no field;
it is not retried every frame. Changing provider or incompatible settings creates a fresh initialization attempt.

`GetCurrentSceneGiStatus()` reports allocation and initialization-recorded state plus `active_allocation_bytes`, split
into field, scratch, upload, and diagnostic categories using actual VMA allocation sizes. These are active-owner totals,
not driver-pool usage or a peak that includes temporarily retiring old fields. Upload includes immutable voxel material,
texture-info, region/axis buffers and their retained staging arenas. Diagnostic is zero unless a snapshot is requested.
Only the temporary gather ABI check is not dispatched by normal rendering; all preprocessing, lighting, integration,
scrolling, publication, and deferred variants have working bodies. Focused GPU tests also compile resource-check variants.

M2 verification (2026-09-05): SDK, Python binding, and tests built in `vs2026-x64-tests`, RelWithDebInfo.
All eight `SdfgiResources.*:SdfgiRuntime.*` checks passed (2.98 seconds), including a forced partial-allocation failure,
two main-queue clear/read submissions on one field, packed RGB9E5/RGBA4 reads, all SH-history layers, ABI sentinels,
six-set descriptor limits, and create/destroy repetition. The RTX 5070 run had all effective RT facilities disabled,
with Vulkan and synchronization validation enabled and no reported errors. All 21 exported shader variants passed
`spirv-val --target-env vulkan1.3`; disassembly confirms 128-byte lights, 48-byte cascades, and gather offsets 496/508.
Actual active-owner allocations: field 294,988,944 bytes, scratch 94,085,120 bytes, input 1,181,696 bytes, diagnostic zero.
No image, editor UI, or full-suite run was performed; this is storage/ABI evidence, not demonstrated lighting parity.

## Cascade placement and scene snapshots

CPU cascade centers follow Godot's creation rounding (`floor(position / probe_size + 0.5)`) and update truncation toward
zero, including negative coordinates. Cascades have four-cell drag margins, eight-cell shifts, and the exact reference
dirty-volume threshold (`dirty_volume > safe_volume / 2`). Entering X/Y/Z slabs are chipped to avoid duplicate coverage.
World bounds undo the vertical multiplier; cascade offsets remain in vertically scaled field coordinates and probe world
offsets are center cells divided by eight. Unused CPU cascade ABI records are zeroed.

Anchor identity changes are diagnosed explicitly but preserve only the overlap mapped by the reference movement rules.
Large moves select full redraw. The reference's per-axis early exit on a full-cascade shift is retained. Missing anchors
leave coverage stationary. The eight-cell stepping loops are evaluated with equivalent 64-bit arithmetic to avoid long
teleport loops and signed overflow; coordinates outside the reference integer range or finite float extent are rejected
before field allocation. No new relocation budget or scheduling policy is introduced.

`SnapshotSdfgiScene` runs once at the scene maintenance boundary. It reads scene component ownership, not camera culling,
render-instance array indices, DDGI signatures, or camera-dependent light buffers. Eligible entries are static, enabled,
ordinary filled-triangle `MeshRenderer`s with supported opaque/masked `Material` data. Dynamic rigid and deforming meshes
are receiver-only; transparent/transmissive, particles, strands, and splats retain forward fallback. Missing/empty data,
non-filled geometry, and invalid bounds are separately diagnosed. This uses existing entity mobility, not new GI tags.

The stable registry key is `(renderer handle, transform-owner entity handle)`. For `LodGroup`, the adapter uses its first
(base) LOD and the group's transform, matching the host's instance ownership and Godot's SDF voxel pass with mesh LOD
selection disabled. Camera LOD factors never enter snapshots, and alternate LOD renderers are not double-counted.
Repeated references with the same key describe one contributor. Unsupported custom render commands without an ordinary
scene mesh/material representation are not collected.

Snapshots retain old/new world bounds and transforms, mesh identity/revision/counts, base/emissive factors and textures,
UV mappings, color interpretation, masking/cutoff, and sidedness. Texture images/views/samplers are retained and the normal
texture-storage content signature captures relevant GPU upload generations. This is the generic texture contract, not a
DDGI signature or trigger. Bindless allocation indices, broad material revisions, metallic/roughness/normal-map controls,
and dynamic receiver transforms are not SDFGI payload identity. Opaque alpha-only edits do not invalidate occupancy;
masked base-texture edits conservatively affect coverage. Zero-emission materials do not track unused emissive textures.

Registry comparisons classify add/remove, transform/bounds, geometry, coverage, and payload changes. The union of old/new
bounds identifies affected cascades without building a giant union across distant objects. These are CPU diagnostics;
geometry reconstruction and lighting reactions are wired in their later milestones, not dispatched by this stage.

Directional lights always use the dynamic list, including static entities, as in the reference. Point/spot mobility uses
the existing entity static flag. The scene snapshot retains host linear color times brightness, shadow intent, position,
travel direction, effective range, all three distance-attenuation coefficients, and both cosine cone thresholds. It does
not apply camera exposure or sort by camera distance. Scene-level constant color or environment cubemap identity,
orientation, gamma, and energy are frozen alongside these inputs. M6 consumes the light snapshot; M7 consumes the sky snapshot.

`GetCurrentSceneGiStatus()` additionally exposes cascade centers/dirty state, pending slab offsets/sizes, anchor replacement,
eligible/excluded contributor counts, contributor change count, affected-cascade change masks, and static/dynamic light
counts. Change masks use `SdfgiChangeFlags` in `SdfgiScene.hpp`; the full inspector is still a later milestone.

M3 verification (2026-09-05): SDK, Python binding, and tests built in `vs2026-x64-tests`, RelWithDebInfo.
All 12 `SdfgiScene.*:SdfgiRuntime.*` checks passed (0.334 seconds), including 200 deterministic movement comparisons
against the pinned reference loops and a live CPU scene covering base LOD, bounds, mobility, materials, and lights.
The existing scene-frame test again ran on the RTX 5070 with all effective RT facilities disabled, Vulkan/synchronization
validation enabled, and no reported validation errors. No additional shader/GPU-storage, image, editor UI, or full-suite
run was needed for these CPU-only changes. Field publication and rendering remain disabled.

## Static voxelization and diagnostic capture

The scene graph records `SdfgiVoxelInputs`, one `SdfgiVoxelCascadeN` for each pending cascade, and completion before any
ordinary camera. Each cascade clears shared albedo/emission/anisotropy/facing scratch, then draws its intersecting static
contributors through three attachment-free orthographic views. There is no depth attachment, depth test, camera culling,
shadow dependency, or material evaluation in the ordinary opaque G-buffer. As in Godot's SDF pass, culling is always
disabled, including for materials that are single-sided in ordinary rendering. Facing uses the normalized interpolated
geometry normal, not the normal map or a back-face-flipped BRDF normal.

The port retains RGB555 plus solid-bit albedo, RGBE8985 emission, six 5-bit emission-anisotropy weights, and atomic six-axis
facing bits. Only base color, vertex color, UV0/UV1 and transforms, base/emissive textures, and masked alpha are evaluated.
Native host texture sampling/color interpretation is reused, not DDGI material/signature behavior. Metallic, roughness,
normal, and other receiver BRDF textures are not voxel payload. Explicit out-of-grid discard implements the reference's
out-of-range storage-write behavior. Missing GPU geometry/texture readiness is diagnosed; incomplete initial voxelization
is retried without publishing a field.

Necessary host adaptations against `_render_sdfgi` and `MODE_RENDER_SDF`:

- Use the existing packed vertex/index buffers and GLTF material ABI in set 0. An immutable SDFGI descriptor version binds
  only used textures plus frozen material/texture-info buffers; it does not borrow camera-selected material indices or
  DDGI resources. Godot's voxel set-1 binding numbers are unchanged.
- Replace Godot's render-scene UBO with a 96-byte per-region/axis block: projection columns, cascade minimum/cell size,
  and region offset/vertical multiplier. A 112-byte draw push block carries model columns, inverse-transpose normal basis
  at byte 64, and material index at byte 100. This is host plumbing, not a coordinate/packing algorithm change.
- Record uploads, scratch clears, all axes, and optional diagnostic copies on the normal main queue. Immutable inputs,
  sampled images/views/samplers, and diagnostic buffers are retained through the submitting frame fence. Cross-frame
  barriers include prior shader/transfer readers before scratch reuse. No maintenance path uses immediate submission.

`RequestCurrentSceneSdfgiVoxelDebug(cascade=0, slice=64)` queues an explicit diagnostic, and
`CaptureCurrentSceneSdfgiVoxelDebug(path)` exports it after a rendered frame. Capturing requests a full rasterization of
the chosen cascade because M4 scratch is shared, not persistent. Three planes are copied immediately after that cascade,
before another clears scratch. Readback waits only when explicitly exporting; normal rendering does not wait for it.
This early host-only view precedes the later SDF/probe inspector. At M4 no field was published; the current M9 pipeline
reconstructs retained cells after rasterization before preprocessing and publication.

The PNG is always 2560 x 1440. Columns are albedo, tone-mapped emission, facing, and occupancy; rows are X-, Y-, and
Z-normal slices. Within a row, horizontal/vertical axes are cyclic `(Y,Z)`, `(Z,X)`, `(X,Y)`, with positive vertical upward.
Each 128-cell plane uses 3 pixels/cell. Albedo uses display gamma 2.2; emission uses `radiance/(1+radiance)` and that gamma.
Facing maps X/Y/Z to red/green/blue; positive bits are full intensity and negative bits 40%; multiple bits combine.
This is a surface-payload diagnostic, not a beauty image or a DDGI comparison.

To reproduce using a freshly built Python binding, copy `Resources/EvoEngine-DemoProjects/Rendering/Assets` into a fresh
disposable `<capture-resources>/EvoEngine-DemoProjects/Rendering/Assets` folder without a `Rendering.eveproj`, then run:

```powershell
python Scripts/capture_sdfgi_voxels.py --module-dir out/build/vs2026-x64-tests/PythonBinding/RelWithDebInfo --resources <capture-resources> --output <output.png>
```

The script creates the Rendering demo in that disposable project, explicitly disables all RT facilities, selects
Automatic SDFGI, resizes the main raster camera to 1440p, waits for actual asset readiness, and captures one voxel snapshot.
It prints GPU/driver/reference, settings-derived cascade state, memory, participation, readiness, and fallback provenance.
It rejects previously generated projects and empty contributor sets instead of accepting an empty scene as Sponza.

M4 image evidence (2026-09-05): current RTX 5070, driver `2496774144`, all four effective RT flags false, Vulkan and
synchronization validation enabled. Default four-cascade settings, anchor `(0,0,3)`, cascade 0 at `(0,0,16)` grid center,
slice 64, 524 static contributors, one dynamic and two deforming exclusions. The 1440p slice capture was inspected and
contains explainable Sponza occupancy/color/facing and an emissive sample, with no validation errors. Local artifact:
`tasks/m4-sponza-voxels.png`, SHA256 `b452d18df60e016443ebcb7d09465f9396033e8b8d43931cc18865f819799323`.
This establishes only the voxel diagnostic baseline; distance fields, transport, ordinary lighting, and user manual
acceptance remain later milestones. Application installation remains at implementation completion.

M4 technical verification: SDK, Python binding, tests, and editor built in `vs2026-x64-tests`, RelWithDebInfo. The combined
resource/voxel GPU check passed 2/2 tests in 3.61 seconds with RT disabled and Vulkan/synchronization validation. The voxel
fixture covers all axes, RGB555/RGBE8985/anisotropy/facing packing, vertex color, UV1 with transformed masked texture,
opaque alpha, reversed winding, two-sided geometry, repeated clear/emission reset, and retained diagnostic snapshots.
The initial fixture needed the host's geometry-upload readiness barrier; no maintenance wait was added. A Python status
list-conversion error found by the Sponza run was corrected. The final capture and GPU runs have no validation errors;
the host full vertex layout only emits harmless unused-tangent attribute performance warnings.
Both final voxel SPIR-V stages pass `spirv-val --target-env vulkan1.3 --scalar-block-layout`, matching the host GLTF scalar
buffer layout enabled by device creation. Inspection confirms draw offsets 0/64/100/104 and scalar normal-array stride 4.
Unchanged earlier CPU evidence is reused; there was no full-suite run, ordinary beauty comparison, or manual checkpoint.

## Stationary distance fields, occlusion, and capacity safety

Each full-cascade voxel pass now runs the reference half-resolution preprocessing sequence before shared scratch is reused:
initialize half 0; jump-flood steps 32, 16, 8, 4, 2, 1; upscale half 0 into full 0; one optimized full-grid step into full 1;
eight occlusion dispatches; STORE; copy dispatch counts to the indirect buffer; clear persistent light/anisotropy textures.
Steps 8 and below use the reference 8-cubed shared-memory optimization. The descriptor ping-pong parity follows the actual
executed `render_region` path, including STORE reading full 1. Occlusion parity comes from cascade probe-world offsets,
with the eight nibbles packed in reference order into the two X halves of the persistent RGBA4-compatible image.
All uploads, dispatches, copies, clears, and internal/cross-frame barriers remain on the normal main queue.

Distance is reference `distance + 1` (zero at a solid), stored as R8 UNORM. The test permits the two adjacent integers
allowed by Vulkan's [floating-point to normalized fixed-point conversion](https://docs.vulkan.org/spec/latest/chapters/fundamentals.html#fundamentals-fixedfpconv),
and requires exact integer distances. It does not change shader math to force a device-specific rounding choice.
The pinned STORE tests nearest-position XYZ without its validity W: a completely empty field consequently retains one
zero-payload origin sentinel and its distance ramp. This reference behavior is preserved, not silently corrected.

STORE retains the raw attempted compact count, caps indirect groups to storage capacity, bounds every compact write,
and atomically marks the whole-field failure status if the reference 25-percent list overflows. CPU diagnostics consume
counts/status only after a submitting frame fence is recycled, or during explicit capture; normal maintenance never
waits for readback. The renderer diagnoses solid overflow and retains Environment fallback. No producer in this milestone
publishes lighting; subsequent light/transport/gather consumers must reject this GPU flag before consuming truncated data.
The stable type/handle light-list limiter is also tested at the reference 1024-static/128-dynamic capacities. M6 applies
it after per-cascade eligibility filtering and exposes excluded counts when actual light uploads are introduced.

`RequestCurrentSceneSdfgiPreprocessDebug(cascade=0, slice=64)` copies persistent SDF/occlusion planes without rerasterizing;
`CaptureCurrentSceneSdfgiPreprocessDebug(path)` waits explicitly and exports one 1440p PNG. `GetCurrentSceneGiStatus()`
reports completed preprocessing masks, raw compact counts, bounded indirect XYZ, GPU failure status, and diagnostics.
Capture errors are separate from algorithm failure. Readback snapshots survive their submitting frame fences and are
counted as diagnostic memory. M9 reconstructs partial/slab updates through the same preprocessing pipeline.

Use the M4 disposable Sponza resource setup and capture command with `--view preprocess`. The nine columns are SDF followed
by occlusion channels 0 through 7; rows are X-, Y-, and Z-normal slices with the same cyclic axes as the voxel view.
SDF display is `min(encoded_byte * 8, 255)`; occlusion displays each 4-bit value times 17, white meaning unoccluded.
Each cell occupies two pixels; the fixed 2560 x 1440 output includes letterboxing.

M5 image evidence (2026-09-05): inspected `tasks/m5-sponza-preprocess.png`, SHA256
`9b8a9673b681155755ed097ad0feabcbf2b93684917bf530be558235f65c9bee`. Same RTX 5070/driver/default settings,
RT pipeline/query/BLAS/TLAS disabled, 524 static Sponza contributors, cascade 0/slice 64. All four cascades completed:
compact counts 68,792 / 19,148 / 3,926 / 886, indirect groups 1,075 / 300 / 62 / 14, GPU failure flags zero.
Allocation bytes: field 294,988,944; scratch 94,085,120; upload 1,590,800; diagnostic 245,840 for this capture.
There were no Vulkan or synchronization validation errors. This is a new SDF/occlusion diagnostic baseline, not a beauty
image, a DDGI comparison, or user manual acceptance.

M5 verification also covers single-cell distance/compact payload and 26 neighbor bits, empty-field behavior, all eight
occlusion nibbles, indirect execution, repeated preprocessing, and forced dense overflow with a guard canary. The focused
GPU test passed in 1.13 seconds. Its first readback used a generic 2D-sized image-copy convenience overload; the test now
uses explicit 3D regions with full-sized buffers. No production readback uses that overload. SDK, Python binding, tests,
and the editor were built in `vs2026-x64-tests`, RelWithDebInfo. No full-suite or additional image matrix was run.
Final combined resource/preprocess/light-capacity checks passed 3/3 in 3.98 seconds, without validation errors; all seven
implemented preprocessing SPIR-V variants passed `spirv-val --target-env vulkan1.3 --scalar-block-layout`.

## Voxel direct lighting

Static positional lights bake into the compact cell payload. All directionals, including entity-static directionals, and
dynamic point/spot lights populate persistent RGB9E5 lighting and six anisotropy weights through Godot's dynamic variant.
The shader retains reference SDF visibility traversal, bias, cross-cascade advancement, RGBE8985 static encoding, RGB9E5
dynamic encoding, and 26-neighbor fill. `has_shadow` remains in the ABI but does not bypass reference SDF visibility or
introduce camera shadow-map sampling. Area/projector code is excluded as agreed.

Godot's directional-first light classification, cascade AABB comparison, positional maximum cascade index 2, and one-cell
subset per update phase are preserved. New/reseeded representations process every cell; subsequent dynamic updates use
`scene_frame % light_update_frames` and the configured 1/2/4/8/16 increment. Directionals reach all cascades. Point AABBs
and transformed spot AABBs follow reference `light_get_aabb`; the cascade comparison remains the reference's comparison
before uploaded positional Y adjustment. No camera culling or camera-dependent light ordering is used.

The accepted host photometry adapter uses scene-linear `diffuse * diffuse_brightness`, energy multiplier 1, native range,
and `1 / (constant + linear*d + quadratic*d*d)`. The named 16-byte extension holds these coefficients and cosine inner
cone; reference `cos_spot_angle` holds cosine outer cone. Spot intensity is the host's clamped linear inner/outer ramp,
verified against `Lighting.slang` and `RenderInstanceStorage.cpp`, not inferred from the host UBO's misleading names.
World-space distance and cone direction drive host photometry; SDF ray positions/directions retain reference field space.
No camera exposure, Godot physical-unit conversion, DDGI light adapter, or new per-light GI controls are introduced.

Necessary host implementation of the accepted static-light reseed contract: each cascade retains an `UnlitCells` buffer,
a bounded copy of STORE's unbaked compact records. It uses the same 524,288-record capacity and 16-byte stride, adding
8 MiB per cascade (32 MiB for defaults). Copying after preprocessing keeps emission, anisotropy, geometry, and neighbor
bits aligned without extra descriptor bindings or a second seed shader. On static-light edits/removal, copy that seed
back before reinjection; unchanged static lighting is never added again. This refresh does not rerasterize geometry or
rebuild SDF/occlusion. M10 will refresh the seed for the accepted occupancy-preserving material-edit extension.
Default field allocation becomes 328,543,376 bytes; scratch is unchanged. Per-frame light staging is included in upload
memory, and optional debug plane buffers in diagnostic memory.

Per-cascade lists are selected deterministically by type/handle and bounded to 1024 static or 128 dynamic inputs. Counts
excluded by capacity are visible in `GetCurrentSceneGiStatus().cascade_lights`. Any light-list overflow skips injection,
diagnoses the failure, and requires whole-provider Environment fallback until recovery; M8 integrates that CPU-known
publication gate. Every direct-light invocation independently rejects the GPU whole-field overflow flag before reading
compact cells, so delayed diagnostic readback cannot permit out-of-capacity accesses. Uploaded inputs and descriptors
are frame-slot versions retained through their fences. Uploads, static reseeding, both injection variants, and debug reads
are ordered on the main queue, including previous-frame readers. No dedicated compute queue or immediate maintenance
submission is used. The atlas binding starts as valid cleared black data. Bounce feedback stays zero until the first
complete transport atlas has been recorded, then uses the configured default 0.5 on subsequent frames (M7).

`RequestCurrentSceneSdfgiLightDebug` and `CaptureCurrentSceneSdfgiLightDebug` expose the persistent per-cascade light volume.
Use the disposable Sponza driver with `--view lighting` if a light diagnostic is needed. Output remains 2560 x 1440, with
X/Y/Z slice rows, total RGB9E5 light in column 0 and its six anisotropy-weighted lobes in columns 1 through 6. Display uses
`radiance/(1+radiance)` then gamma 2.2. Requests do not rebuild geometry; only explicit PNG export waits for readback.

The focused M6 fixture exercises directional/point/spot response, native attenuation/cone/range, positional cascade limit,
entity-static directional classification, dynamic cadence, repeated static injection/edit/removal, emission replacement,
zero-energy output, SDF-wall visibility with `casts_shadow=false`, all 26 neighbor writes, GPU solid overflow, light-list
overflow/recovery, and retained diagnostic planes. It uses the current RTX 5070 with all effective RT facilities disabled;
no Sponza image, additional scene image, or DDGI comparison is needed for these isolated checks.

M6 final verification (2026-09-05): SDK, Python binding, tests, and editor built successfully in `vs2026-x64-tests`,
RelWithDebInfo. Four combined lighting/resource/preprocess tests passed in 4.82 seconds, including the STORE-to-unlit-seed
copy and retained light-debug planes. Both direct-light SPIR-V variants passed `spirv-val --target-env vulkan1.3
--scalar-block-layout`; resource reflection/readback verifies the 128-byte light stride and extension. Vulkan and
synchronization validation reported no errors. The first zero-energy fixture reused an already-consumed upload batch;
re-adding its changed seed fixed the fixture without modifying Godot's light equations. No full suite or app installation
was run; installation and the agreed manual checkpoints remain later milestones.

## Stationary probe transport and storage

M7 ports the executed PROCESS/STORE bodies from the pinned `sdfgi_integrate.glsl` and their `gi.cpp` callers. Every frame
processes the full 17-cubed probe grid in every cascade, then stores all cascades. Deterministic world-hashed Vogel
directions interleave the reference ray count across the history cycle. Cross-cascade SDF sphere tracing, ray bias,
SDF-gradient surface normal, and six-lobe voxel radiance remain reference equations. No DDGI ray/update/convergence path
is used. The reference signed 16-coefficient SH values use 10 fractional bits and int16 saturation; each new history
layer subtracts the old layer from the int32 average before addition. A complete cycle replaces every old sample.
STORE applies the reference SH reconstruction and diffuse band factors, produces 6-by-6 octahedra with mirrored borders,
and writes irradiance to layer `c` and rough-specular radiance to layer `C+c`. Reference normalization factors and
octahedral texel positions are not retuned.

The accepted host sky adapter samples the ready scene-level radiance cubemap, not a local reflection capture or a
diffuse-prefiltered map. It uses explicit `min(2, mip_count-1)` LOD, host rotation in radians, host `pow(rgb, 1/gamma)`,
and indirect-source energy. Constant-color mode bypasses cubemap gamma, matching the native host environment path;
sky-disabled misses are black. Camera exposure never enters the field. The cubemap replaces Godot's octahedral sky;
the otherwise-unused oct-border float2 at push offset 96 is named `sky_lod_inverse_gamma` for the host LOD/gamma metadata.
The push block remains 112 bytes with every reference field offset preserved. An unready cubemap or nonpositive/nonfinite
gamma is diagnosed rather than triggering an asset upload inside maintenance. Ready image/view/descriptor versions and
all push inputs are retained by their submitting frame, including when the source cubemap is replaced.

Direct lighting reads the previous complete atlas for reference bounce feedback, with both 0.0 and default 0.5 verified.
PROCESS reads the resulting voxel lighting; STORE reads all updated averages. Main-queue barriers order these operations,
atlas feedback across frames, history read/modify/write, shared status, and optional diagnostic readers. Each GPU producer
rejects whole-field failure flags. STORE records a generation but leaves `Status.ready=0`: this milestone intentionally
does not publish ordinary lighting. Runtime-only settings are propagated without reallocating the field, while light
and probe frame records freeze their own inputs. No dedicated compute queue or immediate maintenance submission is added.

`RequestCurrentSceneSdfgiProbeDebug(cascade=0, probe=2456)` records all atlas layers, the selected cascade's average SH,
the selected probe's complete history, and generation/status into immutable host-coherent buffers. Probe flattening is
`x + z*17 + y*289`; 2456 selects (8,8,8). `CaptureCurrentSceneSdfgiProbeDebug(path)` waits explicitly, exports a 1440p PNG,
and returns the captured generation/status. Ordinary maintenance does not wait for readback. In the PNG, top-left is all
irradiance layers and top-right all radiance layers, cascades top-to-bottom at half resolution. Bottom-left is the selected
cascade's signed average SH (289 columns, 17 groups of 16 coefficient rows); bottom-middle is the selected probe's signed
history (16 coefficients across, history phases down); bottom-right is SH-L0 over the selected Y probe plane, X across/Z
down. Signed display maps `0.5 + 0.5*v/(1+abs(v))`, with gray zero. Radiance uses Reinhard then gamma 2.2; the L0 plane uses
the reference 0.88622 ambient factor. These are diagnostic visualizations, not beauty output or performance thresholds.

Reproduce with the M4 disposable-resource setup:

```powershell
python Scripts/capture_sdfgi_voxels.py --module-dir out/build/vs2026-x64-tests/PythonBinding/RelWithDebInfo --resources tasks/m4-resources --output tasks/m7-sponza-probes.png --view transport
```

The driver requires a fresh generated project, retains the disposable Sponza assets, warms three default 30-frame history
cycles, and captures one frame. `GetCurrentSceneGiStatus()` reports transport/history/sky metadata, separated memory,
failure diagnostics, and resolved SDFGI GPU stage samples. `SetGpuTimingCaptureEnabled(true)` enables those measurements.

M7 evidence (2026-09-05): inspected 2560-by-1440 `tasks/m7-sponza-probes.png`, SHA256
`f9a7af06456dfad57a4216198f558e8f67846fb61b116dd4e4bb3e22ddabcfbb`, RTX 5070/driver 2496774144, all RT facilities disabled.
Default four cascades, H30, 16 rays, feedback 0.5; captured generation 91/history phase 0, GPU failure flags zero,
ready zero, effective provider Environment. The 524 static contributors and compact counts match the M5 representation.
Field/scratch/upload/diagnostic allocation bytes were 328,543,376 / 94,085,120 / 1,721,872 / 11,323,488.
For application frames 61-90, median GPU milliseconds were injection 0.128816, PROCESS 0.278416, STORE 0.041040. These are
stage observations from one validation-enabled capture, not a full-frame cost or optimization target; initialization,
voxelization, preprocessing, uploads, and camera shading are not included in those three stage measurements.

Four focused resource/transport/lighting tests passed in 5.34 seconds. Coverage includes two identical complete history
cycles, signed saturation, explicit coarse sky LOD and single-mip clamp, gamma/energy/rotation metadata, retained replaced
sky resources, sky-disabled misses, outer-cascade hits, both atlas layers and every border/corner, GPU failure rejection,
retained diagnostic snapshots, and feedback disabled/before-atlas/default 0.5. The initial analytic cross-cascade fixture
lacked zero-valued occupied voxels; correcting its encoding made it match the reference without altering production
tracing. PROCESS and STORE passed Vulkan 1.3 scalar-layout SPIR-V validation. SDK, Python bindings, tests, and editor built
successfully; GPU checks and the Sponza run had no Vulkan or synchronization errors. No full suite, extra image matrix,
DDGI comparison, app installation, or user manual acceptance was performed. M8 is the first ordinary-lighting/manual gate.

## Stationary deferred gather and publication

M8 initially translated the forward `scene_forward_gi_inc.glsl::sdfgi_process` path. M8a replaces that selection with
the paired deferred `environment/gi.glsl::{sdfvoxel_gi_process,sdfgi_process}` implementation: eight-neighbor trilinear weights,
normal-weight floor 0.005, normal bias in probe units, packed occlusion parity/weights with floor 0.01, octahedral
irradiance/radiance addressing, cascade blending, and roughness-dependent radiance-to-irradiance mixing. Mixing starts at
roughness 0.2 (previously 0.5) and the radiance fetch stops at 0.99. Below 0.2, the same reference caller additionally
sphere-traces the SDF and samples the injected RGB9E5 light volumes. It retains radial cascade selection/progression,
self-bias, roughness-dependent softness, next-cascade distance/light blending, factor 0.5 for light meant for anisotropic
sampling, accumulated hit alpha, and roughness blending back to probe radiance. This needs no hardware RT and can reach
off-screen represented static surfaces, but remains voxel-resolution reflection, not triangle-accurate reflection.

Diffuse coverage retains the reference outer `1-blend` fade. Specular coverage is independent: a sharp miss at roughness
zero falls back entirely to environment specular without losing diffuse GI; increasing roughness contributes probe
radiance according to the reference alpha formula. The sharp branch overwrites specular alpha as in Godot, including
at the outer fade. Both incoming terms use the reference energy scaling. No gather equation, probe normalization, bias,
or energy has been tuned to DDGI or to compensate for the review image.

The 512-byte immutable metadata publishes the primary anchor in Y-scaled field coordinates, cascade minima relative to
that anchor, reference world parity offsets, atlas/occlusion addressing, and one generation. Every receiver uses its own
world position and view reflection direction against the same anchor. Gather normals and reflection directions receive
Godot's Y scaling and normalization. Per-camera exposure is absent from field data; exposure normalization is one.
No valid cascade, an unpublished/mismatched generation, or a whole-field GPU failure returns zero coverage before field
sampling, selecting the ordinary Environment fallback.

`SdfgiPublish` follows all probe STORE work. It uploads frame-slot metadata with explicit ordering against earlier
readers, then sets GPU ready only for a nonzero matching generation with zero failure flags. CPU `published` describes
the ordered recording, not synchronous GPU completion; the shader checks ready/failure/generation again. CPU-known
light/transport/preprocess failures prevent publication. Missing anchors retain the last publication without moving
coverage. The scene graph and every eligible camera graph import the same atlas, occlusion, per-cascade SDF/light
volumes, status, and published UBO identities. Set 5 bindings 5/6 contain eight sampled SDF/light-volume views each;
inactive entries reuse valid compatible views and are never indexed beyond the active cascade count. Actual combined
host descriptor limits are checked before pipeline creation. No new image allocation is needed for sharp tracing.
Owner-level main-queue barriers cover publication-to-camera reads and all previous readers before the next
frame's writes, including separately compiled graphs and reused metadata slots. Resources, metadata, descriptors, and
staging survive all submitting fences. No dedicated compute queue or immediate maintenance submission is introduced.

Only enabled ordinary scene raster cameras and the canonical editor Scene camera receive set 5. Immediate, reflection
capture, custom-recorder, utility, and RT camera paths are excluded. Transparent paths retain ordinary Environment
lighting. Static, dynamic, and skinned opaque/masked receivers share the same deferred shader; only static supported
geometry contributes to the field. The non-SDFGI deferred pipeline remains five sets. `GetCurrentSceneGiStatus()` adds
`published_generation` and the camera handles that actually bound set 5. `ReadCurrentSceneSdfgiFieldStatus()` explicitly
waits and reads GPU generation/ready/failure for diagnostics; ordinary status inspection does not add that wait.

`SdfgiLighting.slang` is a DDGI-free entry point. It replaces global environment diffuse and specular base by coverage,
applies native dielectric/metallic/albedo/BRDF response once, preserves indirect-intensity semantics, and does not divide
irradiance by pi again. Local reflection probes blend over the selected specular base. Material/GTAO diffuse visibility
and native rough-specular AO each apply once; packed SDFGI occlusion stays in probe weights, never as a second AO term.
Direct lighting, material emission, material-free opaque G-buffer, alpha-only masked coverage, and final SSR composition
are unchanged. Ordinary material/BRDF/local-reflection helpers are exported without changing their equations; no DDGI
gather, visibility, blend, atlas, or environmental-composition function is called by this entry point.

The shared-module source layout validator includes unused vertex-only `set 0 / binding 14` in its compute reflection.
Rather than widening the host binding's stage visibility, this one deferred variant uses Vulkan pipeline creation plus
emitted SPIR-V verification; the dedicated Gather ABI shader still validates the new six-set layout. Disassembly confirms
that the M8 emitted SDFGI shader had five set-5 bindings (M8a adds bindings 5/6), read-only status, no DDGI resources, no unused vertex draw-index
binding, and no RT capability. The ordinary deferred variant contains no set 5. Both variants and the publication shader
pass `spirv-val --target-env vulkan1.3 --scalar-block-layout`.

Three focused resource/gather tests passed in 4.86 seconds on the current RTX 5070 with all effective RT facilities
disabled and Vulkan/synchronization validation enabled. The combined GPU fixture covers unpublished/published,
generation mismatch, failure/recovery, normal/occlusion/trilinear weighting, cascade blend, outer fade/outside fallback,
roughness mixing, and two separately compiled camera graphs in reversed order. The CPU check covers reference metadata,
negative/shared anchor coordinates, Y scaling, and eligible/excluded camera selection. The resource fixture compiles the
five- and six-set variants, checks packed views/ABI, and verifies retained resources. No Vulkan or synchronization errors
were reported. A fixture initially assumed full renderer startup had created the ordinary pipeline; it now explicitly
creates that variant with the actual host layouts. This correction does not change production behavior.

The inspected first beauty baseline is `tasks/m8-sponza-beauty.png` (2560 x 1440), SHA256
`1b359011b7f86254761154a67a2b6ce93c0db5b8b7d4398c7f2cb80914e56f27`. Reproduce with the disposable-resource driver above,
using `--view beauty --output tasks/m8-sponza-beauty.png`. The final run completed 90 warmup transport passes and captured
generation 91, with GPU ready 1, failure flags 0, effective Automatic SDFGI, and one ordinary camera binding set 5.
RTX 5070/driver 2496774144; RT pipeline/query/BLAS/TLAS all false. Default C4/H30/rays16/feedback0.5 and compact counts
68,792 / 19,148 / 3,926 / 886 were unchanged. Field/scratch/upload/diagnostic allocation bytes were
328,543,376 / 94,085,120 / 1,852,944 / 80. No Vulkan or synchronization errors were reported. An earlier diagnostic attempt
failed to convert the new camera-ID vector to Python; explicitly building a Python list corrected the status binding.
The image has strong yellow illumination and bright floor highlights; the demo includes a yellow point light, but the
visual response still needs user acceptance. This candidate is not yet an accepted visual baseline.

### M8a verification and stationary review

The final M8a build succeeded for SDK, tests, editor, and Python bindings. Seven focused tests passed in 4.448 seconds
on the current RTX 5070 with RT pipeline/query/BLAS/TLAS disabled and Vulkan/synchronization validation enabled.
The existing gather fixture now covers sharp hit/miss, radial bias and light-volume blending, roughness 0/0.1/0.1999/0.2,
stepping from the first to the next cascade, independent diffuse/specular alpha, publication failure/recovery, and
reversed two-camera graphs. Settings tests cover default-on, absent settings, explicit saved false, and field reset;
the lighting fixture supplies fully visible occlusion for its existing feedback check. No Vulkan/synchronization errors.
Final DeferredSdfgi, DeferredEnvironment, and Publish SPIR-V pass Vulkan 1.3 scalar-layout validation; SDFGI has the seven
set-5 bindings and no RT/DDGI resources, while Environment has no set 5. Six affected C++ files pass clang-format 22.1.8.

One 2560-by-1440 RT-disabled Sponza session captured occlusion on, then toggled it off and reconverged the recreated
field. Both images reached generation 91, GPU ready 1, failure flags 0, and one ordinary camera. Both use the new sharp
path, so the pair isolates occlusion rather than comparing to DDGI or mixing shader versions. Files:

- `tasks/m8a-sponza-occlusion-on.png`, SHA256 `371ef87530f90c6a9b2b40c774b89c29249393a7662ff8a6682ebe219f26ab70`.
- `tasks/m8a-sponza-occlusion-off.png`, SHA256 `7bcac90fd22ee8f0f4ff71fc9c9f8a05510c5b2dd5c4783936072d8e0c246a8f`.

The capture log is `tasks/m8a-sponza.log`. Use `capture_sdfgi_voxels.py --view beauty` with an optional
`--occlusion-off-output <second.png>` for this same-session comparison, using fresh disposable resources. No authored
project was cleared. The user reports occlusion on is noticeably better and accepted it as the stationary baseline before
continuing. This is not a claim of exact transport or reflection parity. Sharp tracing remains under local reflection probes and SSR; reflection captures
still do not consume SDFGI. The script verifies the setting/reset path, not interactive GUI operation.

### Manual review launch

Build command: `cmake --build out/build/vs2026-x64-tests --config RelWithDebInfo --target EvoEngine_Tests EvoEngineEditor
PyEvoEngine --parallel 8`. The built editor is
`C:\Users\lllll\Documents\GitHub\EvoEngine\out\build\vs2026-x64-tests\EvoEngine_App\RelWithDebInfo\EvoEngineEditor.exe`.
The dedicated review launch uses a disposable Rendering resource copy containing Sponza, with no generated
`Rendering.eveproj`. It does not clear authored project files, enable the DDGI showcase, or enable RT. It selects
Automatic SDFGI and opens the Scene view at the main camera pose, with a fixed 2560-by-1440 render target and the same
camera/post-processing settings. From the repository root:

```powershell
& ./out/build/vs2026-x64-tests/EvoEngine_App/RelWithDebInfo/EvoEngineEditor.exe --sdfgi-review ./tasks/m4-resources
```

The stationary M8/M8a result is user-accepted, with Use Occlusion retained on. The default settings turn Use Occlusion on.
Inspect Environmental Lighting > Automatic SDFGI > Use Occlusion and allow reconvergence after changing it. Existing
saved explicit false values are preserved when loading normally. M8 and M8a are committed as `93e868f5` and `29a76b4e`.
The current M9 checkpoint adds translation across cascade margins and large relocation/return. Allow a history cycle
after returning, and inspect seams, lingering old lighting, sphere undersides, and reflection transitions. Interactive
movement acceptance is required before M10. No full suite, additional image matrix, DDGI comparison, or app installation
was run; installation is scheduled before M12 final review.

## Automatic scrolling and relocation (M9)

For each dirty cascade, ascending from fine to coarse, rasterize its disjoint entering slabs into cleared material scratch.
Before resetting its compact dispatch, reconstruct retained solid cells at `old_cell + dirty_regions`, unpack retained
occlusion into the eight scratch volumes, and scroll every SH coefficient/history layer through shared history/average
scratch. `SCROLL_STORE` writes that scratch back; with positive bounce feedback, `STORE` regenerates the shifted atlas
before voxel direct lighting. The same half-size JFA, upscale/final refinement, occlusion, compact-store, lighting, full-grid
probe PROCESS, and all-cascade STORE stages then publish one coherent generation. Retained occlusion neighborhoods are
skipped using Godot's per-axis scroll mask; the previously dormant scalar-to-vector mask translation is corrected.

Probe displacement is the signed voxel displacement divided by eight. Overlapping probes copy their exact history and
running sum. Entering child probes seed all history slots from the parent's trilinear average (including signed-16-bit
clamping); entering probes at the outermost cascade retain the destination's previous history, as Godot does.
`render_forward_clustered.cpp::_update_sdfgi` calls `render_region` before `render_static_lights` updates the cascade UBO.
Accordingly, scroll uses the previous cascade metadata; the lighting-input upload then replaces it with the new metadata.
Both versions are frame-owned CPU snapshots, with transfer/compute ordering around the shared per-frame GPU UBO.

Full redraws skip scrolling and rerasterize the entire cascade. Godot does not clear SH history on a full redraw or
teleport: old grid-index history is replaced progressively by normal probe integration. Brief old-light ghosting and
relocation hitches are therefore expected; persistent stale lighting after reconvergence is not. The reference eight-cell
movement threshold, dirty-volume/full-redraw threshold, and per-axis large-relocation behavior are unchanged.

Necessary host adapters:

- Reconstruct retained payload from EvoEngine's unlit compact seed, not its statically baked copy. M6 already refreshes
  static lighting for rebuilt cascades; using baked light here would count retained static light twice. The shader's
  coordinate, albedo/facing, and emission packing are unchanged.
- Scroll reads the whole-provider GPU failure/capacity guard before compact-cell access. Failure prevents publication;
  partial preparation failures force a full retry so a missed slab cannot silently become a valid field.
- Explicit main-queue barriers cover scratch reuse, indirect reads, integer/sampled aliases, both cascade-UBO uploads,
  previous camera readers, and frames in flight. No async queue, immediate-submit maintenance, or new lifetime owner.
- Repeated scene-graph execution in the same scene frame preserves publication without integrating again. Missing-anchor
  frames freeze coverage and reuse the last valid generation; a new updating frame must republish before cameras use it.

Use `Scripts/capture_sdfgi_voxels.py --view beauty --traverse` with fresh disposable Sponza resources for one 1440p session:
signed-axis/diagonal crossings, larger movement, full relocation, and immediate/one-/three-history-cycle return images. The
generic Python capture camera-position helpers preserve rotation and reject nonfinite input. This is explicit diagnostic
automation, not a normal-renderer readback loop. The user has reviewed interactive motion and authorized continuing.

### M9 verification and movement review

SDK, tests, editor, and Python binding built with the manual-review command above; final log:
`tasks/m9-final-build.log`. Twelve distinct focused tests passed: the combined signed-scroll GPU fixture, allocation/ABI,
placement/teleport/anchor CPU checks, scene-frame ownership, two-camera publication/coverage, voxelization, lighting reseed,
and probe transport. Evidence: `tasks/m9-scroll-tests.log/.xml` (2 tests, 7.622 s),
`tasks/m9-regression-tests.log/.xml` (10 tests, 4.469 s), with changed frame/publication paths rechecked after the final
small fixes in `tasks/m9-final-tests.log/.xml` and `tasks/m9-frame-tests.log/.xml` (4 tests each, 2.697 s and 1.571 s).
All 24 exported SPIR-V variants validate with `spirv-val --target-env vulkan1.3 --scalar-block-layout`.
Changed C++ files pass clang-format 22.1.8 checking and scoped whitespace checks. No Vulkan validation/synchronization
errors were reported; pre-existing unused-vertex-attribute performance warnings remain.

Sponza used RTX 5070, driver `2496774144`, occlusion on, and all four effective RT flags false. The initial traversal
helper failed at the Python position conversion before moving; that helper was fixed. A complete traversal then exposed
noticeable residual brightening at one history cycle, so one focused recovery rerun extended the same path to three cycles.
This is the only extra image run beyond the planned traversal and fixes/rechecks that observed concern, not a test matrix.
Final evidence is `tasks/m9-recovery.log`, with 114 scripted movement/recovery steps and one transport update per step:

| Capture | GPU generation | Ready / failures | Local image |
|---|---:|---|---|
| Warmed starting view | 91 | 1 / 0 | `tasks/m9-sponza-recovery.png` |
| Immediate return after full relocation | 116 | 1 / 0 | `tasks/m9-sponza-recovery-return-immediate.png` |
| Return after one history cycle | 147 | 1 / 0 | `tasks/m9-sponza-recovery-return-one-cycle.png` |
| Return after three history cycles | 208 | 1 / 0 | `tasks/m9-sponza-recovery-return-three-cycles.png` |

The starting and recovered cascade positions and compact counts match (68,792 / 19,148 / 3,926 / 886 cells).
CPU compact-count diagnostics are fence-delayed and can still show the away field immediately after returning; the
explicit GPU publication status is current. Visual inspection shows the one-cycle brightening substantially decays by
three cycles, consistent with retained history and bounce feedback. This is not a claim of pixel identity or interactive
motion acceptance. The original occlusion-on M8a baseline is preserved; these are new M9 review candidates.

Launch the built editor with `--sdfgi-review ./tasks/m4-resources` as above. Keep occlusion on, move the Scene camera across
several margins in both directions, then make a large relocation and return. Review cascade seams, lighting sticking to
old coordinates, recovery over several history cycles, and rough/sharp sphere reflections. The shared-field/secondary
fallback contract is covered by the two-camera GPU fixture; no extra image scene or resolution was added. Interactive
GUI movement has not been operated by the agent. The user confirmed the M9 review is complete and requested continuation.
M9 is accepted. M10-M12 remain subsequent milestones; all-app installation is scheduled at implementation completion
before M12.
