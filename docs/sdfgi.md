# Automatic SDFGI

Implementation baseline: `codex/universe-performance`, `f977f012f413`, 2026-09-05.
Capability preflight, the opt-in provider shell, GPU storage, CPU placement/scene inputs, static voxelization, stationary
SDF/occlusion preprocessing, voxel direct-light injection, and stationary probe transport/storage are implemented.
Automatic SDFGI builds the representation when it has an eligible anchor, but reports Environment fallback and publishes
no ordinary lighting. Scrolling and gather entry points remain ABI-only.

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
| `SdfgiPreprocess.hpp/.cpp`, `Shaders/Compute/SdfgiPreprocess.slang` (scroll variants still ABI only) | `gi.cpp::SDFGI::render_region`, its uniform sets, and `shaders/environment/sdfgi_preprocess.glsl` |
| `SdfgiLight.hpp/.cpp`, `Shaders/Compute/SdfgiDirectLight.slang` | `gi.cpp::SDFGI::{render_static_lights,pre_process_gi,update_light}`, `LightStorage::light_get_aabb`, and `shaders/environment/sdfgi_direct_light.glsl` |
| `SdfgiProbe.hpp/.cpp`, `Shaders/Compute/SdfgiIntegrate.slang` (scroll variants still ABI only) | `gi.cpp::SDFGI::{update_probes,store_probes}` and `shaders/environment/sdfgi_integrate.glsl`; host cubemap and diagnostic readback adapters |
| `SdfgiVoxelizer.hpp/.cpp`, `Shaders/Modules/EvoEngine/SdfgiVoxel.slang`, `Shaders/Graphics/Vertex/SDFGI/SdfgiVoxelize.slang`, `Shaders/Graphics/Fragment/SDFGI/SdfgiVoxelize.slang` | ForwardClustered `_render_sdfgi` and `scene_forward_clustered.glsl::MODE_RENDER_SDF`; host-only diagnostic plane readback |
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
readiness/failure/generation/capacity fields. The six-set gather ABI layout is created only for the opted-in SDFGI field;
ordinary and excluded camera pipelines still have their existing five-set layouts and never bind the SDFGI set.

`SdfgiInitialize` imports explicit unmanaged images/buffers into the scene graph, clears every used layer, and leaves
readiness zero. Main-queue barriers cover prior shader/transfer/indirect users, clears, and subsequent consumers even
across separate graph/frame executions. Initialization alone never publishes a lighting result. There is no SDFGI
immediate submission, new queue, or device-idle wait. Allocation/layout/pipeline failure is diagnosed and leaves no field;
it is not retried every frame. Changing provider or incompatible settings creates a fresh initialization attempt.

`GetCurrentSceneGiStatus()` reports allocation and initialization-recorded state plus `active_allocation_bytes`, split
into field, scratch, upload, and diagnostic categories using actual VMA allocation sizes. These are active-owner totals,
not driver-pool usage or a peak that includes temporarily retiring old fields. Upload includes immutable voxel material,
texture-info, region/axis buffers and their retained staging arenas. Diagnostic is zero unless a snapshot is requested.
The remaining ABI-only compute shaders are compiled/layout-checked but never dispatched by normal rendering; focused GPU
tests use explicitly compiled resource-check variants.

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
This early host-only view precedes the later SDF/probe inspector. No field is published and scrolling reconstruction is
not implemented yet.

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
counted as diagnostic memory. Until M9, partial/slab updates are diagnosed as unsupported and cannot publish a field.

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
