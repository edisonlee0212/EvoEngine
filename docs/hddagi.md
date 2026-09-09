# Automatic HDDAGI

Status: H0-H6 implementation and recorded validation are complete. HDDAGI provides deferred diffuse/specular lighting and diffuse illumination for reflection captures. See [comparison results, regression exceptions and reproduction](hddagi-results.md). HDDAGI remains opt-in; SDFGI is still the default. GI and sharp reflections now always use full viewport resolution; the legacy half-resolution setting is ignored on load and no longer saved or exposed.

HDDAGI is an explicit third indirect-GI provider. Existing Environment=0, Automatic DDGI=1 and Automatic SDFGI=2 retain their meanings and defaults; Automatic HDDAGI uses 3. It shares nominal probe coverage and anchor selection, not DDGI/SDFGI field storage. Correctness and recorded results are required; no speedup threshold applies.

## Reference

Godot PR119869 head `da1410fa3516d08cc31b6e86bd6673b9ce776316` is the algorithmic baseline. The adjacent `C:/Users/lllll/Documents/GitHub/godot` checkout is detached at this revision; it is not a build dependency. SDFGI remains pinned to `34d06658a85845111a50db9e485ec4a0701d4298`. Core source falls under Godot's default Expat/MIT entry in COPYRIGHT.txt; retain source/commit attribution and [Godot's notice](licenses/Godot-MIT.txt).

Paths below are relative to `servers/rendering/renderer_rd/` in the HDDAGI reference:

| Host / source | Port contract |
|---|---|
| environment/gi.h and gi.cpp: HDDAGI::create/update/render_region/update_light/update_probes | Resource layouts, circular regions, update classification, light cadence, publication inputs |
| forward_clustered/render_forward_clustered.cpp: hddagi_update and voxel rendering callers | Environment layout recreation, history enum conversion, dedicated geometry pass |
| shaders/forward_clustered/scene_forward_clustered.glsl | Anisotropic raster coverage/material outputs; use EvoEngine's existing supported material adapter |
| shaders/environment/hddagi_preprocess.glsl | REGION_STORE, LIGHT_STORE, LIGHT_SCROLL, OCCLUSION, OCCLUSION_STORE, LIGHTPROBE_SCROLL, LIGHTPROBE_NEIGHBOURS, LIGHTPROBE_GEOMETRY_PROXIMITY, LIGHTPROBE_UPDATE_FRAMES |
| shaders/environment/hddagi_direct_light.glsl | PROCESS_STATIC / PROCESS_DYNAMIC, finite-distance HDDA visibility, previous probe bounce |
| shaders/environment/hddagi_integrate.glsl | PROCESS, FILTER, CAMERA_VISIBILITY; periodic sample slots, hit/version caches, integer rolling sums |
| shaders/environment/gi.glsl | HDDAGI gather, ambient blend, separate sharp reflection traces, cascade fallback |
| shaders/environment/hddagi_filter.glsl | Bilateral reflection filter, HALF_SIZE variant and GI-resolution specialization |
| shaders/environment/hddagi_debug.glsl / hddagi_debug_probes.glsl | Field/probe/occlusion debug |

## Verified defaults

Environment header: four cascades, min cell size 0.2, 16x8x16 region format, bounce/energy=1, sky enabled, normal/probe bias=1.1, reflection bias=2, occlusion bias=0.1; probe and ambient filters on, reflection filter off. RenderingServer project defaults select history index1 =12 updates (choices6,12,18,24,32), light interval index2 =4 (choices1,2,4,8,16), and half-resolution GI=true. One probe process group uses a5x5 directional octahedral tile, rather than SDFGI's independently selected ray count.

EvoEngine retains its shared33x17x33 probes, four cascades,0.8 base probe interval and100% Y scale. Eight cells/probe gives256x128x256 voxels and0.1 cell size. These coverage differences from Godot are intentional; provider switching must not rewrite shared settings.

Contributor eligibility matches current SDFGI, including its `static_entities_only=false` default: eligible rigid MeshRenderers contribute regardless of static flag; optional static-only filtering excludes non-static owners. Skinned/morph/deforming, forward/transmissive/blended and other unsupported contributors remain excluded. Moving eligible rigid meshes are geometry edits with conservative invalidation, not a new deforming-geometry algorithm.

## Resource dimensions and packing

Let V=(X,Y,X), P=V/8+1, C=cascades, H=history.3D field cascades stack along Y. Probe textures flatten probe Y/Z into texture Y. All dimensions use checked arithmetic and Vulkan limits before allocation.

| Family | Dimensions | Storage / sampled format |
|---|---|---|
| Voxel bits | (X/4,C*Y/4,X/4) | RG32_UINT |
| Region presence/version | (X/8,C*Y/8,X/8) | R8_UINT / R16_UINT |
| Lit voxels and neighbor payload | (X,C*Y,X) each | R32_UINT; light sampled as E5B9G9R9_UFLOAT_PACK32 |
| Raster albedo | (X/2,Y/2,3X) | R16_UINT,6 anisotropic faces |
| Raster normal bits | V | R32_UINT atomics |
| Raster emission / anisotropic emission | V/2 each | R32_UINT stores |
| Diffuse / specular / filtered diffuse | (7Px,7PyPz),C layers each | R32_UINT / E5B9G9R9_UFLOAT_PACK32 |
| Hit cache / version / sample history | (5Px,5PyPz),C*H layers | R32_UINT / R16_UINT / R32_UINT |
| RGB running sums | (15Px,5PyPz),C layers | R32_UINT |
| Ambient probes | (Px,PyPz),C layers | RGBA16F |
| Neighbor visibility / process frame | (Px,PyPz),C layers each | R32_UINT |
| Geometry proximity / camera visibility | same | R8_UNORM |
| Occlusion ping-pong | (X+2,C*(Y+2),X+2) each | R16_UINT / R4G4B4A4_UNORM_PACK16 |

Compact light cells, dispatch counts/copies, scene/cascade/light/camera UBOs and per-frame uploads are additional buffers. The reference uses up to8 cascades,128 dynamic/1024 static lights, and a0.5 solid-cell ratio. Capacity overflow must fail publication rather than overwrite storage. Unused upstream declarations such as voxel_disocclusion_tex must not be reproduced as invalid bound resources: audit actual shader consumers and remove unused bindings or supply typed defaults.

Temporal logical bytes = C*Px*Py*Pz*25*(10H+12), covering hit/version/sample rings and RGB sums. Vulkan image padding determines runtime budget; strictly below4GiB applies to temporal allocations, not total field+scratch+retiring bytes. Allocation/preflight includes all formats/view compatibility, atomics, sampled filtering and aggregate descriptors, not only temporal budget.

REGION_STORE uses8x8x8=512 threads and shared bit accumulators; light processing uses64 threads; probe processing uses5x5=25. Other preprocess/camera/filter variants require per-variant checks. No RT/query/AS capability is required. Sample/storage view formats must be queried with the actual mutable/extended usage flags.

## Indexing and invalidation decisions

- Support voxel counts64..256 in multiples of16, separately for X/Z and Y. Shared counts outside the derived range remain authored and use fallback; transactional edits reject without mutation.
- Replace dimension-dependent `& (size-1)` with nonnegative modulo for rectangular/non-power-of-two layouts. Retain fixed block/region bit operations only. Apply identical wrapping to store, lighting, tracing, cache, occlusion, probe history and debug code.
- Preserve per-variant fixed-point precision: probe integration uses8 fractional bits; direct visibility, camera reflection and debug traces use10. The commented-out HQ_RAY experiment uses16 and is not enabled by the reference. Handle zero/near-zero components before reciprocal conversion and validate against an independent voxel-DDA oracle.
- Hit coordinates use8-bit components; validate0..255 before encoding. Face-neighbor offsets and cross-cascade hits require explicit handling at the256 boundary, never silent wrapping into a false hit.
- R16 region-version rollover clears dependent caches before reusing a version. Geometry edits conservatively invalidate caches across all potentially traversed cascades, including previous misses and rays with new intervening blockers. The initial implementation also invalidates caches and histories for represented payload edits; finer retention is deferred. This is a correctness adaptation to EvoEngine's editable rigid geometry.
- Preserve overlapping world-space regions and histories on scroll. Clear entering/incompatible data before bounce/gather. Teleports/layout replacement reset. Shared anchor is authoritative; utility cameras cannot change it.
- Optional camera-visibility scheduling starts disabled until anchor-only behavior is validated. Do not take the first rendered camera as scheduler owner.

## Camera resolution and composition

Godot allocates GI buffers at internal size or floor(internal/2). Each eligible GI pixel traces a sharp-reflection ray using a 2x2 direction-dither pattern, then coplanar neighboring pixels blend their four results. Trace and storage dimensions both equal GI dimensions; the 2x2 blend does not reduce ray dispatch dimensions. FILTER_HALF_SIZE and half-GI specialization are distinct. EvoEngine now always uses full viewport resolution, with at least1 texel for tiny views. Viewport resizing retires camera storage without destroying scene probe history. This deliberately departs from the reference half-resolution default.

Camera data is resolved after GTAO. Do not move normal-map/roughness evaluation into the opaque geometry pass. Use HDDAGI-specific surface/gather/filter/composition resources and a compatible set5 pipeline variant. Existing DDGI/SDFGI resolution and shaders remain unchanged. Preserve separate diffuse/specular coverage, one material/AO/sky composition, and SSR/local-probe priority. Reflection captures sample diffuse only, keep a complete generation across faces, and cannot move the anchor.

## Lifetime and validation

Scene-owned runtime plans once per frame. Frame slots retain immutable input/descriptors and allocation owners. Graph uses cover raster atomic writes, compute reads/writes, indirect buffers, clears and camera reads; GENERAL layout does not replace barriers. Publish only a valid complete generation with shader-visible readiness/failure checks. Failure or unavailable transport selects Environment.

The following milestone checkpoints distinguish source reconnaissance, implementation tests and installed visual/performance evidence. Existing-provider baselines remain unchanged. Slower results and visible artifacts are recorded without an improvement gate; final evidence and limits are in the H6 results report.

H1 settings are available through Environmental Lighting > GI > Provider and Python GiSettings.hddagi. Inactive settings remain authored; only active-provider storage requirements participate in validation. A valid shared anchor enables scene field allocation and graph-driven image initialization. Submitted frame slots retain allocation owners across provider, scene and layout replacement. Initialization recording does not publish transport. The editor/Python status separates active image and retiring allocation bytes. Camera/transport pipeline descriptors are validated as their consuming passes are added.

### Provider foundation validation

The RelWithDebInfo editor and test executable build with the provider foundation. The focused run covers 51 shared-GI, Environmental Lighting, provider migration and HDDAGI tests; all pass with Vulkan core/synchronization validation enabled. HDDAGI allocation and switching tests explicitly disable RT, ray queries and acceleration structures. No Vulkan validation errors were logged. This verifies allocation/view compatibility and partial-failure cleanup, not transport or rendered appearance.

The subsequent lifecycle run passes all53 focused tests. Additional tests overwrite and clear every image before last-texel/last-layer readback, and verify that a submitted scene field survives provider removal until its frame slot retires. No Vulkan core/synchronization errors were logged. Scene allocation waits for a valid shared anchor; unsupported or unready states remain unpublished.

On the NVIDIA GeForce RTX 5070, default image requirements total 802,766,848 padded bytes, including 264,568,832 temporal bytes. These figures exclude future buffers, camera resources and retired generations; they are not peak runtime memory measurements. Reproduce the focused run with `EvoEngine_Tests --gtest_filter=Hddagi*.*:GiProbes.*:GiSettings.*:EnvironmentalLighting*.*:SdfgiRuntime.ProviderDefaultsToAutomaticAndPreservesExplicitChoices` from its build output directory. Local logs and XML results are in `tasks/h1-tests.*`.

## Hierarchy implementation checkpoint

`HddagiVoxelFrame` reuses the SDFGI contributor/material snapshot and three-axis draw adapter, with dedicated HDDAGI images and pipelines. It builds six interleaved RGB565 albedos, RGB9E5 emission, directional emission and geometric-normal bits. The geometric-normal front-face correction is inverted relative to Godot because this world-space raster projection uses EvoEngine's counterclockwise winding convention. Derivatives are evaluated before masked fragments are discarded.

The H2 checkpoint rebuilt the field on geometry/payload or placement changes. H3 replaces this baseline with retained local updates and skips unchanged frames. Raster inputs are immutable and retained with submitted field allocations until frame-slot retirement. No SDF/JFA resources or ray-tracing services are used.

The hierarchy fixture compares GPU hits to an independent double-precision voxel DDA over 64-cubed, 128-cubed, 192x80x192 and 256x128x256 fields, both fractional precisions, signed circular offsets, empty/full/wall/corner patterns and cascade transitions. At exact voxel edges, either incident solid voxel is accepted only if it contains the oracle's first intersection. These cardinal/diagonal and near-axis checks do not establish arbitrary-angle accuracy or finite-distance direct-light semantics. Separate scene/raster fixtures cover masked UV1 input, vertex color, reversed winding, emission, repeated scratch clears, nonstatic contributor parity, movement/removal and unchanged frames.

## Compact payload and bounded visibility

H2 adds the full-size R8UI disocclusion volume omitted from the initial H1 allocation inventory. Per-cascade light-cell storage uses the reference 0.5 capacity ratio and a bounded indirect-dispatch/status buffer. Preflight checks the storage-buffer range and device-padded bytes. Compact positions use eight bits per axis plus pending flags, replacing the reference shader's actual seven-bit packing (its ten-bit comment is inaccurate). Albedo, octahedral normals, emission and cached occlusion follow the reference; occlusion remains initialized to zero until its H4 producer lands. A zero normal sum uses the first contributing direction to avoid NaNs.

Capacity overflow records a GPU failure bit and bounds both writes and indirect dispatch. Frame-owned status copies expose completed cell counts/failure flags only after fence retirement. Python GI status provides `hddagi_allocation_bytes`, `hddagi_light_cell_counts`, `hddagi_failure_flags` and `hddagi_region_version`. Allocation totals include owned field buffers, raster inputs, uploads and status readbacks; shared geometry/textures and driver pipeline allocations are excluded.

Finite direct-light visibility shares the HDDA walk at ten fractional bits. A necessary endpoint correction replaces Godot's `sign(direction) * distance` component box with `direction * distance`; the former can trace beyond diagonal point/spot lights. Zero/near-zero components avoid reciprocal conversion and endpoint comparisons on inactive axes. Endpoint tests cover both directions, diagonal/near-axis rays, starting in or outside the inner cascade, and blockers immediately before/after the light distance. The reference disocclusion cascade offset is applied only to Y; its scalar addition would offset all three coordinates. Rebuilt cells clear obsolete neighbor links and non-light payloads.

H2 validation: 67 focused tests passed with Vulkan core/synchronization validation, including capacity-one overflow and maximum payload coordinates on 64-cubed, 192x80x192, 256x128x256 and 256-cubed fields. Four HDDAGI shader entrypoints pass SPIR-V validation using the runtime's scalar/row-major layout. Updated RTX 5070 default preflight is 836,321,280 image bytes (264,568,832 temporal) plus 268,435,584 field-buffer bytes; the older H1 figure above predates the disocclusion/payload additions. Full build/install passed. These are preflight sizes and automated correctness evidence, not peak memory, performance or manual appearance results.

## Local updates and temporal invalidation

HDDAGI now unions entering eight-cell regions with old/new contributor bounds. Hierarchy writes cover nonoverlapping core boxes; light reconstruction adds a one-cell halo, rasterization two cells. An outgoing one-cell lighting border also rebuilds because retained boundary cells can depend on surfaces that just left the cascade. Unchanged frames skip voxel work. Teleports, anchor replacement, uncertain bounds, recovery and region-version rollover force a fresh rebuild.

Each cascade has two compact processing/dispatch buffers. A GPU pass translates and retains unaffected entries, then rebuilt rectangles append without duplicate halo entries. Counts and indirect dispatch remain capacity bounded, and status readback belongs to the submitted frame. Python status exposes update count and last/total updated hierarchy-region counts. Two compact buffer sets increase the field-buffer budget; these are included in live allocation accounting.

Any voxel update clears cached hits and misses across all cascades before reuse. Cache hits also compare the physical adjacent light-cell region version. The pinned reference writes that version using a shift of eight while reading it using division by eight; the port consistently divides by eight. Exterior face neighbors are retraced in outer cascades rather than wrapped to the opposite boundary. Stable cache entries reuse physical light-cell coordinates; cache-off diagnostics follow the same traversal without mutating cache storage.

Probe history, RGB sums, atlas tiles and metadata use the same minimum-cell region phase. Small scrolling preserves retained world probes and clears newly exposed probes. Represented geometry/payload edits reset histories across cascades because probe paths can cross their boundaries. These reset hooks precede future transport consumers; lighting publication remains disabled until transport and camera integration are complete.

On the RTX 5070 validation setup, negative signed GPU remainder produced incorrect circular mappings in the expanded rectangular fixture. Wrapping now computes unsigned magnitude remainder and restores the sign without signed overflow, including INT32_MIN. A dedicated GPU fixture compares negative/positive and extreme coordinates against a 64-bit CPU modulo oracle over region, voxel and probe-atlas dimensions. This is an observed setup-specific correctness issue, not a general claim about all GPUs or compilers.

H3 validation: 71 focused tests passed, including 24,612 GPU wrap-component comparisons, the existing voxel-DDA/bounded traversal cases, and exact incremental-versus-fresh hierarchy/neighbor/compact payload comparisons for two 64x80x64 cascades. Cache-off, initial fill, version rejection and stable reuse agree; retained/exposed history texels are checked across scrolling, geometry/payload edits, version rollover and teleport. Six compute shaders compile and pass SPIR-V validation with scalar block layout. No Vulkan core/synchronization errors or warnings were logged; pre-existing unused vertex-attribute performance warnings remain. Evidence is in local `tasks/h3-final2.log/xml`. This does not establish lighting or visual quality.

The editor/test targets build in RelWithDebInfo. All enabled applications install successfully using `python Scripts/install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --jobs 8`; the editor is `out/install/vs2026-x64/bin/EvoEngineEditor.exe`. No manual lighting test is claimed before transport is available.

## Probe transport

Scene frames now inject static/dynamic native lights and emission, then integrate cached HDDA rays into 5x5 probe histories. The port retains the reference's fixed convolution weights, 14-bit temporal sums, per-probe phases, 7x7 atlas borders, ambient values, proximity scheduling and neighbor-weighted diffuse filtering. Native cubemap sky follows the existing inverse-gamma and Y-rotation convention. Newly reset probes and light/sky edits receive four consecutive updates; otherwise distant probes update every four frames. Camera visibility culling is disabled initially.

Static radiance has a separate volume so static-light edits can refresh without mutating source emission or rebuilding geometry. Dynamic injection samples only a previously valid filtered diffuse generation for bounce feedback. Static/dynamic light-list truncation counts are exposed per cascade. Occlusion preserves the reference nibble writes together with its host offsets -4 and 0; changing either alone reverses parity. Metadata uses the actual 24-bit neighbor payload in R32UI.

GPU status aggregates compact overflow and non-finite radiance before publishing a transport generation. A failed generation disables feedback and resets probe history on recovery. Frame-owned descriptors, inputs and status copies remain alive through fence retirement. Python/editor readiness is a completed readback; H5 adds camera publication.

`CaptureCurrentSceneHddagiDebug(path, image, layer=0, z_slice=0)` exports Light, StaticLight, Diffuse, FilteredDiffuse, Specular, Occlusion0/1, History, HistorySum, ProcessFrame or Proximity after an explicit fence wait. Images show physical circular storage. Radiance uses Reinhard mapping and gamma 2.2; occlusion preserves four sampled parity channels in RGBA; history sums/phases are normalized for inspection. Export does not advance scene updates. `Scripts/capture_hddagi_transport.py` runs a stationary RT-disabled Rendering demo from a disposable asset copy and records hashes, status and provenance.

H4 evidence: 74 focused tests cover shared-provider regressions, native photometry/shadows, previous-generation feedback, temporal ring/sum conservation, sky removal, overflow/non-finite recovery, rectangular cascades, occlusion wall sides and wrapped margins. Eight compute variants pass SPIR-V validation. Installed Sponza capture `tasks/h4-stationary2/manifest.json` records completed generation 122, zero GPU failure flags, no light-list overflow, and 246485/75321/16765/3048 compact cells. No Vulkan core/synchronization errors or warnings were logged. The lit-voxel slice and filtered atlas were visually inspected; these diagnostics do not establish final camera appearance.

The capture used the installed Python runtime at `C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/python` after building the editor at `C:/Users/lllll/Documents/GitHub/EvoEngine/out/build/vs2026-x64/EvoEngine_App/RelWithDebInfo/EvoEngineEditor.exe` and installing all enabled applications with the command above (`tasks/h4-install2.log`). RTX 5070 field images occupy 970542080 bytes; the capture reports 1507556656 owned allocation bytes including frame data. This is an observed allocation snapshot, not peak memory or a performance comparison. H5/H6 below cover final deferred appearance, reflection captures and comparisons.


## Deferred cameras and reflection captures

Choose **Environmental Lighting > GI provider > Automatic HDDAGI**. Settings take effect at the next scene frame. Reflection filtering, energy and receiver biases preserve probe history; changing shared probe layout or history size replaces the field. Unsupported or unready configurations use Environment and expose the reason. Live allocation reporting includes camera images and retained capture snapshots, with retiring fields reported separately.

Opaque geometry remains material-free and masked geometry evaluates only alpha coverage. After GTAO, a camera pass evaluates the receiver normal and roughness using the same material helpers as deferred lighting. Its RGBA16F surface stores a receiver-only flag alongside roughness; an immutable instance buffer derives that flag from the SDFGI contributor registry. Normal deferred lighting evaluates material response afterward. This repeated material evaluation is intentional and has its own HDDAGI Surface GPU timing.

Camera diffuse/specular targets use R32UI storage with RGB9E5 sampled views and a separate RG8 coverage image. GI and sharp-reflection dimensions both use the full viewport, clamped to one for tiny cameras. The reference traces one ray per eligible GI pixel with 2x2 direction variation and coplanar blending; it does not trace at one quarter of the GI dimensions. The gather roughness transition is 0.25; the optional bilateral reflection filter has a separate 0.3 cutoff and radius 12 at full resolution. Two filter passes use independent scratch images. The final deferred pass samples and composes incoming scene-linear radiance with native material response, AO and local-reflection/SSR priority.

Static bakes, budgeted dynamic probes and immediate reflection captures consume diffuse lighting only. A capture copies the probe atlases, occlusion and GPU readiness status once, retains that generation across faces, and leaves the shared scene anchor unchanged. Later scene updates, provider switches and layout changes cannot mutate those copied inputs. If an immediate call encounters unsubmitted scene-field work, it uses Environment rather than consuming that pending generation. Snapshot copies and all upload/descriptor inputs live through their GPU submissions.

Python `GetCurrentSceneGiStatus()` includes effective provider, `hddagi_cameras` viewport/GI dimensions, readiness/failures, live/retiring memory and `gpu_timings` with application-frame IDs, total sampled GPU span and individual pass durations. `Scripts/capture_hddagi_transport.py --beauty` additionally records the final camera PNG. Diagnostic export waits explicitly and does not advance the field.


H5 closeout: 80 focused tests passed (`tasks/h5-closeout.log/xml`), including two-camera odd/tiny and resolution/filter switches, sharp self-hit/zero-visibility reconstruction, static/dynamic/immediate reflection capture, and provider/layout replacement while faces are pending. Seven camera shader variants passed SPIR-V validation. No Vulkan core/synchronization findings occurred in the final suite or installed capture; existing unused-vertex-attribute performance warnings remain in the synthetic capture fixture. Full application install succeeded with the command above (`tasks/h5-install-closeout.log`).

The installed 2560x1440 Sponza capture (`tasks/h5-closeout-camera/manifest.json`, `beauty.png`) uses RT/query/BLAS/TLAS disabled, 1280x720 GI, history12, light cadence4, completed generation123 and zero GPU failure flags. Owned allocation is1554878640 bytes. The manifest records all HDDAGI settings, device/driver, module hashes and precommit working-tree provenance. The final PNG was visually inspected and offered for user review. This capture alone establishes neither superior quality nor a speedup.

## Comparative validation

[H6 results](hddagi-results.md) record default/full-resolution HDDAGI, SDFGI and RT-enabled DDGI over stationary, slow/fast camera motion and geometry/material/light edits. The benchmark uses three 300-frame repetitions per workload, 120-frame warmups, frame-aligned GPU timings, allocation samples and 1/12/120-frame edit captures. The compact fixture separately exercises thin walls, masks, emission, curved reflections and receiver-only movement. Existing SDFGI/DDGI image goldens pass unchanged; broad-suite and legacy shader exceptions are explicitly listed in the report.

`Scripts/benchmark_hddagi.py` runs against a disposable installed Rendering project. `GetCurrentSceneGiStatus()` exposes application frame IDs and whole-application `vma_allocation_bytes` / `vma_block_bytes` for validation; these include shared scene allocations and exclude driver memory. The existing HDDAGI owned/retiring counters retain their narrower meaning. GPU stage timestamps include voxel updates, direct lighting, probe integration/filtering and camera surface/gather work.

Historical H0-H6 measurements below and in hddagi-results.md retain their original half/full settings. Current benchmark_hddagi.py always uses full HDDAGI resolution and no longer accepts --full-resolution. Old half_resolution YAML keys are ignored.

See [GI refinement](gi-refinement.md) for full-resolution migration and the indoor visibility experiment. The editor's Occlusion bias tooltip now explains its minimum-visibility meaning.
