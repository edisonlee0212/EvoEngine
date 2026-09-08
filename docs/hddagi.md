# Automatic HDDAGI

Status: implementation in progress. H0-H2 are complete. H2 builds raster and compact light payloads, circular region/block occupancy and bounded/probe HDDA through the scene graph. Local updates and transport remain in progress. Selecting HDDAGI uses Environment until transport is implemented and published. No HDDAGI rendering is available yet.

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
- R16 region-version rollover clears dependent caches before reusing a version. Geometry edits conservatively invalidate caches across all potentially traversed cascades, including previous misses and rays with new intervening blockers. Lighting/payload edits retain valid geometry hits but reconverge radiance. This is a correctness adaptation to EvoEngine's editable rigid geometry.
- Preserve overlapping world-space regions and histories on scroll. Clear entering/incompatible data before bounce/gather. Teleports/layout replacement reset. Shared anchor is authoritative; utility cameras cannot change it.
- Optional camera-visibility scheduling starts disabled until anchor-only behavior is validated. Do not take the first rendered camera as scheduler owner.

## Camera resolution and composition

Godot allocates GI buffers at internal size or floor(internal/2); sharp reflection tracing uses a further2x2 sampling pattern. Thus default effective sharp tracing is quarter linear viewport resolution, even though reflection storage has GI dimensions. FILTER_HALF_SIZE and half-GI specialization are distinct. Preserve controls/defaults and record both dimensions; handle tiny/odd views with at least1 allocated texel, bounds checks and correctly clamped reconstruction. Runtime resolution edits retire per-camera storage without destroying scene probe history.

Camera data is resolved after GTAO. Do not move normal-map/roughness evaluation into the opaque geometry pass. Use HDDAGI-specific surface/gather/filter/composition resources and a compatible set5 pipeline variant. Existing DDGI/SDFGI resolution and shaders remain unchanged. Preserve separate diffuse/specular coverage, one material/AO/sky composition, and SSR/local-probe priority. Reflection captures sample diffuse only, keep a complete generation across faces, and cannot move the anchor.

## Lifetime and validation

Scene-owned runtime plans once per frame. Frame slots retain immutable input/descriptors and allocation owners. Graph uses cover raster atomic writes, compute reads/writes, indirect buffers, clears and camera reads; GENERAL layout does not replace barriers. Publish only a valid complete generation with shader-visible readiness/failure checks. Failure or unavailable transport selects Environment.

H0 is source reconnaissance, not a GPU-validation or performance result. Subsequent milestones must record exact test/build/install commands and results. Compare existing providers without changing their baselines. Record warmed stationary/motion/edit captures and timing/memory; slower results or reference artifacts are reported without an improvement gate. Implementation and installed manual review remain pending.

H1 settings are available through Environmental Lighting > GI > Provider and Python GiSettings.hddagi. Inactive settings remain authored; only active-provider storage requirements participate in validation. A valid shared anchor enables scene field allocation and graph-driven image initialization. Submitted frame slots retain allocation owners across provider, scene and layout replacement. Initialization recording does not publish transport. The editor/Python status separates active image and retiring allocation bytes. Camera/transport pipeline descriptors are validated as their consuming passes are added.

### Provider foundation validation

The RelWithDebInfo editor and test executable build with the provider foundation. The focused run covers 51 shared-GI, Environmental Lighting, provider migration and HDDAGI tests; all pass with Vulkan core/synchronization validation enabled. HDDAGI allocation and switching tests explicitly disable RT, ray queries and acceleration structures. No Vulkan validation errors were logged. This verifies allocation/view compatibility and partial-failure cleanup, not transport or rendered appearance.

The subsequent lifecycle run passes all53 focused tests. Additional tests overwrite and clear every image before last-texel/last-layer readback, and verify that a submitted scene field survives provider removal until its frame slot retires. No Vulkan core/synchronization errors were logged. Scene allocation waits for a valid shared anchor; unsupported or unready states remain unpublished.

On the NVIDIA GeForce RTX 5070, default image requirements total 802,766,848 padded bytes, including 264,568,832 temporal bytes. These figures exclude future buffers, camera resources and retired generations; they are not peak runtime memory measurements. Reproduce the focused run with `EvoEngine_Tests --gtest_filter=Hddagi*.*:GiProbes.*:GiSettings.*:EnvironmentalLighting*.*:SdfgiRuntime.ProviderDefaultsToAutomaticAndPreservesExplicitChoices` from its build output directory. Local logs and XML results are in `tasks/h1-tests.*`.

## Hierarchy implementation checkpoint

`HddagiVoxelFrame` reuses the SDFGI contributor/material snapshot and three-axis draw adapter, with dedicated HDDAGI images and pipelines. It builds six interleaved RGB565 albedos, RGB9E5 emission, directional emission and geometric-normal bits. The geometric-normal front-face correction is inverted relative to Godot because this world-space raster projection uses EvoEngine's counterclockwise winding convention. Derivatives are evaluated before masked fragments are discarded.

The current scene path rebuilds the field on represented-geometry/payload or cascade-placement changes and skips unchanged frames. This is the H2 integration baseline; it does not yet implement H3 retained local updates. Raster inputs are immutable and retained with submitted field allocations until frame-slot retirement. No SDF/JFA resources or ray-tracing services are used.

The hierarchy fixture compares GPU hits to an independent double-precision voxel DDA over 64-cubed, 128-cubed, 192x80x192 and 256x128x256 fields, both fractional precisions, signed circular offsets, empty/full/wall/corner patterns and cascade transitions. At exact voxel edges, either incident solid voxel is accepted only if it contains the oracle's first intersection. These cardinal/diagonal and near-axis checks do not establish arbitrary-angle accuracy or finite-distance direct-light semantics. Separate scene/raster fixtures cover masked UV1 input, vertex color, reversed winding, emission, repeated scratch clears, nonstatic contributor parity, movement/removal and unchanged frames.

## Compact payload and bounded visibility

H2 adds the full-size R8UI disocclusion volume omitted from the initial H1 allocation inventory. Per-cascade light-cell storage uses the reference 0.5 capacity ratio and a bounded indirect-dispatch/status buffer. Preflight checks the storage-buffer range and device-padded bytes. Compact positions use eight bits per axis plus pending flags, replacing the reference shader's actual seven-bit packing (its ten-bit comment is inaccurate). Albedo, octahedral normals, emission and cached occlusion follow the reference; occlusion remains initialized to zero until its H4 producer lands. A zero normal sum uses the first contributing direction to avoid NaNs.

Capacity overflow records a GPU failure bit and bounds both writes and indirect dispatch. Frame-owned status copies expose completed cell counts/failure flags only after fence retirement. Python GI status provides `hddagi_allocation_bytes`, `hddagi_light_cell_counts`, `hddagi_failure_flags` and `hddagi_region_version`. Allocation totals include owned field buffers, raster inputs, uploads and status readbacks; shared geometry/textures and driver pipeline allocations are excluded.

Finite direct-light visibility shares the HDDA walk at ten fractional bits. A necessary endpoint correction replaces Godot's `sign(direction) * distance` component box with `direction * distance`; the former can trace beyond diagonal point/spot lights. Zero/near-zero components avoid reciprocal conversion and endpoint comparisons on inactive axes. Endpoint tests cover both directions, diagonal/near-axis rays, starting in or outside the inner cascade, and blockers immediately before/after the light distance. The reference disocclusion cascade offset is applied only to Y; its scalar addition would offset all three coordinates. Rebuilt cells clear obsolete neighbor links and non-light payloads.

H2 validation: 67 focused tests passed with Vulkan core/synchronization validation, including capacity-one overflow and maximum payload coordinates on 64-cubed, 192x80x192, 256x128x256 and 256-cubed fields. Four HDDAGI shader entrypoints pass SPIR-V validation using the runtime's scalar/row-major layout. Updated RTX 5070 default preflight is 836,321,280 image bytes (264,568,832 temporal) plus 268,435,584 field-buffer bytes; the older H1 figure above predates the disocclusion/payload additions. Full build/install passed. These are preflight sizes and automated correctness evidence, not peak memory, performance or manual appearance results.
