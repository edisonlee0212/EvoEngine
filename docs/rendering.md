# EvoEngine Rendering

[Back to README](../README.md)

This page documents the current render frame and the migration path toward a more explicit render model. It is intended
to keep render-layer refactors grounded in the existing engine behavior.

## Current Ownership

`RenderLayer` owns the SDK render frame. It builds descriptor set layouts, creates the built-in graphics, compute, and
ray tracing pipelines, prepares render instance storage, records shadow and camera rendering commands, invokes external
render callbacks, renders gizmos, and hands camera textures to post-processing.

Scene environment data owns the current DDGI runtime, authoring-default, atlas storage, debug visualization,
multi-volume, and large-world clipmap settings. `RenderLayer` executes those settings and exposes them in its inspector.
The DDGI inspector is intentionally debug-first: it exposes pause/reset controls, selected probe coordinates, atlas tile
readouts, atlas sizing, wrapped border/stride details, scroll-aware selected-probe metadata, and selected-probe ray
sample summaries before those settings drive lighting. It also reports CPU-side DDGI frame/pass recording time plus
probe, ray-sample, atlas, and buffer-size counters so expensive or missing probe work is visible during inspection.

Scene-authored DDGI probe volumes use the `DdgiVolume` private component. The implementation exposes persistent probe
counts, uniform probe spacing, priority, blend distance, update/ray budgets, hysteresis, and bias settings. The authored
volume origin is the center of the probe lattice, matching RTXGI's volume placement convention; the first probe is
derived by subtracting half of `(probe_counts - 1) * probe_spacing` from that origin. DDGI debug visualization is owned
by RenderLayer, not the generic editor gizmo
queue: the scene-camera-only `DDGIProbeVisualization` graphics pass draws probe spheres directly from GPU probe
metadata/state and irradiance/visibility atlases while leaving the main camera free of DDGI debug overlays. The
scene-camera-only `DDGIProbeRayVisualization` graphics pass can also draw the selected probe's freshly traced ray/hit
lines from the GPU ray-output buffer when ray debug is enabled. The RenderLayer DDGI debug menu owns the probe sphere
color mode, depth-tested versus x-ray overlay mode, radius, intensity, selected-probe scale, ray-overlay alpha, and
selected-probe ray overlay.
Authored multi-volume behavior is currently priority-only selection, not simultaneous multi-volume contribution:
RenderLayer sorts enabled `DdgiVolume` components by priority when requested, applies `selected_volume_index` to that
sorted list, and traces/samples one active volume. `blend_distance` fades that active volume outside its probe lattice;
positions inside the active lattice remain full weight to match RTXGI volume coverage. True
overlap blending and per-volume resource isolation are future work. Focused source coverage locks that the active source
comes from one candidate in the priority-sorted enabled-volume list and that only that selected candidate drives
trace/sampling setup.

When DDGI runtime, atlas/ray debug visualization, or probe illumination visualization is enabled, the frame graph
declares a frame-local DDGI ray-output buffer plus imported persistent probe metadata, probe-state, irradiance-atlas, and
visibility-atlas resources.
`DDGIAtlasPrepare` clears the frame-local ray output every frame and clears persistent atlas/metadata only when the probe
layout/source changes or history is reset, so budgeted probe updates retain lighting from previous frames.

When ray tracing is available, unpaused DDGI runtime/debug rays also schedule `DDGIRayDiagnostics` as a frame
ray-tracing pass. It traces probe rays through the scene mesh TLAS into the DDGI ray-output buffer using the existing
point-cloud hit-record layout. Probe ray directions use RTXGI-style spherical Fibonacci samples. Fixed
relocation/classification rays remain deterministic, while non-fixed rays receive a frame-wide random orientation across
the active volume so neighboring probes share RTXGI-style spatially coherent ray rotation without resetting probe
history. When probe classification is enabled, inactive probes still trace the fixed-ray prefix so classification can
reactivate probes after geometry or source changes; non-fixed inactive rays write an inactive marker and skip the TLAS
trace.
Miss rays write explicit miss radiance from authored constant environment background when available, and back-facing
triangle hits are encoded with negative ray radiance alpha so update passes can distinguish them from front-facing
radiance samples. The DDGI TLAS contains mesh-compatible render instances: deferred, forward, transparent, and instanced
`Mesh` instances that already have mesh BLAS data and the standard triangle/material payload.
Skinned meshes and strands are intentionally ignored by DDGI. External render instances are also ignored by default; they
can participate only when registered with `DdgiExternalGeometry`, which supplies a BLAS plus a triangle offset compatible
with the standard `EE_INDICES`/`EE_VERTICES` and material payload used by the DDGI closest-hit shader. The closest-hit
shader currently uses geometry normals and material constant albedo, respects material cull mode for DDGI ray hits, then
explicitly evaluates every enabled directional, point, and spot light at the hit point. Point and spot DDGI light queries
respect the light-block far plane before applying attenuation, and shadow-casting direct lights cast hard TLAS shadow rays
so shadow-casting geometry between a light and sampled surface can reduce probe irradiance. DDGI primary probe rays trace
all DDGI geometry, while DDGI light-shadow rays use a separate shadow-caster TLAS mask derived from each renderer's
`cast_shadow` setting; this keeps visible light-debug meshes from blocking their own point lights while still allowing
their emissive materials to appear in probe radiance. Emissive mesh surfaces still contribute only when probe rays hit the
emissive surface; an explicit emissive light sampler remains a follow-up. Two-sided materials shade ray backfaces with
flipped normals so raster and DDGI agree for cube-authored
interior test scenes, but geometric backface hits are still encoded as negative DDGI samples for relocation and
classification feedback. Front-facing DDGI ray radiance remains HDR until the probe update pass applies the irradiance
encoding clamp. It also samples the previous DDGI
irradiance and visibility atlases at the hit point for a recursive diffuse bounce, with clamped recursive albedo to keep
energy stable. Scene-owned DDGI runtime settings expose `indirect_intensity`, which drives both deferred DDGI diffuse
sampling and recursive probe-hit sampling. The closest-hit path intentionally avoids material texture fetches because
DDGI updates can run while scene texture descriptors are still being populated during load.

`DDGIProbeUpdate` consumes that ray-output buffer and writes directional octahedral irradiance tiles plus directional
visibility-moment tiles through a graph compute pass. Steady-state probe updates use the configured hysteresis value.
Light-change refreshes and geometry/material-change refreshes preserve history with their own faster hysteresis values,
while source/layout changes, scroll slot invalidation, manual resets, and pending-list repair still overwrite updated
probe texels. The authored update budget is a ray-sample budget: RenderLayer divides it by the current ray count to pick
how many probes update this frame, so increasing ray count does not multiply per-frame DDGI rays. When adaptive update
budget is enabled, RenderLayer scales the next frame's ray-sample budget from the last probe-update recording cost
toward the configured target milliseconds, clamped by authored min/max bounds; with it disabled, the authored manual
ray-sample budget is used directly. Steady-state probe updates use an RTXGI-style round-robin window and wrap at the end
of the probe list so the configured ray budget is not under-used at atlas boundaries. The update shader uses
RTXGI-style irradiance history blending without EvoEngine's earlier temporal clamp.
Irradiance history starts from zero for empty history, reduces hysteresis for large darkening deltas, slows large
brightening deltas with the runtime brightness threshold, and keeps a minimum darkening step so stale bright radiance can
decay. Visibility moments still use straight hysteresis blending. Each atlas tile stores the
configured interior resolution plus a one-texel octahedral wrap border; the update shader writes interior texels, then
copies wrapped irradiance and visibility borders so bilinear sampling can cross tile edges without reading neighboring
probes. When relocation or classification is enabled, the ray-generation shader marks a stable fixed-ray prefix and the
update shader uses that prefix for relocation/classification feedback while excluding it from irradiance and visibility
blending so fixed rays do not bias the atlas. Backfaces are skipped for irradiance blending, and if enough non-fixed
rays hit backfaces relative to the authored random-ray threshold the directional irradiance blend returns no radiance for
that probe texel. Visibility moment blending raises directional weights by the runtime distance exponent before
accumulating first and second moments. Backfaces use shortened signed distances for relocation/classification, and can
classify probes as inactive when the fixed-ray backface ratio exceeds the authored fixed-ray threshold. Relocation uses
the RTXGI-style fixed-ray policy: inside-geometry probes move along the closest backface direction, near-surface probes
move toward the farthest opposing frontface, and probes with clearance move back toward zero offset only when the
candidate remains inside the probe voxel ellipsoid. Classification only deactivates probes detected inside geometry from
the fixed-ray backface ratio, so valid open-room probes are not pruned just because the fixed-ray prefix misses nearby
surfaces.
The Rendering demo authors a Sponza-tuned 10x6x16 probe lattice with 1.5 spacing and local volume origin (0, 3, 3).
It keeps relocation enabled but classification disabled there because
classification can mark valid near-wall probes inactive from curtain backfaces. The yellow point-light/debug-sphere
intensity is kept moderate so a few ray hits cannot dominate local probe radiance.
After a pending full refresh has completed, classified inactive probes skip the expensive TLAS ray trace in steady-state
updates for non-fixed rays and write a marked zero-hit ray sample; fixed rays continue to trace for
relocation/classification feedback. The update pass uses the previous probe state to decide whether atlas blending runs,
then writes the newly relocated/classified probe state after irradiance and visibility atlas updates. Pending
scene/source/history refresh windows still retrace inactive probes so classification can recover after relevant scene
edits.
It also writes three metadata `vec4` values per probe: irradiance plus hit ratio, normalized average hit
distance plus backface ratio and relocation amount, and relocation offset plus active state. The probe update pass feeds a
persistent probe-state buffer, and the next `DDGIRayDiagnostics` pass offsets probe origins from that state so
relocation/classification can affect subsequent traces. The atlases and metadata are now inspectable before the
deferred-lighting sampler is enabled, and the graph records an explicit ray-output-to-atlas dependency chain.
DDGI-relevant scene input changes, such as geometry transforms, material inputs, light state, and DDGI source changes,
reset a pending probe refresh window so the configured update budget walks the atlas after scene edits without
restarting on camera-only storage churn. The update shader filters traced, shadowed direct irradiance from the ray-output
buffer into atlas texels instead of re-evaluating light loops, and probes that were already inactive preserve their atlas
history until fixed-ray classification reactivates them. When `Probe state` is enabled, RenderLayer copies the metadata
into a debug readback
buffer for inspector readback. Probe sphere illumination is rendered by the scene-camera-only GPU DDGI visualization
pass, which samples and debug-decodes the irradiance atlas per sphere fragment, tone maps it for inspection, writes
depth for opaque self-occlusion in depth-tested mode, and can switch to metadata-driven visibility or hit-ratio debug
coloring through the dedicated probe color mode.
RenderLayer now tracks explicit probe update reasons for light, geometry, material, DDGI source, scroll, manual reset,
steady-state, and pending-list repair paths. Pending refresh windows keep their original reason bits while the update
budget consumes them, and the DDGI inspector shows the last probe update reason for debugging stale or unexpected
updates.
Explicit scene-change refresh windows use faster condition-specific hysteresis during atlas update, so edited geometry,
lights, and materials react faster without discarding all previous probe history.
The DDGI inspector also reads the selected probe's cached metadata when probe-state debug is enabled, showing
irradiance, hit/miss/backface ratios, visibility distance, relocation offset/amount, active state, and update age
alongside the scene-camera GPU probe visualization. When ray debug is enabled and the selected logical probe is included in the
current update window, RenderLayer copies just that probe's compact ray sample block into a staging buffer so the
inspector can show frontface, backface, miss, inactive, fixed-ray, and average-hit-distance summaries plus a small sample
preview. The same selected local probe index feeds `DDGIProbeRayVisualization`, so the scene camera can show the
frontface, backface, miss, and inactive ray lines without routing through generic gizmo tasks. It also reports the last
frame's probe update count and first/last updated probe index so budget starvation and sparse scroll updates are easier
to inspect.

Deferred lighting binds safe fallback DDGI atlas textures every frame and only enables DDGI diffuse sampling in the
render-info block when the frame ray/update chain produced current atlases. With DDGI disabled, raster lighting continues
to use the existing IBL ambient path. Enabled DDGI diffuse sampling uses border-aware octahedral irradiance lookup,
visibility-moment lookup, surface-normal bias, trilinear probe blending, probe-state active/inactive rejection,
RTXGI-style irradiance encoding/decoding, Chebyshev visibility with a low-weight floor, and continuous crushing of tiny
visibility weights. The runtime `final_visibility_strength` diagnostic can fade deferred final-surface visibility
weighting to pure trilinear probe blending for Cornell-style isolation captures without changing probe atlas generation.
The recursive probe-hit DDGI lookup uses the same normal sampling rules so secondary bounce estimates do
not use a looser visibility model than deferred lighting. Both paths normalize by visibility-weighted samples, reconstruct
linear irradiance with the RTXGI squared-gamma scale, floor each trilinear probe component to keep a continuous neighbor
set across probe-cell boundaries, then apply the Lambert diffuse term. Both deferred and recursive sampling return zero
outside active DDGI volume coverage and multiply their final diffuse contribution by the active volume blend weight.
Surface sampling now binds the DDGI probe-state buffer alongside the irradiance and visibility atlases, so relocated
probe offsets affect the same probe-to-surface visibility vectors used by ray generation and classified inactive probes
are rejected from both deferred lighting and recursive probe-hit sampling. The visibility atlas remains a two-channel
distance-moment atlas; it does not own active-state data.

Common DDGI math lives in `Shaders/Includes/DDGI.glsl`: probe-grid coordinates and indices, scroll-index wrapping, probe
world positions, surface bias, spherical probe-ray base directions, octahedral atlas lookup, atlas tile stride,
visibility weighting, and fixed-ray prefix sizing. Deferred lighting, recursive probe-hit shading, probe ray generation,
and probe atlas update all include that helper layer so later RTXGI ports can replace shared DDGI rules without updating
four separate shader copies.
When large-world clipmaps recenter by whole probe steps, RenderLayer now accumulates a DDGI probe scroll offset instead
of clearing persistent probe history. Ray generation, probe update, deferred sampling, and recursive probe-hit sampling
map logical probe-grid coordinates through that scroll offset to physical probe-state and atlas slots. Compatible
scrolls schedule only the newly exposed edge-plane probes through an explicit per-frame update-index buffer, preserving
interior probe history while the update budget consumes the sparse list. Large clipmap jumps, storage-capped volumes,
authored-volume movement, and non-scroll DDGI source changes still fall back to the full history reset path.

### DDGI Baseline Notes

The current Rendering Demo baseline disables static environment light, leaves the Capoeira entity disabled for idle
editor accumulation, and enables a scene-owned DDGI volume at 10x6x16 probes, 1.5 spacing, local volume origin
(0, 3, 3), 64 rays per probe, 16384 ray samples per frame, 0.02 normal/visibility bias, relocation enabled,
classification disabled, adaptive ray-sample budgeting enabled, and probe visualization scale 2.0.
The expected comparison behavior is direct white/yellow scene lighting plus DDGI indirect response, no stale blue probe
light after reset or disabled lights, scene-camera-only GPU probe visualization, and main-camera ray tracing configured after
the scene loads for README/smoke setup. The visible yellow point-light sphere is marked non-shadow-casting so DDGI TLAS
shadow rays do not treat the light visualizer as an occluder for its own point light. The Rendering Demo smoke path
validates that editor-mode main-camera ray tracing auto-fits the Camera panel, validates canonical DDGI runtime/volume
values above, and validates the pre-play yellow
moving point light used for light-response checks before entering play mode. Non-player smoke runs also execute the
configured editor `warmup_frames` before `Play()` so idle
editor exits are caught by the same automated path, then move that point-light sphere while requiring DDGI to report both
light- and geometry-refresh update reasons. The same smoke path disables/restores the point light, requests a DDGI manual
reset and requires the manual-reset update reason plus reset-flag consumption, and mutates/restores the point-light
sphere material while requiring a material-refresh update reason. It also mutates/restores the authored DDGI volume blend
distance while requiring a source-refresh update reason. The same smoke path enables probe-state readback for validation
frames and requires live metadata to contain active probes plus hit, miss, and backface-ratio evidence before and after a
DDGI light update. Disabled-light smoke validation temporarily disables every direct light component, resets DDGI history,
requires live probe metadata to become dark, then reads back the main ray-tracing camera center surface and requires it to
stay dark before restoring canonical lighting. Relocation smoke validation temporarily enables authored-volume probe
relocation, requests a history refresh, and verifies live metadata reports relocated active probes before restoring
canonical settings. Classification smoke validation temporarily enables
authored-volume probe classification, requires source/history refresh, verifies live metadata still has active probes
while also pruning inactive probes, then restores the canonical authored volume setup. It also temporarily disables the
authored volume, enables a 4x4x4 one-cascade clipmap, moves the main camera by one probe step, and requires a scroll
update with only the exposed 16-probe x plane refreshed; it also verifies an interior logical probe maps to the same
physical metadata slot after scrolling and keeps its metadata unchanged before restoring the canonical authored volume
setup. Before entering play mode, idle readback validation runs multiple editor frames that must report only the
steady-state DDGI update reason, keep the pending probe-update count at zero, preserve active/inactive probe counts, and
keep probe metadata finite and bounded; one final idle editor frame then catches unexpected scene refresh churn.
CornellBox smoke validation seeds a direct-only probe atlas with recursive DDGI intensity disabled, then enables
recursive DDGI against that history and requires average active probe irradiance to increase, proving a positive recursive
bounce contribution in the generated indoor scene. The same CornellBox path also temporarily disables probe
classification, captures a direct-only baseline, and requires the classified direct-only probe set to retain enough
active probes and peak irradiance to catch obvious classification dark holes. It then renders the main camera with
classification disabled and enabled, reads back render-texture pixels, and requires the classified Cornell interior to
retain enough final-surface luminance against that baseline. The authored Cornell baseline keeps classification disabled
for the Rendering demo, while DDGIApp enables the safer inside-geometry classification path to reject probes embedded in
the Cornell interior boxes. ThinWall smoke validation builds a
generated two-room scene around a thin center blocker and one shadow-casting point light, then resets DDGI to a
direct-only probe update and
requires the shadow-side probe irradiance average to stay well below the lit side. It also renders the main camera with
DDGI enabled, reads back render-texture pixels, and compares lit-side versus shadow-side visible regions so serious leaks
fail at the final-surface level. Rendering/Sponza smoke validation now also resets DDGI history, renders the canonical
hallway main camera, reads back left/center/right hallway regions, and requires at least one lit region plus visible
lit/shadow contrast so real-scene leakage does not collapse the hallway into uniform indirect light. Current known
validation gaps are pixel-level screenshots for atlas border seams and broader Sponza hallway camera/lighting variants.
`DDGIApp` is the clean player-mode Cornell-box comparison binary for RTXGI side-by-side checks: it opens a 1024x1024
window, uses the generated CornellBox demo setup, keeps DDGI runtime lighting enabled, clears skybox/post processing,
and disables DDGI debug visualization/overlays before entering play mode. The generated Cornell DDGI setup uses a
9x9x9 probe grid centered at the Cornell volume origin with 0.3 spacing, 0.35 edge fade, and 256 rays per probe; DDGIApp uses
a comparison-only fitted 13x13x14 lattice centered at the Cornell volume origin with 0.14333334 spacing, one-cell
edge fade, 0.02 normal bias, and 0.05 view bias. The extra +Z probe extends coverage to the visible short-box front face
near the open side of the Cornell box. Relocation and classification are on by
default for the comparison so probes embedded in the Cornell boxes are rejected before final gather; diagnostic
command-line switches can disable them when comparing those paths directly. `--disable-ddgi-final-visibility` captures the same probe
atlas with final-surface visibility weighting bypassed, separating bad probe radiance from bad final gather rejection.
DDGIApp also uses app-only 2.0 point-light brightness, 1.0 indirect intensity, 2.0 ceiling mesh emission, 16384 DDGI ray
samples per frame, and no camera post-processing so the comparison render isolates DDGI placement, filtering, and final
gather behavior.
Current DDGI visual inspection captures are generated through `Scripts/capture_readme_editor_screenshot.py` into
`out/visual-inspection`: `sponza-rendering-ddgi.png` for the Rendering/Sponza setup and `cornell-box-ddgi.png` for the
primitive Cornell box setup. `thin-wall-ddgi.png` captures the generated thin-wall leak-validation scene. Passing
`--ddgi-atlas-preview` opens a forced RenderLayer DDGI atlas inspection layout on the main viewport and scrolls to the
atlas readout; the current Sponza atlas-border capture is `sponza-ddgi-atlas-preview.png`. The editor capture window
stays 1920x1080, while the main ray-tracing camera render target auto-resizes to the Camera panel. The panel still uses
aspect-fit presentation and mouse mapping as a fallback, so camera output is not stretched.
Focused atlas seam-safety tests validate that wrapped border texels map back to same-tile interior texels for row,
column, and corner borders.

### 3DGS Demo Notes

The `3dgs` launcher profile opens a script-generated 3D Gaussian Splatting project under
`Resources/EvoEngine-DemoProjects/3DGS`. Run `python Scripts/generate_3dgs_demo.py` from the repository root to download
and verify the Spatial Dragon PLY asset before launching the profile. The source asset is Aimi Sekiguchi's
`spatialdragon-3dgs` `data/spatial_dragon.ply`, licensed CC0 1.0, 1,571,111 bytes, with SHA256
`40D7FDEBEB6A9A5755074F4F02A759EEE19BF15F46520A8D79B5F42BDE42921D`.

The generator writes deterministic asset metadata for a `GaussianSplat` asset and can optionally run
`EvoEngineEditor --demo 3dgs --capture-demo-preview` through `--editor <path-to-EvoEngineEditor.exe>`. The preview
capture command accepts `--preview-render-mode rasterization` or `--preview-render-mode raytracing` when a specific
camera path needs to be checked. On first launch, the profile creates and saves a scene containing one
`GaussianSplatRenderer`, a fitted editor/main camera, the default skybox as the camera background, and the default
environmental map for scene lighting. It also includes a tiny off-camera `Ray Tracing TLAS Seed` mesh because EvoEngine's
current ray-tracing camera path only records camera work when a mesh TLAS exists. Ray-tracing cameras composite Gaussian
splats as a raster overlay after the ray-tracing camera pass, or after ray-tracing volumetric clouds when clouds are
enabled. Splats are therefore visible in RayTracing camera mode, but they do not yet participate in TLAS traversal, mesh
occlusion, ray-traced reflections, shadows, DDGI, or ray-hit-distance generation. The demo asset is small enough for Git
without LFS, but the project files live inside the `Resources/EvoEngine-DemoProjects` submodule; publishing the generated
demo permanently requires committing that submodule content separately from the main EvoEngine code.

### DDGI RTXGI Port Map

`out/external/RTXGI-DDGI` is treated as the algorithm reference only. EvoEngine keeps shader source
in GLSL and does not add an HLSL, DXC, or RTXGI runtime dependency. The current native port boundaries are:

- RTXGI volume descriptors and resource allocation map to scene-owned `DdgiSettings`, `DdgiVolume`, and RenderLayer's
  persistent probe metadata, probe-state, irradiance-atlas, and visibility-atlas resources.
- RTXGI probe ray generation maps to
  `Shaders/RayTracing/RayGen/DDGIProbeDiagnostics.rgen`, using EvoEngine TLAS descriptors and native probe-grid push
  constants.
- RTXGI probe closest-hit and miss shading map to
  `Shaders/RayTracing/ClosestHit/DDGIProbeDiagnostics.rchit` and
  `Shaders/RayTracing/Miss/DDGIProbeDiagnostics.rmiss`, while preserving EvoEngine material constants, light buffers,
  shadow rays, and recursive DDGI atlas sampling.
- RTXGI shared coordinate, scroll-index, probe-ray direction, surface-bias, octahedral, visibility, fixed-ray,
  inactive-ray, and classification helper logic maps to `Shaders/Includes/DDGI.glsl`.
- RTXGI irradiance/distance blending, atlas border copy, relocation feedback, classification feedback, inactive-probe
  state writes, and probe metadata writes map to `Shaders/Compute/DDGIProbeUpdate.comp`; atlas blending intentionally
  uses the previous probe state and stores the next state afterward to match RTXGI's blend-before-relocate/classify
  update order while keeping EvoEngine's single GLSL compute pass.
- RTXGI surface sampling maps to `Shaders/Includes/Lighting.glsl` for deferred lighting and the recursive sampling path
  in the DDGI probe closest-hit shader.
- RTXGI volume update scheduling, ray-budget-capped round-robin update windows, atlas clears, descriptor binding, and debug readback map to RenderLayer frame passes:
  `DDGIAtlasPrepare`, `DDGIRayDiagnostics`, and `DDGIProbeUpdate`.

Not yet ported from the RTXGI reference are authored overlapping-volume isolation, variability-based update stopping,
weighted multi-volume update priority, lighting-priority/density selection across simultaneously contributing volumes,
and scene-level validation scenes for atlas seams, thin-wall leakage, classification, and broader large-world scrolling.
Per-probe variable ray counts remain intentionally deferred because RTXGI's shipped update model uses rays-per-probe plus
a volume ray-budget ceiling instead of camera-importance apportioning inside one volume.

RTXGI probe variability/reduction/readback remains intentionally deferred from this vanilla parity pass. The reference
uses a borderless single-channel probe-variability texture with irradiance-interior dimensions plus a two-channel
variability average/reduction texture and readback path to decide when a volume has converged enough to pause updates.
EvoEngine's current DDGI path keeps only the irradiance atlas, visibility atlas, probe-state buffer, and debug metadata
readback; variability resources should be added later with an explicit convergence/reactivation policy so they actually
affect scheduling instead of only adding memory and UI surface.

`RenderInstanceStorage` converts scene state into GPU-friendly frame data. It collects cameras, materials, lights, mesh
instances, skinned mesh instances, particle instances, strand instances, external render instances, indirect draw
commands, and acceleration-structure inputs.

`Camera` owns the camera render texture and GBuffer descriptor state. The current raster path uses a depth target plus
normal and material GBuffer color targets, then resolves lighting into the camera color texture.

`Platform` owns frame synchronization, swapchain image acquisition, command buffer recording, queue submission, and
presentation. `WindowLayer` presents ImGui output or the main camera render texture to the swapchain.

## Frame Flow

A normal interactive frame is ordered as follows:

1. `Application::PreUpdateInternal` updates input and time, then calls `Platform::PreUpdate` when rendering is enabled.
2. `Platform::PreUpdate` resets the current frame fence, flushes queued buffer and texture sync work, acquires the next
   swapchain image, resets frame command buffers, and marks the main camera for rendering in non-editor apps.
3. `Application::UpdateInternal` updates the active scene and layers, then calls `RenderLayer::PrepareForRendering`.
4. `RenderLayer::PrepareForRendering` calls `PrepareSceneForRendering`.
5. `PrepareSceneForRendering` updates render instance storage, binds per-frame descriptor sets, and refreshes ray tracing
   descriptors when ray tracing is enabled.
6. `Application::LateUpdateInternal` calls `RenderLayer::RenderAll`, `RenderLayer::RenderGizmos`, `WindowLayer::Render`,
   and `Platform::LateUpdate`.
7. `Platform::LateUpdate` submits recorded main-queue command buffers, presents the swapchain image, advances the frame
   index, and waits for the submitted frame fence.

## Render Data Preparation

Render data preparation happens before camera rendering. The current frame's `RenderInstanceStorage` receives the active
scene and render settings, calculates the LOD factor, collects editor cameras and scene cameras, updates each camera info
block, collects renderable entities, builds instance info blocks, collects lights, creates indirect draw commands, and
uploads frame buffers.

The per-frame descriptor set exposes render info, environment info, camera info, material info, instance info, kernel
data, light buffers, 2D textures, and cubemaps. Separate descriptor sets expose meshlet buffers, lighting shadow maps,
camera GBuffer textures, render texture storage images, ray tracing geometry data, and ray tracing point-cloud data.

The render graph model records stable names and metadata for current and advanced resources. Current default raster camera
resources are `Frame.RenderInstances`, `Frame.PerFrameDescriptorSet`, `Lighting.DirectionalShadowMap`, `Camera.Depth`,
`Camera.GBuffer`, and `Camera.Color`. Current ray tracing camera resources are `Frame.PerFrameDescriptorSet`,
`Frame.RayTracingDescriptorSet`, `Scene.MeshTLAS`, and `Camera.Color`. Advanced built-in resource descriptors cover
`Camera.MotionVectors`, `Camera.ObjectId`, `Camera.MaterialId`, `Camera.DepthPyramid`, `Camera.ColorHistory`,
`Camera.RadianceHistory`, and `Frame.VisibilityBuffer`.

## Raster Camera Flow

For each collected camera marked as requiring rendering, `RenderLayer::RenderToCamera` records the raster path:

1. Directional shadow cascades render into the directional light shadow map.
2. Camera GBuffer and depth images transition to attachment layouts.
3. The deferred geometry pass renders mesh, meshlet, instanced, skinned, and strand instances.
4. Per-frame external deferred callbacks run inside the geometry pass.
5. GBuffer, depth, and shadow resources transition to shader-readable layouts.
6. The deferred lighting pass draws a fullscreen quad into the camera color render texture.
7. Per-frame external forward callbacks run against the camera.
8. The camera post-processing stack runs outside the camera command recording path.

Point and spot light shadow maps are prepared once before camera iteration. Gizmos render after normal camera rendering.

## Ray Tracing Camera Flow

When Vulkan ray tracing is enabled, scene preparation updates the top-level acceleration structure and ray tracing
descriptor set. Ray tracing cameras then bind the per-frame descriptor set, ray tracing descriptor set, and render
texture storage descriptor set before tracing directly into the camera color image.

## Extension Model

The legacy extension model is callback-slot based. Packages can register functions for point light shadows, spot light
shadows, directional light shadows, deferred camera rendering, and forward camera rendering. Packages can also enqueue
mesh draw commands or register external render instances so their materials and renderer indices are present in frame
storage.

Newer renderers should prefer graph-aware declarations. `RenderLayer::RegisterRenderResource` declares custom frame,
camera, persistent, history, or imported resources. `RenderLayer::RegisterFrameRenderPass` executes a declared pass once
per frame before built-in shadow and camera rendering. `RenderLayer::RegisterCameraRenderPass` executes a declared pass
for each camera before post-processing. The callbacks still execute in the current fixed order, but descriptors record
the pass name, queue, scope, declared resource access, and dependencies. This keeps compatibility with existing packages
while giving newer renderers a path toward graph-scheduled resources and synchronization.

Frame and camera graph pass registration also has opt-in overloads that receive a `RenderGraphExecutionContext`. The
context exposes the current pass descriptor, compiled execution plan, resource usage plans, allocation plans, and a
resource registry for engine-owned bindings such as per-frame descriptor sets, camera render textures, camera color/depth
images, GBuffer descriptor sets, and ray tracing descriptor sets. This is the runtime contract advanced renderers will
use before graph-owned transient images and buffers are allocated by the scheduler.

`RenderGraph::UsesQueue` and `RenderGraph::GetPassIndices` expose queue metadata for future schedulers. The current
executor still runs passes in insertion order; queue inspection is only a scheduling foundation, not async execution by
itself.

`RenderGraph::Compile` builds an execution/resource-use plan without changing execution order. The plan records pass
dependency indices, resource-hazard dependencies, merged schedule dependencies, queue-bucketed schedule steps, resource
first and last use, reader and writer passes, imported resources, graph-owned transient resources that can be reused or
aliased, compatible allocation slots for non-overlapping transients, and resource transition records for future barriers.
A compile context can resolve frame-relative and camera-relative dimensions into concrete allocation sizes, including
full mip-chain counts for descriptors that request `mip_levels = 0`, and allocation slots aggregate the resource states
required by all aliased resources. Current engine-owned camera and frame resources are imported by default; graph-created
advanced resources opt in with `managed_by_graph`.

`RenderGraphTransientResourceStore` allocates graph-owned transient images and buffers from the compiled allocation plan
and binds every aliased logical resource name to the same physical resource in the execution registry. RenderLayer keeps
these stores alive across command recording and submission. This makes graph-owned resources usable by advanced
callbacks; replacing the remaining hand-written barriers with scheduler-generated barriers is still a future migration
step.

Graph-aware frame and camera callbacks can query `RenderGraphExecutionContext::GetCurrentPassTransitions` to inspect the
transitions compiled for their current pass. RenderLayer consumes those image transitions for resources that have bound
`Image` objects in the execution registry, applying the planned target layout before invoking the callback. The compiled
plan also classifies pass-boundary barriers as image layout transitions, same-layout image memory barriers, buffer memory
barriers, or fallback global memory barriers. RenderLayer consumes those barrier plans for graph-aware pass boundaries,
using targeted buffer barriers where a bound `Buffer` exists. Queue changes are represented in the barrier plan, and
RenderLayer applies queue-family ownership release/acquire barriers for imported image and buffer resources when graphics
and compute use different queue families. Actual async queue submission is still future scheduler work.

Built-in camera graph passes also consume graph-planned image transitions at pass boundaries where the existing pass body
matches the graph boundary. Directional shadow rendering, deferred geometry setup, deferred lighting/camera resolve, and
ray tracing camera setup now use the compiled transition plan for bound built-in images, including aggregate bindings
such as the two-image camera GBuffer. The raster camera graph keeps `DeferredCamera` as the stable lighting/resolve pass
name and introduces `DeferredGeometry` before it, so GBuffer/depth attachment-to-shader-read transitions are now graph
boundary transitions. It also inserts a compute-queue `DepthPyramid` producer boundary after deferred geometry. The
producer copies camera depth into mip 0 and hierarchically reduces the graph-owned `Camera.DepthPyramid` mip chain so
allocation, lifetime, scheduling, and barrier ownership are real before downstream occlusion or culling consumers are
introduced.
The current post-processing stack records compute work for SSAO, Bloom copy/downsample/upsample/final mix, SSR reflect,
shared SSR blur, SSR combine, and tone mapping. The camera-color transition from color attachment to storage read/write
is applied from the graph plan before the stack records effect commands.

The callback-slot model is useful for package-local renderers, but advanced algorithms need a more explicit contract. A
renderer that needs custom resources, temporal history, async compute, custom barriers, depth prepasses, visibility
buffers, denoisers, or hybrid raster/ray work should describe its inputs, outputs, execution queue, camera scope, and
ordering constraints instead of depending on hidden callback timing.

## Migration Milestones

1. Baseline and invariants: document the current frame, resources, synchronization points, and validation commands.
2. Pass boundary extraction: split the current monolithic camera rendering function into named internal passes without
   changing scheduling or public renderer APIs.
3. Frame and camera resource model: introduce named frame-global, camera-local, history, and external resources.
4. Minimal render graph skeleton: let passes declare reads, writes, queue type, scope, and execution body while preserving
   the current fixed order.
5. Low-risk pass migration: migrate isolated passes such as deferred lighting or post-processing before geometry and
   shadows.
6. Camera raster pipeline migration: represent the main camera raster path as graph passes while preserving compatibility
   callbacks.
7. Synchronization and frame pacing cleanup: centralize resource state transitions, reduce broad barriers, and remove
   normal-frame device idles where safe.
8. Advanced renderer API: expose registration for frame passes, camera passes, persistent resources, transient resources,
   history resources, and optional built-in dependencies.
9. Advanced rendering foundations: add actual producers and scheduling for common resources such as depth pyramids,
   motion vectors, object/material IDs, temporal history, GPU visibility inputs, async compute work, and render debug
   views. The names and descriptors exist; allocation, production, and scheduling are the next layer.

## Validation

Render-layer refactors should build the relevant executable target before manual editor or app testing. Local render/GPU
validation is intentionally local-only. The focused local validation path is:

```bat
python Scripts\test.py --render-only
```

Direct CTest usage can select the same render-labeled tests:

```bat
ctest --test-dir out/build/vs2026-x64 -C RelWithDebInfo -L render --output-on-failure
```

EcoSysLab compatibility has a focused render/GPU smoke test:

```bat
ctest --test-dir out/build/vs2026-x64 -C RelWithDebInfo -R "EcoSysLab" --output-on-failure
```

That test opens the EcoSysLab project with the package layer registered in-process, instantiates the Apple tree
descriptor, grows the tree for four simulated years using EcoSysLab's normal 30-day timestep, initializes dynamic tree
strands, and advances strand physics while moving the tree entity slowly left and right.
