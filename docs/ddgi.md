# EvoEngine DDGI

[Back to rendering overview](rendering.md)

This page covers EvoEngine's Dynamic Diffuse Global Illumination path. It is intentionally scoped to DDGI; general camera
flow is documented in [rendering.md](rendering.md), and capture/validation commands are in
[rendering-validation.md](rendering-validation.md).

## Ownership

Assigned `EnvironmentalLighting` assets are the runtime source for DDGI settings and volume authoring. Rendering, Bistro,
Cornell, thin-wall, DDGI multi-volume, and generated DDGI validation fixtures author temporary `EnvironmentalLighting`
assets directly. There is no scene-local DDGI private component or extractor fallback. `RenderLayer` selects the resolved
active volumes, owns each volume's resources and convergence state, and exposes aggregate and per-volume runtime/debug
state in the editor.

The authored volume origin is the center of the probe lattice. The first probe position is:

```text
origin - 0.5 * (probe_counts - 1) * probe_spacing
```

EvoEngine supports at most eight enabled DDGI volumes with at most 8192 aggregate resident probes. Enabled paused volumes
retain resources and count toward both limits; disabled volumes count toward neither. An over-limit edit is rejected as a
set without silently dropping a volume. An invalid edit to an existing volume keeps its last valid runtime state, while an
invalid newly created or loaded volume is disabled without rewriting its authored settings.

Enabled volumes are sorted by descending artist priority, descending probe density, then stable entity ID. The first
volume with positive coverage is primary; coverage includes the lattice interior and its one-probe-spacing exterior
influence region so entering a higher-priority volume is continuous. A point deep inside overlapping volumes uses only
the primary. Within the primary's one-spacing interior or exterior boundary band, the first other sorted volume with
positive coverage may contribute as one normalized secondary. No additional volume is accumulated. Uncovered diffuse
weight falls back to diffuse IBL, while specular IBL remains independent.

Only volumes with a valid runtime layout and populated probe history participate in lighting selection. A higher-ranked
volume that is still warming or rebuilding is skipped in favor of the next ready volume, with any remaining weight
falling back to diffuse IBL.

## Runtime Passes

When DDGI runtime or debug visualization is enabled, the frame graph declares:

- frame-local compact DDGI ray-output buffer storing radiance and signed hit distance in 16 bytes per ray;
- per-frame full ray diagnostics for the selected probe, kept separate from the production ray data;
- per-volume persistent probe metadata and state;
- per-volume irradiance, visibility, and variability atlases.

Each active volume owns and executes its own DDGI pass chain:

1. `DDGIAtlasPrepare` clears frame-local ray output every frame and clears persistent atlas/state only when the source,
   layout, scroll slot, or manual reset requires it.
2. `DDGIRayDiagnostics` traces probe rays through the scene TLAS when ray tracing is available.
3. `DDGIProbeUpdate` filters ray results into directional octahedral irradiance and visibility-moment atlas tiles.
4. Deferred lighting and recursive probe-hit shading sample the atlas data when the frame has current DDGI output.

## Ray Generation And Hit Shading

Probe ray directions use spherical Fibonacci samples. Fixed relocation/classification rays are deterministic; non-fixed
rays receive a frame-wide random orientation so neighboring probes rotate coherently without resetting history.

The DDGI TLAS includes mesh-compatible render instances with BLAS data and the standard triangle/material payload. Strands
are ignored by DDGI. External render instances participate only when registered with `DdgiExternalGeometry`.

DDGI closest-hit shading currently:

- evaluates the raster material's UV0-UV3 coordinates, vertex color, world-space tangent frame (including mirrored and
  nonuniform transforms), normal map, base color, and coated emission with a probe-ray footprint derived from hit
  distance, spherical ray density, and triangle texel density;
- uses the geometric normal for ray offsets and the filtered shading normal for Lambert response;
- rejects one-sided backfaces while shading double-sided backfaces; fixed relocation/classification rays retain signed
  geometric backface distance, while non-fixed double-sided hits use positive distance so atlas accumulation keeps their
  radiance;
- evaluates enabled directional, point, and spot lights;
- when emissive sampling is effectively enabled, samples one emissive triangle at each accepted non-fixed probe hit using
  the camera's shared triangle inventory,
  world-area CDF, uniform-area reconstruction, solid-angle PDF, UV0-UV3 material evaluation, and coated LOD-0
  emission. DDGI applies only its Lambert receiver and binary visibility with light-technique weight one; direct emitter
  hits remain unweighted because DDGI has no competing BSDF-continuation technique;
- casts robustly offset hard TLAS shadow rays for shadow-casting direct lights;
- keeps light-debug meshes from shadowing their own point lights by using a shadow-caster TLAS mask;
- samples previous DDGI irradiance/visibility at the hit point for a recursive diffuse bounce.

The Vulkan RT-pipeline path runs a deterministic any-hit alpha-cutoff test for `MASK` materials on both probe and shadow
rays. Primary probe rays use the hit-distance/ray-density texture footprint; nested hard-shadow rays deliberately sample
the alpha mask at LOD 0 so their cutoff matches the deterministic camera/reference shadow policy. Alpha-blended,
transmissive, and refractive surfaces remain fully opaque to those rays, although an accepted probe hit still evaluates
its base color and emissive response. RayQuery DDGI and strand participation are not supported by this path.

The shared emissive inventory admits rigid opaque and alpha-masked deferred/forward meshes. Masked sampled points that
fail the cutoff contribute zero without resampling, preserving the full-triangle area PDF. Moving rigid transforms
rebuild the world-area distribution. Morph/deformed or override-BLAS meshes, skinned meshes, particles, external
instances, alpha-blended/transmissive meshes, strands, and Gaussian splats remain excluded until sampled geometry can
match their TLAS representation exactly. Participating-medium emissive NEE is also outside this overhaul.

Emissive sampling defaults on through `DdgiSettings::RuntimeSettings::enable_emissive_mesh_sampling`. Each asset-owned
DDGI volume can inherit that value or force it on/off. Changing the effective value is a ray-source change: active updates
clear and restart probe history and the deterministic sequence, while a paused volume preserves its current history and
performs that refresh after updates resume. Disabling sampling suppresses only explicit emissive NEE; direct-hit emission
remains.

## Probe Update

Each production update covers the full probe volume. Each frame-in-flight owns its update-index buffer, so probe work
cannot race an older GPU frame.

M6 evaluated round-robin, fixed-budget, and variability-adaptive candidates. They passed the frozen quality, timing, and
memory checks, but converged in 311, 307, and 227 frames respectively against the 44-frame gate. The full-volume policy
was therefore retained and the rejected experimental scheduler switch was removed in M10.

Variability gating requires three consecutive valid complete-volume observations after the authored minimum sample
count. Invalid, non-finite, negative, and zero-weight readbacks break an unfinished consecutive streak without advancing
the sample count; an invalid periodic observation preserves an already-converged state and retries. The exit threshold
is 1.25 times the entry threshold, and a converged volume performs one full refresh observation every 120 eligible
rendered frames. Changing the threshold, minimum sample count, variability enable, or gating enable restarts convergence
and schedules an immediate full update without clearing otherwise valid irradiance history.

Variability-policy changes use `DdgiUpdateReasonVariabilityPolicy` rather than the source bit. They still force a full
zero-hysteresis update, restart variability sampling, and reset the transport sequence, but they are deliberately absent
from atlas clearing, hard probe-state refresh, and the validation history-reset mask. The original reason values remain
stable (`Source` is bit 0 through `PeriodicRefresh` at bit 6); the policy reason is bit 7.

Contributor tracking mirrors the DDGI TLAS: rigid, skinned, particle-instanced, transparent, and DDGI-capable external
geometry participate, while strands, Gaussian splats, and external instances without DDGI geometry do not. Only
ray-visible material fields and referenced texture content, views, and samplers are tracked. The volume's authored
auto-invalidate trigger mask decides which contributor event classes clear probe history and force a full transport
refresh; geometry-triggered auto invalidation also resets relocation/classification state. Its variability-reset mask
independently decides which event classes restart convergence, and its warmup mask decides which run through warmup.
Events remain latched while updates are paused and replay after resume; otherwise ignored events are consumed after policy
evaluation. The supported rigid opaque/masked emissive inventory has a separate stable fingerprint, so its membership,
geometry, transform, material, and importance changes feed the geometry event without reacting to unrelated or unsupported
emitter assets.

The update shader writes:

- irradiance atlas interior texels plus wrapped borders;
- visibility moment atlas interior texels plus wrapped borders;
- probe metadata with irradiance, hit ratio, average hit distance, backface ratio, relocation amount, relocation offset,
  and active state;
- next-frame probe state used by ray generation and surface sampling.

Probe-atlas filtering has a serial fallback path and two cooperative workgroup paths. Set
`EVOENGINE_DDGI_PROBE_UPDATE_VARIANT` before launch to `serial`, `parallel-direct`, or `parallel-shared` to select one
explicitly. The direct path reads compact probe rays from the storage buffer; the shared path first cooperatively caches
up to 256 rays per workgroup. `parallel-shared` is the production default selected by the M4 Sponza comparison. A parallel
request falls back to serial when the device cannot support its workgroup or shared-memory requirements, or when either
required update pipeline fails to initialize. An unknown value also selects the safe serial path. Startup and first
execution emit the requested, selected, and actually executed paths for validation evidence.

A scrolling volume ring-maps compatible history, clears only newly exposed probe slabs before tracing, and performs a
full reset when movement spans a probe-grid dimension.

## Relocation And Classification

Relocation and classification use the fixed-ray prefix:

- inside-geometry probes move along the closest backface direction;
- near-surface probes move toward the farthest opposing frontface;
- probes with clearance move back toward zero offset only when the candidate remains inside the probe voxel ellipsoid;
- classification deactivates probes detected inside geometry from fixed-ray backface ratio.

The Rendering demo keeps relocation enabled but classification disabled because curtain backfaces can incorrectly mark
valid near-wall probes inactive.

## Surface Sampling

Deferred lighting binds safe fallback DDGI atlas textures and one fixed-capacity inactive probe-state buffer sized for the
hard 8192-resident-probe limit. A failed or unavailable atlas binding therefore has zero gather confidence without
out-of-bounds probe-state access. DDGI
sampling is enabled only when the ray/update chain produced current atlases. With DDGI disabled, raster lighting
continues to use the existing IBL ambient path.

Enabled DDGI sampling uses:

- border-aware octahedral irradiance lookup;
- visibility-moment lookup;
- surface-normal bias;
- trilinear probe blending;
- probe-state active/inactive rejection;
- relocated probe offsets;
- volume blend weighting;
- Chebyshev visibility with a low-weight floor.

The shared gather reports volume coverage separately from readiness confidence. Active-probe trilinear support is the
confidence denominator, while irradiance-atlas alpha marks the valid support in the numerator; visibility affects
radiance weighting but not readiness. Full-atlas and scrolling clears write alpha zero, and valid directional updates
write alpha one. A converged black sample therefore remains valid, while a cleared black sample falls back to diffuse
IBL until probes populate it. Deferred and transparent raster lighting compose diffuse indirect as
`I * diffuse_AO * mix(S * diffuse_ibl, ddgi_diffuse_with_S_scaled_sky_misses, clamp(coverage * confidence, 0, 1))`.
`S` is the scene's **Sky Light Intensity Scale** and `I` is **Indirect Lighting Intensity**. DDGI uses the same
Fresnel/metallic diffuse-energy weight. Deferred opaque lighting includes material occlusion and GTAO in `diffuse_AO`;
transparent lighting includes material occlusion but not screen-space GTAO because GTAO is reconstructed from the opaque
depth and GBuffer. Direct light, primary emission, and split-sum reflection-probe specular remain outside `I`.

Metallic surfaces therefore receive no Lambertian DDGI while retaining reflection-probe lighting. Non-finite gather data
falls back to diffuse IBL instead of contaminating the specular path. Recursive probe-hit shading remains Lambert-only and
does not contain `I`; the receiver applies `I` once after diffuse composition. Outside the volume or without any active
finite probe contribution, raster lighting falls back continuously to `S`-scaled diffuse IBL and recursive probe-hit DDGI
returns zero.

Probe miss radiance uses `S`, so `S` and underlying environment-source changes enter the DDGI source signature and refresh
affected probe history. `I` is a resolve-only control and schedules no probe rays. Changing `S` does not rebuild the
intensity-independent environment PDF, diffuse convolution, or GGX-prefiltered cubemap; replacing or editing the source
does. Camera-visible primary background remains controlled by the camera's `background_intensity`, independently of both
lighting controls. SSR is post-processing and ray-traced reflections are not part of this lighting contract.

Asset-owned local reflection probes affect only raster specular IBL and never feed DDGI's diffuse gather or source
signature. An explicit editor reflection-probe bake may include already-converged DDGI diffuse lighting, but it excludes
all local reflection probes. The bake therefore does not invent recursive local specular or a metallic diffuse proxy; see
[Reflection probes](reflection-probes.md) for the capture and fallback contract.

Rough reflection-probe specular may use the scalar visibility from a valid DDGI gather, blended by DDGI coverage and
confidence against white visibility. Disabling DDGI, moving outside every DDGI volume, or losing DDGI coverage therefore
does not darken isolated probe specular, while valid DDGI visibility can reduce leakage on rough probe reflections. DDGI
irradiance remains diffuse-only and is not used as a directional reflection source.

The frozen DDGI baseline manifest, replay evidence, and baseline validator are no longer checked in. Stable ray-camera
reference captures remain under `EvoEngine_Tests/Rendering/DDGI/References/` for manual or ad-hoc image comparison.
Runtime coverage is owned by the focused installed-editor validation scripts for DDGI app, emissive behavior,
multi-volume behavior, environment lighting, and reflection probes.

## Debugging

DDGI debug visualization is scene-camera-only. The main camera is kept free of probe overlays.

With runtime enabled, Pause updates stops probe tracing but preserves compatible history for lighting. Debug-only
resource views never enable GI; with runtime disabled, only Rays traces diagnostic probe rays, and those rays do not
contribute lighting.

Available debug surfaces include:

- `DDGIProbeVisualization` for probe spheres sampled from GPU atlas/state data;
- `DDGIProbeRayVisualization` for selected-probe ray lines captured into the dedicated full diagnostic buffer;
- inspector metadata readback for selected probe irradiance, hit/miss/backface ratios, visibility distance, relocation,
  active state, and update reason;
- inspector emissive readback for the primary volume's effective setting plus shared-triangle, enabled-volume, and
  candidate-ray upper-bound counts. `DDGI Probe Trace` GPU timing includes nested emissive visibility-ray cost;
- a forced atlas readout layout through `Scripts/capture_readme_editor_screenshot.py --ddgi-atlas-preview`.

## Rendering Demo Baseline

The Rendering demo DDGI baseline uses:

- Sponza scene with a tracked sky source, global reflection probe, and five asset-owned hallway/gallery reflection
  probes;
- imported Sponza punctual lights disabled;
- Capoeira disabled for idle editor accumulation;
- asset-owned DDGI volume with 10x6x16 probes;
- 1.5 probe spacing;
- local volume origin `(0, 3, 3)`;
- 256 rays per probe;
- 245760 ray samples per full update;
- 0.02 normal/visibility bias;
- relocation enabled;
- classification disabled;
- full-volume production scheduling;
- top-down white directional light brightness `5.0`;
- yellow point-light/debug sphere marked non-shadow-casting.

`DDGIApp` is the cleaner player-mode Cornell-box comparison binary. It keeps DDGI runtime lighting enabled, clears
skybox/post-processing, disables DDGI debug overlays, and isolates placement/filtering/final-gather behavior. It defaults
to editor mode, so validation selects `--player` explicitly. The M9 closeout capture runs the installed application once
at exactly 1920x1080, requires a render-ready DDGI probe update, and verifies the resulting non-black PNG without changing
the installed ImGui layout:

```powershell
python Scripts/run_ddgi_app_validation.py --config RelWithDebInfo
```

The dedicated application launch is an integration check for DDGI placement, filtering, update readiness, and final
gather behavior. Bistro smoke likewise uses a per-run ImGui INI because its automated full-resolution Scene preview is
not the normal editor layout.

## RTXGI Port Map

The sibling `RTXGI-DDGI` checkout (typically `../RTXGI-DDGI` from the repository root) is treated as an algorithm
reference only. EvoEngine keeps SDK shader source in Slang and does not add an HLSL, DXC, or RTXGI runtime dependency.

| RTXGI concept | EvoEngine path |
| --- | --- |
| Volume descriptors/resources | `DdgiSettings`, asset-owned DDGI volume entries, and per-volume probe metadata/state plus irradiance, visibility, and variability atlases |
| Probe ray generation | `Shaders/RayTracing/RayGen/DDGIProbeDiagnostics.slang` |
| Probe closest-hit/miss | `Shaders/RayTracing/ClosestHit/DDGIProbeDiagnostics.slang`, `Shaders/RayTracing/Miss/DDGIProbeDiagnostics.slang` |
| Shared probe math | `Shaders/Includes/DDGI.slangh` |
| Probe atlas update | `Shaders/Compute/DDGIProbeUpdate.slang` |
| Surface sampling | `Shaders/Includes/Lighting.slangh` and DDGI closest-hit recursive sampling |
| Scheduling and clears | `RenderLayer` DDGI passes: `DDGIAtlasPrepare`, `DDGIRayDiagnostics`, `DDGIProbeUpdate` |

EvoEngine implements deterministic multi-volume overlap isolation from RTXGI's integration guidance rather than copying
the reference test harness's order-dependent accumulate-all loop. Broader scene-level atlas seam/leak/classification
validation scenes remain future work. Variability reduction/readback and bounded application-level convergence scheduling
are implemented; RTXGI itself leaves the scheduling policy to the integration.
