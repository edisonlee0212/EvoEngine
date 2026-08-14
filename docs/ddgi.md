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

DDGI volume transforms use the same Environmental Lighting authoring control as local reflection probes: Position,
Euler Rotation in degrees, and Scale, composed as translation, rotation, then scale. Serialization remains matrix-based.
The inspector normalizes perspective or shear to decomposed TRS and resets a non-finite, singular, or undecomposable
matrix to identity while marking the asset modified.

For an `EnvironmentalLighting` asset assigned to the active scene, a DDGI volume's **Edit in Scene** toggle temporarily
replaces entity selection with the Scene-camera ImGuizmo. It reuses the editor toolbar's local Position, Rotation, and
Scale modes; clicking a Position, Rotation, or Scale label in the volume's Transform controls selects that same global
mode. The gizmo pivot is placed at the visible lattice center (`transform * volume_origin`). The inverse conversion
updates only the stored matrix, so `volume_origin` is preserved. The active authored bound is drawn once as a translucent
orange box even when the volume is disabled. Selecting an entity or empty viewport space, pressing Escape, removing or
reloading the volume, replacing the scene asset, or closing its asset inspector clears the transient target. The target
is editor-only and is never serialized or displayed over the game main camera.

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

When DDGI runtime is enabled, the frame graph declares:

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
  world-area/emissive-texture alias table, uniform-area reconstruction, solid-angle PDF, UV0-UV3 material evaluation,
  and coated LOD-0
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

The shared emissive inventory admits fill-mode rigid, skinned, particle-instanced, override-range, and external triangle
geometry across deferred, forward, and transparent collections. External DDGI geometry must provide `triangle_count` in
addition to its packed offset. Emissive texture content, UV set/transform, runtime material changes, geometry versions,
and moving transforms refresh the proposal and DDGI source signature. Opaque, masked, blend, transmissive, and
diffuse-transmissive emission use the same sampled material evaluator as the camera ray path; blend emission is weighted
by sampled opacity. Unlit materials, strands, Gaussian splats, and external triangle ranges without a count remain
unsupported by emissive NEE and are reported when they have positive emission.

Triangle selection uses a log-domain alias-table build, so a positive emitter retains a representable probability even
beside an emitter with extreme power. Texture-aware importance uses seven barycentric samples per triangle and falls back
to factor luminance when CPU texels are unavailable; this changes only the proposal, not the exact sampled-point PDF or
radiance. Base-color and vertex alpha reduce the proposal weight of mostly cut-out or transparent triangles while a
support floor preserves unbiased sampling. Rejected masked points and one-sided backfaces still contribute zero without
conditional retry. A retry would require the exact receiver-dependent acceptance probability to remain unbiased, while
M1 measured those rejects as a small
fraction of probe work, so M2 retains the single-sample estimator.

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
count. Each texel first scales coefficient of variation by its mean irradiance up to a 0.25 reference level, suppressing
near-dark numerical ratios without hiding bright changes. Each observation then reduces active probe texels to an
average, maximum, and fraction above the authored threshold. M3 captures rejected a raw maximum plus p95 gate because
isolated stochastic texels prevented convergence for 1024 frames even after energy weighting. The reviewed entry policy
therefore requires the average to remain below the threshold, at least 85% of texels to be at or below it, and the
maximum to stay below forty times it. This prevents a materially unstable receiver region or severe localized outlier
from disappearing into a volume-wide mean without treating every rare sample as permanent motion. Exit uses 1.25 times
the average/maximum limits and a 15% unstable fraction. Invalid, non-finite,
negative, inconsistent, and zero-weight readbacks break an unfinished consecutive streak without advancing the sample
count; an invalid periodic observation preserves an already-converged state and retries. The 15% localized allowance is
paired with the mean and 40x maximum guards; it accommodates correctly retained black history without allowing a severe
localized outlier to disappear into the volume average. A converged volume normally pauses tracing and performs one full
stochastic refresh observation every 120 eligible rendered frames. **Pause updates after convergence** is independent of
variability gating: disabling the pause keeps tracing and variability observations active every rendered frame while
preserving convergence/readiness diagnostics. Changing the threshold, minimum sample count, variability enable, or
gating enable restarts convergence and schedules an immediate full update without clearing otherwise valid irradiance
history. Changing only the pause option preserves the current history and convergence state and changes scheduling on
the next frame.

`Scripts/run_ddgi_localized_convergence_validation.py` exercises rare equal-power emission, emissive enable, emissive
motion, and non-emissive geometry motion against the installed editor. Its evidence records localized metrics,
transition reset behavior, repeatability, convergence frames, and the exact rays avoided by the post-convergence
120-frame refresh schedule over a fixed 360-frame horizon.

Variability-policy changes use `DdgiUpdateReasonVariabilityPolicy` rather than the source bit. They still force a full
zero-hysteresis update, restart variability sampling, and reset the transport sequence, but they are deliberately absent
from atlas clearing, hard probe-state refresh, and the validation history-reset mask. The original reason values remain
stable (`Source` is bit 0, `SceneChange` is bit 5, and `PeriodicRefresh` is bit 6); the policy reason is bit 7 and
`HysteresisRestore` is bit 8.

Irradiance history uses the atlas alpha channel as a validity marker and repeated-bright-observation confidence. This
keeps valid black history distinct from a newly cleared atlas. A lone bright observation retains the selected hysteresis
and the conservative RTXGI-style bright-delta scale; repeated observations progressively relax both, while a
gamma-domain step limit tied to the authored brightness threshold bounds the response. Confidence decays when the bright
evidence stops. The pre-gamma `64.0` irradiance ceiling remains a finite-data guard rather than a temporal clamp.

Light membership, lighting, emissive-inventory, geometry, and material-closure changes never clear compatible probe
history or reset relocation/classification state. Geometry changes do schedule one history-preserving relocation update.
Enabled triggers otherwise snap the volume's current hysteresis to the
Render Layer's boosted value, which defaults to `0.85`, and force a probe update. Continued changes hold that value.
After the first quiet frame, hysteresis moves toward the separately configurable `0.97` normal value by the `0.01`
restore-speed default before each forced update. The final `0.97` update completes recovery, after which normal
convergence gating resumes. This response neither starts warmup nor changes its frame counter. Manual reset,
incompatible resources, volume-source changes, and full scrolling reset remain hard invalidations. Emissive factor and
emissive-texture changes are classified by the shared emissive inventory as lighting changes; the material-closure
fingerprint deliberately excludes those fields. A changed emissive-ray population also leaves warmup untouched.

Contributor tracking mirrors the DDGI TLAS: rigid, skinned, particle-instanced, transparent, and DDGI-capable external
geometry participate, while strands, Gaussian splats, and external instances without DDGI geometry do not. Only
ray-visible material fields and referenced texture content, views, and samplers are tracked. The volume's authored
**Hysteresis boost triggers** mask defaults to all contributor event classes and decides which events activate the boost
described above. The variability-reset mask independently decides which event classes restart convergence. Cold-start
warmup still ramps the Render Layer's normal hysteresis from zero to populate empty history quickly. Legacy
`scene_change_hysteresis_trigger_conditions`, `warmup_trigger_conditions`, and `auto_invalidate_trigger_conditions`
values are migrated into `hysteresis_boost_trigger_conditions` when an older asset is loaded and are no longer serialized.
Events remain latched while updates are paused and replay after resume; otherwise ignored events are consumed after policy
evaluation. Boost recovery also freezes while paused. A zero restore speed intentionally holds boosted hysteresis and
forced updates until the setting changes or DDGI is reset. The shared emissive inventory has a separate stable
fingerprint, so its membership, geometry, transform, emissive material, texture, and importance changes feed the
lighting event without reacting to unrelated or unsupported emitter assets.

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

Relocation is intentionally separate from steady-state irradiance tracing. It runs during cold-start/reset warmup,
once for a geometry-change event, and for newly exposed probe bands when a scrolling volume advances. The resulting
offsets remain frozen during ordinary continuous irradiance updates, preventing near-surface probes from repeatedly
oscillating between the outward and return-to-grid relocation rules. Classification remains part of each traced update.

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
`diffuse_AO * mix(F * diffuse_ibl, ddgi_diffuse_with_E_scaled_sky_misses, clamp(coverage * confidence, 0, 1))`.
`E` is **Environment lighting intensity** and `F` is **Diffuse fallback intensity**. DDGI uses the same
Fresnel/metallic diffuse-energy weight. Deferred opaque lighting includes material occlusion and GTAO in `diffuse_AO`;
transparent lighting includes material occlusion but not screen-space GTAO because GTAO is reconstructed from the opaque
depth and GBuffer. Direct light, primary emission, and split-sum reflection-probe specular remain outside this composition.

Metallic surfaces therefore receive no Lambertian DDGI while retaining reflection-probe lighting. Non-finite gather data
falls back to diffuse IBL and the global prefiltered specular source. Recursive probe-hit shading remains Lambert-only.
Outside the volume or without any active finite probe contribution, raster lighting falls back continuously to `F`-scaled diffuse IBL and recursive probe-hit DDGI
returns zero.

The Render Layer's `DDGI Probe Blend Loss` indirect-lighting diagnostic displays the non-negative irradiance difference
between energy-preserving linear cross-probe interpolation and the production square-root-domain interpolation. It uses
the same valid probes, visibility weights, multi-volume composition, coverage, and confidence as production gathering.
Black means the neighboring probe samples agree; brighter colored regions identify energy reduced by nonlinear blending.
The view is diagnostic only and does not change beauty rendering, probe history, or atlas contents.

Probe miss radiance uses `E`, so `E` and underlying environment-source changes enter the DDGI source signature and refresh
affected probe history. `F` is a resolve-only fallback control and schedules no probe rays. Changing `E` does not rebuild the
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
irradiance also supplies a broad fallback where local reflection-probe weight is missing. The fallback reuses the existing
normal-directed gather with no additional atlas samples, converts irradiance to radiance with `/ pi`, and blends by DDGI
coverage and confidence at every material roughness. It is not multiplied by
`specular_fallback_intensity`; that setting controls the original global prefiltered fallback blended underneath it. This
low-frequency proxy does not provide sharp or reflection-direction detail even for smooth materials, never replaces valid
local-probe weight, and is disabled during reflection-probe capture to prevent feedback.

The frozen DDGI baseline manifest, replay evidence, and baseline validator are no longer checked in. Stable ray-camera
reference captures remain under `EvoEngine_Tests/Rendering/DDGI/References/` for manual or ad-hoc image comparison.
Runtime coverage is owned by the focused installed-editor validation scripts for DDGI app, emissive behavior,
multi-volume behavior, environment lighting, and reflection probes.

## Small-Emitter Baseline

`Scripts/run_ddgi_small_emitter_baseline.py` captures the small-emitter evidence contract. It compares the existing
large emitter with a small emitter at equal radiance and a small emitter whose radiance is scaled by the transformed
box surface-area ratio so total emitted power matches the large source. No analytic-light proxy participates.

Every fixture produces linear-HDR DDGI-enabled, DDGI-disabled, and 256-SPP Vulkan ray-tracing reference captures at
1920x1080. The script reports receiver-wall and receiver-floor luminance, isolated DDGI contribution, convergence,
logical memory, GPU timestamps, and deterministic equal-power repeatability. The RTX 5070 is the authoritative
same-machine performance device; other supported Vulkan devices remain correctness targets.

```powershell
python Scripts/run_ddgi_small_emitter_baseline.py --config RelWithDebInfo
```

Evidence is written under `out/ddgi-small-emitter/m0` by default. The approved acceptance limits are 15% equal-power
small-versus-large DDGI energy error, 30% DDGI-versus-path-traced receiver energy error, 0.30 energy-normalized receiver
relative L2, `1e-6` repeat relative L2, 48 static convergence frames, and 5% aggregate DDGI median GPU-time regression
on the RTX 5070.

`Scripts/run_ddgi_emissive_m2_validation.py` is the installed-editor acceptance matrix for the shared alias-table
sampler. It covers equal-power repeatability, multiple emitters, textured UV0/UV3 emitters, alpha-cutout rejection, and
one- versus double-sided behavior with exact GPU outcome accounting:

```powershell
python Scripts/run_ddgi_emissive_m2_validation.py
```

`Scripts/run_ddgi_temporal_response_validation.py` captures frame-exact emissive enable, disable, and HDR response curves
at 1, 2, 4, 8, 16, and 32 update frames. It compares them with settled output and the legacy `0.97` hysteresis plus
`0.25` bright-delta recurrence, and checks exact steady repeats for large emissive, environment, and analytic-light
fixtures. Reports use schema 6 and record `capture.response_frames`.

```powershell
python Scripts/run_ddgi_temporal_response_validation.py
```

## Emissive Mesh Authoring

The `MeshRenderer` and `SkinnedMeshRenderer` inspectors include a **DDGI emissive authoring** section. It keeps the
emissive mesh as the only light representation and shows HDR radiance RGB/luminance, estimated world-space emitting
area, factor-only emitted power (`pi * area * luminance`, doubled for a double-sided material), positive-area triangle
candidates, and the current authoring eligibility reason. Skinned-mesh area is explicitly labeled as a bind-pose
estimate.

The same panel exposes emissive radiance, texture, sidedness, and alpha mode, and explains their sampling cost. A
textured-emission preview is labeled factor-only because the live DDGI inventory includes texture and opacity energy;
the Render Layer DDGI inspector remains authoritative for runtime inventory power, exclusions, and zero-probability
entries after geometry upload.

`Apply target power to radiance` is an explicit user action. It preserves radiance chromaticity by scaling the material
emissive factor against the current area preview. It never creates, pairs, or edits an analytic light, and the engine
never changes radiance automatically when mesh area changes.

## Debugging

The Render Layer inspector is runtime-focused and split into compact Overview, Visualization, and Diagnostics sections.
Persistent global settings and all per-volume authoring live in Environmental Lighting. Inspector state is session-only:
it is not serialized, is cleared on scene load, and applies to every active volume in the scene.

Pause updates stops probe tracing while retaining compatible atlases for lighting. Newly enabled volumes may allocate
their persistent resources while paused, but remain uninitialized until updates resume. Reset history is hidden while
paused. Disabling or removing a volume stops its contribution immediately. Debug controls never enable DDGI or trace
probe rays when the runtime is disabled.

The Overview reports scene state, aggregate probes and resident memory, then one row per runtime volume with name,
stable ID, grid, memory, convergence/warmup state, and an explicit Inspect action. Invalid authored volumes remain visible
with their rejection reason. The selected diagnostic target is a stable volume ID plus probe-grid coordinate; there is no
viewport picking or implicit retargeting. This runtime diagnostic selection is separate from the Environmental Lighting
inspector's explicit whole-volume **Edit in Scene** authoring toggle.

Stable IDs preserve each volume's runtime history, resources, ordering tie-break, and diagnostic selection when authored
volumes are reordered. Environmental Lighting repairs missing or duplicate IDs to deterministic nonzero values when an
asset loads or is edited. Runtime validation rejects invalid programmatic volume sets before they can alias one state.

| Visualization target | Probe spheres | Selected marker | Selected rays |
| --- | --- | --- | --- |
| Raster editor scene viewport | Yes, all runtime volumes | Yes | Yes |
| Game main camera | No | No | No |
| RTX or RayQuery camera | No | No | No |

`Invalid` means the authored runtime set failed atomic validation; inspect the error row and Environmental Lighting
settings. `Resources unavailable` means the selected volume has not prepared its buffers/atlases. `Paused / stale frozen`
means tracked scene inputs changed while the last valid atlas remains in use. `Reset pending` clears only after every
valid runtime volume traces the reset frame. A missing diagnostic target is intentional after its volume is disabled or
removed; choose `Inspect` on another row explicitly.

Available debug surfaces include:

- the Environmental Lighting inspector's DDGI volume bounds, drawn as translucent orange filled boxes with depth testing
  and no depth writes;
- `DDGIProbeVisualization` for every active volume, sampled from its GPU atlas/state data. Radius is a fraction of that
  volume's minimum transformed probe spacing;
- `DDGIProbeRayVisualization` for the explicitly selected volume and probe only;
- inspector metadata readback for selected probe irradiance, hit/miss/backface ratios, visibility distance, relocation,
  active state, and update reason;
- an opt-in emissive sampling readback with actual eligible-hit, NEE-attempt, triangle-selection, categorized rejection,
  shadow, zero-radiance, and nonzero-contribution counts. The older candidate-ray value is explicitly labeled as an
  upper bound. The same counters are emitted under `emissive_sampling` in validation report schema 5;
- emissive inventory diagnostics for estimated emitted power, eligible and excluded positive-emission instances, and
  unrepresentable sampling probabilities. The inspector warns when geometry is excluded or positive power cannot be
  represented in the alias table. `DDGI Probe Trace` GPU timing includes nested emissive visibility-ray cost;
- explicit probe and ray opacity controls, with selected-probe state readback scheduled only while its readout is open.

Exact emissive event counters are disabled by default. `Capture emissive sampling` is a coalesced, scene-wide one-shot
request. It waits while paused, captures ready volumes after resume, and retains the latest completed snapshot. Only an
active request allocates the small counter/readback buffers and enables shader atomics.

`Capture isolated gather timing` is also a one-shot action. It deliberately repeats the DDGI gather in a separate
full-resolution pass so GPU timestamps can isolate gather cost; enabling generic live GPU timing no longer pays that
profiling pass unless this DDGI option is selected. Headless DDGI reports and the multi-volume performance validation
opt in explicitly. On the RTX 5070 M4 captures, the isolated pass cost approximately 0.15 ms at 1920x1080, which is now
avoided during ordinary live GPU profiling. This changes profiling overhead only, not the deferred-lighting estimator.

The M6 RTX 5070 / 1920x1080 profile recorded the following median milliseconds. `Gather profile overhead` is the
explicit duplicate measurement pass and is not production deferred-lighting cost when the option is off.

| Scene | Probe trace | Atlas update | Gather profile overhead |
| --- | ---: | ---: | ---: |
| Equal-power small emitter | 0.041 | 0.061 | 0.147 |
| Large emitter | 0.039 | 0.061 | 0.153 |
| Cornell | 0.091 | 0.111 | 0.142 |
| Sponza | 0.416 | 0.242 | 0.540 |
| Scrolling | 0.040 | 0.060 | 0.147 |
| Eight-volume overlap | 0.209 | 0.090 | 0.489 |

Evidence is in `out/ddgi-small-emitter/m4-baseline/evidence.json`, `out/ddgi-small-emitter/m6-profile`, and
`out/ddgi-small-emitter/m6-multivolume/evidence.json`. Production emissive event atomics and selected-ray readbacks were
already disabled when their debug options are off. No probe-trace or gather estimator rewrite was adopted in M6: the
remaining static small-emitter error requires correctly weighted emitter-directed probe sampling, while the
reviewed shortcuts would bias the directional atlas or trade correctness for an unproven timing gain.

## Graphics QoL Closeout

The M0-M7 series was validated on an NVIDIA GeForce RTX 5070 at 1920x1080. The installed-editor evidence is under
`out/ddgi-small-emitter/m7-app`, `m7-emissive`, `m7-convergence`, and `m7-temporal`; the eight-volume timing and lifecycle
evidence remains under `m6-multivolume`. The final runtime captures cover application readiness, direct emissive-mesh
sampling, exact repeats, localized convergence, light and geometry changes, temporal response, scrolling, and
multi-volume removal without introducing an analytic-light representation.

The production closeout commands and installed executables are:

```powershell
python Scripts/install_apps.py --config RelWithDebInfo --no-open --incremental --jobs 16
cmake --preset vs2026-x64 -DEvoEngine_App-DDGIApp=ON
cmake --build out/build/vs2026-x64 --config RelWithDebInfo --target DDGIApp --parallel 16
cmake --install out/build/vs2026-x64 --config RelWithDebInfo
```

- Editor: `out/install/vs2026-x64/bin/EvoEngineEditor.exe`
- Cornell DDGI player: `out/install/vs2026-x64/bin/DDGIApp.exe`

The complete CTest run passed 558 of 559 tests on its first pass, including all rendering goldens. Its sole failure was
the expected SDK shader-policy inventory delta for the new shared emissive-sampling module (176 modules and 287 imports
instead of 175 and 285); the checked policy manifest was regenerated and the focused policy gate then passed. Focused
DDGI tests, production shader compilation/reflection, Python syntax checks, C++ formatting, runtime validators, and
application installation also passed.

The uniform-only static small-emitter fixture is about 92% below the equal-power large-emitter DDGI result. Diagnostics
and ablations isolate the dominant loss to uniform probe-direction/direct-hit probability, rather than emitter
inventory, temporal retention, or convergence freeze. The exact emissive-ray path below addresses that probability while
preserving the directional irradiance-atlas estimator and direct emissive mesh as the sole light representation; an
analytic proxy and unweighted ray steering remain out of scope.

### Exact Emissive-Ray Estimator Contract

The production population is fixed at 192 uniform structural rays plus 64 exact emissive-triangle rays. There is no
environment-directed population. Uniform misses still evaluate the normal scene environment, while emissive rays use
the same power-weighted emissive-triangle inventory and alpha/material eligibility as camera-ray NEE.

The two populations are separate Monte Carlo estimators. Uniform rays retain environment misses, analytic-light NEE,
emissive-triangle NEE at surface hits, and recursive DDGI, but omit direct surface emission whenever emissive rays are
active. Each emissive ray selects an exact triangle and point, traces the real TLAS to that point, accepts the sample
only when the first hit is the selected instance and primitive, and contributes only that emitter's direct radiance.
This separation prevents the same direct-emission integral from being counted by both populations.

For a texel direction `n`, uniform radiance retains the existing self-normalized DDGI update. The emissive lane adds
`sum(L_e * max(dot(w, n), 0) / p_emissive(w)) / (2pi * N_emissive)`. Misses, blockers, backfaces, alpha rejection, and
invalid samples remain zero-valued samples in the fixed 64-ray denominator. The two linear HDR results are summed before
the existing clamp, gamma encoding, temporal hysteresis, and atlas write. Visibility moments, hit-distance metadata,
relocation, classification, and excessive-backface evidence consume only the 192 uniform rays.

Only emissive rays store compact sampling metadata: an octahedral direction and 32-bit inverse solid-angle PDF in 8
bytes. At 384 probes the default therefore allocates `384 * 64 * 8 = 196608` sample-info bytes. Setting
`emissive_ray_count` to zero restores the uniform direct-emission path and omits this buffer. Legacy YAML
`guided_ray_count` migrates to `emissive_ray_count`; the obsolete emitter-limit key is ignored. The bounding-sphere guide
buffer, top-K emitter cap, mixture-PDF estimator, and environment proposal have been removed.

The earlier 192+64 bounding-sphere experiment motivated the population split but is not performance or quality evidence
for this exact-triangle estimator. Installed-editor validation must be rerun before quoting settled-energy, temporal-
response, flicker, or GPU-time results for the new path.

## Rendering Demo Baseline

The Rendering demo DDGI baseline uses:

- Sponza scene with a tracked sky source, global reflection probe, and five asset-owned hallway/gallery reflection
  probes;
- imported Sponza punctual lights disabled;
- Capoeira disabled for idle editor accumulation;
- asset-owned DDGI volume with 10x6x16 probes;
- 1.5 probe spacing;
- local volume origin `(0, 3, 3)`;
- 192 uniform plus 64 exact emissive rays per probe;
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
| Shared probe math | `Shaders/Modules/EvoEngine/DDGI.slang` |
| Probe atlas update | `Shaders/Compute/DDGIProbeUpdate.slang` |
| Surface sampling | `Shaders/Modules/EvoEngine/Lighting.slang`, `DDGIGatherMulti.slang`, and DDGI closest-hit recursive sampling |
| Scheduling and clears | `RenderLayer` DDGI passes: `DDGIAtlasPrepare`, `DDGIRayDiagnostics`, `DDGIProbeUpdate` |

EvoEngine implements deterministic multi-volume overlap isolation from RTXGI's integration guidance rather than copying
the reference test harness's order-dependent accumulate-all loop. Broader scene-level atlas seam/leak/classification
validation scenes remain future work. Variability reduction/readback and bounded application-level convergence scheduling
are implemented; RTXGI itself leaves the scheduling policy to the integration.
