# Dynamic Diffuse Global Illumination

[Back to rendering overview](rendering.md)

Dynamic Diffuse Global Illumination (DDGI) stores scene irradiance in a grid of probes and samples that grid during
raster lighting. It provides diffuse indirect light and visibility; it is not a source of sharp specular reflection.

EvoEngine's DDGI probe tracing uses the Vulkan ray-tracing pipeline. A device without ray-tracing support can still run
the raster renderer, but it cannot update DDGI probes.

## Automatic Cascades

EnvironmentalLighting owns the shared camera-following probe layout: 33 x 17 x 33 probes per cascade,
four cascades, horizontal base interval 0.8 and Y Scale 100% by default. Each cascade doubles the interval.
DDGI uses the same nominal positions, half-interval snapping and signed integer scroll deltas as SDFGI.
Changing SDFGI's voxel spacing does not change DDGI coverage.

Select **Environmental Lighting → GI → Indirect GI provider → Automatic DDGI** on an RT-enabled startup.
Authored DDGI packs, transforms, priorities and per-volume overrides are removed; obsolete serialized pack keys
are ignored without deleting resource files. The runtime creates one allocation per
cascade, with stable scene/cascade identity. Auxiliary cameras and reflection captures cannot move the field.

There is no artificial 8,192-probe cap. Shared odd counts 3..257, atlas dimensions, buffer/ray limits and
the strict history budget determine whether a layout is supported. Preferred atlas packing is retained
when it fits and repacked into bounded rows otherwise.

## Settings and runtime edits

The GI tab contains shared **Probe settings**, **Indirect GI provider**, and selected-provider controls.
Shared odd probe counts, cascade count, base distance, Y Scale and optional anchor determine nominal coverage.
DDGI-specific controls include tracing, shared irradiance/visibility history, tile resolutions, biases, relocation and classification.
RenderLayer contains only session pause and visualization/debug controls, not duplicate persistent settings.

Python exposes `GiSettings` (`probes`, `provider`, `sdfgi`, `ddgi`), `GetCurrentSceneGiSettings`,
`SetCurrentSceneGiSettings` and `GiSettings.validate()`. Submit the whole bundle when changing a provider
and its layout together. Failed edits leave the previous configuration intact; unsupported loaded values
remain authored and use fallback before allocation. Only the active provider counts toward the steady-state
history budget; inactive and fence-retiring allocations do not block a valid provider switch.

## Cascade Selection

All lighting consumers, including recursive ray-hit feedback and reflection capture, gather the finest
containing cascade. A two-nominal-probe boundary band blends toward coarser cascades; the last cascade
fades to environment diffuse. Invalid or uninitialized probes leave environment fallback weight.
All cascade clears/scroll clears are recorded before any cascade traces against the shared atlas set.

## Runtime Flow

Each active cascade owns persistent probe state, irradiance/visibility atlases, and quantized irradiance history. The render graph
records the following work when required:

1. Prepare or clear persistent state for a new layout, reset, or newly exposed scrolling region.
2. Trace probe rays through the scene acceleration structure.
3. Filter ray results into octahedral irradiance and visibility atlas tiles.
4. Update relocation and classification, then invalidate moved/reactivated probes before publication.
5. Sample ready atlases during deferred lighting and recursive DDGI hit shading.

Probe directions use a spherical Fibonacci distribution. Fixed relocation/classification rays are deterministic; normal
lighting rays receive a periodic per-cascade rotation. The default runtime settings trace 64 scene rays and 8 exact
emissive-triangle rays per updated probe.

The DDGI acceleration structure includes supported triangle geometry and explicitly registered external DDGI geometry.
Strands and Gaussian splats are not traversed. Alpha-masked triangle hits use deterministic cutoff testing. Blended and
transmissive surfaces use straight-through traversal with colored attenuation for probe, emissive-target, and visibility
rays. Volume attenuation is applied between entry and exit intersections, but DDGI does not refract the ray direction.

## Rolling Probe History

**Environmental Lighting → GI → Automatic DDGI settings → History count** selects 5–30 probe updates in steps of 5 (default 30).
The value belongs to the scene's EnvironmentalLighting DDGI settings, is serialized as `runtime.history_count`,
and is available through Python's `GetCurrentSceneDdgiHistoryCount` and `SetCurrentSceneDdgiHistoryCount`.
Missing keys use 30; obsolete hysteresis, temporal-response, convergence, and per-volume trigger keys are ignored.

Each active cascade stores interior irradiance texels in a ring. Irradiance samples are linear and bounded
to 64 before 16-bit quantization. Integer
32-bit running sums replace the outgoing quantized sample exactly; averaged irradiance is gamma-encoded only when
writing the lighting atlas. Borders are rebuilt from those averaged interiors. Invalid estimates leave the old slot intact.

New/reset histories start at zero and always divide by the full selected window. Both uniform rotations and emissive
sampling repeat over the same window using stable cascade/probe identities and optional custom seeds. The phase advances
only after a completed paired irradiance/visibility update, not with display-frame indices. In-flight updates reserve
successive ring slots without waiting on fences or reducing normal update cadence.
For unchanged sampled inputs, replacing a repeated sample leaves the integer sum exactly unchanged. Bounce feedback,
lights, geometry, and relocation can still change those inputs.

Updates continue after filling the window. Manual pause remains available; there is no convergence stopping or
hysteresis boost. Ordinary lighting changes roll through the ring. Window/ray-population/seed changes and incompatible
layouts reset history. Scrolling preserves retained physical slots and clears newly exposed probes. Relocation and
classification changes clear affected histories and atlas tiles before their new positions are sampled.
Reflection-probe capture waits for an initial history window and relocation warmup.

Steady-state history allocations across GI providers must remain **strictly below 4 GiB**, including rings, integer
sums, SDFGI scroll scratch, and DDGI history-origin records. Device-padded allocation sizes are used in runtime preflight;
device buffer/image limits also apply. Invalid edits are rejected without silently reducing quality. Retiring allocations
are excluded from this steady-state limit and remain fence-retained.

This rolling-history policy intentionally departs from DDGI hysteresis, following the integer-ring model in pinned Godot
`C:/Users/lllll/Documents/GitHub/godot` at `34d06658a85845111a50db9e485ec4a0701d4298`.
Godot remains the first reference when behavior is uncertain; DDGI's geometry-based relocation is unchanged.

The rolling-history delivery was checked on RTX 5070 with an installed 2560×1440 Sponza run, RT pipeline/query and
BLAS/TLAS enabled, 30 → 5 → 30 history transitions, finite float captures, and continued per-frame periodic updates.
Focused CPU/shader and Vulkan fixtures cover quantized startup/replacement, rejected samples, moved/reactivated probe
invalidation, and signed scrolling. Appearance acceptance remains manual; old hysteresis images are not acceptance baselines.

## Rolling Visibility

Visibility uses the same periodic history window as irradiance: two 16-bit moment samples and two 32-bit running sums
per interior texel. Samples use the existing cascade distance bound and its square. New histories are zero-filled and
always divided by the full window; rejected/nonfinite samples preserve valid slots. Origin changes, reactivation and
scroll exposure reset affected histories before reuse. The averaged output remains RG16F with regenerated borders.
The former visibility-smoothing setting is ignored on load and removed from active APIs.

Default irradiance and visibility interior tiles are both 8x8; explicit saved resolutions are preserved.
Four cascades with 33x17x33 probes and 30 updates use 1,821,086,784 logical history bytes (about 1.696 GiB),
including both rings/sums and origin records. Visibility 12x12 uses about 2.402 GiB; 16x16 uses about 3.391 GiB.
Device padding, output atlases and ray buffers remain additional allocations. Runtime edits recreate incompatible
layouts and reject configurations whose padded histories reach 4 GiB, without silently reducing quality.

Visibility-history restoration passed 126 focused CPU/shader/GPU tests, including 100 repeated quantized windows,
invalid-estimate preservation and moved/reactivated-probe clearing. Installed RTX 5070, RT-enabled 2560x1440 Sponza
coverage passed visibility 8/16/8 transitions, signed scrolling, teleports, provider switching, memory rejection and
finite captures. Flicker and light-leak appearance acceptance remains manual.

## Relocation And Classification

Relocation moves probes away from nearby or enclosing geometry while keeping the offset within the probe voxel. It runs
during initialization, reset, scrolling exposure, and relevant geometry changes rather than oscillating every steady
lighting frame.

Classification is enabled by default and marks probes inside geometry inactive. It can reduce leaking and wasted work, but thin or double-sided
content may require leaving classification disabled. Relocation and classification use a deterministic prefix of the
probe-ray set and do not change the number of lighting rays.

## Surface Lighting

The gather reconstructs diffuse irradiance with border-aware octahedral lookup, trilinear probe blending, surface bias,
relocated positions, active-state rejection, and visibility moments. Coverage and readiness confidence are tracked
separately so valid black irradiance is not confused with missing data.

Raster diffuse indirect resolves as:

```text
valid DDGI coverage
  -> DDGI irradiance
otherwise
  -> indirect environment source * diffuse_fallback_intensity
```

Probe-ray misses sample the indirect environment source with `environment_lighting_intensity`. Changing the diffuse
fallback intensity does not retrace probes because it affects only the uncovered raster fallback. Camera-visible
backgrounds remain controlled by the camera.

DDGI visibility can reduce leakage in rough reflection-probe lighting. DDGI irradiance may also provide a broad
low-frequency fallback beneath missing global probe weight, but it does not replace a valid local reflection probe or
become a view-dependent reflection source. See [Reflection probes](reflection-probes.md).

## Emissive Geometry

DDGI supports direct hits on emissive geometry and optional explicit emissive-triangle sampling. The explicit estimator
shares the camera ray path's eligible triangle inventory, material evaluation, texture transforms, and area-to-solid-angle
probability calculation.

Eligible rigid, skinned, instanced, and compatible external triangles can enter the emissive distribution. Unsupported
geometry remains visible through ordinary hit shading when the DDGI traversal supports it, but it is not sampled as an
emitter. Emissive sampling can be enabled globally without disabling direct-hit emission.

Mesh and skinned-mesh inspectors expose emissive radiance, estimated emitting area and power, eligibility, and an
explicit action that scales radiance to a target power. The renderer never creates an analytic-light proxy or changes
emissive radiance automatically when geometry changes.

## Diagnostics

The Environmental Lighting inspector owns persistent settings. The Render Layer inspector exposes
runtime state, including:

- cascade readiness, memory, history-window completion, relocation warmup, and rejection reasons;
- probe positions, irradiance/state visualization, and one explicitly selected probe;
- selected-probe rays and metadata such as hit distance, backface ratio, relocation, and active state;
- emissive inventory and sampling eligibility summaries.

Debug selection is editor-session state and is never serialized. Debug visualization is restricted to the editor scene
viewport and does not appear in game cameras or ray-camera output.

## Limitations

- DDGI uses ray-tracing-pipeline traversal; there is no inline-ray-query DDGI backend.
- Probe transport is diffuse and does not reproduce the full camera BSDF.
- Strands and Gaussian splats do not participate in DDGI geometry or emissive sampling.
- Blended and transmissive surfaces use straight-through attenuation without reflection or refraction.
- Local reflection probes affect raster specular lighting and are excluded from the DDGI source signature.

Contributor capture commands and acceptance checks live in [Rendering validation](rendering-validation.md).

## Shared GI delivery validation

M16–M18 introduce shared automatic placement, compact DDGI visibility, and the unified GI interface.
The final focused suite passed 125 CPU, source-contract, shader and current-GPU tests, including migration,
active-only padded history budgets, atomic API/runtime rejection, unsupported loaded-data fallback, scene replacement,
quantized irradiance replacement and moved/reactivated probe invalidation.

Installed Sponza checks used 2560×1440 on RTX 5070: SDFGI with RT pipeline, ray query, BLAS and TLAS explicitly off,
and DDGI with RT enabled. Signed scrolling, teleports, shared-layout edits, rejected allocations, spacing 4/8/4,
visibility-filter edits, provider off/on and direct DDGI/SDFGI/DDGI transitions passed with finite float captures.
These are smoke checks, not gallery/Cornell/EcoSysLab appearance or performance acceptance.

The editor was built before runtime checks. All enabled applications (editor and launcher), packages and Python were
installed using `python Scripts/install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --jobs 8`.
Editor: `out/install/vs2026-x64/bin/EvoEngineEditor.exe`; the installed Python smoke uses
`out/install/vs2026-x64/python/PyEvoEngine.cp315-win_amd64.pyd`.
