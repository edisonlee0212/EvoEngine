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
the strict history budget determine whether a layout is supported.

DDGI atlas packing is fixed at 16 probe columns, with 8x8 interior texels for both irradiance and visibility.
These are no longer GUI, serialization or Python settings; legacy storage values are ignored. Layouts that exceed
the device image limit are rejected rather than repacked. Probe counts and history length remain configurable.

GPU resources use ordinary, non-exportable VMA 3.4.0 allocations. Vulkan external-memory/semaphore interoperability
is not required or enabled. No dedicated DDGI allocation or allocation override is needed.


## Settings and runtime edits

The GI tab contains shared **Probe settings**, **Indirect GI provider**, and selected-provider controls.
Shared odd probe counts, cascade count, base distance, Y Scale and optional anchor determine nominal coverage.
DDGI-specific controls include tracing, shared irradiance/visibility history, biases, relocation and classification.
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

Each active cascade owns persistent probe state, irradiance/visibility atlases, and quantized history for both. The render graph
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


## Rolling Visibility

Visibility uses the same periodic history window as irradiance: two 16-bit moment samples and two 32-bit running sums
per interior texel. Samples normalize half-moments by half the cascade distance bound and half its square. New histories are zero-filled and
always divided by the full window; rejected/nonfinite samples preserve valid slots. Origin changes, reactivation and
scroll exposure reset affected histories before reuse. The averaged output remains RG16F with regenerated borders.
Visibility lookup uses hardware bilinear filtering. Initial and exposed tiles clear to (1, 0); moved-probe tiles clear
to (0, 0). Atlas output is clamped to RG16F's finite range after averaging; no negative atlas sentinel is used.
The former visibility-smoothing setting is ignored on load and removed from active APIs.


Irradiance and visibility interior tiles are fixed at 8x8. The default 33x17x33 probes, four cascades and
30-update history use about 1.696 GiB for history, including both rings/sums and origin records.
Device padding, output atlases and ray buffers remain additional allocations. Runtime edits recreate incompatible
layouts and reject configurations whose padded histories reach 4 GiB, without silently reducing quality.


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

For targeted shader comparisons, `EVOENGINE_DDGI_PROBE_UPDATE_VARIANT` accepts `serial`, `parallel-direct`, or
`parallel-shared` (default). Device limits and pipeline availability still select a safe fallback. This hook changes
the update shader variant, not atlas allocation or lighting settings.

## Limitations

- DDGI uses ray-tracing-pipeline traversal; there is no inline-ray-query DDGI backend.
- Probe transport is diffuse and does not reproduce the full camera BSDF.
- Strands and Gaussian splats do not participate in DDGI geometry or emissive sampling.
- Blended and transmissive surfaces use straight-through attenuation without reflection or refraction.
- Local reflection probes affect raster specular lighting and are excluded from the DDGI source signature.

Contributor capture commands and acceptance checks live in [Rendering validation](rendering-validation.md).

## Validation and known issues

Focused tests cover fixed atlas addressing and device limits, legacy-setting migration, history budgets,
periodic sampling, rolling replacement, rejected estimates, scrolling and moved/reactivated probe invalidation.
Installed smoke coverage uses RT-enabled Sponza at 2560x1440 with editor layers; SDFGI is checked separately
with RT pipeline, ray query, BLAS and TLAS disabled.

The fixed-layout delivery passed 174 focused tests. Two fresh installed Sponza runs retained visible DDGI at
240 and 600 display-loop iterations. These are not completed-update counts: the readiness counter saturates
at the selected history length. That delivery still reported `VUID-vkCmdDraw-None-09600` depth/color image-layout
errors. The subsequent GENERAL-layout policy and allocation-time atlas initialization removed these errors in a
fresh installed RT-enabled 600-loop Sponza validation run; see [image layout policy](rendering.md#image-layouts-and-synchronization).
Gallery/Cornell/EcoSysLab appearance acceptance remains manual.

The post-investigation cleanup passed 186 focused tests and the repository format check, and retained visible DDGI
through 600 iterations in a fresh installed 2560x1440 Sponza smoke. Obsolete atlas repacking and unconditional path
logging were removed without changing shader selection, allocations or lighting policy.

Removing automatic external-memory export restored lighting in diagnostic comparisons; changing atlas columns
or upgrading VMA alone did not. The precise underlying allocation/driver failure mechanism is not established.
