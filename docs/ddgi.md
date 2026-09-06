# Dynamic Diffuse Global Illumination

[Back to rendering overview](rendering.md)

Dynamic Diffuse Global Illumination (DDGI) stores scene irradiance in a grid of probes and samples that grid during
raster lighting. It provides diffuse indirect light and visibility; it is not a source of sharp specular reflection.

EvoEngine's DDGI probe tracing uses the Vulkan ray-tracing pipeline. A device without ray-tracing support can still run
the raster renderer, but it cannot update DDGI probes.

## Authoring And Ownership

### EcoSysLab comparison volume

`Resources/EcoSysLabProject/Assets/Default.evescene` embeds one **EcoSysLab Walls** volume for provider comparisons.
It stays fixed at world center `(0, 2, 0)`, with `15 x 13 x 15` probes at `0.5` spacing (2,925 probes).
Probe-grid bounds are `(-3.5, -1, -3.5)` to `(3.5, 5, 3.5)`, covering the room walls with padding rather than the enormous
ground plane. Relocation is on (distance `0.25`); classification is off. Other DDGI runtime settings retain engine defaults.
The scene still selects Automatic SDFGI. In **Environmental Lighting**, select **Authored DDGI (RT)** to compare, then
switch back to **Automatic SDFGI**; the inactive volume remains stored. DDGI requires RT-enabled engine startup.
Entity transforms, enabled/Static flags, materials, lights, and the currently disabled Wall0 are unchanged.

### Asset ownership

`EnvironmentalLighting` owns shared DDGI settings and references a `DdgiVolumePack`. The pack is a YAML asset containing
volume definitions and stable IDs. GPU buffers, atlases, probe history, relocation, classification, and
diagnostic readbacks are transient `RenderLayer` state.

Create or assign a DDGI volume pack from the Environmental Lighting inspector, add one or more volumes, enable DDGI, and
position the probe grids around the receivers they should light. Volume transforms use position, Euler rotation, and
scale. The authored `volume_origin` is the center of the lattice.

The first untransformed probe position is:

```text
volume_origin - 0.5 * (probe_counts - 1) * probe_spacing
```

The renderer accepts at most eight enabled volumes and 8,192 resident probes in total. Paused volumes keep their
resources and count toward both limits. An invalid edit does not silently discard another volume: an existing runtime
volume retains its last valid state, while a new invalid volume remains disabled.

## Volume Selection

Enabled volumes are ordered by artist priority, probe density, then stable ID. A shaded point selects the first ready
volume with positive coverage. Near the primary volume's boundary, one additional lower-ranked volume may blend with it.
Any uncovered diffuse weight uses the environment diffuse fallback.

A volume contributes only after its layout and probe history are ready. A higher-ranked volume that is warming or
rebuilding does not hide a lower-ranked ready volume.

## Runtime Flow

Each active volume owns persistent probe state, irradiance/visibility atlases, and quantized rolling histories. The render graph
records the following work when required:

1. Prepare or clear persistent state for a new layout, reset, or newly exposed scrolling region.
2. Trace probe rays through the scene acceleration structure.
3. Filter ray results into octahedral irradiance and visibility atlas tiles.
4. Update relocation and classification, then invalidate moved/reactivated probes before publication.
5. Sample ready atlases during deferred lighting and recursive DDGI hit shading.

Probe directions use a spherical Fibonacci distribution. Fixed relocation/classification rays are deterministic; normal
lighting rays receive a periodic per-volume rotation. The default runtime settings trace 192 scene rays and 64 exact
emissive-triangle rays per updated probe.

The DDGI acceleration structure includes supported triangle geometry and explicitly registered external DDGI geometry.
Strands and Gaussian splats are not traversed. Alpha-masked triangle hits use deterministic cutoff testing. Blended and
transmissive surfaces use straight-through traversal with colored attenuation for probe, emissive-target, and visibility
rays. Volume attenuation is applied between entry and exit intersections, but DDGI does not refract the ray direction.

## Rolling Probe History

**RenderLayer → DDGI → Blending → History count** selects 5–30 probe updates in steps of 5 (default 30).
The value belongs to the scene's EnvironmentalLighting DDGI settings, is serialized as `runtime.history_count`,
and is available through Python's `GetCurrentSceneDdgiHistoryCount` and `SetCurrentSceneDdgiHistoryCount`.
Missing keys use 30; obsolete hysteresis, temporal-response, convergence, and per-volume trigger keys are ignored.

Each active volume stores interior irradiance and visibility texels in a ring. Irradiance samples are linear and bounded
to 64 before 16-bit quantization. Visibility moments are normalized by their per-volume distance bounds. Integer
32-bit running sums replace the outgoing quantized sample exactly; averaged irradiance is gamma-encoded only when
writing the lighting atlas. Borders are rebuilt from those averaged interiors. Invalid estimates leave the old slot intact.

New/reset histories start at zero and always divide by the full selected window. Both uniform rotations and emissive
sampling repeat over the same window using stable volume/probe identities and optional custom seeds. The phase advances
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

## Relocation And Classification

Relocation moves probes away from nearby or enclosing geometry while keeping the offset within the probe voxel. It runs
during initialization, reset, scrolling exposure, and relevant geometry changes rather than oscillating every steady
lighting frame.

Classification marks probes inside geometry inactive. It can reduce leaking and wasted work, but thin or double-sided
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
emitter. Emissive sampling can be enabled globally and overridden per volume without disabling direct-hit emission.

Mesh and skinned-mesh inspectors expose emissive radiance, estimated emitting area and power, eligibility, and an
explicit action that scales radiance to a target power. The renderer never creates an analytic-light proxy or changes
emissive radiance automatically when geometry changes.

## Diagnostics

The Environmental Lighting inspector owns persistent settings and volume authoring. The Render Layer inspector exposes
runtime state, including:

- volume readiness, memory, history-window completion, relocation warmup, and rejection reasons;
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
