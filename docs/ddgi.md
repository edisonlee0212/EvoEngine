# Dynamic Diffuse Global Illumination

[Back to rendering overview](rendering.md)

Dynamic Diffuse Global Illumination (DDGI) stores scene irradiance in a grid of probes and samples that grid during
raster lighting. It provides diffuse indirect light and visibility; it is not a source of sharp specular reflection.

EvoEngine's DDGI probe tracing uses the Vulkan ray-tracing pipeline. A device without ray-tracing support can still run
the raster renderer, but it cannot update DDGI probes.

## Authoring And Ownership

`EnvironmentalLighting` owns shared DDGI settings and references a `DdgiVolumePack`. The pack is a YAML asset containing
volume definitions and stable IDs. GPU buffers, atlases, probe history, relocation, classification, convergence, and
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

Each active volume owns persistent probe state plus irradiance, visibility, and variability atlases. The render graph
records the following work when required:

1. Prepare or clear persistent state for a new layout, reset, or newly exposed scrolling region.
2. Trace probe rays through the scene acceleration structure.
3. Filter ray results into octahedral irradiance and visibility atlas tiles.
4. Update relocation, classification, variability, and convergence state.
5. Sample ready atlases during deferred lighting and recursive DDGI hit shading.

Probe directions use a spherical Fibonacci distribution. Fixed relocation/classification rays are deterministic; normal
lighting rays receive a frame-wide random rotation. The default runtime settings trace 192 scene rays and 64 exact
emissive-triangle rays per updated probe.

The DDGI acceleration structure includes supported triangle geometry and explicitly registered external DDGI geometry.
Strands and Gaussian splats are not traversed. Alpha-masked triangle hits use deterministic cutoff testing. Blended and
transmissive surfaces use straight-through traversal with colored attenuation for probe, emissive-target, and visibility
rays. Volume attenuation is applied between entry and exit intersections, but DDGI does not refract the ray direction.

## Probe Updates And Convergence

An update covers the complete probe volume. Irradiance history blends new observations using the volume's hysteresis
settings. Cold-start warmup fills empty history quickly; compatible light, geometry, and material changes temporarily
lower hysteresis without destroying otherwise useful probe data.

Variability measurements determine when stable volumes can pause updates. The render layer applies one variability,
gating, pause, backface-threshold, and convergence-budget policy to every volume. Gating requires three consecutive
complete-volume observations and considers average change, the unstable fraction, and severe outliers. Warmup and a
temporary scene-change hysteresis boost always continue through the return to normal hysteresis, even if variability
converges sooner. An unconverged volume then receives at most 128 additional valid full-volume updates before entering
a distinct maximum-reached state. Both convergence and maximum exhaustion are sampling-complete and retain lighting
from the existing atlases.

There is no periodic refresh. Any scene change that activates the hysteresis boost starts a new convergence cycle;
manual reset, incompatible source or layout changes, scrolling clears, emissive-population changes, and variability
policy changes also restart it. These controls are live `RenderSettings` values and are not serialized into DDGI volume
assets.

Hard resets are reserved for incompatible layouts, source changes, manual reset, or scrolling movement that spans an
entire probe-grid dimension. A compatible scrolling volume ring-maps its history and clears only newly exposed probe
slabs.

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

- volume readiness, memory, convergence, warmup, and rejection reasons;
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
