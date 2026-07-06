# EvoEngine DDGI

[Back to rendering overview](rendering.md)

This page covers EvoEngine's Dynamic Diffuse Global Illumination path. It is intentionally scoped to DDGI; general camera
flow is documented in [rendering.md](rendering.md), and capture/validation commands are in
[rendering-validation.md](rendering-validation.md).

## Ownership

Scene-authored DDGI probe volumes use the `DdgiVolume` private component. Scene environment data owns the runtime
settings, probe-volume selection, debug flags, atlas resources, and large-world clipmap settings. `RenderLayer` executes
those settings and exposes runtime/debug state in the editor.

The authored volume origin is the center of the probe lattice. The first probe position is:

```text
origin - 0.5 * (probe_counts - 1) * probe_spacing
```

Enabled `DdgiVolume` components are sorted by priority. EvoEngine currently traces and samples one selected active volume;
overlapping multi-volume blending and per-volume resource isolation remain future work.

## Runtime Passes

When DDGI runtime or debug visualization is enabled, the frame graph declares:

- frame-local DDGI ray-output buffer;
- persistent probe metadata;
- persistent probe state;
- irradiance atlas;
- visibility atlas.

The DDGI pass chain is:

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

- uses geometry normals and material constant albedo;
- respects material cull mode;
- evaluates enabled directional, point, and spot lights;
- casts hard TLAS shadow rays for shadow-casting direct lights;
- keeps light-debug meshes from shadowing their own point lights by using a shadow-caster TLAS mask;
- samples previous DDGI irradiance/visibility at the hit point for a recursive diffuse bounce.

## Probe Update

The authored update budget is a ray-sample budget. `RenderLayer` divides it by the current rays-per-probe count to choose
how many probes update this frame. Adaptive budgeting can scale the next frame's ray-sample budget from the last update
recording cost.

Steady-state updates use a round-robin window and wrap at the end of the probe list. Light, geometry, material, source,
scroll, and manual-reset changes can schedule explicit refresh windows with condition-specific hysteresis. Large-world
clipmap recentering can scroll probe history by whole probe steps and update only newly exposed edge-plane probes.

The update shader writes:

- irradiance atlas interior texels plus wrapped borders;
- visibility moment atlas interior texels plus wrapped borders;
- probe metadata with irradiance, hit ratio, average hit distance, backface ratio, relocation amount, relocation offset,
  and active state;
- next-frame probe state used by ray generation and surface sampling.

## Relocation And Classification

Relocation and classification use the fixed-ray prefix:

- inside-geometry probes move along the closest backface direction;
- near-surface probes move toward the farthest opposing frontface;
- probes with clearance move back toward zero offset only when the candidate remains inside the probe voxel ellipsoid;
- classification deactivates probes detected inside geometry from fixed-ray backface ratio.

The Rendering demo keeps relocation enabled but classification disabled because curtain backfaces can incorrectly mark
valid near-wall probes inactive.

## Surface Sampling

Deferred lighting binds safe fallback DDGI atlas textures every frame and enables DDGI sampling only when the ray/update
chain produced current atlases. With DDGI disabled, raster lighting continues to use the existing IBL ambient path.

Enabled DDGI sampling uses:

- border-aware octahedral irradiance lookup;
- visibility-moment lookup;
- surface-normal bias;
- trilinear probe blending;
- probe-state active/inactive rejection;
- relocated probe offsets;
- volume blend weighting;
- Chebyshev visibility with a low-weight floor.

Both deferred and recursive probe-hit sampling return zero outside active DDGI volume coverage.

## Debugging

DDGI debug visualization is scene-camera-only. The main camera is kept free of probe overlays.

Available debug surfaces include:

- `DDGIProbeVisualization` for probe spheres sampled from GPU atlas/state data;
- `DDGIProbeRayVisualization` for selected-probe ray lines;
- inspector metadata readback for selected probe irradiance, hit/miss/backface ratios, visibility distance, relocation,
  active state, update age, and update reason;
- atlas preview/readout controls through `Scripts/capture_readme_editor_screenshot.py --ddgi-atlas-preview`.

## Rendering Demo Baseline

The Rendering demo DDGI baseline uses:

- Sponza scene with static environment light disabled;
- imported Sponza punctual lights disabled;
- Capoeira disabled for idle editor accumulation;
- DDGI volume with 10x6x16 probes;
- 1.5 probe spacing;
- local volume origin `(0, 3, 3)`;
- 64 rays per probe;
- 16384 ray samples per frame;
- 0.02 normal/visibility bias;
- relocation enabled;
- classification disabled;
- adaptive ray-sample budgeting enabled;
- top-down white directional light brightness `5.0`;
- yellow point-light/debug sphere marked non-shadow-casting.

`DDGIApp` is the cleaner player-mode Cornell-box comparison binary. It keeps DDGI runtime lighting enabled, clears
skybox/post-processing, disables DDGI debug overlays, and isolates placement/filtering/final-gather behavior.

## RTXGI Port Map

`out/external/RTXGI-DDGI` is treated as an algorithm reference only. EvoEngine keeps shader source in GLSL and does not
add an HLSL, DXC, or RTXGI runtime dependency.

| RTXGI concept | EvoEngine path |
| --- | --- |
| Volume descriptors/resources | `DdgiSettings`, `DdgiVolume`, probe metadata/state, irradiance atlas, visibility atlas |
| Probe ray generation | `Shaders/RayTracing/RayGen/DDGIProbeDiagnostics.rgen` |
| Probe closest-hit/miss | `Shaders/RayTracing/ClosestHit/DDGIProbeDiagnostics.rchit`, `Shaders/RayTracing/Miss/DDGIProbeDiagnostics.rmiss` |
| Shared probe math | `Shaders/Includes/DDGI.glsl` |
| Probe atlas update | `Shaders/Compute/DDGIProbeUpdate.comp` |
| Surface sampling | `Shaders/Includes/Lighting.glsl` and DDGI closest-hit recursive sampling |
| Scheduling and clears | `RenderLayer` DDGI passes: `DDGIAtlasPrepare`, `DDGIRayDiagnostics`, `DDGIProbeUpdate` |

Not yet ported from the RTXGI reference: overlapping-volume isolation, variability-based update prioritization,
probe-variability/reduction/readback, and broader scene-level atlas seam/leak/classification validation scenes.
