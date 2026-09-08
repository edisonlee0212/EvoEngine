# Reflection Probes

[Back to rendering overview](rendering.md)

Reflection probes provide prefiltered image-based specular lighting for raster cameras. EvoEngine separates the global
fallback from placed local probes so scenes can choose a stable environment while refining reflections in selected
regions.

## Assets And Ownership

| Asset | Purpose |
| --- | --- |
| `GlobalReflectionProbe` | One persistent prefiltered cubemap used by `Scene::global_reflection_probe_fallback`. |
| `ReflectionProbePack` | One binary asset containing local probe metadata and baked cubemap payloads. |

`EnvironmentalLighting::reflection_probe_pack` references the local pack. The renderer resolves asset-owned entries
directly; there is no placed reflection-probe component fallback. If the scene-global probe is missing or unavailable,
the engine default global probe keeps raster descriptors valid.

Environmental maps and reflection probes have different roles. An `EnvironmentalMap` supplies diffuse irradiance,
unfiltered environment radiance, and ray-environment sampling data. A `GlobalReflectionProbe` supplies the prefiltered
specular fallback used by raster lighting.

## Authoring Local Probes

Assign a reflection-probe pack through the Environmental Lighting inspector, then add box or sphere entries. Each entry
contains a stable ID, transform, priority, blend distance, intensity, shape, optional box-projection bounds, enable state,
and baked payload.

Transforms are edited as position, Euler rotation, and scale and serialized as a matrix. Box scale is the full influence
size. Nonuniform sphere scale creates an ellipsoidal influence region. Degenerate transforms do not participate in
lighting.

An unbaked entry remains editable and uses the ordinary global fallback until a valid payload is available. Local probe
payloads use 256 by 256 linear HDR cubemap faces with nine prefiltered mips in RGBA16F.

The active scene's Environmental Lighting inspector can edit a probe directly in the scene viewport. This editor-only
mode reuses the normal position, rotation, and scale gizmos and draws the active influence volume. It does not add a
scene component or change serialization ownership.

## Selection And Blending

The resolver keeps at most 32 enabled local probes. Candidates are ordered by descending artist priority, smaller
transformed influence volume, then stable ID.

The shaded world position selects the first containing probe. Away from its boundary, that probe owns the local result.
Inside its blend band, one containing probe with strictly lower priority may contribute. Any remaining weight returns to
the global fallback. Missing, disabled, unloaded, or invalid payloads follow the same fallback path.

Sphere probes sample the reflected world direction. Box probes can apply parallax correction using their local
projection bounds and fall back to an unprojected direction when the projection is invalid.

Valid local samples use their authored intensity and are not multiplied by `environment_lighting_intensity`. Global
fallback weight is multiplied by `specular_fallback_intensity`. Local probes never contribute diffuse irradiance or enter
DDGI probe tracing.

After local/global selection, rough probe specular can use a scalar visibility term derived from material occlusion,
eligible GTAO, and valid DDGI visibility. Missing or uncovered inputs resolve to white visibility so an unavailable
screen-space or DDGI signal does not darken reflections. This term does not affect direct light, emission, visible
backgrounds, or diffuse lighting.

## Explicit Baking

**Bake Local Probe Payload** records one six-face raster capture at the probe position and prefilters it for roughness.
**Bake All Local Probe Payloads** submits the same work for every eligible entry in one renderer batch.

Published results replace the entry's in-memory payload and mark the shared pack unsaved. Baking does not save the asset;
the previous on-disk payload remains intact until the user explicitly saves. Reloading or discarding the asset restores
the persisted payload.

The capture uses the probe's authored bake background. Inherited environment radiance is scaled by
`environment_lighting_intensity`. Raster diffuse fallback remains enabled so surfaces receive the same non-DDGI
environmental diffuse lighting as camera rendering. Specular fallback remains disabled to prevent recursive probe input.

Probe capture includes opaque and alpha-masked geometry, direct lighting, shadows, emission, the visible bake
background, and ready diffuse lighting from the selected DDGI or Automatic SDFGI provider. It excludes local reflection probes, screen-space effects, ambient
occlusion, post-processing, transparent geometry, Gaussian splats, clouds, editor overlays, and external callbacks. This
prevents recursive local-reflection feedback.

Captures reuse normal frame resources and publish only after the owning frame-slot fence completes. A failed readiness
check leaves the previous valid payload available and reports an actionable error.

## Dynamic Updates

Dynamic local-probe updates are controlled by `EnvironmentalLighting` and are enabled by default. The authored
**Faces per frame** budget ranges from one to six and defaults to six. It independently limits capture faces and GGX
filter output faces in a frame. The scheduler completes each capture in priority/stable-ID order and filters every mip
of each selected output face from the completed raw cubemap.

Dynamic and explicit persistent bakes are mutually exclusive. Explicit bake requests are rejected while dynamic updates
are active, and dynamic work waits for already submitted explicit bakes to publish.

Dynamic updates never read back, serialize, save, or mark probe assets dirty. Each probe alternates between two filtered
GPU cubemaps and blends from the previous result over a complete update cycle. Two raw capture cubemaps allow filtering
one completed probe while the next eligible probe captures. A filtered generation is published atomically only after all
six faces have completed and the owning frame-slot fence signals. The last published result remains visible while newer
capture or filtering work is in flight.

Moving a probe restarts an incomplete six-face capture so one generation never mixes capture origins. Disabling dynamic
updates, resetting history, removing a probe, or replacing the scene retires transient resources after submitted frames
finish.

## Interaction With Other Paths

- Raster cameras use local probes and the scene-global prefiltered fallback.
- Ray cameras trace scene geometry and sample the indirect environment source; they do not sample local or global
  prefiltered probe assets as environment radiance.
- Dynamic probe texture bindings and blend weights are excluded from scene-change detection for every GI provider,
  including Automatic SDFGI and Environment, not only DDGI. Current bindings are restored before rendering. Actual
  scene/material/light/camera changes and probe placement/membership changes retain existing invalidation behavior.
- DDGI remains diffuse-only. Its visibility may occlude rough probe lighting, and its irradiance may provide a broad
  fallback where global probe weight is missing.
- Reflection-probe captures include available diffuse GI from the selected provider but always exclude local reflection probes.
- Automatic SDFGI captures reuse the live camera-anchored field, including its occlusion and coverage fades. Missing,
  invalid, or uncovered GI uses normal environment diffuse fallback. Capture cameras never move the GI anchor or run
  extra convergence work. SDFGI specular and sharp SDF tracing are excluded from captures; main-camera reflections remain
  unchanged. Multi-frame captures sample the current publication per submission, not a frozen six-face lighting snapshot.
- SDFGI capture consumption is automatic, without a new setting or asset migration. Dynamic probes refresh normally;
  existing baked payloads require an explicit rebake after the desired region has converged. There is no additional
  SDFGI readiness/convergence wait. This differs from the existing DDGI bake-readiness gate.

Contributor bake, selection, fallback, and dynamic-update checks live in
[Rendering validation](rendering-validation.md).
