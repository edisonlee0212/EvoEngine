# Reflection probes

[Back to rendering](rendering.md)

EvoEngine has two reflection-probe data types with separate ownership:

- `GlobalReflectionProbe` is a persistent `.evereflectionprobe` asset containing one prefiltered cubemap. A scene's
  `Scene::global_reflection_probe_fallback` supplies the global image-based specular fallback.
- `EnvironmentalLighting::LocalReflectionProbe` entries define local box or sphere influence, artist priority, blend
  distance, local intensity, optional box-projection bounds, and a reference to a `GlobalReflectionProbe` payload asset.

The final fallback after the scene reference is the engine default `GlobalReflectionProbe` resource. It is not stored in an
`EnvironmentalMap`; environmental maps provide diffuse irradiance, unfiltered environment cubemaps, and ray-environment
sampling data only.

The environmental-lighting resolver orders enabled asset-owned probes by descending priority, smaller transformed
influence volume, then stable ID, and caps the resolved list to 32 entries. Asset-owned local probes are the runtime
source; there is no placed private component, migration extractor, or legacy component fallback.

## Authoring and storage

Create a persistent `GlobalReflectionProbe` asset before assigning or baking a local probe entry. Its inspector accepts
an existing `Cubemap` or an equirectangular `Texture2D`; both inputs are converted and GGX-prefiltered into the same
canonical asset. The editor does not currently import six independent face files directly. DDS, HDR, or ordinary image
formats may be used only through the existing texture/cubemap import path; their source encoding does not become the
probe's storage encoding.

Every accepted asset is linear HDR in Vulkan face order, at 256x256 with nine mips and
`VK_FORMAT_R16G16B16A16_SFLOAT`. The six faces and all mips contain exactly 524,286 texels, or 4,194,288 serialized bytes.
Persisted and imported CPU payloads use packed half data; there is no steady-state FP32 mirror. A newly baked unsaved
asset may remain GPU-only until save. An `Empty` asset keeps its canonical
black placeholder for safe serialization and descriptors, but it is not a valid local-lighting source and therefore falls
through to global IBL. Loading rejects a wrong schema, format,
layout, resolution, mip count, byte count, payload hash, NaN, infinity, negative RGB radiance, or value that overflowed
FP16. Rejected imports leave the previous runtime asset untouched. Saves download the current GPU cubemap and publish
through a temporary file replacement; an editor bake exposes its completed GPU cubemap without persisting or reloading
it. Empty
maps, metadata-only wrappers, and old noncanonical documents are rejected instead of migrated.

The scene inspector assigns `Scene::global_reflection_probe_fallback` and an optional `EnvironmentalLighting` asset. The
`EnvironmentalLighting` asset inspector owns local-probe entries, per-probe debug bounds, one-entry bake buttons, and the
batch **Bake All Local Probe Payloads** action. The RenderLayer inspector's all-probe bounds toggle draws asset-owned local
probe bounds from the assigned `EnvironmentalLighting` asset. Each asset-owned local probe reports its payload readiness;
its bake action queues the same global reflection-probe capture path used by asset entries at the authored transform
position.

Local-probe transforms are authored as Position, Euler Rotation in degrees, and Scale. The editor composes those fields
as translation, rotation, then scale while the asset continues to serialize the resulting matrix. Opening an entry
normalizes perspective or shear to that TRS representation; a non-finite, singular, or undecomposable matrix is reset to
identity and saved as an asset modification. For a box probe, Scale is its full influence-box size; there is no separate
box-extent field. Optional projection half-extents remain separate because they control parallax lookup inside probe-local
space. **Debug draw bounds** and Render Layer's **Display all reflection probe bounds** both draw the resulting full box or
sphere as a translucent, depth-tested filled volume with no depth writes, so scene geometry can occlude the overlay without
the overlay altering scene depth.

An entry assigned through the active scene's `EnvironmentalLighting` asset can enable **Edit in Scene**. This temporarily
replaces entity selection with the Scene-camera ImGuizmo and reuses the editor toolbar's local Position, Rotation, and
Scale modes. Clicking a Position, Rotation, or Scale label in the probe's Transform controls selects that same global
mode. The active influence volume is always drawn once in cyan, including for a disabled entry, while the stored
matrix remains the only serialized transform. Selecting an entity or empty viewport space, pressing Escape, removing or
reloading the entry, replacing the scene asset, or closing its asset inspector clears the transient target. Viewport
picking is not added; activation is explicit in the asset inspector.

RGBA8/RGBM are not bake or persistence formats because they cannot preserve the required HDR range. BC6H is reserved for
a future cooked-asset path. A derived `VK_FORMAT_B10G11R11_UFLOAT_PACK32` image would occupy 2,097,144 bytes, but it is not
used at runtime yet. The editor reports exact cube-image usage support and the worst packed reconstruction error across
bright glossy content, roughness extremes, dark saturated gradients, and a bake containing a small intense emitter.
Adoption additionally requires normalized RMS error at most 0.002, relative peak error at most 0.02, at least 49%
image-memory saving, and at least a 5% GPU-time improvement on the authoritative GPU. No qualifying timing result has been
measured, so RGBA16F is the mandatory runtime representation.

## Selection and lighting

At most 32 enabled asset-owned local probes may resolve for a scene. **Enable local probe reflections** can remove all local
probes from runtime lighting while leaving their individual enable flags, editing, bounds, and bake actions available. If an asset authors more, the resolver keeps the
deterministic highest-ranked 32 and reports the truncation count. Candidates are ordered by descending artist priority,
then smaller transformed influence volume, then stable ID. Influence and projection dimensions are local to the authored
entry transform. This allows nonuniformly scaled boxes and turns a nonuniformly scaled sphere into an ellipsoid;
degenerate transforms do not participate. Blend distance is also expressed in those local units.
New probes default to a `0.05` local-space blend distance, keeping fallback interpolation to a narrow boundary band;
explicitly authored values remain unchanged.

The shaded world position selects the first containing probe. Away from its boundary that primary owns the local result.
Inside its blend band, at most one containing probe with strictly lower artist priority may contribute; all remaining weight
returns to the environmental fallback. Missing, unloaded, disabled, or invalid local assets use the exact same fallback,
so they cannot cause undefined sampling or black output. Sphere probes use the reflected world direction.
Box probes optionally intersect that direction with their local projection bounds and fall back to the unprojected direction
when the projection is invalid or outside the box.

Global and local radiance use the existing roughness-driven prefiltered mip chain, BRDF LUT, Fresnel, and split-sum
response. Valid local probe samples use the baked payload and per-probe local intensity; they are not multiplied by
`environment_lighting_intensity` at surface shading time. Remaining or missing local-probe weight returns to the
specular fallback. Valid DDGI coverage and confidence replace the scene-global or engine-global prefiltered fallback with
`DDGI_irradiance / pi` at every material roughness. This reuses the normal-directed diffuse gather and adds no atlas
samples; it is deliberately a broad lighting proxy rather than a sharp or view-directional reflection. The global
contribution is controlled by `specular_fallback_intensity` directly, while the DDGI proxy is not.
After local/fallback probe selection, the probe-specular term
is multiplied by a scalar visibility confidence composed from material AO, eligible GTAO visibility, and DDGI probe
visibility. The DDGI value comes from the probe visibility atlas/Chebyshev test, not irradiance RGB; missing, uncovered,
disabled, or invalid DDGI blends toward white visibility so lack of DDGI coverage does not darken probes. Local probes
never enter diffuse irradiance or DDGI. SSR remains an optional post-process, and this system adds neither SSR to lighting
correctness nor ray-traced reflections.

Rough indirect specular uses one scalar visibility term after local/global probe selection and split-sum evaluation:

```text
ddgi_probe_visibility = mix(1, DDGI_visibility, DDGI_coverage * DDGI_confidence)
o = 1 - min(material_AO, eligible_GTAO, ddgi_probe_visibility)
o_grazing = 0.04 * tanh(o / 0.04)
o_trusted = mix(o_grazing, o, smoothstep(0.8, 1, NdotV))
Vspec = 1 - roughness^2 * o_trusted
probe_specular = unoccluded_probe_specular * Vspec
```

The minimum combines estimates of cavity visibility without double-darkening them as a product would. GTAO is eligible
only when the camera has an enabled GTAO pass; disabled AO, missing AO resources, and SSAO all provide a white specular
fallback. DDGI visibility is eligible only through valid DDGI gather coverage and confidence. Roughness controls
confidence in the low-frequency scalar. Because scalar AO and DDGI visibility have no bent-normal or directional-cone
information, their occlusion deficit is approximately linear for weak grazing occlusion but smoothly caps deep grazing
attenuation at 4%. Raw scalar visibility is restored from `NdotV = 0.8` to normal incidence. This is a measured confidence
approximation, not directional specular visibility. A diagnostic ramp ending at `NdotV = 0.35` over-darkened the frozen
rough-metal rim; a ramp starting there removed measurable occlusion from the rough dielectric's grazing-only response. A
3% cap then missed the same dielectric gate after tone mapping and quantization. All candidates were rejected without
weakening either gate. The term is never RGB, never interpolated by metallic value, and never applied to direct light,
emission, the visible background, or diffuse IBL. DDGI irradiance supplies only the broad fallback proxy described above;
it does not replace valid local probes or become sharper on smooth materials. The renderer exposes
`Diffuse Indirect`, `Unoccluded Probe Specular`, `Specular Visibility`, and `Occluded Probe Specular` diagnostic views for
isolating this composition.

The Rendering/Sponza demo owns a tracked sky source, global probe, and five box-projected asset-owned local probe entries
under `Resources/EvoEngine-DemoProjects/Rendering/Assets/Lighting/Sponza`. The local volumes cover the left gallery, right
gallery, and three central-hall segments. These payloads are normal persistent demo inputs, not generated validation
output or an engine-global outdoor fallback.

## Explicit bake policy

An asset-owned local probe's **Bake Local Probe Payload** action performs one explicit six-face raster capture at the
entry transform position. It publishes the prefiltered cubemap to the assigned `GlobalReflectionProbe` in GPU memory and
marks that asset unsaved, but it does not write or reload the asset file. The newly baked result is used immediately by
rendering. The previous on-disk payload remains unchanged until the user explicitly saves the probe asset; saving performs
the canonical GPU readback, validation, content hash, and atomic file replacement. Reloading or discarding the asset
before saving restores the previous persisted payload. It does not update automatically. The fixed contract is 256x256 per face, 90-degree projection,
near plane 0.1, far plane 1000, linear HDR with no tone mapping, and canonical Vulkan face orientation. The shared
camera-style **Background** controls above **Add Local Reflection Probe** select Clear Color, Cubemap, Environmental Map,
Inherit Environmental Lighting, or Engine Default Skybox plus an independent intensity. `diffuse_fallback_intensity` and
`specular_fallback_intensity` are forced to zero during capture and are not bake inputs. Debug visualization is forced off
without reducing the authored directional-shadow PCF sample count.
The inspector reports payload readiness, imported content, an unsaved GPU-resident bake, a shared-asset overwrite, or an actionable bake/load error.
**Bake Local Probe Payload** refreshes one entry. The editor does not run a stale scan or batch stale rebake from probe
inspection. Retryable capture preparation is bounded; paused, unconverged DDGI fails the requested bake with an actionable
status instead of holding the queue indefinitely. Bakes never run automatically; imported probe assets remain valid when
scene lighting changes. Explicit baking is never an implicit save operation.

The captured base cubemap generates a conventional source mip hierarchy before specular filtering. The prefiltered
reflection cubemap copies its perfectly smooth base level directly, then evaluates GGX convolution for progressively
rougher levels with decreasing deterministic sample budgets. Filtering samples the source hierarchy at the GGX-derived
LOD; it does not treat ordinary box-filtered mips as roughness-prefiltered reflections.

`RenderLayer` owns a reusable 256x256 raster target and a lightweight pool of six face-camera records per probe in the
current request. **Bake All Local Probe Payloads** is one renderer batch. Its face cameras are injected into the next
normal immutable frame snapshot; after ordinary camera/shadow work, one cached two-pass capture graph records every face,
copy, source-mip generation, and GGX filter into that frame's main command buffer. All faces reuse one graph plan, resource
registry, and transient binding set. There is no bake-private immediate submission, global frame drain, or CPU fence wait.
The new output cubemaps remain private until the submitted frame slot is recycled and its fence has completed, then the
whole batch is published to the target assets. Reflection capture omits motion vectors, motion coverage, the depth
pyramid, ambient occlusion, and post-processing. Raw capture, GGX scratch resources, and the shared GGX pipeline persist
across probe bakes. Per-bake camera state and authored background are refreshed without 1x1 construction or resize.

Reflection captures never render their own shadow maps. They reuse the current frame's point and spot atlases plus the
completed directional atlas, camera matrices, split depths, and cascade policy from the preferred raster camera: the main
camera when it renders raster lighting, otherwise the editor Scene camera. This includes the ray-traced-main-camera case,
because its fallback Scene camera supplies the raster shadow atlas. Capture recording follows the preferred raster camera,
so the reused directional data is already present earlier in the same command buffer.

Explicit bakes do not calculate or store a source-scene fingerprint. The serialized probe retains only its source kind,
canonical pixels, and payload hash. CPU timing records preparation and command recording without counting frame-fence
latency as work; wall timing includes deferred frame-slot completion. GPU timing separates total batch work, face capture,
and GGX prefiltering.

## Dynamic local-probe updates

An `EnvironmentalLighting` asset enables scene-wide dynamic local-probe updates by default with a six-face **Faces per
frame** budget, adjustable from one to six. While enabled, it continuously updates every enabled, contributing local
probe regardless of scene changes; there is no scene-invalidation or manual mode. The scheduler completes a probe before
advancing in artist-priority/stable-ID order, spills unused face budget into the next probe, and starts another deterministic
sweep whenever the current sweep finishes. A position change restarts a partial six-face capture so one generation never
mixes capture origins. Rotation and scale continue to affect influence/projection metadata without restarting capture.

Dynamic and explicit persistent bakes are mutually exclusive. Explicit bake controls and queue APIs reject work while
dynamics are active, and dynamic enablement waits until every queued/prepared/submitted explicit bake publishes. Both
paths share the same capture cameras, stripped render graph, bindings, raw cubemap, GGX scratch resources, and prefilter
pipeline. Dynamic updates never read back, serialize, reload, mark unsaved, or write a probe asset.

One shared raw six-face cubemap accumulates the current probe. Each dynamic probe owns filtered RGBA16F A and B storage.
The first update writes B, then updates alternate B/A/B/A. When B first becomes ready, the assigned static payload is the
transition source; a missing payload uses the existing per-camera global specular IBL fallback. Each later frame linearly
blends the previous and newest prefiltered HDR samples over one complete scheduled update cycle. The blend reaches one
before the next filtered write can replace its source, so a third cubemap is unnecessary.

All six faces, mips, and transition targets publish only after the normal frame-slot fence signals. During fence latency the
previous target remains fully visible. Disabling local-probe contribution restores static/global selection but suspends
dynamic scheduling and retains the last published A/B textures and transition state; re-enabling contribution resumes from
that dynamic result. Disabling dynamics, resetting history, removing a probe, or replacing the scene/lighting asset retires
transient GPU resources after outstanding frame submissions.

The Reflection Probes inspector reports queue/progress counts, current stable ID and face progress, A/B publication counts,
transition-weight range, capture/prefilter GPU time, and transient memory. **Reset Dynamic Probe History** is shown only
while dynamics are enabled and never modifies persistent assets.

The bake includes built-in opaque and alpha-masked geometry, direct lighting and shadows, emission, the selected visible
bake background scaled by `environment_lighting_intensity`, and only converged DDGI. The background has no independent
intensity input. Background selection affects visible miss pixels, not environment illumination on captured surfaces. It
excludes every local reflection
probe, transparent geometry, Gaussian splats, clouds, editor overlays, external render callbacks, SSR, ambient occlusion,
authored diffuse/specular fallback factors, and all post-processing. This prevents recursive feedback without a metallic
diffuse proxy. Higher-order local-specular interreflection is intentionally absent: a baked probe can reflect the global
environment and diffuse DDGI, but not another local probe.

Validation uses the installed editor at 1920x1080. The reflection-probe suite covers adjacent colored regions, nested and
overlapping priorities, priority ties and entity-order independence, blend transitions, rotated box projection, sphere
lookup, camera motion over a static surface, probe removal, missing assets, global fallback, roughness extremes, exact
metals, and off/on captures of filled bounds for transformed box and sphere probes through the asset-inspector path. The
scalar-visibility extension isolates material AO, GTAO, DDGI visibility, their bounded minimum,
disabled/unavailable/SSAO and missing-DDGI fallbacks, rough and smooth response, grazing retention, probe-boundary
continuity, indirect-intensity invariance, and zero descriptor/ABI growth. It also records logical RGBA16F GPU image
bytes, steady CPU payload bytes, and serialized payload bytes separately.
The unelected packed candidate records its logical GPU bytes with zero retained CPU or serialized bytes, alongside packed
error metrics and the explicit packed-format no-adopt result.
