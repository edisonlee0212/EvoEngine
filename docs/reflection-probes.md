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
The persistent CPU copy is packed half data; there is no steady-state FP32 mirror. An `Empty` asset keeps its canonical
black placeholder for safe serialization and descriptors, but it is not a valid local-lighting source and therefore falls
through to global IBL. Loading rejects a wrong schema, format,
layout, resolution, mip count, byte count, payload hash, NaN, infinity, negative RGB radiance, or value that overflowed
FP16. Rejected imports leave the previous runtime asset untouched. Saves publish through a temporary file replacement,
and an editor bake reloads the successfully persisted document before exposing it as the active runtime payload. Empty
maps, metadata-only wrappers, and old noncanonical documents are rejected instead of migrated.

The scene inspector assigns `Scene::global_reflection_probe_fallback` and an optional `EnvironmentalLighting` asset. The
`EnvironmentalLighting` asset inspector owns local-probe entries and their bake buttons. The RenderLayer inspector's
all-probe bounds toggle draws asset-owned local probe bounds from the assigned `EnvironmentalLighting` asset. Each
asset-owned local probe reports its payload readiness; its bake action queues the same global
reflection-probe capture path used by asset entries at the authored transform position.

RGBA8/RGBM are not bake or persistence formats because they cannot preserve the required HDR range. BC6H is reserved for
a future cooked-asset path. A derived `VK_FORMAT_B10G11R11_UFLOAT_PACK32` image would occupy 2,097,144 bytes, but it is not
used at runtime yet. The editor reports exact cube-image usage support and the worst packed reconstruction error across
bright glossy content, roughness extremes, dark saturated gradients, and a bake containing a small intense emitter.
Adoption additionally requires normalized RMS error at most 0.002, relative peak error at most 0.02, at least 49%
image-memory saving, and at least a 5% GPU-time improvement on the authoritative GPU. No qualifying timing result has been
measured, so RGBA16F is the mandatory runtime representation.

## Selection and lighting

At most 32 enabled asset-owned local probes may resolve for a scene. If an asset authors more, the resolver keeps the
deterministic highest-ranked 32 and reports the truncation count. Candidates are ordered by descending artist priority,
then smaller transformed influence volume, then stable ID. Influence and projection dimensions are local to the authored
entry transform. This allows nonuniformly scaled boxes and turns a nonuniformly scaled sphere into an ellipsoid;
degenerate transforms do not participate. Blend distance is also expressed in those local units.

The shaded world position selects the first containing probe. Away from its boundary that primary owns the local result.
Inside its blend band, at most one containing probe with strictly lower artist priority may contribute; all remaining weight
returns to the global probe. Missing, unloaded, disabled, or invalid local assets also return their weight to the exact same
global descriptor, so they cannot cause undefined sampling or black output. Sphere probes use the reflected world direction.
Box probes optionally intersect that direction with their local projection bounds and fall back to the unprojected direction
when the projection is invalid or outside the box.

Global and local radiance use the existing roughness-driven prefiltered mip chain, BRDF LUT, Fresnel, and split-sum
response. Valid local probe samples use the baked payload and per-probe local intensity; they are not multiplied by
`environment_lighting_intensity` at surface shading time. Remaining or missing local-probe weight returns to the
scene-global or engine-global prefiltered fallback, whose contribution is controlled by
`environment_lighting_intensity * specular_fallback_intensity`. `Indirect Lighting Intensity` never scales reflection-probe
specular. Local probes never enter diffuse irradiance or DDGI. SSR remains an optional post-process, and this system adds
neither SSR to lighting correctness nor ray-traced reflections.

Rough indirect specular uses one scalar visibility term after local/global probe selection and split-sum evaluation:

```text
o = 1 - min(material_AO, GTAO)
o_grazing = 0.04 * tanh(o / 0.04)
o_trusted = mix(o_grazing, o, smoothstep(0.8, 1, NdotV))
Vspec = 1 - roughness^2 * o_trusted
probe_specular = unoccluded_probe_specular * Vspec
```

The minimum combines two estimates of the same cavity visibility without double-darkening them as a product would. GTAO
is eligible only when the camera has an enabled GTAO pass; disabled AO, missing AO resources, and SSAO all provide a white
specular fallback. Roughness controls confidence in the low-frequency scalar. Because scalar GTAO has no bent-normal or
directional-cone information, its occlusion deficit is approximately linear for weak grazing occlusion but smoothly caps
deep grazing attenuation at 4%. Raw scalar visibility is restored from `NdotV = 0.8` to normal incidence. This is a measured
confidence approximation, not directional specular visibility. A diagnostic ramp ending at `NdotV = 0.35` over-darkened
the frozen rough-metal rim; a ramp starting there removed measurable occlusion from the rough dielectric's grazing-only
response. A 3% cap then missed the same dielectric gate after tone mapping and quantization. All candidates were rejected
without weakening either gate. The term is never RGB, never interpolated by metallic value, and never applied to direct
light, emission, the visible background, diffuse IBL, or DDGI. DDGI irradiance
was rejected as a specular source because it contains neither the directional nor frequency information needed for a
reflection. The renderer exposes `Diffuse Indirect`, `Unoccluded Probe Specular`, `Specular Visibility`, and `Occluded
Probe Specular` diagnostic views for isolating this composition.

The Rendering/Sponza demo owns a tracked sky source, global probe, and five box-projected asset-owned local probe entries
under `Resources/EvoEngine-DemoProjects/Rendering/Assets/Lighting/Sponza`. The local volumes cover the left gallery, right
gallery, and three central-hall segments. These payloads are normal persistent demo inputs, not generated validation
output or an engine-global outdoor fallback.

## Explicit bake policy

An asset-owned local probe's **Bake Local Probe Payload** action performs one explicit six-face raster capture at the
entry transform position. It does not update automatically. The fixed contract is 256x256 per face, 90-degree projection,
near plane 0.1, far plane 1000, linear HDR with no tone mapping, and canonical Vulkan face orientation. A deterministic version-2 content fingerprint
covers the capture position, environment source and `environment_lighting_intensity`, built-in
geometry/material/texture content, direct lights, shadow-map and strand-tessellation settings, application shadow
resolutions and light limits, and DDGI configuration. `diffuse_fallback_intensity` and `specular_fallback_intensity` are
not bake inputs. Debug visualization is forced off without reducing the authored directional-shadow PCF sample count.
The inspector reports payload readiness, imported content, a shared-asset overwrite, or an actionable bake/load error.
**Bake Local Probe Payload** refreshes one entry. The editor does not run a stale scan or batch stale rebake from probe
inspection. Retryable capture preparation is bounded; paused, unconverged DDGI fails the requested bake with an actionable
status instead of holding the queue indefinitely. Bakes never run automatically; imported probe assets remain valid when
scene lighting changes.

Version 2 hashes structural YAML with stable map ordering, ignores volatile asset/entity handles, and resolves referenced
asset content. The stored fingerprint is provenance for explicit bakes and future tooling; probe inspection does not
compare it against the live scene.

The bake includes built-in opaque and alpha-masked geometry, direct lighting and shadows, emission, global environment
input scaled by `environment_lighting_intensity`, and only converged DDGI. It excludes every local reflection probe,
transparent geometry, Gaussian splats, clouds, editor overlays, external render callbacks, SSR, ambient occlusion,
diffuse/specular fallback factors, and all post-processing. This prevents recursive feedback without a metallic diffuse
proxy. Higher-order local-specular interreflection is intentionally absent: a baked probe can reflect the global
environment and diffuse DDGI, but not another local probe.

Validation uses the installed editor at 1920x1080. The reflection-probe gate covers adjacent colored regions, nested and
overlapping priorities, priority ties and entity-order independence, blend transitions, rotated box projection, sphere
lookup, camera motion over a static surface, probe removal, missing assets, global fallback, roughness extremes, and exact
metals. The M15 extension isolates material AO, GTAO, their bounded minimum, disabled/unavailable/SSAO fallback, rough and
smooth response, grazing retention, probe-boundary continuity, indirect-intensity invariance, and zero descriptor/ABI
growth. It also records logical RGBA16F GPU image bytes, steady CPU payload bytes, and serialized payload bytes separately.
The unelected packed candidate records its logical GPU bytes with zero retained CPU or serialized bytes, alongside packed
error metrics and the explicit packed-format no-adopt result.
