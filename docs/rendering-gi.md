# Global Illumination

[Back to rendering overview](rendering.md)

Global illumination supplies indirect light to raster cameras and diffuse illumination to reflection captures.
Choose a provider in **Environmental Lighting > GI > Indirect GI provider**. Ray-tracing and ray-query cameras
compute their own indirect lighting through the path integrator.

## Choosing a provider

| Provider | Scene representation | Lighting | Main tradeoff |
| --- | --- | --- | --- |
| Environment | Environment map; no scene GI field | Diffuse environment and reflection-probe fallback | Lowest GI update cost, but no local diffuse light transport. |
| Automatic DDGI | Rays traced through triangle acceleration structures | Diffuse probe irradiance with directional visibility | Represents supported geometry directly; requires Vulkan ray-tracing pipelines and substantial probe history. |
| Automatic SDFGI | Voxelized geometry and signed distance fields | Diffuse probes and voxel-based specular GI | Works without RT hardware; field updates and voxel resolution affect cost and detail. |
| Automatic HDDAGI | Voxel occupancy hierarchy traversed with HDDA | Diffuse probes and voxel-based specular GI | Skips empty space without building distance fields; still limited by voxel detail and probe reconstruction. |

SDFGI is the default. Only the selected provider updates. Unavailable providers expose an Environment fallback reason.
Faster traversal alone does not guarantee less leaking or lower frame time.

## Shared coverage and lifetime

**Probe settings** control odd horizontal/vertical probe counts, cascade count, base distance, Y Scale, and an optional
anchor camera. Defaults are 33 x 17 x 33 probes, four cascades, a base interval of 0.8, and Y Scale 100%. Each cascade
doubles the interval. The main or explicitly selected anchor moves the field; auxiliary cameras and reflection captures
consume the same field without moving it.

Small camera movements scroll the cascades, retaining overlapping data and clearing newly exposed probes. Large moves,
incompatible settings, and resource replacement reset affected locations. Ordinary scene edits refresh geometry and paths
while preserving lighting at unchanged locations. Coarser cascades extend coverage;
outside valid coverage, lighting blends to the environment. Increasing probe counts or history consumes memory as well
as update time. Device limits and a strict history budget below 4 GiB are checked before allocation.

**Use Occlusion** defaults on for scene GI providers. DDGI allocates voxel visibility only while enabled, disabling
relocation. Existing assets retain their saved settings; legacy assets preserve their previous behavior.

## Update passes

These scene updates are shared across cameras.

| Provider | Update sequence | Camera consumption |
| --- | --- | --- |
| DDGI | Update optional voxel occlusion; prepare/scroll atlases; trace probe rays; update irradiance and visibility histories; relocate/classify when needed; invalidate relocated or newly exposed probes. | Deferred lighting blends ready probe irradiance using directional visibility. |
| SDFGI | Voxelize changed regions; construct distance fields and occlusion; inject direct lighting and emission; trace probes; integrate and store temporal irradiance. | Deferred lighting gathers diffuse probes and specular field lighting. |
| HDDAGI | Voxelize changed regions; update occupancy hierarchy and compact lighting cells; inject lights; integrate cached HDDA probe rays; filter diffuse probes. | Surface preparation, full-resolution GI gather, optional two-pass reflection filtering, then deferred composition. |

Light and geometry changes refresh scene representation and cached paths while retaining lighting history at unchanged
probe locations. Temporal histories smooth noise but delay the response to changes. A cleared field needs time to
populate; visual comparisons should use a stationary camera and completed updates.

## DDGI

DDGI traces supported triangle geometry, including compatible external renderers. Alpha masks use cutoff testing;
blended and transmissive surfaces use straight-through colored attenuation, without refracting probe rays. Strands and
Gaussian splats do not participate. DDGI currently requires the ray-tracing pipeline, even when the camera is rasterized;
there is no inline-ray-query DDGI backend.

Defaults trace 64 scene rays and eight emissive-triangle rays per updated probe. History count selects 5–30 updates
in steps of five, defaulting to 30. Rolling irradiance and visibility histories continue updating after filling.

These counts measure probe updates, not independent camera samples. SDFGI defaults to 30 history updates; HDDAGI
defaults to 12 and permits 6, 12, 18, 24 or 32. Longer windows smooth noise but delay lighting response and increase
history storage. They are separate from PT accumulation, ReSTIR reservoirs and NRD histories.

**Use Occlusion** and **Probe Classification** default on. Classification independently disables probes inside geometry
or far from surfaces. Occlusion uses HDDAGI's voxel occupancy and visibility field with fixed probes; fully blocked
contributions remain zero. It needs no signed distance field or HDDAGI lighting/transport histories.

Occlusion disables relocation without clearing its saved preference. Turning occlusion off releases the voxel field,
restores that preference, and uses directional distance moments for visibility. Relocation moves probes away from
geometry within a bounded offset. DDGI retains triangle lighting and its own histories.

Inside cascade coverage, missing probe data first uses coarser cascades, then a bounded nearby-probe search. Candidates
must pass conservative distance visibility; voxel occlusion also checks the connecting segment. The regular gather
contributes zero when all visibility tests reject its probes. Unrecoverable missing data stays dark; diffuse environment
fallback applies only outside coverage. Environment reflections remain separate.

## Sparse support cache

DDGI and HDDAGI use a sparse support cache for visible surfaces whose regular probe support is absent or weak with
occlusion on. The authored grid spacing, cascade count and outer coverage are preserved. Support probes occupy empty
voxel cells and are gathered only across voxel-visible connections. Well-supported regular probes measuring zero remain
dark. DDGI traces support lighting against triangles; HDDAGI uses voxel traversal and its light field, without requiring RT.

Visible requests take priority over secondary demand. The four-way cache holds at most 8,192 probes per raster camera
and updates 16 rays per requested probe per frame. Its fixed buffers total 3.56 MiB, plus a one-byte-per-pixel demand mask
and small per-frame uniform/upload allocations. Support tracing also requests probes at secondary hits. Triangle and
voxel lighting use the cached irradiance for bounce feedback.

Probe lighting uses spherical harmonics with a capped 64-update history. Geometry, lighting and bounce-feedback changes
preserve lighting at unchanged probe locations and shorten its history to an eight-update blend when next updated.
Requested probes trace fresh rays, and gathering still checks current voxel visibility. Field replacement, grid layout
changes and remapped probe locations reset lighting; camera-driven grid scrolling preserves overlapping world-space
history and clears pending local requests. Eight updates describe the blend weight, not a fixed convergence deadline.
Visible cached probes reserve their slots before new requests. A continuous spatial kernel and an eight-update influence
ramp reduce transitions between cells and newly populated probes. Hash eviction bounds storage; finite capacity and voxel
resolution still limit recovery in very complex views.
This adds camera lighting work. SDFGI and reflection captures do not directly query the sparse cache.

## SDFGI

SDFGI rasterizes supported contributors into a voxel field, builds distance information, and traces lighting from a fixed
probe grid. Voxel spacing controls geometric detail independently of the shared nominal probe coverage. Thin features
below the voxel scale may be lost or enlarged.

**Use Occlusion** is on by default and uses precomputed probe visibility to reduce light leaking through walls and
corners. Probe relocation is removed; old serialized relocation values are ignored. The remaining controls include
voxel spacing, ray count, history size, light update cadence, sky contribution, bounce feedback, energy, and normal/probe
bias. Smaller biases can preserve contact lighting but may expose self-intersection; larger biases can detach lighting
from nearby surfaces. Neither substitutes for adequate field resolution.

Diffuse reconstruction blends the eight neighboring nominal probes. Rough specular lighting uses probes, with field
traversal for sharper reflections. Occlusion and finite voxel resolution remain approximations of scene visibility.

## HDDAGI

HDDAGI stores an occupancy hierarchy and compact lit cells. Its traversal skips empty regions and reuses cached probe
paths while their geometry remains valid. Local changes update affected regions; scrolling preserves overlapping data.
Represented geometry or payload edits refresh geometry and cached paths while retaining main probe lighting history.
New samples replace old lighting over the configured history window. Scrolling retains overlapping world probes; newly
exposed locations, layout/resource replacement and explicit resets still start fresh history.

Camera GI always uses the full viewport resolution. **Use Occlusion** defaults on: enabled uses full visibility
attenuation with a zero floor; disabled bypasses that attenuation. The former numeric occlusion bias and half-resolution
controls are no longer exposed. Other controls include history size, light cadence, sky contribution, bounce feedback,
energy, receiver biases, and reflection filtering.

The camera surface pass evaluates normal and roughness before gathering GI. Optional horizontal and vertical filters
smooth sharp-reflection results; deferred lighting then applies material response and ambient occlusion. This adds
camera work beyond shared probe updates and should be included in performance comparisons.

## Composition and reflection captures

Visible camera backgrounds are independent of indirect environment lighting. Environment fallback fills missing GI
coverage; valid dark irradiance is not automatically treated as missing data. Local/global reflection probes and SSR
retain their roles in raster specular composition. See [Reflection probes](reflection-probes.md).

Reflection captures consume diffuse illumination without moving the shared anchor or sampling sharp GI reflections.
HDDAGI retains a lighting snapshot for a capture cycle; SDFGI reads the current publication per submission, so a multi-frame
SDFGI capture can observe lighting changes across faces.

## Diagnostics and known limitations

The Render Layer inspector exposes readiness, failure reasons, memory, timings, and provider-specific visualizers.
Python exposes `GetCurrentSceneGiSettings`, `SetCurrentSceneGiSettings`, and `GetCurrentSceneGiStatus`; settings bundles
contain the shared probes, provider, and DDGI/SDFGI/HDDAGI controls. Check the effective provider and completed transport,
not only the requested provider, when recording results.

SDFGI and HDDAGI have an unresolved black seam at Sponza's pillar/ground junction, also present in the previous golden.
Thin-wall leaks, corner darkening, cascade transitions, and delayed lighting response require separate review from
traversal speed.

Use [Rendering validation](rendering-validation.md) for capture and regression procedures. Compare providers with the
same scene, camera, probe coverage, warmup, and resolution, and record both shared GI and complete camera/frame costs.

[Dynamic Sponza capture script](../Scripts/capture_dynamic_gi.py) isolates light, geometry, and skinned motion;
run `python Scripts/capture_dynamic_gi.py --help` for setup and options.
