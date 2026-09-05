# Universe Runtime Package

[Back to package index](../README.md)

Universe is a simulation and rendering demo runtime package focused on GPU-driven star clusters and planet terrain.

## Build Status

- Registered by default from `EvoEngine_Packages/CMakeLists.txt`.
- Builds as the shared library target `UniversePackage`.
- Registers the `Star Cluster` and `PlanetTerrain` private components plus `UniverseLayer` through `PackageRegistrar`.

## Main Responsibilities

- Count/seed authoring with deterministic cluster-local ordinal samples.
- Layer-owned FP64 compute and one direct-GPU forward billboard draw per camera for all enabled clusters.
- Per-cluster density-wave, color, emission, timing, transform, and visual-radius parameters.
- Planet terrain chunks with dynamic LOD behavior.
- Editor UI for simulation parameters.

## Main Entry Points

| Source | Role |
| --- | --- |
| `UniverseLayer` | Owns packed samples/ranges, independent cluster clocks, frame-ring GPU buffers, shared pipelines, and batched rendering; also updates terrain. |
| `StarCluster` | Authoring-only private component: star count, seed, density-wave settings, colors, emission, radius, phase, pause, and time scale. |
| `PlanetTerrain` | Private component for planet terrain behavior and inspection. |
| `TerrainChunk` | Terrain chunk state used by planet LOD. |

## Registered Types

The package entrypoint registers `Star Cluster` and `PlanetTerrain` as private components. Stars are dense slots inside
their owning cluster rather than individual ECS entities. The legacy per-star data components are no longer registered.

## SDK Integration

Universe uses the SDK's compute pipeline, descriptor, buffer, profiling, and `ForwardExternal` APIs. Each enabled,
nonempty cluster dispatches into a contiguous range of one shared device-local result buffer per frame slot. An
8-byte compute push constant selects the parameter-table entry and star offset. A compute-write to vertex-read
barrier precedes one six-vertex instanced draw for the entire batch per camera. There is no particle assembly,
particle upload or full-population readback. Picking alone copies one 48-byte result per active-camera frame.

Stars are opaque, depth-tested emissive discs rendered after deferred lighting and before volumetric clouds
and transparent geometry. The Universe Layer **Write star depth** option defaults to enabled. It is session-only,
survives scene switches, and resets when the layer is recreated. Depth testing stays enabled when writes are disabled.
With writes enabled, nearer stars replace or reject farther stars regardless of draw order (except equal-depth ties).
Blending is disabled; each disc has solid HDR color/emission and alpha one, with no radial falloff or shader halo.
Star billboards have a minimum diameter of one render-target pixel in each axis, in both overview and follow views.
The vertex shader estimates natural diameter per camera from projection, viewport, radius, and clip-space W.
Below one pixel, stars use base RGB without the emission multiplier, capped below the camera's bloom onset. Each screen
axis contributes its own enlargement ratio, so stretched viewports are handled correctly. Discs smaller than sqrt(2)
pixels snap to pixel centers so the circular mask cannot miss every sample. This changes raster coverage only, not
physical radii or shared compute/picking results.
Galaxy view uses zero enlargement fade. On entering star view, fade strength eases to the Universe Layer's adjustable
**Star-view fade strength** target, default 0.5; exit eases it back to zero using the same one-second curve as star size.
A strength of 1 preserves reciprocal-area compensation and values up to 4 fade more strongly. The target persists
across scene switches for the current layer session. Adjusting it changes only future view-transition draw constants;
it does not regenerate samples or change the bloom cap.
If bloom starts at zero, these subpixel stars must be black; with bloom disabled, base brightness is capped at one.
At one pixel and above, emission is unchanged. Depth behavior is unchanged: off-screen or occluded stars are not
forced visible, and nearby bright objects can still contribute bloom over them.
Bloom supplies the glow. Legacy alpha values remain serialized/in the GPU ABI but no longer affect rendering.
With depth writes disabled, opaque scene depth still occludes stars, but overlapping stars become draw-order dependent.
Stars do not write the G-buffer, motion vectors, or shadows. Devices without Vulkan `shaderFloat64` receive a clear
diagnostic and do not run a lower-precision fallback. Non-exact compute constants explicitly use FP64 literals.

## Population and lifetime

### Orbit buckets

**Star Cluster → Star minimum distance** defaults to **10.0**, measured between nominal centers in cluster-local
rendered units. Inner/outer nominal radii are center/disk diameter divided by 40 (the average ellipse semiaxis after
the existing position conversion). `floor((outer - inner) / distance)` gaps produce that many plus one equally spaced
orbits, including both endpoints. Radii 1–5 at distance 0.75 produce six orbits spaced 0.8 apart. A narrower range
uses only its inner orbit.

Each orbit has `floor(circumference / distance)` slots, with one star per slot. Ellipse circumference and slot positions
use a 2,048-segment FP64 arc-length table; circles use their analytic circumference. Stars choose uniformly among
non-full orbits, then among free slots, and uniformly sample arc length within that slot. The assigned proportion
is discrete; the assigned phase advances using the existing angular-speed equation. Gaussian offsets are retained.
This is **not collision prevention**: offsets, neighboring-slot jitter, intersecting/twisted ellipses, entity scaling,
and varying arc speed can reduce separation. Radii and overlaps between different clusters are not collision constraints.

The layer owns layouts and occupancy. **Universe Layer → Orbit buckets** reports requested/active counts, capacity,
nominal radial spacing, layout revision and per-orbit occupancy. Counts above capacity are retained in authoring but
only available slots are simulated/rendered/picked. Missing serialized distance loads as 10.0; old random orbit positions
are replaced. Invalid/nonfinite settings disable that cluster with a diagnostic. Layouts above 16,384 orbits or
32-bit per-orbit capacity are also rejected rather than silently changing spacing or allocating unbounded tables.

Seeded assignments have deterministic prefixes. Count edits replay allocation up to the active count, releasing the
tail on shrink and reproducing it on regrowth. Unchanged clusters reuse packed samples. Geometry keys are spacing,
center/disk diameter and eccentricity, core proportion and core eccentricity. Geometry edits rebuild the affected
layout and assignments, advancing population revision even when counts match so pending pick results are discarded.
The retained selection still identifies the same ordinal; if that ordinal exceeds active capacity, following detaches.
View-radius animation does not rebuild buckets; spacing/capacity are based on authored dimensions.

### Star size

Each **Star Cluster → Star size** inspector exposes **Minimum radius**, **Maximum radius**, and unitless
**Normalized deviation**. Each physical radius is
`mix(minimum, maximum, clamp(0.5 + normal_sample * deviation, 0, 1))`. This is a clamped normal distribution,
not a truncated/resampled one: samples beyond the normalized limits collect at the endpoints.

Defaults are minimum 0.1, maximum 15.0, and deviation 1/6. Zero deviation produces the midpoint. Minimum is
nonnegative; maximum is never below minimum. Legacy `radius_standard_deviation` values migrate by dividing by the
loaded radius range; legacy `visual_radius` is ignored.

Size samples are deterministic from cluster seed and ordinal, independent of the spatial Gaussian samples.
Changing size settings only updates the parameter table; it does not move stars, rebuild samples or change population
revisions. Rendering, fading, GPU picking, hover rings and CPU follow use each star's resulting radius; overview
framing includes the maximum radius. The three authoring values serialize and clone with the cluster.

The base-sample ABI is now **48 bytes**, including the FP64 size Gaussian and assigned orbital phase. The size sample
uses the previously unused half of the existing Box–Muller pair. Parameters remain 448 bytes and results remain 64
bytes. Parameter padding carries the size deviation and limits; graphics/picking layouts and draw count are unchanged.

Use `SetStarCount(uint32_t)` / `GetStarCount()` and edit `seed` to author a cluster. Gaussian samples depend only on seed
and zero-based ordinal; orbit assignments also depend on bucket geometry. Growing preserves the prefix and shrinking
removes the tail. Non-bucket settings, transforms, and time update only the current frame's parameter table.
Count, seed, bucket geometry, and enabled-membership edits repack shared samples,
reusing unchanged ranges. GPU capacity grows geometrically without automatic shrinking. These rare edits wait for
outstanding submissions before overwriting shared data; unchanged animation frames never explicitly wait.

The layer assigns runtime identities using component instances and their handles, independently of packed offsets.
This distinguishes live clones with copied handles and pooled instances with regenerated handles.
Disabled clusters do not dispatch or draw, but
their clocks continue unless paused. Deletion drops their clock state; changing scenes resets simulation state.
Only authoring fields serialize. Legacy `star_ids` migrate by count when `star_count` is absent; arbitrary-ID sample
distributions are not retained. Obsolete IDs and per-cluster depth-write settings are ignored.

The procedural demo creates one 500,000-star cluster at world origin only when no authored clusters exist.
It uses configured physical disk diameter 10,000,000, time scale 0.1, radius range 0.1–15 with normalized deviation
1/6, minimum star distance 10, and disk/core/center emission 8.
New components use the same defaults;
explicit serialized authoring values are preserved. Runtime diagnostics, packed ranges, revisions, capacity, render slot, and draw count live in the
Universe Layer panel. The component inspector no longer lists positions.

The procedural demo disables tone mapping for its main and scene camera through independently owned temporary
post-processing stacks. All other flags/effect settings are copied, including bloom. Shared source assets remain
unchanged; leaving the demo restores original camera references. Replacing a camera or stack does not let an old
override overwrite the new user-assigned reference. The demo never changes camera near or far distances in either
galaxy or follow view. Both cameras are initially positioned to contain the complete cluster, and their authored clip
ranges remain user-controlled. Global SDK defaults are unchanged.

## Orbit strands

The Universe Layer can display the cluster's nominal density-wave orbits as colored native strands. The master
toggle defaults off. **Orbit display** selects all available orbit buckets, only occupied buckets, or the selected
star's bucket. **Orbit strand radius** is a world-space radius and defaults to 0.1; increase it when an overview makes
the default strands subpixel. Each closed orbit uses 256 cubic segments and interpolates the cluster's center, core,
and disk RGB without star emission.

The layer caches one generated `Strands` asset per active cluster and submits one entity-free draw per cluster through
the ordinary scene geometry path. Consequently the same strands appear in main and scene cameras, depth-test normally,
receive camera post-processing, and cast no shadows. Mesh-shader support and enablement are required. Time, simulation
phase, population revision, and cluster/reference-frame transforms do not rebuild geometry. Orbit layout, density-wave
shape/color, display mode, or strand-radius edits rebuild only affected cached assets. Follow-mode radius scaling does
not rebuild orbit geometry; the panel reports the last rebuild/upload cost and submission status.

## GPU star picking

Universe routes picking to the main camera unless the Scene panel is focused. The SDK exposes read-only image-space
input snapshots, including letterboxing, resolution scaling, texture Y orientation, and overlay rejection; camera
navigation and ordinary editor entity selection remain independent. Picking runs continuously, not only on clicks.
The layer's **Star picking** section shows live hover and retained click selection as cluster identity plus zero-based
ordinal, with sampled render-space center, ray distance from the near plane, age, and readback status. Click misses retain
selection until another valid click replaces it. Selection survives repacking/settings edits; deletion, disable,
reseeding, or ordinal removal retain an unavailable identifying record and stop following. Scene changes reset it.
Snapshots are not live per-star position lists. The selected star additionally has a live CPU world-position readout.

The GPU intersects camera-facing discs with a minimum radius of **3 display pixels** (editable in the layer).
Geometry itself is unchanged. Opaque scene depth at the cursor rejects hidden hits before stars draw; clouds and
transparent geometry do not block picking. Nearest positive ray distance wins, with packed index breaking ties.
With star depth writes disabled, picking still chooses the nearest hit rather than draw-order appearance.
One 256-thread intersection/local-reduction pass is followed by reduction passes (500,000 stars: 1,954 -> 8 -> 1).
Each frame slot owns scratch, a final GPU result and coherently mapped staging. Readback happens only after normal
frame-slot reuse, with no new CPU wait; hover and clicks therefore have frame-ring latency. Population/scene/camera
and reference-frame revisions prevent stale results being applied. Picking adds a steady GPU cost even without clicking.

## Star-local following and hover

With a star selected, press **Space** in either focused camera viewport to toggle scene-camera following. UI/text
input does not trigger this shortcut. Entering converts the scene-camera pose without a view jump, then uses the
editor's one-second smooth transition entirely in star-local space to position the camera at `[0, 0, 20 × radius]`,
looking down negative Z. The selected star and cluster center are therefore aligned in front, with the star nearer.
Nonpositive/nonfinite radii or distances outside the camera's FP32 range cannot be followed;
the selection remains and diagnostics explain why. Movement afterward uses normal local-space editor controls.
While following, hover publication and star selection are locked in both viewports. Continuous GPU picking still
runs, but its results cannot replace selection or publish hover. Space exits and reenables interaction; generations
reject outstanding locked-mode results. Exiting converts the current pose into the scaled galaxy frame and smoothly
moves out to frame the complete cluster, looking at its displayed origin, rather than restoring the old pose.

For every rendering camera in both views, stars beyond 70% of that camera's far distance are smoothly compressed
toward a strict 99% limit. The forward vertex shader scales camera-relative position and billboard radius together,
preserving screen position and apparent size while keeping distant stars inside the camera frustum. Invalid far
distances disable compression for that camera. Compression is monotonic and does not alter compute results, physical
positions/radii, picking, CPU follow evaluation, or orbit strands.

The authored and simulated disk diameter remains 10,000,000. Galaxy view applies a fixed 0.001× Universe display
frame, producing an effective 10,000-diameter cluster without changing physical batch parameters or camera clipping.
Displayed star radii receive the 30× overview boost before this coordinate scale, for a numeric 0.03× physical-radius
factor. In locked view the star-local frame uses physical scale and the radius eases to each star's actual sampled size.
The layer applies this multiplier
only to copied GPU render parameters, without changing authoring values, physical CPU radii, positions, orbital phase,
or samples. Radius and camera use the same one-second quartic ease-out; reversing mid-transition starts from the current
value and pose. A cached per-population Gaussian bound and phase-independent orbital bound
include outliers, cluster transforms and star radii in overview framing for either viewport aspect ratio. The bound
accounts for orbital tilt: vertical tails combine geometrically with a flat disk instead of being added directly to
its final scaled displayed radius. This retains the complete cluster with a 2% framing margin. Authored clusters retain
their own diameter/radius/time settings; the view multiplier applies to rendered radii but not follow distance.

The CPU evaluates only the selected star using its deterministic sample and the same simulation clock/equations.
Its position is the frame origin; negative Z points toward the cluster entity's world origin, with cluster Y as the
up reference. All clusters receive this rigid world-to-star transform through their existing compute parameters.
The star remains at zero and the cluster origin lies on negative Z. Degenerate axes use a previous valid orientation
or deterministic fallback. Settings, independent pause/time scales, and population repacking continue normally.

Only Universe stars and the scene camera are rebased. Other scene objects and the main camera are unchanged and
therefore do not retain their former alignment with the stars during following. Authoring transforms are never changed.
Follow state and selection are runtime-only and reset on scene changes; unavailable targets detach without auto-resuming.

Outside follow mode, the active picking viewport displays a cyan ring around the hovered star, with a two-display-pixel
gap and thickness, accommodating the minimum picking radius. A separate six-vertex draw reads the current GPU result;
it depth-tests, preserves depth/destination alpha, and limits brightness below bloom onset. At a zero bloom onset the
ring is black. Hover still has picking frame-ring latency, but the ring position uses the current simulation result.
All hover rings are suppressed while following; continuous picking remains active. No population readback is added.

For reproducible capture testing only, set `EVOENGINE_UNIVERSE_PICK_BENCHMARK=1` before launching the editor to
override the active camera's cursor to its center and disable clicks. Leave it unset for normal interaction.
The value `follow` additionally selects the first completed hover and enters follow once, for CPU-follow profiling.
Its main-camera image changes because only stars and the scene camera are rebased; compare normal-mode captures for
like-for-like total-frame timing and use follow captures to measure the CPU evaluator separately.

## Validation

Fixed-diameter/radius-view evidence: [FixedGalaxyViewValidation.md](docs/FixedGalaxyViewValidation.md).

Current normalized-radius and scaled-display evidence:
[NormalizedGalaxyDisplayValidation.md](docs/NormalizedGalaxyDisplayValidation.md).

Orbit visualization evidence: [OrbitStrandValidation.md](docs/OrbitStrandValidation.md).

Orbit allocation and current performance evidence: [OrbitBucketValidation.md](docs/OrbitBucketValidation.md).

Current single-cluster transition evidence is recorded in [SingleClusterViewValidation.md](docs/SingleClusterViewValidation.md).
The previous star-local controls/defaults evidence is recorded in [StarLocalViewValidation.md](docs/StarLocalViewValidation.md).
The previous follow/hover milestone is recorded in [StarFollowValidation.md](docs/StarFollowValidation.md).

Continuous picking test/build/performance evidence and the remaining live UI check are recorded in
[StarPickingValidation.md](docs/StarPickingValidation.md).

`UniversePackage_Tests` covers authoring, legacy migration, packing, independent clocks, population boundaries,
disable/enable, deletion, reset, and exact buffer layouts. Its GPU test uses test-only readback to compare mixed
packed ranges and two 250,000-star clusters against double-precision CPU equations, including an output guard.

Build the package and focused tests using `/p:BuildProjectReferences=false` on Windows after building the SDK when its
viewport or scene-camera helper changes. The follow feature does not change SDK rendering coordinates or GPU layouts.

See [batched rendering validation](docs/BatchedRenderingValidation.md) for test evidence, deployment commands,
benchmark results, and remaining visual-validation limits.

## Design Documentation

See the [Universe design documentation](docs/README.md) for the proposed strategy simulation, map design, player
experience, and technical roadmap. These documents describe future direction rather than currently implemented
gameplay.

## Future Work Notes

Per-star culling, indirect drawing, motion vectors, and optional shadow rendering remain future work.
