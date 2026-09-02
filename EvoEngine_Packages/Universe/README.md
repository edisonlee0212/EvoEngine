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
Stars with projected diameter below one pixel use their base RGB without the emission multiplier. The vertex shader
estimates diameter per camera from the projection, viewport, radius, and clip-space W (perspective or orthographic).
Their RGB is uniformly limited just below that camera's bloom threshold minus soft knee, preventing their own
subpixel flicker from feeding bloom. If bloom starts at zero, these subpixel stars must be black; with bloom disabled,
base brightness is capped at one. At one pixel and above, emission is unchanged. Geometry, depth, and the shared
compute results are unchanged; nearby bright objects can still contribute bloom over these stars.
Bloom supplies the glow. Legacy alpha values remain serialized/in the GPU ABI but no longer affect rendering.
With depth writes disabled, opaque scene depth still occludes stars, but overlapping stars become draw-order dependent.
Stars do not write the G-buffer, motion vectors, or shadows. Devices without Vulkan `shaderFloat64` receive a clear
diagnostic and do not run a lower-precision fallback. Non-exact compute constants explicitly use FP64 literals.

## Population and lifetime

Use `SetStarCount(uint32_t)` / `GetStarCount()` and edit `seed` to author a cluster. Samples depend only on seed and
zero-based ordinal; growing preserves the prefix and shrinking removes the tail. Settings, transforms, and time
update only the current frame's parameter table. Count, seed, and enabled-membership edits repack shared samples,
reusing unchanged ranges. GPU capacity grows geometrically without automatic shrinking. These rare edits wait for
outstanding submissions before overwriting shared data; unchanged animation frames never explicitly wait.

The layer assigns runtime identities using component instances and their handles, independently of packed offsets.
This distinguishes live clones with copied handles and pooled instances with regenerated handles.
Disabled clusters do not dispatch or draw, but
their clocks continue unless paused. Deletion drops their clock state; changing scenes resets simulation state.
Only authoring fields serialize. Legacy `star_ids` migrate by count when `star_count` is absent; arbitrary-ID sample
distributions are not retained. Obsolete IDs and per-cluster depth-write settings are ignored.

The procedural demo creates two differently configured 250,000-star clusters (500,000 total) only when no authored
clusters exist. Both use configured disk diameter 3000, time scale 1, and disk/core/center emission 8, retaining
distinct seeds, colors, positions and eccentricities. New components also default to time scale 1 and emission 8;
explicit serialized authoring values are preserved. Runtime diagnostics, packed ranges, revisions, capacity, render slot, and draw count live in the
Universe Layer panel. The component inspector no longer lists positions.

The procedural demo disables tone mapping for its main and scene camera through independently owned temporary
post-processing stacks. All other flags/effect settings are copied, including bloom. Shared source assets remain
unchanged; leaving the demo restores original camera references. Replacing a camera or stack does not let an old
override overwrite the new user-assigned reference. Global SDK defaults are unchanged.

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
editor's one-second smooth transition entirely in star-local space to look toward the origin from 50 times the star's
visual radius, along the existing approach direction. At the origin, the camera's backward direction supplies the
approach direction. Nonpositive/nonfinite radii or distances outside the camera's FP32 range cannot be followed;
the selection remains and diagnostics explain why. Movement afterward uses normal local-space editor controls.
While following, hover publication and star selection are locked in both viewports. Continuous GPU picking still
runs, but its results cannot replace selection or publish hover. Space exits and reenables interaction; generations
reject outstanding locked-mode results. Exiting converts the current pose back to world space, not the old pose.

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

Current star-local controls/defaults evidence is recorded in [StarLocalViewValidation.md](docs/StarLocalViewValidation.md).
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
