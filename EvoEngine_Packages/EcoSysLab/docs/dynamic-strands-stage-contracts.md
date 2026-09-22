# DynamicStrands stage separation

## Current execution path

`EcoSysLabLayer::Update()` registers strand draw callbacks before calling
`DynamicStrandSimulation()`. The latter refreshes the current frame's shared
strand descriptor set, runs box selection, optionally calls
`DynamicTreeStrands::PhysicsStep()`, and calls the meshing `UpdateGeometry()`
method when geometry updates are enabled or stepped. The rendering toggle only
controls draw registration; it does not skip geometry compute.

`DynamicStrands::Physics()` runs pre-step and interaction operators once, then
repeats prediction, an optional fungus callback, position constraints and
colliders, velocity update, velocity constraints and colliders, and structural
damage for each physics substep. Grouping and optional segment collision follow
the loop. The layer schedules fungus independently and distributes its requested
updates among physics substeps when both stages run; without physics it executes
fungus alone. `DynamicStrandsDemo::Update()` also calls `PhysicsStep()` directly
and retains the legacy coupled cadence, so the layer is not the only simulation
caller.

Defaults are a 0.01-second physics frame, 25 substeps, one position and one
velocity constraint iteration, fungus disabled, structural damage enabled, and
segment collision disabled. When fungus is enabled, its independent default is
25 updates per frame even with mechanics paused. The old demo-facing
`PhysicsParameters::enable_fungus` still controls direct `PhysicsStep()` calls;
the layer uses its separate stage toggle.

## Stage data contract

| Stage | Main inputs | Main outputs |
| --- | --- | --- |
| Interaction | Selection operator state, segments and particles | Selection/highlight state in shared strand buffers |
| Fungus | Segment biological state, pair connectivity, current particle positions | Segment rot/health/moisture/diffusion fields; potentially pair integrity |
| Physics | Segment and pair topology, particles, foliage, biological state, constraints and colliders | Particle poses/velocities, foliage poses, pair damage/connectivity, grouping |
| Geometry update | Current simulation buffers and meshing settings | Alpha-shape uniform particles/tetrahedron flags or kinetic-Voronoi meshlet vertices/triangles |
| Draw registration and draw | Meshing buffers, live simulation buffers, materials and cameras | Camera/shadow draw work; alpha-shape draw-time scratch counters |

All stages currently share the per-frame `DynamicStrands::strands_descriptor_sets`
layout. Bindings 0–7 and 10–12 contain simulation/foliage data; the selected
mesher fills bindings 8–9. The alpha-shape draw path also reads current
segment state for colors, highlighting and small-segment visualization. The
kinetic-Voronoi branch path draws derived meshlets. Foliage and segment-pair
shaders read live simulation buffers directly.

## Scheduling behavior and remaining boundaries

- Fungus, physics and geometry updates each have their own run/pause and
  one-step control. Rendering has a separate per-frame visibility control.
- Fungus runs at 25 updates per frame by default, even when physics is paused;
  its update count is independently adjustable. If both fungus and physics run,
  preserve the existing fungus placement between prediction and constraints for
  each physics substep by default.
- A paused geometry stage retains its last derived branch positions/topology.
  Draw submission may continue. Foliage and segment pairs remain live; some
  alpha-shape branch colors, highlights and small-segment visuals may also
  change because their draw shaders read live simulation data.
- Disabling rendering only skips draw registration. Fungus, physics and geometry
  updates may continue. Disabling geometry updates does not disable rendering.
- Fungus-induced health and connectivity changes intentionally affect later
  mechanical steps. Independent scheduling does not erase this data dependency.
- Initialization/upload must establish valid derived geometry before a paused
  geometry stage can draw it. Stage controls must also cover direct demo calls.

## Validation matrix

| Case | Expected result |
| --- | --- |
| All stages on | Matches the pre-refactor board/log behavior and fungus cadence |
| Physics off, fungus on | Biological state advances at its own cadence; mechanics remain still |
| Fungus off, physics on | Mechanics advances using the last biological state |
| Geometry off, rendering on | Branch derived positions/topology stay stale; foliage and pairs track live simulation |
| Geometry on, rendering off | Derived buffers update without camera or shadow draw registration |
| Single-step each paused stage | Only the requested stage advances once; queued requests are consumed once |
| Pause/resume after damage | No implicit reset of fungus state, connectivity, or derived geometry |

The current implementation changes scheduling and APIs while preserving the
shared GPU layout. Fungus diffusion now accepts its own parameter type and the
layer owns its own values and simulated time. `PhysicsParameters` retains the
fungus fields through inheritance for direct-demo compatibility; the layer
editor copies its fungus controls into the separate runtime values when they
change. Direct-demo stage controls and GPU buffer and descriptor separation
remain to be migrated. The GPU split needs an explicit fungus-to-mechanics
handoff and the same validation matrix.
