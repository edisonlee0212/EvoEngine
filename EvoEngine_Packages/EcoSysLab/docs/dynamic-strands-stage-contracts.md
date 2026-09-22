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
under the layer's stage policy. When physics pauses, its scripted time and
pivot/object motion pause too; fungus can still advance its target strands
without an extra layer dispatch.

Defaults are a 0.01-second physics frame, 25 substeps, one position and one
velocity constraint iteration, fungus disabled, structural damage enabled, and
segment collision disabled. When fungus is enabled, its independent default is
25 updates per frame even with mechanics paused. The legacy
`PhysicsParameters::enable_fungus` is the fungus-model eligibility flag (off by
default), while the separate fungus stage run/pause toggle defaults on. This
preserves fungus demos that explicitly enable the model.

## Stage data contract

| Stage | Main inputs | Main outputs |
| --- | --- | --- |
| Interaction | Selection operator state, segments and particles | Selection/highlight state in shared strand buffers |
| Fungus | Segment biological state, pair connectivity, current particle positions | Segment rot/health/moisture/diffusion fields; potentially pair integrity |
| Physics | Segment and pair topology, particles, foliage, biological state, constraints and colliders | Particle poses/velocities, foliage poses, pair damage/connectivity, grouping |
| Geometry update | Current simulation buffers and meshing settings | Alpha-shape uniform particles/tetrahedron flags or kinetic-Voronoi meshlet vertices/triangles |
| Draw registration and draw | Meshing buffers, live simulation buffers, materials and cameras | Camera/shadow draw work; alpha-shape draw-time scratch counters |

Simulation stages use the per-frame `DynamicStrands::strands_descriptor_sets`
layout. Bindings 0–7 and 10–13 contain simulation, foliage, and biology data.
Each mesher owns a separate two-buffer geometry descriptor set; its compute
shaders use set 1 and graphics shaders use set 3. Graphics set 2 remains the
lighting slot. The two meshers share only the descriptor layout schema, which
keeps the editor's pipeline-rebuild controls usable. The alpha-shape draw path
also reads current
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

The current implementation changes scheduling and APIs. Fungus diffusion now accepts its own parameter type and the
layer owns its own values and simulated time. `PhysicsParameters` retains the
fungus fields through inheritance for direct-demo compatibility; the layer
editor copies its fungus controls into the separate runtime values when they
change. Segment biology now uses its own GPU buffer at binding 13, with
upload/download packing kept in sync. The mechanical record is 352 bytes and
the biological record is 144 bytes per segment, 16 bytes more than the former
combined record because four mechanical values had occupied biological
`float3` padding. Pair integrity and stability flags still
live in mechanical buffers: the fungus edge kernel writes them immediately,
followed by its existing visibility barrier. Derived geometry bindings and a
fully explicit fungus-to-mechanics handoff remain to be migrated and validated.

## GPU ownership migration notes

The active fungus node kernel writes segment defense, carbon/lignin health,
white/brown rot, moisture, and diffusion accumulators. It also reads segment
boundary distance and particle root distance. The active edge kernel reads
pair endpoints/connectivity, segment positions, color, profile, and
obstruction fields; it accumulates diffusion into both endpoint segments.
It additionally zeros `bend_twist_bundle_integrity` and
`connectivity_integrity` after health thresholds and propagates
`reach_ground`/`quasi_stable`. Those pair and stability writes are mechanical
handoff outputs, not fungus-private state. Fungus applies pair breaks
immediately even when physics is paused: particle poses and velocities stay
frozen, but pair connectivity, subsequent fungus diffusion, and live pair
rendering reflect the break. Resuming physics uses the already-broken topology.

The original `Segment` and `SegmentPair` layouts co-located these fields with
orientation, mass, constraint, and draw fields in bindings 2 and 3 of the
shared set. The first ABI slice moves the whole biological segment block to
binding 13; pair severing and stability propagation still cross into the
mechanical buffers. Splitting only the diffusion fields would leave the
pair-severing and stability dependency hidden. A stage-owned layout should
carry biological values in a separate resource, expose the small position and
profile inputs fungus actually reads, and apply the integrity/stability
handoff at the same point between prediction and constraints when both stages
run. When mechanics is paused, apply pair breaks without advancing particle
mechanics. Fungus must read the newly broken topology on its next update; do
not queue the break or preserve diffusion across that pair.

The mechanical `Prediction/SegmentPair` kernel reads carbon health before the
interleaved fungus update. `Breaking/SegmentPair` subsequently reads moisture
and pair integrity, while `Prediction/Segment` and `VelocityUpdate/Segment`
read the stability/ground flags propagated by fungus. These are distinct
read phases: an ABI migration must not accidentally make the prediction kernel
see the *new* fungus result one substep earlier than it does today.

### GPU binding inventory

`DynamicStrandsSet0.slang` and `DynamicStrandsSet1.slang` expose simulation
buffers in descriptor sets 0 and 1. `DynamicStrands::UpdateBindings()` binds
the per-frame simulation set. Mesher-owned geometry descriptors use bindings
0 and 1 in compute set 1 or graphics set 3. The buffer owner and stage access
are:

| Binding | Current buffer | Relevant access |
| --- | --- | --- |
| 0–1 | Strands, nodes | Mechanical state and initialization |
| 2 | Segments | Mechanical state and stability; mesher inputs; draw-time color and diagnostics |
| 3 | Segment pairs | Fungus break/connectivity; mechanical constraints and damage; mesher topology; live pair draw |
| 4 | Segment correction data | Mechanical constraints |
| 5–6 | Hashed-grid elements/cell starts | Collision and grouping |
| 7 | Foliage | Mechanical leaf state and live foliage draw |
| 10–11 | Segment particles 0/1 | Mechanical poses; fungus position/root-distance inputs; mesher inputs; live segment/pair draw |
| 12 | Segment connection handles | Bundle constraints |
| 13 | Segment biology | Fungus state and diffusion; mechanical health inputs; diagnostic color modes |

The mesher geometry set has two bindings: alpha shape uses uniform particles
and tetrahedra; kinetic Voronoi uses meshlet vertices and triangles. These
resources no longer occupy slots in the simulation descriptor set.

The active shader access that crosses stage ownership is:

| Shader or pass | Read | Write |
| --- | --- | --- |
| `FungusDiffusion_node` | Segment biology, boundary distance, particle root distance | Segment defense, health, rot, moisture, diffusion accumulators and pair count (binding 13) |
| `FungusDiffusion_edge` | Segment biology/profile/color/obstruction, pair endpoints/connectivity, current particles | Biological diffusion (13), pair integrity (3), ground/stability flags (2) |
| `Operators/FungusFindClosest` and `FungusInjection` | Segment rot density and positions; selection scratch | Selection depth and injected rot density in segments |
| `Prediction/SegmentPair` | Segment carbon health and pair/particle state | Pair strain and strain limits |
| `Breaking/SegmentPair` and `Breaking/Leaf` | Segment moisture/ground state and live mechanics | Pair/leaf integrity and damage |
| Bundle constraints | Segment health/moisture and pair state | Mechanical correction and pair state |
| Alpha-shape update/filter passes | Segment and particle state, pair topology, derived data | Uniform particles and tetrahedra (geometry set 0–1) |
| Kinetic-Voronoi vertex/triangle passes | Segment and particle state, pair topology, derived data | Meshlet vertices and triangles (geometry set 0–1) |
| Segment/pair/foliage draw shaders | Live segments, pairs, foliage and particles; derived data for branches | No simulation buffers; task shaders select draw work |

`DynamicStrands::Upload()` packs CPU segments and particles separately and
uploads pairs, foliage, and mesher data. `Download()` reverses that packing.
A biological buffer split therefore needs both CPU transfer directions, the
active fungus and injection shaders, mechanical health readers, and diagnostic
draw color modes migrated together. The geometry descriptor split preserves
the two alternative mesher buffer types without overloading simulation slots.

The safe handoff order is **mechanical prediction → fungus node → fungus edge →
immediate pair/stability handoff → mechanical constraints and damage** when
both stages run. With physics paused, only the fungus node/edge and immediate
handoff run. The existing `DsFungus::Execute()` places a GPU barrier after the
node and edge dispatches; an ownership split must retain visibility before the
next fungus step, mechanical pass, or live pair draw. Benchmark the extra
buffer traffic/dispatches before removing the old shared fields and bindings.
