# EcoSysLab Runtime Package

[Back to package index](../README.md)

EcoSysLab is EvoEngine's digital forestry and ecosystem simulation runtime package. It provides tree modeling, reconstruction, growth simulation, soil/climate context, spatial competition, dynamic strands, mesh generation, and visualization tools.

## Build Status

- Registered by default from `EvoEngine_Packages/CMakeLists.txt`.
- Builds as `EcoSysLabPackage`.
- Depends on `BillboardCloudsPackage`.
- Defines `ECOSYSLAB_PACKAGE`.
- Copies resources from `EvoEngine_Packages/EcoSysLab/Internals`.
- Emits `EcoSysLab.evepackage` beside the package binary for runtime dependency discovery.
- Tries to link `CGAL::CGAL`; if CGAL is missing, the package still configures but warns that some features may be disabled.

## Main Responsibilities

- Tree descriptors and procedural tree growth.
- Shoot, root, fine-root, pruning, foliage, reproduction, and bark descriptors.
- Climate and soil modeling.
- Forest patch and spatial plant distribution workflows.
- Tree reconstruction through `TreeStructor`.
- Dynamic tree skeletons and dynamic strand physics.
- Tree mesh, skeletal graph, strand profile, strand mesh, and visualization generation.
- Volume assets such as radial bounding volumes and cube volumes.
- Rendering hooks for specialized strand/branch visualization.

Dynamic-strand bundle physics supports the compatibility-preserving `Legacy` mode and the opt-in `CoupledXpbd` and
`Hybrid` torque-capable modes. See [Dynamic-strand bundle solver](docs/dynamic-strands-bundle-solver.md) for selection,
material controls, damage behavior, diagnostics, and current validation limits.

## GPU tree rendering

Add a **BranchRenderer** private component to the same entity as a **Tree**, then assign its branch material.
The editor switches the Plant Visual camera to rasterization when either procedural renderer is present. The branch mode draws cylindrical shoot
and root branches directly from packed skeleton nodes through task and mesh shaders. It updates for growth and skeleton
changes without building CPU branch meshes. Each tree owns persistent node buffers used by camera and shadow draws. The task shader
chooses each node's subdivision counts; mesh workgroups evaluate rings from the nodes and their parents. Every child
starts from its parent's end ring, including at forks.

Add **FoliageRenderer** to draw two-sided rectangular leaf cards from the same uploaded skeleton. Its material override
takes priority over the foliage descriptor material. With neither assigned, EcoSysLab uses a green, rough thin-surface
subsurface material. Opaque and alpha-masked foliage materials are supported. Each renderer component can be enabled
independently; a disabled entity, Tree, or renderer component produces no procedural draw.

Trees instantiated by **ForestPatch** or **ForestDescriptor** receive both renderer components automatically. Their
branch and foliage materials come from the tree's bark and foliage descriptors when available. Trees without a bark
material share an opaque brown branch material within each instantiated patch.

The procedural modes need mesh shader support. BranchRenderer requires an opaque branch material. On unsupported
devices these components produce no procedural draw. CPU geometry is generated only when explicitly requested, and
its SDK MeshRenderer draws independently, including in ray tracing. The GPU renderer caches packed rest nodes per
tree and refreshes them when the skeleton revision, mesh settings, bark parameters, or physics allocation changes.
Each tree keeps its own render buffers and descriptor sets; unchanged trees avoid node assembly, copying, and bounds
scans each frame. Cached local bounds are transformed into scene bounds. Material routing changes refresh that tree's
node metadata; foliage parameters and render-instance indices remain frame specific.

The `ecosyslab` demo starts in the project's `Default.evescene`, retaining its wall and ground colliders. It prepares
the GPU pipelines, instantiates the 2×2 `AppleForest.forestpatch`, and places a Blower beside the trees unless the
scene already has one. It waits for background asset, upload, and shader preparation work to finish before growing
the four trees for four years. Forest patches and descriptors attach DynamicTreeSkeleton to each tree so GPU physics
and wind can move their branches. The patch supplies BranchRenderer and FoliageRenderer materials. Procedural leaves
use the descriptor's leaf width and height in tree-local units, independent of tree extent. The demo does not build
CPU geometry at the end of growth on mesh-shader devices. On devices without mesh shaders, it generates CPU geometry
at the end of growth. If the startup scene has no environment configuration, EcoSysLab creates default climate and soil descriptors in the scene system. Growth runs only while the scene is playing.
On mesh-shader devices, each growth iteration replaces the current GPU shoot pose from the saved growth skeleton.
The rod solver stays paused during continuous growth and runs after growth stops. The live GPU pose feeds procedural
branch, foliage, and shadow draws.

The launcher also offers **Windy Tree** (`EvoEngineEditor --demo windy-tree --editor`). It creates an empty generated
project and enters Play automatically. `WindyTreeDemoSettings` in `src/EcoSysLabLayerDemo.cpp` controls the script-built
16 Spruce trees in a 4×4 grid, ground, materials, lighting, air grid, and Blower without relying on `Default.evescene` or tree assets.
`WindyTreeDemoSettings::TreePreset` can switch between the script-defined Spruce and Acacia presets. The Spruce growth
parameters match the project's Spruce descriptor, while the demo supplies its own materials and smaller foliage cards
to keep branches visible.
The demo uses rasterization for procedural branches and hides foliage. Plant Visual overlays are enabled; gizmos defer first use until their instance data is uploaded. Selection refreshes branch overlays without rebuilding foliage or hidden climate grids. Spruce grows for five simulated years by default (Acacia for eight) after background preparation completes, and the Blower begins emitting wind after growth finishes. Wind particles are hidden by default so the branches remain visible; use Show wind particles in the EcoSysLab panel to display them. Stop returns to
the generated project's empty starting scene.

## Editor panel

The **EcoSysLab** panel groups scene-wide controls into Growth, Environment, Wind & air, Visualization, Geometry, and
Strands (advanced) tabs. Growth shows the tree count and simulated time. **Grow** adds the requested years to a runtime queue; each Play frame advances one configured growth step, including a shorter final step. **Stop growth** clears the queue, and Pause retains it. Environment selects the scene climate and soil descriptors, ambient air settings, and soil geometry controls. Wind & air contains the air-volume
display and grid reset. **Show tree overlay** controls the editor's tree visualization; it does not hide procedural
BranchRenderer or FoliageRenderer draws. Select a tree entity to edit its own components in the inspector.
On mesh-shader devices, the selected tree's shoot and root overlays use the procedural branch surface with debug
colors. Live shoots follow the current GPU physics pose; historical checkpoints show their saved poses, and roots
follow the selected root-model checkpoint. Debug rendering works without a BranchRenderer or branch material. Click a surface
in Plant Visual to inspect its internode; picking reads back a small GPU ID result after its frame completes.
Other enabled trees render their current procedural shoot/root surfaces in translucent white (18% opacity), sharing
the same GPU physics poses. **Display other trees** toggles this forest context. These surfaces do not write depth or
picking IDs, so the selected tree remains inspectable through foreground context trees. Cached tree data and shared
pipelines avoid rebuilding geometry or allocating buffers each frame.
The selected internode is tinted yellow with a white marker at the picked position. Pause physics before rotating,
pruning, or reversing the tree. Devices without mesh shaders retain cylinder overlays and CPU picking.

To profile the five-year, 16-Spruce Windy Tree demo, set `EVOENGINE_WINDY_TREE_PROFILE` to an output directory and run
`EvoEngineEditor.exe --demo windy-tree`. After growth, the editor records three configurations: full rendering,
Plant Visual disabled, and both Plant Visual and branch rendering disabled.
Each configuration warms up for 32 frames and measures 60 frames, saving CPU/GPU timing CSVs and
Chrome traces. The editor exits after recording; normal launches do not run this benchmark. The wind/air field remains
active along with the solver in each configuration; GPU pass timestamps separate their costs.
`python Scripts/profile_windy_tree.py --editor <installed-editor-path> --output <fresh-directory>` runs the capture
and writes a Markdown summary beside the CSVs. CPU scopes separate runtime update, air synchronization, per-tree
render registration, and debug rendering. Tree-cache build scopes and node-upload counters identify structural refreshes.
CPU scope times are inclusive; GPU pass call counts show the fixed-update
duty cycle, and pass times must be distinguished from whole-frame time.
Solver phase timings are opt-in: add `--physics-phase-timings` to record prediction, position constraints,
rotation constraints, and velocity updates. These extra GPU queries affect timing, so compare both paths with the
same option and use a separate capture without it for ordinary frame performance. The script rejects captures
whose timestamp query capacity was exceeded. `--full-physics-writes` selects the original full-record write path
for same-executable numerical and performance comparisons; ordinary launches use motion-only writes.
The corresponding environment flags are `EVOENGINE_TREE_PHYSICS_PHASE_TIMINGS` and
`EVOENGINE_TREE_PHYSICS_FULL_WRITES` (set to `1`, or unset to disable).
**EcoSysLab → Rendering** contains scene-wide branch quality settings, saved with the scene.
BranchRenderer stores only its material; old per-tree quality fields are ignored. Scenes without global settings
use the defaults below. The controls apply to every tree, including Plant Visual trees without BranchRenderer.

**Adaptive tessellation** is enabled by default. Its **Pixel error** target (0.5 pixels) and
**Segment length (pixels)** target (12 pixels) reduce radial samples and longitudinal rings with projected size;
the existing tree mesh subdivision settings remain the maximum detail. Counts use the current GPU pose and the
camera's unjittered projection. Near-plane intersections and visible bark displacement retain full detail.
Every child evaluates the parent's canonical radial count for its start ring. The first junction transition keeps
its full-detail endpoint; remaining spans are simplified. No internodes are removed and no ring data is stored.
Selected and surrounding Plant Visual trees use the same adaptive detail settings and preserve picking IDs.
This first stage adapts camera geometry and Plant Visual; existing shadow and foliage paths retain their behavior.
Use `--full-tessellation` with the profiling script (or set `EVOENGINE_TREE_FULL_TESSELLATION=1`) for the
full-subdivision reference. Changing views or these controls updates draw parameters without rebuilding tree buffers.
Camera branch tasks batch up to 32 internodes, bounded by device mesh-dispatch limits. Conservative camera
frustum bounds include the Bezier control hull, bark displacement and parent-end junction. They are tested
against the current jittered camera projection; tessellation still uses the unjittered view.
Each meshlet emits up to 20 triangles using at most 22 vertices. Ring-strip indexing evaluates identical
samples once without a duplicate search or atomic counters. Ring bounds use explicit component updates;
GPU raster regression tests compare connected thick-branch coverage against the original mesh shader. Angular UV seam
samples and separate internodes remain distinct; the original triangle topology and detail settings are retained.
This path does not change Plant Visual or shadow geometry. Use `--geometry-reference` in the profiling script
for the original task/triangle-local vertex path and `--geometry-details` for nested camera branch GPU timings.

EcoSysLab enables **Distant branch impostors** by default, with **Impostor diameter (pixels)**
(default 3, range 1–8). Small projected internodes become camera-facing bounding quads. The fragment shader
intersects the current GPU pose's rounded tapered segment chain and writes actual surface depth, geometric
normal, tangent, branch-relative UVs and the existing material into the deferred buffer. Parent attachment
width participates in selection; near-plane crossings and curves needing more than two approximation spans
at 0.25-pixel error remain meshes. Selection uses the unjittered camera; quad bounds use the jittered view.
This is an approximation: fine bark displacement and exact mesh silhouettes/attachment shading are not retained.
Switching uses a hard threshold without temporal history or dithering. Plant Visual uses the same threshold for
selected and surrounding trees, including roots. Its analytic surface preserves debug colors, selected-internode
highlights, white position markers and hit-based internode picking. Context trees retain translucent white and
do not overwrite pick IDs or depth. Trees without BranchRenderer use the same scene settings. Foliage, physics
and shadow rendering are unchanged. Devices using the original camera task fallback keep camera meshes.
Use `--impostor-pixels 2`, `3`, or `4` with `--geometry-details` to compare against `--impostor-pixels 0`
(mesh-only); the profiler defaults to 3 pixels. `EVOENGINE_TREE_IMPOSTOR_PIXELS` overrides both camera and debug
thresholds. `--debug-impostor-pixels 0` (or `EVOENGINE_TREE_DEBUG_IMPOSTOR_PIXELS=0`) independently disables debug
impostors for comparison. Both camera draw paths are
included in the branch geometry timing; compare aggregate GPU/frame time, not just time per draw.

Directional branch shadows use one light-facing quad per internode. A fragment shader intersects a short
rounded, tapered segment chain following the current GPU-deformed centerline, including the parent-end junction,
and writes the hit depth. This approximates bark and curved surfaces rather than reproducing every mesh triangle;
point/spot shadows retain the full branch mesh. The quad covers axial branches and supports affine entity transforms.
Run the profiler with `--full-shadows` for the full-geometry reference, or without it for the default impostor path.
Each profiling phase also saves the Scene camera image after timing collection for visual comparisons.
Directional shadow tasks process 32 internodes per workgroup and conservatively reject capsule chains outside
the current cascade before emitting mesh work. Bounds include radii, junctions and affine transforms;
no size threshold discards thin branches. Use `--shadow-reference` for the unbatched, unculled impostor
reference and `--shadow-details` for per-cascade branch GPU timings. These scopes are nested within
Directional Shadow and must not be added to its total.

EcoSysLab enables **Coarse canopy shadows** by default. **Canopy diameter (shadow texels)** defaults to 16
(range 4–512): a complete primary lateral subtree can use a solid ellipsoid when its conservative projected
diameter fits this threshold in a directional-light cascade. Cascade zero always uses branch shadows;
the apical main trunk and root skeleton stay separate. Larger thresholds trade branch gaps for cheaper shadows.
Each lateral subtree's membership is built when the skeleton/render cache changes. Persistent per-tree buffers
hold membership and envelopes; a GPU reduction after deformation updates conservative bounds from the current
pose, including parent attachments, curve control hulls and bark bounds. The ellipsoid encloses that box.
Selection is per cascade and independent of physics LOD/backend. No pose readback or ring mesh upload is needed.
This first version uses solid envelopes: leafless crowns cast fuller shadows, and threshold transitions are hard.
Point/spot shadows and the existing full-geometry directional-shadow override are unchanged.
Use `--canopy-texels 64` in `Scripts/profile_windy_tree.py` to compare against
`--canopy-texels 0`; `EVOENGINE_TREE_CANOPY_TEXELS` is the equivalent override.
Include the **EcoSysLab Canopy Bounds** GPU pass when assessing the net gain. Avoid additional detailed
timing scopes for this comparison: per-tree cascade scopes can exhaust the GPU timestamp capacity.

Rod-to-air feedback uses a rod-driven GPU pass: each rod visits nearby existing air cells and atomically
accumulates weighted motion, then a cell pass resolves the average. Scratch storage grows by grid chunks and
is reused between steps; coupling never registers new cells. The original cell-driven gather path remains
available for numerical and performance comparisons. Run the profiling script with `--gather` for that reference
path and without it for the default rod-driven path, using separate output directories. Both runs use the same
executable, forest preset, fixed-step settings, and rendering configurations. Compare the complete
`EcoSysLab Rod Air` GPU scope, which includes clearing, scattering, and resolving, rather than scatter alone.

The scene saves environment, growth, and advanced dynamic-strands settings. Growth time, dropped material, air motion,
and GPU physics poses remain runtime state and reset when Play starts from the saved scene.

## Shoot physics

Select the `DynamicTreeSkeleton` component on the same entity as `Tree`.
Tree physics uses fixed roots and GPU rod constraints. Each tree owns its vertex, rod, topology, and solver
workspace buffers, with local topology indices. A shared forest scheduler uses stable tree IDs, GPU device-address
records, and batched workgroup lists to dispatch all active trees while retaining their individual time step,
stiffness, damping, and gravity. Growth or reinitialization replaces only the changed tree's state; adding or removing
a tree preserves neighboring dynamic poses. IDs remain stable across rebuilds and are not reused until forest reset.
Wind and air cells remain shared; wind workgroup references resolve a tree ID and local rod index through the registry.
Removed buffers and scene controllers are retained until queued GPU work can finish. Growth always uses this original shoot skeleton,
never the bent GPU pose. On each growth iteration, the new skeleton replaces the GPU dynamic state before rendering;
GPU solving pauses during continuous growth and resumes automatically when growth is idle. Physics also
starts automatically from the original pose after loading. Use **Reset GPU pose to growth** to discard a bend.
Both solver buffer copies retain their initialized topology and rest attributes. Iterative passes write only
vertex position/velocity (32 bytes) and rod rotation (16 bytes) in contiguous attribute columns.
Wind and rendering consume those columns directly. Structural changes initialize both copies again.
Its pose feeds procedural rendering without copying bent poses back to the CPU. Moving or rotating a tree or its forest
parent moves its pinned roots, and branches bend in response. Changing scale
reinitializes GPU physics from the growth pose.
GPU branches sample the local air-cell velocity through the component's **Wind drag** setting. Air cells retain
momentum briefly after the emitting particles move away; growing trees receive no wind force.
The initial wind drag value is 20 and can be adjusted per tree.
The renderer caches packed growth nodes and uploads them only when they change. GPU physics deforms a separate draw
buffer using the cached rest nodes; there is no CPU readback of the bent pose.

To test an earthquake-like response, move or rotate the tree entity (or its parent) in the editor.
The shoot roots follow that transform while the branches respond through the rod simulation. Moving the entity while
physics is paused for growth carries the current bend with it. Changing entity scale reinitializes GPU physics.
The default physics settings advance 0.016 seconds per fixed update with 4 substeps and 8 solver iterations. Stretch
stiffness is 8192, bend stiffness is 512, and damping is 0. Rod mass and stiffness scale with connected branch length
and the internode's thickness; bend stiffness also accounts for thickness at each junction.
The solver's structural-length scaling draws on the [Stable Cosserat Rods tree example](https://github.com/jerry060599/StableCosseratRods/blob/main/KittenEngine/Cosserat/examples/tree.cpp),
but EcoSysLab uses its own solver and one rod per internode at full quality. Existing saved components retain their settings; press
**Reset physics settings to defaults** in the `DynamicTreeSkeleton` inspector to restore all default physics settings
and reinitialize physics from the growth pose.

Branch fracture and contact with `DsBoxCollider`, `DsSphereCollider`, and `DsCylinderCollider` are not yet implemented in
the GPU solver. Existing saved values for those settings are retained for a future GPU implementation.
BranchRenderer and FoliageRenderer use the transient bent GPU pose. Physics settings are saved on
`DynamicTreeSkeleton`, while the saved tree retains its growth pose. Loading starts with that pose and no physics
velocity. A new growth step resets the GPU dynamic pose. Roots and generated CPU meshes do not deform.

### Physics approximation quality

**EcoSysLab → Physics → Approximation tolerance** sets one forest-wide, camera-independent quality value.
Zero (the default) simulates every rod. A positive value allows centerline error up to that fraction of the
original local branch radius. The value is copied into each tree's quality settings; physical properties remain
independent. New trees copy their descriptor's default physical properties, while existing saved components keep
their own settings. Editing a tree does not change its descriptor or neighboring trees.

When growth stops, the CPU builds a nested hierarchy, merging low-error chains before difficult ones, and uploads its immutable topology and
reconstruction maps. Roots, forks, tips, thickness discontinuities and free hinges remain explicit. Levels vary
with the tree; they also enforce frame and merged-length limits. The span guard limits a merged rod to the larger of
`4 * sqrt(tolerance)` local radii or `(1 + 6 * sqrt(tolerance))` times the longest original member. This allows
bounded merging of already-slender internodes while preventing a straight chain from collapsing without limit. Each tree owns reusable GPU solver storage.
The GPU chooses a qualifying level, transfers the current pose and velocity, and builds indirect work lists for
only the active rods. A quality change needs no CPU pose readback, allocation or topology upload. Coarsening is
rejected if the current bend cannot fit the requested tolerance; periodic retries use hysteresis. Refinement
starts from the current pose, without restoring old fine-level motion. Density and stiffness edits refresh solver weights
without rebuilding the hierarchy or resetting the dynamic pose.

Mass and effective compliance are aggregated from original rods. Wind still samples the original member rods,
then distributes those loads to coarse endpoints. After solving, the GPU reconstructs the original skeleton for
branch/foliage rendering, picking and wind feedback. Approximation changes mechanical response as well as cost;
full quality remains the reference. Growth replaces the hierarchy and dynamic state from the growth pose.

Use `Scripts/profile_windy_tree.py --physics-tolerance 0.25` for an opt-in comparison against the default zero
at unchanged timestep, substeps and iterations. Captures record each tree's available rod counts and selected
level. These diagnostic readbacks happen outside measured frames.

### Subtree clustering policies

`BuildTreeClusterHierarchy(rest)` builds a CPU candidate hierarchy from the original physics rest snapshot.
Each rod is a potential complete-subtree root. Preorder intervals encode nested membership without storing
all descendant lists: construction and storage are linear in rod count, with iterative traversal for deep trees.
Candidates retain extent, attachment radius, total length, mass, branch order and pinned-member eligibility.
Non-tree input (cycles or multiply-parented endpoints) is rejected.

`SelectTreeClusters(hierarchy, rules)` accepts these policies:

| Policy | Eligibility |
|---|---|
| `Size` | Extent/tree height and attachment radius/trunk radius both meet their limits |
| `BranchOrder` | Root branch order meets the minimum |
| `Combined` | Both the size and branch-order conditions hold |

Botanical order starts at zero for the trunk, increases for lateral branches, and stays unchanged along apical
continuations. The CPU physics snapshot exports `branch_orders` independently of GPU rod attributes; zero-length
nodes still contribute their branching order even when they produce no rod. Order is derived from topology rather
than possibly stale cached order fields. Order-dependent policies do not select clusters if that metadata is absent.
Use the original rest snapshot, not an already chain-merged topology, to construct this hierarchy.

Multiple rules are alternatives (union). `Combined` is the way to impose both restrictions; adding separate Size
and BranchOrder rules allows either to qualify. The selector chooses the largest eligible ancestor, suppresses
its descendants, and emits individual rods everywhere else. Every original rod has exactly one element owner,
and every element records its parent. Roots and subtrees containing pinned or invalid physical members are retained.
The minimum cluster size is at least two rods. A selected cluster root can be passed directly to
`BuildTreeSubtreeBody` without a policy-specific solver representation.

Extent is the diagonal of a conservative subtree bounding box in the original model's rotation frame, including
rod radii. Tree height is the corresponding Y span; trunk radius is the maximum radius among root rods. These
ratios are invariant under rigid entity transforms and uniform scale. They are geometric heuristics, not mechanical
error bounds. Default rules use Combined, minimum order 2, minimum 2 rods, extent ratio 0.15 and radius ratio 0.25;
these are provisional API defaults, not an enabled scene preset.

The hierarchy is reused when changing rules. CPU reference tests cover multiple policies, exact ownership,
nested candidates, metadata export, transformed trees and a 50,000-rod chain. The scene backend below
selects and transfers live layouts on GPU; the separate approximation control selects chain LOD.

### Subtree physics LOD

Enable **EcoSysLab → Physics → Subtree physics LOD** to treat selected complete subtrees as rigid bodies with
compliant parent joints. The existing rod solver remains the default. The scene-wide settings are copied to
independent per-tree quality records; density, bend stiffness, damping, gravity and wind drag remain per-tree
physical properties. All quality decisions are camera-independent. Stretch stiffness and iteration count apply
to the rod solver; individual rods in the cluster solver are rigid and joints are solved directly.

Three rules can run together: size, botanical branch order, or both requirements together. Size limits are
relative to the original tree height and trunk radius; order zero is the trunk, and lateral children increase
order while apical continuation does not. Rules form a union, and the largest eligible ancestor wins. Roots
and pinned subtrees do not collapse. Disabling all rules retains individual rigid internodes within this solver.
The initial subtree preset enables branch order with a minimum order of two and at least two internodes.

CPU initialization uploads a compact subtree hierarchy, original properties and finest-capacity storage once.
The GPU selects disjoint active bodies, parent links and depth ranges when rules change. There is no CPU pose
readback, topology re-upload or GPU buffer reallocation for live quality changes. Larger trees naturally offer
more distinct clusterings; no fixed number of LOD levels is imposed.

At each switch, GPU state transfer captures current endpoints and rod orientations in each new body's local
frame, recomputes center of mass and cylinder inertia, and fits rigid velocity from linear/angular momentum.
The current bent geometry is preserved; internal deformation becomes frozen while clustered. Refinement
inherits current coarse motion, without restoring historical fine-state oscillations. Parent joints still target
the original rest orientations. Density changes scale mass/inertia without rebuilding topology. Joint stiffness
uses each original boundary joint's bend scale divided by its mean rod length; the direct rigid model is an
approximation of the stretch/shear rod solver, not numerically identical to it.

The forest scheduler retains each tree's buffers and settings. Narrow trees use dependency-ordered walks
batched across GPU invocations. Trees with more than four bodies per depth on average use one workgroup per
tree, processing same-depth bodies concurrently with workgroup barriers. This camera-independent heuristic
avoids CPU dispatches for each depth; package APIs also expose forced modes for comparisons. Child contributions
are gathered deterministically, without shared-parent write races. Geometry reconstruction writes the original
skeleton format consumed directly by branch, foliage and Plant Visual rendering.

Cluster loads sample the same trilinear climate-grid velocity, density and strength as the rod solver. Original
member cylinders retain their dimensions and lever arms, so wind produces both force and torque. A parallel
per-rod pass samples wind and evaluates loads at every substep; bodies then sum their member loads before
the articulated solve. The scene
path bounds endpoint aerodynamic acceleration; uniform wind remains available for mechanics tests. Reconstructed
positions/velocities also feed the existing rod-to-air coupling. Growth pauses physics and replaces changed
trees' GPU state from their growth skeleton; unchanged neighboring allocations are retained. Switching solvers
fits the current GPU pose and invalidates old LOD selection rather than recovering a stored bent pose. Saved
scenes contain configuration and original growth poses; live GPU state is transient.

`GpuTreeClusterPhysics::InitializeDynamic(rest, shared_state)` prepares live clustering on optional existing
GPU skeleton buffers. Step dynamic instances through `GpuTreeClusterForest`, whose `TreeStep` retains per-tree
settings, transform, rules, scheduling preference and running state. Paused entries can change clustering without
moving their displayed pose. Recreate the forest batch when its tree membership/topology changes, and do not
also step its members individually. Shared ownership plus frame retirement keeps replaced buffers alive.

`Initialize(rest, rules)` and the single-tree `Serial`/`LevelParallel` modes remain fixed-layout reference APIs.
`TreeClusterPhysics.*` and `TreeClusterForest.*` cover analytical acceleration, momentum, moving roots,
climate forces/torques, serial/workgroup equivalence, mixed-tree isolation, live rule unions, current-pose
handoff and growth/cache lifecycle. Opt-in scheduling benchmarks use GPU timestamps; synthetic solver timings
must not be presented as forest FPS. The Windy Tree profiler accepts `--cluster-order N` and
`--validate-cluster-switches` for scene measurements and warmup-time coarse/fine transitions.

Branch fracture, collider contact and tree-to-tree contact still await GPU support in the scene physics paths.

### Subtree-body mechanics prototype

`GpuTreeSubtreePhysics` is an opt-in package/test API for validating rigid subtrees, individually or as a coupled pair.
It is not selected by the editor approximation-tolerance control. `BuildTreeSubtreeBody(rest, root_rod)`
collects the selected rod and all descendants, excluding neighboring subtrees and rejecting pinned members.
Cylinder masses determine the center of mass and full inertia tensor, including parallel-axis contributions.

The GPU integrates bending and twisting about a driven parent attachment. The attachment position is exact;
its target orientation is compliant. Gravity and uniform-air drag produce torque from the actual member
positions and dimensions. Translation of the attachment supplies inertial loading for shaking. Angular stiffness
uses torque per radian (twist X, bending Y/Z in the cluster frame), and damping uses inverse seconds; these
prototype settings are not yet calibrated from the existing rod solver's stiffness controls. The solver includes
the gyroscopic term and treats spring/damping terms implicitly. Large-rotation accuracy still depends on timestep.

Initialize once from a rest skeleton, then call `Step(settings, anchor_world, target_rotation_world)`.
The target quaternion is the desired world orientation of the selected root rod's frame, not a rotation delta.
`GetTreeState()` exposes persistent full-resolution buffers in the existing procedural-rendering format.
Only cluster members and the shared attachment vertex are reconstructed; other branches remain untouched.
This state must not also be advanced by `GpuTreePhysics`. No pose readback or allocation is needed during stepping.

For reciprocal coupling, call `InitializeCoupled(rest, parent_rod, child_rod)`. The child must be a strict
descendant of the parent. Its subtree becomes body 1; the rest of the selected parent subtree becomes body 0.
Rods and mass belong to exactly one body. The shared attachment vertex is written once by the parent.
`Step` drives body 0's base position and target orientation; both body rotations remain dynamic.
`angular_stiffness` controls the child joint and `base_angular_stiffness` controls the parent's base attachment.
Both are torque per radian, with X twist and Y/Z bending. Damping is mass-proportional angular damping
relative to the world, not an internal joint damper.

The coupled GPU solver includes translational cross-inertia between parent and child and equal/opposite
joint spring torques. It solves their 6x6 angular system using a 3x3 Schur complement, with implicit spring
and damping terms. Attachment positions and velocities follow the parent rotation exactly. The base is
still prescribed; this is not a free-falling articulated-body solver or a coupling to the full rod solver.
`GetMotionBuffer()` contains one record in single mode and two records (parent, child) in coupled mode.

This prototype uses uniform wind input. It does not yet provide air-grid sampling, collision, fracture,
automatic cluster activation in the solver, live level transitions, or scene controls.
Wind uses the full solver's endpoint drag convention before its per-vertex acceleration cap; the prototype does
not apply that cap. It sums member loads serially for mechanical validation, so no forest speedup is claimed.
The `TreeSubtreePhysics.*` tests compare gravity, shaking and wind against full-resolution physics, and verify
inertia, analytic initial acceleration, bend/twist restoration, rotational covariance, torque-free momentum,
attachment continuity and reconstruction. `TreeSubtreePair.*` additionally verifies disjoint mass/inertia,
analytical child-to-parent loading, reciprocal spring reaction, momentum/energy conservation, frame covariance,
strong-stiffness response and shared-vertex reconstruction. Rigid clusters intentionally discard internal
branch deformation. Physical attachment calibration against flexible trees remains separate from these
mechanical correctness checks.

## Wind authoring

Add a `Blower` private component to an entity to define a wind emission surface. Its editor quad follows the entity
transform; the local +Z axis is the intended flow direction, and the entity's X/Y scale sets the surface dimensions.
Speed, particle rate, influence radius, wind strength, temperature, relative humidity, spread angle, and random seed
are saved with the scene. Older saved `air_density` values migrate to wind strength relative to 1.225 kg/m³.
New particles start at random positions behind the surface over one simulation step, spreading each frame's emission into
a continuous flow. Enabled Blowers emit into one scene-wide GPU particle pool. Particles carry velocity, temperature,
and specific humidity for three seconds and are recycled when the bounded pool fills. Updated Blower settings affect
newly emitted particles.
The EcoSysLab layer's **Show wind particles** option draws colored debug discs directly from that GPU pool; color changes
with speed. Trees register a grow-only, sparse GPU air grid around their shoot skeletons. Wind samples deposit weighted
velocity into registered cells as acceleration; cells retain velocity with adjustable friction and mix temperature and
humidity with incoming samples. GPU rods sample these cells for drag and locally slow or drive the air based on their
current motion. Tracer particles keep their velocity through branches while mixing temperature and humidity with local
air. Cells remain registered as trees grow or move until **Reset air grid** is pressed in the EcoSysLab layer inspector
or all trees are reset. The manual reset re-registers from current GPU tree poses without changing those poses.

The scene-owned `EcoSysLabSystem` supplies ambient temperature, relative humidity, pressure, air coupling,
friction, and air-grid cell size through the **Environment** tab. Cell size defaults to 0.1 m and accepts 0.05–0.2 m.
Press Enter after editing to rebuild registered cells from the trees' current GPU poses without changing those poses.
At every cell size, registration fills an expanded box around each tree. Cells stay fixed in world space while their
air values update during swaying; tree growth and entity movement extend the registered region. Rebuilding clears
the current airflow. Existing scenes with `Climate`, `Soil`, or `AirEnvironment` components transfer their configuration to the scene system when
loaded. Air density is estimated from cell
temperature, specific humidity, and ambient pressure.
This initial model mixes heat and moisture without phase changes or a pressure solver. The EcoSysLab layer's
**Show air volume** raymarches tree-registered cells directly from the GPU grid buffers, clipped by scene depth.
Blue and orange show air cooler and warmer than ambient; opacity increases with wind speed, so still air is invisible.
To inspect registration after a reset, enable **Show registered coverage** under **Show air volume** in the EcoSysLab
panel. Registered still-air cells appear green; normal wind visualization keeps them invisible.

## Main Entry Points

| Source | Role |
| --- | --- |
| `EcoSysLabSystem` | Scene-owned environment, queued growth, interactions, and dynamics. |
| `EcoSysLabLayer` | Package bootstrap and render callback registration. |
| `Editor/` and `EcoSysLabEditorLayer` | Authoring panels, inspectors, asset previews and visualization cameras. |
| `Tree` | Private component for a simulated tree instance. |
| `TreeDescriptor` | Asset describing tree generation/growth behavior. |
| `TreeStructor` | Private component for tree reconstruction workflows. |
| `ClimateDescriptor` / `SoilDescriptor` | Reusable assets for the scene system climate and soil models. |
| `DynamicTreeSkeleton` | Private component for shoot rod physics settings and transient simulation state. |
| `DynamicTreeStrands` | Strand-based tree representation and rendering support. |
| `SpatialPlantDistributionSimulator` | Private component for spatial competition and plant distribution. |

Legacy `Climate` and `Soil` component types remain registered so older scenes can load. EcoSysLab transfers their settings to the scene system and removes those components during migration; existing visualization entities stay in the scene.

## Registered Types

`EcoSysLabPackage.cpp` registers core persistent types, including:

- `TreeStructor`
- `Climate`
- `SpatialPlantDistributionSimulator`
- `DynamicTreeSkeleton`
- `DynamicStrandsDemo`
- `ClimateDescriptor` with `.climate`
- `RadialBoundingVolume` with `.rbv`
- `CubeVolume` with `.cubevolume`
- `ForestPatch` with `.forestpatch`

Package-specific apps request the package by name at startup instead of directly registering these C++ types.

## SDK Integration

EcoSysLab uses almost every major SDK extension point:

- editor panels and inspectors through `EcoSysLabEditorPackage` and its editor layer/adapters
- private components for trees, climate, soil, reconstructors, simulators, and demos
- assets for descriptors, volumes, patches, and graph data
- serialization for persistent project data
- asset references for descriptor/material/resource links
- `RenderLayer` callbacks for custom strand and branch rendering
- temporary SDK assets such as `ParticleInfoList`, `Strands`, and editor cameras

## Future Work Notes

Most forestry features should live here rather than in the SDK. Add SDK functionality only when the feature is reusable outside forestry/agriculture. For new persistent EcoSysLab assets/components, make sure registration, serialization, editor inspection, and asset reference collection are all handled together.

Runtime types and simulation remain usable without the editor companion. Put new inspectors/authoring state under
`Editor/`, following the [extension workflow](../../docs/editor-extension-workflow.md). The `ecosyslab` profile creates
its four demo trees during editor startup; exporting that result requires saving the generated scene/assets. The launcher
profile is not a standalone dataset-generation command.

### GPU solver attribute arrays and coefficient cache

Per-tree vertex and rod allocations store contiguous 16-byte attribute columns: positions/inverse mass,
velocities/radius, rest data and topology; rod rotations have their own column. Both ping-pong allocations
retain initialized static columns. This improves access locality without changing allocation ownership;
it is not a reduction in static storage. Rendering, Plant Visual and wind read the current arrays directly,
without an output conversion pass. CPU diagnostic/grid-reset readback uses `DownloadVertices`/`DownloadRods`
to reconstruct records; `GetVertexBuffer`/`GetRodBuffer` now expose the packed GPU layout.

A GPU preparation pass caches vertex inertia/denominators, rod stretch weights and angle bend weights.
It runs after structural initialization or changes to effective substep duration/stretch/bend stiffness.
Damping, gravity, wind and entity motion remain live parameters and do not invalidate those coefficients.
Internal solver barriers synchronize compute storage reads/writes; the final broad barrier preserves visibility
to downstream rendering, wind, transfers and readback. The solver equations and iteration ordering are unchanged.

`Scripts/profile_windy_tree.py --physics-reference` selects the previous AoS layout, uncached coefficients and
broad per-dispatch barriers for comparison. `--full-physics-writes` additionally enables the older full-record
writes and implies that reference path. The default uses attribute arrays, cached coefficients and scoped barriers.

### Development shader preparation

App builds and installs preserve content-keyed shader binaries. Unchanged shaders reuse their cache; shader source,
imports, defines and compiler changes still invalidate the matching entries. Unused ray camera, ReSTIR and denoising
variants compile when selected. Applications can opt into background preparation of unused rendering modes with
`ApplicationInitializationSettings::prewarm_optional_render_pipelines = true`.
EcoSysLab demo growth waits for required scene shaders and pending asset/geometry/texture work, not optional shader
prewarming. Opaque and masked procedural foliage pipelines are prepared separately when an enabled foliage renderer
first needs each class. Required pipeline failures still prevent readiness.
