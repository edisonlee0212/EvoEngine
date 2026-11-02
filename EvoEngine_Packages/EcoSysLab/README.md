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
Directional branch shadows use one light-facing quad per internode. A fragment shader intersects a short
rounded, tapered segment chain following the current GPU-deformed centerline, including the parent-end junction,
and writes the hit depth. This approximates bark and curved surfaces rather than reproducing every mesh triangle;
point/spot shadows retain the full branch mesh. The quad covers axial branches and supports affine entity transforms.
Run the profiler with `--full-shadows` for the full-geometry reference, or without it for the default impostor path.
Each profiling phase also saves the Scene camera image after timing collection for visual comparisons.

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
but EcoSysLab uses one rod per internode and its own solver. Existing saved components retain their settings; press
**Reset physics settings to defaults** in the `DynamicTreeSkeleton` inspector to restore all default physics settings
and reinitialize physics from the growth pose.

Branch fracture and contact with `DsBoxCollider`, `DsSphereCollider`, and `DsCylinderCollider` are not yet implemented in
the GPU solver. Existing saved values for those settings are retained for a future GPU implementation.
BranchRenderer and FoliageRenderer use the transient bent GPU pose. Physics settings are saved on
`DynamicTreeSkeleton`, while the saved tree retains its growth pose. Loading starts with that pose and no physics
velocity. A new growth step resets the GPU dynamic pose. Roots and generated CPU meshes do not deform.

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
