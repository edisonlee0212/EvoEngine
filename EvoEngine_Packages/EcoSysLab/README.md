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
changes without building CPU branch meshes, and the same forest buffer supplies camera and shadow draws. The task shader
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
tree and refreshes them when the skeleton revision, mesh settings, or bark parameters change. Packed node buffers are
uploaded only when their content or GPU deformation ranges
change; foliage parameters and render-instance indices remain frame specific.

The `ecosyslab` demo starts in the project's `Default.evescene`, retaining its wall and ground colliders. It prepares
the GPU pipelines, instantiates the 2×2 `AppleForest.forestpatch`, and places a Blower beside the trees unless the
scene already has one. It waits for background asset, upload, and shader preparation work to finish before growing
the four trees for four years. Forest patches and descriptors attach DynamicTreeSkeleton to each tree so GPU physics
and wind can move their branches. The patch supplies BranchRenderer and FoliageRenderer materials. Procedural leaves
use the descriptor's leaf width and height in tree-local units, independent of tree extent. The demo does not build
CPU geometry at the end of growth on mesh-shader devices. On devices without mesh shaders, it generates CPU geometry
at the end of growth. If the startup scene has no Soil or Climate, the demo creates them from the bundled descriptors.
On mesh-shader devices, each growth iteration replaces the current GPU shoot pose from the saved growth skeleton.
The rod solver stays paused during continuous growth and runs after growth stops. The live GPU pose feeds procedural
branch, foliage, and shadow draws.

## Editor panel

The **EcoSysLab** panel groups scene-wide controls into Growth, Wind & air, Visualization, Geometry, and
Strands (advanced) tabs. Growth shows the tree count and simulated time, while Wind & air contains the air-volume
display and grid reset. **Show tree overlay** controls the editor's tree visualization; it does not hide procedural
BranchRenderer or FoliageRenderer draws. Select a tree entity to edit its own components in the inspector.

## Shoot physics

Select the `DynamicTreeSkeleton` component on the same entity as `Tree`.
Tree physics uses fixed roots and GPU rod constraints. Enabled trees in a scene share
vertex and rod buffers and solver dispatches while retaining their own time step, stiffness, damping, and gravity.
The shared state is rebuilt when its tree set or a shoot skeleton revision changes, or physics is reinitialized;
rebuilding starts every tree in that scene from its growth pose. Growth always uses this original shoot skeleton,
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

The `Climate` private component supplies scene-level ambient temperature, relative humidity, pressure, air coupling,
friction, and air-grid cell size through its **Air** inspector. Cell size defaults to 0.1 m and accepts 0.05–0.2 m.
Press Enter after editing to rebuild registered cells from the trees' current GPU poses without changing those poses.
At every cell size, registration fills an expanded box around each tree. Cells stay fixed in world space while their
air values update during swaying; tree growth and entity movement extend the registered region. Rebuilding clears
the current airflow. Existing scenes with an `AirEnvironment` component transfer those settings to `Climate` when
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
| `EcoSysLabLayer` | Runtime simulation and render callback registration. |
| `Editor/` and `EcoSysLabEditorLayer` | Authoring panels, inspectors, asset previews and visualization cameras. |
| `Tree` | Private component for a simulated tree instance. |
| `TreeDescriptor` | Asset describing tree generation/growth behavior. |
| `TreeStructor` | Private component for tree reconstruction workflows. |
| `Climate` / `ClimateDescriptor` | Scene climate and ambient air settings for growth and GPU wind simulation. |
| `Soil` / `SoilDescriptor` | Scene component and assets for soil context. |
| `DynamicTreeSkeleton` | Private component for shoot rod physics settings and transient simulation state. |
| `DynamicTreeStrands` | Strand-based tree representation and rendering support. |
| `SpatialPlantDistributionSimulator` | Private component for spatial competition and plant distribution. |

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
