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
tree and refreshes them when the skeleton revision, mesh settings, or bark parameters change. CPU physics poses are
packed each frame while active. Packed node buffers are uploaded only when their content or GPU deformation ranges
change; foliage parameters and render-instance indices remain frame specific.

The `ecosyslab` demo prepares the GPU pipelines and instantiates the 2×2 `AppleForest.forestpatch`. It waits for background asset, upload, and shader preparation work to finish before growing the four trees for four years. The patch supplies BranchRenderer and FoliageRenderer materials. Procedural leaves use the descriptor's leaf width and height in tree-local units, independent of tree extent. The demo does not build CPU
geometry at the end of growth on mesh-shader devices. On devices without mesh shaders, it generates CPU geometry at
the end of growth. If the startup scene has no Soil or Climate, the demo creates them from the bundled descriptors.
On mesh-shader devices, each growth iteration replaces the current GPU shoot pose from the saved growth skeleton.
The rod solver stays paused during continuous growth and runs after growth stops. The live GPU pose feeds procedural
branch, foliage, and shadow draws.

## Shoot physics

Select the `DynamicTreeSkeleton` component on the same entity as `Tree`.
In CPU mode, **Initialize from growth pose** captures the shoot skeleton as a paused rod simulation. **Run physics**
advances it in `EcoSysLabLayer::FixedUpdate`; **Pause physics** holds the current bend, and **Clear physics** restores
the growth pose. The inspector exposes simulated
time per fixed update, substeps, iterations, stretch and bend stiffness, wood density, damping, gravity, and an anchor
offset. Reinitialize after changing density.

The experimental **GPU physics** option uses fixed roots and GPU rod constraints. Enabled trees in a scene share
vertex and rod buffers and solver dispatches while retaining their own time step, stiffness, damping, and gravity.
The shared state is rebuilt when its tree set or a shoot skeleton revision changes, or physics is reinitialized;
rebuilding starts every tree in that scene from its growth pose. Growth always uses this original shoot skeleton,
never the bent GPU pose. On each growth iteration, the new skeleton replaces the GPU dynamic state before rendering;
GPU solving pauses during continuous growth and resumes automatically when growth is idle. Enabled GPU physics also
starts automatically from the original pose after loading. Use **Reset GPU pose to growth** to discard a bend.
Its pose feeds procedural rendering without copying bent poses back to the CPU. Moving or rotating a tree or its forest
parent moves its pinned roots; paused trees follow rigidly, while running trees bend in response. Changing scale
reinitializes GPU physics from the growth pose.
Use the CPU mode for branch fracture and collider contact.
The renderer caches packed growth nodes and uploads them only when they change. GPU physics deforms a separate draw
buffer using the cached rest nodes; there is no CPU readback of the bent pose.

To test an earthquake-like response, move or rotate the tree entity (or its parent) in the editor.
The shoot roots follow that transform while the branches respond through the rod simulation. Moving the entity while
physics is paused carries the current bend with it. Changing entity scale reinitializes GPU physics; CPU physics
must be initialized again manually.
The default physics settings advance 0.016 seconds per fixed update with 4 substeps and 8 solver iterations. Stretch
stiffness is 8192, bend stiffness is 512, and damping is 0. Rod mass and stiffness scale with connected branch length
and the internode's thickness; bend stiffness also accounts for thickness at each junction.
The solver's structural-length scaling draws on the [Stable Cosserat Rods tree example](https://github.com/jerry060599/StableCosseratRods/blob/main/KittenEngine/Cosserat/examples/tree.cpp),
but EcoSysLab uses one rod per internode and its own solver. Existing saved components retain their settings; press
**Reset physics settings to defaults** in the `DynamicTreeSkeleton` inspector to restore all default physics settings
and reinitialize physics from the growth pose.

Optional branch fracture disconnects a child internode from its parent after enough time above the bend limit. Exposure
accumulates faster for stronger over-limit bending and decays slowly during recovery, so repeated shocks can break a
branch. The reference angle is specified for a 1 m junction with a 0.05 m radius; each junction scales the limit by its
length and radius.
The detached subtree stays in the simulation and accelerates under gravity.
Its transient pose makes procedural branch and foliage rendering follow the falling subtree, while the saved growth
skeleton remains unchanged. Resetting physics restores the original connections. Detached branches collide with enabled
`DsBoxCollider`, `DsSphereCollider`, and `DsCylinderCollider` scene volumes used by DynamicStrands. Fracture is disabled
by default; enable it in `DynamicTreeSkeleton` and tune the reference angle and exposure for the tree. Fallen rods sample
their centerlines against these volumes using their radii, including interior points. Contact corrections are shared by the
rod's endpoints and solved with its length and rotation constraints; contact friction also slows supported endpoints after
penetration is resolved, limiting persistent sliding. The solver does not
model full rigid-body angular inertia or continuous collision detection. When one end rests on a collider, a uniform-rod
pivot approximation carries angular velocity forward and applies gravity torque, so the free end accelerates as it falls.
Generated CPU meshes do not follow detached subtrees.

BranchRenderer and FoliageRenderer use the transient bent pose, including child branch offsets at forks. Physics settings
are saved on `DynamicTreeSkeleton`, while the saved tree retains its growth pose. Loading starts with that pose and no
physics velocity. A new growth step resets the transient physics pose before changing the original skeleton. Roots and generated CPU meshes do not deform in this
first version.

## Wind authoring

Add a `Blower` private component to an entity to define a wind emission surface. Its editor quad follows the entity
transform; the local +Z axis is the intended flow direction, and the entity's X/Y scale sets the surface dimensions.
Speed, particle rate, spread angle, and random seed are saved with the scene. Enabled Blowers emit into one scene-wide GPU
particle pool. The particles move at their emitted velocity for three seconds and are recycled when the bounded pool fills.
The EcoSysLab layer's **Show wind particles** option draws colored debug discs directly from that GPU pool; color changes
with speed. This first flow view has no SPH pressure, viscosity, obstacles, or tree forces yet.

## Main Entry Points

| Source | Role |
| --- | --- |
| `EcoSysLabLayer` | Runtime simulation and render callback registration. |
| `Editor/` and `EcoSysLabEditorLayer` | Authoring panels, inspectors, asset previews and visualization cameras. |
| `Tree` | Private component for a simulated tree instance. |
| `TreeDescriptor` | Asset describing tree generation/growth behavior. |
| `TreeStructor` | Private component for tree reconstruction workflows. |
| `Climate` / `ClimateDescriptor` | Scene component and asset for climate state. |
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
