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
its SDK MeshRenderer draws independently, including in ray tracing. The GPU renderer repacks and uploads the compact
skeleton buffer each frame; a later revision can update only changed tree ranges or source it from a compute shader.

The `ecosyslab` demo prepares the GPU pipelines before growth and adds BranchRenderer and FoliageRenderer to its Acacia. It does not build CPU
geometry at the end of growth on mesh-shader devices. On devices without mesh shaders, it generates CPU geometry at
the end of growth. If the startup scene has no Soil or Climate, the demo creates them from the bundled descriptors.

## Shoot physics

After tree growth finishes, select the `DynamicTreeSkeleton` component on the same entity as `Tree`.
**Initialize from growth pose** captures the shoot skeleton as a paused rod simulation. **Run physics** advances it in `EcoSysLabLayer::FixedUpdate`;
**Pause physics** holds the current bend, and **Clear physics** restores the growth pose. The inspector exposes simulated
time per fixed update, substeps, iterations, stretch and bend stiffness, wood density, damping, gravity, an anchor offset,
and a force at an internode tip chosen by node handle. Reinitialize after changing density.

To test an earthquake-like response, run physics and move or rotate the tree entity (or its parent) in the editor.
The shoot roots follow that transform while the branches respond through the rod simulation. Moving the entity while
physics is paused carries the current bend with it. Changing entity scale clears the simulation; initialize it again.

BranchRenderer and FoliageRenderer use the transient bent pose, including child branch offsets at forks. Physics settings
are saved on `DynamicTreeSkeleton`, while the saved tree retains its growth pose. Loading starts with that pose and no
physics velocity. Growth is held while physics is initialized. Roots and generated CPU meshes do not deform in this
first version.

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
its demo tree during editor startup; exporting that result requires saving the generated scene/assets. The launcher
profile is not a standalone dataset-generation command.
