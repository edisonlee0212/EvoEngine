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

## Main Entry Points

| Source | Role |
| --- | --- |
| `EcoSysLabLayer` | Main package layer, editor UI, simulation controls, visualization camera, and render callback registration. |
| `Tree` | Private component for a simulated tree instance. |
| `TreeDescriptor` | Asset describing tree generation/growth behavior. |
| `TreeStructor` | Private component for tree reconstruction workflows. |
| `Climate` / `ClimateDescriptor` | Scene component and asset for climate state. |
| `Soil` / `SoilDescriptor` | Scene component and assets for soil context. |
| `DynamicTreeSkeleton` | Private component for dynamic skeleton simulation. |
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

- layer UI through `EcoSysLabLayer::OnInspect`
- private components for trees, climate, soil, reconstructors, simulators, and demos
- assets for descriptors, volumes, patches, and graph data
- serialization for persistent project data
- asset references for descriptor/material/resource links
- `RenderLayer` callbacks for custom strand and branch rendering
- temporary SDK assets such as `ParticleInfoList`, `Strands`, and editor cameras

## Future Work Notes

Most forestry features should live here rather than in the SDK. Add SDK functionality only when the feature is reusable outside forestry/agriculture. For new persistent EcoSysLab assets/components, make sure registration, serialization, editor inspection, and asset reference collection are all handled together.
