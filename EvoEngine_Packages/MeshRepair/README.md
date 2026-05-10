# MeshRepair Runtime Package

[Back to package index](../README.md)

MeshRepair contains mesh visibility and coloring utilities. It is a small utility runtime package for inspecting or repairing generated mesh data.

## Build Status

- Registered in the Windows block of `EvoEngine_Packages/CMakeLists.txt` by default.
- Builds as the shared library target `MeshRepairPackage`.
- Registers `MeshColoring` through `PackageRegistrar`.

## Main Responsibilities

- Sample mesh triangles for visibility analysis.
- Run visibility tests against a mesh or scene/entity target.
- Expose mesh coloring behavior through a private component.

## Main Entry Points

| Source | Role |
| --- | --- |
| `VisibilityTest` | Static visibility sampling and execution utilities. |
| `MeshColoring` | Private component for editor-driven mesh coloring. |

## SDK Integration

The package works with SDK `Scene`, `Entity`, `Mesh`, editor inspection, and private components.

## Future Work Notes

Keep domain-specific repair operations here. General mesh APIs that other Plugins need repeatedly may be candidates for the SDK.
