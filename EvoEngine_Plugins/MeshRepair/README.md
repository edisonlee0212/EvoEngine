# MeshRepair Plugin

[Back to plugin index](../README.md)

MeshRepair contains mesh visibility and coloring utilities. It is a small utility plugin for inspecting or repairing generated mesh data.

## Build Status

- Registered only in the Windows block of `EvoEngine_Plugins/CMakeLists.txt` by default.
- Builds as `MeshRepairPlugin`.
- Defines `MESH_REPAIR_PLUGIN`.

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

The plugin works with SDK `Scene`, `Entity`, `Mesh`, editor inspection, and private components.

## Future Work Notes

Keep domain-specific repair operations here. General mesh APIs that other plugins need repeatedly may be candidates for the SDK.
