# TextureBaking Runtime Package

[Back to package index](../README.md)

TextureBaking provides mesh-to-mesh texture transfer utilities. It is useful when generated or simplified geometry needs material maps projected from reference geometry.

## Build Status

- Discovered by `EvoEngine_Packages/CMakeLists.txt`; `PackageInfo.cmake` limits support to Windows.
- Builds as the shared library target `TextureBakingPackage`.
- Registers `TextureBaking` through `PackageRegistrar`.

## Main Responsibilities

- Bake diffuse/albedo, normal, roughness, metallic, and AO channels.
- Transfer texture data from a reference mesh/material to a target mesh/material.
- Support configurable texture resolution, UV rewriting, ray casting range, sampling method, back-face culling, unresolved pixel behavior, and post dilation.

## Main Entry Points

| Source | Role |
| --- | --- |
| `TextureBaker` | Static baking implementation and parameter structure. |
| `TextureBaking` | Private component that references target/reference mesh renderers and exposes baking in the editor. |

## SDK Integration

TextureBaking uses SDK meshes, materials, private component references, serialization, relinking, and editor inspection. The runtime package registers the `TextureBaking` private component when it is loaded.

`TextureBakingEditorPackage` registers its inspector from `Editor/`; the runtime component retains only data/behavior and
serialization support. This package is the minimal [editor companion example](../../docs/editor-extension-workflow.md).

## Future Work Notes

Keep user-facing baking workflows in the private component and reusable projection/baking logic in `TextureBaker`. Be careful with asset/component reference serialization when adding new target/reference inputs.
