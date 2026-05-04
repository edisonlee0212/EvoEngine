# TextureBaking Plugin

[Back to Plugin index](../README.md)

TextureBaking provides mesh-to-mesh texture transfer utilities. It is useful when generated or simplified geometry needs material maps projected from reference geometry.

## Build Status

- Registered only in the Windows block of `EvoEngine_Plugins/CMakeLists.txt` by default.
- Builds as `TextureBakingPlugin`.
- Defines `TEXTURE_BAKING_PLUGIN`.

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

TextureBaking uses SDK meshes, materials, private component references, serialization, relinking, and editor inspection. Apps such as `DemoApp` and `EcoSysLabApp` register the `TextureBaking` private component when this Plugin is available.

## Future Work Notes

Keep user-facing baking workflows in the private component and reusable projection/baking logic in `TextureBaker`. Be careful with asset/component reference serialization when adding new target/reference inputs.
