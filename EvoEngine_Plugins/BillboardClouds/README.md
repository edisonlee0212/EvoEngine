# BillboardClouds Plugin

[Back to plugin index](../README.md)

BillboardClouds provides utilities for converting detailed geometry into billboard cloud representations. It is useful for level-of-detail and vegetation-style rendering workflows.

## Build Status

- Registered only in the Windows block of `EvoEngine_Plugins/CMakeLists.txt` by default.
- Builds as `BillboardCloudsPlugin`.
- Defines `BILLBOARD_CLOUDS_PLUGIN`.

## Main Responsibilities

- Collect triangles from target meshes.
- Cluster triangles into billboard planes.
- Project and rasterize geometry into billboard textures.
- Transfer material channels such as albedo, normal, roughness, metallic, and AO.
- Dilate generated textures to fill invalid pixels.

## Main Entry Points

| Source | Role |
| --- | --- |
| `BillboardCloud` | Core data structures and algorithms for clustering, projection, rasterization, and dilation. |
| `BillboardCloudsConverter` | Private component exposing conversion through the editor. |

## SDK Integration

The plugin works with SDK `Mesh`, `Material`, private components, editor inspection, and generated geometry/texture outputs. EcoSysLab conditionally registers `BillboardCloudsConverter` when `BILLBOARD_CLOUDS_PLUGIN` is enabled.

## Future Work Notes

Keep billboard generation logic in this plugin. If a feature becomes a general mesh simplification, material transfer, or texture baking primitive, consider whether it belongs in the SDK or TextureBaking instead.
