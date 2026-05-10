# BillboardClouds Runtime Package

[Back to package index](../README.md)

BillboardClouds provides utilities for converting detailed geometry into billboard cloud representations. It is useful for level-of-detail and vegetation-style rendering workflows.

## Build Status

- Registered from `EvoEngine_Packages/CMakeLists.txt` by default.
- Builds as the shared library target `BillboardCloudsPackage`.
- Registers `BillboardCloudsConverter` through `PackageRegistrar`.

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

The package works with SDK `Mesh`, `Material`, private components, editor inspection, and generated geometry/texture outputs. It can be loaded from the app runtime `Packages` folder.

## Future Work Notes

Keep billboard generation logic in this package. If a feature becomes a general mesh simplification, material transfer, or texture baking primitive, consider whether it belongs in the SDK or TextureBaking instead.
