# Universe Runtime Package

[Back to package index](../README.md)

Universe is a simulation and rendering demo runtime package focused on GPU-driven star clusters and planet terrain.

## Build Status

- Registered by default from `EvoEngine_Packages/CMakeLists.txt`.
- Builds as the shared library target `UniversePackage`.
- Registers the `Star Cluster` and `PlanetTerrain` private components plus `UniverseLayer` through `PackageRegistrar`.

## Main Responsibilities

- Stable per-cluster star IDs and deterministic authoring samples.
- Independent FP64 compute dispatch and direct-GPU forward billboard rendering for every enabled cluster.
- Per-cluster density-wave, color, emission, timing, transform, and visual-radius parameters.
- Planet terrain chunks with dynamic LOD behavior.
- Editor UI for simulation parameters.

## Main Entry Points

| Source | Role |
| --- | --- |
| `UniverseLayer` | Owns the shared FP64 compute/forward pipelines and clock, schedules each cluster, and updates planet terrain. |
| `StarCluster` | Serializable private component and custom renderer that owns one cluster's authoring state, population, and frame-ring GPU buffers. |
| `PlanetTerrain` | Private component for planet terrain behavior and inspection. |
| `TerrainChunk` | Terrain chunk state used by planet LOD. |

## Registered Types

The package entrypoint registers `Star Cluster` and `PlanetTerrain` as private components. Stars are dense slots inside
their owning cluster rather than individual ECS entities. The legacy per-star data components are no longer registered.

## SDK Integration

Universe uses the SDK's compute pipeline, descriptor, buffer, profiling, and `ForwardExternal` APIs. Compute writes each
frame slot's device-local result buffer and the forward vertex shader consumes the same buffer without CPU particle
assembly or upload. Stars are additive, depth-tested emissive billboards rendered before volumetric clouds and
transparent geometry; they do not write the G-buffer, depth, motion vectors, or shadows. Position inspection requests an
asynchronous staging copy only while the inspector tree is expanded. Devices without Vulkan `shaderFloat64` receive a
clear diagnostic and do not run a lower-precision fallback.

## Design Documentation

See the [Universe design documentation](docs/README.md) for the proposed strategy simulation, map design, player
experience, and technical roadmap. These documents describe future direction rather than currently implemented
gameplay.

## Future Work Notes

Per-star culling, indirect drawing, motion vectors, and optional shadow rendering remain future work.
