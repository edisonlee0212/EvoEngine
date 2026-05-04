# Universe Plugin

[Back to Plugin index](../README.md)

Universe is a simulation and rendering demo Plugin focused on large ECS workloads, star clusters, and planet terrain. It is useful as a compact example of data-component-heavy scene simulation.

## Build Status

- Registered by default from `EvoEngine_Plugins/CMakeLists.txt`.
- Builds as `UniversePlugin`.
- Defines `UNIVERSE_PLUGIN`.

## Main Responsibilities

- Star cluster creation and removal.
- Parallel star position calculation and rendering through particles.
- ECS data components for star simulation state.
- Planet terrain chunks with dynamic LOD behavior.
- Editor UI for simulation parameters.

## Main Entry Points

| Source | Role |
| --- | --- |
| `UniverseLayer` | Main Plugin layer for star cluster controls, ECS queries, star archetype creation, and planet terrain updates. |
| `PlanetTerrain` | Private component for planet terrain behavior and inspection. |
| `TerrainChunk` | Terrain chunk state used by planet LOD. |

## Registered Types

`UniverseLayer.cpp` registers several data components:

- `StarPosition`
- `SelectionStatus`
- `StarInfo`
- `SurfaceColor`
- `DisplayColor`
- `OriginalColor`
- `StarOrbitOffset`
- `StarOrbitProportion`
- `StarOrbit`
- `StarClusterIndex`

`PlanetTerrain` is registered by apps that use it, such as `DemoApp`.

## SDK Integration

Universe is a strong example of SDK data components, archetypes, entity queries, temporary particle assets, editor layer inspection, and render-layer draw calls.

## Future Work Notes

Use this Plugin as a reference for ECS-scale demos. Features that are specifically about stars, orbital visualization, or planet terrain belong here; general ECS or renderer improvements belong in the SDK.
