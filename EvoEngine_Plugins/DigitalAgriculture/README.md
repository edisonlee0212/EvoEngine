# DigitalAgriculture Plugin

[Back to plugin index](../README.md)

DigitalAgriculture provides EvoEngine's sorghum and agriculture modeling workflows. It builds on the SDK and cooperates with EcoSysLab for environment-aware workflows such as soil integration.

## Build Status

- Registered by default from `EvoEngine_Plugins/CMakeLists.txt`.
- Builds as `DigitalAgriculturePlugin`.
- Defines `DIGITAL_AGRICULTURE_PLUGIN`.
- Copies resources from `EvoEngine_Plugins/DigitalAgriculture/Internals`.
- Optional illumination functionality is compiled when `CUDA_MODULE_PLUGIN` is available.

## Main Responsibilities

- Sorghum plant descriptors, states, growth stages, and generators.
- Sorghum field/grid workflows.
- Mesh generation for sorghum plants.
- OBJ export for generated sorghum geometry.
- Sensor and illumination-oriented data structures.
- Optional CUDA/OptiX illumination estimation through `CudaModule`.

## Main Entry Points

| Source | Role |
| --- | --- |
| `SorghumLayer` | Main plugin layer, material setup, mesh generation, export, and illumination UI. |
| `Sorghum` | Private component representing an instance of a sorghum plant in a scene. |
| `SorghumDescriptor` | Asset describing plant structure and mesh generation settings. |
| `SorghumGrowthStages` | Asset for staged growth data. |
| `SorghumState` | Asset for a concrete generated sorghum state. |
| `SorghumGenerator` | Asset for procedural sorghum generation. |
| `SorghumField` | Asset for field/grid placement. |
| `SorghumCoordinates` | Asset for coordinate data. |
| `SkyIlluminance` | Asset for lighting/illumination workflows. |

## Registered Types

`SorghumLayer.cpp` registers:

- `SorghumDescriptor` with `.sorghum`
- `Sorghum`
- `SorghumGrowthStages` with `.sgs`
- `SorghumState` with `.ss`
- `SorghumGenerator` with `.sg`
- `SorghumField` with `.sorghumfield`
- `SkyIlluminance` with `.skyilluminance`
- `SorghumCoordinates` with `.sorghumcoords`

When CUDA support is enabled, it also registers:

- `PARSensorGroup` with `.parsensorgroup`
- `CBTFGroup` with `.cbtfgroup`

## SDK Integration

DigitalAgriculture primarily uses private components, assets, editor inspection, asset references, and generated mesh/material workflows. It is also consumed by DatasetGeneration and by the Python bindings.

## Future Work Notes

Sorghum-specific modeling, data generation inputs, field layouts, and agricultural descriptors belong here. General mesh, asset, rendering, or editor improvements should stay in the SDK if they are reusable by other plugins.
