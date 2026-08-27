# DigitalAgriculture Runtime Package

[Back to package index](../README.md)

DigitalAgriculture provides EvoEngine's sorghum and agriculture modeling workflows. It builds on the SDK and cooperates with EcoSysLab for environment-aware workflows such as soil integration.

## Build Status

- Registered by default from `EvoEngine_Packages/CMakeLists.txt`.
- Builds as `DigitalAgriculturePackage`.
- Depends on `EcoSysLabPackage`.
- Defines `DIGITAL_AGRICULTURE_PACKAGE`.
- Copies resources from `EvoEngine_Packages/DigitalAgriculture/Internals`.
- Emits `DigitalAgriculture.evepackage` beside the package binary for runtime dependency discovery.
- Does not require an accelerator-specific compute SDK.

## Main Responsibilities

- Sorghum plant descriptors, states, growth stages, and generators.
- Sorghum field/grid workflows.
- Mesh generation for sorghum plants.
- OBJ export for generated sorghum geometry.
- PAR sensor samples and imported sky-illumination datasets.
- CBTF/BTF import, asset storage, references, and inspection.
- A data-only `BtfMeshRenderer` component reserved for a future Vulkan backend.

## Main Entry Points

| Source | Role |
| --- | --- |
| `SorghumLayer` | Main package layer, material setup, mesh generation, and export. |
| `Sorghum` | Private component representing an instance of a sorghum plant in a scene. |
| `SorghumDescriptor` | Asset describing plant structure and mesh generation settings. |
| `SorghumGrowthStages` | Asset for staged growth data. |
| `SorghumState` | Asset for a concrete generated sorghum state. |
| `SorghumGenerator` | Asset for procedural sorghum generation. |
| `SorghumField` | Asset for field/grid placement. |
| `SorghumCoordinates` | Asset for coordinate data. |
| `SkyIlluminance` | Asset for lighting/illumination workflows. |

## Registered Types

`DigitalAgriculturePackage.cpp` registers:

- `SorghumDescriptor` with `.sorghum`
- `Sorghum`
- `SorghumGrowthStages` with `.sgs`
- `SorghumState` with `.ss`
- `SorghumGenerator` with `.sg`
- `SorghumField` with `.sorghumfield`
- `SkyIlluminance` with `.skyilluminance`
- `SorghumCoordinates` with `.sorghumcoords`
- `PARSensorGroup` with `.parsensorgroup`
- `CBTFGroup` with `.cbtfgroup`
- `BtfMaterial` with `.btf`
- `BtfMeshRenderer`
- `CBTFImporter`

## SDK Integration

DigitalAgriculture primarily uses private components, assets, editor inspection, asset references, and generated mesh/material workflows. It is consumed by DatasetGeneration through a runtime package dependency.

PAR/CBTF data remains loadable and editable, but illumination estimation, BTF rendering, and device upload are intentionally unavailable until a Vulkan implementation replaces the removed backend.

## Future Work Notes

Sorghum-specific modeling, data generation inputs, field layouts, and agricultural descriptors belong here. General mesh, asset, rendering, or editor improvements should stay in the SDK if they are reusable by other packages.
