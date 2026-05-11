# DatasetGeneration Runtime Package

[Back to package index](../README.md)

DatasetGeneration collects batch-oriented workflows for generating synthetic data from EcoSysLab and DigitalAgriculture scenes. It is the bridge between interactive simulation assets and reproducible dataset output.

## Build Status

- Registered by default from `EvoEngine_Packages/CMakeLists.txt`.
- Builds as `DatasetGenerationPackage`.
- Defines `DATASET_GENERATION_PACKAGE`.
- Depends on `EcoSysLabPackage` and `DigitalAgriculturePackage`.
- Emits `DatasetGeneration.evepackage` beside the package binary for runtime dependency discovery.

## Main Responsibilities

- Generate tree datasets from tree descriptors and growth settings.
- Generate tree growth sequences.
- Generate forest and forest patch data.
- Generate sorghum datasets.
- Export point clouds, meshes, rendered images, ray-traced images, depth, statistics, node graphs, and flow graphs.
- Provide tree and sorghum point cloud scanner private components.
- Provide camera capture and point cloud capture settings for reproducible output.

## Main Entry Points

| Source | Role |
| --- | --- |
| `DatasetGenerator` | Static batch-generation API for trees, forests, and sorghum. |
| `TreePointCloudScanner` | Private component for tree point cloud capture. |
| `SorghumPointCloudScanner` | Private component for sorghum point cloud capture. |
| `PointCloudScannerUtils` | Shared capture helpers. |

## Registered Types

`DatasetGenerationPackage.cpp` registers scanner components:

- `TreePointCloudScanner`
- `SorghumPointCloudScanner`

The current data generator apps are bootstrap apps that load `DatasetGeneration` by name. Direct static dataset generation commands are deferred until a package command/API interface is designed.

## SDK Integration

DatasetGeneration depends on scenes, cameras, render layers, generated meshes, asset loading, filesystem paths, and package layers. It uses SDK/project state as input and writes data products to output folders.

## Future Work Notes

Dataset generation code should remain deterministic where possible. New generation workflows should keep settings explicit, serializable, and script-friendly so they can be used from both app targets and Python bindings.
