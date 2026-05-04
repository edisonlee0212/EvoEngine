# LogGrading Plugin

[Back to Plugin index](../README.md)

LogGrading provides forestry log modeling and grading tools. It represents log geometry, defects, grading faces, and procedural log generation for grading workflows.

## Build Status

- Registered only in the Windows block of `EvoEngine_Plugins/CMakeLists.txt` by default.
- Builds as `LogGradingPlugin`.
- Defines `LOG_GRADING_PLUGIN`.

## Main Responsibilities

- Represent log cross sections and boundary points.
- Mark, erase, and color defect regions.
- Compute log grading data and grading faces.
- Generate procedural logs.
- Generate cylinder/flat meshes and particle surfaces for visualization.

## Main Entry Points

| Source | Role |
| --- | --- |
| `LogWood` | Core log data structure and grading calculations. |
| `LogGrader` | Private component exposing procedural log setup, mesh generation, grading, and editor UI. |
| `ProceduralLogParameters` | Parameters for procedural log creation. |

## Registered Types

`LogGradingApp` registers:

- `LogGrader`
- `BasicBarkDescriptor` with `.bs`

## SDK Integration

The Plugin uses SDK private components, meshes, particle lists, materials/descriptors from EcoSysLab, editor inspection, and scene entity/component workflows.

## Future Work Notes

Log grading rules and log-specific visualization should stay in this Plugin. General forestry primitives shared with EcoSysLab should be evaluated carefully before moving code.
