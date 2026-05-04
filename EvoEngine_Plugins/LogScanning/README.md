# LogScanning Plugin

[Back to Plugin index](../README.md)

LogScanning provides scanning and reconstruction support for forestry log workflows. It handles JoeScan scanner integration, scan profiles, scan assets, and reconstruction utilities.

## Build Status

- Registered only in the Windows block of `EvoEngine_Plugins/CMakeLists.txt` by default.
- Builds as `LogScanningPlugin`.
- Defines `LOG_SCANNING_PLUGIN`.
- Links the vendored Pinchot library on Windows and copies `pinchot.dll`.

## Main Responsibilities

- Import JoeScan configuration data.
- Place scanner prefabs from scan head configuration.
- Capture or preserve scan profiles from scanner hardware.
- Store log scan profiles as assets.
- Recenter and regularize scan data.
- Reconstruct profile grids and processed points from scan profiles.

## Main Entry Points

| Source | Role |
| --- | --- |
| `JoeScanScanner` | Private component for scanner lifecycle, scan jobs, config assets, and scan assets. |
| `LogScan` | Asset storing encoder profiles, 2D points, and brightness values. |
| `LogScanReconstruction` | Reconstruction utilities for processed points and profile grids. |
| `JoeScanConfig` | Scanner/head configuration import and prefab placement. |

## Registered Types

`LogGradingApp` registers:

- `LogScan` with `.jscan`
- `JoeScanScanner`

## SDK Integration

The Plugin uses SDK jobs, assets, private components, JSON assets, prefabs, serialization, asset references, fixed update hooks, and editor inspection.

## Future Work Notes

Hardware-facing scanner code should remain isolated here. Keep scan storage formats explicit and stable because grading and reconstruction workflows depend on them.
