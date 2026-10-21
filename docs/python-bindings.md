# Python Bindings

[Back to README](../README.md)

`PythonBinding` builds pybind11 modules for automation workflows.

The current Python binding layer focuses on core scripts and bindings that do not depend on runtime package C++ APIs. Package-specific Python C++ APIs for EcoSysLab and DigitalAgriculture are currently disabled while those domains move to runtime packages.

Example scripts live in `PythonBinding`. Package-level Python APIs should be added through a dedicated dynamic package interface later, so package behavior remains compatible with runtime package loading and unloading.

## Build and import

The normal editor installation builds `PyEvoEngine` unless `PythonBinding-PyEvoEngine=OFF`. It links the editor SDK;
it is not the exported player API and is not built in editor-free composition. Installed modules, native dependencies,
scripts and resource directories live together in `out/install/vs2026-x64/python`.

Use the Python interpreter/ABI selected by CMake. For example, `PyEvoEngine.cp315-win_amd64.pyd` requires 64-bit
CPython 3.15. See [Python selection](building.md#development-solution-and-tests). To check an installation:

```powershell
Push-Location out/install/vs2026-x64/python
python -c "import PyEvoEngine as ee; print(ee.__file__)"
Pop-Location
```

Run automation with that runtime directory as the working directory so package resource paths resolve, and pass absolute
project/resource/output paths when a script changes directories. Keep the native libraries and resources beside the
module; adding only the `.pyd` to `PYTHONPATH` is not a complete deployment.

## Capture and GI automation

| Entry points | Purpose |
| --- | --- |
| `RunDemoWindowless`, `Loop`, `Terminate` | Initialize a demo, advance application frames and release it. |
| `IsCurrentSceneReadyForCapture` | Check scene/shader presentation readiness before measuring. |
| `ConfigureCurrentSceneCameraForCapture` | Choose `Rasterization`, `RayTracing` or `RayQuery`, sample size and bounce count; unavailable requested backends are rejected. |
| `CaptureCurrentScene`, `ResizeCurrentSceneCameraForCapture` | Capture or resize the active main camera. |
| `GetCurrentSceneGiSettings`, `SetCurrentSceneGiSettings`, `GetCurrentSceneGiStatus` | Inspect/change the provider and settings, then verify publication and the effective provider. |

`RunDemoWindowless` currently accepts the case-sensitive setup names `Rendering`, `Bistro`, `Universe` and `Empty`.
These are not the editor's lowercase `--demo` IDs; use the native editor capture path for `sss` and other profiles.
Check boolean results and effective capabilities, allow required warmup, and always terminate the application after use.
The existing `EvoEngine_Tests/Rendering/render_demo_capture.py` is the maintained end-to-end example; use its `--help`
for module, isolated-resource and output arguments. See [testing](testing.md) and [rendering validation](rendering-validation.md).
