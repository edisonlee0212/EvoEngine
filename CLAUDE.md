# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

### Windows (from VS Developer Command Prompt)
```bash
build.cmd [--clean] [--no-test] [--verbose] [Debug|Release]
build.cmd                    # Release, no clean
build.cmd --clean Release    # Clean Release build
build.cmd --no-test          # Skip tests

# Build a single target (faster for Python binding work)
cd out/build/x64-Release
ninja PyDigitalAgriculture
```

**Requirements:** Visual Studio 2019/2022 with C++ workload, vcpkg (`%LOCALAPPDATA%\vcpkg\vcpkg.path.txt`), Ninja, Vulkan SDK, CUDA toolkit (for CudaModule/ray tracing).

**Output:** `out/build/x64-Release/PythonBinding/Release/PyDigitalAgriculture.cp39-win_amd64.pyd`

Note: On Windows the `.pyd` lands in `PythonBinding/Release/`, while dependent DLLs stay in `PythonBinding/`. Import requires:
```python
os.add_dll_directory(binding_dir)          # PythonBinding/
sys.path.insert(0, os.path.join(binding_dir, "Release"))
```

### Linux
```bash
bash build.sh [--clean] [--no-test] [--verbose] [Debug|Release]
```
**Requirements:** clang-14, cmake, ninja-build, libwayland-dev, libxkbcommon-dev, xorg-dev, Vulkan SDK, python3.12-dev.

### Tests
```bash
cd out/build/x64-Release && ctest        # Windows
cd out/build/x64-Release && ctest -V     # Linux (verbose)
```

## Architecture

### Layer Stack

EvoEngine is composed of layers pushed onto the application before `Application::Initialize()`. Layers are global singletons retrieved anywhere via `Application::GetLayer<T>()`.

```
EvoEngine_SDK/          → Core: Application, ECS, RenderLayer, WindowLayer, EditorLayer
EvoEngine_Plugins/      → Domain: SorghumLayer, EcoSysLabLayer, RayTracerLayer
PythonBinding/          → Thin pybind11 wrappers over C++ static methods
EvoEngine_App/          → Executables; see here for layer push order examples
```

**Lifecycle hooks** (in order): `OnCreate → PreUpdate → Update → FixedUpdate → LateUpdate → OnDestroy`

### Entity-Component-System

Two component types with different storage semantics:
- **DataComponent** (`IDataComponent`) — contiguous 16 KB chunks, cache-friendly, no lifecycle. Use for bulk data (position, mass).
- **PrivateComponent** (`IPrivateComponent`) — one instance per entity, has lifecycle callbacks and can hold `AssetRef`. Must implement `CollectAssetRef()` / `Relink()` for YAML serialization.

Assets are referenced by opaque `Handle` (uint64_t), never raw pointers, supporting lazy loading and serialization.

### Plugin System

Plugins registered in `EvoEngine_Plugins/CMakeLists.txt` via `register_evoengine_plugin()`. Each plugin is a CMake target that adds layers, components, and assets. Enable/disable at `EvoEngine_Plugins/CMakeLists.txt:46–63`.

### Python Bindings

Each module is three files:
1. `PythonBinding/src/Py{Plugin}.cpp` — C++ static method implementations
2. `PythonBinding/include/Py{Plugin}.hpp` — declarations
3. `PythonBinding/src/Py{Plugin}Module.cpp` — `PYBIND11_MODULE()` registrations

`PyEvoEngine.cpp` contains shared core APIs (asset management, entity CRUD, `Run()`, `Loop()`, `Terminate()`).

## Sorghum Illumination Pipeline

### APIs (branch `claude/par-calibration`)

```python
# Initialization — no render window needed
pda.PushRayTracerLayer()
pda.RegisterClasses()
pda.PushSorghumLayer()
pda.Run(str(PROJECT_PATH))

# Per-scenario setup
pda.SetSunDirection(azimuth_deg, elevation_deg)   # arg0=azimuth, arg1=elevation (NOT reversed)
pda.SetDirectLightSource(par_direct, par_diffuse)  # must call AFTER SetSunDirection; sets skylight_intensity=par_direct
pda.IlluminationEstimationOnSorghum()

# Result retrieval
results = pda.GetAllIlluminationEstimationResultsOnSorghum()
# results[entity_idx] = list of 5 Vec3:
#   [0] world position    [1] euler rotation    [2] total_area (x=y=z=scalar)
#   [3] total_flux        [4] average_flux  ← .y is primary PAR value
#
# Entities include container/generator entities that emit NaN — filter them:
# valid = [r for r in results if not any(math.isnan(c) for v in r for c in v)]

# Sensor pipeline (horizontal flat-plate probes for open-sky reference)
sensor_handle = pda.SetPARSensors(sorghum_field_entity)
pda.IlluminationEstimationOnSensors(sensor_handle)
sensor_results = pda.GetAllIlluminationEstimationResultsFromSensors(sensor_handle)
# sensor_results[i] = [position (Vec3), energy (Vec3), dominant_direction (Vec3)]
# energy.y is the PAR value — same .y channel, same correction applies
```

### PAR Calibration Physics (`SetDirectLightSource`)

`CalculateEnvironmentalLight` in `EvoEngine_Plugins/CudaModule/include/RayTracer/Environment.cuh`:
- **`SingleLightSource` mode**: returns `color × skylight_intensity` uniformly for every miss ray. With `color=(1,1,1)` and `skylight_intensity=PAR_direct`, every sky-reaching ray returns exactly `PAR_direct`.
- **`Skydome` mode**: returns `NishitaSkyIncidentLight(...) × skylight_intensity`. The Nishita model contains an arbitrary `× 20.0f` constant (line 153, comment: "magic number") and outputs ~0.002–0.05 in internal units — not physically calibrated.
- **`EnvironmentProperties::gamma` defaults to `1.0f`** (`OptiXRayTracer.hpp` line 107) — the `pow(..., 1/gamma)` call at line 179 of `Environment.cuh` is a no-op.

`IlluminationEstimation.cu` accumulates energy as:
```
probe.energy = (1/N) Σ L(ωᵢ) |n·ωᵢ|
```
For uniform `L = PAR_direct` (SingleLightSource, open sky): `probe.energy = PAR_direct × 0.5`.
**Multiply by 2.0** to recover `PARa = PAR_direct × sky_view_factor` in µmol m⁻² s⁻¹.

The physical units live entirely in `skylight_intensity`. The CUDA pipeline is dimensionless.

### Key Source Files

| File | Purpose |
|------|---------|
| `EvoEngine_Plugins/CudaModule/include/RayTracer/Environment.cuh` | `CalculateEnvironmentalLight`, `NishitaSkyIncidentLight`, `SingleLightSource` branch |
| `EvoEngine_Plugins/CudaModule/src/ptx/IlluminationEstimation.cu` | OptiX raygen kernel; `probe.energy = pointEnergy / sampleSize` |
| `EvoEngine_Plugins/CudaModule/include/RayTracer/OptiXRayTracer.hpp` | `EnvironmentProperties` struct (`skylight_intensity`, `ambient_light_intensity`, `gamma`, `sun_direction`) |
| `EvoEngine_Plugins/DigitalAgriculture/include/SorghumModel/PARSensorGroup.hpp` | Horizontal flat-plate sensor asset |
| `EvoEngine_Plugins/DigitalAgriculture/src/PARSensorGroup.cpp` | `CalculateIllumination` → `CudaModule::EstimateIlluminationRayTracing` |
| `PythonBinding/src/PyDigitalAgriculture.cpp` | All Python-facing implementations; `SetDirectLightSource` added on `claude/par-calibration` |
| `PythonBinding/include/PyDigitalAgriculture.hpp` | Declarations including `SetDirectLightSource` |
