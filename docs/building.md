# Building EvoEngine

[Back to README](../README.md)

## Clone With Submodules

```bash
git clone --recursive https://github.com/edisonlee0212/EvoEngine.git
cd EvoEngine
```

If needed:

```bash
git submodule update --init --recursive
```

## Windows Requirements

- Visual Studio 2026 with Desktop development with C++
- CMake
- Vulkan SDK

Generate the Visual Studio build tree:

```bat
python Scripts\build_project.py
```

Build and install runnable app binaries:

```bat
python Scripts\install_apps.py
python Scripts\install_apps.py --config Debug
```

Incremental installs record their configuration in the install directory. Reusing the same configuration preserves the
installed runtime, while switching between Debug and RelWithDebInfo cleans it first so incompatible MSVC runtimes are
not mixed.

The default app install builds the launcher/editor flow and leaves standalone demo executables disabled. Legacy demo
targets can still be enabled explicitly when old automation needs them:

```bat
cmake --preset vs2026-x64 -DEvoEngine_App-DemoApp=ON -DEvoEngine_App-DDGIApp=ON
cmake --build out/build/vs2026-x64 --config RelWithDebInfo --target DemoApp
```

Build one target from the build tree:

```bat
cmake --build out/build/vs2026-x64 --config RelWithDebInfo --target EvoEngineEditor
```

### SDK Build Profiling

Use the same MSBuild scheduling settings as `Scripts/install_apps.py` when comparing SDK build changes:

```bat
cmake --build out/build/vs2026-x64 --config RelWithDebInfo --target EvoEngine_SDK --parallel 8 -- /verbosity:minimal /clp:PerformanceSummary /p:UseMultiToolTask=true /p:EnforceProcessCountAcrossBuilds=true /p:MultiProcMaxCount=8
```

Record the `ClCompile`, `PreLinkEvent`, and `Link` entries separately from total wall time. A warm build verifies build-system overhead. For a representative one-file incremental build, update one SDK source timestamp, run the command, and restore the original timestamp so the benchmark does not modify source contents:

```powershell
$source = Get-Item EvoEngine_SDK/src/GpuProfiler.cpp
$timestamp = $source.LastWriteTimeUtc
try {
    $source.LastWriteTimeUtc = [DateTime]::UtcNow
    cmake --build out/build/vs2026-x64 --config RelWithDebInfo --target EvoEngine_SDK --parallel 8 -- /verbosity:minimal /clp:PerformanceSummary /p:UseMultiToolTask=true /p:EnforceProcessCountAcrossBuilds=true /p:MultiProcMaxCount=8
} finally {
    $source.LastWriteTimeUtc = $timestamp
}
```

The baseline measured on an AMD Ryzen 7 9800X3D with MSVC 19.51.36248.0 and CMake 4.3.2 was:

| Scenario | Compile | Pre-link | Link | Wall time |
| --- | ---: | ---: | ---: | ---: |
| Full SDK recompilation | 34.0 s | 113.7 s | 2.5 s | 151.4 s |
| One-file incremental | 2.1 s | 117.0 s | 2.5 s | 122.9 s |

After replacing the Windows export filter's per-line CMake loop with bulk filtering, the same build tree measured:

| Scenario | Compile | Pre-link | Link | Wall time |
| --- | ---: | ---: | ---: | ---: |
| Full SDK recompilation | 37.4 s | 1.1 s | 2.4 s | 42.2 s |
| One-file incremental | 1.2 s | 1.1 s | 2.4 s | 5.9 s |

The optimized filter produced a byte-identical 59,452-line export definition. Its standalone filtering time fell from
113.2 seconds to 0.9 seconds, so SDK relinks no longer spend most of their time processing the export list.

A CMake unity-build experiment with four SDK sources per batch reduced full compilation from 37.4 seconds to 31.7
seconds (15%) while increasing one-file wall time from 5.9 seconds to 6.4 seconds (8%). It was not adopted because the
clean compilation improvement did not meet the 25% acceptance threshold. Six C++ sources also required individual
compilation because their anonymous-namespace helpers or platform headers conflict when combined, while the C-only
MikkTSpace source could not share the C++ precompiled header.

The Windows runtime SDK now uses the source-level `EVOENGINE_API` contract instead of discovering exports from every
object file and generating a module definition file before each link. Public SDK types and free functions used by
repository apps, runtime packages, and Python bindings must carry `EVOENGINE_API`; new public API is not exported
implicitly. This intentionally does not preserve binary compatibility with SDK consumers built before the change.

On the same RelWithDebInfo build, the explicit interface reduced the DLL export table from 59,452 entries to 5,633
entries (90.5%). A no-op SDK build completed in 1.9 seconds. More importantly, SDK links no longer scan all SDK object
files or regenerate and filter an export definition, so the pre-link cost removed in the earlier filtering milestone is
eliminated rather than merely optimized.

These numbers are machine-specific. Compare optimization results on the same machine, build tree, configuration, target, and parallelism. One-file edit-to-link latency is the primary developer metric; clean compilation is secondary.

Rebuild one runtime package:

```bat
python EvoEngine_Packages\build_package.py EcoSysLab --config RelWithDebInfo --skip-configure
python EvoEngine_Packages\build_package.py DigitalAgriculture --config Debug --skip-configure
```

## Linux Requirements

- `clang-14`
- `cmake`
- `ninja-build`
- `libwayland-dev`
- `libxkbcommon-dev`
- `xorg-dev`
- Vulkan SDK from LunarG
- Python development headers, currently expected around Python 3.12 for the provided setup

Configure, build, and install:

```bash
cmake -S . -B out/build/linux-RelWithDebInfo -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=out/install/linux-RelWithDebInfo
cmake --build out/build/linux-RelWithDebInfo
cmake --install out/build/linux-RelWithDebInfo
```

Linux runtime libraries are deployed as `.so` files beside the installed apps and Python modules.

## Output Layout

Build outputs are generated under:

```text
out/build/<platform>-<config>/
```

Visual Studio app binaries are generated under:

```text
out/build/vs2026-x64/EvoEngine_App/<Config>/
```

CMake install writes a cleaner runtime deployment tree:

```text
out/install/vs2026-x64/bin/
out/install/vs2026-x64/bin/Packages/
out/install/vs2026-x64/python/
```

The `bin` folder contains installed app executables plus runtime libraries, PDBs when available, and resources. Runtime package libraries install under `bin/Packages`. The `python` folder contains installed Python extension modules, Python scripts, and the runtime library/resource payload needed for imports and scripted workflows.

Post-build steps also copy engine resources, Service resources, runtime libraries, PDBs when available, runtime packages, and `imgui.ini` beside build-tree binaries for fast local development. Targeted app and runtime package builds also refresh `out/install/vs2026-x64/bin`, so this install-bin folder is the stable local runtime location for manual testing even when you build individual targets instead of the full install preset.

## VSCode On Windows

VSCode should use `CMakePresets.json` through the CMake Tools extension.

Configure preset:

- `vs2026-x64`

Install build presets:

- `install-vs2026-x64-Debug`
- `install-vs2026-x64-RelWithDebInfo`

Select the configure preset, then choose one of the install build presets. They build the selected configuration and deploy the runtime payload to `out/install/vs2026-x64`.

If CMake is configured with a Visual Studio generator, it creates `.sln` and `.vcxproj` project files. Those files are not final executables. Build with Visual Studio, MSBuild, or:

```bat
cmake --build out/build/vs2026-x64 --config Debug
```

## Useful Commands

Local and CI formatting use the exact release recorded in `.clang-format-version`. Install it once with:

```bat
python -m pip install clang-format==22.1.8
```

```bat
python Scripts\test.py
python Scripts\test.py --all
python Scripts\format_cpp.py
python Scripts\format_cpp.py --check
```
