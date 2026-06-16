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

Build one target from the build tree:

```bat
cmake --build out/build/vs2026-x64 --config RelWithDebInfo --target EvoEngineEditor
```

Rebuild one runtime package:

```bat
cmake --build out/build/vs2026-x64 --config RelWithDebInfo --target EcoSysLabPackage
cmake --build out/build/vs2026-x64 --config Debug --target DigitalAgriculturePackage
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

Post-build steps also copy engine resources, Service resources, runtime libraries, PDBs when available, runtime packages, and `imgui.ini` beside build-tree binaries for fast local development.

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

```bat
python Scripts\test.py
python Scripts\test.py --all
python Scripts\format_cpp.py
python Scripts\format_cpp.py --check
```
