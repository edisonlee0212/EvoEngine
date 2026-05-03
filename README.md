# EvoEngine

![Windows Release](https://github.com/edisonlee0212/EvoEngine/actions/workflows/Windows-RelWithDebInfo.yml/badge.svg)
![Windows Debug](https://github.com/edisonlee0212/EvoEngine/actions/workflows/Windows-Debug.yml/badge.svg)
![Linux Release](https://github.com/edisonlee0212/EvoEngine/actions/workflows/Linux-RelWithDebInfo.yml/badge.svg)
![Linux Debug](https://github.com/edisonlee0212/EvoEngine/actions/workflows/Linux-Debug.yml/badge.svg)

EvoEngine is a C++17 research framework for interactive simulation, digital forestry, digital agriculture, synthetic dataset generation, and Vulkan rendering. The repository is built around a general-purpose SDK and a set of domain plugins. The SDK provides the application runtime, editor, ECS, renderer, asset system, serialization, and automation hooks; plugins add research workflows on top.

Windows is the primary development platform. Linux builds are supported for the core stack, while several plugins are Windows-only or require optional SDKs.

![EvoEngine rendering demo](Resources/GitHub/RenderingDemo.png)

## 1. EvoEngine SDK

The SDK is the foundation of the framework. It lives in `EvoEngine_SDK` and is responsible for the reusable engine/runtime systems that plugins and applications build on.

### SDK Responsibilities

The SDK provides:

- application lifecycle and layer composition
- scene and entity management
- a hybrid ECS with data components and private components
- systems, transforms, hierarchy, prefabs, and scene cloning
- project, folder, file, asset, and metadata management
- YAML-based serialization and type registration
- an ImGui editor layer for scene, entity, asset, console, and inspector workflows
- Vulkan platform setup and rendering infrastructure
- global geometry and texture storage
- material, mesh, camera, light, render texture, and post-processing assets/components
- job scheduling, input events, and frame/fixed-step timing
- resource copying and shader include registration support for plugins

### Repository Layout

| Path | Purpose |
| --- | --- |
| `EvoEngine_SDK` | Core runtime, ECS, editor, renderer, assets, serialization, jobs, input, and utilities. |
| `EvoEngine_Plugins` | Domain modules that extend the SDK. |
| `EvoEngine_App` | Executable apps that choose which SDK layers and plugins to run. |
| `PythonBinding` | pybind11 modules for scripted workflows. |
| `Resources` | Demo projects, screenshots, textures, scripts, and build helpers. |
| `Extern` | Vendored third-party libraries and submodules. |
| `cmake` | CMake helper modules. |

### Application Model

An EvoEngine app is assembled by pushing layers before initialization. A typical interactive app uses:

- `RenderLayer` for Vulkan rendering, render instance preparation, and external render callbacks.
- `WindowLayer` for GLFW windows, input callbacks, resize handling, and presentation.
- `EditorLayer` for ImGui tools, scene views, entity hierarchy, inspectors, asset browser, and console.
- plugin layers such as `EcoSysLabLayer`, `SorghumLayer`, or `UniverseLayer`.

The main loop runs in phases: input/platform update, project update, transform graph calculation, fixed update, scene update, render preparation, late update, render execution, and window presentation. Editor play mode clones the start scene for runtime simulation, then restores the project scene when playback stops.

### ECS and Scene Model

EvoEngine uses two complementary component types:

| Component type | Use for |
| --- | --- |
| Data components | Plain standard-layout structs stored by archetype/chunk. Use them for high-volume simulation and parallel `Scene::ForEach` iteration. |
| Private components | Object-style components with lifecycle hooks, serialization, asset references, editor inspection, and per-entity behavior. |

Scenes own entity metadata, hierarchy, data component storage, private component storage, systems, environment state, and the main camera reference. Scene APIs cover entity creation/destruction, parenting, enable/static/name state, data component access, private component access, systems, queries, cloning, serialization, and prefab conversion.

Use data components when memory layout and parallel iteration matter most. Use private components when the feature needs editor UI, lifecycle methods, polymorphism, serialized state, or asset references.

### Assets, Projects, and Serialization

Assets are handle-based and are managed by the asset, file, and project managers. Projects use `.eveproj` files and asset sidecars such as `.evefilemeta` and `.evefoldermeta`. Asset references serialize by handle and type name, then resolve through the asset manager.

Persistent engine types must be registered with the serialization system. New persistent types usually need:

- a type registration such as `AssetRegistration`, `PrivateComponentRegistration`, `DataComponentRegistration`, or system registration
- `Serialize` and `Deserialize`
- `OnInspect` when editor editing is useful
- `CollectAssetRef` and `Relink` when the type stores `AssetRef`, `EntityRef`, or component/entity handles

### Rendering

The SDK renderer is Vulkan-based and centered on `RenderLayer`. The renderer includes deferred and forward paths, shadow maps, PBR materials, environment lighting, skyboxes, render textures, editor cameras, gizmos, mesh/skinned/instanced/particle/strand draw paths, and optional meshlet, indirect, and ray tracing support where available.

Scene components describe rendering intent. Render instance storage converts scene state into GPU-friendly material, instance, camera, light, and environment buffers. Geometry and texture storage keep mesh and texture resources globally available to render passes.

Plugins can extend rendering through `RenderLayer` callbacks for shadow maps, deferred rendering, forward rendering, and custom render instance registration.

### Jobs, Input, and Time

The SDK job system supports scheduled and immediate parallel work. ECS iteration helpers use jobs to process chunks in parallel. Input is routed from GLFW callbacks through the engine input system and layer event hooks. Timing utilities track frame delta time, fixed timestep state, and update counters.

### Applications

| Target | Purpose |
| --- | --- |
| `DemoApp` | General renderer/framework demo with multiple plugin registrations. |
| `EcoSysLabApp` | Interactive digital forestry and ecosystem workflow. |
| `DigitalAgricultureApp` | Interactive sorghum and agriculture workflow. |
| `LogGradingApp` | Log grading workflow, available when Windows-only plugins are enabled. |
| `TreeDataGeneratorApp` | Batch-oriented tree dataset generation. |
| `SorghumDataGeneratorApp` | Batch-oriented sorghum dataset generation. |
| `EmptyApp` | Minimal SDK app with render/window/editor layers for quick experiments. |

### Python Bindings

`PythonBinding` builds pybind11 modules for automation:

- `PyEcoSysLab`
- `PyDigitalAgriculture`

These modules expose selected SDK/plugin workflows for scripted tree and sorghum generation. Example scripts live in `PythonBinding`.

### Build Requirements

Clone with submodules:

```bash
git clone --recursive https://github.com/edisonlee0212/EvoEngine.git
cd EvoEngine
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

Windows requirements:

- Visual Studio 2019 or 2022 with Desktop development with C++
- CMake and Ninja
- Vulkan SDK
- vcpkg path recorded in `%LOCALAPPDATA%\vcpkg\vcpkg.path.txt`
- a Visual Studio developer command prompt

Windows build:

```bat
build.cmd Release
```

Linux requirements:

- `clang-14`
- `cmake`
- `ninja-build`
- `libwayland-dev`
- `libxkbcommon-dev`
- `xorg-dev`
- Vulkan SDK from LunarG
- Python development headers, currently expected around Python 3.12 for the provided setup

Linux build:

```bash
bash build.sh Release
```

Useful script options:

```bash
--clean
--verbose
--no-test
Debug
Release
```

Build outputs are generated under `out/build/<platform>-<config>/`. App binaries are produced under the `EvoEngine_App` build directory and Python modules are produced under the `PythonBinding` build directory. Post-build steps still copy engine resources, plugin resources, DLLs, PDBs, and `imgui.ini` beside those build-tree binaries for fast local development.

CMake install provides a cleaner runtime deployment tree:

```text
out/install/x64-Release/bin/
out/install/x64-Release/python/
```

The `bin` folder contains installed app executables plus their runtime DLLs/PDBs/resources. The `python` folder contains installed `.pyd` modules, Python scripts, and the same runtime DLL/resource payload needed for imports and scripted workflows.

### VSCode Build

VSCode should use `CMakePresets.json` through the CMake Tools extension. Select one of the configure presets:

- `vs2026-x64-Release` for the Visual Studio generator. This is the easiest option from a normal VSCode window because CMake can discover MSVC and the Windows SDK.
- `x64-Release` for the Ninja generator. Use this only when VSCode has selected a Visual Studio kit or the terminal already has the MSVC developer environment loaded.

Then select a build preset such as:

- `vs2026-x64-Release` to build all Visual Studio generator targets
- `EmptyApp-vs2026-x64-Release` to build only the minimal app with the Visual Studio generator
- `x64-Release` to build all targets
- `DemoApp-Release`
- `EcoSysLabApp-Release`
- `DigitalAgricultureApp-Release`
- `EmptyApp-Release`

The presets use the Ninja generator and write app executables to:

```text
out/build/x64-Debug/EvoEngine_App/
out/build/x64-Release/EvoEngine_App/
```

To deploy a runnable install tree from VSCode/CMake Tools, select `install-x64-Release` after building, or run:

```bat
cmake --preset x64-Release
cmake --build --preset x64-Release
cmake --build --preset install-x64-Release
```

The equivalent direct CMake install command is `cmake --install out/build/x64-Release`.

For the Visual Studio generator preset, use:

```bat
cmake --preset vs2026-x64-Release
cmake --build --preset vs2026-x64-Release
cmake --build --preset install-vs2026-x64-Release
```

After install, app executables are under:

```text
out/install/x64-Release/bin/
out/install/vs2026-x64/bin/
```

Python bindings and scripts are under:

```text
out/install/x64-Release/python/
out/install/vs2026-x64/python/
```

If CMake is configured with a Visual Studio generator instead, it will create `.sln` and `.vcxproj` files. Those files are project files, not final executables. To produce `.exe` files from that generator, the generated solution still needs to be built with Visual Studio, MSBuild, or `cmake --build <build-dir> --config Debug`.

### SDK Extension Guide

When adding new work:

- Add a private component when behavior belongs to an entity and needs lifecycle hooks, editor UI, serialization, or asset references.
- Add data components when the feature is high-volume and benefits from chunked ECS iteration.
- Add a system when behavior should run over a scene independently of one component instance.
- Add an asset when data should be reusable, referenceable, and stored in projects.
- Add a layer when behavior is global to the application or needs top-level UI/render/input hooks.
- Add a plugin when the feature is domain-specific and should remain outside the SDK.
- Add a Python binding when a workflow should run from scripts.

### License

This repository is licensed under the Creative Commons Attribution-NonCommercial 4.0 International license. See `LICENSE` for the full text.

## 2. Plugin Documentation

Plugin documentation is split into separate Markdown files so each module can grow independently without turning the README into a wall of details.

| Plugin | Status | Documentation |
| --- | --- | --- |
| EcoSysLab | Enabled by default | [EvoEngine_Plugins/EcoSysLab/README.md](EvoEngine_Plugins/EcoSysLab/README.md) |
| DigitalAgriculture | Enabled by default | [EvoEngine_Plugins/DigitalAgriculture/README.md](EvoEngine_Plugins/DigitalAgriculture/README.md) |
| DatasetGeneration | Enabled by default | [EvoEngine_Plugins/DatasetGeneration/README.md](EvoEngine_Plugins/DatasetGeneration/README.md) |
| Universe | Enabled by default | [EvoEngine_Plugins/Universe/README.md](EvoEngine_Plugins/Universe/README.md) |
| BillboardClouds | Windows-only registration by default | [EvoEngine_Plugins/BillboardClouds/README.md](EvoEngine_Plugins/BillboardClouds/README.md) |
| TextureBaking | Windows-only registration by default | [EvoEngine_Plugins/TextureBaking/README.md](EvoEngine_Plugins/TextureBaking/README.md) |
| MeshRepair | Windows-only registration by default | [EvoEngine_Plugins/MeshRepair/README.md](EvoEngine_Plugins/MeshRepair/README.md) |
| Gpr | Windows-only registration by default | [EvoEngine_Plugins/Gpr/README.md](EvoEngine_Plugins/Gpr/README.md) |
| LogGrading | Windows-only registration by default | [EvoEngine_Plugins/LogGrading/README.md](EvoEngine_Plugins/LogGrading/README.md) |
| LogScanning | Windows-only registration by default | [EvoEngine_Plugins/LogScanning/README.md](EvoEngine_Plugins/LogScanning/README.md) |
| CudaModule | Present but not registered by default | [EvoEngine_Plugins/CudaModule/README.md](EvoEngine_Plugins/CudaModule/README.md) |
| PhysXPhysics | Present but disabled in its CMake file | [EvoEngine_Plugins/PhysXPhysics/README.md](EvoEngine_Plugins/PhysXPhysics/README.md) |

The plugin index is also available at [EvoEngine_Plugins/README.md](EvoEngine_Plugins/README.md).

### Plugin Build Model

Plugins are registered from `EvoEngine_Plugins/CMakeLists.txt`. The registration macro creates an `EVOENGINE_ENABLE_<PluginName>_PLUGIN` option, adds the plugin subdirectory, and appends the plugin target, include paths, compile definitions, precompiled headers, copied resources, and DLLs to the shared EvoEngine build variables.

The common pattern is:

- plugin source lives under `EvoEngine_Plugins/<PluginName>/include` and `src`
- plugin target is a static library named `<PluginName>Plugin`
- plugin compile definition is usually `<PLUGIN_NAME>_PLUGIN`
- plugin resources may be copied from an `Internals` folder
- app targets link against the enabled plugin list

### Demo Projects and Visual Results

Demo projects and visual assets live under `Resources`.

| Area | Preview |
| --- | --- |
| Rasterized rendering | ![Rendering demo](Resources/GitHub/RenderingDemo.png) |
| Ray tracing | ![Ray tracing demo](Resources/GitHub/RayTracingDemo.png) |
| Planet terrain | ![Planet terrain demo](Resources/GitHub/PlanetsDemo.png) |
| Star clusters | ![Star cluster demo](Resources/GitHub/StarClusterDemo.png) |
| Tree framework | ![Tree framework demo](Resources/GitHub/TreeFrameworkDemo.png) |
| Tree fracture | ![Tree fracture demo](Resources/GitHub/TreeFracture.png) |
| Strand visualization | ![Strand visualization](Resources/GitHub/StrandVisualization.png) |
| Sorghum model | ![Sorghum model](Resources/GitHub/SorghumModel.png) |
| Sorghum point cloud | ![Sorghum point cloud](Resources/GitHub/SorghumPointCloud.png) |
| Sorghum environment lighting | ![Sorghum environment lighting](Resources/GitHub/SorghumEnvLighting.png) |
| Illumination estimation | ![Illumination estimation demo](Resources/GitHub/IlluminationEstimationDemo.png) |

### Related Publications

EvoEngine supports research workflows used in digital forestry and digital agriculture. Related work includes:

- [Learning to Reconstruct Botanical Trees from Single Images, SIGGRAPH Asia 2021](https://storage.googleapis.com/pirk.io/projects/single_tree_reconstruction/index.html)
- [Rhizomorph: The Coordinated Function of Shoots and Roots, SIGGRAPH 2023](https://storage.googleapis.com/pirk.io/projects/rhizomorph/index.html)
- [DeepTree: Modeling Trees with Situated Latents, TVCG 2023](https://storage.googleapis.com/pirk.io/projects/deep_tree/index.html)
- [Latent L-systems: Transformer-based Tree Generator, SIGGRAPH 2024](https://dl.acm.org/doi/pdf/10.1145/3627101)
- [Tree-D Fusion: Simulation-Ready Tree Dataset from Single Images with Diffusion Priors, ECCV 2024](https://link.springer.com/chapter/10.1007/978-3-031-72940-9_25)
- [Interactive Invigoration: Volumetric Modeling of Trees with Strands, SIGGRAPH 2024](https://storage.googleapis.com/pirk.io/projects/invigoration/index.html)
- [TreeStructor: Forest Reconstruction With Neural Ranking, TGRS 2025](https://lewkesy.github.io/treestructor/)
- [Stressful Tree Modeling: Breaking Branches with Strands, SIGGRAPH 2025](https://dl.acm.org/doi/10.1145/3721238.3730745)
- [3D reconstruction identifies loci linked to variation in angle of individual sorghum leaves, PeerJ](https://peerj.com/articles/12628/)
- [Sorghum segmentation and leaf counting using in silico trained deep neural model, The Plant Phenome Journal](https://acsess.onlinelibrary.wiley.com/doi/pdf/10.1002/ppj2.70002)
- [PlantSegNet: 3D point cloud instance segmentation of nearby plant organs with identical semantics, Computers and Electronics in Agriculture](https://www.sciencedirect.com/science/article/abs/pii/S0168169924003132)
