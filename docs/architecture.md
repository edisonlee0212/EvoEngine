# EvoEngine Architecture

[Back to README](../README.md)

## SDK Responsibilities

The SDK lives in `EvoEngine_SDK` and provides the reusable engine/runtime systems that Services, packages, apps, and Python bindings build on:

- application lifecycle and layer composition
- scene and entity management
- hybrid ECS with data components and private components
- systems, transforms, hierarchy, prefabs, and scene cloning
- project, folder, file, asset, and metadata management
- YAML-based serialization and type registration
- ImGui editor layer for scene, entity, asset, console, and inspector workflows
- Vulkan platform setup and rendering infrastructure
- global geometry and texture storage
- material, mesh, camera, light, render texture, and post-processing assets/components
- job scheduling, input events, and frame/fixed-step timing
- resource copying and shader include registration support for Services
- runtime package loading, guarded unloading/reloading, and package-owned type registration

## Application Model

An EvoEngine app is assembled by pushing layers before initialization. A typical interactive app uses:

- `RenderLayer` for Vulkan rendering, render instance preparation, and external render callbacks
- `WindowLayer` for GLFW windows, input callbacks, resize handling, and presentation
- `ImGuiLayer` for editor UI integration
- `EditorLayer` for scene views, entity hierarchy, inspectors, asset browser, console, and editor tools
- optional runtime package layers such as EcoSysLab, DigitalAgriculture, Universe, or other package-owned layers

The main loop runs in phases: input/platform update, project update, transform graph calculation, fixed update, scene update, render preparation, late update, render execution, and window presentation. Editor play mode clones the start scene for runtime simulation, then restores the project scene when playback stops.

## ECS And Scene Model

EvoEngine uses two complementary component types:

| Component type | Use for |
| --- | --- |
| Data components | Plain standard-layout structs stored by archetype/chunk. Use them for high-volume simulation and parallel `Scene::ForEach` iteration. |
| Private components | Object-style components with lifecycle hooks, serialization, asset references, editor inspection, and per-entity behavior. |

Scenes own entity metadata, hierarchy, data component storage, private component storage, systems, environment state, and the main camera reference. Scene APIs cover entity creation/destruction, parenting, enable/static/name state, data component access, private component access, systems, queries, cloning, serialization, and prefab conversion.

Use data components when memory layout and parallel iteration matter most. Use private components when the feature needs editor UI, lifecycle methods, polymorphism, serialized state, or asset references.

## Rendering

The SDK renderer is Vulkan-based and centered on `RenderLayer`. It includes deferred and forward paths, shadow maps, PBR materials, environment lighting, skyboxes, render textures, editor cameras, gizmos, mesh/skinned/instanced/particle/strand draw paths, and optional meshlet, indirect, and ray tracing support where available.

Scene components describe rendering intent. Render instance storage converts scene state into GPU-friendly material, instance, camera, light, and environment buffers. Geometry and texture storage keep mesh and texture resources globally available to render passes.

Services and packages can extend rendering through `RenderLayer` callbacks for shadow maps, deferred rendering, forward rendering, and custom render instance registration.

## Jobs, Input, And Time

The SDK job system supports scheduled and immediate parallel work. ECS iteration helpers use jobs to process chunks in parallel. The runtime owns a general worker pool plus named service executors for main-thread callbacks, asset IO, GPU resource work, future rendering work, and background tasks. `JobSystem` and `Jobs` remain compatibility facades over this engine-owned `TaskRuntime` boundary.

GPU upload/finalization work that is not full frame rendering is routed through the platform-owned `GpuService` instance on a dedicated GPU executor. `Platform::ImmediateSubmit` forwards into that service, and buffers expose async upload/readback entry points for callers that can keep CPU work moving while GPU transfer work completes.

The application frame loop, `RenderLayer::RenderAll`, window rendering, and presentation still run on the main thread. The render executor is reserved for future render-thread work.

Input is routed from GLFW callbacks through the engine input system and layer event hooks. Timing utilities track frame delta time, fixed timestep state, and update counters.

## Applications

| Target | Purpose |
| --- | --- |
| `EvoEngineLauncher` | Project launcher for opening or creating projects before starting the editor. |
| `EvoEngineEditor` | Generic project-required editor shell launched with a `.eveproj` path. |
| `DemoApp` | General renderer/framework demo with multiple Service/package registrations. |
| `EcoSysLabApp` | Interactive digital forestry and ecosystem workflow. |
| `DigitalAgricultureApp` | Interactive sorghum and agriculture workflow. |
