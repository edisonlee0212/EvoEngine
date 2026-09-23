# Runtime components and editor extensions

Keep runtime data and behavior in `include/` and `src/`. Put inspectors, thumbnails, authoring tools and their temporary state in `Editor/include/` and `Editor/src/`. The editor target links the runtime target; the reverse dependency is rejected by the build.

## Add a component

Follow an existing runtime package such as TextureBaking. Derive the component from `IPrivateComponent`, declare its runtime dependencies, and register its type and serialization handlers through the runtime package entrypoints. Keep serialized parameters on the component even when an inspector edits them. Preserve existing type names and fields when migrating saved content.

Do not add `OnInspect`, empty GUI overrides, editor handles or `EVOENGINE_WITH_EDITOR` guards to the component. A component works without an inspector. Existing runtime headers may rely on the runtime SDK's common includes; an editor PCH is never a runtime dependency.

## Add an inspector

Declare the inspector in the package's editor headers and implement it in its editor sources:

```cpp
bool InspectExample(evo_engine::InspectorContext& context, Example& component) {
  return ImGui::DragFloat("Speed", &component.speed);
}
```

Register it from `EvoEngineEditorPackageLoad`:

```cpp
return registrar && registrar->RegisterInspector<Example>(InspectExample, "Example");
```

The boolean reports whether inspection changed the object. Match the registered runtime type name. Use the companion registrar for inspectors, asset preview handlers, editor layers and cleanup; it owns the registrations and keeps callback code mapped through destruction. Do not register duplicate runtime types from the companion.

Capture transient panel state in the inspector callback, or keep per-object state in an editor layer keyed by weak object ownership. Prune expired objects and clear scene-specific state on scene changes. Do not keep inspected assets/components alive in static maps. Release editor cameras, preview textures and pending work during layer destruction or registered cleanup. Package mutation during callbacks must be deferred until the application is stopped and outside frame dispatch.

## Add a companion target

The runtime package CMake file calls `evoengine_configure_runtime_package(ExamplePackage)` and conditionally adds `Editor` when `EVOENGINE_WITH_EDITOR` is enabled. In `Editor/CMakeLists.txt`:

```cmake
file(GLOB_RECURSE editor_sources CONFIGURE_DEPENDS "src/*.cpp" "include/*.hpp")
add_library(ExampleEditorPackage SHARED ${editor_sources})
evoengine_configure_editor_package(ExampleEditorPackage ExamplePackage)
```

Use TextureBaking's editor entrypoint as the minimal descriptor/load/unload example. The helper supplies generated runtime-module and editor identity headers, the editor SDK, PCH, deployment, PDB and `.eveeditorpackage` manifest rules. No component-level build macros are needed.

If editor implementation calls another companion's exported helpers, declare `DEPENDS OtherPackage` on `evoengine_configure_editor_package`. That package must also be a declared runtime dependency. Export its shared editor helpers explicitly or through the target's Windows export settings. Merely using another package's runtime API does not require its companion. EcoSysLab's BillboardClouds dependency demonstrates this distinction.

The editor validates declared companions before making a package ready. Runtime-only packages may omit a companion. A declared missing or incompatible companion blocks activation. Loaded dependents, active callbacks and retained runtime/editor objects can refuse unload; release them and retry through Package Manager.

## Resources and compatibility

Runtime resources belong to the runtime target's resource roots. Put editor-only resources under `Editor/Internals`, retaining their intended deployed relative paths. The editor helper copies and installs that root. Payload generation receives only runtime resource roots; it does not maintain filename exclusion lists. Avoid collisions between the two resource sets.

First-party editor edits change editor identities without invalidating runtime fingerprints. Runtime changes invalidate the corresponding runtime and editor identities. Native package API 3 and companion API 2 require rebuilding older binaries; serialized project/asset formats are unchanged. Third-party dependency provenance remains conservatively shared.

## Boundary checks

`EvoEngineRuntimeBoundary` runs before SDK, service and package builds, with or without editor composition. CMake generates the configuration-resolved link/include/source/PCH graph. The checker follows interface libraries, imported references and aliases, rejects editor targets/paths, and follows first-party includes, including disabled preprocessor branches. It also rejects editor GUI types and per-class editor guards in runtime sources. First-party runtime includes must use literal paths so macros cannot hide their dependencies. Editor include directories are absent from runtime target interfaces.

Run the negative fixtures with `python -m unittest Scripts.tests.test_runtime_boundary`. They exercise indirect includes, guarded includes, shared headers resolved through different runtime include paths, configuration-specific alias links and imported editor PCHs. CI runs these fixtures and invokes the live graph check through ordinary native builds. With testing enabled, `EvoEngineRuntimeHeaderCheck` compiles representative runtime SDK and package headers with PCH disabled. This is representative coverage, not a claim that every legacy public header is independently self-contained.

The remaining `EVOENGINE_WITH_EDITOR` uses select app/target and editor-resource composition. The macro has no default definition in public headers, is not propagated into shared runtime targets, and does not change runtime class layouts.
