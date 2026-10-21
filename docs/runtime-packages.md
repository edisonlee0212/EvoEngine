# Runtime Packages

[Back to README](../README.md)

EvoEngine separates build-time Services from runtime packages.

| Extension type | Folder | Build/runtime model | Use for |
| --- | --- | --- | --- |
| Service | `EvoEngine_Services/<Name>` | Static library selected by CMake options such as `EVOENGINE_ENABLE_PhysXPhysics_SERVICE` | Build-time modules that apps or packages link against directly. |
| Runtime package | `EvoEngine_Packages/<Name>` | DLL/shared library selected by CMake options such as `EVOENGINE_ENABLE_<Name>_PACKAGE` and loaded from a `Packages` runtime folder | Domain features that can be rebuilt, loaded, unloaded, or reloaded independently from the app. |

Package documentation is indexed in [EvoEngine_Packages/README.md](../EvoEngine_Packages/README.md). Service documentation is indexed in [EvoEngine_Services/README.md](../EvoEngine_Services/README.md).

## Build Model

Runtime packages live under `EvoEngine_Packages`. The package CMake entry scans package folders automatically, reads optional metadata from `PackageInfo.cmake`, creates an `EVOENGINE_ENABLE_<Name>_PACKAGE` option, and builds a shared library target named `<Name>Package` by default.

Disable individual package targets with:

```bash
-DEVOENGINE_ENABLE_<Name>_PACKAGE=OFF
```

When `EVOENGINE_ENABLE_RUNTIME_PACKAGES` is `ON`, the SDK builds as a shared library so apps, services, Python bindings, and runtime packages share the same runtime registries.

Package dependencies are declared with `EVOENGINE_PACKAGE_DEPENDS` in `PackageInfo.cmake`. CMake builds dependencies first and emits a sidecar `.evepackage` manifest beside each package binary so the runtime can discover and load dependencies before opening a package library.

## Loading Packages

The SDK initialization default does not enable runtime packages. The generic editor enables the packages named in
project metadata or a demo profile; exported players load the exact package set recorded by the exporter. Custom hosts
can enable loading by:

- setting `ApplicationInitializationSettings::enable_runtime_packages = true`
- adding names to `ApplicationInitializationSettings::startup_runtime_packages`
- declaring `startup_runtime_packages` in a project `.eveproj`
- calling `PackageManager::Load` or `PackageManager::LoadAll`
- using editor runtime package tooling

Runtime packages export:

```cpp
EvoEnginePackageGetDescriptor
EvoEnginePackageLoad
EvoEnginePackageUnload
```

Packages that own RTTI/reflection types should also export:

```cpp
EvoEnginePackageRegisterTypes
```

Use `EvoEnginePackageRegisterTypes` and `PackageRegistrar` to register package-owned types:

```cpp
registrar.RegisterPrivateComponent<MyComponent>("MyComponent");
registrar.RegisterAsset<MyAsset>("MyAsset", {".myasset"});
registrar.RegisterDataComponent<MyData>("MyData");
registrar.RegisterSystem<MySystem>("MySystem");
registrar.RegisterLayer<MyLayer>("My Layer");
```

Register serialized types and their handlers during `EvoEnginePackageRegisterTypes`. Runtime layers use the same
`PackageRegistrar`; first-party packages also register layers in that callback. Keep inspectors, asset previews and
authoring layers in a separate `Editor/` companion. The
companion exports `EvoEngineEditorPackageGetDescriptor`, `EvoEngineEditorPackageLoad` and `EvoEngineEditorPackageUnload`.
See the [companion workflow](editor-extension-workflow.md) for CMake and inspector registration.

Runtime package API 3 and editor companion API 2 validate compiler/configuration, SDK and source identity before
activation. A declared missing or incompatible companion blocks editor activation; runtime exports omit companions.
Use freshly rebuilt binaries and manifests together, rather than copying a DLL from a different configuration.

## Reloading And Unloading

Package unloading is guarded. Load, reload, and unload operations are refused while the app is playing, paused, or stepping. Reload and unload are also refused while package-owned private component instances, package-created objects, or dependent runtime packages still exist.

On Windows, packages are loaded from a shadow copy, so the original DLL can usually be rebuilt while the app process remains open.
Open **View > Runtime Packages**. For build-tree iteration, click `Build`, stop playback, release package-owned objects
and loaded dependents, then click `Reload`. Shadow copying does not make old live objects compatible with a new DLL.
