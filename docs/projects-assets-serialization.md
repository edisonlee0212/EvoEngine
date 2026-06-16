# Projects, Assets, And Serialization

[Back to README](../README.md)

## Projects

Projects use `.eveproj` files. They can carry launch metadata used by `EvoEngineLauncher` and `EvoEngineEditor`:

- `application_name`
- `preferred_editor`
- `startup_runtime_packages`

The generic editor reads package names from this metadata before project assets load, so package-backed projects can be opened from the launcher without hardcoding an app-specific entry point. Launcher-created projects write this metadata before the editor starts.

## Assets And Metadata

Assets are handle-based and managed by the asset, file, and project managers. Projects use asset sidecars such as:

- `.evefilemeta`
- `.evefoldermeta`

Asset references serialize by handle and type name, then resolve through the asset manager.

Interactive project asset loading dispatches scanned assets as an `AssetManager` batch instead of resolving one pending asset per frame. The asset-service path owns per-handle in-flight state, progress snapshots, and blocking sync access, so concurrent requests wait for the first loader instead of creating duplicate asset instances.

Assets can opt into staged async loading by producing a CPU-only payload on the asset-IO executor and applying it later on the bounded main-thread finalization lane. GPU-backed staged assets can also register GPU readiness work; those loads remain `GpuPending` until the tracked GPU jobs complete, and sync access blocks until the asset is fully ready.

`Texture2D` uses staged loading so pixel decoding is separated from asynchronous GPU upload. Runtime texture data updates use the same GPU-service-backed upload path while readback/export waits for pending texture GPU work.

SDK staged assets also include shader/JSON source assets, scene and native prefab YAML, strands, point clouds, and SDK YAML-backed render assets that deserialize through the default `IAsset` path. Legacy non-staged paths, especially imported model prefab formats that still run through Assimp and create engine objects during import, continue to finalize on the main thread.

Headless tools and focused tests that only need project scanning or asset metadata can set:

- `ApplicationInitializationSettings::load_default_resources = false`
- `ApplicationInitializationSettings::load_project_start_scene = false`
- `ApplicationInitializationSettings::load_project_assets = false`

## Serialization

Persistent engine types must be registered with the serialization system. New persistent types usually need:

- a type registration such as `AssetRegistration`, `PrivateComponentRegistration`, `DataComponentRegistration`, or system registration
- concrete `Serialize` and `Deserialize` methods, or explicit serialization registry handlers
- a concrete `OnInspect` method or explicit inspector registry handler when editor editing is useful
- concrete asset-reference collection and relink methods, or explicit support handlers, when the type stores `AssetRef`, `EntityRef`, or component/entity handles
