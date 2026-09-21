# Runtime builds

Build Manager exports a saved project as a self-contained Windows x64 application using prebuilt runtime binaries. The output runs without the editor and remains portable when its entire directory is moved.

Open **View > Build Manager** in the editor. Enter an application name and an absolute output directory, or use **Browse...** to choose a folder with the native Windows dialog. **Startup Scene** defaults to the active scene once the project is idle and playback is stopped, when no build scene has been selected; drag another saved scene from the Project browser to override it. Set the startup window size, display mode, resize permissions, and optional **Show Windows console**. **Build** automatically saves this configuration in the project's `.eveproj` before validation; choosing a build startup scene does not change the project's own startup scene.

Stop playback and save changed scenes/assets before selecting **Build**. Paused playback, pending asset work, a failed project settings save, unsaved scenes/assets, an invalid startup scene or main camera, incompatible binaries/packages, and an occupied destination block export. The destination must be empty or nonexistent and outside the source project. The exporter checks that source files remain unchanged while it copies them and publishes only a complete distribution. Success opens the output directory in Explorer without running the application.

The output contains the named executable, runtime SDK/dependencies, default resources, every project asset and sidecar, a sanitized project file under `Project`, `runtime.yaml`, and `build-report.json`. Only the currently loaded packages and their dependencies are copied and activated. Assets belonging to other providers are still copied without being activated. Files are loose; there is no asset packing. Optional [RuntimeGui components](runtime-gui.md) provide main-camera overlays. Export uses the running editor's exact native configuration, with PDBs included for RelWithDebInfo.

`EVOENGINE_WITH_EDITOR` defaults to `ON` and selects application composition. Runtime SDK/package targets always use their runtime sources and interfaces. Editor code and resources live under each SDK/package `Editor` directory; `EvoEngine_EditorSDK` and companion DLLs supply optional editor behavior. Runtime classes have no editor inspection hooks or empty compatibility implementations. Core ImGui and RuntimeGui are runtime features; editor inspection remains separate. See [the authoring workflow](editor-extension-workflow.md) and [separation design](editor-runtime-separation.md).

Use separate build and install directories for the two variants. Their libraries have the same filenames and must never be installed over each other or mixed in one process.

```powershell
cmake --preset vs2026-x64-runtime
cmake --build out/build/vs2026-x64-runtime --config RelWithDebInfo --target EvoEngineRuntimePayload
```

Editor applications, Python bindings and companion DLLs are omitted in this variant. Normal editor builds retain the default option. Add inspectors in `Editor/src` and register them through the editor companion; do not add editor guards or inspection methods to runtime classes. The build checks runtime include and link dependencies in both configurations.

Configure with `-DBUILD_TESTING=ON` to build `EvoEngineRuntimeBoundaryTests`. Its tests cover component registration, scene cloning, graph serialization, and a Vulkan window with main-camera rendering and keyboard dispatch. Run them with `ctest --test-dir out/build/runtime -C RelWithDebInfo --output-on-failure` on the Windows graphics test machine.

`Scripts/install_apps.py` builds and installs the editor, then prepares a matching runtime template in a separate build tree. It mirrors the editor configuration, native build options, and enabled package set. Templates are published under `bin/RuntimeTemplates/Windows/x64/<configuration>/<template_id>/`; `current.json` selects the latest complete template while older versions remain available. The editor SDK is never replaced by its runtime variant. `--runtime-build-dir` can reuse an already configured runtime tree, but identity mismatches still fail installation.

Each template records file sizes and SHA-256 hashes, native compiler/configuration identity, and separate source identities for the SDK and packages. Source identities include local modifications and dependency revisions/content. Package identities include their declared dependency identities, so changes propagate through dependent packages without invalidating the SDK identity. The build checks for source changes during compilation before assembling the payload. Debug and RelWithDebInfo payloads include host, SDK, and package PDBs; Release payloads omit them. The template identifies EvoEngineRuntime.exe as its host. A template is project-independent: an export must add runtime.yaml and saved project content before it can launch.

Native package API version 3 embeds runtime provenance in each package descriptor. Runtime identities match across editor/player composition; editor companion API 2 validates separate editor SDK/package identities and binding to the loaded runtime module. Loading rejects incompatible SDK/compiler/configuration or mismatched companions before publishing a ready package. Manifests record library hashes. Rebuild older packages against the current SDK.

The host requires a schema-version-1 `runtime.yaml` with the application name, executable-relative `.eveproj` path, matching native identity, and an explicit package name/source-identity list. Optional window dimensions and graphics initialization settings override defaults. The optional `show_console` boolean defaults to `false`. It loads exactly that package set before opening the saved startup scene. Missing metadata, required content/types, or an enabled main camera cause an error dialog and nonzero exit. Unused assets from unloaded providers remain opaque; runtime loading never repairs metadata or saves project assets.

Reflection-probe captures reuse directional shadow maps from a raster camera. In a scene with only ray-traced cameras, probe capture uses unshadowed directional lighting because no raster shadow map exists; the main camera's ray-traced shadows remain available.

Paths resolve from the executable directory, including after relocation or launch from an unrelated working directory. Logs go to `Logs/runtime.log`, engine caches to `Cache`, and application data to `UserData`. Runtime package loading uses the shipped DLLs directly; scratch and third-party temporary files stay below `Cache/Scratch`. Development cache environment overrides are ignored. A writable distribution directory is required. **Show Windows console** also mirrors engine messages to a Windows console while preserving the log file. Shader cache misses log **Compiling Shaders** before compilation; cache hits do not produce this message.

For automated validation, the host accepts `--no-error-dialog --frames 3 --capture UserData/smoke.png`; captures require at least two frames and record the final frame. `Scripts/test_runtime_host.py` exercises rendering, Unicode relocation, opaque assets, metadata preservation, missing-metadata rejection, and the Windows error dialog against a prepared template. Add `--exporter <EvoEngineRuntimeExporter.exe>` to exercise a real export first, including independent startup-scene selection, source preservation, package filtering, and PDB retention.

The runtime `window` configuration accepts `mode` (`windowed`, `borderless_windowed`, `borderless_fullscreen`, or `exclusive_fullscreen`), `width`, `height`, `allow_resize`, and `allow_resolution_change`. Startup uses the primary monitor; both fullscreen modes use its current desktop resolution. The runtime main camera follows the window framebuffer resolution, including resizing and fullscreen transitions; the authored scene is not modified. Unsupported exclusive presentation falls back to a decorated window.

Alt+Enter always switches between the session's last windowed and fullscreen modes. Before either has been used, defaults are the configured decorated window and borderless fullscreen. The windowed position and size are retained for that session, but each launch starts from the saved build configuration. Resize and resolution-change permissions never disable mode switching. Exclusive presentation uses Vulkan's application-controlled fullscreen extension when the device and surface support it. Exclusive startup presents one borderless frame at the same monitor resolution before taking display ownership; this avoids a first-presentation driver stall observed on the Windows test machine.

Android deployment, asset packing, multiple build configurations, custom icons, and a runtime GUI remain future work. Runtime configuration and application data paths are separate from the Windows exporter and window implementation so future platform support can supply its own deployment and storage rules.

## Demo validation

Some demos create or modify their scene during editor startup. To validate their saved runtime content, run the editor with `--demo <id> --resource-root <isolated Resources directory> --author-runtime-scene`. This runs the normal demo setup, saves loaded project assets and the scene, and writes an authoring report under the project's `Cache/RuntimeAuthoring`. It does not remove previously authored demo projects. Use a separate copy of demo resources for this workflow.

`Scripts/test_runtime_demos.py` performs authoring, native export, relocation to a Unicode path, and a 60-frame runtime rendering check for each demo. It verifies source and packaged asset preservation, the exact active package set, a valid screenshot, and executable-relative writes with an unrelated working directory and development cache overrides. Each demo's logs and screenshot remain in the work directory; `results.json` records the outcome.

These are startup and portability checks, not image-quality benchmarks. Short path-traced captures can remain noisy or reflect the scene's authored exposure settings.

Authoring preserves camera resolution instead of resizing it to an editor viewport. Starter scenes with a one-pixel camera use the demo's initial window dimensions for this validation workflow. The existing LSystem starter contains only a camera and light; DigitalAgriculture contains a camera, light, and soil component. Those rows check the saved starters and package activation, without demonstrating generated trees or crops. Review captures alongside the automated results.

Use `--author-warmup-frames <count>` when a demo needs editor frames to finish generating its saved content. The matrix gives EcoSysLab and ProceduralGalaxy 120 frames and requires EcoSysLab's tree-growth completion marker before export.

Validate Galaxy's automatic overview framing with a fresh generated project and scene. Its demo bootstrap frames a newly created cluster; re-authoring a project that already contains the cluster can save the editor's default camera pose instead. Runtime always uses the saved main-camera pose.

Imported source files are not rewritten by demo authoring. Modified imports must first be saved as native project assets. Material texture views reference their saved source textures and retain their own color-space and sampler settings. Temporary BC7 textures without saved sources retain embedded bytes and mip levels; older scenes that contain texture dimensions without pixel data must be recreated from their source assets.

Project glTF material conversions preserve their generated DDS textures under `Assets/.generated/GltfMaterialConversion/v1`. These are project dependencies included in the export; the conversion cache itself is not shipped. Runtime loading reads existing generated assets without creating asset files or metadata.

```powershell
python Scripts/test_runtime_demos.py `
  --editor out/install/vs2026-x64/bin/EvoEngineEditor.exe `
  --exporter out/install/vs2026-x64/bin/EvoEngineRuntimeExporter.exe `
  --template out/install/vs2026-x64/bin/RuntimeTemplates/Windows/x64/RelWithDebInfo/<template_id> `
  --resource-root out/runtime-demo-validation/Resources `
  --work-dir out/runtime-demo-validation/run
```

Use `--demo` to select individual profiles: `rendering`, `rendering-regression`, `ddgi`, `ecosyslab`, `digital-agriculture`, `lsystem`, `procedural-galaxy`, `3dgs`, `bicycle`, or `bistro`. The work directory must not already contain a result directory for a selected demo. Resource preparation and large model downloads remain separate from the test.

Authoring validates required scene references before saving. Build Manager rejects projects with dangling references. The EcoSysLab default scene references the default post-processing stack already embedded in that scene.
