# Single-build native artifacts

## Goal

Build `EvoEngine_SDK` and each runtime package DLL once, then use those exact artifacts in both the editor installation and native runtime template. Editor behavior remains in `EvoEngine_EditorSDK` and package editor companions. One CMake build graph produces both hosts and the runtime template.

This document records the original M1 contract, the two-tree baseline, and the resulting single-build design.

## Original two-tree flow

Before this change, `Scripts/install_apps.py` performed two native builds:

1. Configure and install the editor preset (`EVOENGINE_WITH_EDITOR=ON`). This builds the runtime SDK and packages, editor SDK and companions, editor applications, tests selected by the graph, and resources.
2. Configure the matching runtime preset (`EVOENGINE_WITH_EDITOR=OFF`). This recompiles the runtime SDK and all enabled runtime packages, builds `EvoEngineRuntime`, and runs `EvoEngineRuntimePayload` to assemble the template.

The runtime preset receives the editor build's build and package options. Installation rejects mismatched compiler, configuration, platform, architecture, SDK source identity, package source identities, or package selection.

The existing RelWithDebInfo editor and runtime trees contain duplicate, same-sized outputs for the SDK and all eight enabled runtime packages. Their SHA-256 hashes differ because they are separately compiled and linked artifacts.

| Runtime artifact | Size in each current tree |
| --- | ---: |
| `EvoEngine_SDK.dll` | 19,869,184 bytes |
| `BillboardCloudsPackage.dll` | 3,609,600 bytes |
| `DatasetGenerationPackage.dll` | 2,233,344 bytes |
| `DigitalAgriculturePackage.dll` | 6,178,304 bytes |
| `EcoSysLabPackage.dll` | 24,006,656 bytes |
| `LSystemPackage.dll` | 4,104,704 bytes |
| `MeshRepairPackage.dll` | 534,016 bytes |
| `TextureBakingPackage.dll` | 519,680 bytes |
| `UniversePackage.dll` | 2,500,096 bytes |

The two build identities already have equal `sdk_source_id`, `host_source_id`, package source identities, compiler/toolchain fields, configuration, platform, architecture, and runtime build options. They intentionally differ in `with_editor` and in the presence of editor-only identities.

## Existing sources of binary variation

The runtime SDK and package source trees no longer contain editor implementation, but `EVOENGINE_WITH_EDITOR` is still a global compile definition.

- `EvoEngine_SDK/src/NativeBuildIdentity.cpp` embeds the composition flag in `NativeBuildIdentity::with_editor`.
- `EvoEngine_SDK/src/RuntimePaths.cpp` uses the flag as its pre-application strict-runtime fallback.
- `EvoEngine_Packages/include/EvoEngine_Package_PCH.hpp` embeds the flag in every runtime package descriptor.
- `EvoEngine_App/CMakeLists.txt` makes the editor applications and `EvoEngineRuntime` mutually exclusive.
- `cmake/EvoEngineNativeBuild.cmake` exposes `EvoEngineRuntimePayload` only in the editor-disabled Windows x64 graph.
- Installer, exporter, template, and host tests require editor metadata to set `with_editor=true` and runtime metadata to set `with_editor=false`.

No runtime SDK public header or runtime package source uses the flag to change runtime class layout or behavior. Package CMake files use it only to select their editor companion subdirectories.

## Identity contract

Runtime artifact identity and application composition are separate concepts.

### Runtime artifact identity

The following fields determine whether independently loaded native runtime artifacts are compatible:

- SDK source identity, including runtime SDK/services/CMake inputs and third-party dependency state
- package source identity and transitive runtime package dependency identities
- compiler and compiler version
- build configuration
- platform and architecture
- ABI-affecting runtime build options

These fields must be identical for editor and runtime consumers. Editing an editor-only source or resource must not change them.

### Editor extension identity

Editor SDK identity depends on the runtime SDK identity plus editor SDK sources. An editor package identity depends on its runtime package identity, editor SDK identity, its editor sources, and declared editor dependencies. Runtime payloads never contain these artifacts.

### Composition metadata

Whether an executable or distribution includes editor functionality belongs to the host or payload metadata. It must not alter the shared SDK or runtime package binary. `NativeBuildIdentity::with_editor` remains transitional until host and payload metadata own this distinction.

`IsNativeBuildCompatible` already ignores `with_editor`; source-identity generation already excludes `Editor` directories from SDK and runtime package identities. M1 tests lock down those properties and identical generated runtime identity headers.

## Target build graph

```text
EvoEngineEditor.exe ──> EvoEngine_EditorSDK.dll ──> EvoEngine_SDK.dll
                   └─> runtime package DLLs
                   └─> package editor companion DLLs

EvoEngineRuntime.exe ────────────────────────────> EvoEngine_SDK.dll
                   └─> selected runtime package DLLs

EvoEngineRuntimePayload
  └─> EvoEngineRuntime.exe + the same SDK/package targets + runtime resources
```

`EVOENGINE_WITH_EDITOR` may continue to select whether editor targets are present, but it must not affect compilation of shared runtime targets. Editor-only compile definitions belong on editor hosts, editor SDK, editor companions, and editor tests rather than on the directory globally.

## Required behavior during migration

- `ApplicationInfo::strict_runtime` is authoritative after application construction. Pre-application path behavior must be explicit and composition-neutral.
- The runtime host remains strict and the editor remains non-strict.
- Package descriptors and manifests still reject incompatible source/toolchain/configuration identities.
- Editor companion validation still binds a companion to the exact runtime package module and editor SDK identity.
- Runtime template construction uses explicit runtime targets and resources; it must not discover or copy editor files by directory scan.
- Debug and RelWithDebInfo PDB behavior, package dependency selection, source-change verification, immutable template publication, export, and Unicode relocation remain unchanged.
- Static SDK composition remains supported when runtime packages are disabled.

## Validation gates

1. Identity unit tests demonstrate editor/runtime composition produces equal runtime SDK/package source identities and generated runtime headers.
2. Runtime-boundary checks reject editor headers, targets, GUI extensions, and per-class editor guards in the runtime closure.
3. The editor and runtime host load the same built SDK file; editor and payload package files are copied from the same runtime package targets and have identical hashes.
4. Native identity, package load/unload, strict runtime paths, template preparation, installer, exporter, real export, and relocation tests pass.
5. A fresh single-tree install compiles each shared runtime target once and improves elapsed time relative to the M1 two-tree baseline.

## M1 baseline

The binary inventory above was captured from the existing `out/build/vs2026-x64` and `out/build/vs2026-x64-runtime` trees.

The representative cold baseline used empty, isolated build and install directories, RelWithDebInfo, Visual Studio 18 2026, x64, eight parallel jobs, all eight default runtime packages, editor and launcher enabled, demo applications disabled, tests disabled, and graphics validation disabled. Source and dependency filesystem caches were not flushed, so this is a developer fresh-build comparison rather than a cold-machine benchmark.

| Step | Elapsed time |
| --- | ---: |
| Configure editor graph | 16.16 s |
| Build and install editor graph | 481.98 s |
| Configure runtime graph | 15.05 s |
| Build runtime payload | 380.77 s |
| Total | 893.96 s (14m 53.96s) |

The editor tree produced 1,298 object files totaling 4.00 GiB. The runtime tree produced another 1,132 object files totaling 3.59 GiB. Both trees compiled 350 runtime SDK/package objects; the runtime tree also independently compiled 778 third-party objects. Generated metadata confirmed equal SDK source identity, host source identity, and all runtime package source identities.

Baseline locations:

- `out/build/single-build-baseline-editor`
- `out/install/single-build-baseline-editor`
- `out/build/single-build-baseline-runtime`
- `out/build/single-build-baseline-runtime/RuntimePayload/RelWithDebInfo/template`

The M5 comparison should use the same configuration and job count in one empty build tree. The expected saved work is the entire second configure plus the repeated SDK, runtime package, and third-party compilation; the runtime host and payload assembly remain required.

## Implemented single-tree flow

The editor-enabled graph now builds both `EvoEngineEditor` and `EvoEngineRuntime`. Shared runtime targets no longer receive `EVOENGINE_WITH_EDITOR`; runtime strictness comes from `ApplicationInfo::strict_runtime`, and editor-only identities remain attached to editor SDK and companion targets.

`EvoEngineRuntimePayload` consumes explicit runtime-only target lists from the main graph. It copies the already-built SDK and package DLLs, host, manifests, PDBs, and runtime resources without directory scanning. The build generates a metadata-only runtime identity with `with_editor=false`; this does not rewrite the composition-neutral headers used to compile the shared artifacts. The normal installed editor identity remains `with_editor=true`.

`Scripts/install_apps.py` now performs one configure and one install build, then builds the lightweight payload assembly target in the same build directory. The obsolete `--runtime-build-dir` option was removed. The standalone runtime preset remains available for boundary validation, but it is not part of installation.

## M5 result

The final comparison used a fresh isolated tree with the same RelWithDebInfo configuration, Visual Studio 18 2026 x64 generator, eight jobs, default eight runtime packages, disabled demos/tests, and graphics validation disabled.

| Step | Two-tree baseline | Single-tree result |
| --- | ---: | ---: |
| Configure editor/main graph | 16.16 s | 15.82 s |
| Build and install editor/main graph | 481.98 s | 482.01 s |
| Configure runtime graph | 15.05 s | removed |
| Build/assemble runtime payload | 380.77 s | 43.21 s |
| Total | 893.96 s | 541.04 s |

The single-tree flow saves 352.92 seconds, or 39.5%, on this machine. The retained build phase changed by only 0.03 seconds. The result contains 1,300 object files totaling 4.00 GiB and exactly one set of the 350 runtime SDK/package objects; it eliminates the old runtime tree's 1,132 objects, including its duplicate 350 runtime SDK/package objects and 778 third-party objects.

The fresh template contains 452 files, reports `with_editor=false`, and contains no editor-named DLL or resource. Hash checks confirm that its SDK and sampled package DLLs are byte-identical copies of the main graph outputs.
