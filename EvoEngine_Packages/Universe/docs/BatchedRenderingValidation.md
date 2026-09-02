# Batched star rendering validation

The performance numbers below describe the earlier additive-disc version. The subsequent opaque-disc change disables
blending and removes radial/edge intensity falloff; bloom alone supplies the halo. Legacy alpha no longer affects
forward radiance. A fifteenth GPU test verifies nearest-disc occlusion in both draw orders, solid HDR color through
the rim, discarded quad corners, opaque scene-depth occlusion, and draw-order behavior when depth writes are disabled.
Opaque shader compilation and capture evidence are under out/test-artifacts/universe-opaque.

## Implemented path

Universe Layer owns a packed sample array, cluster parameter table, independent clocks, shared frame-slot outputs,
and one forward draw per camera. The demo defaults to 250,000 stars in each of two clusters. Components contain
only authoring state. Depth writing is a default-on layer setting. Production position readback is removed.

Two correctness issues uncovered during validation were fixed within the package:

- Slang's unsuffixed non-exact constants had been rounded to FP32 before conversion to FP64. Explicit double
  literals remove position errors reaching 0.184 world units at the default phase.
- The SDK's generic RenderTexture::Render transitions color to GENERAL but declares ATTACHMENT_OPTIMAL.
  The package now opens dynamic rendering directly using ForwardExternal's graph-managed attachment layouts.
  SDK sources and binaries were not rebuilt or modified.

Cloned components can share serialized handles. Layer cache identities therefore distinguish component instances
and verify their handles on reuse; runtime identities remain independent of packed offsets.

## Tests and runtime evidence

- 14 UniverseStarCluster tests pass, including count/seed determinism, packing, edits, independent clocks,
  pause/disable/re-enable, deletion/reset, serialization migration, copied-handle live clones, and ABI sizes.
- The GPU test uses test-only readback for 0, 1, 255, 256, 257, 250,000, and 250,000 + 250,000 populations,
  including transformed/independently colored ranges and an untouched output guard. All populated outputs are
  finite and match double-precision reference positions/radius within 0.00003 and RGB/emission within 0.00001.
- Compute, vertex, and fragment shaders compile with scalar layout and pass SPIR-V validation. Reflection matches
  32-byte samples, 448-byte parameters, 64-byte results, 8-byte compute push constants, and the 4-byte camera index.
- A 100-frame 1920x1080 editor capture with Khronos synchronization validation enabled finishes with no validation
  warnings/errors. The layer was injected externally; the existing SDK's validation-OFF build was reused.
- API tracing records one vkCmdDraw with vertexCount=6 and instanceCount=500000. Both differently colored clusters
  appear in the captured image. The forward callback remains before clouds and transparent geometry.
- An earlier combined validation/API-dump run reported SDK ray-tracing pipeline-handle errors, outside the star
  compute/graphics pipelines. The subsequent validation-only run was clean; the combined trace is retained.
- The repository-wide shader-policy script flags the existing .vert.slang and .frag.slang paths because its legacy
  extension regex matches their intermediate suffixes. These same paths exist in HEAD; no policy-script change was made.
- Dedicated interactive multi-camera, opaque/cloud/transparent occlusion, and depth-write-off visual comparisons
  were not performed. Lifecycle/settings cases above are CPU-state tests, not an exhaustive live-editor mutation matrix.

Artifacts are in out/test-artifacts/universe-batching:

- gpu13: current shader binaries and reflection.
- validation-fixed: API trace and capture after the attachment-layout fix.
- validation-only: clean synchronization-validation log and capture.
- run-5 through run-7: final warmed performance captures; earlier runs are retained as intermediate evidence.

## Performance

RTX 5070, 1920x1080 rasterization, 500,000 stars, depth writes on, GPU timestamps on, validation off.
Each capture runs 1,000 frames. CPU frame statistics/FPS below use the final 240 frames. GPU/CPU scoped profiler
statistics cover the capture's 1,000 measured frames; they are not strictly the same late-window sample set.

| Final run | Warm FPS | Frame median / p95 (ms) | Compute median / p95 (ms) | Forward median / p95 (ms) |
| --- | ---: | ---: | ---: | ---: |
| 5 | 229.50 | 4.891 / 6.026 | 0.496 / 0.790 | 0.276 / 0.563 |
| 6 | 227.60 | 4.937 / 5.973 | 0.497 / 0.801 | 0.276 / 0.559 |
| 7 | 225.16 | 4.977 / 6.164 | 0.498 / 0.792 | 0.277 / 0.575 |

CPU parameter preparation was 0.0020 ms median / 0.0037 ms p95; render registration was
0.0006 ms median / 0.0012 ms p95 in all three runs. Readback cost is absent, not an unmeasured rendering dependency.

The earlier 500,000-star captures in out/test-artifacts/universe-500k-depth-write measured about 375 FPS,
0.536 ms compute, and 0.303 ms forward rendering. Current star-pass medians improve approximately 7% and 9%,
respectively, but the end-to-end FPS target is missed and GPU p95 is worse. Current swapchain-presentation CPU
median is about 3.11–3.14 ms versus about 0.09 ms previously. This is an observed timing difference, not proof of
its root cause; the older baseline is not a contemporaneous controlled A/B run.

## Reproduction and deployment

From the repository root:

```powershell
cmake --build out/build/vs2026-x64 --target UniversePackage --config RelWithDebInfo -- /m /p:BuildProjectReferences=false
cmake --build out/build/vs2026-x64-tests --target UniversePackage UniversePackage_Tests --config RelWithDebInfo -- /m /p:BuildProjectReferences=false
ctest --test-dir out/build/vs2026-x64-tests -C RelWithDebInfo -R "^UniverseStarCluster\." --output-on-failure
python Scripts/format_cpp.py --check --root EvoEngine_Packages/Universe --root EvoEngine_Tests/Core/UniverseStarClusterTest.cpp
```

Both package builds and tests succeeded. The first command's existing post-build deployment installs the Universe
DLL/resources into out/install/vs2026-x64/bin; the installed DLL hash matches the built DLL.
No application build or all-app reinstall was run, preserving the package-only requirement.

The existing executable used for all editor smoke tests and benchmarks:
C:/Users/lllll/Documents/GitHub/EvoEngine/out/build/vs2026-x64/EvoEngine_App/RelWithDebInfo/EvoEngineEditor.exe

Launch from its directory with these arguments and absolute artifact paths:

```text
--demo procedural-galaxy --capture-demo-preview <capture.png>
--preview-width 1920 --preview-height 1080 --preview-warmup-frames 1000
--preview-render-mode rasterization --preview-raster-profile-report <report.json>
--preview-gpu-timestamps enabled
```

For the separate validation capture, inject VK_INSTANCE_LAYERS=VK_LAYER_KHRONOS_validation and
VK_LAYER_VALIDATE_SYNC=1, point VK_LAYER_SETTINGS_PATH to the artifact settings directory, and use 100 frames.
Validation and API tracing must remain off for timing captures.
