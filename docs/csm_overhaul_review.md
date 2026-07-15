# CSM Improvement Review

Branch: `codex/render-system-improvements`

Status: CSM-P0 through CSM-P6 implementation and delivery validation are complete. Artistic shadow-bias and softness
tuning is intentionally left to the user.

## Delivered Contract

- Directional cascades retain four layers, Practical Log/Uniform splits, depth-based cascade selection, explicit
  transition blending, distance fade, whole-scene light-space Z coverage, and the 4096 default resolution.
- Render Layer inspection exposes exactly two process-global, non-serialized fit policies: Stable Sphere and Tight
  Light-Space AABB. Stable Sphere remains the editor-startup default.
- Stable Sphere encloses the eight corners of the actual camera-frustum slice, quantizes the radius upward in
  1/16-world-unit increments, and snaps to the effective packed shadow viewport. Tight Light-Space AABB fits the same
  corners without stabilization.
- Cascade split thresholds are stored per camera alongside the camera-indexed shadow matrices.
- Camera near/far extraction and linear-depth reconstruction use the zero-to-one GLM projection terms consumed by
  Vulkan, so cascade selection matches the fitted frustum slice in both modes.
- Directional filtering is one 16-sample Vogel-disc PCF path through a linear depth-comparison sampler. The world-space
  light radius is converted through each cascade's actual extent and packed viewport, and out-of-viewport samples return
  lit to prevent repeated-edge streaking and cross-light atlas reads.
- Constant, slope, and normal-offset bias remain authored in shadow texels. Their defaults remain 0.1, 0.1, and 0.01.
- Built-in strand-renderer geometry now casts directional shadows through the mesh-shader backend when mesh shaders are
  supported and enabled. It remains absent from the historical CSM-P4 caster fixture; point and spot strand shadows are
  disabled while their mesh paths are migrated.
- The implementation does not use depth-buffer fitting, SDSM, depth readback, projection-based cascade selection, or a
  second shadow-filtering family.

Legacy Stable was retired in CSM-P6 and is no longer a runtime, editor, CLI, or validator choice. Legacy references in
the P0-P5 history below are retained only to describe the measurements and artifacts produced before its removal.

## Milestone Outcomes

- CSM-P0 captured the reproducible Legacy Stable baseline, one-/two-/four-light packed-atlas lanes, per-cascade telemetry,
  and the default 16-launch validation cap.
- CSM-P1 added and numerically verified Stable Sphere and Tight Light-Space AABB while preserving Legacy Stable.
- CSM-P2 replaced post-fetch binary comparisons with hardware comparison-before-filtering and added a deterministic D32
  depth-step GPU probe.
- CSM-P3 froze the production 16-sample Vogel-disc comparison-PCF contract, corrected cascade-specific texel scaling, and
  added filter, transition, snap, and TAA-jitter fit margins.
- CSM-P4 made splits camera-consistent, completed packed-atlas/resolution/caster diagnostics, disabled built-in strand
  directional casting, and closed automated plus interactive delivery validation.
- CSM-P5 moved the startup directional default to 4096 and corrected zero-to-one depth reconstruction so Tight
  Light-Space AABB selects its later cascades instead of returning unshadowed out-of-fit samples.
- CSM-P6 removed Legacy Stable from the public fit API, runtime implementation, editor, preview parser, and validator,
  leaving Stable Sphere and Tight Light-Space AABB as the only fit policies.

## CSM-P4 Automated Validation

Build, test, and install completed with:

```bat
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target EvoEngine_Tests EvoEngineEditor --parallel 4
out\build\vs2026-x64\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter=DirectionalShadowCascadeFit.*:GpuService.DirectionalShadowComparisonSamplerFiltersDepthStep:RenderGraph.DirectionalShadowCasterPathsExposeValidationCategories
python Scripts\install_apps.py --config RelWithDebInfo --no-open --incremental
```

The focused suite passed 17 of 17 tests. Its per-camera case compared a 0.1-near, 90-degree, 1920x1080 camera at
`(0, 2, 5)`/15-degree yaw against a 5.0-near, 145-degree, 900x1600 camera at `(-12, 8, 3)` with a distinct three-axis
rotation, and verified their split thresholds and Stable Sphere matrices differ. The installed editor used by renderer
validation was `out\install\vs2026-x64\bin\EvoEngineEditor.exe`.

The final resumed gate command was:

```bat
python Scripts\validate_csm_milestone.py --milestone CSM-P4 --capture-shadow-diagnostics --resume --skip-format --skip-build --skip-install --skip-smoke
```

CSM-P4 records 18 renderer attempts: 12 succeeded and 6 failed attempts remain in the manifest as recovery evidence.
The checked-in validator cap remains 16. The final two diagnostic launches used an explicit one-time exception to 18;
attempt 17 captured Bistro cascade indices and attempt 18 captured the four-light Rendering atlas layout. Both succeeded
without fatal, device-loss, or Vulkan device-loss markers.

Attempt 16 validated the strand-free synthetic caster fixture with 4 regular, 4 instanced, 8 skinned, 4 external, 0
strand, and 0 mesh-shader draws. The normal demo captures separately exercised the mesh-shader path. The generated
cascade-index, atlas-UV, and texel-density diagnostics are nonblank and show the expected cascade regions, packed-light
quadrants, and caster coverage without obvious cross-atlas corruption.

Three of the four P4 timing lanes remained under the P0 10% median ceilings: Rendering one light measured 0.440448 ms,
two lights measured 0.843024 ms, and four lights measured 1.817408 ms. The P4 Bistro sample measured 4.590384 ms against
the 4.017622 ms ceiling, while the matched Stable Sphere P3 sample was 3.707072 ms. This variance is retained in the
report; no tuning or additional renderer launch was added to P4.

The full report, manifest, logs, metrics, and captures are under `out\csm-validation\csm-p4`.

## CSM-P5 Default and Tight AABB Correction

The directional startup default is now `High` (4096), matching the existing point/spot default. Explicit quality
overrides remain unified across all three light types, including `Very High` (8192). The Tight AABB failure was not a
missing fit or array layer: `Cameras.glsl` read the projection's fixed perspective term at `[2][3]` while reconstructing
near/far. With EvoEngine's zero-to-one GLM projection, the depth translation is `[3][2]`, near is `abs(b / a)`, and far
is `abs(b / (a + 1))`. The old values inflated linear depth, selected a farther exact-fit cascade, and made the sampler's
out-of-fit guard return lit.

The authoritative build, focused suite, and all-app install completed with:

```bat
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target EvoEngine_Tests EvoEngineEditor --parallel 4
out\build\vs2026-x64\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter=GraphicsInitializationSettings.*:CameraRenderTechnique.ZeroToOneDepthHelpersUseProjectionTranslation:DirectionalShadowCascadeFit.*:GpuService.DirectionalShadowComparisonSamplerFiltersDepthStep:RenderGraph.DirectionalShadowCasterPathsExposeValidationCategories
python Scripts\install_apps.py --config RelWithDebInfo --no-open --incremental
```

All 21 focused tests passed. The regression projects a representative receiver into each of the four depth slices,
reconstructs its zero-to-one depth, verifies selection of the matching cascade, and confirms the Tight AABB fit contains
it. Three direct installed-editor Bistro captures then exercised Legacy Stable, Stable Sphere, and Tight AABB without a
shadow-resolution override. Each recorded 16 PCF samples and `[0, 0, 4096, 4096]` for all four cascade viewports. All
three images were visually checked; Tight AABB again shadows the later visible regions, and no fatal or device-loss
marker appeared. Artifacts are under `out\csm-validation\csm-p5-default-4096-fit-comparison`.

## CSM-P6 Legacy Stable Removal

Legacy Stable and its split-distance-square extent input were removed from the public fit API and fitting path. Stable
Sphere now has enum value 0 and remains the default; Tight Light-Space AABB has enum value 1. The global fit selection is
not serialized, so no scene migration was required. The editor dropdown, preview parser, and validation script expose
only those two modes, while the preview parser explicitly rejects the retired `legacy-stable` token.

Format, the authoritative editor/test build, the focused suite, and the incremental all-app install completed with:

```bat
python Scripts\format_cpp.py --root EvoEngine_SDK --root EvoEngine_App --root EvoEngine_Tests --check
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target EvoEngine_Tests EvoEngineEditor --parallel 4
out\build\vs2026-x64\EvoEngine_Tests\RelWithDebInfo\EvoEngine_Tests.exe --gtest_filter=LauncherUtils.ShadowCascadeFitNamesExcludeLegacyStable:GraphicsInitializationSettings.*:CameraRenderTechnique.ZeroToOneDepthHelpersUseProjectionTranslation:DirectionalShadowCascadeFit.*:GpuService.DirectionalShadowComparisonSamplerFiltersDepthStep:RenderGraph.DirectionalShadowCasterPathsExposeValidationCategories
python Scripts\install_apps.py --config RelWithDebInfo --no-open --incremental
```

All 21 focused tests passed. The installed editor rejected `--preview-shadow-fit legacy-stable` with exit code 1 and
`Unknown shadow cascade fit mode: legacy-stable` before renderer initialization. Exactly two subsequent renderer
launches captured deterministic 1920x1080 Bistro frames after 120 warmup frames, one per remaining fit policy. Both
recorded 16 PCF samples, a cascade-0 split start of approximately 0.1, and four `[0, 0, 4096, 4096]` viewports. The
captures are nonblank and visually coherent, Tight AABB retains later-cascade shadows, and the logs contain no fatal,
validation-error, or device-loss marker. Artifacts are under `out\csm-validation\csm-p6-fit-smoke`.

## Interactive Validation

Before Legacy Stable was retired, one installed-editor session opened the Rendering demo, switched the camera to
Rasterization, translated camera X from 0.0 to 0.25, and rotated camera Y from 0 to 3 degrees. Legacy Stable, Stable
Sphere, and Tight Light-Space AABB were each
selected through `View > Layer Inspection > Built-in/App Layers > Render Layer > Shadow`. All three modes remained
responsive and nonblank with no crash or obvious cascade/atlas corruption. Before closing the editor, the session
restored Stable Sphere, camera position `(0, 0, 3)`, camera rotation `(0, 0, 0)`, and Ray Tracing render mode.

No CSM implementation milestone remains. Project-specific artistic tuning is outside this review.
