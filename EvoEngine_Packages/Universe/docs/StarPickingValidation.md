# Continuous GPU star picking validation

Validated on `codex/universe-performance`, RTX 5070, RelWithDebInfo, 1920x1080, two 250,000-star clusters.

## Functional evidence

- 29 focused tests pass: Universe authoring/rendering regressions, picking publication/lifecycle rules, real GPU
  intersection/reduction against a CPU reference, and SDK viewport coordinate mapping.
- Real GPU cases include counts 0, 1, 255, 256, 257, 500,000; two clusters; nearest/tied candidates across workgroups;
  perspective/orthographic cameras; off-axis rays and translated/rotated cameras; display scaling and the 3-pixel
  tolerance; near/far clipping; opaque depth; invalid cursor; and UV=1 depth-texel clamping.
- Result/push constants are 48 bytes; reduction candidates are 16 bytes. Shader reflection and standalone Slang /
  `spirv-val --target-env vulkan1.3 --scalar-block-layout` pass for intersection and reduction variants.
- Focused GPU tests and a 100-frame installed-editor capture with a valid center cursor pass Khronos synchronization
  validation. Logs contain only the validation-layer activation message, no validation errors/warnings.
- Clear cancels pending clicks, latest click wins, click misses preserve selection, and leaving the image clears
  hover without canceling a same-camera pending click. Obsolete camera/viewport/population/scene requests are rejected.
- The final source shader and installed shader hashes match. Scoped clang-format and diff whitespace checks pass.

The live UI interaction check remains unverified: Windows window capture returned wallpaper for the running editor,
including after reacquiring the window. No blind UI clicks were attempted. The process started solely for this check
was closed. Panel-focus/overlay behavior is source-reviewed and coordinate/state rules are tested, but actual mouse
hover/click/clear interaction still needs a visible desktop check. Full SDK tests and a broad manual scene-mutation
matrix were not run.

## Performance

Three baseline and three final captures use the same demo/camera, 1,000 fixed measurement frames, GPU timestamps,
and a separate late 240-frame window. Final runs set `EVOENGINE_UNIVERSE_PICK_BENCHMARK=1` to continuously query the
active camera center; baseline has no picking. Values below are medians of the three per-run statistics, not pooled
frame percentiles. Validation was disabled for timing.

| Metric | Before | Picking enabled |
|---|---:|---:|
| Full-window frame median / p95 | 3.0573 / 5.4233 ms | 4.7667 / 6.0229 ms |
| Late-window frame median / p95 | 3.0521 / 5.3645 ms | 4.6258 / 6.1142 ms |
| Report FPS | 281.930 | 232.395 |
| Picking intersection median / p95 | — | 0.526208 / 0.528192 ms |
| Picking reduction median / p95 | — | 0.006048 / 0.006178 ms |
| CPU result consumption median / p95 | — | 0.0006 / 0.0011 ms |

Every final run records 1,000 intersection/reduction GPU samples and 1,000 CPU input/readback observations, confirming
the path was active rather than a disabled-cursor measurement. CPU readback medians per run were 0.0006, 0.0006,
and 0.0009 ms. The existing renderer still issues one star draw per camera.

Picking adds roughly 0.53 ms of measured GPU work. The late-window median frame time increased by 1.5737 ms;
the end-to-end increase is larger than the isolated picking timings and is not explained solely by them. FPS fell
about 17.6%. Continuous execution avoids adding work or a wait specifically on a click, but does not make picking free.
No separate temporal click-hitch benchmark was performed.

Local artifacts: `out/test-artifacts/universe-picking/baseline-{1,2,3}.*`, `after-{1,2,3}.*`, `tests.log`,
`gpu-validation-test.log`, `validation.*`, and `install-final.log`. Standalone shader artifacts are under
`out/test-artifacts/universe-star-pick/`.

## Build, installation, and capture

Reconfigured the existing production build to discover the new globbed package source, preserving its cache:

```powershell
cmake -S . -B out/build/vs2026-x64
python Scripts/install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --no-clean-install --jobs 4
```

Final installation succeeded for the configured applications. SDK rebuild was required for the viewport accessor.
Test executable: `out/build/vs2026-x64-tests/EvoEngine_Tests/RelWithDebInfo/UniversePackage_Tests.exe`.
Installed editor used for captures and attempted interaction check:
`C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/bin/EvoEngineEditor.exe`.

Capture arguments (working directory: installed `bin`):

```text
--demo procedural-galaxy --capture-demo-preview <absolute.png>
--preview-width 1920 --preview-height 1080 --preview-warmup-frames 1000
--preview-render-mode rasterization --preview-raster-profile-report <absolute.json>
--preview-gpu-timestamps enabled
```

Unset `EVOENGINE_UNIVERSE_PICK_BENCHMARK` for normal user interaction. No commit was made.
