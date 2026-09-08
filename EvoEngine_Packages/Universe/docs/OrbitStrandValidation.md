# Orbit strand visualization

> The timing table below records the historical 30,000-diameter, 750-orbit configuration. The current fixed
> 10,000,000-diameter, minimum-distance-10 configuration uses 25,000 orbits; see
> [FixedGalaxyViewValidation.md](FixedGalaxyViewValidation.md) for current validation and captures.

## Result

The Universe Layer owns an optional native-strand visualization of nominal star orbits. All, Occupied, and Selected
modes share the deterministic bucket layout. Each cluster is one entity-free scene-geometry submission regardless of
its loop count. Generated assets and their unlit material are runtime-only; no scene entities or serialized state are
created. Per-point RGB follows the cluster's center/core/disk color interpolation, emission is excluded, depth testing
and camera post-processing remain active, and orbit strands do not cast shadows.

Each closed loop contains 256 cubic segments and 259 control points (`P255, P0 ... P255, P0, P1`). Cached local-space
geometry is transformed at submission, so simulation time, cluster movement, and star-local rebasing do not upload new
geometry. Shape/color/radius/orbit-selection changes do. The view-radius transition does not rebuild orbit geometry.
Mesh-shader-disabled systems show a diagnostic and submit no orbit draw.

## Automated validation

- `UniversePackage_Tests.exe --gtest_filter=Universe*`: 54 tests passed with graphics and synchronization validation.
  Coverage includes display-mode selection, invalid/stale selected identities, loop closure, exact nominal equation,
  unit normals, per-orbit color/radius, cache invalidation, one 256-segment entity-free registration, deterministic
  orbit allocation, compute parity, picking, follow transforms, and scene/demo lifecycle.
- `EvoEngine_Tests.exe --gtest_filter=StrandsMeshShader.*`: all 12 SDK strand tests passed under the same validation.
- `UniversePackage`, `UniversePackage_Tests`, and `EvoEngine_Tests` built successfully. The incremental application
  install completed successfully. No Vulkan validation or synchronization diagnostics were emitted by the tests or
  installed-editor captures.

## Installed-editor smoke and timing

The installed editor at
`C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/bin/EvoEngineEditor.exe` completed three 1,000-frame
1920x1080 procedural-galaxy captures on an RTX 5070. Continuous picking, default 500,000 stars, depth writes, and
synchronization validation were enabled. These are matched single runs, not repeated statistical trials.

| Mode | CPU frame median / p95 (ms) | Late median / p95 (ms) | Deferred geometry median / p95 (ms) | Derived average FPS |
| --- | --- | --- | --- | --- |
| Off | 3.649 / 4.221 | 3.613 / 4.209 | 0.0109 / 0.0110 | 263.7 |
| All (historical 750 loops) | 4.139 / 4.778 | 4.127 / 4.779 | 0.899 / 0.911 | 237.8 |
| Selected (one loop, followed view) | 3.744 / 7.706 | 3.697 / 4.101 | 0.027 / 0.027 | 233.5 |

All mode adds about 0.888 ms to the median deferred-geometry pass and about 0.49 ms to median CPU frame time in this
single comparison. The Selected capture changes both view and follow state, so its total frame time is not a direct
Off comparison; its deferred-geometry timing isolates the expected small one-loop draw. Star compute remains roughly
0.51-0.52 ms median in all three runs, confirming orbit rendering does not alter the GPU population simulation.

At the default 0.1 world-unit strand radius, full-cluster overview strands are subpixel and visually subtle beneath
500,000 stars. The radius is intentionally adjustable in the Universe panel. GPU timings and the direct registration
test confirm the All and Selected geometry submissions; the captured images remain useful as crash/output smoke tests.
Artifacts and empty validation stderr logs are in `out/test-artifacts/universe-orbit-strands`.

## Commands

```powershell
cmake --build out/build/vs2026-x64-tests --config RelWithDebInfo --target UniversePackage UniversePackage_Tests --parallel 4
cmake --build out/build/vs2026-x64-tests --config RelWithDebInfo --target EvoEngine_Tests --parallel 4
python Scripts/install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --no-clean-install --jobs 4
```

Captures used `--demo procedural-galaxy --capture-demo-preview <png> --preview-render-mode rasterization
--preview-width 1920 --preview-height 1080 --preview-warmup-frames 1000 --preview-raster-profile-report <json>`.
`EVOENGINE_UNIVERSE_ORBIT_BENCHMARK` selected `all` or `selected`; Selected additionally used
`EVOENGINE_UNIVERSE_PICK_BENCHMARK=follow`. These environment hooks exist only for reproducible capture automation.
