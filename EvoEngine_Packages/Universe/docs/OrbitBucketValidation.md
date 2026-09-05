# Two-level orbit buckets

The default 500,000-star cluster now uses 25,000 orbits with 1,985,740,912 available slots. The orbit safety limit is
32,768. Requested stars are capped to available capacity without changing serialized count. Assignment and Gaussian
streams are deterministic; size samples and spatial offsets are unchanged. Slot jitter and density-wave motion do not
guarantee collision-free separation.

## Validation

- 51 Universe tests pass with Vulkan synchronization validation, without skips, VUIDs or synchronization hazards.
- All 11 focused editor-camera/viewport tests also pass. Scoped formatting and whitespace checks pass.
- Coverage includes six-orbit spacing, exact multiples, narrow/equal ranges, ellipse arc-slot inversion, unique
  occupancy, orbit-selection and within-slot sampling distributions, deterministic prefixes, invalid/resource-limit
  settings, requested/active counts, serialization/cloning/reuse, independent clusters and clocks, rebasing, and
  stale-click rejection after same-count layout edits. Capacity loss safely detaches follow without automatic resume.
- CPU/GPU comparisons cover 0, 1, 255, 256, 257, 250,000 and 500,000 stars, plus two simultaneous clusters and expanded
  follow coordinates. Sample stride is 48 bytes, with orbital phase at byte 40; parameters/results remain 448/64 bytes.
  Slang compilation and scalar-layout SPIR-V validation pass and confirm these strides.
- The installed editor completed 1920x1080 overview and 1,000-frame automatic-follow captures. Images were inspected:
  the overview is populated and follow shows the centered target, surrounding stars, and no hover ring. Follow logs
  confirm entry and synchronization-validation stderr is empty. No manual inspector/click/Space session or full SDK
  suite was run. No SDK sources changed.

## Timing

The capture table below predates the fixed 10,000,000-diameter, minimum-distance-10 configuration. The current
25,000-orbit layout's CPU fixture construction took **500.61 ms**, allocation took **278.73 ms**, and unchanged updates
remained **0.3/0.3 microseconds** median/p95.

CPU fixture: building default orbit tables took **16.72 ms**, assigning 500,000 stars **148.73 ms**. These are edit-time
costs; count edits replay allocation. Across 1,000 unchanged updates, median/p95
parameter/batch update was **0.3/0.3 microseconds**, with identical sample storage, table storage and revisions.

One matched capture per version, RTX 5070, 1920x1080, 500,000 stars, continuous picking, default depth writes and fade:

| Timing (ms) | Previous median / p95 | Buckets median / p95 |
| --- | --- | --- |
| CPU frame | 3.0906 / 3.7053 | 3.0683 / 3.7226 |
| GPU star compute | 0.5681 / 0.7052 | 0.5576 / 0.6876 |
| GPU forward | 0.2870 / 0.2936 | 0.2869 / 0.2913 |
| GPU pick intersection | 0.5303 / 0.5323 | 0.5283 / 0.5302 |

Derived average FPS: **321.4 → 325.4**. This is a small single-run difference, not evidence of a repeatable speedup.
The existing capture tool waits for scene readiness, then records the first requested 200 frames; its misleadingly
named `--preview-warmup-frames` controls the measured window, not a separate discarded warmup. These captures therefore
are preliminary readiness-gated timings, not a rigorous repeated warmed benchmark. The extra DDGI measure argument
has no effect on raster-profile window length. Layout/allocation cost is measured separately by the CPU fixture.

A subsequent 2,000-frame capture provides a warmed tail of the last 240 frames: CPU-frame median/p95
**3.0135/3.5787 ms** (about **330 FPS** from its average). The first 1,760 frames are excluded from that tail.
There is no equivalently long pre-change tail, so this is a warmed current measurement, not a warmed A/B claim.
Whole-run median/p95 is 2.9997/3.5128 ms; capture stderr is empty. Artifact: `buckets-warmed.json`.

## Build and installation

```powershell
cmake --build out/build/vs2026-x64-tests --config RelWithDebInfo --target UniversePackage UniversePackage_Tests --parallel 4 -- /p:BuildProjectReferences=false
python Scripts/install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --no-clean-install --jobs 4
```

Both commands succeeded. Installed capture executable:
`C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/bin/EvoEngineEditor.exe`.
Captures used `--demo procedural-galaxy --capture-demo-preview <output> --preview-render-mode rasterization
--preview-width 1920 --preview-height 1080 --preview-warmup-frames 200 --preview-raster-profile-report <report>`.
`EVOENGINE_UNIVERSE_PICK_BENCHMARK=1` enabled picking; `follow` enabled the follow smoke, without a profile report and
with 1,000 frames. Validation used `VK_INSTANCE_LAYERS=VK_LAYER_KHRONOS_validation` and `VK_LAYER_VALIDATE_SYNC=1`.
Artifacts: `out/test-artifacts/universe-local-view/buckets-*`. Existing unrelated local changes were preserved.
