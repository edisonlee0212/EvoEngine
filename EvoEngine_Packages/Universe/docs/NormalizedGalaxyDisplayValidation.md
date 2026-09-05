# Normalized star sizes and scaled galaxy display

## Result

Star radius authoring now uses minimum, maximum, and unitless normalized deviation. CPU and FP64 compute evaluation
both calculate `mix(min, max, clamp(0.5 + gaussian * deviation, 0, 1))`; default values are 0.1, 15, and 1/6.
The 448-byte parameter and 64-byte result ABIs remain unchanged. Legacy standard deviation migrates by dividing it by
the loaded radius range, while the obsolete mean radius is ignored.

The physical batch retains the authored 10,000,000 disk diameter and physical radii. Galaxy rendering copies those
parameters into a fixed 0.001x display frame, giving an effective diameter of 10,000. Its 30x radius boost is applied
before the coordinate scale, producing a numeric 0.03x physical-radius factor. Follow view uses the existing physical
star-local frame and actual evaluated radius. Minimum-pixel fade transitions smoothly from zero in galaxy view to the
adjustable star-view target, default 0.5.

Entry rebases the scene camera by `world_to_star * inverse(galaxy_scale)`; exit and target invalidation use
`galaxy_scale * star_to_world`. Stars and the scene camera therefore switch coordinate frames atomically before the
existing one-second radius/camera transition. Overview bounds and orbit positions use the scaled copied parameters.
Authored transforms, physical CPU evaluation, samples, and camera near/far distances are unchanged. Per-camera
70%-99% distant-star compression remains active.

## Validation

- The focused Universe suite passes 59 tests with Vulkan graphics validation enabled.
- Tests cover normalized distribution determinism, bounds and midpoint behavior, serialization/cloning, legacy
  migration, CPU/GPU equality, unchanged binary layouts, scaled position/radius evaluation, finite scaled overview
  framing, and reversible galaxy/star camera transforms.
- Compute and graphics shader compilation, GPU readback parity, picking, forward rendering, and synchronization tests
  run as part of the focused suite.
- All applications installed successfully. The installed editor completed a 100-frame 1920x1080 procedural-galaxy
  capture with Vulkan validation and GPU timestamps after the 30x/0.0 fade update. It exited 0 with empty stderr, and
  the captured overview contains the complete scaled cluster. CPU frame median/p95 was 3.629/4.087 ms; star compute
  was 0.521/0.771 ms and star forward rendering was 0.291/0.575 ms. This is a correctness smoke rather than a
  controlled performance comparison.
- Capture artifacts are under `out/test-artifacts/universe-normalized-scaled-display`. The installed executable was
  `out/install/vs2026-x64/bin/EvoEngineEditor.exe`.

## Commands

```powershell
cmake --build out/build/vs2026-x64-tests --config RelWithDebInfo --target UniversePackage_Tests -- /m
$env:EVOENGINE_ENABLE_GRAPHICS_VALIDATION='1'
out/build/vs2026-x64-tests/EvoEngine_Tests/RelWithDebInfo/UniversePackage_Tests.exe --gtest_color=no
python Scripts/install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --no-clean-install --jobs 4
```
