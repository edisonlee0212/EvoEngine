# Single-cluster overview and detail views

> This milestone's diameter-scaling transition has been superseded by fixed 10,000,000-diameter geometry and a
> render-radius-only 100x-to-1x transition. See [FixedGalaxyViewValidation.md](FixedGalaxyViewValidation.md).

Pre-commit verification reran all 46 Universe tests under Vulkan synchronization validation and 11 focused
editor-camera/viewport tests successfully. Compute and vertex Slang/SPIR-V validation and application installation
also passed. Logs: `out/test-artifacts/universe-local-view/commit-*`. The full SDK suite and live UI smoke were not
repeated; unrelated local editor state and dirty submodules are excluded from delivery.

## Per-cluster radius distribution

Star Cluster now exposes mean radius, standard deviation and minimum/maximum radii. Positive deviation applies
a clamped normal distribution; zero retains the previous uniform radius and ignores limits. The new deterministic
Gaussian sample uses the unused second output of the existing Z Box–Muller pair, leaving position samples unchanged.
Base samples are now 40 bytes; parameters/results remain 448/64 bytes. Follow uses the evaluated selected radius,
and overview bounds include the maximum. Older ABI measurements below describe their original milestones.

All 46 Universe tests passed with Vulkan synchronization validation and no VUID or synchronization hazards.
Coverage includes normal moments, clipping/degenerate limits, uniform legacy data, serialization/cloning/reset,
parameter-only edits, seed/repacking determinism, follow distance, overview bounds and CPU/GPU parity through
500,000 stars. Slang compilation and scalar SPIR-V validation passed; SPIR-V confirms 40/448/64-byte strides and
the radius sample at byte offset 32. Scoped formatting/whitespace checks passed. Package/test builds and the
application install command below succeeded. Artifacts: `out/test-artifacts/universe-local-view/radius-*`.
No interactive inspector smoke or full SDK test run was performed for this change; no SDK sources were modified.

## Fade control

The Universe Layer now exposes **Star fade strength**, a session-level slider from 0 to 4 (default 1).
Brightness attenuation is `(1 / enlargement_area)^strength`; zero disables enlargement fading but retains the
non-emissive bloom cap. Per-frame render packets capture the value for all camera draws. The graphics push constants
grow from 16 to 20 bytes; the FP64 result/sample/parameter layouts and compute push constants are unchanged.
All 43 Universe tests passed with synchronization validation, including fade strengths 0, 0.5, 1, 2 and 4,
unchanged resolved-star emission, bloom caps and 20-byte push-constant reflection. Vertex SPIR-V validation and
scoped formatting passed; package/test builds and application installation succeeded. Artifacts: `fade-control-*`
under `out/test-artifacts/universe-local-view`. No new interactive slider smoke or full SDK suite was run.

Previous refinement: tighter tilt-aware overview bounds and 2% padding replace the original conservative sum and 5%
padding. The new 500,000-star regression requires a bound at least 10% smaller than the previous default bound and
checks every star remains inside it. GPU brightness now uses reciprocal enlargement area rather than the previous
linear diameter fade. All 43 Universe tests passed under synchronization validation; vertex SPIR-V validation,
formatting and whitespace checks passed. Both package/test builds and application installation succeeded.
Installed `framing-overview.png` shows the closer overview; capture exited zero with empty stderr. The shared overview
helper also supplies the unlock transition endpoint. This run did not repeat live click/Space interaction or the full
SDK suite. Latest artifacts are `out/test-artifacts/universe-local-view/framing-*`.

One bootstrapped cluster retains 500,000 stars, seed 1 and the original blue/yellow/white color settings. Defaults:
time scale 0.1, visual radius 1, disk diameter 30,000, emission 8. Authored clusters are not deleted or rewritten.
Demo camera far clipping was 1,000,000 for this historical capture. Universe no longer overrides either camera clip
distance; only the demo-specific tone-mapping override is restored on leaving the demo.

Space enters star-local follow, then moves to local `[0,0,20 × radius]`, looking down negative Z. This aligns the
selected star and cluster center in front of the camera. Space exits by rebasing the current camera into world
coordinates and transitioning to a whole-cluster overview looking at world origin. Both camera and disk diameter
use the existing one-second quartic ease-out. Runtime diameter scales 1→100 (30,000→3,000,000), returning 100→1 on
unlock; base samples, their buffer, authoring settings and GPU layouts are unchanged. Normal controls resume after
each transition. Reversing mid-transition starts from the current scale/pose; invalidation detaches and shrinks safely.

Overview bounds include the actual population's largest absolute Gaussian samples, orbital extrema across phases,
center offsets, entity rotation/scale/translation, and visual radius. Bounds are cached only on population repacking;
framing requires no position readback or additional per-star animation pass. The sphere is fitted to the narrower
projection axis with 2% margin and near-plane allowance. The initial demo main and scene cameras are framed too.
Orbital tilt bounds the vertical component of each orbit; untilted disks combine their horizontal radius and vertical
Gaussian tails geometrically. This tightens the previous sum-of-radii overview without clipping outliers.

## Initial overview/detail validation

The package and focused test targets built successfully against the existing SDK. All 42 Universe tests passed,
covering aligned 20-radius focus, whole-population conservative bounds across phases/transforms/aspect ratios,
reversible scale easing without sample rebuilds, defaults, and isolated far-clip restoration.
Artifacts: `out/test-artifacts/universe-local-view/single-*`.

Build/install succeeded with:

```powershell
cmake --build out/build/vs2026-x64-tests --config RelWithDebInfo --target UniversePackage --parallel 4 -- /p:BuildProjectReferences=false
cmake --build out/build/vs2026-x64-tests --config RelWithDebInfo --target UniversePackage_Tests --parallel 4 -- /p:BuildProjectReferences=false
python Scripts/install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --no-clean-install --jobs 4
```

Installed smoke executable: `C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/bin/EvoEngineEditor.exe`.
No SDK sources or authored project/submodule files were changed.

## Distant-star visibility

Forward billboards now clamp to one render-target pixel on each axis. Discs below sqrt(2) pixels snap to pixel
centers to guarantee a covered sample. Stars requiring enlargement use their capped non-emissive base color divided
by the enlargement area at the default fade strength: 2x diameter gives one-quarter brightness, 10x gives one hundredth. Each axis contributes
its enlargement factor independently. This area compensation retains distant points without restoring subpixel bloom. Physical radius, compute data, depth
testing/writes and the batched draw are unchanged; occluded/off-screen stars remain hidden.

GPU tests cover perspective/orthographic projection, camera FOVs, viewport sizes, partial-pixel offsets, stable
one-pixel coverage, distance-dependent brightness, bloom limits and opaque-depth occlusion.

Initial distant-star validation: all 42 Universe tests passed with Vulkan synchronization validation enabled, including the
500,000-star expanded-diameter CPU/GPU parity case and the minimum-pixel raster tests. No VUID, validation error,
or synchronization hazard was reported. The existing fixture-only unregistered-serializable diagnostic remains.
The earlier 10 focused SDK camera/viewport tests also passed. Scoped clang-format and whitespace checks passed.
Final compute and vertex Slang compilation and scalar-layout SPIR-V validation passed with empty logs. Reflection
confirmed the unchanged 448/32/64-byte parameter/sample/result layouts and 8-byte compute/16-byte graphics push constants
at that milestone; the adjustable fade control now uses 20-byte graphics push constants as noted above.

The new scale initially amplified the compute shader's existing truncated sine/cosine error to 0.000077 units,
exceeding the original parity tolerance. Adding one FP64 polynomial term to each function fixed the issue without
loosening the tests. GPU layouts remain unchanged.

An installed 1920x1080 locked-view capture completed 1,000 measured frames after warmup with synchronization
validation and no diagnostics. `pixel-follow.png` shows the selected star centered with a field of faint distant
one-pixel stars; stdout confirms follow entry. This was a visibility smoke, not a controlled performance comparison.
The final reinstall additionally includes a tested zero-radius guard (zero-sized stars remain degenerate rather
than writing black pixels). All demo radii in the capture were positive, so that guard does not change its result.
The previous interactive lock/unlock smoke was interrupted and was not repeated; endpoint/coordinate-conversion,
reversal and input-lock behavior are covered by focused tests. No full SDK suite was run. No commit was made.
