# Star-local follow and hover validation

Branch: `codex/universe-performance`. Test hardware: RTX 5070, RelWithDebInfo, two 250,000-star clusters.

## Implementation

- Selection is retained by runtime cluster identity, seed, and ordinal until a valid replacement or scene reset.
  Unavailable targets detach without automatically resuming when they reappear.
- CPU single-star evaluation uses original world parameters and the same simulation clock as compute. A rigid
  star-local frame is composed into the uploaded cluster matrices, preserving all existing GPU buffer layouts.
- Scene-camera rebasing also converts active smooth-transition endpoints and movement velocity. Normal editor
  camera-buffer collection occurs after the Universe update. Other cameras and scene entities are not rebased.
- Hover draws one current-GPU-result annulus in the picking viewport only, with depth testing, no depth writes,
  destination-alpha preservation, and brightness below bloom onset. Following suppresses this draw.
- No full-population staging or readback was added. Existing continuous picking still reads one 48-byte result.

## Validation protocol

Focused Universe tests cover selection publication, CPU follow lifetime/transforms, independent clocks, population
boundaries and GPU compute/reference parity. Offscreen graphics tests cover hover ring geometry, display scaling,
depth rejection, bloom-limited RGB and alpha preservation. SDK viewport tests cover Space gating and camera rebasing.
All 30 Universe tests and 10 focused SDK viewport/camera tests passed. The final five-test GPU subset and an installed
100-frame editor capture passed Khronos synchronization validation without Vulkan warnings/errors. GPU fixtures
still emit the existing non-Vulkan `Serializable type is unregistered!` asset-registration message.

Validation initially exposed an uninitialized multiview mask in stack-created graphics test pipelines. The three
fixtures and both Universe graphics pipelines now explicitly set single-view mask zero; the final rerun is clean.
The new standalone follow test also exposed a missing Camera forward declaration, now fixed in StarPicking.hpp.

Live desktop smoke succeeded using the installed editor: click selection/replacement, selection retained across
panel focus changes, Space follow/unfollow, live CPU position updates, visible hover annulus and suppression during
following. Only the editor process launched for this smoke was closed; another running editor was left untouched.
The computer-use skill supplied screenshot-grounded interaction for this check. Full SDK tests, a broad manual
scene-mutation matrix, and prolonged manual camera-navigation testing were not run.

Standalone shaders use scalar layout and Vulkan 1.3 SPIR-V validation. Application installation uses:

```powershell
cmake -S . -B out/build/vs2026-x64
python Scripts/install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --no-clean-install --jobs 4
```

Installation completed successfully for all configured applications. The final package-only refresh also succeeded:

```powershell
cmake --build out/build/vs2026-x64 --config RelWithDebInfo --target UniversePackage --parallel 4 -- /p:BuildProjectReferences=false
cmake --install out/build/vs2026-x64 --config RelWithDebInfo
```

The compute, star vertex/fragment and hover vertex/fragment shaders all compiled and passed `spirv-val --target-env
vulkan1.3 --scalar-block-layout`. Reflection/SPIR-V confirm the 32/448/64-byte buffer strides and 32-byte hover push
constant. All 15 touched C++ implementation/header/test files pass the pinned clang-format check; scoped diff
whitespace checks pass. Pre-existing dirty files/submodules remain outside this change; no commit was made.

Installed editor: `C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/bin/EvoEngineEditor.exe`.

## Performance protocol

Three fresh before/after captures use 1920x1080, 1,000 warmup frames, 1,000 measured frames, GPU timestamps,
and `EVOENGINE_UNIVERSE_PICK_BENCHMARK=1`. Each also reports a late 240-frame window. Report the median of the
three per-run statistics, rather than pooling percentiles. Validation is disabled for these timing captures.

The separate `follow` benchmark value selects the first valid hover and enters follow for CPU evaluation timings.
It is not a like-for-like total-frame comparison: the main camera is unchanged while the star field is rebased.

Fresh baseline: full-window frame median/p95 5.5677/7.0212 ms; late-window 5.5737/7.0199 ms; report FPS 189.801.
Picking intersection median/p95 0.528240/0.808821 ms; reduction 0.006016/0.011680 ms; CPU readback 0.0006/0.0011 ms.

| Metric | Picking baseline | Follow/highlight implementation, not following |
|---|---:|---:|
| Frame median / p95 (ms) | 5.5677 / 7.0212 | 5.4974 / 7.2624 |
| Late-window median / p95 (ms) | 5.5737 / 7.0199 | 5.5196 / 7.2545 |
| Report average FPS | 189.801 | 199.268 |
| Pick intersection median / p95 (ms) | 0.528240 / 0.808821 | 0.526544 / 0.797538 |
| Pick reduction median / p95 (ms) | 0.006016 / 0.011680 | 0.006016 / 0.011680 |
| Hover ring median / p95 (ms) | Not present | 0.000672 / 0.001088 |

After-capture compute median/p95: 0.494848/0.778912 ms; star forward: 0.263968/0.501821 ms. Each normal after capture
records 1,000 hover samples and 1,000 samples per picking pass. The follow capture logs successful entry, records
1,000 picking samples per pass and no hover timestamp samples, confirming suppression rather than hidden geometry.
Active CPU follow work measures 0.0011/0.0020 ms median/p95 over 1,000 observations.

The full-window median improves by 0.0703 ms, while p95 worsens by 0.2412 ms. These small, mixed changes do not
establish a reliable end-to-end speedup. Report FPS is derived from average frame duration, not the reciprocal of
median frame time. The follow capture's altered main-camera star composition is not used for the before/after claim.

Local artifacts: `out/test-artifacts/universe-follow/`. Build, test, installation, shader validation and captures
are retained there. The older picking-only validation document describes its historical milestone, including the
now-superseded clear-selection action.
