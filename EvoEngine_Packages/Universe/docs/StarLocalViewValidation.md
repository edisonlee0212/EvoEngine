# Star-local view controls and demo defaults

Branch: `codex/universe-performance`. Package-only source changes; existing SDK camera APIs are reused.

## Changes

- Following locks hover and click publication, keeps the selected star, and invalidates asynchronous results on both
  lock edges. GPU intersection/reduction/readback continue. Space exits; unavailable targets detach safely.
- After no-jump coordinate rebasing, the existing smooth camera transition targets 50 visual radii in star-local
  coordinates and looks at zero. The current radial direction is retained; camera backward is used at zero.
  Invalid or FP32-unrepresentable distances are rejected without discarding selection.
- New clusters default to time scale 1 and emission 8 in all three bands. Both bootstrap demo clusters use disk
  diameter 3000 and retain distinct seeds/colors/shapes/positions and 250,000 stars each. Authored scenes are not rewritten.
- Demo cameras receive private post-processing stacks with tone mapping disabled and other settings deep-copied.
  Original references are restored on leaving the demo; user replacement references are respected. SDK global
  defaults, opaque rendering, depth behavior, bloom, and subpixel emission suppression remain unchanged.

## Validation

Focused validation passed: 40 Universe tests, 10 existing camera/viewport tests, and five GPU tests with Vulkan
synchronization validation enabled. No VUID or Vulkan validation warning/error was reported. The GPU fixture's
existing `Serializable type is unregistered!` diagnostic remains unrelated to Vulkan validation.
Both package targets built against the existing SDK in the test tree. The production application build/install
also succeeded; no SDK source edits were needed for this change. The scoped clang-format 22.1.8 check passed
(27 files), as did the scoped whitespace check. The unrelated existing `imgui.ini` trailing blank line was left alone.
Local artifacts: `out/test-artifacts/universe-local-view/`.

Commit gate: reran all 40 Universe tests and 20 focused SDK tests (viewport/rebase, post-processing runtime,
bloom GPU limiter, preview shader contract, and rendering documentation); all passed with no skips.
Formatting passed for all 26 staged C++ files using clang-format 22.1.8. The previous successful application
installation already contains this source state; this commit adds no further runtime changes.

Installed-editor synchronization validation also passed a 100-frame capture with continuous picking, with no
VUIDs or Vulkan validation warning/errors. The visible interactive smoke selected cluster A star 183761, entered
follow using Space, displayed `Hover: none` and the locked-state diagnostic, and completed the inward focus
transition while the CPU star position continued updating. Further input was stopped when user input was detected;
the editor (PID 25140) was left open. Locked clicks in both viewports and live Space exit were not manually completed
in this run; automated lock/generation and camera conversion tests passed. The exact 50-radius endpoint and facing
direction are numerical-test assertions, not inferred from screenshots. The full legacy scene-edit/manual matrix
and the full SDK test suite were not rerun.

## Build and install

```powershell
cmake -S . -B out/build/vs2026-x64-tests
cmake --build out/build/vs2026-x64-tests --config RelWithDebInfo --target UniversePackage --parallel 4 -- /p:BuildProjectReferences=false
cmake --build out/build/vs2026-x64-tests --config RelWithDebInfo --target UniversePackage_Tests --parallel 4 -- /p:BuildProjectReferences=false
cmake -S . -B out/build/vs2026-x64
python Scripts/install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --no-clean-install --jobs 4
```

Manual/capture executable: `C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/bin/EvoEngineEditor.exe`.

## Performance protocol

Three uncontended baseline captures use 1920x1080, 500,000 stars, 1,000 warmup and 1,000 measured frames, GPU
timestamps, and `EVOENGINE_UNIVERSE_PICK_BENCHMARK=1`. Statistics are medians of the three per-run statistics.
Baseline frame median/p95: 5.5840/7.2850 ms; late-window 5.6199/8.3824 ms; report average FPS 192.898.
Baseline GPU compute: 0.496224/0.801896 ms; intersection: 0.528224/0.794275 ms; reduction: 0.006016/0.011680 ms;
forward: 0.264448/0.490422 ms; hover: 0.000640/0.001088 ms.

Updated three-run median-of-statistics:

| Measurement | Median (ms) | p95 (ms) |
| --- | ---: | ---: |
| CPU frame | 5.4924 | 6.9666 |
| CPU frame, late window | 5.4950 | 7.0414 |
| GPU compute | 0.495808 | 0.780616 |
| GPU forward | 0.264256 | 0.512363 |
| GPU picking intersection | 0.528224 | 0.795334 |
| GPU picking reduction | 0.006048 | 0.011490 |
| GPU hover ring | 0.000736 | 0.001120 |

Median reported average FPS: **194.952**, versus baseline **192.898**. The reported FPS uses the capture's elapsed
wall-clock interval and is not the reciprocal of the profiler's median CPU-frame interval.

The follow capture confirmed entry in stdout, zero hover-ring samples, and 1,000 intersection/reduction samples.
CPU follow median/p95 was 0.0011/0.0021 ms versus normal-mode 0.0007/0.0013 ms: median overhead 0.0004 ms.
Follow CPU frame was 5.5305/7.6135 ms; GPU forward was 0.295392/0.600240 ms. These whole-frame/forward numbers
are not a same-view comparison. The normal-mode ring costs approximately 0.000736 ms median and disappears while locked.

Updated rendering settings change visible geometry/overdraw and post-processing work. Before/after totals therefore
must not be attributed solely to the interaction lock or camera transition. A separate `follow` benchmark measures
active local-view CPU work and confirms that picking continues while hover rendering is suppressed.
