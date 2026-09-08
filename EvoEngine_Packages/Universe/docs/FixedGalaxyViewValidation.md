# Fixed galaxy geometry and scaled display transition

> The physical 10,000,000-diameter geometry remains current. The normalized-radius and 0.001x galaxy-display update
> supersedes the mean/deviation radius description and unscaled overview details below; see
> `NormalizedGalaxyDisplayValidation.md` for current behavior and evidence.
> The subsequent view-default update further changes the maximum radius to 15, galaxy radius boost to 30x, and
> view-specific fade targets to 0.0/0.5.

## Result

The default and bootstrapped 500,000-star cluster now uses a fixed disk diameter of 10,000,000, time scale 0.1, and
minimum star distance 10. Star radius uses a clamped normal distribution with mean 1, standard deviation 5, minimum
0.1, and maximum 10.

The resulting 25,000 orbit bands require the expanded 32,768-orbit safety limit. Universe does not modify camera near
or far distances. For every rendering camera, stars beyond 70% of its current far distance are smoothly compressed
below 99% in the vertex shader with proportional radius scaling; physical GPU results and apparent size remain
unchanged. Camera clip settings remain user-controlled through galaxy/follow transitions, scene changes, and demo
override restoration.

Space no longer changes disk geometry. The one-second quartic transition changes only a render-time radius multiplier:
100x in galaxy view and 1x in star-local view. CPU batch parameters always retain physical radii and fixed orbital
geometry. Follow therefore evaluates the selected star's actual deterministic radius and places the camera 20 times
that value away. Before GPU upload, a copied parameter table scales mean, deviation, minimum, and maximum together.
Returning to galaxy view frames the fixed cluster using the final 100x maximum displayed radius.

This removes the rapid rotation caused by recomputing the large accumulated phase against a changing orbit diameter.
Tests verify that evaluated XYZ positions remain exactly equal at 1x, intermediate, and 100x radius scales; only the
result radius changes. Orbit samples, population revisions, GPU layouts, and draw counts remain unchanged. Orbit strand
geometry also remains cached through the transition.

## Validation

- All 57 Universe tests passed with Vulkan graphics and synchronization validation enabled.
- Coverage includes fixed disk dimensions, new defaults and serialization, deterministic sampled radius, physical
  follow distance, reversible radius interpolation, invariant XYZ positions/orbit phase, 100x overview bounds, CPU/GPU
  parity through 500,000 stars, picking, and lifecycle behavior.
- The package and focused test target built successfully. All applications installed successfully with the command
  below. An installed 100-frame overview capture completed without Vulkan or synchronization diagnostics and was
  visually inspected to confirm the complete expanded cluster remains framed.
- Default orbit layout: 25,000 orbits and 1,985,740,912 slots. CPU fixture construction/allocation measured
  500.61/278.73 ms; unchanged updates remained 0.3/0.3 microseconds median/p95.

The installed RTX 5070 1920x1080 smoke capture measured a 2.520/2.906 ms CPU median/p95, 0.525 ms median star compute,
0.293 ms median star forward rendering, and 399.7 derived average FPS over 100 frames. This short first-frame window is
a correctness smoke, not a warmed performance comparison. Its PNG, JSON report, stdout, and empty stderr log are under
`out/test-artifacts/universe-fixed-galaxy-view/diameter-10m*`.

View-specific clipping/compression installed-editor captures also passed with empty stderr. The 100-frame galaxy view
measured 4.810/6.470 ms CPU median/p95, 0.519 ms compute, and 0.288 ms forward. The 500-frame automatic-follow view
measured 5.397/6.885 ms CPU median/p95, 0.519 ms compute, and 0.238 ms forward. These are validation captures, not a
controlled performance comparison. Their PNGs, reports, and logs use the `view-depth-*` prefix in the same artifact
directory.

The subsequent camera-independent compression update preserved authored clip planes and changed the compression range
to 70%-99% of each rendering camera's far distance. Its installed 100-frame 1920x1080 galaxy capture completed with
empty stderr and was visually inspected. Automated tests cover follow entry/exit and camera replacement; the installed
automatic center-ray follow capture did not acquire a star, so it is not counted as a follow-transition smoke result.
Artifacts use the `out/test-artifacts/universe-camera-independent-compression` directory.

The captures below predate the 10,000,000 diameter increase and are retained only as historical radius-transition
evidence. They used the previous fixed 1,000,000 diameter at 1920x1080 with continuous picking and 500,000 stars:

| View | Frames | CPU median / p95 (ms) | Late median / p95 (ms) | GPU compute / forward median (ms) | Average FPS |
| --- | ---: | ---: | ---: | ---: | ---: |
| Galaxy, 100x radius | 600 | 4.383 / 4.949 | 4.380 / 5.132 | 0.534 / 1.394 | 226.6 |
| Follow, actual radius | 1,000 | 3.638 / 4.757 | 3.603 / 4.077 | 0.518 / 0.258 | 261.7 |

These are single smoke captures with different camera views, not a controlled performance comparison. The overview's
large discs intentionally increase fragment/depth work. Both PNGs were inspected: galaxy view contains the complete
cluster with enlarged varied discs; follow view retains the centered selected star and the surrounding fixed-diameter
population. Artifacts and empty stderr logs are under `out/test-artifacts/universe-fixed-galaxy-view`.

## Commands

```powershell
cmake --build out/build/vs2026-x64-tests --config RelWithDebInfo --target UniversePackage_Tests --parallel 4
python Scripts/install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --no-clean-install --jobs 4
```

Installed executable:
`C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/bin/EvoEngineEditor.exe`.
