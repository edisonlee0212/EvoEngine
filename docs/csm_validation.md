# CSM Validation

Use this validation gate for every CSM overhaul milestone before making that milestone's commit:

```bat
python Scripts\validate_csm_milestone.py --milestone CSM-P<N>
```

The gate uses the installed editor at `out\install\vs2026-x64\bin\EvoEngineEditor.exe`. By default it:

- Runs the C++ format check for `EvoEngine_SDK`, `EvoEngine_App`, and `EvoEngine_Tests`.
- Builds the `EvoEngineEditor` target from `out\build\vs2026-x64` in `RelWithDebInfo`.
- Installs apps with `python Scripts\install_apps.py --config RelWithDebInfo --no-open --incremental`.
- Ensures the generated Bistro project exists.
- Launches Bistro for 30 seconds with `EvoEngineEditor.exe --demo bistro --editor`.
- Launches the Rendering demo for 30 seconds with `EvoEngineEditor.exe --demo rendering --editor`.
- Captures deterministic 1920x1080 rasterization previews for Bistro and Rendering.

CSM-P4 also reuses the Rendering capture as the deterministic one-light lane and adds only two- and four-light captures.
The historical CSM-P0 gate used the same light lanes with the now-retired Legacy Stable fit; the current validator rejects
the `CSM-P0` label rather than silently producing a different baseline. The hard cap is 16 nonredundant renderer launches
per milestone gate, including smoke, primary, diagnostic, and packed-light captures. Any exception requires explicit
approval, remains visible in the persistent attempt manifest, and does not change the checked-in cap. The generated
report records the actual count.

When a milestone changes or investigates directional shadows, also capture CSM diagnostics:

```bat
python Scripts\validate_csm_milestone.py --milestone CSM-P<N> --capture-shadow-diagnostics
```

Diagnostic captures use the editor preview flags:

- `--preview-shadow-fit <stable-sphere|tight-aabb>`
- `--preview-shadow-pcf-samples <1-64>`
- `--preview-shadow-debug cascade-index`
- `--preview-shadow-debug light-uv`
- `--preview-shadow-debug light-depth`
- `--preview-shadow-debug atlas-uv`
- `--preview-shadow-debug texel-density`
- `--preview-shadow-debug-cascade <0-3>`
- `--preview-shadow-debug-light <directional-light-index>`

Shadow-map resolution is selected at renderer startup. Directional, point, and spot shadow maps default to `High`
(`4096 x 4096`). To validate a unified quality override, pass:

```bat
python Scripts\validate_csm_milestone.py --milestone CSM-P<N> --shadow-map-resolution medium
```

The matching editor startup flag is `--shadow-map-resolution <low|medium|high|very-high>`, where:

- `low` = 1024
- `medium` = 2048
- `high` = 4096
- `very-high` = 8192

CSM validation uses the following runtime shadow policy:

- fit policy: Stable Sphere by default, with Tight Light-Space AABB available through RenderLayer inspection;
- split policy: Practical Log/Uniform;
- directional sampling: 16-sample Vogel-disc PCF through a linear depth-comparison sampler;
- point and spot sampling: the existing 32-sample PCF paths;
- built-in strand-renderer directional, point, and spot shadow casting: enabled through the mesh-shader backend when
  mesh shaders are supported and enabled;
- `--preview-strand-punctual-fixture` rejects captures unless its frame-global draw telemetry records one strand on
  every point-light cube face and one strand in the spot-light shadow pass;
- directional PCF radius: directional-light `light_size` in world units, converted independently through each cascade's
  light-space X/Y half-extent;
- directional bias: constant, slope, and normal offset are authored in shadow texels and scaled by the larger
  packed-viewport world-units-per-texel value; defaults are 0.1, 0.1, and 0.01 texels respectively.

Stable Sphere encloses the eight corners of the actual camera-frustum slice, rounds the sphere radius upward to a
1/16-world-unit increment, and snaps the projection to the effective packed shadow viewport. Tight Light-Space AABB fits
independent light-space X and Y bounds around the same eight corners and intentionally does not snap. Both modes retain
the whole-scene light-space depth range.

Each directional fit includes the declared world-space PCF radius plus the comparison-filter and, for stable fits,
projection-snap margin. The two frustum-derived fits also extend their fitted slice over the same bounded half-width used
by cascade-transition blending. A conservative camera-jitter envelope keeps TAA edge receivers inside the fit. Samples
outside the light's local packed viewport return lit instead of repeating edge depth or crossing into another light's
atlas region. Cascade split distances are stored alongside each camera block, so the shader's cascade selection,
transition blending, distance fade, and that camera's shadow matrices use the same near plane.

Use Stable Sphere for the production stable choice or Tight Light-Space AABB for the maximum-utilization comparison.

Directional-light bias values are serialized numerically and have no schema marker. Existing values are interpreted in
the current texel units without automatic migration; scenes authored with unusually small values may need manual
retuning.

Legacy Stable was retired in CSM-P6 and is no longer present in the runtime enum, editor, preview parser, or validator.
Legacy references in the P0-P5 sections below describe historical measurements and artifacts only.

## CSM-P0 Baseline

The 2026-07-14 CSM-P0 gate completed all 16 allowed launches with 1,800 timing samples per measured lane. It used Legacy
Stable, 32 PCF samples, the then-default 8192 directional resolution, and the deterministic light-count fixtures described
above.

| Lane | Median GPU | P95 GPU | Shadow draws | Indirect commands | 10% median ceiling |
| --- | ---: | ---: | ---: | ---: | ---: |
| Bistro, one directional light | 3.652384 ms | 3.829238 ms | 4 | 10,268 | 4.017622 ms |
| Rendering, one directional light | 0.454208 ms | 0.457858 ms | 4 | 2,084 | 0.499629 ms |
| Rendering, two packed directional lights | 0.848064 ms | 0.857590 ms | 8 | 4,168 | 0.932870 ms |
| Rendering, four packed directional lights | 1.669504 ms | 1.833253 ms | 16 | 8,336 | 1.836454 ms |

Use matching scene, light-count, resolution, warmup, and device settings when applying these ceilings. The generated
baseline report and raw metrics are under `out\csm-validation\csm-p0`.

## CSM-P1 Fit Comparison

CSM-P1 used six renderer launches for matched Bistro and Rendering captures across all three fit modes. The numerical
suite verified all eight frustum-slice corners remain inside Stable Sphere and Tight Light-Space AABB, Stable Sphere
rounds its radius upward to a 1/16-world-unit increment, Legacy Stable and Stable Sphere move in integer shadow texels at
8192 and 4096 viewport sizes, and Tight Light-Space AABB intentionally follows sub-texel camera motion.

Projected-area change below is relative to Legacy Stable; positive values mean a smaller projection and therefore more
available shadow texels over the receiver slice.

| Scene and fit | Cascade 0 | Cascade 1 | Cascade 2 | Cascade 3 | Motion behavior |
| --- | ---: | ---: | ---: | ---: | --- |
| Bistro, Stable Sphere | +31.2% | +49.5% | +51.9% | +48.2% | Integer-texel stabilized |
| Bistro, Tight AABB | +63.4% | +75.8% | +78.4% | +74.5% | Sub-texel; shimmer trade-off |
| Rendering, Stable Sphere | -63.8% | -45.4% | -43.0% | -46.6% | Integer-texel stabilized |
| Rendering, Tight AABB | +48.8% | +73.8% | +78.9% | +71.2% | Sub-texel; shimmer trade-off |

Stable Sphere is deliberately rotation-stable and can be more conservative than Legacy Stable for a wide or portrait
frustum. Tight AABB consistently maximizes utilization, but its unsnapped bounds make it the comparison choice for users
who accept camera-motion shimmer. The matched reports and raw metrics are under `out\csm-validation\csm-p1-legacy`,
`out\csm-validation\csm-p1-sphere`, and `out\csm-validation\csm-p1-tight`.

## CSM-P2 Comparison Sampling

CSM-P2 changed only directional shadows from a linearly filtered depth fetch followed by one binary comparison to a
linear depth-comparison sampler. Point and spot sampling and the directional descriptor binding layout remain unchanged.
A deterministic 2x1 D32 GPU probe verifies comparison coverage `[0, 0.25, 0.5, 0.75, 1]` across a depth step and verifies
fully lit and fully shadowed one-tap references. The 14-launch gate retained all five diagnostic modes for both demos and
found no atlas bleed, cascade seam, missing caster coverage, or diagnostic corruption.

At the P2 32-sample setting, directional-shadow rendering stayed within 0.2% of the same-fit P1 medians, while Deferred
Lighting increased by 17.9-29.7%. CSM-P3 therefore retains the stable Vogel-disc pattern but evaluates a directional-only
16-sample contract; point and spot PCF remain at 32 samples.

| Fit and scene | P1 deferred median | P2 deferred median | Change | P2 shadow-render median |
| --- | ---: | ---: | ---: | ---: |
| Legacy, Bistro | 1.684368 ms | 2.022640 ms | +20.1% | 3.652736 ms |
| Legacy, Rendering | 2.868240 ms | 3.566016 ms | +24.3% | 0.452896 ms |
| Stable Sphere, Bistro | 1.731136 ms | 2.040640 ms | +17.9% | 3.712640 ms |
| Stable Sphere, Rendering | 2.749504 ms | 3.566896 ms | +29.7% | 0.442624 ms |

The P2 reports and artifacts are under `out\csm-validation\csm-p2-legacy` and `out\csm-validation\csm-p2`.

## CSM-P3 Production PCF Contract

CSM-P3 freezes directional filtering at 16 Vogel-disc samples through the depth-comparison sampler. The directional
light's `light_size` is the filter radius in world units; each cascade converts that radius through its own light-space
X/Y extent and effective packed viewport. Samples outside the local light viewport return lit, and the fit includes the
filter, half-texel comparison, stabilization, transition, and TAA-jitter margins needed by the selected mode.

The implementation preserves the directional constant, slope, and normal-offset defaults at 0.1, 0.1, and 0.01 shadow
texels. No artistic bias tuning is included; authored values remain available for project-specific adjustment.

| Fit, resolution, and scene | Median shadow GPU | P0 10% ceiling |
| --- | ---: | ---: |
| Stable Sphere, 8192, Bistro | 3.707072 ms | 4.017622 ms |
| Stable Sphere, 8192, Rendering | 0.439168 ms | 0.499629 ms |
| Tight AABB, 8192, Bistro | 2.918432 ms | 4.017622 ms |
| Tight AABB, 8192, Rendering | 0.228832 ms | 0.499629 ms |
| Stable Sphere, 2048, Bistro | 3.064832 ms | 4.017622 ms |
| Stable Sphere, 2048, Rendering | 0.413040 ms | 0.499629 ms |

The reports and raw metrics are under `out\csm-validation\csm-p3`, `out\csm-validation\csm-p3-tight`, and
`out\csm-validation\csm-p3-medium`.

## CSM-P4 Delivery Validation

CSM-P4 closed per-camera splits, packed-atlas coverage, the non-default 2048 resolution lane, directional caster-path
accounting, and the final editor comparison. The focused C++/GPU/shader suite passed 17 of 17 tests. Its distinct-camera
case used near planes 0.1 and 5.0, FOV values 90 and 145 degrees, resolutions 1920x1080 and 900x1600, and different
positions/rotations, then verified both the per-camera split thresholds and Stable Sphere matrices differ. The editor
build and incremental all-app install completed, and the installed editor path was
`out\install\vs2026-x64\bin\EvoEngineEditor.exe`.

The P4 manifest contains 18 attempts: 12 succeeded and 6 failed attempts are retained as recovery evidence. The final two
diagnostic launches used an explicitly approved one-time extension beyond the checked-in 16-launch cap. Attempt 17
captured Bistro cascade indices and attempt 18 captured the four-light Rendering atlas layout; both completed without
fatal or device-loss markers. The strand-free caster fixture succeeded on attempt 16 with 4 regular, 4 instanced, 8
skinned, 4 external, 0 strand, and 0 mesh-shader draws. Normal demo captures separately covered mesh-shader directional
casting.

The cascade-index, atlas-UV, and texel-density captures were visually checked and were nonblank, with the expected
cascade bands, four packed directional-light quadrants, and caster coverage. The interactive check used one Rendering
editor session in Rasterization mode, translated camera X by 0.25, rotated camera Y by 3 degrees, and selected Legacy
Stable, Stable Sphere, and Tight Light-Space AABB through Render Layer inspection. All modes remained responsive and
nonblank. Stable Sphere, the original camera transform, and Ray Tracing mode were restored before the editor closed.

Rendering one-light, two-light, and four-light shadow medians remained within their P0 10% ceilings. The P4 Bistro sample
measured 4.590384 ms against its 4.017622 ms ceiling, while the matched Stable Sphere P3 sample measured 3.707072 ms. The
P4 report preserves that variance; no artistic tuning or additional renderer launch was performed.

The final report, manifest, logs, metrics, and captures are under `out\csm-validation\csm-p4`.

## CSM-P5 Default and Tight AABB Correction

CSM-P5 changed the directional startup default from the historical 8192 value to 4096 without changing explicit
quality overrides. It also corrected `Cameras.glsl` near/far extraction for EvoEngine's zero-to-one GLM projection. The
old helper used projection element `[2][3]`, inflating linearized depth and assigning later Tight AABB receivers to a
farther exact-fit cascade or beyond the final shadow distance. The corrected helper uses `[3][2]`, `abs(b / a)` for the
near plane, and `abs(b / (a + 1))` for the far plane.

The authoritative editor/test build and incremental all-app install succeeded. The focused suite passed 21 of 21 tests,
including a regression that projects and selects a receiver in every Tight AABB cascade. Exactly three deterministic
1920x1080 Bistro captures then ran from the installed editor, one per fit mode, with no shadow-resolution override. Each
recorded the requested fit, 16 PCF samples, one light, and `[0, 0, 4096, 4096]` for all four cascade viewports. The three
images were visually checked; later visible regions remain shadowed in Tight AABB, and the logs contain no fatal,
device-loss, or capture-failure marker. Captures, metrics, and logs are under
`out\csm-validation\csm-p5-default-4096-fit-comparison`.

## CSM-P6 Legacy Stable Removal

CSM-P6 removed Legacy Stable and its dedicated extent input from the public fit API and runtime fitting path. Stable
Sphere remains the global non-serialized default and Tight Light-Space AABB remains the comparison mode. The editor,
preview parser, and validator now expose only those two choices. The historical `CSM-P0` validator label is rejected so
it cannot silently regenerate that baseline with a different fit.

The full C++ format check, authoritative editor/test build, and incremental all-app install succeeded. The focused suite
passed 21 of 21 tests, including both remaining fits, enum/name coverage, and rejection of the retired preview token. The
installed editor also rejected `--preview-shadow-fit legacy-stable` with exit code 1 before renderer initialization.

Exactly two deterministic installed-editor Bistro captures then exercised Stable Sphere and Tight Light-Space AABB at
1920x1080 after 120 warmup frames. Both recorded 16 PCF samples, a cascade-0 split start of approximately 0.1, and four
`[0, 0, 4096, 4096]` viewports. Both images are nonblank and visually coherent, Tight AABB retains later-cascade shadows,
and no fatal, exception, assertion, validation-error, or device-loss marker appeared. Captures, metrics, and logs are
under `out\csm-validation\csm-p6-fit-smoke`.

To exercise a non-default practical split placement, override the preview split lambda:

```bat
python Scripts\validate_csm_milestone.py --milestone CSM-P<N> --shadow-split-lambda 0.5 --capture-shadow-diagnostics
```

The matching editor preview flag is `--preview-shadow-split-lambda <0-1>`.

To compare cascade transitions and the final shadow-distance fade, override those widths:

```bat
python Scripts\validate_csm_milestone.py --milestone CSM-P<N> --shadow-cascade-transition-width 5 --shadow-distance-fade 20 --capture-shadow-diagnostics
```

The matching editor preview flags are `--preview-shadow-cascade-transition-width <view-depth-units>` and
`--preview-shadow-distance-fade <view-depth-units>`.

## Directional Shadow Resource Policy

EvoEngine stores directional CSM cascades as four array layers in a single `directional_light_shadow_map_resolution`
texture. Each layer is a full `directional_light_shadow_map_resolution` square. Shadow-casting directional lights then
share viewport regions inside each cascade layer:

- one shadow-casting directional light receives the full layer resolution;
- two to four shadow-casting directional lights receive quadrant viewports;
- more than four shadow-casting lights recursively subdivide the fourth quadrant, subject to
  `max_directional_light_size`.

The diagnostics UI and texel-density debug view expose the effective directional-light viewport size for the selected
light.

The active shadow-map quality initializes directional, point, and spot shadow-map resolution fields together. The Shadow
diagnostics UI displays the active quality, full directional layer size, and packed directional viewport sizes.

Validation output is written to `out\csm-validation\<milestone>\`:

- `renderer-attempts.json` persistently records each renderer command fingerprint and its running, succeeded, or failed
  status. `--resume` reuses only exact successful commands with valid artifacts; interrupted or invalid attempts remain
  counted, and failed logs are retained with attempt suffixes.
- `logs\` contains smoke and capture logs.
- `previews\` contains rendered PNGs.
- `validation-report.md` records the exact editor path, smoke commands, output paths, and visual checklist.
- Primary and packed-light captures also write metrics JSON containing the directional-shadow GPU timing, draw counts,
  split ranges, orthographic extents, viewport sizes, world-units-per-texel, PCF radius/sample count, and bias settings.

The smoke step passes only when each demo stays open for the requested duration and no fatal/error markers appear in the captured output. A milestone is not closed until the generated images are also visually checked.

## Visual Checks

For each milestone, inspect the generated previews and, when the change affects camera stability or cascade transitions, also move the camera in the editor:

- Cascade seams: check Bistro road curb, street edge, and building facade transitions.
- Near/far sharpness: check Bistro curb stones, plant pot, motorbike front, and distant building shadows.
- Grazing surfaces: check curb edges and shallow-angle road surfaces.
- Camera motion: translate and rotate the camera to catch cascade swimming or shimmer.
- Foliage shadow silhouettes: shadow maps intentionally treat material alpha as opaque; check Bistro trees and plant
  leaves for acceptable solid-shadow coverage where visible.
- Rendering demo coverage: check near primitives, far surfaces, camera rotation, and camera translation.

If a milestone intentionally changes output, keep the before/after captures under that milestone folder or note the comparison in `validation-report.md`.

The final CSM overhaul review for this branch is recorded in `docs\csm_overhaul_review.md`.
