# CSM Validation

Use this validation gate for every CSM overhaul milestone before making that milestone's commit:

```bat
python Scripts\validate_csm_milestone.py --milestone M<N>
```

The gate uses the installed editor at `out\install\vs2026-x64\bin\EvoEngineEditor.exe`. By default it:

- Runs the C++ format check for `EvoEngine_SDK`, `EvoEngine_App`, and `EvoEngine_Tests`.
- Builds the `EvoEngineEditor` target from `out\build\vs2026-x64` in `RelWithDebInfo`.
- Installs apps with `python Scripts\install_apps.py --config RelWithDebInfo --no-open --incremental`.
- Ensures the generated Bistro project exists.
- Launches Bistro for 30 seconds with `EvoEngineEditor.exe --demo bistro --editor`.
- Launches the Rendering demo for 30 seconds with `EvoEngineEditor.exe --demo rendering --editor`.
- Captures deterministic 1920x1080 rasterization previews for Bistro and Rendering.

When a milestone changes or investigates directional shadows, also capture CSM diagnostics:

```bat
python Scripts\validate_csm_milestone.py --milestone M<N> --capture-shadow-diagnostics
```

Diagnostic captures use the editor preview flags:

- `--preview-shadow-debug cascade-index`
- `--preview-shadow-debug light-uv`
- `--preview-shadow-debug light-depth`
- `--preview-shadow-debug atlas-uv`
- `--preview-shadow-debug texel-density`
- `--preview-shadow-debug-cascade <0-3>`
- `--preview-shadow-debug-light <directional-light-index>`

Shadow-map resolution is selected at renderer startup. The default quality is `High` (`4096 x 4096` for directional,
point, and spot shadow maps). To validate another quality, pass:

```bat
python Scripts\validate_csm_milestone.py --milestone M<N> --shadow-map-resolution medium
```

The matching editor startup flag is `--shadow-map-resolution <low|medium|high|very-high>`, where:

- `low` = 1024
- `medium` = 2048
- `high` = 4096
- `very-high` = 8192

CSM validation uses the fixed runtime shadow policy:

- fit policy: Legacy Stable;
- split policy: Practical Log/Uniform;
- sampling: PCF;
- PCF radius: `100 x light_size` texels.

Milestones that need to tune practical split placement can override the preview split lambda:

```bat
python Scripts\validate_csm_milestone.py --milestone M<N> --shadow-split-lambda 0.5 --capture-shadow-diagnostics
```

The matching editor preview flag is `--preview-shadow-split-lambda <0-1>`.

Milestones that need to compare cascade transitions and the final shadow-distance fade can override those widths:

```bat
python Scripts\validate_csm_milestone.py --milestone M<N> --shadow-cascade-transition-width 5 --shadow-distance-fade 20 --capture-shadow-diagnostics
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

- `logs\` contains smoke and capture logs.
- `previews\` contains rendered PNGs.
- `validation-report.md` records the exact editor path, smoke commands, output paths, and visual checklist.

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
