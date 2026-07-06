# CSM Hazel Backcheck

Reference repository: `C:\Users\lllll\Documents\GitHub\Hazel`

This document records the corrective Hazel-reference review for early CSM milestones that were implemented before the
reference workflow was consistently applied.

## M10: Validation Coverage

Hazel reference files:

- `Hazelnut\src\Panels\SceneRendererPanel.cpp`
  - Visualization panel exposes `Show Shadow Cascades`.
  - Cascade settings expose cascade fading, transition fade, split lambda, near/far offsets, origin scaling, and manual
    cascade split controls.
  - Shadow Map panel displays the selected directional cascade layer.
- `Hazel\src\Hazel\Renderer\SceneRenderer.cpp`
  - `SceneRenderer::CalculateCascades` and `SceneRenderer::CalculateCascadesManualSplit` define the split/fitting inputs
    that need validation.
  - `SceneRenderer::ShadowMapPass` renders each directional cascade and animated cascade pass.
- `Hazelnut\Resources\Shaders\HazelPBR_Static.glsl`
  - `ShowCascades` visualizes selected cascade regions in the final shaded output.

EvoEngine validation coverage:

- `Scripts\validate_csm_milestone.py` runs the format check, builds `EvoEngineEditor`, installs apps, and performs the
  required 30-second Bistro and Rendering smoke runs.
- The same script captures deterministic Bistro and Rendering rasterization output for final shaded-result comparison.
- With `--capture-shadow-diagnostics`, it captures cascade index, light UV, light depth, atlas UV, and texel-density
  diagnostics for the selected light/cascade.
- `docs\csm_validation.md` names the manual visual checks needed for Hazel-equivalent CSM tuning: cascade seams, near/far
  detail, grazing surfaces, camera translation/rotation, alpha-tested foliage, and Rendering demo coverage.
- Editor preview camera override flags, `--preview-camera-position` and `--preview-camera-look-at`, are available for
  controlled static camera comparisons when an automated still frame is more useful than interactive inspection.

Backcheck result:

- The M1 validation gate covers Hazel's core CSM inspection surfaces: final shaded output, cascade selection, shadow-map
  inspection, split/fade controls, camera movement risk, and alpha-tested scene content.
- EvoEngine's extra atlas-UV and texel-density diagnostics are intentional because EvoEngine packs multiple
  shadow-casting directional lights into viewports inside each cascade layer, unlike Hazel's single primary directional
  light path.
- No validation code change is required for M10. Later Hazel backchecks should use the existing gate with
  `--capture-shadow-diagnostics`, and add controlled camera overrides only when a specific comparison needs them.
