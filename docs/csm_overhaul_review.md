# CSM Overhaul Review

Branch: `codex/csm-overhaul`

Final validation command:

```bat
python Scripts\validate_csm_milestone.py --milestone M9 --shadow-cascade-transition-width 5 --shadow-distance-fade 20 --capture-shadow-diagnostics
```

Validation result:

- C++ format check passed for `EvoEngine_SDK`, `EvoEngine_App`, and `EvoEngine_Tests`.
- `EvoEngineEditor` built from `out\build\vs2026-x64` in `RelWithDebInfo`.
- Apps installed with `python Scripts\install_apps.py --config RelWithDebInfo --no-open --incremental`.
- Bistro stayed open for the 30-second smoke run.
- Rendering stayed open for the 30-second smoke run.
- Final captures were written under `out\csm-validation\m9\previews`.
- Logs and the generated validation report were written under `out\csm-validation\m9`.

Milestone commits:

- `c6e15f90` Add CSM milestone validation gate
- `865ba67c` Add CSM shadow diagnostics
- `a177ead5` Add practical CSM split policy
- `b278e7ae` Add CSM cascade fit policy
- `44a17905` Add directional shadow slope bias
- `40c26fe8` Add directional shadow sampling modes
- `8176d843` Add explicit CSM transition fades
- `f28643e9` Document CSM shadow resource policy
- M9 final review is recorded by this `Record final CSM overhaul review` commit.

Visual review:

- Bistro final rasterization is nonblank and covers the curb, near street detail, plant pots, motorbike, facade shadows,
  foliage, and the known very dark right-side covered street.
- Bistro CSM diagnostics are nonblank. Cascade-index debug shows the explicit transition bands from M7, and
  atlas-UV/texel-density debug confirms the selected single directional light uses the full cascade layer.
- Rendering final rasterization is nonblank and now exercises directional CSM through the top-down directional light
  alongside the existing point and imported spot lights.
- Supplemental Bistro camera-override captures were generated for translation and rotation sanity checks:
  `bistro-controlled-translation-1920x1080.png`,
  `bistro-controlled-rotation-1920x1080.png`,
  `bistro-controlled-translation-cascade-index-1920x1080.png`, and
  `bistro-controlled-rotation-cascade-index-1920x1080.png`.

Sampled image comparisons:

- M1 to M9 Bistro primary: mean RGB delta `11.521677`, changed samples `17.7670%`, max sampled channel delta `235`.
- M1 to M9 Rendering primary: mean RGB delta `0.668614`, changed samples `65.6003%`, max sampled channel delta `34`.
- M8 to M9 Bistro primary: mean RGB delta `0.000000`, changed samples `0.0000%`, max sampled channel delta `0`.
- M8 to M9 Rendering primary: mean RGB delta `0.448133`, changed samples `53.4568%`, max sampled channel delta `43`.
- M8 to M9 Bistro cascade-index and texel-density diagnostics: mean RGB delta `0.000000`, changed samples `0.0000%`.

Notes:

- The Rendering M8-to-M9 numeric delta is visually minor and limited to the dark spot-lit scene; no CSM code changed
  between those milestones.
- The automated validation captures static and controlled camera-override frames. An additional interactive editor pass
  that translates and rotates the Bistro scene camera is still useful as PR hardening, but is not required to close the
  milestone gate.
