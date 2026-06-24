# Scots Pine Lab PC Handoff

Date: 2026-06-24
Branch: `358-create-scots-pine-dataset-generator`

This branch is the active Scots pine synthetic dataset branch. It now contains the headless Scots pine data generator, production preview script, deferred vertex-color fixes, sparse whorl controls, annotation skeleton export, 360-degree orbit rendering, and screen-space annotation JSON generation.

## Current Goal

Continue the Scots pine synthetic dataset pipeline on the lab PC:

- render 360-view synthetic samples headlessly;
- keep one clean composited image per view;
- write one screen-space skeleton JSON per view;
- write one world-space skeleton JSON per sample;
- profile where time is going before scaling to larger datasets.

For the current 360-orbit dataset contract, each sample produces 721 dataset files:

- 360 `*_composited.jpg`;
- 360 `*_annotation_screen_space.json`;
- 1 `*_annotation_skeleton.json`.

No raw RGBA, foreground masks, annotation overlay images, or camera helper JSON files should remain in the per-sample scene folders for this production path.

## Important Source Files

- `Scripts/scots_pine_synth_preview.py`
  - Main orchestration script for preview and batch generation.
  - Supports `--orbit-views`, `--export-annotation-screen-space`, `--engine-sample-chunk-size`, `--prune-camera-views`, `--no-foreground-mask`, and `--no-annotation-overlay`.
  - Uses chunked one-sample workers when requested to avoid the monolithic 32-sample `bad allocation` failure.
  - Writes `phase_profile_summary.json` and `phase_profile_summary.csv`.

- `EvoEngine_App/src/ScotsPineDataGeneratorApp.cpp`
  - Headless app entry for `--synthetic-config`.

- `EvoEngine_Packages/LSystem/src/ScotsPineBatchGenerator.cpp`
  - C++ synthetic batch implementation.
  - Emits composited images, world-space annotation skeleton JSON, and camera metadata used by the Python postprocess.

- `EvoEngine_Packages/include/ScotsPineBatchContract.hpp`
  - Shared synthetic config contract.

- `EvoEngine_Packages/LSystem/src/ScotsPine.cpp`
  - Scots pine growth, organ geometry, and organ color logic.

- `EvoEngine_Packages/LSystem/src/PineGrowthModel.cpp`
  - Seasonal growth and sparse annual whorl logic.

- `Resources/ScotsPineSynthetic/Backgrounds/extracted_side_view.png`
  - Current production background plate.

- `Resources/ScotsPineSynthetic/Calibration/canonical_rig.json`
  - Current canonical camera/intrinsic reference.

## Latest Successful 32-Sample Run

Output folder on the original PC:

`C:\AlexC\02_skellies\04_EvoEngine\EvoEngine\output\editor_previews\preview_20260624_045100`

That output folder is intentionally not meant to be the source of truth for the lab PC. Treat it as local evidence; reproduce on the lab PC after pulling the branch.

Validated contents:

- 32 scene folders;
- 23,072 dataset files total;
- 11,520 clean composited JPG renders;
- 11,520 screen-space annotation JSON files;
- 32 world-space annotation JSON files;
- 0 raw RGBA files;
- 0 masks;
- 0 annotation overlay files;
- 0 camera helper files in scene folders;
- image size: `1372x1040`;
- every render datapoint in `render_profile.json` succeeded: `11520/11520`.

Timing from `phase_profile_summary.csv`:

- full observed pipeline wall time: about 60.8 minutes;
- engine worker wall time: about 42.6 minutes;
- screen-space annotation postprocess: about 18.1 minutes;
- screen-space annotation mean: about 94.35 ms per view;
- GPU readback diagnostic mean: about 158.98 ms per view;
- RGB write diagnostic mean: about 139.12 ms per view;
- composite diagnostic mean: about 98.10 ms per view.

The diagnostic engine phase rows can be nested and should not be summed as exclusive wall-clock steps. Use the observed wall-clock rows for throughput.

## Reproduction Command

Build the generator first, then run from the repository root. Adjust the worker path to the lab PC build directory if needed.

```powershell
& 'C:\Users\penan\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' Scripts\scots_pine_synth_preview.py `
  --worker out\build\codex-whorl-vs2026-x64\EvoEngine_App\RelWithDebInfo\ScotsPineDataGeneratorApp.exe `
  --scene-count 32 `
  --engine-sample-chunk-size 1 `
  --image-only `
  --render-resolution background `
  --render-mode rasterization `
  --non-strict-parity `
  --vary-scene-pine-seed `
  --out-root output\editor_previews `
  --output-name orbit_721_32_chunked `
  --base-seed 2026062406 `
  --plant-blur-radius-px 1.25 `
  --orbit-views 360 `
  --profile-warmup-datapoints 0 `
  --no-foreground-mask `
  --no-annotation-overlay `
  --export-annotation-screen-space `
  --prune-camera-views
```

The worker path used on the original PC was:

`out\build\codex-whorl-vs2026-x64\EvoEngine_App\RelWithDebInfo\ScotsPineDataGeneratorApp.exe`

## Known Failure and Workaround

A monolithic 32-sample run failed around global sample 6 with `bad allocation`. The successful run used:

`--engine-sample-chunk-size 1`

That launches one engine process per sample and then merges the per-sample render profiles.

## Practical Next Steps

1. Pull this branch on the lab PC.
2. Configure/build `ScotsPineDataGeneratorApp` in a local build folder.
3. Run a 1-sample / small-view smoke first, for example `--scene-count 1 --orbit-views 4`.
4. Then run the 32 x 360 batch command above.
5. Compare the new run's `phase_profile_summary.csv` against the timings listed here.
6. If scaling beyond 32 samples, keep chunking enabled until the underlying engine memory growth is diagnosed.

## Output Validation Checklist

For each `scene_XXXXXX` folder:

- exactly 721 files;
- exactly 360 `*_composited.jpg`;
- exactly 360 `*_annotation_screen_space.json`;
- exactly 1 `*_annotation_skeleton.json`;
- no `*_rgba.png`;
- no `*_foreground_mask.png`;
- no `*_annotation_overlay.png`;
- no `*_camera_views.json`.

For one sample JSON spot check:

- screen-space schema is `scots_pine_annotation_screen_space`;
- world-space schema is `scots_pine_annotation_skeleton`;
- each needle has 64 points;
- each sheath has 3 points;
- each stem has 64 points.

## Branch Hygiene

Generated folders under `output/` and `tmp/` are local evidence and should not be committed unless a later task explicitly asks for a small curated artifact. The branch should carry source, scripts, calibration resources, and this handoff note; the lab PC should regenerate heavy outputs locally.

## Nested Project Caveat

The current generator defaults still load:

`Resources/DigitalAgricultureProject/test.eveproj`

That directory is a nested Git project/submodule. On the original PC, the required Scots pine project files exist there locally:

- `Assets/ScotsPineDataGenerator.evescene`
- `Assets/ScotsPineDataGenerator.evescene.evefilemeta`
- `Assets/New ScotsPineDescriptor.spine`
- `Assets/New ScotsPineDescriptor.spine.evefilemeta`

Before running on the lab PC, verify those files exist in the nested project checkout. If they do not, copy or sync that nested project state separately before treating a render failure as an engine/script bug.
