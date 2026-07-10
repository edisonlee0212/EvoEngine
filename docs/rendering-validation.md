# Rendering Validation

[Back to rendering overview](rendering.md)

Rendering validation is local-only because it requires a Vulkan-capable GPU and often produces image artifacts for manual
inspection. Hosted CI should stay focused on format/build checks unless a task explicitly changes that policy.

## Baseline Checks

For rendering documentation or lightweight render behavior changes, run the smallest checks that exercise the changed
surface:

```bat
python Scripts\format_cpp.py --check --root EvoEngine_SDK --root EvoEngine_App --root EvoEngine_Tests
git diff --check
```

Validate and prepare generated demo resources with:

```bat
python Scripts\prepare_demos.py --editor out\install\vs2026-x64\bin\EvoEngineEditor.exe
```

Pass `--demo <id>` for a single demo, `--validate` for validation only, `--prepare` for missing-file preparation only,
`--override` to force preparation without validation, or `--no-previews` when preview images are outside the current task.

If app behavior changed, build or install the relevant executable before manual validation:

```bat
cmake --build out\build\vs2026-x64 --config RelWithDebInfo --target DemoApp
python Scripts\install_apps.py --config RelWithDebInfo --no-open --incremental
```

Installed binaries are written to:

```text
out\install\vs2026-x64\bin
```

## README Screenshot Capture

Use the unified demo preparation script to refresh the README editor image:

```bat
python Scripts\prepare_demos.py --demo rendering --override --no-previews
```

The script writes `Resources/GitHub/RenderingDemo.png` for the Rendering demo. Inspect the image after capture.

Useful DDGI inspection variants:

```bat
python Scripts\capture_readme_editor_screenshot.py --output out\visual-inspection\sponza-rendering-ddgi.png --demo-setup Rendering --width 1920 --height 1080 --warmup-frames 512
python Scripts\capture_readme_editor_screenshot.py --output out\visual-inspection\sponza-ddgi-atlas-preview.png --demo-setup Rendering --ddgi-atlas-preview --width 1920 --height 1080 --warmup-frames 512
python Scripts\capture_readme_editor_screenshot.py --output out\visual-inspection\cornell-box-ddgi.png --demo-setup CornellBox --width 1920 --height 1080 --warmup-frames 512
python Scripts\capture_readme_editor_screenshot.py --output out\visual-inspection\thin-wall-ddgi.png --demo-setup ThinWall --width 1920 --height 1080 --warmup-frames 512
```

## Demo Preview Capture

Preview captures should be run from an installed app tree so shader and resource installation are validated with the same
executable a reviewer can launch.

Rendering regression examples:

```bat
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\rendering-regression-rasterization.png --preview-render-mode rasterization --preview-warmup-frames 1800 --preview-width 1280 --preview-height 720
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\rendering-regression-raytracing.png --preview-render-mode raytracing --preview-warmup-frames 512 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo rendering-regression --editor --capture-demo-preview out\rendering-regression-rayquery.png --preview-render-mode rayquery --preview-warmup-frames 512 --preview-sample-size 4 --preview-width 1280 --preview-height 720 --preview-deterministic
```

`--preview-render-mode` accepts `rasterization`, `raytracing`, and `rayquery`. RayQuery captures require a device with
RayQuery support. `--preview-sample-size` controls manual samples per rendered frame for ray techniques.

Other useful preview flags:

- `--preview-firefly-clamp enabled|disabled`
- `--preview-firefly-clamp-threshold <value>`
- `--preview-auto-spp enabled|disabled`
- `--preview-auto-spp-min-samples <n>`
- `--preview-auto-spp-max-samples <n>`
- `--preview-auto-spp-threshold <value>`
- `--preview-ser disabled|automatic|enabled`
- `--preview-ao ssao|gtao|disabled`
- `--preview-taa enabled|disabled`
- `--preview-debug none|taa-motion|taa-depth-confidence|taa-history-confidence`

The TAA debug modes enable TAA automatically and capture the motion-vector, depth-confidence, or accumulated
history-confidence output. Preview post-processing overrides require `--capture-demo-preview`.

## DemoApp Smoke Run

A temporary run config can exercise the Rendering demo smoke path:

```yaml
mode: smoke_test
demo_setup: Rendering
application_mode: Editor
warmup_frames: 30
frames_after_play: 30
max_load_frames: 30000
max_play_frames: 1000
exit_on_complete: true
```

Run it with:

```bat
out\build\vs2026-x64\EvoEngine_App\RelWithDebInfo\DemoApp.exe --run-config out\documentation-rendering-smoke.yaml
```

The Rendering smoke path validates the canonical DDGI volume, the top-down directional light, the yellow point light, DDGI
update reasons, disabled-light behavior, relocation/classification toggles, and a real Sponza hallway lighting readback.

## Bistro Reference Parity

Bistro path-tracing parity compares EvoEngine against `vk_gltf_renderer`. The reference input must use the same
`KHR_lights_punctual` directional light intensity as the EvoEngine scene. For Bistro light-unit comparisons, use the
normalized glTF with directional Sun intensity `10`, not the raw downloaded `6830` asset value.

Example:

```bat
C:\Users\lllll\Documents\GitHub\vk_gltf_renderer\_bin\Release\vk_gltf_renderer.exe --headless --size 1920 1080 --scenefile C:\Users\lllll\Documents\GitHub\EvoEngine\Resources\.generated\niagara_bistro\bistro-directional-intensity-10.gltf --frames 512 --maxFrames 512 --ptSamples 4 --ptAdaptiveSampling 0 --renderSystem 0 --envSystem 0 --gltfCamera 0 --output out\bistro-reference-raytracing-2048spp-1920x1080-raw.png
out\install\vs2026-x64\bin\EvoEngineEditor.exe --demo bistro --editor --capture-demo-preview out\evoengine-bistro-raytracing-2048spp-1920x1080.png --preview-render-mode raytracing --preview-warmup-frames 512 --preview-sample-size 4 --preview-width 1920 --preview-height 1080 --preview-deterministic
python Scripts\compare_reference_render.py out\bistro-reference-raytracing-2048spp-1920x1080-raw.png out\evoengine-bistro-raytracing-2048spp-1920x1080.png --ignore-alpha --out out\bistro-raytracing-2048spp-1920x1080-rgb-diff.json
```

Current accepted tracking target is normalized RGB MAE `<= 0.02` and RMS `<= 0.04` full-frame against the alpha-normalized
reference. Focused diagnostic crops may use the same threshold family but must record measured crop and residual
statistics.

## Visual Checks

Before closing rendering milestones, inspect the generated images and record the exact executable and command used.

Minimum visual checks:

- Rendering demo editor screenshot is nonblank and has the expected editor layout.
- RT-Bistro and 3DGS-Bicycle gallery images remain valid.
- Rasterization and ray captures are not blank.
- Bistro reference captures preserve RGB and normalize alpha for tooling when needed.
- Logs do not contain crash, hang, validation, device-lost, or missing-file errors.
