# GI refinement

## Full-resolution HDDAGI

HDDAGI now always uses the full camera viewport for diffuse GI and sharp reflections. The editor checkbox, C++/Python setting and serialized `half_resolution` field are removed. Legacy assets containing either value still load; the obsolete key is ignored and omitted when saved. Scripts no longer expose the half/full switch. Other provider settings and probe history are unchanged.

The camera layout preserves odd dimensions exactly and clamps empty/tiny extents to one texel. Reflection filtering retains its full-resolution radius of 12. Viewport resizing and filter changes retain the scene's probe field. The shader parameter ABI remains unchanged, with pixel stride always one.

This deliberately departs from pinned Godot HDDAGI's half-resolution default. Historical timings in [the H6 report](hddagi-results.md) describe the earlier revision and are not fresh measurements of this change.

## Indoor visibility experiment

`GiOcclusion.PartitionedRoomComparison` creates two adjacent spaces with a 0.25-unit partition, floor, ceiling and outer walls. A white emissive sphere at (-2, 1.5, -2) illuminates the left space; a camera at (2, 1.5, 5) observes the right. Both spaces are open toward the camera. Environment illumination is zero; materials are diffuse. Removing the partition provides an open control, not ground-truth irradiance.

All providers use 17×17×17 probes, two cascades and base distance 1. The viewport is 960×540; HDDAGI is full resolution. Each observation settles for 180 frames. The central quarter-width/quarter-height image rectangle measures luminance from render-target RGB using weights 0.2126/0.7152/0.0722. This is a local leakage/darkening indicator, not a scene-wide quality score or a cross-provider energy calibration. DDGI uses deterministic seed 42; RT is enabled for the fixture so DDGI can run. Separate HDDAGI regression fixtures run without RT.

The six current variants are SDFGI occlusion off/on; HDDAGI Use Occlusion off/on; and DDGI visibility moment bias 0.02/0. DDGI's existing minimum weight is followed by cubic low-weight suppression, unlike the former HDDAGI visibility floor. This experiment does not remove that suppression or introduce a voxel dependency into DDGI.

Build `EvoEngine_Tests` before running. From its `RelWithDebInfo` output directory, set `EVOENGINE_GI_OCCLUSION_CAPTURE_DIRECTORY` to an absolute disposable output directory, then run `EvoEngine_Tests.exe --gtest_filter=GiOcclusion.PartitionedRoomComparison`. The fixture is skipped without that environment variable. It prints `GI_OCCLUSION` observations and writes the twelve PNGs. It asserts provider readiness and finite samples without making an arbitrary image-improvement threshold a test requirement.

### Observations

Historical run before the checkbox change (HDDAGI minimum visibility 0.1/0.01, not off/on), on RTX 5070, driver 595.71, with Vulkan core/synchronization validation enabled:

| Variant | Closed partition mean | Open control mean |
|---|---:|---:|
| SDFGI occlusion off | 0.813645 | 0.493222 |
| SDFGI occlusion on | 0.342025 | 0.594100 |
| HDDAGI minimum visibility 0.1 | 0.925590 | 0.556829 |
| HDDAGI minimum visibility 0.01 | 0.938616 | 0.553974 |
| DDGI moment bias 0.02 | 0.0191258 | 0.490473 |
| DDGI moment bias 0 | 0.0191259 | 0.490563 |

SDFGI occlusion reduces the closed-region signal by about 58%, while its open control remains bright. HDDAGI's smaller visibility floor does not improve this fixture; the closed room remains substantially illuminated. DDGI already rejects most illumination across the partition here, and changing moment bias has negligible effect. The captures also show residual DDGI corner/edge artifacts. These are observations of a synthetic scene; the small HDDAGI difference is not proof of a general causal trend because temporal sampling phase can differ across fresh fields.

This earlier experiment did not test zero HDDAGI bias. The user's Sponza result at zero motivates the checkbox below; the historical small-bias comparison does not establish its behavior. DDGI retains its 0.02 default and existing moment visibility.

A standalone repeat produced the same qualitative finding. Closed means, in the table's order, were 0.817379, 0.342028, 0.925510, 0.938616, 0.0191258 and 0.0191259; open means were 0.492725, 0.596475, 0.556749, 0.609908, 0.489306 and 0.490325. The open HDDAGI 0.01 result varied noticeably, reinforcing that these are finite-frame observations rather than converged ground truth. Local logs and 24 images are in `tasks/refinement-tests.log`, `tasks/indoor-repeat.log`, `tasks/indoor-occlusion` and `tasks/indoor-occlusion-repeat`.

## Validation

- Rebuilt the test executable using `cmake --build out/build/vs2026-x64-tests --config RelWithDebInfo --target EvoEngine_Tests --parallel 8`. The initial run exposed one stale half-size assertion; it was corrected to require the actual full viewport. The final 30 focused tests passed, followed by the standalone indoor fixture passing. Core/synchronization validation reported no findings; existing unused shader-interface performance warnings remain. Logs/XML: `tasks/refinement-final-tests.*` and `tasks/indoor-repeat.*`. The broad repository suite and image goldens were not rerun for this change.
- Test/capture executable: `C:/Users/lllll/Documents/GitHub/EvoEngine/out/build/vs2026-x64-tests/EvoEngine_Tests/RelWithDebInfo/EvoEngine_Tests.exe`. No shader ABI or shader source changed; the runtime GPU fixtures exercise the existing full-resolution paths.
- Installed RT-disabled Sponza capture with `Scripts/capture_hddagi_transport.py --module-dir out/install/vs2026-x64/python --resources tasks/refinement-capture-resources --output tasks/refinement-installed --frames 120 --beauty` passed and was visually inspected. Its manifest confirms viewport and GI both 2560×1440, ready transport and zero failure flags. This installed run uses the ordinary validation-OFF production build; it is separate from the validation-enabled test runs.
- Editor built before the installed capture: `C:/Users/lllll/Documents/GitHub/EvoEngine/out/build/vs2026-x64/EvoEngine_App/RelWithDebInfo/EvoEngineEditor.exe`. All applications installed successfully with `python Scripts/install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --jobs 8` (`tasks/refinement-install-closeout.log`). Installed editor: `C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/bin/EvoEngineEditor.exe`; installed Python runtime: `C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/python`.
- Pinned C++ formatting and Python compilation checks passed. Active source references to the removed setting are limited to the legacy-loading regression test; historical result records retain their original settings.


## HDDAGI Use Occlusion

The editor exposes **Use Occlusion**, enabled by default. Enabled applies visibility without a minimum floor (the former bias of 0); disabled uses visibility 1 and skips both gather occlusion texture samples. Occlusion generation and storage remain available for immediate toggling and other probe operations. No measured performance improvement is claimed.

C++/Python and YAML expose `use_occlusion`. Legacy `occlusion_bias` values below 1 migrate to enabled, and values at least 1 migrate to disabled; intermediate strengths intentionally become binary. An explicit `use_occlusion` overrides the legacy key. New saves omit `occlusion_bias`. The gather metadata retains its 320-byte layout, replacing the float with a 32-bit flag.


Checkbox validation: all 31 `Hddagi*.*:GiSettings.*:GiProbes.*` tests pass (`tasks/gi-refinement-hddagi.log/.xml`), including zero-visibility occlusion on/off, constant radiance, reflection capture, settings defaults, legacy migration and round-trip persistence. Both gather ABI variants and the camera gather compile and pass SPIR-V validation (`tasks/checkbox-shaders.log`). GPU tests enable Vulkan core/synchronization validation and report no findings. All apps installed successfully with `python Scripts/install_apps.py --preset vs2026-x64 --config RelWithDebInfo --incremental --no-open --jobs 8` (`tasks/gi-refinement-install-closeout.log`).
