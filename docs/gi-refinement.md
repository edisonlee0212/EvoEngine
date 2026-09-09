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


## Relocated SDFGI corner reconstruction

The reported target is the left foreground Sponza pillar base and its adjoining floor seam. Raw-irradiance diagnostics confirmed missing GI rather than material AO alone. A coverage map did not capture the full problem: some pixels retained valid but dark probes while rejecting nearby illuminated probes. Camera reconstruction now includes relocated neighbors outside the original eight grid corners, with normalized support widened by the maximum relocation distance (1 + 0.45 probe intervals). A bounded local receiver escape also handles grazing rays that cannot leave the filtered surface footprint within the original ray-distance limit. Receiver bias and visibility now use the geometric normal, transformed inversely for the anisotropic field; the shading normal still controls irradiance and BRDF evaluation. The strict hemisphere and subsequent obstruction checks remain; no minimum visibility or radiance floor is introduced. Relocation-off behavior is unchanged.

The RT-disabled GPU regression now checks a lit relocated probe beyond the original eight corners: old reconstruction returns zero; the new gather returns constant unit radiance. Placing a wall between it and the receiver still returns zero. The new grazing-ray case fails on the original visibility shader in all eight fixture phases and passes with the correction. Additional cases verify a tilted shading normal cannot incorrectly reject a physically visible probe, including nonuniform vertical scale. Existing occupied-field, opposite-hemisphere, nearby-wall, scrolling, history, reflection-capture and finite-output checks remain active.

### Installed Sponza comparison

Both runs use the same installed SDK DLL, fresh copies of tracked Rendering assets, the main raster camera at 2560x1440, 1,800 settling frames, followed by 120 measured GPU frames and four readback-drain frames. RT pipeline, ray query, BLAS and TLAS are disabled. The before run uses the `f007b802` SDFGI modules; the after run uses the corrected modules. Manifests record the actual installed DLL and shader hashes, effective provider, transport status and raw timing samples. Both runs report Automatic SDFGI with ready transport and zero failure flags. These production captures use validation OFF; validation-enabled GPU fixtures are separate.

| Observation | Before | After |
|---|---:|---:|
| Zero-RGB diffuse PNG pixels in pillar/floor region | 3,929 | 115 |
| Diffuse PNG pixels with every RGB channel below 5 | 4,011 | 250 |
| DeferredCamera GPU median / p95 (ms) | 4.680 / 4.979 | 11.159 / 11.902 |
| Whole instrumented GPU span median / p95 (ms) | 21.342 / 22.005 | 27.846 / 28.657 |

The region is x=[300,710), y=[1300,1410) in the 2560x1440 diffuse PNG; alpha is excluded. Zero-valued RGB pixels decrease by 97.1%. The continuous pillar-floor seam is substantially reduced, but a dark contact strip and some black pixels remain: this is not a claim that every corner artifact is eliminated. The wider support smooths illumination more and substantially increases gather cost. This is one stationary comparison, not a broad performance benchmark or exact irradiance reference. Local evidence: `tasks/relocation-release-before`, `tasks/relocation-verified-after`, their logs, and `tasks/capture-relocation-final.py`. Earlier exploratory captures used different settling endpoints and are not the table's measurements.

### Validation and delivery

Forty distinct focused tests pass: the 31 HDDAGI/shared-setting tests above, eight SDFGI relocation/gather/lighting tests, and the unchanged `RenderingDemo.SdfgiGoldenImage`. Final logs/XML are `tasks/gi-refinement-verified-tests.*`. A late atlas-read optimization failed the unchanged golden (28.55 dB PSNR); the original shaders passed on the same rebuilt runtime. Removing that optimization restored the golden (58.34 dB in the isolated check). It is excluded from the delivered change; neither baseline nor thresholds were modified. The new missing-neighbor regression's before failure is recorded in `tasks/relocation-support-before.log`; the grazing failure is in `tasks/checkbox-regression-before.log`. Vulkan core/synchronization validation reports no errors or hazards. Three HDDAGI gather variants, all three deferred provider variants, and both SDFGI direct-light variants compile and pass standalone SPIR-V validation with the runtime scalar block layout. Final deferred checks are in `tasks/gi-refinement-verified-shaders.log`. The broad repository suite was not rerun.

The relevant test executable was rebuilt before captures: `C:/Users/lllll/Documents/GitHub/EvoEngine/out/build/vs2026-x64-tests/EvoEngine_Tests/RelWithDebInfo/EvoEngine_Tests.exe`. All applications were built and installed with the install command above (`tasks/gi-refinement-final-install.log`). The installed capture runtime is `C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/python`; editor executable: `C:/Users/lllll/Documents/GitHub/EvoEngine/out/install/vs2026-x64/bin/EvoEngineEditor.exe`. The final installed shaders match source, with all diagnostic shader overrides removed. Pinned C++ formatting and Python compilation checks pass.
