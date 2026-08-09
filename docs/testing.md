# Testing EvoEngine

[Back to README](../README.md)

EvoEngine uses CTest for local test execution. The test suite includes C++ unit tests, render/GPU tests, Python-driven render capture tests, app smoke tests, and Windows launcher smoke tests.

## Common Commands

Local render/GPU tests:

```bat
python Scripts\test.py
```

All CTest tests:

```bat
python Scripts\test.py --all
```

`Scripts\test.py` configures test builds with a local install prefix under `out\install\<build-dir-name>` unless
`--cmake-arg -DCMAKE_INSTALL_PREFIX=...` is provided. This keeps runtime package post-build deploy steps out of system
locations such as `Program Files`.

List tests:

```bat
python Scripts\test.py --list
```

Run CTest directly:

```bat
ctest --test-dir out/build/vs2026-x64 -C RelWithDebInfo --output-on-failure
```

Run focused launcher tests:

```bat
ctest --test-dir out/build/vs2026-x64 -C RelWithDebInfo -R "Launcher" --output-on-failure
```

## Render Tests

`python Scripts/test.py` defaults to render/GPU-labeled tests. It builds the `EvoEngine_RenderTests` aggregate target and runs tests such as:

- Rendering demo rasterization golden-image comparison
- Rendering demo RTX path-tracing golden-image comparison at 1280x720, 2048 SPP, and 3 bounces
- Rendering demo RayQuery golden-image comparison at 1280x720, 2048 SPP, and 3 bounces
- Python render capture workflows
- launcher/editor smoke coverage for project startup and the Rendering demo profile

The two ray-camera goldens accumulate 8 samples per frame for 256 frames. Requested ray backends must be available;
the tests reject normal camera fallback so a raster image cannot be accepted as a ray baseline.

Visual artifacts are written under:

```text
out/test-artifacts/latest/
```

Render comparisons report PSNR and SSIM when applicable.

## CI Scope

GitHub Actions run repository-wide format checks and platform compilation checks. Rendering tests are intentionally local-only because they require a Vulkan-capable GPU environment and produce visual artifacts for inspection.

## Test Organization Notes

Cheap unit tests should stay isolated as normal CTest/GTest cases. GUI and process smoke tests can group multiple named subtests inside one launched process when the checks share a lifecycle and can report clear per-step failures. Process-boundary tests should remain separate when their purpose is to verify startup, shutdown, or child process behavior.
