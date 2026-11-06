# GoPro RAW sky preview

`GprToDng` uses the repository's GoPro SDK to decompress VC-5 Bayer data into a DNG while retaining parsed camera metadata. It does not make a gamma-encoded RGB preview. Build the engine's GoPro libraries first, then:

```powershell
cmake -S Tools/GprToDng -B out/gopro-sky/gpr-build-x64 -G "Visual Studio 18 2026" -A x64 -T host=x64 -DCMAKE_INSTALL_PREFIX=D:/Sorghum-Vulkan/out/gopro-sky/tools
cmake --build out/gopro-sky/gpr-build-x64 --config RelWithDebInfo
cmake --install out/gopro-sky/gpr-build-x64 --config RelWithDebInfo
```

Install `rawpy`, `numpy`, and `Pillow` for Python 3.12 (the engine Python ABI). In this workspace they are isolated in `out/gopro-sky/dependencies`. Render the selected image with:

```powershell
$env:PYTHONPATH="D:/Sorghum-Vulkan/out/gopro-sky/dependencies"
& "C:/Users/sdjkl/AppData/Local/Programs/Python/Python312/python.exe" Scripts/capture_gopro_soybean.py
```

The script reuses `D:/UIUCGoProCamera/convert_fisheye_sky_to_latlong.py` for projection and RGBE writing. All generated files are under `out/gopro-sky/G1706994`; the original photo folder is untouched. `--prepare-only` skips the engine, and `--capture-only` reuses the converted HDR/settings. `--intensity`, `--rotation` (degrees), and `--samples` control the preview. The source camera is assumed to point at zenith with a 180-degree equidistant fisheye and the recorded pixel circle; these are provisional, not a lens/level calibration. Rotation is relative to the image, not surveyed geographic north.

The Python API `ConfigureCurrentSceneCudaEnvironmentMapForCapture(hdr_path, intensity=1, rotation_degrees=0, gamma=2.2)` selects the scene environment for both incoming OptiX radiance and the visible sky. OptiX reads the unfiltered environment cubemap rather than a baked reflection probe. No separate procedural sun is added. The preview uses an explicitly unmeasured lower-hemisphere RGB of (0.02, 0.025, 0.02), matching the original converter's default distant-ground approximation and avoiding a black strip beyond the finite soil mesh. Local soil reflection is still ray traced. `--ground-color 0 0 0` disables the distant-ground approximation.

The preview uses a 4 × 10 narrow-leaf field with 30 in row spacing and 5 in seeding spacing, retained soil, an environment intensity multiplier of 2 (visual gain, not calibration), 512 image samples/pixel and 2 bounces. It saves a separate `GeneratedAssets/Scenes/Soybean_G1706994_HDR_Preview.evescene`, not the study's original scene. Scene serialization retains the source HDR texture via its temporary assets. On interactive reload, choose **Scene** as the OptiX environment type if the editor's layer settings override it.

This is **not** an absolute radiometric HDR measurement. It is a single exposure with linear 16-bit demosaicing, no automatic brightening, camera white balance, and no exposure/ISO normalization. Saturated sun values are not recovered. The current OptiX sampler is not yet environment-importance-sampled; increasing preview samples does not establish convergence for plant illumination experiments. No field-search measurements are run by this script.
