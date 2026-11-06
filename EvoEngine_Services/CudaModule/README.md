# CudaModule Service

[Back to Service index](../README.md)

CudaModule provides CUDA and OptiX-based acceleration for specialized rendering, ray tracing, illumination estimation, point cloud scanning, and related workflows.

## Build Status

- Present in `EvoEngine_Services/CudaModule`.
- Disabled by default through `ServiceInfo.cmake`.
- If registered, its CMake requires CUDA and OptiX; the Service returns early when either is unavailable.
- Builds as `CudaModuleService`.
- Defines `CUDA_MODULE_SERVICE`.

## Main Responsibilities

- OptiX ray tracing infrastructure.
- CUDA buffer and math utilities.
- Camera rendering PTX.
- Illumination estimation PTX.
- Point cloud scanning PTX.
- Vulkan/CUDA interop support.
- Ray tracer layer and camera abstractions.
- BTF material/rendering support.
- Light probe and triangle illumination helpers.

## Main Entry Points

| Source | Role |
| --- | --- |
| `RayTracerLayer` | Layer for CUDA/OptiX ray tracing workflows. |
| `OptiXRayTracer` | OptiX tracing implementation. |
| `CUDAModule` | CUDA module support code. |
| `RayTracerCamera` | Camera abstraction for ray tracing. |
| `BasicPointCloudScanner` | Utility for CUDA-assisted point cloud scanning. |
| `TriangleIlluminationEstimator` | Illumination estimation helper. |
| `BtfMaterial` / `BtfMeshRenderer` | BTF material/rendering support. |

## SDK Integration

CudaModule plugs into layers, rendering, camera workflows, point cloud generation, and optional DigitalAgriculture illumination tools. Other code checks `CUDA_MODULE_SERVICE` before using these APIs.

In **Scene** lighting mode, the resolved environment intensity scales the sky seen by rays.
Camera paths and illumination probes that hit an emissive surface use the material's emitted radiance
independently of the raster diffuse fallback intensity.

## Scene (RT) volumetric clouds

In DigitalAgricultureApp, open **Ray Tracer Layer > Scene (RT) Window** and look toward the sky
in **Color** mode. Under **Cloud settings**, use **Cloud coverage** for sky coverage, **Density**
and **Extinction scale** for opacity, and **Bottom altitude**, **Top altitude** and **Cloud type**
for vertical shape. The other groups control noise, wind, lighting, ray marching and diagnostic views.
Changes reset camera accumulation. The preview uses a fixed cloud time and requires one traceable object.

Use **Broad cloud bank** for connected cloud masses with gaps of blue sky. It sets **Base noise scale** to
0.00045 and **Detail noise scale** to 0.0035, with coverage 0.90, density 2.5 and extinction 0.12.
Smaller noise scales produce larger features in world space. **Scattered clouds** restores the defaults
for those five controls. Both presets also apply to probes when **Clouds affect illumination probes** is enabled.

By default the OptiX camera marches each primary ray up to its first surface hit using the SDK cloud shader compiled
to CUDA and the same four noise fields as Vulkan. The viewport setting affects primary camera radiance.
**Transmittance** in the debug views shows clear paths in white and opaque paths in black.
The optional **Surface shadow preview** applies the SDK's approximate sun-direction cloud shadow
to the camera's surface color. It starts off.

To measure the same clouds, enable **Clouds affect illumination probes** and **Volumetric clouds** in the Ray
Tracer Layer, then run the probe measurement again. The measurement switch defaults to off. Triangle, voxel and PAR
probes march their own world-space ray segments, including reflected segments. Each segment contributes cloud
radiance plus its transmittance times the radiance beyond it. In **Skydome** mode the separately sampled solar
disk also passes through the density field after geometry visibility is tested. Camera resolution, history, debug
views and the surface shadow preview strength are display controls and do not enter the measurement.

In C++, pass the cloud settings to `CudaModule::EstimateIlluminationRayTracing`. Each probe's `energy` holds
total linear RGB irradiance. An optional final `std::vector<glm::vec3>*` receives the direct Skydome sun part of
that same estimate. Subtract it from the total to obtain the remaining sky, cloud and surface contributions.

Under **Cloud resolution and history**, **Cloud resolution** sets each dimension of the OptiX cloud image
to **Full**, **Half** or **Quarter**. Geometry keeps its full resolution. **Temporal reprojection** uses the
previous camera transform and cloud depth to look up last frame's cloud radiance and transmittance.
**History blend** sets the weight of valid history (0 to 0.98), independently of camera accumulation.
Changing cloud settings, lighting or viewport size discards history. The default **Full** with reprojection
off uses the original per-ray march. The other modes render a separate cloud image before the camera pass.
Surfaces in front occlude the cloud, and hits inside the cloud layer march only up to that surface.

The weather map, height profiles, tiled 3D shape and erosion noise, and adaptive volume marching
place this renderer in the same family as Guerrilla's early Nubis cloud layers. This comparison
describes the method. The engine generates its own noise fields and evaluates its own lighting model.
[Guerrilla's production presentation](https://www.guerrilla-games.com/read/nubis-authoring-real-time-volumetric-cloudscapes-with-the-decima-engine)

## Future Work Notes

For photo-based sky previews, `PyEvoEngine.ConfigureCurrentSceneCudaEnvironmentMapForCapture` loads a linear lat-long HDR and selects the **Scene** environment. OptiX imports `EnvironmentalMap.environment_cubemap` (unfiltered incoming radiance), not the scene's baked global reflection probe. The HDR supplies both background and lighting; a separate Skydome sun is not added. See `Tools/GprToDng/README.md` for the GoPro RAW preview workflow and calibration/sampling limitations.

Treat this Service as optional. Code outside the Service should compile without CUDA/OptiX and should guard CUDA-specific includes and registration with `CUDA_MODULE_SERVICE`.
