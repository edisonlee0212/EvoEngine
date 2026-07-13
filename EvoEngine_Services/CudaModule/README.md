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
- Camera-rendering Nishita skydome lighting with independently controlled direct-sun intensity, color, and Blender-style
  angular diameter; set the diameter to `0` for hard shadows. Illumination-estimation kernels retain their existing energy
  model.
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

## Future Work Notes

Treat this Service as optional. Code outside the Service should compile without CUDA/OptiX and should guard CUDA-specific includes and registration with `CUDA_MODULE_SERVICE`.
