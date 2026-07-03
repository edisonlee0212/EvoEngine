#pragma once

#include "OptiXRayTracer.hpp"
#include "memory"
#include <unordered_map>

#include "VulkanInterlop.hpp"

namespace evo_engine {
class Semaphore;
class Image;
class Texture2D;
class Cubemap;
class RenderTexture;
}  // namespace evo_engine

struct cudaGraphicsResource;
namespace evo_engine {
class CudaModule {
#pragma region Class related

  CudaModule() = default;

  CudaModule(CudaModule &&) = default;

  CudaModule(const CudaModule &) = default;

  CudaModule &operator=(CudaModule &&) = default;

  CudaModule &operator=(const CudaModule &) = default;

#pragma endregion
  void *optix_handle_ = nullptr;
  bool initialized_ = false;
  std::unique_ptr<OptiXRayTracer> ray_tracer_;
  std::unordered_map<uint64_t, std::pair<uint32_t, std::shared_ptr<CudaImage>>> texture_2d_cache_;

  friend class RayTracerLayer;

 public:
  static std::unique_ptr<OptiXRayTracer> &GetRayTracer();

  static CudaModule &GetInstance();

  static void Init();

  static void Terminate();

  static void EstimateIlluminationRayTracing(const EnvironmentProperties &environmentProperties,
                                             const RayProperties &rayProperties,
                                             std::vector<IlluminationSampler<glm::vec3>> &lightProbes, unsigned seed,
                                             float pushNormalDistance);

  static void EstimateIlluminationRayTracingSpectral(const EnvironmentProperties &environmentProperties,
                                                     const RayProperties &rayProperties,
                                                     std::vector<IlluminationSampler<glm::vec3>> &lightProbes,
                                                     unsigned seed, float pushNormalDistance);

  static void SamplePointCloud(const EnvironmentProperties &environmentProperties,
                               std::vector<PointCloudSample> &samples);
  static std::shared_ptr<CudaImage> ImportTexture2D(const std::shared_ptr<evo_engine::Texture2D> &texture2D);
  static std::shared_ptr<CudaImage> ImportCubemap(const std::shared_ptr<evo_engine::Cubemap> &cubemap);
  static std::shared_ptr<CudaImage> ImportRenderTexture(
      const std::shared_ptr<evo_engine::RenderTexture> &renderTexture);
  static std::shared_ptr<CudaSemaphore> ImportSemaphore(const std::shared_ptr<evo_engine::Semaphore> &semaphore);
};
}  // namespace evo_engine
