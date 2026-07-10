#pragma once
#include "RenderGraph.hpp"

namespace evo_engine {
class Camera;

class PostProcessingPass final {
 public:
  struct Parameters {
    std::shared_ptr<Camera> camera;
    RenderGraphTransientResourceStore* transient_resources = nullptr;
    bool immediate = false;
    bool ray_camera = false;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(const char* dependency);
  [[nodiscard]] static RenderPassDescriptor CreateRayTracingDescriptor(const char* dependency);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
