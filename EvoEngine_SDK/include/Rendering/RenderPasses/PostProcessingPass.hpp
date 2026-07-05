#pragma once
#include "RenderGraph.hpp"

namespace evo_engine {
class Camera;

class PostProcessingPass final {
 public:
  struct Parameters {
    std::shared_ptr<Camera> camera;
    bool immediate = false;
    bool tone_mapping_only = false;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(const char* dependency);
  [[nodiscard]] static RenderPassDescriptor CreateRayTracingDescriptor(const char* dependency);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
