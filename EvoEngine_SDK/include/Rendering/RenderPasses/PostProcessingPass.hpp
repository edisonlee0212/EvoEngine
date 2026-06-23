#pragma once
#include "RenderGraph.hpp"

namespace evo_engine {
class Camera;

class PostProcessingPass final {
 public:
  struct Parameters {
    std::shared_ptr<Camera> camera;
    bool immediate = false;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor(const char* dependency);
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
