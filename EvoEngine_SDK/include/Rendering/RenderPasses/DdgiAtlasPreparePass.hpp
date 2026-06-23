#pragma once
#include "RenderGraph.hpp"

namespace evo_engine {
class DdgiAtlasPreparePass final {
 public:
  struct Parameters {
    bool clear_persistent_resources = false;
    float* record_time_ms = nullptr;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context, const Parameters& parameters);
};
}  // namespace evo_engine
