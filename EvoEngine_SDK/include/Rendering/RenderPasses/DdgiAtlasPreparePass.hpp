#pragma once
#include "RenderGraph.hpp"

namespace evo_engine {
class EVOENGINE_API DdgiAtlasPreparePass final {
 public:
  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Execute(const RenderGraphExecutionContext& context);
};
}  // namespace evo_engine
