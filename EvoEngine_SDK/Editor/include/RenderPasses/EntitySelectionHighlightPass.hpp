#pragma once

#include "EvoEngineEditorAPI.hpp"

#include "EntitySelectionHighlight.hpp"
#include "RenderGraph.hpp"

#include <glm/glm.hpp>

namespace evo_engine {
class EVOENGINE_API Camera;
class EVOENGINE_API GraphicsPipeline;

struct EntitySelectionHighlightPushConstant {
  glm::vec4 outline_color{1.0f, 0.75f, 0.0f, 1.0f};
  glm::vec4 settings{0.0f};
};

class EVOENGINE_EDITOR_API EntitySelectionHighlightPass final {
 public:
  struct Parameters {
    std::shared_ptr<Camera> camera;
    std::shared_ptr<GraphicsPipeline> pipeline;
    EntitySelectionHighlight::Snapshot presentation;
  };

  [[nodiscard]] static RenderPassDescriptor CreateDescriptor();
  static void Record(VkCommandBuffer command, const RenderGraphExecutionContext& context, const Parameters& parameters);
};

}  // namespace evo_engine
