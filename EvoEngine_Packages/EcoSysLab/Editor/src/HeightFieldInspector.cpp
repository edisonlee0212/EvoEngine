#include "EcoSysLabObjectInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "EditorTextureRegistry.hpp"
#include "SDKInspectionAdapters.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;

bool HeightFieldInspector::Inspect(InspectorContext& context, HeightField& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  changed = ImGui::DragInt("Precision level", &target.precision_level) || changed;

  ImGui::Checkbox("Show noise graph", &show_noise_graph);
  if (show_noise_graph) {
    changed =
        evo_engine::DrawProceduralNoiseGraph(target.noises_graph, "Height field noise graph", editor_layer) | changed;
  }
  if (ImGui::DragFloat2("Position offset", &target.position_offset.x, 0.1f)) {
    changed = true;
  }
  if (ImGui::TreeNode("Visualization")) {
    bool resolution_changed = false;
    if (ImGui::DragInt("Resolution", &resolution, 1, 16, 1024)) {
      resolution = glm::clamp(resolution, 16, 1024);
      resolution_changed = true;
    }
    if (ImGui::DragFloat("Position scale", &position_scale, 0.1f)) {
      resolution_changed = true;
    }

    if (!test_texture_2d) {
      test_texture_2d = AssetManager::CreateTemporaryAsset<Texture2D>();
    }
    if (show_test_texture) {
      if (changed || resolution_changed || target.GetHandle() != current_handle) {
        current_handle = target.GetHandle();
        std::vector<glm::vec4> color(resolution * resolution);
        Jobs::RunParallelFor(resolution * resolution, [&](size_t i) {
          float x = i / resolution;
          float y = i % resolution;
          x /= resolution;
          y /= resolution;
          color[i] =
              glm::vec4(glm::vec3(target.GetValue(glm::vec2(x, y) * position_scale + target.position_offset)), 1.0f);
        });

        test_texture_2d->SetRgbaChannelData(color, glm::uvec2(resolution));
      }

      const auto texture_storage = test_texture_2d->PeekTexture2DStorage();
      if (const auto texture_id = EditorTextureRegistry::GetTextureId(texture_storage.image, texture_storage.image_view,
                                                                      texture_storage.sampler)) {
        ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 10.0f);
        debug_scale = glm::clamp(debug_scale, 0.1f, 10.0f);
        ImGui::Image(texture_id,
                     ImVec2(texture_storage.image->GetExtent().width * debug_scale,
                            texture_storage.image->GetExtent().height * debug_scale),
                     ImVec2(0, 1), ImVec2(1, 0));
      }
    }
    ImGui::TreePop();
  }
  return changed;
}
