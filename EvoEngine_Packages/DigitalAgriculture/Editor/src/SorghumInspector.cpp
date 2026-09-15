#include "BtfMaterial.hpp"
#include "BtfMeshRenderer.hpp"
#include "CBTFGroup.hpp"
#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureInspectorStates.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumLayer.hpp"
using namespace digital_agriculture_package;
bool SorghumInspector::Inspect(InspectorContext& context, Sorghum& sorghum) {
  if (preview_owner.lock().get() != &sorghum) {
    ui_previous_referenced_entity = {};
    ui_node_debug_info_list.reset();
    preview_owner = sorghum.GetScene()->GetPrivateComponent(sorghum.GetOwner(), "Sorghum");
  }
  const auto& editor_layer = context.editor_layer;
  auto& sorghum_generator = sorghum.sorghum_generator;
  auto& sorghum_growth_stages = sorghum.sorghum_growth_stages;
  auto& sorghum_state = sorghum.sorghum_state;
  auto& sorghum_descriptor = sorghum.sorghum_descriptor;
  bool changed = false;
  if (editor_layer->DragAndDropButton<SorghumGenerator>(sorghum_generator, "SorghumGenerator"))
    changed = true;
  if (editor_layer->DragAndDropButton<SorghumGrowthStages>(sorghum_growth_stages, "SorghumGrowthStages"))
    changed = true;

  if (editor_layer->DragAndDropButton<SorghumState>(sorghum_state, "SorghumState"))
    changed = true;

  if (editor_layer->DragAndDropButton<SorghumDescriptor>(sorghum_descriptor, "SorghumDescriptor"))
    changed = true;

  if (ImGui::Button("Form meshes")) {
    sorghum.GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
  }

  if (const auto ssg = sorghum_generator.Get<SorghumGenerator>()) {
    if (ImGui::TreeNode("Sorghum Descriptor settings")) {
      auto& seed = ui_seed;
      if (ImGui::DragInt("Seed", &seed)) {
        auto sd = sorghum_descriptor.Get<SorghumDescriptor>();
        if (!sd) {
          sd = AssetManager::CreateTemporaryAsset<SorghumDescriptor>();
          sorghum_descriptor = sd;
        }
        ssg->Apply(sd, seed);
        sorghum.GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
      }
      ImGui::TreePop();
    }
  }
  if (const auto sgs = sorghum_growth_stages.Get<SorghumGrowthStages>()) {
    if (ImGui::TreeNode("Sorghum Growth Descriptor settings")) {
      auto& time = ui_time;
      if (ImGui::SliderFloat("Time", &time, 0.0f, sgs->GetCurrentEndTime())) {
        time = glm::clamp(time, 0.0f, sgs->GetCurrentEndTime());
        auto sd = sorghum_descriptor.Get<SorghumDescriptor>();
        if (!sd) {
          sd = AssetManager::CreateTemporaryAsset<SorghumDescriptor>();
          sorghum_descriptor = sd;
        }
        sgs->Apply(sd, time);
        sorghum.GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
      }
      ImGui::TreePop();
    }
  }
  auto& debug_rendering = ui_debug_rendering;
  ImGui::Checkbox("Debug", &debug_rendering);
  if (debug_rendering) {
    auto& node_render_size = ui_node_render_size;
    if (ImGui::TreeNode("Debug settings")) {
      ImGui::DragFloat("Node size", &node_render_size, 0.01f, 0.0f, 1.f);
      ImGui::TreePop();
    }
    auto& previous_referenced_entity = ui_previous_referenced_entity;
    auto& node_debug_info_list = ui_node_debug_info_list;
    if (!node_debug_info_list)
      node_debug_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
    constexpr bool show_all_node = false;
    if (show_all_node) {
      if (const auto sd = sorghum_descriptor.Get<SorghumDescriptor>()) {
        std::vector<ParticleInfo> particle_infos;
        const auto owner = sorghum.GetOwner();
        const auto scene = sorghum.GetScene();
        const auto plant_position = scene->GetDataComponent<GlobalTransform>(owner).GetPosition();
        for (const auto& leaf_state : sd->leaves) {
          const auto start_index = particle_infos.size();
          particle_infos.resize(start_index + leaf_state.spline.segments.size());
          for (int i = 0; i < leaf_state.spline.segments.size(); i++) {
            auto& matrix = particle_infos[start_index + i].instance_matrix;
            matrix.value = glm::translate(leaf_state.spline.segments.at(i).position + plant_position) *
                           glm::scale(glm::vec3(node_render_size * leaf_state.spline.segments.at(i).radius));
            particle_infos[start_index + i].instance_color =
                glm::vec4((leaf_state.index % 3) * 0.5f, ((leaf_state.index / 3) % 3) * 0.5f,
                          ((leaf_state.index / 9) % 3) * 0.5f, 1.0f);
          }
        }
        node_debug_info_list->SetParticleInfos(particle_infos);
      }
    } else {
      const auto owner = sorghum.GetOwner();
      if (ImGui::Button("Refresh leaf nodes") || previous_referenced_entity != owner) {
        if (const auto sd = sorghum_descriptor.Get<SorghumDescriptor>()) {
          std::vector<ParticleInfo> particle_infos;

          const auto scene = sorghum.GetScene();
          const auto plant_position = scene->GetDataComponent<GlobalTransform>(owner).GetPosition();
          for (const auto& leaf_state : sd->leaves) {
            SorghumSpline leaf_part;
            leaf_part.segments = leaf_state.spline.GetLeafPart();
            const auto segments = leaf_part.RebuildFixedSizeSegments(8);
            const auto start_index = particle_infos.size();
            particle_infos.resize(start_index + segments.size());
            for (int i = 0; i < segments.size(); i++) {
              auto& matrix = particle_infos[start_index + i].instance_matrix;
              matrix.value = glm::translate(segments.at(i).position + plant_position) *
                             glm::scale(glm::vec3(node_render_size * segments.at(i).radius));
              particle_infos[start_index + i].instance_color =
                  glm::vec4((leaf_state.index % 3) * 0.5f, ((leaf_state.index / 3) % 3) * 0.5f,
                            ((leaf_state.index / 9) % 3) * 0.5f, 1.0f);
            }
          }
          node_debug_info_list->SetParticleInfos(particle_infos);
        }
      }
    }
    editor_layer->DrawGizmoCubes(node_debug_info_list);
  }

  return changed;
}
