#include "FoliageDescriptor.hpp"
#include "TreeModel.hpp"
using namespace eco_sys_lab;

void FoliageDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "leaf_size" << YAML::Value << leaf_size;
  out << YAML::Key << "leaf_count_per_internode" << YAML::Value << leaf_count_per_internode;
  out << YAML::Key << "position_variance" << YAML::Value << position_variance;
  out << YAML::Key << "rotation_variance" << YAML::Value << rotation_variance;
  out << YAML::Key << "branching_angle" << YAML::Value << branching_angle;
  out << YAML::Key << "max_node_thickness" << YAML::Value << max_node_thickness;
  out << YAML::Key << "min_root_distance" << YAML::Value << min_root_distance;
  out << YAML::Key << "max_end_distance" << YAML::Value << max_end_distance;
  out << YAML::Key << "horizontal_tropism" << YAML::Value << horizontal_tropism;
  out << YAML::Key << "gravitropism" << YAML::Value << gravitropism;
  leaf_material.Save("leaf_material", out);
}

void FoliageDescriptor::Deserialize(const YAML::Node& in) {
  if (in["leaf_size"])
    leaf_size = in["leaf_size"].as<glm::vec2>();
  if (in["leaf_count_per_internode"])
    leaf_count_per_internode = in["leaf_count_per_internode"].as<int>();
  if (in["position_variance"])
    position_variance = in["position_variance"].as<float>();
  if (in["rotation_variance"])
    rotation_variance = in["rotation_variance"].as<float>();
  if (in["branching_angle"])
    branching_angle = in["branching_angle"].as<float>();
  if (in["max_node_thickness"])
    max_node_thickness = in["max_node_thickness"].as<float>();
  if (in["min_root_distance"])
    min_root_distance = in["min_root_distance"].as<float>();
  if (in["max_end_distance"])
    max_end_distance = in["max_end_distance"].as<float>();
  if (in["horizontal_tropism"])
    horizontal_tropism = in["horizontal_tropism"].as<float>();
  if (in["gravitropism"])
    gravitropism = in["gravitropism"].as<float>();
  leaf_material.Load("leaf_material", in);
}

bool FoliageDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;

  if (ImGui::DragFloat2("Leaf size", &leaf_size.x, 0.001f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Leaf per node", &leaf_count_per_internode, 1, 0, 50))
    changed = true;
  if (ImGui::DragFloat("Position variance", &position_variance, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Rotation variance", &rotation_variance, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Branching angle", &branching_angle, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Max node thickness", &max_node_thickness, 0.001f, 0.0f, 5.0f))
    changed = true;
  if (ImGui::DragFloat("Min root distance", &min_root_distance, 0.01f, 0.0f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Max end distance", &max_end_distance, 0.01f, 0.0f, 10.0f))
    changed = true;

  changed = ImGui::DragFloat("Horizontal Tropism", &horizontal_tropism, 0.001f, 0.0f, 1.0f) || changed;
  changed = ImGui::DragFloat("Gravitropism", &gravitropism, 0.001f, 0.0f, 1.0f) || changed;
  if (editorLayer->DragAndDropButton<Material>(leaf_material, "Leaf Material"))
    changed = true;
  return changed;
}

void FoliageDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  if (leaf_material.Get<Material>())
    list.push_back(leaf_material);
}

void FoliageDescriptor::GenerateFoliageMatrices(std::vector<glm::mat4>& matrices, const SkeletonNodeInfo& internodeInfo,
                                                const float treeSize) const {
  if (internodeInfo.thickness < max_node_thickness && internodeInfo.root_distance > min_root_distance &&
      internodeInfo.end_distance < max_end_distance) {
    for (int i = 0; i < leaf_count_per_internode * internodeInfo.leaves; i++) {
      const auto leafSize = leaf_size * treeSize * 0.1f;
      glm::quat rotation = internodeInfo.global_rotation *
                           glm::quat(glm::radians(glm::vec3(glm::gaussRand(0.0f, rotation_variance), branching_angle,
                                                            glm::linearRand(0.0f, 360.0f))));
      auto front = rotation * glm::vec3(0, 0, -1);
      auto up = rotation * glm::vec3(0, 1, 0);
      TreeModel::ApplyTropism(glm::vec3(0, -1, 0), gravitropism, front, up);
      const auto horizontalDirection = glm::vec3(front.x, 0.0f, front.z);
      if (glm::length(horizontalDirection) > glm::epsilon<float>()) {
        TreeModel::ApplyTropism(glm::normalize(horizontalDirection), horizontal_tropism, front, up);
      }
      auto foliagePosition =
          glm::mix(internodeInfo.global_position, internodeInfo.GetGlobalEndPosition(), glm::linearRand(0.f, 1.f)) +
          front * (leafSize.y + glm::linearRand(0.0f, position_variance) * treeSize * 0.1f);
      if (glm::any(glm::isnan(foliagePosition)) || glm::any(glm::isnan(front)) || glm::any(glm::isnan(up)))
        continue;
      const auto leafTransform = glm::translate(foliagePosition) * glm::mat4_cast(glm::quatLookAt(front, up)) *
                                 glm::scale(glm::vec3(leafSize.x, 1.0f, leafSize.y));
      matrices.emplace_back(leafTransform);
    }
  }
}
