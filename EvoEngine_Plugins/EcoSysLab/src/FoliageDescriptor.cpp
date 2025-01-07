#include "FoliageDescriptor.hpp"
#include "TreeModel.hpp"
using namespace eco_sys_lab_plugin;

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
  leaf_material_ref.Save("leaf_material_ref", out);
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
  leaf_material_ref.Load("leaf_material_ref", in);
}

bool FoliageDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
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
  if (editor_layer->DragAndDropButton<Material>(leaf_material_ref, "Leaf Material"))
    changed = true;
  return changed;
}

void FoliageDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  if (leaf_material_ref.Get<Material>())
    list.push_back(leaf_material_ref);
}

std::shared_ptr<Texture2D> FoliageDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = ProjectManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/FoliageDescriptor.png"));
  }
  return thumbnail;
}

void FoliageDescriptor::GenerateFoliageMatrices(std::vector<glm::mat4>& matrices,
                                                const SkeletonNodeInfo& internode_info, const float tree_size) const {
  if (internode_info.thickness <= max_node_thickness && internode_info.root_distance >= min_root_distance &&
      internode_info.end_distance <= max_end_distance) {
    for (int i = 0; i < leaf_count_per_internode * internode_info.leaves; i++) {
      const auto current_leaf_size = leaf_size * tree_size * 0.1f;
      glm::quat rotation = internode_info.global_rotation *
                           glm::quat(glm::radians(glm::vec3(glm::gaussRand(0.0f, rotation_variance), branching_angle,
                                                            glm::linearRand(0.0f, 360.0f))));
      auto front = rotation * glm::vec3(0, 0, -1);
      auto up = rotation * glm::vec3(0, 1, 0);
      TreeModel::ApplyTropism(glm::vec3(0, -1, 0), gravitropism, front, up);
      if (const auto horizontal_direction = glm::vec3(front.x, 0.0f, front.z);
          glm::length(horizontal_direction) > glm::epsilon<float>()) {
        TreeModel::ApplyTropism(glm::normalize(horizontal_direction), horizontal_tropism, front, up);
      }
      auto foliage_position =
          glm::mix(internode_info.global_position, internode_info.GetGlobalEndPosition(), glm::linearRand(0.f, 1.f)) +
          front * (current_leaf_size.y + glm::linearRand(0.0f, position_variance) * 0.1f);
      if (glm::any(glm::isnan(foliage_position)) || glm::any(glm::isnan(front)) || glm::any(glm::isnan(up)))
        continue;
      const auto leaf_transform = glm::translate(foliage_position) * glm::mat4_cast(glm::quatLookAt(front, up)) *
                                  glm::scale(glm::vec3(current_leaf_size.x, 1.0f, current_leaf_size.y));
      matrices.emplace_back(leaf_transform);
    }
  }
}
