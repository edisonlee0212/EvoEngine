#include "BasicFruitDescriptor.hpp"

using namespace eco_sys_lab_plugin;

void BasicFruitDescriptor::PrepareGrowthController(ShootGrowthController& shoot_growth_controller) const {
  shoot_growth_controller.fruit = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                      const SkeletonNode<InternodeGrowthData>& internode) {
    return internode.data.light_intake > fruit_flushing_lighting_requirement;
  };
  shoot_growth_controller.fruit_fall_probability = [&](std::mt19937& random_engine,
                                                       const ShootGrowthData& shoot_growth_data,
                                                       const SkeletonNode<InternodeGrowthData>& internode) {
    return fruit_fall_probability;
  };
}
void BasicFruitDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "fruit_flushing_lighting_requirement" << YAML::Value << fruit_flushing_lighting_requirement;
  out << YAML::Key << "fruit_fall_probability" << YAML::Value << fruit_fall_probability;
  out << YAML::Key << "fruit_distance_to_branch_end_limit" << YAML::Value << fruit_distance_to_branch_end_limit;
}
void BasicFruitDescriptor::Deserialize(const YAML::Node& in) {
  // Structure
  if (in["fruit_flushing_lighting_requirement"])
    fruit_flushing_lighting_requirement = in["fruit_flushing_lighting_requirement"].as<float>();
  if (in["fruit_fall_probability"])
    fruit_fall_probability = in["fruit_fall_probability"].as<float>();
  if (in["fruit_distance_to_branch_end_limit"])
    fruit_distance_to_branch_end_limit = in["fruit_distance_to_branch_end_limit"].as<float>();
}
bool BasicFruitDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNodeEx("Fruit")) {
    changed =
        ImGui::DragFloat("Lighting requirement", &fruit_flushing_lighting_requirement, 0.01f, 0.0f, 1.0f) || changed;
    changed = ImGui::DragFloat("Drop prob", &fruit_fall_probability, 0.01f) || changed;
    changed = ImGui::DragFloat("Distance To End Limit", &fruit_distance_to_branch_end_limit, 0.01f) || changed;
    ImGui::TreePop();
  }
  return changed;
}
void BasicFruitDescriptor::GenerateFruitMatrices(std::vector<glm::mat4>& matrices,
                                                 const SkeletonNodeInfo& internode_info, float tree_size) const {
}