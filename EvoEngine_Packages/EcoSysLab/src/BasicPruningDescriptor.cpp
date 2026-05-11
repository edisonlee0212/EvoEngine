#include "BasicPruningDescriptor.hpp"

using namespace eco_sys_lab_package;

void BasicPruningDescriptor::PrepareController(const SimulationSettings& simulation_settings,
                                               ShootPruningController& shoot_pruning_controller) const {
  shoot_pruning_controller.breaking_force = [&](std::mt19937& random_engine, const ShootGrowthData& shoot_growth_data,
                                                const SkeletonNode<InternodeGrowthData>& internode) {
    if (branch_strength != 0.f && !internode.IsEndNode() && internode.info.thickness != 0.f &&
        internode.info.length != 0.f) {
      float branch_water_factor = 1.f;
      if (branch_strength_lighting_threshold != 0.f &&
          internode.data.descendant_total_light_intake < branch_strength_lighting_threshold) {
        branch_water_factor = 1.f - branch_strength_lighting_loss;
      }

      return glm::pow(internode.info.thickness / 0.002f, branch_strength_thickness_factor) * branch_water_factor *
             internode.data.strength * branch_strength;
    }
    return FLT_MAX;
  };
  shoot_pruning_controller.internode_strength = [&](std::mt19937& random_engine,
                                                    const ShootGrowthData& shoot_growth_data,
                                                    const SkeletonNode<InternodeGrowthData>& internode) {
    return 1.f;
  };
  shoot_pruning_controller.end_to_base_pruning_factor =
      [&](std::mt19937& random_engine, const glm::mat4&, const ClimateModel&, const VoxelSoilModel&,
          const ShootSkeleton&, const SkeletonNode<InternodeGrowthData>& internode) {
        if (trunk_protection && internode.info.order == 0) {
          return 0.f;
        }
        float pruning_probability = 0.0f;
        if (light_pruning_factor != 0.f) {
          if (internode.IsEndNode()) {
            if (internode.data.light_intake < light_pruning_factor) {
              pruning_probability += 999.f;
            }
          }
        }
        if (internode.data.sagging_stress > 1.) {
          pruning_probability +=
              branch_breaking_multiplier * glm::pow(internode.data.sagging_stress, branch_breaking_multiplier);
        }
        return pruning_probability;
      };
  shoot_pruning_controller.base_to_end_pruning_factor =
      [&](std::mt19937& random_engine, const glm::mat4& global_transform, const ClimateModel& climate_model,
          const VoxelSoilModel& soil_model, const ShootSkeleton& shoot_skeleton,
          const SkeletonNode<InternodeGrowthData>& internode) {
        if (trunk_protection && internode.info.order == 0) {
          return 0.f;
        }

        if (max_flow_length != 0 && max_flow_length < internode.info.chain_index) {
          return 999.f;
        }
        if (const auto max_distance = shoot_skeleton.PeekNode(0).info.end_distance;
            max_distance > 1.f && internode.info.order > 0 &&
            internode.info.root_distance / max_distance < low_branch_pruning) {
          if (const auto parent_handle = internode.GetParentHandle(); parent_handle != -1) {
            const auto& parent = shoot_skeleton.PeekNode(parent_handle);
            if (parent.PeekChildHandles().size() > 1) {
              return 999.f;
            }
          }
        }
        if (simulation_settings.crown_shyness_distance > 0.f && internode.IsEndNode()) {
          const glm::vec3 end_position = global_transform * glm::vec4(internode.info.GetGlobalEndPosition(), 1.0f);
          bool prune_by_crown_shyness = false;
          climate_model.environment_grid.voxel_grid.PeekEach(
              end_position, simulation_settings.crown_shyness_distance * 2.0f, [&](const EnvironmentVoxel& data) {
                if (prune_by_crown_shyness)
                  return;
                for (const auto& i : data.internode_voxel_registrations) {
                  if (i.tree_skeleton_index == shoot_skeleton.data.entity_index)
                    continue;
                  if (glm::distance(end_position, i.position) < simulation_settings.crown_shyness_distance)
                    prune_by_crown_shyness = true;
                }
              });
          if (prune_by_crown_shyness)
            return 999.f;
        }
        constexpr float pruning_probability = 0.0f;
        return pruning_probability;
      };
}

void BasicPruningDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "low_branch_pruning" << YAML::Value << low_branch_pruning;
  out << YAML::Key << "trunk_protection" << YAML::Value << trunk_protection;
  out << YAML::Key << "max_flow_length" << YAML::Value << max_flow_length;
  out << YAML::Key << "light_pruning_factor" << YAML::Value << light_pruning_factor;
  out << YAML::Key << "branch_strength" << YAML::Value << branch_strength;
  out << YAML::Key << "branch_strength_thickness_factor" << YAML::Value << branch_strength_thickness_factor;
  out << YAML::Key << "branch_strength_lighting_threshold" << YAML::Value << branch_strength_lighting_threshold;
  out << YAML::Key << "branch_strength_lighting_loss" << YAML::Value << branch_strength_lighting_loss;
  out << YAML::Key << "branch_breaking_factor" << YAML::Value << branch_breaking_factor;
  out << YAML::Key << "branch_breaking_multiplier" << YAML::Value << branch_breaking_multiplier;
}

void BasicPruningDescriptor::Deserialize(const YAML::Node& in) {
  if (in["low_branch_pruning"]) {
    low_branch_pruning = in["low_branch_pruning"].as<float>();
  }
  if (in["trunk_protection"])
    trunk_protection = in["trunk_protection"].as<bool>();
  if (in["max_flow_length"])
    max_flow_length = in["max_flow_length"].as<int>();
  if (in["light_pruning_factor"])
    light_pruning_factor = in["light_pruning_factor"].as<float>();
  if (in["branch_strength"])
    branch_strength = in["branch_strength"].as<float>();
  if (in["branch_strength_thickness_factor"])
    branch_strength_thickness_factor = in["branch_strength_thickness_factor"].as<float>();
  if (in["branch_strength_lighting_threshold"])
    branch_strength_lighting_threshold = in["branch_strength_lighting_threshold"].as<float>();
  if (in["branch_strength_lighting_loss"])
    branch_strength_lighting_loss = in["branch_strength_lighting_loss"].as<float>();
  if (in["branch_breaking_factor"])
    branch_breaking_factor = in["branch_breaking_factor"].as<float>();
  if (in["branch_breaking_multiplier"])
    branch_breaking_multiplier = in["branch_breaking_multiplier"].as<float>();
}

bool BasicPruningDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNodeEx("Pruning", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::DragFloat("Low Branch Pruning", &low_branch_pruning, 0.01f, 0.0f, 1.f))
      changed = true;
    changed = ImGui::Checkbox("Trunk Protection", &trunk_protection) || changed;
    changed = ImGui::DragInt("Max chain length", &max_flow_length, 1) || changed;
    changed = ImGui::DragFloat("Light pruning threshold", &light_pruning_factor, 0.01f) || changed;

    changed = ImGui::DragFloat("Branch strength", &branch_strength, 0.01f, 0.0f) || changed;
    changed =
        ImGui::DragFloat("Branch strength thickness factor", &branch_strength_thickness_factor, 0.01f, 0.0f) || changed;
    changed = ImGui::DragFloat("Branch strength lighting threshold", &branch_strength_lighting_threshold, 0.01f, 0.0f,
                               1.0f) ||
              changed;
    changed =
        ImGui::DragFloat("Branch strength lighting loss", &branch_strength_lighting_loss, 0.01f, 0.0f, 1.0f) || changed;
    changed =
        ImGui::DragFloat("Branch breaking multiplier", &branch_breaking_multiplier, 0.01f, 0.01f, 10.0f) || changed;

    changed = ImGui::DragFloat("Branch breaking factor", &branch_breaking_factor, 0.01f, 0.01f, 10.0f) || changed;

    ImGui::TreePop();
  }

  return changed;
}