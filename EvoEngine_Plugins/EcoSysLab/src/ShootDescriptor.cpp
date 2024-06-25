#include "ShootDescriptor.hpp"

using namespace eco_sys_lab;

void ShootDescriptor::PrepareController(ShootGrowthController& shoot_growth_controller) const {
  shoot_growth_controller.m_baseInternodeCount = base_internode_count;
  shoot_growth_controller.m_breakingForce = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    if (branch_strength != 0.f && !internode.IsEndNode() && internode.info.thickness != 0.f &&
        internode.info.length != 0.f) {
      float branch_water_factor = 1.f;
      if (branch_strength_lighting_threshold != 0.f &&
          internode.data.max_descendant_light_intensity < branch_strength_lighting_threshold) {
        branch_water_factor = 1.f - branch_strength_lighting_loss;
      }

      return glm::pow(internode.info.thickness / end_node_thickness, branch_strength_thickness_factor) *
             branch_water_factor * internode.data.strength * branch_strength;
    }
    return FLT_MAX;
  };
  shoot_growth_controller.m_sagging = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    float strength = end_node_thickness * internode.data.sagging_force * gravity_bending_strength /
                     glm::pow(internode.info.thickness / end_node_thickness, gravity_bending_thickness_factor);
    strength = gravity_bending_max * (1.f - glm::exp(-glm::abs(strength)));
    return glm::max(internode.data.sagging, strength);
  };
  shoot_growth_controller.m_baseNodeApicalAngle = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    return glm::gaussRand(base_node_apical_angle_mean_variance.x, base_node_apical_angle_mean_variance.y);
  };

  shoot_growth_controller.m_internodeGrowthRate = growth_rate / internode_length;

  shoot_growth_controller.m_branchingAngle = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    float value = glm::gaussRand(branching_angle_mean_variance.x, branching_angle_mean_variance.y);
    /*
            if(const auto noise = m_branchingAngle.Get<ProceduralNoise2D>())
            {
                    noise->Process(glm::vec2(internode.GetHandle(), internode.info.m_rootDistance), value);
            }*/
    return value;
  };
  shoot_growth_controller.m_rollAngle = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    float value = glm::gaussRand(roll_angle_mean_variance.x, roll_angle_mean_variance.y);
    /*
            if (const auto noise = roll_angle.Get<ProceduralNoise2D>())
            {
                    noise->Process(glm::vec2(internode.GetHandle(), internode.info.m_rootDistance), value);
            }*/
    value += roll_angle_noise_2d.GetValue(glm::vec2(internode.GetHandle(), internode.info.root_distance));
    return value;
  };
  shoot_growth_controller.m_apicalAngle = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    if (straight_trunk != 0.f && internode.data.order == 0 && internode.info.root_distance < straight_trunk)
      return 0.f;
    float value = glm::gaussRand(apical_angle_mean_variance.x, apical_angle_mean_variance.y);
    /*
            if (const auto noise = apical_angle.Get<ProceduralNoise2D>())
            {
                    noise->Process(glm::vec2(internode.GetHandle(), internode.info.m_rootDistance), value);
            }*/
    value += apical_angle_noise_2d.GetValue(glm::vec2(internode.GetHandle(), internode.info.root_distance));
    return value;
  };
  shoot_growth_controller.m_gravitropism = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    return gravitropism;
  };
  shoot_growth_controller.m_phototropism = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    return phototropism;
  };
  shoot_growth_controller.m_horizontalTropism = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    return horizontal_tropism;
  };

  shoot_growth_controller.m_internodeStrength = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    return 1.f;
  };

  shoot_growth_controller.m_internodeLength = internode_length;
  shoot_growth_controller.m_internodeLengthThicknessFactor = internode_length_thickness_factor;
  shoot_growth_controller.m_endNodeThickness = end_node_thickness;
  shoot_growth_controller.m_thicknessAccumulationFactor = thickness_accumulation_factor;
  shoot_growth_controller.m_thicknessAgeFactor = thickness_age_factor;
  shoot_growth_controller.m_internodeShadowFactor = internode_shadow_factor;

  shoot_growth_controller.m_lateralBudCount = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    if (max_order == -1 || internode.data.order < max_order) {
      return lateral_bud_count;
    }
    return 0;
  };
  shoot_growth_controller.m_m_apicalBudExtinctionRate = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    if (internode.info.root_distance < 0.5f)
      return 0.f;
    return apical_bud_extinction_rate;
  };
  shoot_growth_controller.m_lateralBudFlushingRate = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    float flushing_rate = lateral_bud_flushing_rate;
    if (internode.data.inhibitor_sink > 0.0f)
      flushing_rate *= glm::exp(-internode.data.inhibitor_sink);
    return flushing_rate;
  };
  shoot_growth_controller.m_apicalControl = apical_control;
  shoot_growth_controller.m_rootDistanceControl = root_distance_control;
  shoot_growth_controller.m_heightControl = height_control;

  shoot_growth_controller.m_apicalDominance = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    return apical_dominance * internode.data.light_intensity;
  };
  shoot_growth_controller.m_apicalDominanceLoss = apical_dominance_loss;
  shoot_growth_controller.m_leaf = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    return internode.data.light_intensity > leaf_flushing_lighting_requirement;
  };

  shoot_growth_controller.m_leafFallProbability = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    return leaf_fall_probability;
  };
  shoot_growth_controller.m_fruit = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    return internode.data.light_intensity > fruit_flushing_lighting_requirement;
  };
  shoot_growth_controller.m_fruitFallProbability = [&](const SkeletonNode<InternodeGrowthData>& internode) {
    return fruit_fall_probability;
  };
}

void ShootDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "base_internode_count" << YAML::Value << base_internode_count;
  out << YAML::Key << "straight_trunk" << YAML::Value << straight_trunk;
  out << YAML::Key << "base_node_apical_angle_mean_variance" << YAML::Value << base_node_apical_angle_mean_variance;

  out << YAML::Key << "growth_rate" << YAML::Value << growth_rate;
  roll_angle.Save("roll_angle", out);
  apical_angle.Save("apical_angle", out);

  roll_angle_noise_2d.Save("roll_angle_noise_2d", out);
  apical_angle_noise_2d.Save("apical_angle_noise_2d", out);

  out << YAML::Key << "branching_angle_mean_variance" << YAML::Value << branching_angle_mean_variance;
  out << YAML::Key << "roll_angle_mean_variance" << YAML::Value << roll_angle_mean_variance;
  out << YAML::Key << "apical_angle_mean_variance" << YAML::Value << apical_angle_mean_variance;
  out << YAML::Key << "gravitropism" << YAML::Value << gravitropism;
  out << YAML::Key << "phototropism" << YAML::Value << phototropism;
  out << YAML::Key << "horizontal_tropism" << YAML::Value << horizontal_tropism;
  out << YAML::Key << "gravity_bending_strength" << YAML::Value << gravity_bending_strength;
  out << YAML::Key << "gravity_bending_thickness_factor" << YAML::Value << gravity_bending_thickness_factor;
  out << YAML::Key << "gravity_bending_max" << YAML::Value << gravity_bending_max;

  out << YAML::Key << "internode_length" << YAML::Value << internode_length;
  out << YAML::Key << "internode_length_thickness_factor" << YAML::Value << internode_length_thickness_factor;
  out << YAML::Key << "end_node_thickness" << YAML::Value << end_node_thickness;
  out << YAML::Key << "thickness_accumulation_factor" << YAML::Value << thickness_accumulation_factor;
  out << YAML::Key << "thickness_age_factor" << YAML::Value << thickness_age_factor;
  out << YAML::Key << "internode_shadow_factor" << YAML::Value << internode_shadow_factor;

  out << YAML::Key << "lateral_bud_count" << YAML::Value << lateral_bud_count;
  out << YAML::Key << "max_order" << YAML::Value << max_order;
  out << YAML::Key << "apical_bud_extinction_rate" << YAML::Value << apical_bud_extinction_rate;
  out << YAML::Key << "lateral_bud_flushing_rate" << YAML::Value << lateral_bud_flushing_rate;
  out << YAML::Key << "apical_control" << YAML::Value << apical_control;
  out << YAML::Key << "height_control" << YAML::Value << height_control;
  out << YAML::Key << "root_distance_control" << YAML::Value << root_distance_control;

  out << YAML::Key << "apical_dominance" << YAML::Value << apical_dominance;
  out << YAML::Key << "apical_dominance_loss" << YAML::Value << apical_dominance_loss;

  out << YAML::Key << "trunk_protection" << YAML::Value << trunk_protection;
  out << YAML::Key << "max_flow_length" << YAML::Value << max_flow_length;
  out << YAML::Key << "light_pruning_factor" << YAML::Value << light_pruning_factor;
  out << YAML::Key << "branch_strength" << YAML::Value << branch_strength;
  out << YAML::Key << "branch_strength_thickness_factor" << YAML::Value << branch_strength_thickness_factor;
  out << YAML::Key << "branch_strength_lighting_threshold" << YAML::Value << branch_strength_lighting_threshold;
  out << YAML::Key << "branch_strength_lighting_loss" << YAML::Value << branch_strength_lighting_loss;
  out << YAML::Key << "branch_breaking_factor" << YAML::Value << branch_breaking_factor;
  out << YAML::Key << "branch_breaking_multiplier" << YAML::Value << branch_breaking_multiplier;

  out << YAML::Key << "leaf_flushing_lighting_requirement" << YAML::Value << leaf_flushing_lighting_requirement;
  out << YAML::Key << "leaf_fall_probability" << YAML::Value << leaf_fall_probability;
  out << YAML::Key << "leaf_distance_to_branch_end_limit" << YAML::Value << leaf_distance_to_branch_end_limit;

  out << YAML::Key << "fruit_flushing_lighting_requirement" << YAML::Value << fruit_flushing_lighting_requirement;
  out << YAML::Key << "fruit_fall_probability" << YAML::Value << fruit_fall_probability;
  out << YAML::Key << "fruit_distance_to_branch_end_limit" << YAML::Value << fruit_distance_to_branch_end_limit;

  bark_material.Save("bark_material", out);
}

void ShootDescriptor::Deserialize(const YAML::Node& in) {
  if (in["base_internode_count"])
    base_internode_count = in["base_internode_count"].as<int>();
  if (in["straight_trunk"])
    straight_trunk = in["straight_trunk"].as<float>();
  if (in["base_node_apical_angle_mean_variance"])
    base_node_apical_angle_mean_variance = in["base_node_apical_angle_mean_variance"].as<glm::vec2>();

  if (in["growth_rate"])
    growth_rate = in["growth_rate"].as<float>();
  roll_angle.Load("roll_angle", in);
  apical_angle.Load("apical_angle", in);

  roll_angle_noise_2d.Load("roll_angle_noise_2d", in);
  apical_angle_noise_2d.Load("apical_angle_noise_2d", in);

  if (in["branching_angle_mean_variance"])
    branching_angle_mean_variance = in["branching_angle_mean_variance"].as<glm::vec2>();
  if (in["roll_angle_mean_variance"])
    roll_angle_mean_variance = in["roll_angle_mean_variance"].as<glm::vec2>();
  if (in["apical_angle_mean_variance"])
    apical_angle_mean_variance = in["apical_angle_mean_variance"].as<glm::vec2>();
  if (in["gravitropism"])
    gravitropism = in["gravitropism"].as<float>();
  if (in["phototropism"])
    phototropism = in["phototropism"].as<float>();
  if (in["horizontal_tropism"])
    horizontal_tropism = in["horizontal_tropism"].as<float>();
  if (in["gravity_bending_strength"])
    gravity_bending_strength = in["gravity_bending_strength"].as<float>();
  if (in["gravity_bending_thickness_factor"])
    gravity_bending_thickness_factor = in["gravity_bending_thickness_factor"].as<float>();
  if (in["gravity_bending_max"])
    gravity_bending_max = in["gravity_bending_max"].as<float>();

  if (in["internode_length"])
    internode_length = in["internode_length"].as<float>();
  if (in["internode_length_thickness_factor"])
    internode_length_thickness_factor = in["internode_length_thickness_factor"].as<float>();
  if (in["end_node_thickness"])
    end_node_thickness = in["end_node_thickness"].as<float>();
  if (in["thickness_accumulation_factor"])
    thickness_accumulation_factor = in["thickness_accumulation_factor"].as<float>();
  if (in["thickness_age_factor"])
    thickness_age_factor = in["thickness_age_factor"].as<float>();
  if (in["internode_shadow_factor"])
    internode_shadow_factor = in["internode_shadow_factor"].as<float>();

  if (in["lateral_bud_count"])
    lateral_bud_count = in["lateral_bud_count"].as<int>();
  if (in["max_order"])
    max_order = in["max_order"].as<int>();
  if (in["apical_bud_extinction_rate"])
    apical_bud_extinction_rate = in["apical_bud_extinction_rate"].as<float>();
  if (in["lateral_bud_flushing_rate"])
    lateral_bud_flushing_rate = in["lateral_bud_flushing_rate"].as<float>();
  if (in["apical_control"])
    apical_control = in["apical_control"].as<float>();
  if (in["root_distance_control"])
    root_distance_control = in["root_distance_control"].as<float>();
  if (in["height_control"])
    height_control = in["height_control"].as<float>();

  if (in["apical_dominance"])
    apical_dominance = in["apical_dominance"].as<float>();
  if (in["apical_dominance_loss"])
    apical_dominance_loss = in["apical_dominance_loss"].as<float>();

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

  if (in["leaf_flushing_lighting_requirement"])
    leaf_flushing_lighting_requirement = in["leaf_flushing_lighting_requirement"].as<float>();
  if (in["leaf_fall_probability"])
    leaf_fall_probability = in["leaf_fall_probability"].as<float>();
  if (in["leaf_distance_to_branch_end_limit"])
    leaf_distance_to_branch_end_limit = in["leaf_distance_to_branch_end_limit"].as<float>();

  // Structure
  if (in["fruit_flushing_lighting_requirement"])
    fruit_flushing_lighting_requirement = in["fruit_flushing_lighting_requirement"].as<float>();
  if (in["fruit_fall_probability"])
    fruit_fall_probability = in["fruit_fall_probability"].as<float>();
  if (in["fruit_distance_to_branch_end_limit"])
    fruit_distance_to_branch_end_limit = in["fruit_distance_to_branch_end_limit"].as<float>();

  bark_material.Load("bark_material", in);
}

bool ShootDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  changed = ImGui::DragFloat("Growth rate", &growth_rate, 0.01f, 0.0f, 10.0f) || changed;
  changed = ImGui::DragFloat("Straight Trunk", &straight_trunk, 0.1f, 0.0f, 100.f) || changed;
  if (ImGui::TreeNodeEx("Internode", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragInt("Base node count", &base_internode_count, 1, 0, 3) || changed;
    changed =
        ImGui::DragFloat2("Base apical angle mean/var", &base_node_apical_angle_mean_variance.x, 0.1f, 0.0f, 100.0f) ||
        changed;

    changed = ImGui::DragInt("Lateral bud count", &lateral_bud_count, 1, 0, 3) || changed;
    changed = ImGui::DragInt("Max Order", &max_order, 1, -1, 100) || changed;
    if (ImGui::TreeNodeEx("Angles")) {
      changed =
          ImGui::DragFloat2("Branching angle base/var", &branching_angle_mean_variance.x, 0.1f, 0.0f, 100.0f) || changed;
      // editorLayer->DragAndDropButton<ProceduralNoise2D>(m_branchingAngle, "Branching Angle Noise");
      changed = ImGui::DragFloat2("Roll angle base/var", &roll_angle_mean_variance.x, 0.1f, 0.0f, 100.0f) || changed;
      // editorLayer->DragAndDropButton<ProceduralNoise2D>(m_rollAngle, "Roll Angle Noise");
      changed = ImGui::DragFloat2("Apical angle base/var", &apical_angle_mean_variance.x, 0.1f, 0.0f, 100.0f) || changed;
      // editorLayer->DragAndDropButton<ProceduralNoise2D>(m_apicalAngle, "Apical Angle Noise");
      if (ImGui::TreeNodeEx("Roll Angle Noise2D")) {
        changed = roll_angle_noise_2d.OnInspect() | changed;
        ImGui::TreePop();
      }
      if (ImGui::TreeNodeEx("Apical Angle Noise2D")) {
        changed = apical_angle_noise_2d.OnInspect() | changed;
        ImGui::TreePop();
      }
      ImGui::TreePop();
    }
    changed = ImGui::DragFloat("Internode length", &internode_length, 0.001f) || changed;
    changed =
        ImGui::DragFloat("Internode length thickness factor", &internode_length_thickness_factor, 0.0001f, 0.0f, 1.0f) ||
        changed;
    changed =
        ImGui::DragFloat3("Thickness min/factor/age", &end_node_thickness, 0.0001f, 0.0f, 1.0f, "%.6f") || changed;

    changed = ImGui::DragFloat("Bending strength", &gravity_bending_strength, 0.01f, 0.0f, 1.0f, "%.3f") || changed;
    changed =
        ImGui::DragFloat("Bending thickness factor", &gravity_bending_thickness_factor, 0.1f, 0.0f, 10.f, "%.3f") ||
        changed;
    changed = ImGui::DragFloat("Bending angle factor", &gravity_bending_max, 0.01f, 0.0f, 1.0f, "%.3f") || changed;

    changed = ImGui::DragFloat("Internode shadow factor", &internode_shadow_factor, 0.001f, 0.0f, 1.0f) || changed;

    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Bud fate", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragFloat("Gravitropism", &gravitropism, 0.01f) || changed;
    changed = ImGui::DragFloat("Phototropism", &phototropism, 0.01f) || changed;
    changed = ImGui::DragFloat("Horizontal Tropism", &horizontal_tropism, 0.01f) || changed;

    changed = ImGui::DragFloat("Apical bud extinction rate", &apical_bud_extinction_rate, 0.01f, 0.0f, 1.0f, "%.5f") ||
              changed;
    changed =
        ImGui::DragFloat("Lateral bud flushing rate", &lateral_bud_flushing_rate, 0.01f, 0.0f, 1.0f, "%.5f") || changed;

    changed = ImGui::DragFloat2("Inhibitor val/loss", &apical_dominance, 0.01f) || changed;
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Tree Shape Control", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragFloat("Apical control", &apical_control, 0.01f) || changed;
    changed = ImGui::DragFloat("Root distance control", &root_distance_control, 0.01f) || changed;
    changed = ImGui::DragFloat("Height control", &height_control, 0.01f) || changed;

    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Pruning", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::Checkbox("Trunk Protection", &trunk_protection) || changed;
    changed = ImGui::DragInt("Max chain length", &max_flow_length, 1) || changed;
    changed = ImGui::DragFloat("Light pruning threshold", &light_pruning_factor, 0.01f) || changed;

    changed = ImGui::DragFloat("Branch strength", &branch_strength, 0.01f, 0.0f) || changed;
    changed =
        ImGui::DragFloat("Branch strength thickness factor", &branch_strength_thickness_factor, 0.01f, 0.0f) || changed;
    changed =
        ImGui::DragFloat("Branch strength lighting threshold", &branch_strength_lighting_threshold, 0.01f, 0.0f, 1.0f) ||
        changed;
    changed =
        ImGui::DragFloat("Branch strength lighting loss", &branch_strength_lighting_loss, 0.01f, 0.0f, 1.0f) || changed;
    changed =
        ImGui::DragFloat("Branch breaking multiplier", &branch_breaking_multiplier, 0.01f, 0.01f, 10.0f) || changed;

    changed = ImGui::DragFloat("Branch breaking factor", &branch_breaking_factor, 0.01f, 0.01f, 10.0f) || changed;

    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Leaf")) {
    changed =
        ImGui::DragFloat("Lighting requirement", &leaf_flushing_lighting_requirement, 0.01f, 0.0f, 1.0f) || changed;
    changed = ImGui::DragFloat("Drop prob", &leaf_fall_probability, 0.01f) || changed;
    changed = ImGui::DragFloat("Distance To End Limit", &leaf_distance_to_branch_end_limit, 0.01f) || changed;
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Fruit")) {
    changed =
        ImGui::DragFloat("Lighting requirement", &fruit_flushing_lighting_requirement, 0.01f, 0.0f, 1.0f) || changed;
    changed = ImGui::DragFloat("Drop prob", &fruit_fall_probability, 0.01f) || changed;
    changed = ImGui::DragFloat("Distance To End Limit", &fruit_distance_to_branch_end_limit, 0.01f) || changed;
    ImGui::TreePop();
  }

  editor_layer->DragAndDropButton<Material>(bark_material, "Bark Material##SBS");

  return changed;
}

void ShootDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  if (roll_angle.Get<ProceduralNoise2D>())
    list.push_back(roll_angle);
  if (apical_angle.Get<ProceduralNoise2D>())
    list.push_back(apical_angle);

  if (bark_material.Get<Material>())
    list.push_back(bark_material);
}
